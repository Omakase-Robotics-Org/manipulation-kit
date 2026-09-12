"""Cross-model consistency of every D1 description file in this repository.

Pins the D1-arm invariants (description/d1-arm-notes.md):

  1. both arms are the SAME physical arm — the J1..J7(+TCP) chain is
     identical across every model file AND across both sides;
  2. with mirror-symmetric joint values the wrist frames (Link7) land 180°
     apart from a perfect mirror, while TCP_Link and the YUBI hand root are
     exactly mirror-symmetric;
  3. the YUBI hand mount compensates: 180° rotation with the y offset sign
     flipped on the _L tree;
  4. J7's range is ±90°, so the 180° wrist offset is unreachable in joint
     space (the physical hand mount is the only compensation);
  5. the committed generated URDFs (description/d1/*.urdf and
     d1_yubi_description_v2/urdf/d1_yubi.urdf) match their generators;
  6. the arm STL meshes duplicated between the d1_arm packages and
     d1_yubi_description_v2 are byte-identical, so the two ROS packages
     cannot fork the robot's geometry;
  7. d1_wholebody.urdf reproduces the vendor body URDF's world positions;
  8. d1_wholebody_gripper.urdf wears the D1 stock parallel gripper on the tool
     flange with the CAD's jaw kinematics and CAD masses, the same part the
     same way up on both arms, and no XML comment in any generated model
     contains a `--` (which would make it unreadable to every parser).

The earlier suite checked mirror symmetry of link POSITIONS only, which is
exactly how an orientation asymmetry went unnoticed — hence the explicit
rotation checks here.
"""
import hashlib
import json
import math
import os
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import pytest

import manipulation_kit
from manipulation_kit import assets
from manipulation_kit.description.tools.export_description import (
    PKG_V2, YUBI_MESH_SOURCES, is_optional)
from manipulation_kit.guard import geometry as g
from manipulation_kit.guard.urdf_model import UrdfModel

HERE = os.path.dirname(os.path.abspath(__file__))
# Assets resolve INSIDE the package (no $D1_SDK_DIR, no sibling checkout), so
# the tests import the anchor rather than counting "..".
KIT = os.path.dirname(os.path.abspath(manipulation_kit.__file__))
DESCRIPTION = os.path.join(KIT, "description")
CONFIG = os.path.join(KIT, "config")
REPO = os.path.normpath(os.path.join(KIT, "..", ".."))

#: The CAD is not in this repository (LICENSE-STATUS.md). Checks that OPEN a
#: mesh carry this mark; checks that the URDF still NAMES the right meshes run
#: unconditionally, because those are the ones that catch a generator that
#: quietly stopped referencing half the robot. Set $MKIT_ASSETS_DIR to run
#: both halves, as CI does.
needs_assets = pytest.mark.skipif(not assets.have_external_assets(),
                                  reason=assets.NO_ASSETS_REASON)

#: What ``description/d1/meshes/{body,gripper}/`` must contain. Written down
#: rather than listed off disk, because the files are external now: a test that
#: enumerates the directory would assert "the ten meshes I found are ten" and
#: pass on an empty one. This list is the claim; the directory is the evidence.
BODY_MESH_NAMES = (
    "backcamer_Link.STL", "base_link.STL", "carcamer_Link.STL",
    "dhead_Link.STL", "fcamer_Link.STL", "headcamera_Link.STL",
    "lwheel_Link.STL", "rwheel_Link.STL", "slider_Link.STL",
    "uphead_Link.STL",
)
GRIPPER_MESH_NAMES = (
    "base_link.STL", "camera_plate.STL", "camera_plate_full.STL",
    "tcp_l_Link.STL", "tcp_r_Link.STL",
)

D1 = os.path.join(DESCRIPTION, "d1", "d1.urdf")
D1_WB = os.path.join(DESCRIPTION, "d1", "d1_wholebody.urdf")
D1_WB_GRIPPER = os.path.join(DESCRIPTION, "d1", "d1_wholebody_gripper.urdf")
D1_YUBI = os.path.join(DESCRIPTION, "d1_yubi_description_v2", "urdf", "d1_yubi.urdf")
D1_ARM = {
    side: os.path.join(DESCRIPTION, "d1_arm", side,
                       f"d1_arm_{side}.urdf")
    for side in ("left", "right")
}
XACRO = {
    side: os.path.join(DESCRIPTION, "d1_arm", side,
                       f"d1_arm_{side}_with_yubi.urdf.xacro")
    for side in ("left", "right")
}

# Dual-arm models wearing the YUBI hand: joints are Joint{i}_{R,L}, hand mount
# joints yubi_{R,L}_flange.
DUAL_MODELS = {"d1.urdf": D1, "d1_wholebody.urdf": D1_WB, "d1_yubi.urdf": D1_YUBI}

# Every dual-arm model, whatever end effector it wears. The ARM chain must be
# identical in all of them; only what hangs off TCP_Link differs.
ARM_CHAIN_MODELS = dict(DUAL_MODELS,
                        **{"d1_wholebody_gripper.urdf": D1_WB_GRIPPER})

# q_L that mirrors q_R (from config/home_pose.json: A/B values are sign-
# flipped on J1/J3/J5/J7, equal on J2/J4/J6).
MIRROR_SIGN = (-1, 1, -1, 1, -1, 1, -1)

MIRROR = ((1, 0, 0), (0, -1, 0), (0, 0, 1))  # y-plane mirror


# ---------------------------------------------------------------- helpers
def _mat_mul(A, B):
    return tuple(tuple(sum(A[i][k] * B[k][j] for k in range(3))
                       for j in range(3)) for i in range(3))


def _mat_T(A):
    return tuple(tuple(A[j][i] for j in range(3)) for i in range(3))


def _rot_angle_axis(D):
    """Angle (rad) and unit axis of rotation matrix D."""
    tr = D[0][0] + D[1][1] + D[2][2]
    ang = math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0)))
    s = 2.0 * math.sin(ang)
    if abs(s) > 1e-8:
        return ang, ((D[2][1] - D[1][2]) / s, (D[0][2] - D[2][0]) / s,
                     (D[1][0] - D[0][1]) / s)
    if ang < 0.1:
        return ang, (0.0, 0.0, 0.0)
    # angle ~ pi: axis from the diagonal of (D + I) / 2
    ax = [math.sqrt(max(0.0, (D[i][i] + 1.0) / 2.0)) for i in range(3)]
    # fix signs from off-diagonals
    if ax[0] > 1e-6:
        ax[1] = math.copysign(ax[1], D[0][1])
        ax[2] = math.copysign(ax[2], D[0][2])
    elif ax[1] > 1e-6:
        ax[2] = math.copysign(ax[2], D[1][2])
    return ang, tuple(ax)


def _rpy_to_R(rpy):
    return g.from_rpy_xyz(*rpy, 0.0, 0.0, 0.0).R


def _joint_transform(model_path, joint_name):
    """(xyz tuple, R matrix, axis tuple, (lower, upper) or None)."""
    root = ET.parse(model_path).getroot()
    for j in root.findall("joint"):
        if j.get("name") == joint_name:
            o = j.find("origin")
            xyz = tuple(float(v) for v in (o.get("xyz") or "0 0 0").split())
            rpy = tuple(float(v) for v in (o.get("rpy") or "0 0 0").split())
            a = j.find("axis")
            axis = (tuple(float(v) for v in a.get("xyz").split())
                    if a is not None else (0.0, 0.0, 1.0))
            lim = j.find("limit")
            lims = ((float(lim.get("lower")), float(lim.get("upper")))
                    if lim is not None and lim.get("lower") is not None else None)
            return xyz, _rpy_to_R(rpy), axis, lims
    raise KeyError(f"{joint_name} not in {model_path}")


def _close(a, b, tol):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def _R_close(A, B, tol=2e-4):
    return all(abs(A[i][j] - B[i][j]) < tol for i in range(3) for j in range(3))


def _chain(model_path, suffix):
    """[(xyz, R, axis, limits)] for Joint1..7 + fixed TCP joint."""
    out = [_joint_transform(model_path, f"Joint{i}_{suffix}") for i in range(1, 8)]
    out.append(_joint_transform(model_path, f"JointTCP_{suffix}"))
    return out


# ------------------------------------------------- generated files current
def test_generated_urdfs_match_generator():
    """description/d1/*.urdf are exactly what the generator emits."""
    gen = os.path.join(DESCRIPTION, "d1", "tools", "generate_d1_urdf.py")
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run([sys.executable, gen, "--out-dir", tmp],
                       check=True, capture_output=True)
        for name, committed in (("d1.urdf", D1),
                                ("d1_wholebody.urdf", D1_WB),
                                ("d1_wholebody_gripper.urdf", D1_WB_GRIPPER)):
            with open(os.path.join(tmp, name)) as f:
                fresh = f.read()
            with open(committed) as f:
                assert f.read() == fresh, (
                    f"{name} is stale — regenerate with "
                    "python3 description/d1/tools/generate_d1_urdf.py")


def test_d1_yubi_urdf_matches_generator():
    """description/d1_yubi_description_v2/urdf/d1_yubi.urdf is exactly what
    its generator emits from description/d1_arm/."""
    gen = os.path.join(DESCRIPTION, "d1_yubi_description_v2", "tools",
                       "assemble_d1_yubi.py")
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run([sys.executable, gen, "--out-dir", tmp],
                       check=True, capture_output=True)
        with open(os.path.join(tmp, "d1_yubi.urdf")) as f:
            fresh = f.read()
    with open(D1_YUBI) as f:
        assert f.read() == fresh, (
            "d1_yubi.urdf is stale — regenerate with python3 "
            "description/d1_yubi_description_v2/tools/assemble_d1_yubi.py")


# ------------------------------------------- one mesh tree, not two copies
# d1-sdk carried the arm STLs TWICE — once in the vendor package
# (d1_arm) and once inside d1_yubi_description_v2 — because ROS
# resolves package:// per package, and a test asserted the 18 copies stayed
# byte-identical so the robot's geometry could not fork between RViz and the
# simulators.
#
# manipulation-kit deletes the second copy (27 MB) instead of policing it, and
# the exporter rewrites the v2 package:// URIs onto the single vendor tree via
# YUBI_MESH_SOURCES. Verified byte-identical output at import time. What has to
# be pinned now is different: the alias table must COVER every package:// URI
# the URDF uses, or an export silently loses meshes — the failure the old
# duplication made impossible and this table reintroduces.


def test_yubi_mesh_alias_covers_every_package_uri():
    """Every ``package://d1_yubi_description_v2/...`` mesh in d1_yubi.urdf must
    resolve through YUBI_MESH_SOURCES onto a file that exists in
    d1_arm/. A URI the table does not cover exports as a dangling
    reference, which is exactly what deleting the duplicate tree risks."""
    prefix = "package://d1_yubi_description_v2/"
    refs = {m.get("filename") for m in ET.parse(D1_YUBI).getroot().iter("mesh")}
    assert refs, "d1_yubi.urdf has no meshes — it is the mesh-bearing model"
    uncovered, missing = [], []
    for ref in sorted(refs):
        assert ref.startswith(prefix), f"unexpected mesh URI {ref}"
        rel = ref[len(prefix):]
        src_dir = next((src for dst, src in YUBI_MESH_SOURCES.items()
                        if rel.startswith(dst + "/")), None)
        if src_dir is None:
            # Not aliased: it must then still exist inside the v2 package, the
            # way yubi_description/meshes does (d1_wholebody.urdf reaches those
            # by relative path, so they cannot be aliased away).
            if os.path.isfile(os.path.join(DESCRIPTION, PKG_V2,
                                           *rel.split("/"))):
                continue
            uncovered.append(rel)
            continue
        dst = next(d for d in YUBI_MESH_SOURCES if rel.startswith(d + "/"))
        pkg_rel = "/".join(("description", *src_dir.split("/"),
                            *rel[len(dst) + 1:].split("/")))
        # The arm CAD is external (LICENSE-STATUS.md), so "does the alias land
        # on a real file" is only answerable with an assets checkout. The
        # COVERAGE half above is answerable always, and is the half that
        # catches a URI the table forgot.
        if assets.have_external_assets() and assets.resolve(pkg_rel) is None:
            missing.append(rel)
    assert not uncovered, (
        f"YUBI_MESH_SOURCES does not cover {uncovered} — add the mapping or "
        "the export drops those meshes")
    assert not missing, f"aliased to a file that does not exist: {missing}"


def test_no_second_copy_of_the_arm_meshes():
    """The duplicate tree must not come back. If someone re-vendors it, the
    27 MB returns AND the fork it used to enable returns with it."""
    dupe = os.path.join(DESCRIPTION, "d1_yubi_description_v2",
                        "d1_arm_yubi_description")
    assert not os.path.exists(dupe), (
        f"{dupe} is back — the arm meshes live once, in d1_arm/; "
        "the exporter aliases the package:// paths (YUBI_MESH_SOURCES)")


@pytest.mark.parametrize("path", sorted(set(ARM_CHAIN_MODELS.values())
                                        | set(D1_ARM.values())))
def test_model_tree_is_sound(path):
    """Every model parses, is single-rooted, and no link has two parents."""
    root = ET.parse(path).getroot()
    links = [l.get("name") for l in root.findall("link")]
    parents = {}
    for j in root.findall("joint"):
        child = j.find("child").get("link")
        assert child not in parents, \
            f"{os.path.basename(path)}: link {child} has two parent joints"
        parents[child] = j.find("parent").get("link")
    roots = [l for l in links if l not in parents]
    assert len(roots) == 1, f"{os.path.basename(path)}: roots {roots}"


# ------------------------------------------------------- chain identity
def _all_chains():
    chains = {}
    for name, path in ARM_CHAIN_MODELS.items():
        for suf in ("R", "L"):
            chains[f"{name}:{suf}"] = _chain(path, suf)
    chains["d1_arm_right.urdf:R"] = _chain(D1_ARM["right"], "R")
    chains["d1_arm_left.urdf:L"] = _chain(D1_ARM["left"], "L")
    return chains


def test_arm_chain_identical_across_all_models_and_sides():
    """Every model file in the repo, both sides: same J1..J7+TCP chain.

    The two arms are the same physical arm (vendor-confirmed); any model
    whose left arm is a mirrored chain (like the vendor's merged
    urdf20260725 drop — see d1-arm-notes.md §5) must fail here.
    """
    chains = _all_chains()
    ref_key = "d1.urdf:R"
    ref = chains[ref_key]
    for key, chain in chains.items():
        for i, ((x0, r0, a0, l0), (x1, r1, a1, l1)) in enumerate(zip(ref, chain)):
            jn = f"Joint{i + 1}" if i < 7 else "JointTCP"
            assert _close(x0, x1, 1e-6), f"{key} {jn} origin xyz != {ref_key}"
            assert _R_close(r0, r1), f"{key} {jn} origin rotation != {ref_key}"
            if i < 7:  # axis is meaningless on the fixed TCP joint
                assert _close(a0, a1, 1e-9), f"{key} {jn} axis != {ref_key}"
            if l0 is not None:
                assert l1 is not None and _close(l0, l1, 1e-4), \
                    f"{key} {jn} limits {l1} != {ref_key} {l0}"


def test_j7_range_is_90_deg_everywhere():
    """J7 = ±1.5708 rad in every model: a 180° wrist correction is
    unreachable in joint space, so the hand-mount compensation (below) is
    the only one possible."""
    for key, chain in _all_chains().items():
        lo, hi = chain[6][3]
        assert abs(lo + 1.5708) < 1e-4 and abs(hi - 1.5708) < 1e-4, \
            f"{key} J7 limits ({lo}, {hi}) != ±1.5708"


# ------------------------------------------------------ hand mount 180°
def _xacro_mount(side):
    """(xyz, R) of the tip->hand joint in a *_with_yubi xacro."""
    with open(XACRO[side]) as f:
        text = f.read()
    m = re.search(r'<origin xyz="([^"]+)" rpy="([^"]+)"/>', text)
    assert m, f"no origin in {XACRO[side]}"
    xyz = tuple(float(v) for v in m.group(1).split())
    rpy = []
    for tok in m.group(2).split():
        deg = re.fullmatch(r"\$\{radians\((-?[\d.]+)\)\}", tok)
        rpy.append(math.radians(float(deg.group(1))) if deg else float(tok))
    return xyz, _rpy_to_R(tuple(rpy))


def _hand_mounts():
    """{model: {side: (xyz, R)}} for every file that mounts the hand."""
    out = {}
    for name, path in DUAL_MODELS.items():
        out[name] = {s: _joint_transform(path, f"yubi_{s}_flange")[:2]
                     for s in ("R", "L")}
    # xacros: vendor "right" package = _R tree, "left" = _L tree
    out["with_yubi.xacro"] = {"R": _xacro_mount("right"),
                              "L": _xacro_mount("left")}
    return out


def test_hand_mount_180_apart_and_consistent_across_models():
    """The _L hand mount = _R mount with the y offset sign flipped and a
    180° rotation — in every file that mounts the hand, including the
    d1_arm *_with_yubi xacros (whose left file used to carry a verbatim
    copy of the right transform: a hand 180° from reality)."""
    mounts = _hand_mounts()
    ref = mounts["d1.urdf"]
    for name, m in mounts.items():
        for s in ("R", "L"):
            assert _close(m[s][0], ref[s][0], 1e-9), f"{name} {s} mount xyz"
            assert _R_close(m[s][1], ref[s][1]), f"{name} {s} mount rotation"
    # the 180°-apart relation itself
    (xr, Rr), (xl, Rl) = ref["R"], ref["L"]
    assert _close(xl, (xr[0], -xr[1], xr[2]), 1e-9), \
        "L mount xyz is not the y-flip of R"
    ang, _axis = _rot_angle_axis(_mat_mul(_mat_T(Rr), Rl))
    assert abs(ang - math.pi) < 2e-4, \
        f"L mount rotation is {math.degrees(ang):.2f}° from R, expected 180°"


# ---------------------------------------------------- FK mirror invariants
@pytest.fixture(scope="module")
def model():
    return UrdfModel()


def _mirror_conjugate(R):
    return _mat_mul(_mat_mul(MIRROR, R), MIRROR)


# a few in-range right-arm configurations (rad), incl. the measured home pose
Q_CASES = [
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    (-0.8466, 1.2728, 1.3167, -1.9167, 1.2020, -0.1268, 0.3163),  # ~home
    (-0.35, -0.70, 0.30, -0.86, 0.07, -0.27, -0.88),
    (0.01, -0.93, -0.13, -0.86, -0.82, -0.15, 0.65),
    (-0.75, -0.55, 0.25, 0.90, 0.15, -0.21, 0.95),
]


@pytest.mark.parametrize("qR", Q_CASES)
def test_mirror_pose_wrist_180_tcp_and_hand_exact_mirror(model, qR):
    """Mirror joint values: positions mirror everywhere; Link7 orientation
    is 180° (about the local flange z axis) from a perfect mirror; TCP_Link
    and the yubi hand root are EXACT mirrors — the hand mount compensates
    the wrist flip. Measured facts, not convention (d1-arm-notes.md §2)."""
    qL = [s * v for s, v in zip(MIRROR_SIGN, qR)]
    q = {f"Joint{i + 1}_R": qR[i] for i in range(7)}
    q.update({f"Joint{i + 1}_L": qL[i] for i in range(7)})
    tfs = model.link_transforms(q)

    def D(link_r, link_l):
        tr, tl = tfs[link_r], tfs[link_l]
        assert _close((tr.t[0], -tr.t[1], tr.t[2]), tl.t, 1e-5), \
            f"{link_l} position is not the mirror of {link_r}"
        return _mat_mul(_mat_T(_mirror_conjugate(tr.R)), tl.R)

    # links 1..6: positions mirror (interior frame ORIENTATIONS need not —
    # an identical, non-mirror chain differs by rotations about its own
    # joint axes; only the position check is a symmetry invariant here)
    for i in range(1, 7):
        tr, tl = tfs[f"Link{i}_R"], tfs[f"Link{i}_L"]
        assert _close((tr.t[0], -tr.t[1], tr.t[2]), tl.t, 1e-5), \
            f"Link{i}_L position is not the mirror of Link{i}_R"

    # wrist: 180° about the local flange axis (z) — CANNOT be mirrored away
    ang, axis = _rot_angle_axis(D("Link7_R", "Link7_L"))
    assert abs(ang - math.pi) < 5e-4, \
        f"Link7 must be 180° from the mirror, got {math.degrees(ang):.3f}°"
    assert abs(abs(axis[2]) - 1.0) < 1e-3, \
        f"Link7 flip axis {axis} is not the local flange z axis"

    # TCP and hand: exact mirror again
    for r, l in (("TCP_Link_R", "TCP_Link_L"),
                 ("yubi_R_hand_root", "yubi_L_hand_root")):
        ang, _ = _rot_angle_axis(D(r, l))
        assert ang < 5e-4, \
            f"{l} orientation deviates from the mirror of {r} by " \
            f"{math.degrees(ang):.3f}° — the hand-mount compensation is broken"


# ------------------------------------------------ wholebody vendor anchors
def test_wholebody_fk_uses_calibrated_lift_mount():
    """Preserve vendor relative anchors with the published-height zero correction."""
    wb = UrdfModel(D1_WB)
    tfs = wb.link_transforms({})
    assert _close(tfs["Base_R"].t, (0.0, 0.037, .984), 1e-5)
    assert _close(tfs["neck_pan_link"].t, (0.0016171, 0.0, 1.105), 1e-5)
    assert _close(tfs["head_link"].t, (0.0016171, 0.0285, 1.1605), 1e-5)


# ------------------------------------- d1_wholebody_gripper: the stock gripper
#
# d1_wholebody_gripper.urdf is the same robot as d1_wholebody.urdf wearing the
# D1 stock parallel gripper from its vendor CAD (dx-manipulator
# hands/d1/parallel_gripper/) instead of the YUBI hand. The tests below pin the
# two things that are easy to get wrong and invisible in a position-only check:
# the mount frame, and the HANDEDNESS (a wrist mount in this codebase has
# already shipped with the two per-arm rotations swapped).
GRIPPER_JAW_STROKE = 0.035          # per jaw, m
GRIPPER_TCP_Z = 0.136               # registered tool config, m along flange +z
GRIPPER_JAW_TIP_Z = 0.1435          # CAD jaw tips, m along flange +z


def _gripper_model():
    return UrdfModel(D1_WB_GRIPPER)


def _apply(R, v):
    return tuple(sum(R[i][k] * v[k] for k in range(3)) for i in range(3))


def test_gripper_model_has_no_yubi_left_in_it():
    """Swapping the end effector replaces it; it does not add a second one."""
    root = ET.parse(D1_WB_GRIPPER).getroot()
    names = ([l.get("name") for l in root.findall("link")]
             + [j.get("name") for j in root.findall("joint")])
    assert not [n for n in names if "yubi" in n]
    # per side: base + 2 jaws + camera plate + wrist cam + optical frame,
    # and the 6 joints that hang them together
    assert sum(1 for n in names if n.startswith("gripper_")) == (6 + 6) * 2


def test_no_double_hyphen_inside_comments():
    """A `--` inside an XML comment makes the file unreadable to EVERY parser.
    A generator in this repo has emitted exactly that bug before, so assert it
    on every generated model, not just the new one."""
    for path in sorted(set(ARM_CHAIN_MODELS.values())):
        with open(path) as f:
            text = f.read()
        for chunk in text.split("<!--")[1:]:
            assert "--" not in chunk.split("-->")[0], \
                f"{os.path.basename(path)}: double hyphen inside an XML comment"


def test_gripper_jaw_joints_match_the_vendor_cad():
    """Jaw joints are the CAD's: same origin and axis for both, ±35 mm of
    travel, and a velocity limit that is not zero (the CAD shipped
    velocity="0", which planners read as an immovable joint)."""
    root = ET.parse(D1_WB_GRIPPER).getroot()
    for side in ("R", "L"):
        origins = []
        for jaw, expected in (("r", (0.0, GRIPPER_JAW_STROKE)),
                              ("l", (-GRIPPER_JAW_STROKE, 0.0))):
            name = f"gripper_{side}_tcp_{jaw}_joint"
            j = next(x for x in root.findall("joint") if x.get("name") == name)
            assert j.get("type") == "prismatic"
            xyz, R, axis, lims = _joint_transform(D1_WB_GRIPPER, name)
            assert _close(lims, expected, 1e-9), f"{name} limits {lims}"
            lim = j.find("limit")
            assert float(lim.get("velocity")) > 0.0, f"{name} cannot move"
            assert float(lim.get("effort")) > 0.0
            origins.append((xyz, axis))
        assert origins[0] == origins[1], \
            f"the two {side} jaws must share origin and axis (they are one rail)"
        assert _close(origins[0][0], (0.0, 0.0, 0.10847), 1e-9)


def test_gripper_jaws_open_70mm_and_close_to_zero():
    """q = 0 is the OPEN 70 mm gap and |q| = 0.035 is CLOSED — the opposite
    polarity to the CAN 2.0 wire command, where 0.0 is closed. Derived from the
    committed jaw boxes, so it fails if a sign is flipped anywhere."""
    root = ET.parse(D1_WB_GRIPPER).getroot()
    for side in ("R", "L"):
        faces, travel = {}, {}
        for jaw, q_closed in (("r", GRIPPER_JAW_STROKE), ("l", -GRIPPER_JAW_STROKE)):
            _xyz, _R, axis, _lim = _joint_transform(
                D1_WB_GRIPPER, f"gripper_{side}_tcp_{jaw}_joint")
            col = next(l for l in root.findall("link")
                       if l.get("name") == f"gripper_{side}_tcp_{jaw}_link"
                       ).find("collision")
            ctr = tuple(float(v) for v in col.find("origin").get("xyz").split())
            size = tuple(float(v) for v in
                         col.find("geometry/box").get("size").split())
            # The joint axis is expressed in the jaw link's own frame, and the
            # box is axis-aligned in that frame, so both project onto it
            # directly. Signed position of the box's two faces along +axis:
            along = sum(ctr[i] * axis[i] for i in range(3))
            half = sum(abs(axis[i]) * size[i] / 2.0 for i in range(3))
            # Inner face = the one nearer the jaw plane (the joint origin).
            faces[jaw] = abs(along) - half
            # Which way the jaw moves as it closes, along +axis.
            travel[jaw] = q_closed * (1.0 if along < 0 else -1.0)
        assert _close((faces["r"], faces["l"]),
                      (GRIPPER_JAW_STROKE, GRIPPER_JAW_STROKE), 1e-4), \
            f"{side} inner jaw faces at {faces}, expected 35 mm each at q = 0"
        assert travel["r"] > 0 and travel["l"] > 0, (
            f"{side} jaws do not both close INWARD at |q| = 0.035: {travel}")


def test_gripper_mount_frame_is_the_tool_flange():
    """The gripper is bolted to TCP_Link with no translation, so its registered
    TCP (136 mm along flange +z) lands where the tool really is. Cross-checked
    against the YUBI jaw tips the same arm used to carry: the two agree to
    within 5 mm along the approach axis, which is what makes the 136 mm tool
    config valid for both."""
    tfs = _gripper_model().link_transforms({})
    yubi = UrdfModel(D1_WB).link_transforms({})
    for side in ("R", "L"):
        flange = tfs[f"TCP_Link_{side}"]
        base = tfs[f"gripper_{side}_base_link"]
        assert _close(base.t, flange.t, 1e-12), "mount translation must be zero"
        approach = tuple(base.R[i][2] for i in range(3))
        assert _close(approach, tuple(flange.R[i][2] for i in range(3)), 1e-9)

        tcp = tuple(base.t[i] + GRIPPER_TCP_Z * approach[i] for i in range(3))
        tip = tuple(base.t[i] + GRIPPER_JAW_TIP_Z * approach[i] for i in range(3))
        # YUBI jaw-tip midpoint: 93.43 mm along the hand root's +x
        hand = yubi[f"yubi_{side}_hand_root"]
        yubi_tip = tuple(hand.t[i] + 0.09343 * hand.R[i][0] for i in range(3))
        along = sum((yubi_tip[i] - tip[i]) * approach[i] for i in range(3))
        assert abs(along) < 5e-3, (
            f"{side} gripper jaw tips are {along * 1000:.1f} mm from where the "
            "YUBI jaw tips were — the mount frame is probably wrong")
        # the TCP sits between the flange and the jaw tips, on the tool axis
        assert 0.0 < GRIPPER_TCP_Z < GRIPPER_JAW_TIP_Z
        assert _close(tcp, tuple(base.t[i] + GRIPPER_TCP_Z * approach[i]
                                 for i in range(3)), 1e-12)


def test_gripper_handedness_is_mirrored_pose_identical_part():
    """The two arms wear the SAME part the SAME way up, in mirrored poses.

    At the zero (T-pose) configuration:
      * the two gripper bases sit at mirrored positions;
      * the approach axis points outward along each arm (+y on _R, -y on _L);
      * the jaw-travel LINE is fore/aft on both (its direction is free — the
        two jaws are symmetric, so a half turn about the approach axis only
        swaps which jaw is called `r`);
      * the gripper's own +y ("up") points UP on both arms.
    The last one is the whole reason `_R` carries a half turn about the
    approach axis and `_L` does not: TCP_Link_R has +y DOWN and TCP_Link_L has
    +y UP, so mounting the part identically on both flanges would put it upside
    down on one arm. Remember `_R` is the vendor unit on the PHYSICAL LEFT.
    Tolerance 2e-4 throughout: the vendor rpy values are rounded (1.5708).
    """
    tfs = _gripper_model().link_transforms({})
    axes, pos = {}, {}
    for side in ("R", "L"):
        tf = tfs[f"gripper_{side}_base_link"]
        pos[side] = tf.t
        axes[side] = {ax: tuple(tf.R[i][k] for i in range(3))
                      for k, ax in enumerate("xyz")}

    # mirrored placement
    assert _close(pos["L"], (pos["R"][0], -pos["R"][1], pos["R"][2]), 1e-5), \
        f"gripper bases are not mirrored: {pos}"
    # approach: outward along each arm
    assert _close(axes["R"]["z"], (0.0, 1.0, 0.0), 2e-4)
    assert _close(axes["L"]["z"], (0.0, -1.0, 0.0), 2e-4)
    # jaw travel: the same fore/aft LINE on both arms, sign free
    for side in ("R", "L"):
        assert abs(abs(axes[side]["x"][0]) - 1.0) < 2e-4, \
            f"{side} jaws do not travel fore/aft: {axes[side]['x']}"
    mirrored_L = (axes["L"]["x"][0], -axes["L"]["x"][1], axes["L"]["x"][2])
    dot = sum(axes["R"]["x"][i] * mirrored_L[i] for i in range(3))
    assert abs(abs(dot) - 1.0) < 2e-4, \
        f"jaw-travel lines are not mirror images (|dot| = {abs(dot)})"
    # up is up on both — this is the check that catches a swapped pair
    for side in ("R", "L"):
        assert axes[side]["y"][2] > 0.999, (
            f"the {side} gripper is mounted upside down: its +y is "
            f"{axes[side]['y']}, expected +z")


def test_gripper_mass_is_the_measured_15kg_not_the_cad_mass():
    """Shu weighed the gripper at 1.5 kg (2026-07-29), confirming the value
    this SDK registers as ToolConfig::defaultGripper(). The vendor CAD claimed
    0.3279 kg — a shell-only export, 4.6x light. The URDF must carry the
    measured figure: anything loading this file for dynamics would otherwise
    get a 4.6x-light tool with CAD provenance making it look authoritative.

    The camera plate (78 g, mesh volume x aluminium book density) and the
    placeholder camera module (30 g) sit on their OWN links and are NOT part
    of the weighed 1.5 kg — nor of the registered tool config, deliberately
    (registering the extra ~108 g is a controller-facing decision)."""
    root = ET.parse(D1_WB_GRIPPER).getroot()
    for side in ("R", "L"):
        masses = {l.get("name"): float(l.find("inertial/mass").get("value"))
                  for l in root.findall("link")
                  if l.get("name", "").startswith(f"gripper_{side}_")}
        gripper = sum(m for n, m in masses.items()
                      if "camera" not in n and "cam" not in n)
        assert abs(gripper - 1.5) < 1e-4, f"{side} gripper mass {gripper}, want 1.5"
        assert abs(gripper - 0.327917) > 1.0, "the CAD mass must not have come back"
        assert abs(masses[f"gripper_{side}_camera_plate"] - 0.1053) < 1e-4
        assert abs(masses[f"gripper_{side}_wrist_cam_link"] - 0.030) < 1e-6


def test_gripper_wrist_camera_sits_up_top_and_looks_15deg_into_the_jaws():
    """The arm-end plate holds the camera on the gripper's +y ("up" on both
    arms, by the same half-turn normalisation the handedness test pins), and
    its lens — wrist_cam_link +x, the family lens convention — is the approach
    axis pitched 15 deg TOWARD the fingers.

    Cross-checked against the robot (2026-08-21): FK of the d1 teleop dataset
    at a grasp frame puts the camera exactly where the head camera sees the
    real bracket (top of the wrist), and the real wrist streams show the jaws
    at the BOTTOM edge of the image, which is the +y="image down" choice here.
    """
    tfs = _gripper_model().link_transforms({})
    pos = {}
    for side in ("R", "L"):
        base = tfs[f"gripper_{side}_base_link"]
        cam = tfs[f"gripper_{side}_wrist_cam_link"]
        pos[side] = cam.t
        # camera above the wrist: at the zero pose the gripper's +y is world
        # up, so the camera must sit ABOVE the flange
        assert cam.t[2] - base.t[2] > 0.070, (
            f"{side} wrist camera is not on top of the wrist "
            f"(dz = {cam.t[2] - base.t[2]:.4f} m)")
        # lens = cam +x, at 15 deg to the approach axis (base +z), leaning
        # toward the fingers
        lens = tuple(cam.R[i][0] for i in range(3))
        approach = tuple(base.R[i][2] for i in range(3))
        dot = sum(lens[i] * approach[i] for i in range(3))
        assert abs(dot - math.cos(math.radians(15.0))) < 2e-4, (
            f"{side} lens is {math.degrees(math.acos(max(-1, min(1, dot)))):.2f}"
            " deg off the approach axis, want 15")
        # image down (cam +y): from the camera's perch above the wrist it
        # must lean back DOWN toward the flange/centreline (negative along
        # the approach axis, -sin 15 deg) — that is what projects the fingers
        # below the image centre.  Rolled 180 deg it would be +sin 15.
        down = tuple(cam.R[i][1] for i in range(3))
        assert sum(down[i] * approach[i] for i in range(3)) < -0.2, (
            f"{side} image-down points {down}; the jaws would sit at the TOP "
            "edge of the image, and the real streams show them at the bottom")
    assert _close(pos["L"], (pos["R"][0], -pos["R"][1], pos["R"][2]), 1e-5), \
        f"wrist cameras are not mirrored: {pos}"


def test_gripper_assembly_com_is_the_validated_68mm():
    """The one COM figure with hardware behind it: the wrist stopped sagging
    when this lever moved from 0 to 68 mm (arm.h). base_link's COM is solved to
    hit it, so recomputing it from the URDF's own links is a real check.

    The camera plate and module are excluded the same way they are excluded
    from the 1.5 kg: the 68 mm lever was validated for the bare gripper, and
    their ~108 g rides separate links on purpose."""
    root = ET.parse(D1_WB_GRIPPER).getroot()
    jz = float(root.find("joint[@name='gripper_R_tcp_r_joint']/origin")
               .get("xyz").split()[2])
    total = moment = 0.0
    for link in root.findall("link"):
        name = link.get("name", "")
        if (not name.startswith("gripper_R_")
                or "camera" in name or "cam" in name):
            continue
        mass = float(link.find("inertial/mass").get("value"))
        com = [float(v) for v in link.find("inertial/origin").get("xyz").split()]
        # a jaw's own frame maps its local +x onto the flange +z (rpy pi,-pi/2,0)
        z = com[2] if name.endswith("_base_link") else com[0] + jz
        total += mass
        moment += mass * z
    assert abs(moment / total - 0.068) < 1e-4, \
        f"assembly COM is {moment / total * 1000:.2f} mm, want 68.0 mm"


@pytest.mark.parametrize("path", [D1_WB, D1_WB_GRIPPER],
                         ids=["d1_wholebody", "d1_wholebody_gripper"])
def test_wholebody_meshes_resolve_from_a_fresh_checkout(path):
    """The whole-body files are the assets OTHER repositories consume, so every
    mesh reference must resolve with nothing but this checkout — relative to the
    URDF file, not `package://`, not absolute, not relative to the caller's
    cwd."""
    root = ET.parse(path).getroot()
    base = os.path.dirname(path)
    refs = [m.get("filename") for m in root.iter("mesh")]
    # 4 hifi body + 18 arm + 4 gripper (the ten vendor body meshes merged
    # into one CAD-split visual per host link)
    assert len(refs) >= 26, f"only {len(refs)} mesh refs — geometry went missing"
    for ref in refs:
        assert not ref.startswith("package://"), f"{ref} needs a ROS workspace"
        assert not os.path.isabs(ref), f"{ref} is absolute"
        full = os.path.normpath(os.path.join(base, ref))
        if not os.path.isfile(full) and is_optional(ref):
            # The decorative body visuals are an optional ~50 MB layer this
            # repo does not carry (rendering only; the guard and collision
            # never see them). `mkit-urdf fetch-visuals` drops them in.
            continue
        if assets.is_external(full):
            # CAD with unresolved redistribution rights (LICENSE-STATUS.md).
            # The REFERENCE is the thing this test is about and it is checked
            # above; the bytes live in manipulation-kit-assets and are opened
            # here only when a checkout is configured.
            resolved = assets.resolve(
                os.path.relpath(full, assets.PACKAGE_ROOT))
            if resolved is None:
                continue
            full = str(resolved)
        assert os.path.isfile(full), f"missing mesh {full}"
        if ref.endswith(".obj"):                        # hifi body visuals
            with open(full) as f:
                body = f.read()                          # v/vn come first,
            assert "\nv " in body and "\nf " in body, \
                f"{ref} has no geometry"                 # faces are at the end
            assert "\nvn " in body, f"{ref} lost its vertex normals"
            continue
        with open(full, "rb") as f:
            blob = f.read()
        count = int.from_bytes(blob[80:84], "little")   # binary STL
        assert len(blob) == 84 + 50 * count, f"truncated STL {ref}"
        assert count > 0, f"{ref} has zero triangles"


def test_guard_model_stays_mesh_free():
    """d1.urdf is the motion-guard model: pyguard parses it directly and it has
    to load on the robot, in a planner or in a bare sim with ZERO mesh assets.
    The whole-body variants carry the real CAD; this one must not."""
    assert not list(ET.parse(D1).getroot().iter("mesh")), \
        "d1.urdf must stay mesh-free — it is the guard model"


@pytest.mark.parametrize("path", [D1, D1_WB, D1_WB_GRIPPER],
                         ids=["d1", "d1_wholebody", "d1_wholebody_gripper"])
def test_collision_geometry_is_always_primitives(path):
    """Visual and collision are deliberately different answers in the `d1/`
    family: the CAD is what the robot LOOKS like, primitives are what it
    collides with (fast, stable contact, and what the guard and the C++
    validators mirror). A mesh must never leak into a <collision> here.
    (d1_yubi_description_v2 is a different package and does use mesh
    collision — that is its own choice, not this family's.)"""
    for col in ET.parse(path).getroot().iter("collision"):
        assert col.find("geometry/mesh") is None, \
            f"{os.path.basename(path)}: collision geometry must stay primitives"


def test_arm_meshes_are_placed_at_identity():
    """This file's J1..J7+TCP chain is byte-derived from the vendor D1 arm
    URDFs, so the link frames are IDENTICAL and the vendor mesh drops in with
    no transform. If a nonzero origin ever appears here, either the chain
    drifted or someone started guessing offsets."""
    root = ET.parse(D1_WB_GRIPPER).getroot()
    seen = 0
    for link in root.findall("link"):
        for vis in link.findall("visual"):
            mesh = vis.find("geometry/mesh")
            if mesh is None or "d1_arm" not in mesh.get("filename"):
                continue
            seen += 1
            origin = vis.find("origin")
            assert _close(_f3(origin.get("xyz")), (0, 0, 0), 1e-12)
            assert _close(_f3(origin.get("rpy")), (0, 0, 0), 1e-12)
    assert seen == 18, f"expected 18 arm meshes (9 links x 2 arms), got {seen}"


def _f3(text):
    return tuple(float(v) for v in (text or "0 0 0").split())


def test_body_cad_lands_where_the_generator_measured_independently():
    """The hifi body visuals are pre-baked in LINK-LOCAL coordinates (split out
    of the 2026-08-23 full-robot CAD), so two things must hold or the robot
    renders askew: every hifi visual mounts at IDENTITY, and the chassis mesh's
    own geometry stands on the floor — its wheels bottom out at z=0 in
    chassis_link, which IS the floor frame. The z-extents also pin the
    static/moving split of the lift column: the STATIC chassis mesh includes
    the outer column shells (top ≥ 0.8 m), which must NOT ride the lift."""
    root = ET.parse(D1_WB_GRIPPER).getroot()
    hifi_mounts = {}
    for link in root.findall("link"):
        for vis in link.findall("visual"):
            mesh = vis.find("geometry/mesh")
            if mesh is not None and "body_hifi" in mesh.get("filename"):
                hifi_mounts[link.get("name")] = _f3(
                    vis.find("origin").get("xyz"))
    assert sorted(hifi_mounts) == ["chassis_link", "head_link",
                                   "neck_pan_link", "torso_column"], \
        f"hifi visuals on {sorted(hifi_mounts)}"
    for name, xyz in hifi_mounts.items():
        assert _close(xyz, (0.0, 0.0, 0.0), 1e-9), \
            f"{name} hifi visual not at identity: {xyz}"
    hifi = os.path.join(DESCRIPTION, "d1", "meshes", "body_hifi",
                        "chassis_link_hifi.obj")
    if not os.path.isfile(hifi):
        pytest.skip("optional visual layer absent (mkit-urdf fetch-visuals); "
                    "the mount-at-identity half of this test still ran")
    with open(hifi) as f:
        zs = [float(line.split()[3]) for line in f if line.startswith("v ")]
    lo_z, hi_z = min(zs), max(zs)
    assert abs(lo_z) < 0.01, f"chassis hifi wheels at z={lo_z}, want the floor"
    assert hi_z > 0.8, f"chassis hifi top at z={hi_z}: the static outer lift " \
        "column belongs to the chassis, not to the lift"


# ------------------------------------------------------------ export flavours
EXPORT_TOOL = os.path.join(DESCRIPTION, "tools", "export_description.py")



def _export(flavour, dest):
    subprocess.run([sys.executable, EXPORT_TOOL, flavour, "--dest", dest],
                   check=True, capture_output=True)


@pytest.mark.parametrize("flavour", ["d1_yubi", "d1_wholebody_gripper",
                                     "d1_collision"])
def test_every_export_flavour_resolves_its_own_meshes(flavour):
    """An export a consumer cannot load is worse than no export.

    This is a regression test with a real history: `d1_collision` shipped
    `d1_wholebody.urdf` alongside the mesh-free `d1.urdf`, and once the
    whole-body variants gained real CAD visuals that copy arrived with all 32
    of its <mesh> references dangling — it parsed, and then failed to load.
    Every reference in every exported URDF must resolve RELATIVE TO THE
    EXPORTED FILE, which is how simulators outside ROS find meshes.
    """
    with tempfile.TemporaryDirectory() as dest:
        _export(flavour, dest)
        urdfs = [f for f in sorted(os.listdir(dest)) if f.endswith(".urdf")]
        assert urdfs, f"{flavour} exported no URDF"
        for name in urdfs:
            for mesh in ET.parse(os.path.join(dest, name)).getroot().iter("mesh"):
                ref = mesh.get("filename")
                assert not ref.startswith("package://"), \
                    f"{flavour}/{name} still has a package:// URI: {ref}"
                assert not os.path.isabs(ref), \
                    f"{flavour}/{name} has an absolute mesh path: {ref}"
                if not os.path.exists(os.path.join(dest, ref)):
                    with open(os.path.join(dest, "PROVENANCE.json")) as pf:
                        manifest = json.load(pf)
                    declared = (set(manifest["absent_optional"])
                                | set(manifest["absent_external"]))
                    assert ref in declared, (
                        f"{flavour}/{name} references {ref}, which was neither "
                        "exported nor declared absent in PROVENANCE.json")
                    continue
                assert os.path.exists(os.path.join(dest, ref)), \
                    f"{flavour}/{name} references {ref}, which was not exported"


def test_wholebody_gripper_export_is_the_whole_robot():
    """The authoritative-asset export carries the body, both arms and the
    gripper — not a subset that renders as half a robot."""
    with tempfile.TemporaryDirectory() as dest:
        _export("d1_wholebody_gripper", dest)
        refs = {m.get("filename") for m in ET.parse(
            os.path.join(dest, "d1_wholebody_gripper.urdf")
        ).getroot().iter("mesh")}
    body = {r for r in refs if r.startswith("meshes/body_hifi/")}
    arms = {r for r in refs if r.startswith("d1_arm/")}
    grip = {r for r in refs if r.startswith("meshes/gripper/")}
    with open(os.path.join(DESCRIPTION, "d1", "meshes", "body_hifi",
                           "color_regions.json")) as stream:
        regions = json.load(stream)
    expected_body = {
        f"meshes/body_hifi/{host}_{color}.obj"
        for host, data in regions["hosts"].items() for color in data["faces"]
    }
    assert body == expected_body, sorted(body)
    assert len(arms) == 18, sorted(arms)
    assert len(grip) == 4, sorted(grip)
    assert refs == body | arms | grip, sorted(refs - (body | arms | grip))


# ------------------------------------------------------------- camera frames
CAMERA_MODELS = {"d1_wholebody.urdf": D1_WB,
                 "d1_wholebody_gripper.urdf": D1_WB_GRIPPER}


def _stl_vertices(path):
    """Every vertex of a binary STL — an INDEPENDENT reader, so the head camera
    check below re-derives the frame instead of trusting the generator's."""
    size = os.path.getsize(path)
    with open(path, "rb") as f:
        count = int.from_bytes(f.read(84)[80:84], "little")
        assert size == 84 + 50 * count, f"{path} is not a binary STL"
        out = []
        for _ in range(count):
            import struct as _s
            v = _s.unpack("<12f", f.read(50)[:48])
            out += [v[3:6], v[6:9], v[9:12]]
        return out


def _link_names(path):
    return [l.get("name") for l in ET.parse(path).getroot().findall("link")]


@pytest.mark.parametrize("name,path", sorted(CAMERA_MODELS.items()))
def test_camera_frames_exist_only_where_they_should(name, path):
    """The whole-body variants carry camera frames; the guard model does not.

    d1.urdf is a keep-out model — pyguard enumerates its links and a camera
    frame is not a volume — so cameras are deliberately absent there. Each
    whole-body variant carries its own wrist cameras: the YUBI's sit on the
    hand, the gripper's on the arm-end camera plate (V2.0, 2026-08-21).
    """
    links = _link_names(path)
    assert "head_camera_link" in links
    assert "head_camera_optical_frame" in links

    wrists = [n for n in links if n.endswith("_cam_optical_frame")]
    if "gripper" in name:
        assert sorted(wrists) == ["gripper_L_wrist_cam_optical_frame",
                                  "gripper_R_wrist_cam_optical_frame"]
    else:
        assert sorted(wrists) == ["yubi_L_hand_cam_optical_frame",
                                  "yubi_R_hand_cam_optical_frame"]

    guard_links = _link_names(D1)
    assert not [n for n in guard_links if "camera" in n or "optical" in n], \
        "d1.urdf is the safety model; keep camera frames out of it"


@pytest.mark.parametrize("name,path", sorted(CAMERA_MODELS.items()))
def test_optical_frames_put_the_lens_on_plus_z(name, path):
    """Every ``*_optical_frame`` is the ROS optical convention relative to its
    parent camera link: the parent's +x (the lens direction on both D1 camera
    families) becomes the optical +z, image right is +x and image down is +y.

    This is the fact a task config cannot be trusted to know. d1-isaaclab
    mounted renderers on the raw camera links with ``convention="ros"``, which
    puts the optical axis on +z, and rendered two ceilings and the inside of
    the head shell — scoring a believable 0/4 with nothing raised.
    """
    want = ((0.0, 0.0, 1.0),      # optical x = parent -z
            (0.0, 1.0, 0.0),      # optical y = parent +y
            (-1.0, 0.0, 0.0))     # optical z = parent +x  <- the lens
    frames = [l for l in _link_names(path) if l.endswith("_optical_frame")]
    assert frames, f"{name} carries no optical frames"
    for frame in frames:
        xyz, R, _axis, _lims = _joint_transform(path, f"{frame}_joint")
        assert _close(xyz, (0.0, 0.0, 0.0), 1e-9), \
            f"{frame} must be a pure rotation of its camera link, got {xyz}"
        assert _R_close(R, want), (
            f"{frame} does not map its parent's +x onto the optical +z; a "
            f"renderer mounted there would look the wrong way. R = {R}")


@needs_assets
def test_head_camera_is_the_front_face_of_the_vendor_housing():
    """The head camera frame is SOLVED from the vendor mesh, and this re-derives
    it independently: read ``headcamera_Link.STL``, take its AABB in the frame
    the mesh is drawn in, and the frame must sit at the centre of the +x face.

    Why the +x face is the lens face: in ``head_link`` coordinates the housing
    is a 27 x 26 x 90 mm bar — 90 mm wide ACROSS the robot (head_link +z is
    dual_base +y) and only 27 mm deep. A bar like that faces along its shallow
    axis. NOMINAL, not calibrated: the lens offset within a multi-sensor bar
    (order 1 cm) is not modelled, and nor is the 180 deg roll about the lens.
    """
    # The visual on head_link is the CAD-split hifi mesh; the vendor housing
    # is no longer DRAWN, but it is still the file the mount is SOLVED from,
    # and it rides head_link at identity (BODY_MESH_HOSTS) — so the AABB in
    # file coordinates is already head_link-local, same as ever.
    pts = _stl_vertices(str(assets.resolve(
        "description/d1/meshes/body/headcamera_Link.STL")))
    lo = [min(p[i] for p in pts) for i in range(3)]
    hi = [max(p[i] for p in pts) for i in range(3)]
    # a bar: shallow in x, 90 mm across in z
    assert hi[0] - lo[0] < 0.035 and hi[2] - lo[2] > 0.08, \
        f"housing is not the expected bar shape: {[hi[i] - lo[i] for i in range(3)]}"
    want = (hi[0], (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0)

    for path in CAMERA_MODELS.values():
        xyz, R, _axis, _lims = _joint_transform(path, "head_camera_mount")
        assert _close(xyz, want, 1e-6), (
            f"{os.path.basename(path)}: head_camera_link at {xyz}, but the "
            f"vendor housing's front-face centre is {want}")
        # The mount carries ONE real rotation: the 17.3 deg downward pitch
        # measured off the D435 slab in the 2026-08-23 full-robot CAD (about
        # head_link +z, the lateral axis). Everything optical still belongs
        # on head_camera_optical_frame.
        c, sn = math.cos(0.301131), math.sin(0.301131)
        assert _R_close(R, ((c, -sn, 0), (sn, c, 0), (0, 0, 1))), \
            "head_camera_mount must pitch the lens 17.3 deg down (CAD-" \
            "measured); optical rotation belongs on the optical frame"


#: What a camera frame is allowed to weigh, in kg. Not zero, and not absent —
#: see :func:`generate_d1_urdf.frame_link`. Four frames at 1e-6 kg is 4
#: micrograms on a 150 kg robot.
FRAME_MASS_KG = 1e-6


def test_camera_frames_weigh_effectively_nothing():
    """Camera frames are FRAMES: no geometry, and a mass that is stated and
    negligible.

    Stated, not omitted, and that distinction is measured rather than assumed:
    an inertial-less link imported into Isaac Sim with merge_fixed_joints=False
    (which a consumer needs, or the frame is collapsed away) had a DEFAULT mass
    substituted by PhysX — each hand came out at 2.5 kg against its measured
    1.5, and the robot gained 4.8 kg with nothing in the URDF to point at.
    Whole-body mass is load-bearing here: the lift actuator sizing and the
    1.5 kg-per-side gripper correction both read it.
    """
    for path in CAMERA_MODELS.values():
        for link in ET.parse(path).getroot().findall("link"):
            name = link.get("name")
            if not (name.endswith("_optical_frame")
                    or name == "head_camera_link"):
                continue
            assert link.find("collision") is None, f"{name} carries collision"
            assert link.find("visual") is None, f"{name} carries visual"
            inertial = link.find("inertial")
            assert inertial is not None, (
                f"{name} has no <inertial>. That reads as massless and is not: "
                "a physics importer will substitute a default. State it.")
            mass = float(inertial.find("mass").get("value"))
            assert mass == pytest.approx(FRAME_MASS_KG, rel=1e-6), \
                f"{name} weighs {mass} kg, want {FRAME_MASS_KG}"


@needs_assets
def test_body_meshes_are_the_vendor_cad():
    """description/d1/meshes/body/ is a straight copy of the vendor package —
    not decimated, because simplifying it to a 4 k-triangle budget moved the
    surface by up to 48 mm. Zero-triangle vendor placeholders are excluded."""
    stls = sorted(BODY_MESH_NAMES)
    assert len(stls) == 10, f"expected 10 body meshes, got {stls}"
    for name in stls:
        path = assets.resolve(f"description/d1/meshes/body/{name}")
        assert path is not None, f"{name} not in the assets checkout"
        with open(path, "rb") as f:
            blob = f.read()
        count = int.from_bytes(blob[80:84], "little")
        assert count > 100, f"{name} has {count} triangles"
        assert len(blob) == 84 + 50 * count


@needs_assets
def test_gripper_meshes_match_the_hands_package():
    """``hands/d1/parallel_gripper/descriptions/meshes/`` owns the gripper
    geometry; ``description/d1/meshes/gripper/`` is a vendored copy the URDF
    generator references. Both now live in THIS repo, so what used to be a
    cross-repo check that skipped itself whenever a sibling checkout was
    missing is now unconditional — which is the point of consolidating."""
    for name in sorted(GRIPPER_MESH_NAMES):
        here = assets.resolve(f"description/d1/meshes/gripper/{name}")
        src = assets.resolve(
            f"hands/d1/parallel_gripper/descriptions/meshes/{name}")
        assert here is not None and src is not None, \
            f"{name} not in the assets checkout"
        with open(here, "rb") as f:
            mine = hashlib.sha256(f.read()).hexdigest()
        with open(src, "rb") as f:
            theirs = hashlib.sha256(f.read()).hexdigest()
        assert mine == theirs, (
            f"description/d1/meshes/gripper/{name} has forked from "
            "hands/d1/parallel_gripper/descriptions/meshes/ — edit it there "
            "and re-copy, never here")


# ------------------------------------- the bigger tool still clears the body
#
# The stock gripper's jaw rail is 160 mm across; the YUBI palm it replaces is
# 67 mm. Swapping in a tool more than twice as wide is exactly the change that
# turns a pose the guard has always accepted into a collision, and it would
# show up first as a self-collision on the real robot, not in a diff. So the
# clearance is a NUMBER with a test on it.
CLEARANCE_POSES = {
    "zero": (0.0,) * 7,
    "home": (-0.8466, 1.2728, 1.3167, -1.9167, 1.2020, -0.1268, 0.3163),
    "elbow-to-chest": (-0.35, -0.70, 0.30, -0.86, 0.07, -0.27, -0.88),
    "arm-crossing": (0.01, -0.93, -0.13, -0.86, -0.82, -0.15, 0.65),
    "folded-EE": (-0.75, -0.55, 0.25, 0.90, 0.15, -0.21, 0.95),
}
BODY_KEEPOUT_LINKS = ("torso_column", "chassis_link", "head_link",
                      "neck_pan_link")
#: Measured margin at the tightest pose (home) is 114 mm. 50 mm is the line
#: below which someone should look again, not the line where it breaks.
MIN_TOOL_BODY_CLEARANCE_M = 0.050


def _world_aabb(tf, half, ctr):
    pts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                loc = (ctr[0] + sx * half[0], ctr[1] + sy * half[1],
                       ctr[2] + sz * half[2])
                pts.append(tuple(
                    sum(tf.R[i][k] * loc[k] for k in range(3)) + tf.t[i]
                    for i in range(3)))
    return ([min(p[i] for p in pts) for i in range(3)],
            [max(p[i] for p in pts) for i in range(3)])


def _collision_boxes(path, keep):
    out = []
    for link in ET.parse(path).getroot().findall("link"):
        name = link.get("name")
        if not keep(name):
            continue
        for col in link.findall("collision"):
            box = col.find("geometry/box")
            if box is None or "_exempt" in (col.get("name") or ""):
                continue
            size = [float(v) for v in box.get("size").split()]
            org = col.find("origin")
            ctr = [float(v) for v in
                   ((org.get("xyz") if org is not None else None)
                    or "0 0 0").split()]
            out.append((name, col.get("name") or name,
                        [s / 2 for s in size], ctr))
    return out


def test_gripper_clears_the_body_at_every_pose_the_guard_accepts():
    model = UrdfModel(D1_WB_GRIPPER)
    tool = _collision_boxes(D1_WB_GRIPPER, lambda n: n.startswith("gripper_"))
    body = _collision_boxes(D1_WB_GRIPPER, lambda n: n in BODY_KEEPOUT_LINKS)
    assert tool and body
    for pose, qR in CLEARANCE_POSES.items():
        q = {f"Joint{i + 1}_R": qR[i] for i in range(7)}
        q.update({f"Joint{i + 1}_L": MIRROR_SIGN[i] * qR[i] for i in range(7)})
        tfs = model.link_transforms(q)
        worst, what = 1e9, ""
        for tlink, tname, thalf, tctr in tool:
            ta = _world_aabb(tfs[tlink], thalf, tctr)
            for blink, bname, bhalf, bctr in body:
                ba = _world_aabb(tfs[blink], bhalf, bctr)
                sep = max(max(ta[0][i] - ba[1][i], ba[0][i] - ta[1][i])
                          for i in range(3))
                if sep < worst:
                    worst, what = sep, f"{tname} vs {bname}"
        assert worst > MIN_TOOL_BODY_CLEARANCE_M, (
            f"pose {pose!r}: the gripper comes within {worst * 1000:.1f} mm of "
            f"the body ({what}). The stock gripper is 160 mm wide against the "
            "YUBI hand's 67 mm — re-check the guard before shipping this.")


def test_no_unmodelled_void_between_the_flange_and_the_gripper():
    """The vendor gripper CAD has ZERO geometry below z = 16.5 mm — it starts at
    the gripper's own mount plate and models nothing reaching back to the arm.
    Link7's mesh ends exactly at the flange face (z = 0), so 16.5 mm of real
    hardware is missing from the export. (It IS real: the registered 136 mm TCP
    and the YUBI jaw tips at 148.4 mm vs this CAD's 143.5 mm both confirm the
    gripper sits where it is. Moving it 16.5 mm flusher would put the TCP at
    ~119.5 mm and contradict the value validated on the robot.)

    A void there is a hole a planner could swing something through. The first
    8 mm is now the REAL arm-end camera plate (its own link, welded at
    identity, so its z spans compose directly); the 8..16.5 mm remainder is
    still the ASSUMED gripper-end plate box. This test pins that the collision
    boxes across both links tile the tool axis CONTIGUOUSLY from the flange
    face outward, with no gap — so neither fill can be dropped by accident."""
    root = ET.parse(D1_WB_GRIPPER).getroot()
    for side in ("R", "L"):
        spans = []
        for link_name in (f"gripper_{side}_base_link",
                          f"gripper_{side}_camera_plate"):
            link = next(l for l in root.findall("link")
                        if l.get("name") == link_name)
            for col in link.findall("collision"):
                box = col.find("geometry/box")
                assert box is not None
                size = [float(v) for v in box.get("size").split()]
                ctr = [float(v) for v in col.find("origin").get("xyz").split()]
                spans.append((ctr[2] - size[2] / 2, ctr[2] + size[2] / 2))
        spans.sort()
        assert abs(spans[0][0]) < 1e-9, (
            f"{side}: collision starts at {spans[0][0] * 1000:.2f} mm, not the "
            "flange face — the 16.5 mm void is open again")
        reach = spans[0][1]
        for lo, hi in spans[1:]:
            assert lo <= reach + 1e-9, (
                f"{side}: gap in the collision model from "
                f"{reach * 1000:.2f} to {lo * 1000:.2f} mm along the tool axis")
            reach = max(reach, hi)
        assert reach > 0.070, f"{side}: collision only reaches {reach * 1000:.1f} mm"


def test_the_assumed_adapter_is_labelled_as_assumed():
    """It is not vendor geometry and must never read as though it were.
    Since the arm-end camera plate arrived, the assumed part is only the
    gripper-end plate spanning z = 8 .. 16.5 mm."""
    text = open(D1_WB_GRIPPER).read()
    assert text.count("gripper_adapter_ASSUMED") == 2
    assert "ASSUMED gripper-end plate spanning the camera plate" in text


# ---------------------------------------------------------------------------
# HOME pose: exactly one file owns it, repo-wide
# ---------------------------------------------------------------------------

CANONICAL_HOME = os.path.join(CONFIG, "home_pose.json")


def _tracked_files():
    """Every file git tracks in this repo, or ``None`` outside a checkout."""
    try:
        out = subprocess.run(["git", "-C", REPO, "ls-files"],
                             capture_output=True, text=True, check=True)
    except (subprocess.CalledProcessError, OSError):
        return None
    return [p for p in out.stdout.splitlines() if p]


def test_only_one_real_home_pose_file():
    """`devices/omakase_arm/config/home_pose.json` is the ONLY home_pose.json
    with content; any other must be a symlink pointing at it.

    The description package used to carry its own radians copy and sat a full
    pose (the pre-2026-06 "だらん") behind the robot for weeks. A second real
    file is the bug, so it is banned outright rather than diffed.
    """
    tracked = _tracked_files()
    if tracked is None:
        pytest.skip("not a git checkout (installed wheel); "
                    "this test scans tracked files")
    reals, links = [], []
    for rel in tracked:
        if os.path.basename(rel) != "home_pose.json":
            continue
        path = os.path.join(REPO, rel)
        (links if os.path.islink(path) else reals).append(path)

    assert reals == [CANONICAL_HOME], (
        "home_pose.json must exist exactly once as a real file "
        f"({CANONICAL_HOME}); found {reals}")
    for link in links:
        assert os.path.realpath(link) == os.path.realpath(CANONICAL_HOME), (
            f"{link} is a symlink but does not point at the canonical "
            f"home_pose.json (resolves to {os.path.realpath(link)})")


def test_home_pose_numbers_are_not_copied_into_docs_or_code():
    """No tracked file may reproduce the HOME angles as literals.

    Docs, headers and viewer constants that inline the numbers are copies that
    nothing updates. Consumers must read the file (or, cross-repo, the
    `/api/d1/home_pose` endpoint that reads it).
    """
    with open(CANONICAL_HOME) as f:
        pose = json.load(f)["home_pose"]
    # Two distinct, non-mirrored angles: matching BOTH in one file means the
    # pose was pasted in, not that a value coincidentally appears.
    needles = [f"{pose[1]:g}", f"{pose[3]:g}"]

    tracked = _tracked_files()
    if tracked is None:
        pytest.skip("not a git checkout (installed wheel); "
                    "this test scans tracked files")
    offenders = []
    for rel in tracked:
        path = os.path.join(REPO, rel)
        if os.path.islink(path) or os.path.samefile(path, CANONICAL_HOME):
            continue
        try:
            with open(path, encoding="utf-8") as f:
                text = f.read()
        except (UnicodeDecodeError, IsADirectoryError, FileNotFoundError):
            continue
        if all(n in text for n in needles):
            offenders.append(rel)

    assert not offenders, (
        "these files inline the HOME angles instead of reading "
        f"manipulation_kit/config/home_pose.json: {offenders}")
