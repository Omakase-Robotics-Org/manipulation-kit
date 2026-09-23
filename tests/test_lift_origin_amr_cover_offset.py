"""The upper body starts 29 mm above the CAD chain, and nothing inside it moved.

The built D1's AMR cover is taller than the CAD models it, so the whole upper
body — column, torso, both arms, neck, head — rides 29 mm higher than the
vendor chain says.  The description applies that once, at the lift joint
origin (``base_footprint -> dual_base``: 0.484 -> 0.513 m).  See the AMR COVER
HEIGHT OFFSET block in ``description/d1/tools/generate_d1_urdf.py``.

MEASURED by Shu with a tape on d1-3, 2026-09-16, lift at 0, floor-referenced,
±2 mm.  The fit is over four frames spanning 660 mm of the robot:

    frame                              URDF+29    measured    residual
    torso body bottom                    689        700        -11 mm
    torso top                           1109.6     1120        -10 mm
    head RealSense (head_camera_link)   1262.7     1266         -3 mm
    head top                            1318.9     1322         -3 mm

Flat residuals over that span are what a single offset at the base looks like;
a wrong-sized correction applied INSIDE the upper body would fan out with
height.  (The rejected alternative — +53.83 mm added to the moving column,
fitted to a single 2026-09-13 gripper-clearance reading — gives +14…+27 mm and
rising.  That earlier reading and this body measurement disagree by ~25 mm;
the body measurement, being four frames rather than one, is the one adopted.)

What the checks below pin, and why each one is here:

1. **The offset is at the lift origin.**  0.513 m, and the 0…0.300 m travel
   untouched — the rail still reports what the actuator reports.
2. **Nothing inside ``dual_base`` moved.**  Arm mounts at the CAD 0.50 m, neck
   pan at 0.621 m, both torso cameras at their CAD heights, the torso keep-out
   boxes at their CAD sizes, and no leftover per-visual offsets on the column.
   This is what makes the change safe for every shoulder-relative consumer:
   the guard keep-outs, ``safety_zones.json``, the frozen ``safety_zones.h``
   and every IK result are bit-for-bit unchanged.
3. **The floor-referenced heights the tape actually read.**
4. **The AMR under the lift did not move**: chassis boxes stay floor-relative.
5. **One copy of the constant.**  Only the generator authors it; nothing else
   in the repository may spell a height correction out again.

FK is the package's own numpy chain (``manipulation_kit.arms.urdf_chain``) over
the shipped URDF — no simulator, no extra dependency.
"""
import importlib.util
import json
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest

import manipulation_kit
from manipulation_kit.arms.urdf_chain import ACTUATED, UrdfChain
from manipulation_kit.guard.urdf_model import UrdfModel

KIT = os.path.dirname(os.path.abspath(manipulation_kit.__file__))
CONFIG = os.path.join(KIT, "config")
DESCRIPTION = os.path.join(KIT, "description")
D1_WB_GRIPPER = os.path.join(DESCRIPTION, "d1", "d1_wholebody_gripper.urdf")

#: The measured correction (m) and the CAD lift zero it is added to.
OFFSET = 0.029
CAD_LIFT_ZERO = 0.484
LIFT_ZERO = CAD_LIFT_ZERO + OFFSET          # 0.513

#: Floor-referenced heights at q = 0 that the tape read, with the tolerance
#: the fit is claimed to: 12 mm covers the largest residual above plus the
#: ±2 mm the tape itself is good for.  Anything looser would not notice a
#: second wrong correction; anything tighter would be claiming a precision
#: a tape measure does not have.
MEASURED = {
    "head_camera_link": 1.266,   # head RealSense origin, neck level
}
MEASURED_TOLERANCE_M = 0.012

#: Zero-config world heights this description must produce, to the six
#: significant digits the generator emits.
ZERO_CONFIG_Z = {
    "dual_base": LIFT_ZERO,
    "torso_column": LIFT_ZERO,
    "Base_R": 1.013,
    "Base_L": 1.013,
    "Link1_R": 1.013,
    "Link1_L": 1.013,
    "neck_pan_link": 1.134,
    "head_link": 1.1895,
    "head_camera_link": 1.2627,
}
TOLERANCE_M = 1e-5

#: CAD origins inside ``dual_base`` that this change must leave alone.
CAD_MOUNT_Z = 0.50
CAD_NECK_PAN_Z = 0.621
CAD_CHEST_CAMERA_Z = 0.5653
CAD_BACK_FISHEYE_Z = 0.4822
CAD_TORSO_CORE_HEIGHT = 0.49

#: The right gripper is the ``_L`` tree (``+Y`` = ``_R`` = the PHYSICAL LEFT
#: arm; see the generator's side-naming note), and HOME's second seven joints
#: are the ones that drive it.
RIGHT_ARM_SUFFIX = "L"
RIGHT_ARM_HOME_OFFSET = 7


@pytest.fixture(scope="module")
def model():
    return UrdfModel(D1_WB_GRIPPER)


@pytest.fixture(scope="module")
def urdf_root():
    return ET.parse(D1_WB_GRIPPER).getroot()


def joint_origin(root, name):
    j = next(j for j in root.findall("joint") if j.get("name") == name)
    return [float(v) for v in j.find("origin").get("xyz").split()]


def home_pose_rad(offset):
    with open(os.path.join(CONFIG, "home_pose.json")) as stream:
        hp = json.load(stream)["home_pose"]
    return [math.radians(hp[offset + i]) for i in range(7)]


def link_pose(model, link, q):
    """4x4 of ``link`` in the URDF root frame, via the package's numpy FK."""
    path, cur = [], link
    while cur in model.parent_link:
        path.append(model.joints[cur])
        cur = model.parent_link[cur]
    names = [j.name for j in reversed(path) if j.type in ACTUATED]
    chain = UrdfChain(model, ee_body=link, joint_names=names)
    chain.set_joints([q.get(n, 0.0) for n in names])
    p, rot = chain.ee_pose()
    out = np.eye(4)
    out[:3, :3] = rot.as_matrix()
    out[:3, 3] = p
    return out


def collision_corners(model, link, q):
    """Every collision-box corner of ``link``, in the root frame."""
    world = link_pose(model, link, q)
    out = []
    for prim in model.collisions[link]:
        assert prim.kind == "box", f"{prim.name} is a {prim.kind}"
        local = np.eye(4)
        local[:3, :3] = np.array(prim.origin.R)
        local[:3, 3] = np.array(prim.origin.t)
        pose = world @ local
        half = np.array(prim.size) / 2.0
        corners = np.array([[sx * half[0], sy * half[1], sz * half[2]]
                            for sx in (-1, 1) for sy in (-1, 1)
                            for sz in (-1, 1)])
        out.append((prim.name, corners @ pose[:3, :3].T + pose[:3, 3]))
    return out


def gripper_assembly_bottom(model, side, q):
    """(lowest z, which collision box) over the WHOLE gripper assembly.

    "Complete assembly" is the jaws, the gripper body, AND the wrist-camera
    plate, which is the part that actually reaches lowest (16 mm below the
    jaws). Measuring the jaws alone reads 16 mm high, which is exactly how a
    correction like this gets set wrong.
    """
    best = (float("inf"), None)
    for link in model.links:
        if not link.startswith(f"gripper_{side}") or not model.collisions[link]:
            continue
        for name, corners in collision_corners(model, link, q):
            z = float(corners[:, 2].min())
            if z < best[0]:
                best = (z, name)
    return best


# ------------------------------------------- the offset, where it is applied
def test_the_offset_is_at_the_lift_joint_origin(model, urdf_root):
    """0.484 + 0.029, with the actuator's own travel untouched."""
    lift = model.joints["dual_base"]
    assert lift.name == "lift" and lift.type == "prismatic"
    assert abs(lift.origin.t[2] - LIFT_ZERO) < 1e-9, lift.origin.t
    assert abs(lift.origin.t[2] - CAD_LIFT_ZERO - OFFSET) < 1e-12
    assert (lift.lower, lift.upper) == (0.0, 0.3)
    assert joint_origin(urdf_root, "lift")[2] == pytest.approx(LIFT_ZERO)


def test_the_generator_is_the_only_author_of_the_offset():
    """One constant, in the file that emits the URDFs.

    Every other frame in this repository is expressed relative to ``dual_base``
    or to the shoulders, so nothing else has any business restating a height
    correction.  A second copy is how two files end up 25 mm apart.
    """
    generator = load_script("d1", "tools", "generate_d1_urdf.py")
    assert generator.AMR_COVER_HEIGHT_OFFSET == OFFSET
    assert generator.DUAL_BASE_GROUND_Z == pytest.approx(LIFT_ZERO)

    assembler = load_script("d1_yubi_description_v2", "tools",
                            "assemble_d1_yubi.py")
    from manipulation_kit.guard import guard
    for mod, label in ((assembler, "assemble_d1_yubi"), (guard, "guard.guard")):
        offenders = [n for n in dir(mod)
                     if ("COVER" in n and "OFFSET" in n) or "EXTENSION" in n]
        assert not offenders, (
            f"{label} restates a height correction ({offenders}); it works in "
            "the dual_base / shoulder frame and must not know about the lift")


def load_script(*parts):
    path = os.path.join(DESCRIPTION, *parts)
    spec = importlib.util.spec_from_file_location(parts[-1][:-3], path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# --------------------------------------- nothing INSIDE dual_base may move
def test_arm_mounts_and_neck_keep_their_cad_origins(urdf_root):
    """The shoulders sit at the CAD 0.50 m on the column, the neck at 0.621."""
    for name, y in (("mount_R", 0.037), ("mount_L", -0.037)):
        xyz = joint_origin(urdf_root, name)
        assert xyz == pytest.approx([0.0, y, CAD_MOUNT_Z], abs=1e-9), name
    assert joint_origin(urdf_root, "neck_pan")[2] == pytest.approx(
        CAD_NECK_PAN_Z, abs=1e-9)


def test_torso_cameras_keep_their_cad_heights(urdf_root):
    """Both bolt through the column and are authored in its frame."""
    for name, z in (("chest_camera_mount", CAD_CHEST_CAMERA_Z),
                    ("back_fisheye_mount", CAD_BACK_FISHEYE_Z)):
        assert joint_origin(urdf_root, name)[2] == pytest.approx(z, abs=1e-9), \
            name


def test_the_column_visuals_carry_no_offset(urdf_root):
    """No per-visual z shim survived, and no derived "extended" mesh.

    The previous attempt at this correction lengthened the column mesh and
    offset the vest regions by 53.8 mm; both are gone, and the shipped meshes
    are the plain CAD split again.
    """
    column = next(l for l in urdf_root.findall("link")
                  if l.get("name") == "torso_column")
    meshes = {}
    for vis in column.findall("visual"):
        mesh = os.path.basename(vis.find("geometry/mesh").get("filename"))
        meshes[mesh] = [float(v) for v in vis.find("origin").get("xyz").split()]
    assert "torso_column_white.obj" in meshes, sorted(meshes)
    assert not any("extended" in m for m in meshes), sorted(meshes)
    for mesh, xyz in meshes.items():
        assert xyz == pytest.approx([0.0, 0.0, 0.0], abs=1e-12), mesh


def test_the_torso_keepout_starts_at_the_sleeve_lip(model):
    """torso_core is the keep-out the C++/JS validators mirror. It keeps the
    CAD top (0.49 m) and now starts at the moving sleeve's lower lip, 79 mm
    above the lift origin: the band below is fixed column already covered by
    the chassis lift_pole box, so the guard loses no keep-out at q_lift = 0."""
    from manipulation_kit.description.d1.tools.generate_d1_urdf import (
        TORSO_SLEEVE_LIP_M,
    )
    core = next(p for p in model.collisions["torso_column"]
                if p.name == "torso_core")
    assert TORSO_SLEEVE_LIP_M == pytest.approx(0.079)
    height = CAD_TORSO_CORE_HEIGHT - TORSO_SLEEVE_LIP_M
    assert core.size[2] == pytest.approx(height, abs=1e-9)
    assert core.origin.t[2] == pytest.approx(TORSO_SLEEVE_LIP_M + height / 2.0,
                                             abs=1e-9)
    assert core.size[0] == pytest.approx(0.09) and core.size[1] == pytest.approx(0.11)


def test_the_guard_chest_keepout_is_unchanged():
    """Shoulder-relative, in the dual_base frame, so the offset cannot touch it."""
    from manipulation_kit.guard import guard
    assert guard.DEFAULT_CHEST_KEEPOUT == {
        "A": ((-0.1245, -0.13, 0.44), (0.1245, 0.06, 0.60)),
        "B": ((-0.1245, -0.06, 0.44), (0.1245, 0.13, 0.60)),
    }


# --------------------------------------------- the heights the tape measured
@pytest.mark.parametrize("link,expected", sorted(ZERO_CONFIG_Z.items()))
def test_zero_config_world_heights(model, link, expected):
    z = link_pose(model, link, {})[2, 3]
    assert abs(z - expected) < TOLERANCE_M, (
        f"{link} is {z * 1000:.4f} mm above the floor, expected "
        f"{expected * 1000:.4f} mm")


def test_the_tcp_is_where_the_arm_puts_it(model):
    """Zero config, right-hand tree: 0.8106 m out and 1.013 m up."""
    p = link_pose(model, "TCP_Link_R", {})[:3, 3]
    assert p == pytest.approx([0.0, 0.8106, 1.013], abs=1e-4)


def test_the_head_camera_lands_on_the_tape_measurement(model):
    """The frame the head-camera calibration work cares about, floor-referenced."""
    for link, measured in MEASURED.items():
        z = link_pose(model, link, {})[2, 3]
        assert abs(z - measured) < MEASURED_TOLERANCE_M, (
            f"{link} is {z * 1000:.1f} mm above the floor; the tape read "
            f"{measured * 1000:.0f} mm on d1-3 (2026-09-16)")


def test_the_whole_upper_body_moved_by_exactly_the_offset(model):
    """Every frame above the lift is its CAD height plus the one offset.

    The point of applying the correction at the lift origin is that the upper
    body keeps its shape; this is what would catch a stray per-link shim.
    """
    for link, expected in ZERO_CONFIG_Z.items():
        z = link_pose(model, link, {})[2, 3]
        assert abs((z - OFFSET) - (expected - OFFSET)) < TOLERANCE_M
        assert abs(z - expected) < TOLERANCE_M, link
    # and the lift still spans exactly its stroke
    top = link_pose(model, "head_link", {"lift": 0.3})[2, 3]
    bottom = link_pose(model, "head_link", {"lift": 0.0})[2, 3]
    assert abs((top - bottom) - 0.3) < 1e-9


# -------------------------------------------------- the AMR under the lift
def test_chassis_and_wheels_did_not_move(model):
    """The offset raises the lift's load; the AMR boxes stay floor-relative.

    They model the CAD cover, whose keep-out top (0.4987 m) still sits above
    the 0.460 m deck the tape found, so they remain conservative — which is
    all a keep-out volume has to be.
    """
    chassis = link_pose(model, "chassis_link", {"lift": 0.0})
    assert abs(chassis[2, 3]) < 1e-12, "chassis_link is the floor frame"
    boxes = dict(collision_corners(model, "chassis_link", {}))
    assert abs(boxes["chassis_body"][:, 2].min()) < 1e-9, "wheels off the floor"
    assert abs(boxes["chassis_cover"][:, 2].max() - 0.4987) < 1e-6


def test_the_gripper_assembly_rides_the_offset_too(model):
    """The wrist-camera plate is still the lowest point, 29 mm up from the CAD.

    At the 0.201 m lift the 2026-09-13 clearance check was taken at, this model
    now reads 0.879 m rather than the 0.850 m it read before the offset. That
    reading claimed 0.904 m; it disagrees with the 2026-09-16 body measurement
    by 25 mm and is NOT what this description is fitted to (see the module
    docstring). Pinned here so a future re-fit has to face the disagreement
    rather than rediscover it.
    """
    q = {f"Joint{i + 1}_{RIGHT_ARM_SUFFIX}": v
         for i, v in enumerate(home_pose_rad(RIGHT_ARM_HOME_OFFSET))}
    q["lift"] = 0.201
    z, name = gripper_assembly_bottom(model, RIGHT_ARM_SUFFIX, q)
    assert "camera_plate" in name, name
    assert abs(z - (0.850170 + OFFSET)) < 1e-4, z
