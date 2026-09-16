"""The composed gripper + camera-plate description: it parses, its meshes
resolve, the camera frames match what the plate mesh actually says, and the
gripper portion is byte-for-byte the vendored ``gripper.urdf``.

Everything here runs on the stdlib + numpy-free math; a stricter pass under
`yourdfpy` runs when it happens to be installed, same policy as
``test_description.py``."""
import math
import struct
import xml.etree.ElementTree as ET

import pytest

from manipulation_kit.hands.d1.parallel_gripper import description_path
from manipulation_kit.hands.d1.parallel_gripper.description import (
    CAMERA_ARM_YAW_RAD,
    CAMERA_MOUNT_XYZ_M,
    CAMERA_PLATE_THICKNESS_M,
    CAMERA_TILT_RAD,
    load_urdf,
    mesh_paths,
)


from manipulation_kit import assets

#: The CAD is not in this repository (LICENSE-STATUS.md). Every check that
#: OPENS a mesh — as opposed to checking that the URDF names one — carries this
#: mark, so a public checkout runs the whole suite and says out loud which
#: claims it could not test. Set $MKIT_ASSETS_DIR (or `mkit-urdf fetch-assets`)
#: and the same tests run for real; CI does exactly that.
needs_assets = pytest.mark.skipif(not assets.have_external_assets(),
                                  reason=assets.NO_ASSETS_REASON)


GRIPPER_LINKS = {"base_link", "tcp_r_Link", "tcp_l_Link"}
CAMERA_LINKS = {"camera_plate", "wrist_camera", "wrist_camera_optical"}
CAMERA_JOINTS = {"camera_plate_joint", "wrist_camera_joint",
                 "wrist_camera_optical_joint"}


def _root():
    return load_urdf(camera=True).getroot()


def _stl_vertices(path):
    blob = path.read_bytes()
    count = struct.unpack("<I", blob[80:84])[0]
    assert len(blob) == 84 + 50 * count, f"truncated STL {path.name}"
    return [struct.unpack_from("<9f", blob, 84 + 50 * t + 12)[c * 3:c * 3 + 3]
            for t in range(count) for c in range(3)]


def test_topology():
    root = _root()
    assert root.get("name") == "gripper_with_camera"
    assert {ln.get("name") for ln in root.findall("link")} == GRIPPER_LINKS | CAMERA_LINKS
    joints = {j.get("name"): j for j in root.findall("joint")}
    assert CAMERA_JOINTS <= set(joints)
    for name in CAMERA_JOINTS:
        assert joints[name].get("type") == "fixed"
    assert joints["camera_plate_joint"].find("parent").get("link") == "base_link"
    assert joints["wrist_camera_joint"].find("parent").get("link") == "camera_plate"
    assert (joints["wrist_camera_optical_joint"].find("parent").get("link")
            == "wrist_camera")


def test_gripper_portion_is_the_vendored_description():
    """The composed file must not fork the gripper: every gripper link and
    joint element is textually identical to the one in ``gripper.urdf``."""
    plain = load_urdf(camera=False, absolute_meshes=False).getroot()
    composed = load_urdf(camera=True, absolute_meshes=False).getroot()
    for tag, names in (("link", GRIPPER_LINKS), ("joint", {"tcp_r_joint", "tcp_l_joint"})):
        for name in names:
            a = plain.find(f"{tag}[@name='{name}']")
            b = composed.find(f"{tag}[@name='{name}']")
            a.tail = b.tail = None  # only the whitespace after the element differs
            assert ET.tostring(a) == ET.tostring(b), f"{tag} {name} diverged"


@needs_assets
def test_meshes_resolve_and_plate_is_in_metres():
    paths = mesh_paths(camera=True)
    assert len(paths) == 8  # 3 gripper links + plate, visual + collision each
    plate = next(p for p in paths if p.name == "camera_plate.STL")
    verts = _stl_vertices(plate)
    zs = [v[2] for v in verts]
    ys = [v[1] for v in verts]
    assert max(abs(c) for v in verts for c in v) < 0.15, "plate is not in metres"
    assert min(zs) == pytest.approx(0.0, abs=1e-4), "disc face must sit on the flange plane"
    assert max(ys) == pytest.approx(0.0999, abs=5e-4)


@needs_assets
def test_plate_stays_inside_the_flange_gap():
    """The vendor gripper body mesh starts at z = 16.5 mm; inside the body
    footprint the plate must stay below that, or the two meshes interpenetrate."""
    plate = next(p for p in mesh_paths(camera=True) if p.name == "camera_plate.STL")
    for x, y, z in _stl_vertices(plate):
        if abs(x) < 0.0285 and abs(y) < 0.029:
            assert z <= 0.0165 + 1e-6, "plate protrudes into the gripper body"


@needs_assets
def test_camera_mount_frame_matches_the_plate_mesh():
    """The wrist_camera joint origin must lie ON the plate's tilted camera
    face and its rpy must be exactly the face tilt."""
    root = _root()
    joint = root.find("joint[@name='wrist_camera_joint']")
    xyz = [float(v) for v in joint.find("origin").get("xyz").split()]
    rpy = [float(v) for v in joint.find("origin").get("rpy").split()]
    assert xyz == pytest.approx(list(CAMERA_MOUNT_XYZ_M), abs=1e-6)
    assert rpy == pytest.approx([CAMERA_TILT_RAD, 0.0, 0.0], abs=1e-9)

    n = (0.0, -math.sin(CAMERA_TILT_RAD), math.cos(CAMERA_TILT_RAD))
    plate = next(p for p in mesh_paths(camera=True) if p.name == "camera_plate.STL")
    verts = _stl_vertices(plate)
    # plane offset of the camera face = the outermost plane among triangles
    # whose OWN normal is the face normal (the whole-mesh max would pick up
    # the arm tip instead)
    d_mount = None
    for t in range(len(verts) // 3):
        a, b, c = verts[3 * t], verts[3 * t + 1], verts[3 * t + 2]
        u = [b[i] - a[i] for i in range(3)]
        v = [c[i] - a[i] for i in range(3)]
        nx = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
              u[0] * v[1] - u[1] * v[0])
        norm = math.sqrt(sum(k * k for k in nx)) or 1.0
        if sum(k * ni for k, ni in zip(nx, n)) / norm > 0.99997:
            d = sum(k * ni for k, ni in zip(a, n))
            d_mount = d if d_mount is None else max(d_mount, d)
    d_frame = sum(c * ni for c, ni in zip(xyz, n))
    assert d_frame == pytest.approx(d_mount, abs=5e-4), (
        "wrist_camera origin is not on the plate's camera face")


def test_optical_frame_is_the_ros_convention():
    """optical = mount rotated pi about Z, so +Z stays the face normal, +X is
    image right and +Y is image down (toward the fingers — the wrist streams
    show the jaws at the bottom edge of the frame)."""
    joint = _root().find("joint[@name='wrist_camera_optical_joint']")
    assert joint.find("origin").get("xyz") == "0 0 0"
    rpy = [float(v) for v in joint.find("origin").get("rpy").split()]
    assert rpy == pytest.approx([0.0, 0.0, math.pi], abs=1e-6)


def test_clocking_constants():
    """Camera on top of the wrist on both arms: left mounts with yaw pi,
    right with yaw 0 (measured from the d1 teleop dataset, 2026-08-21)."""
    assert CAMERA_ARM_YAW_RAD["left"] == pytest.approx(math.pi)
    assert CAMERA_ARM_YAW_RAD["right"] == 0.0
    # MEASURED on d1-3 2026-09-16 (Shu, callipers); the CAD drop and the
    # committed camera_plate.STL both still model an 8 mm disc.
    assert CAMERA_PLATE_THICKNESS_M == 0.002


def test_added_mass_is_the_plate_plus_the_camera():
    """The composed URDF weighs the measured 1.5 kg gripper plus the plate
    (mesh volume at aluminium book density) and the placeholder camera."""
    plain = sum(float(ln.find("inertial/mass").get("value"))
                for ln in load_urdf().getroot().findall("link"))
    composed = sum(float(ln.find("inertial/mass").get("value"))
                   for ln in _root().findall("link")
                   if ln.find("inertial") is not None)
    added = composed - plain
    assert 0.09 < added < 0.13, f"plate + camera add {added * 1e3:.0f} g, expected ~108 g"


def test_no_double_hyphen_inside_comments():
    text = description_path("gripper_with_camera.urdf").read_text()
    for chunk in text.split("<!--")[1:]:
        assert "--" not in chunk.split("-->")[0]


@needs_assets
def test_loads_in_a_real_urdf_parser():
    yourdfpy = pytest.importorskip("yourdfpy")
    robot = yourdfpy.URDF.load(str(description_path("gripper_with_camera.urdf")),
                               load_meshes=True, build_scene_graph=True)
    assert set(robot.link_map) == GRIPPER_LINKS | CAMERA_LINKS
    assert robot.base_link == "base_link"
