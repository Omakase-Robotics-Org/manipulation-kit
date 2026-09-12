"""The bundled vendor CAD description: it parses, its meshes resolve, and
the numbers a consumer composes against are what the CAD says.

No optional dependency here — the URDF is parsed with the stdlib. A second,
STRICTER pass runs under `yourdfpy` when it happens to be installed (a real
URDF parser that also resolves and loads every mesh), so CI on a machine
with it gets full validation and a bare machine still gets the structure
checks."""
import math
import struct
import xml.etree.ElementTree as ET

import pytest

from manipulation_kit.hands.d1.parallel_gripper import description_path
from manipulation_kit.hands.d1.parallel_gripper.description import (
    JAW_OPEN_GAP_M,
    JAW_STROKE_M,
    JAW_TIP_Z_M,
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


LINKS = {"base_link", "tcp_r_Link", "tcp_l_Link"}
JOINTS = {"tcp_r_joint", "tcp_l_joint"}


def _root():
    return load_urdf().getroot()


def test_description_path_resolves():
    assert description_path().is_file()
    with pytest.raises(FileNotFoundError):
        description_path("not-a-file.urdf")


def test_parses_and_has_the_vendor_topology():
    root = _root()
    assert root.tag == "robot" and root.get("name") == "gripper"
    assert {ln.get("name") for ln in root.findall("link")} == LINKS
    assert {j.get("name") for j in root.findall("joint")} == JOINTS
    for joint in root.findall("joint"):
        assert joint.get("type") == "prismatic"
        assert joint.find("parent").get("link") == "base_link"


def test_no_double_hyphen_inside_comments():
    """A `--` inside an XML comment makes the file unreadable to EVERY
    parser. ElementTree above would already have failed, but assert it
    explicitly: this exact bug has shipped in a generated URDF here before."""
    text = description_path().read_text()
    for chunk in text.split("<!--")[1:]:
        body = chunk.split("-->")[0]
        assert "--" not in body, "double hyphen inside an XML comment"


@needs_assets
def test_mesh_references_resolve_from_a_fresh_checkout():
    paths = mesh_paths()
    assert len(paths) == 6  # visual + collision for each of the three links
    for path in paths:
        assert path.is_file(), f"missing mesh {path}"
        # binary STL: 80-byte header + uint32 count + 50 bytes per triangle
        blob = path.read_bytes()
        count = struct.unpack("<I", blob[80:84])[0]
        assert len(blob) == 84 + 50 * count, f"truncated STL {path.name}"
        assert count > 100


def test_no_ros_package_uris_left():
    """`package://` only resolves inside a ROS workspace; this ships as
    Python package data and is loaded by absolute path."""
    assert "package://" not in description_path().read_text()
    assert not any("j6_Link" in str(p) for p in mesh_paths()), (
        "the body mesh still carries the arm link name it was extracted off")


def test_absolute_meshes_flag():
    for mesh in load_urdf(absolute_meshes=True).getroot().iter("mesh"):
        assert mesh.get("filename").startswith("/")
    for mesh in load_urdf(absolute_meshes=False).getroot().iter("mesh"):
        assert not mesh.get("filename").startswith("/")


def test_finger_joints_can_actually_move():
    """The vendor drop shipped `velocity="0"` on both jaws — a CAD export
    artefact that planners and simulators read as an immovable joint."""
    for joint in _root().findall("joint"):
        limit = joint.find("limit")
        assert float(limit.get("velocity")) > 0.0, (
            f"{joint.get('name')} has a zero velocity limit")
        assert float(limit.get("effort")) > 0.0


def test_jaw_stroke_and_mimic():
    root = _root()
    limits = {j.get("name"): j.find("limit") for j in root.findall("joint")}
    right, left = limits["tcp_r_joint"], limits["tcp_l_joint"]
    assert (float(right.get("lower")), float(right.get("upper"))) == (0.0, JAW_STROKE_M)
    assert (float(left.get("lower")), float(left.get("upper"))) == (-JAW_STROKE_M, 0.0)
    assert JAW_OPEN_GAP_M == pytest.approx(2 * JAW_STROKE_M)

    mimic = root.find("joint[@name='tcp_l_joint']/mimic")
    assert mimic is not None, "the passive jaw lost its mimic coupling"
    assert mimic.get("joint") == "tcp_r_joint"
    assert float(mimic.get("multiplier")) == -1.0


def test_jaws_are_mirror_images_across_the_travel_axis():
    """Both jaws hang off the same origin with the same axis; their meshes
    are mirrored, which is why one description mounts on either arm."""
    root = _root()
    origins = []
    for name in ("tcp_r_joint", "tcp_l_joint"):
        joint = root.find(f"joint[@name='{name}']")
        origins.append((joint.find("origin").get("xyz"),
                        joint.find("origin").get("rpy"),
                        joint.find("axis").get("xyz")))
    assert origins[0] == origins[1]
    # ... and the two jaw inertials sit symmetrically about the jaw plane
    coms = {}
    for name in ("tcp_r_Link", "tcp_l_Link"):
        xyz = root.find(f"link[@name='{name}']/inertial/origin").get("xyz")
        coms[name] = [float(v) for v in xyz.split()]
    assert coms["tcp_r_Link"][2] == pytest.approx(-coms["tcp_l_Link"][2], abs=1e-6)


@needs_assets
def test_geometry_constants_match_the_meshes():
    """JAW_TIP_Z_M is the number consumers place a TCP against — derive it
    from the committed mesh rather than trusting the constant."""
    root = _root()
    joint = root.find("joint[@name='tcp_r_joint']")
    origin_z = float(joint.find("origin").get("xyz").split()[2])
    # joint rpy (pi, -pi/2, 0) maps the jaw's local +x onto base +z
    rpy = [float(v) for v in joint.find("origin").get("rpy").split()]
    assert rpy[0] == pytest.approx(math.pi, abs=1e-3)
    assert rpy[1] == pytest.approx(-math.pi / 2, abs=1e-3)

    jaw = next(p for p in mesh_paths() if p.name == "tcp_r_Link.STL")
    blob = jaw.read_bytes()
    count = struct.unpack("<I", blob[80:84])[0]
    local_x = [struct.unpack_from("<f", blob, 84 + 50 * t + 12 + 4 * c)[0]
               for t in range(count) for c in (0, 3, 6)]
    assert origin_z + max(local_x) == pytest.approx(JAW_TIP_Z_M, abs=1e-4)


def test_urdf_mass_is_the_measured_mass_not_the_cad_mass():
    """The committed URDF must sum to the WEIGHED 1.5 kg (Shu 2026-07-29), not
    the 0.3279 kg the shell-only CAD export claimed. Shipping the CAD number
    would put a 4.6x-light mass, with CAD provenance making it look
    authoritative, into anything that loads this file for dynamics."""
    from manipulation_kit.hands.d1.parallel_gripper import toolconfig as tc
    total = sum(float(link.find("inertial/mass").get("value"))
                for link in _root().findall("link"))
    assert total == pytest.approx(tc.MEASURED_MASS_KG, abs=1e-6)
    assert total == pytest.approx(tc.tool_config().mass_kg, abs=1e-6), (
        "the URDF and the registered tool config must describe one object")
    assert tc.CAD_MASS_KG == pytest.approx(0.327917, abs=1e-6)  # kept as a record
    assert tc.CAD_JAW_TIP_Z_MM == pytest.approx(JAW_TIP_Z_M * 1000, abs=1e-3)


def test_urdf_com_matches_the_validated_registered_com():
    """The assembly COM must land on the registered 68 mm — the one COM figure
    with hardware behind it. The jaws keep their CAD inertials; base_link
    carries the corrected mass and a COM solved to hit that total."""
    from manipulation_kit.hands.d1.parallel_gripper import toolconfig as tc
    root = _root()
    joint_z = float(root.find("joint[@name='tcp_r_joint']/origin")
                    .get("xyz").split()[2])
    total, moment = 0.0, 0.0
    for link in root.findall("link"):
        mass = float(link.find("inertial/mass").get("value"))
        com = [float(v) for v in
               link.find("inertial/origin").get("xyz").split()]
        # a jaw's own frame maps its local +x onto the base +z (rpy pi,-pi/2,0)
        z = com[2] if link.get("name") == "base_link" else com[0] + joint_z
        total += mass
        moment += mass * z
    assert moment / total == pytest.approx(tc.tool_config().com_mm[2] / 1000.0,
                                          abs=1e-6)


def test_base_link_inertial_is_labelled_an_estimate():
    """Nobody should read base_link's mass properties as CAD-precise: the mass
    is measured, the COM is solved, the tensor is scaled."""
    text = description_path().read_text()
    head = text[:text.index("</inertial>")]
    assert "ESTIMATE" in head
    assert "0.244768569107893" in head, "the CAD original must stay quoted"


@needs_assets
def test_loads_in_a_real_urdf_parser():
    """Full validation when a real URDF library is available: yourdfpy
    builds the kinematic tree AND loads every mesh off disk."""
    yourdfpy = pytest.importorskip("yourdfpy")
    robot = yourdfpy.URDF.load(str(description_path()),
                               load_meshes=True, build_scene_graph=True)
    assert set(robot.link_map) == LINKS
    assert set(robot.joint_map) == JOINTS
    assert robot.base_link == "base_link"
    assert len(robot.actuated_joints) >= 1
