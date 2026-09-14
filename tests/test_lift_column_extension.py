"""The moving lift cover is as long as the robot's, pinned to the measurement.

The D1's lift telescopes, and the built robot's moving cover is 53.829712 mm
longer than any CAD in this repository models it — so every height the cover
carries (arm mounts, neck, torso boxes, torso cameras) is the CAD number plus
that extension. See the MOVING LIFT COLUMN EXTENSION block in
``description/d1/tools/generate_d1_urdf.py``.

What the checks below pin, and why each one is here:

1. **The measurement itself.** At the real 0.201 m lift and the HOME pose from
   ``config/home_pose.json``, the lowest point of the complete right gripper
   assembly — jaws, body, wrist-camera plate — is 0.904 m above
   ``base_footprint``, 20 mm over the 884 mm wagon top Shu measured against on
   2026-09-13. This is the number the whole change exists to reproduce, and it
   is checked against the URDF's own collision geometry, not against a
   constant multiplied back out.
2. **Agreement with d1-isaaclab**, which builds its
   ``assets/d1/d1_bimanual_gripper.urdf`` from this repository's
   ``d1_wholebody_gripper.urdf`` and carried this geometry first (PR #43,
   commit ``d5a71ec``). If the two repositories' head chains disagree, one of
   them is rendering a different robot.
3. **The full-down clearance the correction exists to preserve.** Extending the
   cover at its TOP (rather than raising the lift joint origin, d1-isaaclab's
   rejected PR #42) leaves the actuator zero and the cover's bottom lip exactly
   where they were.
4. **One extension, not four.** The constant is spelled out in four places that
   cannot import each other cheaply (the generator, the YUBI assembler, the
   guard's chest keep-out, and the mesh tool); these pin them together.

FK is the package's own numpy chain (``manipulation_kit.arms.urdf_chain``) over
the shipped URDF — no simulator, no extra dependency.
"""
import json
import math
import os

import numpy as np
import pytest

import manipulation_kit
from manipulation_kit.arms.urdf_chain import ACTUATED, UrdfChain
from manipulation_kit.guard.urdf_model import UrdfModel

KIT = os.path.dirname(os.path.abspath(manipulation_kit.__file__))
CONFIG = os.path.join(KIT, "config")
DESCRIPTION = os.path.join(KIT, "description")
D1_WB_GRIPPER = os.path.join(DESCRIPTION, "d1", "d1_wholebody_gripper.urdf")

#: The measured correction (m). Every other number in this file follows from
#: it and from the CAD it corrects.
EXTENSION = 0.053829712

#: The lift height the measurement was taken at, and what was measured there:
#: the complete open gripper assembly's lowest point, 20 mm above an 884 mm
#: wagon top. The tolerance is 3 mm — an order of magnitude below the 53.8 mm
#: correction, and wide enough for the generator's six-significant-digit output
#: (0.3 µm) and for the tape measure this came off.
MEASURED_LIFT_M = 0.201
MEASURED_GRIPPER_BOTTOM_M = 0.904
MEASURED_TOLERANCE_M = 0.003
WAGON_TOP_M = 0.884

#: Root-frame heights d1-isaaclab's d1_bimanual_gripper.urdf reports for the
#: head chain at q = 0 after the same correction (commit d5a71ec). Read off
#: that URDF, not restated from this one.
ISAAC_NECK_PAN_Z = 1.158829712
ISAAC_HEAD_LINK_Z = 1.214329712
ISAAC_ARM_BASE_Z = 1.037829712
#: 0.01 mm: tight enough that a millimetre of drift fails, loose enough for the
#: six significant digits this generator emits.
ISAAC_TOLERANCE_M = 1e-5

#: The right gripper is the ``_L`` tree (``+Y`` = ``_R`` = the PHYSICAL LEFT
#: arm; see the generator's side-naming note), and HOME's second seven joints
#: are the ones that drive it.
RIGHT_ARM_SUFFIX = "L"
RIGHT_ARM_HOME_OFFSET = 7


@pytest.fixture(scope="module")
def model():
    return UrdfModel(D1_WB_GRIPPER)


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

    "Complete assembly" is the measurement convention Shu confirmed: the jaws,
    the gripper body, AND the wrist-camera plate, which is the part that
    actually reaches lowest (16 mm below the jaws). Measuring the jaws alone
    reads 16 mm high, which is exactly how a correction like this gets set
    wrong.
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


def measurement_pose():
    """HOME on the right arm, at the lift height the measurement was taken at."""
    q = {f"Joint{i + 1}_{RIGHT_ARM_SUFFIX}": v for i, v in
         enumerate(home_pose_rad(RIGHT_ARM_HOME_OFFSET))}
    q["lift"] = MEASURED_LIFT_M
    return q


# ---------------------------------------------------------- the measurement
def test_gripper_assembly_clears_the_wagon_at_the_measured_lift(model):
    """0.904 m above base_footprint: what Shu measured on the robot."""
    z, name = gripper_assembly_bottom(model, RIGHT_ARM_SUFFIX,
                                      measurement_pose())
    assert abs(z - MEASURED_GRIPPER_BOTTOM_M) < MEASURED_TOLERANCE_M, (
        f"lowest gripper collision ({name}) is at {z * 1000:.3f} mm, not the "
        f"measured {MEASURED_GRIPPER_BOTTOM_M * 1000:.0f} ± "
        f"{MEASURED_TOLERANCE_M * 1000:.0f} mm")
    assert z - WAGON_TOP_M > 0.015, "the gripper does not clear the wagon"


def test_the_lowest_part_is_the_wrist_camera_plate(model):
    """The convention, pinned: the plate is what reaches lowest, not the jaws.

    A future edit that drops the plate from the model would still pass the
    height check by 16 mm — this is what notices.
    """
    _, name = gripper_assembly_bottom(model, RIGHT_ARM_SUFFIX,
                                      measurement_pose())
    assert "camera_plate" in name, name


def test_both_grippers_are_at_the_same_measured_height(model):
    """HOME is mirror-symmetric, so the left assembly must read the same."""
    q = {f"Joint{i + 1}_R": v for i, v in enumerate(home_pose_rad(0))}
    q["lift"] = MEASURED_LIFT_M
    z, _ = gripper_assembly_bottom(model, "R", q)
    assert abs(z - MEASURED_GRIPPER_BOTTOM_M) < MEASURED_TOLERANCE_M


# ------------------------------------------------- agreement with the sim
@pytest.mark.parametrize("link,expected", [
    ("Base_R", ISAAC_ARM_BASE_Z),
    ("Base_L", ISAAC_ARM_BASE_Z),
    ("neck_pan_link", ISAAC_NECK_PAN_Z),
    ("head_link", ISAAC_HEAD_LINK_Z),
])
def test_upper_body_matches_d1_isaaclab(model, link, expected):
    """Same world heights as d1-isaaclab's asset, which is built from this one."""
    z = link_pose(model, link, {})[2, 3]
    assert abs(z - expected) < ISAAC_TOLERANCE_M, (
        f"{link} is at {z * 1000:.6f} mm; d1-isaaclab (d5a71ec) has it at "
        f"{expected * 1000:.6f} mm")


# ------------------------------------------- what the extension must NOT move
def test_lift_joint_keeps_the_actuator_zero_and_travel(model):
    """The correction is a longer cover, not a re-zeroed rail."""
    lift = model.joints["dual_base"]
    assert lift.name == "lift" and lift.type == "prismatic"
    assert abs(lift.origin.t[2] - 0.484) < 1e-9, lift.origin.t
    assert (lift.lower, lift.upper) == (0.0, 0.3)


def test_full_down_cover_clearance_is_unchanged(model):
    """At q_lift = 0 the cover's bottom lip still clears the fixed AMR cover.

    The lip sits 0.079 m up the moving frame (MOVING_COLUMN_BOTTOM_Z, measured
    off the cover mesh) and the fixed cover tops out at 0.552 m, so the gap is
    11 mm — before this change and after it, because the extension is at the
    TOP. Raising the lift origin instead would have opened it to 64.8 mm.
    """
    column = link_pose(model, "torso_column", {"lift": 0.0})[2, 3]
    assert abs(column + 0.079 - 0.563) < 1e-9
    assert abs((column + 0.079) - 0.552 - 0.011) < 1e-9

    # And the guard keep-out that stands in for the cover did not shrink: its
    # floor is still the frame origin, only its top followed the torso up.
    core = next(p for p in model.collisions["torso_column"]
                if p.name == "torso_core")
    assert abs(core.origin.t[2] - core.size[2] / 2.0) < 1e-9
    assert abs(core.size[2] - (0.49 + EXTENSION)) < 1e-5


def test_chassis_and_wheels_did_not_move(model):
    """Only the lift's load moved; the AMR under it is where it always was."""
    chassis = link_pose(model, "chassis_link", {"lift": 0.0})
    assert abs(chassis[2, 3]) < 1e-12, "chassis_link is the floor frame"
    boxes = {name: corners for name, corners
             in collision_corners(model, "chassis_link", {})}
    assert abs(boxes["chassis_body"][:, 2].min()) < 1e-9, "wheels off the floor"
    assert abs(boxes["chassis_cover"][:, 2].max() - 0.4987) < 1e-6


# ------------------------------------------------- one constant, four copies
def test_every_copy_of_the_extension_agrees():
    """The generator, the YUBI assembler, the guard and the mesh tool.

    None of them can cheaply import the others (the generators run as scripts,
    the guard is stdlib-only by design), so the constant is written down four
    times. This is what keeps the four honest.
    """
    import importlib.util

    from manipulation_kit.description.d1.tools import extend_lift_column
    from manipulation_kit.guard import guard

    def load(*parts):
        path = os.path.join(DESCRIPTION, *parts)
        spec = importlib.util.spec_from_file_location(parts[-1][:-3], path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod

    generator = load("d1", "tools", "generate_d1_urdf.py")
    assembler = load("d1_yubi_description_v2", "tools", "assemble_d1_yubi.py")
    for name, value in (
            ("generate_d1_urdf", generator.LIFT_COLUMN_EXTENSION),
            ("assemble_d1_yubi", assembler.LIFT_COLUMN_EXTENSION),
            ("guard.guard", guard.LIFT_COLUMN_EXTENSION),
            ("extend_lift_column", extend_lift_column.EXTENSION)):
        assert value == EXTENSION, f"{name} has {value}, not {EXTENSION}"
    assert generator.MOVING_COLUMN_BOTTOM_Z < extend_lift_column.LIP_TOP_Z, (
        "the mesh tool must cut ABOVE the cover lip, or the lip moves with "
        "the walls and the full-down clearance changes")


def test_the_mesh_tool_lengthens_the_cover_without_moving_its_lip(tmp_path):
    """A wall spanning the cut stretches; the lip below it does not move."""
    from manipulation_kit.description.d1.tools import extend_lift_column

    src = tmp_path / "cover.obj"
    src.write_text("\n".join([
        "# a lip vertex, a wall vertex above the cut, and one face",
        "v 0.050000000 0.000000000 0.079000000",
        "v 0.050000000 0.000000000 0.300000000",
        "v 0.060000000 0.000000000 0.300000000",
        "vn 0.0 1.0 0.0",
        "f 1//1 2//1 3//1",
    ]) + "\n")
    out = tmp_path / "cover_extended.obj"
    moved = extend_lift_column.extend(src, out)
    assert moved == 2
    text = out.read_text()
    zs = [float(line.split()[3]) for line in text.splitlines()
          if line.startswith("v ")]
    assert zs[0] == pytest.approx(0.079)                    # lip stays
    assert zs[1] == pytest.approx(0.300 + EXTENSION)        # wall lengthens
    assert "f 1//1 2//1 3//1" in text and "vn 0.0 1.0 0.0" in text


def test_the_urdf_names_the_extended_cover_and_carries_the_vest_up():
    """The cover is a longer mesh; everything bolted to it gets an offset."""
    import xml.etree.ElementTree as ET

    root = ET.parse(D1_WB_GRIPPER).getroot()
    column = next(l for l in root.findall("link")
                  if l.get("name") == "torso_column")
    seen = {}
    for vis in column.findall("visual"):
        mesh = os.path.basename(vis.find("geometry/mesh").get("filename"))
        seen[mesh] = float(vis.find("origin").get("xyz").split()[2])
    assert "torso_column_white.obj" not in seen, \
        "the un-extended cover mesh would render 53.8 mm short at its top"
    assert seen.pop("torso_column_white_extended.obj") == 0.0
    assert seen, "the torso vest visuals disappeared"
    for mesh, z in seen.items():
        assert abs(z - EXTENSION) < 1e-6, f"{mesh} did not ride the cover up"
