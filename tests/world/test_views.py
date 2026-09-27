"""``size`` is mandatory, names are unique, and geometry answers are measured."""

from __future__ import annotations

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from manipulation_kit.world import (ArmView, ContainerView, FrameGraph,
                                    GripperView, ObjectView, SurfaceView,
                                    WorldView)


def test_an_object_without_a_measured_size_cannot_be_built():
    """Every clearance comes out of the extent. A missing one is how a grasp
    silently becomes a collision, so there is no default to fall back on."""
    with pytest.raises(TypeError):
        ObjectView("block", p=(0.4, 0.1, 0.05))          # size omitted
    with pytest.raises(ValueError):
        ObjectView("block", p=(0.4, 0.1, 0.05), size=(0.05, 0.0, 0.05))


def test_two_things_may_not_answer_to_the_same_name():
    with pytest.raises(ValueError, match="same name"):
        WorldView.of([ObjectView("block", p=(0.4, 0, 0), size=(0.05,) * 3),
                      ObjectView("block", p=(0.5, 0, 0), size=(0.05,) * 3)])


def test_the_principal_axis_is_the_long_horizontal_one():
    frames = FrameGraph()
    block = ObjectView("block", p=(0.4, 0.1, 0.05), size=(0.09, 0.04, 0.05))
    assert np.allclose(block.principal_axis(frames), [1.0, 0.0, 0.0])
    turned = ObjectView("block", p=(0.4, 0.1, 0.05), size=(0.09, 0.04, 0.05),
                        r=R.from_euler("z", 90, degrees=True))
    assert np.allclose(np.abs(turned.principal_axis(frames)), [0.0, 1.0, 0.0],
                       atol=1e-9)


def test_a_square_footprint_has_no_principal_axis():
    """Inventing one rolls the wrist for nothing."""
    frames = FrameGraph()
    cube = ObjectView("cube", p=(0.4, 0.1, 0.05), size=(0.05, 0.05, 0.05))
    assert cube.principal_axis(frames) is None


def test_a_square_footprint_still_has_faces_to_square_the_jaws_to():
    """No PREFERRED grasp is not the same as no wrong one.

    MEASURED (blocks-eval, 2026-09-19): a 40 mm cube yawed 11.7 deg presents
    47.3 mm across base-aligned jaws, and the driven gripper can take 43.96.
    The pads met two corners, stalled at a 46 mm gap and held nothing, five
    trials out of five.
    """
    frames = FrameGraph()
    for yaw in (0.0, 11.7, 25.0, 44.0):
        cube = ObjectView("cube", p=(0.44, -0.05, 0.19), size=(0.04,) * 3,
                          r=R.from_euler("z", yaw, degrees=True))
        assert cube.principal_axis(frames) is None
        axis = cube.footprint_axis(frames)
        assert axis is not None and abs(float(axis[2])) < 1e-9
        # it IS one of the cube's own horizontal faces' normals
        body = cube.axes_in_base(frames)
        assert min(abs(abs(float(np.dot(axis, body[:, i]))) - 1.0)
                   for i in (0, 1)) < 1e-9
    # where there IS a long axis, nothing changes
    block = ObjectView("block", p=(0.4, 0.1, 0.05), size=(0.09, 0.04, 0.05))
    assert np.allclose(block.footprint_axis(frames),
                       block.principal_axis(frames))


def test_a_container_tests_membership_in_its_own_axes():
    frames = FrameGraph()
    box = ContainerView("box", p=(0.4, 0.0, 0.05), size=(0.20, 0.10, 0.10),
                        interior=(0.18, 0.08, 0.09),
                        r=R.from_euler("z", 90, degrees=True))
    # 80 mm along the box's own long axis, which now runs along base +y
    assert box.contains([0.4, 0.08, 0.05], frames)
    assert not box.contains([0.48, 0.0, 0.05], frames)


# An open-top tray like the color-sort scene's: 180 x 180 mm outside, a
# 17 mm-deep interior, standing on a table whose top is at z = 0.170.
TABLE_TOP_Z = 0.170
TRAY = ContainerView("tray", p=(0.55, -0.20, TABLE_TOP_Z + 0.010),
                     size=(0.18, 0.18, 0.020), interior=(0.16, 0.16, 0.017))
TRAY_FLOOR_Z = TABLE_TOP_Z + 0.010 - 0.017 / 2.0


def _cube(x, y, underside_z, edge=0.030, yaw_deg=0.0):
    return ObjectView("cube", p=(x, y, underside_z + edge / 2.0),
                      size=(edge,) * 3, r=R.from_euler("z", yaw_deg, degrees=True))


def test_a_cube_standing_on_a_shallow_trays_floor_is_inside_it():
    """A 30 mm cube on the floor of a 17 mm-deep tray stands 21.5 mm proud
    of the rim. The open top is the point of a tray: the cube is in it."""
    frames = FrameGraph()
    assert TRAY.depth_declared()
    cube = _cube(0.55, -0.20, TRAY_FLOOR_Z)
    assert cube.top_face_z(frames) > TRAY.rim_z(frames)
    assert TRAY.contains_object(cube, frames)
    # a yawed cube near a wall, its corners still inside the interior
    assert TRAY.contains_object(_cube(0.55 + 0.055, -0.20, TRAY_FLOOR_Z,
                                      yaw_deg=10.0), frames)


def test_a_cube_beside_or_across_the_wall_is_not_inside():
    frames = FrameGraph()
    # on the table next to the tray
    assert not TRAY.contains_object(_cube(0.55 + 0.20, -0.20, TABLE_TOP_Z),
                                    frames)
    # its centre over the interior, one edge over the wall
    assert not TRAY.contains_object(_cube(0.55 + 0.070, -0.20, TRAY_FLOOR_Z),
                                    frames)


def test_a_cube_in_the_air_over_the_tray_is_not_inside():
    """Held 40 mm over the rim: horizontally over the interior, but its
    underside is above the rim."""
    frames = FrameGraph()
    over = _cube(0.55, -0.20, TRAY.rim_z(frames) + 0.040)
    assert not TRAY.contains_object(over, frames)
    # between the floor and the rim (lowered into it, not yet released) is in
    assert TRAY.contains_object(_cube(0.55, -0.20, TRAY_FLOOR_Z + 0.005),
                                frames)
    # ...and pad_m widens the rim bound as it widens the others
    grazing = _cube(0.55, -0.20, TRAY.rim_z(frames) + 0.005)
    assert not TRAY.contains_object(grazing, frames)
    assert TRAY.contains_object(grazing, frames, pad_m=0.01)


def test_a_container_without_a_declared_depth_keeps_the_three_axis_test():
    """No measured interior and no rim height: the interior is the 90 %
    estimate of the outside and the floor/rim are a guess, so the object's
    whole extent has to fit in that box, as before."""
    frames = FrameGraph()
    guessed = ContainerView("tray", p=TRAY.p, size=TRAY.size)
    assert not guessed.interior_measured and not guessed.depth_declared()
    assert not guessed.contains_object(_cube(0.55, -0.20, TRAY_FLOOR_Z), frames)
    small = _cube(0.55, -0.20, TRAY_FLOOR_Z + 0.001, edge=0.010)
    assert guessed.contains_object(small, frames)
    # a declared rim height alone is a declared depth
    rimmed = ContainerView("tray", p=TRAY.p, size=TRAY.size,
                           rim_height_m=0.010)
    object.__setattr__(rimmed, "interior_measured", False)
    assert rimmed.depth_declared()
    assert rimmed.contains_object(_cube(0.55, -0.20, TRAY_FLOOR_Z), frames)


def test_a_containers_interior_may_not_exceed_its_outside():
    with pytest.raises(ValueError):
        ContainerView("box", p=(0.4, 0, 0), size=(0.1, 0.1, 0.1),
                      interior=(0.2, 0.1, 0.1))


def test_a_surface_supports_only_what_is_on_top_of_it():
    frames = FrameGraph()
    table = SurfaceView("table", p=(0.4, 0.0, 0.0), size=(0.9, 0.8, 0.02))
    assert table.supports([0.4, 0.1, 0.012], frames)
    assert not table.supports([0.4, 0.1, 0.3], frames)     # floating
    assert not table.supports([2.0, 0.1, 0.012], frames)   # off the edge


def test_an_arm_view_insists_on_seven_finite_joints():
    with pytest.raises(ValueError):
        ArmView("left", joints=np.zeros(6))
    with pytest.raises(ValueError):
        ArmView("left", joints=np.full(7, np.nan))
    with pytest.raises(ValueError):
        ArmView("middle", joints=np.zeros(7))


def test_a_gripper_reports_measured_holding_not_a_command():
    gripper = GripperView("left", closedness=1.0, holding=False)
    assert not gripper.holding
    with pytest.raises(ValueError):
        GripperView("left", closedness=1.5)


def test_holder_of_names_the_side_that_reports_holding_the_object():
    world = WorldView.of(
        [ObjectView("block", p=(0.4, 0, 0), size=(0.05,) * 3)],
        grippers=[GripperView("left", 1.0, holding=True, held_object="block"),
                  GripperView("right", 0.0)])
    assert world.holder_of("block") == "left"
    assert world.holder_of("cup") is None
