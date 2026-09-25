"""``manipulation_kit.description.head_camera``: where the head camera is.

The frames themselves have been in the whole-body URDF since it gained
cameras. What is new is the ACCESSOR, and an accessor's only job is not to
become a second copy of the geometry — so these read the committed URDF and
check that what comes back moves with it, rather than pinning transcribed
numbers.

The two facts most likely to be got wrong by a consumer are checked directly:
positive ``neck_pitch`` looks DOWN (the URDF joint is the motor frame, and the
daemon reports the opposite sign), and the LIFT cannot move the camera in
``base`` because it sits below it.
"""

from __future__ import annotations

import math

import numpy as np
import pytest



def test_the_head_camera_frame_comes_from_the_committed_urdf():
    """It is an ACCESSOR, not a second copy of the geometry: the numbers have
    to move when the asset does."""
    import xml.etree.ElementTree as ET

    from manipulation_kit.description import WHOLEBODY_GRIPPER_URDF
    from manipulation_kit.description.head_camera import head_camera_pose

    root = ET.parse(WHOLEBODY_GRIPPER_URDF).getroot()
    joints = {j.get("name"): j for j in root.findall("joint")}
    assert "head_camera_mount" in joints
    assert "head_camera_optical_frame_joint" in joints

    p, r = head_camera_pose(neck_pitch=0.0)
    # level neck: the lens looks forward and 15 deg down (the rev1 head part)
    forward = r.as_matrix()[:, 2]
    assert math.degrees(math.asin(-forward[2])) == pytest.approx(15.0, abs=0.1)
    assert forward[0] > 0.9 and abs(forward[1]) < 1e-4
    assert 0.70 < p[2] < 0.80 and 0.05 < p[0] < 0.15


def test_the_neck_pitches_the_camera_down_and_the_lift_does_not_move_it():
    from manipulation_kit.description.head_camera import (floor_to_base_m,
                                                          head_camera_pose)
    level = head_camera_pose(neck_pitch=0.0)[1].as_matrix()[:, 2]
    down = head_camera_pose(neck_pitch=0.30)[1].as_matrix()[:, 2]
    assert -down[2] > -level[2], "positive neck_pitch must look DOWN"
    assert math.degrees(math.asin(-down[2])) == pytest.approx(
        15.0 + math.degrees(0.30), abs=0.2)
    # the LIFT is below `base`, so it cannot move the camera in `base`
    assert floor_to_base_m(0.0) == pytest.approx(0.513)
    assert floor_to_base_m(0.205) == pytest.approx(0.718)


def test_the_neck_yaw_turns_the_camera_the_way_the_joint_says():
    from manipulation_kit.description.head_camera import head_camera_pose
    left = head_camera_pose(neck_yaw=0.40)[1].as_matrix()[:, 2]
    assert abs(left[1]) > 0.3, "neck_pan did not turn the lens"


def test_the_daemons_logical_pitch_is_flipped_for_the_urdf():
    """``GET /v1/neck/state`` reports logical pitch = -motor. A consumer that
    gets that wrong aims the camera twice the neck angle away."""
    from manipulation_kit.description.head_camera import (head_camera_pose,
                                                          pose_from_neck_state)
    a = pose_from_neck_state({"pitch": -0.30, "yaw": 0.1})
    b = head_camera_pose(neck_pitch=0.30, neck_yaw=0.1)
    assert np.allclose(a[0], b[0])
    assert np.allclose(a[1].as_quat(), b[1].as_quat())


def test_a_urdf_without_the_camera_says_which_link_is_missing():
    from manipulation_kit.description import GUARD_URDF
    from manipulation_kit.description.head_camera import head_camera_pose
    with pytest.raises(KeyError) as caught:
        head_camera_pose(urdf_path=str(GUARD_URDF))
    assert "head_camera_optical_frame" in str(caught.value)


def test_the_flip_takes_the_executors_typed_neck_state_and_refuses_nonsense():
    """``neck_joints_from_state`` is the ONE implementation of the flip; it
    reads the firmware executor's ``NeckState`` (``pitch_rad``/``yaw_rad``) as
    well as a raw body, and a missing or non-finite pitch is an error, never a
    level head."""
    from types import SimpleNamespace

    from manipulation_kit.description.head_camera import (
        neck_joints_from_state, pose_from_neck_state)
    typed = SimpleNamespace(pitch_rad=-0.30, yaw_rad=0.1, enabled=True,
                            moving=False)
    assert neck_joints_from_state(typed) == (0.30, 0.1)
    assert np.allclose(pose_from_neck_state(typed)[0],
                       pose_from_neck_state({"pitch": -0.30, "yaw": 0.1})[0])
    for bad in ({"yaw": 0.1}, {"pitch": float("nan")},
                SimpleNamespace(yaw_rad=0.0)):
        with pytest.raises(ValueError):
            neck_joints_from_state(bad)
