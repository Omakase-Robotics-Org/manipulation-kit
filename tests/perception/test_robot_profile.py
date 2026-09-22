"""One robot's measured numbers, and the lens models they feed (step 9).

The profile carries d1-2's MEASURED wrist fisheyes (d1-calibrate-wrist,
2026-09-22), its gripper gap and its head-camera mount; the wrist camera
projects through the equidistant fisheye model in numpy, and the head camera
applies the mount — and says ``calibrated`` — only when it has one.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from manipulation_kit.agent.robot import (LiveRobot, load_scene,
                                          wrist_camera_from_scene)
from manipulation_kit.description.robot_profile import (RobotProfile,
                                                        WristIntrinsics,
                                                        scene_robot_block)
from manipulation_kit.perception import HeadCamera, WristCamera
from manipulation_kit.perception.wrist import (fisheye_distort,
                                               fisheye_undistort)

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "tests" / "data" / "robot_profile"
D1_2_SCENE = ROOT / "examples" / "agent" / "scenes" / "d1-2_tape_cup.json"
K = (-0.033292, -0.016422, 0.008260, -0.002041)       # d1-2 left, rounded


def _wrist(model="fisheye", k=K, **extra):
    return WristCamera(fx=238.5, fy=239.7, cx=314.0, cy=224.8, width=640,
                       height=480, p=np.zeros(3), r=R.identity(),
                       model=model, k=k, **extra)


def test_a_fisheye_projects_a_known_point_and_recovers_it():
    camera = _wrist()
    for off_axis_deg, azimuth_deg in ((0.0, 0.0), (10.0, 30.0), (30.0, 120.0),
                                      (55.0, -60.0), (75.0, 200.0)):
        theta = math.radians(off_axis_deg)
        phi = math.radians(azimuth_deg)
        point = 0.4 * np.array([math.sin(theta) * math.cos(phi),
                                math.sin(theta) * math.sin(phi),
                                math.cos(theta)])
        u, v = camera.project(point)
        # the model, by hand: equidistant radius theta_d, scaled by f
        theta_d = float(fisheye_distort(theta, K))
        assert u == pytest.approx(314.0 + 238.5 * theta_d * math.cos(phi),
                                  abs=1e-9)
        assert v == pytest.approx(224.8 + 239.7 * theta_d * math.sin(phi),
                                  abs=1e-9)
        # ...and back: the ray through that pixel IS the point's direction
        ray = camera.ray(u, v)
        assert np.allclose(ray, point / np.linalg.norm(point), atol=1e-9)
    assert float(fisheye_undistort(fisheye_distort(1.2, K), K)) == \
        pytest.approx(1.2, abs=1e-12)


def test_a_pinhole_model_of_the_wrist_lens_is_wrong_off_axis():
    """Why the lens model matters: ~10 % at 30 deg, where the look looks."""
    fisheye, pinhole = _wrist(), _wrist(model="pinhole", k=())
    theta = math.radians(30.0)
    point = np.array([math.sin(theta), 0.0, math.cos(theta)])
    r_fish = fisheye.project(point)[0] - 314.0
    r_pin = pinhole.project(point)[0] - 314.0
    assert (r_pin - r_fish) / r_fish > 0.09
    # a pixel near the edge, unprojected by each: tens of degrees apart
    edge = fisheye.ray(600.0, 224.8)
    assert math.degrees(math.acos(float(np.dot(edge, pinhole.ray(600.0, 224.8))))) > 20


def test_the_wrist_camera_trusts_only_its_calibrated_radius():
    camera = _wrist(valid_radius_px=200.0)
    theta = math.radians(70.0)
    far = np.array([math.sin(theta), 0.0, math.cos(theta)])
    seen = camera.project_point(far)
    assert not seen.visible and "calibrated radius" in seen.reason
    near = camera.project_point(np.array([0.05, 0.0, 0.4]))
    assert near.visible


def test_the_profile_reads_the_d1_inference_calibration_files_verbatim():
    built = RobotProfile.from_files(
        "d1-2", wrist={"left": DATA / "wrist_left_intrinsics.json",
                       "right": DATA / "wrist_right_intrinsics.json"})
    committed = RobotProfile.named("d1-2")
    assert built.wrist_cameras == committed.wrist_cameras
    left = committed.wrist_cameras["left"]
    assert (left.model, left.width, left.height) == ("fisheye", 640, 480)
    assert left.fx == pytest.approx(238.5446, abs=1e-4)
    assert left.valid_radius_px == pytest.approx(304.3)
    assert committed.wrist_cameras["right"].cx == pytest.approx(328.3726, abs=1e-4)
    assert committed.hand.open_gap_m == pytest.approx(0.0605)
    assert RobotProfile.from_json(committed.to_json()) == committed
    with pytest.raises(ValueError, match="the left wrist, not the right"):
        RobotProfile.from_files("x", wrist={"right": DATA / "wrist_left_intrinsics.json"})
    with pytest.raises(LookupError, match="d1-2"):
        RobotProfile.named("no-such-robot")


def test_the_d1_2_head_mount_is_the_fitted_pose_whatever_the_nominal_is():
    """The delta is kept WITH the nominal it was fitted against (17.25 deg),
    so the rebuilt head_link -> optical is the fit's absolute pose — not the
    delta pasted onto the kit's current 15 deg nominal."""
    mount = RobotProfile.named("d1-2").head_mount_delta
    p, r = mount.head_link_to_optical()
    assert np.allclose(p, [0.132879, -0.107000, -0.004431], atol=2e-6)
    assert np.allclose(r.as_matrix()[:, 2], [0.978358, 0.202684, -0.041656],
                       atol=2e-6)
    assert np.allclose(np.asarray(mount.xyz_m) * 1000, [-23.9, -43.0, 24.6],
                       atol=0.1)


def test_the_head_camera_with_the_d1_2_profile_moves_a_wagon_point():
    """The run4/run5 error, closed: the same pixel on the wagon top (the
    cube's, d1-2 2026-09-22) lands 8 cm further out with the measured mount
    than through the URDF nominal — and only the measured one says
    calibrated."""
    mount = RobotProfile.named("d1-2").head_mount_delta
    kwargs = dict(width=640, height=480, fx=606.54, fy=605.90, cx=325.76,
                  cy=250.35, neck_pitch=0.62)
    nominal = HeadCamera.from_robot(**kwargs)
    measured = HeadCamera.from_robot(mount_delta=mount, **kwargs)
    assert measured.calibrated and not nominal.calibrated
    assert "calibrated mount" in measured.to_text()
    assert "CALIBRATED" in measured.notes[0]
    a = nominal.locate(568, 436, plane_z=0.166).p
    b = measured.locate(568, 436, plane_z=0.166).p
    shift = float(np.linalg.norm(a - b))
    assert 0.06 < shift < 0.10, shift
    assert b[0] > a[0]                      # further out, not sideways
    assert HeadCamera.from_json(measured.to_json()).calibrated


def test_a_scene_naming_the_profile_gets_the_measured_wrists():
    scene = load_scene(D1_2_SCENE)
    wrists = wrist_camera_from_scene(scene, measured_only=True)
    assert set(wrists) == {"left", "right"}
    assert wrists["left"]["model"] == "fisheye" and len(wrists["left"]["k"]) == 4
    # a scene key overrides the profile's, and a placeholder never reaches
    # hardware
    raw = json.loads(D1_2_SCENE.read_text(encoding="utf-8"))
    raw["robot"]["hand"] = {"open_gap_m": 0.058}
    assert scene_robot_block(raw)["hand"]["open_gap_m"] == 0.058
    placeholder = {"robot": {"wrist_camera": {
        "fx": 320.0, "fy": 320.0, "cx": 320.0, "cy": 240.0, "width": 640,
        "height": 480, "measured": False}}}
    assert wrist_camera_from_scene(placeholder) is not None
    assert wrist_camera_from_scene(placeholder, measured_only=True) is None


def test_a_robot_built_with_the_profile_looks_through_the_fisheye(d1_arm):
    scene = json.loads(D1_2_SCENE.read_text(encoding="utf-8"))
    del scene["robot"]
    robot = LiveRobot.from_flag("kinematic", kin=d1_arm, scene=scene,
                                profile=RobotProfile.named("d1-2"))
    assert robot.profile.name == "d1-2" and robot.has_wrist_camera()
    cameras = robot.cameras()
    assert cameras["left_wrist"].model == "fisheye"
    assert cameras["right_wrist"].valid_radius_px == pytest.approx(326.0, abs=1)
    assert WristIntrinsics.from_json(
        json.loads((DATA / "wrist_right_intrinsics.json").read_text())).model == "fisheye"
