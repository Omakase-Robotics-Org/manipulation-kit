"""One robot's measured numbers, and the lens models they feed (step 9).

The profile is built from d1-2's own ``omakase.camera_calibration/2`` file (a
copy under ``tests/data`` — the kit ships no robot's numbers): its MEASURED
wrist fisheyes (d1-calibrate-wrist, 2026-09-22), its gripper gap and its
head-camera mount; the wrist camera
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
from manipulation_kit.description.robot_profile import (NotACalibrationFile,
                                                        RobotProfile,
                                                        scene_robot_block)
from manipulation_kit.perception import HeadCamera, WristCamera
from manipulation_kit.perception.wrist import (fisheye_distort,
                                               fisheye_undistort)

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "tests" / "data" / "robot_profile"
D1_2_SCENE = ROOT / "examples" / "agent" / "scenes" / "d1-2_tape_cup.json"
#: d1-2's calibration file, as the robot holds it (installed 2026-09-24
#: 01:47Z: seiryu-calib wrist mounts + head-correct head mount)
D1_2 = ROOT / "tests" / "data" / "d1-2.camera_calibration.json"
#: the same robot's file as first MIGRATED from the v1 profile (2026-09-23):
#: the reference the v1 parity tests hold the reader to
D1_2_MIGRATED = DATA / "d1-2.camera_calibration.migrated-20260923.json"
#: the kit's old committed profile (manipulation_kit.robot_profile/1), kept
#: as the reference the v2 file must reproduce
D1_2_V1 = DATA / "d1-2.robot_profile-v1.json"
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


def test_the_profile_carries_the_wrist_calibration_files_numbers_verbatim():
    """The v2 file's wrist lenses are d1-calibrate-wrist's
    ``wrist_<side>_intrinsics.json`` numbers, bit for bit, and the old
    committed profile's."""
    profile = RobotProfile.load(D1_2_MIGRATED)
    v1 = json.loads(D1_2_V1.read_text(encoding="utf-8"))
    assert set(profile.wrist_cameras) == {"left", "right"}
    for side in ("left", "right"):
        wrist = profile.wrist_cameras[side]
        raw = json.loads((DATA / f"wrist_{side}_intrinsics.json").read_text())
        old = v1["wrist_cameras"][side]
        for ref in (raw, old):
            assert (wrist.fx, wrist.fy, wrist.cx, wrist.cy) == (
                ref["fx"], ref["fy"], ref["cx"], ref["cy"])
            assert (wrist.width, wrist.height) == (ref["width"], ref["height"])
            assert wrist.model == ref["model"] == "fisheye"
            assert list(wrist.k) == ref["k"]
            assert wrist.valid_radius_px == ref["valid_radius_px"]
            assert wrist.rms_px == ref["rms_px"]
    assert profile.name == "d1-2"
    assert profile.hand.open_gap_m == v1["hand"]["open_gap_m"] == 0.0605
    # the installed file's lenses are the same fit (seiryu re-serialised
    # them: the last bit of one focal length moved)
    installed = RobotProfile.load(D1_2)
    for side in ("left", "right"):
        a, b = installed.wrist_cameras[side], profile.wrist_cameras[side]
        assert np.allclose([a.fx, a.fy, a.cx, a.cy], [b.fx, b.fy, b.cx, b.cy],
                           rtol=0, atol=1e-9)
        assert a.k == b.k and a.valid_radius_px == b.valid_radius_px


def test_per_robot_values_are_not_the_kits():
    """``named`` and the committed profiles are gone; a bare robot name says
    where the numbers live now."""
    assert not hasattr(RobotProfile, "named")
    import manipulation_kit.description.robot_profile as rp
    assert not hasattr(rp, "PROFILES")
    with pytest.raises(NotACalibrationFile, match="camera_calibration.json"):
        RobotProfile.resolve("d1-2")
    with pytest.raises(FileNotFoundError):
        RobotProfile.resolve("no/such/file.json")
    assert RobotProfile.resolve(None) is None
    # a relative path resolves against the scene's directory
    assert RobotProfile.resolve(D1_2.name, relative_to=D1_2.parent).name == "d1-2"


def test_the_head_mount_round_trips_to_the_files_absolute_pose():
    """fixture -> HeadMountDelta -> head_link_to_optical == the file's
    absolute T_parent_camera, and == what the old delta + nominal produced."""
    from manipulation_kit.description.robot_profile import HeadMountDelta
    for path in (D1_2, D1_2_MIGRATED):
        doc = json.loads(path.read_text(encoding="utf-8"))
        absolute = doc["cameras"]["head"]["mount"]["T_parent_camera"]
        p, r = RobotProfile.load(path).head_mount_delta.head_link_to_optical()
        assert np.allclose(p, absolute["xyz_m"], atol=1e-9)
        q = r.as_quat()
        q = q if np.dot(q, absolute["quat_xyzw"]) >= 0 else -q
        assert np.allclose(q, absolute["quat_xyzw"], atol=1e-9)
    doc = json.loads(D1_2_MIGRATED.read_text(encoding="utf-8"))
    absolute = doc["cameras"]["head"]["mount"]["T_parent_camera"]
    mount = RobotProfile.load(D1_2_MIGRATED).head_mount_delta
    p, r = mount.head_link_to_optical()
    assert np.allclose(p, absolute["xyz_m"], atol=1e-9)
    q = r.as_quat()
    q = q if np.dot(q, absolute["quat_xyzw"]) >= 0 else -q
    assert np.allclose(q, absolute["quat_xyzw"], atol=1e-9)
    old = json.loads(D1_2_V1.read_text(encoding="utf-8"))["head_mount_delta"]
    old_mount = HeadMountDelta(tuple(old["xyz_m"]), tuple(old["rpy_deg"]),
                               tuple(old["nominal_xyz_m"]),
                               tuple(old["nominal_quat_xyzw"]))
    p_old, r_old = old_mount.head_link_to_optical()
    assert np.allclose(p, p_old, atol=1e-9)
    assert np.allclose(r.as_matrix(), r_old.as_matrix(), atol=1e-9)
    assert np.allclose(mount.xyz_m, old["xyz_m"], atol=1e-6)
    assert np.allclose(mount.rpy_deg, old["rpy_deg"], atol=1e-4)
    assert mount.nominal_quat_xyzw == tuple(old["nominal_quat_xyzw"])
    assert mount.rms_px == pytest.approx(3.892)


def test_the_d1_2_head_mount_is_the_fitted_pose_whatever_the_nominal_is():
    """The delta is kept WITH the nominal it was fitted against (17.25 deg),
    so the rebuilt head_link -> optical is the fit's absolute pose — not the
    delta pasted onto the kit's current 15 deg nominal."""
    mount = RobotProfile.load(D1_2_MIGRATED).head_mount_delta
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
    mount = RobotProfile.load(D1_2_MIGRATED).head_mount_delta
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


def test_the_installed_head_correct_mount_is_what_the_head_camera_uses():
    """The installed file's head mount (seiryu-calib head-correct, arm FK as
    the world reference, 0.43 px) is fitted against the kit's CURRENT 15 deg
    nominal; the head camera rebuilds exactly that pose, and the wagon pixel
    moves 4.5 cm further out than the nominal puts it."""
    mount = RobotProfile.load(D1_2).head_mount_delta
    assert mount.rms_px == pytest.approx(0.4253)
    assert np.allclose(np.asarray(mount.xyz_m) * 1000, [-21.2, 6.5, -11.3],
                       atol=0.1)
    kwargs = dict(width=640, height=480, fx=606.54, fy=605.90, cx=325.76,
                  cy=250.35, neck_pitch=0.62)
    a = HeadCamera.from_robot(**kwargs).locate(568, 436, plane_z=0.166).p
    b = HeadCamera.from_robot(mount_delta=mount, **kwargs).locate(
        568, 436, plane_z=0.166).p
    assert np.allclose(b - a, [0.0445, -0.0056, 0.0], atol=0.001)


def test_a_scene_resolved_against_the_profile_gets_the_measured_wrists(tmp_path):
    scene = load_scene(D1_2_SCENE, profile=D1_2)
    wrists = wrist_camera_from_scene(scene, measured_only=True)
    assert set(wrists) == {"left", "right"}
    assert wrists["left"]["model"] == "fisheye" and len(wrists["left"]["k"]) == 4
    # a scene key overrides the profile's, and a placeholder never reaches
    # hardware
    raw = json.loads(D1_2_SCENE.read_text(encoding="utf-8"))
    raw["robot"] = {"hand": {"open_gap_m": 0.058}}
    assert scene_robot_block(raw, profile=D1_2)["hand"]["open_gap_m"] == 0.058
    # a scene may name the file itself, relative to the scene
    (tmp_path / "cal.json").write_text(D1_2.read_text(encoding="utf-8"))
    named = dict(raw, robot={"profile": "cal.json"})
    (tmp_path / "scene.json").write_text(json.dumps(named))
    assert set(wrist_camera_from_scene(load_scene(tmp_path / "scene.json"),
                                       measured_only=True)) == {"left", "right"}
    placeholder = {"robot": {"wrist_camera": {
        "fx": 320.0, "fy": 320.0, "cx": 320.0, "cy": 240.0, "width": 640,
        "height": 480, "measured": False}}}
    assert wrist_camera_from_scene(placeholder) is not None
    assert wrist_camera_from_scene(placeholder, measured_only=True) is None


def test_a_robot_built_with_the_profile_looks_through_the_fisheye(d1_arm):
    scene = json.loads(D1_2_SCENE.read_text(encoding="utf-8"))
    assert "robot" not in scene                  # the kit's scene carries none
    robot = LiveRobot.from_flag("kinematic", kin=d1_arm, scene=scene,
                                profile=RobotProfile.load(D1_2))
    assert robot.profile.name == "d1-2" and robot.has_wrist_camera()
    cameras = robot.cameras()
    assert cameras["left_wrist"].model == "fisheye"
    assert cameras["right_wrist"].valid_radius_px == pytest.approx(326.0, abs=1)
    # a path works as well as a profile
    by_path = LiveRobot.from_flag("kinematic", kin=d1_arm, scene=scene,
                                  profile=D1_2)
    assert by_path.profile.name == "d1-2" and by_path.has_wrist_camera()
