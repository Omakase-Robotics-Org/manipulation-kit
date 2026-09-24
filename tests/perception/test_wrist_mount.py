"""The MEASURED wrist mount (kit issue #28): the calibration file's
``cameras.<side>_wrist.mount`` — seiryu-calib's plate -> optical fit — is what
the wrist camera is built with, and the trace says which mount it used.

Fixture: d1-2's installed file (2026-09-24 01:47Z). Left mount
[4.8, 82.3, 30.3] mm in ``gripper_R_camera_plate``, right [-4.7, 74.6, 34.1]
mm in ``gripper_L_camera_plate``, against the nominal [0, 79.2, 14.5] mm.
"""

from __future__ import annotations

import json
import math
import warnings
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from manipulation_kit.agent.robot import LiveRobot, wrist_camera_from_scene
from manipulation_kit.description.robot_profile import (RobotProfile,
                                                        WristMount)
from manipulation_kit.perception import WristCamera
from manipulation_kit.perception.wrist import (nominal_optical_in_plate,
                                               optical_in_flange)

ROOT = Path(__file__).resolve().parents[2]
D1_2 = ROOT / "tests" / "data" / "d1-2.camera_calibration.json"
D1_2_SCENE = ROOT / "examples" / "agent" / "scenes" / "d1-2_tape_cup.json"
CLOCK = {"left": math.pi, "right": 0.0}
MEASURED_MM = {"left": [4.8, 82.3, 30.3], "right": [-4.7, 74.6, 34.1]}


@pytest.fixture(scope="module")
def profile():
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")     # the wrist mount gates are WARN
        return RobotProfile.load(D1_2)


def _doc():
    return json.loads(D1_2.read_text(encoding="utf-8"))


def _pair(profile, side, flange_p=np.zeros(3), flange_r=R.identity()):
    kw = profile.wrist_cameras[side].camera_kwargs()
    nominal = WristCamera.from_flange(side, flange_p, flange_r, **kw)
    measured = WristCamera.from_flange(side, flange_p, flange_r,
                                       mount=profile.wrist_mounts[side], **kw)
    return nominal, measured


def test_the_file_s_nominal_is_the_kit_s_nominal():
    """seiryu-calib fitted against the kit's own URDF nominal, so the file's
    ``nominal`` and :func:`nominal_optical_in_plate` are the same pose — the
    measured absolute pose replaces exactly the thing the kit would use."""
    p, r = nominal_optical_in_plate()
    for slot in ("left_wrist", "right_wrist"):
        nominal = _doc()["cameras"][slot]["mount"]["nominal"]["T_parent_camera"]
        assert np.allclose(p, nominal["xyz_m"], atol=1e-6)
        assert (r.inv() * R.from_quat(nominal["quat_xyzw"])).magnitude() < 1e-4


@pytest.mark.parametrize("side", ["left", "right"])
def test_the_measured_mount_moves_the_lens_by_the_fitted_translation(
        profile, side):
    mount = profile.wrist_mounts[side]
    assert [round(v * 1000, 1) for v in mount.xyz_m] == MEASURED_MM[side]
    flange_r = R.from_euler("xyz", [0.3, -0.2, 1.1])
    flange_p = np.array([0.4, 0.1, 0.25])
    nominal, measured = _pair(profile, side, flange_p, flange_r)
    delta_plate = (np.asarray(mount.xyz_m)
                   - np.asarray(nominal_optical_in_plate()[0]))
    expected = flange_r.apply(R.from_euler("z", CLOCK[side]).apply(delta_plate))
    assert np.allclose(measured.p - nominal.p, expected, atol=1e-12)
    # 16-20 mm on d1-2: the lens is not on the plate's face
    assert 0.015 < float(np.linalg.norm(expected)) < 0.021
    # and the rotation is the fitted one, composed with the side's clocking
    r_expected = (flange_r * R.from_euler("z", CLOCK[side])
                  * R.from_quat(mount.quat_xyzw))
    assert (measured.r.inv() * r_expected).magnitude() < 1e-12
    # the trace says which mount
    assert nominal.to_json()["mount"] == "nominal"
    assert not nominal.calibrated
    assert measured.to_json()["mount"] == "measured"
    assert measured.calibrated
    assert "MEASURED wrist-camera mount" in measured.notes[0]
    assert "NOMINAL" in nominal.notes[0]


@pytest.mark.parametrize("side", ["left", "right"])
def test_a_pad_mark_moves_by_the_pixels_seiryu_calib_reported(profile, side):
    """The jaw-pad centres (100 mm down the approach axis, driven 64 mm
    apart) projected through the measured vs the nominal mount move by the
    pixels seiryu-calib's own ``pad_shift_from_nominal_px`` check reports
    for the same fit (32.2 px left, 27.1 px right) — an independent
    cross-check of the frame chain, the side clocking and the fisheye."""
    nominal, measured = _pair(profile, side)
    clock = R.from_euler("z", CLOCK[side])
    shifts = []
    for sign in (1.0, -1.0):
        pad = clock.apply([sign * 0.032, 0.0, 0.100])
        u0, v0 = nominal.project(pad)
        u1, v1 = measured.project(pad)
        assert measured.project_point(pad).visible
        shifts.append(math.hypot(u1 - u0, v1 - v0))
    gate = _doc()["cameras"][f"{side}_wrist"]["mount"]["gate"]
    reported = next(c["value"] for c in gate["checks"]
                    if c["name"] == "pad_shift_from_nominal_px")
    assert max(shifts) == pytest.approx(reported, abs=1.0)


def test_a_mark_projects_where_the_measured_pose_puts_it(profile):
    """A servo/look mark (a point 12 cm ahead of the flange) lands on the
    pixel an independently composed camera at the measured pose gives."""
    side = "left"
    kw = profile.wrist_cameras[side].camera_kwargs()
    mount = profile.wrist_mounts[side]
    _nominal, measured = _pair(profile, side)
    clock = R.from_euler("z", CLOCK[side])
    by_hand = WristCamera(p=clock.apply(mount.xyz_m),
                          r=clock * R.from_quat(mount.quat_xyzw), side=side,
                          **{k: v for k, v in kw.items()
                             if k != "valid_radius_px"})
    mark = clock.apply([0.01, -0.02, 0.12])
    assert np.allclose(measured.project(mark), by_hand.project(mark),
                       atol=1e-9)


def test_the_scene_block_carries_the_mount_to_the_robot(profile, d1_arm):
    block = profile.scene_block()["wrist_camera"]
    assert block["left"]["mount"]["parent_link"] == "gripper_R_camera_plate"
    wrists = wrist_camera_from_scene({"robot": profile.scene_block()},
                                     measured_only=True)
    assert isinstance(wrists["right"]["mount"], WristMount)
    scene = json.loads(D1_2_SCENE.read_text(encoding="utf-8"))
    robot = LiveRobot.from_flag("kinematic", kin=d1_arm, scene=scene,
                                profile=profile)
    cameras = robot.cameras()
    for side in ("left", "right"):
        cam = cameras[f"{side}_wrist"]
        assert cam.mount == "measured" and cam.to_json()["mount"] == "measured"
    # a profile without wrist mounts is the nominal, and says so
    doc = _doc()
    for slot in ("left_wrist", "right_wrist"):
        doc["cameras"][slot]["mount"] = None
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        bare = RobotProfile.from_camera_calibration(doc)
    assert bare.wrist_mounts == {}
    robot = LiveRobot.from_flag("kinematic", kin=d1_arm, scene=scene,
                                profile=bare)
    assert robot.cameras()["left_wrist"].to_json()["mount"] == "nominal"


def test_a_mount_for_the_other_side_is_refused(profile):
    with pytest.raises(ValueError, match="left wrist camera's mount given "
                                         "for the right arm"):
        optical_in_flange("right", profile.wrist_mounts["left"])
    # a flat (both-hands) scene block cannot carry one side's mount
    flat = dict(profile.wrist_cameras["left"].camera_kwargs(),
                mount=profile.wrist_mounts["left"].to_json())
    with pytest.raises(ValueError, match="gripper_L_camera_plate"):
        wrist_camera_from_scene({"robot": {"wrist_camera": flat}})


def test_the_wrist_mount_round_trips_through_json(profile):
    for side, mount in profile.wrist_mounts.items():
        again = WristMount.from_json(side, mount.to_json())
        assert again == mount
