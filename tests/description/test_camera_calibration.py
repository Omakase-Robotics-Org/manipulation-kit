"""The ``omakase.camera_calibration/2`` reader: structure, gates, migration,
and the promise that the wheel carries a schema but no robot's values."""

from __future__ import annotations

import copy
import json
import warnings
from pathlib import Path

import pytest

import manipulation_kit
from manipulation_kit.description import camera_calibration as cc
from manipulation_kit.description.robot_profile import RobotProfile

REPO = Path(__file__).resolve().parents[2]
D1_2 = REPO / "tests" / "data" / "d1-2.camera_calibration.json"
D1_2_V1 = REPO / "tests" / "data" / "robot_profile" / "d1-2.robot_profile-v1.json"


def _doc():
    return json.loads(D1_2.read_text(encoding="utf-8"))


def _write(tmp_path, doc, name="cal.json"):
    path = tmp_path / name
    path.write_text(json.dumps(doc), encoding="utf-8")
    return path


def _fail(doc, slot="left_wrist", layer="intrinsics"):
    gate = doc["cameras"][slot][layer]["gate"]
    gate["verdict"] = "FAIL"
    gate["reasons"] = ["rms 1.4 px over the 1.0 px FAIL line"]
    gate["override"] = None
    return doc


def test_the_d1_2_file_loads():
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        calib = cc.load(D1_2)
    assert calib.robot == "d1-2"
    assert {"head", "left_wrist", "right_wrist"} <= set(calib.cameras)
    head = calib.camera("head")
    assert (head.width, head.height) == (640, 480)
    assert head.mount.parent_link == "head_link"
    assert head.intrinsics.distortion.model == "inverse_brown_conrady"
    left = calib.camera("left_wrist")
    assert left.intrinsics.distortion.model == "kannala_brandt"
    assert len(left.intrinsics.distortion.coefficients) == 4
    assert calib.hand.open_gap_m == 0.0605


def test_a_warn_gate_is_accepted_and_said(tmp_path):
    doc = _doc()
    doc["cameras"]["head"]["mount"]["gate"]["verdict"] = "WARN"
    with pytest.warns(UserWarning, match="'head' mount gate is WARN"):
        cc.load(_write(tmp_path, doc))


def test_a_failed_gate_is_refused_naming_camera_layer_and_reasons(tmp_path):
    path = _write(tmp_path, _fail(_doc()))
    with pytest.raises(cc.FailedCalibrationGate,
                       match=r"'left_wrist' intrinsics failed.*rms 1.4 px") as err:
        cc.load(path)
    assert (err.value.camera, err.value.layer) == ("left_wrist", "intrinsics")
    with pytest.raises(cc.FailedCalibrationGate):
        RobotProfile.load(path)
    # a failed MOUNT is refused the same way
    with pytest.raises(cc.FailedCalibrationGate, match="'head' mount"):
        cc.load(_write(tmp_path, _fail(_doc(), "head", "mount"), "m.json"))


def test_a_failed_gate_with_a_file_override_is_accepted(tmp_path):
    doc = _fail(_doc())
    doc["cameras"]["left_wrist"]["intrinsics"]["gate"]["override"] = {
        "reason": "only board available; re-run next week", "by": "shu",
        "at": "2026-09-23T03:00:00Z"}
    with pytest.warns(UserWarning, match="under an override"):
        profile = RobotProfile.load(_write(tmp_path, doc))
    assert "left" in profile.wrist_cameras
    # an override without a reason is not an override
    doc["cameras"]["left_wrist"]["intrinsics"]["gate"]["override"] = {"reason": ""}
    with pytest.raises(cc.CalibrationFormatError, match="override.reason"):
        cc.load(_write(tmp_path, doc, "empty.json"))


def test_a_failed_gate_with_a_caller_override_is_accepted(tmp_path):
    path = _write(tmp_path, _fail(_doc()))
    with pytest.warns(UserWarning, match="caller allowed a failed gate"):
        profile = RobotProfile.load(path, allow_failed_gate=True)
    assert profile.wrist_cameras["left"].model == "fisheye"
    with pytest.warns(UserWarning):
        assert RobotProfile.resolve(path, allow_failed_gate=True) is not None


def test_a_wrong_schema_string_is_refused():
    doc = _doc()
    doc["schema"] = "omakase.camera_calibration/1"
    with pytest.raises(cc.CalibrationFormatError, match=r"\$\.schema"):
        cc.validate(doc)


@pytest.mark.parametrize("model,count", [("kannala_brandt", 5),
                                         ("brown_conrady", 4),
                                         ("brown_conrady_rational", 5),
                                         ("none", 1)])
def test_a_wrong_coefficient_count_is_refused(model, count):
    doc = _doc()
    doc["cameras"]["left_wrist"]["intrinsics"]["distortion"] = {
        "model": model, "coefficients": [0.0] * count}
    with pytest.raises(cc.CalibrationFormatError, match="coefficients"):
        cc.validate(doc)


@pytest.mark.parametrize("mutate,match", [
    (lambda d: d["cameras"]["head"]["mount"]["T_parent_camera"].update(
        quat_xyzw=[0.0, 0.0, 0.0, 2.0]), "unit quaternion"),
    (lambda d: d["cameras"]["left_wrist"]["intrinsics"].update(fx=float("nan")),
     "finite"),
    (lambda d: d["cameras"]["left_wrist"]["intrinsics"].pop("gate"),
     "missing required key 'gate'"),
    (lambda d: d["cameras"]["left_wrist"]["intrinsics"]["distortion"].update(
        model="fisheye"), "distortion.model"),
    (lambda d: d["cameras"]["left_wrist"]["intrinsics"].update(cx=900.0),
     "principal point"),
    (lambda d: d["cameras"]["head"]["mount"]["gate"].update(checks=[]),
     "at least one check"),
    (lambda d: d["cameras"]["head"]["mount"]["gate"].update(verdict="OK"),
     "verdict"),
    (lambda d: d["cameras"]["head"]["stream"].update(width=0), "stream.width"),
    (lambda d: d.update(hand={"open_gap_m": 0.5}), "open_gap_m"),
    (lambda d: d.update(extra=1), "unknown key 'extra'"),
    (lambda d: d.pop("robot"), "'robot'"),
])
def test_structural_problems_are_refused(mutate, match):
    doc = copy.deepcopy(_doc())
    mutate(doc)
    with pytest.raises(cc.CalibrationFormatError, match=match):
        cc.validate(doc)


def test_an_old_robot_profile_gives_the_migration():
    with pytest.raises(cc.CalibrationFormatError,
                       match=r"robot_profile/1.*camera_calibration.json.*seiryu-calib migrate"):
        RobotProfile.load(D1_2_V1)


def test_a_measured_wrist_mount_is_refused_rather_than_ignored(tmp_path):
    doc = _doc()
    doc["cameras"]["left_wrist"]["mount"] = copy.deepcopy(
        doc["cameras"]["head"]["mount"])
    doc["cameras"]["left_wrist"]["mount"]["parent_link"] = "left_camera_plate"
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        cc.load(_write(tmp_path, doc))           # the FILE is valid...
        with pytest.raises(ValueError, match="does not apply one to a wrist"):
            RobotProfile.load(_write(tmp_path, doc))   # ...the kit refuses


def test_a_distorted_brown_conrady_wrist_is_refused(tmp_path):
    doc = _doc()
    doc["cameras"]["right_wrist"]["intrinsics"]["distortion"] = {
        "model": "brown_conrady", "coefficients": [0.1, 0.0, 0.0, 0.0, 0.0]}
    with pytest.raises(ValueError, match="kannala_brandt"):
        RobotProfile.load(_write(tmp_path, doc))
    doc["cameras"]["right_wrist"]["intrinsics"]["distortion"]["coefficients"] = [0.0] * 5
    assert RobotProfile.load(_write(tmp_path, doc)).wrist_cameras["right"].model == "pinhole"


def test_the_default_is_the_robot_s_own_file(tmp_path, monkeypatch):
    assert cc.DEFAULT_PATH.name == "camera_calibration.json"
    assert cc.DEFAULT_PATH.parent.parts[-2:] == (".config", "omakase")
    monkeypatch.setattr(cc, "DEFAULT_PATH", tmp_path / "camera_calibration.json")
    assert cc.installed_path() is None                 # not a robot
    cc.DEFAULT_PATH.write_text(D1_2.read_text(encoding="utf-8"))
    assert cc.installed_path() == cc.DEFAULT_PATH      # a robot


def test_the_shipped_schema_is_the_reader_s():
    schema = json.loads(cc.SCHEMA_PATH.read_text(encoding="utf-8"))
    assert schema["properties"]["schema"]["const"] == cc.SCHEMA
    models = schema["$defs"]["distortion"]["properties"]["model"]["enum"]
    assert set(models) == set(cc.DISTORTION_COEFFICIENTS)


def test_the_package_carries_no_per_robot_values():
    """A schema ships; a robot's numbers never do."""
    package = Path(manipulation_kit.__file__).resolve().parent
    assert not (package / "description" / "profiles").exists()
    offenders = []
    for path in package.rglob("*.json"):
        if "schemas" in path.parts or "_client" in path.parts:
            continue
        try:
            doc = json.loads(path.read_text(encoding="utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue
        if isinstance(doc, dict) and (
                "robot" in doc or str(doc.get("schema", "")).startswith(
                    ("omakase.camera_calibration", "manipulation_kit.robot_profile"))):
            offenders.append(str(path.relative_to(package)))
    assert offenders == []
    pyproject = (REPO / "pyproject.toml").read_text(encoding="utf-8")
    assert "profiles/" not in pyproject
    assert "schemas/*.schema.json" in pyproject
