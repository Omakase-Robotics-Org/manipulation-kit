"""Guard profiles: d1 is the guard's defaults, sim equals d1, only d1-arms-stand
loosens and only the body checks and the hand envelope; one neck yaw;
safety_zones.json deprecated."""

from __future__ import annotations

import importlib.util
import json
import xml.etree.ElementTree as ET

import pytest

from manipulation_kit import description
from manipulation_kit.description import bundle as db
from manipulation_kit.guard import MotionGuard

# Poses (deg, SDK order J1..J7) the guard's own tests verified by FK.
POSE_BODY_HIT = [-73.4, 113.3, -132.1, -65.6, 89.0, -41.8, -2.0]
POSE_ELBOW_INTO_TORSO = [-20.8, -106.6, -3.6, -95.7, 24.0, -38.2, -63.9]
POSE_AA_A = [158.8, 87.8, -134.1, -66.6, 89.8, -20.3, 90.0]
POSE_AA_B = [-158.8, 87.8, 134.1, -66.6, -89.8, -20.3, 90.0]

POLICY_KEYS = ("check_body", "check_arm_arm", "check_self", "check_hand_envelope",
               "body_margin_m",
               "arm_arm_margin_m", "self_margin_m", "body_exempt_links",
               "disabled_body_boxes", "chest_keepout", "chest_exempt_links")


def _policy(name):
    p = db.guard_profile(name)
    return {k: p[k] for k in POLICY_KEYS}


def _guard(name):
    return MotionGuard(**db.motion_guard_kwargs(db.guard_profile(name)))


def test_each_model_s_bundle_carries_its_profiles():
    carried = set()
    for model, m in db.ROBOT_MODELS.items():
        files = db.bundle_files(model)
        for name in m.guard_profiles:
            prof = json.loads(files[f"guard/profile.{name}.json"])
            assert prof == db.guard_profile(name)
            assert prof["schema"] == db.GUARD_PROFILE_SCHEMA
            carried.add(name)
        others = {p for p in files if p.startswith("guard/")} - {
            f"guard/profile.{n}.json" for n in m.guard_profiles}
        assert not others, (model, others)
    assert carried == set(db.GUARD_PROFILES)


def test_d1_builds_the_default_guard():
    d1, default = _guard("d1"), MotionGuard()
    for attr in ("check_body", "check_arm_arm", "check_self",
                 "body_margin_m",
                 "arm_arm_margin_m", "self_margin_m", "body_exempt_links",
                 "disabled_body_boxes", "chest_keepout", "chest_exempt_links"):
        assert getattr(d1, attr) == getattr(default, attr), attr


def test_sim_is_d1():
    assert _policy("sim") == _policy("d1")


def test_only_d1_arms_stand_loosens_and_only_the_body_and_the_end_effectors():
    d1, stand = _policy("d1"), _policy("d1-arms-stand")
    changed = {k for k in POLICY_KEYS if d1[k] != stand[k]}
    assert changed == {"check_body", "chest_keepout", "check_hand_envelope"}
    assert stand["check_body"] is False and stand["chest_keepout"] is None
    assert stand["check_hand_envelope"] is False and d1["check_hand_envelope"] is True
    assert stand["check_arm_arm"] and stand["check_self"]
    for name in db.ROBOT_MODELS["d1"].guard_profiles:
        if name != "d1-arms-stand":
            assert _policy(name) == d1, name


def test_the_d1_bundle_carries_the_stand_profile_and_not_d1_bench():
    assert db.ROBOT_MODELS["d1"].guard_profiles == ("d1", "d1-arms-stand", "sim")
    files = db.bundle_files("d1")
    assert "guard/profile.d1-arms-stand.json" in files
    assert "guard/profile.d1-bench.json" not in files
    assert "d1-bench" not in db.GUARD_PROFILES
    # The stand's own bundle carries the very same profile.
    assert (db.bundle_files("d1-arms-stand")["guard/profile.d1-arms-stand.json"]
            == files["guard/profile.d1-arms-stand.json"])


def test_the_profile_file_says_the_end_effectors_are_not_checked():
    stand = json.loads(db.bundle_files("d1")["guard/profile.d1-arms-stand.json"])
    assert stand["check_hand_envelope"] is False and stand["check_body"] is False
    assert stand["chest_keepout"] is None
    assert json.loads(db.bundle_files("d1")["guard/profile.d1.json"])[
        "check_hand_envelope"] is True


def test_selection_rules():
    assert db.guard_profile("d1")["selected_by"] == {"backend": "real", "record": None}
    assert db.guard_profile("d1-arms-stand")["selected_by"] == {"backend": "real",
                                                               "record": None}
    assert db.guard_profile("sim")["selected_by"] == {"backend": "sim", "record": None}


def test_daemon_keys_repeat_the_policy():
    for name in db.GUARD_PROFILES:
        p = db.guard_profile(name)
        assert p["daemon_keys"] == {"arm.guard_body": p["check_body"],
                                    "arm.guard_body_margin_m": p["body_margin_m"],
                                    "arm.guard_arm_arm_margin_m": p["arm_arm_margin_m"]}


@pytest.mark.parametrize("pose", [POSE_BODY_HIT, POSE_ELBOW_INTO_TORSO])
def test_a_body_hit_is_refused_on_the_robot_and_not_on_the_stand(pose):
    assert not _guard("d1").check(pose, None).ok
    assert not _guard("sim").check(pose, None).ok
    assert _guard("d1-arms-stand").check(pose, None).ok


def test_the_stand_still_keeps_the_arms_apart():
    assert not _guard("d1-arms-stand").check(POSE_AA_A, POSE_AA_B).ok


def test_an_unknown_profile_is_refused():
    with pytest.raises(KeyError, match="unknown guard profile"):
        db.guard_profile("d1-relaxed")


def _generator():
    path = description.ROOT / "d1" / "tools" / "generate_d1_urdf.py"
    spec = importlib.util.spec_from_file_location("_gen_d1_urdf", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_one_neck_yaw_everywhere():
    want = description.NECK_YAW_LIMITS_RAD
    gen = _generator()
    assert tuple(gen.NECK_YAW_LIMITS_RAD) == want
    assert tuple(gen.NECK_PAN[3]) == want
    seen = 0
    for urdf in sorted((description.ROOT).rglob("*.urdf")):
        for j in ET.parse(str(urdf)).getroot().iter("joint"):
            if j.get("name") == "neck_pan":
                lim = j.find("limit")
                assert (float(lim.get("lower")), float(lim.get("upper"))) == want, urdf
                seen += 1
    assert seen >= 2, "the whole-body URDFs carry the neck"


def test_safety_zones_json_says_it_is_deprecated():
    zones = json.loads((description.ROOT.parent / "config" / "safety_zones.json")
                       .read_text(encoding="utf-8"))
    assert zones["_deprecated"].startswith("DEPRECATED")


def test_the_profile_file_keys_the_daemon_reads_are_pinned(tmp_path):
    # d1-firmwared reads these keys from guard/profile.<name>.json in a
    # bundle: `check_body` is canonical, `daemon_keys["arm.guard_body"]`
    # repeats it.
    db.write_bundle(tmp_path / "b")
    b = db.load_bundle(tmp_path / "b")
    for name in db.ROBOT_MODELS["d1"].guard_profiles:
        doc = b.read_json(f"guard/profile.{name}.json")
        assert doc["schema"] == "manipulation_kit.guard_profile/1"
        assert isinstance(doc["check_body"], bool)
        assert set(doc["daemon_keys"]) == {"arm.guard_body", "arm.guard_body_margin_m",
                                           "arm.guard_arm_arm_margin_m"}
        assert doc["daemon_keys"]["arm.guard_body"] is doc["check_body"]
        assert set(doc["selected_by"]) == {"backend", "record"}
    stand = b.read_json("guard/profile.d1-arms-stand.json")
    assert stand["selected_by"] == {"backend": "real", "record": None}
    assert stand["check_body"] is False
    assert b.read_json("guard/profile.d1.json")["check_body"] is True
