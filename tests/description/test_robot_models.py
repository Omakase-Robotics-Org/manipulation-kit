"""Robot models: each one is its own bundle with its own URDFs, collision
model, guard profile and lock; two D1 arms on a stand is a model of its own,
with no D1 body and no end effector, and keeps the arms apart as the D1
does."""

from __future__ import annotations

import importlib.util
import json
import re
import xml.etree.ElementTree as ET

import pytest

from manipulation_kit import description
from manipulation_kit.description import bundle as db
from manipulation_kit.description import robots
from manipulation_kit.guard import MotionGuard

# Poses (deg, SDK order J1..J7) the guard's own tests verified by FK.
POSE_BODY_HIT = [-73.4, 113.3, -132.1, -65.6, 89.0, -41.8, -2.0]
POSE_AA_A = [158.8, 87.8, -134.1, -66.6, 89.8, -20.3, 90.0]
POSE_AA_B = [-158.8, 87.8, 134.1, -66.6, -89.8, -20.3, 90.0]

STAND = "d1-arms-stand"


def _stand_guard():
    m = robots.robot_model(STAND)
    return MotionGuard(str(description.ROOT / "d1" / m.guard_urdf),
                       **db.motion_guard_kwargs(db.guard_profile(m.guard_profiles[0])))


def _urdf(name):
    return ET.parse(str(description.ROOT / "d1" / name)).getroot()


@pytest.mark.parametrize("model", sorted(robots.ROBOT_MODELS))
def test_every_model_s_bundle_is_locked(model):
    lock = db.read_lock(db.lock_path(model))
    assert lock["model"] == model
    assert db.lock_problems(lock, db.digest(model)) == []
    assert lock["files"] == db.file_hashes(db.bundle_files(model))


@pytest.mark.parametrize("model", sorted(robots.ROBOT_MODELS))
def test_a_written_bundle_states_its_model(model, tmp_path):
    man = db.write_bundle(tmp_path / model, model=model)
    assert man["model"] == model
    b = db.load_bundle(tmp_path / model)
    doc = b.robot()
    assert doc == robots.robot_model(model).document()
    for role in ("guard", "view"):
        assert b.path(doc["urdf"][role]).is_file()
    assert f"guard/profile.{doc['guard_profile']}.json" in {
        e["path"] for e in man["files"]}
    assert doc["recorded_input"] in robots.RECORDED_INPUT


def test_model_ids_name_the_structure():
    for model in robots.ROBOT_MODELS:
        assert re.fullmatch(r"[a-z0-9]+(-[a-z0-9]+)*", model), model
        assert not {"bench", "demo"} & set(model.split("-")), model


def test_the_d1_bundle_keeps_its_paths():
    files = db.bundle_files("d1")
    for p in ("kinematics/d1.urdf", "kinematics/d1_wholebody_gripper.urdf",
              "guard/profile.d1.json", "guard/profile.d1-bench.json",
              "guard/profile.sim.json", "robot.json"):
        assert p in files
    doc = json.loads(files["robot.json"])
    assert doc["end_effectors"]["collision_checked"] is True
    assert doc["recorded_input"] == "refused"


def test_the_stand_says_its_end_effectors_are_not_checked():
    doc = json.loads(db.bundle_files(STAND)["robot.json"])
    assert doc["end_effectors"]["collision_checked"] is False
    assert doc["recorded_input"] == "operator_override"
    assert doc["devices"] == ["arms", "end_effectors"]


def test_a_bundle_of_an_unknown_model_is_refused(tmp_path):
    db.write_bundle(tmp_path / "b", model=STAND)
    man = json.loads((tmp_path / "b" / "manifest.json").read_text())
    man["model"] = "d2"
    (tmp_path / "b" / "manifest.json").write_text(json.dumps(man))
    assert any("not a known robot model" in p for p in db.verify_bundle(tmp_path / "b"))


def test_an_unknown_model_is_refused():
    with pytest.raises(KeyError, match="unknown robot model"):
        robots.robot_model("d1-arms")


def test_the_stand_has_no_d1_body_and_no_end_effector():
    for name in ("d1_arms_stand.urdf", "d1_arms_stand_visual.urdf"):
        root = _urdf(name)
        links = {link.get("name") for link in root.iter("link")}
        assert not links & {"torso_column", "head_link", "chassis_link",
                            "neck_pan_link", "base_footprint"}, name
        assert {"dual_base", "stand", "Base_R", "Base_L",
                "TCP_Link_R", "TCP_Link_L"} <= links
        # The arms end at their flanges: nothing hangs off TCP_Link.
        children = {j.find("parent").get("link") for j in root.iter("joint")}
        assert not {"TCP_Link_R", "TCP_Link_L"} & children, name
        assert not any(link.startswith(("hand_envelope", "gripper", "yubi"))
                       for link in links), name
        joints = {j.get("name") for j in root.iter("joint")}
        assert "lift" not in joints and "neck_pan" not in joints


def test_the_stand_mounts_the_arms_where_the_d1_does():
    stand, d1 = _urdf("d1_arms_stand.urdf"), _urdf("d1.urdf")

    def origin(root, name):
        j = next(j for j in root.iter("joint") if j.get("name") == name)
        o = j.find("origin")
        return o.get("xyz"), o.get("rpy")

    for side in ("R", "L"):
        for name in [f"mount_{side}"] + [f"Joint{i}_{side}" for i in range(1, 8)] + [
                f"JointTCP_{side}"]:
            assert origin(stand, name) == origin(d1, name), name


def test_the_stand_guard_checks_the_arms_only():
    guard = _stand_guard()
    assert guard.body_aabbs == [] and guard.body_cylinders == []
    assert guard.chest_keepout == {}
    assert guard.check(POSE_BODY_HIT, None).ok
    assert not guard.check(POSE_AA_A, POSE_AA_B).ok


def test_the_stand_exports_as_a_variant(tmp_path):
    from manipulation_kit.description.tools import export_description as ex
    from manipulation_kit.description.variants import resolve

    v = resolve(robots.robot_model(STAND).view_variant)
    manifest = ex.export(v.flavour, str(tmp_path))
    assert set(manifest["files"]) == {"d1_arms_stand_visual.urdf"}
