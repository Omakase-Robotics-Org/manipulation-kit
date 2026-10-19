"""Robot models: each one is its own bundle with its own URDFs, collision
model, guard profile and lock; the arm + end-effector bench rig is a model of
its own, with no D1 body, and keeps the arms apart as the D1 does."""

from __future__ import annotations

import importlib.util
import json
import xml.etree.ElementTree as ET

import pytest

from manipulation_kit import description
from manipulation_kit.description import bundle as db
from manipulation_kit.description import robots
from manipulation_kit.guard import MotionGuard
from manipulation_kit.hands.catalog import FIVE_FINGER_DEFAULT

# Poses (deg, SDK order J1..J7) the guard's own tests verified by FK.
POSE_BODY_HIT = [-73.4, 113.3, -132.1, -65.6, 89.0, -41.8, -2.0]
POSE_AA_A = [158.8, 87.8, -134.1, -66.6, 89.8, -20.3, 90.0]
POSE_AA_B = [-158.8, 87.8, 134.1, -66.6, -89.8, -20.3, 90.0]

RIG = "arm-ee-bench"


def _generator():
    path = description.ROOT / "d1" / "tools" / "generate_d1_urdf.py"
    spec = importlib.util.spec_from_file_location("_gen_d1_urdf_models", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _rig_guard():
    m = robots.robot_model(RIG)
    return MotionGuard(str(description.ROOT / "d1" / m.guard_urdf),
                       **db.motion_guard_kwargs(db.guard_profile(m.guard_profiles[0])))


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


def test_the_d1_bundle_keeps_its_paths():
    files = db.bundle_files("d1")
    for p in ("kinematics/d1.urdf", "kinematics/d1_wholebody_gripper.urdf",
              "guard/profile.d1.json", "guard/profile.d1-bench.json",
              "guard/profile.sim.json", "robot.json"):
        assert p in files


def test_a_bundle_of_an_unknown_model_is_refused(tmp_path):
    db.write_bundle(tmp_path / "b", model=RIG)
    man = json.loads((tmp_path / "b" / "manifest.json").read_text())
    man["model"] = "d2"
    (tmp_path / "b" / "manifest.json").write_text(json.dumps(man))
    assert any("not a known robot model" in p for p in db.verify_bundle(tmp_path / "b"))


def test_an_unknown_model_is_refused():
    with pytest.raises(KeyError, match="unknown robot model"):
        robots.robot_model("d1-arms")


def test_the_rig_has_no_d1_body():
    for name in ("arm_ee_bench.urdf", "arm_ee_bench_visual.urdf"):
        root = ET.parse(str(description.ROOT / "d1" / name)).getroot()
        links = {link.get("name") for link in root.iter("link")}
        assert not links & {"torso_column", "head_link", "chassis_link",
                            "neck_pan_link", "base_footprint"}, name
        assert {"dual_base", "stand", "Base_R", "Base_L",
                "hand_envelope_R", "hand_envelope_L"} <= links
        joints = {j.get("name"): j for j in root.iter("joint")}
        assert "lift" not in joints and "neck_pan" not in joints


def test_the_rig_mounts_the_arms_where_the_d1_does():
    gen = _generator()
    for side in ("R", "L"):
        rig = ET.parse(str(description.ROOT / "d1" / "arm_ee_bench.urdf")).getroot()
        d1 = ET.parse(str(description.ROOT / "d1" / "d1.urdf")).getroot()

        def origin(root, name):
            j = next(j for j in root.iter("joint") if j.get("name") == name)
            o = j.find("origin")
            return o.get("xyz"), o.get("rpy")

        for name in [f"mount_{side}"] + [f"Joint{i}_{side}" for i in range(1, 8)] + [
                f"JointTCP_{side}"]:
            assert origin(rig, name) == origin(d1, name), name
    assert gen.ENVELOPES["wuji/wujihand2"] is gen.FIVE_FINGER_ENVELOPE


def test_the_rig_s_hand_envelope_is_the_catalog_s_five_finger_default():
    gen = _generator()
    assert tuple((p, tuple(a), tuple(b), r) for p, a, b, r in gen.FIVE_FINGER_ENVELOPE) \
        == FIVE_FINGER_DEFAULT.capsules


def test_the_rig_guard_has_no_body_and_keeps_the_arms_apart():
    guard = _rig_guard()
    assert guard.body_aabbs == [] and guard.body_cylinders == []
    assert guard.chest_keepout == {}
    assert guard.check(POSE_BODY_HIT, None).ok
    assert not guard.check(POSE_AA_A, POSE_AA_B).ok


def test_the_rig_exports_as_a_variant(tmp_path):
    from manipulation_kit.description.tools import export_description as ex
    from manipulation_kit.description.variants import resolve

    v = resolve(robots.robot_model(RIG).view_variant)
    manifest = ex.export(v.flavour, str(tmp_path))
    assert set(manifest["files"]) == {"arm_ee_bench_visual.urdf"}
