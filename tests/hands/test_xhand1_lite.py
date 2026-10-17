"""XHAND1 Lite: the six-joint table and its glove retarget map (contract v2)."""
import pytest

import manipulation_kit.hands as hands
from manipulation_kit.gloves import HandPose
from manipulation_kit.gloves.channels import spec_of
from manipulation_kit.hands.robotera.xhand1.retarget import RetargetConfig as XhConfig
from manipulation_kit.hands.robotera.xhand1_lite import AXIS_NAMES, JOINTS
from manipulation_kit.hands.robotera.xhand1_lite.retarget import (
    JointMap,
    RetargetConfig,
    Retargeter,
    default_joint_maps,
)


def _pose(values):
    return HandPose("right", dict(values))


def test_the_six_joints_in_vendor_sdk_order():
    assert AXIS_NAMES == ("thumb_bend", "thumb_rota1", "index_j1",
                          "mid_j1", "ring_j1", "pinky_j1")
    assert [j.name for j in JOINTS] == list(AXIS_NAMES)
    assert all(j.unit == "rad" and j.min < j.max for j in JOINTS)
    r = hands.get_retarget("robotera/xhand1_lite")
    assert r.JOINTS == JOINTS


def test_fingers_read_mp_and_pip_like_the_dh116s():
    r = Retargeter()
    assert set(r.required_channels()) == {"thumb_cm_yaw", "thumb_cm_pitch",
                                          "thumb_mp_pitch"} | {
        f"{f}_{j}_pitch" for f in ("index", "middle", "ring", "pinky") for j in ("mp", "pip")}
    assert all(spec_of(c) for c in r.required_channels())
    values = {c: 0.0 for c in r.required_channels()}
    values.update(middle_mp_pitch=1.0)                 # half of the middle mean
    out = r.joint_targets(_pose(values))
    mid = JOINTS[3]
    lo = default_joint_maps()["mid_j1"].lo
    assert out[3] == pytest.approx(lo + 0.5 * (mid.max - lo))
    assert out[2] == out[4] == out[5] == pytest.approx(default_joint_maps()["index_j1"].lo)


def test_open_and_closed_hand_land_on_the_interval_ends_inside_the_limits():
    r = Retargeter()
    maps = default_joint_maps()
    for v, end in ((0.0, "lo"), (1.0, "hi")):
        out = r.joint_targets(_pose({c: v for c in r.required_channels()}))
        for t, j in zip(out, JOINTS):
            assert t == pytest.approx(getattr(maps[j.name], end))
            assert j.min <= t <= j.max


def test_the_open_end_is_zero_radians_where_the_joint_allows_it():
    for j in JOINTS:
        lo = default_joint_maps()[j.name].lo
        assert lo == pytest.approx(min(max(0.0, j.min), j.max))


def test_a_missing_input_holds_only_its_joint():
    r = Retargeter()
    values = {c: 0.5 for c in r.required_channels()}
    values["thumb_cm_pitch"] = None        # the LitchiBot glove: uncalibrated
    values["ring_pip_pitch"] = None        # a silent distal sensor
    out = r.joint_targets(_pose(values))
    assert [AXIS_NAMES[i] for i, t in enumerate(out) if t is None] == [
        "thumb_rota1", "ring_j1"]
    assert r.joint_targets(_pose({})) == [None] * 6


def test_the_thumb_can_be_resourced_without_code():
    maps = default_joint_maps()
    maps["thumb_rota1"] = JointMap((("thumb_mp_pitch", 1.0),),
                                         maps["thumb_rota1"].lo,
                                         maps["thumb_rota1"].hi)
    r = Retargeter(RetargetConfig(joints=maps))
    assert "thumb_cm_pitch" not in r.required_channels()


def test_configs_are_refused_across_hands_and_outside_limits():
    with pytest.raises(ValueError, match="another hand"):
        Retargeter(XhConfig())
    maps = default_joint_maps()
    j = JOINTS[2]
    maps[j.name] = JointMap((("index_mp_pitch", 1.0),), j.min, j.max + 0.5)
    with pytest.raises(ValueError, match="leaves the joint's limits"):
        RetargetConfig(joints=maps)
    with pytest.raises(ValueError, match="missing joints"):
        RetargetConfig(joints={k: v for k, v in default_joint_maps().items()
                               if k != "pinky_j1"})


def test_the_v1_call_returns_six_radians():
    out = hands.get_retarget("robotera/xhand1_lite")(lambda c: 0.5)
    assert out.shape == (6,)
