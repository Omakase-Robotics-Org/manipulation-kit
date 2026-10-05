"""Retarget contract v2: required_channels() and joint_targets(pose), for the
DH116S (beside its unchanged v1 call) and the XHAND1, plus the shared joint
helpers."""
import math
import random

import numpy as np
import pytest

import manipulation_kit.hands as hands
from manipulation_kit.gloves import HandPose
from manipulation_kit.gloves.channels import spec_of
from manipulation_kit.hands.joints import JointSpec, check_descriptor, fill_held
from manipulation_kit.hands.leadshine.dh116s.retarget import (
    CHANNELS as DH_CHANNELS,
    JOINTS as DH_JOINTS,
    POS_MAX,
    RetargetConfig as DhConfig,
    Retargeter as DhRetargeter,
)
from manipulation_kit.hands.robotera.xhand1 import AXIS_NAMES as XH_NAMES
from manipulation_kit.hands.robotera.xhand1 import JOINT_LIMITS_RAD
from manipulation_kit.hands.robotera.xhand1.retarget import (
    JointMap,
    RetargetConfig as XhConfig,
    Retargeter as XhRetargeter,
    default_joint_maps,
)


def _pose(values, side="right"):
    return HandPose(side, dict(values))


def _full(value, channels):
    return _pose({c: value for c in channels})


# ------------------------------------------------------------------ DH116S --
def test_dh116s_joints_are_the_daemon_descriptor():
    """The daemon's DH116S descriptor: AXIS_NAMES, fraction, 0..1."""
    assert [j.name for j in DH_JOINTS] == ["thumb_swing", "thumb_flex", "index_flex",
                                           "middle_flex", "ring_flex", "pinky_flex"]
    assert all(j.unit == "fraction" and (j.min, j.max) == (0.0, 1.0) for j in DH_JOINTS)


def test_dh116s_default_required_channels_skip_zero_weights():
    req = DhRetargeter().required_channels()
    assert "thumb_cm_roll" not in req                      # blend 0
    assert not any(c.endswith("_dip_pitch") for c in req)  # w_dip 0
    assert set(req) == {"thumb_cm_yaw", "thumb_cm_pitch", "thumb_mp_pitch"} | {
        f"{f}_{j}_pitch" for f in ("index", "middle", "ring", "pinky") for j in ("mp", "pip")}
    assert all(spec_of(c) is not None for c in req)


def test_dh116s_blend_channel_is_required_only_when_weighted():
    cfg = DhConfig(thumb_swing_roll_blend=0.3, thumb_swing_blend_channel="thumb_cm_swing")
    req = DhRetargeter(cfg).required_channels()
    assert "thumb_cm_swing" in req and "thumb_cm_roll" not in req
    cfg = DhConfig(thumb_swing_roll_blend=1.0)
    assert "thumb_cm_yaw" not in DhRetargeter(cfg).required_channels()


@pytest.mark.parametrize("seed", range(20))
def test_dh116s_v2_agrees_with_v1_when_every_channel_is_present(seed):
    rng = random.Random(seed)
    cfg = DhConfig(w_dip=rng.choice([0.0, 0.2]),
                   thumb_swing_roll_blend=rng.choice([0.0, 0.4]),
                   invert=tuple(rng.random() < 0.3 for _ in range(6)),
                   out_lo=tuple(rng.choice([0, 1000]) for _ in range(6)),
                   out_hi=tuple(rng.choice([POS_MAX, 8000]) for _ in range(6)))
    values = {c: rng.random() for c in DH_CHANNELS}
    v1 = DhRetargeter(cfg)(lambda c: values[c])
    v2 = DhRetargeter(cfg).joint_targets(_pose(values))
    assert np.allclose(np.array(v2) * POS_MAX, v1, atol=0.5 + 1e-9)


def test_dh116s_v2_agrees_with_v1_under_ema():
    cfg = DhConfig(ema_alpha=0.3)
    a, b = DhRetargeter(cfg), DhRetargeter(cfg)
    for k in range(10):
        values = {c: (k % 3) / 2 for c in DH_CHANNELS}
        v1 = a(lambda c: values[c])
        v2 = b.joint_targets(_pose(values))
    assert np.allclose(np.array(v2) * POS_MAX, v1, atol=0.5 + 1e-9)


def test_dh116s_a_missing_input_holds_only_its_axis():
    values = {c: 1.0 for c in DH_CHANNELS}
    values["ring_pip_pitch"] = None
    out = DhRetargeter().joint_targets(_pose(values))
    assert out == [1.0, 1.0, 1.0, 1.0, None, 1.0]
    # an absent key is the same as None, and a zero-weight channel never matters
    values = {c: 0.0 for c in DhRetargeter().required_channels()}
    assert DhRetargeter().joint_targets(_pose(values)) == [0.0] * 6


def test_dh116s_hold_does_not_drag_the_ema():
    r = DhRetargeter(DhConfig(ema_alpha=0.5))
    present = {c: 1.0 for c in DH_CHANNELS}
    gone = dict(present, index_mp_pitch=None)
    assert r.joint_targets(_pose(gone))[2] is None          # seeds the rest
    assert r.joint_targets(_pose(present))[2] == 1.0         # first value, not an average
    assert r.joint_targets(_pose({c: 0.0 for c in DH_CHANNELS}))[2] == pytest.approx(0.5)


def test_dh116s_v1_call_and_channels_are_unchanged():
    """Consumers of the version-1 seam see the same names and numbers."""
    r = hands.get_retarget("leadshine/dh116s")
    assert r(lambda c: 1.0).tolist() == [POS_MAX] * 6
    assert len(DH_CHANNELS) == 16 and "thumb_cm_roll" in DH_CHANNELS
    assert DhConfig().thumb_swing_blend_channel == "thumb_cm_roll"


# ------------------------------------------------------------------- XHAND1 --
def test_xhand1_joints_are_the_daemon_table():
    """The hand driver's XHAND1 descriptor: names, rad, the manual's degrees."""
    assert XH_NAMES == (
        "thumb_bend", "thumb_rota1", "thumb_rota2",
        "index_bend", "index_j1", "index_j2", "mid_j1",
        "mid_j2", "ring_j1", "ring_j2", "pinky_j1", "pinky_j2")
    assert JOINT_LIMITS_RAD[1] == pytest.approx((-1.0471975511965976, 1.5707963267948966))
    assert JOINT_LIMITS_RAD[3] == pytest.approx((math.radians(-5), math.radians(17)))
    r = hands.get_retarget("robotera/xhand1")
    assert [j.name for j in r.JOINTS] == list(XH_NAMES)
    assert all(j.unit == "rad" for j in r.JOINTS)


def test_xhand1_open_hand_is_the_open_preset_and_a_fist_is_full_flexion():
    r = XhRetargeter()
    req = r.required_channels()
    open_ = dict.fromkeys(req, 0.0)
    open_["index_mp_swing"] = 0.5                           # neutral spread
    assert r.joint_targets(_pose(open_)) == pytest.approx([0.0] * 12)
    fist = dict(open_, **{c: 1.0 for c in req if c != "index_mp_swing"})
    out = r.joint_targets(_pose(fist))
    assert out[0] == out[1] == out[2] == pytest.approx(math.radians(90))
    assert out[3] == pytest.approx(0.0)
    assert out[4:] == pytest.approx([math.radians(110)] * 8)


def test_xhand1_targets_stay_inside_the_joint_limits_for_any_flexion():
    r = XhRetargeter()
    req = r.required_channels()
    for v in (0.0, 0.25, 0.5, 1.0):
        out = r.joint_targets(_full(v, req))
        for t, (lo, hi) in zip(out, JOINT_LIMITS_RAD):
            assert lo <= t <= hi


def test_xhand1_missing_inputs_hold_their_joints_only():
    r = XhRetargeter()
    values = {c: 0.5 for c in r.required_channels()}
    values["index_mp_swing"] = None             # UDCAP declares spreads uncalibrated
    values["thumb_cm_pitch"] = None             # LitchiBot declares this uncalibrated
    out = r.joint_targets(_pose(values))
    assert [XH_NAMES[i] for i, t in enumerate(out) if t is None] == [
        "thumb_rota1", "index_bend"]


def test_xhand1_config_refuses_targets_outside_the_hand():
    maps = default_joint_maps()
    maps["index_bend"] = JointMap((("index_mp_swing", 1.0),), -0.3, 0.3)
    with pytest.raises(ValueError, match="leaves the joint's limits"):
        XhConfig(joints=maps)
    maps = default_joint_maps()
    del maps["pinky_j2"]
    with pytest.raises(ValueError, match="missing joints"):
        XhConfig(joints=maps)
    maps = default_joint_maps()
    maps["pinky_j2"] = JointMap((("pinky_pip_pitch", 0.0),), 0.0, 1.0)
    with pytest.raises(ValueError, match="positive weight"):
        XhConfig(joints=maps)


def test_xhand1_invert_and_blend():
    maps = default_joint_maps()
    maps["index_j2"] = JointMap(
        (("index_pip_pitch", 3.0), ("index_dip_pitch", 1.0)), 0.0, math.radians(110), invert=True)
    r = XhRetargeter(XhConfig(joints=maps))
    assert "index_dip_pitch" in r.required_channels()
    values = {c: 0.0 for c in r.required_channels()}
    values.update(index_pip_pitch=1.0, index_dip_pitch=0.0, index_mp_swing=0.5)
    assert r.joint_targets(_pose(values))[5] == pytest.approx(math.radians(110) * 0.25)


def test_xhand1_v1_call_returns_twelve_radians():
    r = hands.get_retarget("robotera/xhand1")
    out = r(lambda c: 0.5)
    assert out.shape == (12,) and np.all(np.isfinite(out))


def test_xhand1_ema_smooths_present_joints():
    r = XhRetargeter(XhConfig(ema_alpha=0.5))
    req = r.required_channels()
    r.joint_targets(_full(0.0, req))
    assert r.joint_targets(_full(1.0, req))[4] == pytest.approx(math.radians(110) / 2)


# --------------------------------------------------------- joint helpers --
def test_check_descriptor_accepts_the_live_shape_and_names_each_difference():
    live = [{"name": j.name, "unit": j.unit, "min": j.min, "max": j.max}
            for j in DH_JOINTS]
    check_descriptor(DH_JOINTS, live)
    live[1] = dict(live[1], unit="rad")
    live[2] = dict(live[2], max=0.9)
    with pytest.raises(ValueError) as err:
        check_descriptor(DH_JOINTS, live)
    assert "thumb_flex" in str(err.value) and "index_flex" in str(err.value)
    with pytest.raises(ValueError, match="declares 5 joints"):
        check_descriptor(DH_JOINTS, live[:5])


def test_check_descriptor_accepts_float32_limits_and_nothing_coarser():
    """The driver computes XHAND1 limits in float32 (90 deg = 1.5707964)."""
    import numpy as np
    from manipulation_kit.hands.robotera.xhand1 import JOINTS as XH
    live = [{"name": j.name, "unit": j.unit, "min": float(np.float32(j.min)),
             "max": float(np.float32(j.max))} for j in XH]
    check_descriptor(XH, live)
    live[0] = dict(live[0], max=1.57)          # the manual's rounded column
    with pytest.raises(ValueError, match="thumb_bend"):
        check_descriptor(XH, live)


def test_fill_held_uses_the_last_command_and_refuses_with_none_to_hold():
    assert fill_held([0.1, None], [0.0, 0.7]) == [0.1, 0.7]
    assert fill_held([0.1, None], [0.0, None]) is None
    assert JointSpec("j", "rad", -1.0, 1.0).clamp(3.0) == 1.0


def test_every_v2_hand_answers_the_protocol():
    for model in ("leadshine/dh116s", "robotera/xhand1", "robotera/xhand1_lite"):
        r = hands.get_retarget(model)
        req = r.required_channels()
        assert req and all(spec_of(c) for c in req)
        assert len(r.joint_targets(_full(0.5, req))) == len(r.JOINTS)
        assert r.joint_targets(_pose({})) == [None] * len(r.JOINTS)
