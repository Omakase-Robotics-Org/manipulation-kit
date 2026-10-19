"""Wuji Hand 2: the glove's radians 1:1 (``absolute``, the default) against the range map."""
import math
import textwrap

import pytest

from manipulation_kit.binding import BindingError, load_binding, parse_binding
from manipulation_kit.gloves import HandAngles, HandPose
from manipulation_kit.hands import get_retarget
from manipulation_kit.hands.wuji.wujihand2 import AXIS_NAMES, JOINT_LIMITS_RAD
from manipulation_kit.hands.wuji.wujihand2.retarget import (
    RetargetConfig,
    Retargeter,
    config_for,
)

FINGERS = ("index", "middle", "ring", "pinky")


def _i(name):
    return AXIS_NAMES.index(name)


def _angles(**kw):
    ch = {}
    for f in FINGERS:
        ch.update({f"{f}_mp_pitch": 0.0, f"{f}_pip_pitch": 0.0, f"{f}_dip_pitch": 0.0,
                   f"{f}_mp_swing": 0.0})
    ch.update(kw)
    return HandAngles("right", ch)


def test_absolute_is_the_default():
    assert RetargetConfig().mapping == "absolute"
    assert get_retarget("wuji/wujihand2").cfg.mapping == "absolute"


def test_fingers_follow_the_glove_one_to_one_in_radians():
    r = Retargeter()
    out = r.joint_targets(None, angles=_angles(
        index_mp_pitch=0.3, index_mp_swing=-0.11, index_pip_pitch=0.9, index_dip_pitch=0.5,
        pinky_mp_swing=0.2, ring_mp_pitch=1.2))
    assert out[_i("index_S1")] == 0.3 and out[_i("index_S2")] == -0.11
    assert out[_i("index_S3")] == 0.9 and out[_i("index_S4")] == 0.5
    # The pinky's spread is not cut to the range map's 0.45 rad interval.
    assert out[_i("pinky_S2")] == 0.2 and out[_i("ring_S1")] == 1.2
    assert out[_i("middle_S1")] == 0.0


def test_a_small_swing_stays_small_unlike_the_range_map():
    swing = math.radians(5)
    absolute = Retargeter().joint_targets(None, angles=_angles(index_mp_swing=swing))
    assert absolute[_i("index_S2")] == swing
    # The same 5 degrees through a 5 degree calibrated range: the range map
    # drives the joint to the top of its interval.
    ranged = Retargeter(RetargetConfig(mapping="range")).joint_targets(
        HandPose("right", {"index_mp_swing": 1.0}))
    assert ranged[_i("index_S2")] == pytest.approx(0.648)


def test_the_output_is_clamped_to_the_joints_limits():
    out = Retargeter().joint_targets(None, angles=_angles(
        index_mp_swing=1.0, middle_mp_swing=-1.0, ring_pip_pitch=3.0, pinky_mp_pitch=-3.0))
    assert out[_i("index_S2")] == JOINT_LIMITS_RAD[_i("index_S2")][1] == 0.698
    assert out[_i("middle_S2")] == JOINT_LIMITS_RAD[_i("middle_S2")][0] == -0.698
    assert out[_i("ring_S3")] == JOINT_LIMITS_RAD[_i("ring_S3")][1]
    assert out[_i("pinky_S1")] == JOINT_LIMITS_RAD[_i("pinky_S1")][0]


def test_zero_rad_is_subtracted_per_joint():
    zero = [0.0] * 20
    zero[_i("index_S2")] = 0.05
    zero[_i("index_S1")] = -0.1
    r = Retargeter(RetargetConfig(zero_rad=tuple(zero)))
    out = r.joint_targets(None, angles=_angles(index_mp_swing=0.12, index_mp_pitch=0.2))
    assert out[_i("index_S2")] == pytest.approx(0.07)
    assert out[_i("index_S1")] == pytest.approx(0.3)
    assert out[_i("middle_S2")] == 0.0


def test_a_side_whose_spread_is_reversed_inverts_that_joint():
    cfg = RetargetConfig()
    from dataclasses import replace
    joints = dict(cfg.joints)
    joints["index_S2"] = replace(joints["index_S2"], invert=True)
    out = Retargeter(RetargetConfig(joints=joints)).joint_targets(
        None, angles=_angles(index_mp_swing=0.1, middle_mp_swing=0.1))
    assert out[_i("index_S2")] == -0.1 and out[_i("middle_S2")] == 0.1


def test_a_missing_angle_holds_only_its_joint():
    angles = _angles()
    angles.radians["ring_pip_pitch"] = None
    angles.radians["middle_mp_swing"] = float("nan")
    out = Retargeter().joint_targets(None, angles=angles)
    assert out[_i("ring_S3")] is None and out[_i("middle_S2")] is None
    assert out[_i("ring_S1")] == 0.0 and out[_i("ring_S4")] == 0.0


def test_absolute_needs_radians_and_refuses_a_flexion_pose_in_their_place():
    r = Retargeter()
    with pytest.raises(ValueError, match="angles"):
        r.joint_targets(HandPose("right", {}))
    with pytest.raises(TypeError, match="radians"):
        r.joint_targets(None, angles=HandPose("right", {"index_mp_pitch": 0.5}))


def test_no_channel_range_is_read_in_absolute_mode():
    # The flexion pose carries no finger channel at all, and nothing is asked of it.
    class Strict:
        def get(self, channel, default=None):
            raise AssertionError(f"asked the flexion pose for {channel}")
    out = Retargeter().joint_targets(Strict(), angles=_angles(index_mp_pitch=0.4))
    assert out[_i("index_S1")] == 0.4
    assert out[:4] == [0.0] * 4  # the thumb is held


def test_the_thumb_is_unchanged_and_still_follows_the_flexion_pose():
    declaration_cfg = RetargetConfig(hold=(False,) * 4 + (False,) * 16)
    pose = HandPose("right", {"thumb_cm_yaw": 1.0, "thumb_mp_pitch": 0.5})
    absolute = Retargeter(declaration_cfg).joint_targets(pose, angles=_angles())
    ranged = Retargeter(RetargetConfig(hold=declaration_cfg.hold, mapping="range")
                        ).joint_targets(HandPose("right", {
                            **pose.flex, **{f"{f}_{c}": 0.0 for f in FINGERS for c in
                                            ("mp_pitch", "pip_pitch", "dip_pitch")},
                            **{f"{f}_mp_swing": 0.5 for f in FINGERS}}))
    assert absolute[:4] == ranged[:4] == [0.8, 0.6, 0.5, 0.3]


def test_smoothing_runs_once_over_the_whole_row():
    r = Retargeter(RetargetConfig(ema_alpha=0.5))
    r.joint_targets(None, angles=_angles(index_mp_pitch=0.0))
    out = r.joint_targets(None, angles=_angles(index_mp_pitch=1.0))
    assert out[_i("index_S1")] == pytest.approx(0.5)


def test_the_range_mapping_is_the_table_it_was():
    r = Retargeter(RetargetConfig(mapping="range"))
    out = r.joint_targets(HandPose("right", {
        **{f"{f}_{c}": 0.5 for f in FINGERS for c in
           ("mp_pitch", "pip_pitch", "dip_pitch", "mp_swing")}}))
    assert out[_i("index_S1")] == pytest.approx(0.785)
    assert out[_i("index_S2")] == pytest.approx(0.0)
    assert out[_i("pinky_S2")] == pytest.approx(0.0)
    assert out[_i("index_S3")] == pytest.approx(1.047)
    # The version-1 seam is the range map in either mode.
    assert list(get_retarget("wuji/wujihand2")(lambda c: 0.5)) == pytest.approx(list(
        Retargeter(RetargetConfig(mapping="range"))(lambda c: 0.5)))


def test_mapping_and_zero_rad_are_checked():
    with pytest.raises(ValueError, match="mapping must be one of"):
        RetargetConfig(mapping="relative")
    with pytest.raises(ValueError, match="zero_rad needs 20"):
        RetargetConfig(zero_rad=(0.0,) * 3)
    with pytest.raises(ValueError, match="finite"):
        RetargetConfig(zero_rad=(float("nan"),) + (0.0,) * 19)


def _bind(body):
    return textwrap.dedent('''
        schema = "omakase.binding.v1"
        [[source]]
        device = "glove"
        family = "teleop_gloves.pose_stream.v1"
        [[bind]]
        from = "glove"
        to = "hand/right"
        retarget = "wuji/wujihand2"
    ''') + textwrap.dedent(body)


def test_a_binding_picks_the_mapping(tmp_path):
    p = tmp_path / "b.toml"
    p.write_text(_bind(""))
    assert load_binding(p).binds[0].build_retarget().cfg.mapping == "absolute"
    p.write_text(_bind('[bind.retarget_config]\nmapping = "range"\n'))
    assert load_binding(p).binds[0].build_retarget().cfg.mapping == "range"
    p.write_text(_bind('[bind.retarget_config]\nmapping = "range"\n'
                       "zero_rad = [" + ", ".join(["0.0"] * 20) + "]\n"))
    assert load_binding(p).binds[0].retarget_config.zero_rad == (0.0,) * 20


def test_a_binding_with_a_bad_mapping_is_refused_at_load(tmp_path):
    p = tmp_path / "b.toml"
    p.write_text(_bind('[bind.retarget_config]\nmapping = "mirror"\n'))
    with pytest.raises(BindingError) as err:
        load_binding(p)
    assert "mapping must be one of ('absolute', 'range'), got 'mirror'" in str(err.value)
    p.write_text(_bind("[bind.retarget_config]\nmapping = 3\n"))
    with pytest.raises(BindingError, match="expected a string"):
        load_binding(p)
