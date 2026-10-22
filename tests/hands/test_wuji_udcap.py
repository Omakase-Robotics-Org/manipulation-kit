"""A UDCAP glove driving the Wuji Hand 2 1:1: the config, a real capture, the example binding."""
import json
from dataclasses import replace
from pathlib import Path

import pytest

from manipulation_kit.binding import load_binding
from manipulation_kit.gloves import HandAngles
from manipulation_kit.gloves.v1 import DeclarationError, parse_declaration
from manipulation_kit.hands.wuji.wujihand2 import AXIS_NAMES, JOINT_LIMITS_RAD
from manipulation_kit.hands.wuji.wujihand2.retarget import (
    RetargetConfig,
    Retargeter,
)
from manipulation_kit.hands.wuji.wujihand2.udcap import THUMB_SOURCES, udcap_config

ROOT = Path(__file__).resolve().parents[2]
CAPTURE = ROOT / "tests" / "data" / "gloves" / "udcap_right_range.jsonl"
EXAMPLE = ROOT / "docs" / "binding-examples" / "udcap-wuji-right.toml"
FINGERS = ("index", "middle", "ring", "pinky")


def _i(name):
    return AXIS_NAMES.index(name)


def _angles(**kw):
    ch = {}
    for f in FINGERS:
        ch.update({f"{f}_mp_pitch": 0.0, f"{f}_pip_pitch": 0.0, f"{f}_dip_pitch": 0.0})
    ch.update({"thumb_cm_pitch": 0.0, "thumb_cm_yaw": 0.0, "thumb_mp_pitch": 0.0,
               "thumb_ip_pitch": 0.0, "index_mp_swing": 0.0, "ring_mp_swing": 0.0,
               "pinky_mp_swing": 0.0})
    ch.update(kw)
    return HandAngles("right", ch)


def _clamp(joint, v):
    lo, hi = JOINT_LIMITS_RAD[_i(joint)]
    return min(hi, max(lo, v))


def test_the_config_holds_only_the_middle_spread_and_drives_the_thumb():
    cfg = udcap_config()
    assert cfg.mapping == "absolute" and cfg.thumb_absolute
    assert [n for n, h in zip(AXIS_NAMES, cfg.hold) if h] == ["middle_S2"]
    assert {j: cfg.joints[j].sources[0][0] for j in THUMB_SOURCES} == THUMB_SOURCES
    assert not any(ch.endswith("middle_mp_swing") or ch == "thumb_cm_roll"
                   for ch in Retargeter(cfg).required_channels())


def test_every_joint_follows_the_glove_one_to_one_and_the_middle_spread_is_zero():
    r = Retargeter(udcap_config())
    out = r.joint_targets(None, angles=_angles(
        index_mp_pitch=0.4, index_pip_pitch=0.9, index_dip_pitch=0.3,
        index_mp_swing=-0.2, ring_mp_swing=0.1, pinky_mp_swing=0.3,
        thumb_cm_pitch=0.5, thumb_cm_yaw=0.4, thumb_mp_pitch=0.6, thumb_ip_pitch=0.2))
    assert out[_i("index_S1")] == 0.4 and out[_i("index_S3")] == 0.9
    assert out[_i("index_S4")] == 0.3
    assert out[_i("index_S2")] == -0.2 and out[_i("ring_S2")] == 0.1
    assert out[_i("pinky_S2")] == 0.3 and out[_i("middle_S2")] == 0.0
    assert [out[_i(f"thumb_S{n}")] for n in (1, 2, 3, 4)] == [0.5, 0.4, 0.6, 0.2]


def test_the_thumb_is_clamped_to_the_wujis_limits():
    out = Retargeter(udcap_config()).joint_targets(None, angles=_angles(
        thumb_cm_pitch=1.4, thumb_cm_yaw=0.96, thumb_mp_pitch=1.8, thumb_ip_pitch=-1.5))
    assert out[_i("thumb_S1")] == _clamp("thumb_S1", 1.4) == 1.291
    assert out[_i("thumb_S2")] == _clamp("thumb_S2", 0.96) == 0.698
    assert out[_i("thumb_S3")] == 1.57 and out[_i("thumb_S4")] == -1.047


def test_a_hand_whose_spread_or_thumb_reads_the_other_way_inverts_that_joint():
    cfg = udcap_config()
    joints = dict(cfg.joints)
    for j in ("index_S2", "thumb_S2"):
        joints[j] = replace(joints[j], invert=True)
    out = Retargeter(udcap_config(joints=joints)).joint_targets(
        None, angles=_angles(index_mp_swing=-0.2, ring_mp_swing=0.1, thumb_cm_yaw=0.3))
    assert out[_i("index_S2")] == 0.2 and out[_i("ring_S2")] == 0.1
    assert out[_i("thumb_S2")] == -0.3


def test_a_missing_thumb_angle_holds_only_that_joint():
    angles = _angles()
    angles.radians["thumb_cm_yaw"] = None
    out = Retargeter(udcap_config()).joint_targets(None, angles=angles)
    assert out[_i("thumb_S2")] is None and out[_i("thumb_S1")] == 0.0


def test_the_default_thumb_still_reads_the_flexion_pose():
    # Not thumb_absolute: the thumb is held at 0 by default, as before.
    out = Retargeter(RetargetConfig()).joint_targets(None, angles=_angles())
    assert out[:4] == [0.0] * 4


def test_a_real_right_hand_capture_stays_inside_the_limits_and_follows_the_glove():
    rows = [json.loads(line) for line in CAPTURE.read_text().splitlines()]
    assert len(rows) == 299
    r = Retargeter(udcap_config())
    seen = {n: [] for n in AXIS_NAMES}
    for row in rows:
        angles = HandAngles("right", row["radians"])
        out = r.joint_targets(None, angles=angles)
        for name, v in zip(AXIS_NAMES, out):
            if name == "middle_S2":
                assert v == 0.0
                continue
            if name.endswith("_S2") and not name.startswith("thumb"):
                # This capture predates the driver's spread channels.
                assert v is None
                continue
            lo, hi = JOINT_LIMITS_RAD[_i(name)]
            assert v is not None and lo <= v <= hi, (name, v)
            seen[name].append(v)
        for f in FINGERS:
            for n, ch in ((1, "mp"), (3, "pip"), (4, "dip")):
                want = _clamp(f"{f}_S{n}", row["radians"][f"{f}_{ch}_pitch"])
                assert out[_i(f"{f}_S{n}")] == want
        assert out[_i("thumb_S3")] == _clamp("thumb_S3", row["radians"]["thumb_mp_pitch"])
        assert out[_i("thumb_S4")] == _clamp("thumb_S4", row["radians"]["thumb_ip_pitch"])
    # The capture moves every finger and the thumb through their range.
    assert max(seen["index_S3"]) > 1.5 and max(seen["thumb_S1"]) > 1.0


def _udcap_declaration(with_synth_middle=True):
    chans = {}
    for f in FINGERS:
        for j in ("mp_pitch", "pip_pitch", "dip_pitch"):
            chans[f"{f}_{j}"] = dict(kind="flex", provenance="measured", note="x",
                                     default_range=[0.0, 1.5])
    for f in ("index", "ring", "pinky"):
        chans[f"{f}_mp_swing"] = dict(kind="swing", provenance="measured", note="x",
                                      default_range=[-0.4, 0.4])
    chans["middle_mp_swing"] = dict(kind="swing", provenance="synthesized", note="x",
                                    default_range=[-0.2, 0.2],
                                    derived_from=["index_mp_swing", "ring_mp_swing"])
    for c, rng in (("thumb_cm_pitch", 1.4), ("thumb_mp_pitch", 1.0), ("thumb_ip_pitch", 1.0)):
        chans[c] = dict(kind="flex", provenance="measured", note="x", default_range=[0.0, rng])
    chans["thumb_cm_yaw"] = dict(kind="yaw", provenance="measured", note="x",
                                 default_range=[0.0, 0.96])
    return parse_declaration({"type": "channel_declaration", "device_model": "test/udcap",
                              "channels": chans})


def test_the_udcap_config_binds_where_the_default_map_is_refused():
    declaration = _udcap_declaration()
    declaration.check_drive_sources(Retargeter(udcap_config()).required_channels())
    default = Retargeter(RetargetConfig(hold=(False,) * 20))
    with pytest.raises(DeclarationError, match="middle_mp_swing"):
        declaration.check_drive_sources(default.required_channels())


def test_the_example_binding_is_the_udcap_config():
    bind = load_binding(EXAMPLE).binds[0]
    built = bind.build_retarget().cfg
    want = udcap_config()
    assert built.mapping == want.mapping and built.thumb_absolute
    assert built.hold == want.hold and built.hold_rad == want.hold_rad
    assert {j: m.sources for j, m in built.joints.items()} == {
        j: m.sources for j, m in want.joints.items()}
