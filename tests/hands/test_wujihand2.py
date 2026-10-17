"""Wuji Hand 2: the twenty-joint table and its glove retarget map (contract v2)."""
import json
import math
import os
import random
import textwrap
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

import manipulation_kit.hands as hands
from manipulation_kit.binding import load_binding
from manipulation_kit.gloves import HandPose
from manipulation_kit.gloves.calibration import ChannelCalibration, ChannelRange, load_ranges
from manipulation_kit.gloves.channels import spec_of
from manipulation_kit.gloves.pose import HandAngles
from manipulation_kit.gloves.v1 import DeclarationError, PoseStreamDecoder, parse_declaration
from manipulation_kit.hands.joints import check_descriptor, fill_held
from manipulation_kit.hands.robotera.xhand1.retarget import RetargetConfig as XhConfig
from manipulation_kit.hands.wuji.wujihand2 import AXIS_NAMES, JOINT_LIMITS_RAD, JOINTS
from manipulation_kit.hands.wuji.wujihand2.retarget import (
    DEFAULT_HOLD,
    PINKY_SPREAD_RAD,
    SPREAD_RAD,
    JointMap,
    RetargetConfig,
    Retargeter,
    config_for,
    default_joint_maps,
)

DATA = Path(__file__).resolve().parents[1] / "data" / "gloves"
FINGERS = ("index", "middle", "ring", "pinky")
S2 = [AXIS_NAMES.index(f"{f}_S2") for f in FINGERS]


def _pose(values):
    return HandPose("right", dict(values))


def _hold_thumb_and_spread():
    return [True] * 4 + [i in S2 for i in range(4, 20)]


# The glove-to-Wuji table that drove a right Wuji Hand 2 on 2026-09-25, row by
# row: (joint, glove field, weight, input range in degrees, output range in
# rad). It read finger_angles_rad.<finger>.<field> in radians, took the
# weighted sum in degrees, clamped it to the input range and mapped it
# linearly onto the output range; the four thumb joints were held at 0.
_TABLE = {}
for _f in FINGERS:
    _TABLE[f"{_f}_S1"] = ("mp_pitch", 1.5 if _f == "index" else 1.0, (0.0, 75.0), (0.0, 1.57))
    _TABLE[f"{_f}_S2"] = ("mp_swing", 1.0, (-18.564, 18.564), (-0.648, 0.648))
    _TABLE[f"{_f}_S3"] = ("pip_pitch", 1.0, (0.0, 95.0), (0.0, 2.094))
    _TABLE[f"{_f}_S4"] = ("dip_pitch", 1.0, (0.0, 80.0), (0.0, 1.57))


def _table(radians):
    out = []
    for name in AXIS_NAMES:
        if name.startswith("thumb_"):
            out.append(0.0)
            continue
        field, weight, (lo, hi), (olo, ohi) = _TABLE[name]
        total = weight * math.degrees(radians[f"{name.split('_')[0]}_{field}"])
        total = min(max(total, lo), hi)
        out.append(olo + (total - lo) / (hi - lo) * (ohi - olo))
    return out


# --------------------------------------------------------------- the table --
def test_the_twenty_joints_in_sdk_order():
    assert AXIS_NAMES[:4] == ("thumb_S1", "thumb_S2", "thumb_S3", "thumb_S4")
    assert AXIS_NAMES[-4:] == ("pinky_S1", "pinky_S2", "pinky_S3", "pinky_S4")
    assert len(AXIS_NAMES) == len(set(AXIS_NAMES)) == 20
    assert [j.name for j in JOINTS] == list(AXIS_NAMES)
    assert all(j.unit == "rad" and j.min < 0.0 < j.max for j in JOINTS)
    assert hands.get_retarget("wuji/wujihand2").JOINTS == JOINTS


def test_the_table_is_its_own_descriptor():
    """A provider builds its descriptor from JOINTS; check_descriptor
    accepts exactly that and refuses a moved limit."""
    live = [{"name": j.name, "unit": j.unit, "min": j.min, "max": j.max} for j in JOINTS]
    check_descriptor(JOINTS, live)
    live[6]["max"] = 2.0
    with pytest.raises(ValueError, match="index_S3"):
        check_descriptor(JOINTS, live)


def _urdf_limits(path: Path):
    joints = [j for j in ET.parse(path).getroot().findall("joint") if j.get("type") == "revolute"]
    return [(j.get("name"), float(j.find("limit").get("lower")),
             float(j.find("limit").get("upper"))) for j in joints]


def test_the_table_matches_both_vendor_urdfs():
    root = os.environ.get("MKIT_WUJI_URDF_DIR")
    if not root or not (Path(root) / "right.urdf").is_file():
        pytest.skip("no $MKIT_WUJI_URDF_DIR/{right,left}.urdf: the vendor URDFs are "
                    "not part of this repository")
    suffixes = ["cmc_flex", "cmc_abd", "mcp", "ip"] + ["mcp_flex", "mcp_abd", "pip", "dip"] * 4
    for side in ("right", "left"):
        urdf = _urdf_limits(Path(root) / f"{side}.urdf")
        assert len(urdf) == 20
        for i, ((name, lo, hi), suffix) in enumerate(zip(urdf, suffixes)):
            finger = AXIS_NAMES[i].split("_")[0]
            assert name.startswith(f"{side[0]}_{finger}") and name.endswith(suffix), name
            assert (lo, hi) == JOINT_LIMITS_RAD[i], (side, name)


# ------------------------------------------------------------- the map --
def test_the_defaults_follow_every_finger_joint_and_hold_the_thumb():
    r = Retargeter()
    assert DEFAULT_HOLD == (True,) * 4 + (False,) * 16
    assert set(r.required_channels()) == {
        f"{f}_{c}" for f in FINGERS for c in ("mp_pitch", "mp_swing", "pip_pitch", "dip_pitch")}
    assert all(spec_of(c) for c in r.required_channels())
    assert r.JOINTS == JOINTS


def test_open_neutral_and_closed():
    r = Retargeter()
    channels = r.required_channels()
    open_ = r.joint_targets(_pose({c: 0.0 for c in channels}))
    closed = r.joint_targets(_pose({c: 1.0 for c in channels}))
    neutral = r.joint_targets(_pose({c: 0.5 for c in channels}))
    for i, j in enumerate(JOINTS):
        if j.name.startswith("thumb_"):
            assert open_[i] == closed[i] == 0.0
        elif j.name.endswith("_S2"):
            spread = PINKY_SPREAD_RAD if j.name == "pinky_S2" else SPREAD_RAD
            assert (open_[i], neutral[i], closed[i]) == pytest.approx(
                (-spread, 0.0, spread))
            assert j.min < -spread and spread < j.max
        else:
            assert (open_[i], closed[i]) == pytest.approx((0.0, j.max))


def test_the_calibration_and_the_map_reproduce_the_table_that_ran():
    # The file holds the table's flexion; its spread rows (S2 = 2 x swing,
    # saturating at +-18.564 degrees) are a +-0.324 rad swing range, given
    # here the way a glove declares it.
    ranges = load_ranges(DATA / "litchibot_wujihand2.calibration.json")
    assert not [c for c in ranges if c.endswith("_mp_swing")]
    swing = {f"{f}_mp_swing": ChannelRange(-0.3240029223402273, 0.3240029223402273)
             for f in FINGERS}
    calibration = ChannelCalibration(swing, ranges, known={**swing, **ranges})
    r = Retargeter(RetargetConfig())
    rng = random.Random(20260925)
    for _ in range(2000):
        radians = {}
        for f in FINGERS:
            for c, (lo, hi) in (("mp_pitch", (-0.3, 1.8)), ("pip_pitch", (-0.3, 2.0)),
                                ("dip_pitch", (-0.3, 1.7)), ("mp_swing", (-0.6, 0.6))):
                radians[f"{f}_{c}"] = rng.uniform(lo, hi)
        pose = calibration.normalize_hand(HandAngles("right", radians))
        expected = _table(radians)
        p = AXIS_NAMES.index("pinky_S2")
        expected[p] *= PINKY_SPREAD_RAD / SPREAD_RAD   # the pinky's narrower interval
        assert r.joint_targets(pose) == pytest.approx(expected, abs=1e-9)


def test_the_declared_spread_is_scaled_once_and_reaches_the_pinky_interval():
    # A glove that declares the wearer's spread (+-0.182 rad on the pinky)
    # and an operator file without swing ranges: the wearer's full spread is
    # the joint's full interval.
    ranges = load_ranges(DATA / "litchibot_wujihand2.calibration.json")
    declared = {f"{f}_mp_swing": ChannelRange(-0.182, 0.182) for f in FINGERS}
    calibration = ChannelCalibration(declared, ranges, known={**declared, **ranges})
    r = Retargeter(RetargetConfig())
    radians = {f"{f}_{c}": 0.0 for f in FINGERS for c in ("mp_pitch", "pip_pitch", "dip_pitch")}
    p = AXIS_NAMES.index("pinky_S2")
    out = r.joint_targets(calibration.normalize_hand(
        HandAngles("right", {**radians, **{c: 0.182 for c in declared}})))
    assert out[p] == pytest.approx(PINKY_SPREAD_RAD)
    assert out[AXIS_NAMES.index("ring_S2")] == pytest.approx(SPREAD_RAD)
    out = r.joint_targets(calibration.normalize_hand(
        HandAngles("right", {**radians, **{c: -0.3 for c in declared}})))
    assert out[p] == pytest.approx(-PINKY_SPREAD_RAD)
    assert JOINTS[p].min < -PINKY_SPREAD_RAD and PINKY_SPREAD_RAD < JOINTS[p].max


def test_a_missing_input_holds_only_its_joint():
    r = Retargeter()
    values = {c: 0.5 for c in r.required_channels()}
    values["ring_pip_pitch"] = None
    values["index_mp_swing"] = None
    out = r.joint_targets(_pose(values))
    assert [AXIS_NAMES[i] for i, t in enumerate(out) if t is None] == ["index_S2", "ring_S3"]
    empty = r.joint_targets(_pose({}))
    assert empty[:4] == [0.0] * 4 and empty[4:] == [None] * 16


def test_held_joints_read_nothing_and_command_their_hold_angle():
    hold = [False] * 20
    hold[AXIS_NAMES.index("middle_S3")] = True
    hold_rad = [0.0] * 20
    hold_rad[AXIS_NAMES.index("middle_S3")] = 0.4
    r = Retargeter(RetargetConfig(hold=tuple(hold), hold_rad=tuple(hold_rad)))
    assert "middle_pip_pitch" not in r.required_channels()
    assert "thumb_cm_yaw" in r.required_channels()   # the thumb now follows
    out = r.joint_targets(_pose({}))
    assert out[AXIS_NAMES.index("middle_S3")] == 0.4
    held = [None] * 20
    assert fill_held(out, held) is None, "nothing commanded yet: the rest has no value"


def test_the_thumb_follows_once_released():
    r = Retargeter(RetargetConfig(hold=(False,) * 20))
    values = {c: 1.0 for c in r.required_channels()}
    out = r.joint_targets(_pose(values))
    assert out[:4] == pytest.approx([0.8, 0.6, 1.0, 0.6])
    assert set(r.required_channels()) >= {"thumb_cm_yaw", "thumb_mp_pitch"}
    assert not {"thumb_cm_pitch", "thumb_cm_swing", "thumb_ip_pitch"} & set(r.required_channels())


def test_opposition_drives_s1_and_s2_together_inside_their_limits():
    maps = default_joint_maps()
    assert maps["thumb_S1"].sources == maps["thumb_S2"].sources == (("thumb_cm_yaw", 1.0),)
    assert maps["thumb_S3"].sources == maps["thumb_S4"].sources == (("thumb_mp_pitch", 1.0),)
    for i, name in enumerate(AXIS_NAMES[:4]):
        assert JOINTS[i].min <= maps[name].lo < maps[name].hi <= JOINTS[i].max
    r = Retargeter(RetargetConfig(hold=(False,) * 20))
    half = r.joint_targets(_pose({"thumb_cm_yaw": 0.5, "thumb_mp_pitch": 0.0}))
    assert half[:4] == pytest.approx([0.4, 0.3, 0.0, 0.0])


def test_configs_are_refused_across_hands_and_outside_limits():
    with pytest.raises(ValueError, match="another hand"):
        Retargeter(XhConfig())
    maps = default_joint_maps()
    maps["index_S2"] = JointMap((("index_mp_swing", 1.0),), -0.8, 0.8)
    with pytest.raises(ValueError, match="leaves the joint's limits"):
        RetargetConfig(joints=maps)
    with pytest.raises(ValueError, match="hold needs 20"):
        RetargetConfig(hold=(True,) * 4)
    with pytest.raises(ValueError, match="hold_rad 2.0 leaves"):
        RetargetConfig(hold_rad=(2.0,) + (0.0,) * 19)


def test_the_v1_call_returns_twenty_radians():
    out = hands.get_retarget("wuji/wujihand2")(lambda c: 0.5)
    assert out.shape == (20,)
    assert out[:4].tolist() == [0.0] * 4


# ------------------------------------------- against a real glove stream --
def _declaration_and_frames():
    dec = PoseStreamDecoder()
    hands_ = []
    for line in (DATA / "per_hand.session_head.jsonl").read_text().splitlines():
        hands_ += dec.feed(line)
    return dec.declaration_for("right"), hands_


def test_a_litchibot_bind_is_refused_for_spread_and_accepted_with_it_held():
    declaration, frames = _declaration_and_frames()
    with pytest.raises(DeclarationError) as err:
        declaration.check_drive_sources(Retargeter().required_channels())
    refused = [line.split(" ")[0] for line in str(err.value).splitlines()]
    assert sorted(refused) == sorted(f"{f}_mp_swing" for f in FINGERS)

    r = Retargeter(RetargetConfig(hold=tuple(_hold_thumb_and_spread())))
    declaration.check_drive_sources(r.required_channels())
    ranges = {k: v for k, v in load_ranges(DATA / "litchibot_wujihand2.calibration.json").items()
              if not k.endswith("_mp_swing")}
    calibration = declaration.calibration(ranges)
    assert frames
    for angles in frames:
        out = r.joint_targets(calibration.normalize_hand(angles))
        assert all(t is not None for t in out)
        for t, j in zip(out, JOINTS):
            assert j.min <= t <= j.max
        assert [out[i] for i in S2] == [0.0] * 4


def test_a_binding_file_holds_the_spread(tmp_path):
    hold = json.dumps(_hold_thumb_and_spread())
    path = tmp_path / "binding.toml"
    path.write_text(textwrap.dedent(f'''
        schema = "omakase.binding.v1"

        [[source]]
        device = "glove"
        family = "teleop_gloves.pose_stream.v1"

        [[bind]]
        from = "glove"
        to = "hand/left"
        retarget = "wuji/wujihand2"

        [bind.retarget_config]
        hold = {hold}

        [bind.retarget_config.joints.index_S2]
        invert = true
    '''))
    bind = load_binding(path).binds[0]
    r = bind.build_retarget()
    assert isinstance(r, Retargeter)
    assert r.cfg.hold == tuple(_hold_thumb_and_spread())
    assert r.cfg.joints["index_S2"].invert
    assert not any(c.endswith("_mp_swing") for c in r.required_channels())


def test_a_held_joint_never_asks_the_pose_for_its_channels():
    class Strict(dict):
        def get(self, channel, default=None):
            if channel.endswith("_swing"):
                raise KeyError(f"no range for {channel}")
            return super().get(channel, default)

    hold = list(DEFAULT_HOLD)
    for name in ("index_S2", "middle_S2", "ring_S2", "pinky_S2"):
        hold[AXIS_NAMES.index(name)] = True
    r = Retargeter(RetargetConfig(hold=tuple(hold)))
    pose = Strict({ch: 0.5 for ch in r.required_channels()})
    out = r.joint_targets(pose)
    assert out[AXIS_NAMES.index("index_S2")] == 0.0
    assert out[AXIS_NAMES.index("index_S1")] is not None


def _spread_declaration(middle="synthesized", swing="measured"):
    """A device-wide declaration shaped like a glove that measures every finger
    joint and the index, ring and pinky spreads, and derives the middle spread
    from the index and ring. Synthetic numbers."""
    chans = {}
    for f in FINGERS:
        for j in ("mp_pitch", "pip_pitch", "dip_pitch"):
            chans[f"{f}_{j}"] = dict(kind="flex", provenance="measured",
                                     default_range=[0.0, 1.5])
        chans[f"{f}_mp_swing"] = dict(kind="swing", provenance=swing,
                                      default_range=None if swing == "uncalibrated"
                                      else [-0.6, 0.6])
    if middle == "synthesized":
        chans["middle_mp_swing"].update(provenance="synthesized",
                                        derived_from=["index_mp_swing", "ring_mp_swing"])
    elif middle == "missing-input":
        chans["middle_mp_swing"].update(provenance="synthesized",
                                        derived_from=["index_mp_swing", "ring_mp_swing"])
        chans["ring_mp_swing"].update(provenance="uncalibrated", default_range=None)
    for name, c in chans.items():
        c.setdefault("note", "synthetic")
    return parse_declaration({"type": "channel_declaration", "device_model": "test/glove",
                              "channels": chans})


def test_a_glove_that_derives_the_middle_spread_binds_with_middle_s2_held():
    declaration = _spread_declaration()
    with pytest.raises(DeclarationError) as err:
        declaration.check_drive_sources(Retargeter(config_for(declaration)).required_channels())
    refused = [line.split(" ")[0] for line in str(err.value).splitlines()]
    assert refused == ["middle_mp_swing"]
    hold = list(DEFAULT_HOLD)
    hold[AXIS_NAMES.index("middle_S2")] = True
    r = Retargeter(config_for(declaration, RetargetConfig(hold=tuple(hold))))
    declaration.check_drive_sources(r.required_channels())
    assert "middle_mp_swing" not in r.required_channels()


def test_config_for_does_not_loosen_what_cannot_be_driven():
    # uncalibrated swing: every spread still refused
    declaration = _spread_declaration(middle="uncalibrated", swing="uncalibrated")
    r = Retargeter(config_for(declaration))
    with pytest.raises(DeclarationError, match="UNCALIBRATED"):
        declaration.check_drive_sources(r.required_channels())
    # a synthesized middle whose input is itself uncalibrated stays refused
    declaration = _spread_declaration(middle="missing-input")
    r = Retargeter(config_for(declaration))
    with pytest.raises(DeclarationError) as err:
        declaration.check_drive_sources(r.required_channels())
    assert "ring_mp_swing" in str(err.value) and "UNCALIBRATED" in str(err.value)
    # holding the spreads still binds, config_for leaves held joints alone
    hold = tuple(_hold_thumb_and_spread())
    cfg = config_for(declaration, RetargetConfig(hold=hold))
    declaration.check_drive_sources(Retargeter(cfg).required_channels())
    assert cfg.joints == RetargetConfig(hold=hold).joints


def _thumb_declaration(thumb="derived"):
    """A device-wide declaration shaped like the LitchiBot glove's thumb: yaw,
    swing and the last joint derived on the wearer, cm_pitch uncalibrated,
    ip_pitch undeclared. Synthetic numbers. ``thumb="uncalibrated"`` makes
    every thumb channel uncalibrated."""
    chans = {}
    for f in FINGERS:
        for j in ("mp_pitch", "pip_pitch", "dip_pitch"):
            chans[f"{f}_{j}"] = dict(kind="flex", provenance="measured",
                                     default_range=[0.0, 1.5])
        chans[f"{f}_mp_swing"] = dict(kind="swing", provenance="uncalibrated",
                                      default_range=None)
    rng = {"thumb_cm_yaw": [0.0, 1.0], "thumb_cm_swing": [-0.436, 0.436],
           "thumb_mp_pitch": [0.0, 1.571]}
    kinds = {"thumb_cm_yaw": "yaw", "thumb_cm_swing": "swing", "thumb_mp_pitch": "flex"}
    for name, r in rng.items():
        if thumb == "derived":
            chans[name] = dict(kind=kinds[name], provenance="derived", default_range=r)
        else:
            chans[name] = dict(kind=kinds[name], provenance="uncalibrated", default_range=None)
    chans["thumb_cm_pitch"] = dict(kind="flex", provenance="uncalibrated", default_range=None)
    for c in chans.values():
        c.setdefault("note", "synthetic")
    return parse_declaration({"type": "channel_declaration", "device_model": "test/glove",
                              "channels": chans})


def test_a_glove_that_solves_the_thumb_releases_all_four_thumb_joints():
    declaration = _thumb_declaration()
    cfg = config_for(declaration)
    thumb = [AXIS_NAMES.index(f"thumb_S{i}") for i in range(1, 5)]
    assert [cfg.hold[i] for i in thumb] == [False, False, False, False]
    assert {n: cfg.joints[n] for n in AXIS_NAMES[:4]} == {
        n: default_joint_maps()[n] for n in AXIS_NAMES[:4]}
    # fingers keep their holds: the spreads here are uncalibrated and still refused
    r = Retargeter(cfg)
    assert {"thumb_cm_yaw", "thumb_mp_pitch"} <= set(r.required_channels())
    assert not {"thumb_cm_pitch", "thumb_cm_swing", "thumb_ip_pitch"} & set(r.required_channels())

    # a bind that holds the finger spreads is accepted, and its thumb follows
    hold = tuple(i in S2 for i in range(20))
    cfg = config_for(declaration, RetargetConfig(hold=hold))
    r = Retargeter(cfg)
    declaration.check_drive_sources(r.required_channels())

    cal = declaration.calibration()
    values = {f"{f}_{j}": 0.0 for f in FINGERS for j in ("mp_pitch", "pip_pitch", "dip_pitch")}
    open_ = HandAngles("right", {**values, "thumb_cm_yaw": 0.0, "thumb_cm_swing": 0.0,
                                 "thumb_mp_pitch": 0.0})
    out = r.joint_targets(cal.normalize_hand(open_))
    assert [out[i] for i in thumb] == pytest.approx([0.0, 0.0, 0.0, 0.0], abs=1e-9)
    opposed = HandAngles("right", {**values, "thumb_cm_yaw": 1.0, "thumb_cm_swing": 0.436,
                                   "thumb_mp_pitch": 1.571})
    out = r.joint_targets(cal.normalize_hand(opposed))
    assert [out[i] for i in thumb] == pytest.approx([0.8, 0.6, 1.0, 0.6], abs=1e-9)
    # the thumb's lateral swing drives nothing
    swung = HandAngles("right", {**opposed.radians, "thumb_cm_swing": -0.436})
    assert r.joint_targets(cal.normalize_hand(swung)) == out


def test_a_thumb_without_its_flexion_stays_held():
    declaration = _thumb_declaration()
    chans = {n: c for n, c in declaration.channels.items()}
    assert "thumb_mp_pitch" in chans
    raw = {"type": "channel_declaration", "device_model": "test/glove", "channels": {}}
    for name, c in chans.items():
        if name == "thumb_mp_pitch":
            continue
        raw["channels"][name] = dict(
            kind=c.kind.value if hasattr(c.kind, "value") else c.kind,
            provenance=c.provenance.value,
            default_range=None if c.default_range is None else [c.default_range.lo, c.default_range.hi],
            note="synthetic")
    cfg = config_for(parse_declaration(raw))
    assert cfg.hold[:4] == (True,) * 4


def test_an_uncalibrated_thumb_keeps_the_default_hold():
    declaration = _thumb_declaration(thumb="uncalibrated")
    cfg = config_for(declaration)
    assert cfg.hold == DEFAULT_HOLD
    assert cfg.joints == default_joint_maps()
    r = Retargeter(cfg)
    assert not any(c.startswith("thumb_") for c in r.required_channels())
