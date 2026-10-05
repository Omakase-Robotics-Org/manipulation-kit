"""ChannelRange / ChannelCalibration: the Rust semantics, None-preserving, and
the calibration file round trip."""
import json
import math

import pytest

from manipulation_kit.gloves import (
    CALIBRATION_SCHEMA,
    ChannelCalibration,
    ChannelRange,
    HandAngles,
    UnknownChannels,
    load_ranges,
    save_ranges,
)


def test_lo_reads_zero_hi_reads_one_linear_between_clamped_outside():
    r = ChannelRange(0.2, 1.2)
    assert r.normalize(0.2) == 0.0
    assert r.normalize(1.2) == 1.0
    assert r.normalize(0.7) == pytest.approx(0.5)
    assert r.normalize(-5.0) == 0.0 and r.normalize(5.0) == 1.0


def test_invert_flips_after_the_clamp():
    r = ChannelRange(0.0, 1.0, invert=True)
    assert r.normalize(0.0) == 1.0 and r.normalize(2.0) == 0.0
    assert r.inverted() == ChannelRange(0.0, 1.0)


def test_a_reversed_range_is_a_sign_flip_not_an_error():
    r = ChannelRange(1.0, 0.0)
    assert r.normalize(1.0) == 0.0 and r.normalize(0.0) == 1.0


def test_a_bipolar_range_lands_neutral_at_one_half():
    r = ChannelRange(-0.349, 0.349)
    assert r.normalize(0.0) == pytest.approx(0.5)


@pytest.mark.parametrize("lo,hi", [(1.0, 1.0), (0.0, math.inf), (math.nan, 1.0)])
def test_a_range_with_no_span_or_a_non_finite_bound_is_refused(lo, hi):
    with pytest.raises(ValueError):
        ChannelRange(lo, hi)


def test_none_in_none_out_and_never_zero():
    cal = ChannelCalibration({"index_mp_pitch": ChannelRange(0.0, 1.4)})
    assert cal.normalize("index_mp_pitch", None) is None
    assert cal.normalize("index_mp_pitch", math.nan) is None
    assert cal.normalize("index_mp_pitch", 0.7) == pytest.approx(0.5)


def test_a_channel_with_no_range_is_an_error_not_a_guess():
    cal = ChannelCalibration({"index_mp_pitch": ChannelRange(0.0, 1.4)})
    with pytest.raises(UnknownChannels):
        cal.normalize("middle_mp_pitch", 0.3)


def test_overrides_win_and_unknown_overrides_are_refused():
    defaults = {"index_mp_pitch": ChannelRange(0.0, 1.4),
                "middle_mp_pitch": ChannelRange(0.0, 1.4)}
    cal = ChannelCalibration(defaults, {"index_mp_pitch": ChannelRange(0.0, 0.7)})
    assert cal.normalize("index_mp_pitch", 0.7) == 1.0
    assert cal.normalize("middle_mp_pitch", 0.7) == pytest.approx(0.5)
    with pytest.raises(UnknownChannels):
        ChannelCalibration(defaults, {"index_mp_pich": ChannelRange(0.0, 1.0)})
    merged = cal.overriding({"middle_mp_pitch": ChannelRange(0.0, 0.7)})
    assert merged.normalize("middle_mp_pitch", 0.7) == 1.0
    assert merged.normalize("index_mp_pitch", 0.7) == 1.0


def test_known_admits_a_declared_channel_with_no_default():
    cal = ChannelCalibration({}, {"index_mp_pitch": ChannelRange(0.0, 1.2)},
                             known=["index_mp_pitch"])
    assert cal.normalize("index_mp_pitch", 0.6) == pytest.approx(0.5)


def test_normalize_hand_keeps_missing_and_unranged_channels_none():
    cal = ChannelCalibration({"index_mp_pitch": ChannelRange(0.0, 1.0),
                              "middle_mp_pitch": ChannelRange(0.0, 1.0)})
    pose = cal.normalize_hand(HandAngles(
        "right", {"index_mp_pitch": 0.25, "middle_mp_pitch": None,
                  "index_mp_swing": 0.1}, timestamp_s=3.0))
    assert pose.side == "right" and pose.timestamp_s == 3.0
    assert pose.flex == {"index_mp_pitch": 0.25, "middle_mp_pitch": None,
                         "index_mp_swing": None}
    assert pose.get("ring_mp_pitch") is None
    assert pose.missing(["index_mp_pitch", "middle_mp_pitch"]) == ["middle_mp_pitch"]
    with pytest.raises(KeyError):
        pose.as_flex_callable()("middle_mp_pitch")


def test_file_round_trip_keeps_overrides_only(tmp_path):
    defaults = {"index_mp_pitch": ChannelRange(0.0, 1.4),
                "thumb_cm_yaw": ChannelRange(0.0, 0.96)}
    cal = ChannelCalibration(defaults, {"thumb_cm_yaw": ChannelRange(0.1, 0.8, True)})
    path = tmp_path / "op.json"
    cal.save(path, device_model="udexreal/udcap")
    data = json.loads(path.read_text())
    assert data == {"schema": CALIBRATION_SCHEMA, "device_model": "udexreal/udcap",
                    "ranges": {"thumb_cm_yaw": {"lo": 0.1, "hi": 0.8, "invert": True}}}
    again = ChannelCalibration.load(path, defaults)
    assert again.overrides == cal.overrides
    assert again.range_for("index_mp_pitch") == defaults["index_mp_pitch"]


@pytest.mark.parametrize("doc,match", [
    ({"schema": "nope", "ranges": {}}, "schema"),
    ({"schema": CALIBRATION_SCHEMA, "ranges": {}, "extra": 1}, "unknown key"),
    ({"schema": CALIBRATION_SCHEMA, "ranges": {"index_mcp": {"lo": 0, "hi": 1}}},
     "not a glove channel"),
    ({"schema": CALIBRATION_SCHEMA, "ranges": {"index_mp_pitch": {"lo": 1, "hi": 1}}},
     "non-zero span"),
    ({"schema": CALIBRATION_SCHEMA, "ranges": {"index_mp_pitch": {"lo": 0}}},
     "lo, hi"),
    ({"schema": CALIBRATION_SCHEMA, "ranges": {"index_mp_pitch": {"lo": True, "hi": 1}}},
     "must be a number"),
])
def test_a_bad_file_is_refused_naming_the_problem(tmp_path, doc, match):
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(doc))
    with pytest.raises(ValueError, match=match):
        load_ranges(path)


def test_save_ranges_writes_a_loadable_file(tmp_path):
    path = tmp_path / "r.json"
    save_ranges(path, {"index_mp_swing": ChannelRange(-0.3, 0.3)})
    assert load_ranges(path) == {"index_mp_swing": ChannelRange(-0.3, 0.3)}


def test_a_declaration_builds_its_calibration():
    from manipulation_kit.gloves.v1 import parse_declaration
    decl = parse_declaration({
        "device_model": "x/y", "channels": {
            "index_mp_pitch": {"kind": "flex", "provenance": "measured"},
            "index_mp_swing": {"kind": "swing", "provenance": "uncalibrated",
                               "default_range": [-0.6, 0.2], "note": "no stage"},
            "index_pip_pitch": {"kind": "flex", "provenance": "measured",
                                "default_range": [0.0, 1.7]},
        }})
    cal = decl.calibration({"index_mp_pitch": ChannelRange(0.0, 1.2)})
    assert cal.normalize("index_mp_pitch", 0.6) == pytest.approx(0.5)
    assert cal.normalize("index_pip_pitch", 0.85) == pytest.approx(0.5)
    with pytest.raises(UnknownChannels):        # uncalibrated: not even by file
        decl.calibration({"index_mp_swing": ChannelRange(-0.6, 0.2)})
