"""teleop_gloves.pose_stream.v1 decoding, in both dialects.

Fixtures (``tests/data/gloves/``, provenance in its README.md):

* ``device_wide.pose_stream.v1.jsonl`` — the device-wide dialect's golden
  vectors (hello, a UDCAP declaration, one pose frame), each record a
  ``{"name", "wire", "expect"}`` wrapper around the wire bytes;
* ``per_hand.session_head.jsonl`` — the LitchiBot driver's per-hand dialect:
  hello, a right-hand declaration, five frames;
* ``per_hand.dropout.jsonl`` — the same driver with the ring finger's distal
  sensor silent, so three ring channels are absent from every frame.

The fixtures keep the reference recordings' keys, key order and numbers;
free text, driver names and sensor node names were replaced.
``test_derived_fixtures_decode_like_the_originals`` checks that against the
references when ``MKIT_GLOVE_REFERENCE_DIR`` holds them, and skips otherwise.
"""
import json
import math
import os
from pathlib import Path

import pytest

from manipulation_kit.gloves.v1 import (
    DeclarationError,
    PoseStreamDecoder,
    Provenance,
    parse_declaration,
    vocabulary_declaration,
)

DATA = Path(__file__).resolve().parents[1] / "data" / "gloves"


def _omakase_lines(path=DATA / "device_wide.pose_stream.v1.jsonl"):
    return [json.loads(line)["wire"] for line in path.read_text().splitlines()]


def _lines(name):
    return (DATA / name).read_text().splitlines()


def _decode(lines, **kw):
    dec = PoseStreamDecoder(**kw)
    hands = []
    for line in lines:
        hands.extend(dec.feed(line))
    return dec, hands


# ------------------------------------------------------- device-wide dialect --
def test_udcap_declaration_is_device_wide_and_complete():
    dec, _ = _decode(_omakase_lines()[:2])
    decl = dec.declaration_for("right")
    assert decl is dec.declaration_for("left")      # no side: both hands
    assert decl.device_model == "udexreal/udcap"
    assert len(decl.channels) == 28
    assert decl.channels["thumb_cm_roll"].slot == ("thumb", "swing")
    assert decl.channels["index_total_pitch"].provenance is Provenance.SYNTHESIZED
    assert decl.channels["index_total_pitch"].derived_from == (
        "index_mp_pitch", "index_pip_pitch", "index_dip_pitch")
    assert decl.channels["index_mp_swing"].provenance is Provenance.UNCALIBRATED
    assert "index_mp_swing" not in decl.default_ranges()
    assert decl.default_ranges()["index_pip_pitch"].hi == pytest.approx(math.radians(100))


def test_udcap_frame_decodes_every_declared_channel():
    _, hands = _decode(_omakase_lines())
    (right,) = hands
    assert right.side == "right" and right.timestamp_s == 20.5
    assert right.radians["index_pip_pitch"] == pytest.approx(1.7453292519943295)
    assert right.radians["index_total_pitch"] == pytest.approx(4.537856055185257)
    # thumb.swing is thumb_cm_roll on this device, thumb.thumb_yaw is the yaw
    assert right.radians["thumb_cm_roll"] == 0.0
    assert right.radians["thumb_cm_yaw"] == 0.0
    assert set(right.radians) == set(_decode(_omakase_lines()[:2])[0]
                                     .declaration_for("right").channels)


def test_uncalibrated_and_absent_channels_are_none_never_zero():
    """The golden frame omits index.swing (uncalibrated). v1's reference
    decoder read that as 0.0; this one must answer None."""
    _, (right,) = _decode(_omakase_lines())
    for ch in ("index_mp_swing", "middle_mp_swing", "ring_mp_swing",
               "pinky_mp_swing", "index_mp_roll", "pinky_mp_roll"):
        assert right.radians[ch] is None, ch


def test_an_uncalibrated_channel_is_none_even_when_a_number_is_sent():
    lines = _omakase_lines()
    frame = json.loads(lines[2])
    frame["frames"]["right"]["finger_angles_rad"]["index"]["swing"] = 0.3
    _, (right,) = _decode(lines[:2] + [json.dumps(frame)])
    assert right.radians["index_mp_swing"] is None


def test_non_finite_and_non_numeric_values_are_none():
    lines = _omakase_lines()
    frame = json.loads(lines[2])
    fingers = frame["frames"]["right"]["finger_angles_rad"]
    fingers["index"]["mcp_flex"] = "1.0"
    fingers["middle"]["mcp_flex"] = None
    fingers["ring"]["mcp_flex"] = True
    fingers["pinky"]["mcp_flex"] = float("nan")
    _, (right,) = _decode(lines[:2] + [frame])           # a dict, so NaN survives
    for ch in ("index_mp_pitch", "middle_mp_pitch", "ring_mp_pitch", "pinky_mp_pitch"):
        assert right.radians[ch] is None, ch


def test_a_frame_before_its_declaration_decodes_to_nothing():
    lines = _omakase_lines()
    dec, hands = _decode([lines[0], lines[2]])
    assert hands == [] and dec.undeclared_frames == 1


def test_without_a_declaration_on_request_the_vocabulary_is_used():
    lines = _omakase_lines()
    _, (right,) = _decode([lines[2]], require_declaration=False)
    assert right.radians["thumb_cm_roll"] == 0.0
    assert right.radians["index_mp_swing"] is None       # absent, not 0.0
    assert "thumb_cm_swing" not in vocabulary_declaration().channels


def test_unknown_types_are_dropped_and_schema_is_reported_not_enforced():
    lines = _omakase_lines()
    dec = PoseStreamDecoder()
    for line in lines[:2]:
        dec.feed(line)
    assert dec.feed({"type": "diagnostics", "schema": "x"}) == []
    frame = json.loads(lines[2])
    frame["schema"] = "teleop_gloves.pose_stream.v9"
    assert len(dec.feed(frame)) == 1
    assert dec.last_schema == "teleop_gloves.pose_stream.v9"
    assert dec.last_sequence == 3


# ---------------------------------------------------------------- LitchiBot --
def test_litchibot_declaration_is_per_hand_with_v1_fields_and_nodes():
    dec, hands = _decode(_lines("per_hand.session_head.jsonl"))
    decl = dec.declaration_for("right")
    assert decl.side == "right"
    assert dec.declaration_for("left") is None
    assert decl.device_model == "litchibot/glove"
    assert decl.channels["thumb_cm_swing"].slot == ("thumb", "swing")
    assert "thumb_cm_roll" not in decl.channels
    assert decl.channels["index_dip_pitch"].derived_from_nodes == (
        "node-1", "node-2")
    assert decl.channels["index_dip_pitch"].derived_from == ()
    assert decl.channels["thumb_cm_pitch"].provenance is Provenance.UNCALIBRATED
    assert len(hands) == 5 and all(h.side == "right" for h in hands)


def test_litchibot_frames_put_the_thumb_swing_slot_in_thumb_cm_swing():
    _, hands = _decode(_lines("per_hand.session_head.jsonl"))
    first = json.loads(_lines("per_hand.session_head.jsonl")[2])
    wire_thumb = first["frames"]["right"]["finger_angles_rad"]["thumb"]
    assert hands[0].radians["thumb_cm_swing"] == wire_thumb["swing"]
    assert hands[0].radians["thumb_cm_yaw"] == wire_thumb["thumb_yaw"]
    assert hands[0].radians["thumb_mp_pitch"] == wire_thumb["pip_flex"]
    # thumb.mcp_flex is on the wire (0.0) and declared uncalibrated
    assert wire_thumb["mcp_flex"] == 0.0
    assert hands[0].radians["thumb_cm_pitch"] is None


def test_litchibot_dropout_leaves_the_silent_channels_none():
    _, hands = _decode(_lines("per_hand.dropout.jsonl"))
    assert len(hands) == 5
    for hand in hands:
        for ch in ("ring_pip_pitch", "ring_dip_pitch", "ring_total_pitch"):
            assert hand.radians[ch] is None, ch
        assert hand.radians["ring_mp_pitch"] is not None
        assert hand.radians["index_pip_pitch"] is not None


# ---------------------------------------------------------- declarations --
def _decl(channels, **extra):
    return {"type": "channel_declaration", "device_model": "x/y",
            "channels": channels, **extra}


FLEX = {"kind": "flex", "provenance": "measured", "derived_from": []}


def test_a_name_outside_the_vocabulary_is_refused():
    with pytest.raises(DeclarationError, match="not in the glove channel vocabulary"):
        parse_declaration(_decl({"index_mcp_flex": FLEX}))


def test_a_v1_field_that_disagrees_with_the_vocabulary_is_refused():
    bad = dict(FLEX, v1_field={"finger": "index", "field": "pip_flex"})
    with pytest.raises(DeclarationError, match="another joint"):
        parse_declaration(_decl({"index_mp_pitch": bad}))


def test_two_channels_claiming_the_thumb_swing_slot_are_refused():
    roll = {"kind": "roll", "provenance": "measured"}
    swing = {"kind": "swing", "provenance": "derived"}
    with pytest.raises(DeclarationError, match="both claim v1 slot thumb.swing"):
        parse_declaration(_decl({"thumb_cm_roll": roll, "thumb_cm_swing": swing}))


def test_a_kind_that_disagrees_with_the_vocabulary_is_refused():
    with pytest.raises(DeclarationError, match="declared kind"):
        parse_declaration(_decl({"index_mp_swing": FLEX}))


def test_a_zero_span_default_range_is_refused():
    with pytest.raises(DeclarationError, match="non-zero span"):
        parse_declaration(_decl({"index_mp_pitch": dict(FLEX, default_range=[1.0, 1.0])}))


def test_drive_source_refusals_follow_the_rust_rule():
    dec, _ = _decode(_omakase_lines()[:2])
    udcap = dec.declaration_for("right")
    assert udcap.refuse_as_drive_source("index_mp_pitch") is None
    assert "UNCALIBRATED" in udcap.refuse_as_drive_source("index_mp_swing")
    assert "SYNTHESIZED" in udcap.refuse_as_drive_source("index_total_pitch")
    assert "does not report" in udcap.refuse_as_drive_source("thumb_cm_swing")
    with pytest.raises(DeclarationError) as err:
        udcap.check_drive_sources(["index_mp_pitch", "index_mp_swing", "index_total_pitch"])
    assert str(err.value).count("\n") == 1               # two refusals, one error
    # a synthesis whose sources are not all there is the only thing there is
    partial = parse_declaration(_decl({
        "index_mp_pitch": FLEX,
        "index_total_pitch": {"kind": "total", "provenance": "synthesized",
                              "derived_from": ["index_mp_pitch", "index_pip_pitch"]},
    }))
    assert partial.refuse_as_drive_source("index_total_pitch") is None


# ---------------------------------------------- derived vs original bytes --
def _same_decode(original_lines, derived_lines):
    _, a = _decode(original_lines)
    _, b = _decode(derived_lines)
    assert [(h.side, h.radians, h.timestamp_s) for h in a] == \
           [(h.side, h.radians, h.timestamp_s) for h in b]
    da, db = _decode(original_lines)[0], _decode(derived_lines)[0]
    for side in ("left", "right"):
        x, y = da.declaration_for(side), db.declaration_for(side)
        if x is None:
            assert y is None
            continue
        assert list(x.channels) == list(y.channels)
        for name in x.channels:
            cx, cy = x.channels[name], y.channels[name]
            # node names were replaced in the fixtures; their count was not
            assert (cx.kind, cx.provenance, cx.slot, cx.derived_from,
                    len(cx.derived_from_nodes), cx.default_range) == \
                   (cy.kind, cy.provenance, cy.slot, cy.derived_from,
                    len(cy.derived_from_nodes), cy.default_range)


def test_derived_fixtures_decode_like_the_originals():
    """``MKIT_GLOVE_REFERENCE_DIR`` holds the reference recordings the fixtures
    were derived from, under the fixtures' own names (the per-hand session as
    ``per_hand.session.jsonl``, of which the fixture is the first 7 lines)."""
    root = os.environ.get("MKIT_GLOVE_REFERENCE_DIR")
    if not root:
        pytest.skip("MKIT_GLOVE_REFERENCE_DIR not set: the reference recordings "
                    "are not part of this repository")
    ref = Path(root)
    pairs = [
        ("device_wide.pose_stream.v1.jsonl",
         lambda p: _same_decode(_omakase_lines(p), _omakase_lines())),
        ("per_hand.session.jsonl",
         lambda p: _same_decode(p.read_text().splitlines()[:7],
                                _lines("per_hand.session_head.jsonl"))),
        ("per_hand.dropout.jsonl",
         lambda p: _same_decode(p.read_text().splitlines(),
                                _lines("per_hand.dropout.jsonl"))),
    ]
    found = [(name, check) for name, check in pairs if (ref / name).is_file()]
    if not found:
        pytest.skip(f"none of {[n for n, _ in pairs]} in {ref}")
    for name, check in found:
        check(ref / name)
