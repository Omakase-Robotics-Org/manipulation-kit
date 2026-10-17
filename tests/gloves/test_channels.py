"""The glove channel vocabulary, and its drift against the Rust table it mirrors.

``manipulation_kit.gloves.channels.CHANNELS`` mirrors the channel table of
the glove stream encoder, a Rust source file that is not part of this
repository, so the exact comparison is opt-in: put a copy of it at
``$MKIT_GLOVE_REFERENCE_DIR/channels.rs`` and
``test_channels_match_the_rust_table`` parses it and compares every entry.
Without it the test SKIPS and says so (``-rs``); the structural tests below
still run everywhere.
"""
import os
import re
from pathlib import Path

import pytest

from manipulation_kit.gloves.channels import (
    ALL_CHANNELS,
    CHANNELS,
    EXTENSION_CHANNELS,
    FINGERS,
    SAME_NAME_DIFFERENT_THING,
    ChannelKind,
    channels_for_slot,
    spec_of,
)


def test_every_channel_is_named_once_and_on_a_known_finger():
    names = [s.name for s in ALL_CHANNELS]
    assert len(names) == len(set(names))
    assert len(CHANNELS) == 28
    assert all(s.finger in FINGERS for s in ALL_CHANNELS)


def test_no_two_vocabulary_channels_share_a_v1_slot():
    slots = [s.v1_slot for s in CHANNELS if s.v1_slot]
    assert len(slots) == len(set(slots))


def test_thumb_swing_is_the_one_slot_with_two_names():
    shared = {}
    for s in ALL_CHANNELS:
        if s.v1_slot:
            shared.setdefault(s.v1_slot, []).append(s.name)
    multi = {slot: names for slot, names in shared.items() if len(names) > 1}
    assert multi == {("thumb", "swing"): ["thumb_cm_roll", "thumb_cm_swing"]}
    assert [s.name for s in channels_for_slot("thumb", "swing")] == [
        "thumb_cm_roll", "thumb_cm_swing"]


def test_thumb_yaw_off_the_thumb_is_no_channel():
    for finger in FINGERS[1:]:
        assert list(channels_for_slot(finger, "thumb_yaw")) == []


def test_swing_is_the_only_bipolar_kind():
    for s in ALL_CHANNELS:
        assert s.kind.bipolar == (s.kind is ChannelKind.SWING)


def test_same_name_channels_carry_a_caveat_and_no_other_does():
    for s in CHANNELS:
        assert bool(s.caveat) == (s.name in SAME_NAME_DIFFERENT_THING), s.name


def test_the_extension_is_outside_the_rust_table():
    assert [s.name for s in EXTENSION_CHANNELS] == ["thumb_cm_swing"]
    assert spec_of("thumb_cm_swing").kind is ChannelKind.SWING
    assert spec_of("index_mp_pitchh") is None


def test_the_dh116s_channels_are_all_in_the_vocabulary():
    from manipulation_kit.hands.leadshine.dh116s.retarget import CHANNELS as DH
    assert all(spec_of(c) is not None for c in DH)


# ---------------------------------------------------------------- drift --
def _args(body: str):
    """Split a Rust call's argument list at top-level commas."""
    out, depth, cur, in_str, esc = [], 0, "", False, False
    for ch in body:
        if in_str:
            cur += ch
            if esc:
                esc = False
            elif ch == "\\":
                esc = True
            elif ch == '"':
                in_str = False
            continue
        if ch == '"':
            in_str = True
        elif ch in "([":
            depth += 1
        elif ch in ")]":
            depth -= 1
        elif ch == "," and depth == 0:
            out.append(cur.strip())
            cur = ""
            continue
        cur += ch
    if cur.strip():
        out.append(cur.strip())
    return out


def _calls(src: str):
    """Every ``spec(...)`` call's argument list, in source order."""
    for m in re.finditer(r"(?<!fn )\bspec\(", src):
        i, depth = m.end(), 1
        while depth:
            depth += {"(": 1, ")": -1}.get(src[i], 0)
            i += 1
        yield _args(src[m.end():i - 1])


def _value(arg: str, finger: str):
    arg = arg.replace("$f", f'"{finger}"')
    if arg == "None":
        return None
    m = re.fullmatch(r"Some\((.*)\)", arg, re.S)
    if m:
        arg = m.group(1)
    m = re.fullmatch(r'concat!\("(\w+)",\s*"(\w+)"\)', arg)
    if m:
        return m.group(1) + m.group(2)
    m = re.fullmatch(r"ChannelKind::(\w+)", arg)
    if m:
        return m.group(1).upper()
    return arg.strip('"')


def _rust_table(src: str):
    """``(name, finger, KIND, v1_field, omakase_channel)`` in channels() order."""
    macro = src[src.index("macro_rules! finger"):src.index("const THUMB:")]
    thumb = src[src.index("const THUMB:"):src.index("const INDEX")]
    rolls = src[src.index("const ROLLS"):src.index("pub fn channels")]
    rows = []

    def take(block, finger):
        for args in _calls(block):
            vals = [_value(a, finger) for a in args[:5]]
            rows.append(tuple(vals))

    take(thumb, "thumb")
    for finger in ("index", "middle", "ring", "pinky"):
        take(macro, finger)
    take(rolls, "")
    return rows


def test_channels_match_the_rust_table():
    root = os.environ.get("MKIT_GLOVE_REFERENCE_DIR")
    path = Path(root) / "channels.rs" if root else None
    if path is None or not path.is_file():
        pytest.skip("no $MKIT_GLOVE_REFERENCE_DIR/channels.rs: the encoder's "
                    "channel table is not part of this repository")
    src = path.read_text()
    rust = _rust_table(src)
    ours = [(s.name, s.finger, s.kind.name, s.v1_field, s.omakase_channel)
            for s in CHANNELS]
    assert ours == rust
    same = re.search(r"SAME_NAME_DIFFERENT_THING: \[&str; \d+\] = \[(.*?)\]", src)
    assert tuple(re.findall(r'"(\w+)"', same.group(1))) == SAME_NAME_DIFFERENT_THING
