"""omakase.binding.v1: loading, and every startup refusal."""
import json
import textwrap

import pytest

from manipulation_kit.binding import (
    BINDING_SCHEMA,
    DAEMON_SIDE,
    BindingError,
    load_binding,
    parse_binding,
)
from manipulation_kit.gloves import CALIBRATION_SCHEMA, ChannelRange

GOOD = textwrap.dedent('''
    schema = "omakase.binding.v1"

    [[source]]
    device = "udcap-right"
    family = "teleop_gloves.pose_stream.v1"

    [[source]]
    device = "pedal"
    family = "teleop_controls.event_stream.v1"
    instance = 0

    [[bind]]
    from        = "udcap-right"
    to          = "hand/right"
    retarget    = "robotera/xhand1"
    calibration = "operators/alice.json"

    [[bind]]
    from  = "pedal/#0"
    to    = "gripper/left/openness"
    range = { lo = 0.0, hi = 1.0, invert = true }

    [[bind]]
    from = "pedal/#1"
    to   = "accessory/suction/valve"
    map  = { 0 = 0, 2 = 1 }
''')


@pytest.fixture
def binding_dir(tmp_path):
    (tmp_path / "operators").mkdir()
    (tmp_path / "operators" / "alice.json").write_text(json.dumps({
        "schema": CALIBRATION_SCHEMA,
        "ranges": {"index_mp_pitch": {"lo": 0.05, "hi": 1.3}}}))
    return tmp_path


def _write(d, text):
    path = d / "binding.toml"
    path.write_text(text)
    return path


def _doc(binds, sources=None):
    return {"schema": BINDING_SCHEMA,
            "source": sources if sources is not None else [
                {"device": "glove", "family": "teleop_gloves.pose_stream.v1"},
                {"device": "pedal", "family": "teleop_controls.event_stream.v1"}],
            "bind": binds}


def _problems(doc, **kw):
    with pytest.raises(BindingError) as err:
        parse_binding(doc, **kw)
    return err.value.problems


def test_a_good_file_loads(binding_dir):
    b = load_binding(_write(binding_dir, GOOD))
    hand, grip, valve = b.binds
    assert hand.whole_hand and hand.side == "right" and hand.channel is None
    assert hand.retarget == "robotera/xhand1"
    assert hand.calibration == (binding_dir / "operators" / "alice.json").resolve()
    assert hand.calibration_ranges == {"index_mp_pitch": ChannelRange(0.05, 1.3)}
    assert grip.channel == "#0" and grip.range == ChannelRange(0.0, 1.0, True)
    assert valve.map == {0: 0, 2: 1} and valve.side is None
    assert b.source("pedal").instance == 0
    assert b.hand_binds() == (hand,)
    assert DAEMON_SIDE == {"left": "a", "right": "b"}


def test_available_outputs_are_checked_exactly(binding_dir):
    path = _write(binding_dir, GOOD)
    load_binding(path, available_outputs={
        "hand/right", "gripper/left/openness", "accessory/suction/valve"})
    with pytest.raises(BindingError) as err:
        load_binding(path, available_outputs={"hand/right", "gripper/left"})
    assert [p for p in err.value.problems if "has no output" in p] == [
        "[[bind]] #2 (pedal/#0 -> gripper/left/openness): the robot has no output "
        "'gripper/left/openness'",
        "[[bind]] #3 (pedal/#1 -> accessory/suction/valve): the robot has no output "
        "'accessory/suction/valve'"]


def test_two_binds_to_the_same_output_are_refused_naming_both_froms():
    problems = _problems(_doc([
        {"from": "glove", "to": "hand/right"},
        {"from": "glove/index_mp_pitch", "to": "hand/right/index_flex"},
    ]))
    assert problems == ["two binds drive the same output: glove -> hand/right and "
                        "glove/index_mp_pitch -> hand/right/index_flex"]
    problems = _problems(_doc([
        {"from": "pedal/#0", "to": "gripper/left/openness"},
        {"from": "pedal/#1", "to": "gripper/left/openness"},
    ]))
    assert len(problems) == 1 and "pedal/#0" in problems[0] and "pedal/#1" in problems[0]


def test_a_shared_prefix_that_is_not_a_path_prefix_is_not_a_conflict():
    parse_binding(_doc([
        {"from": "pedal/#0", "to": "gripper/left/open"},
        {"from": "pedal/#1", "to": "gripper/left/openness"},
    ]))


def test_an_unknown_source_device_is_refused():
    assert _problems(_doc([{"from": "nobody/#0", "to": "gripper/left"}])) == [
        "[[bind]] #1 (nobody/#0 -> gripper/left): device 'nobody' is not a [[source]]"]


def test_a_zero_span_range_is_refused():
    (p,) = _problems(_doc([{"from": "pedal/#0", "to": "gripper/left",
                            "range": {"lo": 0.4, "hi": 0.4}}]))
    assert "range" in p and "non-zero span" in p


def test_every_problem_is_reported_at_once():
    problems = _problems({"schema": "omakase.binding.v0", "source": [], "bind": [
        {"from": "a/#0", "to": "gripper/left", "range": {"lo": 1, "hi": 1}},
        {"from": "a/#1", "to": "gripper/left"},
    ]})
    assert len(problems) == 5   # schema, 2 unknown sources, span, one duplicate pair
    assert any("schema" in p for p in problems)


@pytest.mark.parametrize("bind,match", [
    ({"from": "pedal", "to": "gripper/left"}, "binds a whole hand"),
    ({"from": "glove/index_mp_pitch", "to": "hand/right"}, "not to one channel"),
    ({"from": "pedal", "to": "hand/left"}, "a whole hand binds to a teleop_gloves"),
    ({"from": "glove", "to": "hand/middle"}, "side must be one of"),
    ({"from": "glove", "to": "hand/right", "retarget": "nobody/hand9"}, "no hand map"),
    ({"from": "glove", "to": "hand/right", "retarget": "dh116s"}, "<maker>/<model>"),
    ({"from": "glove", "to": "hand/right", "range": {"lo": 0, "hi": 1}}, "no range or map"),
    ({"from": "pedal/#0", "to": "gripper/left", "retarget": "leadshine/dh116s"},
     "whole-hand bind only"),
    ({"from": "pedal/#0", "to": "gripper/left", "map": {"0": 1},
      "range": {"lo": 0, "hi": 1}}, "not both"),
    ({"from": "pedal/#0", "to": "accessory/suction"}, "accessory/<name>/<channel>"),
    ({"from": "pedal/#0", "to": "gripper/left", "ragne": {}}, "unknown key"),
    ({"from": "pedal/ #0", "to": "gripper/left"}, "from must be"),
    ({"from": "pedal/#0", "to": "gripper"}, "to must be an output path"),
    ({"from": "glove", "to": "hand/right", "calibration": "missing.json"}, "calibration"),
])
def test_malformed_binds_are_refused(tmp_path, bind, match):
    problems = _problems(_doc([bind]), base_dir=tmp_path)
    assert any(match in p for p in problems), problems


def test_a_source_declared_twice_is_refused():
    problems = _problems(_doc([], sources=[
        {"device": "g", "family": "teleop_gloves.pose_stream.v1"},
        {"device": "g", "family": "teleop_gloves.pose_stream.v1"}]))
    assert problems == ["[[source]] #2: device 'g' is declared twice"]


def test_invalid_toml_is_a_binding_error(tmp_path):
    with pytest.raises(BindingError, match="not valid TOML"):
        load_binding(_write(tmp_path, "schema = \n"))


@pytest.mark.parametrize("device", ["omakase@5555", "udcap-right", "litchibot:right",
                                    "glove.v2_left", "omakase@5555+2"])
def test_device_ids_as_drivers_publish_them(device):
    """The UDCAP glove driver's device_id is role@udp_port, e.g. omakase@5555."""
    b = parse_binding({"schema": BINDING_SCHEMA,
                       "source": [{"device": device,
                                   "family": "teleop_gloves.pose_stream.v1"}],
                       "bind": [{"from": device, "to": "hand/right"},
                                {"from": f"{device}/index_mp_pitch",
                                 "to": "gripper/left"}]})
    assert [x.device for x in b.binds] == [device, device]
    assert b.binds[1].channel == "index_mp_pitch"


@pytest.mark.parametrize("device", ["a/b", "a#1", "omakase @5555", "", "x\ty"])
def test_device_ids_with_separators_or_whitespace_are_refused(device):
    problems = _problems({"schema": BINDING_SCHEMA, "bind": [],
                          "source": [{"device": device,
                                      "family": "teleop_gloves.pose_stream.v1"}]})
    assert any("device must be" in p for p in problems)


def test_from_with_a_session_suffix_is_refused():
    problems = _problems(_doc([{"from": "glove#abc", "to": "hand/right"}]))
    assert any("from must be" in p for p in problems)
