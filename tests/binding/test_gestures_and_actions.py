"""Gesture maps on controls sources, and action/<name> outputs."""
import pytest

from manipulation_kit.binding import (
    ACTIONS,
    BINDING_SCHEMA,
    DEFAULT_HOLD_S,
    GESTURES,
    BindingError,
    parse_binding,
)

SOURCES = [
    {"device": "pedal", "family": "teleop_controls.event_stream.v1", "instance": 0},
    {"device": "glove", "family": "teleop_gloves.pose_stream.v1"},
]


def _doc(*binds):
    return {"schema": BINDING_SCHEMA, "source": SOURCES, "bind": list(binds)}


def _problems(*binds, **kw):
    with pytest.raises(BindingError) as err:
        parse_binding(_doc(*binds), **kw)
    return err.value.problems


def test_the_recording_pedal_binds():
    (b,) = parse_binding(_doc({"from": "pedal/#0", "to": "action/recording",
                               "map": {"tap": "toggle", "hold": "abort"}})).binds
    assert b.gestures and b.map == {"tap": "toggle", "hold": "abort"}
    assert b.hold_s == DEFAULT_HOLD_S == 0.5
    assert b.side is None
    assert GESTURES == ("press", "release", "tap", "hold")
    assert ACTIONS["recording"] == ("start", "stop", "toggle", "abort")


def test_hold_s_is_per_bind_and_only_where_it_splits_something():
    (b,) = parse_binding(_doc({"from": "pedal/#0", "to": "action/recording",
                               "map": {"hold": "abort"}, "hold_s": 1})).binds
    assert b.hold_s == 1.0
    (b,) = parse_binding(_doc({"from": "pedal/#0", "to": "gripper/left",
                               "map": {"press": "close", "release": "open"}})).binds
    assert b.gestures and b.hold_s is None
    (p,) = _problems({"from": "pedal/#0", "to": "gripper/left",
                      "map": {"press": "close"}, "hold_s": 0.5})
    assert "hold_s applies to a gesture map with tap or hold" in p
    for bad in (0, -1, True, "0.5"):
        (p,) = _problems({"from": "pedal/#0", "to": "action/recording",
                          "map": {"tap": "toggle"}, "hold_s": bad})
        assert "number of seconds > 0" in p


def test_discrete_value_maps_still_work():
    (b,) = parse_binding(_doc({"from": "pedal/#0", "to": "accessory/suction/valve",
                               "map": {"0": 0, "2": 1}})).binds
    assert not b.gestures and b.map == {0: 0, 2: 1} and b.hold_s is None


def test_unknown_gestures_and_mixed_maps_are_refused():
    (p,) = _problems({"from": "pedal/#0", "to": "gripper/left",
                      "map": {"tap": "open", "long_press": "close"}})
    assert "['long_press'] are neither a discrete value" in p
    (p,) = _problems({"from": "pedal/#0", "to": "gripper/left",
                      "map": {"tap": "open", "2": "close"}})
    assert "either all gestures or all discrete values" in p


def test_gestures_need_a_controls_source():
    problems = _problems({"from": "glove/index_mp_pitch", "to": "gripper/left",
                          "map": {"tap": "open"}})
    assert any("gestures need a teleop_controls.event_stream.v1 source" in p
               for p in problems)


def test_action_outputs_are_checked_against_the_action_table_not_the_robot():
    parse_binding(_doc({"from": "pedal/#0", "to": "action/recording",
                        "map": {"tap": "start"}}), available_outputs={"gripper/left"})
    (p,) = _problems({"from": "pedal/#0", "to": "action/teleport",
                      "map": {"tap": "start"}})
    assert "'action/<name>' with <name> one of ['recording']" in p
    (p,) = _problems({"from": "pedal/#0", "to": "action/recording",
                      "map": {"tap": "pause"}})
    assert "['pause'] are not verbs of action/recording" in p
    (p,) = _problems({"from": "pedal/#0", "to": "action/recording",
                      "range": {"lo": 0, "hi": 1}})
    assert "driven by a map to its verbs" in p
    (p,) = _problems({"from": "pedal/#0", "to": "action/recording/now",
                      "map": {"tap": "start"}})
    assert "'action/<name>'" in p


def test_two_binds_to_one_action_are_one_output():
    (p,) = _problems(
        {"from": "pedal/#0", "to": "action/recording", "map": {"tap": "toggle"}},
        {"from": "pedal/#1", "to": "action/recording", "map": {"hold": "abort"}})
    assert "two binds drive the same output" in p


def test_the_module_docstring_example_loads(tmp_path):
    """The example in manipulation_kit.binding's docstring is a valid file."""
    import json
    import textwrap

    import manipulation_kit.binding as binding
    from manipulation_kit.binding import load_binding
    from manipulation_kit.gloves import CALIBRATION_SCHEMA

    doc = binding.__doc__
    example = doc[doc.index("::\n") + 3:doc.index("Two kinds of bind")]
    (tmp_path / "operators").mkdir()
    (tmp_path / "operators" / "alice-udcap.json").write_text(
        json.dumps({"schema": CALIBRATION_SCHEMA, "ranges": {}}))
    path = tmp_path / "binding.toml"
    path.write_text(textwrap.dedent(example))
    hand, rec, grip = load_binding(path).binds
    assert hand.retarget_config.joints["thumb_rota1"].lo == 0.12
    assert rec.to == "action/recording" and rec.hold_s == 0.8
    assert grip.range.invert
