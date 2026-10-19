"""Whole-arm binds, the engage/home actions, and gestures on a leader source."""
import pytest

from manipulation_kit.binding import (
    ACTIONS,
    ARM_MODES,
    BINDING_SCHEMA,
    BindingError,
    parse_binding,
)

SOURCES = [
    {"device": "ker-left", "family": "teleop_leaders.joint_stream.v1"},
    {"device": "ker-right", "family": "teleop_leaders.joint_stream.v1"},
    {"device": "pedal", "family": "teleop_controls.event_stream.v1", "instance": 0},
    {"device": "glove", "family": "teleop_gloves.pose_stream.v1"},
]


def _doc(*binds):
    return {"schema": BINDING_SCHEMA, "source": SOURCES, "bind": list(binds)}


def _problems(*binds, **kw):
    with pytest.raises(BindingError) as err:
        parse_binding(_doc(*binds), **kw)
    return err.value.problems


def test_a_leader_binds_a_whole_arm():
    b = parse_binding(_doc({"from": "ker-left", "to": "arm/left"}),
                      available_outputs={"arm/left"})
    (arm,) = b.binds
    assert arm.whole_arm and not arm.whole_hand
    assert arm.kind == "arm" and arm.side == "left" and arm.channel is None
    assert arm.mode is None and arm.map is None and arm.range is None
    assert b.arm_binds() == (arm,) and b.hand_binds() == ()


@pytest.mark.parametrize("mode", ARM_MODES)
def test_mode_is_absolute_or_relative(mode):
    (arm,) = parse_binding(_doc({"from": "ker-right", "to": "arm/right",
                                 "mode": mode})).binds
    assert arm.mode == mode


def test_two_leaders_two_arms_and_a_hand_coexist():
    b = parse_binding(_doc({"from": "ker-left", "to": "arm/left"},
                           {"from": "ker-right", "to": "arm/right"},
                           {"from": "glove", "to": "hand/right"}))
    assert [x.to for x in b.arm_binds()] == ["arm/left", "arm/right"]
    assert [x.to for x in b.hand_binds()] == ["hand/right"]


def test_a_whole_arm_needs_a_leader_source():
    (p,) = _problems({"from": "glove", "to": "arm/left"})
    assert "a whole arm binds to a teleop_leaders.joint_stream.v1 source; " \
           "'glove' is teleop_gloves.pose_stream.v1" in p
    (p,) = _problems({"from": "pedal", "to": "arm/left"})
    assert "a whole arm binds to a teleop_leaders.joint_stream.v1 source" in p


def test_a_leader_does_not_bind_a_whole_hand():
    (p,) = _problems({"from": "ker-left", "to": "hand/left"})
    assert "a whole hand binds to a teleop_gloves.pose_stream.v1 source" in p


def test_an_arm_is_bound_whole_only():
    (p,) = _problems({"from": "ker-left/j1", "to": "arm/left"})
    assert "an arm is bound whole" in p
    (p,) = _problems({"from": "ker-left", "to": "arm/left/j1"})
    assert "an arm is bound whole" in p and "no channel binds" not in p
    problems = _problems({"from": "ker-left/j1", "to": "arm/left/j1",
                          "range": {"lo": 0, "hi": 1}})
    assert any("an arm is bound whole" in p for p in problems)


def test_the_arm_side_and_the_robot_are_checked():
    (p,) = _problems({"from": "ker-left", "to": "arm/middle"})
    assert "side must be one of ('left', 'right'), got 'middle'" in p
    (p,) = _problems({"from": "ker-left", "to": "arm/right"},
                     available_outputs={"arm/left"})
    assert "the robot has no output 'arm/right'" in p


def test_a_bad_mode_is_refused_with_the_choices():
    (p,) = _problems({"from": "ker-left", "to": "arm/left", "mode": "mirror"})
    assert "mode must be one of ('absolute', 'relative'), got 'mirror'" in p
    (p,) = _problems({"from": "ker-left", "to": "arm/left", "mode": True})
    assert "mode must be one of" in p


def test_mode_belongs_to_a_whole_arm_only():
    (p,) = _problems({"from": "glove", "to": "hand/right", "mode": "absolute"})
    assert "mode applies to a whole-arm bind only" in p
    (p,) = _problems({"from": "pedal/#0", "to": "gripper/left/openness",
                      "range": {"lo": 0, "hi": 1}, "mode": "relative"})
    assert "mode applies to a whole-arm bind only" in p


@pytest.mark.parametrize("extra", [
    {"range": {"lo": 0, "hi": 1}},
    {"map": {"tap": "toggle"}},
])
def test_a_whole_arm_takes_no_range_or_map(extra):
    problems = _problems({"from": "ker-left", "to": "arm/left", **extra})
    assert any("a whole-arm bind takes no range or map" in p for p in problems)


@pytest.mark.parametrize("key,value", [
    ("retarget", "robotera/xhand1"),
    ("calibration", "x.json"),
    ("retarget_config", {"ema_alpha": 0.2}),
    ("hold_s", 1.0),
])
def test_hand_and_gesture_keys_are_refused_on_an_arm(key, value):
    problems = _problems({"from": "ker-left", "to": "arm/left", key: value})
    assert any(key in p and "applies to" in p for p in problems), problems


def test_two_binds_to_one_arm_are_one_output():
    (p,) = _problems({"from": "ker-left", "to": "arm/left"},
                     {"from": "ker-right", "to": "arm/left"})
    assert "two binds drive the same output: ker-left -> arm/left and " \
           "ker-right -> arm/left" in p


def test_a_channel_less_bind_to_anything_else_names_both_choices():
    (p,) = _problems({"from": "ker-left", "to": "gripper/left"})
    assert "binds a whole hand or a whole arm" in p
    assert "'hand/<left|right>' or 'arm/<left|right>'" in p


def test_the_engage_and_home_actions_are_declared():
    assert ACTIONS["engage"] == ("start", "stop", "toggle", "one_arm")
    assert ACTIONS["home"] == ("run",)


def test_the_ker_trigger_holds_to_engage_one_arm():
    (b,) = parse_binding(_doc({"from": "ker-left/left_trigger",
                               "to": "action/engage",
                               "map": {"hold": "one_arm"}}),
                         available_outputs=set()).binds
    assert b.gestures and b.map == {"hold": "one_arm"}
    assert b.device == "ker-left" and b.channel == "left_trigger"
    assert b.hold_s == 0.5 and b.side is None and not b.whole_arm


def test_a_leader_start_button_taps_engage():
    (b,) = parse_binding(_doc({"from": "ker-right/start", "to": "action/engage",
                               "map": {"press": "toggle"}})).binds
    assert b.gestures and b.hold_s is None


def test_the_pedal_drives_engage_and_home():
    engage, home = parse_binding(_doc(
        {"from": "pedal/#0", "to": "action/engage", "map": {"tap": "toggle"}},
        {"from": "pedal/#0", "to": "action/home", "map": {"hold": "run"},
         "hold_s": 5.0})).binds
    assert engage.map == {"tap": "toggle"} and engage.hold_s == 0.5
    assert home.map == {"hold": "run"} and home.hold_s == 5.0


def test_action_verbs_are_per_action():
    (p,) = _problems({"from": "pedal/#0", "to": "action/home",
                      "map": {"tap": "toggle"}})
    assert "['toggle'] are not verbs of action/home ('run',)" in p
    (p,) = _problems({"from": "pedal/#0", "to": "action/engage",
                      "map": {"tap": "abort"}})
    assert "['abort'] are not verbs of action/engage" in p
    (p,) = _problems({"from": "pedal/#0", "to": "action/recording",
                      "map": {"tap": "one_arm"}})
    assert "['one_arm'] are not verbs of action/recording" in p


def test_gestures_are_refused_from_a_glove_and_name_both_families():
    (p,) = _problems({"from": "glove/index_mp_pitch", "to": "action/engage",
                      "map": {"hold": "one_arm"}})
    assert "gestures need a teleop_controls.event_stream.v1 or " \
           "teleop_leaders.joint_stream.v1 source; 'glove' is " \
           "teleop_gloves.pose_stream.v1" in p


def test_one_trigger_cannot_mix_gestures_and_values():
    (p,) = _problems({"from": "ker-left/left_trigger", "to": "action/engage",
                      "map": {"hold": "one_arm", "1": "toggle"}})
    assert "either all gestures or all discrete values" in p


def test_every_problem_is_listed_at_once():
    problems = _problems(
        {"from": "glove", "to": "arm/left", "mode": "mirror"},
        {"from": "ker-left", "to": "arm/right", "mode": "fast"},
        {"from": "pedal/#0", "to": "action/home", "map": {"tap": "go"}})
    assert len(problems) == 4
