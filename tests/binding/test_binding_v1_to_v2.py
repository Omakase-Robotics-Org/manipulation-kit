"""A v1 binding's action binds as v2 inputs and triggers."""
import pytest

from manipulation_kit.binding import (
    BINDING_SCHEMA,
    BindingError,
    DerivedInput,
    Trigger,
    parse_binding,
    v1_action_triggers,
)

CONTROLS = "teleop_controls.event_stream.v1"
LEADERS = "teleop_leaders.joint_stream.v1"
SOURCES = [{"device": "pedal", "family": CONTROLS},
           {"device": "ker", "family": LEADERS}]

PEDAL = [
    {"from": "pedal/#0", "to": "action/engage", "map": {"tap": "toggle"}},
    {"from": "pedal/#0", "to": "action/home", "map": {"hold": "run"}, "hold_s": 5.0},
]


def v1(*binds):
    return parse_binding({"schema": BINDING_SCHEMA, "source": SOURCES, "bind": list(binds)})


def one_arm(channel):
    return {"from": f"ker/{channel}", "to": "action/engage", "map": {"hold": "one_arm"}}


def test_the_pedal_engage_and_home():
    inputs, triggers = v1_action_triggers(v1(*PEDAL))
    assert inputs == ()
    assert triggers == (
        Trigger("pedal/#0", "engage.toggle", "tap", args={"arms": "both"}),
        Trigger("pedal/#0", "home.run", "hold", hold_s=5.0))


def test_recording_verbs_map_one_to_one_per_gesture():
    b = v1({"from": "pedal/#1", "to": "action/recording",
            "map": {"tap": "toggle", "hold": "abort"}, "hold_s": 0.8},
           {"from": "pedal/#2", "to": "action/recording",
            "map": {"press": "start", "release": "stop"}})
    _, triggers = v1_action_triggers(b)
    assert [(t.from_, t.on, t.do, t.hold_s) for t in triggers] == [
        ("pedal/#1", "tap", "recording.toggle", None),
        ("pedal/#1", "hold", "recording.abort", 0.8),
        ("pedal/#2", "press", "recording.start", None),
        ("pedal/#2", "release", "recording.stop", None)]


def test_engage_start_stop_verbs():
    b = v1({"from": "pedal/#3", "to": "action/engage", "map": {"tap": "start"}},
           {"from": "pedal/#4", "to": "action/engage", "map": {"press": "stop"}})
    _, triggers = v1_action_triggers(b)
    assert [(t.on, t.do, t.args) for t in triggers] == [
        ("tap", "engage.start", {"arms": "both"}),
        ("press", "engage.stop", {"arms": "both"})]


@pytest.mark.parametrize("channel, other", [("left_gripper", "right"),
                                            ("right_gripper", "left")])
def test_one_arm_keeps_the_gripped_side_still_and_engages_the_other(channel, other):
    side = channel.split("_")[0]
    inputs, triggers = v1_action_triggers(v1(*PEDAL, one_arm(channel)))
    assert inputs == (DerivedInput(f"ker-{side}-grip", f"ker/{channel}", 0.8, 0.6),)
    assert triggers[-1] == Trigger(
        "pedal/#0", "engage.start", "hold", with_=(f"ker-{side}-grip",),
        modifiers="during", hold_s=2.0, args={"arms": other})
    assert len(triggers) == 3


def test_both_one_arm_sides_together_form_the_design_example():
    inputs, triggers = v1_action_triggers(
        v1(*PEDAL, one_arm("left_gripper"), one_arm("right_gripper")))
    assert [i.name for i in inputs] == ["ker-left-grip", "ker-right-grip"]
    assert [(t.with_, t.args["arms"]) for t in triggers[2:]] == [
        (("ker-left-grip",), "right"), (("ker-right-grip",), "left")]


def test_the_converted_objects_parse_as_a_v2_file():
    inputs, triggers = v1_action_triggers(
        v1(*PEDAL, one_arm("left_gripper"), one_arm("right_gripper")))
    doc = {"schema": "omakase.binding.v2", "source": SOURCES,
           "input": [{"name": i.name, "from": i.from_,
                      "threshold": {"on": i.on, "off": i.off}} for i in inputs],
           "trigger": [{"from": t.from_, "on": t.on, "do": t.do, "args": t.args,
                        **({"with": list(t.with_), "modifiers": t.modifiers}
                           if t.with_ else {}),
                        **({"hold_s": t.hold_s} if t.hold_s else {})} for t in triggers]}
    assert parse_binding(doc).triggers == triggers


def test_one_arm_without_an_engage_switch_is_refused():
    with pytest.raises(BindingError) as err:
        v1_action_triggers(v1(one_arm("left_gripper")))
    assert any("needs an engage switch" in p for p in err.value.problems)


@pytest.mark.parametrize("channel", ["left_trigger", "left_gripper_extra"])
def test_one_arm_on_another_channel_is_refused(channel):
    with pytest.raises(BindingError) as err:
        v1_action_triggers(v1(*PEDAL, one_arm(channel)))
    assert any("<left|right>_gripper" in p for p in err.value.problems)


def test_a_home_hold_under_the_floor_is_refused():
    b = v1({"from": "pedal/#0", "to": "action/home", "map": {"hold": "run"}})  # 0.5 s
    with pytest.raises(BindingError) as err:
        v1_action_triggers(b)
    assert any("at least 0.8 s" in p for p in err.value.problems)


def test_a_binding_without_actions_converts_to_nothing():
    assert v1_action_triggers(v1({"from": "ker", "to": "arm/left"})) == ((), ())
