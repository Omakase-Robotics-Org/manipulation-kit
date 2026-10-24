"""``omakase.binding.v2``: sources, derived inputs, triggers and the catalog."""
import copy
from pathlib import Path

import pytest

from manipulation_kit.binding import (
    BINDING_SCHEMA,
    BINDING_SCHEMA_V2,
    CATALOG,
    CONTROLLERS_FAMILY,
    BindingError,
    load_binding,
    parse_binding,
)

EXAMPLE = (Path(__file__).resolve().parents[2] / "docs" / "binding-examples"
           / "v2-pedal-ker-udcap.toml")

CONTROLS = "teleop_controls.event_stream.v1"
LEADERS = "teleop_leaders.joint_stream.v1"

SOURCES = [
    {"device": "pedal", "family": CONTROLS},
    {"device": "box", "family": CONTROLS},
    {"device": "glove", "family": CONTROLS, "companion": "sticks",
     "sides": {"left": [0, 1], "right": [2, 3]}},
    {"device": "sticks", "family": CONTROLLERS_FAMILY},
    {"device": "pad", "family": CONTROLLERS_FAMILY},
    {"device": "ker", "family": LEADERS},
    {"device": "hand", "family": "teleop_gloves.pose_stream.v1"},
]
INPUTS = [
    {"name": "lgrip", "from": "ker/left_gripper", "threshold": {"on": 0.8, "off": 0.6}},
    {"name": "rgrip", "from": "ker/right_gripper", "threshold": {"on": 0.8, "off": 0.6}},
]


def doc(*triggers, sources=None, inputs=INPUTS, binds=()):
    d = {"schema": BINDING_SCHEMA_V2, "source": copy.deepcopy(sources or SOURCES),
         "input": copy.deepcopy(list(inputs)), "trigger": list(triggers)}
    if binds:
        d["bind"] = list(binds)
    return d


def trig(frm="pedal/#0", on="tap", do="engage.toggle", **kw):
    t = {"from": frm, "on": on, "do": do}
    if do.startswith("engage.") and "args" not in kw:
        kw["args"] = {"arms": "both"}
    t.update(kw)
    return t


def problems_of(document):
    with pytest.raises(BindingError) as err:
        parse_binding(document)
    return err.value.problems


def refused(document, needle):
    ps = problems_of(document)
    assert any(needle in p for p in ps), (needle, ps)
    return ps


def test_the_design_example_parses():
    b = load_binding(EXAMPLE)
    assert b.schema == BINDING_SCHEMA_V2
    assert [s.device for s in b.sources] == ["pedal", "udcap-buttons", "udcap-sticks",
                                             "ker-0"]
    glove = b.source("udcap-buttons")
    assert glove.endpoint == "127.0.0.1:5582" and glove.companion == "udcap-sticks"
    assert glove.sides == {"left": (0, 1, 2, 3), "right": (4, 5, 6, 7)}
    assert [i.name for i in b.inputs] == ["ker-left-grip", "ker-right-grip"]
    assert (b.input("ker-left-grip").on, b.input("ker-left-grip").off) == (0.8, 0.6)
    assert [t.do for t in b.triggers] == [
        "engage.toggle", "home.run", "engage.start", "engage.start", "engage.stop",
        "base.drive"]
    drive = b.triggers[-1]
    assert drive.args == {
        "linear": "udcap-sticks/left_stick_y", "angular": "udcap-sticks/left_stick_x",
        "invert_linear": True, "invert_angular": True, "deadband": 0.15,
        "max_linear_m_s": 0.25, "max_angular_rad_s": 0.5}
    one_arm = b.triggers[2]
    assert one_arm.with_ == ("ker-left-grip",) and one_arm.modifiers == "during"
    assert one_arm.hold_s == 2.0 and one_arm.args == {"arms": "right"}
    assert len(b.binds) == 2


def test_v1_files_still_parse_and_have_no_triggers():
    b = parse_binding({"schema": BINDING_SCHEMA, "source": SOURCES[:1]})
    assert b.schema == BINDING_SCHEMA and b.inputs == () and b.triggers == ()
    assert b.source("pedal").endpoint is None and b.source("pedal").sides is None


def test_v1_refuses_the_v2_keys():
    ps = problems_of({"schema": BINDING_SCHEMA,
                      "source": [{"device": "a", "family": CONTROLS, "endpoint": "h:1"}],
                      "trigger": []})
    assert any("unknown key(s) ['endpoint']" in p for p in ps)
    assert any("unknown top-level key(s) ['trigger']" in p for p in ps)


def test_v2_still_binds_hands_arms_grippers():
    b = parse_binding(doc(binds=[
        {"from": "ker", "to": "arm/left"},
        {"from": "pedal/#1", "to": "gripper/left/openness",
         "range": {"lo": 0.0, "hi": 1.0}}]))
    assert [x.to for x in b.binds] == ["arm/left", "gripper/left/openness"]


def test_v2_refuses_an_action_bind():
    refused(doc(binds=[{"from": "pedal/#0", "to": "action/engage",
                        "map": {"tap": "toggle"}}]), "is run by a [[trigger]]")


def test_unknown_keys_are_refused():
    refused(doc(trig(extra=1)), "unknown key(s) ['extra']")
    d = doc(trig())
    d["input"][0]["colour"] = "red"
    refused(d, "unknown key(s) ['colour']")


# ---- [[source]] ----

@pytest.mark.parametrize("endpoint", ["localhost", "h:0", "h:65536", "h:x", ":5", 5,
                                      "a b:5"])
def test_a_bad_endpoint_is_refused(endpoint):
    s = copy.deepcopy(SOURCES)
    s[0]["endpoint"] = endpoint
    refused(doc(sources=s), "endpoint must be")


@pytest.mark.parametrize("endpoint", ["127.0.0.1:1", "robot.local:65535", "[::1]:5582"])
def test_a_good_endpoint_is_accepted(endpoint):
    s = copy.deepcopy(SOURCES)
    s[0]["endpoint"] = endpoint
    assert parse_binding(doc(sources=s)).source("pedal").endpoint == endpoint


def test_companion_needs_sides():
    s = copy.deepcopy(SOURCES)
    del s[2]["sides"]
    refused(doc(sources=s), "companion needs sides")


def test_companion_must_be_a_controllers_source():
    s = copy.deepcopy(SOURCES)
    s[2]["companion"] = "box"
    refused(doc(sources=s), "must be a teleop_controllers.input_stream.v1 source")
    s[2]["companion"] = "nobody"
    refused(doc(sources=s), "is not a [[source]]")


def test_sides_only_on_a_controls_source():
    s = copy.deepcopy(SOURCES)
    s[5]["sides"] = {"left": [0]}
    refused(doc(sources=s), "sides applies to a teleop_controls.event_stream.v1 source")


@pytest.mark.parametrize("sides", [
    {"left": [0, 1], "right": [1, 2]},      # not disjoint
    {"left": [-1]}, {"left": []}, {"left": [0, 0]}, {"left": [True]},
    {"middle": [0]}, {}, [0, 1]])
def test_bad_sides_are_refused(sides):
    s = copy.deepcopy(SOURCES)
    s[2]["sides"] = sides
    problems_of(doc(sources=s))


# ---- [[input]] ----

def test_input_names_are_unique_and_not_a_device_id():
    refused(doc(inputs=[INPUTS[0], INPUTS[0]]), "used twice")
    clash = dict(INPUTS[0], name="pedal")
    refused(doc(inputs=[clash]), "collides with the device id")


@pytest.mark.parametrize("src, needle", [
    ("hand/index_mp_pitch", "leaders or controls source"),
    ("sticks/left_stick_y", "leaders or controls source"),
    ("ker", "must be '<device>/<channel>'"),
    ("nobody/x", "must be '<device>/<channel>'")])
def test_input_from_is_a_leaders_or_controls_channel(src, needle):
    refused(doc(inputs=[dict(INPUTS[0], **{"from": src})]), needle)


@pytest.mark.parametrize("thr", [
    {"on": 0.6, "off": 0.6}, {"on": 0.5, "off": 0.6}, {"on": 1.1, "off": 0.5},
    {"on": 0.8, "off": -0.1}])
def test_threshold_needs_off_below_on_within_0_1(thr):
    refused(doc(inputs=[dict(INPUTS[0], threshold=thr)]), "0 <= off < on <= 1")


def test_threshold_shape():
    refused(doc(inputs=[dict(INPUTS[0], threshold={"on": 0.8})]), "threshold must be")
    refused(doc(inputs=[dict(INPUTS[0], threshold={"on": 0.8, "off": 0.6, "x": 1})]),
            "threshold must be")
    assert parse_binding(doc(inputs=[dict(INPUTS[0], threshold={"on": 1, "off": 0})]))


# ---- [[trigger]] shape ----

def test_from_must_resolve():
    refused(doc(trig(frm="nobody/#0")), "must be '<device>/<channel>' of a [[source]]")
    refused(doc(trig(frm="ghost")), "neither an [[input]] name")
    refused(doc(trig(frm="hand/index_mp_pitch", on="press", do="base.stop")),
            "controls, controllers or leaders")


def test_an_input_name_can_be_the_key():
    b = parse_binding(doc(trig(frm="lgrip", on="tap", do="recording.toggle")))
    assert b.triggers[0].from_ == "lgrip"


def test_with_must_be_non_empty_and_unique_and_not_the_key():
    refused(doc(trig(**{"with": []}, modifiers="before")), "non-empty list")
    refused(doc(trig(**{"with": ["lgrip", "lgrip"]}, modifiers="before")),
            "a modifier twice")
    refused(doc(trig(**{"with": ["pedal/#0"]}, modifiers="before")), "the key itself")


def test_modifier_that_is_the_key_through_an_input():
    refused(doc(trig(frm="ker/left_gripper", on="tap", do="recording.toggle",
                     **{"with": ["lgrip"]}, modifiers="before")), "the key itself")


def test_modifiers_value_and_hold_s_rules():
    refused(doc(trig(**{"with": ["lgrip"]}, modifiers="after")), "modifiers must be one of")
    refused(doc(trig(on="hold", do="engage.start")), "needs hold_s")
    refused(doc(trig(on="hold", do="engage.start", hold_s=0)), "positive")
    refused(doc(trig(on="tap", do="engage.start", hold_s=1.0)), "applies to on = 'hold'")
    refused(doc(trig(on="double")), "on must be one of")
    refused(doc(trig(do="engage.launch")), "unknown action")


def test_a_trigger_needs_its_required_keys():
    refused(doc({"from": "pedal/#0", "on": "tap"}), "do is required")


# ---- the catalog ----

def test_catalog_rows():
    assert list(CATALOG) == [
        "engage.start", "engage.stop", "engage.toggle", "home.run", "recording.start",
        "recording.stop", "recording.toggle", "recording.abort", "base.drive", "base.stop"]
    assert CATALOG["base.drive"].type == "held"
    assert CATALOG["engage.toggle"].gestures == ("tap",)
    assert CATALOG["engage.toggle"].implicit_press_stop
    assert CATALOG["engage.stop"].safety_class == "stop"
    assert not CATALOG["engage.stop"].modifiers_allowed
    assert CATALOG["recording.abort"].gestures == (
        "press", "release", "tap", "hold", "while")


@pytest.mark.parametrize("do, on, extra", [
    ("engage.stop", "tap", {}), ("engage.stop", "hold", {"hold_s": 1.0}),
    ("base.stop", "release", {}),
    ("engage.start", "press", {}), ("engage.start", "while", {}),
    ("engage.toggle", "hold", {"hold_s": 1.0}), ("engage.toggle", "press", {}),
    ("home.run", "tap", {}), ("home.run", "press", {}),
    ("base.drive", "tap", {}), ("base.drive", "hold", {"hold_s": 1.0}),
])
def test_a_gesture_outside_the_class_is_refused(do, on, extra):
    t = trig(on=on, do=do, **extra)
    if do == "base.drive":
        t["args"] = {"linear": "sticks/y", "angular": "sticks/x"}
    refused(doc(t), "allows only")


@pytest.mark.parametrize("do", ["engage.stop", "base.stop"])
def test_stop_takes_no_modifiers(do):
    refused(doc(trig(on="press", do=do, **{"with": ["lgrip"]}, modifiers="before")),
            "takes no modifiers")


def test_continuous_motion_takes_no_modifiers():
    t = trig(frm="glove/#0", on="while", do="base.drive", **{"with": ["lgrip"]},
             modifiers="before", args={"linear": "sticks/y", "angular": "sticks/x"})
    refused(doc(t), "takes no modifiers")


def test_large_motion_hold_has_a_floor():
    refused(doc(trig(on="hold", do="home.run", hold_s=0.79)), "at least 0.8 s")
    assert parse_binding(doc(trig(on="hold", do="home.run", hold_s=0.8)))


@pytest.mark.parametrize("on, extra", [("press", {}), ("release", {}), ("tap", {}),
                                       ("hold", {"hold_s": 0.2}), ("while", {})])
def test_no_motion_takes_any_gesture(on, extra):
    assert parse_binding(doc(trig(on=on, do="recording.start", **extra)))


@pytest.mark.parametrize("args", [None, {}, {"arms": "all"}, {"arms": "left", "x": 1}])
def test_engage_needs_arms_both_left_or_right(args):
    t = {"from": "pedal/#0", "on": "tap", "do": "engage.start"}
    if args is not None:
        t["args"] = args
    problems_of(doc(t))


def test_engage_arms_values():
    for arms in ("both", "left", "right"):
        assert parse_binding(doc(trig(do="engage.start", args={"arms": arms})))


@pytest.mark.parametrize("do", ["home.run", "recording.start", "base.stop"])
def test_actions_without_arguments_refuse_them(do):
    on = "hold" if do == "home.run" else "tap" if do != "base.stop" else "press"
    extra = {"hold_s": 5.0} if do == "home.run" else {}
    refused(doc(trig(on=on, do=do, args={"arms": "both"}, **extra)), "takes no arguments")


DRIVE = {"linear": "sticks/left_stick_y", "angular": "sticks/left_stick_x"}


def drive(frm="glove/#0", **args):
    return trig(frm=frm, on="while", do="base.drive", args={**DRIVE, **args})


def test_base_drive_defaults_and_overrides():
    (t,) = parse_binding(doc(drive())).triggers
    assert t.args == {**DRIVE, "invert_linear": False, "invert_angular": False,
                      "deadband": 0.15, "max_linear_m_s": 0.25, "max_angular_rad_s": 0.5}
    (t,) = parse_binding(doc(drive(deadband=0, max_linear_m_s=3, max_angular_rad_s=9,
                                    invert_linear=True))).triggers
    assert (t.args["deadband"], t.args["max_linear_m_s"],
            t.args["max_angular_rad_s"], t.args["invert_linear"]) == (0, 3, 9, True)


@pytest.mark.parametrize("args, needle", [
    ({"deadband": 1.0}, "deadband"), ({"deadband": -0.1}, "deadband"),
    ({"deadband": True}, "deadband"),
    ({"max_linear_m_s": 0}, "positive"), ({"max_angular_rad_s": -1}, "positive"),
    ({"invert_linear": 1}, "true or false"), ({"bogus": 1}, "not ['bogus']"),
    ({"linear": "sticks/#1"}, "axis address"), ({"linear": "sticks"}, "axis address"),
    ({"angular": "nobody/x"}, "axis address"),
    ({"linear": "pedal/#0"}, "axis address"),
    ({"angular": "ker/left_gripper"}, "must be on a teleop_controllers"),
])
def test_base_drive_argument_checks(args, needle):
    refused(doc(drive(**args)), needle)


def test_base_drive_needs_both_axes():
    t = trig(frm="glove/#0", on="while", do="base.drive", args={"linear": "sticks/y"})
    refused(doc(t), "needs the argument 'angular'")


def test_base_drive_axes_follow_the_deadmans_companion():
    # the glove has companion "sticks": axes on another controllers source are refused
    refused(doc(drive(linear="pad/y", angular="pad/x")), "has companion 'sticks'")
    # no companion: any controllers source will do
    assert parse_binding(doc(drive(frm="box/#0", linear="pad/y", angular="pad/x")))
    # a deadman that is an input is judged by the input's own device
    inputs = INPUTS + [{"name": "glove-a", "from": "glove/#0",
                        "threshold": {"on": 0.8, "off": 0.6}}]
    refused(doc(drive(frm="glove-a", linear="pad/y", angular="pad/x"), inputs=inputs),
            "has companion 'sticks'")


# ---- rules over several triggers ----

def test_duplicate_key_modifiers_gesture_hold():
    refused(doc(trig(on="tap", do="recording.start"), trig(on="tap", do="recording.stop")),
            "two triggers on the same gesture")
    refused(doc(trig(on="hold", do="home.run", hold_s=5.0),
                trig(on="hold", do="recording.abort", hold_s=5.0)),
            "hold_s 5.0")


def test_same_gesture_different_modifier_sets_is_fine():
    t = dict(on="hold", do="engage.start", hold_s=2.0, modifiers="during")
    assert parse_binding(doc(
        trig(**t, **{"with": ["lgrip"]}, args={"arms": "right"}),
        trig(**t, **{"with": ["rgrip"]}, args={"arms": "left"})))


def test_modifier_set_order_does_not_matter():
    t = dict(on="tap", do="recording.toggle", modifiers="before")
    refused(doc(trig(**t, **{"with": ["lgrip", "rgrip"]}),
                trig(**t, **{"with": ["rgrip", "lgrip"]})), "same gesture")


def test_a_modified_key_has_one_hold():
    t = dict(modifiers="during", **{"with": ["lgrip"]})
    refused(doc(trig(on="hold", do="engage.start", hold_s=1.0, **t),
                trig(on="hold", do="recording.abort", hold_s=3.0, **t)),
            "one hold only")


def test_an_unmodified_key_may_have_stages():
    assert parse_binding(doc(trig(on="hold", do="recording.abort", hold_s=1.0),
                             trig(on="hold", do="home.run", hold_s=5.0)))


def test_modified_triggers_must_say_modifiers():
    refused(doc(trig(**{"with": ["lgrip"]})), "must say modifiers")


def test_modifiers_without_with_is_refused():
    refused(doc(trig(modifiers="before")), "no 'with'")


def test_before_and_during_do_not_mix_on_one_key():
    refused(doc(trig(on="tap", do="engage.start", **{"with": ["lgrip"]}, modifiers="before"),
                trig(on="tap", do="engage.start", **{"with": ["rgrip"]}, modifiers="during",
                     args={"arms": "left"})),
            "mixes 'before' and 'during'")


def test_while_does_not_share_its_key_and_modifiers():
    refused(doc(drive(), trig(frm="glove/#0", on="tap", do="recording.toggle")),
            "'while' trigger cannot share")
    # a different modifier set is a different key
    assert parse_binding(doc(
        drive(), trig(frm="glove/#0", on="tap", do="recording.toggle",
                      **{"with": ["lgrip"]}, modifiers="before")))


def test_a_stop_key_does_not_also_start_motion():
    stop = trig(frm="glove/#1", on="press", do="engage.stop")
    refused(doc(stop, trig(frm="glove/#1", on="tap", do="engage.start")),
            "a key that stops must not also start motion")
    refused(doc(stop, trig(frm="glove/#1", on="hold", do="home.run", hold_s=5.0)),
            "a key that stops must not also start motion")
    # a modified start still shares the key
    refused(doc(stop, trig(frm="glove/#1", on="tap", do="engage.start",
                           **{"with": ["lgrip"]}, modifiers="before")),
            "a key that stops must not also start motion")
    # no_motion on the same key is fine
    assert parse_binding(doc(stop, trig(frm="glove/#1", on="tap", do="recording.toggle")))


def test_engage_toggle_is_exempt_from_the_stop_key_rule():
    assert parse_binding(doc(trig(on="tap", do="engage.toggle"),
                             trig(on="hold", do="home.run", hold_s=5.0)))


def test_a_modifier_may_not_be_a_motion_hold_key():
    refused(doc(trig(frm="lgrip", on="hold", do="home.run", hold_s=2.0),
                trig(on="tap", do="engage.start", **{"with": ["lgrip"]}, modifiers="before")),
            "also the key of a hold trigger that starts motion")
    # a modifier that is only the key of a tap is fine
    assert parse_binding(doc(
        trig(frm="lgrip", on="tap", do="recording.toggle"),
        trig(on="tap", do="engage.start", **{"with": ["lgrip"]}, modifiers="before")))


def test_a_button_modifier_that_is_a_motion_hold_key_through_its_address():
    refused(doc(trig(frm="box/#0", on="hold", do="home.run", hold_s=2.0),
                trig(on="tap", do="engage.start", **{"with": ["box/#0"]}, modifiers="before")),
            "also the key of a hold trigger that starts motion")


@pytest.mark.parametrize("mod", ["ker/left_gripper", "sticks/left_stick_y", "pad/a"])
def test_raw_analog_modifiers_are_refused(mod):
    refused(doc(trig(on="tap", do="engage.start", **{"with": [mod]}, modifiers="before")),
            "wrap it in an [[input]]")


def test_a_controls_button_and_an_input_are_valid_modifiers():
    assert parse_binding(doc(trig(on="tap", do="engage.start",
                                  **{"with": ["box/#2", "lgrip"]}, modifiers="before")))


def test_unknown_modifier_is_refused():
    refused(doc(trig(on="tap", do="engage.start", **{"with": ["ghost"]}, modifiers="before")),
            "neither an [[input]] name")


def test_every_problem_is_listed_at_once():
    ps = problems_of(doc(trig(do="engage.launch"), trig(frm="ghost"),
                         trig(frm="pedal/#1", on="tap", do="engage.stop")))
    assert len(ps) >= 3
