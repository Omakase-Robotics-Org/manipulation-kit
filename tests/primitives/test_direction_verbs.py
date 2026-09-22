"""Every verb on ``Direction``, and the ONE allowlist between a verb and a model."""

from __future__ import annotations

import math
from dataclasses import fields

import numpy as np

from manipulation_kit.primitives import (NOT_MODEL_BINDABLE, PRIMITIVES,
                                         Approach, Grasp, Lift, Retreat,
                                         decode, direction_doc, domains,
                                         domains_in, tool_schemas)
from manipulation_kit.primitives.types import BAD_ARGUMENT, UNKNOWN_FRAME, Plan
from manipulation_kit.world import ALIASES, TOOL, Direction


def _same_plan(a, b):
    assert isinstance(a, Plan) and isinstance(b, Plan), (a, b)
    assert len(a.waypoints) == len(b.waypoints)
    for wa, wb in zip(a.waypoints, b.waypoints):
        assert wa.label == wb.label
        assert np.array_equal(wa.p, wb.p)
        assert np.array_equal(wa.r.as_quat(), wb.r.as_quat())
    assert len(a.steps) == len(b.steps)
    for sa, sb in zip(a.steps, b.steps):
        assert type(sa) is type(sb)
        if hasattr(sa, "q"):
            assert np.array_equal(sa.q, sb.q)
    assert a.notes == b.notes


def test_a_named_direction_and_its_vector_plan_identically(d1_arm, observe):
    """An alias is data: ``"down"``, ``ALIASES["down"]``, ``Direction((0,0,-2))``
    and the model's ``[0, 0, -1]`` are the same request and the same plan."""
    world = observe(d1_arm)
    by_name = Grasp(object="red_block", side="left", direction="down").plan(world, d1_arm)
    by_alias = Grasp(object="red_block", side="left",
                     direction=ALIASES["down"]).plan(world, d1_arm)
    by_vector = Grasp(object="red_block", side="left",
                      direction=Direction((0.0, 0.0, -2.0))).plan(world, d1_arm)
    decoded = decode("grasp", {"object": "red_block", "side": "left",
                               "direction": [0, 0, -1]}, world)
    assert isinstance(decoded, Grasp)
    by_model = decoded.plan(world, d1_arm)
    for other in (by_alias, by_vector, by_model):
        _same_plan(by_name, other)
    # and a horizontal one, through the object-free Approach path
    fwd = Approach(object="red_block", side="left", direction="forward").plan(world, d1_arm)
    fwd_v = Approach(object="red_block", side="left",
                     direction={"axis": [1, 0, 0], "frame": "base"}).plan(world, d1_arm)
    if isinstance(fwd, Plan):
        _same_plan(fwd, fwd_v)
    else:
        assert fwd.to_json() == fwd_v.to_json()


def test_no_verb_field_reaches_the_model_unless_allowlisted():
    """Every dataclass field is either in the exported schema or in
    ``NOT_MODEL_BINDABLE`` — never silently both or neither (run8: a planner
    knob became a schema field just by being declared)."""
    schemas = {s["name"]: s for s in tool_schemas()}
    assert "policy" in NOT_MODEL_BINDABLE
    for cls in PRIMITIVES:
        names = {f.name for f in fields(cls)}
        exported = set(schemas[cls.name()]["parameters"]["properties"])
        assert exported == names - set(NOT_MODEL_BINDABLE), cls.name()
        assert not exported & set(NOT_MODEL_BINDABLE), cls.name()
        # the deleted names are deleted, not hidden — the planner's roll
        # included (step 3: grasp_geometry.roll_candidates is the one sweep)
        assert not names & {"jaw_turn_deg", "approach", "roll_rad"}, cls.name()
    # the drift gate still holds with a direction in the table
    assert domains() == domains_in(tool_schemas())


def test_decode_rejects_policy_and_roll_rad_from_the_model(d1_arm, observe):
    world = observe(d1_arm, held={"left": "red_block"})
    refused = decode("pour", {"source": "box", "target": "box",
                              "policy": "act:invented"}, world)
    assert not getattr(refused, "ok", True), refused
    assert refused.reason == BAD_ARGUMENT
    assert "policy" in refused.detail
    assert [u.measured.get("argument") for u in refused.unmet] == ["policy"]
    # the Python door stays open for policy, explicitly ...
    call = decode("pour", {"source": "box", "target": "box",
                           "policy": "act:invented"}, world,
                  model_bindable_only=False)
    assert getattr(call, "policy", None) == "act:invented"
    # ... and roll_rad is no longer a field at all, through either door
    for verb in ("grasp", "approach"):
        for door in (True, False):
            refused = decode(verb, {"object": "red_block",
                                    "roll_rad": math.pi / 2}, world,
                             model_bindable_only=door)
            assert not getattr(refused, "ok", True), (verb, door, refused)
            assert refused.reason == BAD_ARGUMENT
            assert "roll_rad" in refused.detail


def test_tool_schemas_expose_direction_as_alias_or_vector(d1_arm, observe):
    world = observe(d1_arm)
    for verb in ("approach", "grasp", "lift", "retreat"):
        prop = next(s for s in tool_schemas(world) if s["name"] == verb
                    )["parameters"]["properties"]["direction"]
        named, free = prop["oneOf"]
        assert named == {"type": "string", "enum": list(ALIASES),
                         "description": "a named direction"}
        assert free["type"] == "object"
        assert free["required"] == ["axis", "frame"]
        assert free["properties"]["axis"]["minItems"] == 3
        frames = free["properties"]["frame"]["enum"]
        assert frames[:2] == ["base", "tool"] and "object:red_block" in frames
    defaults = {s["name"]: s["parameters"]["properties"]["direction"]["default"]
                for s in tool_schemas() if "direction" in s["parameters"]["properties"]}
    assert defaults == {"approach": "down", "grasp": "down", "lift": "up",
                        "retreat": {"axis": [0.0, 0.0, -1.0], "frame": "tool"},
                        "probe": "down", "press": "forward",
                        "handover": "left"}
    # without a world the object frames are a pattern, not an empty enum
    free = next(s for s in tool_schemas() if s["name"] == "grasp"
                )["parameters"]["properties"]["direction"]["oneOf"][1]
    assert "pattern" in free["properties"]["frame"]
    # decode takes both forms, and refuses the deleted names
    for value, expect in (("forward", ALIASES["forward"]),
                          ([1, 0, 0], ALIASES["forward"]),
                          ({"axis": [0, 0, 1], "frame": "tool"},
                           ALIASES["along_tool"])):
        call = decode("approach", {"object": "red_block", "direction": value}, world)
        assert isinstance(call, Approach) and call.direction == expect
    for old in ("top_down", "front", "side_left", "side_right"):
        refused = decode("grasp", {"object": "red_block", "direction": old}, world)
        assert refused.reason == BAD_ARGUMENT, old
    refused = decode("grasp", {"object": "red_block", "approach": "top_down"}, world)
    assert refused.reason == BAD_ARGUMENT and "approach" in refused.detail


def test_an_unknown_object_frame_is_refused_by_the_verb_not_planned_in_base(d1_arm, observe):
    world = observe(d1_arm)
    verb = Approach(object="red_block", side="left",
                    direction=Direction((0, 0, -1), "object:ghost"))
    plan = verb.plan(world, d1_arm)
    assert not getattr(plan, "ok", True)
    assert plan.reason == UNKNOWN_FRAME
    # a known object frame DOES resolve: the block's own -z is base -z
    same = Approach(object="red_block", side="left",
                    direction=Direction((0, 0, -1), "object:red_block"))
    _same_plan(same.plan(world, d1_arm),
               Approach(object="red_block", side="left").plan(world, d1_arm))


def test_an_upward_approach_is_a_refusal_not_an_exception(d1_arm, observe):
    world = observe(d1_arm)
    for verb in (Approach, Grasp):
        plan = verb(object="red_block", side="left", direction="up").plan(world, d1_arm)
        assert plan.reason in ("precondition_unmet", BAD_ARGUMENT), plan
        assert any(u.code == BAD_ARGUMENT for u in plan.unmet)


def test_a_lift_rises(d1_arm, observe):
    world = observe(d1_arm, held={"left": "red_block"})
    sideways = Lift(object="red_block", side="left", direction="forward")
    assert any(u.code == BAD_ARGUMENT for u in sideways.preconditions(world))
    assert Lift(object="red_block", side="left").direction == ALIASES["up"]
    tilted = Lift(object="red_block", side="left", direction=[0.2, 0.0, 1.0])
    assert tilted.preconditions(world) == []


def test_retreat_backs_out_along_the_tool_by_default(d1_arm, observe):
    world = observe(d1_arm)
    default = Retreat(side="left", distance_m=0.05)
    assert default.direction == Direction((0, 0, -1), TOOL)
    explicit = Retreat(side="left", distance_m=0.05,
                       direction={"axis": [0, 0, -1], "frame": "tool"})
    _same_plan(default.plan(world, d1_arm), explicit.plan(world, d1_arm))
    back = default.plan(world, d1_arm)
    arm = world.arm("left")
    moved = back.waypoints[0].p - arm.tool_p
    assert np.allclose(moved, -arm.tool_r.as_matrix()[:, 2] * 0.05, atol=1e-9)


def test_the_planner_reports_the_roll_it_used(d1_arm, observe):
    """The squared posture that plans says nothing; a quarter turn the sweep
    had to take says which (``test_grasp_geometry`` has the scene where the
    squared wrist is refused and the turn is taken)."""
    world = observe(d1_arm)
    square = Approach(object="red_block", side="left").plan(world, d1_arm)
    assert isinstance(square, Plan), square
    assert not any(n.startswith("jaws rolled") for n in square.notes)


def test_the_direction_prompt_text_is_generated_from_the_aliases(agent_examples):
    text = direction_doc()
    for name, d in ALIASES.items():
        assert f"  {d.label()}: " in text, name
    import astra_loop
    # the loop hands the model the kit-generated ROBOT FACTS (step 7): the
    # direction lines are in them, not quoted in the example's prompt
    from manipulation_kit.agent import robot_facts
    for line in text.splitlines():
        assert line.strip() in robot_facts()
    for old in ("top_down", "side_left", "side_right"):
        assert old not in astra_loop.SYSTEM
    assert not hasattr(astra_loop, "PLANNER_ONLY_ARGS")
    assert not hasattr(astra_loop, "_hide_planner_args")
