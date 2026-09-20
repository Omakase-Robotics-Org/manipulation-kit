"""The model-independent action boundary, now that it is in the wheel.

Everything here imports from ``manipulation_kit.primitives`` and nothing here
knows a model exists — which is the claim the move was made for. The drift
gate is the part that matters: it compares the kit's own argument table with
what is read back OUT of a rendered schema, so the two halves of the
comparison no longer come from the same call (the old test asked
``domains_in`` to re-run the generator).
"""

from __future__ import annotations

import json

import pytest

from manipulation_kit.primitives import BY_VERB, PLAN_REASONS, PRIMITIVES, Grasp
from manipulation_kit.primitives.offer import (candidates_for, label_for, offer,
                                               why_nothing)
from manipulation_kit.primitives.schema import (decode, domains, domains_in,
                                                tool_schemas)


# --------------------------------------------------------------------------- #
# the wheel carries it, and carries no model with it
# --------------------------------------------------------------------------- #

def test_the_boundary_installs_with_the_package_and_imports_nothing_extra():
    """R, section 1: "None are installed. Sibling imports and sys.path
    injection make copying files part of the integration."

    And the counter-argument the old README made — that shipping the schema
    would make every kinematics consumer depend on a model — measured here:
    importing the whole boundary pulls in no provider SDK and no HTTP client.
    """
    import subprocess
    import sys
    probe = (
        "import sys\n"
        "from manipulation_kit.primitives.schema import tool_schemas, decode\n"
        "from manipulation_kit.primitives.offer import offer, candidates_for\n"
        "from manipulation_kit.primitives.reach import choose_side\n"
        "tool_schemas()\n"
        "bad = [m for m in sys.modules if m.split('.')[0] in "
        "('openai','anthropic','requests','httpx','aiohttp','urllib3')]\n"
        "assert not bad, bad\n"
        "print('ok')\n")
    out = subprocess.run([sys.executable, "-c", probe], capture_output=True,
                         text=True)
    assert out.returncode == 0, out.stdout + out.stderr


# --------------------------------------------------------------------------- #
# G5: one definition set
# --------------------------------------------------------------------------- #

def test_the_export_and_the_kits_own_table_agree_on_every_domain():
    """The drift gate, with independent halves."""
    assert domains_in(tool_schemas()) == domains()


def test_every_verb_of_the_kit_reaches_the_tool_schemas():
    from manipulation_kit.primitives.schema import NOT_MODEL_BINDABLE
    tools = {s["name"]: s for s in tool_schemas()}
    assert set(tools) == set(BY_VERB)
    for cls in PRIMITIVES:
        described = set(tools[cls.name()]["parameters"]["properties"])
        bindable = set(cls.arguments()) - set(NOT_MODEL_BINDABLE)
        assert described == bindable, (
            f"{cls.name()}: the schema describes {described} and the "
            f"dataclass takes {bindable}")


def test_deployment_configuration_is_not_offered_to_a_model():
    """R, section 3: "Default checkpoint selection is deployment
    configuration, not a free model string"."""
    pour = next(s for s in tool_schemas() if s["name"] == "pour")
    assert "policy" not in pour["parameters"]["properties"]
    # ...and the limitation reaches capability discovery, not only a docstring
    assert "UNKNOWN" in pour["description"]
    assert "learned_policy_required" in pour["description"]


def test_the_tool_schemas_are_json():
    json.dumps(tool_schemas())


def test_the_schema_narrows_names_by_ROLE_not_by_presence(d1_arm, observe):
    """R, section 3: ``tool_schemas`` applied ALL world names to object,
    source, target and destination, so tables became grasp candidates and
    plain blocks became placement destinations."""
    world = observe(d1_arm)
    schemas = {s["name"]: s for s in tool_schemas(world)}
    grasp = schemas["grasp"]["parameters"]["properties"]["object"]["enum"]
    place_to = schemas["place"]["parameters"]["properties"]["to"]["enum"]
    assert grasp == ["red_block"], grasp
    assert sorted(place_to) == ["box", "table"], place_to
    assert "table" not in grasp and "red_block" not in place_to


def test_a_name_that_cannot_play_its_role_is_refused_at_decode(d1_arm, observe):
    """The enum narrows what a model can SAY; decode is what happens when it
    says something else anyway."""
    world = observe(d1_arm)
    refusal = decode("grasp", {"object": "table", "side": "left"}, world)
    assert getattr(refusal, "ok", True) is False
    assert refusal.reason == "bad_argument"
    assert "graspable" in refusal.detail


def test_a_schema_enum_is_not_a_promise_that_the_arm_can_reach_it(d1_arm,
                                                                  observe):
    """R, section 3: "Do not equate its name enum with IK feasibility."
    A name in the enum still has to survive the plan."""
    world = observe(d1_arm, block_p=(0.52, 0.25, 0.45))
    grasp = next(s for s in tool_schemas(world) if s["name"] == "grasp")
    assert "red_block" in grasp["parameters"]["properties"]["object"]["enum"]
    call = decode("grasp", {"object": "red_block", "side": "left"}, world)
    assert not getattr(call.plan(world, d1_arm), "ok", False)


# --------------------------------------------------------------------------- #
# G2: a refusal always reaches the caller
# --------------------------------------------------------------------------- #

def test_an_unreachable_candidate_never_becomes_a_word_in_the_prompt(
        d1_arm, observe):
    world = observe(d1_arm)
    offered, refused = offer(candidates_for(world), world, d1_arm)
    assert offered and refused
    assert all(item.plan.ok for item in offered)
    assert all(not item.error.ok for item in refused)


def test_every_refusal_carries_a_reason_a_caller_can_render(d1_arm, observe):
    """dx-inspect-robots PR #17: the guard refused and the model was told
    'executing move_by over 4 steps', thirty times, with no reason."""
    world = observe(d1_arm)
    _offered, refused = offer(candidates_for(world), world, d1_arm)
    for item in refused:
        assert item.error.reason in PLAN_REASONS
        assert item.label in item.sentence()
        assert item.error.reason in item.sentence()


def test_when_nothing_is_offerable_the_caller_still_gets_sentences(d1_arm,
                                                                   observe):
    world = observe(d1_arm, block_p=(0.52, 0.25, 0.45))
    offered, refused = offer([Grasp(object="red_block", side="left")], world,
                             d1_arm)
    assert not offered
    text = why_nothing(refused)
    assert "Nothing is possible" in text and "red_block" in text


def test_offer_returns_everything_it_planned(d1_arm, observe):
    """R, section 3: the cap "bounds the OFFERED list only" — after planning
    every candidate. A gate that plans 51 things and hands back 20 has bounded
    neither the computation nor what the caller can see. Capping is a
    RENDERING decision and it left with the renderer."""
    world = observe(d1_arm)
    candidates = candidates_for(world)
    offered, refused = offer(candidates, world, d1_arm)
    assert len(offered) + len(refused) == len(candidates)


def test_a_candidate_carries_a_stable_id_independent_of_menu_order(d1_arm,
                                                                    observe):
    """R, section 3: "Choices are index-only" — and an index changes meaning
    the moment the list is rebuilt from a newer observation."""
    world = observe(d1_arm)
    first, _ = offer(candidates_for(world), world, d1_arm)
    again, _ = offer(list(reversed(candidates_for(world))), world, d1_arm)
    assert {o.id for o in first} == {o.id for o in again}
    assert all("(" in o.id and "=" in o.id for o in first)


def test_a_hand_nobody_can_read_gets_no_candidates(d1_arm, observe):
    """Offering both 'grasp' and 'release' for a gripper that reports nothing
    is offering to guess."""
    world = observe(d1_arm)
    blind = world.with_(grippers={"right": world.gripper("right")})
    sides = {getattr(c, "side", None) for c in candidates_for(blind)}
    assert "left" not in sides


def test_the_correction_family_includes_a_yaw(d1_arm, observe):
    """R, section 3: "There are no yaw correction candidates"."""
    world = observe(d1_arm)
    assert any(getattr(c, "dyaw", 0.0) for c in candidates_for(world))


@pytest.mark.parametrize("call,expected", [
    (("no_such_verb", {}), "no verb called"),
    (("grasp", {"nonexistent": 1}), "not ['nonexistent']"),
    (("grasp", {"object": "red_block", "standoff_m": "8cm"}), "must be a number"),
    (("nudge", {"side": "left", "dx": float("nan")}), "must be finite"),
    (("place", {"object": "red_block", "to": "box", "clearance_m": -0.5}),
     "must be between"),
])
def test_every_way_a_call_can_be_malformed_ends_as_one_typed_refusal(call,
                                                                     expected):
    """R14/R3: the old loop caught TypeError and ValueError around the
    CONSTRUCTOR only, and planning happened outside that handler."""
    refusal = decode(*call)
    assert getattr(refusal, "ok", True) is False
    assert refusal.reason == "bad_argument"
    assert expected in refusal.detail


def test_a_well_formed_call_decodes_to_the_primitive_it_named(d1_arm, observe):
    world = observe(d1_arm)
    call = decode("grasp", {"object": "red_block", "side": "left",
                            "approach": "top_down"}, world)
    assert isinstance(call, Grasp)
    assert label_for(call) == "grasp red_block with the left hand, top down"
