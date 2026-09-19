"""The examples are runnable, and the two model-facing exports do not drift.

``examples/agent`` is outside the wheel (Shu, 2026-09-19) but it is not outside
the suite: a broken example is a broken explanation, and the schema drift gate
in particular is the thing standing between "one vocabulary" and "two robots
depending on which model is driving".
"""

from __future__ import annotations

import json


# --------------------------------------------------------------------------- #
# G5: one definition set, two exports
# --------------------------------------------------------------------------- #

def test_the_two_exports_enumerate_the_same_verbs(agent_examples, d1_arm, observe):
    import schema
    from manipulation_kit.primitives import BY_VERB

    tools = {s["name"] for s in schema.tool_schemas()}
    assert tools == set(BY_VERB), "the tool schemas and the kit disagree on verbs"

    menu = schema.choice_menu(observe(d1_arm), d1_arm, cap=200)
    offered_and_refused = {c["verb"] for c in menu["choices"]}
    offered_and_refused |= {r["verb"] for r in menu["refused"]}
    assert offered_and_refused <= tools, (
        "the menu offered a verb the tool schemas do not describe")


def test_the_two_exports_agree_on_every_argument_domain(agent_examples, d1_arm,
                                                        observe):
    import schema

    domains = schema.domains_in(schema.tool_schemas())
    menu = schema.choice_menu(observe(d1_arm), d1_arm, cap=200)
    for choice in menu["choices"] + menu["refused"]:
        verb = choice["verb"]
        for name, value in choice["arguments"].items():
            domain = domains[verb][name]
            if domain["kind"] == "enum":
                assert value in domain["values"], \
                    f"{verb}.{name}={value!r} is outside the tool schema's enum"
            elif domain["kind"] == "number":
                assert domain["minimum"] <= value <= domain["maximum"], \
                    f"{verb}.{name}={value!r} is outside the tool schema's range"


def test_every_verb_of_the_kit_reaches_the_tool_schemas(agent_examples):
    import schema
    from manipulation_kit.primitives import PRIMITIVES

    tools = {s["name"]: s for s in schema.tool_schemas()}
    for cls in PRIMITIVES:
        assert cls.name() in tools
        described = set(tools[cls.name()]["parameters"]["properties"])
        assert described == set(cls.arguments()), (
            f"{cls.name()}: the schema describes {described} and the "
            f"dataclass takes {set(cls.arguments())}")


def test_the_schema_narrows_object_names_to_what_is_actually_there(
        agent_examples, d1_arm, observe):
    """The offer gate, one level up: a model cannot ask for the green block if
    the word is not in its schema."""
    import schema
    world = observe(d1_arm)
    grasp = next(s for s in schema.tool_schemas(world) if s["name"] == "grasp")
    assert grasp["parameters"]["properties"]["object"]["enum"] == list(world.names())
    assert "object" in grasp["parameters"]["required"]


def test_the_tool_schemas_are_json(agent_examples):
    import schema
    json.dumps(schema.tool_schemas())


# --------------------------------------------------------------------------- #
# G2: a refusal always reaches the caller
# --------------------------------------------------------------------------- #

def test_an_unreachable_candidate_never_becomes_a_word_in_the_prompt(
        agent_examples, d1_arm, observe):
    import offer as offer_module
    world = observe(d1_arm)
    offered, refused = offer_module.offer(
        offer_module.candidates_for(world), world, d1_arm, cap=200)
    assert offered and refused
    for item in offered:
        assert item.plan.ok
    for item in refused:
        assert not item.error.ok


def test_every_refusal_carries_a_reason_a_caller_can_render(agent_examples,
                                                            d1_arm, observe):
    """dx-inspect-robots PR #17: the guard refused and the model was told
    'executing move_by over 4 steps', thirty times, with no reason."""
    import offer as offer_module
    from manipulation_kit.primitives import PLAN_REASONS
    world = observe(d1_arm)
    _offered, refused = offer_module.offer(
        offer_module.candidates_for(world), world, d1_arm, cap=200)
    for item in refused:
        assert item.error.reason in PLAN_REASONS
        assert item.label in item.sentence()
        assert item.error.reason in item.sentence()


def test_when_nothing_is_offerable_the_caller_still_gets_sentences(
        agent_examples, d1_arm, observe):
    import offer as offer_module
    from manipulation_kit.primitives import Grasp
    world = observe(d1_arm, block_p=(0.52, 0.25, 0.45))
    offered, refused = offer_module.offer(
        [Grasp(object="red_block", side="left")], world, d1_arm)
    assert not offered
    text = offer_module.why_nothing(refused)
    assert "Nothing is possible" in text
    assert "red_block" in text


def test_the_cap_bounds_the_menu_but_never_the_refusals(agent_examples, d1_arm,
                                                        observe):
    import offer as offer_module
    world = observe(d1_arm)
    candidates = offer_module.candidates_for(world)
    offered, refused = offer_module.offer(candidates, world, d1_arm, cap=5)
    assert len(offered) == 5
    all_offered, all_refused = offer_module.offer(candidates, world, d1_arm,
                                                  cap=1000)
    assert len(refused) == len(all_refused)
    assert len(all_offered) > 5


def test_the_menu_spends_its_slots_on_task_verbs_before_corrections(
        agent_examples, d1_arm, observe):
    """A menu that filled its twenty slots with nudges has hidden the grasp."""
    import schema
    menu = schema.choice_menu(observe(d1_arm), d1_arm, cap=20)
    verbs = [c["verb"] for c in menu["choices"]]
    assert "grasp" in verbs
    assert verbs.index("grasp") < verbs.index("nudge")


# --------------------------------------------------------------------------- #
# the loop, and G4
# --------------------------------------------------------------------------- #

def test_the_scripted_loop_completes_the_task_and_measures_it(agent_examples,
                                                              tmp_path):
    import astra_loop
    trace = astra_loop.loop(astra_loop.ScriptedModel(), max_turns=8,
                            trace_path=tmp_path / "trace.jsonl")
    assert trace.records
    assert trace.records[-1].goal_verdict["verdict"] == "true"
    lines = (tmp_path / "trace.jsonl").read_text().strip().splitlines()
    assert len(lines) == len(trace.records)
    assert json.loads(lines[0])["choice"]["name"] == "grasp"


def test_a_model_that_says_done_early_does_not_end_the_run(agent_examples,
                                                           tmp_path):
    """G4. The scripted model claims 'done' at the carry; the task verifier
    says otherwise and the loop keeps going. The trace keeps both columns."""
    import astra_loop
    trace = astra_loop.loop(astra_loop.ScriptedModel(), max_turns=8)
    claimed = [r for r in trace.records if r.claimed]
    assert claimed, "the stub no longer over-claims; the gate is untested"
    assert trace.summary()["claimed_but_unmeasured"] == 1
    assert len(trace.records) > claimed[0].iteration + 1


def test_the_loop_stops_at_max_turns_even_when_nothing_works(agent_examples):
    """The second exit. A loop with only a success exit runs all night."""
    import astra_loop

    class Stubborn:
        def __call__(self, messages, tools):
            return {"name": "grasp",
                    "arguments": {"object": "red_block", "side": "right",
                                  "approach": "top_down"},
                    "claimed": ""}

    trace = astra_loop.loop(Stubborn(), max_turns=3)
    assert len(trace.records) == 3
    assert all(r.verdict is None for r in trace.records)
    assert all(r.refused for r in trace.records)


def test_a_malformed_tool_call_is_answered_not_crashed_on(agent_examples):
    import astra_loop

    class Nonsense:
        def __call__(self, messages, tools):
            return {"name": "grasp", "arguments": {"nonexistent": 1},
                    "claimed": ""}

    trace = astra_loop.loop(Nonsense(), max_turns=2)
    assert len(trace.records) == 2


def test_the_trace_records_refusals_with_their_reasons(agent_examples):
    import astra_loop

    class AsksForTheImpossible:
        def __call__(self, messages, tools):
            return {"name": "grasp",
                    "arguments": {"object": "red_block", "side": "right",
                                  "approach": "top_down"},
                    "claimed": ""}

    trace = astra_loop.loop(AsksForTheImpossible(), max_turns=1)
    record = trace.records[0]
    assert record.refused
    assert record.refused[0]["refusal"]["reason"]


def test_the_jev_menu_renders_without_a_model(agent_examples, capsys):
    import jev_menu
    assert jev_menu.main([]) == 0
    printed = capsys.readouterr().out
    assert "Which single action" in printed
    assert "grasp red_block" in printed


def test_the_offer_example_runs(agent_examples, capsys):
    import offer as offer_module
    assert offer_module.main([]) == 0
    assert "offered of" in capsys.readouterr().out


def test_the_astra_example_runs_dry(agent_examples, capsys):
    import astra_loop
    assert astra_loop.main(["--dry-run"]) == 0
    assert "CLAIMED DONE, NOT MEASURED" in capsys.readouterr().out


def test_the_decision_trace_separates_the_claim_from_the_measurement(
        agent_examples):
    from trace import DecisionRecord, DecisionTrace
    tracker = DecisionTrace()
    tracker.write(DecisionRecord(0, world={}, claimed="done",
                                 goal_verdict={"verdict": "false",
                                               "reason": "not in the box"}))
    tracker.write(DecisionRecord(1, world={}, claimed=None,
                                 goal_verdict={"verdict": "true", "reason": "in"}))
    assert len(tracker.disagreements()) == 1
    assert tracker.summary()["claimed_but_unmeasured"] == 1
