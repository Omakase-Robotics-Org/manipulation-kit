"""The examples are runnable, and they CONSUME the packaged boundary.

``examples/agent`` is outside the wheel (Shu, 2026-09-19) and what is left in
it is the part that knows a model exists: the prompt, the provider clients,
the scripted stand-in, the Jev menu renderer and the demo scene. The gate, the
schema and the arm choice moved into ``manipulation_kit.primitives`` — their
own tests live beside them in ``tests/primitives/`` — and these assert that
the examples use them rather than keeping a second copy.

A broken example is a broken explanation, so they run here too.
"""

from __future__ import annotations

import json

import pytest


# --------------------------------------------------------------------------- #
# the examples import the wheel, they do not re-implement it
# --------------------------------------------------------------------------- #

def test_no_example_keeps_its_own_gate_or_schema():
    """The split, asserted as a fact about the tree rather than a promise in
    a README: the three modules that moved are gone from examples/."""
    from pathlib import Path
    agent = Path(__file__).resolve().parents[2] / "examples" / "agent"
    assert not (agent / "offer.py").exists()
    assert not (agent / "schema.py").exists()
    assert not (agent / "chain.py").exists()
    for name in ("astra_loop.py", "menu.py"):
        text = (agent / name).read_text(encoding="utf-8")
        assert "manipulation_kit.primitives" in text


def test_the_loop_runs_a_plan_through_the_shared_runner(agent_examples):
    """R, section 1: the old loop hardcoded ``Mirror``, its own private step
    walker and a block attached to the hand, and then claimed "swap the
    executor; the loop does not change"."""
    from pathlib import Path
    agent = Path(__file__).resolve().parents[2] / "examples" / "agent"
    text = (agent / "astra_loop.py").read_text(encoding="utf-8")
    assert "from manipulation_kit.executor import run" in text
    assert "isinstance(step, JointStep)" not in text, (
        "the loop is walking plan steps itself again")


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
    assert trace.stop == "goal_verified"
    lines = (tmp_path / "trace.jsonl").read_text().strip().splitlines()
    assert len(lines) == len(trace.records)
    first = json.loads(lines[0])
    assert first["choice"]["name"] == "grasp"
    # the TASK is in the record, and so is the observation that followed
    assert first["task"] == astra_loop.DEFAULT_TASK
    assert first["observation_after"]


def test_the_model_is_told_the_task(agent_examples):
    """R, section 1: "The real model is never told the actual task in
    ``loop()``; ``goal`` affects grading only"."""
    import astra_loop
    seen = {}

    class Listening(astra_loop.ScriptedModel):
        def __call__(self, messages, tools):
            seen.setdefault("first", list(messages))
            return super().__call__(messages, tools)

    astra_loop.loop(Listening(), task="put the red block in the box",
                    max_turns=1)
    text = "\n".join(m["content"] for m in seen["first"])
    assert "put the red block in the box" in text


def test_a_model_that_says_done_early_does_not_end_the_run(agent_examples):
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
    assert trace.stop == "max_turns"
    assert all(r.verdict is None for r in trace.records)
    assert all(r.refused for r in trace.records)


def test_a_model_that_stops_still_has_the_goal_measured(agent_examples):
    """R, section 3: "It breaks on no call without checking the goal,
    contrary to its two-stop-condition docstring"."""
    import astra_loop

    class Quits:
        def __call__(self, messages, tools):
            return {"name": None, "arguments": {}, "claimed": "done"}

    trace = astra_loop.loop(Quits(), max_turns=3)
    assert trace.stop == "model_stopped"
    assert trace.records[-1].goal_verdict["verdict"] == "false"
    assert trace.summary()["claimed_but_unmeasured"] == 1


def test_two_tool_calls_in_one_turn_are_a_protocol_error(agent_examples):
    """R, section 3: "It accepts the first call if several arrive despite
    asking for exactly one" — which teaches the model the rest ran too."""
    import astra_loop

    class Chatty:
        def __call__(self, messages, tools):
            return {"name": None, "arguments": {}, "call_id": "",
                    "protocol_error": "you called 2 tools", "claimed": ""}

    trace = astra_loop.loop(Chatty(), max_turns=2)
    assert len(trace.records) == 2
    assert all(r.verdict is None for r in trace.records)


def test_a_malformed_tool_call_is_answered_not_crashed_on(agent_examples):
    import astra_loop

    class Nonsense:
        def __call__(self, messages, tools):
            return {"name": "grasp", "arguments": {"nonexistent": 1},
                    "claimed": ""}

    trace = astra_loop.loop(Nonsense(), max_turns=2)
    assert len(trace.records) == 2
    assert all(r.refused for r in trace.records)


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
    assert record.refused and record.refused[0]["reason"]


def test_the_summary_keeps_transport_and_task_apart(agent_examples):
    """R, section 1: "``model_stopped``, transport completion, primitive
    success, and task success must remain separate results"."""
    import astra_loop
    trace = astra_loop.loop(astra_loop.ScriptedModel(), max_turns=8)
    summary = trace.summary()
    for key in ("transport_completed", "verdicts", "goal_verdict", "stop"):
        assert key in summary


# --------------------------------------------------------------------------- #
# the menu renderer
# --------------------------------------------------------------------------- #

def test_the_jev_menu_carries_the_task_and_a_way_to_not_move(agent_examples,
                                                             d1_arm, observe):
    """R, section 3: "The question asks which action advances 'the task'
    without taking a task", and there was no wait / rescan / stop."""
    from menu import choice_menu
    menu = choice_menu(observe(d1_arm), d1_arm, task="put the block in the box")
    assert "put the block in the box" in menu["question"]
    assert menu["task"] == "put the block in the box"
    ids = {c["id"] for c in menu["choices"]}
    assert {"wait", "rescan", "stop"} <= ids


def test_the_menu_reports_what_the_cap_hid(agent_examples, d1_arm, observe):
    """R, section 3: "The reported 'tried' count omits valid-but-truncated
    candidates"."""
    from menu import choice_menu
    menu = choice_menu(observe(d1_arm), d1_arm, task="t", cap=4)
    motions = [c for c in menu["choices"] if "index" in c]
    assert len(motions) == 4
    assert menu["hidden"]
    assert menu["offered"] == len(motions) + len(menu["hidden"])


def test_the_menu_spends_its_slots_on_task_verbs_before_corrections(
        agent_examples, d1_arm, observe):
    """A menu that filled its twenty slots with nudges has hidden the grasp."""
    from menu import choice_menu
    menu = choice_menu(observe(d1_arm), d1_arm, task="t", cap=20)
    verbs = [c["verb"] for c in menu["choices"] if "verb" in c]
    assert verbs.index("grasp") < verbs.index("nudge")


def test_the_menu_interleaves_the_two_hands(agent_examples, d1_arm, observe):
    """R, section 3: "Candidate order is left-arm first, then right; cap=20
    can remove the needed right-hand action"."""
    from menu import choice_menu
    menu = choice_menu(observe(d1_arm), d1_arm, task="t", cap=8)
    sides = [c["arguments"].get("side") for c in menu["choices"] if "verb" in c]
    assert "right" in sides[:8]


def test_the_jev_menu_renders_without_a_model(agent_examples, capsys):
    import jev_menu
    assert jev_menu.main([]) == 0
    printed = capsys.readouterr().out
    assert "Which single action" in printed
    assert "grasp red_block" in printed


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
