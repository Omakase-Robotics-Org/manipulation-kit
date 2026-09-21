"""observe -> offer -> tool call -> execute -> VERIFY, as a runnable loop.

The shape of the thing, in one file: a function-calling model is given the
TASK, the world as text and the kit's primitives as JSON Schema tools; it names
one verb with arguments; the kit decodes and re-checks the BOUND call (a model
may ask for anything, it does not get to skip the guard); the plan runs on an
executor, and the turn ends with a MEASURED verdict from a verifier rather than
with the model's opinion.

Everything checkable comes from the WHEEL —
``manipulation_kit.primitives.schema.tool_schemas`` / ``decode``,
``manipulation_kit.primitives.offer.check``,
``manipulation_kit.primitives.reach.choose_side``,
``manipulation_kit.executor.run``. What is in this file is the part that knows
a model exists: the provider clients, the prompt, the scripted stand-in, and
the message bookkeeping.

THE EXECUTOR IS AN ARGUMENT. ``--executor kinematic`` mirrors the plan onto a
model of the robot; ``--executor firmware --robot http://d1-2:4750`` runs it on
a real D1 through ``manipulation_kit.executors.firmware``. The loop does not
change, which is what the Executor protocol is for — and unlike the previous
version of this file, that is now true rather than claimed: the private step
walker and the block-follows-the-hand mirror are behind the same interface as
the firmware transport.

Three stop reasons, never conflated:

``goal_verified``  the TASK verifier measured TRUE
``model_stopped``  the model returned no call — recorded, and the goal is
                   still measured before believing it
``max_turns``      the cap. A loop with only the first exit runs all night.

    python examples/agent/astra_loop.py --dry-run     # scripted, no key needed
    OPENAI_API_KEY=... OPENAI_MODEL=gpt-5 python examples/agent/astra_loop.py

`openai` is NOT a dependency of this repository: ``pip install openai`` before
using ``--model openai``.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence


sys.path.insert(0, str(Path(__file__).resolve().parent))

from manipulation_kit.executor import run  # noqa: E402
from manipulation_kit.primitives import Place  # noqa: E402
from manipulation_kit.primitives.offer import check, label_for  # noqa: E402
from manipulation_kit.primitives.reach import choose_side  # noqa: E402
from manipulation_kit.primitives.schema import decode, tool_schemas  # noqa: E402
from mirror import MirrorRobot  # noqa: E402
from scene import demo_scene  # noqa: E402
from trace import DecisionRecord, DecisionTrace  # noqa: E402

DEFAULT_TASK = "put the red block in the box"

SYSTEM = """You drive a D1 humanoid's two arms through a fixed set of verbs.

Rules that are not negotiable, because the robot enforces them anyway:
- You never give an orientation. Name an approach (top_down, front, side_left,
  side_right) and the robot derives the wrist pose from the object.
- `nudge` translations are snapped to a 10/30/50 mm grid, and its `dyaw` is
  clamped to +-15 degrees about the hand's own approach axis. Every OTHER
  number you give is used as you write it, inside the range in the schema.
- An action the motion guard or the inverse kinematics refuses is reported back
  to you with the reason and how far short it was. Read it and choose
  differently; repeating it will not work.
- You do not decide whether the task is done. A measurement does.

Call exactly one tool per turn."""


@dataclass
class Stop:
    reason: str
    detail: str = ""


class ScriptedModel:
    """A stand-in for a function-calling model: plays a correct pick-and-place.

    It exists so the loop, the gate and the verifier are runnable and testable
    with no key, no network and no model. It also makes one deliberate mistake
    (it claims "done" one turn early) so the trace's claimed-vs-measured column
    has something in it. It is NOT a result: see the README on what the
    simulation findings do and do not support.
    """

    def __init__(self, obj: str = "red_block", to: str = "box"):
        self.script: List[Dict[str, Any]] = [
            {"name": "grasp", "arguments": {"object": obj, "side": "left",
                                            "approach": "top_down"}},
            {"name": "lift", "arguments": {"object": obj, "side": "left",
                                           "height_m": 0.1}},
            {"name": "carry", "arguments": {"object": obj, "to": to,
                                            "side": "left"}},
            {"name": "place", "arguments": {"object": obj, "to": to,
                                            "side": "left"}},
        ]
        self.turn = 0

    def use_side(self, side: str) -> None:
        """Play the script with the hand the task planner chose."""
        for call in self.script:
            if "side" in call["arguments"]:
                call["arguments"]["side"] = side

    def __call__(self, messages, tools) -> Dict[str, Any]:
        if self.turn >= len(self.script):
            return {"name": None, "arguments": {}, "claimed": "done"}
        call = self.script[self.turn]
        self.turn += 1
        # The deliberate over-claim: it says "done" at the CARRY, one turn
        # before the block is in the box. The loop ignores it and keeps going,
        # and the trace records the disagreement — which is the whole exhibit.
        claimed = "done" if self.turn == len(self.script) - 1 else ""
        return dict(call, claimed=claimed, call_id=f"scripted-{self.turn}")


class OpenAIModel:
    """The real thing, through the Responses API. Imported only when used."""

    def __init__(self, model: str, api_key: str):
        from openai import OpenAI  # noqa: PLC0415
        self.client = OpenAI(api_key=api_key)
        self.model = model

    def __call__(self, messages, tools) -> Dict[str, Any]:
        response = self.client.responses.create(
            model=self.model,
            input=messages,
            tools=[{"type": "function", **t} for t in tools])
        calls = [i for i in response.output
                 if getattr(i, "type", "") == "function_call"]
        if not calls:
            return {"name": None, "arguments": {},
                    "claimed": getattr(response, "output_text", "")}
        if len(calls) > 1:
            # The contract says exactly one. Taking the first silently taught
            # the model that the rest were executed too.
            return {"name": None, "arguments": {}, "call_id": "",
                    "protocol_error": (
                        f"you called {len(calls)} tools; the contract is "
                        f"exactly one per turn. Choose one and call it again."),
                    "claimed": ""}
        item = calls[0]
        return {"name": item.name, "arguments": json.loads(item.arguments),
                "call_id": getattr(item, "call_id", "") or getattr(item, "id", ""),
                "claimed": ""}


def build_model(dry_run: bool):
    key = os.environ.get("OPENAI_API_KEY")
    if dry_run or not key:
        if not dry_run:
            print("[astra_loop] no OPENAI_API_KEY; running the scripted stub",
                  file=sys.stderr)
        return ScriptedModel()
    return OpenAIModel(os.environ.get("OPENAI_MODEL", "gpt-5"), key)


def _say(messages: List[Dict[str, Any]], call_id: str, text: str) -> None:
    """Feed a result back CORRELATED with the call that produced it.

    The old loop appended every result as anonymous user prose, so a model
    with two outstanding ideas could not tell which one the refusal was about.
    """
    messages.append({"role": "user",
                     "content": (f"[result of {call_id}] {text}" if call_id
                                 else text)})


def loop(model, robot=None, *, task: str = DEFAULT_TASK, max_turns: int = 8,
         trace_path: Optional[Path] = None, goal=None) -> DecisionTrace:
    world0, kin = demo_scene()
    robot = robot if robot is not None else MirrorRobot(kin)
    # WHICH HAND — decided before anything moves, by planning the whole chain
    # (Approach, Grasp, Lift, Carry, Place) for BOTH arms and taking the one
    # that can DELIVER. The near hand is only the tie-break; see
    # manipulation_kit.primitives.reach for what that cost on the blocks-eval
    # wagon (2026-09-19, F10).
    hand = choose_side(world0, kin, obj="red_block", destination="box")
    if goal is None:
        goal = Place(object="red_block", to="box", side=hand.side)
    trace = DecisionTrace(trace_path)
    trace.task = task
    messages: List[Dict[str, Any]] = [{"role": "system", "content": SYSTEM},
                                      {"role": "user", "content": f"TASK: {task}"}]
    if not hand.reachable:
        # F10/F12: an impossible task is refused at turn zero rather than
        # discovered three verbs in. The old loop computed this and ignored it.
        record = DecisionRecord(iteration=0, world=world0.to_json())
        record.stop = "unreachable_task"
        record.refused = [c.to_json() for c in hand.chains.values()]
        trace.write(record)
        trace.stop = Stop("unreachable_task", hand.reason).reason
        return trace

    if isinstance(model, ScriptedModel):
        model.use_side(hand.side)
    stop = Stop("max_turns", f"{max_turns} turns without a measured goal")
    for turn in range(max_turns):
        world = robot.world()
        messages.append({"role": "user", "content": world.to_text()})
        tools = tool_schemas(world)      # REFRESHED: the names narrow as the
        record = DecisionRecord(iteration=turn, world=world.to_json())
        record.task = task

        call = model(messages, tools)
        call_id = call.get("call_id") or ""
        record.choice = {"name": call["name"], "arguments": call["arguments"],
                         "call_id": call_id}
        record.claimed = call.get("claimed") or None
        if call.get("protocol_error"):
            _say(messages, call_id, call["protocol_error"])
            trace.write(record)
            continue
        if not call["name"]:
            # The model stopped. That is a claim about the task, and the task
            # verifier is what decides — the old loop broke here without
            # checking the goal at all, contrary to its own docstring.
            goal_report = goal.verifier(world)(robot.world())
            record.goal_verdict = goal_report.to_json()
            record.stop = "model_stopped"
            trace.write(record)
            stop = Stop("goal_verified" if goal_report.verdict == "true"
                        else "model_stopped", goal_report.reason)
            break

        # THE GATE, on the BOUND call. A model may ask for anything; decode
        # checks the arguments against the kit's own table and the guard
        # decides the rest. This is the step PR #17 was missing.
        primitive = decode(call["name"], call["arguments"], world)
        if not isinstance(primitive, object) or getattr(primitive, "ok", None) is False:
            record.refused = [primitive.to_json()]
            _say(messages, call_id, f"that call is malformed: {primitive}")
            trace.write(record)
            continue
        plan = check(primitive, world, kin)
        if not getattr(plan, "ok", False):
            record.refused = [plan.to_json()]
            _say(messages, call_id, f"{label_for(primitive)} was refused: {plan}")
            trace.write(record)
            continue
        record.offered = [{"id": f"{primitive.name()}", "label": label_for(primitive)}]

        record.plan = plan.to_json()
        report = run(plan, robot.executor)
        record.run = report.to_json()
        after = robot.world()
        verdict = primitive.verifier(world)(after)
        record.verdict = verdict.to_json()
        record.observation_after = after.to_json()
        # WHY the transport stopped, not just that it did. A barrier failure
        # now carries a typed reason and a number ("the tool point is 27 mm
        # from the grasp pose after 2 corrections"); a model told only
        # "barrier_failed" has to guess what to do differently.
        how = ("ok" if report.completed else
               f"{report.stop_reason} — {report.error}")
        _say(messages, call_id,
             f"{label_for(primitive)}: transport {how}; "
             f"measured {verdict.verdict} — {verdict.reason}")
        goal_report = goal.verifier(world)(after)
        record.goal_verdict = goal_report.to_json()
        trace.write(record)
        if goal_report.verdict == "true":
            stop = Stop("goal_verified", goal_report.reason)
            break
    trace.stop = stop.reason
    trace.stop_detail = stop.detail
    return trace


def build_robot(kind: str, kin, robot_url: str):
    if kind == "kinematic":
        return MirrorRobot(kin)
    from manipulation_kit.executors.firmware import FirmwareExecutor  # noqa: PLC0415
    from live import LiveRobot  # noqa: PLC0415
    return LiveRobot(FirmwareExecutor(base_url=robot_url), kin)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--task", default=DEFAULT_TASK)
    parser.add_argument("--dry-run", action="store_true",
                        help="use the scripted stub even if a key is present")
    parser.add_argument("--executor", choices=("kinematic", "firmware"),
                        default="kinematic")
    parser.add_argument("--robot", default="http://127.0.0.1:4750")
    parser.add_argument("--max-turns", type=int, default=8)
    parser.add_argument("--trace", type=Path, default=None)
    args = parser.parse_args(argv)

    _world, kin = demo_scene()
    robot = build_robot(args.executor, kin, args.robot)
    trace = loop(build_model(args.dry_run), robot, task=args.task,
                 max_turns=args.max_turns, trace_path=args.trace)
    for record in trace.records:
        name = (record.choice or {}).get("name") or "(no call)"
        verdict = (record.verdict or {}).get("verdict", "-")
        print(f"turn {record.iteration}: {name:9s} -> {verdict}"
              f"   {(record.verdict or {}).get('reason', '')[:70]}")
    print("\n" + json.dumps(trace.summary(), indent=2))
    for record in trace.disagreements():
        measured = record.goal_verdict or record.verdict or {}
        print(f"\nCLAIMED DONE, NOT MEASURED, at turn {record.iteration}: "
              f"the task verifier says {measured.get('verdict')} — "
              f"{measured.get('reason')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
