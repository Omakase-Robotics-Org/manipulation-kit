"""observe -> offer -> tool call -> execute -> VERIFY, as a runnable loop.

The shape of the thing, in one file: a function-calling model is given the
world as text and the kit's primitives as JSON Schema tools, it names one verb
with arguments, the kit re-runs the offer gate on the BOUND call (a model may
ask for anything; it does not get to skip the guard), executes the plan on a
kinematic mirror, and the turn ends with a MEASURED verdict from a verifier
rather than with the model's opinion.

Two stop conditions, always: the task verifier passes, or ``--max-turns`` is
reached. A loop with only the first exit is the one that runs all night.

    python examples/agent/astra_loop.py --dry-run     # scripted, no key needed
    OPENAI_API_KEY=... OPENAI_MODEL=gpt-5 python examples/agent/astra_loop.py

The model and the key come from the environment. With no key the loop runs a
scripted stub that plays a correct pick-and-place, so the loop itself — and the
verifier gates in it — can be exercised and tested with nothing installed.

The executor here is ``KinematicExecutor``: it moves a model of the robot and
nothing else. Point it at a real D1 by swapping in
``manipulation_kit.executors.firmware.FirmwareExecutor`` — the loop does not
change, which is the point of the Executor protocol.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from manipulation_kit.executor import KinematicExecutor, wire  # noqa: E402
from manipulation_kit.primitives import (GripStep, JointStep,  # noqa: E402
                                         Place, by_verb)
from manipulation_kit.primitives.approach import tool_from_link7  # noqa: E402
from chain import choose_side  # noqa: E402
from offer import offer, why_nothing  # noqa: E402
from scene import BLOCK_P, demo_scene, observe  # noqa: E402
from schema import tool_schemas  # noqa: E402
from trace import DecisionRecord, DecisionTrace  # noqa: E402

SYSTEM = """You drive a D1 humanoid's two arms through a fixed set of verbs.

Rules that are not negotiable, because the robot enforces them anyway:
- You never give an orientation. Name an approach (top_down, front, side_left,
  side_right) and the robot derives the wrist pose from the object.
- Translations you ask for are snapped to a 10/30/50 mm grid.
- An action the motion guard or the inverse kinematics refuses is reported back
  to you with the reason and how far short it was. Read it and choose
  differently; repeating it will not work.
- You do not decide whether the task is done. A measurement does.

Call exactly one tool per turn."""


class ScriptedModel:
    """A stand-in for a function-calling model: plays a correct pick-and-place.

    It exists so the loop, the gate and the verifier are runnable and testable
    with no key, no network and no model. It also makes one deliberate mistake
    (it claims "done" one turn early) so the trace's claimed-vs-measured column
    has something in it.
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
        return dict(call, claimed=claimed)


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
        for item in response.output:
            if getattr(item, "type", "") == "function_call":
                return {"name": item.name,
                        "arguments": json.loads(item.arguments),
                        "claimed": ""}
        return {"name": None, "arguments": {},
                "claimed": getattr(response, "output_text", "")}


def build_model(dry_run: bool):
    key = os.environ.get("OPENAI_API_KEY")
    if dry_run or not key:
        if not dry_run:
            print("[astra_loop] no OPENAI_API_KEY; running the scripted stub",
                  file=sys.stderr)
        return ScriptedModel()
    return OpenAIModel(os.environ.get("OPENAI_MODEL", "gpt-5"), key)


class Mirror:
    """The scene the KinematicExecutor moves: the block follows a closed hand."""

    def __init__(self, kin):
        self.kin = kin
        self.executor = KinematicExecutor(kin)
        self.executor.next_object["left"] = "red_block"
        self.executor.next_object["right"] = "red_block"
        self.block = np.array(BLOCK_P, dtype=float)

    def world(self):
        held = {s: self.executor.held.get(s) for s in ("left", "right")}
        return observe(self.kin, block_p=self.block,
                       closed=dict(self.executor.grippers), held=held)

    def run(self, plan) -> int:
        state = self.executor.state()
        joints = {s: np.array(q, dtype=float) for s, q in state.joints.items()}
        grippers = dict(state.grippers)
        sent = 0
        for step in plan.steps:
            if isinstance(step, JointStep):
                joints[step.side] = np.asarray(step.q, dtype=float)
                self.executor.send_joints(wire(joints, grippers), t=sent * 0.02)
            elif isinstance(step, GripStep):
                self.executor.set_gripper(step.side, step.closedness, grip=step.grip)
                grippers[step.side] = float(step.closedness)
            sent += 1
            for side in ("left", "right"):
                if self.executor.held.get(side) == "red_block":
                    self.block = tool_from_link7(*self.kin.ee_pose(side))[0].copy()
        return sent


def loop(model, *, max_turns: int = 8, trace_path: Optional[Path] = None,
         goal=None) -> DecisionTrace:
    world0, kin = demo_scene()
    # WHICH HAND — decided before anything moves, by planning the whole chain
    # (Approach, Grasp, Lift, Carry, Place) for BOTH arms and taking the one
    # that can DELIVER. The near hand is only the tie-break; see chain.py for
    # what that cost on the blocks-eval wagon (2026-09-19, F10).
    hand = choose_side(world0, kin, obj="red_block", destination="box")
    if goal is None:
        goal = Place(object="red_block", to="box", side=hand.side)
    mirror = Mirror(kin)
    trace = DecisionTrace(trace_path)
    messages: List[Dict[str, Any]] = [{"role": "system", "content": SYSTEM}]
    tools = tool_schemas(mirror.world())

    if isinstance(model, ScriptedModel):
        model.use_side(hand.side)
    for turn in range(max_turns):
        world = mirror.world()
        messages.append({"role": "user", "content": world.to_text()})
        record = DecisionRecord(iteration=turn, world=world.to_json())

        call = model(messages, tools)
        record.choice = {"name": call["name"], "arguments": call["arguments"]}
        record.claimed = call.get("claimed") or None
        if not call["name"]:
            trace.write(record)
            break

        # The gate, on the BOUND call. A model may ask for anything; the guard
        # still decides. This is the step PR #17 was missing.
        try:
            primitive = by_verb(call["name"])(**call["arguments"])
        except (TypeError, ValueError) as exc:
            messages.append({"role": "user", "content": f"that call is malformed: {exc}"})
            trace.write(record)
            continue
        offered, refused = offer([primitive], world, kin)
        record.refused = [r.to_json() for r in refused]
        if not offered:
            messages.append({"role": "user", "content": why_nothing(refused)})
            trace.write(record)
            continue

        chosen = offered[0]
        record.plan = chosen.plan.to_json()
        sent = mirror.run(chosen.plan)
        record.run = {"steps_sent": sent}

        after = mirror.world()
        report = primitive.verifier(world)(after)
        record.verdict = report.to_json()
        messages.append({"role": "user",
                         "content": f"{chosen.label}: {report.verdict} — {report.reason}"})
        goal_report = goal.verifier(world)(after)
        record.goal_verdict = goal_report.to_json()
        trace.write(record)
        if goal_report.verdict == "true":
            break
    return trace


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dry-run", action="store_true",
                        help="use the scripted stub even if a key is present")
    parser.add_argument("--max-turns", type=int, default=8)
    parser.add_argument("--trace", type=Path, default=None)
    args = parser.parse_args(argv)

    trace = loop(build_model(args.dry_run), max_turns=args.max_turns,
                 trace_path=args.trace)
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
