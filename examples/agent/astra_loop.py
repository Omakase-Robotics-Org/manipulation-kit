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
from mirror import MirrorRobot, SceneMirrorRobot  # noqa: E402
from scene import demo_scene  # noqa: E402
from trace import DecisionRecord, DecisionTrace  # noqa: E402

DEFAULT_TASK = "put the red block in the box"

SYSTEM = """You drive a D1 humanoid's two arms through a fixed set of verbs.
Each observation may carry three photos: the head camera (scene from above the
torso) and both wrist cameras (each looking along its hand past the jaws).
Use them to judge what the text cannot: whether the object stands or has
tipped, whether the jaws straddle it, whether it is inside the container.
Object positions in the text come from a measured scene and can be off by
1-2 cm; when a photo contradicts the text, say which and act on the photo
(nudge, re-approach with a smaller standoff, release and retry).

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
        def clean(message):
            content = message.get("content")
            if not isinstance(content, list):
                return message
            return dict(message, content=[
                {k: v for k, v in p.items() if not k.startswith("_")}
                for p in content])

        response = self.client.responses.create(
            model=self.model,
            input=[clean(m) for m in messages],
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


def build_model(dry_run: bool, obj: str = "red_block", to: str = "box"):
    key = os.environ.get("OPENAI_API_KEY")
    if dry_run or not key:
        if not dry_run:
            print("[astra_loop] no OPENAI_API_KEY; running the scripted stub",
                  file=sys.stderr)
        return ScriptedModel(obj, to)
    return OpenAIModel(os.environ.get("OPENAI_MODEL", "gpt-6-astra"), key)


def _say(messages: List[Dict[str, Any]], call_id: str, text: str) -> None:
    """Feed a result back CORRELATED with the call that produced it.

    The old loop appended every result as anonymous user prose, so a model
    with two outstanding ideas could not tell which one the refusal was about.
    """
    messages.append({"role": "user",
                     "content": (f"[result of {call_id}] {text}" if call_id
                                 else text)})


def _dump_messages(trace_path: Optional[Path], messages: List[Dict[str, Any]]) -> None:
    """Keep the model's whole chat history next to the trace, rewritten every
    turn so a crash mid-turn still leaves it on disk (Shu, 2026-09-22)."""
    if trace_path is None:
        return
    path = Path(trace_path).with_suffix(".messages.json")

    def slim(message):
        content = message.get("content")
        if not isinstance(content, list):
            return message
        parts = [dict(p, image_url=f"<{p.get('_file', 'image')}>")
                 if p.get("type") == "input_image" else p for p in content]
        return dict(message, content=parts)

    path.write_text(json.dumps([slim(m) for m in messages], indent=1,
                               default=str), encoding="utf-8")


def _snapshot(trace_path: Optional[Path], turn: int) -> None:
    """Optional camera record per turn: run ``$ASTRA_SNAPSHOT_CMD`` with
    ``ASTRA_TURN`` and ``ASTRA_OUT_DIR`` set. The model is NOT shown these
    frames (the loop is text-only); they are for the human reading the run."""
    cmd = os.environ.get("ASTRA_SNAPSHOT_CMD")
    if not cmd or trace_path is None:
        return []
    import subprocess  # noqa: PLC0415
    env = dict(os.environ, ASTRA_TURN=str(turn),
               ASTRA_OUT_DIR=str(Path(trace_path).parent))
    try:
        subprocess.run(cmd, shell=True, env=env, timeout=40, check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as exc:  # noqa: BLE001 - a snapshot must never stop a run
        print(f"[astra_loop] snapshot failed: {exc!r}", file=sys.stderr)
        return []
    out = Path(trace_path).parent
    # The model is shown the head and the RIGHT wrist frame of this turn (the
    # left wrist sees nothing useful while the right hand works). Attach in a
    # fixed order so the trace is comparable turn to turn.
    wanted = (f"turn{turn}_base_0_rgb.jpg", f"turn{turn}_right_wrist_0_rgb.jpg",
              f"turn{turn}_left_wrist_0_rgb.jpg")
    return [out / name for name in wanted if (out / name).exists()]


def _observation(text: str, frames) -> Any:
    """The user turn: the world as text, plus this turn's camera frames as
    images when there are any (Shu, 2026-09-22: 'Astra に画像を渡すのは必須')."""
    if not frames:
        return text
    import base64  # noqa: PLC0415
    parts: List[Dict[str, Any]] = [{"type": "input_text", "text": text}]
    for path in frames:
        data = base64.b64encode(Path(path).read_bytes()).decode("ascii")
        parts.append({"type": "input_image", "detail": "high",
                      "image_url": f"data:image/jpeg;base64,{data}",
                      "_file": Path(path).name})
    return parts


def loop(model, robot=None, *, task: str = DEFAULT_TASK, max_turns: int = 8,
         trace_path: Optional[Path] = None, goal=None, world0=None, kin=None,
         obj: str = "red_block", destination: str = "box") -> DecisionTrace:
    if world0 is None or kin is None:
        demo_world, demo_kin = demo_scene()
        world0 = demo_world if world0 is None else world0
        kin = demo_kin if kin is None else kin
    robot = robot if robot is not None else MirrorRobot(kin)
    # WHICH HAND — decided before anything moves, by planning the whole chain
    # (Approach, Grasp, Lift, Carry, Place) for BOTH arms and taking the one
    # that can DELIVER. The near hand is only the tie-break; see
    # manipulation_kit.primitives.reach for what that cost on the blocks-eval
    # wagon (2026-09-19, F10).
    # Flat or awkward objects refuse a top-down grasp (object_too_flat); try
    # the approach directions in order and keep the first reachable chain.
    hand = None
    for approach in ("top_down", "front", "side_right", "side_left"):
        candidate = choose_side(world0, kin, obj=obj, destination=destination,
                                approach=approach)
        if hand is None or (candidate.reachable and not hand.reachable):
            hand = candidate
        if hand.reachable:
            break
    if goal is None:
        goal = Place(object=obj, to=destination, side=hand.side)
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
        _dump_messages(trace_path, messages)
        trace.stop = Stop("unreachable_task", hand.reason).reason
        return trace

    if isinstance(model, ScriptedModel):
        model.use_side(hand.side)
    stop = Stop("max_turns", f"{max_turns} turns without a measured goal")
    for turn in range(max_turns):
        world = robot.world()
        frames = _snapshot(trace_path, turn)
        messages.append({"role": "user",
                         "content": _observation(world.to_text(), frames)})
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
            _dump_messages(trace_path, messages)
            continue
        if not call["name"]:
            # The model stopped. That is a claim about the task, and the task
            # verifier is what decides — the old loop broke here without
            # checking the goal at all, contrary to its own docstring.
            goal_report = goal.verifier(world)(robot.world())
            record.goal_verdict = goal_report.to_json()
            record.stop = "model_stopped"
            trace.write(record)
            _dump_messages(trace_path, messages)
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
            _dump_messages(trace_path, messages)
            continue
        plan = check(primitive, world, kin)
        if not getattr(plan, "ok", False):
            record.refused = [plan.to_json()]
            _say(messages, call_id, f"{label_for(primitive)} was refused: {plan}")
            trace.write(record)
            _dump_messages(trace_path, messages)
            continue
        record.offered = [{"id": f"{primitive.name()}", "label": label_for(primitive)}]

        record.plan = plan.to_json()
        # ASSOCIATION half of the pickup predicate: tell the live adapter WHAT
        # the coming stroke closes on, or every later lift/place is refused
        # with gripper_unknown (d1-2 run2, 2026-09-22: two real grasps, no lift).
        if hasattr(robot, "expect") and primitive.name() in ("grasp",):
            robot.expect(getattr(plan, "side", None) or primitive.side,
                         primitive.object)
        try:
            report = run(plan, robot.executor)
        except Exception as exc:
            # The turn is written BEFORE the exception propagates: run1 on d1-2
            # (2026-09-22) lost its whole trace to an httpx timeout in here.
            record.run = {"completed": False, "stop_reason": "exception",
                          "error": repr(exc)}
            trace.write(record)
            _dump_messages(trace_path, messages)
            raise
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
        _dump_messages(trace_path, messages)
        if goal_report.verdict == "true":
            stop = Stop("goal_verified", goal_report.reason)
            break
    trace.stop = stop.reason
    trace.stop_detail = stop.detail
    return trace


def build_robot(kind: str, kin, robot_url: str, scene=None, world0=None,
                obj: str = "red_block"):
    if kind == "kinematic":
        if world0 is not None:
            return SceneMirrorRobot(kin, world0, obj)
        return MirrorRobot(kin)
    from manipulation_kit.executors.firmware import FirmwareExecutor  # noqa: PLC0415
    from live import LiveRobot  # noqa: PLC0415
    from manipulation_kit.executors.firmware.client import FirmwareClient  # noqa: PLC0415
    # The daemon answers /v1/gripper/{side}/set only when the stroke is done;
    # the client default of 2 s is too short for a real close (d1-2, 2026-09-22).
    client = FirmwareClient(robot_url, timeout=20.0)
    return LiveRobot(FirmwareExecutor(base_url=robot_url, client=client), kin, scene)


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
    parser.add_argument("--scene", type=Path, default=None,
                        help="a MEASURED scene file (examples/agent/scenes/*.json); "
                             "default: the built-in demo scene")
    parser.add_argument("--object", default="red_block",
                        help="the scene object to move (default red_block)")
    parser.add_argument("--destination", default="box",
                        help="the scene container to place it in (default box)")
    args = parser.parse_args(argv)

    _world, kin = demo_scene()
    scene = None
    world0 = None
    if args.scene is not None:
        import time as _time  # noqa: PLC0415
        from live import frames_from, load_scene, objects_from  # noqa: PLC0415
        scene = load_scene(args.scene)
        import dataclasses as _dc  # noqa: PLC0415
        world0 = _dc.replace(_world, objects=tuple(objects_from(scene)),
                                frames=frames_from(scene, now=_time.time()))
    robot = build_robot(args.executor, kin, args.robot, scene,
                        world0=world0, obj=args.object)
    trace = loop(build_model(args.dry_run, args.object, args.destination), robot, task=args.task,
                 max_turns=args.max_turns, trace_path=args.trace,
                 world0=world0, kin=kin, obj=args.object,
                 destination=args.destination)
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
