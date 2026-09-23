"""observe -> offer -> tool call -> execute -> VERIFY, as a runnable loop.

The part that knows a MODEL exists: the prompt, the OpenAI client, ``main()``.
Everything a customer must TRUST is in the wheel, ``manipulation_kit.agent``
(policy, loop, robot, trace; design C.12). The numbers the kit owns reach the
model as generated ROBOT FACTS, never from this prompt. See examples/README.md::

    python examples/agent/astra_loop.py --dry-run          # scripted, no key
    python examples/agent/astra_loop.py --model gpt-6-astra --executor firmware \\
        --robot http://d1-2:4750 --scene my_scene.json  # + --robot-profile PATH off the robot
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parent))

from manipulation_kit.agent import (DecisionTrace, KinematicMirror,  # noqa: E402
                                    LiveRobot, OperatorPolicy, UnknownExecutor, run)
from manipulation_kit.agent.robot import (frames_from, head_camera_from_scene,  # noqa: E402,F401
                                          objects_from, with_declared_hand)
from manipulation_kit.primitives import Place  # noqa: E402
from run_scene import (add_profile_arguments, perceived_scene,  # noqa: E402,F401
                       resolve_profile, robot_head_state, scene_for_run)
from scene import DEMO_WRIST_CAMERA, demo_scene  # noqa: E402
from scripted import ScriptedModel, two_things_on  # noqa: E402,F401


def default_task(obj: str, destination: str) -> str:  # without --task: ITS things
    return f"put the {obj} into the {destination}"


DEFAULT_TASK = default_task("red_block", "box")

SYSTEM = """You drive a D1 humanoid's two arms through a fixed set of verbs.
Observations may carry labelled photos — the head camera, and the wrist
cameras looking along each hand past the jaws. Use them for what the text
cannot say: whether the object stands, whether the jaws straddle it, whether
it is inside the container.

YOU ARE THE DETECTOR: nothing on this robot measures where things are. First
`declare_scene` each thing the task needs from the head photo, in base
metres, with its tight outer size (its outline, never its shadow).
`locate(u, v)` walks a pixel's ray to the table: use it on the pixel where the
object TOUCHES the table and give its size to get the centre. If the table height is
`provisional`, say so. Re-declare whenever a photo disagrees with the text;
before a grasp the robot takes a wrist look, and a disagreeing wrist photo is
answered with `locate` on that wrist camera.

Rules the robot enforces anyway:
- Never give an orientation; give a `direction`, the way the hand travels.
- ROBOT FACTS and OPERATOR POLICY hold every number the robot owns.
- A refused action comes back with its reason and how far short it was;
  choose differently.
- You do not decide whether the task is done. A measurement does.

Call exactly one tool per turn."""


class OpenAIModel:
    """The real thing, through the Responses API. Imported only when used."""

    def __init__(self, model: str, *, max_output_tokens: int = 12000,
                 reasoning: str = "medium", debug: bool = False):
        from openai import OpenAI  # noqa: PLC0415 - reads OPENAI_API_KEY itself
        self.client, self.model, self.debug = OpenAI(), model, debug
        # a big explicit budget: three photos once spent the default on hidden
        # reasoning and returned nothing (d1-2 2026-09-22)
        self.extra: Dict[str, Any] = {"max_output_tokens": int(max_output_tokens), **(
            {"reasoning": {"effort": reasoning}} if reasoning else {})}

    def __call__(self, messages, tools) -> Dict[str, Any]:
        def clean(m):
            if not isinstance(m.get("content"), list):
                return m
            return dict(m, content=[{k: v for k, v in p.items()
                                     if not k.startswith("_")} for p in m["content"]])
        response = self.client.responses.create(
            model=self.model, input=[clean(m) for m in messages], **self.extra,
            tools=[{"type": "function", **t} for t in tools])
        calls = [i for i in response.output if getattr(i, "type", "") == "function_call"]
        if self.debug:
            print(f"[astra_loop] {response.status} {response.usage}", file=sys.stderr)
        if len(calls) != 1:
            # none: the model stopped; several: the contract is ONE per turn
            return {"name": None, "arguments": {}, "call_id": "",
                    "claimed": "" if calls else getattr(response, "output_text", ""),
                    **({"protocol_error": f"you called {len(calls)} tools; call "
                                          f"exactly one"} if calls else {})}
        item = calls[0]
        return {"name": item.name, "arguments": json.loads(item.arguments),
                "call_id": getattr(item, "call_id", "") or getattr(item, "id", ""),
                "claimed": ""}


def loop(model, robot=None, *, task: Optional[str] = None,
         max_turns: Optional[int] = None, trace_path: Optional[Path] = None,
         world0=None, kin=None, obj: str = "red_block",
         destination: str = "box", camera=None,
         policy: Optional[OperatorPolicy] = None, observe=None):
    """The kit's loop with this file's prompt. No robot: the demo mirror."""
    if robot is None:
        demo_world, demo_kin = demo_scene()
        robot = KinematicMirror(kin or demo_kin, world0 or demo_world,
                                wrist_intrinsics=DEMO_WRIST_CAMERA)
    robot.head_camera = camera if camera is not None else robot.head_camera
    policy = policy or OperatorPolicy()
    policy = policy if max_turns is None else dataclasses.replace(policy, max_turns=max_turns)
    return run(goal=Place(object=obj, to=destination), robot=robot,
               policy=policy, ask=model, system=SYSTEM,
               task=task or default_task(obj, destination),
               trace=DecisionTrace(trace_path), observe=observe,
               on_side=getattr(model, "use_side", None))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    add = parser.add_argument
    for flag, default in (("--task", None), ("--model", None),
                          ("--reasoning", "medium"), ("--executor", "kinematic"),
                          ("--executor-class", None), ("--isaac-url", None),
                          ("--robot", "http://127.0.0.1:4750"), ("--perceive", None),
                          ("--perceive-opts", ""), ("--snapshot-cmd", None),
                          ("--object", "red_block"), ("--destination", "box")):
        add(flag, default=default)
    add("--dry-run", action="store_true", help="the scripted stub, whatever --model says")
    add_profile_arguments(parser)
    add("--max-output-tokens", type=int, default=12000)
    add("--debug", action="store_true")
    add("--trace", type=Path, default=None)
    add("--scene", type=Path, default=None, help="a MEASURED scene file")
    OperatorPolicy.add_arguments(parser)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.scene is not None and args.perceive is not None:
        parser.error("--scene and --perceive are mutually exclusive")
    try:
        policy = OperatorPolicy.from_args(args)
        profile = resolve_profile(args)
    except (ValueError, LookupError, OSError) as exc:
        parser.error(str(exc))
    snapshotter = None
    if args.snapshot_cmd:
        from snapshot import Snapshotter  # noqa: PLC0415
        snapshotter = Snapshotter(args.snapshot_cmd, (args.trace or Path("run/trace.jsonl")).parent)
    (world0, kin), options = demo_scene(), {}
    scene = scene_for_run(args, snapshotter=snapshotter, profile=profile)
    if scene is not None:
        world0 = with_declared_hand(dataclasses.replace(
            world0, objects=tuple(objects_from(scene)),
            frames=frames_from(scene, now=time.time())), scene)
    elif args.executor == "kinematic":
        options = {"world0": world0, "wrist_intrinsics": DEMO_WRIST_CAMERA}
    url = args.isaac_url if args.executor == "isaac" and args.isaac_url else args.robot
    try:
        robot = LiveRobot.from_flag(args.executor, kin=kin, policy=policy,
                                    scene=scene, url=url, profile=profile,
                                    executor_class=args.executor_class, **options)
    except UnknownExecutor as exc:
        parser.error(str(exc))
    robot.head_camera = robot.head_camera or head_camera_from_scene(scene)
    if args.dry_run or not args.model:
        model = ScriptedModel(args.object, args.destination, declare=(
            two_things_on(world0, obj=args.object, destination=args.destination)
            if scene is not None else None))
    else:
        model = OpenAIModel(args.model, max_output_tokens=args.max_output_tokens,
                            reasoning=args.reasoning, debug=args.debug)
    with robot:
        trace = loop(model, robot, task=args.task, trace_path=args.trace,  # None: derived
                     obj=args.object, destination=args.destination, policy=policy,
                     observe=snapshotter.observe if snapshotter else None)
    for record in trace.records:   # a scene tool's line is ITS answer, not a verdict
        verdict, seen = record.verdict or {}, (record.observation_after or {}).get("answer")
        said = (f"-   {seen}" if seen is not None
                else f"{verdict.get('verdict', '-')}   {verdict.get('reason', '')}")
        print(f"turn {record.iteration}: {(record.choice or {}).get('name') or '(no call)':9s}"
              f" -> {said[:76]}")
    print("\n" + json.dumps(trace.summary(), indent=2))
    for record in trace.disagreements():
        measured = record.goal_verdict or record.verdict or {}
        print(f"\nCLAIMED DONE, NOT MEASURED, at turn {record.iteration}: the "
              f"task verifier says {measured.get('verdict')} — {measured.get('reason')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
