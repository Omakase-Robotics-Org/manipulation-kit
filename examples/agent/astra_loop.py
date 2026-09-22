"""observe -> offer -> tool call -> execute -> VERIFY, as a runnable loop.

What is in this file is the part that knows a MODEL exists: the prompt, the
OpenAI client, a scripted stand-in, and ``main()``. Everything a customer has
to TRUST is in the wheel — ``manipulation_kit.agent``: the operator policy,
the loop and its stop conditions, the robot, the trace (design C.12)::

    python examples/agent/astra_loop.py --dry-run          # scripted, no key
    python examples/agent/astra_loop.py --model gpt-6-astra \\
        --executor firmware --robot http://d1-2:4750 --scene my_scene.json
    python examples/agent/astra_loop.py --executor isaac --isaac-url tcp://…

THE EXECUTOR IS A FLAG. ``kinematic`` mirrors the plan onto a model of the
robot, ``firmware`` runs it on a D1, ``isaac`` in d1-isaaclab's sim (when that
package is installed; it registers itself). Same loop, policy, prompt, trace.

The operator policy is flags (``--max-grip soft --allowed-directions down
--vel-ratio 0.15 ...``) or ``--policy FILE``; see
``manipulation_kit.agent.OperatorPolicy``. ``pip install openai`` for
``--model``; the SDK reads ``OPENAI_API_KEY`` itself.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parent))

from manipulation_kit.agent import (DecisionTrace, KinematicMirror,  # noqa: E402
                                    LiveRobot, OperatorPolicy,
                                    UnknownExecutor, run)
from manipulation_kit.agent.robot import (head_camera_from_scene,  # noqa: E402
                                          load_scene, objects_from,
                                          frames_from, with_declared_hand)
from manipulation_kit.primitives import Place  # noqa: E402
from scene import DEMO_WRIST_CAMERA, demo_scene  # noqa: E402

DEFAULT_TASK = "put the red block in the box"

SYSTEM = """You drive a D1 humanoid's two arms through a fixed set of verbs.
Each observation may carry photos: the head camera (scene from above the
torso) and the wrist cameras (each looking along its hand past the jaws), each
labelled. Use them to judge what the text cannot: whether the object stands or
has tipped, whether the jaws straddle it, whether it is inside the container.

YOU ARE THE DETECTOR. Nothing on this robot measures where the things are.
Your first job is to look at the head photo and say, in metres in the robot's
base frame, where each thing the task needs is and how big it is —
`declare_scene`. Re-declare whenever a photo disagrees with the text.

How to get metres out of a photograph:
- the HEAD CAMERA block gives its intrinsics and where its lens is in base;
- `locate(u, v)` walks a pixel's ray to the table plane and returns the
  base-frame point with its uncertainty. USE IT, on the pixel where the object
  TOUCHES the table, and give the object's size to get its centre;
- both arms' TOOL POINTS are in the observation text in the same base metres,
  and the hands are usually in the head photo: your scale reference;
- a single camera cannot measure the table's height. If the text says it is
  `provisional`, say so and declare a better one from what you can see.

Sizes matter as much as positions: `size` is the object's tight outer
dimensions, read from its base outline, never from its shadow. ROBOT FACTS
says what the jaws take; over-reporting a side makes every grasp of it refused
as too wide.

`confidence` below 1 is expected. Declare your best estimate, then FIX IT BY
MEASURING: before every grasp the robot takes a wrist look and tells you where
the object should appear; if the wrist photo disagrees, `locate` on that wrist
camera and the hand is corrected.

Rules the robot enforces anyway:
- You never give an orientation. Give a `direction` — the way the hand travels.
- ROBOT FACTS gives the nudge grid and the directions; OPERATOR POLICY what
  is allowed tonight. Every other number is used as you write it.
- A refused action comes back with the reason and how far short it was. Read
  it and choose differently; repeating it will not work.
- You do not decide whether the task is done. A measurement does.

Call exactly one tool per turn."""


class ScriptedModel:
    """A stand-in for a function-calling model: plays a correct pick-and-place.

    So the loop, the policy, the gate and the verifier are runnable with no
    key and no network. It makes one deliberate mistake (it claims "done" one
    turn early) so the trace's claimed-vs-measured column has something in it,
    and it answers a wrist look by asking again — it cannot see. NOT a result.
    """

    def __init__(self, obj: str = "red_block", to: str = "box",
                 declare: Optional[List[Dict[str, Any]]] = None):
        #: SCRIPTED FICTION: with an empty scene it declares the two things it
        #: was handed (``two_things_on``). A real model reads the photograph.
        self.declare = declare
        side = {"side": "left"}
        self.script: List[Dict[str, Any]] = [
            {"name": "approach", "arguments": dict(object=obj, direction="down",
                                                   **side)},
            {"name": "grasp", "arguments": dict(object=obj, direction="down",
                                                **side)},
            # a shorter hop from a perceived 0.17 m table: 0.10 m of lift from
            # there leaves the arm's envelope — a reach fact, not the loop's
            {"name": "lift", "arguments": dict(object=obj, height_m=0.05
                                               if declare else 0.1, **side)},
            {"name": "carry", "arguments": dict(object=obj, to=to, **side)},
            {"name": "place", "arguments": dict(object=obj, to=to, **side)},
        ]
        self.turn = 0

    def use_side(self, side: str) -> None:
        """Play the script with the hand the task planner chose."""
        for call in self.script:
            call["arguments"]["side"] = side

    def __call__(self, messages, tools) -> Dict[str, Any]:
        if self.declare is not None and "declare_scene" in {t["name"] for t in tools}:
            payload, self.declare = self.declare, None
            return {"name": "declare_scene", "claimed": "",
                    "call_id": "scripted-declare",
                    "arguments": {"objects": payload}}
        last = next((str(m["content"]) for m in reversed(messages)
                     if str(m.get("content", "")).startswith("[result of")), "")
        if "WRIST LOOK" in last and "call grasp again" in last:
            self.turn -= 1          # the photo "agrees": ask again
        if self.turn >= len(self.script):
            return {"name": None, "arguments": {}, "claimed": "done"}
        call = self.script[self.turn]
        self.turn += 1
        # The deliberate over-claim, at the CARRY.
        claimed = "done" if self.turn == len(self.script) - 1 else ""
        return dict(call, claimed=claimed, call_id=f"scripted-{self.turn}")


class OpenAIModel:
    """The real thing, through the Responses API. Imported only when used."""

    def __init__(self, model: str, *, max_output_tokens: int = 12000,
                 reasoning: str = "medium", debug: bool = False):
        from openai import OpenAI  # noqa: PLC0415
        self.client = OpenAI()          # reads OPENAI_API_KEY itself
        self.model, self.debug = model, debug
        # gpt-6-astra with three photos spent its whole default output budget
        # on hidden reasoning and returned NOTHING (d1-2 2026-09-22).
        self.extra: Dict[str, Any] = {"max_output_tokens": int(max_output_tokens)}
        if reasoning:
            self.extra["reasoning"] = {"effort": reasoning}

    def __call__(self, messages, tools) -> Dict[str, Any]:
        def clean(message):
            content = message.get("content")
            if not isinstance(content, list):
                return message
            return dict(message, content=[
                {k: v for k, v in p.items() if not k.startswith("_")}
                for p in content])

        response = self.client.responses.create(
            model=self.model, input=[clean(m) for m in messages], **self.extra,
            tools=[{"type": "function", **t} for t in tools])
        calls = [i for i in response.output
                 if getattr(i, "type", "") == "function_call"]
        if self.debug:
            print(f"[astra_loop] output {[getattr(i, 'type', '?') for i in response.output]}"
                  f" status={getattr(response, 'status', '?')} "
                  f"usage={getattr(response, 'usage', None)}", file=sys.stderr)
        if not calls:
            return {"name": None, "arguments": {},
                    "claimed": getattr(response, "output_text", "")}
        if len(calls) > 1:
            # The contract says exactly one. Taking the first silently taught
            # the model that the rest were executed too.
            return {"name": None, "arguments": {}, "call_id": "",
                    "protocol_error": (f"you called {len(calls)} tools; the "
                                       f"contract is exactly one per turn. "
                                       f"Choose one and call it again."),
                    "claimed": ""}
        item = calls[0]
        return {"name": item.name, "arguments": json.loads(item.arguments),
                "call_id": getattr(item, "call_id", "") or getattr(item, "id", ""),
                "claimed": ""}


def two_things_on(world, *, obj: str, destination: str
                  ) -> Optional[List[Dict[str, Any]]]:
    """Two objects on the widest surface in ``world``, for the STUB only: a
    perceived table with no things, and a stand-in that cannot see. Not
    measurements of anything; the trace says ``scripted-declare``."""
    from manipulation_kit.world import SurfaceView  # noqa: PLC0415
    from scene import BLOCK_P, BOX_P  # noqa: PLC0415
    surfaces = [o for o in world.objects if isinstance(o, SurfaceView)]
    if not surfaces or any(o.name in (obj, destination) for o in world.objects):
        return None
    table = max(surfaces, key=lambda s: float(s.size[0]) * float(s.size[1]))
    top = float(table.p[2]) + float(table.size[2]) / 2.0
    return [{"name": obj, "kind": "object",
             "p": [BLOCK_P[0], BLOCK_P[1], top + 0.025],
             "size": [0.045, 0.02, 0.05], "confidence": 0.3},
            {"name": destination, "kind": "container",
             "p": [BOX_P[0], BOX_P[1], top + 0.03], "size": [0.12, 0.12, 0.06],
             "interior": [0.10, 0.10, 0.05], "confidence": 0.3}]


def loop(model, robot=None, *, task: str = DEFAULT_TASK,
         max_turns: Optional[int] = None, trace_path: Optional[Path] = None,
         world0=None, kin=None, obj: str = "red_block",
         destination: str = "box", camera=None,
         policy: Optional[OperatorPolicy] = None, observe=None):
    """The whole example in one call: the kit's loop with this file's prompt.
    With no robot, the demo scene in the kinematic mirror."""
    if robot is None:
        demo_world, demo_kin = demo_scene()
        robot = KinematicMirror(kin or demo_kin, world0 or demo_world,
                                wrist_intrinsics=DEMO_WRIST_CAMERA)
    if camera is not None:
        robot.head_camera = camera
    policy = policy or OperatorPolicy()
    if max_turns is not None:
        policy = dataclasses.replace(policy, max_turns=max_turns)
    return run(goal=Place(object=obj, to=destination), robot=robot,
               policy=policy, ask=model, task=task, system=SYSTEM,
               trace=DecisionTrace(trace_path), observe=observe,
               on_side=getattr(model, "use_side", None))


def perceived_scene(source: str, *, trace_path: Optional[Path], obj: str,
                    destination: str, options: str = "", neck=None, lift=None,
                    snapshotter=None) -> Dict[str, Any]:
    """``--perceive``: MEASURE the scene from one head frame (a path, or
    ``snapshot`` = one grab before turn 0). Once, before turn zero; the scene
    is written beside the trace as ``scene_perceived.json``."""
    import shlex  # noqa: PLC0415

    import perceive  # noqa: PLC0415
    if source == "snapshot":
        if trace_path is None or snapshotter is None:
            raise SystemExit("--perceive snapshot needs --trace and "
                             "--snapshot-cmd: the frame is written beside the "
                             "trace")
        image = snapshotter.capture(0)[0][1]
    else:
        image = Path(source)
        if not image.exists():
            raise SystemExit(f"--perceive {source}: no such frame")
    tokens = shlex.split(options)
    if "--objects" not in tokens:
        tokens += ["--objects", f"{obj}:object,{destination}:container"]
    args = perceive.build_parser().parse_args(["--image", str(image)] + tokens)
    scene = perceive.perceive(args, neck=neck, lift=lift)
    if trace_path is not None:
        out = Path(trace_path).parent / "scene_perceived.json"
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(scene, indent=1) + "\n", encoding="utf-8")
    return scene


def robot_head_state(robot_url: str):
    """The LIVE neck and lift, typed, from the firmware executor — fails
    closed rather than perceiving through a level-neck default."""
    from manipulation_kit.executors.firmware import FirmwareExecutor  # noqa: PLC0415
    from manipulation_kit.perception import HeadPoseUnknown, read_head_state  # noqa: PLC0415
    try:
        return read_head_state(FirmwareExecutor(base_url=robot_url,
                                                heartbeat=False))
    except HeadPoseUnknown as exc:
        raise SystemExit(f"[astra_loop] --perceive on a live robot needs the "
                         f"neck pose: {exc}") from None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--task", default=DEFAULT_TASK)
    parser.add_argument("--dry-run", action="store_true",
                        help="the scripted stub, whatever --model says")
    parser.add_argument("--model", default=None,
                        help="an OpenAI model name (e.g. gpt-6-astra); "
                             "without it, the scripted stub")
    parser.add_argument("--max-output-tokens", type=int, default=12000)
    parser.add_argument("--reasoning", default="medium")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--executor", default="kinematic",
                        help="firmware | kinematic | isaac | any registered")
    parser.add_argument("--executor-class", default=None,
                        metavar="MODULE:FACTORY")
    parser.add_argument("--robot", default="http://127.0.0.1:4750",
                        help="the executor's address (d1-firmwared URL)")
    parser.add_argument("--isaac-url", default=None,
                        help="the Isaac env server, for --executor isaac")
    parser.add_argument("--trace", type=Path, default=None)
    parser.add_argument("--scene", type=Path, default=None,
                        help="a MEASURED scene file (examples/agent/scenes/)")
    parser.add_argument("--perceive", default=None, metavar="IMAGE|snapshot")
    parser.add_argument("--perceive-opts", default="",
                        help="flags passed verbatim to perceive.py")
    parser.add_argument("--snapshot-cmd", default=None,
                        help="camera grab, with {turn} and {out_dir}; see "
                             "snapshot.py")
    parser.add_argument("--object", default="red_block")
    parser.add_argument("--destination", default="box")
    OperatorPolicy.add_arguments(parser)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.scene is not None and args.perceive is not None:
        parser.error("--scene and --perceive are mutually exclusive: one "
                     "reads a scene somebody measured, the other measures it")
    try:
        policy = OperatorPolicy.from_args(args)
    except ValueError as exc:
        parser.error(str(exc))
    snapshotter = None
    if args.snapshot_cmd:
        from snapshot import Snapshotter  # noqa: PLC0415
        snapshotter = Snapshotter(args.snapshot_cmd,
                                  (args.trace or Path("run/trace.jsonl")).parent)
    world0, kin = demo_scene()
    scene = None
    if args.perceive is not None:
        neck, lift = (robot_head_state(args.robot) if args.executor == "firmware"
                      else (None, None))
        scene = perceived_scene(args.perceive, trace_path=args.trace,
                                obj=args.object, destination=args.destination,
                                options=args.perceive_opts, neck=neck,
                                lift=lift, snapshotter=snapshotter)
    elif args.scene is not None:
        scene = load_scene(args.scene)
    options: Dict[str, Any] = {}
    if scene is not None:
        world0 = with_declared_hand(dataclasses.replace(
            world0, objects=tuple(objects_from(scene)),
            frames=frames_from(scene, now=time.time())), scene)
    elif args.executor == "kinematic":
        options = {"world0": world0, "wrist_intrinsics": DEMO_WRIST_CAMERA}
    url = args.isaac_url if args.executor == "isaac" and args.isaac_url else args.robot
    try:
        robot = LiveRobot.from_flag(args.executor, kin=kin, policy=policy,
                                    scene=scene, url=url,
                                    executor_class=args.executor_class,
                                    **options)
    except UnknownExecutor as exc:
        parser.error(str(exc))
    if robot.head_camera is None:
        robot.head_camera = head_camera_from_scene(scene)
    stub = args.dry_run or not args.model
    model = (ScriptedModel(args.object, args.destination,
                           declare=two_things_on(world0, obj=args.object,
                                                 destination=args.destination)
                           if scene is not None else None)
             if stub else OpenAIModel(args.model,
                                      max_output_tokens=args.max_output_tokens,
                                      reasoning=args.reasoning, debug=args.debug))
    with robot:
        trace = loop(model, robot, task=args.task, trace_path=args.trace,
                     obj=args.object, destination=args.destination,
                     policy=policy,
                     observe=snapshotter.observe if snapshotter else None)
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
