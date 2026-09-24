"""System 2 decides, System 1 aligns: Astra drives the verbs, Jev-Omni judges
the wrist look — ``astra_loop`` with a :class:`manipulation_kit.agent.Servo`.

Jev-Omni (``akhilaaa3/Jev-Omni``, Apache-2.0) is a 12B multimodal decision
CLASSIFIER: it cannot write a pixel, but it places the object RELATIVE TO A
MARK the kit has drawn; the kit does the rest (``manipulation_kit.agent.servo``).

The model's side is here and in ``jev_judge.py``; nothing here decides a
direction, a step or whether the stroke may run::

    python examples/agent/jev_servo.py --dry-run --misplace-mm 40   # no model, no key
    python examples/agent/jev_servo.py --model gpt-6-astra --executor firmware \\
        --robot http://d1-2:4750 --scene my_scene.json --robot-profile PATH \\
        --snapshot-cmd "grab_frames.sh --turn {turn} --out {out_dir}"
    # the judge on a workstation (jev_judge_server.py), recorded, never stepping:
    ... --judge-url http://100.x.y.z:8766 --judge-only
    # offline, a finished run's wrist photos judged again (nothing moves):
    ... --rejudge run/ --robot-profile PATH --judge-url http://127.0.0.1:8766

``--judge jev`` or the server need ``huggingface_hub``, ``torch``,
``torchvision``, ``transformers`` and a CUDA GPU with ~47 GB free.
"""

from __future__ import annotations

import dataclasses
import json
import sys
import time
from pathlib import Path
from typing import Optional, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parent))

import astra_loop  # noqa: E402
from jev_judge import LABELS, JevJudge, RemoteJudge, rejudge  # noqa: E402,F401
from manipulation_kit.agent import (DecisionTrace, LiveRobot,  # noqa: E402
                                    OperatorPolicy, Servo, UnknownExecutor,
                                    geometry_judge, run)
from manipulation_kit.agent.judge import ALL_VIEWS, FORMULATIONS  # noqa: E402
from manipulation_kit.agent.servo import DEFAULT_BUDGET_S, servo_line  # noqa: E402
from manipulation_kit.agent.robot import (  # noqa: E402
    frames_from, head_camera_from_scene, objects_from,
    wrist_camera_from_scene, with_declared_hand)
from manipulation_kit.primitives import Place  # noqa: E402
from run_scene import resolve_profile, scene_for_run  # noqa: E402
from scene import DEMO_WRIST_CAMERA, demo_scene  # noqa: E402
from scripted import ScriptedModel, two_things_on  # noqa: E402

class SaidTrace(DecisionTrace):
    """The trace, and one line per judged stroke as it happens."""

    def write(self, record):
        if record.servo is not None:
            print(f"turn {record.iteration}: {servo_line(record.servo)}",
                  flush=True)
        return super().write(record)


def wrist_frames(snapshotter):
    """``Servo``'s frame seam over the example's camera-grab contract: one
    fresh grab per look, that hand's wrist file out of it."""
    counter = {"n": 1000}

    def frame(side: str) -> Optional[Path]:
        counter["n"] += 1
        for _label, path in snapshotter.capture(counter["n"]):
            if path.name.endswith(f"_{side}_wrist_0_rgb.jpg"):
                return path
        return None
    return frame


def build_parser():
    parser = astra_loop.build_parser()
    parser.description = __doc__.splitlines()[0]
    add = parser.add_argument
    add("--judge", choices=("jev", "remote", "geometry"), default=None,
        help="jev: the classifier here; remote: it behind "
             "jev_judge_server.py (--judge-url); geometry: the no-photo "
             "stand-in (default under --dry-run)")
    add("--judge-url", default=None, help="implies --judge remote")
    add("--judge-only", action="store_true", help="judge and record every "
        "wrist look, never step")
    add("--refine", action="store_true", help="after 'on', move the "
        "DECLARATION to the best-judged mark 5 mm around it (no motion)")
    add("--rejudge", type=Path, default=None, metavar="RUN_DIR",
        help="offline: judge the wrist photos a live run saved in RUN_DIR "
             "(trace.jsonl + turn*_*_wrist_0_rgb.jpg) and print each answer")
    add("--misplace-mm", type=float, default=0.0, help="geometry judge: "
        "the TRUE object is this far (base +y) from the declaration")
    add("--judge-questions", choices=FORMULATIONS, default="choice")
    add("--judge-views", default=",".join(ALL_VIEWS), help="mirrored views "
        "averaged per photo, e.g. rot180 (default: all four)")
    add("--servo-budget-s", type=float, default=DEFAULT_BUDGET_S)
    add("--verbose-servo", action="store_true",
        help="print each judgement's per-question answers too")
    return parser


def make_judge(parser, args, kind: str, world0):
    how = dict(formulation=args.judge_questions, debug=args.verbose_servo,
               views=tuple(v for v in args.judge_views.split(",") if v))
    if kind == "jev":
        return JevJudge(**how)
    if kind == "remote":
        if not args.judge_url:
            parser.error("--judge remote needs --judge-url")
        return RemoteJudge(args.judge_url, **how)
    declared = world0.find(args.object)
    if declared is None:
        parser.error(f"{args.object!r} is not in the scene; the geometry "
                     f"judge needs it declared")
    return geometry_judge(
        {args.object: declared.p + [0.0, args.misplace_mm / 1000.0, 0.0]})


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
    if args.rejudge is not None:
        from manipulation_kit.description.robot_profile import with_profile
        scene = scene_for_run(args, profile=profile) if args.scene else None
        wrist = wrist_camera_from_scene(
            with_profile(scene, profile) if profile else scene,
            measured_only=True) or {}
        if not wrist:
            parser.error("--rejudge needs the wrist lenses: --robot-profile "
                         "PATH or a --scene naming one")
        world0, kin = demo_scene()
        judge = make_judge(parser, args, args.judge or (
            "remote" if args.judge_url else "jev"), world0)
        return 0 if rejudge(args.rejudge, judge, kin=kin, wrist=wrist) else 1
    snapshotter = None
    run_dir = (args.trace or Path("run/trace.jsonl")).parent
    if args.snapshot_cmd:
        from snapshot import Snapshotter  # noqa: PLC0415
        snapshotter = Snapshotter(args.snapshot_cmd, run_dir)
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
    if robot.head_camera is None:
        robot.head_camera = head_camera_from_scene(scene)
    dry = args.dry_run or not args.model
    judge_kind = args.judge or ("remote" if args.judge_url else
                                "geometry" if dry else "jev")
    if judge_kind != "geometry" and snapshotter is None:
        parser.error(f"--judge {judge_kind} needs --snapshot-cmd: it judges "
                     f"a photo")
    servo = Servo(wrist_frames(snapshotter) if snapshotter else lambda s: None,
                  make_judge(parser, args, judge_kind, world0), out_dir=run_dir,
                  observe_only=args.judge_only, refine=args.refine,
                  budget_s=args.servo_budget_s,
                  log=lambda line: print(f"[servo] {line}", file=sys.stderr,
                                         flush=True))
    if dry:
        model = ScriptedModel(args.object, args.destination, declare=(
            two_things_on(world0, obj=args.object, destination=args.destination)
            if scene is not None else None))
    else:
        model = astra_loop.OpenAIModel(args.model, reasoning=args.reasoning,
                                       max_output_tokens=args.max_output_tokens,
                                       debug=args.debug)
    with robot:
        trace = run(goal=Place(object=args.object, to=args.destination),
                    robot=robot, policy=policy, ask=model, task=args.task,
                    system=astra_loop.SYSTEM, trace=SaidTrace(args.trace),
                    observe=snapshotter.observe if snapshotter else None,
                    on_side=getattr(model, "use_side", None), servo=servo)
    for record in trace.records:
        said = f"  [{servo_line(record.servo)}]" if record.servo else ""
        print(f"turn {record.iteration}: "
              f"{(record.choice or {}).get('name') or '(no call)':9s} -> "
              f"{(record.verdict or {}).get('verdict', '-')}{said}")
    print("\n" + json.dumps(trace.summary(), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
