"""System 2 decides, System 1 aligns: Astra drives the verbs, Jev-Omni judges
the wrist look — ``astra_loop`` with a :class:`manipulation_kit.agent.Servo`.

Jev-Omni (``akhilaaa3/Jev-Omni``, Apache-2.0) is a 12B multimodal decision
CLASSIFIER: one image, a question, a few options, a probability per option. It
cannot write a pixel, so it cannot answer the loop's wrist-look question the
way a generative model does. What it can do — measured in report
``jev-servo-loop`` on rendered wrist frames, 8/8 with the mark drawn against
a prior-shaped answer without it — is place the object RELATIVE TO A MARK the
kit has drawn. The kit does the rest (``manipulation_kit.agent.servo``).

The part that knows a model exists is here and in ``jev_judge.py`` (the words
of the question, the option labels, the local and remote judge). Nothing here decides a direction, a step or
whether the stroke may run::

    python examples/agent/jev_servo.py --dry-run --misplace-mm 40   # no model, no key
    python examples/agent/jev_servo.py --model gpt-6-astra --executor firmware \\
        --robot http://d1-2:4750 --scene my_scene.json --robot-profile PATH \\
        --snapshot-cmd "grab_frames.sh --turn {turn} --out {out_dir}"
    # the judge on a workstation (jev_judge_server.py), recorded, never stepping:
    ... --judge-url http://100.x.y.z:8766 --judge-only
    # offline, a finished run's wrist photos judged again (nothing moves):
    python examples/agent/jev_servo.py --rejudge run/ --robot-profile PATH \
        --judge-url http://127.0.0.1:8766

Extra needs beyond the kit, for ``--judge jev`` or the server:
``huggingface_hub``, ``torch``, ``torchvision``, ``transformers`` and a CUDA GPU
with ~47 GB free (measured); the model's own ``requirements.txt`` omits
``torchvision`` (needed at load).
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
from manipulation_kit.agent.robot import (  # noqa: E402
    frames_from, head_camera_from_scene, objects_from,
    wrist_camera_from_scene, with_declared_hand)
from manipulation_kit.primitives import Place  # noqa: E402
from run_scene import resolve_profile, scene_for_run  # noqa: E402
from scene import DEMO_WRIST_CAMERA, demo_scene  # noqa: E402
from scripted import ScriptedModel, two_things_on  # noqa: E402

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
    add("--judge-url", default=None, help="the judge server, e.g. "
        "http://100.x.y.z:8766 (implies --judge remote)")
    add("--judge-only", action="store_true",
        help="judge and record every wrist look, never step: the model "
             "answers the look as without a servo")
    add("--refine", action="store_true",
        help="after 'on', re-mark the same photo 5 mm around the declaration "
             "and move the DECLARATION to the best-judged mark (no motion)")
    add("--rejudge", type=Path, default=None, metavar="RUN_DIR",
        help="offline: judge the wrist photos a live run saved in RUN_DIR "
             "(trace.jsonl + turn*_*_wrist_0_rgb.jpg) and print each answer")
    add("--misplace-mm", type=float, default=0.0,
        help="geometry judge: the TRUE object is this far (base +y) from "
             "where the scene declares it")
    return parser


def make_judge(parser, args, kind: str, world0):
    if kind == "jev":
        return JevJudge(debug=args.debug)
    if kind == "remote":
        if not args.judge_url:
            parser.error("--judge remote needs --judge-url")
        return RemoteJudge(args.judge_url, debug=args.debug)
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
                  observe_only=args.judge_only, refine=args.refine)
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
                    system=astra_loop.SYSTEM, trace=DecisionTrace(args.trace),
                    observe=snapshotter.observe if snapshotter else None,
                    on_side=getattr(model, "use_side", None), servo=servo)
    for record in trace.records:
        servo_line = ""
        if record.servo:
            n = sum(1 for s in record.servo["steps"] if s["step_m"])
            servo_line = f"  [servo {record.servo['outcome']}, {n} step(s)]"
        verdict = record.verdict or {}
        print(f"turn {record.iteration}: "
              f"{(record.choice or {}).get('name') or '(no call)':9s} -> "
              f"{verdict.get('verdict', '-')}{servo_line}")
    print("\n" + json.dumps(trace.summary(), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
