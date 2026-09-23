"""System 2 decides, System 1 aligns: Astra drives the verbs, Jev-Omni judges
the wrist look — ``astra_loop`` with a :class:`manipulation_kit.agent.Servo`.

Jev-Omni (``akhilaaa3/Jev-Omni``, Apache-2.0) is a 12B multimodal decision
CLASSIFIER: one image, a question, a few options, a probability per option. It
cannot write a pixel, so it cannot answer the loop's wrist-look question the
way a generative model does. What it can do — measured in report
``jev-servo-loop`` on rendered wrist frames, 8/8 with the mark drawn against
a prior-shaped answer without it — is place the object RELATIVE TO A MARK the
kit has drawn. The kit does the rest (``manipulation_kit.agent.servo``).

The part that knows a model exists is in this file: the words of the question,
the option labels, the client. Nothing here decides a direction, a step or
whether the stroke may run::

    python examples/agent/jev_servo.py --dry-run --misplace-mm 40   # no model, no key
    python examples/agent/jev_servo.py --model gpt-6-astra --executor firmware \\
        --robot http://d1-2:4750 --scene my_scene.json --robot-profile PATH \\
        --snapshot-cmd "grab_frames.sh --turn {turn} --out {out_dir}"

Extra needs beyond the kit: ``huggingface_hub``, ``torch``, ``torchvision``,
``transformers`` and a CUDA GPU with ~26 GiB free for ``--judge jev``; the
model's own ``requirements.txt`` omits ``torchvision`` (needed at load).
"""

from __future__ import annotations

import dataclasses
import json
import sys
import time
from pathlib import Path
from typing import Dict, Optional, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parent))

import astra_loop  # noqa: E402
from manipulation_kit.agent import (DecisionTrace, LiveRobot,  # noqa: E402
                                    OperatorPolicy, Servo, ServoLook,
                                    UnknownExecutor, geometry_judge, run)
from manipulation_kit.agent.robot import (frames_from,  # noqa: E402
                                          head_camera_from_scene, objects_from,
                                          with_declared_hand)
from manipulation_kit.primitives import Place  # noqa: E402
from run_scene import resolve_profile, scene_for_run  # noqa: E402
from scene import DEMO_WRIST_CAMERA, demo_scene  # noqa: E402
from scripted import ScriptedModel, two_things_on  # noqa: E402

#: what each kit choice id is called in the question. The ids are the kit's.
LABELS: Dict[str, str] = {
    "on": "the {object} is entirely inside the green box",
    "left": "the {object} sticks out to the LEFT of the green box",
    "right": "the {object} sticks out to the RIGHT of the green box",
    "above": "the {object} sticks out ABOVE the green box",
    "below": "the {object} sticks out BELOW the green box",
    "not_visible": "the {object} is not in the photo"}

STATE = ("Photo from the camera on a robot hand, looking along the hand past "
         "its two gripper fingers. A green box has been drawn on the photo "
         "where the robot BELIEVES the {object} is, slightly larger than the "
         "{object} should appear; a green cross marks the box's centre.")
QUESTION = "How does the {object} sit relative to the green box?"


class JevJudge:
    """Jev-Omni as the servo's judge. Loaded on first use (~14 s, ~26 GiB)."""

    def __init__(self, debug: bool = False):
        self.classifier, self.debug = None, debug

    def __call__(self, look: ServoLook) -> Dict[str, float]:
        if look.image is None:
            raise RuntimeError("Jev judges a photo; this robot gave none "
                               "(--snapshot-cmd)")
        if self.classifier is None:
            from huggingface_hub import snapshot_download  # noqa: PLC0415
            sys.path.insert(0, snapshot_download("akhilaaa3/Jev-Omni"))
            from jev_omni import load_jev_omni  # noqa: PLC0415
            self.classifier = load_jev_omni()
        name = look.object.replace("_", " ")
        started = time.time()
        labels = {c: LABELS[c].format(object=name) for c in look.choices}
        result = self.classifier.predict(
            state=STATE.format(object=name),
            question=QUESTION.format(object=name),
            options=[labels[c] for c in look.choices],
            media=str(look.image), modality="image")
        by_label = {label: c for c, label in labels.items()}
        out = {by_label[label]: float(p)
               for label, p in result["probabilities"].items()}
        if self.debug:
            print(f"[jev_servo] {(time.time() - started) * 1000:.0f} ms "
                  f"{look.image.name}: {out}", file=sys.stderr)
        return out


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
    parser.add_argument("--judge", choices=("jev", "geometry"), default=None,
                        help="jev: the classifier (needs --snapshot-cmd on a "
                             "robot); geometry: the no-photo stand-in "
                             "(default under --dry-run)")
    parser.add_argument("--misplace-mm", type=float, default=0.0,
                        help="geometry judge: the TRUE object is this far "
                             "(base +y) from where the scene declares it")
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
    judge_kind = args.judge or ("geometry" if dry else "jev")
    if judge_kind == "jev":
        if snapshotter is None:
            parser.error("--judge jev needs --snapshot-cmd: it judges a photo")
        judge = JevJudge(debug=args.debug)
    else:
        declared = world0.find(args.object)
        if declared is None:
            parser.error(f"{args.object!r} is not in the scene; the geometry "
                         f"judge needs it declared")
        truth = declared.p + [0.0, args.misplace_mm / 1000.0, 0.0]
        judge = geometry_judge({args.object: truth})
    servo = Servo(wrist_frames(snapshotter) if snapshotter else lambda s: None,
                  judge, out_dir=run_dir)
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
