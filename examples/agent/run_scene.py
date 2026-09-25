"""Where an ``astra_loop.py`` run's scene comes from, before turn 0.

MEASURED from one head frame (``--perceive IMAGE|snapshot``, through
``perceive.py``, with the live neck on ``--executor firmware``) or READ from a
scene file (``--scene``, its ``robot`` block resolved against the robot
profile). Split out of ``astra_loop.py`` so the loop file stays a prompt and
a call.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Optional

import perceive


def perceived_scene(source: str, *, trace_path: Optional[Path], obj: str,
                    destination: str, options: str = "", neck=None, lift=None,
                    snapshotter=None, profile: Optional[Path] = None,
                    allow_failed_calibration: bool = False
                    ) -> Dict[str, Any]:
    """``astra_loop.py --perceive``: MEASURE the scene from one head frame (a
    path, or ``snapshot`` = one grab before turn 0), once, before turn zero;
    written beside the trace as ``scene_perceived.json``."""
    import shlex  # noqa: PLC0415
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
    if profile and "--robot-profile" not in tokens:
        tokens += ["--robot-profile", str(profile)]
    if allow_failed_calibration and "--allow-failed-calibration" not in tokens:
        tokens.append("--allow-failed-calibration")
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


def add_profile_arguments(parser) -> None:
    """``--robot-profile PATH`` and ``--allow-failed-calibration``."""
    parser.add_argument(
        "--robot-profile", type=Path, default=None, metavar="PATH",
        help="the robot's omakase.camera_calibration/2 file (hand gap, head "
             "mount, wrist lenses). Default: "
             "~/.config/omakase/camera_calibration.json when it exists (on "
             "the robot), else the file a --scene names, else none")
    parser.add_argument(
        "--allow-failed-calibration", action="store_true",
        help="accept a calibration layer whose gate FAILED (refused "
             "otherwise, unless the file records an override)")


def resolve_profile(args):
    """The :class:`RobotProfile` of :func:`robot_profile_path`, or ``None``."""
    from manipulation_kit.description.robot_profile import RobotProfile  # noqa: PLC0415
    return RobotProfile.resolve(robot_profile_path(args),
                                allow_failed_gate=args.allow_failed_calibration)


def robot_profile_path(args) -> Optional[Path]:
    """The calibration file a run uses: ``--robot-profile`` when given; else
    none when ``--scene`` names its own (``"robot": {"profile": PATH}``);
    else the robot's installed file
    (``~/.config/omakase/camera_calibration.json``, when it exists); else
    none."""
    from manipulation_kit.description.camera_calibration import (  # noqa: PLC0415
        installed_path)
    if args.robot_profile is not None:
        return Path(args.robot_profile)
    scene = getattr(args, "scene", None)
    if scene is not None and Path(scene).is_file():
        named = (json.loads(Path(scene).read_text(encoding="utf-8"))
                 .get("robot") or {}).get("profile")
        if named is not None:
            return None
    return installed_path()


def scene_for_run(args, *, snapshotter=None, profile=None
                  ) -> Optional[Dict[str, Any]]:
    """The scene an ``astra_loop.py`` run starts from: MEASURED from one head
    frame (``--perceive``, with the live neck on ``--executor firmware``),
    READ from ``--scene`` (resolved against ``profile``, the resolved
    ``--robot-profile``), or ``None``."""
    if args.perceive is not None:
        neck, lift = (robot_head_state(args.robot) if args.executor == "firmware"
                      else (None, None))
        return perceived_scene(args.perceive, trace_path=args.trace,
                               obj=args.object, destination=args.destination,
                               options=args.perceive_opts, neck=neck, lift=lift,
                               snapshotter=snapshotter,
                               profile=robot_profile_path(args),
                               allow_failed_calibration=args.allow_failed_calibration)
    if args.scene is not None:
        from manipulation_kit.agent.robot import load_scene  # noqa: PLC0415
        return load_scene(args.scene, profile=profile,
                          allow_failed_gate=args.allow_failed_calibration)
    return None
