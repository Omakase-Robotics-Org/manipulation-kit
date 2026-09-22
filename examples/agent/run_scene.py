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
                    snapshotter=None, profile: Optional[str] = None
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


def scene_for_run(args, *, snapshotter=None, profile=None
                  ) -> Optional[Dict[str, Any]]:
    """The scene an ``astra_loop.py`` run starts from: MEASURED from one head
    frame (``--perceive``, with the live neck on ``--executor firmware``),
    READ from ``--scene`` (resolved against ``--robot-profile``), or ``None``."""
    if args.perceive is not None:
        neck, lift = (robot_head_state(args.robot) if args.executor == "firmware"
                      else (None, None))
        return perceived_scene(args.perceive, trace_path=args.trace,
                               obj=args.object, destination=args.destination,
                               options=args.perceive_opts, neck=neck, lift=lift,
                               snapshotter=snapshotter,
                               profile=args.robot_profile)
    if args.scene is not None:
        from manipulation_kit.agent.robot import load_scene  # noqa: PLC0415
        return load_scene(args.scene, profile=profile)
    return None
