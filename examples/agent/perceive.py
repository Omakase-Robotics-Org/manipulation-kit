"""One head frame -> a scene file, with NO per-scene calibration anywhere in it.

A thin command line over :mod:`manipulation_kit.perception`, which owns every
number: the camera model, the plane fit, the height policy, the measurement
and the scene file. This file decodes the image, parses flags, and picks a
detector — that is all, and ``tests/perception`` is where the numerics are
pinned.

The rule it obeys (Shu, 2026-09-22 — 「中途半端にこっちでシーンごとの calib を
するのは消したい」): ROBOT-specific calibration is allowed — the head camera's
intrinsics, the neck joints, the lift. SCENE-specific numbers are not inputs:
no table width, no far-edge x, no table height. ``--table-width`` (a known
length, solves the height) and ``--table-z`` (declares it) are optional; with
neither the height is ``provisional`` and the agent loop's model is expected
to declare it.

``--detector model`` (the default) finds NOTHING on purpose: the loop's own
model is shown the frame and declares the objects itself. ``--detector astra``
is a separate box-detector call (``detector.py``); ``--detector mask`` is the
kit's two-colour fallback that needs no key.

    # the default: nothing but the robot's own numbers
    python examples/agent/perceive.py --image head.jpg \
        --neck-pitch 0.52 --neck-yaw 0.0 --lift 0.205 \
        --out scenes/live.json --debug /tmp/fit.png

    # ...and the same frame with the optional known length, for comparison
    python examples/agent/perceive.py --image head.jpg --neck-pitch 0.52 \
        --table-width 0.60 --objects charger:object,cup:container \
        --detector mask
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parent))

from manipulation_kit.perception import (  # noqa: E402  (after the path insert)
    DEFAULT_FX, HeadCamera, HeadCameraConfig, PlaneFitError, perceive_frame,
    read_intrinsics, requested_objects)
from manipulation_kit.perception.measure import (  # noqa: E402
    debug_image, detect_objects_mask)


def load_image(path):
    """An ``(H, W, 3)`` uint8 RGB array from a file: Pillow, else OpenCV.

    The one thing the kit will not do is decode an image; this is where the
    example does it.
    """
    import numpy as np  # noqa: PLC0415
    try:
        from PIL import Image  # noqa: PLC0415
    except ImportError:
        pass
    else:
        with Image.open(path) as handle:
            return np.asarray(handle.convert("RGB"), dtype=np.uint8)
    try:
        import cv2  # noqa: PLC0415
    except ImportError as exc:      # pragma: no cover - environment specific
        raise RuntimeError(
            "reading an image needs Pillow or OpenCV: pip install pillow "
            "(or opencv-python-headless)") from exc
    bgr = cv2.imread(str(path))     # pragma: no cover - environment specific
    if bgr is None:
        raise FileNotFoundError(path)
    return np.ascontiguousarray(bgr[:, :, ::-1])


def write_png(path, rgb) -> None:
    import numpy as np  # noqa: PLC0415
    try:
        from PIL import Image  # noqa: PLC0415
    except ImportError:             # pragma: no cover - environment specific
        import cv2  # noqa: PLC0415
        cv2.imwrite(str(path), np.asarray(rgb)[:, :, ::-1])
        return
    Image.fromarray(np.asarray(rgb, dtype=np.uint8)).save(str(path))


def _requested(spec: str) -> List[Dict[str, str]]:
    try:
        return requested_objects(spec)
    except ValueError as exc:
        raise SystemExit(f"--objects: {exc}") from None


def parse_extents(specs: Sequence[str], flag: str) -> Dict[str, List[float]]:
    """``["cup=0.08,0.08,0.10"]`` -> ``{"cup": [0.08, 0.08, 0.10]}``, for
    ``--interior`` and ``--size``: the two things one view cannot see."""
    out: Dict[str, List[float]] = {}
    for spec in specs or ():
        if "=" not in spec:
            raise SystemExit(f"{flag}: {spec!r} is not NAME=LX,LY,LZ")
        name, values = spec.split("=", 1)
        try:
            numbers = [float(v) for v in values.split(",")]
        except ValueError:
            raise SystemExit(f"{flag} {name}: {values!r} is not three "
                             f"numbers") from None
        if len(numbers) != 3 or min(numbers) <= 0:
            raise SystemExit(f"{flag} {name}: need three positive metres, "
                             f"got {values!r}")
        out[name.strip()] = numbers
    return out


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="perceive.py", description=__doc__.splitlines()[0],
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--image", type=Path, required=True,
                        help="one head-camera frame (jpg/png)")
    robot = parser.add_argument_group(
        "the robot's own calibration — the only kind allowed here")
    robot.add_argument("--fx", type=float, default=DEFAULT_FX,
                       help="focal length in pixels")
    robot.add_argument("--fy", type=float, default=None,
                       help="vertical focal length (default: fx)")
    robot.add_argument("--cx", type=float, default=None,
                       help="principal point x (default: the image centre)")
    robot.add_argument("--cy", type=float, default=None,
                       help="principal point y (default: the image centre)")
    robot.add_argument("--intrinsics", type=Path, default=None,
                       help="read fx/fy/cx/cy from a camera JSON instead")
    robot.add_argument("--neck-pitch", type=float, default=None,
                       help="URDF neck_tilt joint of a RECORDED frame, "
                            "radians, POSITIVE LOOKS DOWN (not the daemon's "
                            "logical pitch; a live run reads the neck state "
                            "itself). Default 0")
    robot.add_argument("--neck-yaw", type=float, default=None,
                       help="URDF neck_pan joint, radians. Default 0")
    robot.add_argument("--lift", type=float, default=None,
                       help="slider height_m; does NOT move the camera in "
                            "base, only reports the table's floor height")
    robot.add_argument("--robot-profile", default=None, metavar="NAME|FILE",
                       help="this robot's measured profile (e.g. d1-2): its "
                            "head-camera MOUNT is applied on top of the URDF "
                            "nominal and the camera says calibrated. Without "
                            "it the frame is the nominal one")
    scene = parser.add_argument_group(
        "OPTIONAL scene numbers — you should not need any of these")
    scene.add_argument("--table-width", type=float, default=None,
                       help="a known length across the table's far edge; "
                            "solves the plane HEIGHT, which one camera "
                            "cannot measure")
    scene.add_argument("--table-z", type=float, default=None,
                       help="declare the table-top z in base outright")
    scene.add_argument("--table-depth", type=float, default=None,
                       help="override the near-to-far depth, metres")
    scene.add_argument("--table-name", default="table")
    scene.add_argument("--interior", action="append", default=[],
                       metavar="NAME=LX,LY,LZ",
                       help="declare a container's interior as MEASURED "
                            "(metres); without it Place refuses to put "
                            "anything in it. Repeatable.")
    scene.add_argument("--size", action="append", default=[],
                       metavar="NAME=LX,LY,LZ",
                       help="declare an object's size as MEASURED (metres). "
                            "Repeatable.")
    parser.add_argument("--objects", default="",
                        help="name:kind[:colour] list for --detector "
                             "astra|mask, e.g. "
                             "'charger:object:white,cup:container:brown'")
    parser.add_argument("--detector", choices=("model", "astra", "mask"),
                        default="model",
                        help="'model' (the default) detects NOTHING here and "
                             "leaves the things to the loop's own model. "
                             "'astra' is a separate box-detector call; "
                             "'mask' is a colour fallback kept for the tests")
    parser.add_argument("--model", default="gpt-6-astra",
                        help="the detector model for --detector astra")
    parser.add_argument("--out", type=Path, default=None,
                        help="write the scene here as well as printing it")
    parser.add_argument("--debug", type=Path, default=None,
                        help="write an annotated PNG of the fit here")
    return parser


def head_camera(args: argparse.Namespace, width: int, height: int, *,
                neck=None, lift=None) -> HeadCamera:
    """The camera: from the LIVE neck/lift states when given (typed, from the
    executor), else from the recorded frame's joint flags. Both at once is
    two answers to one question, and is refused. ``--robot-profile`` adds the
    robot's MEASURED head mount."""
    from manipulation_kit.description.robot_profile import (  # noqa: PLC0415
        RobotProfile)
    profile = RobotProfile.resolve(getattr(args, "robot_profile", None))
    mount = None if profile is None else profile.head_mount_delta
    intrinsics = {"fx": args.fx, "fy": args.fy, "cx": args.cx, "cy": args.cy}
    if args.intrinsics is not None:
        intrinsics.update(read_intrinsics(args.intrinsics))
    if neck is not None:
        if args.neck_pitch is not None or args.neck_yaw is not None:
            raise SystemExit("--neck-pitch/--neck-yaw given AND a live neck "
                             "state: two answers to where the head points")
        if args.lift is not None and lift is not None:
            raise SystemExit("--lift given AND a live lift state")
        return HeadCamera.from_config(HeadCameraConfig.from_intrinsics(
            intrinsics, width=width, height=height, neck=neck, lift=lift),
            mount_delta=mount)
    return HeadCamera.from_robot(
        width=width, height=height, fx=intrinsics["fx"],
        fy=intrinsics.get("fy"), cx=intrinsics.get("cx"),
        cy=intrinsics.get("cy"), neck_pitch=args.neck_pitch or 0.0,
        neck_yaw=args.neck_yaw or 0.0, lift_m=args.lift, mount_delta=mount)


def _mask(frame, plane, wanted):
    try:
        return detect_objects_mask(frame, plane, wanted)
    except ValueError as exc:
        raise SystemExit(f"--detector mask: {exc}") from None


def perceive(args: argparse.Namespace, *, neck=None,
             lift=None) -> Dict[str, Any]:
    """The whole pipeline, as a function, so the loop can call it."""
    image = load_image(args.image)
    camera = head_camera(args, image.shape[1], image.shape[0], neck=neck,
                         lift=lift)
    requested = _requested(args.objects) if args.objects else []
    detector = None
    if args.detector == "mask":
        detector = _mask
    elif args.detector == "astra":
        from detector import AstraDetector  # noqa: PLC0415
        astra = AstraDetector(model=args.model)
        detector = lambda frame, plane, wanted: astra.detect(frame, wanted)  # noqa: E731
    result = perceive_frame(
        image, camera, table_z=args.table_z, known_length_m=args.table_width,
        requested=requested, detector=detector, table_name=args.table_name,
        table_depth_m=args.table_depth,
        interiors=parse_extents(args.interior, "--interior"),
        sizes=parse_extents(args.size, "--size"),
        source_image=str(args.image))
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(result.scene, indent=1) + "\n",
                                  encoding="utf-8")
    if args.debug:
        Path(args.debug).parent.mkdir(parents=True, exist_ok=True)
        write_png(args.debug, debug_image(image, result.plane,
                                          result.detections))
    return result.scene


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        scene = perceive(args)
    except PlaneFitError as exc:
        print(f"[perceive] the plane fit refused ({exc.reason}): {exc.detail}",
              file=sys.stderr)
        return 2
    print(json.dumps(scene, indent=1))
    table = scene["_perceive"]["table"]
    print(f"\ntable top z = {table['z']:+.3f} m in base "
          f"({table['height_source']}, "
          f"+-{table['height_uncertainty_m'] * 1000:.0f} mm)", file=sys.stderr)
    for item in scene["objects"]:
        print(f"{item['name']:>12}  p {item['p']}  size {item['size']}  "
              f"confidence {item['confidence']}", file=sys.stderr)
    print(f"level correction "
          f"{scene['_perceive']['diagnostics']['level_correction_deg']:+.1f} "
          f"deg", file=sys.stderr)
    if args.out:
        print(f"wrote {args.out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
