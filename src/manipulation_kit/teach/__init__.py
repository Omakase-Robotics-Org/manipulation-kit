"""Teach D1 gestures by hand, for the omakaseos gesture library.

The port of the teach tool that stopped working when d1-firmwared took the arm
link (d1-sdk ``gesture_record`` + omakase-core ``/d1_teach``), over the
daemon's generated client and the kit's :class:`FirmwareExecutor`:

    record   (record.py)   brakes off (default), hand-guide, sample -> raw JSON
    keyframes(process.py)  cut the sag, smooth, reduce, HOME in/out, limit dynamics
    check    (check.py)    the daemon's spline: limits + rates + timing (hard),
                           guard clearance (advisory)
    export   (export.py)   check-then-write the omakaseos CSV (gesture_csv.py)
    play     (play.py)     through FirmwareExecutor, lease and barriers included
    register (registry.py) the gesture.yaml entry omakaseos reads

CLI: ``mkit-teach`` (:mod:`manipulation_kit.teach.cli`). Operator guide:
``docs/teach.md``.
"""
from __future__ import annotations

from .check import CheckReport, GuardFinding, ascii_preview, check_gesture
from .export import UnsafeGesture, export
from .gesture_csv import (HEADER, JOINT_NAMES, Gesture, GestureFormatError,
                          Keyframe, load_csv, load_home, parse_csv, save_csv,
                          to_csv, trajectory_points)
from .process import (DEFAULT_SPEED, KeyframeOptions, SpeedPolicy, SpeedStretch,
                      keyframes_from_poses,
                      keyframes_from_samples, limit_joint_dynamics,
                      settle_index, smooth_samples, trim_idle)
from .record import GUIDES, Recording, record

__all__ = [
    "CheckReport", "DEFAULT_SPEED", "GUIDES", "Gesture", "GuardFinding", "GestureFormatError", "HEADER",
    "JOINT_NAMES", "Keyframe", "KeyframeOptions", "Recording", "UnsafeGesture",
    "ascii_preview", "check_gesture", "export", "keyframes_from_poses",
    "keyframes_from_samples", "limit_joint_dynamics", "load_csv", "load_home",
    "parse_csv", "record", "save_csv", "settle_index", "smooth_samples",
    "SpeedPolicy", "SpeedStretch", "to_csv", "trim_idle",
    "trajectory_points",
]
