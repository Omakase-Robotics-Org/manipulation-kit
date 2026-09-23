"""Teach D1 gestures by hand, for the omakaseos gesture library.

The port of the teach tool that stopped working when d1-firmwared took the arm
link (d1-sdk ``gesture_record`` + omakase-core ``/d1_teach``), over the
daemon's generated client and the kit's :class:`FirmwareExecutor`:

    record   (record.py)   hand-guide, sample the joints -> raw recording JSON
    keyframes(process.py)  smooth, snap HOME, reduce, trim idle, limit dynamics
    check    (check.py)    the daemon's spline through guard + limits + rates
    export   (export.py)   check-then-write the omakaseos CSV (gesture_csv.py)
    play     (play.py)     through FirmwareExecutor, lease and barriers included
    register (registry.py) the gesture.yaml entry omakaseos reads

CLI: ``mkit-teach`` (:mod:`manipulation_kit.teach.cli`). Operator guide:
``docs/teach.md``.
"""
from __future__ import annotations

from .check import CheckReport, ascii_preview, check_gesture
from .export import UnsafeGesture, export
from .gesture_csv import (HEADER, JOINT_NAMES, Gesture, GestureFormatError,
                          Keyframe, load_csv, load_home, parse_csv, save_csv,
                          to_csv, trajectory_points)
from .process import (KeyframeOptions, keyframes_from_poses,
                      keyframes_from_samples, limit_joint_dynamics,
                      smooth_samples, trim_idle)
from .record import GUIDES, Recording, record

__all__ = [
    "CheckReport", "GUIDES", "Gesture", "GestureFormatError", "HEADER",
    "JOINT_NAMES", "Keyframe", "KeyframeOptions", "Recording", "UnsafeGesture",
    "ascii_preview", "check_gesture", "export", "keyframes_from_poses",
    "keyframes_from_samples", "limit_joint_dynamics", "load_csv", "load_home",
    "parse_csv", "record", "save_csv", "smooth_samples", "to_csv", "trim_idle",
    "trajectory_points",
]
