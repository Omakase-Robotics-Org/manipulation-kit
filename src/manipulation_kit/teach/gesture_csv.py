"""The omakaseos D1 gesture CSV — read, written and turned into the knots the
player uploads, exactly as the player does it.

THE CONTRACT (pinned 2026-09-23 from the code that reads it, not from prose)
--------------------------------------------------------------------------
Reader of record: omakase-core ``robot_stack/robots/omakase/d1/
firmware_session.py::trajectory_from_csv`` (commit f62faf24, the player since
the firmwared migration 2050ca49 / faa32a71, 2026-09-08..10). Writer of
record: d1-sdk ``devices/omakase_arm/include/omakase_arm/gesture_csv.h::toCsv``
(what ``gesture_record`` wrote) and omakase-core
``status_server/d1/teach.py::_keyframes_to_csv_text`` (its Python mirror).

* ``#`` lines are comments. A row whose first cell is ``duration``, ``kp`` or
  ``kd`` is skipped. The header is
  ``duration,R1,R2,R3,R4,R5,R6,R7,L1,L2,L3,L4,L5,L6,L7``.
* Every data row has EXACTLY 15 finite numbers: ``duration`` (seconds, > 0,
  the time FROM the previous row TO this one) and 14 joint angles in DEGREES.
  The firmware reader refuses any other count — so there is no gripper
  column, and adding one would make every omakaseos robot refuse the file.
* Joint order: ``R1..R7`` = SDK ArmSide **A** = the physical LEFT arm = the
  kit's logical ``left``; ``L1..L7`` = ArmSide **B** = physical RIGHT. The
  R/L tags are the vendored mesh trees' historical names, not sides.
* HOME (``manipulation_kit/config/home_pose.json``) is the first AND the last
  row. The player does not trust that: it OVERWRITES row 0 and the last row
  with HOME and starts the spline at HOME at ``t = 0`` — row 0's ``duration``
  is therefore ignored by the firmware player (the legacy ``gesture_play``
  used it as a HOME->HOME dwell). Row ``k >= 1`` lands at
  ``t_k = sum(duration_1..duration_k)``.
* The daemon samples those knots as a uniform Catmull-Rom spline on a linear
  time base (d1-firmware ``crates/d1fw-core/src/arm_trajectory.rs::sample``,
  ported in omakase-core ``tests/unit/robots/omakase/
  test_firmware_gesture_trajectory.py``) and guard-checks every millisecond.

This module's writer produces the file byte-for-byte in ``toCsv``'s format
(4 decimals, fixed), with the teach metadata as ``#`` comments — which every
reader skips — so nothing downstream has to learn a new column.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

#: The 14 CSV joint columns, in order. A = physical LEFT, B = physical RIGHT.
JOINT_NAMES: Tuple[str, ...] = tuple(f"R{i}" for i in range(1, 8)) + tuple(
    f"L{i}" for i in range(1, 8))
HEADER = "duration," + ",".join(JOINT_NAMES)
N_JOINTS = 14
#: Rows the readers skip by their first cell (gesture_csv.h::parseCsv).
SKIPPED_ROWS = ("duration", "kp", "kd")

#: The comment prefix this package's metadata lines carry.
META_PREFIX = "# mkit-teach:"
#: Written into a force-saved CSV that failed the check. ``play`` refuses a
#: file carrying it unless ``no_safety`` is passed — the old panel's
#: "Force-save even if unsafe (will need --no-safety to play)".
UNSAFE_TAG = "UNSAFE"

DEFAULT_HOME = (Path(__file__).resolve().parents[1] / "config" / "home_pose.json")


class GestureFormatError(ValueError):
    """The file is not a gesture the omakaseos player would accept."""


@dataclass
class Keyframe:
    duration: float
    positions: List[float]          # 14 degrees, CSV column order

    def __post_init__(self) -> None:
        self.duration = float(self.duration)
        self.positions = [float(v) for v in self.positions]


@dataclass
class Gesture:
    """A keyframe gesture plus the metadata its ``#`` lines carry."""

    keyframes: List[Keyframe]
    #: ``key -> value`` from ``# mkit-teach: key=value`` lines (name,
    #: sentiment, usage, source, home, ...)
    meta: Dict[str, str] = field(default_factory=dict)
    #: violations recorded when the file was force-saved; non-empty = unsafe
    unsafe: List[str] = field(default_factory=list)
    comments: List[str] = field(default_factory=list)

    @property
    def total_duration_s(self) -> float:
        return float(sum(k.duration for k in self.keyframes))

    def array(self) -> np.ndarray:
        return np.array([k.positions for k in self.keyframes], dtype=float)


def load_home(path: Optional[Path] = None) -> List[float]:
    """HOME as 14 degrees in CSV order (A1..A7, B1..B7), from the kit's file.

    ``joint_order`` is honoured rather than assumed, and anything but 14
    finite numbers is refused — a gesture pinned to a guessed HOME opens and
    closes with an unscripted transit.
    """
    doc = json.loads(Path(path or DEFAULT_HOME).read_text(encoding="utf-8"))
    order = doc.get("joint_order") or [f"A{i}" for i in range(1, 8)] + [
        f"B{i}" for i in range(1, 8)]
    values = dict(zip(order, doc["home_pose"]))
    try:
        home = [float(values[f"A{i}"]) for i in range(1, 8)] + [
            float(values[f"B{i}"]) for i in range(1, 8)]
    except KeyError as exc:
        raise GestureFormatError(f"home_pose.json lacks joint {exc}") from exc
    if len(doc["home_pose"]) != N_JOINTS or not all(map(math.isfinite, home)):
        raise GestureFormatError("home_pose.json must hold 14 finite degrees")
    return home


def _meta_line(line: str) -> Optional[Tuple[str, str]]:
    if not line.startswith(META_PREFIX):
        return None
    body = line[len(META_PREFIX):].strip()
    key, sep, value = body.partition("=")
    if not sep:
        return body.strip(), ""
    return key.strip(), value.strip()


def parse_csv(text: str) -> Gesture:
    """Parse with the FIRMWARE reader's strictness (15 finite numbers, positive
    durations), collecting the ``# mkit-teach:`` metadata on the way."""
    keyframes: List[Keyframe] = []
    meta: Dict[str, str] = {}
    unsafe: List[str] = []
    comments: List[str] = []
    for number, raw in enumerate(text.splitlines(), start=1):
        line = raw.strip()
        if not line:
            continue
        if line.startswith("#"):
            item = _meta_line(line)
            if item is None:
                comments.append(line)
            elif item[0] == UNSAFE_TAG:
                unsafe.append(item[1])
            else:
                meta[item[0]] = item[1]
            continue
        cells = [c.strip() for c in line.split(",")]
        if cells[0].lower() in SKIPPED_ROWS:
            continue
        try:
            values = [float(c) for c in cells]
        except ValueError as exc:
            raise GestureFormatError(f"line {number}: not numbers: {line[:60]}") from exc
        if len(values) != 1 + N_JOINTS:
            raise GestureFormatError(
                f"line {number}: {len(values)} values; the omakaseos player "
                f"requires exactly 15 (duration + 14 degrees)")
        if not all(math.isfinite(v) for v in values) or values[0] <= 0:
            raise GestureFormatError(
                f"line {number}: needs a positive duration and 14 finite degrees")
        keyframes.append(Keyframe(values[0], values[1:]))
    if not keyframes:
        raise GestureFormatError("gesture CSV is empty")
    return Gesture(keyframes, meta, unsafe, comments)


def load_csv(path: Path) -> Gesture:
    return parse_csv(Path(path).read_text(encoding="utf-8"))


def to_csv(gesture: Gesture) -> str:
    """The CSV text: ``toCsv``'s header comments and 4-decimal rows, then the
    metadata as ``# mkit-teach:`` comments (ignored by every reader)."""
    lines = [
        "# D1 dual-arm gesture (omakaseos keyframe format, angles in DEGREES)",
        "# joint order: R1..R7 (ArmSide::A = physical LEFT), L1..L7 (ArmSide::B = physical RIGHT)",
        "# first & last keyframe snapped to HOME pose",
    ]
    for key, value in gesture.meta.items():
        lines.append(f"{META_PREFIX} {key}={value}")
    for reason in gesture.unsafe:
        lines.append(f"{META_PREFIX} {UNSAFE_TAG}={reason}")
    lines.append(HEADER)
    for kf in gesture.keyframes:
        if len(kf.positions) != N_JOINTS:
            raise GestureFormatError("a keyframe needs 14 joint angles")
        if not (math.isfinite(kf.duration) and kf.duration > 0):
            raise GestureFormatError("every keyframe duration must be > 0")
        lines.append(f"{kf.duration:.4f}," + ",".join(f"{v:.4f}" for v in kf.positions))
    return "\n".join(lines) + "\n"


def save_csv(path: Path, gesture: Gesture) -> None:
    Path(path).write_text(to_csv(gesture), encoding="utf-8")


def trajectory_points(gesture: Gesture, home: Sequence[float]) -> List[Dict]:
    """The knots the omakaseos player uploads — a port of
    ``firmware_session.trajectory_from_csv``: HOME at ``t = 0`` standing in for
    row 0, rows 1.. at their cumulative durations, the last row replaced by
    HOME. ``a`` = physical LEFT (R1..R7), ``b`` = physical RIGHT (L1..L7)."""
    home = [float(v) for v in home]
    if len(home) != N_JOINTS or not all(map(math.isfinite, home)):
        raise GestureFormatError("HOME must contain 14 finite degrees")
    rows = [[k.duration] + list(k.positions) for k in gesture.keyframes]
    rows[0][1:] = home
    rows[-1][1:] = home
    points = [{"t": 0.0, "a": home[:7], "b": home[7:]}]
    elapsed = 0.0
    for row in rows[1:]:
        elapsed += row[0]
        points.append({"t": elapsed, "a": row[1:8], "b": row[8:15]})
    return points


def sample(points: Sequence[Dict], t: float) -> np.ndarray:
    """The daemon's spline at ``t`` (14 degrees): a port of d1-firmware
    ``arm_trajectory.rs::sample`` as omakase-core's guard test ports it —
    uniform Catmull-Rom through the knots, linear time inside a segment, end
    knots duplicated as their own neighbours."""
    ts = [p["t"] for p in points]
    n = len(points)
    if n == 1:
        return np.array(points[0]["a"] + points[0]["b"], dtype=float)
    i2 = min(max(sum(1 for x in ts if x < t), 1), n - 1)
    i1 = i2 - 1
    q = [np.array(list(points[k]["a"]) + list(points[k]["b"]), dtype=float)
         for k in (max(i1 - 1, 0), i1, i2, min(i2 + 1, n - 1))]
    span = ts[i2] - ts[i1]
    u = min(max((t - ts[i1]) / span, 0.0), 1.0) if span > 0 else 1.0
    v0, v1, v2, v3 = q
    return 0.5 * ((2 * v1) + (-v0 + v2) * u
                  + (2 * v0 - 5 * v1 + 4 * v2 - v3) * u * u
                  + (-v0 + 3 * v1 - 3 * v2 + v3) * u ** 3)


def sample_path(points: Sequence[Dict], step_s: float) -> Tuple[np.ndarray, np.ndarray]:
    """``(times, poses[N, 14])`` over the whole spline at ``step_s``."""
    end = float(points[-1]["t"])
    count = max(2, int(math.ceil(end / step_s)) + 1)
    times = np.linspace(0.0, end, count)
    return times, np.array([sample(points, float(t)) for t in times])
