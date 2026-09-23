"""Recording -> keyframe gesture: the reduction ``gesture_record`` ran, in Python.

Ported from d1-sdk ``devices/omakase_arm/example/gesture_record.cpp`` (the
finalize half, after the capture loop) and
``devices/omakase_arm/include/omakase_arm/gesture_csv.h`` (``smoothSamples``,
``reduceSamples``, ``buildGesture``, ``limitJointDynamics``), read-only clone
of d1-sdk-workspace @ f142fc6; plus omakase-core
``status_server/d1/teach.py::_trim_idle_keyframes`` (the panel's "Trim idle
pauses"). Defaults are gesture_record's: smoothing window 5, epsilon 1.5 deg,
25 deg/s, 120 deg/s^2, 0.05 s minimum keyframe, wrist locked at HOME.

Order, as in gesture_record: lock wrist -> smooth -> snap HOME -> reduce ->
build (real elapsed time per keyframe) -> [trim idle] -> limit dynamics.

TWO DELIBERATE DEVIATIONS, both because the player changed underneath:

1. **Row 0 is HOME itself.** gesture_record's ``buildGesture`` dropped the
   HOME sample and wrote the first kept sample as row 0 (the old
   ``gesture_play`` prepended HOME). The firmware player OVERWRITES row 0 with
   HOME (``firmware_session.trajectory_from_csv``), which would silently drop
   that first real keyframe — so row 0 here is HOME, with a nominal
   ``min_keyframe_s`` duration the firmware player ignores.
2. **Dynamics are limited on the spline the DAEMON plays**, i.e. the knots
   :func:`~manipulation_kit.teach.gesture_csv.trajectory_points` builds, not
   the ``[HOME, kf0, ...]`` path gesture_play sampled. Same algorithm (measure
   each segment's realized peak on the curve, stretch its duration by
   ``peak/cap`` or ``sqrt(peak/cap)``, six passes), different — correct — curve.

Keyframe reduction offers gesture_record's greedy collinear-within-epsilon
walk (``method="collinear"``, the default) and a Douglas–Peucker split in joint
space (``method="dp"``, max-abs-joint error, the same epsilon). Both keep the
first and last sample and honour ``min_spacing_s``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np

from .gesture_csv import Gesture, Keyframe, N_JOINTS, sample, trajectory_points

#: gesture_record defaults
SMOOTH_WINDOW = 5
EPSILON_DEG = 1.5
MAX_JOINT_VEL_DEG_S = 25.0
MAX_JOINT_ACC_DEG_S2 = 120.0
MIN_KEYFRAME_S = 0.05
#: buildGesture's floor on a keyframe span before the playability pass
BUILD_MIN_DURATION_S = 0.02
#: teach.py: a keyframe is "no motion" when every joint moved less than this
IDLE_MOVE_EPS_DEG = 0.5
#: the wrist joints gesture_record pinned at HOME while recording (0-based,
#: both arms): J5/J6/J7. Under compliance the free wrist droops under gravity
#: and the raw capture reads as "wrist pointing down".
WRIST_JOINTS = (4, 5, 6, 11, 12, 13)
#: segment sample count in limitJointDynamics
_LIMIT_STEPS = 64
_LIMIT_PASSES = 6


def lock_wrist(samples: np.ndarray, home: Sequence[float]) -> np.ndarray:
    out = np.array(samples, dtype=float, copy=True)
    for j in WRIST_JOINTS:
        out[:, j] = float(home[j])
    return out


def _odd(window: int) -> int:
    window = max(0, int(window))
    return window + 1 if window > 1 and window % 2 == 0 else window


def median_filter(samples: np.ndarray, window: int) -> np.ndarray:
    """Centered sliding median, window clamped at the ends (gesture_csv.h)."""
    window = _odd(window)
    samples = np.asarray(samples, dtype=float)
    if window <= 1 or len(samples) <= 2:
        return samples.copy()
    half, n = window // 2, len(samples)
    return np.array([np.median(samples[max(0, i - half):min(n, i + half + 1)], axis=0)
                     for i in range(n)])


def moving_average(samples: np.ndarray, window: int) -> np.ndarray:
    window = _odd(window)
    samples = np.asarray(samples, dtype=float)
    if window <= 1 or len(samples) <= 2:
        return samples.copy()
    half, n = window // 2, len(samples)
    return np.array([samples[max(0, i - half):min(n, i + half + 1)].mean(axis=0)
                     for i in range(n)])


def smooth_samples(samples: np.ndarray, window: int = SMOOTH_WINDOW) -> np.ndarray:
    """Median (spike rejection) then mean (jitter low-pass), same window.
    The old panel's "Smoothing (jitter rejection)"; ``window <= 1`` is off."""
    return moving_average(median_filter(samples, window), window)


def _within(samples: np.ndarray, a: int, b: int, eps: float) -> bool:
    if b - a < 2:
        return True
    alpha = (np.arange(a + 1, b) - a) / float(b - a)
    line = samples[a] + np.outer(alpha, samples[b] - samples[a])
    return bool(np.max(np.abs(line - samples[a + 1:b])) <= eps)


def reduce_collinear(samples: np.ndarray, epsilon_deg: float = EPSILON_DEG) -> List[int]:
    """gesture_csv.h ``reduceSamples``: from the last kept anchor, extend the
    segment while every intermediate sample stays within ``epsilon_deg`` (per
    joint) of the straight line; returns the kept indices."""
    samples = np.asarray(samples, dtype=float)
    n = len(samples)
    if n <= 2:
        return list(range(n))
    kept, anchor = [0], 0
    while anchor + 1 < n:
        best = anchor + 1
        for end in range(anchor + 2, n):
            if _within(samples, anchor, end, epsilon_deg):
                best = end
            else:
                break
        kept.append(best)
        anchor = best
    return kept


def reduce_douglas_peucker(samples: np.ndarray, epsilon_deg: float = EPSILON_DEG) -> List[int]:
    """Douglas–Peucker in joint space: split at the sample farthest (max-abs
    joint error) from the chord until every sample is within ``epsilon_deg``."""
    samples = np.asarray(samples, dtype=float)
    n = len(samples)
    if n <= 2:
        return list(range(n))
    keep = {0, n - 1}
    stack = [(0, n - 1)]
    while stack:
        a, b = stack.pop()
        if b - a < 2:
            continue
        alpha = (np.arange(a + 1, b) - a) / float(b - a)
        line = samples[a] + np.outer(alpha, samples[b] - samples[a])
        err = np.max(np.abs(line - samples[a + 1:b]), axis=1)
        k = int(np.argmax(err))
        if err[k] > epsilon_deg:
            m = a + 1 + k
            keep.add(m)
            stack.extend(((a, m), (m, b)))
    return sorted(keep)


def enforce_min_spacing(kept: Sequence[int], times: Sequence[float],
                        min_spacing_s: float) -> List[int]:
    """Drop interior keyframes closer than ``min_spacing_s`` to the previous
    kept one (the first and last always stay)."""
    kept = list(kept)
    if min_spacing_s <= 0 or len(kept) <= 2:
        return kept
    out = [kept[0]]
    for idx in kept[1:-1]:
        if times[idx] - times[out[-1]] >= min_spacing_s:
            out.append(idx)
    if times[kept[-1]] - times[out[-1]] < min_spacing_s and len(out) > 1:
        out.pop()
    out.append(kept[-1])
    return out


def build_gesture(samples: np.ndarray, times: Sequence[float], kept: Sequence[int],
                  *, first_duration_s: float = MIN_KEYFRAME_S,
                  min_duration_s: float = BUILD_MIN_DURATION_S) -> Gesture:
    """Keyframes at the kept samples, each lasting the REAL elapsed time since
    the previous kept one (gesture_record assumed a uniform ``dt``; the
    timestamps are used here). Row 0 is ``samples[kept[0]]`` — HOME after
    snapping — with ``first_duration_s`` (see deviation 1)."""
    frames = [Keyframe(first_duration_s, samples[kept[0]])]
    for prev, idx in zip(kept, kept[1:]):
        frames.append(Keyframe(max(min_duration_s, float(times[idx] - times[prev])),
                               samples[idx]))
    return Gesture(frames)


def trim_idle(gesture: Gesture, max_idle_s: float,
              move_eps_deg: float = IDLE_MOVE_EPS_DEG) -> Gesture:
    """teach.py ``_trim_idle_keyframes``: cap the duration of a keyframe that
    does not move (every joint within ``move_eps_deg`` of the previous one) to
    ``max_idle_s``. Poses are untouched, so geometry and velocity only get
    safer. ``max_idle_s <= 0`` is off (the panel's "Off (keep pauses)")."""
    frames = [Keyframe(k.duration, k.positions) for k in gesture.keyframes]
    if not max_idle_s or max_idle_s <= 0 or len(frames) < 2:
        return Gesture(frames, dict(gesture.meta), list(gesture.unsafe))
    prev = frames[0].positions
    for kf in frames[1:]:
        delta = max(abs(a - b) for a, b in zip(kf.positions, prev))
        if delta < move_eps_deg and kf.duration > max_idle_s:
            kf.duration = float(max_idle_s)
        prev = kf.positions
    return Gesture(frames, dict(gesture.meta), list(gesture.unsafe))


def segment_peaks_per_joint(points: Sequence[dict], steps: int = _LIMIT_STEPS):
    """Per segment, per joint ``(|vel| deg/s [14], |acc| deg/s^2 [14])`` peaks
    measured on the daemon's spline by finite differences INSIDE the segment,
    ``steps`` samples per segment — limitJointDynamics' measurement. (A
    Catmull-Rom spline is C1: velocity is continuous across a knot,
    acceleration is not, so an acceleration cap is a per-segment notion.)"""
    peaks = []
    for s in range(len(points) - 1):
        t0, t1 = points[s]["t"], points[s + 1]["t"]
        h = (t1 - t0) / steps
        p = np.array([sample(points, t0 + h * k) for k in range(steps + 1)])
        v = np.diff(p, axis=0) / h
        a = np.diff(v, axis=0) / h
        peaks.append((np.max(np.abs(v), axis=0),
                      np.max(np.abs(a), axis=0) if len(a) else np.zeros(N_JOINTS)))
    return peaks


def segment_peaks(points: Sequence[dict], steps: int = _LIMIT_STEPS):
    """Per segment ``(peak |vel|, peak |acc|)`` over all joints."""
    return [(float(np.max(v)), float(np.max(a)))
            for v, a in segment_peaks_per_joint(points, steps)]


def limit_joint_dynamics(gesture: Gesture, home: Sequence[float],
                         max_vel_deg_s: float = MAX_JOINT_VEL_DEG_S,
                         max_acc_deg_s2: float = MAX_JOINT_ACC_DEG_S2,
                         min_duration_s: float = MIN_KEYFRAME_S) -> Gesture:
    """gesture_csv.h ``limitJointDynamics``, on the player's knots: STRETCH
    durations (never shorten, never touch a pose) until every segment's
    realized peak velocity/acceleration is within the caps. Row 0's duration
    is not a segment the firmware player plays and is only floored."""
    frames = [Keyframe(max(k.duration, min_duration_s), k.positions)
              for k in gesture.keyframes]
    out = Gesture(frames, dict(gesture.meta), list(gesture.unsafe))
    if max_vel_deg_s <= 0 and max_acc_deg_s2 <= 0:
        return out
    for _ in range(_LIMIT_PASSES):
        points = trajectory_points(out, home)
        changed = False
        for s, (vel, acc) in enumerate(segment_peaks(points)):
            scale = 1.0
            if max_vel_deg_s > 0 and vel > max_vel_deg_s:
                scale = max(scale, vel / max_vel_deg_s)
            if max_acc_deg_s2 > 0 and acc > max_acc_deg_s2:
                scale = max(scale, math.sqrt(acc / max_acc_deg_s2))
            if scale > 1.0 + 1e-4:
                kf = frames[s + 1]          # segment s ends at row s+1
                kf.duration = max(min_duration_s, kf.duration * scale)
                changed = True
        if not changed:
            break
    return out


@dataclass(frozen=True)
class KeyframeOptions:
    """The knobs of :func:`keyframes_from_samples`, gesture_record's defaults."""

    smooth_window: int = SMOOTH_WINDOW
    epsilon_deg: float = EPSILON_DEG
    method: str = "collinear"
    min_spacing_s: float = 0.0
    lock_wrist: bool = True
    pin_home: bool = True
    max_idle_s: float = 0.0
    speed_limit: bool = True
    max_joint_vel_deg_s: float = MAX_JOINT_VEL_DEG_S
    max_joint_acc_deg_s2: float = MAX_JOINT_ACC_DEG_S2
    min_keyframe_s: float = MIN_KEYFRAME_S


def keyframes_from_samples(times: Sequence[float], samples: np.ndarray,
                           home: Sequence[float],
                           options: Optional[KeyframeOptions] = None) -> Gesture:
    """A dense stream (``times`` s, ``samples[N, 14]`` deg) -> a gesture."""
    o = options or KeyframeOptions()
    times = np.asarray(times, dtype=float)
    q = np.asarray(samples, dtype=float).reshape(-1, N_JOINTS)
    if len(q) < 2:
        raise ValueError("not enough samples to build a gesture")
    if o.method not in ("collinear", "dp"):
        raise ValueError(f"method must be 'collinear' or 'dp', got {o.method!r}")
    if o.lock_wrist:
        q = lock_wrist(q, home)
    q = smooth_samples(q, o.smooth_window)
    if o.pin_home:
        q[0] = home
        q[-1] = home
    reduce = reduce_collinear if o.method == "collinear" else reduce_douglas_peucker
    kept = enforce_min_spacing(reduce(q, o.epsilon_deg), times, o.min_spacing_s)
    gesture = build_gesture(q, times, kept, first_duration_s=o.min_keyframe_s)
    return finish(gesture, home, o)


def keyframes_from_poses(poses: Sequence[Sequence[float]], home: Sequence[float],
                         segment_s: float,
                         options: Optional[KeyframeOptions] = None) -> Gesture:
    """Operator-stepped keyframes (Enter per pose) -> a gesture: HOME pinned
    before and after (unless ``pin_home`` is off), ``segment_s`` per move,
    then the same trim/limit as a stream. No smoothing or reduction: every
    pose was chosen."""
    o = options or KeyframeOptions()
    rows = [list(map(float, p)) for p in poses]
    if o.lock_wrist:
        rows = [list(r) for r in lock_wrist(np.array(rows).reshape(-1, N_JOINTS), home)]
    if o.pin_home:
        rows = [list(home)] + rows + [list(home)]
    if len(rows) < 2:
        raise ValueError("a gesture needs at least two keyframes")
    frames = [Keyframe(o.min_keyframe_s, rows[0])] + [
        Keyframe(segment_s, r) for r in rows[1:]]
    return finish(Gesture(frames), home, o)


def finish(gesture: Gesture, home: Sequence[float], o: KeyframeOptions) -> Gesture:
    if o.max_idle_s and o.max_idle_s > 0:
        gesture = trim_idle(gesture, o.max_idle_s)
    if o.speed_limit:
        gesture = limit_joint_dynamics(gesture, home, o.max_joint_vel_deg_s,
                                       o.max_joint_acc_deg_s2, o.min_keyframe_s)
    return gesture
