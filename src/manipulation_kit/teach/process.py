"""Recording -> keyframe gesture: the reduction ``gesture_record`` ran, in Python.

Ported from d1-sdk ``devices/omakase_arm/example/gesture_record.cpp`` (the
finalize half, after the capture loop) and
``devices/omakase_arm/include/omakase_arm/gesture_csv.h`` (``smoothSamples``,
``reduceSamples``, ``buildGesture``, ``limitJointDynamics``), read-only clone
of d1-sdk-workspace @ f142fc6; plus omakase-core
``status_server/d1/teach.py::_trim_idle_keyframes`` (the panel's "Trim idle
pauses"). Defaults are gesture_record's (smoothing window 5, epsilon 1.5 deg,
0.05 s minimum keyframe, wrist locked at HOME) except the speed ceiling:
:class:`SpeedPolicy`, 150 deg/s and 600 deg/s^2 (gesture_record's was 25 and
120, which slowed a hand-taught swing down visibly).

Order: trim the release sag -> lock wrist -> smooth -> reduce -> build (real
elapsed time per keyframe) -> HOME in / HOME out -> [trim idle] -> limit
dynamics (gesture_record's, plus the two HOME rules below).

HOME RULES (Shu, 2026-09-23), for a hand-guided take:

* **The start sag is not motion.** When the brakes open at HOME the arm drops
  for a moment before the operator carries it. :func:`settle_index` finds the
  last sample, within the first ``sag_max_s`` (0.5 s), whose joint speed is
  above ``sag_vel_deg_s`` (8 deg/s) and cuts the stream there; a take with no
  such spike loses nothing. The motion then starts at HOME and blends into
  the first kept sample at the constant ``home_speed_deg_s`` below — after
  smoothing, so the join is continuous in position, and the daemon's C1
  Catmull-Rom plus the limiter keep it continuous in velocity.
* **The return to HOME is a constant speed.** The player replaces the last
  row with HOME, so the last recorded pose is kept as a real row and a HOME
  row is APPENDED after it, lasting ``max|pose - HOME| / home_speed_deg_s``
  (20 deg/s — deliberately slow and constant, well under the ceiling, since
  it is not taught motion): a long return and a short one
  move at the same joint speed. The HOME-in blend is timed the same way.
  A stream end (or start) already within ``epsilon_deg`` of HOME is snapped
  to HOME instead, as gesture_record did.

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
from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional, Sequence

import numpy as np

from .gesture_csv import (JOINT_NAMES, Gesture, GestureFormatError, Keyframe,
                          N_JOINTS, sample, trajectory_points)

#: gesture_record defaults
SMOOTH_WINDOW = 5
EPSILON_DEG = 1.5
MIN_KEYFRAME_S = 0.05
#: buildGesture's floor on a keyframe span before the playability pass
BUILD_MIN_DURATION_S = 0.02
#: teach.py: a keyframe is "no motion" when every joint moved less than this
IDLE_MOVE_EPS_DEG = 0.5
#: the wrist joints gesture_record pinned at HOME while recording (0-based,
#: both arms): J5/J6/J7. Under compliance the free wrist droops under gravity
#: and the raw capture reads as "wrist pointing down".
WRIST_JOINTS = (4, 5, 6, 11, 12, 13)
#: joint speed of the HOME-in blend and the appended return to HOME [deg/s]
HOME_SPEED_DEG_S = 20.0
#: the start sag: only the first this-many seconds may be cut [s] ...
SAG_MAX_S = 0.5
#: ... and a sample is part of the sag while its joint speed exceeds this
SAG_VEL_DEG_S = 8.0
#: segment sample count in limitJointDynamics
_LIMIT_STEPS = 64
_LIMIT_PASSES = 6


@dataclass(frozen=True)
class SpeedPolicy:
    """THE gesture speed policy — the ceiling ``export`` stretches a taught
    gesture to, and ``check`` / ``play`` refuse above. One place, used by all
    three.

    It is the only speed policy a taught gesture meets: d1-firmwared caps a
    trajectory at 350 deg/s per step and the omakaseos player validates no
    speed at all.

    Default: 150 deg/s, 600 deg/s^2 (Shu 2026-09-23 21:09Z, "option c"): a
    taught motion plays back at the speed it was recorded at, and only what
    is faster than that is stretched. d1-2 take2 is the reason: its J1 swing
    peaked ~130 deg/s after smoothing, and gesture_record's legacy caps
    (``limitJointDynamics``: 25 deg/s, 120 deg/s^2, to which the 30 library
    CSVs were repaired) stretched it from 3.83 s to 5.52 s — visibly slower
    than taught. ``--max-joint-vel`` / ``--max-joint-acc`` set another
    ceiling per gesture, and the caps a CSV was exported with travel IN the
    CSV (``# mkit-teach: max_joint_vel=… max_joint_acc=…``), so ``check`` and
    ``play`` hold that file to its own ceiling unless told otherwise.
    """

    max_joint_vel_deg_s: float = 150.0
    max_joint_acc_deg_s2: float = 600.0

    #: the CSV metadata keys
    VEL_KEY = "max_joint_vel"
    ACC_KEY = "max_joint_acc"

    def __post_init__(self) -> None:
        for name in ("max_joint_vel_deg_s", "max_joint_acc_deg_s2"):
            value = float(getattr(self, name))
            if not (math.isfinite(value) and value > 0):
                raise ValueError(f"{name} must be a positive finite number, got {value!r}")
            object.__setattr__(self, name, value)

    def meta(self) -> Dict[str, str]:
        """The ``# mkit-teach:`` lines that carry this policy in a CSV."""
        return {self.VEL_KEY: f"{self.max_joint_vel_deg_s:g}",
                self.ACC_KEY: f"{self.max_joint_acc_deg_s2:g}"}

    def describe(self) -> str:
        return f"{self.max_joint_vel_deg_s:g} deg/s, {self.max_joint_acc_deg_s2:g} deg/s^2"

    def override(self, vel: Optional[float] = None,
                 acc: Optional[float] = None) -> "SpeedPolicy":
        """This policy with the caps that were given (``None`` = keep)."""
        return SpeedPolicy(self.max_joint_vel_deg_s if vel is None else vel,
                           self.max_joint_acc_deg_s2 if acc is None else acc)

    @classmethod
    def of(cls, gesture: Gesture, vel: Optional[float] = None,
           acc: Optional[float] = None) -> "SpeedPolicy":
        """The policy a gesture was exported under (its CSV metadata; the
        defaults for a file that predates the keys), then the overrides."""
        return cls.from_meta(gesture.meta).override(vel, acc)

    @classmethod
    def from_meta(cls, meta: Mapping[str, str]) -> "SpeedPolicy":
        values = {}
        for key, name in ((cls.VEL_KEY, "max_joint_vel_deg_s"),
                          (cls.ACC_KEY, "max_joint_acc_deg_s2")):
            if key not in meta:
                continue
            try:
                values[name] = float(meta[key])
            except ValueError:
                values[name] = float("nan")
            if not (math.isfinite(values[name]) and values[name] > 0):
                raise GestureFormatError(
                    f"# mkit-teach: {key}={meta[key]!r} is not a positive number")
        return cls(**values)


#: the default ceiling (Shu 2026-09-23: taught speed plays as taught)
DEFAULT_SPEED = SpeedPolicy()
#: gesture_record's ``limitJointDynamics`` caps, the library's repaired speed
LEGACY_SPEED = SpeedPolicy(25.0, 120.0)


@dataclass(frozen=True)
class SpeedStretch:
    """How much :func:`limit_joint_dynamics` slowed a gesture down."""

    before_s: float
    after_s: float
    policy: SpeedPolicy
    #: per joint, the peak |velocity| / |acceleration| of the UNLIMITED
    #: gesture on the daemon's spline (what was taught, after smoothing)
    peak_vel_deg_s: List[float] = field(default_factory=list)
    peak_acc_deg_s2: List[float] = field(default_factory=list)

    @property
    def stretched(self) -> bool:
        return self.after_s > self.before_s + 1e-3

    def summary(self) -> str:
        if self.peak_vel_deg_s:
            jv = int(np.argmax(self.peak_vel_deg_s))
            ja = int(np.argmax(self.peak_acc_deg_s2))
            peaks = (f"{JOINT_NAMES[jv]} peak {self.peak_vel_deg_s[jv]:.0f} deg/s, "
                     f"{JOINT_NAMES[ja]} {self.peak_acc_deg_s2[ja]:.0f} deg/s^2 recorded")
        else:
            peaks = "no motion"
        if not self.stretched:
            return (f"kept as recorded: {self.after_s:.2f} s ({peaks}, within "
                    f"{self.policy.describe()})")
        return (f"stretched {self.before_s:.2f} s -> {self.after_s:.2f} s to meet "
                f"{self.policy.describe()} ({peaks})")


def speed_stretch(before: Gesture, after: Gesture, home: Sequence[float],
                  policy: SpeedPolicy) -> SpeedStretch:
    """Measure what limiting did: the taught peaks and both play times."""
    peaks = segment_peaks_per_joint(trajectory_points(before, home))
    vel = [float(v) for v in np.max([v for v, _ in peaks], axis=0)] if peaks else []
    acc = [float(a) for a in np.max([a for _, a in peaks], axis=0)] if peaks else []
    return SpeedStretch(before.played_s, after.played_s, policy, vel, acc)


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
                         speed: SpeedPolicy = DEFAULT_SPEED,
                         min_duration_s: float = MIN_KEYFRAME_S) -> Gesture:
    """gesture_csv.h ``limitJointDynamics``, on the player's knots: STRETCH
    durations (never shorten, never touch a pose) until every segment's
    realized peak velocity/acceleration is within ``speed``. Row 0's duration
    is not a segment the firmware player plays and is only floored."""
    frames = [Keyframe(max(k.duration, min_duration_s), k.positions)
              for k in gesture.keyframes]
    out = Gesture(frames, dict(gesture.meta), list(gesture.unsafe))
    max_vel_deg_s = speed.max_joint_vel_deg_s
    max_acc_deg_s2 = speed.max_joint_acc_deg_s2
    for _ in range(_LIMIT_PASSES):
        points = trajectory_points(out, home)
        changed = False
        for s, (vel, acc) in enumerate(segment_peaks(points)):
            scale = 1.0
            if vel > max_vel_deg_s:
                scale = max(scale, vel / max_vel_deg_s)
            if acc > max_acc_deg_s2:
                scale = max(scale, math.sqrt(acc / max_acc_deg_s2))
            if scale > 1.0 + 1e-4:
                kf = frames[s + 1]          # segment s ends at row s+1
                kf.duration = max(min_duration_s, kf.duration * scale)
                changed = True
        if not changed:
            break
    return out


def settle_index(times: Sequence[float], samples: np.ndarray,
                 max_s: float = SAG_MAX_S, vel_deg_s: float = SAG_VEL_DEG_S) -> int:
    """The index the stream starts at once the release sag is over.

    Within the first ``max_s`` of the take, the LAST sample whose max joint
    speed (median-filtered, so one encoder spike is not a sag) exceeds
    ``vel_deg_s``; the stream starts after it. A drop-and-catch has a
    turnaround with near-zero speed at its bottom, which is why it is the last
    fast sample and not the first slow one. No fast sample -> 0 (nothing is
    cut); ``max_s <= 0`` or ``vel_deg_s <= 0`` -> 0 (off).
    """
    times = np.asarray(times, dtype=float)
    q = median_filter(np.asarray(samples, dtype=float), 3)
    if max_s <= 0 or vel_deg_s <= 0 or len(q) < 3:
        return 0
    dt = np.diff(times)
    dt[dt <= 0] = np.inf
    speed = np.max(np.abs(np.diff(q, axis=0)), axis=1) / dt   # speed[i]: i -> i+1
    window = np.nonzero(times[1:] - times[0] <= max_s)[0]
    fast = [int(i) for i in window if speed[i] > vel_deg_s]
    if not fast:
        return 0
    return min(fast[-1] + 1, len(q) - 2)


def home_move_s(pose: Sequence[float], home: Sequence[float],
                speed_deg_s: float = HOME_SPEED_DEG_S,
                min_s: float = MIN_KEYFRAME_S) -> float:
    """Duration of a move between ``pose`` and HOME at a constant joint speed:
    the largest joint distance over ``speed_deg_s`` (never below ``min_s``)."""
    distance = float(np.max(np.abs(np.asarray(pose, float) - np.asarray(home, float))))
    if speed_deg_s <= 0:
        return max(min_s, 0.0)
    return max(min_s, distance / speed_deg_s)


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
    #: stretch to ``speed`` (off: keep the taught timing; ``export``'s check
    #: still holds the result to ``speed`` and refuses what exceeds it)
    speed_limit: bool = True
    speed: SpeedPolicy = DEFAULT_SPEED
    min_keyframe_s: float = MIN_KEYFRAME_S
    #: HOME-in blend and appended return speed [deg/s]
    home_speed_deg_s: float = HOME_SPEED_DEG_S
    #: start-sag cut (0 = off): window [s] and speed threshold [deg/s]
    sag_max_s: float = SAG_MAX_S
    sag_vel_deg_s: float = SAG_VEL_DEG_S


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
    start = settle_index(times, q, o.sag_max_s, o.sag_vel_deg_s)
    times, q = times[start:], q[start:]
    if o.lock_wrist:
        q = lock_wrist(q, home)
    q = smooth_samples(q, o.smooth_window)
    home_arr = np.asarray(home, dtype=float)
    snap_start = snap_end = False
    if o.pin_home:
        snap_start = float(np.max(np.abs(q[0] - home_arr))) <= o.epsilon_deg
        snap_end = float(np.max(np.abs(q[-1] - home_arr))) <= o.epsilon_deg
        if snap_start:
            q[0] = home_arr
        if snap_end:
            q[-1] = home_arr
    reduce = reduce_collinear if o.method == "collinear" else reduce_douglas_peucker
    kept = enforce_min_spacing(reduce(q, o.epsilon_deg), times, o.min_spacing_s)
    gesture = build_gesture(q, times, kept, first_duration_s=o.min_keyframe_s)
    if o.pin_home:
        frames = list(gesture.keyframes)
        if not snap_start:
            first = frames[0]
            frames[0] = Keyframe(home_move_s(first.positions, home, o.home_speed_deg_s,
                                             o.min_keyframe_s), first.positions)
            frames.insert(0, Keyframe(o.min_keyframe_s, list(home_arr)))
        if not snap_end:
            frames.append(Keyframe(home_move_s(frames[-1].positions, home,
                                               o.home_speed_deg_s, o.min_keyframe_s),
                                   list(home_arr)))
        gesture = Gesture(frames)
    return finish(gesture, home, o)


def keyframes_from_poses(poses: Sequence[Sequence[float]], home: Sequence[float],
                         segment_s: float,
                         options: Optional[KeyframeOptions] = None) -> Gesture:
    """Operator-stepped keyframes (Enter per pose) -> a gesture: HOME pinned
    before and after (unless ``pin_home`` is off), ``segment_s`` per move
    (the last one, back to HOME, at the constant ``home_speed_deg_s``),
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
    if o.pin_home:      # the return to HOME at the constant HOME speed
        frames[-1].duration = home_move_s(rows[-2], home, o.home_speed_deg_s,
                                          o.min_keyframe_s)
    return finish(Gesture(frames), home, o)


def finish(gesture: Gesture, home: Sequence[float], o: KeyframeOptions) -> Gesture:
    """Trim idle pauses, then limit to ``o.speed``. The result carries the
    policy (``max_joint_vel`` / ``max_joint_acc``) and, when it had to slow
    the gesture down, ``speed_stretch`` in its metadata — which ``export``
    writes into the CSV header."""
    if o.max_idle_s and o.max_idle_s > 0:
        gesture = trim_idle(gesture, o.max_idle_s)
    limited = (limit_joint_dynamics(gesture, home, o.speed, o.min_keyframe_s)
               if o.speed_limit else gesture)
    stretch = speed_stretch(gesture, limited, home, o.speed)
    meta = dict(limited.meta)
    meta.update(o.speed.meta())
    meta.pop(STRETCH_KEY, None)
    if stretch.stretched:
        meta[STRETCH_KEY] = stretch.summary()
    return Gesture(limited.keyframes, meta, list(limited.unsafe))


#: metadata key: what limiting did to the taught timing (only when it slowed it)
STRETCH_KEY = "speed_stretch"
#: the metadata ``export`` carries from a reduced gesture into its CSV
SPEED_META_KEYS = (SpeedPolicy.VEL_KEY, SpeedPolicy.ACC_KEY, STRETCH_KEY)
