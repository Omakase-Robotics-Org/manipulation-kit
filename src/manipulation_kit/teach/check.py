"""``check`` — would the omakaseos player (and the daemon under it) accept this?

The old panel's "Python safety validator" (omakase-core ``d1_safety.
check_smooth_path``, itself a mirror of d1-sdk ``safety_zones.h::
checkSmoothPath``) sampled gesture_play's ``[HOME, kf0, ...]`` path. The path
that plays today is the daemon's, so that is the one sampled here: the knots
of :func:`~manipulation_kit.teach.gesture_csv.trajectory_points` through the
daemon's Catmull-Rom (:func:`~manipulation_kit.teach.gesture_csv.sample`),
every ``step_s`` (default 10 ms; the daemon checks every 1 ms, so this is a
sampling, and a violation shorter than a step can slip).

HARD failures (``CheckReport.ok = False``; ``export`` refuses, ``play``
refuses without ``--no-safety``):

* timing — every knot time finite, starting at 0, strictly increasing, and
  every angle finite (the document's ``Waypoint`` contract);
* joint limits — the URDF box, NOT clamped (``MotionGuard(clamp_limits=
  False)``), so a limit is a violation, never a silent clip;
* the COUPLED wrist-roll limit (``config/coupled_joint_limits.json``: |J7| as
  a function of |J6|, both arms), which the box does not know about;
* per segment, per joint, the peak velocity and acceleration measured on
  that spline the way ``limitJointDynamics`` measures them (64 samples inside
  each segment), against the gesture's :class:`~manipulation_kit.teach.
  process.SpeedPolicy` — the caps its CSV was exported with
  (``# mkit-teach: max_joint_vel=… max_joint_acc=…``; the default
  150 deg/s, 600 deg/s^2 for a file without them), or the caller's
  ``speed`` — with 3 % slack.

ADVISORY findings (``CheckReport.guard_findings``, printed as warnings, never
a failure): the :class:`~manipulation_kit.guard.MotionGuard` clearances —
body keep-out, chest keep-out, arm-arm and same-arm self-collision. A taught
gesture is poses an operator guided the arm through BY HAND, so the guard's
capsule model is not the arbiter of whether those poses are reachable without
contact (Shu, 2026-09-23). Each finding names the closest distance, the
frames and when it happened.

What that does NOT change: d1-firmwared runs its own guard (the same model
and margins, ``d1fw-core arm_trajectory.rs::validate``) on every 1 ms sample
of ``POST /v1/arm/trajectory/start`` and again during playback, and refuses
the upload on any clearance violation. A gesture with a guard finding
therefore still does not play on a daemon at the reference margins; that
decision is the daemon's (its per-robot ``[arm] guard_body_margin_m`` /
``guard_arm_arm_margin_m``), not this tool's. See ``docs/teach.md``.

FK: the flange (``Link7``) of each arm is placed at every keyframe through the
guard's own URDF model, and the workspace it sweeps is reported.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from ..arms.coupled_limits import load_coupled_limits
from .gesture_csv import (Gesture, JOINT_NAMES, N_JOINTS, sample_path,
                          trajectory_points)
from .process import SpeedPolicy, segment_peaks_per_joint

DEFAULT_STEP_S = 0.01
RATE_SLACK = 1.03
#: row 0 / last row further than this from HOME are replaced by the player
HOME_TOL_DEG = 1e-3


#: MotionGuard clearance kinds, in report order
GUARD_KINDS = ("body", "chest", "arm-arm", "self")


@dataclass
class GuardFinding:
    """The worst sample of one MotionGuard clearance kind over the spline."""

    kind: str                 # one of GUARD_KINDS
    t: float                  # [s] on the played spline, closest sample
    clearance_m: float        # closest clearance, negative = overlap (self:
                              # after the guard's same-arm margin)
    detail: str               # the guard's own line: arm, frames, margin
    samples: int = 1          # samples below the margin
    step_s: float = 0.0

    def summary(self) -> str:
        return (f"guard (advisory) {self.kind}: closest {self.clearance_m * 1000:+.1f} mm "
                f"at t={self.t:.2f}s — {self.detail}; below the margin for "
                f"{self.samples} sample(s) (~{self.samples * self.step_s:.2f} s)")


def _classify(message: str, rep) -> Optional[Tuple[str, float]]:
    """``(kind, clearance_m)`` of one MotionGuard clearance line, or ``None``
    for anything else (a joint-limit line). The distance is the report's own
    unclipped minimum for that kind (the line prints it clipped at 0)."""
    if " of body box " in message:
        kind = "body"
    elif " of chest keep-out" in message:
        kind = "chest"
    elif " vs arm B " in message:
        kind = "arm-arm"
    elif " self: " in message:
        kind = "self"
    else:
        return None
    value = {"body": rep.min_body_clearance, "chest": rep.min_chest_clearance,
             "arm-arm": rep.min_arm_arm, "self": rep.min_self_clearance}[kind]
    return kind, float(value)


def _printed_m(message: str) -> float:
    """The distance a guard line prints (clipped at 0 for body/arm-arm; the
    overlap depth, negated, for self)."""
    found = re.search(r"(\d+\.\d+) m", message)
    value = float(found.group(1)) if found else 0.0
    return -value if " self: " in message else value


@dataclass
class CheckReport:
    #: limits, coupled wrist limit, rates and timing all pass. The guard's
    #: clearance findings do NOT enter it (see ``guard_findings``).
    ok: bool
    violations: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    guard_findings: List[GuardFinding] = field(default_factory=list)
    duration_s: float = 0.0
    keyframes: int = 0
    peak_vel_deg_s: List[float] = field(default_factory=list)
    peak_acc_deg_s2: List[float] = field(default_factory=list)
    min_body_clearance_m: float = float("inf")
    min_arm_arm_m: float = float("inf")
    min_self_clearance_m: float = float("inf")
    flange_range_m: Dict[str, Tuple[List[float], List[float]]] = field(default_factory=dict)
    #: the speed ceiling the rates were checked against
    speed: Optional[SpeedPolicy] = None

    @property
    def guard_clear(self) -> bool:
        """No MotionGuard clearance finding (the daemon's guard would accept
        the clearances at the reference margins)."""
        return not self.guard_findings

    def clearance_note(self) -> str:
        """One line for the CSV: the minimum clearances, and the guard kinds
        that fell below their margin."""
        text = (f"body {_mm(self.min_body_clearance_m)} arm-arm "
                f"{_mm(self.min_arm_arm_m)} self {_mm(self.min_self_clearance_m)}")
        if self.guard_findings:
            text += " (below margin: " + ", ".join(
                f"{f.kind} t={f.t:.2f}s" for f in self.guard_findings) + ")"
        return text

    def summary(self) -> str:
        lines = [f"check: {'OK' if self.ok else 'UNSAFE'} — {self.keyframes} keyframes, "
                 f"{self.duration_s:.2f} s on the daemon's spline"]
        if self.peak_vel_deg_s:
            jv = int(np.argmax(self.peak_vel_deg_s))
            ja = int(np.argmax(self.peak_acc_deg_s2))
            lines.append(f"  peak velocity {self.peak_vel_deg_s[jv]:.1f} deg/s "
                         f"({JOINT_NAMES[jv]}), peak acceleration "
                         f"{self.peak_acc_deg_s2[ja]:.1f} deg/s^2 ({JOINT_NAMES[ja]})"
                         + (f"; ceiling {self.speed.describe()}" if self.speed else ""))
        lines.append(f"  min clearance: body {self.min_body_clearance_m:.3f} m, "
                     f"arm-arm {self.min_arm_arm_m:.3f} m, self "
                     f"{self.min_self_clearance_m:.3f} m")
        for side, (lo, hi) in sorted(self.flange_range_m.items()):
            lines.append(f"  {side} flange sweeps x[{lo[0]:+.3f},{hi[0]:+.3f}] "
                         f"y[{lo[1]:+.3f},{hi[1]:+.3f}] z[{lo[2]:+.3f},{hi[2]:+.3f}] m")
        for w in self.warnings:
            lines.append(f"  warning: {w}")
        for f in self.guard_findings:
            lines.append(f"  WARNING: {f.summary()}")
        if self.guard_findings:
            lines.append("  (advisory: the operator guided the arm through these "
                         "poses by hand. d1-firmwared still guards the upload "
                         "with its own margins and refuses it if they are not met.)")
        for v in self.violations[:20]:
            lines.append(f"  VIOLATION: {v}")
        if len(self.violations) > 20:
            lines.append(f"  ... and {len(self.violations) - 20} more")
        return "\n".join(lines)


def _mm(value_m: float) -> str:
    return "n/a" if not math.isfinite(value_m) or value_m >= 1e8 else f"{value_m * 1000:.1f}mm"


def _timing_violations(points) -> List[str]:
    """The ``Waypoint`` contract, before anything samples the spline."""
    out = []
    if not points or points[0]["t"] != 0.0:
        out.append("the first knot must be at t = 0")
    for i, p in enumerate(points):
        t = p["t"]
        if not math.isfinite(t):
            out.append(f"knot {i}: time {t!r} is not finite")
        elif i and not t > points[i - 1]["t"]:
            out.append(f"knot {i}: time {t:.4f} s does not increase "
                       f"(previous {points[i - 1]['t']:.4f} s)")
        if not all(math.isfinite(float(v)) for v in list(p["a"]) + list(p["b"])):
            out.append(f"knot {i}: a joint angle is not finite")
    return out


def _guard():
    from ..guard import MotionGuard  # noqa: PLC0415 - parses the URDF
    return MotionGuard(clamp_limits=False)


def _flange_positions(guard, joints14: Sequence[float]) -> Dict[str, List[float]]:
    out = {}
    for side, suf, sl in (("left", "R", slice(0, 7)), ("right", "L", slice(7, 14))):
        q = {f"Joint{i + 1}_{suf}": math.radians(v)
             for i, v in enumerate(joints14[sl])}
        tf = guard.model.link_transforms(q)[f"Link7_{suf}"]
        out[side] = [float(v) for v in _translation(tf)]
    return out


def _translation(tf) -> Sequence[float]:
    for name in ("p", "t", "pos", "translation"):
        if hasattr(tf, name):
            return getattr(tf, name)
    raise AttributeError("guard Tf carries no translation")


def check_gesture(gesture: Gesture, home: Sequence[float], *, guard=None,
                  coupled=None, step_s: float = DEFAULT_STEP_S,
                  speed: Optional[SpeedPolicy] = None) -> CheckReport:
    """``speed=None``: the policy the gesture carries (:meth:`SpeedPolicy.of`)."""
    speed = speed if speed is not None else SpeedPolicy.of(gesture)
    max_vel_deg_s = speed.max_joint_vel_deg_s
    max_acc_deg_s2 = speed.max_joint_acc_deg_s2
    guard = guard if guard is not None else _guard()
    coupled = coupled if coupled is not None else load_coupled_limits()
    report = CheckReport(ok=True, keyframes=len(gesture.keyframes), speed=speed)
    rows = gesture.array()
    for label, idx in (("first", 0), ("last", -1)):
        gap = float(np.max(np.abs(rows[idx] - np.asarray(home))))
        if gap > HOME_TOL_DEG:
            report.warnings.append(
                f"the {label} row is {gap:.2f} deg from HOME; the omakaseos "
                f"player replaces it with HOME, so that pose will not play")
    points = trajectory_points(gesture, home)
    timing = _timing_violations(points)
    if timing:
        report.ok = False
        report.violations.extend(f"timing: {v}" for v in timing)
        return report
    report.duration_s = float(points[-1]["t"])
    times, poses = sample_path(points, step_s)
    seen = set()
    found: Dict[str, GuardFinding] = {}

    def add(t: float, what: str) -> None:
        key = re.sub(r"[-+]?\d+(\.\d+)?", "#", what)   # one line per kind
        if key in seen:
            return
        seen.add(key)
        report.ok = False
        report.violations.append(f"t={t:.2f}s {what}")

    for t, q in zip(times, poses):
        rep = guard.check(list(q[:7]), list(q[7:]))
        report.min_body_clearance_m = min(report.min_body_clearance_m,
                                          rep.min_body_clearance,
                                          rep.min_chest_clearance)
        report.min_arm_arm_m = min(report.min_arm_arm_m, rep.min_arm_arm)
        report.min_self_clearance_m = min(report.min_self_clearance_m,
                                          rep.min_self_clearance)
        here: Dict[str, Tuple[float, str]] = {}      # kind -> (printed m, line)
        for v in rep.violations:
            item = _classify(v, rep)
            if item is None:            # a joint limit: hard
                add(float(t), v)
                continue
            printed = _printed_m(v)
            if item[0] not in here or printed < here[item[0]][0]:
                here[item[0]] = (printed, v)   # the worst pair names the frames
        for kind, (_, line) in here.items():
            clearance = _classify(line, rep)[1]
            worst = found.get(kind)
            if worst is None:
                worst = found[kind] = GuardFinding(kind, float(t), clearance, line,
                                                   samples=0, step_s=float(step_s))
            elif clearance < worst.clearance_m:
                worst.t, worst.clearance_m, worst.detail = float(t), clearance, line
            worst.samples += 1
        for arm, sl in (("A", slice(0, 7)), ("B", slice(7, 14))):
            qr = np.radians(q[sl])
            for lim in coupled:
                why = lim.violation(qr)
                if why is not None:
                    add(float(t), f"arm {arm} {why}")
    report.guard_findings = [found[k] for k in GUARD_KINDS if k in found]
    peaks = segment_peaks_per_joint(points)
    if peaks:
        vel = np.array([v for v, _ in peaks])
        acc = np.array([a for _, a in peaks])
        report.peak_vel_deg_s = [float(v) for v in vel.max(axis=0)]
        report.peak_acc_deg_s2 = [float(v) for v in acc.max(axis=0)]
        for j in range(N_JOINTS):
            if report.peak_vel_deg_s[j] > max_vel_deg_s * RATE_SLACK:
                s = int(np.argmax(vel[:, j]))
                add(float(points[s]["t"]),
                    f"{JOINT_NAMES[j]} velocity {report.peak_vel_deg_s[j]:.1f} "
                    f"deg/s over the {max_vel_deg_s:g} deg/s cap (segment {s + 1})")
            if report.peak_acc_deg_s2[j] > max_acc_deg_s2 * RATE_SLACK:
                s = int(np.argmax(acc[:, j]))
                add(float(points[s]["t"]),
                    f"{JOINT_NAMES[j]} acceleration {report.peak_acc_deg_s2[j]:.1f} "
                    f"deg/s^2 over the {max_acc_deg_s2:g} deg/s^2 cap (segment {s + 1})")
    try:
        per_side: Dict[str, List[List[float]]] = {"left": [], "right": []}
        for row in rows:
            for side, p in _flange_positions(guard, row).items():
                per_side[side].append(p)
        report.flange_range_m = {
            side: (list(np.min(ps, axis=0)), list(np.max(ps, axis=0)))
            for side, ps in per_side.items()}
    except (AttributeError, KeyError):
        report.warnings.append("flange FK unavailable from this guard")
    return report


_BARS = " ▁▂▃▄▅▆▇█"


def ascii_preview(gesture: Gesture, home: Sequence[float], *, width: int = 60,
                  step_s: Optional[float] = None) -> str:
    """One strip per joint over the played spline, each scaled to its own
    range (printed beside it) — a matplotlib-free look at the motion."""
    points = trajectory_points(gesture, home)
    end = float(points[-1]["t"])
    step = step_s or max(end / max(width, 1), 1e-3)
    times, poses = sample_path(points, step)
    idx = np.linspace(0, len(times) - 1, min(width, len(times))).astype(int)
    lines = [f"{'':4}0s{'':{max(0, len(idx) - 8)}}{end:.1f}s"]
    for j, name in enumerate(JOINT_NAMES):
        col = poses[idx, j]
        lo, hi = float(col.min()), float(col.max())
        if hi - lo < 0.05:
            strip = "─" * len(idx)
        else:
            strip = "".join(_BARS[1 + int(round((v - lo) / (hi - lo) * 7))] for v in col)
        lines.append(f"{name:>3} {strip} [{lo:+7.1f}, {hi:+7.1f}] deg")
    return "\n".join(lines)
