"""``check`` — would the omakaseos player (and the daemon under it) accept this?

The old panel's "Python safety validator" (omakase-core ``d1_safety.
check_smooth_path``, itself a mirror of d1-sdk ``safety_zones.h::
checkSmoothPath``) sampled gesture_play's ``[HOME, kf0, ...]`` path. The path
that plays today is the daemon's, so that is the one sampled here: the knots
of :func:`~manipulation_kit.teach.gesture_csv.trajectory_points` through the
daemon's Catmull-Rom (:func:`~manipulation_kit.teach.gesture_csv.sample`),
every ``step_s`` (default 10 ms; the daemon checks every 1 ms, so this is a
sampling, and a violation shorter than a step can slip — the daemon still
refuses it at upload).

At every sample:

* joint limits — the URDF box, NOT clamped (``MotionGuard(clamp_limits=
  False)``), so a limit is a violation, never a silent clip;
* the COUPLED wrist-roll limit (``config/coupled_joint_limits.json``: |J7| as
  a function of |J6|, both arms), which the box does not know about;
* body keep-out, chest keep-out, arm-arm and same-arm self-collision — the
  kit's :class:`~manipulation_kit.guard.MotionGuard`, the model and margins the
  daemon embeds;

and per segment, per joint, the peak velocity and acceleration measured on
that spline the way ``limitJointDynamics`` measures them (64 samples inside
each segment), against gesture_record's playability caps (25 deg/s,
120 deg/s^2), with 3 % slack (the library's repaired files peak at 25.0-25.4).

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
from .process import (MAX_JOINT_ACC_DEG_S2, MAX_JOINT_VEL_DEG_S,
                      segment_peaks_per_joint)

DEFAULT_STEP_S = 0.01
RATE_SLACK = 1.03
#: row 0 / last row further than this from HOME are replaced by the player
HOME_TOL_DEG = 1e-3


@dataclass
class CheckReport:
    ok: bool
    violations: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    duration_s: float = 0.0
    keyframes: int = 0
    peak_vel_deg_s: List[float] = field(default_factory=list)
    peak_acc_deg_s2: List[float] = field(default_factory=list)
    min_body_clearance_m: float = float("inf")
    min_arm_arm_m: float = float("inf")
    min_self_clearance_m: float = float("inf")
    flange_range_m: Dict[str, Tuple[List[float], List[float]]] = field(default_factory=dict)

    def summary(self) -> str:
        lines = [f"check: {'OK' if self.ok else 'UNSAFE'} — {self.keyframes} keyframes, "
                 f"{self.duration_s:.2f} s on the daemon's spline"]
        if self.peak_vel_deg_s:
            jv = int(np.argmax(self.peak_vel_deg_s))
            ja = int(np.argmax(self.peak_acc_deg_s2))
            lines.append(f"  peak velocity {self.peak_vel_deg_s[jv]:.1f} deg/s "
                         f"({JOINT_NAMES[jv]}), peak acceleration "
                         f"{self.peak_acc_deg_s2[ja]:.1f} deg/s^2 ({JOINT_NAMES[ja]})")
        lines.append(f"  min clearance: body {self.min_body_clearance_m:.3f} m, "
                     f"arm-arm {self.min_arm_arm_m:.3f} m, self "
                     f"{self.min_self_clearance_m:.3f} m")
        for side, (lo, hi) in sorted(self.flange_range_m.items()):
            lines.append(f"  {side} flange sweeps x[{lo[0]:+.3f},{hi[0]:+.3f}] "
                         f"y[{lo[1]:+.3f},{hi[1]:+.3f}] z[{lo[2]:+.3f},{hi[2]:+.3f}] m")
        for w in self.warnings:
            lines.append(f"  warning: {w}")
        for v in self.violations[:20]:
            lines.append(f"  VIOLATION: {v}")
        if len(self.violations) > 20:
            lines.append(f"  ... and {len(self.violations) - 20} more")
        return "\n".join(lines)


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
                  max_vel_deg_s: float = MAX_JOINT_VEL_DEG_S,
                  max_acc_deg_s2: float = MAX_JOINT_ACC_DEG_S2) -> CheckReport:
    guard = guard if guard is not None else _guard()
    coupled = coupled if coupled is not None else load_coupled_limits()
    report = CheckReport(ok=True, keyframes=len(gesture.keyframes))
    rows = gesture.array()
    for label, idx in (("first", 0), ("last", -1)):
        gap = float(np.max(np.abs(rows[idx] - np.asarray(home))))
        if gap > HOME_TOL_DEG:
            report.warnings.append(
                f"the {label} row is {gap:.2f} deg from HOME; the omakaseos "
                f"player replaces it with HOME, so that pose will not play")
    points = trajectory_points(gesture, home)
    report.duration_s = float(points[-1]["t"])
    times, poses = sample_path(points, step_s)
    seen = set()

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
        for v in rep.violations:
            add(float(t), v)
        for arm, sl in (("A", slice(0, 7)), ("B", slice(7, 14))):
            qr = np.radians(q[sl])
            for lim in coupled:
                why = lim.violation(qr)
                if why is not None:
                    add(float(t), f"arm {arm} {why}")
    peaks = segment_peaks_per_joint(points)
    if peaks:
        vel = np.array([v for v, _ in peaks])
        acc = np.array([a for _, a in peaks])
        report.peak_vel_deg_s = [float(v) for v in vel.max(axis=0)]
        report.peak_acc_deg_s2 = [float(v) for v in acc.max(axis=0)]
        for j in range(N_JOINTS):
            if max_vel_deg_s > 0 and report.peak_vel_deg_s[j] > max_vel_deg_s * RATE_SLACK:
                s = int(np.argmax(vel[:, j]))
                add(float(points[s]["t"]),
                    f"{JOINT_NAMES[j]} velocity {report.peak_vel_deg_s[j]:.1f} "
                    f"deg/s over the {max_vel_deg_s:g} deg/s cap (segment {s + 1})")
            if max_acc_deg_s2 > 0 and report.peak_acc_deg_s2[j] > max_acc_deg_s2 * RATE_SLACK:
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
