"""``play`` — a gesture CSV through the kit's :class:`FirmwareExecutor`.

The same path the omakaseos firmware player takes (omakase-core
``firmware_session.py``: approach, then ``trajectory_from_csv`` knots with the
first one re-anchored to the measured pose, one ``/v1/arm/trajectory/start``),
but through the executor, so the lease (class ``policy`` by default, ``operator``
for a person at the robot), position mode at THIS run's ``vel_ratio``, the
knot contract, the cancel-on-any-exit and the arrival barrier are the kit's
own rather than re-derived here.

Refusals, all BEFORE anything moves:

* a file force-saved as unsafe (``# mkit-teach: UNSAFE=...``) — unless
  ``no_safety`` (the old ``gesture_play --no-safety``);
* a file that fails :func:`~manipulation_kit.teach.check.check_gesture`'s
  HARD checks (joint limits incl. the coupled wrist limit, rates, timing) —
  unless ``no_safety``. Rates are held to the ceiling the CSV was exported
  with (``# mkit-teach: max_joint_vel/max_joint_acc``), or ``speed``. This flag only skips the kit's pre-flight, never the
  daemon's;
* a latched arm controller.

MotionGuard clearance findings are NOT a refusal here (a taught gesture's
poses were reached by hand; see :mod:`~manipulation_kit.teach.check`): they
are announced, through ``announce``, before anything moves.

The daemon does NOT follow that rule: it checks every sample of the upload
against its own guard — realistic geometry, 20 mm margin, clearance always on
(Shu 2026-09-23 21:14Z: the guard is on everywhere; the daemon API's
per-job relaxation is being removed, d1-firmware PR #106 and follow-up) —
and refuses the upload
on the same violations ``check`` warns about. No ``guard`` field is sent.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence

from ..executor import controller_fault
from .check import CheckReport, check_gesture
from .gesture_csv import Gesture, trajectory_points
from .process import SpeedPolicy, segment_peaks_per_joint
from .record import HOME_TOL_DEG, _q16, move_to, pose_deg


#: position-mode ratio of the HOME approach before a gesture: a calm move,
#: not taught motion
APPROACH_RATIO = 0.3
#: the playback ratio: headroom over the gesture's own peak, and its floor
RATIO_HEADROOM = 1.3
MIN_PLAY_RATIO = 0.3


def playback_ratio(gesture: Gesture, home: Sequence[float]):
    """``(ratio, why)``: the position-mode ratio that lets the controller
    track this gesture at the speed it was taught.

    In position mode the controller follows the daemon's 1 ms targets at no
    more than ``MAX_JOINT_RATE_DEG_S x vel_ratio`` (140 deg/s at 1.0): at the
    old fixed 0.15 that is ~21 deg/s, and d1-2 task6's 118 deg/s J7 swing
    was low-passed into a slow, smoothed wave that barely turned the wrist.
    The ratio must never be a hidden brake — the CSV's
    :class:`~manipulation_kit.teach.process.SpeedPolicy` is the speed gate —
    so it is derived from the gesture's own peak on the played spline:
    ``clamp(1.3 x peak / 140, 0.3, 1.0)``. The daemon's acceleration ratio
    has no documented physical scale, so it is set equal."""
    from ..executors.firmware.executor import MAX_JOINT_RATE_DEG_S  # noqa: PLC0415
    peaks = segment_peaks_per_joint(trajectory_points(gesture, home))
    peak = max((float(max(v)) for v, _ in peaks), default=0.0)
    ratio = min(max(RATIO_HEADROOM * peak / MAX_JOINT_RATE_DEG_S, MIN_PLAY_RATIO), 1.0)
    why = (f"playback ratio {ratio:.2f}: gesture peak {peak:.1f} deg/s x "
           f"{RATIO_HEADROOM:g} over the controller's {MAX_JOINT_RATE_DEG_S:g} "
           f"deg/s at ratio 1.0, within [{MIN_PLAY_RATIO:g}, 1]")
    return ratio, why


@dataclass
class PlayReport:
    ok: bool
    detail: str
    check: Optional[CheckReport] = None
    approached: bool = False
    waypoints: int = 0
    arrived: Optional[bool] = None
    notes: List[str] = field(default_factory=list)


def preflight(gesture: Gesture, home: Sequence[float], *, no_safety: bool = False,
              speed: Optional[SpeedPolicy] = None, check_kwargs=None) -> PlayReport:
    """Everything ``play`` decides without the robot. ``speed=None``: the
    ceiling the CSV was exported with (``SpeedPolicy.of``)."""
    if gesture.unsafe and not no_safety:
        return PlayReport(False, "this CSV was force-saved UNSAFE ("
                          + "; ".join(gesture.unsafe)
                          + "); re-teach it, or pass --no-safety to play it anyway")
    report = None
    if not no_safety:
        report = check_gesture(gesture, home, speed=speed, **(check_kwargs or {}))
        if not report.ok:
            return PlayReport(False, "refused by the pre-flight check:\n"
                              + report.summary(), check=report)
    return PlayReport(True, "pre-flight passed" if report else
                      "pre-flight SKIPPED (--no-safety)", check=report)


def guard_notes(report: Optional[CheckReport]) -> List[str]:
    """The advisory guard lines to show an operator before a gesture moves."""
    if report is None or not report.guard_findings:
        return []
    return [f.summary() for f in report.guard_findings] + [
        "the daemon checks clearance on every sample with its own geometry and "
        "20 mm margin, always, and will refuse the upload on the same "
        "violations (docs/teach.md, 'Guard')"]


def play(robot, gesture: Gesture, home: Sequence[float], *, no_safety: bool = False,
         speed: Optional[SpeedPolicy] = None, vel_ratio: Optional[float] = None,
         check_kwargs=None, settle_s: float = 2.0,
         announce: Callable[[str], None] = lambda line: None) -> PlayReport:
    """Play on an ENTERED :class:`FirmwareExecutor`. Returns what happened.
    ``announce`` receives the advisory guard warnings and the playback ratio
    BEFORE anything moves. The HOME approach runs at the executor's ratio as
    entered; the gesture at ``vel_ratio``, or — ``None`` — the one
    :func:`playback_ratio` derives from the gesture itself."""
    pre = preflight(gesture, home, no_safety=no_safety, speed=speed,
                    check_kwargs=check_kwargs)
    if not pre.ok:
        return pre
    advisory = guard_notes(pre.check)
    for line in advisory:
        announce(f"WARNING: {line}")
    fault = controller_fault(robot.state())
    if fault is not None:
        return PlayReport(False, f"controller fault before playing: {fault}",
                          check=pre.check)
    # The approach is its own move, as in omakase-core: a far-away knot in
    # front of HOME bends the spline through the torso (firmware_session).
    arrival = move_to(robot, home, tol_deg=HOME_TOL_DEG)
    approached = arrival is not None
    if approached and not arrival.arrived:
        return PlayReport(False, f"did not reach HOME before the gesture: "
                          f"{arrival.detail}", check=pre.check, approached=True)
    if vel_ratio is None:
        vel_ratio, why = playback_ratio(gesture, home)
    else:
        why = f"playback ratio {vel_ratio:.2f} (--vel-ratio)"
    announce(f"{why}; HOME approach at {robot.vel_ratio:.2f}")
    robot.set_ratios(vel_ratio)
    points = trajectory_points(gesture, home)
    measured = pose_deg(robot.state())
    points[0] = {"t": 0.0, "a": measured[:7], "b": measured[7:]}
    sent = robot.play_waypoints(points)
    end = robot.wait_arrived(_q16(home))
    notes = list(advisory) + [why]
    settle = robot.settle(settle_s)
    if not settle.settled:
        notes.append(f"not settled: {settle.detail}")
    if not end.arrived:
        return PlayReport(False, f"the gesture played but the arms did not "
                          f"measurably return to HOME: {end.detail}",
                          check=pre.check, approached=approached, waypoints=sent,
                          arrived=False, notes=notes)
    return PlayReport(True, f"played {sent} knots, "
                      f"{points[-1]['t']:.2f} s; back at HOME ({end.detail})",
                      check=pre.check, approached=approached, waypoints=sent,
                      arrived=True, notes=notes)
