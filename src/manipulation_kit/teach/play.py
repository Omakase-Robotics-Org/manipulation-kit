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
* a file that fails :func:`~manipulation_kit.teach.check.check_gesture` —
  unless ``no_safety``. The daemon re-guards every sample regardless; this
  flag only skips the kit's pre-flight, never the daemon's;
* a latched arm controller.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence

from ..executor import controller_fault
from .check import CheckReport, check_gesture
from .gesture_csv import Gesture, trajectory_points
from .record import HOME_TOL_DEG, _q16, move_to, pose_deg


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
              check_kwargs=None) -> PlayReport:
    """Everything ``play`` decides without the robot."""
    if gesture.unsafe and not no_safety:
        return PlayReport(False, "this CSV was force-saved UNSAFE ("
                          + "; ".join(gesture.unsafe)
                          + "); re-teach it, or pass --no-safety to play it anyway")
    report = None
    if not no_safety:
        report = check_gesture(gesture, home, **(check_kwargs or {}))
        if not report.ok:
            return PlayReport(False, "refused by the pre-flight check:\n"
                              + report.summary(), check=report)
    return PlayReport(True, "pre-flight passed" if report else
                      "pre-flight SKIPPED (--no-safety)", check=report)


def play(robot, gesture: Gesture, home: Sequence[float], *, no_safety: bool = False,
         check_kwargs=None, settle_s: float = 2.0) -> PlayReport:
    """Play on an ENTERED :class:`FirmwareExecutor`. Returns what happened."""
    pre = preflight(gesture, home, no_safety=no_safety, check_kwargs=check_kwargs)
    if not pre.ok:
        return pre
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
    points = trajectory_points(gesture, home)
    measured = pose_deg(robot.state())
    points[0] = {"t": 0.0, "a": measured[:7], "b": measured[7:]}
    sent = robot.play_waypoints(points)
    end = robot.wait_arrived(_q16(home))
    notes = []
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
