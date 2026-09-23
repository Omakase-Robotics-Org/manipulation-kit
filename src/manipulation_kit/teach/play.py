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

The daemon's own guard follows the same rule by default. ``play`` uploads the
gesture with ``guard="speed_only"`` (d1-firmware PR #102, Shu 2026-09-23:
keep the speed guard, the range guard may be off for gesture playback): the
daemon skips its clearance checks for this one job and still refuses joint
limits, the 350 deg/s step cap, bad timing and a first knot more than 3
degrees from feedback, and still stops on cancel / estop / soft kill.
``guard="full"`` opts back into the daemon's clearance checks, which then
refuse a gesture that dips under that robot's ``[arm]`` margins. A daemon
that predates the field publishes no ``TrajectoryGuard``; ``play`` then says
so and plays under that daemon's full guard.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence

#: The daemon trajectory guard ``play`` asks for unless told otherwise.
DEFAULT_GUARD = "speed_only"
GUARDS = ("speed_only", "full")

from ..executor import controller_fault
from .check import CheckReport, check_gesture
from .gesture_csv import Gesture, trajectory_points
from .process import SpeedPolicy
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


def guard_notes(report: Optional[CheckReport],
                guard: str = DEFAULT_GUARD) -> List[str]:
    """The advisory guard lines to show an operator before a gesture moves."""
    if report is None or not report.guard_findings:
        return []
    if guard == "speed_only":
        tail = ("the daemon will NOT check clearances for this gesture "
                "(guard=speed_only): it plays these poses as taught; pass "
                "--guard full to have the daemon refuse them instead "
                "(docs/teach.md, 'Guard')")
    else:
        tail = ("the daemon guards the upload with its own margins "
                "(guard=full) and will refuse it if they are not met "
                "(docs/teach.md, 'Guard')")
    return [f.summary() for f in report.guard_findings] + [tail]


def play(robot, gesture: Gesture, home: Sequence[float], *, no_safety: bool = False,
         speed: Optional[SpeedPolicy] = None,
         check_kwargs=None, settle_s: float = 2.0, guard: str = DEFAULT_GUARD,
         announce: Callable[[str], None] = lambda line: None) -> PlayReport:
    """Play on an ENTERED :class:`FirmwareExecutor`. Returns what happened.
    ``announce`` receives the advisory guard warnings BEFORE anything moves.
    ``guard`` is the daemon trajectory guard for the upload (module doc)."""
    if guard not in GUARDS:
        raise ValueError(f"guard must be one of {GUARDS}, not {guard!r}")
    pre = preflight(gesture, home, no_safety=no_safety, speed=speed,
                    check_kwargs=check_kwargs)
    if not pre.ok:
        return pre
    send_guard: Optional[str] = guard
    fallback = None
    if guard not in robot.trajectory_guards():
        send_guard = None
        fallback = (f"this daemon does not offer trajectory guard={guard} "
                    f"(d1-firmware PR #102 is not deployed); the gesture plays "
                    f"under the daemon's full guard, which refuses clearance "
                    f"violations")
    advisory = guard_notes(pre.check, guard if send_guard else "full")
    if fallback:
        advisory.insert(0, fallback)
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
    points = trajectory_points(gesture, home)
    measured = pose_deg(robot.state())
    points[0] = {"t": 0.0, "a": measured[:7], "b": measured[7:]}
    sent = robot.play_waypoints(points, guard=send_guard)
    end = robot.wait_arrived(_q16(home))
    notes = list(advisory)
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
