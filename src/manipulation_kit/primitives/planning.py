"""Pose waypoints in, a fully checked joint path out — or a typed refusal.

This is the one place a primitive touches the kinematics, so it is the one
place the three guarantees are made:

1. **Every joint step passed the same gate the robot's teleop passes.**
   :meth:`~manipulation_kit.arms.kinematics.GuardedArm.solve_ee` runs the DLS
   IK with the limits enforced INSIDE the iteration, re-checks by forward
   kinematics that the solution actually reaches the pose it was asked for,
   bounds the step with :func:`~manipulation_kit.arms.ik.clamp_joint_step`, and
   collision-checks the resulting TWO-ARM posture. A pose that fails any of
   those ends the plan with the stage's own reason.
2. **The plan is built before anything moves.** A long travel is several
   ``solve_ee`` calls, because each one is clamped to
   ``safety.MAX_JOINT_STEP_RAD``; the path is the sequence of accepted joint
   vectors. That is the same ramp ``dx-inspect-robots`` already streams, and
   it is why a primitive can be refused at waypoint 3 without having executed
   waypoints 0-2 on the robot.
3. **Planning leaves the model exactly as it found it.** ``solve_ee`` POSES
   the kinematic mirror to iterate; a ``plan()`` that left it somewhere else
   would make the next plan depend on the previous one. Both arms' joints are
   saved and restored, always, including on refusal.
"""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from ..arms import safety
from ..arms.ik import clamp_joint_step
from ..world import ArmView, WorldView
from . import approach as ap
from .types import (GUARD_REJECT, IK_FAIL, INFEASIBLE, JointStep, PlanError,
                    UNREACHABLE_OBJECT, Waypoint)

#: How many clamped ``solve_ee`` calls ONE INTERPOLATION KNOT may take before
#: the path is declared not to be converging. A knot is at most one
#: ``MAX_STEP_M`` / ``MAX_STEP_RAD`` of travel, so two or three solves is
#: normal and eight means the solver is circling.
MAX_SOLVES_PER_KNOT = 8

#: Ceiling on the knots one waypoint may be split into. At ``MAX_STEP_M`` =
#: 0.03 m that is 6 m of travel — no primitive waypoint is remotely that long,
#: so hitting it is a bug, not a long move.
MAX_KNOTS_PER_WAYPOINT = 200

#: Clearance points a rejected waypoint is retried through, as ``(up, out)``
#: metres from the tool point the leg STARTS at, keeping the orientation it
#: starts with: lift the hand and swing it away from the body, then travel.
#: "out" is +y for the left arm and -y for the right — away from the torso
#: either way.
#:
#: SHORT, ORDERED CHEAPEST-DETOUR-FIRST, AND FIXED. ``plan()`` is pure, so the
#: candidate a given world picks must not depend on anything except that
#: world. These two were measured, not guessed: 144 candidates (up, out and a
#: fore/aft offset, 0-25 cm each) were swept against 37 top-down grasps spread
#: over the ``blocks-eval`` wagon (x 0.43-0.49, y -0.13..+0.13, both arms,
#: every one starting from HOME). 33 of the 37 can be planned at all; these
#: two cover ALL 33, in a mean of 1.45 attempts, and no third candidate in the
#: sweep covers anything they miss — including every fore/aft variant, which
#: is why the offset is two numbers and not three. The other four targets, far
#: and near the centre line where the arm reaches across itself, have no
#: working via among the 144 and are refused; that is what a refusal is for.
VIA_OFFSETS_M: Tuple[Tuple[float, float], ...] = (
    (0.20, 0.20),
    (0.25, 0.25),
)

#: Refusals a detour can plausibly fix. A guard rejection is a statement about
#: the PATH; ``ik_fail`` on a straight line that has already walked most of the
#: way is usually the solver stuck in a local basin, which a different approach
#: direction also moves. ``unreachable_object`` is not here: no via makes an
#: arm longer.
VIA_REASONS: Tuple[str, ...] = (GUARD_REJECT, IK_FAIL)

#: How close the tool point must get to a knot before the path moves on.
#: 3 mm, just above the IK's own 2 mm position tolerance: asking for tighter
#: than the solver converges to is how a loop spins forever.
ARRIVE_TOL_M = 0.003
#: and the orientation half, a touch above the solver's 5e-2 rad
ARRIVE_TOL_RAD = 0.06


class Kin:
    """A borrowed :class:`~manipulation_kit.arms.kinematics.ArmKinematics`, restored on exit.

    Planning is pure from the caller's point of view; internally the solver
    needs a posable model. This context manager is how both stay true.
    """

    def __init__(self, kin, world: WorldView):
        self.kin = kin
        self.world = world
        self._saved = {}

    def __enter__(self) -> "Kin":
        for side in self.kin.sides:
            self._saved[side] = np.array(self.kin.joints(side), dtype=float)
            arm: Optional[ArmView] = self.world.arm(side)
            if arm is not None:
                self.kin.set_joints(side, arm.joints)
        return self

    def __exit__(self, *_exc) -> None:
        for side, q in self._saved.items():
            self.kin.set_joints(side, q)

    # -- reads, in the plan's own units ------------------------------------ #
    def tool_pose(self, side: str) -> Tuple[np.ndarray, R]:
        p7, r7 = self.kin.ee_pose(side)
        return ap.tool_from_link7(p7, r7)

    def joints(self, side: str) -> np.ndarray:
        return np.array(self.kin.joints(side), dtype=float)


def _pose_error(kin: "Kin", side: str, p, r: R) -> Tuple[float, float]:
    p_now, r_now = kin.tool_pose(side)
    return (float(np.linalg.norm(p_now - np.asarray(p, dtype=float))),
            float(np.linalg.norm((r_now.inv() * r).as_rotvec())))


def knots(p0, r0: R, p1, r1: R,
          *, max_step_m: float = safety.MAX_STEP_M,
          max_step_rad: float = safety.MAX_STEP_RAD):
    """Split one waypoint into per-tick targets the IK can actually track.

    THE REASON THIS EXISTS, because it looks like padding and is not: the
    solver is a LOCAL one. Damped least squares seeded at the current posture
    walks downhill, and asking it to go from HOME (wrist forward) to a top-down
    grasp in one call puts it in a local minimum — measured on the bundled
    URDF, a top-down pose at (0.40, 0.12, 0.18) that 14 of 40 random restarts
    solve returns ``None`` from the HOME seed at 60, 300 AND 1000 iterations.
    Walk the same pose in ``MAX_STEP_M`` / ``MAX_STEP_RAD`` knots and every one
    of them converges, because each is a small perturbation of a solved
    posture. It is also the shape the robot needs: the knots ARE the 50 Hz
    targets, bounded by the same per-tick caps the teleop clutch enforces.

    Straight line in position, shortest arc in orientation.
    """
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    rotvec = (r0.inv() * r1).as_rotvec()
    steps = max(int(np.ceil(np.linalg.norm(p1 - p0) / max_step_m)),
                int(np.ceil(np.linalg.norm(rotvec) / max_step_rad)), 1)
    steps = min(steps, MAX_KNOTS_PER_WAYPOINT)
    for k in range(1, steps + 1):
        f = k / steps
        yield p0 + (p1 - p0) * f, r0 * R.from_rotvec(rotvec * f)


def _straight(kin: Kin, side: str, wp: Waypoint, *, primitive: str,
              index: int) -> Tuple[List[JointStep], Optional[PlanError]]:
    """ONE waypoint, straight line, no detour. The old whole of ``solve_path``.

    Kept separate because the via search runs it three times — once to find
    out that the straight line does not work, once per leg of the detour — and
    all three must be the same code, or "the via worked" would mean something
    different from "the straight line worked".
    """
    steps: List[JointStep] = []
    pos_err, rot_err = _pose_error(kin, side, wp.p, wp.r)
    if pos_err <= ARRIVE_TOL_M and rot_err <= ARRIVE_TOL_RAD:
        return steps, None
    p0, r0 = kin.tool_pose(side)
    for p_knot, r_knot in knots(p0, r0, wp.p, wp.r):
        for _ in range(MAX_SOLVES_PER_KNOT):
            pos_err, rot_err = _pose_error(kin, side, p_knot, r_knot)
            if pos_err <= ARRIVE_TOL_M and rot_err <= ARRIVE_TOL_RAD:
                break
            p7, r7 = ap.link7_from_tool(p_knot, r_knot)
            result = kin.kin.solve_ee(side, p7, r7)
            if not result.ok:
                residual, _ = _pose_error(kin, side, wp.p, wp.r)
                return steps, PlanError(
                    _plan_reason(result.reason),
                    _explain(result.reason, side, wp),
                    waypoint_index=index,
                    waypoint_label=wp.label, residual_m=residual,
                    primitive=primitive, side=side)
            steps.append(JointStep(side, kin.joints(side), index))
    residual, rot_residual = _pose_error(kin, side, wp.p, wp.r)
    if residual > ARRIVE_TOL_M * 4 or rot_residual > ARRIVE_TOL_RAD * 2:
        return steps, PlanError(
            UNREACHABLE_OBJECT,
            f"the {side} tool point stopped converging on {wp.label!r}: "
            f"{residual * 1000:.0f} mm and "
            f"{np.degrees(rot_residual):.0f} deg short after the whole "
            f"interpolated path",
            waypoint_index=index, waypoint_label=wp.label,
            residual_m=residual, primitive=primitive, side=side)
    return steps, None


def _outward(side: str) -> float:
    """Which way is AWAY from the torso for this arm. +y is the robot's left."""
    return 1.0 if side == "left" else -1.0


def _via_note(wp: Waypoint, up: float, out: float) -> str:
    return (f"the straight line to {wp.label!r} was refused; routed via a "
            f"clearance point {up * 100:.0f} cm up and {out * 100:.0f} cm "
            f"out from the torso")


def _detour(kin: Kin, side: str, wp: Waypoint, *, primitive: str, index: int,
            q_start: np.ndarray,
            ) -> Tuple[Optional[List[JointStep]], str]:
    """Try the candidate clearance points, then the READY re-seed. In order.

    Deterministic by construction: a fixed list walked front to back, the first
    one whose BOTH legs plan wins, and the mirror is put back to ``q_start``
    between attempts so attempt *n* cannot inherit attempt *n-1*'s posture.
    """
    out_sign = _outward(side)
    for up, out in VIA_OFFSETS_M:
        kin.kin.set_joints(side, q_start)
        p0, r0 = kin.tool_pose(side)
        via = Waypoint(f"via:{wp.label}",
                       p0 + np.array([0.0, out_sign * out, up]), r0)
        leg1, error = _straight(kin, side, via, primitive=primitive, index=index)
        if error is not None:
            continue
        leg2, error = _straight(kin, side, wp, primitive=primitive, index=index)
        if error is None:
            return leg1 + leg2, _via_note(wp, up, out)
    # Last resort: the arm's own READY posture. It is a joint vector the kit
    # SEARCHED for (``ik.find_ready_seed``: maximum clearance, guard-clean), so
    # when the straight line dies on geometry near HOME, re-seeding the solver
    # from there is a different basin rather than a different point on the same
    # line. Skipped when this arm has no distinct READY, because ``ready()``
    # falls back to HOME and ramping HOME->HOME is not a second attempt.
    kin.kin.set_joints(side, q_start)
    ready = np.asarray(kin.kin.ready(side), dtype=float)
    if not np.allclose(ready, kin.kin.home(side)):
        ramp, error = joint_ramp(kin, side, ready, primitive=primitive,
                                 label="ready")
        if error is None:
            leg, error = _straight(kin, side, wp, primitive=primitive,
                                   index=index)
            if error is None:
                return [JointStep(side, step.q, index) for step in ramp] + leg, (
                    f"the straight line to {wp.label!r} was refused by the "
                    f"guard; re-seeded from the READY posture")
    return None, ""


def solve_path(kin: Kin, side: str, waypoints: Sequence[Waypoint], *,
               primitive: str, start_index: int = 0, allow_via: bool = True,
               ) -> Tuple[List[JointStep], Optional[PlanError], List[str]]:
    """Drive the tool point through ``waypoints``; return the steps or the refusal.

    Each waypoint is split into per-tick knots (see :func:`knots`) and each
    accepted ``solve_ee`` is one :class:`~.types.JointStep`. A waypoint already
    satisfied contributes no step, which is what makes a zero ``Nudge`` an
    empty plan rather than a fake one.

    WHEN A KNOT IS GUARD-REJECTED THE STRAIGHT LINE IS NOT THE ANSWER. Measured
    2026-09-19 on the ``blocks-eval`` scene: a top-down grasp over the wagon
    has a standoff pose and a grasp pose that are both guard-CLEAN (36 mm of
    body clearance), and the straight line from HOME to them puts ``Link4_R``
    inside ``torso_belly`` at knot 1-3. Returning the first rejected knot's
    refusal deleted the grasp from the model's menu — 61 of 61 turn-0
    approaches refused — for a goal the arm can hold perfectly well. So a
    guard-rejected waypoint is retried through :data:`VIA_OFFSETS_M`, and only
    a waypoint no candidate reaches is refused. The refusal, when it comes,
    is still the STRAIGHT line's: same reason, same knot index, same residual,
    so nothing about the detour changes what a dead end looks like.

    Returns ``(steps, error, notes)``; ``notes`` names any detour taken, so a
    plan that went around something says so in its own record.
    """
    steps: List[JointStep] = []
    notes: List[str] = []
    for index, wp in enumerate(waypoints):
        at = start_index + index
        q_start = kin.joints(side)
        leg, error = _straight(kin, side, wp, primitive=primitive, index=at)
        if error is None:
            steps += leg
            continue
        if not allow_via or error.reason not in VIA_REASONS:
            return steps + leg, error, notes
        detour, note = _detour(kin, side, wp, primitive=primitive, index=at,
                               q_start=q_start)
        if detour is None:
            # Nothing worked. Re-walk the straight line so the steps handed
            # back, and the refusal, describe the path that was ASKED for
            # rather than the last candidate that happened to be tried.
            kin.kin.set_joints(side, q_start)
            leg, error = _straight(kin, side, wp, primitive=primitive, index=at)
            return steps + leg, error, notes
        steps += detour
        notes.append(note)
    return steps, None, notes


def _plan_reason(ik_reason: str) -> str:
    return {"ik_fail": IK_FAIL, "infeasible": INFEASIBLE,
            "guard_reject": GUARD_REJECT}.get(ik_reason, IK_FAIL)


def _explain(ik_reason: str, side: str, wp: Waypoint) -> str:
    if ik_reason == "guard_reject":
        return (f"the motion guard refused the {side} arm's posture at "
                f"{wp.label!r} — it would hit the body, the other arm or itself")
    if ik_reason == "infeasible":
        return (f"the solver returned a posture for {wp.label!r} that does not "
                f"reach it; treat as a solver bug, not an unreachable target")
    return (f"no in-limit {side}-arm posture puts the tool on {wp.label!r}")


def joint_ramp(kin: Kin, side: str, q_goal, *, primitive: str,
               label: str = "goal",
               limit: float = safety.MAX_JOINT_STEP_RAD,
               ) -> Tuple[List[JointStep], Optional[PlanError]]:
    """A guarded joint-space ramp to ``q_goal`` — no IK, for HOME and stow.

    ``GoHome`` has no Cartesian goal: the destination is a joint vector, and
    running it through IK would invent an end-effector pose nobody asked for
    and could fail to reach a posture the arm demonstrably holds. It still
    passes the collision guard at every step, and still moves at most
    ``MAX_JOINT_STEP_RAD`` per step.
    """
    steps: List[JointStep] = []
    q_goal = np.asarray(q_goal, dtype=float).reshape(7)
    for _ in range(MAX_KNOTS_PER_WAYPOINT):
        q_now = kin.joints(side)
        if float(np.max(np.abs(q_goal - q_now))) <= 1e-6:
            return steps, None
        q_next, _ = clamp_joint_step(q_now, q_goal, limit)
        if kin.kin.gate.installed and not kin.kin.guard_ok(side, q_next):
            return steps, PlanError(
                GUARD_REJECT,
                f"the motion guard refused the {side} arm on the way to {label}",
                waypoint_label=label, primitive=primitive, side=side,
                residual_m=float(np.max(np.abs(q_goal - q_now))))
        kin.kin.set_joints(side, q_next)
        steps.append(JointStep(side, q_next, 0))
    return steps, PlanError(
        UNREACHABLE_OBJECT,
        f"the {side} arm did not converge on {label} within the step budget",
        waypoint_label=label, primitive=primitive, side=side)
