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


def solve_path(kin: Kin, side: str, waypoints: Sequence[Waypoint], *,
               primitive: str, start_index: int = 0,
               ) -> Tuple[List[JointStep], Optional[PlanError]]:
    """Drive the tool point through ``waypoints``; return the steps or the refusal.

    Each waypoint is split into per-tick knots (see :func:`knots`) and each
    accepted ``solve_ee`` is one :class:`~.types.JointStep`. A waypoint already
    satisfied contributes no step, which is what makes a zero ``Nudge`` an
    empty plan rather than a fake one.
    """
    steps: List[JointStep] = []
    for index, wp in enumerate(waypoints):
        pos_err, rot_err = _pose_error(kin, side, wp.p, wp.r)
        if pos_err <= ARRIVE_TOL_M and rot_err <= ARRIVE_TOL_RAD:
            continue
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
                        waypoint_index=start_index + index,
                        waypoint_label=wp.label, residual_m=residual,
                        primitive=primitive, side=side)
                steps.append(JointStep(side, kin.joints(side), start_index + index))
        residual, rot_residual = _pose_error(kin, side, wp.p, wp.r)
        if residual > ARRIVE_TOL_M * 4 or rot_residual > ARRIVE_TOL_RAD * 2:
            return steps, PlanError(
                UNREACHABLE_OBJECT,
                f"the {side} tool point stopped converging on {wp.label!r}: "
                f"{residual * 1000:.0f} mm and "
                f"{np.degrees(rot_residual):.0f} deg short after the whole "
                f"interpolated path",
                waypoint_index=start_index + index, waypoint_label=wp.label,
                residual_m=residual, primitive=primitive, side=side)
    return steps, None


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
