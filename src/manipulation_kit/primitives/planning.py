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

from typing import TYPE_CHECKING, List, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from ..arms import safety
from ..arms.ik import clamp_joint_step
from ..world import ArmView, WorldView
from . import grasp_geometry as gg
from . import orientation as ap
from .types import (GUARD_REJECT, IK_FAIL, INFEASIBLE, JOINT_LIMIT, JointStep, PlanError, UNREACHABLE_OBJECT, Waypoint)

if TYPE_CHECKING:  # pragma: no cover
    from .clearance import SceneGate

#: How many clamped ``solve_ee`` calls ONE INTERPOLATION KNOT may take before
#: the path is declared not to be converging. A knot is at most one
#: ``MAX_STEP_M`` / ``MAX_STEP_RAD`` of travel, so two or three solves is
#: normal and eight means the solver is circling.
MAX_SOLVES_PER_KNOT = 8

#: Ceiling on the knots one waypoint may be split into. At ``MAX_STEP_M`` =
#: 0.03 m that is 6 m of travel — no primitive waypoint is remotely that long,
#: so hitting it is a bug, not a long move.
MAX_KNOTS_PER_WAYPOINT = 200

#: THE FALLBACK, not the default. With a scene in :class:`Kin` a free-space
#: leg is planned up-and-over by construction (:func:`_over_the_top`); these
#: are what is tried after that, and after the straight line, has failed.
#:
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
#: ``joint_limit`` is here for the same reason as ``ik_fail``: a coupled
#: limit (the D1 wrist roll) binds in one posture branch and not in another,
#: and a detour arrives in a different one.
VIA_REASONS: Tuple[str, ...] = (GUARD_REJECT, IK_FAIL, JOINT_LIMIT)

#: How close the tool point must get to a knot before the path moves on.
#: 3 mm, just above the IK's own 2 mm position tolerance: asking for tighter
#: than the solver converges to is how a loop spins forever.
ARRIVE_TOL_M = 0.003
#: and the orientation half, a touch above the solver's 5e-2 rad
ARRIVE_TOL_RAD = 0.06

#: How far a knot may be MISSED and the path still be walked on. The solver
#: converges to :data:`ARRIVE_TOL_M` over most of the workspace and plateaus
#: near the reach limit — measured on the bundled URDF, the last four knots of
#: a HOME -> standoff travel at (0.38, 0.25) sit at 4.7-9.1 mm however many
#: solves they are given. Walking on from those is right; walking on from a
#: knot the solver is 50 mm from is not, and the old code could not tell the
#: two apart because it checked NOTHING at a knot and then judged the final
#: waypoint against this same window (R: "Exhausting eight solves does not
#: refuse an intermediate knot").
#:
#: So: exhaustion inside this window walks on and the residual travels with
#: the plan; exhaustion outside it is a refusal naming the knot. The window is
#: WIDER than the 3 mm support margin, which is why a grasp additionally
#: validates its ACHIEVED descent rather than its ideal waypoint — see
#: ``Grasp.plan``.
PATH_TOL_M = 0.012
PATH_TOL_RAD = 0.12

#: How far the HAND reaches sideways from the tool point, for deciding what a
#: transit passes over [m]: half the 67 mm palm plus a little. The scene gate
#: checks the arm's links; the hand is kept clear in transit by rising over
#: what is in its way, and this is the corridor it rises over.
TRANSIT_HALF_WIDTH_M = 0.04

#: The hand as a box around the tool point, in the TCP frame [m]: across the
#: jaws (x) the open fingers, 60.5 mm apart on d1-2, plus their thickness;
#: across the palm (y) half its 67 mm; along the approach axis (z) from the
#: flange (100 mm behind the pad centre) to the pad tips (29 mm past it). Used
#: only to say how far the hand HANGS below the tool point for a given
#: orientation — the rise height of an up-and-over transit.
HAND_BOX_TCP_M = ((-0.045, 0.045), (-0.034, 0.034),
                  (-ap.TOOL_Z_M, gg.PAD.lead_m))


class IncompleteObservation(LookupError):
    """The world does not describe an arm the guard has to reason about."""


def missing_arms(kin, world: WorldView) -> Tuple[str, ...]:
    """Sides the model has and the observation does not.

    The collision guard is DUAL-ARM: it decides whether a posture is safe by
    looking at both arms at once. An observation with one arm in it leaves the
    other at whatever the shared model happened to be left at, so the same
    plan is safe or unsafe depending on the previous caller — which is exactly
    the hidden-state dependence R8 names.
    """
    return tuple(side for side in kin.sides if world.arm(side) is None)


class Kin:
    """A borrowed :class:`~manipulation_kit.arms.kinematics.ArmKinematics`, restored on exit.

    Planning is pure from the caller's point of view; internally the solver
    needs a posable model. This context manager is how both stay true — and
    it now holds the model's own lock for the whole transaction, because two
    interleaved plans posing the same mirror contaminate each other and
    "restored afterwards" does not help when the other plan reads in between.

    It also REFUSES an observation that does not carry every arm the model
    has. Silently keeping the model's own joints for a missing arm made the
    plan depend on whoever posed the mirror last.

    ``scene`` is the optional :class:`~.clearance.SceneGate` of the world the
    plan is made in: the arm's links against the tables, containers and
    objects, swept between consecutive steps.
    """

    def __init__(self, kin, world: WorldView, *, require_all_arms: bool = True,
                 scene: Optional["SceneGate"] = None):
        self.kin = kin
        self.world = world
        self.require_all_arms = bool(require_all_arms)
        #: the scene half of the collision check (``primitives.clearance``):
        #: when set, every accepted joint step is swept against it and free
        #: transit rises over what is in its way. ``None`` = the body guard
        #: only, which is what a plan made without a world scene gets.
        self.scene = scene
        #: scene refusals met while searching one waypoint's routes, so a
        #: dead end whose STRAIGHT line died on the body guard still says
        #: which obstacle closed the ways around it
        self.scene_refusals: List[PlanError] = []
        self._saved = {}
        self._lock = getattr(kin, "lock", None)
        self._held = False

    def __enter__(self) -> "Kin":
        missing = missing_arms(self.kin, self.world)
        if self.require_all_arms and missing:
            raise IncompleteObservation(
                f"the observation reports no joints for: {', '.join(missing)}. "
                f"The collision guard checks BOTH arms at once, so a plan made "
                f"without one is a statement about a posture nobody measured.")
        if self._lock is not None:
            self._lock.acquire()
            self._held = True
        try:
            for side in self.kin.sides:
                self._saved[side] = np.array(self.kin.joints(side), dtype=float)
                arm: Optional[ArmView] = self.world.arm(side)
                if arm is not None:
                    self.kin.set_joints(side, arm.joints)
        except BaseException:
            # Half-posed is worse than not posed: put back whatever was read
            # before re-raising, then drop the lock.
            self.__exit__(None, None, None)
            raise
        return self

    def __exit__(self, *_exc) -> None:
        try:
            for side, q in self._saved.items():
                self.kin.set_joints(side, q)
        finally:
            self._saved = {}
            if self._held and self._lock is not None:
                self._held = False
                self._lock.release()

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
    if steps > MAX_KNOTS_PER_WAYPOINT:
        # NOT clamped. Clamping kept the count and widened the spacing, which
        # is the one thing the spacing exists to bound. ``_straight`` refuses
        # such a leg before it gets here; this is the belt to that's braces.
        raise ValueError(
            f"a {float(np.linalg.norm(p1 - p0)) * 1000:.0f} mm / "
            f"{float(np.degrees(np.linalg.norm(rotvec))):.0f} deg leg needs "
            f"{steps} knots at the planner's spacing, over the "
            f"{MAX_KNOTS_PER_WAYPOINT} budget")
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
    worst_m = worst_rad = 0.0
    pos_err, rot_err = _pose_error(kin, side, wp.p, wp.r)
    if pos_err <= ARRIVE_TOL_M and rot_err <= ARRIVE_TOL_RAD:
        return steps, None
    p0, r0 = kin.tool_pose(side)
    plan_knots = list(knots(p0, r0, wp.p, wp.r))
    if _too_far(p0, r0, wp.p, wp.r):
        residual, rot_residual = _pose_error(kin, side, wp.p, wp.r)
        return steps, PlanError(
            INFEASIBLE,
            f"the leg to {wp.label!r} is {residual * 1000:.0f} mm and "
            f"{np.degrees(rot_residual):.0f} deg long, which needs more than "
            f"the {MAX_KNOTS_PER_WAYPOINT} interpolation knots this planner "
            f"will spend at its {safety.MAX_STEP_M * 1000:.0f} mm / "
            f"{np.degrees(safety.MAX_STEP_RAD):.0f} deg spacing. Split it "
            f"into shorter waypoints rather than widening the spacing",
            waypoint_index=index, waypoint_label=wp.label,
            residual_m=residual, residual_rad=rot_residual, stage="knots",
            primitive=primitive, side=side)
    for knot_index, (p_knot, r_knot) in enumerate(plan_knots):
        converged = False
        for _ in range(MAX_SOLVES_PER_KNOT):
            pos_err, rot_err = _pose_error(kin, side, p_knot, r_knot)
            if pos_err <= ARRIVE_TOL_M and rot_err <= ARRIVE_TOL_RAD:
                converged = True
                break
            p7, r7 = ap.link7_from_tool(p_knot, r_knot)
            q_before = kin.joints(side)
            result = kin.kin.solve_ee(side, p7, r7)
            if result.ok and kin.scene is not None:
                error = _scene_check(kin, side, q_before, kin.joints(side), wp,
                                     primitive=primitive, index=index)
                if error is not None:
                    # leave the model where the last ACCEPTED step put it
                    kin.kin.set_joints(side, q_before)
                    return steps, error
            if not result.ok:
                residual, rot_residual = _pose_error(kin, side, wp.p, wp.r)
                return steps, PlanError(
                    _plan_reason(result.reason),
                    _explain(result.reason, side, wp,
                             getattr(result, "detail", "")),
                    waypoint_index=index,
                    waypoint_label=wp.label, residual_m=residual,
                    residual_rad=rot_residual, stage="straight",
                    primitive=primitive, side=side)
            if not wp.allow_via:
                stray = _off_line_m(kin.tool_pose(side)[0], p0, wp.p)
                if stray > PATH_TOL_M:
                    # A leg whose SHAPE is the promise (a descent, a lift, a
                    # nudge) is walked on its line or refused. ``solve_ee``
                    # reaching the knot is not enough: with the wrist roll
                    # held inside the coupled limit the solver can converge
                    # on another posture branch, and the clamped partial
                    # steps toward it carry the tool off the line (measured:
                    # 110 mm on a 17 cm leg across a crate).
                    kin.kin.set_joints(side, q_before)
                    residual, rot_residual = _pose_error(kin, side, wp.p, wp.r)
                    return steps, PlanError(
                        IK_FAIL,
                        f"the {side} arm cannot keep the tool on the straight "
                        f"line to {wp.label!r}: the next step leaves it by "
                        f"{stray * 1000:.0f} mm (the solver changed posture "
                        f"branch), over the {PATH_TOL_M * 1000:.0f} mm path "
                        f"window",
                        waypoint_index=index, waypoint_label=wp.label,
                        residual_m=residual, residual_rad=rot_residual,
                        stage="off_line", primitive=primitive, side=side)
            steps.append(JointStep(side, kin.joints(side), index))
        if not converged:
            # EXHAUSTION IS CHECKED, not shrugged off. Every knot now gets a
            # verdict of its own: inside PATH_TOL the path walks on and the
            # miss is recorded, outside it the leg is refused AT THAT KNOT
            # rather than 13 knots later against a window four times the
            # tolerance.
            pos_err, rot_err = _pose_error(kin, side, p_knot, r_knot)
            worst_m, worst_rad = max(worst_m, pos_err), max(worst_rad, rot_err)
            if pos_err > PATH_TOL_M or rot_err > PATH_TOL_RAD:
                residual, rot_residual = _pose_error(kin, side, wp.p, wp.r)
                return steps, PlanError(
                    IK_FAIL,
                    f"the {side} arm did not converge on knot {knot_index} of "
                    f"{len(plan_knots)} on the way to {wp.label!r}: "
                    f"{pos_err * 1000:.1f} mm and {np.degrees(rot_err):.1f} "
                    f"deg short after {MAX_SOLVES_PER_KNOT} solves, outside "
                    f"the {PATH_TOL_M * 1000:.0f} mm / "
                    f"{np.degrees(PATH_TOL_RAD):.0f} deg path window",
                    waypoint_index=index, waypoint_label=wp.label,
                    residual_m=residual, residual_rad=rot_residual,
                    stage="knot_exhausted", primitive=primitive, side=side)
    residual, rot_residual = _pose_error(kin, side, wp.p, wp.r)
    if residual > PATH_TOL_M or rot_residual > PATH_TOL_RAD:
        return steps, PlanError(
            UNREACHABLE_OBJECT,
            f"the {side} tool point stopped converging on {wp.label!r}: "
            f"{residual * 1000:.1f} mm and "
            f"{np.degrees(rot_residual):.1f} deg short after the whole "
            f"interpolated path",
            waypoint_index=index, waypoint_label=wp.label,
            residual_m=residual, residual_rad=rot_residual, stage="arrival",
            primitive=primitive, side=side)
    return steps, None


def _off_line_m(p, a, b) -> float:
    """Distance [m] of ``p`` from the SEGMENT ``a``-``b``."""
    p, a, b = (np.asarray(v, dtype=float) for v in (p, a, b))
    ab = b - a
    n = float(np.dot(ab, ab))
    t = 0.0 if n <= 0.0 else min(max(float(np.dot(p - a, ab)) / n, 0.0), 1.0)
    return float(np.linalg.norm(p - (a + t * ab)))


def _scene_check(kin: Kin, side: str, q_from, q_to, wp: Waypoint, *,
                 primitive: str, index: int) -> Optional[PlanError]:
    """Sweep one accepted joint step against the scene; the refusal, or None.

    The refusal is the body guard's own reason, ``guard_reject``, so every
    caller that already routes around a guard rejection (the via search) and
    every model that already reads one keeps working — with the obstacle
    NAMED and ``residual_m`` = how far the link is inside the clearance that
    obstacle requires (Shu decision 4: refuse, and say by how much).
    """
    report = kin.scene.swept_ok(kin, side, q_from, q_to)
    if report.ok:
        return None
    error = PlanError(
        GUARD_REJECT,
        f"on the way to {wp.label!r}, {report.describe(side)}. The scene "
        f"guard checks the arm's links against every declared table, "
        f"container and object; correct the declaration if {report.obstacle!r}"
        f" is not where the world says, or approach from another direction",
        waypoint_index=index, waypoint_label=wp.label,
        residual_m=report.depth_m, stage="scene", primitive=primitive,
        side=side, attempted=(f"obstacle:{report.obstacle}",
                              f"link:{report.link}"))
    kin.scene_refusals.append(error)
    return error


def _with_scene_refusals(kin: Kin, error: PlanError) -> PlanError:
    """A refusal that is NOT the scene's own, told what the scene refused.

    The via search re-walks the straight line to report a dead end, so a
    straight line that died on the body guard is what comes back — even when
    every route around it was closed by a table. The model needs the table's
    name to correct anything; this appends it.
    """
    if error.stage == "scene" or not kin.scene_refusals:
        return error
    import dataclasses  # noqa: PLC0415
    first = kin.scene_refusals[0]
    return dataclasses.replace(
        error,
        detail=(f"{error.detail}; the routes around it were refused by the "
                f"scene guard: {first.detail}"),
        attempted=tuple(error.attempted) + tuple(
            a for a in first.attempted if a not in error.attempted))


def _hand_hang(kin: Kin, side: str, p_tool, r_tool: R) -> float:
    """How far below the tool point the hand — and what it holds — reaches."""
    corners = np.array([[x, y, z] for x in HAND_BOX_TCP_M[0]
                        for y in HAND_BOX_TCP_M[1] for z in HAND_BOX_TCP_M[2]])
    hang = float(max(0.0, -np.min(r_tool.apply(corners)[:, 2])))
    gripper = kin.world.gripper(side)
    held = None if gripper is None else gripper.held_object
    item = kin.world.find(held) if held else None
    if item is not None:
        try:
            hang = max(hang, float(p_tool[2]) - item.bottom_z(kin.world.frames))
        except Exception:  # noqa: BLE001 - an unresolved frame: keep the hand
            pass
    return hang


def _transit_ceiling(kin: Kin, side: str, p_from, p_to, r_from: R, r_to: R
                     ) -> Tuple[float, Tuple[str, ...]]:
    """What the hand would pass over, low, on the tool segment ``p_from -> p_to``
    (the hand's hang taken at whichever end orientation hangs it lower)."""
    hang = max(_hand_hang(kin, side, p_from, r_from),
               _hand_hang(kin, side, p_from, r_to))
    return kin.scene.in_the_way(p_from, p_to, hang_m=hang,
                                width_m=TRANSIT_HALF_WIDTH_M)


def _over_the_top(kin: Kin, side: str, wp: Waypoint
                  ) -> Optional[Tuple[float, Tuple[str, ...],
                                      List[Tuple[str, List[Waypoint]]]]]:
    """Up-and-over routes for a free transit, or None if nothing is in the way.

    THE DEFAULT SHAPE OF A FREE TRANSIT with a scene: rise to the height that
    clears everything the hand would otherwise pass over low (its top + that
    obstacle's required clearance + how far the hand hangs below the tool
    point), traverse at that height, descend straight onto the waypoint. By
    construction, not as a recovery — the old planner flew the straight line
    and detoured only when the body guard said no, which it never did for a
    table, because it cannot see one.

    Where the arm rises TO is the one choice: straight up first, then the
    fixed clearance points of :data:`VIA_OFFSETS_M` (up and away from the
    torso, measured to be where this arm can reorient from HOME), each raised
    to at least the clearance height. A route one of whose legs would itself
    pass low over something is not offered.

    Returns ``(height, names in the way, [(how, waypoints)])``.
    """
    scene = kin.scene
    if scene is None or not scene.obstacles:
        return None
    p0, r0 = kin.tool_pose(side)
    height, names = _transit_ceiling(kin, side, p0, wp.p, r0, wp.r)
    if not names:
        return None
    tail: List[Waypoint] = []
    if wp.p[2] < height:
        tail.append(Waypoint(f"over:{wp.label}", [wp.p[0], wp.p[1], height],
                             wp.r))
    tail.append(wp)
    rises: List[Tuple[str, Optional[np.ndarray]]] = [
        ("straight up", None if p0[2] >= height
         else np.array([p0[0], p0[1], height]))]
    out_sign = _outward(side)
    for up, out in VIA_OFFSETS_M:
        rises.append((f"{max(up, height - p0[2]) * 100:.0f} cm up and "
                      f"{out * 100:.0f} cm out",
                      p0 + np.array([0.0, out_sign * out,
                                     max(up, float(height - p0[2]))])))
    routes: List[Tuple[str, List[Waypoint]]] = []
    for how, rise in rises:
        legs = ([] if rise is None
                else [Waypoint(f"rise:{wp.label}", rise, r0)]) + tail
        points = [(p0, r0)] + [(leg.p, leg.r) for leg in legs]
        if any(_transit_ceiling(kin, side, a[0], b[0], a[1], b[1])[1]
               for a, b in zip(points[:-1], points[1:])):
            continue
        routes.append((how, legs))
    return height, names, routes


def _too_far(p0, r0, p1, r1: R) -> bool:
    """Is this leg longer than the knot budget can cover at its own spacing?

    ``knots`` used to CAP the count at 200 by widening the spacing, which
    silently broke the per-tick bound the spacing exists to enforce. A leg
    that long is a caller asking for the wrong thing, so it is refused.
    """
    span_m = float(np.linalg.norm(np.asarray(p1, dtype=float)
                                  - np.asarray(p0, dtype=float)))
    span_rad = float(np.linalg.norm((r0.inv() * r1).as_rotvec()))
    return (span_m > MAX_KNOTS_PER_WAYPOINT * safety.MAX_STEP_M
            or span_rad > MAX_KNOTS_PER_WAYPOINT * safety.MAX_STEP_RAD)


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


def _fly_over(kin: Kin, side: str, wp: Waypoint, over, q_start, *,
              primitive: str, index: int
              ) -> Tuple[Optional[List[JointStep]], str]:
    """Walk the up-and-over routes in order; the first that plans wins.

    Returns ``(steps, note)``, or ``(None, note)`` when no route plans — the
    caller then falls back to the straight line and the fixed vias, and the
    note says the hand's clearance over ``names`` was not constructed.
    """
    height, names, routes = over
    what = ", ".join(repr(n) for n in names)
    for how, legs in routes:
        kin.kin.set_joints(side, q_start)
        route: List[JointStep] = []
        error = None
        for leg in legs:
            part, error = _straight(kin, side, leg, primitive=primitive,
                                    index=index)
            route += part
            if error is not None:
                break
        if error is None:
            return route, (f"transit to {wp.label!r} rose {how} to "
                           f"z={height:.3f} m to clear {what}")
    kin.kin.set_joints(side, q_start)
    return None, (f"no route over {what} planned (the hand would pass under "
                  f"z={height:.3f} m); the transit to {wp.label!r} fell back "
                  f"to the direct path, with the arm's links still checked "
                  f"against the scene")


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

    A DETOUR IS PER LEG. ``allow_via`` here is the caller's global switch and
    ``Waypoint.allow_via`` is the leg's own: a detour is only ever taken when
    BOTH say yes. Free-space transit says yes; a grasp descent, a lift, a
    nudge and a retreat say no, because for those the shape of the path is the
    promise and a 25 cm clearance hop that lands on the endpoint has not kept
    it (R10).

    WITH A SCENE (``kin.scene``) A FREE LEG IS UP-AND-OVER BY CONSTRUCTION.
    When the hand would pass low over a table, container or object on the
    straight line, the leg rises over it, traverses and descends
    (:func:`_over_the_top`); the fixed clearance points are where it rises
    through when straight up does not plan. Only when no route over plans is
    the straight line / via search above tried — every step of it still swept
    against the scene for the arm's links — and the note says so.

    Returns ``(steps, error, notes)``; ``notes`` names any detour taken, so a
    plan that went around something says so in its own record.
    """
    steps: List[JointStep] = []
    notes: List[str] = []
    for index, wp in enumerate(waypoints):
        at = start_index + index
        q_start = kin.joints(side)
        kin.scene_refusals = []
        if allow_via and wp.allow_via:
            over = _over_the_top(kin, side, wp)
            if over is not None:
                route, note = _fly_over(kin, side, wp, over, q_start,
                                        primitive=primitive, index=at)
                if route is not None:
                    steps += route
                    notes.append(note)
                    continue
                # No route over plans. The FALLBACK is the pre-scene transit
                # (straight line, then the fixed vias) — with every step
                # still swept against the scene for the arm's links. Refusing
                # here would refuse legs the arm demonstrably flies; the note
                # says the hand's clearance was not constructed.
                pending_note = note
            else:
                pending_note = ""
        else:
            pending_note = ""
        leg, error = _straight(kin, side, wp, primitive=primitive, index=at)
        if error is None:
            steps += leg
            if pending_note:
                notes.append(pending_note)
            continue
        if not (allow_via and wp.allow_via) or error.reason not in VIA_REASONS:
            return steps + leg, error, notes
        detour, note = _detour(kin, side, wp, primitive=primitive, index=at,
                               q_start=q_start)
        if detour is None:
            # Nothing worked. Re-walk the straight line so the steps handed
            # back, and the refusal, describe the path that was ASKED for
            # rather than the last candidate that happened to be tried.
            kin.kin.set_joints(side, q_start)
            leg, error = _straight(kin, side, wp, primitive=primitive, index=at)
            return steps + leg, _with_scene_refusals(kin, error), notes
        steps += detour
        if pending_note:
            notes.append(pending_note)
        notes.append(note)
    return steps, None, notes


def _plan_reason(ik_reason: str) -> str:
    return {"ik_fail": IK_FAIL, "infeasible": INFEASIBLE,
            "guard_reject": GUARD_REJECT,
            "joint_limit": JOINT_LIMIT}.get(ik_reason, IK_FAIL)


def _explain(ik_reason: str, side: str, wp: Waypoint, detail: str = "") -> str:
    if ik_reason == "joint_limit":
        return (f"the {side} arm cannot put the tool on {wp.label!r} inside "
                f"its coupled joint limits: {detail}. Another roll of the "
                f"hand, or a spot the wrist reaches with less pitch, may")
    if ik_reason == "guard_reject":
        return (f"the motion guard refused the {side} arm's posture at "
                f"{wp.label!r} — it would hit the body, the other arm or itself")
    if ik_reason == "infeasible":
        return (f"the solver returned a posture for {wp.label!r} that does not "
                f"reach it; treat as a solver bug, not an unreachable target")
    return (f"no in-limit {side}-arm posture puts the tool on {wp.label!r}")


def coupled_limit_notes(kin, steps) -> Tuple[str, ...]:
    """A note per coupled limit the plan's postures come NEAR.

    The plan is inside every coupled limit by construction (the IK projects
    onto them); what a reader cannot see from the joints is that the wrist
    ends up a few degrees from where the hardware stops — the posture where a
    small calibration error, a cable, or an unmeasured stretch of the table
    decides. ``kin`` is the arm model (``Kin.kin``); a model without coupled
    limits gets no notes. The closest posture per limit is reported.
    """
    from ..arms.coupled_limits import NEAR_LIMIT_DEG  # noqa: PLC0415
    from .types import ContactStep  # noqa: PLC0415
    limits_of = getattr(kin, "coupled_limits", None)
    if limits_of is None:
        return ()
    closest = {}
    for step in steps:
        if isinstance(step, JointStep):
            postures = ((step.side, step.q),)
        elif isinstance(step, ContactStep):
            postures = tuple((step.side, q) for q in step.path)
        else:
            continue
        for side, q in postures:
            for lim in limits_of(side):
                margin = lim.margin_to_limit_deg(q)
                key = (side, lim.name)
                if key not in closest or margin < closest[key][0]:
                    closest[key] = (margin, np.asarray(q, dtype=float), lim)
    notes = []
    for (side, name), (margin, q, lim) in sorted(closest.items(),
                                                 key=lambda kv: kv[0]):
        if margin >= NEAR_LIMIT_DEG:
            continue
        j6 = float(np.degrees(q[lim.driver_joint - 1]))
        j7 = float(np.degrees(q[lim.driven_joint - 1]))
        notes.append(
            f"near the coupled {name} limit: {side} J{lim.driven_joint}="
            f"{j7:+.1f} deg at J{lim.driver_joint}={j6:+.1f} deg is "
            f"{margin:.1f} deg inside +/-{lim.limit_deg(j6):.1f} deg "
            f"({lim.describe_source(j6)})")
    return tuple(notes)


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
    check = getattr(kin.kin, "posture_violation", None)
    why = check(side, q_goal) if check is not None else None
    if why is not None:
        # a joint-space goal did not come through the IK, so the posture
        # check the IK makes on every solution is made here
        return steps, PlanError(
            JOINT_LIMIT, f"the {side} arm cannot hold {label}: {why}",
            waypoint_label=label, primitive=primitive, side=side,
            stage="joint_ramp")
    for _ in range(MAX_KNOTS_PER_WAYPOINT):
        q_now = kin.joints(side)
        if float(np.max(np.abs(q_goal - q_now))) <= 1e-6:
            return steps, None
        q_next, _ = clamp_joint_step(q_now, q_goal, limit)
        if kin.kin.gate.installed and not kin.kin.guard_ok(side, q_next):
            return steps, PlanError(
                GUARD_REJECT,
                f"the motion guard refused the {side} arm on the way to "
                f"{label}, {np.degrees(np.max(np.abs(q_goal - q_now))):.1f} "
                f"deg from it",
                waypoint_label=label, primitive=primitive, side=side,
                stage="joint_ramp",
                # RADIANS go in residual_rad. They used to go in residual_m
                # and be printed as millimetres.
                residual_rad=float(np.max(np.abs(q_goal - q_now))))
        if kin.scene is not None:
            error = _scene_check(kin, side, q_now, q_next,
                                 Waypoint(label, *kin.tool_pose(side)),
                                 primitive=primitive, index=-1)
            if error is not None:
                return steps, error
        kin.kin.set_joints(side, q_next)
        steps.append(JointStep(side, q_next, 0))
    return steps, PlanError(
        UNREACHABLE_OBJECT,
        f"the {side} arm did not converge on {label} within the step budget",
        waypoint_label=label, primitive=primitive, side=side,
        stage="joint_ramp")


#: Two consecutive contact-leg knots closer than this on every joint [rad]
#: are one knot: the solver re-solved without moving.
DUPLICATE_KNOT_RAD = 1e-9


def leg_knots(kin: Kin, side: str, q_start, leg_steps, d
              ) -> Tuple[List[np.ndarray], List[float]]:
    """``(path, s)`` of a contact leg: its joint knots from ``q_start`` and
    each knot's distance along the base-frame travel ``d`` [m], measured by
    forward kinematics on the SOLVED posture (the leg's own geometry, not its
    ideal). Shared by every verb that ends in a
    :class:`~.types.ContactStep` (``probe``, ``press``, a fingertip ``grasp``).

    A knot the solver did not move (it re-solved a posture pinned at a limit:
    d1-2 2026-09-23, J5 held at 173 deg for 35 of 41 knots) is the same
    sample twice. It is dropped HERE, where it is made: kept, it has the same
    distance as the one before it, hence the same time, and the daemon
    refuses the whole leg ("increasing times"). ``kin`` is a borrowed
    :class:`Kin`; the side's joints are left at the last knot (the caller's
    ``with`` block restores them).
    """
    d = np.asarray(d, dtype=float).reshape(3)
    path = [np.asarray(q_start, dtype=float)]
    for s in leg_steps:
        q = np.asarray(s.q, dtype=float)
        if np.max(np.abs(q - path[-1])) > DUPLICATE_KNOT_RAD:
            path.append(q)
    kin.kin.set_joints(side, path[0])
    p0 = kin.tool_pose(side)[0]
    dist = [0.0]
    for q in path[1:]:
        kin.kin.set_joints(side, q)
        along = float(np.dot(kin.tool_pose(side)[0] - p0, d))
        dist.append(max(dist[-1], along))
    return path, dist
