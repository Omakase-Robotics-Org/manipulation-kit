"""The nine kinematic verbs, plus the Pour contract a learned executor fills.

Each is a frozen dataclass whose fields are exactly what a model binds: names,
one of a handful of enumerated choices, and — in ``Nudge`` alone — numbers. The
orientation is never a field; :mod:`.approach` derives it.

Reading order, because they compose: ``Approach`` stands off, ``Grasp``
descends and closes, ``Lift`` raises the object, ``Carry`` takes it over the
destination, ``Place`` lowers it in, ``Release`` opens. ``Nudge`` is the
correction, ``Retreat`` backs out, ``GoHome`` resets. ``Pour`` is the one whose
body is a policy.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, List, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from ..world import ContainerView, FrameError, ObjectView, SurfaceView, WorldView
from . import approach as ap
from . import verifiers as V
from .planning import Kin, joint_ramp, solve_path
from .types import (AUTO, BOTH, FRAME_STALE, GOHOME_SIDE_CHOICES, GRIPS,
                    LearnedPrimitive,
                    LEARNED_POLICY_REQUIRED, NO_SUCH_OBJECT, NUDGE_FRAMES,
                    NUDGE_GRID_M, NUDGE_MAX_YAW_RAD, PlanError, Plan, Primitive,
                    SIDE_CHOICES, SIDES, TOP_DOWN, UNKNOWN_FRAME, Unmet,
                    Verifier, GripStep, SettleStep, Waypoint)

#: default standoff along the approach axis [m] — far enough that the descent
#: is a straight line the guard can clear, short enough to stay in reach
DEFAULT_STANDOFF_M = 0.08
#: how far above a destination a carried object travels [m]
DEFAULT_CLEARANCE_M = 0.10
#: default lift [m]
DEFAULT_LIFT_M = 0.10


# --------------------------------------------------------------------------- #
# shared precondition helpers
# --------------------------------------------------------------------------- #

def _check_side(side: str) -> List[Unmet]:
    if side not in SIDE_CHOICES:
        return [Unmet("bad_side", f"{side!r} is not one of {SIDE_CHOICES}")]
    return []


def _locate(world: WorldView, name: str, what: str = "object"
            ) -> Tuple[Optional[ObjectView], Optional[np.ndarray],
                       Optional[R], List[Unmet]]:
    """Find ``name`` and resolve it into the base frame, or say exactly why not."""
    if not name:
        return None, None, None, [Unmet(NO_SUCH_OBJECT, f"no {what} was named")]
    item = world.find(name)
    if item is None:
        known = ", ".join(world.names()) or "nothing is detected"
        return None, None, None, [Unmet(
            NO_SUCH_OBJECT, f"there is no {what} called {name!r}",
            f"the world holds: {known}")]
    try:
        p, r = item.pose_in_base(world.frames)
    except FrameError as exc:
        code = FRAME_STALE if exc.reason == FRAME_STALE else UNKNOWN_FRAME
        return item, None, None, [Unmet(
            code, f"{name} was measured in {item.frame_id!r}: {exc.detail}",
            "re-observe it, or register a fresh frame")]
    return item, p, r, []


def _resolved_side(want: str, world: WorldView, p_base) -> str:
    return ap.choose_side(p_base) if want == AUTO else want


def _holding(world: WorldView, side: str) -> Optional[str]:
    gripper = world.gripper(side)
    if gripper is None or not gripper.holding:
        return None
    return gripper.held_object or "something"


def _must_hold(world: WorldView, side: str, name: str) -> List[Unmet]:
    gripper = world.gripper(side)
    if gripper is None:
        return [Unmet("gripper_unknown",
                      f"the {side} gripper reports nothing, so whether it holds "
                      f"{name} is unknown", "read the gripper state first")]
    if not gripper.holding:
        return [Unmet("not_holding", f"the {side} gripper is not holding anything",
                      f"grasp {name} first")]
    if gripper.held_object and gripper.held_object != name:
        return [Unmet("not_holding",
                      f"the {side} gripper is holding {gripper.held_object}, "
                      f"not {name}")]
    return []


def _must_be_free(world: WorldView, side: str) -> List[Unmet]:
    held = _holding(world, side)
    if held:
        return [Unmet("already_holding",
                      f"the {side} gripper is already holding {held}",
                      "place or release it first")]
    return []


def _plan_for(primitive: Primitive, world: WorldView, kin, side: str,
              waypoints, *, notes=()):
    """Solve a tool path and wrap the result, with the model always restored."""
    with Kin(kin, world) as borrowed:
        steps, error, detours = solve_path(borrowed, side, waypoints,
                                           primitive=primitive.name())
    if error is not None:
        return error
    return Plan(primitive.name(), side, tuple(waypoints), tuple(steps),
                tuple(notes) + tuple(detours))


# --------------------------------------------------------------------------- #
# Approach
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Approach(Primitive):
    """Stand the open hand off an object along a named approach axis.

    The pose is ``object + (-direction) * standoff``, oriented so the tool's
    approach axis points at the object and the jaws are square to it. Nothing
    is grasped; this is the move that makes the next one a straight line.
    """

    VERB = "approach"
    object: str = ""
    side: str = AUTO
    approach: str = TOP_DOWN
    standoff_m: float = DEFAULT_STANDOFF_M

    def preconditions(self, world: WorldView) -> List[Unmet]:
        unmet = _check_side(self.side)
        item, p, _r, found = _locate(world, self.object)
        unmet += found
        if self.approach not in ap.APPROACHES:
            unmet.append(Unmet("bad_approach",
                               f"{self.approach!r} is not one of {ap.APPROACHES}"))
        if not 0.02 <= self.standoff_m <= 0.30:
            unmet.append(Unmet("bad_standoff",
                               f"standoff_m must be 0.02-0.30 m, got "
                               f"{self.standoff_m}"))
        if p is not None and self.side != AUTO:
            unmet += _must_be_free(world, self.side)
        return unmet

    def _geometry(self, world: WorldView):
        item, p, _r, unmet = _locate(world, self.object)
        if unmet:
            return None, None, None, unmet
        side = _resolved_side(self.side, world, p)
        r_tcp = ap.grasp_orientation(side, self.approach, item, world.frames)
        return side, ap.standoff_pose(p, self.approach, self.standoff_m), r_tcp, []

    def plan(self, world: WorldView, kin) -> Any:
        unmet = self.preconditions(world)
        if unmet:
            return self._unmet_error(unmet, "" if self.side == AUTO else self.side)
        side, p_stand, r_tcp, _ = self._geometry(world)
        notes = () if self.side != AUTO else (f"side chosen automatically: {side}",)
        return _plan_for(self, world, kin, side,
                         [Waypoint("standoff", p_stand, r_tcp)], notes=notes)

    def verifier(self, world0: WorldView) -> Verifier:
        side, p_stand, _r, unmet = self._geometry(world0)
        if unmet:
            return V.Never(self.name(), world0,
                           f"approach cannot be verified: {unmet[0]}")
        return V.ToolAt(self.name(), world0, side, p_stand)


# --------------------------------------------------------------------------- #
# Grasp
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Grasp(Primitive):
    """Open, descend along the approach axis, close on the object.

    The geometry is ``d1-inference``'s ``topdown.grasp_from_above`` generalised
    to the approach set: hover, straight travel, close. What stayed in
    d1-inference is the part that needs a camera — the abnormal-descent gate —
    and the torque verdict is read back here through
    :class:`~manipulation_kit.world.GripperView`.
    """

    VERB = "grasp"
    object: str = ""
    side: str = AUTO
    approach: str = TOP_DOWN
    standoff_m: float = DEFAULT_STANDOFF_M
    grip: str = "soft"

    def preconditions(self, world: WorldView) -> List[Unmet]:
        unmet = Approach(object=self.object, side=self.side,
                         approach=self.approach,
                         standoff_m=self.standoff_m).preconditions(world)
        if self.grip not in GRIPS:
            unmet.append(Unmet("bad_grip", f"grip must be one of {GRIPS}, "
                                           f"got {self.grip!r}"))
        item = world.find(self.object)
        if item is not None and not ap.fits_jaws(item):
            unmet.append(Unmet(
                "object_too_wide",
                f"{self.object}'s narrowest side is "
                f"{item.min_horizontal_extent() * 1000:.0f} mm and the driven "
                f"jaws open {ap.JAW_OPEN_M * 1000:.0f} mm",
                "use a different tool, or a different object"))
        return unmet

    def _geometry(self, world: WorldView):
        item, p, _r, unmet = _locate(world, self.object)
        if unmet:
            return None, None, None, None, unmet
        side = _resolved_side(self.side, world, p)
        r_tcp = ap.grasp_orientation(side, self.approach, item, world.frames)
        return side, ap.standoff_pose(p, self.approach, self.standoff_m), p, r_tcp, []

    def plan(self, world: WorldView, kin) -> Any:
        unmet = self.preconditions(world)
        if unmet:
            return self._unmet_error(unmet, "" if self.side == AUTO else self.side)
        side, p_stand, p_grasp, r_tcp, _ = self._geometry(world)
        waypoints = [Waypoint("standoff", p_stand, r_tcp),
                     Waypoint("grasp", p_grasp, r_tcp)]
        # The jaws open BEFORE the arm moves and close only once the tool is on
        # the object: an open-on-arrival stroke sweeps the pads through whatever
        # is beside it.
        with Kin(kin, world) as borrowed:
            steps, error, detours = solve_path(borrowed, side, waypoints,
                                               primitive=self.name())
        if error is not None:
            return error
        all_steps = ((GripStep(side, 0.0, self.grip, 0),) + tuple(steps)
                     + (GripStep(side, 1.0, self.grip, 1), SettleStep(1.0)))
        notes = () if self.side != AUTO else (f"side chosen automatically: {side}",)
        return Plan(self.name(), side, tuple(waypoints), all_steps,
                    notes + tuple(detours))

    def verifier(self, world0: WorldView) -> Verifier:
        side, _stand, _p, _r, unmet = self._geometry(world0)
        if unmet:
            return V.Never(self.name(), world0,
                           f"grasp cannot be verified: {unmet[0]}")
        return V.Holding(self.name(), world0, side, world0.find(self.object))


# --------------------------------------------------------------------------- #
# Lift
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Lift(Primitive):
    """Raise the held object straight up, orientation unchanged."""

    VERB = "lift"
    object: str = ""
    side: str = AUTO
    height_m: float = DEFAULT_LIFT_M

    def preconditions(self, world: WorldView) -> List[Unmet]:
        unmet = _check_side(self.side)
        _item, _p, _r, found = _locate(world, self.object)
        unmet += found
        if not 0.01 <= self.height_m <= 0.40:
            unmet.append(Unmet("bad_height",
                               f"height_m must be 0.01-0.40 m, got {self.height_m}"))
        side = self._side(world)
        if side is not None:
            unmet += _must_hold(world, side, self.object)
        return unmet

    def _side(self, world: WorldView) -> Optional[str]:
        if self.side != AUTO:
            return self.side if self.side in SIDES else None
        holder = world.holder_of(self.object)
        if holder:
            return holder
        for side in SIDES:
            if _holding(world, side):
                return side
        return None

    def plan(self, world: WorldView, kin) -> Any:
        unmet = self.preconditions(world)
        if unmet:
            return self._unmet_error(unmet, self._side(world) or "")
        side = self._side(world)
        with Kin(kin, world) as borrowed:
            p_tool, r_tool = borrowed.tool_pose(side)
        goal = p_tool + np.array([0.0, 0.0, float(self.height_m)])
        return _plan_for(self, world, kin, side,
                         [Waypoint("lifted", goal, r_tool)])

    def verifier(self, world0: WorldView) -> Verifier:
        side = self._side(world0)
        if side is None:
            return V.Never(self.name(), world0,
                           "no hand is holding anything, so no lift can be measured")
        return V.ObjectRose(self.name(), world0, side, self.object, self.height_m)


# --------------------------------------------------------------------------- #
# Carry
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Carry(Primitive):
    """Take the held object over a destination, at a safe clearance above it.

    The travel is horizontal at the carry height, not a straight line to the
    goal: a diagonal descent into a container clips the rim, and the rim is the
    thing the clearance is measured from.
    """

    VERB = "carry"
    object: str = ""
    to: str = ""
    side: str = AUTO
    clearance_m: float = DEFAULT_CLEARANCE_M

    def preconditions(self, world: WorldView) -> List[Unmet]:
        unmet = _check_side(self.side)
        _item, _p, _r, found = _locate(world, self.object)
        unmet += found
        _dest, dest_p, _dr, dfound = _locate(world, self.to, "destination")
        unmet += dfound
        if not 0.0 <= self.clearance_m <= 0.40:
            unmet.append(Unmet("bad_clearance",
                               f"clearance_m must be 0-0.40 m, got {self.clearance_m}"))
        side = self._side(world)
        if side is not None:
            unmet += _must_hold(world, side, self.object)
        return unmet

    def _side(self, world: WorldView) -> Optional[str]:
        return Lift(object=self.object, side=self.side)._side(world)

    def _goal(self, world: WorldView):
        dest = world.find(self.to)
        dest_p, _ = dest.pose_in_base(world.frames)
        if isinstance(dest, ContainerView):
            top = dest.rim_z(world.frames)
        elif isinstance(dest, SurfaceView):
            top = dest.top_z(world.frames)
        else:
            top = float(dest_p[2]) + float(dest.size[2]) / 2.0
        return np.array([float(dest_p[0]), float(dest_p[1]),
                         top + float(self.clearance_m)])

    def plan(self, world: WorldView, kin) -> Any:
        unmet = self.preconditions(world)
        if unmet:
            return self._unmet_error(unmet, self._side(world) or "")
        side = self._side(world)
        with Kin(kin, world) as borrowed:
            p_tool, r_tool = borrowed.tool_pose(side)
        goal = self._goal(world)
        # rise first, then travel: the two-waypoint path is what keeps the
        # object over the clearance height for the whole horizontal move
        rise = np.array([p_tool[0], p_tool[1], max(float(p_tool[2]), float(goal[2]))])
        waypoints = [Waypoint("clearance", rise, r_tool),
                     Waypoint("over_destination", goal, r_tool)]
        return _plan_for(self, world, kin, side, waypoints)

    def verifier(self, world0: WorldView) -> Verifier:
        side = self._side(world0)
        if side is None:
            return V.Never(self.name(), world0,
                           "no hand is holding anything, so no carry can be measured")
        return V.ObjectOver(self.name(), world0, side, self.object, self.to)


# --------------------------------------------------------------------------- #
# Place
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Place(Primitive):
    """Lower the held object into a container / onto a surface and let go.

    The descent stops with the object's own half-height plus ``clearance_m``
    above the destination's floor, so it is set down rather than dropped, and
    the release is part of the plan: "placed" that leaves the object in the
    jaws is not placed.
    """

    VERB = "place"
    object: str = ""
    to: str = ""
    side: str = AUTO
    clearance_m: float = 0.01

    def preconditions(self, world: WorldView) -> List[Unmet]:
        unmet = Carry(object=self.object, to=self.to, side=self.side,
                      clearance_m=0.0).preconditions(world)
        dest = world.find(self.to)
        if dest is not None and not isinstance(dest, (ContainerView, SurfaceView)):
            unmet.append(Unmet(
                "no_such_target",
                f"{self.to} is a plain object, not a container or a surface",
                "name a container or a surface"))
        return unmet

    def _side(self, world: WorldView) -> Optional[str]:
        return Lift(object=self.object, side=self.side)._side(world)

    def _drop_pose(self, world: WorldView):
        obj = world.find(self.object)
        dest = world.find(self.to)
        dest_p, _ = dest.pose_in_base(world.frames)
        if isinstance(dest, ContainerView):
            floor = float(dest_p[2]) - float(dest.interior[2]) / 2.0
        else:
            floor = dest.top_z(world.frames)
        z = floor + obj.height() / 2.0 + float(self.clearance_m)
        return np.array([float(dest_p[0]), float(dest_p[1]), z])

    def plan(self, world: WorldView, kin) -> Any:
        unmet = self.preconditions(world)
        if unmet:
            return self._unmet_error(unmet, self._side(world) or "")
        side = self._side(world)
        with Kin(kin, world) as borrowed:
            p_tool, r_tool = borrowed.tool_pose(side)
        # The tool point is at the object's grasp point, so the tool descends
        # to the object's resting centre — not to the container floor.
        drop = self._drop_pose(world)
        offset = np.asarray(p_tool) - world.find(self.object).pose_in_base(
            world.frames)[0]
        above = drop + offset + np.array([0.0, 0.0, DEFAULT_CLEARANCE_M])
        waypoints = [Waypoint("over_destination", above, r_tool),
                     Waypoint("set_down", drop + offset, r_tool)]
        with Kin(kin, world) as borrowed:
            steps, error, detours = solve_path(borrowed, side, waypoints,
                                               primitive=self.name())
        if error is not None:
            return error
        all_steps = tuple(steps) + (GripStep(side, 0.0, "soft", 1), SettleStep(1.0))
        return Plan(self.name(), side, tuple(waypoints), all_steps,
                    tuple(detours))

    def verifier(self, world0: WorldView) -> Verifier:
        side = self._side(world0)
        if side is None:
            return V.Never(self.name(), world0,
                           "no hand is holding anything, so no place can be measured")
        return V.ObjectIn(self.name(), world0, side, self.object, self.to)


# --------------------------------------------------------------------------- #
# Release
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Release(Primitive):
    """Open the jaws. No arm motion — that is what makes it safe to offer alone."""

    VERB = "release"
    side: str = AUTO

    def preconditions(self, world: WorldView) -> List[Unmet]:
        unmet = _check_side(self.side)
        if self._side(world) is None:
            unmet.append(Unmet("bad_side", "no side given and neither hand is "
                                           "holding anything"))
        return unmet

    def _side(self, world: WorldView) -> Optional[str]:
        if self.side in SIDES:
            return self.side
        for side in SIDES:
            if _holding(world, side):
                return side
        return None

    def plan(self, world: WorldView, kin) -> Any:
        unmet = self.preconditions(world)
        if unmet:
            return self._unmet_error(unmet)
        side = self._side(world)
        return Plan(self.name(), side, (),
                    (GripStep(side, 0.0, "soft", -1), SettleStep(1.0)))

    def verifier(self, world0: WorldView) -> Verifier:
        side = self._side(world0)
        if side is None:
            return V.Never(self.name(), world0, "no side to verify")
        return V.NotHolding(self.name(), world0, side)


# --------------------------------------------------------------------------- #
# Nudge — the ONLY free-numeric verb
# --------------------------------------------------------------------------- #

def snap(value_m: float) -> float:
    """Snap a translation to the +-10/30/50 mm grid, zero below half of 10 mm.

    Coarse AND fine in the same vocabulary. A menu of only 50 mm steps cannot
    express the 30 mm correction a task needs (Raptor's Jev run, 2026-09-19);
    a menu of only 10 mm steps pays three turns for every real move.
    """
    value = float(value_m)
    if abs(value) < NUDGE_GRID_M[0] / 2.0:
        return 0.0
    nearest = min(NUDGE_GRID_M, key=lambda g: abs(abs(value) - g))
    return math.copysign(nearest, value)


@dataclass(frozen=True)
class Nudge(Primitive):
    """A bounded correction: "a little to the right", and nothing more.

    This is the escape hatch that makes the primitive vocabulary usable for a
    vision-driven model — Shu's own reason for wanting 6-DoF. Translation
    survives because it is well conditioned and its feedback is legible.
    Rotation does not: the only turn on offer is ``dyaw``, clamped to +-15 deg
    ABOUT THE APPROACH AXIS, the one rotation with an obvious visual meaning.

    Everything is snapped and clamped on construction-of-the-plan, not
    rejected: a model asking for 23 mm gets 30 mm and is TOLD so in the plan's
    notes, which is far better behaviour than a refusal it has to guess at.
    """

    VERB = "nudge"
    side: str = AUTO
    dx: float = 0.0
    dy: float = 0.0
    dz: float = 0.0
    dyaw: float = 0.0            # radians, about the approach axis
    frame: str = "tool"          # tool | base

    def snapped(self) -> Tuple[np.ndarray, float]:
        delta = np.array([snap(self.dx), snap(self.dy), snap(self.dz)])
        dyaw = max(-NUDGE_MAX_YAW_RAD, min(NUDGE_MAX_YAW_RAD, float(self.dyaw)))
        return delta, dyaw

    def preconditions(self, world: WorldView) -> List[Unmet]:
        unmet = _check_side(self.side)
        if self.frame not in NUDGE_FRAMES:
            unmet.append(Unmet("bad_frame",
                               f"frame must be one of {NUDGE_FRAMES}, "
                               f"got {self.frame!r}"))
        side = self._side(world)
        if side is None:
            unmet.append(Unmet("bad_side", "name which hand to nudge"))
        elif world.arm(side) is None:
            unmet.append(Unmet("arm_unknown",
                               f"the {side} arm is not in this observation"))
        delta, dyaw = self.snapped()
        if not np.any(delta) and dyaw == 0.0:
            unmet.append(Unmet(
                "no_motion",
                f"every component snaps to zero (the grid is "
                f"{[int(g * 1000) for g in NUDGE_GRID_M]} mm)",
                "ask for at least 10 mm, or a yaw"))
        return unmet

    def _side(self, world: WorldView) -> Optional[str]:
        if self.side in SIDES:
            return self.side
        for side in SIDES:
            if _holding(world, side):
                return side
        return SIDES[0] if world.arm(SIDES[0]) is not None else None

    def plan(self, world: WorldView, kin) -> Any:
        unmet = self.preconditions(world)
        if unmet:
            return self._unmet_error(unmet, self._side(world) or "")
        side = self._side(world)
        delta, dyaw = self.snapped()
        with Kin(kin, world) as borrowed:
            p_tool, r_tool = borrowed.tool_pose(side)
        world_delta = r_tool.apply(delta) if self.frame == "tool" else delta
        r_goal = (R.from_rotvec(r_tool.as_matrix()[:, 2] * dyaw) * r_tool
                  if dyaw else r_tool)
        notes = []
        asked = np.array([self.dx, self.dy, self.dz])
        if not np.allclose(asked, delta, atol=1e-9):
            notes.append(f"snapped to the grid: "
                         f"{[round(float(v) * 1000) for v in delta]} mm "
                         f"(asked {[round(float(v) * 1000) for v in asked]} mm)")
        if abs(float(self.dyaw)) > NUDGE_MAX_YAW_RAD:
            notes.append(f"yaw clamped to {math.degrees(dyaw):+.0f} deg "
                         f"(asked {math.degrees(self.dyaw):+.0f})")
        plan = _plan_for(self, world, kin, side,
                         [Waypoint("nudged", p_tool + world_delta, r_goal)],
                         notes=notes)
        return plan

    def verifier(self, world0: WorldView) -> Verifier:
        side = self._side(world0)
        if side is None:
            return V.Never(self.name(), world0, "no side to verify")
        delta, _ = self.snapped()
        arm = world0.arm(side)
        if arm is None or arm.tool_p is None:
            return V.Never(self.name(), world0,
                           f"the {side} arm reports no tool point")
        # the verifier measures the BASE-frame displacement, whichever frame
        # the model expressed it in
        if self.frame == "tool" and arm.tool_r is not None:
            delta = arm.tool_r.apply(delta)
        # Tolerance scales with what was asked (V.moved_tol): a fixed 15 or
        # 20 mm window is wider than the 10 mm bottom of NUDGE_GRID_M, so the
        # finest correction on the menu could not fail. It scored a hand that
        # moved 0 mm as TRUE on the 2026-09-19 agent-eval run.
        return V.ToolMoved(self.name(), world0, side, delta)


# --------------------------------------------------------------------------- #
# Retreat
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Retreat(Primitive):
    """Back the hand straight out along its own approach axis.

    Straight back, not up: a hand inside a container that rises first lifts the
    container with it.
    """

    VERB = "retreat"
    side: str = AUTO
    distance_m: float = 0.10

    def preconditions(self, world: WorldView) -> List[Unmet]:
        unmet = _check_side(self.side)
        side = self._side(world)
        if side is None or world.arm(side) is None:
            unmet.append(Unmet("arm_unknown", "name which hand to retreat"))
        if not 0.01 <= self.distance_m <= 0.40:
            unmet.append(Unmet("bad_distance",
                               f"distance_m must be 0.01-0.40 m, got "
                               f"{self.distance_m}"))
        return unmet

    def _side(self, world: WorldView) -> Optional[str]:
        if self.side in SIDES:
            return self.side
        return SIDES[0] if world.arm(SIDES[0]) is not None else None

    def plan(self, world: WorldView, kin) -> Any:
        unmet = self.preconditions(world)
        if unmet:
            return self._unmet_error(unmet, self._side(world) or "")
        side = self._side(world)
        with Kin(kin, world) as borrowed:
            p_tool, r_tool = borrowed.tool_pose(side)
        back = -r_tool.as_matrix()[:, 2] * float(self.distance_m)
        return _plan_for(self, world, kin, side,
                         [Waypoint("retreated", p_tool + back, r_tool)])

    def verifier(self, world0: WorldView) -> Verifier:
        side = self._side(world0)
        arm = None if side is None else world0.arm(side)
        if arm is None or arm.tool_p is None or arm.tool_r is None:
            return V.Never(self.name(), world0,
                           "no tool pose reported, so a retreat cannot be measured")
        back = -arm.tool_r.as_matrix()[:, 2] * float(self.distance_m)
        # Deliberately TIGHTER than the scaled default (0.4 x 100 mm = 40 mm):
        # backing out of a container is a clearance move, and 30 mm is the
        # clearance that matters.
        return V.ToolMoved(self.name(), world0, side, back, tol_m=0.03)


# --------------------------------------------------------------------------- #
# GoHome
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class GoHome(Primitive):
    """Ramp the arms back to the kit's measured HOME posture.

    Joint space on purpose. HOME is a joint vector the robot demonstrably
    holds; routing it through IK would invent an end-effector goal nobody asked
    for, and could fail to reach a posture that is known good. Every step still
    passes the collision guard.
    """

    VERB = "go_home"
    side: str = BOTH

    @classmethod
    def arg_enums(cls):
        return {"side": GOHOME_SIDE_CHOICES}

    def _sides(self) -> Tuple[str, ...]:
        return SIDES if self.side in (BOTH, AUTO) else (self.side,)

    def preconditions(self, world: WorldView) -> List[Unmet]:
        if self.side not in GOHOME_SIDE_CHOICES:
            return [Unmet("bad_side", f"side must be one of "
                                      f"{GOHOME_SIDE_CHOICES}, got {self.side!r}")]
        missing = [s for s in self._sides() if world.arm(s) is None]
        if missing:
            return [Unmet("arm_unknown",
                          f"no joints reported for: {', '.join(missing)}")]
        return []

    def plan(self, world: WorldView, kin) -> Any:
        unmet = self.preconditions(world)
        if unmet:
            return self._unmet_error(unmet, self.side)
        steps = []
        with Kin(kin, world) as borrowed:
            for side in self._sides():
                part, error = joint_ramp(borrowed, side, kin.home(side),
                                         primitive=self.name(), label="HOME")
                steps += part
                if error is not None:
                    return error
        return Plan(self.name(), self.side, (), tuple(steps) + (SettleStep(2.0),))

    def verifier(self, world0: WorldView) -> Verifier:
        from ..arms.d1.arm.kinematics import load_home
        home = load_home(quiet=True)
        return V.JointsAt(self.name(), world0,
                          {s: home[s] for s in self._sides()})


# --------------------------------------------------------------------------- #
# Pour — the contract, not the body
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Pour(LearnedPrimitive):
    """Tip the held source over the target. THE BODY IS A LEARNED POLICY.

    Shu, 2026-09-19: 「Pour は ACT」. The kit owns the parts a policy does not:
    the preconditions (is the source actually held, is the target there, is the
    frame fresh) and the MEASURED verifier. :meth:`plan` refuses with
    ``learned_policy_required`` — that is not a stub, it is the contract: a
    consumer holding an executor that can run ``policy`` handles that reason by
    running it, and one that cannot reports it as the reason the verb is
    unavailable, which is exactly what a caller needs to hear.

    The executor lives in ``d1-inference``, with the checkpoint. It cannot live
    here (the kit has no model runtime) and it cannot live in ``omakase-core``
    (which must not depend on ``d1-inference``).
    """

    VERB = "pour"
    source: str = ""
    target: str = ""
    side: str = AUTO
    tilt_deg: float = 75.0
    policy: str = "act:pourwithsmallpotjp"

    def preconditions(self, world: WorldView) -> List[Unmet]:
        unmet = _check_side(self.side)
        _src, _p, _r, found = _locate(world, self.source, "source")
        unmet += found
        _dst, _dp, _dr, dfound = _locate(world, self.target, "target")
        unmet += dfound
        if not 15.0 <= self.tilt_deg <= 120.0:
            unmet.append(Unmet("bad_tilt",
                               f"tilt_deg must be 15-120, got {self.tilt_deg}"))
        if not self.policy:
            unmet.append(Unmet("no_policy",
                               "pour is a learned verb and names no policy",
                               "pass policy='act:<checkpoint>'"))
        side = self._side(world)
        if side is not None:
            unmet += _must_hold(world, side, self.source)
        return unmet

    def _side(self, world: WorldView) -> Optional[str]:
        return Lift(object=self.source, side=self.side)._side(world)

    def plan(self, world: WorldView, kin) -> Any:
        unmet = self.preconditions(world)
        if unmet:
            return self._unmet_error(unmet, self._side(world) or "")
        return PlanError(
            LEARNED_POLICY_REQUIRED,
            f"pour is executed by the learned policy {self.policy!r}; the kit "
            f"checks its preconditions and measures its result but does not "
            f"plan its motion",
            primitive=self.name(), side=self._side(world) or "")

    def verifier(self, world0: WorldView) -> Verifier:
        return V.Tilted(self.name(), world0, self.source, self.target,
                        math.radians(self.tilt_deg))


#: the v1 verb set, in the order a pick-and-place uses them
PRIMITIVES: Tuple[type, ...] = (Approach, Grasp, Lift, Carry, Place, Release,
                                Nudge, Retreat, GoHome, Pour)

BY_VERB = {cls.name(): cls for cls in PRIMITIVES}


def by_verb(verb: str) -> type:
    """Resolve a verb name to its dataclass, or raise with the whole vocabulary."""
    try:
        return BY_VERB[verb]
    except KeyError:
        raise ValueError(f"no primitive called {verb!r}; the verbs are "
                         f"{sorted(BY_VERB)}") from None
