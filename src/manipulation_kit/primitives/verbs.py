"""The nine kinematic verbs, plus the Pour contract a learned executor fills.

Each is a frozen dataclass whose fields are exactly what a model binds: names,
one of a handful of enumerated choices, a :class:`~manipulation_kit.world.Direction`
(which way the hand travels), and — in ``Nudge`` alone — correction numbers.
The orientation is never a field; :func:`.orientation.align_tool` derives it,
and WHERE the hand meets the object (grasp point, descent floor, standoff, the
one roll sweep, the fit) is :mod:`.grasp_geometry`'s. No verb has a roll
field: ``Approach`` and ``Grasp`` try :func:`.grasp_geometry.roll_candidates`
in order and the plan's notes say which roll was used.

Reading order, because they compose: ``Approach`` stands off, ``Grasp``
descends and closes, ``Lift`` raises the object, ``Carry`` takes it over the
destination, ``Place`` lowers it in, ``Release`` opens. ``Nudge`` is the
correction, ``Retreat`` backs out, ``GoHome`` resets. ``Pour`` is the one whose
body is a policy. ``Handover`` passes a held object to the other hand — the
chain of the others over two sides.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, List, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from ..world import (UPRIGHT_TOL_RAD as _UPRIGHT_TOL_RAD, BASE, ContainerView,
                     FrameError, ObjectView, SurfaceView, WorldView)
from ..world.direction import ALIASES, Direction, frame_rotation, parse_direction
from . import grasp_geometry as gg
from . import orientation as ap
from . import verifiers as V
from .arguments import check_arguments
from .clearance import SceneGate, policy_of
from .planning import (IncompleteObservation, Kin, coupled_limit_notes,
                       joint_ramp, leg_knots, solve_path,
                       CONTACT_KNOT_M)
from .types import (ALREADY_HOLDING, ARM_UNKNOWN, AUTO, BAD_SIDE, BOTH,
                    FRAME_STALE, GOHOME_SIDE_CHOICES, GRIPPER_UNKNOWN, INCOMPLETE_OBSERVATION, LearnedPrimitive,
                    LEARNED_POLICY_REQUIRED, NO_FIT, NO_MOTION, NO_SUCH_OBJECT,
                    NOT_HOLDING, NUDGE_GRID_M, NUDGE_MAX_YAW_RAD, OBJECT_TILTED,
                    OBJECT_TOO_FLAT, OBJECT_TOO_WIDE, PlanBinding, PlanError,
                    Plan, Primitive, SIDES, BAD_ARGUMENT,
                    UNREACHABLE_DESTINATION, UNREACHABLE_HANDOVER,
                    UNKNOWN_FRAME, SIDE_CHOICES,
                    UNSUPPORTED_GEOMETRY, Unmet,
                    Verifier, ContactCriterion, ContactPolicy, ContactStep,
                    GripStep, JointStep, SettleStep, VERB_CONTACT_POLICY,
                    Waypoint)

#: default gap between the finger tips and the object's silhouette at the
#: standoff [m] (:func:`.grasp_geometry.standoff_point`)
DEFAULT_STANDOFF_M = gg.DEFAULT_STANDOFF_M
#: how far above a destination a carried object travels [m] — the LARGEST
#: rung of :data:`CARRY_CLEARANCE_LADDER_M`, not a fixed height
DEFAULT_CLEARANCE_M = 0.10
#: Transit clearances above a destination's rim a carry will TRY, largest
#: first. The transit height is CHOSEN FROM THE REACHABLE SET rather than
#: fixed, because a constant is a statement about the robot's arm that nobody
#: measured: on the ``blocks-eval`` wagon (2026-09-19) the bin rim plus a
#: constant 100 mm lands **109 mm outside** the holding arm's reachable set,
#: and every trial that got as far as a lift then lost both ``Carry`` and
#: ``Place`` to ``ik_fail`` at ``over_destination``. The rungs step down 20 mm
#: at a time and the ladder STOPS at :data:`RIM_MARGIN_M` above what the
#: carried object itself needs — the clearance exists to clear the rim, so it
#: is floored by the rim and the object, never shrunk until the IK stops
#: complaining. A destination no rung reaches is refused with
#: :data:`~.types.UNREACHABLE_DESTINATION`, which is the caller's cue to use
#: the other arm rather than the kit's cue to try harder.
CARRY_CLEARANCE_LADDER_M: Tuple[float, ...] = (0.10, 0.08, 0.06, 0.04)
#: the carried object's own UNDERSIDE must stay this far above the rim [m]
RIM_MARGIN_M = 0.010
#: how far above the RIM a placed object is let go when the set-down inside
#: the container cannot be reached [m]
PLACE_RELEASE_MARGIN_M = 0.010
#: default lift [m]
DEFAULT_LIFT_M = 0.10
#: how close under a held object something has to be to catch it on release [m]
RELEASE_SUPPORT_M = 0.030


# --------------------------------------------------------------------------- #
# shared precondition helpers
# --------------------------------------------------------------------------- #

#: How long a settle may take before the executor gives up on it [s].
SETTLE_S = 1.5


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
    """The hand this verb will actually use — resolved ONCE, before checking.

    ``side="auto"`` used to stay unresolved through ``preconditions``, so the
    occupancy check was skipped for exactly the calls that most needed it:
    ``Grasp(object=...)`` with no side could pick a hand already holding
    something, and its first step opened that hand (R2). Everything
    downstream — the plan, the notes, the verifier — takes the value this
    returns.
    """
    if want != AUTO:
        return want
    free = [s for s in SIDES if _occupancy(world, s) == "free"]
    return ap.choose_side(p_base, available=tuple(free) if free else SIDES)


def _occupancy(world: WorldView, side: str) -> str:
    """``"free"`` / ``"holding"`` / ``"unknown"`` — three states, not two.

    ``_holding`` used to map "no gripper report" onto "not holding", so an
    absent gripper read as an empty hand and passed a free-hand check. A hand
    nobody can see is not an empty hand.
    """
    gripper = world.gripper(side)
    if gripper is None:
        return "unknown"
    return "holding" if gripper.holding else "free"


def _holding(world: WorldView, side: str) -> Optional[str]:
    gripper = world.gripper(side)
    if gripper is None or not gripper.holding:
        return None
    return gripper.held_object or "something"


def _must_hold(world: WorldView, side: str, name: str) -> List[Unmet]:
    gripper = world.gripper(side)
    if gripper is None:
        return [Unmet(GRIPPER_UNKNOWN,
                      f"the {side} gripper reports nothing, so whether it holds "
                      f"{name!r} is unknown", "read the gripper state first")]
    if not gripper.holding:
        return [Unmet(NOT_HOLDING,
                      f"the {side} gripper is not holding anything",
                      f"grasp {name} first")]
    if gripper.held_object and gripper.held_object != name:
        return [Unmet(NOT_HOLDING,
                      f"the {side} gripper is holding {gripper.held_object!r}, "
                      f"not {name!r}")]
    if not gripper.held_object:
        # An unidentified hold is NOT a hold of an arbitrary named object.
        return [Unmet(GRIPPER_UNKNOWN,
                      f"the {side} gripper reports a hold but not WHAT it is "
                      f"holding, so this cannot be shown to be {name!r}",
                      "publish GripperView(held_object=...) from the producer")]
    return []


def _must_be_free(world: WorldView, side: str) -> List[Unmet]:
    """This hand is MEASURED empty. Unknown is a refusal, not a default."""
    state = _occupancy(world, side)
    if state == "holding":
        return [Unmet(ALREADY_HOLDING,
                      f"the {side} gripper is already holding "
                      f"{_holding(world, side)}",
                      "place or release it first, or use the other hand")]
    if state == "unknown":
        return [Unmet(GRIPPER_UNKNOWN,
                      f"the {side} gripper reports nothing, so whether it is "
                      f"free is unknown — and the first thing this verb does "
                      f"is open it",
                      "read the gripper state into the world first")]
    return []


def _holder_of(world: WorldView, name: str, want: str) -> Tuple[Optional[str],
                                                                List[Unmet]]:
    """Which hand holds ``name`` — or the typed reason no hand does.

    ``Lift``/``Carry``/``Place``/``Pour`` with the default ``side="auto"`` used
    to skip their hold check when no holder could be found and then ask the
    kinematics for ``tool_pose(None)``, which raises ``ValueError`` out of
    ``plan()`` — an exception for what is an ordinary model mistake (R3).
    """
    if want in SIDES:
        return want, _must_hold(world, want, name)
    holder = world.holder_of(name)
    if holder:
        return holder, []
    occupied = [s for s in SIDES if _occupancy(world, s) == "holding"]
    unknown = [s for s in SIDES if _occupancy(world, s) == "unknown"]
    if len(occupied) == 1:
        side = occupied[0]
        return side, _must_hold(world, side, name)
    if len(occupied) > 1:
        return None, [Unmet(
            NOT_HOLDING,
            f"both hands are holding something and neither reports {name!r}",
            "name the side explicitly, or re-observe the held object")]
    if unknown:
        return None, [Unmet(
            GRIPPER_UNKNOWN,
            f"the {', '.join(unknown)} gripper reports nothing, so whether "
            f"{name!r} is held cannot be established",
            "read the gripper state into the world, or name the side")]
    return None, [Unmet(
        NOT_HOLDING, f"neither hand is holding {name!r}",
        f"grasp {name} first")]


def _incomplete(primitive: Primitive, side: str, exc) -> PlanError:
    return PlanError(INCOMPLETE_OBSERVATION, str(exc),
                     primitive=primitive.name(), side=side,
                     unmet=(Unmet(ARM_UNKNOWN, str(exc),
                                  "publish both arms in the observation"),))


#: The fields that name what a verb acts ON. Those are not obstacles to it —
#: a grasp must reach its object, a place its destination — so the scene gate
#: leaves them out (and whatever a hand holds, which rides the tool).
SCENE_TARGET_FIELDS: Tuple[str, ...] = ("object", "to", "source", "target")


def _scene_for(primitive: Primitive, world: WorldView, kin
               ) -> SceneGate:
    """The scene gate this verb's plan is checked against."""
    names = [getattr(primitive, f, "") for f in SCENE_TARGET_FIELDS]
    return SceneGate.of(world, kin, exclude=[n for n in names if n])


def _unchecked_note(scene) -> Tuple[str, ...]:
    if not scene.unresolved:
        return ()
    return (f"not checked for clearance (frame does not resolve): "
            f"{', '.join(scene.unresolved)}",)


def _solve(primitive: Primitive, world: WorldView, kin, side: str, waypoints):
    """``(steps, error, notes)`` with the model restored and the lock held."""
    scene = _scene_for(primitive, world, kin)
    try:
        with Kin(kin, world, scene=scene) as borrowed:
            steps, error, notes = solve_path(borrowed, side, waypoints,
                                             primitive=primitive.name())
            return steps, error, list(notes) + list(_unchecked_note(scene))
    except IncompleteObservation as exc:
        return [], _incomplete(primitive, side, exc), []


def _plan(primitive: Primitive, world: WorldView, kin, side: str, waypoints,
          steps, notes, *, reference: Optional[str] = None) -> Plan:
    """A checked plan, BOUND to the posture and observation it was checked in
    — and, for a grasp, to WHERE on the hand its waypoints put the contact
    (``reference``: a fingertip plan is not a pad plan)."""
    return Plan(primitive.name(), side, tuple(waypoints), tuple(steps),
                tuple(notes) + coupled_limit_notes(kin, steps),
                binding=PlanBinding.of(world, kin, reference=reference))


def _plan_for(primitive: Primitive, world: WorldView, kin, side: str,
              waypoints, *, notes=(), extra_steps=()):
    """Solve a tool path and wrap the result, with the model always restored."""
    steps, error, detours = _solve(primitive, world, kin, side, waypoints)
    if error is not None:
        return error
    return _plan(primitive, world, kin, side, waypoints,
                 tuple(steps) + tuple(extra_steps),
                 tuple(notes) + tuple(detours))


def support_geometry_known(world: WorldView, name: str, what: str = "object"
                           ) -> List[Unmet]:
    """Is there enough MEASURED geometry under ``name`` to plan against?

    Every support number in this package — the descent floor, the rim
    clearance, the hang below the tool — reads the vertical extent off the
    RESOLVED pose, which is exact for a yawed box and for a tilted one alike
    (``ObjectView.extent_along``). What a tilt takes away is the object's own
    underside as a stand-in for the table: a 30 deg block's lowest corner is
    not where the surface is. So a tilted object is refused only when there
    is ALSO no measured surface under it (:func:`.grasp_geometry.support_of`)
    to give the descent its floor; with one it is planned in its own frame
    and the notes say so (req 4). Above
    :data:`~manipulation_kit.world.UPRIGHT_TOL_RAD` is where that switch
    happens, not a gate.

    A DESTINATION is stricter: a place into a tilted container or onto a
    sloped surface is not modelled by this version, whatever it stands on.
    """
    item = world.find(name)
    if item is None:
        return []
    try:
        tilt = item.tilt_rad(world.frames)
    except FrameError:
        return []
    if isinstance(item, SurfaceView) and not item.level(world.frames):
        return [Unmet(UNSUPPORTED_GEOMETRY,
                      f"{name} is not level and this version places onto level "
                      f"surfaces only")]
    if tilt <= _UPRIGHT_TOL_RAD:
        return []
    if what == "destination":
        return [Unmet(
            OBJECT_TILTED,
            f"{name} is tilted {math.degrees(tilt):.0f} deg off upright and "
            f"this version places into upright containers only",
            "level it, or choose another destination",
            {"tilt_deg": round(math.degrees(tilt), 1)})]
    if gg.support_of(world, name) is not None:
        return []
    return [Unmet(
        OBJECT_TILTED,
        f"{name} is tilted {math.degrees(tilt):.0f} deg off upright and no "
        f"measured surface is under it, so nothing gives the descent a floor "
        f"(a tilted object's own underside is a corner, not the table)",
        "measure the surface it stands on into the world, or level it",
        {"tilt_deg": round(math.degrees(tilt), 1)})]


def _supported_by(world: WorldView, name: str) -> Optional[str]:
    """What is under ``name`` right now, close enough to catch it.

    A surface whose top is within :data:`RELEASE_SUPPORT_M` of the object's
    underside, or a container whose interior already contains it. Anything
    else — including "nothing is published" — is no support, and the honest
    answer for a Release is a refusal rather than a drop.
    """
    obj = world.find(name)
    if obj is None:
        return None
    try:
        under = obj.bottom_z(world.frames)
    except FrameError:
        return None
    for item in world.objects:
        try:
            if isinstance(item, SurfaceView):
                if (item.over(obj.pose_in_base(world.frames)[0], world.frames)
                        and -RELEASE_SUPPORT_M <= under - item.top_z(world.frames)
                        <= RELEASE_SUPPORT_M):
                    return item.name
            elif isinstance(item, ContainerView):
                if (item.contains_object(obj, world.frames, pad_m=0.01)
                        and under - item.floor_z(world.frames)
                        <= RELEASE_SUPPORT_M):
                    return item.name
        except FrameError:
            continue
    return None


def _other_hand_holds(world: WorldView, side: str, name: str) -> Optional[str]:
    """The OTHER hand, when it reports holding ``name`` by name, else ``None``.

    An unidentified hold ("something") never counts: whether the other hand
    has THIS object is exactly the question, and a torque stall cannot say.
    """
    for other in SIDES:
        if other == side:
            continue
        gripper = world.gripper(other)
        if (gripper is not None and gripper.holding and name
                and gripper.held_object == name):
            return other
    return None


def _achieved_tool(kin, world: WorldView, side: str, q):
    """Where the tool point ACTUALLY ends up for a solved joint vector.

    Forward kinematics on the plan's own last step, with the mirror put back.
    An ideal waypoint is what was asked for; this is what was solved.
    """
    try:
        with Kin(kin, world) as borrowed:
            borrowed.kin.set_joints(side, np.asarray(q, dtype=float))
            return borrowed.tool_pose(side)[0]
    except IncompleteObservation:
        return None


def _with_settle(plan: Plan, timeout_s: float = SETTLE_S) -> Plan:
    """Append a settle to a plan that moves. A verifier run on a moving robot
    measures the middle of the motion, which is not a verdict."""
    import dataclasses  # noqa: PLC0415
    if plan.steps and isinstance(plan.steps[-1], SettleStep):
        return plan
    return dataclasses.replace(plan, steps=tuple(plan.steps)
                               + (SettleStep(timeout_s),))


def _hang_below_tool(world: WorldView, name: str, p_tool) -> float:
    """How far a held object's UNDERSIDE hangs below the tool point [m].

    MEASURED off the world rather than assumed, because the tool point is the
    pad CENTRE and a top-down grasp sits it ABOVE the object's centre (F6,
    :func:`~.grasp_geometry.grasp_pose`): "the object's half height" is only the
    right number when the two coincide, and it silently under-states the hang
    by 12 mm on a 40 mm cube. With the tool at the object's centre this
    reduces to exactly ``height / 2``, which is the form the rule is stated in.

    A frame that will not resolve falls back to the half height: a carry whose
    object cannot be located has bigger problems, and the precondition that
    catches them has already run.
    """
    item = world.find(name)
    if item is None:
        return 0.0
    try:
        return float(p_tool[2]) - item.bottom_z(world.frames)
    except FrameError:
        return item.vertical_extent_local() / 2.0


def _clearance_ladder(asked_m: float, floor_m: float) -> Tuple[float, ...]:
    """The transit clearances to try, largest first, floored and de-duplicated.

    The caller's own ``clearance_m`` is the FIRST rung — asking for 150 mm
    still gets 150 mm when it plans — and the ladder below it is the fixed
    descending list. Nothing below ``floor_m`` is ever tried; if the ask is
    itself below the floor, the floor is the single rung, because the floor is
    about the object and the rim and the ask is not.
    """
    rungs = [float(asked_m)] + [c for c in CARRY_CLEARANCE_LADDER_M
                                if c < float(asked_m) - 1e-9]
    out: List[float] = []
    for rung in rungs:
        if rung < float(floor_m) - 1e-9:
            continue
        if not out or abs(rung - out[-1]) > 1e-9:
            out.append(float(rung))
    return tuple(out) if out else (float(floor_m),)


def _first_reachable(primitive: Primitive, world: WorldView, kin, side: str,
                     attempts):
    """Try each ``(clearance, notes, waypoints)`` in order; first that plans wins.

    Returns ``(plan, best_clearance, best_error)`` with ``plan`` ``None`` when
    every attempt was refused. "Best" is the SMALLEST residual, so the number
    handed to the caller is how close this arm ever got rather than how close
    the last thing tried got.

    Deterministic by construction, like the via search it sits on top of: a
    fixed ordered list, each attempt planned by the same ``_plan_for``, and
    ``plan()`` stays pure because ``Kin`` restores the mirror every time.
    """
    best_error = None
    best_clearance = float("nan")
    best_residual = float("inf")
    for clearance, notes, waypoints in attempts:
        result = _plan_for(primitive, world, kin, side, waypoints, notes=notes)
        if getattr(result, "ok", False):
            return result, float(clearance), None
        residual = (float(result.residual_m)
                    if math.isfinite(result.residual_m) else float("inf"))
        if best_error is None or residual < best_residual:
            best_error, best_residual, best_clearance = (result, residual,
                                                         float(clearance))
    return None, best_clearance, best_error


def _unreachable_destination(primitive: Primitive, side: str,
                             ladder: Tuple[float, ...], best_clearance: float,
                             error) -> PlanError:
    """Every rung was refused — report it as ONE reason, with the numbers.

    The min residual and the rung that came closest travel with the refusal,
    because "unreachable" without a number is the refusal a model cannot act
    on. The underlying stage reason (``ik_fail`` / ``guard_reject``) is named
    in the detail rather than returned, so a caller can tell "this arm cannot
    get there at all" apart from "this one waypoint was rejected" without
    replaying the ladder itself.
    """
    to = getattr(primitive, "to", "") or "the destination"
    residual = float(error.residual_m) if error is not None else float("nan")
    gap = (f"{residual * 1000:.0f} mm short" if math.isfinite(residual)
           else "no residual measured")
    stage = (error.waypoint_label if error is not None else "over_destination")
    return PlanError(
        UNREACHABLE_DESTINATION,
        f"no plan was found for the {side} arm under this search: every "
        f"transit height from {ladder[0] * 1000:.0f} down to "
        f"{ladder[-1] * 1000:.0f} mm above {to}'s rim was tried and each was "
        f"refused at {stage!r}; the closest was "
        f"{best_clearance * 1000:.0f} mm, {gap} "
        f"({error.reason if error is not None else 'nothing tried'}). That is "
        f"a statement about this search, not a proof of impossibility — but "
        f"lowering the clearance further would drive the object into the rim, "
        f"so the answers are the other arm or a nearer destination",
        waypoint_index=error.waypoint_index if error is not None else -1,
        waypoint_label=stage,
        residual_m=residual, primitive=primitive.name(), side=side,
        stage="clearance_ladder",
        attempted=tuple(f"{c * 1000:.0f} mm" for c in ladder))


# --------------------------------------------------------------------------- #
# directions
# --------------------------------------------------------------------------- #

#: the default direction of each verb that has one
DOWN = ALIASES["down"]
UP = ALIASES["up"]
#: straight back out along the hand's own approach axis
OUT_ALONG_TOOL = ALIASES["along_tool"].opposite()
#: the least vertical component a LIFT direction must have: a lift rises
LIFT_MIN_RISE = 0.5


def _coerce_direction(primitive: Primitive, name: str = "direction") -> None:
    """Let a Python caller write ``direction="down"`` or ``[0, 0, -1]``.

    A value that does not parse is LEFT AS IT IS, so ``check_arguments``
    reports it as a typed ``bad_argument`` rather than the constructor
    raising.
    """
    value = getattr(primitive, name)
    if isinstance(value, Direction):
        return
    try:
        object.__setattr__(primitive, name, parse_direction(value))
    except (TypeError, ValueError):
        pass


def _resolve(direction: Direction, world: WorldView, side: Optional[str], *,
             tool_r=None) -> Tuple[Optional[np.ndarray], List[Unmet]]:
    """``direction`` in BASE for ``side``, or the typed reason it will not
    resolve. An unknown frame is a refusal, never a silent base."""
    try:
        return direction.resolve(world, side=side, tool_r=tool_r), []
    except FrameError as exc:
        code = FRAME_STALE if exc.reason == FRAME_STALE else UNKNOWN_FRAME
        return None, [Unmet(code, f"direction {direction.label()!r} is "
                                  f"expressed in {exc.frame_id!r}, which does "
                                  f"not resolve: {exc.detail}",
                            "use a base-frame direction, or name a frame "
                            "the world holds")]


def _approach_unmet(direction: Direction, world: WorldView,
                    side: Optional[str]) -> List[Unmet]:
    """Can the hand travel ``direction`` onto an object at all?"""
    d, unmet = _resolve(direction, world, side)
    if unmet or d is None:
        return unmet
    if float(d[2]) > ap.VERTICAL_COS:
        return [Unmet(BAD_ARGUMENT,
                      f"direction {direction.label()!r} travels UP onto the "
                      f"object, i.e. from underneath it, and no hand here "
                      f"reaches that way",
                      "use down or a horizontal direction",
                      {"argument": "direction",
                       "axis_base": [round(float(c), 3) for c in d]})]
    return []


def _roll_note(roll_rad: float) -> Tuple[str, ...]:
    """The planner reports the roll it used, since nobody else sets one."""
    if not roll_rad:
        return ()
    return (f"jaws rolled {math.degrees(roll_rad):+.0f} deg about the approach "
            f"axis (planner choice)",)


# --------------------------------------------------------------------------- #
# the geometry Approach and Grasp share
# --------------------------------------------------------------------------- #

@dataclass(frozen=True, eq=False)
class _Meet:
    """Where and how the hand meets one object, for one side — computed ONCE
    and shared by ``Approach`` and ``Grasp``, so the standoff the first stands
    at is exactly the one the second descends from."""

    item: ObjectView
    side: str
    #: the direction as asked (or the object's own face, see below)
    direction: Direction
    #: base-frame, roll 0 — the input every grasp_geometry call takes
    spec: "gg.GraspSpec"
    support: Optional[SurfaceView]
    p_grasp: np.ndarray
    p_stand: np.ndarray
    #: :func:`.grasp_geometry.roll_candidates`, best first
    rolls: Tuple[float, ...]
    #: the hand's MEASURED opening, when the world carries one
    open_gap_m: Optional[float]
    notes: Tuple[str, ...]
    #: the frames ``item`` resolves through
    frames: Any

    @property
    def d(self) -> np.ndarray:
        return self.spec.direction.vector()

    def r_tcp(self, roll_rad: float) -> R:
        return ap.grasp_orientation(self.side, self.d, self.item,
                                    self.frames, roll_rad=roll_rad)


def _own_frame(direction: Direction, item: ObjectView, world: WorldView
               ) -> Tuple[Direction, Tuple[str, ...]]:
    """A DESCENT onto an object tilted past ``UPRIGHT_TOL_RAD`` is taken along
    its own top face's normal, ``object:<name>``, and the notes say so.

    ``down`` on a 30 deg block would put the jaws square to nothing; the face
    normal puts the pads flat on two faces, and the roll then follows the
    object's footprint (req 4). An explicit direction is used as given.
    """
    if direction != DOWN:
        return direction, ()
    try:
        if not gg.tilted(item, world.frames):
            return direction, ()
        tilt = item.tilt_rad(world.frames)
        own = gg.own_face_direction(item, world.frames)
    except FrameError:
        return direction, ()
    return own, (f"{item.name} is tilted {math.degrees(tilt):.0f} deg off "
                 f"upright: the descent is taken along its own top face's "
                 f"normal ({own.frame}) and the jaws are squared to its "
                 f"footprint",)


def _meet(world: WorldView, name: str, side_arg: str, direction: Direction,
          contact: str, standoff_m: float, *, droop_margin_m: float = 0.0,
          clearance_m: Optional[float] = None
          ) -> Tuple[Optional[_Meet], List[Unmet]]:
    """The shared geometry, or the typed reason it cannot be computed.

    ``droop_margin_m`` (:class:`~.clearance.ClearancePolicy`) raises the
    fingertip floor of a descent by how far the real arm sags below the
    commanded pose; 0 for the rigid model and for every verifier."""
    item, p, _r, unmet = _locate(world, name)
    if unmet:
        return None, unmet
    side = _resolved_side(side_arg, world, p)
    direction, notes = _own_frame(direction, item, world)
    d, unmet = _resolve(direction, world, side)
    if unmet or d is None:
        return None, unmet
    if float(d[2]) > ap.VERTICAL_COS:
        return None, _approach_unmet(direction, world, side)
    reference = gg.REFERENCES.get(contact, gg.PAD)
    spec = gg.GraspSpec(Direction(tuple(float(c) for c in d), BASE,
                                  via=direction.label()),
                        reference, 0.0, float(standoff_m))
    try:
        support = gg.support_of(world, name)
        p_grasp, _r_tcp, grasp_notes = gg.grasp_pose(
            item, world.frames, spec, side=side, support=support,
            droop_margin_m=droop_margin_m, clearance_m=clearance_m)
        p_stand = gg.standoff_point(item, world.frames, spec, p_grasp)
        hand = world.gripper(side)
        opening = getattr(hand, "open_gap_m", None)
        rolls = gg.roll_candidates(item, world.frames, spec, open_gap_m=opening)
    except FrameError as exc:
        code = FRAME_STALE if exc.reason == FRAME_STALE else UNKNOWN_FRAME
        return None, [Unmet(code, f"{name}: {exc.detail}")]
    return _Meet(item, side, direction, spec, support, p_grasp, p_stand, rolls,
                 opening, tuple(notes) + tuple(grasp_notes),
                 world.frames), []


def _rolls_in_order(meet: _Meet, world: WorldView) -> Tuple[float, ...]:
    """``meet.rolls``, with the one the wrist ALREADY holds at the standoff
    moved first — a Grasp after an Approach continues the posture the
    Approach chose instead of re-choosing from the top of the list."""
    arm = world.arm(meet.side)
    if (arm is None or arm.tool_r is None or arm.tool_p is None
            or len(meet.rolls) < 2
            or np.linalg.norm(np.asarray(arm.tool_p) - meet.p_stand) > 0.02):
        return meet.rolls
    held = [r for r in meet.rolls
            if _facing_error(arm.tool_r, meet.r_tcp(r)) <= V.FACING_TOL_RAD]
    return tuple(held) + tuple(r for r in meet.rolls if r not in held)


def _facing_error(measured: R, target: R) -> float:
    """Angle between two tool orientations, a jaw half turn not counted."""
    return min(float(np.linalg.norm((measured.inv() * (target * flip)).as_rotvec()))
               for flip in V._JAW_SYMMETRY)


class _AtTheRollTaken(Verifier):
    """The verifier of whichever candidate roll the arm actually ENDED at.

    The roll is chosen at planning time from :func:`.grasp_geometry.
    roll_candidates`, which a verifier built from the world before the verb
    cannot know; the candidates are quarter turns apart and the facing
    tolerance is 5 deg, so the measured wrist names its candidate
    unambiguously and that candidate's checks decide.
    """

    def __init__(self, primitive: str, world0: WorldView, side: str,
                 options):
        super().__init__(primitive, world0)
        self.side = side
        self.options = tuple(options)
        self.describes = (self.options[0][1].describes
                          + " (at the candidate roll the planner took)")

    def measure(self, world1: WorldView):
        return self.measure_run(world1, None)

    def measure_run(self, world1: WorldView, run: Any = None):
        arm = world1.arm(self.side)
        if arm is None or arm.tool_r is None:
            return self.options[0][1](world1, run)
        _r, chosen = min(self.options,
                         key=lambda o: _facing_error(arm.tool_r, o[0]))
        return chosen(world1, run)


def _by_roll(primitive: str, world0: WorldView, meet: _Meet, build) -> Verifier:
    options = [(meet.r_tcp(r), build(meet.r_tcp(r))) for r in meet.rolls]
    if len(options) == 1:
        return options[0][1]
    return _AtTheRollTaken(primitive, world0, meet.side, options)


def _first_roll_that_plans(primitive: Primitive, world: WorldView, kin,
                           meet: _Meet, waypoints_for, check=None, solve=None):
    """Try each roll in order; the first whose path solves (and passes
    ``check``) wins. ``(roll, waypoints, steps, detours, extra_notes)`` or
    ``(None, error)`` — the SQUARED roll's refusal when every one failed,
    with the rolls tried in ``attempted``."""
    import dataclasses  # noqa: PLC0415
    first_error = None
    others: List[str] = []
    for roll in _rolls_in_order(meet, world):
        waypoints = waypoints_for(meet.r_tcp(roll))
        steps, error, detours = (solve or _solve)(primitive, world, kin,
                                                  meet.side, waypoints)
        extra: Tuple[str, ...] = ()
        if error is None and check is not None:
            error, extra = check(steps, meet.r_tcp(roll))
        if error is None:
            return roll, waypoints, steps, detours, extra
        if first_error is None:
            first_error = error
        else:
            others += [a for a in (error.attempted or ())
                       if str(a).startswith("obstacle:")]
    if len(meet.rolls) > 1:
        # keep what the failure itself attempted (the scene gate names the
        # obstacle there), add the rolls that were tried, and any obstacle
        # that refused ANOTHER roll — an ik_fail on the squared roll must not
        # hide that the quarter turn was refused by the shelf
        own = tuple(first_error.attempted or ())
        first_error = dataclasses.replace(
            first_error, attempted=own
            + tuple(f"roll {math.degrees(r):+.0f} deg" for r in meet.rolls)
            + tuple(dict.fromkeys(o for o in others if o not in own)))
    return None, first_error


def _meet_notes(primitive: Primitive, meet: _Meet, roll: float) -> Tuple[str, ...]:
    notes: Tuple[str, ...] = ()
    if getattr(primitive, "side", AUTO) == AUTO:
        notes += (f"side chosen automatically: {meet.side}",)
    return notes + meet.notes + _roll_note(roll)


# --------------------------------------------------------------------------- #
# Approach
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Approach(Primitive):
    """Stand the open hand off an object, ready to travel along a direction.

    The pose is the one ``Grasp`` descends from: the finger tips
    ``standoff_m`` short of the object's silhouette along ``direction``
    (:func:`.grasp_geometry.standoff_point`), oriented so the tool's approach
    axis points along ``direction`` and the jaws are square to the object —
    trying :func:`.grasp_geometry.roll_candidates` in order. Nothing is
    grasped; this is the move that makes the next one a straight line.
    """

    VERB = "approach"
    DIRECTION_ARRIVES = True
    object: str = ""
    side: str = AUTO
    #: which way the hand will TRAVEL onto the object (default: down onto it)
    direction: Direction = DOWN
    standoff_m: float = DEFAULT_STANDOFF_M
    #: WHERE on the hand the coming grasp makes contact — the standoff is
    #: measured from the same geometry the grasp will use
    contact: str = gg.PAD.name

    def __post_init__(self) -> None:
        _coerce_direction(self)

    def preconditions(self, world: WorldView) -> List[Unmet]:
        unmet = check_arguments(self)
        if unmet:
            return unmet
        item, p, _r, found = _locate(world, self.object)
        unmet += found
        unmet += support_geometry_known(world, self.object)
        if p is not None and not unmet:
            # RESOLVE FIRST, then check the hand this verb will actually use.
            side = self.resolve_side(world)
            unmet += _must_be_free(world, side)
            direction, _notes = _own_frame(self.direction, item, world)
            unmet += _approach_unmet(direction, world, side)
        return unmet

    def resolve_side(self, world: WorldView) -> Optional[str]:
        """The hand this verb binds to, or ``None`` when the object is lost."""
        _item, p, _r, unmet = _locate(world, self.object)
        if unmet or p is None:
            return None if self.side == AUTO else self.side
        return _resolved_side(self.side, world, p)

    def _meet(self, world: WorldView):
        return _meet(world, self.object, self.side, self.direction,
                     self.contact, self.standoff_m)

    def plan(self, world: WorldView, kin) -> Any:
        unmet = self.preconditions(world)
        if unmet:
            return self._unmet_error(unmet, self.resolve_side(world) or "")
        meet, unmet = self._meet(world)
        if unmet:
            return self._unmet_error(unmet, self.resolve_side(world) or "")
        found = _first_roll_that_plans(
            self, world, kin, meet,
            # THE STANDOFF IS THE WHOLE VERB, so the tool is gated there —
            # the same ``arrive`` Grasp puts on the same pose. Without it an
            # Approach had no tool-space barrier at all: on d1-2
            # (2026-09-22) one finished 167 mm and 53 deg off its standoff
            # with ``arrivals: []`` and reported completed.
            lambda r: [Waypoint("standoff", meet.p_stand, r, allow_via=True,
                                arrive=True)])
        if found[0] is None:
            return found[1]
        roll, waypoints, steps, detours, _extra = found
        # The OPEN STROKE IS IN THE PLAN. The docstring promised an open hand
        # and the plan emitted joints only, so "approach" left the jaws
        # wherever the last verb put them and the next Grasp descended with a
        # closed hand (R2). The stroke goes first, before the arm moves, and
        # the executor waits for it to finish.
        all_steps = ((GripStep(meet.side, 0.0, "soft", 0),) + tuple(steps)
                     + (SettleStep(SETTLE_S),))
        return _plan(self, world, kin, meet.side, waypoints, all_steps,
                     _meet_notes(self, meet, roll) + tuple(detours)
                     + ("the jaws are opened before the arm moves, and the "
                        "stroke is waited for",),
                     reference=meet.spec.reference.name)

    def verifier(self, world0: WorldView) -> Verifier:
        meet, unmet = self._meet(world0)
        if unmet:
            return V.Never(self.name(), world0,
                           f"approach cannot be verified: {unmet[0]}")
        # Position AND orientation: the next verb descends along the wrist
        # this one was supposed to establish, so verifying the point alone
        # certifies half of what the step is for (R13).
        return _by_roll(self.name(), world0, meet, lambda r_tcp: V.All(
            self.name(), world0, [
                V.ToolAt(self.name(), world0, meet.side, meet.p_stand),
                V.ToolFacing(self.name(), world0, meet.side, r_tcp),
                V.NotHolding(self.name(), world0, meet.side)]))


# --------------------------------------------------------------------------- #
# Grasp
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Grasp(Primitive):
    """Open, travel along a direction onto the object, close on it.

    The geometry is ``d1-inference``'s ``topdown.grasp_from_above`` generalised
    to any direction and either contact (:mod:`.grasp_geometry`): hover at the
    standoff, straight travel, close. ``contact="tip"`` takes the object
    between the finger TIPS rather than the pad centres — how a card lying
    flat on a table is picked up. What stayed in d1-inference is the part that
    needs a camera — the abnormal-descent gate — and the torque verdict is
    read back here through :class:`~manipulation_kit.world.GripperView`.
    """

    VERB = "grasp"
    DIRECTION_ARRIVES = True
    object: str = ""
    side: str = AUTO
    direction: Direction = DOWN
    standoff_m: float = DEFAULT_STANDOFF_M
    grip: str = "soft"
    #: WHERE on the hand the object is taken: "pad" (the pad centre, default)
    #: or "tip" (the finger tips — flat things off a surface)
    contact: str = gg.PAD.name

    def __post_init__(self) -> None:
        _coerce_direction(self)

    def _approach(self) -> Approach:
        return Approach(object=self.object, side=self.side,
                        direction=self.direction, standoff_m=self.standoff_m,
                        contact=self.contact)

    def preconditions(self, world: WorldView) -> List[Unmet]:
        unmet = check_arguments(self)
        if unmet:
            return unmet
        unmet += self._approach().preconditions(world)
        if any(u.code in (NO_SUCH_OBJECT, FRAME_STALE, UNKNOWN_FRAME,
                          BAD_ARGUMENT) for u in unmet):
            # nothing below can be computed without the object and a
            # direction that resolves
            return unmet
        meet, missing = self._meet(world)
        if missing or meet is None:
            return unmet + [u for u in missing if u not in unmet]
        problems = [gg.fit_problems(meet.item, world.frames,
                                    meet.spec.with_roll(r), meet.r_tcp(r),
                                    open_gap_m=meet.open_gap_m,
                                    support=meet.support)
                    for r in meet.rolls]
        if all(problems):
            # no candidate roll fits: report the SQUARED one, every reason
            unmet += problems[0]
        return unmet

    def _meet(self, world: WorldView, *, droop_margin_m: float = 0.0,
              clearance_m: Optional[float] = None):
        return _meet(world, self.object, self.side, self.direction,
                     self.contact, self.standoff_m,
                     droop_margin_m=droop_margin_m, clearance_m=clearance_m)

    def by_contact(self, world: WorldView) -> bool:
        """Does this grasp finish its descent by contact
        (:func:`.grasp_geometry.descends_by_contact`)?"""
        meet, unmet = self._meet(world)
        if unmet or meet is None:
            return False
        return gg.descends_by_contact(meet.item, world.frames, meet.spec,
                                      meet.support)

    def resolve_side(self, world: WorldView) -> Optional[str]:
        return self._approach().resolve_side(world)

    def plan(self, world: WorldView, kin) -> Any:
        unmet = self.preconditions(world)
        if unmet:
            return self._unmet_error(unmet, self.resolve_side(world) or "")
        # A FINGERTIP DESCENT ONTO A SURFACE FINISHES BY CONTACT: it stops
        # with the tips TIP_SEARCH_START_M over the floor and then searches
        # down (a ContactStep) until they touch it. Every other descent stops
        # at a fixed height, raised by the real arm's sag (F16; the typed
        # replacement of MKIT_SUPPORT_CLEARANCE_M, see primitives.clearance).
        by_contact = self.by_contact(world)
        droop = 0.0 if by_contact else policy_of(kin).droop_margin_m
        meet, unmet = self._meet(
            world, droop_margin_m=droop,
            clearance_m=gg.TIP_SEARCH_START_M if by_contact else None)
        if unmet:
            return self._unmet_error(unmet, self.resolve_side(world) or "")
        side = meet.side
        floor, _ = gg.descent_floor(meet.item, world.frames, meet.support)
        descends = float(meet.d[2]) < -1e-3
        #: the search leg: from the tips' start height to CONTACT_OVERTRAVEL_M
        #: past the modelled floor, along the travel
        search_m = 0.0
        if by_contact:
            r0 = meet.r_tcp(0.0)
            search_m = (gg.achieved_clearance(meet.p_grasp, r0, floor)
                        / max(-float(meet.d[2]), 1e-6)
                        + gg.CONTACT_OVERTRAVEL_M)

        def waypoints_for(r_tcp):
            search = ([Waypoint("contact_limit", meet.p_grasp + meet.d * search_m,
                                r_tcp, allow_via=False, knot_m=CONTACT_KNOT_M,
                                exact=True)]
                      if by_contact else [])
            return [
                # getting to the standoff is free-space transit: a detour is
                # a better answer than a refusal. THE TOOL IS CHECKED THERE,
                # before the descent: an arm that starts the travel from
                # 25 mm off to the side arrives 25 mm off to the side, and the
                # descent is the one leg that may not be re-routed (F17).
                Waypoint("standoff", meet.p_stand, r_tcp, allow_via=True,
                         arrive=True),
                # the descent is NOT. Its straightness along the approach
                # axis is the whole promise of the verb, and a 25 cm
                # clearance hop that ends on the grasp point has left the
                # corridor (R10). The tool is checked here too — this is the
                # pose the jaws close on.
                Waypoint("grasp", meet.p_grasp, r_tcp, allow_via=False,
                         arrive=True)] + search

        def solve_by_contact(primitive, world_, kin_, side_, waypoints):
            # the search leg is MEANT to reach the surface: the first thing
            # its ray meets after the object (the table) is left out of the
            # scene check, like a probe's; the leg's knots go inside the
            # ContactStep, so no runner can play them blind
            scene = SceneGate.for_contact(
                world_, kin_, meet.p_grasp, meet.d, search_m,
                exclude=[n for n in (getattr(primitive, f, "")
                                     for f in SCENE_TARGET_FIELDS) if n])
            try:
                with Kin(kin_, world_, scene=scene) as borrowed:
                    steps, error, notes = solve_path(
                        borrowed, side_, waypoints, primitive=primitive.name())
                    if error is not None:
                        return steps, error, notes
                    pre = [s for s in steps if s.waypoint in (0, 1)]
                    leg = [s for s in steps if s.waypoint == 2]
                    q_start = (pre[-1].q if pre else borrowed.joints(side_))
                    path, dist = leg_knots(borrowed, side_, q_start, leg,
                                           meet.d)
                    # where the SOLVED leg starts the tips, for the back-off
                    borrowed.kin.set_joints(side_, path[0])
                    p_start, r_start = borrowed.tool_pose(side_)
            except IncompleteObservation as exc:
                return [], _incomplete(primitive, side_, exc), []
            if len(path) < 2 or dist[-1] <= 1e-4:
                return [], PlanError(
                    BAD_ARGUMENT, f"the {side_} fingertip search has no length",
                    primitive=primitive.name(), side=side_), []
            # CONTACT POLICY back_off (types.ContactPolicy): after a stop ON
            # the support, retreat before the close — the jaws must not drag
            # the tips across the table (d1-2 2026-09-24, turn 6)
            backoff, backoff_note = gg.surface_backoff(
                meet.item, world_.frames, meet.d, support=meet.support,
                floor_z=floor,
                clearance_m=gg.achieved_clearance(p_start, r_start, floor),
                distance_m=policy_of(kin_).contact_backoff_m)
            contact = ContactStep(
                side_, Direction(tuple(float(c) for c in meet.d), BASE),
                float(search_m), ContactCriterion(
                    joint_torque_nm=gg.TIP_CONTACT_NM),
                waypoint=2, path=tuple(path), s=tuple(dist),
                policy=(VERB_CONTACT_POLICY[primitive.name()]
                        if backoff is not None else ContactPolicy.STAY),
                backoff=backoff)
            measured = ((f"the {scene.contact_target!r} the tips search for "
                         f"is left out of the scene check (the search is "
                         f"meant to touch it)",)
                        if scene.contact_target else ())
            return (pre + [contact], None,
                    list(notes) + list(measured) + list(_unchecked_note(scene))
                    + [backoff_note])

        def achieved(steps, r_tcp):
            # VALIDATE THE ACHIEVED DESCENT, not the ideal waypoint. The path
            # window is 12 mm and the support margin is 3 mm, so a solver that
            # plateaus low lands the finger tips in the table while every
            # waypoint coordinate still reads correct (F5's ten failures in
            # ten). Measured against the SAME floor the grasp point used.
            joint_steps = [s for s in steps if isinstance(s, JointStep)]
            if not descends or not joint_steps:
                return None, ()
            p_tool = _achieved_tool(kin, world, side, joint_steps[-1].q)
            if p_tool is None:
                return None, ()
            clearance = gg.achieved_clearance(p_tool, r_tcp, floor)
            if clearance < ap.MIN_ACHIEVED_CLEARANCE_M:
                return PlanError(
                    UNSUPPORTED_GEOMETRY,
                    f"the {side} arm's SOLVED descent leaves the finger tips "
                    f"{clearance * 1000:+.1f} mm above what {self.object} "
                    f"stands on, under the "
                    f"{ap.MIN_ACHIEVED_CLEARANCE_M * 1000:.0f} mm this plan "
                    f"has to keep. The waypoint asked for "
                    f"{(gg.TIP_SEARCH_START_M if by_contact else ap.SUPPORT_CLEARANCE_M + droop) * 1000:.0f} mm; the "
                    f"IK did not "
                    f"get there, and the fingers would jam on the surface "
                    f"before the jaws close",
                    waypoint_index=1, waypoint_label="grasp",
                    residual_m=float(ap.MIN_ACHIEVED_CLEARANCE_M - clearance),
                    stage="achieved_clearance",
                    primitive=self.name(), side=side), ()
            if by_contact:
                return None, (
                    f"the solved descent stops with the finger tips "
                    f"{clearance * 1000:.1f} mm off the surface, then SEARCHES "
                    f"down by contact: at most {search_m * 1000:.1f} mm "
                    f"({gg.CONTACT_OVERTRAVEL_M * 1000:.0f} mm past the "
                    f"modelled surface), stopping at a "
                    f"{gg.TIP_CONTACT_NM:.1f} Nm joint-torque rise, and the "
                    f"jaws close where the tips touched (no droop margin: "
                    f"the height is measured, not guessed)",)
            return None, (f"the solved descent keeps the pad tips "
                          f"{clearance * 1000:.1f} mm off the surface",)

        found = _first_roll_that_plans(
            self, world, kin, meet, waypoints_for, check=achieved,
            solve=solve_by_contact if by_contact else None)
        if found[0] is None:
            return found[1]
        roll, waypoints, steps, detours, clearance_notes = found
        # The jaws open BEFORE the arm moves and close only once the tool is on
        # the object: an open-on-arrival stroke sweeps the pads through
        # whatever is beside it.
        all_steps = ((GripStep(side, 0.0, self.grip, 0),) + tuple(steps)
                     + (GripStep(side, 1.0, self.grip, 1),
                        SettleStep(SETTLE_S)))
        return _plan(self, world, kin, side, waypoints, all_steps,
                     _meet_notes(self, meet, roll) + tuple(detours)
                     + tuple(clearance_notes),
                     reference=meet.spec.reference.name)

    def verifier(self, world0: WorldView) -> Verifier:
        meet, unmet = self._meet(world0)
        if unmet:
            return V.Never(self.name(), world0,
                           f"grasp cannot be verified: {unmet[0]}")
        # the travel the plan made, so the hold is graded on where the
        # fingers got to and whether the approach finished — not on the jaw
        # gap alone (V.GraspStroke)
        floor, _ = gg.descent_floor(meet.item, world0.frames, meet.support)
        stroke = V.GraspStroke(
            meet.d, meet.item, floor_z=floor,
            by_contact=gg.descends_by_contact(meet.item, world0.frames,
                                              meet.spec, meet.support),
            floor_name=("" if meet.support is None
                        else f"{meet.support.name}'s top"))
        return _by_roll(self.name(), world0, meet, lambda r_tcp: V.Holding(
            self.name(), world0, meet.side, meet.item,
            jaw_axis=ap.jaw_axis(r_tcp), reference=meet.spec.reference,
            stroke=stroke))


# --------------------------------------------------------------------------- #
# Lift
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Lift(Primitive):
    """Raise the held object along a rising direction (default straight up),
    orientation unchanged."""

    VERB = "lift"
    object: str = ""
    side: str = AUTO
    height_m: float = DEFAULT_LIFT_M
    #: which way the object travels; it must RISE (base z >= 0.5 of it)
    direction: Direction = UP

    def __post_init__(self) -> None:
        _coerce_direction(self)

    def preconditions(self, world: WorldView) -> List[Unmet]:
        unmet = check_arguments(self)
        if unmet:
            return unmet
        _item, _p, _r, found = _locate(world, self.object)
        unmet += found
        if unmet:
            return unmet
        # THE HOLD CHECK RUNS EVEN WHEN NO HAND HOLDS ANYTHING. It used to be
        # skipped exactly then, and planning went on to ask the kinematics for
        # ``tool_pose(None)`` (R3).
        side, held = _holder_of(world, self.object, self.side)
        unmet += held
        if not unmet:
            d, bad = _resolve(self.direction, world, side)
            unmet += bad
            if d is not None and float(d[2]) < LIFT_MIN_RISE:
                unmet.append(Unmet(
                    BAD_ARGUMENT,
                    f"a lift rises, and direction {self.direction.label()!r} "
                    f"climbs only {float(d[2]):.2f} of its length",
                    "use carry or nudge to move it sideways",
                    {"argument": "direction",
                     "axis_base": [round(float(c), 3) for c in d]}))
        return unmet

    def _side(self, world: WorldView) -> Optional[str]:
        return _holder_of(world, self.object, self.side)[0]

    def plan(self, world: WorldView, kin) -> Any:
        unmet = self.preconditions(world)
        if unmet:
            return self._unmet_error(unmet, self._side(world) or "")
        side = self._side(world)
        try:
            with Kin(kin, world) as borrowed:
                p_tool, r_tool = borrowed.tool_pose(side)
        except IncompleteObservation as exc:
            return _incomplete(self, side, exc)
        d, _bad = _resolve(self.direction, world, side, tool_r=r_tool)
        goal = p_tool + d * float(self.height_m)
        # STRAIGHT ALONG IT. A lift that routes through a 25 cm clearance point, or
        # through READY, is not a lift of a held object — it is a swing with
        # something in the hand, and the orientation promise goes with it.
        return _plan_for(self, world, kin, side,
                         [Waypoint("lifted", goal, r_tool, allow_via=False)],
                         extra_steps=(SettleStep(SETTLE_S),))

    def verifier(self, world0: WorldView) -> Verifier:
        side = self._side(world0)
        if side is None:
            return V.Never(self.name(), world0,
                           "no hand is holding anything, so no lift can be measured")
        try:
            rise = float(self.direction.resolve(world0, side=side)[2])
        except FrameError as exc:
            return V.Never(self.name(), world0,
                           f"the lift direction does not resolve: {exc}")
        return V.ObjectRose(self.name(), world0, side, self.object,
                            self.height_m * rise)


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
        unmet = check_arguments(self)
        if unmet:
            return unmet
        _item, _p, _r, found = _locate(world, self.object)
        unmet += found
        _dest, _dest_p, _dr, dfound = _locate(world, self.to, "destination")
        unmet += dfound
        unmet += support_geometry_known(world, self.object)
        unmet += support_geometry_known(world, self.to, "destination")
        if unmet:
            return unmet
        _side, held = _holder_of(world, self.object, self.side)
        return unmet + held

    def _side(self, world: WorldView) -> Optional[str]:
        return _holder_of(world, self.object, self.side)[0]

    def _destination_top(self, world: WorldView) -> Tuple[np.ndarray, float]:
        """``(destination centre in base, the z a carried object must clear)``."""
        dest = world.find(self.to)
        dest_p, _ = dest.pose_in_base(world.frames)
        if isinstance(dest, ContainerView):
            top = dest.rim_z(world.frames)
        elif isinstance(dest, SurfaceView):
            top = dest.top_z(world.frames)
        else:
            top = dest.top_face_z(world.frames)
        return np.asarray(dest_p, dtype=float), float(top)

    def ladder(self, world: WorldView, p_tool) -> Tuple[float, ...]:
        """The transit clearances this carry may use, largest first.

        Public because it is the number a caller has to be able to read back:
        a refusal names the rungs it tried, and the chain planner that chooses
        an arm by reachability has to ask the same question the plan asks.
        """
        floor = _hang_below_tool(world, self.object, p_tool) + RIM_MARGIN_M
        return _clearance_ladder(self.clearance_m, floor)

    def plan(self, world: WorldView, kin) -> Any:
        unmet = self.preconditions(world)
        if unmet:
            return self._unmet_error(unmet, self._side(world) or "")
        side = self._side(world)
        try:
            with Kin(kin, world) as borrowed:
                p_tool, r_tool = borrowed.tool_pose(side)
        except IncompleteObservation as exc:
            return _incomplete(self, side, exc)
        dest_p, top = self._destination_top(world)
        ladder = self.ladder(world, p_tool)
        attempts = []
        for clearance in ladder:
            goal = np.array([float(dest_p[0]), float(dest_p[1]),
                             top + float(clearance)])
            # Rise first, then travel: the horizontal leg is what keeps the
            # object over the clearance height for the whole move. When the
            # chosen transit height is BELOW where the tool already is the
            # rise is a no-op and the leg descends — monotonically, to a
            # height that is itself already clear of the rim, so the object
            # never passes under the height it is going to end at.
            rise = np.array([p_tool[0], p_tool[1],
                             max(float(p_tool[2]), float(goal[2]))])
            attempts.append((
                clearance,
                (f"transit {clearance * 1000:.0f} mm above {self.to}'s rim"
                 + ("" if clearance >= ladder[0] - 1e-9 else
                    f" (the {ladder[0] * 1000:.0f} mm rung is out of reach)"),),
                [Waypoint("clearance", rise, r_tool, allow_via=False),
                 Waypoint("over_destination", goal, r_tool, allow_via=False)]))
        plan, best, error = _first_reachable(self, world, kin, side, attempts)
        if plan is not None:
            return _with_settle(plan)
        return _unreachable_destination(self, side, ladder, best, error)

    def verifier(self, world0: WorldView) -> Verifier:
        side = self._side(world0)
        if side is None:
            return V.Never(self.name(), world0,
                           "no hand is holding anything, so no carry can be measured")
        # Horizontal position was the WHOLE of the old verdict, so an object
        # dangling below the rim counted as carried over the bin (R12). The
        # clearance half is measured against the same rim the plan used.
        return V.All(self.name(), world0, [
            V.ObjectOver(self.name(), world0, side, self.object, self.to),
            V.ObjectClears(self.name(), world0, self.object, self.to),
            V.Holding(self.name(), world0, side, world0.find(self.object))])


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
    #: May the object be LET GO above the destination when the arm cannot
    #: reach down to set it down? It used to happen silently — a Place that
    #: could not reach the floor released one centimetre over the rim and
    #: called itself a place (R, section 3: "Place silently falls back to rim
    #: release"). Dropping a thing is an application decision with a height
    #: and a suitability attached, so it is a field, and the plan says which
    #: rung it took.
    allow_drop: bool = False

    def preconditions(self, world: WorldView) -> List[Unmet]:
        unmet = check_arguments(self)
        if unmet:
            return unmet
        # Its OWN clearance, not a freshly built Carry's zero. The old code
        # validated ``Carry(clearance_m=0.0)`` and never looked at
        # ``self.clearance_m``, so a negative or nonfinite one reached
        # ``_drop_pose`` (R14). ``check_arguments`` above is what now does it.
        unmet += Carry(object=self.object, to=self.to, side=self.side,
                       clearance_m=0.0).preconditions(world)
        dest = world.find(self.to)
        if dest is not None and not isinstance(dest, (ContainerView, SurfaceView)):
            unmet.append(Unmet(
                "no_such_target",
                f"{self.to} is a plain object, not a container or a surface",
                "name a container or a surface"))
        obj = world.find(self.object)
        if (obj is not None and isinstance(dest, ContainerView)
                and not any(u.code in (FRAME_STALE, UNKNOWN_FRAME) for u in unmet)):
            try:
                fits = dest.fits_inside(obj, world.frames)
                measured = dest.interior_measured
            except FrameError:
                fits, measured = True, True
            if not fits:
                # THE OBJECT HAS TO FIT. Nothing checked this: a bar wider
                # than the bin planned a descent into it and the verifier
                # then asked only whether its CENTRE was inside (R9/R12).
                unmet.append(Unmet(
                    NO_FIT,
                    f"{self.object} is wider than {self.to}'s interior "
                    f"({[round(float(v) * 1000) for v in dest.interior]} mm)",
                    "place it on a surface, or name a larger container"))
            elif not measured:
                unmet.append(Unmet(
                    NO_FIT,
                    f"{self.to}'s interior is ESTIMATED at 90% of its outside "
                    f"size, not measured, and this placement needs the walls "
                    f"to be where they are said to be",
                    "publish ContainerView(interior=...) from a measurement"))
        return unmet

    def _side(self, world: WorldView) -> Optional[str]:
        return _holder_of(world, self.object, self.side)[0]

    def _drop_pose(self, world: WorldView, *, from_rim: bool = False):
        """Where the object's CENTRE ends up when it is let go.

        Two rungs, and the difference between them is what the descent is for:
        ``set_down`` puts the object's underside on the container's own floor,
        which is what "placed" should mean and is tried first; ``from_rim``
        lets it go one :data:`PLACE_RELEASE_MARGIN_M` above the RIM, which is
        a shallower reach for an arm that cannot get down into the bin. The
        verifier does not change for either — the object still has to be
        inside the interior AABB and at rest.
        """
        obj = world.find(self.object)
        dest = world.find(self.to)
        dest_p, _ = dest.pose_in_base(world.frames)
        if isinstance(dest, ContainerView):
            floor = dest.floor_z(world.frames)
            rim = dest.rim_z(world.frames)
        else:
            floor = rim = dest.top_z(world.frames)
        base = rim if from_rim else floor
        margin = (PLACE_RELEASE_MARGIN_M if from_rim else float(self.clearance_m))
        # The RESOLVED vertical extent: a yawed box is not taller, and a box
        # measured in a table frame is not at the height its local z says.
        z = base + obj.vertical_extent(world.frames) / 2.0 + margin
        return np.array([float(dest_p[0]), float(dest_p[1]), z])

    def plan(self, world: WorldView, kin) -> Any:
        unmet = self.preconditions(world)
        if unmet:
            return self._unmet_error(unmet, self._side(world) or "")
        side = self._side(world)
        try:
            with Kin(kin, world) as borrowed:
                p_tool, r_tool = borrowed.tool_pose(side)
        except IncompleteObservation as exc:
            return _incomplete(self, side, exc)
        # The tool point is at the object's grasp point, so the tool descends
        # to the object's resting centre — not to the container floor.
        offset = np.asarray(p_tool) - world.find(self.object).pose_in_base(
            world.frames)[0]
        carry = Carry(object=self.object, to=self.to, side=side)
        _dest_p, top = carry._destination_top(world)
        ladder = carry.ladder(world, p_tool)
        releases = [("set_down", self._drop_pose(world) + offset)]
        rim_release = self._drop_pose(world, from_rim=True) + offset
        if self.allow_drop and rim_release[2] > releases[0][1][2] + 1e-6:
            releases.append(("rim_release", rim_release))
        attempts = []
        for label, release in releases:
            for clearance in ladder:
                above = np.array([float(release[0]), float(release[1]),
                                  top + float(clearance)])
                if above[2] <= release[2] + 1e-6:
                    continue
                how = ("setting it down on the floor of " + self.to
                       if label == "set_down" else
                       f"DROPPING it {PLACE_RELEASE_MARGIN_M * 1000:.0f} mm "
                       f"above {self.to}'s rim (allow_drop=True), which the "
                       f"arm can reach and the floor of it is not")
                attempts.append((
                    clearance,
                    (f"transit {clearance * 1000:.0f} mm above {self.to}'s "
                     f"rim, {how}",),
                    # RISE BEFORE TRAVEL, then descend. Called directly on an
                    # object still below the rim, the old first waypoint was a
                    # diagonal to a point over the destination, which clips
                    # the rim on the way (R10). The rise leg is a no-op when
                    # the tool is already above the transit height.
                    [Waypoint("clearance",
                              np.array([float(p_tool[0]), float(p_tool[1]),
                                        max(float(p_tool[2]), float(above[2]))]),
                              r_tool, allow_via=False),
                     # the transit pose over the destination is this verb's
                     # standoff, and the set-down is a constrained descent,
                     # like a grasp's: both are checked AT THE TOOL, because
                     # the release happens where the tool ends up and not
                     # where seven angles say it should be (F17).
                     Waypoint("over_destination", above, r_tool,
                              allow_via=False, arrive=True),
                     Waypoint(label, release, r_tool, allow_via=False,
                              arrive=True)]))
        plan, best, error = _first_reachable(self, world, kin, side, attempts)
        if plan is None:
            if not self.allow_drop and rim_release[2] > releases[0][1][2] + 1e-6:
                error = PlanError(
                    UNREACHABLE_DESTINATION,
                    f"the {side} arm cannot reach down to set {self.object} on "
                    f"the floor of {self.to}. Letting go above the rim would "
                    f"drop it {(rim_release[2] - releases[0][1][2]) * 1000:.0f} "
                    f"mm; pass allow_drop=True if that is acceptable for this "
                    f"object",
                    waypoint_label="set_down", stage="clearance_ladder",
                    residual_m=(float(error.residual_m) if error is not None
                                else float("nan")),
                    attempted=tuple(f"{c * 1000:.0f} mm" for c in ladder),
                    primitive=self.name(), side=side)
                return error
            return _unreachable_destination(self, side, ladder, best, error)
        all_steps = (tuple(plan.steps)
                     + (GripStep(side, 0.0, "soft", 1), SettleStep(SETTLE_S)))
        import dataclasses  # noqa: PLC0415
        return dataclasses.replace(plan, steps=all_steps)

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
    """Open the jaws. No arm motion — which is NOT the same as safe.

    "No arm motion makes it safe to offer alone" ignored gravity: opening the
    hand 30 cm over a table drops whatever is in it (R, section 3). So a
    release over nothing is refused unless the caller says it means to drop,
    and the check is measured — the held object's underside against a surface
    or a container floor it is already resting in / just above.
    """

    VERB = "release"
    side: str = AUTO
    #: open the hand even when nothing measurably supports what it holds
    allow_drop: bool = False

    def preconditions(self, world: WorldView) -> List[Unmet]:
        unmet = check_arguments(self)
        if unmet:
            return unmet
        side = self._side(world)
        if side is None:
            unmet.append(Unmet(BAD_SIDE, "no side given and neither hand is "
                                         "measurably holding anything",
                               "name a side"))
            return unmet
        if self.allow_drop:
            return unmet
        held = _holding(world, side)
        if held is None:
            return unmet
        if _other_hand_holds(world, side, held):
            # A HANDOVER, not a drop: the other hand measurably holds the
            # same named object, so that hand is what supports it.
            return unmet
        support = _supported_by(world, held)
        if support is None:
            unmet.append(Unmet(
                "unsupported_release",
                f"nothing in this observation supports {held!r}: opening the "
                f"{side} hand now drops it",
                "place it first, or pass allow_drop=True to drop it "
                "deliberately"))
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
            return self._unmet_error(unmet, self._side(world) or "")
        side = self._side(world)
        return Plan(self.name(), side, (),
                    (GripStep(side, 0.0, "soft", -1), SettleStep(SETTLE_S)),
                    binding=PlanBinding.of(world, kin))

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
    express the 30 mm correction a task needs (a classifier-driven run, 2026-09-19);
    a menu of only 10 mm steps pays three turns for every real move.
    """
    value = float(value_m)
    if not math.isfinite(value):
        # min()/copysign() are perfectly happy with a NaN and hand back a
        # plausible +10 mm. ``Nudge(dx=NaN)`` did exactly that (R14).
        raise ValueError(f"a nudge component must be finite, got {value_m!r}")
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
    ABOUT THE APPROACH AXIS, the one rotation with an obvious visual meaning —
    the same roll :func:`.orientation.align_tool` calls ``roll_rad``, applied
    by the same function (:func:`.orientation.roll_tool`). ``frame`` uses the
    shared frame names of :class:`~manipulation_kit.world.Direction`.

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
    frame: str = "tool"          # tool | base (world.direction.TOOL / BASE)

    def snapped(self) -> Tuple[np.ndarray, float]:
        delta = np.array([snap(self.dx), snap(self.dy), snap(self.dz)])
        dyaw = max(-NUDGE_MAX_YAW_RAD, min(NUDGE_MAX_YAW_RAD, float(self.dyaw)))
        return delta, dyaw

    def preconditions(self, world: WorldView) -> List[Unmet]:
        # Arguments FIRST and nothing after them on failure: ``snapped`` must
        # never see a nonfinite number.
        unmet = check_arguments(self)
        if unmet:
            return unmet
        side = self._side(world)
        if side is None:
            unmet.append(Unmet(BAD_SIDE, "name which hand to nudge"))
            return unmet
        arm = world.arm(side)
        if arm is None:
            unmet.append(Unmet(ARM_UNKNOWN,
                               f"the {side} arm is not in this observation"))
        elif self.frame == "tool" and arm.tool_r is None:
            # A tool-frame correction needs the tool's orientation. Without
            # it the old verifier silently graded the delta as if it were
            # base-frame (R13); refusing to PLAN it is the better half.
            unmet.append(Unmet(
                ARM_UNKNOWN,
                f"the {side} arm reports no tool orientation, so a correction "
                f"in its own frame cannot be expressed",
                "use frame='base', or publish ArmView.tool_r"))
        delta, dyaw = self.snapped()
        if not np.any(delta) and dyaw == 0.0:
            unmet.append(Unmet(
                NO_MOTION,
                f"every component snaps to zero (the grid is "
                f"{[int(g * 1000) for g in NUDGE_GRID_M]} mm) and no yaw was "
                f"asked for",
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
        try:
            with Kin(kin, world) as borrowed:
                p_tool, r_tool = borrowed.tool_pose(side)
        except IncompleteObservation as exc:
            return _incomplete(self, side, exc)
        rot = frame_rotation(self.frame, world, side=side, tool_r=r_tool)
        world_delta = delta if rot is None else rot.apply(delta)
        r_goal = ap.roll_tool(r_tool, dyaw)
        notes = []
        asked = np.array([self.dx, self.dy, self.dz])
        if not np.allclose(asked, delta, atol=1e-9):
            notes.append(f"snapped to the grid: "
                         f"{[round(float(v) * 1000) for v in delta]} mm "
                         f"(asked {[round(float(v) * 1000) for v in asked]} mm)")
        if abs(float(self.dyaw)) > NUDGE_MAX_YAW_RAD:
            notes.append(f"yaw clamped to {math.degrees(dyaw):+.0f} deg "
                         f"(asked {math.degrees(self.dyaw):+.0f})")
        # A BOUNDED CORRECTION STAYS BOUNDED. The default detour search
        # could answer a 10 mm nudge with a 25 cm clearance hop that lands on
        # the same endpoint, which is not the move that was asked for (R10).
        return _plan_for(self, world, kin, side,
                         [Waypoint("nudged", p_tool + world_delta, r_goal,
                                   allow_via=False, exact=True)],
                         notes=notes, extra_steps=(SettleStep(SETTLE_S),))

    def verifier(self, world0: WorldView) -> Verifier:
        side = self._side(world0)
        if side is None:
            return V.Never(self.name(), world0, "no side to verify")
        try:
            delta, dyaw = self.snapped()
        except ValueError as exc:
            return V.Never(self.name(), world0, str(exc))
        arm = world0.arm(side)
        if arm is None or arm.tool_p is None:
            return V.Never(self.name(), world0,
                           f"the {side} arm reports no tool point")
        # the verifier measures the BASE-frame displacement, whichever frame
        # the model expressed it in
        if self.frame == "tool":
            if arm.tool_r is None:
                return V.Never(
                    self.name(), world0,
                    f"the {side} arm reports no tool orientation, so a "
                    f"tool-frame displacement cannot be resolved into the "
                    f"base frame — treating it as base-frame would grade the "
                    f"wrong motion")
            delta = arm.tool_r.apply(delta)
        # Tolerance scales with what was asked (V.moved_tol): a fixed 15 or
        # 20 mm window is wider than the 10 mm bottom of NUDGE_GRID_M, so the
        # finest correction on the menu could not fail. It scored a hand that
        # moved 0 mm as TRUE on the 2026-09-19 agent-eval run.
        parts = [V.ToolMoved(self.name(), world0, side, delta)]
        if dyaw:
            # AND THE TURN. ``Nudge(dyaw=0.2).verifier(w)(w)`` returned TRUE
            # — "the left hand moved 0 mm as asked" — because the rotation
            # was dropped from the verdict entirely (R13).
            axis = None if arm.tool_r is None else arm.tool_r.as_matrix()[:, 2]
            parts.append(V.ToolTurned(self.name(), world0, side, dyaw,
                                      axis=axis))
        return parts[0] if len(parts) == 1 else V.All(self.name(), world0, parts)


# --------------------------------------------------------------------------- #
# Retreat
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Retreat(Primitive):
    """Back the hand straight out — by default along its own approach axis.

    Straight back, not up: a hand inside a container that rises first lifts the
    container with it. ``direction`` defaults to ``-along_tool`` (tool -z), the
    axis that used to be hardcoded; any other direction is a straight move
    along it.
    """

    VERB = "retreat"
    side: str = AUTO
    distance_m: float = 0.10
    direction: Direction = OUT_ALONG_TOOL

    def __post_init__(self) -> None:
        _coerce_direction(self)

    def preconditions(self, world: WorldView) -> List[Unmet]:
        unmet = check_arguments(self)
        if unmet:
            return unmet
        side = self._side(world)
        if side is None or world.arm(side) is None:
            unmet.append(Unmet(ARM_UNKNOWN, "name which hand to retreat"))
        elif self.direction.frame != "tool":
            unmet += _resolve(self.direction, world, side)[1]
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
        try:
            with Kin(kin, world) as borrowed:
                p_tool, r_tool = borrowed.tool_pose(side)
        except IncompleteObservation as exc:
            return _incomplete(self, side, exc)
        d, bad = _resolve(self.direction, world, side, tool_r=r_tool)
        if bad:
            return self._unmet_error(bad, side)
        back = d * float(self.distance_m)
        # STRAIGHT BACK. A hand inside a container that is allowed to detour
        # up and out lifts the container with it (R10).
        return _plan_for(self, world, kin, side,
                         [Waypoint("retreated", p_tool + back, r_tool,
                                   allow_via=False)],
                         extra_steps=(SettleStep(SETTLE_S),))

    def verifier(self, world0: WorldView) -> Verifier:
        side = self._side(world0)
        arm = None if side is None else world0.arm(side)
        if arm is None or arm.tool_p is None or arm.tool_r is None:
            return V.Never(self.name(), world0,
                           "no tool pose reported, so a retreat cannot be measured")
        try:
            back = (self.direction.resolve(world0, side=side, tool_r=arm.tool_r)
                    * float(self.distance_m))
        except FrameError as exc:
            return V.Never(self.name(), world0,
                           f"the retreat direction does not resolve: {exc}")
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
        unmet = check_arguments(self)
        if unmet:
            return unmet
        missing = [s for s in SIDES if world.arm(s) is None]
        if missing:
            # BOTH arms, even for a one-sided GoHome: the guard checks the
            # pair, so a ramp planned without the other arm is checked against
            # whatever the shared model was left at (R8).
            return [Unmet(ARM_UNKNOWN,
                          f"no joints reported for: {', '.join(missing)}; the "
                          f"collision guard checks both arms at once",
                          "publish both arms in the observation")]
        return []

    def plan(self, world: WorldView, kin) -> Any:
        unmet = self.preconditions(world)
        if unmet:
            return self._unmet_error(unmet, self.side)
        steps = []
        try:
            with Kin(kin, world,
                     scene=_scene_for(self, world, kin)) as borrowed:
                for side in self._sides():
                    part, error = joint_ramp(borrowed, side, kin.home(side),
                                             primitive=self.name(), label="HOME")
                    steps += part
                    if error is not None:
                        return error
        except IncompleteObservation as exc:
            return _incomplete(self, self.side, exc)
        return Plan(self.name(), self.side, (),
                    tuple(steps) + (SettleStep(2.0),),
                    binding=PlanBinding.of(world, kin))

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
        unmet = check_arguments(self)
        if unmet:
            return unmet
        _src, _p, _r, found = _locate(world, self.source, "source")
        unmet += found
        _dst, _dp, _dr, dfound = _locate(world, self.target, "target")
        unmet += dfound
        if not self.policy:
            unmet.append(Unmet("no_policy",
                               "pour is a learned verb and names no policy",
                               "pass policy='act:<checkpoint>'"))
        if unmet:
            return unmet
        # The hold check runs unconditionally here too: an empty hand used to
        # skip it and be reported as ``learned_policy_required``, which reads
        # as "ask the policy executor" rather than "nothing is held" (R3).
        _side, held = _holder_of(world, self.source, self.side)
        return unmet + held

    def _side(self, world: WorldView) -> Optional[str]:
        return _holder_of(world, self.source, self.side)[0]

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


# --------------------------------------------------------------------------- #
# Handover
# --------------------------------------------------------------------------- #

#: the least a giving hand may back away after letting go [m]: its finger
#: tips lead the pad centre by ``grasp_geometry.PAD.lead_m`` (29 mm), and a
#: retreat shorter than that plus the tool tolerance leaves them around the
#: object
HANDOVER_MIN_RETREAT_M = 0.05


def _other(side: str) -> str:
    return "right" if side == "left" else "left"


@dataclass(frozen=True)
class Handover(Primitive):
    """Pass the held object from one hand to the other, in free air.

    One plan over BOTH arms (:func:`.reach.plan_handover`): the giving hand
    takes the object to a meeting point both arms reach
    (:data:`.reach.HANDOVER_MEETING_POINTS_M`, first that plans), the
    receiving hand approaches and grasps it travelling ``direction``, the
    giving hand opens and backs out ``clearance_m`` along its own approach
    axis. The receiving close must end MEASURABLY holding before the giving
    hand opens (``GripStep.expect_hold``); otherwise the run stops with both
    hands as they are.
    """

    VERB = "handover"
    #: the receiving hand TRAVELS onto the object along ``direction``
    DIRECTION_ARRIVES = True
    object: str = ""
    #: the hand that holds it now ("auto": whichever one does)
    from_side: str = AUTO
    #: the hand that takes it ("auto": the other one)
    to_side: str = AUTO
    #: which way the RECEIVING hand travels onto the object. The default,
    #: ``left`` (+y), is the right hand reaching across toward a left-hand
    #: giver; from the left hand it is ``right``. A direction that travels
    #: away from the giver is refused rather than planned the long way round.
    direction: Direction = ALIASES["left"]
    #: how far the giving hand backs out after letting go [m]
    clearance_m: float = DEFAULT_CLEARANCE_M

    def __post_init__(self) -> None:
        _coerce_direction(self)

    @classmethod
    def applicable(cls, world: WorldView) -> bool:
        """Only when exactly one hand holds a NAMED object and the other is
        measurably free — the one state in which a handover means anything."""
        holders = [s for s in SIDES if _occupancy(world, s) == "holding"]
        if len(holders) != 1:
            return False
        giver = holders[0]
        return (bool(world.gripper(giver).held_object)
                and _occupancy(world, _other(giver)) == "free")

    @classmethod
    def arg_enums(cls):
        return {"from_side": SIDE_CHOICES, "to_side": SIDE_CHOICES}

    def sides(self, world: WorldView) -> Tuple[Optional[str], Optional[str]]:
        """``(giver, receiver)``, resolved ONCE; ``None`` where it cannot be."""
        giver = (self.from_side if self.from_side in SIDES
                 else _holder_of(world, self.object, AUTO)[0])
        if self.to_side in SIDES:
            receiver: Optional[str] = self.to_side
        else:
            receiver = None if giver is None else _other(giver)
        return giver, receiver

    def resolve_side(self, world: WorldView) -> Optional[str]:
        """The hand this verb CLOSES — the receiver."""
        return self.sides(world)[1]

    def preconditions(self, world: WorldView) -> List[Unmet]:
        unmet = check_arguments(self)
        if unmet:
            return unmet
        _item, _p, _r, found = _locate(world, self.object)
        unmet += found
        if unmet:
            return unmet
        if self.clearance_m < HANDOVER_MIN_RETREAT_M - 1e-12:
            unmet.append(Unmet(
                BAD_ARGUMENT,
                f"clearance_m {self.clearance_m} is how far the giving hand "
                f"backs out after letting go, and under "
                f"{HANDOVER_MIN_RETREAT_M} m its finger tips are still around "
                f"the object", "use 0.05 m or more",
                {"argument": "clearance_m"}))
        giver, held = _holder_of(world, self.object, self.from_side)
        unmet += held
        if giver is None or held:
            return unmet
        receiver = self.to_side if self.to_side in SIDES else _other(giver)
        if receiver == giver:
            unmet.append(Unmet(BAD_SIDE,
                               f"the {giver} hand cannot hand over to itself",
                               "name the other hand as to_side"))
            return unmet
        unmet += _must_be_free(world, receiver)
        d, bad = _resolve(self.direction, world, receiver)
        unmet += bad
        if d is None:
            return unmet
        unmet += _approach_unmet(self.direction, world, receiver)
        toward = np.array([0.0, 1.0 if giver == "left" else -1.0, 0.0])
        g_arm, r_arm = world.arm(giver), world.arm(receiver)
        if (g_arm is not None and r_arm is not None
                and g_arm.tool_p is not None and r_arm.tool_p is not None):
            gap = np.asarray(g_arm.tool_p, float) - np.asarray(r_arm.tool_p, float)
            gap[2] = 0.0
            if np.linalg.norm(gap) > 1e-6:
                toward = gap / np.linalg.norm(gap)
        if float(np.dot(d, toward)) < -1e-6:
            unmet.append(Unmet(
                BAD_ARGUMENT,
                f"direction {self.direction.label()!r} takes the {receiver} "
                f"hand AWAY from the {giver} hand that holds {self.object}",
                f"use {'left' if giver == 'left' else 'right'}: the {receiver} "
                f"hand travels toward the giver",
                {"argument": "direction",
                 "axis_base": [round(float(c), 3) for c in d]}))
        return unmet

    def plan(self, world: WorldView, kin) -> Any:
        from . import reach  # noqa: PLC0415 - reach imports this module
        unmet = self.preconditions(world)
        giver, receiver = self.sides(world)
        if unmet:
            return self._unmet_error(unmet, receiver or "")
        was = world.gripper(giver)
        grip = was.grip if was is not None and was.grip else "soft"
        chain, tried = reach.plan_handover(
            self, world, kin, obj=self.object, giver=giver, receiver=receiver,
            direction=self.direction, retreat_m=float(self.clearance_m),
            grip=grip)
        if chain is None:
            return _unreachable_handover(self, receiver, tried)
        return _compose_handover(self, world, kin, giver, receiver, chain)

    def verifier(self, world0: WorldView) -> Verifier:
        giver, receiver = self.sides(world0)
        item = world0.find(self.object)
        if giver is None or receiver is None or item is None:
            return V.Never(self.name(), world0,
                           "a handover needs one hand holding the object and "
                           "the other to take it")
        return V.All(self.name(), world0, [
            V.Holding(self.name(), world0, receiver, item),
            V.NotHolding(self.name(), world0, giver),
            V.ToolClearOf(self.name(), world0, giver, self.object,
                          HANDOVER_MIN_RETREAT_M)])


def _reindexed(step, offset: int):
    """A sub-plan's step with its waypoint index moved into the whole plan."""
    import dataclasses  # noqa: PLC0415
    index = getattr(step, "waypoint", -1)
    if index is None or index < 0:
        return step
    return dataclasses.replace(step, waypoint=int(index) + offset)


def _compose_handover(primitive: "Handover", world: WorldView, kin,
                      giver: str, receiver: str, chain) -> Plan:
    """The five links as ONE plan: waypoints concatenated, each step's
    waypoint index moved with them, and the receiver's close marked
    ``expect_hold`` so the giver never opens on a receiver holding nothing."""
    import dataclasses  # noqa: PLC0415
    waypoints: List[Waypoint] = []
    steps: List[Any] = []
    notes: List[str] = [
        "meeting at ({:.2f}, {:+.2f}, {:.2f}) m (held object centre, base)"
        .format(*chain.meeting),
        f"{giver} hand gives, {receiver} hand takes"]
    for label, link in zip(chain.labels, chain.links):
        plan = link.result
        offset = len(waypoints)
        waypoints += list(plan.waypoints)
        for step in plan.steps:
            step = _reindexed(step, offset)
            if (label == "grasp" and isinstance(step, GripStep)
                    and step.closedness >= 0.5):
                step = dataclasses.replace(step, expect_hold=True)
            steps.append(step)
        notes += [f"{label}: {n}" for n in plan.notes]
    notes.append(f"the {receiver} hand must report holding before the "
                 f"{giver} hand opens; otherwise the run stops there")
    return Plan(primitive.name(), receiver, tuple(waypoints), tuple(steps),
                tuple(notes), binding=PlanBinding.of(world, kin,
                                                     reference=gg.PAD.name))


def _unreachable_handover(primitive: "Handover", side: str, tried) -> PlanError:
    """Every meeting point was refused — ONE reason, every rung named."""
    best = None
    for chain in tried:
        if chain.skipped or not chain.links:
            continue
        key = (len([l for l in chain.links if l.ok]),
               -(chain.links[-1].result.residual_m
                 if math.isfinite(chain.links[-1].result.residual_m)
                 else float("inf")))
        if best is None or key > best[0]:
            best = (key, chain)
    error = None if best is None else best[1].links[-1].result
    stage = "meeting_ladder"
    return PlanError(
        UNREACHABLE_HANDOVER,
        f"no meeting point plans for both hands under this search: "
        f"{len(tried)} were tried and each was refused — "
        + "; ".join(c.sentence() for c in tried)
        + ". That is a statement about this search: move the object nearer "
          "the middle of the robot, or put it down and pick it up with the "
          "other hand",
        waypoint_label=("" if best is None else best[1].broke_at or ""),
        residual_m=(float("nan") if error is None else error.residual_m),
        primitive=primitive.name(), side=side, stage=stage,
        attempted=tuple(c.sentence() for c in tried))


# The contact verbs live in .contact (they import this module's helpers, so
# they are registered here, after everything they need is defined).
from .contact import Press, Probe  # noqa: E402

#: the verb set, in the order a pick-and-place uses them, then the contact verbs
PRIMITIVES: Tuple[type, ...] = (Approach, Grasp, Lift, Carry, Place, Release,
                                Nudge, Retreat, GoHome, Pour, Probe, Press,
                                Handover)

BY_VERB = {cls.name(): cls for cls in PRIMITIVES}


def by_verb(verb: str) -> type:
    """Resolve a verb name to its dataclass, or raise with the whole vocabulary."""
    try:
        return BY_VERB[verb]
    except KeyError:
        raise ValueError(f"no primitive called {verb!r}; the verbs are "
                         f"{sorted(BY_VERB)}") from None
