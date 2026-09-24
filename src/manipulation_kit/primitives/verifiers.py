"""Measured verdicts. The rule is: never TRUE by default.

Three outcomes, and the third one is the point. ``TRUE`` needs evidence in the
later world. ``FALSE`` is evidence of the opposite. ``UNKNOWN`` is what a
verifier must say when the robot in front of it cannot produce the evidence at
all — an object nobody is detecting any more, a gripper with no report, a pour
on a vessel with no scale under it. Collapsing UNKNOWN into TRUE is how a task
gets marked done because nobody was looking, and collapsing it into FALSE makes
a working robot look broken.

The other rule, tested as ``G6``: a verifier called with the world it was built
from returns ``FALSE`` (or ``UNKNOWN``), never ``TRUE``. An executor that did
nothing must not be able to pass.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional, Sequence

import numpy as np
from scipy.spatial.transform import Rotation as R

from ..world import (STATED, ContainerView, FrameError, ObjectView,
                     SurfaceView, WorldView)
from .orientation import JAW_CLEARANCE_M
from .types import Verdict, VerdictReport, Verifier

#: how close a measured tool point must be to a commanded ABSOLUTE pose to
#: count. An arrival is judged against a place, so one number is right for it.
TOOL_TOL_M = 0.02
#: A DISPLACEMENT is judged against its own size instead. 20 mm is wider than
#: the smallest step ``Nudge`` offers (``NUDGE_GRID_M`` starts at 10 mm), so a
#: 10 mm nudge that moved nothing at all scored TRUE — the correction the fine
#: grid exists for was the one the verifier could not measure (agent-eval run,
#: 2026-09-19). The tolerance is now a fraction of what was asked for, with a
#: floor at the IK's own convergence: below ~3 mm a FALSE would be measuring
#: the solver, not the robot.
MOVED_TOL_FRACTION = 0.4
MIN_MOVED_TOL_M = 0.003
#: fraction of a commanded displacement that must actually happen
MOVED_FRACTION = 0.7
#: a gripper at or below this closedness counts as open
OPEN_CLOSEDNESS = 0.15
#: How far the MEASURED pad gap may sit either side of the object's own width
#: and still be that object between the pads. It is the planner's own clearance
#: (``orientation.JAW_CLEARANCE_M``, 4 mm) rather than a second number: the kit
#: refuses to plan a grasp that does not leave this much per side, so a gap
#: outside the window is either the jaws closed on something else or on
#: themselves. MEASURED, 2026-09-19: a 40 mm cube stops the sim's jaws at a
#: 41.2 mm face gap (1.15 mm of PhysX contact band) while carrying the cube
#: through 149.8 mm of lift.
GRIP_WIDTH_TOL_M = JAW_CLEARANCE_M
#: Base of a lifted object within this of a surface top = still standing on it.
RESTING_TOL_M = 0.003
#: How far a measured tool orientation may be from the commanded one [rad].
#: 5 degrees: wider than the IK's own convergence, tight enough that a wrist
#: 90 degrees out fails.
FACING_TOL_RAD = math.radians(5.0)
#: ...and the same for a commanded TURN. A yaw nudge is bounded at 15 degrees,
#: so the window is a fraction of the ask with a floor, exactly as
#: :func:`moved_tol` is for a translation.
TURN_TOL_FRACTION = 0.4
MIN_TURN_TOL_RAD = math.radians(2.0)
#: How near the pads a named object has to be to count as the thing between
#: them, when the producer does not report ``held_object`` [m]. Generous: it
#: is a corroboration, not a grasp-quality metric.
ASSOCIATION_TOL_M = 0.08
#: How far ABOVE the descent floor a grasp's fingertip search may stop by
#: contact and still have reached the surface it was sent to [m]. The tip
#: height is forward kinematics from the joint encoders, and the real arm's
#: sag is not in it (F16: ~10 mm at x 0.48), so a contact on the table can
#: read up to that much high. A contact higher than this stopped on something
#: before the surface — for a grasp, usually the object's own top.
SEARCH_STOP_TOL_M = 0.010

#: :class:`Holding`'s per-criterion outcomes (``measured["checks"]``)
CHECK_PASS, CHECK_FAIL, CHECK_UNMEASURED = "pass", "fail", "unmeasured"


class GraspStroke:
    """What a grasp's travel was planned to do, for :class:`Holding` to hold
    it to — the evidence a jaw gap cannot give.

    A stall inside the width window says the pads stopped on something the
    object's size could make. It does not say the fingers were IN the object:
    jaws closing on a tape roll's rim, or on its top edge, stall at a
    plausible width too. So a grasp's hold is also graded on

    * **insertion** — the finger TIPS at closure against the object's near
      face along the travel (its top, for a descent), from the world the
      grasp was planned in (:func:`.grasp_geometry.insertion_along`);
    * **approach** — whether the travel got where it was sent: the
      executor's arrival at the ``grasp`` waypoint and its stop reason, and
      for a fingertip descent that finishes by contact, whether the search
      stopped on the SURFACE or on something above it.

    ``direction`` is the base-frame unit travel, ``item`` the object as the
    plan saw it, ``floor_z`` the descent floor (:func:`.grasp_geometry.
    descent_floor`), ``by_contact`` whether the descent ends in a search.
    """

    def __init__(self, direction, item: ObjectView, *, floor_z: float,
                 by_contact: bool = False, floor_name: str = ""):
        d = np.asarray(direction, dtype=float).reshape(3)
        self.direction = d / max(float(np.linalg.norm(d)), 1e-12)
        self.item = item
        self.floor_z = float(floor_z)
        self.by_contact = bool(by_contact)
        self.floor_name = str(floor_name)

    @property
    def descends(self) -> bool:
        from .orientation import VERTICAL_COS  # noqa: PLC0415
        return float(self.direction[2]) < -VERTICAL_COS


#: At or under this pad-face gap the jaws closed on NOTHING: the stroke
#: stalled on the pads themselves (or on a sliver no verb is planned for).
#: Capped at half the named object's width, so a declared 6 mm card is still
#: a card at 5 mm.
EMPTY_GAP_M = 0.006
#: How far a MEASURED stalled gap may sit from the DECLARED width and still be
#: that object, as a factor either way. The declaration is a model's estimate
#: from a photo; the gap is the robot's own measurement (d1-2, 2026-09-23: a
#: tape roll declared 50 mm was held at 57.1 mm and the old +-4 mm window
#: called it a miss). Beyond a factor of two the jaws are on something the
#: declaration does not describe, and that is UNKNOWN, not TRUE.
WIDTH_PLAUSIBLE_FACTOR = 2.0


def grip_width_window(width_m: float, tol_m: float = GRIP_WIDTH_TOL_M):
    """The pad gaps that are ``width_m`` held, rather than air or the pads.

    Symmetric on purpose: BELOW the window the jaws travelled past the object
    (they are on themselves, or on something thinner), ABOVE it they never
    reached it.
    """
    width = float(width_m)
    return max(0.0, width - float(tol_m)), width + float(tol_m)


#: :func:`grip_fit` outcomes
FIT_HELD, FIT_EMPTY, FIT_BLOCKED, FIT_IMPLAUSIBLE = (
    "held", "empty", "blocked", "implausible")


def grip_fit(gap_m: float, declared_m: float, *,
             open_gap_m: Optional[float] = None,
             reference=None) -> Dict[str, Any]:
    """What a STALLED pad-face gap says about the object it was closed on.

    MEASUREMENT OUTRANKS DECLARATION. The declared width is somebody's estimate;
    a stalled gap is where the pads actually stopped. So the gap decides, and
    the declaration is only graded:

    * ``empty``   the gap is at most :data:`EMPTY_GAP_M` (or half the declared
                  width, for something thinner): nothing between the pads;
    * ``blocked`` the gap is within one planner clearance of the hand's open
                  gap (``open_gap_m``, else the nominal driven opening): the
                  pads barely moved, stopped by something before the object;
    * ``implausible``  more than :data:`WIDTH_PLAUSIBLE_FACTOR` off the
                  declaration either way: the jaws stalled on something the
                  declaration does not describe;
    * ``held``    anything else — the object, at its MEASURED width.

    ``reference`` is the grasp's :class:`~.grasp_geometry.GraspReference`. The
    gap is the pad-FACE gap for either reference (a parallel gripper's faces
    are parallel), so it does not move the band's lower half; it sets the
    ``blocked`` margin, which is the per-side clearance that reference plans
    with (pad 4 mm, tip 2 mm): a hold wider than ``open - clearance`` is pads
    that travelled less than the planner would ever leave free. The +-
    :data:`GRIP_WIDTH_TOL_M` window is kept as ``matches_declaration`` — a
    note on how good the declaration was, not the verdict.
    """
    from ..hands.d1.parallel_gripper.description import (  # noqa: PLC0415
        DRIVEN_OPEN_GAP_M)
    from .grasp_geometry import PAD  # noqa: PLC0415
    ref = PAD if reference is None else reference
    gap, declared = float(gap_m), float(declared_m)
    opening = DRIVEN_OPEN_GAP_M if open_gap_m is None else float(open_gap_m)
    empty = min(EMPTY_GAP_M, 0.5 * declared)
    blocked = opening - float(ref.clearance_per_side_m)
    low = declared / WIDTH_PLAUSIBLE_FACTOR
    high = min(blocked, declared * WIDTH_PLAUSIBLE_FACTOR)
    w_low, w_high = grip_width_window(declared)
    if gap <= empty:
        kind = FIT_EMPTY
    elif gap >= blocked:
        kind = FIT_BLOCKED
    elif not low <= gap <= high:
        kind = FIT_IMPLAUSIBLE
    else:
        kind = FIT_HELD
    return {"fit": kind, "declared_width_m": round(declared, 4),
            "width_band_m": [round(max(low, empty), 4), round(high, 4)],
            "width_window_m": [round(w_low, 4), round(w_high, 4)],
            "matches_declaration": bool(w_low <= gap <= w_high),
            "open_gap_m": round(opening, 4), "contact": ref.name}


def resting_on(obj: ObjectView, world: WorldView, *,
               tol_m: float = RESTING_TOL_M) -> Optional[SurfaceView]:
    """The surface ``obj`` is still standing on, if any.

    A lift that reports a rise while the object's underside is still on the
    table is measuring the SURFACE moving, or a pose estimate drifting. The
    test is the object's own base against the surface top, over its footprint —
    ``SurfaceView.supports`` is about a CENTRE and a 30 mm band, which a 40 mm
    cube satisfies while held 12 mm in the air.
    """
    try:
        p = obj.pose_in_base(world.frames)[0]
    except LookupError:
        return None
    base_z = float(p[2]) - obj.height() / 2.0
    for surface in world.surfaces():
        try:
            sp, sr = surface.pose_in_base(world.frames)
        except LookupError:
            continue
        top = float(sp[2]) + float(surface.size[2]) / 2.0
        if abs(base_z - top) > float(tol_m):
            continue
        local = sr.inv().apply(np.asarray(p, dtype=float) - sp)
        half = np.asarray(surface.size, dtype=float) / 2.0
        if abs(local[0]) <= half[0] and abs(local[1]) <= half[1]:
            return surface
    return None


def moved_tol(delta, *, fraction: float = MOVED_TOL_FRACTION,
              floor_m: float = MIN_MOVED_TOL_M) -> float:
    """How far off a commanded displacement may land and still count.

    ``max(3 mm, 0.4 * |delta|)``: proportional, so the 10 mm end of the nudge
    grid is graded by a 4 mm window and the 50 mm end by a 20 mm one, and a
    hand that did not move fails BOTH.
    """
    return max(float(floor_m),
               float(fraction) * float(np.linalg.norm(np.asarray(delta, dtype=float))))


def _unknown(reason: str, **measured) -> VerdictReport:
    return VerdictReport(Verdict.UNKNOWN, reason, measured)


def _true(reason: str, **measured) -> VerdictReport:
    return VerdictReport(Verdict.TRUE, reason, measured)


def _false(reason: str, **measured) -> VerdictReport:
    return VerdictReport(Verdict.FALSE, reason, measured)


#: A parallel gripper is its own mirror image under a half turn about its
#: approach axis (TCP +z), so both of these are the SAME grasp and a verifier
#: that only knows one of them fails a correct wrist half the time.
_JAW_SYMMETRY = (R.identity(), R.from_rotvec([0.0, 0.0, math.pi]))


def _tool_point(world: WorldView, side: str) -> Optional[np.ndarray]:
    arm = world.arm(side)
    return None if arm is None or arm.tool_p is None else np.asarray(arm.tool_p)


def _object_p(world: WorldView, name: str) -> Optional[np.ndarray]:
    item = world.find(name)
    if item is None:
        return None
    try:
        return item.pose_in_base(world.frames)[0]
    except LookupError:
        return None


def _why_missing(world: WorldView, name: str) -> str:
    """Why ``name`` has no base-frame position — absent, or a bad frame.

    ``ObjectIn`` caught the object's own frame failure and let the
    DESTINATION's escape as a raw ``FrameError`` out of ``contains`` (R12).
    Both go through here now, and both produce UNKNOWN rather than a
    traceback.
    """
    item = world.find(name)
    if item is None:
        return f"{name!r} is not in the later observation"
    try:
        item.pose_in_base(world.frames)
    except FrameError as exc:
        return (f"{name!r} was measured in {item.frame_id!r} and that frame "
                f"cannot be resolved: {exc.reason}")
    except LookupError as exc:
        return f"{name!r}'s frame could not be resolved: {exc}"
    return f"{name!r} has no usable position"


def _stated(world1: WorldView, world0: WorldView, name: str) -> str:
    """``"declared"`` / ``"judged"`` when ``name``'s position is a STATEMENT
    (:data:`~manipulation_kit.world.views.STATED`) in the later world, else
    "". A judged pose (the wrist-look servo's classifier moved the
    declaration) is treated exactly like a declared one: plannable, never a
    measurement of where the thing is."""
    item = world1.find(name)
    if item is None:
        item = world0.find(name)
    if item is None or item.provenance not in STATED:
        return ""
    return item.provenance


def _inferred(world: WorldView, name: str) -> str:
    """"" for a sighted pose; otherwise the words that say it is not one.

    An ``attached`` pose is the tool pose composed with the grasp recorded at
    the stroke, a ``predicted`` one is where the hand let go — both inferences
    (:mod:`manipulation_kit.world.attach`). A verdict computed from either says
    so, and a NEGATIVE one becomes UNKNOWN: a confident FALSE from a pose
    nobody saw is as wrong as a confident TRUE (Astra review 11).
    """
    item = world.find(name)
    if item is None or item.provenance not in ("attached", "predicted"):
        return ""
    how = ("riding the hand — the tool pose and the grasp recorded at the "
           "stroke" if item.provenance == "attached"
           else "where the hand let go")
    return f"{name!r}'s pose is {item.provenance} ({how}): inferred, not sighted"


def turn_tol(asked_rad: float) -> float:
    return max(MIN_TURN_TOL_RAD, TURN_TOL_FRACTION * abs(float(asked_rad)))


class ToolAt(Verifier):
    """The tool point of ``side`` reached a commanded base-frame position."""

    describes = "the hand arrived where the plan sent it"

    def __init__(self, primitive: str, world0: WorldView, side: str, target,
                 tol_m: float = TOOL_TOL_M):
        super().__init__(primitive, world0)
        self.side = side
        self.target = np.asarray(target, dtype=float).reshape(3)
        self.tol_m = float(tol_m)

    def measure(self, world1: WorldView) -> VerdictReport:
        now = _tool_point(world1, self.side)
        if now is None:
            return _unknown(f"the {self.side} arm reports no tool point, so "
                            f"arrival cannot be measured")
        gap = float(np.linalg.norm(now - self.target))
        measured = {"tool_p": [round(float(v), 4) for v in now],
                    "target_p": [round(float(v), 4) for v in self.target],
                    "gap_m": round(gap, 4)}
        if gap <= self.tol_m:
            return _true(f"the {self.side} tool point is {gap * 1000:.0f} mm "
                         f"from the commanded pose", **measured)
        return _false(f"the {self.side} tool point is {gap * 1000:.0f} mm from "
                      f"the commanded pose (tolerance {self.tol_m * 1000:.0f} mm)",
                      **measured)


class ToolMoved(Verifier):
    """The tool point of ``side`` moved by a commanded displacement."""

    describes = "the hand moved by what was asked"

    def __init__(self, primitive: str, world0: WorldView, side: str, delta,
                 tol_m: Optional[float] = None):
        super().__init__(primitive, world0)
        self.side = side
        self.delta = np.asarray(delta, dtype=float).reshape(3)
        #: ``None`` means "scale it to the request" — see :func:`moved_tol`.
        #: A caller that passes a number owns it.
        self.tol_m = moved_tol(self.delta) if tol_m is None else float(tol_m)
        self.before = _tool_point(world0, side)

    def measure(self, world1: WorldView) -> VerdictReport:
        now = _tool_point(world1, self.side)
        if now is None or self.before is None:
            return _unknown(f"the {self.side} arm reports no tool point, so "
                            f"the displacement cannot be measured")
        moved = now - self.before
        err = float(np.linalg.norm(moved - self.delta))
        measured = {"moved_m": [round(float(v), 4) for v in moved],
                    "asked_m": [round(float(v), 4) for v in self.delta],
                    "error_m": round(err, 4)}
        if err <= self.tol_m:
            return _true(f"the {self.side} hand moved "
                         f"{np.linalg.norm(moved) * 1000:.0f} mm as asked", **measured)
        return _false(f"the {self.side} hand is {err * 1000:.0f} mm off the "
                      f"requested displacement", **measured)


class ToolFacing(Verifier):
    """The tool's ORIENTATION reached the one the plan derived.

    ``Approach`` exists to establish a wrist the next verb descends along, and
    it verified the point alone (R13). An arm at the right place with the
    pads 90 degrees out has not approached anything.
    """

    describes = "the hand is oriented the way the approach derived"

    def __init__(self, primitive: str, world0: WorldView, side: str, target,
                 tol_rad: float = FACING_TOL_RAD):
        super().__init__(primitive, world0)
        self.side = side
        self.target = target
        self.tol_rad = float(tol_rad)

    def measure(self, world1: WorldView) -> VerdictReport:
        arm = world1.arm(self.side)
        if arm is None or arm.tool_r is None:
            return _unknown(f"the {self.side} arm reports no tool orientation, "
                            f"so the wrist cannot be compared")
        # A parallel gripper is symmetric under a half turn about its own
        # approach axis, so the nearer of the two representatives is the
        # honest comparison.
        best = min(float(np.linalg.norm(
            (arm.tool_r.inv() * (self.target * flip)).as_rotvec()))
            for flip in _JAW_SYMMETRY)
        measured = {"orientation_error_deg": round(math.degrees(best), 2),
                    "tolerance_deg": round(math.degrees(self.tol_rad), 2)}
        if best <= self.tol_rad:
            return _true(f"the {self.side} wrist is "
                         f"{math.degrees(best):.1f} deg from the derived "
                         f"orientation", **measured)
        return _false(f"the {self.side} wrist is {math.degrees(best):.1f} deg "
                      f"from the derived orientation", **measured)


class ToolTurned(Verifier):
    """The tool ROTATED by a commanded angle about a stated axis.

    ``Nudge`` snapped a yaw, planned it, and then returned a translation-only
    verdict: ``Nudge(side="left", dyaw=0.2).verifier(w)(w)`` was TRUE — "the
    left hand moved 0 mm as asked" (R13). A rotation nobody measures is a
    rotation nobody performed, as far as the record goes.
    """

    describes = "the hand turned by what was asked"

    def __init__(self, primitive: str, world0: WorldView, side: str,
                 roll_rad: float, axis=None, tol_rad: Optional[float] = None):
        super().__init__(primitive, world0)
        self.side = side
        self.roll_rad = float(roll_rad)
        self.axis = None if axis is None else np.asarray(axis, dtype=float)
        self.tol_rad = turn_tol(roll_rad) if tol_rad is None else float(tol_rad)
        arm = world0.arm(side)
        self.before = None if arm is None else arm.tool_r

    def measure(self, world1: WorldView) -> VerdictReport:
        arm = world1.arm(self.side)
        if arm is None or arm.tool_r is None or self.before is None:
            return _unknown(f"the {self.side} arm reports no tool orientation "
                            f"in both observations, so the turn cannot be "
                            f"measured")
        delta = (self.before.inv() * arm.tool_r).as_rotvec()
        axis = (self.before.as_matrix()[:, 2] if self.axis is None
                else self.axis)
        # the component ABOUT THE APPROACH AXIS, expressed in the pre-action
        # tool frame — which is the axis the nudge was defined around
        local = self.before.inv().apply(np.asarray(axis, dtype=float))
        local = local / max(float(np.linalg.norm(local)), 1e-12)
        turned = float(np.dot(delta, local))
        err = abs(turned - self.roll_rad)
        measured = {"turned_deg": round(math.degrees(turned), 2),
                    "asked_deg": round(math.degrees(self.roll_rad), 2),
                    "off_axis_deg": round(math.degrees(float(np.linalg.norm(
                        delta - local * turned))), 2),
                    "tolerance_deg": round(math.degrees(self.tol_rad), 2)}
        if err <= self.tol_rad:
            return _true(f"the {self.side} hand turned "
                         f"{math.degrees(turned):+.1f} deg about its approach "
                         f"axis as asked", **measured)
        return _false(f"the {self.side} hand turned "
                      f"{math.degrees(turned):+.1f} deg of the "
                      f"{math.degrees(self.roll_rad):+.1f} deg asked",
                      **measured)


class Holding(Verifier):
    """Something is held, and it is the OBJECT — three measurements, not a flag.

    "The jaws are closed" is not a number on a scale, and grading it as one is
    F8: the Isaac env gated ``held_by`` on a jaw-travel ratio above 0.6, which
    on 70 mm pads is a 28 mm ceiling, so a 40 mm cube could not be reported held
    however well it was gripped — and was not, through 149.8 mm of measured
    lift. What a hold actually is:

    1. **The jaws stopped short of where they were sent** (``jaw_stalled``).
       Commanded closed, no longer moving, not at the target: for a force- or
       torque-limited drive that IS the stop, and it is the same signal
       ``GripperReport.holding`` carries on the robot.
    2. **A body is between the two pad faces** — which only the producer can
       see, and which is exactly what ``holding`` means here.
    3. **The gap is one the named object could make** (:func:`grip_fit`).
       The MEASURED gap outranks the DECLARED width: a stall inside the
       plausibility band is the object, at its measured width, and the verdict
       says so (``width_correction``) when that differs from the declaration
       by more than :data:`GRIP_WIDTH_TOL_M`. Only a gap at (near) zero —
       nothing between the pads — or at (near) the open gap — blocked before
       the object — is FALSE; a gap more than a factor of two off the
       declaration is UNKNOWN.

    Every part is optional-if-unmeasured and never optional-if-measured: a
    producer that reports no gap gets (1) and (2) — the documented fallback for
    an object of unknown size — and a producer that reports nothing at all gets
    UNKNOWN, never TRUE.
    """

    describes = "the jaws stalled on the object, at a gap its own width could make"

    def __init__(self, primitive: str, world0: WorldView, side: str,
                 obj: Optional[ObjectView] = None,
                 tol_m: float = GRIP_WIDTH_TOL_M, *, jaw_axis=None,
                 reference=None, stroke: Optional[GraspStroke] = None):
        super().__init__(primitive, world0)
        self.side = side
        self.obj = obj
        self.tol_m = float(tol_m)
        #: the grasp's planned travel (:class:`GraspStroke`); ``None`` for a
        #: hold that was not taken by a travel onto the object (a handover's
        #: receiver), which is graded on the jaws alone
        self.stroke = stroke
        #: the grasp's GraspReference (pad / tip); None = pad
        self.reference = reference
        #: the base-frame jaw axis the grasp was planned with. The width the
        #: pads must span is the object's extent ALONG THIS, not its smallest
        #: side (R9).
        self.jaw_axis = None if jaw_axis is None else np.asarray(
            jaw_axis, dtype=float).reshape(3)

    def _width(self, world1: WorldView) -> Optional[float]:
        if self.obj is None:
            return None
        if self.jaw_axis is None:
            return float(self.obj.min_horizontal_extent())
        for world in (world1, self.world0):
            item = world.find(self.obj.name) or self.obj
            try:
                return float(item.extent_along(self.jaw_axis, world.frames))
            except LookupError:
                continue
        return None

    # -- the stroke's two criteria ------------------------------------------ #
    def _insertion(self, world1: WorldView) -> Dict[str, Any]:
        """Criterion (a): how far past the object's near face the finger
        tips were when the jaws closed."""
        from . import grasp_geometry as gg  # noqa: PLC0415
        stroke = self.stroke
        arm = world1.arm(self.side)
        if arm is None or arm.tool_p is None or arm.tool_r is None:
            return {"verdict": CHECK_UNMEASURED,
                    "why": f"the {self.side} arm reports no tool pose"}
        tip = gg.fingertip_point(arm.tool_p, arm.tool_r)
        try:
            frames = self.world0.frames
            insertion = gg.insertion_along(stroke.item, frames,
                                           stroke.direction, tip)
            need = gg.min_insertion_m(stroke.item, frames, stroke.direction)
            top = stroke.item.top_face_z(frames)
        except LookupError as exc:
            return {"verdict": CHECK_UNMEASURED,
                    "why": f"{stroke.item.name!r}'s planned pose does not "
                           f"resolve: {exc}"}
        out = {"verdict": CHECK_PASS if insertion >= need else CHECK_FAIL,
               "insertion_m": round(insertion, 4),
               "insertion_min_m": round(need, 4),
               "tip_p": [round(float(v), 4) for v in tip],
               "object_provenance": stroke.item.provenance}
        if stroke.descends:
            out.update(tip_z_m=round(float(tip[2]), 4),
                       object_top_z_m=round(float(top), 4))
        return out

    def _approach(self, world1: WorldView, run: Any) -> Dict[str, Any]:
        """Criterion (b): did the travel reach its commanded depth, or stop
        early — the executor's arrival and stop reason, and where a
        fingertip search stopped."""
        from ..executor import ARRIVE_TOL_ALONG_M  # noqa: PLC0415
        stroke = self.stroke
        out: Dict[str, Any] = {"by_contact": stroke.by_contact}
        fails = []
        evidence = False
        if run is not None:
            evidence = True
            out["run_completed"] = bool(run.completed)
            out["stop_reason"] = run.stop_reason or ""
            arrival = next((a for a in reversed(tuple(run.arrivals))
                            if a.waypoint_label == "grasp"), None)
            if arrival is not None:
                out["arrived_at_grasp"] = bool(arrival.arrived)
                along = float(arrival.tool_along_m)
                if math.isfinite(along):
                    out["tool_along_m"] = round(along, 4)
            if not run.completed:
                fails.append(f"the run stopped before the close finished "
                             f"({run.stop_reason or 'no reason given'}"
                             f"{': ' + run.error if run.error else ''})")
            elif arrival is not None and not arrival.arrived:
                fails.append(f"the {self.side} arm did not arrive at the "
                             f"grasp point: {arrival.detail}")
            elif "tool_along_m" in out and out["tool_along_m"] < -ARRIVE_TOL_ALONG_M:
                fails.append(f"the {self.side} tool stopped "
                             f"{-out['tool_along_m'] * 1000:.0f} mm short of "
                             f"the grasp point along the approach")
        tip = self._search_stop(world1, run)
        if stroke.by_contact and tip is not None:
            evidence = True
            made, p = tip
            above = float(p[2]) - stroke.floor_z
            out.update(search_contact=made,
                       search_stop_z_m=round(float(p[2]), 4),
                       search_stop_above_floor_m=round(above, 4))
            if made and above > SEARCH_STOP_TOL_M:
                try:
                    top = stroke.item.top_face_z(self.world0.frames)
                    what = (f"{stroke.item.name!r}'s top is "
                            f"{(top - stroke.floor_z) * 1000:.0f} mm up")
                except LookupError:
                    what = f"{stroke.item.name!r} is where it stopped"
                fails.append(
                    f"the fingertip search stopped by contact "
                    f"{above * 1000:.0f} mm above "
                    f"{stroke.floor_name or 'the surface'} ({what}): it met "
                    f"something before the surface it was sent to — a "
                    f"collision, not a grasp")
        if not evidence:
            out.update(verdict=CHECK_UNMEASURED,
                       why="no run report and no contact record")
            return out
        out["verdict"] = CHECK_FAIL if fails else CHECK_PASS
        if fails:
            out["why"] = "; ".join(fails)
        return out

    def _search_stop(self, world1: WorldView, run: Any):
        """``(made, fingertip point)`` where this grasp's search leg stopped:
        the run's own contact report, else the contact the loop folded into
        the later world (verb ``grasp``, this side, after ``world0``)."""
        from . import grasp_geometry as gg  # noqa: PLC0415
        for report in reversed(tuple(getattr(run, "contacts", ()) or ())):
            if report.side != self.side:
                continue
            travel = -np.asarray(report.normal_hint, dtype=float)
            travel = travel / max(float(np.linalg.norm(travel)), 1e-12)
            return (bool(report.made),
                    np.asarray(report.p_tool, dtype=float)
                    + travel * gg.PAD.lead_m)
        for contact in reversed(tuple(world1.contacts)):
            if (contact.side == self.side and contact.verb == "grasp"
                    and float(contact.stamp) >= float(self.world0.stamp)):
                return bool(contact.made), np.asarray(contact.p, dtype=float)
        return None

    def measure(self, world1: WorldView) -> VerdictReport:
        return self.measure_run(world1, None)

    def measure_run(self, world1: WorldView, run: Any = None) -> VerdictReport:
        gripper = world1.gripper(self.side)
        if gripper is None:
            return _unknown(f"no gripper report for the {self.side} hand")
        name = None if self.obj is None else self.obj.name
        measured = {"holding": bool(gripper.holding),
                    "closedness": round(float(gripper.closedness), 3),
                    "jaw_stalled": gripper.jaw_stalled,
                    "held_object": gripper.held_object,
                    "named_object": name}
        if gripper.jaw_gap_m is not None:
            measured["jaw_gap_m"] = round(float(gripper.jaw_gap_m), 4)
        # EVERY CRITERION, WITH ITS OWN VERDICT, in every record — so the log
        # and the model see which one failed, not only that one did.
        checks: Dict[str, Dict[str, Any]] = {
            "jaw_gap": {"verdict": CHECK_UNMEASURED}}
        insertion = approach = None
        if self.stroke is not None:
            insertion = self._insertion(world1)
            approach = self._approach(world1, run)
            checks["insertion"] = insertion
            checks["approach"] = approach
            for key in ("tip_z_m", "object_top_z_m", "insertion_m",
                        "insertion_min_m"):
                if key in insertion:
                    measured[key] = insertion[key]
            measured["approach_completed"] = (
                None if approach["verdict"] == CHECK_UNMEASURED
                else approach["verdict"] == CHECK_PASS)
        measured["checks"] = checks
        # (0) CONTRADICTORY IDENTITY beats everything. A hand that says it is
        # holding something else is not holding this.
        if (name is not None and gripper.held_object
                and gripper.held_object != name):
            return _false(f"the {self.side} gripper reports holding "
                          f"{gripper.held_object!r}, not {name!r}", **measured)
        # (1) the stroke stopped on something, rather than running to target
        if gripper.jaw_stalled is False:
            return _false(f"the {self.side} jaws never stopped short of the "
                          f"close they were commanded — nothing arrested them",
                          **measured)
        # (3) a gap the named object could make.  Checked BEFORE the producer's
        # verdict so "it closed on itself" is reported as itself rather than as
        # a bare `nothing held`.
        width = self._width(world1)
        fit = None
        if width is not None and gripper.jaw_gap_m is not None:
            gap_m = float(gripper.jaw_gap_m)
            fit = grip_fit(gap_m, width, open_gap_m=gripper.open_gap_m,
                           reference=self.reference)
            measured.update(fit, object_width_m=round(float(width), 4))
            checks["jaw_gap"] = {
                "verdict": (CHECK_PASS if fit["fit"] == FIT_HELD
                            else CHECK_UNMEASURED
                            if fit["fit"] == FIT_IMPLAUSIBLE else CHECK_FAIL),
                "fit": fit["fit"], "jaw_gap_m": round(gap_m, 4),
                "width_band_m": fit["width_band_m"],
                "width_window_m": fit["width_window_m"],
                "matches_declaration": fit["matches_declaration"]}
            if fit["fit"] == FIT_EMPTY:
                return _false(
                    f"the {self.side} gripper stalled at {gap_m * 1000:.1f} mm "
                    f"— it closed on itself, not on {name!r}: nothing is "
                    f"between the pads", **measured)
            if fit["fit"] == FIT_BLOCKED:
                return _false(
                    f"the {self.side} gripper stalled at {gap_m * 1000:.1f} mm, "
                    f"at (or past) its {fit['open_gap_m'] * 1000:.1f} mm open "
                    f"gap — something blocked the pads before {name!r}: the "
                    f"jaws never reached it", **measured)
        # (2) the half only the producer can see: a body between the pad faces
        if not gripper.holding:
            return _false(f"the {self.side} gripper reports nothing between its "
                          f"pads", **measured)
        # (5) THE STROKE. A stall at a width the object could make is not a
        # hold if the travel stopped early or the fingers never got into the
        # object (d1-2, 2026-09-24: a 48 mm stall inside a 46-54 mm window
        # was reported held, and the roll never left the table).
        stalled = ("" if gripper.jaw_gap_m is None
                   else f" stalled at {gripper.jaw_gap_m * 1000:.0f} mm")
        if approach is not None and approach["verdict"] == CHECK_FAIL:
            return _false(f"the {self.side} jaws{stalled}, but the approach "
                          f"did not complete: {approach['why']}", **measured)
        if insertion is not None and insertion["verdict"] == CHECK_FAIL:
            face = "top" if self.stroke.descends else "near face"
            return _false(
                f"the {self.side} jaws{stalled} with the finger tips "
                f"{insertion['insertion_m'] * 1000:+.0f} mm past "
                f"{self.stroke.item.name!r}'s {face} (insertion "
                f"{insertion['insertion_m'] * 1000:+.1f} mm; a hold needs "
                f"{insertion['insertion_min_m'] * 1000:.1f} mm): a pinch on "
                f"its rim or edge, not a hold", **measured)
        if fit is not None and fit["fit"] == FIT_IMPLAUSIBLE:
            return _unknown(
                f"the {self.side} jaws stalled on something at "
                f"{gripper.jaw_gap_m * 1000:.1f} mm, but {name!r} was declared "
                f"{width * 1000:.1f} mm — more than a factor of "
                f"{WIDTH_PLAUSIBLE_FACTOR:.0f} off, so this is not evidence it "
                f"is {name!r}. Look again and re-declare it", **measured)
        gap = ("" if gripper.jaw_gap_m is None
               else f" at a {gripper.jaw_gap_m * 1000:.1f} mm gap")
        if fit is not None and not fit["matches_declaration"]:
            # the MEASURED width replaces the declared one (the loop hands
            # this to the world source)
            measured["width_provenance"] = "measured"
            measured["width_correction"] = {
                "object": name, "declared_m": fit["declared_width_m"],
                "measured_m": round(float(gripper.jaw_gap_m), 4),
                "jaw_axis": (None if self.jaw_axis is None
                             else [round(float(v), 4) for v in self.jaw_axis])}
            gap += (f"; declared {width * 1000:.1f} mm, width corrected to "
                    f"the MEASURED {gripper.jaw_gap_m * 1000:.1f} mm")
        if insertion is not None and insertion["verdict"] == CHECK_UNMEASURED:
            # conservative: a hold is jaws AND fingers in the object, and the
            # second half was not measured
            return _unknown(f"the {self.side} jaws stalled{gap}, but how far "
                            f"the fingers got into {name!r} could not be "
                            f"measured: {insertion['why']}", **measured)
        if name is None:
            return _true(f"the {self.side} gripper is holding{gap}", **measured)
        # (4) ASSOCIATION. "Something is gripped" is not "the named block is
        # gripped", and a torque stall cannot tell them apart. Either the
        # producer names what it holds, or the named object is measured at the
        # pads; with neither, this is UNKNOWN (R11).
        if gripper.held_object == name:
            measured["association"] = "producer"
            return _true(f"the {self.side} gripper is holding {name!r}{gap}",
                         **measured)
        at = _object_p(world1, name)
        tool = _tool_point(world1, self.side)
        if at is None or tool is None:
            measured["association"] = None
            return _unknown(
                f"the {self.side} jaws stalled on something{gap}, but nothing "
                f"in this observation ties it to {name!r}: the gripper does "
                f"not report a held object and "
                f"{'the arm reports no tool point' if tool is None else _why_missing(world1, name)}. "
                f"Publish GripperView(held_object=...) or keep observing the "
                f"object to turn this into a verdict", **measured)
        distance = float(np.linalg.norm(np.asarray(at) - tool))
        measured["object_to_tool_m"] = round(distance, 4)
        stated = _stated(world1, self.world0, name)
        said = f"{stated} at" if stated else "measured"
        if distance > ASSOCIATION_TOL_M:
            return _false(
                f"the {self.side} jaws stalled on something, but {name!r} is "
                f"{said} {distance * 1000:.0f} mm from the tool point — "
                f"whatever is between the pads, it is not that", **measured)
        if stated:
            # A DECLARED or JUDGED position is where the grasp was planned
            # to, so "it is at the tool point" is true by construction and
            # measures nothing. Only the measured gap (grip_fit) can tie
            # the stall to this object.
            measured["provenance"] = stated
            if fit is None:
                measured["association"] = None
                return _unknown(
                    f"the {self.side} jaws stalled on something{gap}, and "
                    f"{name!r} is {stated} {distance * 1000:.0f} mm from the "
                    f"tool point — but a {stated} position is where the "
                    f"grasp aimed, not a measurement, and no jaw gap was "
                    f"measured to say the pads closed on {name!r}", **measured)
            measured["association"] = "stated_position+grip_fit"
            return _true(f"the {self.side} gripper is holding {name!r}{gap}: "
                         f"the measured gap fits {name!r}, {stated} "
                         f"{distance * 1000:.0f} mm from the tool point",
                         **measured)
        measured["association"] = "measured_position"
        return _true(f"the {self.side} gripper is holding {name!r}{gap}, and "
                     f"{name!r} is measured {distance * 1000:.0f} mm from the "
                     f"tool point", **measured)


class NotHolding(Verifier):
    """The gripper let go: nothing held AND the jaws actually opened."""

    describes = "the gripper is open and holding nothing"

    def __init__(self, primitive: str, world0: WorldView, side: str):
        super().__init__(primitive, world0)
        self.side = side

    def measure(self, world1: WorldView) -> VerdictReport:
        gripper = world1.gripper(self.side)
        if gripper is None:
            return _unknown(f"no gripper report for the {self.side} hand")
        measured = {"holding": bool(gripper.holding),
                    "closedness": round(float(gripper.closedness), 3)}
        if gripper.holding:
            return _false(f"the {self.side} gripper still reports holding", **measured)
        if gripper.closedness > OPEN_CLOSEDNESS:
            return _false(f"the {self.side} gripper reports nothing held but its "
                          f"jaws are {gripper.closedness:.2f} closed", **measured)
        return _true(f"the {self.side} gripper is open and empty", **measured)


class ToolClearOf(Verifier):
    """A hand's tool point is at least ``min_gap_m`` OUTSIDE an object's box.

    The giving hand's half of a ``handover``: opening the jaws is not the end
    of the give — a hand still around the object drags it when it moves next.
    The gap is measured from the tool point (the pad centre) to the object's
    own oriented box, so a hand that backed straight out along its approach
    axis is measured on the axis it backed out along.

    Like the other clearance verdicts, a POSITIVE gap to an inferred pose
    (``attached`` to the other hand, ``predicted``) stays TRUE and says so; a
    negative one is UNKNOWN, because a pose nobody saw cannot convict.
    """

    describes = "the hand is clear of the object"

    def __init__(self, primitive: str, world0: WorldView, side: str,
                 name: str, min_gap_m: float):
        super().__init__(primitive, world0)
        self.side = side
        self.name = name
        self.min_gap_m = float(min_gap_m)

    def measure(self, world1: WorldView) -> VerdictReport:
        tool = _tool_point(world1, self.side)
        if tool is None:
            return _unknown(f"the {self.side} arm reports no tool point, so "
                            f"its clearance from {self.name!r} cannot be "
                            f"measured")
        item = world1.find(self.name)
        if item is None:
            return _unknown(_why_missing(world1, self.name))
        try:
            p, r = item.pose_in_base(world1.frames)
        except LookupError:
            return _unknown(_why_missing(world1, self.name))
        local = r.inv().apply(np.asarray(tool, dtype=float) - np.asarray(p))
        half = np.asarray(item.size, dtype=float).reshape(3) / 2.0
        gap = float(np.linalg.norm(np.maximum(np.abs(local) - half, 0.0)))
        measured = {"gap_m": round(gap, 4),
                    "min_gap_m": round(self.min_gap_m, 4)}
        inferred = _inferred(world1, self.name)
        if inferred:
            measured["provenance"] = item.provenance
        if gap >= self.min_gap_m:
            return _true(f"the {self.side} hand's tool point is "
                         f"{gap * 1000:.0f} mm clear of {self.name!r}"
                         + (f" — {inferred}" if inferred else ""), **measured)
        if inferred:
            return _unknown(f"by its inferred pose {self.name!r} is only "
                            f"{gap * 1000:.0f} mm from the {self.side} hand, "
                            f"but {inferred}", **measured)
        return _false(f"the {self.side} hand is only {gap * 1000:.0f} mm from "
                      f"{self.name!r} (needs {self.min_gap_m * 1000:.0f} mm)",
                      **measured)


class ObjectRose(Verifier):
    """The OBJECT went up — not the hand. Lift is about the thing, not the arm.

    And it went up OFF something: an object whose underside is still on the
    surface it started on has not been lifted, however the hand moved, so a
    reported rise that leaves it resting is a FALSE rather than a TRUE with a
    caveat. The check is skipped when the world publishes no surface, which is
    the honest fallback rather than an assumption about the table.
    """

    describes = "the object is higher than it was, off its support, and still held"

    def __init__(self, primitive: str, world0: WorldView, side: str,
                 name: str, height_m: float):
        super().__init__(primitive, world0)
        self.side = side
        self.name = name
        self.height_m = float(height_m)
        self.z0 = _object_p(world0, name)

    def measure(self, world1: WorldView) -> VerdictReport:
        now = _object_p(world1, self.name)
        if now is None or self.z0 is None:
            return _unknown(f"a rise cannot be compared: "
                            f"{_why_missing(world1, self.name)}")
        rise = float(now[2] - self.z0[2])
        need = self.height_m * MOVED_FRACTION
        gripper = world1.gripper(self.side)
        measured = {"rise_m": round(rise, 4), "asked_m": round(self.height_m, 4),
                    "holding": None if gripper is None else bool(gripper.holding)}
        inferred = _inferred(world1, self.name)
        item = world1.find(self.name)
        if item is not None:
            measured["provenance"] = item.provenance
        if gripper is None:
            # "Still held" is half the predicate, so a missing gripper report
            # is missing EVIDENCE, not a pass (R12). The old code skipped the
            # clause and could return TRUE with ``grippers={}``.
            return _unknown(f"{self.name} rose {rise * 1000:.0f} mm, but the "
                            f"{self.side} gripper reports nothing, so whether "
                            f"it is still held cannot be measured", **measured)
        if not gripper.holding:
            return _false(f"{self.name} rose {rise * 1000:.0f} mm but the "
                          f"{self.side} gripper is no longer holding it", **measured)
        if gripper.held_object and gripper.held_object != self.name:
            return _false(f"the {self.side} gripper is holding "
                          f"{gripper.held_object!r}, not {self.name!r}",
                          **measured)
        if rise < need:
            return _false(f"{self.name} rose {rise * 1000:.0f} mm of the "
                          f"{self.height_m * 1000:.0f} mm asked", **measured)
        obj = world1.find(self.name)
        support = None if obj is None else resting_on(obj, world1)
        measured["resting_on"] = None if support is None else support.name
        if support is not None:
            return _false(f"{self.name} reports a {rise * 1000:.0f} mm rise but "
                          f"its underside is still on {support.name}", **measured)
        # A held object's pose is the ATTACHED one (the tool pose and the
        # grasp recorded at the stroke), so the rise is the hand's, measured,
        # with the gripper reporting the object still in it. That is the
        # evidence a lift can have without a fresh sight, and the verdict says
        # which kind it is.
        return _true(f"{self.name} rose {rise * 1000:.0f} mm"
                     + (f" — {inferred}" if inferred else ""), **measured)


class ObjectOver(Verifier):
    """The object is over the destination, horizontally, and still held."""

    describes = "the object is above the destination"

    def __init__(self, primitive: str, world0: WorldView, side: str,
                 name: str, destination: str, tol_m: float = 0.05):
        super().__init__(primitive, world0)
        self.side = side
        self.name = name
        self.destination = destination
        self.tol_m = float(tol_m)

    def measure(self, world1: WorldView) -> VerdictReport:
        here = _object_p(world1, self.name)
        there = _object_p(world1, self.destination)
        if here is None:
            return _unknown(_why_missing(world1, self.name))
        if there is None:
            return _unknown(_why_missing(world1, self.destination))
        gap = float(np.linalg.norm(here[:2] - there[:2]))
        gripper = world1.gripper(self.side)
        measured = {"horizontal_gap_m": round(gap, 4),
                    "holding": None if gripper is None else bool(gripper.holding),
                    "held_object": None if gripper is None else gripper.held_object}
        if gripper is None:
            return _unknown(f"{self.name} is {gap * 1000:.0f} mm from over "
                            f"{self.destination}, but the {self.side} gripper "
                            f"reports nothing, so whether it is still held "
                            f"cannot be measured", **measured)
        if not gripper.holding:
            return _false(f"the {self.side} gripper dropped {self.name} on the way",
                          **measured)
        if gripper.held_object and gripper.held_object != self.name:
            return _false(f"the {self.side} gripper is holding "
                          f"{gripper.held_object!r}, not {self.name!r}",
                          **measured)
        inferred = _inferred(world1, self.name)
        if inferred:
            measured["provenance"] = world1.find(self.name).provenance
        if gap <= self.tol_m:
            return _true(f"{self.name} is {gap * 1000:.0f} mm from over "
                         f"{self.destination}"
                         + (f" — {inferred}" if inferred else ""), **measured)
        if inferred:
            return _unknown(f"by its inferred pose {self.name} is "
                            f"{gap * 1000:.0f} mm from over "
                            f"{self.destination}, but {inferred}; a sighting "
                            f"is what can call it", **measured)
        return _false(f"{self.name} is {gap * 1000:.0f} mm from over "
                      f"{self.destination}", **measured)


class ObjectClears(Verifier):
    """The object's UNDERSIDE is above a destination's rim / top face.

    The other half of a carry. Horizontal position alone passed an object
    dangling below the rim of the bin it was over (R12), which is the state
    just before it catches on the way across.
    """

    describes = "the object is clear of the destination's rim"

    def __init__(self, primitive: str, world0: WorldView, name: str,
                 destination: str, margin_m: float = 0.0):
        super().__init__(primitive, world0)
        self.name = name
        self.destination = destination
        self.margin_m = float(margin_m)

    def measure(self, world1: WorldView) -> VerdictReport:
        obj = world1.find(self.name)
        target = world1.find(self.destination)
        if obj is None:
            return _unknown(_why_missing(world1, self.name))
        if target is None:
            return _unknown(_why_missing(world1, self.destination))
        try:
            under = obj.bottom_z(world1.frames)
            if isinstance(target, ContainerView):
                top = target.rim_z(world1.frames)
                what = "rim"
            elif isinstance(target, SurfaceView):
                top = target.top_z(world1.frames)
                what = "top"
            else:
                top = target.top_face_z(world1.frames)
                what = "top"
        except LookupError as exc:
            return _unknown(f"the clearance cannot be resolved: {exc}")
        gap = under - top
        measured = {"underside_z_m": round(under, 4),
                    f"{what}_z_m": round(top, 4),
                    "clearance_m": round(gap, 4)}
        inferred = _inferred(world1, self.name)
        if inferred:
            measured["provenance"] = obj.provenance
        if gap >= self.margin_m:
            return _true(f"{self.name}'s underside is {gap * 1000:.0f} mm above "
                         f"{self.destination}'s {what}"
                         + (f" — {inferred}" if inferred else ""), **measured)
        if inferred:
            return _unknown(f"by its inferred pose {self.name}'s underside is "
                            f"{-gap * 1000:.0f} mm below {self.destination}'s "
                            f"{what}, but {inferred}", **measured)
        return _false(f"{self.name}'s underside is {-gap * 1000:.0f} mm BELOW "
                      f"{self.destination}'s {what}", **measured)


class ObjectIn(Verifier):
    """The object ended up inside a container, or on a surface, and was let go.

    Every clause is measured and every missing clause is UNKNOWN:

    1. the object's own EXTENT is inside the interior (or its footprint is on
       the surface) — not its centre, which passed a bar twice the bin's width;
    2. it is SUPPORTED — its underside on the container floor or the surface
       top, within :data:`RESTING_TOL_M`. This is what this robot can measure
       of "at rest": nothing publishes a velocity, so a falling object passing
       through the interior is excluded by where its underside is rather than
       by watching it stop. The limitation is named in the verdict;
    3. the gripper LET GO, and is not holding something else instead.

    A gripper that reports nothing makes the whole thing UNKNOWN. It used to
    be read as "released" (R12).
    """

    describes = ("the object's extent is in/on the destination, supported, and "
                 "no longer held")

    def __init__(self, primitive: str, world0: WorldView, side: str,
                 name: str, destination: str, pad_m: float = 0.01):
        super().__init__(primitive, world0)
        self.side = side
        self.name = name
        self.destination = destination
        self.pad_m = float(pad_m)

    def measure(self, world1: WorldView) -> VerdictReport:
        obj = world1.find(self.name)
        target = world1.find(self.destination)
        if obj is None:
            return _unknown(_why_missing(world1, self.name))
        if target is None:
            return _unknown(_why_missing(world1, self.destination))
        try:
            p = obj.pose_in_base(world1.frames)[0]
            # BOTH frames, inside the same guard. The destination's own frame
            # failure used to escape from ``contains``/``supports`` as an
            # exception, outside the catch that covered the object (R12).
            target.pose_in_base(world1.frames)
        except LookupError:
            return _unknown(f"{_why_missing(world1, self.name)}; "
                            f"{_why_missing(world1, self.destination)}")
        gripper = world1.gripper(self.side)
        measured = {"p": [round(float(v), 4) for v in p],
                    "holding": None if gripper is None else bool(gripper.holding),
                    "held_object": None if gripper is None else gripper.held_object}
        try:
            if isinstance(target, ContainerView):
                inside = target.contains_object(obj, world1.frames,
                                                pad_m=self.pad_m)
                floor = target.floor_z(world1.frames)
                where = f"inside {self.destination}"
            elif isinstance(target, SurfaceView):
                inside = target.over(p, world1.frames, pad_m=self.pad_m)
                floor = target.top_z(world1.frames)
                where = f"on {self.destination}"
            else:
                return _unknown(f"{self.destination} is a plain object, not a "
                                f"container or a surface — 'placed in' has no "
                                f"measurable meaning for it")
            under = obj.bottom_z(world1.frames)
        except LookupError as exc:
            return _unknown(f"the placement cannot be resolved: {exc}")
        rest = under - floor
        measured["inside"] = bool(inside)
        measured["underside_above_floor_m"] = round(rest, 4)
        inferred = _inferred(world1, self.name)
        if inferred:
            measured["provenance"] = obj.provenance
            if gripper is not None and gripper.holding and (
                    not gripper.held_object
                    or gripper.held_object == self.name):
                return _false(f"the {self.side} gripper is still holding "
                              f"{self.name}", **measured)
            # "In the cup, released, standing on the floor" is a claim about
            # where the thing CAME TO REST, and an inferred pose has not seen
            # it come to rest anywhere: never TRUE, never a confident FALSE.
            return _unknown(
                f"by its inferred pose {self.name} would be "
                f"{'' if inside else 'NOT '}{where} "
                f"({rest * 1000:+.0f} mm from the floor), but {inferred}; "
                f"look, and declare what you see, to measure it", **measured)
        if not inside:
            return _false(f"{self.name} — all of it, not just its centre — is "
                          f"not {where}", **measured)
        if gripper is None:
            return _unknown(f"{self.name} is {where}, but the {self.side} "
                            f"gripper reports nothing, so whether it was "
                            f"released cannot be measured", **measured)
        if gripper.holding and (not gripper.held_object
                                or gripper.held_object == self.name):
            return _false(f"{self.name} is {where} but the {self.side} gripper "
                          f"is still holding it", **measured)
        if abs(rest) > RESTING_TOL_M + self.pad_m:
            return _false(
                f"{self.name} is {where} and released, but its underside is "
                f"{rest * 1000:+.0f} mm from the floor it should be standing "
                f"on — it is in the air or through the bottom, not set down",
                **measured)
        return _true(f"{self.name} is {where}, released, and standing on its "
                     f"floor ({rest * 1000:+.0f} mm). Note: nothing on this "
                     f"robot publishes a velocity, so 'supported' is what is "
                     f"measured here, not 'has come to rest'", **measured)


class JointsAt(Verifier):
    """Both/one arm's joints reached a named posture, in degrees of error."""

    describes = "the arm reached the posture"

    def __init__(self, primitive: str, world0: WorldView,
                 targets, tol_deg: float = 3.0):
        super().__init__(primitive, world0)
        self.targets = {side: np.asarray(q, dtype=float).reshape(7)
                        for side, q in dict(targets).items()}
        self.tol_deg = float(tol_deg)

    def measure(self, world1: WorldView) -> VerdictReport:
        worst = 0.0
        measured = {}
        for side, q_goal in self.targets.items():
            arm = world1.arm(side)
            if arm is None:
                return _unknown(f"the {side} arm is not in the later observation")
            err = float(np.degrees(np.max(np.abs(arm.joints - q_goal))))
            measured[f"{side}_worst_joint_deg"] = round(err, 2)
            worst = max(worst, err)
        if worst <= self.tol_deg:
            return _true(f"every joint is within {worst:.1f} deg of the posture",
                         **measured)
        return _false(f"the worst joint is {worst:.1f} deg from the posture "
                      f"(tolerance {self.tol_deg:.1f} deg)", **measured)


class Tilted(Verifier):
    """A pour: the source tilted, and — only where instrumented — it emptied.

    The honest verdict for the D1 as it stands is ``UNKNOWN`` once the tilt is
    confirmed: nothing on this robot weighs the pot or reads the cup's level.
    Saying ``TRUE`` because the arm rotated is exactly the failure that makes a
    measured verifier worth having; saying ``FALSE`` would call a successful
    pour a failure. So it says it does not know, and names what would settle it.
    """

    describes = "the source tilted; whether liquid arrived is not instrumented"

    def __init__(self, primitive: str, world0: WorldView, source: str,
                 target: str, tilt_rad: float):
        super().__init__(primitive, world0)
        self.source = source
        self.target = target
        self.tilt_rad = float(tilt_rad)
        self.r0 = self._tilt(world0)

    def _tilt(self, world: WorldView) -> Optional[float]:
        item = world.find(self.source)
        if item is None:
            return None
        try:
            r = item.pose_in_base(world.frames)[1]
        except LookupError:
            return None
        # angle between the source's own +z and world up
        axis = r.as_matrix()[:, 2]
        return float(math.acos(max(-1.0, min(1.0, float(axis[2])))))

    def measure(self, world1: WorldView) -> VerdictReport:
        now = self._tilt(world1)
        if now is None or self.r0 is None:
            return _unknown(f"{self.source} is not in both observations, so the "
                            f"tilt cannot be compared")
        change = now - self.r0
        measured = {"tilt_deg": round(math.degrees(now), 1),
                    "tilt_change_deg": round(math.degrees(change), 1),
                    "asked_deg": round(math.degrees(self.tilt_rad), 1)}
        if change < self.tilt_rad * MOVED_FRACTION:
            return _false(f"{self.source} tilted {math.degrees(change):.0f} deg "
                          f"of the {math.degrees(self.tilt_rad):.0f} deg asked",
                          **measured)
        return _unknown(
            f"{self.source} tilted {math.degrees(change):.0f} deg over "
            f"{self.target}, but nothing on this robot measures whether liquid "
            f"arrived — instrument the source's mass or the target's level to "
            f"turn this into a verdict", **measured)


#: How far a published contact surface's face may sit from the contact it
#: was fitted to and still be that contact's surface [m]. A single probe's
#: plane passes through its contact exactly; a fit over three leaves a
#: residual, and 3 mm is the hardware gate's own accuracy target (decision 2).
CONTACT_PLANE_TOL_M = 0.003


def _new_contacts(world0: WorldView, world1: WorldView, side: str,
                  verb: str = ""):
    """The contacts ``world1`` carries that ``world0`` did not, for ``side``."""
    base = len(getattr(world0, "contacts", ()))
    fresh = tuple(getattr(world1, "contacts", ()))[base:]
    return [c for c in fresh if c.side == side and (not verb or c.verb == verb)]


class ContactMade(Verifier):
    """The hand met resistance along its direction — MEASURED by the leg.

    Reads the :class:`~manipulation_kit.world.ContactView` a run's
    :class:`~manipulation_kit.executor.ContactReport` became
    (``primitives.contact.record_contacts``). No new contact in the later
    world is UNKNOWN — nobody folded the evidence in — never TRUE; a contact
    leg that ran its whole travel is FALSE, with the travel.
    """

    describes = "the hand touched something, measured by the torque watch"

    def __init__(self, primitive: str, world0: WorldView, side: str, *,
                 max_travel_m: float, verb: str = ""):
        super().__init__(primitive, world0)
        self.side = side
        self.max_travel_m = float(max_travel_m)
        self.verb = verb or primitive

    def measure(self, world1: WorldView) -> VerdictReport:
        new = _new_contacts(self.world0, world1, self.side, self.verb)
        if not new:
            return _unknown(
                f"no {self.verb} contact on the {self.side} hand reached this "
                f"world; fold the run's report in "
                f"(primitives.contact.record_contacts)")
        c = new[-1]
        measured = {"stopped_by": c.stopped_by,
                    "p": [round(float(v), 4) for v in c.p],
                    "normal": [round(float(v), 3) for v in c.normal],
                    "travel_m": (None if not math.isfinite(c.travel_m)
                                 else round(float(c.travel_m), 4)),
                    "torque_nm": (None if not math.isfinite(c.torque_nm)
                                  else round(float(c.torque_nm), 3))}
        if not c.made:
            travel = (f"{c.travel_m * 1000:.0f} mm" if math.isfinite(c.travel_m)
                      else "its whole leg")
            return _false(f"nothing resisted the {self.side} hand: the leg "
                          f"stopped by {c.stopped_by} after {travel}",
                          **measured)
        if (math.isfinite(c.travel_m)
                and not -0.005 <= c.travel_m <= self.max_travel_m + 0.005):
            return _false(f"the {self.side} contact is {c.travel_m * 1000:.0f} "
                          f"mm along a leg of {self.max_travel_m * 1000:.0f} "
                          f"mm, which is not a point on it", **measured)
        return _true(f"the {self.side} hand met resistance after "
                     f"{c.travel_m * 1000:.0f} mm ({c.torque_nm:.2f} Nm rise)",
                     **measured)


class SurfaceMeasured(Verifier):
    """A SURFACE named ``name`` now carries the plane the contact measured."""

    describes = "the touched surface was published with its measured plane"

    def __init__(self, primitive: str, world0: WorldView, side: str, name: str,
                 *, tol_m: float = CONTACT_PLANE_TOL_M, verb: str = ""):
        super().__init__(primitive, world0)
        self.side = side
        self.name = name
        self.tol_m = float(tol_m)
        self.verb = verb or primitive

    def measure(self, world1: WorldView) -> VerdictReport:
        new = _new_contacts(self.world0, world1, self.side, self.verb)
        if not new:
            return _unknown(f"no {self.verb} contact reached this world, so "
                            f"no surface was measured")
        c = new[-1]
        if not c.made:
            return _false(f"nothing was touched, so {self.name!r} was not "
                          f"measured")
        surface = world1.find(self.name)
        if not isinstance(surface, SurfaceView):
            return _false(f"no surface called {self.name!r} was published")
        if surface.plane_source != "contact":
            return _false(f"{self.name!r} is known from "
                          f"{surface.plane_source or 'an unstated source'}, "
                          f"not from contact")
        try:
            off = surface.plane_offset(c.p, world1.frames)
            normal = surface.top_normal(world1.frames)
        except FrameError as exc:
            return _unknown(f"{self.name!r} does not resolve: {exc}")
        measured = {"offset_m": round(off, 4),
                    "normal": [round(float(v), 4) for v in normal],
                    "height_uncertainty_m": surface.height_uncertainty_m}
        if abs(off) > self.tol_m:
            return _false(f"{self.name!r}'s face is {off * 1000:+.1f} mm from "
                          f"the contact it should pass through", **measured)
        return _true(f"{self.name!r} passes {abs(off) * 1000:.1f} mm from the "
                     f"measured contact", **measured)


class Never(Verifier):
    """For a primitive whose success is not measurable here at all."""

    describes = "not measurable on this robot"

    def __init__(self, primitive: str, world0: WorldView, why: str):
        super().__init__(primitive, world0)
        self.why = why

    def measure(self, world1: WorldView) -> VerdictReport:
        return _unknown(self.why)


class All(Verifier):
    """Every part must hold; UNKNOWN anywhere makes the whole UNKNOWN.

    The order matters: a FALSE beats an UNKNOWN, because evidence of failure is
    still evidence, while an unmeasurable half cannot rescue a measured miss.
    """

    def __init__(self, primitive: str, world0: WorldView,
                 parts: Sequence[Verifier]):
        super().__init__(primitive, world0)
        self.parts = tuple(parts)
        self.describes = "; ".join(p.describes for p in self.parts)

    def measure(self, world1: WorldView) -> VerdictReport:
        return self.measure_run(world1, None)

    def measure_run(self, world1: WorldView, run: Any = None) -> VerdictReport:
        reports = [p(world1, run) for p in self.parts]
        measured = {}
        for part, report in zip(self.parts, reports):
            measured[type(part).__name__] = report.to_json()
        for report in reports:
            if report.verdict == Verdict.FALSE:
                return VerdictReport(Verdict.FALSE, report.reason, measured)
        for report in reports:
            if report.verdict == Verdict.UNKNOWN:
                return VerdictReport(Verdict.UNKNOWN, report.reason, measured)
        return VerdictReport(Verdict.TRUE,
                             "; ".join(r.reason for r in reports), measured)
