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
from typing import Optional, Sequence

import numpy as np
from scipy.spatial.transform import Rotation as R

from ..world import (ContainerView, FrameError, ObjectView, SurfaceView,
                     WorldView)
from .approach import JAW_CLEARANCE_M
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
#: (``approach.JAW_CLEARANCE_M``, 4 mm) rather than a second number: the kit
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


def grip_width_window(width_m: float, tol_m: float = GRIP_WIDTH_TOL_M):
    """The pad gaps that are ``width_m`` held, rather than air or the pads.

    Symmetric on purpose: BELOW the window the jaws travelled past the object
    (they are on themselves, or on something thinner), ABOVE it they never
    reached it.
    """
    width = float(width_m)
    return max(0.0, width - float(tol_m)), width + float(tol_m)


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
                 dyaw_rad: float, axis=None, tol_rad: Optional[float] = None):
        super().__init__(primitive, world0)
        self.side = side
        self.dyaw_rad = float(dyaw_rad)
        self.axis = None if axis is None else np.asarray(axis, dtype=float)
        self.tol_rad = turn_tol(dyaw_rad) if tol_rad is None else float(tol_rad)
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
        err = abs(turned - self.dyaw_rad)
        measured = {"turned_deg": round(math.degrees(turned), 2),
                    "asked_deg": round(math.degrees(self.dyaw_rad), 2),
                    "off_axis_deg": round(math.degrees(float(np.linalg.norm(
                        delta - local * turned))), 2),
                    "tolerance_deg": round(math.degrees(self.tol_rad), 2)}
        if err <= self.tol_rad:
            return _true(f"the {self.side} hand turned "
                         f"{math.degrees(turned):+.1f} deg about its approach "
                         f"axis as asked", **measured)
        return _false(f"the {self.side} hand turned "
                      f"{math.degrees(turned):+.1f} deg of the "
                      f"{math.degrees(self.dyaw_rad):+.1f} deg asked",
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
    3. **The gap is one the named object could make**: within
       :data:`GRIP_WIDTH_TOL_M` of its own narrowest width. Below the window the
       jaws went past it (closed on themselves, or on something thinner); above
       it they never reached it.

    Every part is optional-if-unmeasured and never optional-if-measured: a
    producer that reports no gap gets (1) and (2) — the documented fallback for
    an object of unknown size — and a producer that reports nothing at all gets
    UNKNOWN, never TRUE.
    """

    describes = "the jaws stalled on the object, at a gap its own width could make"

    def __init__(self, primitive: str, world0: WorldView, side: str,
                 obj: Optional[ObjectView] = None,
                 tol_m: float = GRIP_WIDTH_TOL_M, *, jaw_axis=None):
        super().__init__(primitive, world0)
        self.side = side
        self.obj = obj
        self.tol_m = float(tol_m)
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

    def measure(self, world1: WorldView) -> VerdictReport:
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
        if width is not None and gripper.jaw_gap_m is not None:
            low, high = grip_width_window(width, self.tol_m)
            measured["width_window_m"] = [round(low, 4), round(high, 4)]
            measured["object_width_m"] = round(float(width), 4)
            if gripper.jaw_gap_m < low:
                return _false(
                    f"the {self.side} gripper stalled at "
                    f"{gripper.jaw_gap_m * 1000:.1f} mm, inside "
                    f"{name}'s {width * 1000:.1f} mm — it closed on "
                    f"itself, not on the object", **measured)
            if gripper.jaw_gap_m > high:
                return _false(
                    f"the {self.side} gripper stopped at "
                    f"{gripper.jaw_gap_m * 1000:.1f} mm, wider than "
                    f"{name}'s {width * 1000:.1f} mm — the jaws never "
                    f"reached it", **measured)
        # (2) the half only the producer can see: a body between the pad faces
        if not gripper.holding:
            return _false(f"the {self.side} gripper reports nothing between its "
                          f"pads", **measured)
        gap = ("" if gripper.jaw_gap_m is None
               else f" at a {gripper.jaw_gap_m * 1000:.1f} mm gap")
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
        if distance > ASSOCIATION_TOL_M:
            return _false(
                f"the {self.side} jaws stalled on something, but {name!r} is "
                f"measured {distance * 1000:.0f} mm from the tool point — "
                f"whatever is between the pads, it is not that", **measured)
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
        return _true(f"{self.name} rose {rise * 1000:.0f} mm", **measured)


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
        if gap <= self.tol_m:
            return _true(f"{self.name} is {gap * 1000:.0f} mm from over "
                         f"{self.destination}", **measured)
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
        if gap >= self.margin_m:
            return _true(f"{self.name}'s underside is {gap * 1000:.0f} mm above "
                         f"{self.destination}'s {what}", **measured)
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
        reports = [p(world1) for p in self.parts]
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
