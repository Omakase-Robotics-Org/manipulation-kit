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

from ..world import ContainerView, ObjectView, SurfaceView, WorldView
from .types import Verdict, VerdictReport, Verifier

#: how close a measured tool point must be to a commanded one to count
TOOL_TOL_M = 0.02
#: fraction of a commanded displacement that must actually happen
MOVED_FRACTION = 0.7
#: a gripper at or below this closedness counts as open
OPEN_CLOSEDNESS = 0.15


def _unknown(reason: str, **measured) -> VerdictReport:
    return VerdictReport(Verdict.UNKNOWN, reason, measured)


def _true(reason: str, **measured) -> VerdictReport:
    return VerdictReport(Verdict.TRUE, reason, measured)


def _false(reason: str, **measured) -> VerdictReport:
    return VerdictReport(Verdict.FALSE, reason, measured)


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
                 tol_m: float = TOOL_TOL_M):
        super().__init__(primitive, world0)
        self.side = side
        self.delta = np.asarray(delta, dtype=float).reshape(3)
        self.tol_m = float(tol_m)
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


class Holding(Verifier):
    """The gripper's own torque-stop report says something is held.

    ``GripperReport.holding`` is the measurement: the closing stroke met an
    object and stopped squeezing at the preset's stop torque. The jaw gap is
    checked beside it because a gripper that closed all the way is holding
    nothing while still reporting a stall against itself.
    """

    describes = "the gripper reports holding, at a gap the object could make"

    def __init__(self, primitive: str, world0: WorldView, side: str,
                 obj: Optional[ObjectView] = None):
        super().__init__(primitive, world0)
        self.side = side
        self.obj = obj

    def measure(self, world1: WorldView) -> VerdictReport:
        gripper = world1.gripper(self.side)
        if gripper is None:
            return _unknown(f"no gripper report for the {self.side} hand")
        measured = {"holding": bool(gripper.holding),
                    "closedness": round(float(gripper.closedness), 3)}
        if gripper.jaw_gap_m is not None:
            measured["jaw_gap_m"] = round(float(gripper.jaw_gap_m), 4)
        if not gripper.holding:
            return _false(f"the {self.side} gripper reports nothing held", **measured)
        if self.obj is not None and gripper.jaw_gap_m is not None:
            width = self.obj.min_horizontal_extent()
            if gripper.jaw_gap_m < width * 0.4:
                return _false(
                    f"the {self.side} gripper stalled at "
                    f"{gripper.jaw_gap_m * 1000:.0f} mm, far inside "
                    f"{self.obj.name}'s {width * 1000:.0f} mm — it closed on "
                    f"itself, not on the object", **measured)
        return _true(f"the {self.side} gripper reports holding", **measured)


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
    """The OBJECT went up — not the hand. Lift is about the thing, not the arm."""

    describes = "the object is higher than it was, and still held"

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
            return _unknown(f"{self.name} is not in the later observation, so "
                            f"its height cannot be compared")
        rise = float(now[2] - self.z0[2])
        need = self.height_m * MOVED_FRACTION
        gripper = world1.gripper(self.side)
        measured = {"rise_m": round(rise, 4), "asked_m": round(self.height_m, 4),
                    "holding": None if gripper is None else bool(gripper.holding)}
        if gripper is not None and not gripper.holding:
            return _false(f"{self.name} rose {rise * 1000:.0f} mm but the "
                          f"{self.side} gripper is no longer holding it", **measured)
        if rise >= need:
            return _true(f"{self.name} rose {rise * 1000:.0f} mm", **measured)
        return _false(f"{self.name} rose {rise * 1000:.0f} mm of the "
                      f"{self.height_m * 1000:.0f} mm asked", **measured)


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
        if here is None or there is None:
            return _unknown(f"{self.name} or {self.destination} is not in the "
                            f"later observation")
        gap = float(np.linalg.norm(here[:2] - there[:2]))
        gripper = world1.gripper(self.side)
        measured = {"horizontal_gap_m": round(gap, 4),
                    "holding": None if gripper is None else bool(gripper.holding)}
        if gripper is not None and not gripper.holding:
            return _false(f"the {self.side} gripper dropped {self.name} on the way",
                          **measured)
        if gap <= self.tol_m:
            return _true(f"{self.name} is {gap * 1000:.0f} mm from over "
                         f"{self.destination}", **measured)
        return _false(f"{self.name} is {gap * 1000:.0f} mm from over "
                      f"{self.destination}", **measured)


class ObjectIn(Verifier):
    """The object ended up inside a container, or on a surface, and was let go.

    Both halves are required. An object still in the jaws above the box is not
    placed, and a released object beside the box is not placed either.
    """

    describes = "the object is in/on the destination and no longer held"

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
        if obj is None or target is None:
            return _unknown(f"{self.name} or {self.destination} is not in the "
                            f"later observation")
        try:
            p = obj.pose_in_base(world1.frames)[0]
        except LookupError as exc:
            return _unknown(f"{self.name}'s frame could not be resolved: {exc}")
        gripper = world1.gripper(self.side)
        measured = {"p": [round(float(v), 4) for v in p],
                    "holding": None if gripper is None else bool(gripper.holding)}
        if isinstance(target, ContainerView):
            inside = target.contains(p, world1.frames, pad_m=self.pad_m)
            where = f"inside {self.destination}"
        elif isinstance(target, SurfaceView):
            inside = target.supports(p, world1.frames, pad_m=self.pad_m)
            where = f"on {self.destination}"
        else:
            return _unknown(f"{self.destination} is a plain object, not a "
                            f"container or a surface — 'placed in' has no "
                            f"measurable meaning for it")
        measured["inside"] = bool(inside)
        if not inside:
            return _false(f"{self.name} is not {where}", **measured)
        if gripper is not None and gripper.holding:
            return _false(f"{self.name} is {where} but the {self.side} gripper "
                          f"is still holding it", **measured)
        return _true(f"{self.name} is {where}", **measured)


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
