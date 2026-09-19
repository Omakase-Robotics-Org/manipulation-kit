"""The primitive contract: what a verb is, what it may refuse with, how it is judged.

One page, because the value of this package is that every verb obeys the SAME
three-part contract and a consumer can therefore treat them interchangeably:

``preconditions(world) -> list[Unmet]``
    Cheap, no kinematics. "There is no such object", "that hand is already
    holding something", "the frame it was measured in is stale". A non-empty
    list means the verb is not applicable *as asked* — which is a different
    thing from "the arm cannot get there", and is reported separately so a
    caller can tell a naming mistake from a reach problem.

``plan(world, kin) -> Plan | PlanError``
    PURE. Pose waypoints, each solved with
    :meth:`~manipulation_kit.arms.kinematics.GuardedArm.solve_ee` (so each one
    passes the same IK, the same joint-step clamp and the same collision guard
    the teleop stack runs), producing a joint path that is fully checked BEFORE
    the first joint moves. It leaves ``kin`` exactly as it found it. A refusal
    is a :class:`PlanError` naming the reason, the waypoint index and the
    residual — never a silent no-op, because a refusal nobody can see is the
    failure this whole package was written after (dx-inspect-robots PR #17:
    the guard stopped a move after 19 mm and the model was told, thirty times,
    that the move executed).

``verifier(world_before) -> Verifier``
    A callable over a LATER world that returns a MEASURED verdict. Never
    ``True`` by default: a verifier handed an unchanged world must say
    ``FALSE``, and one whose evidence does not exist on this robot must say
    ``UNKNOWN``. "The model said it was done" is not evidence.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, fields
from typing import Any, Dict, List, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from ..arms import sides
from ..world import WorldView

# --------------------------------------------------------------------------- #
# vocabulary
# --------------------------------------------------------------------------- #

#: logical sides plus AUTO, which asks the kit to choose (and then says which)
SIDES: Tuple[str, ...] = sides.SIDES
AUTO = "auto"
#: ``GoHome`` is the one verb that means something for BOTH arms at once —
#: everything else acts with one hand. It is a per-verb widening of the ``side``
#: domain rather than a global one, declared through
#: :meth:`Primitive.arg_enums` so the schema exports pick it up automatically.
#: (They did not, once, and the drift gate is what said so.)
BOTH = "both"
SIDE_CHOICES: Tuple[str, ...] = SIDES + (AUTO,)
GOHOME_SIDE_CHOICES: Tuple[str, ...] = SIDES + (AUTO, BOTH)

#: The named approach set. This is the WHOLE of the orientation vocabulary a
#: model gets: it names one of these four, and the kit derives the quaternion
#: from it plus the object's principal axis. See :mod:`.approach` for why.
TOP_DOWN = "top_down"
FRONT = "front"
SIDE_LEFT = "side_left"
SIDE_RIGHT = "side_right"
APPROACHES: Tuple[str, ...] = (TOP_DOWN, FRONT, SIDE_LEFT, SIDE_RIGHT)

#: firmware ``GripPreset`` — the presets own the stop torque (d1-firmware #55),
#: which is why a number is not offered here.
GRIPS: Tuple[str, ...] = ("soft", "firm", "strong")

#: The correction grid, metres. Coarse AND fine in the same menu: a task that
#: needs a 30 mm correction fails when 50 mm is the only offer (Raptor's Jev
#: run, 2026-09-19), and a menu of only fine steps costs turns.
NUDGE_GRID_M: Tuple[float, ...] = (0.010, 0.030, 0.050)
#: the only rotation a model may ask for, about the approach axis
NUDGE_MAX_YAW_RAD = math.radians(15.0)
#: ``Nudge`` frames: the tool's own axes, or the robot base
NUDGE_FRAMES: Tuple[str, ...] = ("tool", "base")

# --------------------------------------------------------------------------- #
# refusal reasons
# --------------------------------------------------------------------------- #

#: the three the kit's guarded IK already produces, verbatim
IK_FAIL = "ik_fail"
INFEASIBLE = "infeasible"
GUARD_REJECT = "guard_reject"
#: ...and the ones that are about the WORLD rather than the arm
UNREACHABLE_OBJECT = "unreachable_object"
NO_SUCH_OBJECT = "no_such_object"
FRAME_STALE = "frame_stale"
UNKNOWN_FRAME = "unknown_frame"
PRECONDITION_UNMET = "precondition_unmet"
#: a primitive whose body is a learned policy, asked to produce a kinematic plan
LEARNED_POLICY_REQUIRED = "learned_policy_required"

PLAN_REASONS: Tuple[str, ...] = (
    IK_FAIL, INFEASIBLE, GUARD_REJECT, UNREACHABLE_OBJECT, NO_SUCH_OBJECT,
    FRAME_STALE, UNKNOWN_FRAME, PRECONDITION_UNMET, LEARNED_POLICY_REQUIRED)


@dataclass(frozen=True)
class Unmet:
    """One unsatisfied precondition, in words a model can act on."""

    code: str
    detail: str = ""
    #: what would satisfy it, when there is a mechanical answer
    remedy: str = ""

    def __str__(self) -> str:
        text = f"{self.code}: {self.detail}" if self.detail else self.code
        return f"{text} ({self.remedy})" if self.remedy else text


# --------------------------------------------------------------------------- #
# plans
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Waypoint:
    """One TOOL-POINT pose the plan passes through, in the base frame.

    The tool point is the jaw pocket — the pad CENTRE, 100 mm along the TCP
    frame's +z from the flange (``hands.d1.parallel_gripper.description
    .PAD_CENTRE_Z_M``, MEASURED on d1-3 2026-09-16). "Put the tool on the
    block" therefore means what it says; converting to the Link7 pose the IK
    takes is :mod:`.planning`'s job and happens once.
    """

    label: str
    p: np.ndarray
    r: R

    def __post_init__(self) -> None:
        object.__setattr__(self, "p", np.asarray(self.p, dtype=float).reshape(3))

    def to_json(self) -> Dict[str, Any]:
        return {"label": self.label,
                "p": [round(float(v), 4) for v in self.p],
                "quat_xyzw": [round(float(v), 4) for v in self.r.as_quat()]}


@dataclass(frozen=True)
class JointStep:
    """One dual-arm-safe joint command for ``side``, already guard-checked."""

    side: str
    q: np.ndarray                 # 7 joint angles [rad]
    waypoint: int                 # which Waypoint this step is converging on

    def __post_init__(self) -> None:
        object.__setattr__(self, "q", np.asarray(self.q, dtype=float).reshape(7))


@dataclass(frozen=True)
class GripStep:
    """A gripper stroke, run to completion before the next joint step."""

    side: str
    closedness: float             # 0 open .. 1 closed
    grip: str = "soft"
    waypoint: int = -1


@dataclass(frozen=True)
class SettleStep:
    """Wait until the arms are stationary. A verifier run on a moving robot
    measures the middle of the motion, which is not a verdict."""

    timeout_s: float = 2.0


Step = Any  # JointStep | GripStep | SettleStep


@dataclass(frozen=True)
class Plan:
    """A fully pre-checked motion: every joint step already passed IK + guard.

    ``steps`` is the whole of what an :class:`~manipulation_kit.executor.Executor`
    runs, in order. A plan is a value: building it moved nothing and re-running
    ``plan()`` on the same world gives the same plan.
    """

    primitive: str
    side: str
    waypoints: Tuple[Waypoint, ...] = ()
    steps: Tuple[Step, ...] = ()
    #: anything the caller should be told that is not a refusal (e.g. "side
    #: chosen automatically", "approach yaw taken from the principal axis")
    notes: Tuple[str, ...] = ()

    @property
    def ok(self) -> bool:
        return True

    def joint_steps(self) -> Tuple[JointStep, ...]:
        return tuple(s for s in self.steps if isinstance(s, JointStep))

    def duration_s(self, hz: float = 50.0) -> float:
        return len(self.joint_steps()) / float(hz)

    def to_json(self) -> Dict[str, Any]:
        return {
            "primitive": self.primitive, "side": self.side,
            "waypoints": [w.to_json() for w in self.waypoints],
            "joint_steps": len(self.joint_steps()),
            "grip_steps": [{"side": s.side, "closedness": s.closedness,
                            "grip": s.grip}
                           for s in self.steps if isinstance(s, GripStep)],
            "notes": list(self.notes),
        }

    def __bool__(self) -> bool:
        return True


@dataclass(frozen=True)
class PlanError:
    """Why no plan exists — with the waypoint that failed and by how much.

    The residual is the load-bearing half. "guard_reject" alone leaves a model
    guessing; "guard_reject at waypoint 1 (grasp), 0.019 m short" tells it to
    try another approach, and is exactly the line that was missing when the
    guard silently stopped a move 19 mm in.
    """

    reason: str
    detail: str = ""
    waypoint_index: int = -1
    waypoint_label: str = ""
    residual_m: float = float("nan")
    primitive: str = ""
    side: str = ""
    unmet: Tuple[Unmet, ...] = ()

    def __post_init__(self) -> None:
        if self.reason not in PLAN_REASONS:
            raise ValueError(f"unknown plan-refusal reason {self.reason!r}; "
                             f"the vocabulary is {PLAN_REASONS}")

    @property
    def ok(self) -> bool:
        return False

    def __bool__(self) -> bool:
        return False

    def to_json(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {"reason": self.reason, "detail": self.detail,
                               "primitive": self.primitive, "side": self.side}
        if self.waypoint_index >= 0:
            out["waypoint_index"] = self.waypoint_index
            out["waypoint_label"] = self.waypoint_label
        if math.isfinite(self.residual_m):
            out["residual_m"] = round(float(self.residual_m), 4)
        if self.unmet:
            out["unmet"] = [str(u) for u in self.unmet]
        return out

    def __str__(self) -> str:
        where = (f" at waypoint {self.waypoint_index} ({self.waypoint_label})"
                 if self.waypoint_index >= 0 else "")
        gap = (f", {self.residual_m * 1000:.0f} mm short"
               if math.isfinite(self.residual_m) else "")
        head = f"{self.primitive or 'primitive'} refused: {self.reason}{where}{gap}"
        return f"{head} — {self.detail}" if self.detail else head


# --------------------------------------------------------------------------- #
# verdicts
# --------------------------------------------------------------------------- #

class Verdict(str):
    """TRUE / FALSE / UNKNOWN, as a string so it serialises with no adapter."""

    TRUE: "Verdict"
    FALSE: "Verdict"
    UNKNOWN: "Verdict"


Verdict.TRUE = Verdict("true")
Verdict.FALSE = Verdict("false")
Verdict.UNKNOWN = Verdict("unknown")
VERDICTS: Tuple[Verdict, ...] = (Verdict.TRUE, Verdict.FALSE, Verdict.UNKNOWN)


@dataclass(frozen=True)
class VerdictReport:
    """The verdict and the numbers it was read off.

    ``measured`` is kept so a trace record can be argued with later: a verifier
    that says FALSE and cannot say what it measured is not much better than a
    model that says TRUE.
    """

    verdict: Verdict
    reason: str = ""
    measured: Dict[str, Any] = field(default_factory=dict)

    def __bool__(self) -> bool:
        return self.verdict == Verdict.TRUE

    def to_json(self) -> Dict[str, Any]:
        return {"verdict": str(self.verdict), "reason": self.reason,
                "measured": dict(self.measured)}


class Verifier:
    """Callable over a LATER :class:`~manipulation_kit.world.WorldView`.

    Built from the world BEFORE the primitive ran, so it can measure a
    difference rather than a state. Subclasses implement :meth:`measure`.
    """

    #: what this verifier checks, one line, for a menu or a trace
    describes = "nothing"

    def __init__(self, primitive: str, world0: WorldView):
        self.primitive = primitive
        self.world0 = world0

    def __call__(self, world1: WorldView) -> VerdictReport:
        return self.measure(world1)

    def measure(self, world1: WorldView) -> VerdictReport:  # pragma: no cover
        raise NotImplementedError

    def unchanged(self, world1: WorldView) -> bool:
        """True when nothing this verifier cares about moved.

        The default is the one that makes G6 hold: a verifier called with the
        world it was built from must not return TRUE.
        """
        return world1 is self.world0


# --------------------------------------------------------------------------- #
# the primitive base
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Primitive:
    """One verb. Frozen, so a plan and its trace record cannot disagree later.

    ``VERB`` is a plain class attribute and deliberately NOT a dataclass field:
    a field would take the first positional slot of every subclass, so
    ``Grasp("red_block")`` would bind the verb name instead of the object.
    """

    #: the name a model says. Defaults to the class name lowercased, so the
    #: two cannot drift.
    VERB = ""

    @classmethod
    def name(cls) -> str:
        return cls.VERB or cls.__name__.lower()

    @classmethod
    def arguments(cls) -> Tuple[str, ...]:
        """The dataclass fields a caller binds, in declaration order."""
        return tuple(f.name for f in fields(cls))

    @classmethod
    def arg_enums(cls) -> Dict[str, Tuple[str, ...]]:
        """Per-verb narrowing/widening of an argument's closed set.

        The kit owns this, not the schema exporter: a verb that accepts a value
        no exported schema mentions is a robot with two different action spaces
        depending on which model is driving.
        """
        return {}

    def preconditions(self, world: WorldView) -> List[Unmet]:  # pragma: no cover
        raise NotImplementedError

    def plan(self, world: WorldView, kin) -> Any:              # pragma: no cover
        raise NotImplementedError

    def verifier(self, world0: WorldView) -> Verifier:         # pragma: no cover
        raise NotImplementedError

    # -- shared helpers ---------------------------------------------------- #
    def _unmet_error(self, unmet: Sequence[Unmet], side: str = "") -> PlanError:
        """Turn a precondition failure into the matching typed plan refusal.

        A missing object and a stale frame keep their own reasons rather than
        collapsing into ``precondition_unmet``: they are the two a caller can
        actually do something about.
        """
        codes = {u.code for u in unmet}
        for reason in (NO_SUCH_OBJECT, FRAME_STALE, UNKNOWN_FRAME):
            if reason in codes:
                first = next(u for u in unmet if u.code == reason)
                return PlanError(reason, first.detail, primitive=self.name(),
                                 side=side, unmet=tuple(unmet))
        return PlanError(PRECONDITION_UNMET, "; ".join(str(u) for u in unmet),
                         primitive=self.name(), side=side, unmet=tuple(unmet))


class LearnedPrimitive(Primitive):
    """Marker: this verb's BODY is a learned policy, not a kinematic plan.

    It keeps the whole contract — the kit still checks its preconditions and
    still owns its measured verifier — but :meth:`Primitive.plan` returns
    ``PlanError(learned_policy_required)`` and the executor that can run it
    lives where the policy lives (``d1-inference``), because ``omakase-core``
    must not depend on that package and the kit must not grow a model runtime.

    Shu, 2026-09-19: 「Pour は ACT」. Astra orchestrates at the VLA's level and
    the learned policy is the body of one verb.

    Subclasses declare their own ``policy`` FIELD (``"act:pourwithsmallpotjp"``
    — the executor resolves it, the kit does not). It is not declared here
    because a base-class field takes the first positional slot of every
    subclass, and ``Pour("kettle", "cup")`` naming a checkpoint would be a
    trap.
    """


PRIMITIVE_CONTRACT = __doc__
