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
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np
from scipy.spatial.transform import Rotation as R

from ..arms import safety, sides
from ..world import WorldView
from ..world.direction import BASE as _BASE, TOOL as _TOOL

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

#: The grasp directions a planner TRIES, in order, when it is choosing one —
#: aliases of :data:`manipulation_kit.world.ALIASES` (the way the tool travels
#: onto the object). ONE constant, so the prompt, the example's hand chooser
#: and the offer generator cannot disagree about the set or its order again.
#: It is a search order, not the vocabulary: a verb takes any
#: :class:`~manipulation_kit.world.Direction`.
GRASP_DIRECTIONS: Tuple[str, ...] = ("down", "forward", "left", "right")

#: firmware ``GripPreset`` — the presets own the stop torque (d1-firmware #55),
#: which is why a number is not offered here.
GRIPS: Tuple[str, ...] = ("soft", "firm", "strong")

#: The correction grid, metres. Coarse AND fine in the same menu: a task that
#: needs a 30 mm correction fails when 50 mm is the only offer (Raptor's Jev
#: run, 2026-09-19), and a menu of only fine steps costs turns.
NUDGE_GRID_M: Tuple[float, ...] = (0.010, 0.030, 0.050)
#: the only rotation a model may ask for, about the approach axis
NUDGE_MAX_YAW_RAD = math.radians(15.0)
#: ``Nudge`` frames: the tool's own axes, or the robot base — the same frame
#: names a :class:`~manipulation_kit.world.Direction` uses
NUDGE_FRAMES: Tuple[str, ...] = (_TOOL, _BASE)

# --------------------------------------------------------------------------- #
# refusal reasons
# --------------------------------------------------------------------------- #

#: the three the kit's guarded IK already produces, verbatim
IK_FAIL = "ik_fail"
INFEASIBLE = "infeasible"
GUARD_REJECT = "guard_reject"
#: the pose needs more of a joint than the arm has IN THAT POSTURE — a coupled
#: limit (the D1 wrist roll J7 narrows with J6,
#: :mod:`manipulation_kit.arms.coupled_limits`); the detail names it
JOINT_LIMIT = "joint_limit"
#: ...and the ones that are about the WORLD rather than the arm
UNREACHABLE_OBJECT = "unreachable_object"
#: the DESTINATION of a carry/place is outside this arm's reachable set at
#: every transit height the primitive is allowed to try. Distinct from
#: ``ik_fail`` on purpose: ``ik_fail`` is one waypoint the solver could not
#: reach and a different waypoint may work, while this one has already tried
#: the whole ladder and is a statement about the ARM and the destination.
#: The caller's answer is another arm (or moving the destination), never a
#: smaller clearance — measured 2026-09-19, the blocks-eval bin rim plus a
#: constant 100 mm sits 109 mm outside the holding arm's reach.
UNREACHABLE_DESTINATION = "unreachable_destination"
#: no MEETING POSE of a ``handover`` plans for both arms: every candidate of
#: ``reach.HANDOVER_MEETING_POINTS_M`` was tried and each was refused for the
#: giving arm's transit, the receiving arm's approach or grasp, or the giving
#: arm's retreat. Distinct from ``unreachable_destination``: there is no
#: destination the model named — the kit chose every point it tried — so the
#: answer is to move the object (or the robot), not to name another place.
UNREACHABLE_HANDOVER = "unreachable_handover"
NO_SUCH_OBJECT = "no_such_object"
FRAME_STALE = "frame_stale"
UNKNOWN_FRAME = "unknown_frame"
PRECONDITION_UNMET = "precondition_unmet"
#: a primitive whose body is a learned policy, asked to produce a kinematic plan
LEARNED_POLICY_REQUIRED = "learned_policy_required"
#: the observation does not carry what a checked plan needs — a missing arm, a
#: gripper that reports nothing, an object whose shape this v1 does not model.
#: Distinct from ``precondition_unmet``: the ASK is fine, the LOOK is not.
INCOMPLETE_OBSERVATION = "incomplete_observation"
#: the world moved (or the tool changed) between planning and execution
STALE_PLAN = "stale_plan"
#: the geometry is outside what this version models — a tilted box, a
#: non-level support. Refused rather than approximated (R9).
UNSUPPORTED_GEOMETRY = "unsupported_geometry"
#: a bound argument is missing, the wrong type, nonfinite or out of range
BAD_ARGUMENT = "bad_argument"

PLAN_REASONS: Tuple[str, ...] = (
    IK_FAIL, INFEASIBLE, GUARD_REJECT, JOINT_LIMIT, UNREACHABLE_OBJECT,
    UNREACHABLE_DESTINATION, UNREACHABLE_HANDOVER, NO_SUCH_OBJECT, FRAME_STALE,
    UNKNOWN_FRAME, PRECONDITION_UNMET, LEARNED_POLICY_REQUIRED, INCOMPLETE_OBSERVATION,
    STALE_PLAN, UNSUPPORTED_GEOMETRY, BAD_ARGUMENT)

#: Unmet codes the kit itself produces. Open to extension by a consumer, but
#: everything the verbs raise is in here, so a caller can switch on it rather
#: than parse prose. (``Unmet.code`` used to be an open string and
#: ``PlanError.to_json`` flattened each condition into a sentence — R: "Export
#: {code, detail, remedy} objects".)
ALREADY_HOLDING = "already_holding"
NOT_HOLDING = "not_holding"
GRIPPER_UNKNOWN = "gripper_unknown"
ARM_UNKNOWN = "arm_unknown"
OBJECT_TOO_WIDE = "object_too_wide"
OBJECT_TOO_FLAT = "object_too_flat"
OBJECT_TILTED = "object_tilted"
NO_FIT = "no_fit"
NO_MOTION = "no_motion"
BAD_SIDE = "bad_side"
UNMET_CODES: Tuple[str, ...] = (
    ALREADY_HOLDING, NOT_HOLDING, GRIPPER_UNKNOWN, ARM_UNKNOWN,
    OBJECT_TOO_WIDE, OBJECT_TOO_FLAT, OBJECT_TILTED, NO_FIT, NO_MOTION,
    BAD_SIDE, NO_SUCH_OBJECT, FRAME_STALE, UNKNOWN_FRAME, BAD_ARGUMENT)


@dataclass(frozen=True)
class Unmet:
    """One unsatisfied precondition, in words a model can act on.

    It serialises as an OBJECT, not as a sentence. A caller that has to
    ``str()`` a refusal and grep it is a caller that will eventually grep it
    wrong; ``{"code": "already_holding", ...}`` is the same information a
    program can branch on, with the prose beside it rather than instead of it.
    """

    code: str
    detail: str = ""
    #: what would satisfy it, when there is a mechanical answer
    remedy: str = ""
    #: the numbers the condition was read off, units in the key
    measured: Dict[str, Any] = field(default_factory=dict)

    def __str__(self) -> str:
        text = f"{self.code}: {self.detail}" if self.detail else self.code
        return f"{text} ({self.remedy})" if self.remedy else text

    def to_json(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {"code": self.code, "detail": self.detail}
        if self.remedy:
            out["remedy"] = self.remedy
        if self.measured:
            out["measured"] = dict(self.measured)
        return out


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
    #: MAY the planner route around a refusal on the way to this waypoint?
    #:
    #: ``True`` for free-space transit, where a 20-25 cm clearance detour (or
    #: a READY re-seed) is a better answer than a refusal. ``False`` for every
    #: leg whose SHAPE is the promise: a grasp descent must stay in its
    #: approach corridor, a lift must go up, a nudge of 10 mm must not travel
    #: 25 cm to get there, a retreat must back straight out of the container
    #: it is inside. Those are semantic constraints, not endpoint preferences,
    #: and a detour that satisfies the endpoint has not done what was asked
    #: (R10).
    allow_via: bool = True
    #: Must the arm be measurably ON this waypoint — AT THE TOOL — before the
    #: plan goes on?
    #:
    #: The joint-space barrier every plan already gets is a statement about
    #: seven angles; this one is a statement about the jaw pocket, and they
    #: are not the same claim. MEASURED 2026-09-21 on ``blocks-eval``: a
    #: descent that began 2.2-2.8 deg from the commanded posture — inside the
    #: executor's 3 deg, which exists to tolerate the real arm's gravity droop
    #: (F16, J1 ~0.9 deg) — put the tool CENTIMETRES off at a 0.5 m reach, and
    #: the stroke closed the jaws beside the block ("stalled at 13.5 mm inside
    #: block_red's 43.8 mm"). A tolerance loose enough for the droop is too
    #: loose for the jaws.
    #:
    #: ``False`` by default: it costs a measured wait and, when the tool is
    #: off, a correction, so it is spent where the tool point IS the promise —
    #: a grasp's standoff and descent, a place's transit and set-down. Free
    #: transit legs (and ``Approach``, whose verifier already measures the
    #: tool point) do not set it.
    arrive: bool = False
    #: The spacing [m] of this leg's interpolation knots; ``None`` = the
    #: planner's ``safety.MAX_STEP_M``. Only ever FINER: a contact leg is
    #: played until something resists, so the arm stops BETWEEN knots, where
    #: the joint-space interpolation bows off the line (0.2 mm at 25 mm
    #: spacing on the d1-2 probe, in the same direction every probe).
    knot_m: Optional[float] = None

    def __post_init__(self) -> None:
        if self.knot_m is not None and not (0.0 < float(self.knot_m)
                                            <= safety.MAX_STEP_M):
            raise ValueError(
                f"knot_m must be in (0, {safety.MAX_STEP_M}] m, got "
                f"{self.knot_m!r}")
        p = np.array(self.p, dtype=float).reshape(3)
        p.setflags(write=False)
        object.__setattr__(self, "p", p)

    def to_json(self) -> Dict[str, Any]:
        return {"label": self.label,
                "p": [round(float(v), 4) for v in self.p],
                "quat_xyzw": [round(float(v), 4) for v in self.r.as_quat()],
                "allow_via": bool(self.allow_via),
                "arrive": bool(self.arrive),
                **({} if self.knot_m is None else
                   {"knot_m": round(float(self.knot_m), 4)})}


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
    #: the run may go on only if this stroke ends MEASURABLY holding
    #: (``StrokeReport.holding is True``). Set where what follows depends on
    #: the hold — a handover's receiving close, before the giving hand opens:
    #: a receiver that closed on air would otherwise be followed by the giver
    #: dropping the object. Unknown is a stop, not a pass.
    expect_hold: bool = False


@dataclass(frozen=True)
class SettleStep:
    """Wait until the arms are stationary. A verifier run on a moving robot
    measures the middle of the motion, which is not a verdict."""

    timeout_s: float = 2.0


@dataclass(frozen=True)
class ContactCriterion:
    """When to call it contact. MEASURED, never commanded.

    Position mode only (Shu, 2026-09-22): a contact leg is a position-
    commanded motion WATCHED for resistance, not a compliant or torque-mode
    motion. So every number here is a threshold on what the transport
    MEASURES, and none of them is ever sent to a controller.

    ``joint_torque_nm``       the rise, on any one joint of the moving arm,
                              over the torque that joint read BEFORE the
                              motion started [Nm]. A rise, not an absolute:
                              the arm carries its own weight and a held tool.
    ``tool_force_n``          a force at the tool, when a transport can
                              ESTIMATE one [N]. No transport in this package
                              can, so a criterion that sets it is refused
                              (``unmeasured``) rather than silently ignored.
    ``stall_velocity_rad_s``  the arm counts as STOPPED ON something only
                              while no joint moves faster than this — a torque
                              rise on a moving arm is acceleration or gravity,
                              not a surface. Skipped when the transport does
                              not publish velocity.
    ``settle_s``              how long both have to hold before it is called
                              contact [s]. The command is FROZEN while it is
                              being confirmed, so confirming costs no travel.
    """

    joint_torque_nm: float = 4.0
    tool_force_n: Optional[float] = None
    stall_velocity_rad_s: float = 0.02
    settle_s: float = 0.15

    def __post_init__(self) -> None:
        for name in ("joint_torque_nm", "stall_velocity_rad_s", "settle_s"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"ContactCriterion.{name} must be a positive "
                                 f"finite number, got {getattr(self, name)!r}")
            object.__setattr__(self, name, value)
        if self.tool_force_n is not None:
            force = float(self.tool_force_n)
            if not math.isfinite(force) or force <= 0.0:
                raise ValueError(f"ContactCriterion.tool_force_n must be a "
                                 f"positive finite number or None, got "
                                 f"{self.tool_force_n!r}")
            object.__setattr__(self, "tool_force_n", force)

    def to_json(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "joint_torque_nm": round(self.joint_torque_nm, 3),
            "stall_velocity_rad_s": round(self.stall_velocity_rad_s, 4),
            "settle_s": round(self.settle_s, 3)}
        if self.tool_force_n is not None:
            out["tool_force_n"] = round(self.tool_force_n, 3)
        return out


#: The least time between two knots of a contact leg [s]: one 50 Hz tick.
CONTACT_KNOT_MIN_DT_S = 0.02


@dataclass(frozen=True)
class ContactStep:
    """A straight leg that STOPS ON WHATEVER RESISTS, and reports where.

    Run by the executor's ``move_until`` (the protocol's contact capability):
    the leg is position-commanded knot by knot while :attr:`criterion` watches
    the measured torque, and the motion is stopped the moment it is met. An
    executor with no ``move_until`` fails this step with ``transport_error``
    — never plays the leg blind, because a leg played blind is a position-
    controlled arm driven into a table.

    THE LEG'S KNOTS LIVE INSIDE THIS STEP (``path``), not as ``JointStep``s
    beside it, so a runner that does not understand a contact step cannot
    play the leg by accident: it can only refuse the step.

    ``direction``   the travel, RESOLVED into the base frame at plan time.
    ``path``        the leg's joint knots for ``side``, from the posture the
                    leg starts at (``path[0]``) to the one ``max_travel_m``
                    along ``direction`` (``path[-1]``), each already solved by
                    the same IK, clamp and guard as every other plan step.
    ``s``           each knot's distance along ``direction`` from the start [m].
    ``speed_m_s``   how fast the tool travels along the leg. Slow on purpose:
                    what the arm overshoots after contact is this speed times
                    the transport's detection latency.
    ``hold_s``      after contact, keep the command where it stopped this long
                    (a press holds; a probe does not).
    ``retract``     after the hold, travel the leg BACK to its start (a press
                    returns to its standoff). Without it the arm is re-
                    commanded at the posture it MEASURED at contact, so it
                    stops pushing but stays touching.
    ``waypoint``    the index of the plan waypoint the leg ends at.
    """

    side: str
    direction: Any                      # world.Direction, base frame
    max_travel_m: float
    criterion: ContactCriterion = field(default_factory=ContactCriterion)
    waypoint: int = -1
    path: Tuple[np.ndarray, ...] = ()
    s: Tuple[float, ...] = ()
    speed_m_s: float = 0.01
    hold_s: float = 0.0
    retract: bool = False

    def __post_init__(self) -> None:
        path = tuple(np.array(q, dtype=float).reshape(7) for q in self.path)
        for q in path:
            q.setflags(write=False)
        object.__setattr__(self, "path", path)
        object.__setattr__(self, "s", tuple(float(v) for v in self.s))
        if len(self.s) != len(path):
            raise ValueError(f"ContactStep: {len(path)} knots but "
                             f"{len(self.s)} distances")
        if len(path) < 2:
            raise ValueError("ContactStep: a contact leg needs its start and at "
                             "least one knot")
        for k, q in enumerate(path):
            if not np.all(np.isfinite(q)):
                raise ValueError(f"ContactStep: knot {k} is not finite: "
                                 f"{q.tolist()}")
        for k, (a, b) in enumerate(zip(self.s, self.s[1:])):
            if not math.isfinite(b) or b < a:
                raise ValueError(f"ContactStep: the distance along the leg "
                                 f"must not decrease, knot {k + 1} is {b!r} "
                                 f"after {a!r}")
        if getattr(self.direction, "frame", _BASE) != _BASE:
            raise ValueError("ContactStep.direction must be resolved into the "
                             "base frame at plan time")
        for name in ("max_travel_m", "speed_m_s"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"ContactStep.{name} must be positive, got "
                                 f"{getattr(self, name)!r}")
        if not math.isfinite(float(self.hold_s)) or float(self.hold_s) < 0.0:
            raise ValueError(f"ContactStep.hold_s must be >= 0, got "
                             f"{self.hold_s!r}")

    def timed_path(self) -> Tuple[Tuple[float, np.ndarray], ...]:
        """``(t, q)`` per knot, ``t`` seconds from the leg start at
        :attr:`speed_m_s` — the one timing every transport plays.

        STRICTLY INCREASING. A knot that moves the joints without advancing
        along the direction (the solver correcting the wrist, not the tool
        point) has the distance of the knot before it; timed by distance
        alone it had the same TIME, and d1-firmwared refuses a trajectory
        whose times do not increase — the whole leg, at upload (d1-2
        2026-09-23, table probe 4 of 10). Such a knot is given
        :data:`CONTACT_KNOT_MIN_DT_S` after the one before it.
        """
        out: List[Tuple[float, np.ndarray]] = []
        for s, q in zip(self.s, self.path):
            t = max(0.0, s) / self.speed_m_s
            if out and t < out[-1][0] + CONTACT_KNOT_MIN_DT_S:
                t = out[-1][0] + CONTACT_KNOT_MIN_DT_S
            out.append((t, q))
        return tuple(out)

    def duration_s(self) -> float:
        return self.timed_path()[-1][0]


#: The closed set of things a plan can contain. Spelt as a Union rather than
#: ``Any`` so a consumer's type checker can see that the generic runner's
#: ``raise TypeError`` and the firmware runner's silent skip were not the same
#: behaviour over the same set.
Step = Union[JointStep, GripStep, SettleStep, ContactStep]


def _revision_fields(revision: str) -> Dict[str, str]:
    """``"a=1;b=2"`` -> ``{"a": "1", "b": "2"}``; a revision that is not in
    that form is one opaque field."""
    out: Dict[str, str] = {}
    for part in str(revision).split(";"):
        key, sep, value = part.partition("=")
        out[key if sep else "revision"] = value if sep else part
    return out


@dataclass(frozen=True)
class PlanBinding:
    """The posture and the observation a plan was CHECKED against.

    A plan is only a safety claim about the world it was planned in. Between
    the check and the first byte there is a model turn, a network round trip
    and possibly an operator with a hand on the arm; run it against a
    different posture and the "fully checked" joint path starts from somewhere
    nobody checked. Worse, the untouched arm is commanded from live feedback
    (``executor.run_steps``) and the firmware transport PREPENDS the measured
    pose to the path, so a drifted second arm silently becomes part of a
    dual-arm posture the guard never saw (R8).

    So every plan carries this, and every executor checks it before it moves.
    """

    #: side -> the 7 measured joint angles the plan was built from [rad]
    q0: Dict[str, Tuple[float, ...]] = field(default_factory=dict)
    #: ``WorldView.observation_id()`` — revision, stamp, frames, object poses
    observation: Tuple[Any, ...] = ()
    world_stamp: float = 0.0
    frames_now: float = 0.0
    #: the tool geometry the plan was computed with; a gripper swapped between
    #: plan and run moves the tool point and every waypoint with it
    tool_revision: str = ""
    #: how far a joint may have drifted and the plan still be the plan [rad]
    joint_tol_rad: float = 0.05
    #: how old the observation may be when the plan runs [s]; NaN = no limit
    max_age_s: float = float("nan")
    #: was the COLLISION GUARD installed and consulted for every step?
    #:
    #: An analytical plan made against a guard-disabled model is a perfectly
    #: good answer to "could the arm hold this pose" and is NOT an executable
    #: checked plan — and ``Plan.ok`` was ``True`` for both
    #: (``tests/primitives/test_plans.py`` had the case). The distinction now
    #: travels with the plan, and the hardware boundary refuses the unguarded
    #: kind unless the caller says so explicitly.
    guarded: bool = True
    #: the FIRMWARE CONTRACT the observation came through — for d1-firmwared
    #: the sha256 of the OpenAPI document the executor's client was generated
    #: from (``WorldView.firmware_spec``), ``"kinematic"`` for the mirror, ""
    #: when the producer did not say. ``executor.check_binding`` refuses to
    #: play a plan against a transport that drives a different one.
    firmware_spec: str = ""

    @classmethod
    def of(cls, world, kin=None, *, joint_tol_rad: float = 0.05,
           max_age_s: float = float("nan"),
           reference: Optional[str] = None) -> "PlanBinding":
        from .orientation import tool_revision  # noqa: PLC0415 - cycle at import
        q0 = {side: tuple(float(v) for v in arm.joints)
              for side, arm in world.arms.items()}
        gate = getattr(kin, "gate", None)
        return cls(q0=q0, observation=world.observation_id(),
                   world_stamp=float(world.stamp),
                   frames_now=float(world.frames.now),
                   tool_revision=tool_revision(reference),
                   joint_tol_rad=float(joint_tol_rad),
                   max_age_s=float(max_age_s),
                   guarded=bool(getattr(gate, "installed", False)),
                   firmware_spec=str(getattr(world, "firmware_spec", "") or ""))

    def drift(self, *, joints=None, world=None, now: float = float("nan"),
              tool_revision: str = "") -> Optional[str]:
        """Why this plan must NOT be run now, or ``None`` when it may be.

        Every clause is a comparison against something MEASURED. Missing
        evidence is a refusal, not a pass: an executor that cannot report the
        joints of both arms cannot show that the posture is the one that was
        checked.
        """
        if joints is not None:
            for side, q_planned in self.q0.items():
                q_now = joints.get(side)
                if q_now is None:
                    return (f"the executor reports no joints for the {side} arm, "
                            f"so the posture this plan was checked against "
                            f"cannot be confirmed")
                worst = max(abs(float(a) - float(b))
                            for a, b in zip(q_now, q_planned))
                if worst > self.joint_tol_rad:
                    return (f"the {side} arm has moved {math.degrees(worst):.1f} "
                            f"deg since this plan was checked (tolerance "
                            f"{math.degrees(self.joint_tol_rad):.1f} deg) — "
                            f"re-observe and replan")
            for side in joints:
                if side not in self.q0:
                    return (f"this plan was checked without the {side} arm in "
                            f"the observation, and the robot has one")
        if world is not None and tuple(world.observation_id()) != tuple(self.observation):
            return ("the world has been re-observed since this plan was "
                    "checked — replan against the observation you are holding")
        if tool_revision and self.tool_revision:
            # Every field BOTH revisions state must agree. An executor states
            # the hand; a grasp plan also states WHERE on it the contact is
            # (``reference=pad|tip``), so a tip plan runs on the hand it was
            # planned for and is refused where a pad plan is expected.
            planned = _revision_fields(self.tool_revision)
            now = _revision_fields(tool_revision)
            changed = sorted(k for k in set(planned) & set(now)
                             if planned[k] != now[k])
            if changed or not set(planned) & set(now):
                return (f"the tool configuration changed ({self.tool_revision} "
                        f"-> {tool_revision}); every waypoint is a TOOL-POINT "
                        f"pose")
        if math.isfinite(self.max_age_s) and math.isfinite(now):
            age = float(now) - self.world_stamp
            if age > self.max_age_s:
                return (f"the observation this plan was checked against is "
                        f"{age:.1f}s old (limit {self.max_age_s:.1f}s)")
        return None

    def to_json(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "q0_deg": {side: [round(math.degrees(v), 2) for v in q]
                       for side, q in sorted(self.q0.items())},
            "world_stamp": round(float(self.world_stamp), 3),
            "frames_now": round(float(self.frames_now), 3),
            "tool_revision": self.tool_revision,
            "joint_tol_deg": round(math.degrees(self.joint_tol_rad), 2),
            "guarded": bool(self.guarded),
        }
        if self.firmware_spec:
            out["firmware_spec"] = self.firmware_spec
        if math.isfinite(self.max_age_s):
            out["max_age_s"] = round(float(self.max_age_s), 3)
        return out


def _step_json(step: Step) -> Dict[str, Any]:
    """One step, losslessly. The replay format the trace claimed to be."""
    if isinstance(step, JointStep):
        return {"step": "joint", "side": step.side,
                "q_rad": [round(float(v), 6) for v in step.q],
                "waypoint": int(step.waypoint)}
    if isinstance(step, GripStep):
        out = {"step": "grip", "side": step.side,
               "closedness": round(float(step.closedness), 4),
               "grip": step.grip, "waypoint": int(step.waypoint)}
        if step.expect_hold:
            out["expect_hold"] = True
        return out
    if isinstance(step, SettleStep):
        return {"step": "settle", "timeout_s": round(float(step.timeout_s), 3)}
    if isinstance(step, ContactStep):
        return {"step": "contact", "side": step.side,
                "direction": step.direction.to_json(),
                "max_travel_m": round(float(step.max_travel_m), 4),
                "criterion": step.criterion.to_json(),
                "speed_m_s": round(float(step.speed_m_s), 4),
                "hold_s": round(float(step.hold_s), 3),
                "retract": bool(step.retract),
                "waypoint": int(step.waypoint),
                "s_m": [round(float(v), 5) for v in step.s],
                "q_rad": [[round(float(v), 6) for v in q] for q in step.path]}
    raise TypeError(f"not a plan step: {step!r}")


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
    #: what this plan was checked against. ``None`` only for a plan built by
    #: hand in a test; :func:`manipulation_kit.executor.run` refuses to move
    #: an unbound plan unless the caller says so explicitly.
    binding: Optional[PlanBinding] = None

    @property
    def ok(self) -> bool:
        return True

    def joint_steps(self) -> Tuple[JointStep, ...]:
        return tuple(s for s in self.steps if isinstance(s, JointStep))

    def duration_s(self, hz: float = 50.0) -> float:
        return len(self.joint_steps()) / float(hz)

    def final_joints(self) -> Dict[str, Tuple[float, ...]]:
        """The last commanded posture per side — the plan's ENDPOINT.

        Published because it is the thing a transport must be able to prove it
        reached: the stream clamp used to stop 6.5 deg short of it and report
        ``completed`` (R4), and a test cannot catch that without the number.
        """
        out: Dict[str, Tuple[float, ...]] = {}
        for step in self.joint_steps():
            out[step.side] = tuple(float(v) for v in step.q)
        return out

    def to_json(self, *, full: bool = False) -> Dict[str, Any]:
        """The plan as data. ``full=True`` includes every joint command.

        The default stays a SUMMARY — a trace line per turn that carried 200
        seven-vectors would be unreadable — but it now says so, and the full
        form is a real replayable record rather than a count (R: "``to_json``
        stores counts rather than joint commands").
        """
        out: Dict[str, Any] = {
            "schema": "manipulation_kit.plan/2",
            "primitive": self.primitive, "side": self.side,
            "waypoints": [w.to_json() for w in self.waypoints],
            "joint_steps": len(self.joint_steps()),
            "grip_steps": [{"side": s.side, "closedness": s.closedness,
                            "grip": s.grip}
                           for s in self.steps if isinstance(s, GripStep)],
            "final_joints_deg": {side: [round(math.degrees(v), 3) for v in q]
                                 for side, q in sorted(self.final_joints().items())},
            "notes": list(self.notes),
            "binding": None if self.binding is None else self.binding.to_json(),
        }
        contacts = [s for s in self.steps if isinstance(s, ContactStep)]
        if contacts:
            out["contact_steps"] = [
                {"side": s.side, "direction": s.direction.label(),
                 "max_travel_m": round(float(s.max_travel_m), 4),
                 "criterion": s.criterion.to_json(),
                 "hold_s": round(float(s.hold_s), 3),
                 "retract": bool(s.retract)} for s in contacts]
        if full:
            out["steps"] = [_step_json(step) for step in self.steps]
        else:
            out["steps_elided"] = ("summary only; call to_json(full=True) for "
                                   "the joint commands")
        return out

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
    #: the ANGULAR half of the residual [rad]. ``joint_ramp`` used to put
    #: radians in ``residual_m`` and call them metres.
    residual_rad: float = float("nan")
    #: which part of the search gave up — "straight", "via", "ready_reseed",
    #: "clearance_ladder". "Unreachable" is a statement about a SEARCH.
    stage: str = ""
    #: the rungs / candidates that were tried, so a caller can tell "this arm
    #: cannot get there at all" from "this one waypoint was rejected"
    attempted: Tuple[str, ...] = ()

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
        out: Dict[str, Any] = {"schema": "manipulation_kit.refusal/2",
                               "reason": self.reason, "detail": self.detail,
                               "primitive": self.primitive, "side": self.side}
        if self.waypoint_index >= 0:
            out["waypoint_index"] = self.waypoint_index
            out["waypoint_label"] = self.waypoint_label
        if math.isfinite(self.residual_m):
            out["residual_m"] = round(float(self.residual_m), 4)
        if math.isfinite(self.residual_rad):
            out["residual_rad"] = round(float(self.residual_rad), 4)
        if self.stage:
            out["stage"] = self.stage
        if self.attempted:
            out["attempted"] = list(self.attempted)
        if self.unmet:
            out["unmet"] = [u.to_json() for u in self.unmet]
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
    #: True when the verb's ``direction`` is the way the hand TRAVELS ONTO
    #: something (approach, grasp, probe, press) rather than the way it
    #: leaves (lift, retreat). An operator's ``allowed_directions``
    #: (``manipulation_kit.agent.OperatorPolicy``) restricts exactly these.
    DIRECTION_ARRIVES = False

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

    @classmethod
    def arg_roles(cls) -> Dict[str, str]:
        """Per-verb ROLE of a name argument, where it differs from the table's.

        Same reasoning as :meth:`arg_enums`: ``target`` is a vessel for
        ``pour`` and anything in the world for ``press``, and the verb says
        so once, so the schema export and ``decode`` cannot disagree.
        """
        return {}

    @classmethod
    def applicable(cls, world: WorldView) -> bool:
        """Is this verb worth DESCRIBING to a model in this world at all?

        ``True`` for every verb but the ones whose very meaning needs a world
        state — ``handover`` needs one hand holding a named thing and the
        other measurably free. :func:`.schema.tool_schemas` leaves a verb out
        when this says ``False``; ``decode`` and ``plan`` still refuse it with
        the typed reason, so hiding it is a prompt economy, never the check.
        """
        return True

    def preconditions(self, world: WorldView) -> List[Unmet]:  # pragma: no cover
        raise NotImplementedError

    def plan(self, world: WorldView, kin) -> "Union[Plan, PlanError]":  # pragma: no cover
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
