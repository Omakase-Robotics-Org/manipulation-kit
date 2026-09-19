"""The Executor seam: the kit says WHAT to command, never HOW to send it.

The protocol lives here, in the wheel, because a plan is meaningless without a
statement of what running one means. The implementations that touch hardware do
not — with one deliberate exception, :mod:`manipulation_kit.executors.firmware`,
which is behind an optional extra and documented as the exception (Shu,
2026-09-19). Everything in THIS module is pure: two test doubles and a walker.

Why a protocol and not a base class: on the robot the implementation is the
d1-firmwared REST client holding an arm lease; in sim it is the Isaac env
server's socket; in a test it is :class:`KinematicExecutor`, which moves a
model and nothing else. They share no code, only a shape.

The wire vector is 16 wide and laid out like ``dx-inspect-robots``' ``packing``
— ``[left 7 joints, left gripper, right 7 joints, right gripper]``, joints in
RADIANS, grippers as closedness 0 (open) .. 1 (closed). Radians because that is
what the kit computes in; the degrees conversion belongs at the wire, in the
one transport that has a wire.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Protocol, Sequence, Tuple, runtime_checkable

import numpy as np

from .primitives.approach import tool_revision
from .primitives.types import (GripStep, JointStep, Plan, SettleStep)

#: wire layout — see the module docstring
ARM_DOF = 7
WIRE_DIM = 16
LEFT_JOINTS = slice(0, 7)
LEFT_GRIPPER = 7
RIGHT_JOINTS = slice(8, 15)
RIGHT_GRIPPER = 15
JOINT_SLICE: Dict[str, slice] = {"left": LEFT_JOINTS, "right": RIGHT_JOINTS}
GRIPPER_INDEX: Dict[str, int] = {"left": LEFT_GRIPPER, "right": RIGHT_GRIPPER}
SIDES: Tuple[str, str] = ("left", "right")


def wire(joints: Dict[str, Sequence[float]],
         grippers: Optional[Dict[str, float]] = None) -> np.ndarray:
    """Build a 16-vector from per-side joints (rad) and closedness."""
    out = np.zeros(WIRE_DIM, dtype=float)
    for side, q in joints.items():
        out[JOINT_SLICE[side]] = np.asarray(q, dtype=float).reshape(ARM_DOF)
    for side, value in (grippers or {}).items():
        out[GRIPPER_INDEX[side]] = float(value)
    return out


@dataclass(frozen=True)
class RawState:
    """What an executor can say about the robot without interpreting it.

    ``grippers`` is MEASURED — where the jaws are. ``commanded_grippers`` is
    what they are currently being ASKED for, which after a grasp is NOT the
    same number and must not be confused with it: a jaw that stopped on a 40 mm
    cube measures 0.41 while it is still being commanded to 1.0, and it is the
    command that keeps the squeeze on. Empty when the executor cannot say.
    """

    joints: Dict[str, np.ndarray]          # side -> 7 radians, MEASURED
    grippers: Dict[str, float] = field(default_factory=dict)   # side -> closedness
    holding: Dict[str, bool] = field(default_factory=dict)     # side -> measured
    stationary: bool = True
    stamp: float = 0.0
    extra: Dict[str, Any] = field(default_factory=dict)
    #: side -> the closedness currently COMMANDED, when the executor knows it
    commanded_grippers: Dict[str, float] = field(default_factory=dict)

    def vector(self) -> np.ndarray:
        return wire(self.joints, self.grippers)


@dataclass(frozen=True)
class SettleReport:
    """Did the arms actually stop, and what was the worst thing still moving."""

    settled: bool
    waited_s: float = 0.0
    worst_velocity_deg_s: float = float("nan")
    detail: str = ""


@dataclass(frozen=True)
class ArrivalReport:
    """Did the arm MEASURABLY get to the posture it was commanded to?

    "The transport says completed" and "the arm is there" are different
    claims. A trajectory job reports ``completed`` when it has played its last
    waypoint; a stopped arm can be short of the target and a stationary arm
    can still have moving jaws (R7). Everything that must not happen until the
    arm is really there — a close, a release, a verdict — waits on this.
    """

    arrived: bool
    worst_error_rad: float = float("nan")
    waited_s: float = 0.0
    detail: str = ""

    def to_json(self) -> Dict[str, Any]:
        return {"arrived": bool(self.arrived),
                "worst_error_deg": (None if not np.isfinite(self.worst_error_rad)
                                    else round(float(np.degrees(
                                        self.worst_error_rad)), 3)),
                "waited_s": round(float(self.waited_s), 3),
                "detail": self.detail}


@dataclass(frozen=True)
class StrokeReport:
    """Did a gripper stroke reach a TERMINAL state before the next step?

    :class:`~manipulation_kit.primitives.types.GripStep` promises "run to
    completion before the next joint step". Firmware's ``set_gripper`` posted
    the command and discarded the reply, so the promise was the docstring's
    and nobody else's — and F13 is what that costs: the evidence was read
    while the jaws were still closing.
    """

    settled: bool
    closedness: float = float("nan")
    holding: Optional[bool] = None
    stalled: Optional[bool] = None
    waited_s: float = 0.0
    detail: str = ""

    def to_json(self) -> Dict[str, Any]:
        return {"settled": bool(self.settled),
                "closedness": (None if not np.isfinite(self.closedness)
                               else round(float(self.closedness), 3)),
                "holding": self.holding, "stalled": self.stalled,
                "waited_s": round(float(self.waited_s), 3),
                "detail": self.detail}


#: Default barriers. Both are generous — they are deadlines, not budgets.
ARRIVE_TOL_RAD = math.radians(3.0)
ARRIVE_TIMEOUT_S = 3.0
STROKE_TIMEOUT_S = 3.0


@runtime_checkable
class Executor(Protocol):
    """What it takes to run a :class:`Plan`, stated once.

    Four verbs move the robot and two more say when it has ARRIVED. The
    barriers are part of the protocol rather than each consumer's private
    business because the lesson that produced them (F5 and F13, fixed in the
    Isaac executor and nowhere else) is about the SEAM: a plan's steps are
    ordered, and an executor that returns from a step before the robot has
    done it has broken the order for everyone downstream.

    A transport that genuinely blocks can implement the two barriers as
    ``return ArrivalReport(True, ...)`` — but it has to SAY so, because
    ``manipulation_kit.executor.run`` refuses to close jaws on a claim nobody
    made.
    """

    def state(self) -> RawState:
        """The MEASURED robot state. Not the last command echoed back."""
        ...

    def send_joints(self, q16, *, t: float) -> None:
        """Command the 16-vector at plan time ``t`` seconds from the start."""
        ...

    def set_gripper(self, side: str, closedness: float, *, grip: str) -> None:
        """Run a gripper stroke to ``closedness`` with a named force preset."""
        ...

    def settle(self, timeout_s: float) -> SettleReport:
        """Block until the arms are stationary, or the timeout expires."""
        ...

    # -- barriers: measured arrival, not transport completion --------------- #
    def wait_arrived(self, q16, *, tol_rad: float = ARRIVE_TOL_RAD,
                     timeout_s: float = ARRIVE_TIMEOUT_S) -> ArrivalReport:
        """Block until the MEASURED joints are within ``tol_rad`` of ``q16``."""
        ...

    def wait_gripper_settled(self, side: str, *,
                             timeout_s: float = STROKE_TIMEOUT_S
                             ) -> StrokeReport:
        """Block until the jaws reach a terminal state (target, or stalled)."""
        ...


# --------------------------------------------------------------------------- #
# running a plan
# --------------------------------------------------------------------------- #

#: Why a run stopped, when it was not "every step was sent".
NOT_BOUND = "not_bound"
UNGUARDED = "unguarded_plan"
STALE_BINDING = "stale_binding"
REFUSED_PLAN = "refused_plan"
BARRIER_FAILED = "barrier_failed"
TRANSPORT_ERROR = "transport_error"
STOP_REASONS: Tuple[str, ...] = (NOT_BOUND, UNGUARDED, STALE_BINDING,
                                 REFUSED_PLAN, BARRIER_FAILED, TRANSPORT_ERROR)


@dataclass(frozen=True)
class RunReport:
    """What happened, step by step, so a refusal is never silent.

    ``completed`` is about the EXECUTOR, not about the task: it says every step
    was sent AND every barrier in the plan was met. Whether the task worked is
    the verifier's answer, and the two are reported separately on purpose —
    the whole design exists because "the command was sent" was being read as
    "the thing happened".

    ``stop_reason`` is a code from :data:`STOP_REASONS`, so an execution
    failure has the same kind of stable vocabulary a plan refusal has. It used
    to be prose in ``error``, or an exception with no vocabulary at all.
    """

    plan_primitive: str
    side: str
    completed: bool
    steps_sent: int
    settle: Optional[SettleReport] = None
    error: str = ""
    stop_reason: str = ""
    stopped_at: int = -1
    arrivals: Tuple[ArrivalReport, ...] = ()
    strokes: Tuple[StrokeReport, ...] = ()

    def to_json(self) -> Dict[str, Any]:
        return {"primitive": self.plan_primitive, "side": self.side,
                "completed": self.completed, "steps_sent": self.steps_sent,
                "settled": None if self.settle is None else self.settle.settled,
                "stop_reason": self.stop_reason,
                "stopped_at": self.stopped_at,
                "arrivals": [a.to_json() for a in self.arrivals],
                "strokes": [s.to_json() for s in self.strokes],
                "error": self.error}


def check_binding(plan: Plan, executor: "Executor", *,
                  allow_unbound: bool = False, now: float = float("nan")
                  ) -> Optional[str]:
    """Is this plan still a statement about the robot in front of us?

    Called before the FIRST byte of every run. The plan carries the posture,
    the observation and the tool it was checked against
    (:class:`~manipulation_kit.primitives.types.PlanBinding`); this compares
    them with what the executor measures right now. Drift is a refusal, not a
    clamp: the answer to "the arm moved since you planned" is to replan.

    Returns the reason to refuse, or ``None``.
    """
    binding = getattr(plan, "binding", None)
    if binding is None:
        if allow_unbound:
            return None
        return ("this plan carries no binding, so there is nothing to check it "
                "against. Build it with a primitive's plan(), or pass "
                "allow_unbound=True and own the consequence")
    state = executor.state()
    stamp = state.stamp if math.isfinite(state.stamp) else now
    return binding.drift(joints=state.joints, now=stamp,
                         tool_revision=tool_revision())


def _arrival_of(executor: "Executor", q16, *, tol_rad: float,
                timeout_s: float) -> ArrivalReport:
    """The arrival barrier, or a stated absence of one.

    An executor that does not implement ``wait_arrived`` gets an UNKNOWN
    arrival rather than an assumed one — and ``run`` treats that as a failed
    barrier before a stroke, because closing the jaws on an unverified
    posture is the thing the barrier exists to stop.
    """
    wait = getattr(executor, "wait_arrived", None)
    if wait is None:
        return ArrivalReport(False, detail=(
            f"{type(executor).__name__} implements no wait_arrived, so the "
            f"arm's arrival cannot be measured"))
    return wait(q16, tol_rad=tol_rad, timeout_s=timeout_s)


def _stroke_of(executor: "Executor", side: str, *, timeout_s: float
               ) -> StrokeReport:
    wait = getattr(executor, "wait_gripper_settled", None)
    if wait is None:
        return StrokeReport(False, detail=(
            f"{type(executor).__name__} implements no wait_gripper_settled, "
            f"so the stroke's completion cannot be measured"))
    return wait(side, timeout_s=timeout_s)


def run(plan: Plan, executor: "Executor", *, hz: float = 50.0,
        allow_unbound: bool = False, allow_unguarded: bool = False,
        arrive_tol_rad: float = ARRIVE_TOL_RAD,
        arrive_timeout_s: float = ARRIVE_TIMEOUT_S,
        stroke_timeout_s: float = STROKE_TIMEOUT_S) -> RunReport:
    """Walk a plan's steps through an executor, in order, at ``hz``.

    Joint steps are batched into one 16-vector per step so both arms move
    together — a dual-arm robot commanded one arm at a time is two robots
    disagreeing about the posture the guard just approved.

    Nothing is sent until :func:`check_binding` passes.
    """
    if not getattr(plan, "ok", False):
        return RunReport(getattr(plan, "primitive", "?"),
                         getattr(plan, "side", ""), False, 0,
                         error=str(plan), stop_reason=REFUSED_PLAN)
    binding = getattr(plan, "binding", None)
    if binding is not None and not binding.guarded and not allow_unguarded:
        return RunReport(
            plan.primitive, plan.side, False, 0, stop_reason=UNGUARDED,
            error=("this plan was made against a model with NO COLLISION "
                   "GUARD installed. That is a valid analytical answer and it "
                   "is not an executable checked plan; build it with a guarded "
                   "kinematic model, or pass allow_unguarded=True and own it"))
    drift = check_binding(plan, executor, allow_unbound=allow_unbound)
    if drift is not None:
        return RunReport(
            plan.primitive, plan.side, False, 0, error=drift,
            stop_reason=(NOT_BOUND if getattr(plan, "binding", None) is None
                         else STALE_BINDING))
    # An executor that can take a whole plan at once is asked to: the firmware
    # one uploads it as a daemon-played trajectory, which is strictly better
    # than this loop re-timing it over a socket. See
    # manipulation_kit.executors.firmware for the argument. It gets the SAME
    # barrier settings; ``hz`` used to be silently dropped on this path.
    own = getattr(executor, "run_plan", None)
    if own is not None:
        return own(plan, arrive_tol_rad=arrive_tol_rad,
                   arrive_timeout_s=arrive_timeout_s,
                   stroke_timeout_s=stroke_timeout_s)
    return run_steps(plan, executor, hz=hz, arrive_tol_rad=arrive_tol_rad,
                     arrive_timeout_s=arrive_timeout_s,
                     stroke_timeout_s=stroke_timeout_s)


def run_steps(plan: Plan, executor: "Executor", *, hz: float = 50.0,
              schedule: Optional[Sequence[float]] = None,
              arrive_tol_rad: float = ARRIVE_TOL_RAD,
              arrive_timeout_s: float = ARRIVE_TIMEOUT_S,
              stroke_timeout_s: float = STROKE_TIMEOUT_S) -> RunReport:
    """The step-by-step walk itself, with no delegation.

    Separate from :func:`run` so an executor that OVERRIDES ``run_plan`` can
    still reach the generic loop for its fallback transport without calling
    itself. (It did, once, and the stack trace said so.)

    ``schedule`` is the plan time for each JOINT step, in order. A transport
    that has a rate ceiling computes one (``FirmwareExecutor.stream_schedule``)
    so that honouring the ceiling stretches TIME rather than shrinking the
    motion; with no schedule the steps go out at a flat ``hz``.

    THE BARRIERS ARE HERE, not in each transport. Before a gripper stroke the
    arm must be measurably AT the posture the last joint step commanded;
    after one, the stroke must have reached a terminal state; a settle that
    fails ends the run. Every one of those used to be a step that returned
    immediately and a report that said ``completed``.
    """
    period = 1.0 / float(hz)
    sent = 0
    settle: Optional[SettleReport] = None
    arrivals: List[ArrivalReport] = []
    strokes: List[StrokeReport] = []
    begin = getattr(executor, "begin_run", None)
    if begin is not None:
        begin(plan)
    state = executor.state()
    missing = [s for s in SIDES if s not in state.joints]
    if missing:
        # The arm nobody is moving is still COMMANDED, every tick, at whatever
        # this dict says. Defaulting it to zero would command a T-pose through
        # the torso; refusing is the only safe answer.
        return RunReport(
            plan.primitive, plan.side, False, 0, stop_reason=TRANSPORT_ERROR,
            error=(f"the executor reports no joints for {missing} — the "
                   f"untouched arm is commanded at its measured pose in every "
                   f"dual-arm vector, and there is nothing to command it at"))
    joints = {side: np.array(q, dtype=float) for side, q in state.joints.items()}
    # A plan that does not touch the jaws still has to put a number in every
    # 16-vector it sends, and that number is the COMMAND the hand is already
    # under — never where the jaws happen to be. Seeding it from the
    # measurement tells a force- or torque-limited gripper to stop squeezing
    # and hold position, which is how a held object is dropped by a Lift that
    # never mentioned the gripper: MEASURED 2026-09-19, three trials out of
    # three — grasp verified holding at a 41.2 mm gap, the first knot of the
    # Lift re-commanded 0.41 (where the jaws had stopped), the squeeze went to
    # zero, and the cube stayed on the wagon while the arm went up. The same
    # rule as F5 one level out: a measurement is not a command.
    grippers = {side: float(state.commanded_grippers.get(side, value))
                for side, value in state.grippers.items()}
    last_vector: Optional[np.ndarray] = None

    def stop(index: int, reason: str, detail: str) -> RunReport:
        end = getattr(executor, "end_run", None)
        if end is not None:
            end(plan)
        return RunReport(plan.primitive, plan.side, False, sent, settle,
                         error=detail, stop_reason=reason, stopped_at=index,
                         arrivals=tuple(arrivals), strokes=tuple(strokes))

    times = None if schedule is None else list(schedule)
    joint_index = 0
    for index, step in enumerate(plan.steps):
        if isinstance(step, JointStep):
            joints[step.side] = np.asarray(step.q, dtype=float)
            last_vector = wire(joints, grippers)
            at = (joint_index * period if times is None
                  else float(times[joint_index]))
            executor.send_joints(last_vector, t=at)
            joint_index += 1
            sent += 1
        elif isinstance(step, GripStep):
            # ARRIVE BEFORE YOU CLOSE. A stroke run while the arm is still
            # travelling closes the jaws somewhere along the path.
            if last_vector is not None:
                arrival = _arrival_of(executor, last_vector,
                                      tol_rad=arrive_tol_rad,
                                      timeout_s=arrive_timeout_s)
                arrivals.append(arrival)
                if not arrival.arrived:
                    return stop(index, BARRIER_FAILED,
                                f"the {plan.side} arm had not arrived at the "
                                f"posture before the gripper stroke: "
                                f"{arrival.detail}")
            executor.set_gripper(step.side, step.closedness, grip=step.grip)
            grippers[step.side] = float(step.closedness)
            stroke = _stroke_of(executor, step.side, timeout_s=stroke_timeout_s)
            strokes.append(stroke)
            if not stroke.settled:
                return stop(index, BARRIER_FAILED,
                            f"the {step.side} gripper stroke did not reach a "
                            f"terminal state: {stroke.detail}")
            sent += 1
        elif isinstance(step, SettleStep):
            settle = executor.settle(step.timeout_s)
            sent += 1
            if not settle.settled:
                # BOTH runners used to carry on here and report completion.
                return stop(index, BARRIER_FAILED,
                            f"the arms did not settle: {settle.detail}")
        else:
            return stop(index, TRANSPORT_ERROR, f"not a plan step: {step!r}")
    end = getattr(executor, "end_run", None)
    if end is not None:
        end(plan)
    return RunReport(plan.primitive, plan.side, True, sent, settle,
                     arrivals=tuple(arrivals), strokes=tuple(strokes))


# --------------------------------------------------------------------------- #
# test doubles
# --------------------------------------------------------------------------- #

class RecordingExecutor:
    """Records what it was told and changes nothing. The null executor, with a log.

    Deliberately does NOT move its reported state when joints are sent: it is
    the executor a verifier must FAIL against. "Ran the plan, nothing moved,
    verdict TRUE" is the failure mode the measured-verifier rule exists to
    prevent, and this class is how that is tested.
    """

    def __init__(self, state: Optional[RawState] = None):
        self._state = state or RawState(joints={s: np.zeros(ARM_DOF) for s in SIDES},
                                        grippers={s: 0.0 for s in SIDES})
        self.sent: List[Tuple[float, np.ndarray]] = []
        self.grips: List[Tuple[str, float, str]] = []
        self.settles: List[float] = []
        self.arrival_waits: List[float] = []
        self.stroke_waits: List[Tuple[str, float]] = []
        #: a test that wants to exercise the steps AFTER a barrier sets this
        self.pretend_arrived = False

    def state(self) -> RawState:
        return self._state

    def send_joints(self, q16, *, t: float) -> None:
        q = np.asarray(q16, dtype=float).reshape(WIRE_DIM)
        self.sent.append((float(t), q))

    def set_gripper(self, side: str, closedness: float, *, grip: str) -> None:
        self.grips.append((side, float(closedness), grip))

    def settle(self, timeout_s: float) -> SettleReport:
        self.settles.append(float(timeout_s))
        return SettleReport(True, 0.0, 0.0, "nothing was moving; nothing was sent")

    # -- barriers ---------------------------------------------------------- #
    # A recorder moves nothing, so nothing arrives. It SAYS so rather than
    # claiming an arrival it cannot have had: this class exists to be the
    # executor a verifier must fail against, and a fake barrier would make it
    # the executor a RUNNER passes against instead.
    def wait_arrived(self, q16, *, tol_rad: float = ARRIVE_TOL_RAD,
                     timeout_s: float = ARRIVE_TIMEOUT_S) -> ArrivalReport:
        self.arrival_waits.append(float(timeout_s))
        if not self.pretend_arrived:
            return ArrivalReport(False, detail=(
                "a RecordingExecutor moves nothing, so the arm never arrives"))
        return ArrivalReport(True, 0.0, 0.0, "pretended, by the test's request")

    def wait_gripper_settled(self, side: str, *,
                             timeout_s: float = STROKE_TIMEOUT_S
                             ) -> StrokeReport:
        self.stroke_waits.append((side, float(timeout_s)))
        if not self.pretend_arrived:
            return StrokeReport(False, detail=(
                "a RecordingExecutor moves nothing, so no stroke completes"))
        return StrokeReport(True, self._state.grippers.get(side, float("nan")),
                            detail="pretended, by the test's request")

    # -- inspection -------------------------------------------------------- #
    def cadence(self) -> np.ndarray:
        """Gaps between consecutive plan times — the commanded rate."""
        times = np.array([t for t, _ in self.sent], dtype=float)
        return np.diff(times) if times.size > 1 else np.zeros(0)


class KinematicExecutor:
    """A fake robot: the commands land on a kinematic model and nowhere else.

    It mirrors — it does not simulate. There is no dynamics, no contact and no
    time: a commanded posture is reached exactly. That is the right fidelity
    for testing that a PLAN is well formed (continuous, in limits, guard-clean,
    ending where the primitive said) without pretending to test whether the
    physical grasp holds, which no amount of kinematics can tell you.

    ``holds`` lets a test say "the gripper closed on the block": closing past
    ``hold_at`` with an object named makes ``holding`` true, opening clears it.
    """

    def __init__(self, kin, *, hold_at: float = 0.5,
                 held: Optional[Dict[str, Optional[str]]] = None):
        self.kin = kin
        self.hold_at = float(hold_at)
        self.grippers: Dict[str, float] = {s: 0.0 for s in SIDES}
        self.held: Dict[str, Optional[str]] = dict(held or {})
        self.sent: List[Tuple[float, np.ndarray]] = []
        self.grips: List[Tuple[str, float, str]] = []
        self.settles: List[float] = []
        #: what the jaws would close on next, set by a test before a Grasp
        self.next_object: Dict[str, Optional[str]] = {s: None for s in SIDES}

    def state(self) -> RawState:
        return RawState(
            joints={s: np.array(self.kin.joints(s), dtype=float) for s in SIDES},
            grippers=dict(self.grippers),
            holding={s: self.held.get(s) is not None for s in SIDES},
            stationary=True)

    def send_joints(self, q16, *, t: float) -> None:
        q = np.asarray(q16, dtype=float).reshape(WIRE_DIM)
        self.sent.append((float(t), q.copy()))
        for side in SIDES:
            self.kin.set_joints(side, q[JOINT_SLICE[side]])
            self.grippers[side] = float(q[GRIPPER_INDEX[side]])

    def set_gripper(self, side: str, closedness: float, *, grip: str) -> None:
        self.grips.append((side, float(closedness), grip))
        self.grippers[side] = float(closedness)
        if closedness >= self.hold_at:
            self.held[side] = self.next_object.get(side)
        else:
            self.held[side] = None

    def settle(self, timeout_s: float) -> SettleReport:
        self.settles.append(float(timeout_s))
        return SettleReport(True, 0.0, 0.0, "a kinematic mirror is always settled")

    # -- barriers ---------------------------------------------------------- #
    # A mirror reaches a commanded posture exactly and instantaneously, so
    # these are honest ``True``s rather than pretended ones — and they are
    # still MEASURED against the model rather than assumed, so a test that
    # poses the model elsewhere makes them fail.
    def wait_arrived(self, q16, *, tol_rad: float = ARRIVE_TOL_RAD,
                     timeout_s: float = ARRIVE_TIMEOUT_S) -> ArrivalReport:
        want = np.asarray(q16, dtype=float).reshape(WIRE_DIM)
        worst = 0.0
        for side in SIDES:
            worst = max(worst, float(np.max(np.abs(
                np.asarray(self.kin.joints(side), dtype=float)
                - want[JOINT_SLICE[side]]))))
        if worst <= tol_rad:
            return ArrivalReport(True, worst, 0.0,
                                 "the mirror is at the commanded posture")
        return ArrivalReport(False, worst, float(timeout_s),
                             f"the mirror is {np.degrees(worst):.1f} deg from "
                             f"the commanded posture")

    def wait_gripper_settled(self, side: str, *,
                             timeout_s: float = STROKE_TIMEOUT_S
                             ) -> StrokeReport:
        return StrokeReport(True, self.grippers.get(side, 0.0),
                            holding=self.held.get(side) is not None,
                            stalled=self.held.get(side) is not None,
                            detail="a kinematic stroke completes at once")

    def tool_pose(self, side: str):
        from .primitives.approach import tool_from_link7
        return tool_from_link7(*self.kin.ee_pose(side))
