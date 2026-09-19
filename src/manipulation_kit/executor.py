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

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Protocol, Sequence, Tuple, runtime_checkable

import numpy as np

from .primitives.types import GripStep, JointStep, Plan, SettleStep

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


@runtime_checkable
class Executor(Protocol):
    """Four verbs. Anything that can do these can run any :class:`Plan`."""

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


# --------------------------------------------------------------------------- #
# running a plan
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class RunReport:
    """What happened, step by step, so a refusal is never silent.

    ``completed`` is about the EXECUTOR, not about the task: it says every step
    was sent. Whether the task worked is the verifier's answer, and the two are
    reported separately on purpose — the whole design exists because "the
    command was sent" was being read as "the thing happened".
    """

    plan_primitive: str
    side: str
    completed: bool
    steps_sent: int
    settle: Optional[SettleReport] = None
    error: str = ""

    def to_json(self) -> Dict[str, Any]:
        return {"primitive": self.plan_primitive, "side": self.side,
                "completed": self.completed, "steps_sent": self.steps_sent,
                "settled": None if self.settle is None else self.settle.settled,
                "error": self.error}


def run(plan: Plan, executor: Executor, *, hz: float = 50.0) -> RunReport:
    """Walk a plan's steps through an executor, in order, at ``hz``.

    Joint steps are batched into one 16-vector per step so both arms move
    together — a dual-arm robot commanded one arm at a time is two robots
    disagreeing about the posture the guard just approved.
    """
    if not getattr(plan, "ok", False):
        return RunReport(getattr(plan, "primitive", "?"), getattr(plan, "side", ""),
                         False, 0, error=str(plan))
    # An executor that can take a whole plan at once is asked to: the firmware
    # one uploads it as a daemon-played trajectory, which is strictly better
    # than this loop re-timing it over a socket. See
    # manipulation_kit.executors.firmware for the argument.
    own = getattr(executor, "run_plan", None)
    if own is not None:
        return own(plan)
    return run_steps(plan, executor, hz=hz)


def run_steps(plan: Plan, executor: Executor, *, hz: float = 50.0) -> RunReport:
    """The step-by-step walk itself, with no delegation.

    Separate from :func:`run` so an executor that OVERRIDES ``run_plan`` can
    still reach the generic loop for its fallback transport without calling
    itself. (It did, once, and the stack trace said so.)
    """
    period = 1.0 / float(hz)
    sent = 0
    settle: Optional[SettleReport] = None
    state = executor.state()
    missing = [s for s in SIDES if s not in state.joints]
    if missing:
        # The arm nobody is moving is still COMMANDED, every tick, at whatever
        # this dict says. Defaulting it to zero would command a T-pose through
        # the torso; refusing is the only safe answer.
        raise ValueError(
            f"the executor reports no joints for {missing} — the untouched arm "
            f"is commanded at its measured pose in every dual-arm vector, and "
            f"there is nothing to command it at")
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
    for step in plan.steps:
        if isinstance(step, JointStep):
            joints[step.side] = np.asarray(step.q, dtype=float)
            executor.send_joints(wire(joints, grippers), t=sent * period)
            sent += 1
        elif isinstance(step, GripStep):
            executor.set_gripper(step.side, step.closedness, grip=step.grip)
            grippers[step.side] = float(step.closedness)
            sent += 1
        elif isinstance(step, SettleStep):
            settle = executor.settle(step.timeout_s)
            sent += 1
        else:  # pragma: no cover - the Step union is closed
            raise TypeError(f"not a plan step: {step!r}")
    return RunReport(plan.primitive, plan.side, True, sent, settle)


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

    def tool_pose(self, side: str):
        from .primitives.approach import tool_from_link7
        return tool_from_link7(*self.kin.ee_pose(side))
