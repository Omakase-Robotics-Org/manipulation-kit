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

import dataclasses
import inspect
import math
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import (Any, Dict, List, Mapping, Optional, Protocol, Sequence,
                    Tuple, runtime_checkable)

import numpy as np

from .primitives.orientation import link7_from_tool, tool_from_link7, tool_revision
from .primitives.types import (ContactCriterion, ContactStep, GripStep,
                               JointStep, Plan, SettleStep, Waypoint)

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


#: The arm modes a controller can report — the daemon's ``ArmMode``, with a
#: mode this kit does not know read as ``"unknown"``. Position is the only one
#: this kit COMMANDS (``Plan.requires_mode``); the others are read so a latched
#: controller (``"error"``) can be told apart from one that is holding a pose.
ARM_MODES: Tuple[str, ...] = ("idle", "position", "pvt", "torque", "release",
                              "error", "unknown")


def _vec7(value: Any, name: str) -> Optional[np.ndarray]:
    if value is None:
        return None
    out = np.array(value, dtype=float).reshape(-1)
    if out.size != ARM_DOF:
        raise ValueError(f"{name} must hold {ARM_DOF} values, got {out.size}")
    return out


@dataclass(frozen=True)
class JointState:
    """One arm, as MEASURED by the transport that drives it.

    Every field is what the producer reports and nothing else: a transport
    that cannot measure velocity or torque leaves them ``None`` rather than
    publishing zeros a reader would take for a stationary, unloaded arm. On
    d1-firmwared the fields are the generated ``ArmState`` model's, field for
    field (``executors/firmware/client.py``), converted to radians.
    """

    q: np.ndarray                          # measured joint angles [rad]
    qd: Optional[np.ndarray] = None        # measured joint velocity [rad/s]
    torque_nm: Optional[np.ndarray] = None  # measured joint torque [Nm]
    mode: str = "unknown"                  # one of ARM_MODES
    error_code: int = 0                    # the controller's own; 0 = none
    stationary: bool = True

    def __post_init__(self) -> None:
        object.__setattr__(self, "q", _vec7(self.q, "q"))
        object.__setattr__(self, "qd", _vec7(self.qd, "qd"))
        object.__setattr__(self, "torque_nm", _vec7(self.torque_nm, "torque_nm"))
        mode = str(self.mode)
        object.__setattr__(self, "mode", mode if mode in ARM_MODES else "unknown")
        object.__setattr__(self, "error_code", int(self.error_code))
        object.__setattr__(self, "stationary", bool(self.stationary))

    @property
    def faulted(self) -> bool:
        """Is the controller latched — ``mode == "error"`` or a non-zero code?

        A latched arm answers every command with nothing, and its feedback
        looks like a perfectly still arm; that is how d1-2 run5 (2026-09-22)
        kept planning on an arm that could not move.
        """
        return self.mode == "error" or self.error_code != 0


@dataclass(frozen=True)
class HandState:
    """One gripper, as MEASURED — and what it is currently being ASKED for.

    ``None`` everywhere means "the producer does not say", never a default:
    a gripper whose aperture is unknown is not an open one, and a transport
    that does not publish a fault is not reporting a healthy one.

    ``closedness``  0 open .. 1 closed, measured.
    ``commanded``   the closedness the hand is currently COMMANDED to. After a
                    grasp this is 1.0 while ``closedness`` reads 0.41, and it is
                    the command that keeps the squeeze on (F9).
    ``holding``     the producer's own verdict that something is between the
                    jaws.
    ``jaw_gap_m``   the measured pad-face separation [m].
    ``torque_nm``   the jaw motor's measured torque.
    ``stalled``     the last stroke stopped ON something (``True``) or ran to
                    its target (``False``).
    ``fault``       the producer's fault, by name; ``None`` = healthy.
    ``open_gap_m``  the pad-face gap this hand reaches when driven fully open
                    [m] — on d1-firmwared, derived from the daemon's
                    ``open_rad`` through the hand description's kinematic map.
    """

    closedness: Optional[float] = None
    commanded: Optional[float] = None
    holding: Optional[bool] = None
    jaw_gap_m: Optional[float] = None
    torque_nm: Optional[float] = None
    stalled: Optional[bool] = None
    fault: Optional[str] = None
    open_gap_m: Optional[float] = None


@dataclass(frozen=True, init=False)
class RawState:
    """What an executor can say about the robot without interpreting it.

    ``arms`` and ``hands`` are per side and carry everything the transport
    measures (:class:`JointState`, :class:`HandState`). ``extra`` is for data
    that is genuinely foreign to this vocabulary — a simulator's raw gripper
    scale, say — and nothing the kit reads travels through it.

    TRANSITIONAL (0.16, removed in 0.17): the flat ``joints`` / ``grippers``
    / ``holding`` / ``commanded_grippers`` / ``stationary`` accessors, and the
    same keywords at construction, so ``wire()`` and out-of-tree executors
    (d1-isaaclab's ``agent_eval``) keep working for one release. They are
    views of ``arms``/``hands``, not a second copy.
    """

    arms: Mapping[str, JointState]
    hands: Mapping[str, HandState]
    stamp: float
    #: genuinely foreign data only — nothing the kit reads travels here
    extra: Dict[str, Any]

    def __init__(self, arms: Optional[Mapping[str, JointState]] = None,
                 hands: Optional[Mapping[str, HandState]] = None, *,
                 stamp: float = 0.0,
                 extra: Optional[Dict[str, Any]] = None,
                 joints: Optional[Mapping[str, Sequence[float]]] = None,
                 grippers: Optional[Mapping[str, float]] = None,
                 holding: Optional[Mapping[str, bool]] = None,
                 stationary: Optional[bool] = None,
                 commanded_grippers: Optional[Mapping[str, float]] = None):
        flat = (joints is not None or grippers is not None or holding is not None
                or stationary is not None or commanded_grippers is not None)
        if flat and (arms is not None or hands is not None):
            raise TypeError("RawState takes arms=/hands= OR the transitional "
                            "flat keywords (joints=, grippers=, ...), not both")
        if flat:
            still = True if stationary is None else bool(stationary)
            arms = {side: JointState(q=q, stationary=still)
                    for side, q in (joints or {}).items()}
            grippers, holding = dict(grippers or {}), dict(holding or {})
            commanded = dict(commanded_grippers or {})
            hands = {}
            for side in list(grippers) + [s for s in list(holding) + list(commanded)
                                          if s not in grippers]:
                if side in hands:
                    continue
                hands[side] = HandState(
                    closedness=(None if grippers.get(side) is None
                                else float(grippers[side])),
                    commanded=(None if commanded.get(side) is None
                               else float(commanded[side])),
                    holding=(None if holding.get(side) is None
                             else bool(holding[side])))
        object.__setattr__(self, "arms", MappingProxyType(dict(arms or {})))
        object.__setattr__(self, "hands", MappingProxyType(dict(hands or {})))
        object.__setattr__(self, "stamp", float(stamp))
        object.__setattr__(self, "extra", dict(extra or {}))

    # -- the one sanctioned shim: flat views, removed in 0.17 --------------- #
    @property
    def joints(self) -> Dict[str, np.ndarray]:
        """side -> measured joints [rad]. Transitional; read ``arms``."""
        return {side: arm.q for side, arm in self.arms.items()}

    @property
    def grippers(self) -> Dict[str, float]:
        """side -> measured closedness, where known. Transitional; ``hands``."""
        return {side: hand.closedness for side, hand in self.hands.items()
                if hand.closedness is not None}

    @property
    def holding(self) -> Dict[str, bool]:
        """side -> the producer's hold verdict, where it gives one."""
        return {side: hand.holding for side, hand in self.hands.items()
                if hand.holding is not None}

    @property
    def commanded_grippers(self) -> Dict[str, float]:
        """side -> the closedness currently COMMANDED, where known."""
        return {side: hand.commanded for side, hand in self.hands.items()
                if hand.commanded is not None}

    @property
    def stationary(self) -> bool:
        return all(arm.stationary for arm in self.arms.values())

    # -- reading it -------------------------------------------------------- #
    def faults(self) -> Dict[str, str]:
        """side -> why that arm's controller is latched, for every latched arm."""
        return {side: (f"the {side} arm controller reports mode "
                       f"{arm.mode!r} with error code {arm.error_code}")
                for side, arm in self.arms.items() if arm.faulted}

    def vector(self) -> np.ndarray:
        return wire(self.joints, self.grippers)


@dataclass(frozen=True)
class NeckState:
    """The head's two axes, in the daemon's LOGICAL frame (not flipped here).

    ``description.head_camera.pose_from_neck_state`` owns any sign convention
    between this and the camera pose; a transport that flipped it too would be
    the third flip (D1 perception, 2026-09-22).
    """

    pitch_rad: float
    yaw_rad: float
    enabled: bool
    moving: bool


@dataclass(frozen=True)
class LiftState:
    """The torso lift (the daemon's "slider")."""

    height_m: float
    moving: bool
    #: the drive's alarm, by the drive's own text; ``None`` = no alarm
    alarm: Optional[str] = None


def read_neck(executor: Any) -> Optional[NeckState]:
    """``executor.neck_state()`` when the transport has a neck, else ``None``.

    An optional capability, duck-typed like ``run_plan``: a transport with no
    head (a test double, a single-arm rig) simply does not define it.
    """
    fn = getattr(executor, "neck_state", None)
    return None if fn is None else fn()


def read_lift(executor: Any) -> Optional[LiftState]:
    """``executor.lift_state()`` when the transport has a lift, else ``None``."""
    fn = getattr(executor, "lift_state", None)
    return None if fn is None else fn()


@dataclass(frozen=True)
class SettleReport:
    """Did the arms actually stop, and what was the worst thing still moving."""

    settled: bool
    waited_s: float = 0.0
    worst_velocity_deg_s: float = float("nan")
    detail: str = ""


@dataclass(frozen=True)
class Correction:
    """One in-place re-solve of a waypoint the tool arrived off.

    Recorded per round, before and after, because "it was corrected" is a
    claim with a number behind it or it is nothing: a round that moved the
    tool from 27 mm to 26 mm is a round that did not work, and it looks
    exactly like one that did until the two numbers are beside each other.
    """

    round: int
    tool_error_before_m: float
    tool_error_after_m: float = float("nan")
    #: was the corrected posture actually commanded? ``False`` when the guard
    #: (or the IK) refused it — a refused correction is NOT sent.
    sent: bool = True
    detail: str = ""

    def to_json(self) -> Dict[str, Any]:
        return {"round": int(self.round),
                "tool_error_before_m": round(float(self.tool_error_before_m), 5),
                "tool_error_after_m": (
                    None if not np.isfinite(self.tool_error_after_m)
                    else round(float(self.tool_error_after_m), 5)),
                "sent": bool(self.sent), "detail": self.detail}


@dataclass(frozen=True)
class ArrivalReport:
    """Did the arm MEASURABLY get to the posture it was commanded to?

    "The transport says completed" and "the arm is there" are different
    claims. A trajectory job reports ``completed`` when it has played its last
    waypoint; a stopped arm can be short of the target and a stationary arm
    can still have moving jaws (R7). Everything that must not happen until the
    arm is really there — a close, a release, a verdict — waits on this.

    THERE ARE TWO ANSWERS HERE AND THEY ARE DIFFERENT. ``worst_error_rad`` is
    about seven angles; ``tool_error_m`` is about the jaw pocket, computed by
    the kit's own forward kinematics from the SAME two postures. A tolerance
    wide enough for the real arm's gravity droop (3 deg, F16) is centimetres
    at the tool at a half-metre reach, which is how a grasp that passed every
    barrier closed beside the block (F17). Both travel, always, so a trace can
    be argued with afterwards.
    """

    arrived: bool
    worst_error_rad: float = float("nan")
    waited_s: float = 0.0
    detail: str = ""
    #: distance between the COMMANDED tool point and the MEASURED one [m]
    tool_error_m: float = float("nan")
    #: the orientation half, as a rotation-vector magnitude [rad]
    tool_rot_error_rad: float = float("nan")
    #: the in-place corrections this barrier ran, in order
    corrections: Tuple[Correction, ...] = ()
    #: which waypoint this barrier was gating, when it was gating one
    waypoint_label: str = ""
    #: had the arm STOPPED when the tool point was read? ``None`` when the
    #: barrier did not ask (no tool gate, or a transport with no settle).
    settled: Optional[bool] = None
    #: ``tool_error_m`` split about the approach axis: the component the jaws
    #: close in, and the one that says how deep the descent got (signed, +
    #: past the waypoint). The gate judges these two, not the total.
    tool_across_m: float = float("nan")
    tool_along_m: float = float("nan")

    def to_json(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "arrived": bool(self.arrived),
            "worst_error_deg": (None if not np.isfinite(self.worst_error_rad)
                                else round(float(np.degrees(
                                    self.worst_error_rad)), 3)),
            "tool_error_m": (None if not np.isfinite(self.tool_error_m)
                             else round(float(self.tool_error_m), 5)),
            "tool_rot_error_deg": (
                None if not np.isfinite(self.tool_rot_error_rad)
                else round(float(np.degrees(self.tool_rot_error_rad)), 3)),
            "waited_s": round(float(self.waited_s), 3),
            "detail": self.detail}
        if self.waypoint_label:
            out["waypoint_label"] = self.waypoint_label
        if np.isfinite(self.tool_across_m):
            out["tool_across_m"] = round(float(self.tool_across_m), 5)
            out["tool_along_m"] = round(float(self.tool_along_m), 5)
        if self.settled is not None:
            out["settled"] = bool(self.settled)
        if self.corrections:
            out["corrections"] = [c.to_json() for c in self.corrections]
        return out


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
    #: the producer's own name for the outcome — d1-firmwared's
    #: ``StrokeKind`` (``grasp``/``contact``/``empty``/``open``/``timeout``/
    #: ``fault``/``overload``/``blind``/``lost``). Empty when it has none.
    kind: str = ""
    #: the GRIPPER's fault, by the producer's own name, when it is faulted.
    #: Distinct from "not settled": a faulted gripper's jaws are perfectly
    #: stationary, which is what made a fault read as a finished stroke.
    fault: Optional[str] = None

    @property
    def faulted(self) -> bool:
        return self.fault is not None

    def to_json(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "settled": bool(self.settled),
            "closedness": (None if not np.isfinite(self.closedness)
                           else round(float(self.closedness), 3)),
            "holding": self.holding, "stalled": self.stalled,
            "waited_s": round(float(self.waited_s), 3),
            "detail": self.detail}
        if self.kind:
            out["kind"] = self.kind
        if self.fault is not None:
            out["fault"] = self.fault
        return out


#: Why a contact leg (:class:`~manipulation_kit.primitives.types.ContactStep`)
#: stopped. ``contact``: the criterion was met — something resisted.
#: ``max_travel``: the whole leg was travelled and nothing did. ``fault``: the
#: arm controller latched during the leg. ``guard``: the transport refused or
#: aborted the motion itself (the daemon's motion guard or slew gate, a
#: cancel nobody here asked for). ``unmeasured``: the transport cannot
#: measure what the criterion is about (no joint torque, or a tool force no
#: transport here estimates) — so the leg was never started.
CONTACT = "contact"
MAX_TRAVEL = "max_travel"
CONTACT_FAULT = "fault"
CONTACT_GUARD = "guard"
UNMEASURED = "unmeasured"
CONTACT_STOPS: Tuple[str, ...] = (CONTACT, MAX_TRAVEL, CONTACT_FAULT,
                                  CONTACT_GUARD, UNMEASURED)
#: How many state reads the pre-motion torque baseline is averaged over. One
#: reading carries the controller's sample noise straight into the threshold.
CONTACT_BASELINE_SAMPLES = 3
#: A rise this many times the criterion's threshold is contact AT ONCE,
#: without waiting out ``settle_s``: the confirm window exists to reject a
#: transient, and a transient does not double the threshold.
CONTACT_ABORT_FACTOR = 2.0


@dataclass(frozen=True)
class ContactReport:
    """Where a contact leg stopped, and why — MEASURED, never the command.

    ``p_tool`` is the tool point (the pad centre) at the posture the arm
    REPORTED when the leg stopped, through the kit's own forward kinematics:
    on contact the command is ahead of the arm by exactly the amount the
    surface is pushing back, so the command is the one number that is not
    where the surface is. ``travel_m`` is how far that measured tool point
    got along the leg; ``normal_hint`` is minus the travel — the way the
    surface pushes back. ``torque_nm`` is the largest rise over the pre-motion
    baseline, on joint ``joint`` (0-based).

    ``leg_t_s`` is how far along the leg's own timing the COMMAND had got
    when it was frozen — what a retract plays back from. ``q_stop`` is the
    measured posture itself.
    """

    made: bool
    p_tool: np.ndarray
    travel_m: float
    normal_hint: np.ndarray
    torque_nm: float
    stopped_by: str
    detail: str = ""
    side: str = ""
    joint: int = -1
    q_stop: Optional[np.ndarray] = None
    leg_t_s: float = 0.0
    elapsed_s: float = 0.0

    def __post_init__(self) -> None:
        if self.stopped_by not in CONTACT_STOPS:
            raise ValueError(f"unknown contact stop {self.stopped_by!r}; the "
                             f"vocabulary is {CONTACT_STOPS}")
        object.__setattr__(self, "p_tool",
                           np.array(self.p_tool, dtype=float).reshape(3))
        object.__setattr__(self, "normal_hint",
                           np.array(self.normal_hint, dtype=float).reshape(3))
        if self.q_stop is not None:
            object.__setattr__(self, "q_stop", _vec7(self.q_stop, "q_stop"))

    def to_json(self) -> Dict[str, Any]:
        def num(value: float, places: int) -> Optional[float]:
            return (None if not np.isfinite(value)
                    else round(float(value), places))
        out: Dict[str, Any] = {
            "made": bool(self.made), "stopped_by": self.stopped_by,
            "side": self.side,
            "p_tool": [num(v, 4) for v in self.p_tool],
            "travel_m": num(self.travel_m, 4),
            "normal_hint": [num(v, 4) for v in self.normal_hint],
            "torque_nm": num(self.torque_nm, 3),
            "leg_t_s": num(self.leg_t_s, 3),
            "elapsed_s": num(self.elapsed_s, 3), "detail": self.detail}
        if self.joint >= 0:
            out["joint"] = int(self.joint)
        return out


class ContactWatch:
    """The contact criterion, applied to a stream of MEASURED arm states.

    One implementation for every transport — the streamed default, the
    firmware trajectory override, a simulator — so "contact" means the same
    thing whichever wire carried the leg. Fed ``(t, JointState)`` samples, it
    answers one of three words:

    ``free``        keep moving.
    ``confirming``  a joint's torque has risen past the threshold: FREEZE the
                    command (resend it / cancel the trajectory) and keep
                    sampling. A rise that goes away was a transient —
                    acceleration, a cable — and the motion resumes.
    ``contact``     the rise has held, with the arm stalled, for
                    ``settle_s``; or it has reached
                    :data:`CONTACT_ABORT_FACTOR` x the threshold at once.
    """

    FREE = "free"
    CONFIRMING = "confirming"
    CONTACT = "contact"

    def __init__(self, criterion: ContactCriterion, baseline_nm) -> None:
        self.criterion = criterion
        self.baseline = np.asarray(baseline_nm, dtype=float).reshape(ARM_DOF)
        self.peak_nm = 0.0
        self.joint = -1
        self._since: Optional[float] = None

    def sample(self, t: float, arm: JointState) -> str:
        rise = np.abs(np.asarray(arm.torque_nm, dtype=float) - self.baseline)
        j = int(np.argmax(rise))
        if float(rise[j]) > self.peak_nm:
            self.peak_nm, self.joint = float(rise[j]), j
        threshold = self.criterion.joint_torque_nm
        if float(rise[j]) >= CONTACT_ABORT_FACTOR * threshold:
            return self.CONTACT
        if float(rise[j]) < threshold:
            self._since = None
            return self.FREE
        still = (arm.qd is None or float(np.max(np.abs(arm.qd)))
                 <= self.criterion.stall_velocity_rad_s)
        if not still:
            self._since = None
            return self.CONFIRMING
        if self._since is None:
            self._since = float(t)
        if float(t) - self._since >= self.criterion.settle_s - 1e-9:
            return self.CONTACT
        return self.CONFIRMING


def interpolate_leg(timed: Sequence[Tuple[float, Any]], t: float) -> np.ndarray:
    """The leg's joint command at leg time ``t`` (linear between knots)."""
    times = [float(k[0]) for k in timed]
    if t <= times[0]:
        return np.asarray(timed[0][1], dtype=float).reshape(ARM_DOF)
    for (t0, q0), (t1, q1) in zip(timed, timed[1:]):
        if t <= float(t1):
            span = float(t1) - float(t0)
            f = 1.0 if span <= 0 else (float(t) - float(t0)) / span
            return (np.asarray(q0, dtype=float)
                    + f * (np.asarray(q1, dtype=float) - np.asarray(q0, dtype=float)))
    return np.asarray(timed[-1][1], dtype=float).reshape(ARM_DOF)


def leg_ticks(timed: Sequence[Tuple[float, Any]], hz: float
              ) -> List[Tuple[float, np.ndarray]]:
    """The leg resampled at ``hz``: ``(leg time, q)`` per command tick."""
    end = float(timed[-1][0])
    count = max(1, int(math.ceil(end * float(hz) - 1e-9)))
    return [(min(end, k / float(hz)), interpolate_leg(timed, k / float(hz)))
            for k in range(1, count + 1)]


def retract_path(step: ContactStep, report: "ContactReport"
                 ) -> List[Tuple[float, np.ndarray]]:
    """The leg played BACK from where its command was frozen to its start.

    ``(t, q)`` knots, ``t`` from the start of the retract at the leg's own
    speed — the same knots the guard approved on the way in, in reverse.
    """
    timed = step.timed_path()
    here = float(min(report.leg_t_s, timed[-1][0]))
    back = [(0.0, interpolate_leg(timed, here))]
    for t, q in reversed(timed):
        if float(t) < here - 1e-9:
            back.append((here - float(t), np.asarray(q, dtype=float)))
    return back


def contact_kin(executor: Any, kin=None):
    """The kinematic model a contact report is measured with: the caller's,
    the executor's own, or the kit's — the tool gate's order."""
    if kin is not None:
        return kin
    own = getattr(executor, "kin", None)
    return own if own is not None else _fallback_kinematics()


def contact_report(kin, side: str, timed, *, q_start, q_stop, made: bool,
                   stopped_by: str, torque_nm: float = float("nan"),
                   joint: int = -1, leg_t_s: float = 0.0,
                   elapsed_s: float = 0.0, detail: str = "",
                   direction=None) -> "ContactReport":
    """Build the report from two MEASURED postures and the leg's geometry.

    ``direction`` is the leg's planned travel in the base frame (the
    ``ContactStep``'s); without one it is the leg's end tool point minus its
    start one, by forward kinematics over the knots it was given.
    """
    if direction is not None:
        axis = np.asarray(direction, dtype=float).reshape(3)
    else:
        p_a, _ = ToolGate._tool(kin, side, timed[0][1])
        p_b, _ = ToolGate._tool(kin, side, timed[-1][1])
        axis = np.asarray(p_b, dtype=float) - np.asarray(p_a, dtype=float)
    n = float(np.linalg.norm(axis))
    u = axis / n if n > 1e-9 else np.zeros(3)
    p0, _ = ToolGate._tool(kin, side, q_start)
    q_end = q_start if q_stop is None else q_stop
    p1, _ = ToolGate._tool(kin, side, q_end)
    travel = float(np.dot(np.asarray(p1) - np.asarray(p0), u))
    return ContactReport(made, p1, travel, -u, float(torque_nm), stopped_by,
                         detail, side, joint,
                         None if q_stop is None else np.asarray(q_stop),
                         float(leg_t_s), float(elapsed_s))


def stream_move_until(executor: Any, path: Sequence[Tuple[float, Any]], *,
                      side: str, criterion: ContactCriterion,
                      hz: float = 50.0, kin=None, t0: float = 0.0,
                      command=None, direction=None) -> "ContactReport":
    """THE DEFAULT ``move_until``: stream the leg, watch the torque, stop.

    For any transport that streams joints (``send_joints``) and reports
    joint torque (``JointState.torque_nm``). ``path`` is the leg as
    ``(t, q)`` knots (``ContactStep.timed_path()``); it is resampled at
    ``hz`` and sent one tick at a time, and after EVERY tick the arm is read
    and the :class:`ContactWatch` asked. On ``confirming`` the command is
    frozen (the same knot is re-sent), so the arm stops pushing while the
    rise is confirmed; on ``contact`` the leg ends. Time is PLAN time — tick
    ``k`` goes out at ``t0 + k / hz`` — so a transport paces it exactly as it
    paces every other step.

    ``command`` is the 16-vector the rest of the robot is under (the other
    arm, both grippers). Without it the other arm is held where it is
    measured and each hand at its commanded closedness.

    The report is MEASURED: where the arm REPORTS it is when the leg ends,
    never the knot that was sent.
    """
    kin = contact_kin(executor, kin)
    timed = [(float(t), np.asarray(q, dtype=float).reshape(ARM_DOF))
             for t, q in path]
    period = 1.0 / float(hz)

    def unmeasured(why: str, q) -> "ContactReport":
        return contact_report(kin, side, timed, direction=direction,
                              q_start=q, q_stop=None, made=False,
                              stopped_by=UNMEASURED, detail=why)

    samples = [executor.state() for _ in range(CONTACT_BASELINE_SAMPLES)]
    first = samples[0].arms.get(side)
    if first is None:
        raise KeyError(f"{type(executor).__name__} reports no {side} arm")
    q_start = np.asarray(first.q, dtype=float)
    if criterion.tool_force_n is not None:
        return unmeasured("the criterion asks for a TOOL FORCE and this "
                          "transport estimates none; watch joint torque "
                          "(tool_force_n=None)", q_start)
    for state in samples:
        arm = state.arms.get(side)
        if arm is not None and arm.faulted:
            return contact_report(kin, side, timed, direction=direction,
                                  q_start=q_start, q_stop=arm.q, made=False,
                                  stopped_by=CONTACT_FAULT,
                                  detail=controller_fault(state) or "")
        if arm is None or arm.torque_nm is None:
            return unmeasured(f"{type(executor).__name__} publishes no joint "
                              f"torque for the {side} arm, so nothing could "
                              f"tell contact from free motion; the leg was "
                              f"not started", q_start)
    baseline = np.mean([s.arms[side].torque_nm for s in samples], axis=0)
    watch = ContactWatch(criterion, baseline)
    if command is None:
        state = samples[-1]
        joints = {s: np.asarray(a.q, dtype=float) for s, a in state.arms.items()}
        grippers = {}
        for s, hand in state.hands.items():
            value = hand.commanded if hand.commanded is not None else hand.closedness
            if value is not None:
                grippers[s] = float(value)
        command = wire(joints, grippers)
    vector = np.array(command, dtype=float).reshape(WIRE_DIM)
    ticks = leg_ticks(timed, hz)
    index = -1
    leg_t = 0.0
    elapsed = 0.0
    tail = 0.0
    t = float(t0)
    verdict = ContactWatch.FREE
    while True:
        if verdict == ContactWatch.FREE and index + 1 < len(ticks):
            index += 1
            leg_t, q_cmd = ticks[index]
            vector[JOINT_SLICE[side]] = q_cmd
        executor.send_joints(vector, t=t)
        t += period
        elapsed += period
        state = executor.state()
        arm = state.arms.get(side)
        if arm is None or arm.torque_nm is None:
            return contact_report(
                kin, side, timed, direction=direction, q_start=q_start,
                q_stop=None if arm is None else arm.q, made=False,
                stopped_by=UNMEASURED, leg_t_s=leg_t, elapsed_s=elapsed,
                detail="the arm's torque stopped being reported mid-leg")
        if arm.faulted:
            return contact_report(
                kin, side, timed, direction=direction, q_start=q_start,
                q_stop=arm.q, made=False, stopped_by=CONTACT_FAULT,
                torque_nm=watch.peak_nm, joint=watch.joint, leg_t_s=leg_t,
                elapsed_s=elapsed,
                detail=controller_fault(state) or "")
        verdict = watch.sample(elapsed, arm)
        if verdict == ContactWatch.CONTACT:
            return contact_report(
                kin, side, timed, direction=direction, q_start=q_start,
                q_stop=arm.q, made=True, stopped_by=CONTACT,
                torque_nm=watch.peak_nm, joint=watch.joint, leg_t_s=leg_t,
                elapsed_s=elapsed,
                detail=(f"joint {watch.joint + 1} rose "
                        f"{watch.peak_nm:.2f} Nm over its pre-motion torque "
                        f"(threshold {criterion.joint_torque_nm:.2f} Nm)"))
        if verdict == ContactWatch.FREE and index + 1 >= len(ticks):
            # THE END OF THE LEG IS WATCHED TOO, for one settle window: a
            # surface met on the last tick is still a surface.
            tail += period
            if tail >= criterion.settle_s - 1e-9:
                return contact_report(
                    kin, side, timed, direction=direction, q_start=q_start,
                    q_stop=arm.q, made=False, stopped_by=MAX_TRAVEL,
                    torque_nm=watch.peak_nm, joint=watch.joint,
                    leg_t_s=leg_t, elapsed_s=elapsed,
                    detail=(f"travelled the whole leg and nothing resisted "
                            f"(largest rise {watch.peak_nm:.2f} Nm, threshold "
                            f"{criterion.joint_torque_nm:.2f} Nm)"))


class StreamingContact:
    """Mixin: give a streaming transport the kit's default ``move_until``.

    A transport opts IN — ``move_until`` is a capability, and a transport
    that has not said it can watch torque must fail a contact step rather
    than have one played on it. Requires ``state()`` publishing
    ``JointState.torque_nm`` and ``send_joints``.
    """

    def move_until(self, path, *, side: str, criterion: ContactCriterion,
                   hz: float = 50.0, kin=None, t0: float = 0.0,
                   command=None, direction=None) -> "ContactReport":
        return stream_move_until(self, path, side=side, criterion=criterion,
                                 hz=hz, kin=kin, t0=t0, command=command,
                                 direction=direction)


#: Default barriers. Both are generous — they are deadlines, not budgets.
ARRIVE_TOL_RAD = math.radians(3.0)
ARRIVE_TIMEOUT_S = 3.0
STROKE_TIMEOUT_S = 3.0

#: How far the TOOL POINT may be from the commanded one at a waypoint that
#: says :attr:`~manipulation_kit.primitives.types.Waypoint.arrive` [m].
#:
#: 5 mm, and the number is picked from both ends. Below it, the driven jaws
#: still close on a blocks-eval cube: the graspable width is 43.96 mm against
#: a 40 mm block, so 4 mm of the opening is spare per side and a 5 mm miss is
#: inside the pads. Above it, nothing is gained by being stricter — the
#: planner's own knot tolerance is 3 mm (``planning.ARRIVE_TOL_M``) and the
#: IK converges to about 2 mm, so a 3 mm gate would fire on the solver's own
#: convergence noise and correct what is already as good as the plan.
ARRIVE_TOL_M = 0.005
#: How far the tool point may be short of (or past) the waypoint ALONG the
#: approach axis [m]. A different question with a different answer.
#:
#: ACROSS the axis is where a grasp is won or lost: the jaws close on that
#: plane, and :data:`ARRIVE_TOL_M` is the slack the pads have. ALONG it, the
#: tool point is the pad CENTRE of a 58 mm deep pad, so 10 mm still leaves two
#: thirds of the pad on the object — and the last millimetres of a descent are
#: taken up by CONTACT, deliberately: ``orientation.SUPPORT_CLEARANCE_M`` stops
#: the fingertips 3 mm above what the object stands on and a position-
#: controlled arm parks a few mm high when they touch. MEASURED on blocks-eval
#: (2026-09-21), grasps whose jaws closed correctly sat 3.8-4.4 mm short along
#: the axis and 2-4 mm across it, while the ones that jammed were 13-17 mm
#: short AND 10-12 mm across. One number for both would either refuse every
#: working grasp or accept the jams.
ARRIVE_TOL_ALONG_M = 0.010
#: ...and the orientation half, 5 deg.
#:
#: Bounded from BOTH sides by numbers this package already owns, and the band
#: is narrower than it looks. Below: the IK's own rotation convergence is
#: ``safety.IK_ROT_TOL`` = 0.05 rad = 2.9 deg and the planner walks a path
#: window of ``planning.PATH_TOL_RAD`` = 6.9 deg, so a gate tighter than about
#: 3 deg refuses postures the planner itself calls converged — and asks the
#: correction to re-solve to a tolerance the solver does not have. Above: a
#: parallel gripper is forgiving in roll until the object's PRESENTED width
#: grows past the jaws, which for a 40 mm cube in a 43.96 mm opening happens
#: at 11.7 deg (``orientation.grasp_orientation``). 5 deg sits clear of the
#: solver's noise and well inside where the geometry bites.
#:
#: (The 2 deg this was first written with was below IK_ROT_TOL: measured on
#: blocks-eval, grasps whose jaws closed correctly tracked 2.5-3.0 deg of
#: wrist error, which is ~1 mm of contact-line shift on a 40 mm block.)
ARRIVE_TOL_ROT_RAD = math.radians(5.0)
#: How long the arm is given to STOP at a gated waypoint before the tool point
#: is read [s].
#:
#: A measurement taken while the arm is still converging is not a steady-state
#: offset, and feeding it forward corrects for a position the arm was passing
#: through. MEASURED on blocks-eval (2026-09-21): reading at the instant the
#: 3 deg joint gate passed, three rounds took 23.8 -> 14.9 -> 9.1 -> 7.2 mm
#: and never converged, because each round was measuring the same descent at a
#: different moment of the same settle. With the arm stopped first, one round
#: does it.
ARRIVE_SETTLE_S = 2.0
#: Below this, the ARM is stopped as far as this barrier is concerned [deg/s].
#:
#: A settle is allowed to be about the whole robot — the Isaac one waits for
#: the jaws too, deliberately, because a verifier read while a stroke is still
#: closing is F13. This barrier is not asking that question: it wants to know
#: whether the TOOL POINT has stopped moving, and a hand still ramping open
#: says nothing about it. MEASURED on blocks-eval: a grasp was refused with
#: "still moving at 0.0 deg/s ... the right jaws are still moving", i.e. on a
#: perfectly stationary arm. So the flag is read WITH its number, and 3 deg/s
#: is the same threshold the harness itself calls stationary.
ARM_STATIONARY_DEG_S = 3.0
#: How many ``solve_ee`` calls ONE correction may take. The same number, and
#: the same reason, as ``planning.MAX_SOLVES_PER_KNOT``: the solver is local
#: and works on LINK7, so reaching a TOOL pose is a few small solves, and
#: eight means it is circling.
MAX_CORRECTION_SOLVES = 8
#: ...and how close it has to get before the correction is sent. The planner's
#: own knot tolerances (``planning.ARRIVE_TOL_M`` / ``ARRIVE_TOL_RAD``): asking
#: the correction to beat the path that produced the waypoint is asking the
#: same solver for a precision it has already been shown not to have.
SOLVE_TOL_M = 0.003
SOLVE_TOL_RAD = 0.06
#: How many in-place corrections one waypoint gets before the run stops.
#:
#: Two. The correction feeds the measured steady-state offset forward, so the
#: first round removes most of a systematic droop and the second removes what
#: the first one's own new posture adds; a third round that has not converged
#: is not converging, and the honest answer is the typed refusal rather than
#: an arm nodding at a block while the model waits.
MAX_ARRIVAL_CORRECTIONS = 2


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

    OPTIONAL CAPABILITIES, duck-typed (``hasattr``) like ``run_plan`` and
    ``begin_run``/``end_run``, because a transport without the hardware has
    nothing to say about it:

    ``run_plan(plan, *, hz, arrive_tol_rad, arrive_timeout_s,
    stroke_timeout_s, gate)``
        take a whole plan at once (the firmware trajectory upload).
    ``neck_state() -> NeckState | None`` / ``lift_state() -> LiftState | None``
        the head and the torso lift, read through the transport. See
        :func:`read_neck` / :func:`read_lift`.
    ``firmware_spec: str``
        which firmware contract this transport drives; recorded in a plan's
        binding and compared by :func:`check_binding`.
    ``move_until(path, *, side, criterion, hz, kin) -> ContactReport``
        play a contact leg (``ContactStep``) as a position-commanded motion
        WATCHED by :class:`ContactWatch`, stop it on the criterion, and say
        where the tool stopped, measured. :func:`stream_move_until` is the
        default for any streaming transport that reports joint torque
        (opt in with :class:`StreamingContact`); the firmware transport
        overrides it with a trajectory upload, a state poll and a cancel;
        the kinematic mirror reports ``made=False, stopped_by="max_travel"``.
        A transport without it FAILS a contact step (``transport_error``) —
        a contact leg is never played blind.

    MODES: position only. This kit commands position control and nothing
    else (Shu, 2026-09-22); ``JointState.mode`` is READ so a latched
    controller stops the run (:data:`CONTROLLER_FAULT`), never set.
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
#: An arm controller is LATCHED (``JointState.faulted``: mode ``error`` or a
#: non-zero error code). Checked before the first byte and between steps, so
#: the run stops in the kit rather than in whichever example remembered to
#: look (d1-2 run5, 2026-09-22: the loop kept planning on a latched arm).
CONTROLLER_FAULT = "controller_fault"
STOP_REASONS: Tuple[str, ...] = (NOT_BOUND, UNGUARDED, STALE_BINDING,
                                 REFUSED_PLAN, BARRIER_FAILED, TRANSPORT_ERROR,
                                 CONTROLLER_FAULT)

#: WHY a barrier failed, in the same vocabulary a plan refusal uses.
#:
#: ``stop_reason`` says a barrier stopped the run; this says which one and
#: with what numbers, so "the right tool point is 27 mm from the grasp pose
#: after 2 corrections" reaches the caller instead of a jaw-stall symptom
#: three steps later.
ARRIVED_OFF_BY = "arrived_off_by"
ARRIVAL_UNKNOWN = "arrival_unknown"
NOT_SETTLED = "not_settled"
STROKE_UNFINISHED = "stroke_unfinished"
#: The GRIPPER is faulted: the producer reports a fault outcome or a fault
#: code, and no further stroke does anything until it is cleared. Its own
#: reason, not ``stroke_unfinished``, because the answers differ — an
#: unfinished stroke can be waited for, a faulted gripper cannot. (d1-2,
#: 2026-09-22: a firm hold wound its torque to -4.17 Nm, the motor latched,
#: and every later stroke ended ``fault`` with the jaws stationary — which
#: the barrier accepted as a finished stroke. Design review, finding 13.)
GRIPPER_FAULT = "gripper_fault"
#: A stroke the plan marked ``GripStep.expect_hold`` finished WITHOUT a
#: measured hold (``StrokeReport.holding`` false or unknown). The run stops
#: before the next step, because that step depends on the hold: in a
#: ``handover`` it is the giving hand opening, and a receiver that closed on
#: air would turn it into a drop.
HOLD_NOT_CONFIRMED = "hold_not_confirmed"
RUN_REASONS: Tuple[str, ...] = (ARRIVED_OFF_BY, ARRIVAL_UNKNOWN, NOT_SETTLED,
                                STROKE_UNFINISHED, GRIPPER_FAULT,
                                HOLD_NOT_CONFIRMED, CONTROLLER_FAULT)


@dataclass(frozen=True)
class RunRefusal:
    """Why execution stopped, as data rather than as a sentence.

    Deliberately the SHAPE a plan refusal already has
    (:meth:`~manipulation_kit.primitives.types.PlanError.to_json`,
    ``manipulation_kit.refusal/2``): same keys, same units — ``residual_m`` is
    metres and ``residual_rad`` is radians — so a caller that can read one
    refusal can read the other without a second parser. Only the REASON
    vocabulary is new (:data:`RUN_REASONS`), which is why the schema version
    does not move: nothing that could read a refusal/2 object can read this
    one any less well.
    """

    reason: str
    detail: str = ""
    waypoint_label: str = ""
    residual_m: float = float("nan")
    residual_rad: float = float("nan")
    stage: str = ""
    attempted: Tuple[str, ...] = ()
    primitive: str = ""
    side: str = ""

    def __post_init__(self) -> None:
        if self.reason not in RUN_REASONS:
            raise ValueError(f"unknown execution-refusal reason "
                             f"{self.reason!r}; the vocabulary is {RUN_REASONS}")

    def to_json(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {"schema": "manipulation_kit.refusal/2",
                               "reason": self.reason, "detail": self.detail,
                               "primitive": self.primitive, "side": self.side}
        if self.waypoint_label:
            out["waypoint_label"] = self.waypoint_label
        if np.isfinite(self.residual_m):
            out["residual_m"] = round(float(self.residual_m), 5)
        if np.isfinite(self.residual_rad):
            out["residual_rad"] = round(float(self.residual_rad), 5)
        if self.stage:
            out["stage"] = self.stage
        if self.attempted:
            out["attempted"] = list(self.attempted)
        return out


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
    #: the typed half of ``error``, when the stop has one
    refusal: Optional[RunRefusal] = None
    #: every contact leg's MEASURED outcome, in order
    contacts: Tuple["ContactReport", ...] = ()

    def to_json(self) -> Dict[str, Any]:
        out = {"primitive": self.plan_primitive, "side": self.side,
               "completed": self.completed, "steps_sent": self.steps_sent,
               "settled": None if self.settle is None else self.settle.settled,
               "stop_reason": self.stop_reason,
               "stopped_at": self.stopped_at,
               "arrivals": [a.to_json() for a in self.arrivals],
               "strokes": [s.to_json() for s in self.strokes],
               "error": self.error}
        if self.refusal is not None:
            out["refusal"] = self.refusal.to_json()
        if self.contacts:
            out["contacts"] = [c.to_json() for c in self.contacts]
        return out


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
    spec = getattr(binding, "firmware_spec", "")
    have = firmware_spec_of(executor)
    if spec and have is not None and spec != have:
        return (f"this plan was checked against an observation from firmware "
                f"client spec {spec[:12]}, and this executor drives spec "
                f"{have[:12]}; a plan is not played against a different "
                f"firmware than the one it was checked on — re-observe and "
                f"replan")
    state = executor.state()
    stamp = state.stamp if math.isfinite(state.stamp) else now
    return binding.drift(joints=state.joints, now=stamp,
                         tool_revision=tool_revision())


def firmware_spec_of(executor: Any) -> Optional[str]:
    """Which firmware contract ``executor`` drives, or ``None`` if it cannot say.

    For d1-firmwared, the sha256 of the OpenAPI document its client was
    generated from; ``"kinematic"`` for the kinematic mirror, so a dry-run's
    plan is refused on hardware and vice versa. A transport without the
    attribute (a recorder, an out-of-tree executor) is not checked.
    """
    spec = getattr(executor, "firmware_spec", None)
    return None if spec in (None, "") else str(spec)


def controller_fault(state: "RawState") -> Optional[str]:
    """The reason to stop because an arm controller is latched, or ``None``."""
    faults = state.faults()
    if not faults:
        return None
    return ("; ".join(faults[s] for s in sorted(faults))
            + ". A latched controller moves nothing and its feedback looks "
              "like a still arm; clear the error on the robot (console: "
              "Clear error, then Home) and re-observe before planning again")


def stroke_refusal(plan: Plan, side: str, stroke: "StrokeReport") -> "RunRefusal":
    """Why a gripper step stopped the run, in the refusal vocabulary."""
    if stroke.faulted:
        return RunRefusal(
            GRIPPER_FAULT,
            f"the {side} gripper is FAULTED ({stroke.fault}"
            + (f", stroke {stroke.kind}" if stroke.kind else "")
            + f"): {stroke.detail}. The jaws are stationary because the motor "
              f"is not driving them, not because the stroke finished",
            stage="gripper_stroke", primitive=plan.primitive, side=side)
    return RunRefusal(
        STROKE_UNFINISHED,
        f"the {side} gripper stroke did not reach a terminal state: "
        f"{stroke.detail}",
        stage="gripper_stroke", primitive=plan.primitive, side=side)


def hold_refusal(plan: Plan, step: GripStep, stroke: "StrokeReport"
                 ) -> Optional["RunRefusal"]:
    """The refusal for a settled stroke whose plan needed a HOLD, or ``None``.

    Only for a :class:`GripStep` with ``expect_hold``; every other stroke is
    judged by :func:`stroke_refusal` alone. Unknown (``holding is None``) is
    a refusal too: the step after this one is only safe on a measured hold.
    """
    if not getattr(step, "expect_hold", False) or stroke.holding is True:
        return None
    measured = ("reports nothing held" if stroke.holding is False
                else "cannot say whether it holds anything")
    return RunRefusal(
        HOLD_NOT_CONFIRMED,
        f"the {step.side} gripper closed ({stroke.kind or 'stroke settled'}) "
        f"and {measured}; the next step depends on that hold, so the run "
        f"stops here with everything else as it is",
        stage="gripper_stroke", primitive=plan.primitive, side=step.side)


def fault_refusal(plan: Plan, detail: str) -> "RunRefusal":
    return RunRefusal(CONTROLLER_FAULT, detail, stage="controller",
                      primitive=plan.primitive, side=plan.side)


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


def _final_arrival(executor: "Executor", plan: Plan, q16, *, tol_rad: float,
                   timeout_s: float, gate: Optional["ToolGate"] = None
                   ) -> ArrivalReport:
    """The joint-space barrier at the END of a plan's motion.

    The same :func:`_arrival_of` a stroke waits on, run where no stroke and no
    gated waypoint follows the last leg: before a settle and at the end of the
    plan. Without it a transport's ``completed`` WAS the run's ``completed``
    (d1-2, 2026-09-22: an Approach whose J7 stopped 50 deg short of its
    command, the tool 167 mm and 53 deg off, reported ``completed`` with no
    arrival at all). When the barrier fails and a kinematic model is at hand,
    the report carries the TOOL miss too — "50 deg on one joint" and "167 mm
    at the tool" are the same fact, and the second is the one a reader acts on.
    """
    label = "end of motion"
    arrival = _arrival_of(executor, q16, tol_rad=tol_rad, timeout_s=timeout_s)
    if arrival.arrived:
        return _with_label(arrival, label)
    detail = (f"the {plan.side} arm is not at the posture the plan ended on: "
              f"{arrival.detail}")
    miss = None
    if gate is not None and plan.side in JOINT_SLICE:
        try:
            q_meas = executor.state().joints.get(plan.side)
            if q_meas is not None:
                want = np.asarray(q16, dtype=float).reshape(WIRE_DIM)
                miss = gate._miss(gate.model(executor), plan.side,
                                  want[JOINT_SLICE[plan.side]],
                                  np.asarray(q_meas, dtype=float))
        except Exception:  # noqa: BLE001 - the joint answer stands alone
            miss = None
    if miss is not None:
        detail = f"{detail}; {miss.sentence(plan.side, label)}"
    return _with_label(arrival, label, detail=detail, miss=miss)


def _stroke_of(executor: "Executor", side: str, *, timeout_s: float
               ) -> StrokeReport:
    wait = getattr(executor, "wait_gripper_settled", None)
    if wait is None:
        return StrokeReport(False, detail=(
            f"{type(executor).__name__} implements no wait_gripper_settled, "
            f"so the stroke's completion cannot be measured"))
    return wait(side, timeout_s=timeout_s)


# --------------------------------------------------------------------------- #
# the tool-space barrier
# --------------------------------------------------------------------------- #

#: the arm model the barrier falls back to when nobody hands it one
DEFAULT_ARM_MODEL = "d1/arm"
_FALLBACK_KIN: Dict[str, Any] = {}


def _fallback_kinematics(model: str = DEFAULT_ARM_MODEL):
    """The kit's own guarded model, built once per process and shared.

    A last resort, not a default worth relying on: the barrier's corrections
    are collision-checked against THIS model's guard, so a caller whose plan
    was made against a differently configured guard should pass that model in
    (``run(..., kin=kin)``) rather than let a second one be built behind its
    back. Built with the same factory and the same defaults the planners use,
    so when it IS used it is the same geometry and the same guard config.
    """
    if model not in _FALLBACK_KIN:
        from .arms import get_arm_kinematics  # noqa: PLC0415 - optional cost
        _FALLBACK_KIN[model] = get_arm_kinematics(model, quiet=True)
    return _FALLBACK_KIN[model]


@dataclass(frozen=True)
class ToolMiss:
    """One tool-pose error, split the way the gripper feels it.

    ``across_m`` is the component perpendicular to the approach axis — the
    plane the jaws close in, and the one that decides whether the pads land on
    the object or beside it. ``along_m`` is the component down that axis: how
    deep the descent got, signed positive PAST the waypoint.
    """

    total_m: float
    across_m: float
    along_m: float
    rot_rad: float
    p_cmd: np.ndarray
    r_cmd: Any
    p_meas: np.ndarray
    r_meas: Any

    @classmethod
    def between(cls, p_cmd, r_cmd, p_meas, r_meas) -> "ToolMiss":
        p_cmd = np.asarray(p_cmd, dtype=float)
        p_meas = np.asarray(p_meas, dtype=float)
        delta = p_meas - p_cmd
        #: the TCP frame's +z is the approach axis (see primitives.orientation)
        axis = np.asarray(r_cmd.as_matrix()[:, 2], dtype=float)
        along = float(np.dot(delta, axis))
        return cls(float(np.linalg.norm(delta)),
                   float(np.linalg.norm(delta - along * axis)), along,
                   float(np.linalg.norm((r_meas.inv() * r_cmd).as_rotvec())),
                   p_cmd, r_cmd, p_meas, r_meas)

    def sentence(self, side: str, label: str) -> str:
        return (f"the {side} tool point is {self.across_m * 1000:.1f} mm "
                f"across the approach axis and {self.along_m * 1000:+.1f} mm "
                f"along it from the pose commanded at "
                f"{label or 'this waypoint'}, {math.degrees(self.rot_rad):.1f} "
                f"deg off")


class ToolGate:
    """"Is the JAW POCKET where the plan said?" — measured, and corrected.

    The joint-space barrier (:meth:`Executor.wait_arrived`) answers a question
    about seven angles. This one answers the question the verb was about, by
    running the kit's own forward kinematics over the two postures the
    executor can already report — the one it was COMMANDED and the one it
    MEASURES — and comparing the tool points. Nothing new is asked of the
    transport: no RPC, no pose read, no extra protocol method.

    When the tool is off and the arm has settled there, the offset is a
    steady-state error (gravity droop, a controller's deadband), so it is fed
    FORWARD: re-solve the same commanded tool pose shifted by minus the
    measured offset, seeded at the commanded joints, guard it, send it, look
    again. At most :data:`MAX_ARRIVAL_CORRECTIONS` rounds, then a typed
    refusal — never a stroke onto an unverified pose.
    """

    def __init__(self, *, kin=None, tol_rad: float = ARRIVE_TOL_RAD,
                 timeout_s: float = ARRIVE_TIMEOUT_S,
                 tol_m: float = ARRIVE_TOL_M,
                 tol_along_m: float = ARRIVE_TOL_ALONG_M,
                 tol_rot_rad: float = ARRIVE_TOL_ROT_RAD,
                 correct: bool = True,
                 max_rounds: int = MAX_ARRIVAL_CORRECTIONS,
                 settle_timeout_s: float = ARRIVE_SETTLE_S,
                 guarded: bool = True):
        self.kin = kin
        self.tol_rad = float(tol_rad)
        self.timeout_s = float(timeout_s)
        self.tol_m = float(tol_m)
        self.tol_along_m = float(tol_along_m)
        self.tol_rot_rad = float(tol_rot_rad)
        self.correct = bool(correct)
        self.max_rounds = int(max_rounds)
        self.settle_timeout_s = float(settle_timeout_s)
        #: was the PLAN guarded? Then so is every posture this gate invents.
        self.guarded = bool(guarded)

    # -- kinematics -------------------------------------------------------- #
    def model(self, executor: "Executor"):
        """The kinematic model to compute with: the caller's, the executor's,
        or the kit's own — in that order, and the choice is stated once."""
        if self.kin is not None:
            return self.kin
        own = getattr(executor, "kin", None)
        if own is not None:
            return own
        return _fallback_kinematics()

    @staticmethod
    def _tool(kin, side: str, q) -> Tuple[np.ndarray, Any]:
        """The tool pose for a joint vector, with the model put back.

        Same borrow-and-restore contract as
        :class:`manipulation_kit.primitives.planning.Kin`: the model is shared
        with whoever is planning on it, so the lock is held and the joints go
        back even on the way out through an exception.
        """
        lock = getattr(kin, "lock", None)
        if lock is not None:
            lock.acquire()
        try:
            saved = np.array(kin.joints(side), dtype=float)
            try:
                kin.set_joints(side, np.asarray(q, dtype=float).reshape(ARM_DOF))
                p7, r7 = kin.ee_pose(side)
            finally:
                kin.set_joints(side, saved)
        finally:
            if lock is not None:
                lock.release()
        return tool_from_link7(p7, r7)

    def _measured(self, executor: "Executor", side: str
                  ) -> Tuple[Optional[np.ndarray], Optional[SettleReport],
                             Optional[RawState]]:
        """The arm's posture ONCE IT HAS STOPPED, and the settle that says so.

        Not a nicety: the correction feeds a STEADY-STATE offset forward, and
        a posture read mid-settle is not one. Measured on blocks-eval, reading
        at the instant the coarse joint gate passed made every round correct
        for a pose the arm was travelling through — 23.8, 14.9, 9.1, 7.2 mm,
        converging on nothing.
        """
        settle = None
        wait = getattr(executor, "settle", None)
        if wait is not None and self.settle_timeout_s > 0:
            settle = wait(self.settle_timeout_s)
            if not settle.settled and _arm_stopped(settle):
                # It failed on something that is not the arm — the jaws, on
                # every executor whose settle waits for them too. The tool
                # point is not moving, which is the only thing this barrier
                # asked.
                settle = SettleReport(
                    True, settle.waited_s, settle.worst_velocity_deg_s,
                    f"the arms are stationary at "
                    f"{settle.worst_velocity_deg_s:.1f} deg/s ({settle.detail})")
        state = executor.state()
        q = state.joints.get(side)
        return (None if q is None
                else np.asarray(q, dtype=float).reshape(ARM_DOF)), settle, state

    def _miss(self, kin, side: str, q_cmd, q_meas) -> "ToolMiss":
        """How far the tool ended up from where it was sent, DECOMPOSED.

        Along the approach axis and across it, because the two mean different
        things: across is where the jaws close, along is how deep the descent
        got — and a descent is stopped by CONTACT on purpose.
        """
        p_cmd, r_cmd = self._tool(kin, side, q_cmd)
        p_meas, r_meas = self._tool(kin, side, q_meas)
        return ToolMiss.between(p_cmd, r_cmd, p_meas, r_meas)

    def _ok(self, miss: "ToolMiss") -> bool:
        return (miss.across_m <= self.tol_m
                and abs(miss.along_m) <= self.tol_along_m
                and miss.rot_rad <= self.tol_rot_rad)

    # -- the barrier ------------------------------------------------------- #
    def check(self, executor: "Executor", q16, side: str, *, label: str = "",
              t: float = 0.0) -> Tuple[ArrivalReport, Optional[np.ndarray]]:
        """Gate one waypoint. Returns the report and the vector now commanded.

        The second half of the pair is what makes a correction real to the
        rest of the run: after a corrected posture is sent, THAT is the
        command the arm is under, and the steps that follow have to carry it
        rather than the one that missed.
        """
        want = np.asarray(q16, dtype=float).reshape(WIRE_DIM)
        arrival = _arrival_of(executor, want, tol_rad=self.tol_rad,
                              timeout_s=self.timeout_s)
        try:
            kin = self.model(executor)
        except Exception as exc:  # noqa: BLE001 - no model, no tool answer
            return (_with_label(arrival, label, detail=(
                f"{arrival.detail}; and the tool point could not be computed: "
                f"no kinematic model ({exc})")), None)
        q_meas, settle, state = self._measured(executor, side)
        if q_meas is None:
            return (ArrivalReport(
                False, arrival.worst_error_rad, arrival.waited_s,
                f"{type(executor).__name__} reports no measured joints for the "
                f"{side} arm, so where the tool ended up cannot be measured",
                waypoint_label=label), None)
        miss = self._miss(kin, side, want[JOINT_SLICE[side]], q_meas)
        if not arrival.arrived and settle is not None and settle.settled:
            # THE BARRIER'S OWN WAIT IS NOT THE ARM'S LAST CHANCE. The joint
            # gate has a deadline (2 s of an Isaac tick budget, 3 s of wall
            # clock on the robot) and a long travel can still be converging
            # when it expires — before this gate existed, that travel simply
            # ran on into the next leg. The settle that follows holds the same
            # command and lets it finish, so the honest question is where the
            # arm is NOW, not where it was when the clock ran out.
            worst = _joint_error(state, want)
            if np.isfinite(worst) and worst <= self.tol_rad:
                arrival = ArrivalReport(
                    True, worst, arrival.waited_s + settle.waited_s,
                    f"arrived once the arm stopped ({arrival.detail})")
        if not arrival.arrived:
            # The joint barrier failed FIRST. It stays the answer — the arm is
            # still travelling, or it stopped short — but the tool numbers go
            # with it, because "2.8 deg short" and "27 mm off at the tool" are
            # the same fact said usefully.
            return (_with_label(arrival, label, miss=miss), None)
        if settle is not None and not settle.settled:
            # STILL MOVING, so there is no tool point to report: the number
            # just read is where the arm was passing, not where it is going to
            # be, and a measurement on a moving robot is not a measurement
            # (F13). Nothing to feed forward either — a correction needs a
            # STEADY-STATE offset.
            return (_report(False, arrival, miss, label, settled=False, detail=(
                f"the {side} arm had not stopped at "
                f"{label or 'this waypoint'}, so where the tool ended up is "
                f"not yet a fact: {settle.detail}")), None)
        if self._ok(miss):
            return (_report(True, arrival, miss, label,
                            settled=None if settle is None else settle.settled,
                            detail=miss.sentence(side, label)), None)
        return self._correct(executor, kin, want, side, label=label, t=t,
                             arrival=arrival, miss=miss)

    # -- the correction ---------------------------------------------------- #
    def _refuse(self, arrival: ArrivalReport, miss: "ToolMiss", label: str,
                rounds: Sequence[Correction], why: str,
                settled: Optional[bool] = True
                ) -> Tuple[ArrivalReport, Optional[np.ndarray]]:
        return (_report(False, arrival, miss, label, settled=settled,
                        detail=why, corrections=rounds), None)

    def _tolerances(self) -> str:
        return (f"(tolerance {self.tol_m * 1000:.0f} mm across, "
                f"{self.tol_along_m * 1000:.0f} mm along, "
                f"{math.degrees(self.tol_rot_rad):.0f} deg)")

    def _correct(self, executor, kin, want: np.ndarray, side: str, *,
                 label: str, t: float, arrival: ArrivalReport, miss: "ToolMiss"
                 ) -> Tuple[ArrivalReport, Optional[np.ndarray]]:
        """Feed the measured LATERAL offset forward, up to ``max_rounds`` times."""
        off = f"{miss.sentence(side, label)} {self._tolerances()}"
        if miss.across_m <= self.tol_m and miss.rot_rad <= self.tol_rot_rad:
            # ONLY THE DEPTH IS WRONG, and depth is what contact takes: the
            # descent stopped short of the waypoint. Commanding it deeper
            # presses the fingers into whatever stopped them — that is F5, ten
            # grasps out of ten — so this is a refusal and the answer belongs
            # to the caller: re-observe, nudge, or approach from elsewhere.
            return self._refuse(
                arrival, miss, label, (),
                f"{off}. The jaws are lined up and the descent stopped "
                f"{abs(miss.along_m) * 1000:.1f} mm short; pushing deeper "
                f"would drive the fingers into whatever stopped them")
        if not self.correct or self.max_rounds < 1:
            return self._refuse(arrival, miss, label, (),
                                f"{off}, and correction is switched off")
        gate = getattr(kin, "gate", None)
        if self.guarded and not getattr(gate, "installed", False):
            # The PLAN was checked by a guard. A posture this gate invents is
            # a posture nobody checked, and inventing it against a model with
            # no guard installed is exactly what the plan's own ``guarded``
            # flag exists to stop.
            return self._refuse(
                arrival, miss, label, (),
                f"{off}. It cannot be corrected: this plan was checked by a "
                f"collision guard and the model given to the barrier has none "
                f"installed, so the corrected posture could not be guarded")
        rounds: List[Correction] = []
        command = np.array(want, dtype=float)
        sent: Optional[np.ndarray] = None
        for index in range(1, self.max_rounds + 1):
            before = miss.across_m
            q_meas, _settle, _state = self._measured(executor, side)
            if q_meas is None:
                return self._refuse(
                    arrival, miss, label, rounds,
                    f"{off}, and the {side} arm's joints cannot be read")
            q_new, why = self._solve(kin, side, command, q_meas, miss)
            if q_new is None:
                rounds.append(Correction(index, before, float("nan"),
                                         sent=False, detail=why))
                return self._refuse(arrival, miss, label, rounds,
                                    f"{off}. The correction was refused: {why}")
            command = np.array(command, dtype=float)
            command[JOINT_SLICE[side]] = q_new
            executor.send_joints(command, t=t)
            sent = command
            again = _arrival_of(executor, command, tol_rad=self.tol_rad,
                                timeout_s=self.timeout_s)
            q_meas, settled, _state = self._measured(executor, side)
            if q_meas is None:
                rounds.append(Correction(
                    index, before, float("nan"),
                    detail="the joints could not be read back"))
                return self._refuse(
                    arrival, miss, label, rounds,
                    f"{off}, and after the correction the {side} arm's joints "
                    f"could not be read back")
            # AGAINST THE ORIGINAL COMMANDED POSE. The shifted target is a
            # means; landing on the pose the plan asked for is the end.
            p_now, r_now = self._tool(kin, side, q_meas)
            miss = ToolMiss.between(miss.p_cmd, miss.r_cmd, p_now, r_now)
            rounds.append(Correction(index, before, miss.across_m))
            if settled is not None and not settled.settled:
                return self._refuse(
                    arrival, miss, label, rounds,
                    f"{off}, and after the correction the {side} arm had not "
                    f"stopped: {settled.detail}", False)
            if not again.arrived:
                return self._refuse(
                    arrival, miss, label, rounds,
                    f"{off}, and the corrected posture was not reached: "
                    f"{again.detail}")
            if self._ok(miss):
                return (_report(
                    True, again, miss, label, settled=True,
                    detail=(f"{miss.sentence(side, label)} after {index} "
                            f"correction{'' if index == 1 else 's'}"),
                    corrections=rounds), sent)
        return self._refuse(
            arrival, miss, label, rounds,
            f"{miss.sentence(side, label)} after {len(rounds)} corrections "
            f"{self._tolerances()}")

    def _solve(self, kin, side: str, command: np.ndarray, q_meas,
               miss: "ToolMiss") -> Tuple[Optional[np.ndarray], str]:
        """The same tool pose, shifted by minus the measured LATERAL offset.

        An arm that lands 8 mm to the side under a command is asked for a
        command 8 mm the other way, and lands where the plan wanted it. The
        offset is measured against the pose the arm is CURRENTLY commanded at —
        which after the first round is no longer the plan's — so the rounds
        compose instead of each re-applying the whole of the first error.

        ONLY the lateral half is fed forward. The component along the approach
        axis is what contact takes, and asking a stopped arm to go deeper is
        how fingers jam (F5); a descent that stopped short is a replan, not a
        shove.

        Seeded at the commanded joints, so the solver stays in the branch the
        plan chose. Guarded twice: ``solve_ee`` consults the gate itself, and
        the committed vector is re-checked here, because a correction that is
        only as safe as one call is one refactor away from being safe by
        accident. The orientation is fed forward only when it is itself out of
        tolerance: a gripper is forgiving about roll and a correction that
        chases rotation noise spends a round for nothing.
        """
        q_cmd = np.asarray(command, dtype=float)[JOINT_SLICE[side]]
        lock = getattr(kin, "lock", None)
        if lock is not None:
            lock.acquire()
        try:
            saved = {s: np.array(kin.joints(s), dtype=float)
                     for s in getattr(kin, "sides", SIDES)}
            try:
                # BOTH arms at what they are commanded: the guard judges the
                # two-arm posture, and the other arm's own command is part of
                # it (R8).
                for s in saved:
                    if s in JOINT_SLICE:
                        kin.set_joints(s, np.asarray(command, dtype=float)[
                            JOINT_SLICE[s]])
                p_c, r_c = self._tool(kin, side, q_cmd)
                p_now, r_now = self._tool(kin, side, q_meas)
                axis = np.asarray(r_c.as_matrix()[:, 2], dtype=float)
                delta = np.asarray(p_now, dtype=float) - np.asarray(p_c,
                                                                    dtype=float)
                p_target = np.asarray(miss.p_cmd, dtype=float) - delta
                # A BOUNDED SHOVE. The full feed-forward would ask a descent
                # that stopped short to go all the way down, and what stops a
                # descent short is usually contact — F5, fingers jammed on the
                # table, ten grasps out of ten. So the target may sit at most
                # one ``tol_along_m`` PAST the waypoint along the approach
                # axis: enough to take out a gravity droop, not enough to
                # press the fingers into whatever stopped them.
                past = float(np.dot(p_target - np.asarray(miss.p_cmd,
                                                          dtype=float), axis))
                if past > self.tol_along_m:
                    p_target = p_target - (past - self.tol_along_m) * axis
                r_target = miss.r_cmd
                if miss.rot_rad > self.tol_rot_rad:
                    r_target = (r_now * r_c.inv()).inv() * miss.r_cmd
                # RE-SOLVED UNTIL THE TOOL IS THERE, exactly as the planner
                # walks a knot (``planning._straight``). One ``solve_ee`` is
                # not enough and the reason is geometric: the IK's target is
                # LINK7 and its tolerances are Link7's (2 mm / 2.9 deg,
                # ``safety.IK_POS_TOL`` / ``IK_ROT_TOL``), while the tool point
                # is 100 mm further along +z — so a solution the solver calls
                # converged can leave the TOOL 7 mm out, and re-asking from it
                # does nothing because from Link7's point of view it has
                # arrived. Measured: one solve took a 17.0 mm miss to 7.8 mm
                # and the next two rounds returned the same joints.
                kin.set_joints(side, q_cmd)
                reached = False
                for _ in range(MAX_CORRECTION_SOLVES):
                    p_fk, r_fk = self._tool(kin, side, kin.joints(side))
                    if (float(np.linalg.norm(p_fk - p_target)) <= SOLVE_TOL_M
                            and float(np.linalg.norm(
                                (r_fk.inv() * r_target).as_rotvec()))
                            <= SOLVE_TOL_RAD):
                        reached = True
                        break
                    p7, r7 = link7_from_tool(p_target, r_target)
                    result = kin.solve_ee(side, p7, r7)
                    if not getattr(result, "ok", False):
                        if getattr(result, "reason", "") == "guard_reject":
                            return None, ("the motion guard refused the "
                                          "corrected posture, so it was not "
                                          "sent")
                        return None, (f"the IK could not solve the corrected "
                                      f"pose ({result.reason})")
                q_new = np.array(kin.joints(side), dtype=float).reshape(ARM_DOF)
                if not reached:
                    # The solver PLATEAUS — ``planning`` says so in its own
                    # words ("the last four knots ... sit at 4.7-9.1 mm however
                    # many solves they are given"). A plateau is not a failure
                    # while the posture it plateaued on is still closer to the
                    # target than the arm is to the waypoint; when it is not,
                    # there is nothing to gain and the round is refused rather
                    # than spent.
                    short = float(np.linalg.norm(
                        self._tool(kin, side, q_new)[0] - p_target))
                    if short > SOLVE_TOL_M and short >= miss.total_m:
                        return None, (
                            f"the IK stopped {short * 1000:.1f} mm from the "
                            f"corrected pose after {MAX_CORRECTION_SOLVES} "
                            f"solves, no closer than the "
                            f"{miss.total_m * 1000:.1f} mm the arm already is")
                if (getattr(getattr(kin, "gate", None), "installed", False)
                        and not kin.guard_ok(side, q_new)):
                    return None, ("the motion guard refused the corrected "
                                  "posture, so it was not sent")
                return q_new, ""
            finally:
                for s, q in saved.items():
                    kin.set_joints(s, q)
        except Exception as exc:  # noqa: BLE001 - a model that cannot solve
            return None, f"the corrected pose could not be solved: {exc}"
        finally:
            if lock is not None:
                lock.release()


def _report(arrived: bool, arrival: ArrivalReport, miss: "ToolMiss",
            label: str, *, settled: Optional[bool] = None, detail: str = "",
            corrections: Sequence[Correction] = ()) -> ArrivalReport:
    """One ArrivalReport, with every number the barrier measured on it."""
    return ArrivalReport(arrived, arrival.worst_error_rad, arrival.waited_s,
                         detail or arrival.detail, miss.total_m, miss.rot_rad,
                         tuple(corrections), label, settled,
                         miss.across_m, miss.along_m)


def _remedy(arrival: ArrivalReport, gate: "ToolGate") -> str:
    """The refusal, plus what would actually answer it.

    A number with no move attached costs the caller a turn to work out, and
    an agent given one spends that turn re-asking for the same verb with a
    different argument — measured, five of twelve turns of an agent trial
    (2026-09-21). The two misses have different answers and the barrier knows
    which one it saw.
    """
    if arrival.settled is None:
        # The JOINT barrier is what failed: the arm never got to the posture
        # at all, so the tool numbers describe a journey rather than a miss.
        return (f"{arrival.detail}. The arm did not reach the commanded "
                f"posture — it was still travelling, or something stopped it. "
                f"Re-observe before asking for the same move again")
    if (arrival.tool_across_m <= gate.tol_m
            and abs(arrival.tool_along_m) > gate.tol_along_m):
        # Lined up and short: that is what CONTACT looks like. Only then — a
        # miss that is off in BOTH is not a story about the surface, and
        # measured in an agent trial this wording went out on an arm 320 mm away.
        return (f"{arrival.detail}. The arm is stationary short of the "
                f"waypoint along its own approach axis, which is what CONTACT "
                f"looks like: something is under the fingers. Re-observe, or "
                f"stand off and come in along a different approach — asking "
                f"for the same descent again will not move it")
    return (f"{arrival.detail}. Nudge the tool the measured amount and "
            f"re-plan, or approach the object along a different axis; the "
            f"jaws are not over it and closing them would miss")


def _joint_error(state: Optional[RawState], want) -> float:
    """Worst measured joint error against a commanded 16-vector [rad]."""
    if state is None:
        return float("nan")
    want = np.asarray(want, dtype=float).reshape(WIRE_DIM)
    worst = 0.0
    for side in SIDES:
        q = state.joints.get(side)
        if q is None:
            return float("nan")
        worst = max(worst, float(np.max(np.abs(
            np.asarray(q, dtype=float) - want[JOINT_SLICE[side]]))))
    return worst


def _arm_stopped(settle: SettleReport) -> bool:
    """Is the ARM stationary, whatever else the settle was waiting for?"""
    worst = float(settle.worst_velocity_deg_s)
    return bool(np.isfinite(worst) and worst <= ARM_STATIONARY_DEG_S)


def _with_label(report: ArrivalReport, label: str, *, detail: str = "",
                miss: Optional["ToolMiss"] = None) -> ArrivalReport:
    """The same report, told which waypoint it was about (and how far off)."""
    if miss is None:
        return ArrivalReport(report.arrived, report.worst_error_rad,
                             report.waited_s, detail or report.detail,
                             waypoint_label=label)
    return _report(report.arrived, report, miss, label, detail=detail)


def arrive_labels(plan: Plan) -> Dict[int, str]:
    """``{waypoint index: label}`` for the waypoints that demand a tool check.

    Read off the plan rather than off the verb, so a hand-built plan and a
    primitive's plan are gated by exactly the same rule, and a plan that asks
    for nothing gets the barrier it always had.
    """
    return {index: (wp.label or f"waypoint {index}")
            for index, wp in enumerate(getattr(plan, "waypoints", ()))
            if isinstance(wp, Waypoint) and wp.arrive}


def _leaves_waypoint(steps: Sequence[Any], index: int) -> bool:
    """Is the step after ``index`` somewhere other than the same waypoint?

    The gate belongs at the END of a waypoint's knots — one measured wait per
    waypoint, not one per interpolation knot, which on a 25 cm travel would be
    a dozen of them.
    """
    step = steps[index]
    nxt = steps[index + 1] if index + 1 < len(steps) else None
    return not (isinstance(nxt, JointStep) and nxt.waypoint == step.waypoint
                and nxt.side == step.side)


def _same(a: Optional[np.ndarray], b: Optional[np.ndarray]) -> bool:
    return (a is not None and b is not None
            and np.array_equal(np.asarray(a), np.asarray(b)))


def barrier_refusal(plan: Plan, side: str, arrival: ArrivalReport,
            gate: "ToolGate") -> RunRefusal:
    """The typed half of a failed tool-space barrier.

    ``arrived_off_by`` when the tool point is measurably off the pose the plan
    commanded — the number the caller can act on ("27 mm at the grasp pose
    after 2 corrections": nudge, re-observe, or pick another approach) —
    against ``arrival_unknown`` for a barrier that could not measure at all,
    which is a transport problem and not a geometry one.
    """
    measured = np.isfinite(arrival.tool_error_m)
    off = measured and (
        arrival.tool_across_m > gate.tol_m
        or abs(arrival.tool_along_m) > gate.tol_along_m
        or (np.isfinite(arrival.tool_rot_error_rad)
            and arrival.tool_rot_error_rad > gate.tol_rot_rad))
    if arrival.settled is False:
        reason = NOT_SETTLED
    else:
        reason = ARRIVED_OFF_BY if off else ARRIVAL_UNKNOWN
    return RunRefusal(
        reason, _remedy(arrival, gate) if off else arrival.detail,
        arrival.waypoint_label, arrival.tool_error_m,
        arrival.tool_rot_error_rad, stage="tool_arrival",
        attempted=tuple(
            f"round {c.round}: {c.tool_error_before_m * 1000:.1f} mm -> "
            + ("refused" if not c.sent else
               ("unknown" if not np.isfinite(c.tool_error_after_m)
                else f"{c.tool_error_after_m * 1000:.1f} mm"))
            for c in arrival.corrections),
        primitive=plan.primitive, side=side)


def run(plan: Plan, executor: "Executor", *, hz: float = 50.0,
        allow_unbound: bool = False, allow_unguarded: bool = False,
        arrive_tol_rad: float = ARRIVE_TOL_RAD,
        arrive_timeout_s: float = ARRIVE_TIMEOUT_S,
        stroke_timeout_s: float = STROKE_TIMEOUT_S,
        kin=None, correct_arrival: bool = True,
        tool_tol_m: float = ARRIVE_TOL_M,
        tool_tol_along_m: float = ARRIVE_TOL_ALONG_M,
        tool_rot_tol_rad: float = ARRIVE_TOL_ROT_RAD,
        settle_timeout_s: float = ARRIVE_SETTLE_S,
        max_corrections: int = MAX_ARRIVAL_CORRECTIONS) -> RunReport:
    """Walk a plan's steps through an executor, in order, at ``hz``.

    Joint steps are batched into one 16-vector per step so both arms move
    together — a dual-arm robot commanded one arm at a time is two robots
    disagreeing about the posture the guard just approved.

    Nothing is sent until :func:`check_binding` passes.

    ``kin`` is the kinematic model the TOOL-SPACE barrier computes with (see
    :class:`ToolGate`). Pass the one the plan was built against; with nothing
    passed the gate takes ``executor.kin`` if the transport carries one, and
    otherwise builds the kit's own guarded ``d1/arm`` once per process.

    Every tolerance and timeout the barriers use is a parameter HERE, once —
    ``tool_tol_along_m`` and ``settle_timeout_s`` (the tool gate's settle
    wait) included — and all of them, ``hz`` too, reach a transport's own
    ``run_plan`` rather than stopping at this function (L13).

    A latched arm controller stops the run before anything is sent, with
    :data:`CONTROLLER_FAULT` (and between steps, see :func:`run_steps`).
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
    gate = ToolGate(kin=kin, tol_rad=arrive_tol_rad,
                    timeout_s=arrive_timeout_s, tol_m=tool_tol_m,
                    tol_along_m=tool_tol_along_m,
                    tol_rot_rad=tool_rot_tol_rad, correct=correct_arrival,
                    max_rounds=max_corrections,
                    settle_timeout_s=settle_timeout_s,
                    guarded=bool(getattr(binding, "guarded", True)))
    own = getattr(executor, "run_plan", None)
    if own is not None:
        # A transport that predates the tool-space gate keeps the joint-space
        # barrier and says so by its own signature; the kwarg is offered, not
        # forced, so an out-of-tree ``run_plan`` is not broken by this change.
        return own(plan, **_accepted(own, hz=hz, arrive_tol_rad=arrive_tol_rad,
                                     arrive_timeout_s=arrive_timeout_s,
                                     stroke_timeout_s=stroke_timeout_s,
                                     gate=gate))
    return run_steps(plan, executor, hz=hz, arrive_tol_rad=arrive_tol_rad,
                     arrive_timeout_s=arrive_timeout_s,
                     stroke_timeout_s=stroke_timeout_s, gate=gate)


def _fault_now(executor: "Executor") -> Optional[str]:
    """A fresh read of the controllers: the reason to stop, or ``None``.

    A transport that cannot be read at this moment is not reported as a
    fault — the barrier or the transport error that follows says so.
    """
    try:
        return controller_fault(executor.state())
    except Exception:  # noqa: BLE001 - see the docstring
        return None


def _between_steps(steps: Sequence[Any], index: int) -> bool:
    """Is ``steps[index]`` the first step of a new step, not another knot?"""
    step, previous = steps[index], steps[index - 1]
    if not isinstance(step, JointStep) or not isinstance(previous, JointStep):
        return True
    return step.waypoint != previous.waypoint or step.side != previous.side


def _accepted(fn, **kwargs) -> Dict[str, Any]:
    """The subset of ``kwargs`` ``fn`` can take, by its own signature."""
    try:
        params = inspect.signature(fn).parameters
    except (TypeError, ValueError):       # a builtin / C callable
        return dict(kwargs)
    if any(p.kind is inspect.Parameter.VAR_KEYWORD for p in params.values()):
        return dict(kwargs)
    return {k: v for k, v in kwargs.items() if k in params}


def run_steps(plan: Plan, executor: "Executor", *, hz: float = 50.0,
              schedule: Optional[Sequence[float]] = None,
              arrive_tol_rad: float = ARRIVE_TOL_RAD,
              arrive_timeout_s: float = ARRIVE_TIMEOUT_S,
              stroke_timeout_s: float = STROKE_TIMEOUT_S,
              gate: Optional[ToolGate] = None) -> RunReport:
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

    AND THE BARRIER IS ALSO ABOUT THE TOOL. At every waypoint the plan marks
    :attr:`~manipulation_kit.primitives.types.Waypoint.arrive` — a grasp's
    standoff and descent, a place's transit and set-down — the jaw pocket
    itself has to be where the plan put it, measured by :class:`ToolGate`,
    corrected in place when it is not, and refused with ``arrived_off_by``
    when the correction does not land it. A joint tolerance loose enough for
    the real arm's droop is centimetres at the tool (F17), and the descent
    that follows a standoff cannot be re-routed, so the error has to be caught
    BEFORE it rather than diagnosed after the jaws close on nothing.
    """
    period = 1.0 / float(hz)
    sent = 0
    settle: Optional[SettleReport] = None
    arrivals: List[ArrivalReport] = []
    strokes: List[StrokeReport] = []
    contacts: List[ContactReport] = []
    if gate is None:
        gate = ToolGate(tol_rad=arrive_tol_rad, timeout_s=arrive_timeout_s,
                        guarded=bool(getattr(getattr(plan, "binding", None),
                                             "guarded", True)))
    labels = arrive_labels(plan)
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
    fault = controller_fault(state)
    if fault is not None:
        end = getattr(executor, "end_run", None)
        if end is not None:
            end(plan)
        return RunReport(plan.primitive, plan.side, False, 0,
                         stop_reason=CONTROLLER_FAULT, error=fault,
                         refusal=fault_refusal(plan, fault))
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
    grippers: Dict[str, float] = {}
    for side, value in state.grippers.items():
        commanded = state.commanded_grippers.get(side)
        if commanded is not None:
            grippers[side] = float(commanded)
            continue
        if state.holding.get(side):
            # NO COMMAND HISTORY AND SOMETHING IN THE HAND. Re-commanding the
            # measured aperture tells a force-limited gripper to stop
            # squeezing and hold position, which is how a Lift that never
            # mentions the jaws drops what it is carrying (F9, three trials of
            # three). The fallback was documented as honest; it is only honest
            # when the hand is EMPTY. With a hold and no retained command,
            # refusing is the answer.
            return RunReport(
                plan.primitive, plan.side, False, 0,
                stop_reason=TRANSPORT_ERROR,
                error=(f"the {side} gripper reports a hold at "
                       f"{value:.2f} closedness and the executor cannot say "
                       f"what it is COMMANDED at. Every dual-arm vector has "
                       f"to carry a number for it, and the measurement is not "
                       f"that number — re-commanding it releases the squeeze. "
                       f"Publish RawState.commanded_grippers."))
        grippers[side] = float(value)
    last_vector: Optional[np.ndarray] = None

    def stop(index: int, reason: str, detail: str,
             refusal: Optional[RunRefusal] = None) -> RunReport:
        end = getattr(executor, "end_run", None)
        if end is not None:
            end(plan)
        if reason != CONTROLLER_FAULT:
            # A barrier that failed because the controller LATCHED under it
            # is a controller fault, and the caller is owed that name rather
            # than "the arm did not arrive".
            fault = _fault_now(executor)
            if fault is not None:
                reason, detail = CONTROLLER_FAULT, fault
                refusal = fault_refusal(plan, fault)
        return RunReport(plan.primitive, plan.side, False, sent, settle,
                         error=detail, stop_reason=reason, stopped_at=index,
                         arrivals=tuple(arrivals), strokes=tuple(strokes),
                         refusal=refusal, contacts=tuple(contacts))

    times = None if schedule is None else list(schedule)
    joint_index = 0
    #: the plan time of the last command sent, so a contact leg continues
    #: the run's own timeline instead of restarting it
    t_last = -period
    #: the vector the last barrier was run against, so a waypoint gate and the
    #: stroke that follows it do not pay for the same measurement twice
    gated: Optional[np.ndarray] = None

    def gate_at(side: str, label: str, at: float):
        """Run the tool-space gate, keep the report, adopt any correction."""
        nonlocal last_vector, gated
        report, corrected = gate.check(executor, last_vector, side,
                                       label=label, t=at)
        arrivals.append(report)
        if corrected is not None:
            # THE CORRECTION IS NOW THE COMMAND. Everything after it — the
            # next dual-arm vector, the pre-stroke barrier — has to carry the
            # posture the arm is actually under, not the one that missed.
            last_vector = corrected
            joints[side] = np.array(corrected[JOINT_SLICE[side]], dtype=float)
        gated = last_vector
        return report

    for index, step in enumerate(plan.steps):
        if index and _between_steps(plan.steps, index):
            # BETWEEN STEPS, not between knots: a waypoint boundary, a stroke
            # or a settle. Reading the whole robot at every 50 Hz knot would
            # cost the stream its cadence; a fault is caught at the next
            # boundary, and the barrier in front of every stroke reads the
            # arm anyway.
            fault = _fault_now(executor)
            if fault is not None:
                return stop(index, CONTROLLER_FAULT, fault,
                            fault_refusal(plan, fault))
        if isinstance(step, JointStep):
            joints[step.side] = np.asarray(step.q, dtype=float)
            last_vector = wire(joints, grippers)
            at = (joint_index * period if times is None
                  else float(times[joint_index]))
            at = max(at, t_last + period)
            executor.send_joints(last_vector, t=at)
            t_last = at
            joint_index += 1
            sent += 1
            gated = None
            # THE WAYPOINT ENDS HERE, and this one says the tool has to be on
            # it. Catching a miss at the standoff is the whole point: the
            # descent that follows may not be re-routed, so an error carried
            # into it arrives at the block.
            label = labels.get(step.waypoint)
            if label is not None and _leaves_waypoint(plan.steps, index):
                arrival = gate_at(step.side, label, at)
                if not arrival.arrived:
                    # The REFUSAL's detail, not the arrival's: it is the same
                    # sentence plus the move that answers it, and a consumer
                    # that only reads ``error`` is owed that too.
                    refusal = barrier_refusal(plan, step.side, arrival, gate)
                    return stop(index, BARRIER_FAILED, refusal.detail, refusal)
        elif isinstance(step, GripStep):
            # ARRIVE BEFORE YOU CLOSE. A stroke run while the arm is still
            # travelling closes the jaws somewhere along the path.
            if last_vector is not None and not _same(gated, last_vector):
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
                refusal = stroke_refusal(plan, step.side, stroke)
                return stop(index, BARRIER_FAILED, refusal.detail, refusal)
            refusal = hold_refusal(plan, step, stroke)
            if refusal is not None:
                return stop(index, BARRIER_FAILED, refusal.detail, refusal)
            sent += 1
        elif isinstance(step, SettleStep):
            # A SETTLE IS NOT AN ARRIVAL. "Stationary" is true of an arm that
            # stopped 50 deg short on one joint (d1-2, 2026-09-22: J7 held at
            # -39.5 deg against a -90 deg command, the tool 167 mm off, and the
            # run said completed). The last commanded posture is measured
            # first; the settle then says the arm has stopped there.
            if last_vector is not None and not _same(gated, last_vector):
                arrival = _final_arrival(executor, plan, last_vector,
                                         tol_rad=arrive_tol_rad,
                                         timeout_s=arrive_timeout_s, gate=gate)
                arrivals.append(arrival)
                gated = last_vector
                if not arrival.arrived:
                    refusal = barrier_refusal(plan, plan.side, arrival, gate)
                    return stop(index, BARRIER_FAILED, arrival.detail, refusal)
            settle = executor.settle(step.timeout_s)
            sent += 1
            if not settle.settled:
                # BOTH runners used to carry on here and report completion.
                return stop(index, BARRIER_FAILED,
                            f"the arms did not settle: {settle.detail}")
        elif isinstance(step, ContactStep):
            watch = getattr(executor, "move_until", None)
            if watch is None:
                # LOUDLY. Playing the leg's knots as ordinary joint steps
                # would drive a position-controlled arm into whatever it
                # meets with nothing watching.
                return stop(index, TRANSPORT_ERROR, (
                    f"{type(executor).__name__} implements no move_until, so "
                    f"the {step.side} contact leg cannot be watched for "
                    f"resistance; a contact leg is never played blind"))
            if last_vector is None:
                last_vector = wire(joints, grippers)
            report = watch(step.timed_path(), side=step.side,
                           criterion=step.criterion,
                           **_accepted(watch, hz=hz, kin=gate.kin,
                                       t0=t_last + period,
                                       command=last_vector,
                                       direction=step.direction.vector()))
            contacts.append(report)
            if report.stopped_by == CONTACT_FAULT:
                fault = _fault_now(executor) or report.detail
                return stop(index, CONTROLLER_FAULT, fault,
                            fault_refusal(plan, fault))
            if report.stopped_by in (CONTACT_GUARD, UNMEASURED):
                return stop(index, TRANSPORT_ERROR, (
                    f"the {step.side} contact leg stopped by "
                    f"{report.stopped_by}: {report.detail}"))
            t_last += max(period, report.elapsed_s)
            for q in _after_contact(step, report, hz):
                vector = np.array(last_vector, dtype=float)
                vector[JOINT_SLICE[step.side]] = q
                t_last += period
                executor.send_joints(vector, t=t_last)
                last_vector = vector
            joints[step.side] = np.array(last_vector[JOINT_SLICE[step.side]],
                                         dtype=float)
            # A contact leg ends where the contact stopped it, and that is
            # MEASURED by the leg itself; the hold that follows is not a
            # posture the arm is expected to reach through the surface.
            gated = last_vector
            sent += 1
        else:
            return stop(index, TRANSPORT_ERROR, f"not a plan step: {step!r}")
    if last_vector is not None and not _same(gated, last_vector):
        # THE LAST LEG IS MEASURED TOO. A plan that ends on joint steps with
        # no stroke and no gated waypoint after them used to return
        # ``completed`` the moment the last command left, which says nothing
        # about where the arm is.
        arrival = _final_arrival(executor, plan, last_vector,
                                 tol_rad=arrive_tol_rad,
                                 timeout_s=arrive_timeout_s, gate=gate)
        arrivals.append(arrival)
        if not arrival.arrived:
            refusal = barrier_refusal(plan, plan.side, arrival, gate)
            return stop(len(plan.steps) - 1, BARRIER_FAILED, arrival.detail,
                        refusal)
    end = getattr(executor, "end_run", None)
    if end is not None:
        end(plan)
    return RunReport(plan.primitive, plan.side, True, sent, settle,
                     arrivals=tuple(arrivals), strokes=tuple(strokes),
                     contacts=tuple(contacts))


def _after_contact(step: ContactStep, report: ContactReport,
                   hz: float) -> List[np.ndarray]:
    """What a streaming runner commands once a contact leg has stopped.

    Three shapes, and each is a plain position command:

    * no retract (a probe): ONE command at the posture the arm MEASURED at
      the stop. The frozen command sits inside the surface by however far
      the arm was pushing; re-commanding the measurement stops the push and
      leaves the tool touching.
    * a hold (a press that made contact): the frozen command, kept for
      ``hold_s`` — the press IS the command being held against the surface.
    * a retract: the leg played back to its start at the leg's own speed.
    """
    if not step.retract:
        if report.q_stop is None:
            return []
        return [np.asarray(report.q_stop, dtype=float)]
    timed = retract_path(step, report)
    out: List[np.ndarray] = []
    if report.made and step.hold_s > 0.0:
        out += [np.asarray(timed[0][1], dtype=float)] * max(
            1, int(math.ceil(step.hold_s * float(hz))))
    if len(timed) > 1:
        out += [q for _t, q in leg_ticks(timed, hz)]
    return out


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
        """Where the arm is — which, for a recorder, is where it started.

        UNLESS the test asked it to pretend. ``pretend_arrived`` is a claim
        about the BARRIERS, and the tool-space half of them reads the measured
        posture: a double that says "arrived" while reporting joints a
        radian away is not pretending, it is lying, and the gate would
        (correctly) refuse it. So the pretence is whole — the last commanded
        posture is reported back — and the honest default, the one the
        "a verifier must FAIL against this" tests use, is untouched.
        """
        if not self.pretend_arrived or not self.sent:
            return self._state
        commanded = self.sent[-1][1]
        return dataclasses.replace(self._state, arms={
            s: dataclasses.replace(
                arm, q=np.array(commanded[JOINT_SLICE[s]], dtype=float))
            for s, arm in self._state.arms.items()})

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

    ``open_gap_m`` is the driven-open pad gap of the hand this mirror stands
    in for — a MEASUREMENT of a particular robot's hand (d1-2's opens to
    60.5 mm, the nominal description to 51.96 mm), published as
    ``HandState.open_gap_m`` exactly as the firmware transport publishes the
    daemon's. ``None`` (the default) says the mirror does not know, and the
    planner falls back to the hand description's nominal gap.

    ``firmware_spec`` is ``"kinematic"``: a plan checked against a mirror's
    observation is refused on hardware, and the other way round.
    """

    firmware_spec = "kinematic"

    def __init__(self, kin, *, hold_at: float = 0.5,
                 held: Optional[Dict[str, Optional[str]]] = None,
                 open_gap_m: Optional[float] = None):
        if open_gap_m is not None and not (math.isfinite(float(open_gap_m))
                                           and float(open_gap_m) > 0.0):
            raise ValueError(f"open_gap_m must be a positive gap in metres, "
                             f"got {open_gap_m!r}")
        self.kin = kin
        self.open_gap_m = None if open_gap_m is None else float(open_gap_m)
        self.hold_at = float(hold_at)
        self.grippers: Dict[str, float] = {s: 0.0 for s in SIDES}
        self.held: Dict[str, Optional[str]] = dict(held or {})
        self.sent: List[Tuple[float, np.ndarray]] = []
        self.grips: List[Tuple[str, float, str]] = []
        self.settles: List[float] = []
        #: what the jaws would close on next, set by a test before a Grasp
        self.next_object: Dict[str, Optional[str]] = {s: None for s in SIDES}

    def state(self) -> RawState:
        # A mirror has no contact, so where its jaws are IS what they were
        # commanded to — and it publishes both, because the runner refuses to
        # carry a held hand's aperture forward on a measurement alone (F9).
        # No dynamics: the joints are exactly where they were sent, nothing is
        # moving, and a mirror has no controller to latch — so ``position``
        # with no error, and no torque (there is no load to measure).
        return RawState(
            arms={s: JointState(q=np.array(self.kin.joints(s), dtype=float),
                                qd=np.zeros(ARM_DOF), mode="position",
                                error_code=0, stationary=True)
                  for s in SIDES},
            hands={s: HandState(closedness=self.grippers[s],
                                commanded=self.grippers[s],
                                holding=self.held.get(s) is not None,
                                open_gap_m=self.open_gap_m)
                   for s in SIDES})

    def neck_state(self) -> Optional[NeckState]:
        """A mirror has no head: ``None``, the optional capability's "absent"."""
        return None

    def lift_state(self) -> Optional[LiftState]:
        """A mirror has no lift: ``None``."""
        return None

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
        from .primitives.orientation import tool_from_link7
        return tool_from_link7(*self.kin.ee_pose(side))

    # -- contact ----------------------------------------------------------- #
    def move_until(self, path, *, side: str, criterion: ContactCriterion,
                   hz: float = 50.0, kin=None, direction=None,
                   **_ignored) -> ContactReport:
        """A mirror has no contact: the whole leg is travelled, and SAID so.

        ``made=False, stopped_by="max_travel"`` — honest, and a dry-run keeps
        working: the plan runs to the end of its leg, and the verifier
        reports that nothing was touched instead of the run failing.
        """
        timed = [(float(t), np.asarray(q, dtype=float)) for t, q in path]
        q_start = np.array(self.kin.joints(side), dtype=float)
        for t, q in timed[1:]:
            self.sent.append((float(t), wire(
                {s: (q if s == side else self.kin.joints(s)) for s in SIDES},
                self.grippers)))
        self.kin.set_joints(side, timed[-1][1])
        return contact_report(
            self.kin, side, timed, direction=direction, q_start=q_start,
            q_stop=timed[-1][1],
            made=False, stopped_by=MAX_TRAVEL, leg_t_s=timed[-1][0],
            elapsed_s=timed[-1][0],
            detail=("a kinematic mirror has no contact and measures no "
                    "torque: the whole leg was travelled and nothing could "
                    "resist"))
