"""``move_until`` on the executors: the watch, the default, the firmware override.

The firmware half runs against a FAKE GENERATED CLIENT — the typed
``trajectory_start`` / ``trajectory_status`` / ``trajectory_cancel`` and
``arm_state`` the adapter exposes over ``d1fw_api`` — that plays the uploaded
leg against the executor's own fake clock and blocks the arm at a scripted
leg time, where the torque starts to rise. Nothing opens a socket and nothing
here talks to a robot.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import pytest

from manipulation_kit.executor import (ContactWatch, HandState, JointState,
                                       KinematicExecutor, RawState,
                                       StreamingContact, stream_move_until)
from manipulation_kit.executors.firmware import FirmwareExecutor
from manipulation_kit.executors.firmware.client import ROUTES
from manipulation_kit.executors.firmware.executor import INTERPOLATION_S
from manipulation_kit.primitives import ContactCriterion, ContactStep, Probe
from manipulation_kit.primitives.orientation import tool_from_link7
from manipulation_kit.world import ArmView, GripperView, WorldView

SIDES = ("left", "right")


def _arm(torque, qd=None, mode="position", code=0) -> JointState:
    return JointState(q=np.zeros(7), qd=np.zeros(7) if qd is None else qd,
                      torque_nm=np.asarray(torque, dtype=float), mode=mode,
                      error_code=code)


def _rise(nm: float, joint: int = 2) -> np.ndarray:
    out = np.zeros(7)
    out[joint] = nm
    return out


# --------------------------------------------------------------------------- #
# the watch
# --------------------------------------------------------------------------- #

def test_a_transient_rise_freezes_the_command_and_then_lets_it_go():
    watch = ContactWatch(ContactCriterion(joint_torque_nm=4.0, settle_s=0.1),
                         np.zeros(7))
    assert watch.sample(0.00, _arm(_rise(1.0))) == "free"
    assert watch.sample(0.02, _arm(_rise(4.5))) == "confirming"
    assert watch.sample(0.04, _arm(_rise(4.5))) == "confirming"
    assert watch.sample(0.06, _arm(_rise(0.5))) == "free"      # it went away
    assert watch.peak_nm == pytest.approx(4.5) and watch.joint == 2


def test_a_rise_on_a_moving_arm_is_not_contact_until_it_stops():
    watch = ContactWatch(ContactCriterion(joint_torque_nm=4.0, settle_s=0.1),
                         np.zeros(7))
    moving = np.full(7, 0.2)
    for k in range(20):
        assert watch.sample(0.02 * k, _arm(_rise(5.0), qd=moving)) == "confirming"
    t = 0.40
    verdicts = [watch.sample(t + 0.02 * k, _arm(_rise(5.0))) for k in range(7)]
    assert verdicts[:5] == ["confirming"] * 5 and verdicts[-1] == "contact"


def test_twice_the_threshold_is_contact_at_once():
    watch = ContactWatch(ContactCriterion(joint_torque_nm=4.0), np.zeros(7))
    assert watch.sample(0.0, _arm(_rise(8.1), qd=np.full(7, 1.0))) == "contact"


def test_the_rise_is_over_the_baseline_not_the_absolute_torque():
    load = np.array([0.0, 9.0, -3.0, 2.0, 0.0, 0.0, 0.0])
    watch = ContactWatch(ContactCriterion(joint_torque_nm=4.0), load)
    assert watch.sample(0.0, _arm(load + _rise(1.0))) == "free"


# --------------------------------------------------------------------------- #
# the streamed default refuses what it cannot measure
# --------------------------------------------------------------------------- #

class _NoTorque(StreamingContact, KinematicExecutor):
    """Streams, but reports no torque — as the plain mirror does."""


def _leg(kin, side="left"):
    q0 = np.array(kin.joints(side), dtype=float)
    q1 = q0.copy()
    q1[3] += 0.05
    return [(0.0, q0), (1.0, q1)]


def test_a_transport_without_torque_does_not_start_the_leg(d1_arm):
    rig = _NoTorque(d1_arm)
    report = rig.move_until(_leg(d1_arm), side="left",
                            criterion=ContactCriterion(), kin=d1_arm)
    assert report.stopped_by == "unmeasured" and not report.made
    assert rig.sent == []


def test_a_tool_force_criterion_is_refused_not_ignored(d1_arm):
    class Torque(StreamingContact, KinematicExecutor):
        def state(self):
            base = super().state()
            return RawState({s: JointState(q=a.q, qd=a.qd, torque_nm=np.zeros(7),
                                           mode="position")
                             for s, a in base.arms.items()}, base.hands)
    rig = Torque(d1_arm)
    report = rig.move_until(_leg(d1_arm), side="left",
                            criterion=ContactCriterion(tool_force_n=10.0),
                            kin=d1_arm)
    assert report.stopped_by == "unmeasured" and "TOOL FORCE" in report.detail
    assert rig.sent == []


# --------------------------------------------------------------------------- #
# the firmware override, over a fake GENERATED client
# --------------------------------------------------------------------------- #

@dataclass
class _Arm:
    mode: str = "position"
    error_code: int = 0
    feedback_joints: Tuple[float, ...] = (0.0,) * 7
    command_joints: Tuple[float, ...] = (0.0,) * 7
    feedback_velocity: Tuple[float, ...] = (0.0,) * 7
    feedback_torque: Tuple[float, ...] = (0.0,) * 7
    stationary: bool = True


@dataclass
class _Grip:
    kind: str = "empty"
    jaw_rad: float = 0.0
    torque_nm: float = 0.0
    holding: bool = False
    grip_preload_rad: float = 0.0
    live: bool = True
    open_rad: float = 1.16
    coil_c: int = 30
    fault_code: Optional[str] = None


@dataclass
class _Status:
    """The generated ``TrajectoryStatus``, as far as the executor reads it."""
    id: int
    phase: str
    elapsed_ms: int
    message: Optional[str] = None


LOAD = (0.2, 6.0, -1.0, 2.5, 0.0, 0.3, 0.0)


@dataclass
class FakeGeneratedClient:
    """The adapter surface ``FirmwareExecutor.move_until`` may use — and a
    ``request`` that REFUSES the trajectory routes, so a hand-built call from
    the contact path fails the test. ``_play`` (the ordinary leg uploader)
    still goes through ``request`` and is served by ``legacy``."""

    clock: Dict[str, float]
    #: leg time [s] at which the arm is blocked; None = nothing in the way
    block_at_s: Optional[float] = None
    #: Nm of rise per second the COMMAND runs past the block
    stiffness_nm_s: float = 40.0
    #: leg time at which the controller latches
    fault_at_s: Optional[float] = None
    arms: Dict[str, _Arm] = field(default_factory=lambda: {
        "a": _Arm(feedback_torque=LOAD, feedback_joints=(10.0,) * 7,
                  command_joints=(10.0,) * 7),
        "b": _Arm(feedback_torque=LOAD)})
    calls: List[Tuple[str, Any]] = field(default_factory=list)
    cancelled: List[int] = field(default_factory=list)
    _job: Optional[Dict[str, Any]] = None
    _next_id: int = 100
    legacy_play: bool = False

    # -- typed, generated operations --------------------------------------- #
    def trajectory_start(self, points, *, holder=None) -> _Status:
        self.calls.append(("trajectory_start", [tuple(p[:1]) for p in points]))
        self._next_id += 1
        self._job = {"id": self._next_id, "points": list(points),
                     "t0": self.clock["t"], "frozen": None}
        return _Status(self._next_id, "running", 0)

    def trajectory_status(self, job: int) -> _Status:
        self.calls.append(("trajectory_status", job))
        state = self._job
        assert state is not None and state["id"] == job
        tau = self._tau()
        if state["frozen"] is not None:
            return _Status(job, "cancelled", int(state["frozen"] * 1000))
        if tau >= state["points"][-1][0]:
            return _Status(job, "completed", int(tau * 1000))
        return _Status(job, "running", int(tau * 1000))

    def trajectory_cancel(self, job: int) -> _Status:
        self.calls.append(("trajectory_cancel", job))
        self.cancelled.append(job)
        if self._job is not None and self._job["id"] == job:
            self._job["frozen"] = self._tau()
        return _Status(job, "cancelled", 0)

    def arm_state(self, side: str) -> _Arm:
        self.calls.append(("arm_state", side))
        arm = self.arms[side]
        if side != "a" or self._job is None or self.legacy_play:
            return arm
        tau = self._tau() if self._job["frozen"] is None else self._job["frozen"]
        leg = tau - INTERPOLATION_S
        command = self._command(tau)
        if self.fault_at_s is not None and leg >= self.fault_at_s:
            arm.mode, arm.error_code = "error", 5
        if self.block_at_s is not None and leg >= self.block_at_s:
            held = self._command(self.block_at_s + INTERPOLATION_S)
            rise = self.stiffness_nm_s * (leg - self.block_at_s)
            torque = list(LOAD)
            torque[1] += rise
            arm.feedback_joints = tuple(held)
            arm.feedback_velocity = (0.0,) * 7
            arm.feedback_torque = tuple(torque)
        else:
            arm.feedback_joints = tuple(command)
            arm.feedback_velocity = (0.0,) * 6 + (1.0,)
            arm.feedback_torque = LOAD
        arm.command_joints = tuple(command)
        return arm

    def gripper_state(self, side: str) -> _Grip:
        return _Grip()

    def _tau(self) -> float:
        return self.clock["t"] - self._job["t0"]

    def _command(self, tau: float) -> np.ndarray:
        pts = self._job["points"]
        for (t0, a0, _b0), (t1, a1, _b1) in zip(pts, pts[1:]):
            if tau <= t1:
                f = 0.0 if t1 == t0 else (tau - t0) / (t1 - t0)
                return np.asarray(a0) + f * (np.asarray(a1) - np.asarray(a0))
        return np.asarray(pts[-1][1], dtype=float)

    # -- the enveloped escape hatch ---------------------------------------- #
    def request(self, method: str, path: str, body: Any = None) -> Any:
        self.calls.append(("request", (method, path)))
        if path == "/v1/arm/trajectory/start" and self.legacy_play:
            last = body["waypoints"][-1]
            self.arms["a"].feedback_joints = tuple(last["a"])
            self.arms["b"].feedback_joints = tuple(last["b"])
            return {"id": 7, "phase": "running", "elapsed_ms": 0}
        if path.startswith("/v1/arm/trajectory/") and self.legacy_play:
            if path.endswith("/cancel"):
                return None
            return {"id": 7, "phase": "completed", "elapsed_ms": 10}
        if path.startswith("/v1/arm/trajectory"):
            raise AssertionError(f"hand-built trajectory call {method} {path}")
        raise AssertionError(f"unexpected call {method} {path}")


def _firmware(client) -> FirmwareExecutor:
    def sleep(seconds: float) -> None:
        client.clock["t"] += float(seconds)
    return FirmwareExecutor(client, sleep=sleep, clock=lambda: client.clock["t"],
                            heartbeat=False)


def _fw_leg(kin, seconds: float = 2.0):
    q0 = np.radians(np.full(7, 10.0))
    q1 = q0.copy()
    q1[1] += 0.10
    return [(0.0, q0), (seconds / 2, (q0 + q1) / 2), (seconds, q1)]


def _tool(kin, side, q) -> np.ndarray:
    saved = np.array(kin.joints(side), dtype=float)
    try:
        kin.set_joints(side, np.asarray(q, dtype=float))
        return tool_from_link7(*kin.ee_pose(side))[0]
    finally:
        kin.set_joints(side, saved)


def test_the_firmware_leg_stops_on_contact_through_the_generated_operations(d1_arm):
    client = FakeGeneratedClient(clock={"t": 0.0}, block_at_s=0.8)
    robot = _firmware(client)
    leg = _fw_leg(d1_arm)
    report = robot.move_until(leg, side="left", criterion=ContactCriterion(),
                              kin=d1_arm)
    assert report.made and report.stopped_by == "contact", report.detail
    kinds = [c[0] for c in client.calls]
    assert "trajectory_start" in kinds and "trajectory_cancel" in kinds
    assert not any(c[0] == "request" for c in client.calls)
    # the FIRST cancel is the freeze: it came as soon as the rise crossed
    # 4 Nm, i.e. ~0.1 s of leg past the block, not after the settle window
    first_cancel = kinds.index("trajectory_cancel")
    polls_before = kinds[:first_cancel].count("trajectory_status")
    assert polls_before * robot.period == pytest.approx(
        INTERPOLATION_S + 0.8 + 0.1, abs=3 * robot.period)
    # MEASURED: the tool is where the arm was blocked, not where the command got
    blocked = np.radians(client._command(0.8 + INTERPOLATION_S))
    assert np.allclose(report.q_stop, blocked, atol=1e-9)
    frozen = np.radians(client.arms["a"].command_joints)
    assert not np.allclose(frozen, blocked)
    assert report.p_tool == pytest.approx(_tool(d1_arm, "left", blocked), abs=1e-9)
    assert robot._jobs == []


def test_the_firmware_leg_uploads_the_leg_from_the_measured_pose(d1_arm):
    client = FakeGeneratedClient(clock={"t": 0.0})
    robot = _firmware(client)
    robot.move_until(_fw_leg(d1_arm), side="left",
                     criterion=ContactCriterion(), kin=d1_arm)
    start = next(c for c in client.calls if c[0] == "trajectory_start")
    times = [t for (t,) in start[1]]
    # measured start at 0, then the leg's own timing after the interpolation
    assert times[0] == 0.0
    # (the leg's own first knot IS the start posture, which the measured
    # pose stands in for — the daemon refuses a first waypoint off feedback)
    assert times[1:] == pytest.approx([INTERPOLATION_S + t
                                       for t, _q in _fw_leg(d1_arm) if t > 0])


def test_the_firmware_leg_reports_max_travel_when_nothing_resists(d1_arm):
    client = FakeGeneratedClient(clock={"t": 0.0})
    report = _firmware(client).move_until(_fw_leg(d1_arm), side="left",
                                          criterion=ContactCriterion(),
                                          kin=d1_arm)
    assert not report.made and report.stopped_by == "max_travel"
    assert "trajectory_cancel" not in [c[0] for c in client.calls]


def test_a_latched_controller_mid_leg_stops_the_firmware_leg_with_fault(d1_arm):
    client = FakeGeneratedClient(clock={"t": 0.0}, fault_at_s=0.5)
    robot = _firmware(client)
    report = robot.move_until(_fw_leg(d1_arm), side="left",
                              criterion=ContactCriterion(), kin=d1_arm)
    assert report.stopped_by == "fault" and not report.made
    assert client.cancelled, "the job was left playing on a latched arm"
    assert robot._jobs == []


def test_the_contact_routes_are_in_the_d1_2_document():
    for route in ("/v1/arm/trajectory/start", "/v1/arm/trajectory/{id}/status",
                  "/v1/arm/trajectory/{id}/cancel", "/v1/arm/{side}/state"):
        assert route in ROUTES


def _observe(kin) -> WorldView:
    arms, grippers = [], []
    for side in SIDES:
        p, r = tool_from_link7(*kin.ee_pose(side))
        arms.append(ArmView(side, joints=kin.joints(side), tool_p=p, tool_r=r,
                            mode="position"))
        grippers.append(GripperView(side, 0.0))
    return WorldView.of([], arms=arms, grippers=grippers)


def test_a_probe_plan_runs_on_the_firmware_executor_and_reports_its_contact(
        d1_arm):
    """The whole ``run_plan`` path: stroke, standoff upload, the contact leg
    through the override, and the relieve — on the fake.

    Starts fingertips-down (the pose ``tests/primitives/test_contact.py``
    uses): from HOME the probe's in-place turn needs J7 past the coupled
    wrist-roll limit, which is not what this pins."""
    d1_arm.set_joints("left", np.radians(
        [-21.091, -68.997, 37.832, -84.469, -5.509, -34.506, -50.0]))
    world = _observe(d1_arm)
    plan = Probe(side="left", direction="down").plan(world, d1_arm)
    assert plan.ok, str(plan)
    step = next(s for s in plan.steps if isinstance(s, ContactStep))
    q0 = np.degrees(step.path[0])
    client = FakeGeneratedClient(clock={"t": 0.0}, block_at_s=2.0)
    client.arms["a"] = _Arm(feedback_joints=tuple(np.degrees(d1_arm.joints("left"))),
                            command_joints=tuple(np.degrees(d1_arm.joints("left"))),
                            feedback_torque=LOAD)
    client.arms["b"] = _Arm(feedback_joints=tuple(np.degrees(d1_arm.joints("right"))),
                            command_joints=tuple(np.degrees(d1_arm.joints("right"))),
                            feedback_torque=LOAD)
    robot = _firmware(client)

    def gripper_set(side, closedness, *, grip=None, timeout_s=None):
        client.calls.append(("gripper_set", side))
    client.gripper_set = gripper_set          # type: ignore[attr-defined]

    original = robot.move_until

    def contact_leg(path, **kwargs):
        client.legacy_play = False
        client.arms["a"].feedback_joints = tuple(q0)
        out = original(path, **kwargs)
        client.legacy_play = True
        return out

    robot.move_until = contact_leg           # type: ignore[assignment]
    client.legacy_play = True
    report = robot.run_plan(plan)
    assert report.completed, report.error
    (contact,) = report.contacts
    assert contact.made and contact.stopped_by == "contact"
    assert contact.travel_m == pytest.approx(2.0 * step.speed_m_s, abs=0.004)
    assert report.to_json()["contacts"][0]["stopped_by"] == "contact"
