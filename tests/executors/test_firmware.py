"""The firmware transport, against a FAKE client. Nothing here opens a socket.

This is the one module in the package that has a wire, so it is also the one
whose tests have to be most careful not to need one. The fake speaks the same
REST surface the daemon advertises in ``openapi/d1-firmwared.v1.json`` — the
enveloped ``request`` method, ``arm_state``, ``gripper_set``, ``gripper_state``
— and records everything, so the lease discipline, the rate clamp and the
trajectory arithmetic are all checked as DATA.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Tuple

import numpy as np
import pytest

from manipulation_kit.executors.firmware import (ANCHOR_GAP_DEG,
                                                 INTERPOLATION_S,
                                                 MAX_JOINT_RATE_DEG_S,
                                                 FirmwareExecutor,
                                                 FirmwareUnavailable, Lease)
from manipulation_kit.primitives import GoHome, Grasp

REACHABLE = (0.38, 0.25, 0.05)


@dataclass
class FakeArmState:
    mode: str = "position"
    error_code: int = 0
    feedback_joints: Tuple[float, ...] = (0.0,) * 7
    command_joints: Tuple[float, ...] = (0.0,) * 7
    feedback_velocity: Tuple[float, ...] = (0.0,) * 7
    feedback_torque: Tuple[float, ...] = (0.0,) * 7
    feedback_temperature: Tuple[float, ...] = (30.0,) * 7
    frame_serial: int = 1
    stationary: bool = True


@dataclass
class FakeGripperState:
    kind: str = "idle"
    jaw_rad: float = 1.16
    torque_nm: float = 0.0
    holding: bool = False
    grip_preload_rad: float = 0.0
    live: bool = True
    open_rad: float = 1.16
    coil_c: int = 30


@dataclass
class FakeClient:
    """Everything ``FirmwareExecutor`` is allowed to call, and a log of it."""

    arms: Dict[str, FakeArmState] = field(
        default_factory=lambda: {"a": FakeArmState(), "b": FakeArmState()})
    grippers: Dict[str, FakeGripperState] = field(
        default_factory=lambda: {"a": FakeGripperState(), "b": FakeGripperState()})
    calls: List[Tuple[str, str, Any]] = field(default_factory=list)
    lease_epoch: int = 7
    #: phases the trajectory status returns, in order
    phases: List[str] = field(default_factory=lambda: ["completed"])
    settle_after: int = 0

    def request(self, method: str, path: str, body: Any = None) -> Any:
        self.calls.append((method, path, body))
        if path == "/v1/arm/lease" and method == "POST":
            return {"holder": body["holder"], "class": body.get("class", "ambient"),
                    "epoch": self.lease_epoch, "ttl_s": body.get("ttl_s", 30),
                    "expires_in_s": body.get("ttl_s", 30), "preempted_from": None}
        if path == "/v1/arm/lease" and method == "DELETE":
            return None
        if path.endswith("/mode") or path == "/v1/arm/move_joints_both":
            return None
        if path == "/v1/arm/trajectory/start":
            return {"id": 42, "phase": "running", "elapsed_ms": 0}
        if path.startswith("/v1/arm/trajectory/"):
            phase = self.phases.pop(0) if len(self.phases) > 1 else self.phases[0]
            return {"id": 42, "phase": phase, "elapsed_ms": 10, "message": "boom"}
        raise AssertionError(f"unexpected call {method} {path}")

    def arm_state(self, side: str) -> FakeArmState:
        self.calls.append(("GET", f"/v1/arm/{side}/state", None))
        state = self.arms[side]
        if self.settle_after > 0:
            self.settle_after -= 1
            return FakeArmState(stationary=False,
                                feedback_velocity=(12.0,) + (0.0,) * 6,
                                feedback_joints=state.feedback_joints,
                                command_joints=state.command_joints)
        return state

    def gripper_state(self, side: str) -> FakeGripperState:
        self.calls.append(("GET", f"/v1/gripper/{side}/state", None))
        return self.grippers[side]

    def gripper_set(self, side: str, closedness: float, *, grip=None) -> None:
        self.calls.append(("POST", f"/v1/gripper/{side}/set",
                           {"closedness": closedness, "grip": grip}))

    # -- inspection -------------------------------------------------------- #
    def posts(self, path: str) -> List[Any]:
        return [body for method, p, body in self.calls
                if method == "POST" and p == path]


@pytest.fixture
def executor():
    """A firmware executor over the fake, with time frozen and sleep recorded."""
    client = FakeClient()
    slept: List[float] = []
    clock = {"t": 0.0}

    def fake_sleep(seconds: float) -> None:
        slept.append(float(seconds))
        clock["t"] += float(seconds)

    robot = FirmwareExecutor(client, sleep=fake_sleep,
                             clock=lambda: clock["t"])
    robot.slept = slept          # type: ignore[attr-defined]
    robot.tick = clock           # type: ignore[attr-defined]
    return robot


# --------------------------------------------------------------------------- #
# lease
# --------------------------------------------------------------------------- #

def test_the_lease_is_taken_at_class_policy_and_handed_back(executor):
    with executor as robot:
        assert robot.lease == Lease("manipulation-kit", "policy", 7, 30, 30, None)
        taken = robot.client.posts("/v1/arm/lease")
        assert taken == [{"holder": "manipulation-kit", "class": "policy",
                          "ttl_s": 30}]
    released = [b for m, p, b in executor.client.calls
                if m == "DELETE" and p == "/v1/arm/lease"]
    assert released == [{"holder": "manipulation-kit"}]


def test_the_lease_is_released_even_when_the_body_raises(executor):
    with pytest.raises(RuntimeError, match="deliberate"):
        with executor:
            raise RuntimeError("deliberate")
    assert any(m == "DELETE" for m, p, _b in executor.client.calls
               if p == "/v1/arm/lease")


def test_a_failed_release_does_not_mask_the_real_exception(executor):
    def explode(method, path, body=None):
        if method == "DELETE":
            raise OSError("the daemon went away")
        return FakeClient.request(executor.client, method, path, body)
    executor.acquire()
    executor.client.request = explode
    executor.release()                       # must not raise
    assert executor.lease is None


def test_a_new_lease_epoch_means_we_were_displaced_and_forget_what_we_sent(executor):
    """An unchanged epoch means we never lost the arms; a higher one means
    everything we believed about their pose and mode is stale."""
    executor.acquire()
    executor._last_sent_deg = {"left": np.zeros(7), "right": np.zeros(7)}
    executor.client.lease_epoch = 8
    executor.acquire()
    assert executor._last_sent_deg == {}


def test_every_motion_command_carries_the_lease_holder(executor):
    with executor as robot:
        robot.send_joints(np.zeros(16), t=0.0)
    for body in executor.client.posts("/v1/arm/move_joints_both"):
        assert body["holder"] == "manipulation-kit"
    for method, path, body in executor.client.calls:
        if path.endswith("/mode"):
            assert body["holder"] == "manipulation-kit"


# --------------------------------------------------------------------------- #
# mode entry
# --------------------------------------------------------------------------- #

def test_position_mode_resends_the_ratios_even_when_already_holding(executor):
    """``recover`` leaves its OWN ratios behind, and a policy served at those
    drove the right arm to ~90 Nm on d1-2 (2026-09-10)."""
    with executor:
        pass
    modes = [b for m, p, b in executor.client.calls if p.endswith("/mode")]
    assert len(modes) == 2
    for body in modes:
        assert body["mode"] == "position"
        assert body["vel_ratio"] == pytest.approx(0.15)
        assert body["acc_ratio"] == pytest.approx(0.15)


def test_a_percent_velocity_ratio_is_refused_rather_than_clamped():
    """Passing 15 instead of 0.15 asks for 1500% and gets 100%."""
    with pytest.raises(ValueError, match="FRACTION"):
        FirmwareExecutor(FakeClient(), vel_ratio=15.0)


def test_engaging_position_from_idle_with_a_stale_command_is_refused(executor):
    executor.client.arms["a"] = FakeArmState(
        mode="idle", command_joints=(48.0,) + (0.0,) * 6)
    with pytest.raises(FirmwareUnavailable, match="stale command"):
        executor.position_mode()


def test_engaging_position_from_idle_is_allowed_when_it_is_anchored(executor):
    executor.client.arms["a"] = FakeArmState(
        mode="idle", command_joints=(ANCHOR_GAP_DEG - 0.5,) + (0.0,) * 6)
    executor.position_mode()      # no raise


# --------------------------------------------------------------------------- #
# streaming
# --------------------------------------------------------------------------- #

def test_streaming_runs_at_fifty_hertz(executor, d1_arm, observe):
    executor.transport = "stream"
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    assert plan.ok
    stamps = []
    inner = executor.client.request

    def timed(method, path, body=None):
        if path == "/v1/arm/move_joints_both":
            stamps.append(executor.tick["t"])
        return inner(method, path, body)

    executor.client.request = timed
    with executor as robot:
        robot.run_plan(plan)
    posts = executor.client.posts("/v1/arm/move_joints_both")
    assert len(posts) == len(plan.joint_steps())
    gaps = np.diff(np.array(stamps))
    assert gaps.size
    assert np.allclose(gaps, 0.02, atol=1e-9), "the stream is not at 50 Hz"
    assert all(s <= 0.02 + 1e-9 for s in executor.slept)


def test_the_streamed_command_is_rate_clamped_to_140_deg_per_second(executor):
    """dx-vr-teleop's anti-lunge clamp: 7 deg per tick at 20 Hz. A buffered
    flush must be spread over several commands, not arrive as one lunge."""
    with executor as robot:
        robot.send_joints(np.zeros(16), t=0.0)
        lunge = np.zeros(16)
        lunge[0] = np.radians(90.0)          # 90 degrees in one tick
        robot.send_joints(lunge, t=0.02)
    posts = executor.client.posts("/v1/arm/move_joints_both")
    step = abs(posts[1]["a"][0] - posts[0]["a"][0])
    assert step == pytest.approx(MAX_JOINT_RATE_DEG_S * robot.period)
    assert step == pytest.approx(2.8)


def test_continuous_motion_passes_the_clamp_untouched(executor):
    """The clamp defeats warps, never normal tracking: 2 deg per 20 ms tick is
    100 deg/s and must arrive exactly as asked."""
    with executor as robot:
        robot.send_joints(np.zeros(16), t=0.0)
        step = np.zeros(16)
        step[0] = np.radians(2.0)
        robot.send_joints(step, t=0.02)
    posts = executor.client.posts("/v1/arm/move_joints_both")
    assert posts[1]["a"][0] == pytest.approx(2.0)


def test_a_late_command_is_sent_immediately_rather_than_slept_on(executor):
    with executor as robot:
        robot.send_joints(np.zeros(16), t=0.0)
        robot.tick["t"] = 5.0                 # the caller stalled
        robot.send_joints(np.zeros(16), t=0.02)
    assert all(s >= 0.0 for s in executor.slept)


def test_degrees_go_on_the_wire_and_radians_stay_in_the_kit(executor):
    with executor as robot:
        q = np.zeros(16)
        q[0] = np.radians(10.0)
        robot.send_joints(q, t=0.0)
    body = executor.client.posts("/v1/arm/move_joints_both")[0]
    assert body["a"][0] == pytest.approx(10.0)
    assert body["wait"] is False


def test_the_logical_side_crossover_is_applied_once(executor):
    """logical left = SDK side A = the daemon's ``a``."""
    with executor as robot:
        robot.set_gripper("left", 1.0, grip="soft")
        robot.set_gripper("right", 0.0, grip="strong")
    grips = [(p, b) for m, p, b in executor.client.calls
             if p.startswith("/v1/gripper/")]
    assert grips[0][0] == "/v1/gripper/a/set"
    assert grips[0][1] == {"closedness": 1.0, "grip": "soft"}
    assert grips[1][0] == "/v1/gripper/b/set"


# --------------------------------------------------------------------------- #
# trajectory (the default)
# --------------------------------------------------------------------------- #

def test_a_plan_is_uploaded_as_one_trajectory_by_default(executor, d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    assert plan.ok
    with executor as robot:
        report = robot.run_plan(plan)
    assert report.completed
    assert not executor.client.posts("/v1/arm/move_joints_both")
    uploads = executor.client.posts("/v1/arm/trajectory/start")
    assert uploads, "the default transport did not upload a trajectory"
    assert all(b["holder"] == "manipulation-kit" for b in uploads)


def test_the_trajectory_starts_at_the_measured_pose(executor, d1_arm, observe):
    """The daemon refuses a trajectory whose first pose is more than 3 degrees
    from feedback, so the first waypoint has to BE the feedback."""
    executor.client.arms["a"] = FakeArmState(feedback_joints=(5.0,) + (0.0,) * 6)
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    with executor as robot:
        robot.run_plan(plan)
    first = executor.client.posts("/v1/arm/trajectory/start")[0]["waypoints"][0]
    assert first["t"] == 0.0
    assert first["a"][0] == pytest.approx(5.0)


def test_trajectory_segments_respect_the_same_rate_ceiling(executor, d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    with executor as robot:
        robot.run_plan(plan)
    for upload in executor.client.posts("/v1/arm/trajectory/start"):
        points = upload["waypoints"]
        assert points[1]["t"] >= INTERPOLATION_S - 1e-9
        for before, after in zip(points, points[1:]):
            dt = after["t"] - before["t"]
            assert dt > 0.0, "waypoint times must be strictly increasing"
            span = max(max(abs(x - y) for x, y in zip(after["a"], before["a"])),
                       max(abs(x - y) for x, y in zip(after["b"], before["b"])))
            assert span / dt <= MAX_JOINT_RATE_DEG_S + 1e-6


def test_gripper_strokes_split_the_trajectory_so_ordering_survives(
        executor, d1_arm, observe):
    """Open, travel, close: if the whole path went up as one job the stroke
    would land at the wrong moment."""
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    with executor as robot:
        robot.run_plan(plan)
    order = [p for m, p, _b in executor.client.calls
             if p in ("/v1/arm/trajectory/start", "/v1/gripper/a/set")]
    assert order == ["/v1/gripper/a/set", "/v1/arm/trajectory/start",
                     "/v1/gripper/a/set"]


def test_a_failed_trajectory_raises_with_the_daemons_own_message(
        executor, d1_arm, observe):
    executor.client.phases = ["failed"]
    d1_arm.set_joints("left", d1_arm.home("left") + 0.2)
    plan = GoHome().plan(observe(d1_arm, block_p=REACHABLE), d1_arm)
    assert plan.ok and plan.joint_steps()
    with pytest.raises(FirmwareUnavailable, match="boom"):
        with executor as robot:
            robot.run_plan(plan)


def test_a_refused_plan_reaches_the_daemon_not_at_all(executor, d1_arm, observe):
    world = observe(d1_arm, block_p=(0.52, 0.25, 0.45))
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    assert not plan.ok
    with executor as robot:
        report = robot.run_plan(plan)
    assert not report.completed and plan.reason in report.error
    assert not executor.client.posts("/v1/arm/trajectory/start")


# --------------------------------------------------------------------------- #
# state and settle
# --------------------------------------------------------------------------- #

def test_state_reports_radians_and_the_measured_holding_flag(executor):
    executor.client.arms["a"] = FakeArmState(feedback_joints=(90.0,) + (0.0,) * 6)
    executor.client.grippers["a"] = FakeGripperState(holding=True, jaw_rad=0.58)
    state = executor.state()
    assert state.joints["left"][0] == pytest.approx(np.pi / 2)
    assert state.holding["left"] is True
    assert state.grippers["left"] == pytest.approx(0.5, abs=1e-3)


def test_settle_polls_until_both_arms_report_stationary(executor):
    executor.client.settle_after = 6        # three polls of two arms
    report = executor.settle(2.0)
    assert report.settled
    assert report.waited_s > 0.0


def test_settle_gives_up_and_says_what_was_still_moving(executor):
    executor.client.settle_after = 10_000
    report = executor.settle(0.1)
    assert not report.settled
    assert report.worst_velocity_deg_s == pytest.approx(12.0)
    assert "still moving" in report.detail


def test_an_unreadable_gripper_is_absent_rather_than_reported_open(executor):
    """A gripper that cannot be read is UNKNOWN. Reporting it as open is how a
    verifier concludes the robot let go of something it is still holding."""
    def explode(side):
        raise RuntimeError("gripper is silent")
    executor.client.gripper_state = explode
    state = executor.state()
    assert "left" not in state.grippers and "right" not in state.grippers


def test_an_unknown_transport_is_refused_at_construction():
    with pytest.raises(ValueError, match="trajectory"):
        FirmwareExecutor(FakeClient(), transport="udp")
