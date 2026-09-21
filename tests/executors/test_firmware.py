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

    #: how many status polls a trajectory runs for before it completes
    play_polls: int = 0
    #: how many gripper reads a stroke takes to reach its target
    stroke_polls: int = 1
    #: the arm never gets where it was sent — the "stopped short" case
    arrival_offset_deg: float = 0.0
    _pending: Dict[str, Any] = field(default_factory=dict)
    _stroke: Dict[str, Any] = field(default_factory=dict)
    cancelled: List[int] = field(default_factory=list)

    def request(self, method: str, path: str, body: Any = None) -> Any:
        self.calls.append((method, path, body))
        if path == "/v1/arm/lease" and method == "POST":
            return {"holder": body["holder"], "class": body.get("class", "ambient"),
                    "epoch": self.lease_epoch, "ttl_s": body.get("ttl_s", 30),
                    "expires_in_s": body.get("ttl_s", 30), "preempted_from": None}
        if path == "/v1/arm/lease" and method == "DELETE":
            return None
        if path.endswith("/mode"):
            return None
        if path == "/v1/arm/move_joints_both":
            # THE ARM FOLLOWS. A fake whose feedback never moves cannot tell
            # a transport that reaches its endpoint from one that stops short,
            # which is exactly the bug R4 is about.
            self._apply({"a": body["a"], "b": body["b"]})
            return None
        if path == "/v1/arm/trajectory/start":
            self._pending = {"left": 0, "waypoints": body["waypoints"],
                             "polls": self.play_polls}
            return {"id": 42, "phase": "running", "elapsed_ms": 0}
        if path.endswith("/cancel"):
            self.cancelled.append(int(path.split("/")[-2]))
            self._pending = {}
            return None
        if path.startswith("/v1/arm/trajectory/"):
            phase = self.phases.pop(0) if len(self.phases) > 1 else self.phases[0]
            if self._pending.get("polls", 0) > 0:
                self._pending["polls"] -= 1
                return {"id": 42, "phase": "running", "elapsed_ms": 10}
            if phase == "completed" and self._pending:
                self._apply(self._pending["waypoints"][-1])
                self._pending = {}
            return {"id": 42, "phase": phase, "elapsed_ms": 10, "message": "boom"}
        raise AssertionError(f"unexpected call {method} {path}")

    def _apply(self, point: Dict[str, Any]) -> None:
        for key, side in (("a", "a"), ("b", "b")):
            reached = tuple(float(v) + self.arrival_offset_deg
                            for v in point[key])
            self.arms[side] = FakeArmState(
                mode=self.arms[side].mode, feedback_joints=reached,
                command_joints=tuple(float(v) for v in point[key]),
                stationary=True)

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
        stroke = self._stroke.get(side)
        if stroke is not None:
            if stroke["polls"] > 0:
                stroke["polls"] -= 1
                # still travelling: a different jaw reading every poll
                return FakeGripperState(kind="moving",
                                        jaw_rad=stroke["jaw"] + 0.01 * stroke["polls"])
            self.grippers[side] = FakeGripperState(
                kind="idle", jaw_rad=stroke["jaw"],
                holding=self.grippers[side].holding)
            self._stroke.pop(side)
        return self.grippers[side]

    def gripper_set(self, side: str, closedness: float, *, grip=None) -> None:
        self.calls.append(("POST", f"/v1/gripper/{side}/set",
                           {"closedness": closedness, "grip": grip}))
        target = (1.0 - float(closedness)) * self.grippers[side].open_rad
        self._stroke[side] = {"jaw": target, "polls": int(self.stroke_polls)}

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
        assert robot.lease == Lease(robot.holder, "policy", 7, 30, 30, None)
        taken = robot.client.posts("/v1/arm/lease")
        assert taken == [{"holder": robot.holder, "class": "policy",
                          "ttl_s": 30}]
    released = [b for m, p, b in executor.client.calls
                if m == "DELETE" and p == "/v1/arm/lease"]
    assert released == [{"holder": executor.holder}]


def test_two_sessions_do_not_share_one_holder_name(executor):
    """R6. Every process defaulting to the literal 'manipulation-kit' means
    the daemon cannot tell them apart: a second one renews the first one's
    lease, and either one's DELETE hands back the arms the other is driving."""
    from manipulation_kit.executors.firmware import default_holder
    mine, theirs = default_holder(), default_holder()
    assert mine != theirs
    assert mine.startswith("manipulation-kit/")
    assert executor.holder.startswith("manipulation-kit/")


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
        assert body["holder"] == executor.holder
    for method, path, body in executor.client.calls:
        if path.endswith("/mode"):
            assert body["holder"] == executor.holder


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
    # 50 Hz OR SLOWER, never faster: a segment that would break the 140 deg/s
    # ceiling at 20 ms is given more time rather than a smaller motion (R4).
    assert np.all(gaps >= 0.02 - 1e-9), "the stream went faster than 50 Hz"
    assert np.all(gaps <= 0.02 + 1e-9 + MAX_JOINT_RATE_DEG_S)


def test_a_lunge_is_refused_rather_than_clipped(executor):
    """R4. The clamp used to CLIP each component against the last transmitted
    vector, which changes the checked geometry, can change the path, and stops
    short of the endpoint. A synthetic 0 -> 10 -> 20 deg plan transmitted
    0 -> 2.8 -> 5.6 and reported completed.

    A command too large for one POST is now refused BEFORE it is sent."""
    from manipulation_kit.executors.firmware import (MAX_COMMAND_STEP_DEG,
                                                     RateRefused)
    with executor as robot:
        robot.send_joints(np.zeros(16), t=0.0)
        lunge = np.zeros(16)
        lunge[0] = np.radians(90.0)          # 90 degrees in one tick
        with pytest.raises(RateRefused, match="over the"):
            robot.send_joints(lunge, t=0.02)
    posts = executor.client.posts("/v1/arm/move_joints_both")
    assert len(posts) == 1, "the refused command must not reach the daemon"
    assert MAX_COMMAND_STEP_DEG == pytest.approx(np.degrees(0.25))


def test_a_streamed_plan_reaches_its_exact_endpoint(executor, d1_arm, observe):
    """R4, the measured half. A real Grasp streamed with the old clamp ended
    6.5283 deg from its final target at the worst joint while the report said
    completed, and the close stroke followed the truncated travel.

    Nothing here may differ from the plan by more than the arrival tolerance."""
    executor.transport = "stream"
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    assert plan.ok
    with executor as robot:
        report = robot.run_plan(plan)
    assert report.completed, report.error
    final = plan.final_joints()["left"]
    sent = executor.client.posts("/v1/arm/move_joints_both")
    assert len(sent) == len(plan.joint_steps())
    worst = max(abs(a - b) for a, b in zip(sent[-1]["a"], np.degrees(final)))
    assert worst < 1e-6, f"the stream stopped {worst:.4f} deg short"


def test_the_stream_stretches_time_instead_of_shrinking_the_motion(
        executor, d1_arm, observe):
    """R4. Honouring 140 deg/s is a statement about WHEN, not about WHERE.

    Every knot keeps its geometry; a segment that would be too fast at the
    base period is given more time."""
    executor.transport = "stream"
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    times = executor.stream_schedule(plan)
    assert len(times) == len(plan.joint_steps())
    # the first segment gets the same interpolation window the trajectory
    # upload gives its own, so the controller eases in rather than steps in
    assert times[0] >= INTERPOLATION_S - 1e-9
    previous = {s: np.degrees(np.asarray(q, dtype=float))
                for s, q in plan.binding.q0.items()}
    assert all(b > a for a, b in zip(times, times[1:]))
    for step, before, after in zip(plan.joint_steps(), [0.0] + times[:-1], times):
        span = float(np.max(np.abs(np.degrees(np.asarray(step.q))
                                   - previous[step.side])))
        dt = after - before
        # 0.1% slack: the schedule is rounded to microseconds on the wire
        assert span / max(dt, 1e-9) <= MAX_JOINT_RATE_DEG_S * 1.001
        previous[step.side] = np.degrees(np.asarray(step.q))


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
    assert all(b["holder"] == executor.holder for b in uploads)


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
    would land at the wrong moment.

    The travel is TWO jobs, not one, and that is the tool-space barrier: a
    grasp's standoff is a waypoint the plan marks ``arrive``, so the batch is
    flushed and the jaw pocket measured there before the descent — which is
    the one leg that may not be re-routed — is uploaded at all.
    """
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    with executor as robot:
        robot.run_plan(plan)
    order = [p for m, p, _b in executor.client.calls
             if p in ("/v1/arm/trajectory/start", "/v1/gripper/a/set")]
    assert order == ["/v1/gripper/a/set", "/v1/arm/trajectory/start",
                     "/v1/arm/trajectory/start", "/v1/gripper/a/set"]


def test_a_failed_trajectory_stops_the_run_with_the_daemons_own_message(
        executor, d1_arm, observe):
    """A transport fault is an EXECUTION outcome with a code, not a bare
    exception a consumer has to know to catch (R6/R7)."""
    executor.client.phases = ["failed"]
    d1_arm.set_joints("left", d1_arm.home("left") + 0.2)
    plan = GoHome().plan(observe(d1_arm, block_p=REACHABLE), d1_arm)
    assert plan.ok and plan.joint_steps()
    with executor as robot:
        report = robot.run_plan(plan)
    assert not report.completed
    assert report.stop_reason == "transport_error"
    assert "boom" in report.error


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


# --------------------------------------------------------------------------- #
# R5 — one time origin per run
# --------------------------------------------------------------------------- #

def test_a_second_plan_gets_its_own_time_origin(executor):
    """R5. ``_t0`` was set once, at the first send of the FIRST plan, while
    the runner restarts ``t`` at zero for every plan. On a fake clock the
    first plan went out at 0.00 / 0.02 / 0.04 s and, after a one-second model
    turn, the second plan sent all three at 1.04 s — a burst with no time
    between the commands at all."""
    stamps: List[float] = []
    inner = executor.client.request

    def timed(method, path, body=None):
        if path == "/v1/arm/move_joints_both":
            stamps.append(executor.tick["t"])
        return inner(method, path, body)

    executor.client.request = timed
    with executor as robot:
        robot.begin_run()
        for i in range(3):
            robot.send_joints(np.zeros(16), t=i * robot.period)
        robot.tick["t"] += 1.0            # the model thought about it
        robot.begin_run()
        for i in range(3):
            robot.send_joints(np.zeros(16), t=i * robot.period)
    gaps = np.diff(np.array(stamps[3:]))
    assert gaps.size == 2
    assert np.all(gaps >= robot.period - 1e-9), (
        f"the second plan flushed at {stamps[3:]}")


def test_a_stall_stretches_the_schedule_instead_of_flushing_it(executor):
    """R5. A blocking stroke or a late reply makes every later point overdue.
    Flushing them turns a 2.8 deg-per-command bound into no bound at all."""
    stamps: List[float] = []
    inner = executor.client.request

    def timed(method, path, body=None):
        if path == "/v1/arm/move_joints_both":
            stamps.append(executor.tick["t"])
        return inner(method, path, body)

    executor.client.request = timed
    with executor as robot:
        robot.begin_run()
        robot.send_joints(np.zeros(16), t=0.0)
        robot.tick["t"] += 0.5             # a slow gripper stroke
        for i in range(1, 4):
            robot.send_joints(np.zeros(16), t=i * robot.period)
    gaps = np.diff(np.array(stamps[1:]))
    assert np.all(gaps >= robot.period - 1e-9), (
        f"the points after the stall arrived together: {stamps}")


# --------------------------------------------------------------------------- #
# R6 — the lease, for the whole session
# --------------------------------------------------------------------------- #

def test_the_lease_is_renewed_across_more_than_two_ttls(executor):
    """R6. ``acquire`` was the only renewal implementation and nothing called
    it periodically: a model turn, a long trajectory or a slow settle longer
    than the 30 s TTL simply lost the arms."""
    with executor as robot:
        robot._want_heartbeat = False
        taken = len(robot.client.posts("/v1/arm/lease"))
        for _ in range(3):
            robot.tick["t"] += robot.ttl_s * 0.9
            robot.renew()
        renewals = len(robot.client.posts("/v1/arm/lease")) - taken
    assert renewals == 3, f"{renewals} renewals over 2.7 TTLs"


def test_a_preemption_during_a_run_stops_it_rather_than_being_absorbed(executor):
    """R6. A new epoch means somebody senior held the arms in between, so the
    mode, the ratios and the posture are all stale. It used to clear a cache
    and carry on."""
    from manipulation_kit.executors.firmware import LeasePreempted
    with executor as robot:
        robot._want_heartbeat = False
        robot.client.lease_epoch = 9
        robot.tick["t"] += robot.ttl_s
        with pytest.raises(LeasePreempted, match="epoch"):
            robot.renew()


def test_a_failed_entry_hands_the_arms_back(executor):
    """R6. ``__enter__`` acquired and then changed mode with no cleanup: the
    probe confirmed NO DELETE was ever sent after a failed entry, so the arms
    were held until the TTL expired by a process that never ran anything."""
    executor.client.arms["b"] = FakeArmState(
        mode="idle", command_joints=(48.0,) + (0.0,) * 6)
    with pytest.raises(FirmwareUnavailable, match="stale command"):
        with executor:
            pass
    released = [b for m, p, b in executor.client.calls
                if m == "DELETE" and p == "/v1/arm/lease"]
    assert released == [{"holder": executor.holder}]
    assert executor.lease is None


def test_a_polling_failure_cancels_the_job_it_was_watching(executor, d1_arm,
                                                           observe):
    """R6. ``_play`` cancelled only on its own deadline, so a status-poll
    exception left the daemon playing a path nobody was watching."""
    d1_arm.set_joints("left", d1_arm.home("left") + 0.2)
    plan = GoHome().plan(observe(d1_arm, block_p=REACHABLE), d1_arm)
    inner = executor.client.request
    calls = {"n": 0}

    def flaky(method, path, body=None):
        if path.startswith("/v1/arm/trajectory/") and path.endswith("/status"):
            calls["n"] += 1
            if calls["n"] == 1:
                raise OSError("the daemon went away")
        return inner(method, path, body)

    with executor as robot:
        robot.client.request = flaky
        with pytest.raises(OSError):
            robot.run_plan(plan)
    assert executor.client.cancelled == [42]


# --------------------------------------------------------------------------- #
# R7 — measured arrival and stroke barriers
# --------------------------------------------------------------------------- #

def test_the_jaws_do_not_close_on_an_arm_that_stopped_short(executor, d1_arm,
                                                            observe):
    """R7. Trajectory ``completed`` says the daemon played its last waypoint.
    A stopped arm can be short of the target — F5, ten grasps out of ten, the
    arm holding 2.5 deg out with the fingers jammed on the table — and the
    close stroke used to follow regardless."""
    executor.client.arrival_offset_deg = 9.0     # never quite gets there
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    with executor as robot:
        report = robot.run_plan(plan)
    assert not report.completed
    assert report.stop_reason == "barrier_failed"
    assert "from the commanded posture" in report.error
    closes = [b for m, p, b in executor.client.calls
              if p == "/v1/gripper/a/set" and b["closedness"] == 1.0]
    assert not closes, "the jaws closed on an arm that had not arrived"


def test_a_stroke_that_never_settles_stops_the_run(executor, d1_arm, observe):
    """R7 / F13. ``set_gripper`` posted the command and discarded the reply,
    so the next step ran while the jaws were still travelling and the evidence
    was read mid-stroke."""
    executor.client.stroke_polls = 10_000
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    with executor as robot:
        report = robot.run_plan(plan)
    assert not report.completed
    assert report.stop_reason == "barrier_failed"
    assert "terminal state" in report.error
    assert not executor.client.posts("/v1/arm/trajectory/start"), (
        "the arm moved after a stroke that never finished")


def test_a_stroke_that_stalls_on_the_object_is_a_completed_stroke(executor):
    """The terminal state of a force-limited close is a STALL, not the target
    position. Waiting for the commanded closedness would time out on every
    successful grasp."""
    executor.client.grippers["a"] = FakeGripperState(holding=True, jaw_rad=0.58)
    report = executor.wait_gripper_settled("left", timeout_s=1.0)
    assert report.settled and report.holding


def test_a_failed_settle_stops_the_run(executor, d1_arm, observe):
    """R7. Both runners continued after ``SettleReport(False)`` and reported
    completion."""
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    with executor as robot:
        robot.client.settle_after = 10_000
        report = robot.run_plan(plan)
    assert not report.completed
    assert report.stop_reason == "barrier_failed"
    assert "still moving" in report.error


# --------------------------------------------------------------------------- #
# R14 — construction arguments
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize("kwargs", [
    {"hz": 0.0}, {"hz": float("nan")}, {"max_joint_rate_deg_s": -1.0},
    {"arrive_timeout_s": float("inf")}, {"stroke_timeout_s": 0.0},
    {"ttl_s": 0},
])
def test_a_nonsense_construction_argument_is_refused(kwargs):
    """R14. A NaN hz makes every period a NaN, every sleep a no-op and every
    deadline unreachable, and nothing downstream would have said so."""
    with pytest.raises(ValueError):
        FirmwareExecutor(FakeClient(), **kwargs)
