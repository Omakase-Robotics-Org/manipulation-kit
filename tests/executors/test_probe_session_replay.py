"""The d1-2 probe trial of 2026-09-23 01:42Z, replayed on a fake daemon.

What happened on the robot (``docs/probe-hardware-trial.md``, kit
``feat/perceive-head`` @ fcc2087, d1-firmwared 0.3.0 @ 897b796): after
``pose_for_probe.py`` (Approach to the standoff over x 0.403 y 0.10) and a
5 cm Nudge down, three air probes and three table probes worked, and table
probe 4 was refused by the daemon at upload — ``HTTP 400: trajectory values
must be finite with increasing times`` — with the left wrist at J5 = 171.5 deg
(box +/-173) and J7 flipped from -55 to +42.5 deg.

Two faults, both reproduced here from the standoff posture alone:

1. THE WRIST FLIP. ``Probe`` "keeps the roll the hand has" by asking
   ``align_tool`` for the jaw axis it has, and ``align_tool`` folds that axis
   to the half-turn representative nearest its PADS_DOWN SEED — not the hand.
   A hand whose jaw axis pointed the other way (the d1-2 standoff: TCP x =
   -base x) was turned 180 deg in place: J5 -5 -> +169 deg in a 105-sample
   re-aim, then back, then again, parking J5 against its stop.
2. THE ZERO-DURATION KNOTS. With J5 pinned at the stop, the contact leg's
   solver re-solved the same posture (35 of 41 knots identical), each got
   the same distance along the leg, hence the same TIME, and the daemon
   refused the whole upload.

The fake validates every upload with the daemon's own rule
(``d1fw-core/src/arm_trajectory.rs::validate``) and blocks the left arm where
its tool point reaches the table plane (tape 0.166 m + the 29 mm pad lead).
Nothing here talks to a robot.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import pytest

from manipulation_kit.executor import run
from manipulation_kit.executors.firmware import (FirmwareExecutor,
                                                 TrajectoryInvalid)
from manipulation_kit.executors.firmware.client import check_waypoints
from manipulation_kit.executors.firmware.errors import FirmwareError
from manipulation_kit.primitives import (ContactCriterion, ContactStep, Nudge,
                                         Probe, record_contacts)
from manipulation_kit.primitives.orientation import tool_from_link7
from manipulation_kit.primitives.types import JOINT_LIMIT, JointStep
from manipulation_kit.world import ArmView, GripperView, WorldView
from manipulation_kit.world.direction import Direction

SIDES = ("left", "right")
#: the left arm at the Approach standoff (pose_for_probe.py, mirror), deg —
#: FK: tool (0.403, 0.091, 0.285), fingertips down, TCP x along -base x
STANDOFF_LEFT_DEG = (-21.091, -68.997, 37.832, -84.469, -5.509, -34.506,
                     -55.314)
def _right_home_deg():
    """The right arm stood at HOME; read from the canonical file, not inlined
    (test_description_consistency)."""
    import json
    from pathlib import Path
    cfg = (Path(__file__).resolve().parents[2] / "src" / "manipulation_kit"
           / "config" / "home_pose.json")
    return tuple(round(float(v), 2)
                 for v in json.loads(cfg.read_text())["home_pose"][7:14])


RIGHT_DEG = _right_home_deg()
#: the arm after the live run (brief, 2026-09-23 01:42Z)
LIVE_END_LEFT_DEG = (-23.5, -79.4, 52.1, -85.4, 171.5, 29.6, 42.5)
TABLE_Z_M = 0.166
PAD_LEAD_M = 0.029
LOAD = (0.5, 6.0, -1.0, 2.5, 0.0, 0.3, 0.0)
DAEMON_400 = ("invalid input: trajectory values must be finite with "
              "increasing times <=120 seconds")


# --------------------------------------------------------------------------- #
# the fake daemon
# --------------------------------------------------------------------------- #

def daemon_refusal(points) -> Optional[str]:
    """``arm_trajectory.rs::validate`` @ 897b796, rule for rule."""
    if not 2 <= len(points) <= 10_000 or points[0][0] != 0.0:
        return "invalid input: trajectory needs 2..10000 points starting at t=0"
    for i, (t, a, b) in enumerate(points):
        if (not math.isfinite(t) or t < 0.0 or t > 120.0
                or (i > 0 and t <= points[i - 1][0])
                or not all(math.isfinite(v) for v in list(a) + list(b))):
            return DAEMON_400
    return None


class _Arm:
    def __init__(self, fb, cmd=None, torque=LOAD):
        self.mode, self.error_code, self.stationary = "position", 0, True
        self.feedback_joints = tuple(float(v) for v in fb)
        self.command_joints = tuple(float(v) for v in (fb if cmd is None else cmd))
        self.feedback_velocity = (0.0,) * 7
        self.feedback_torque = tuple(torque)
        self.feedback_temperature = (30.0,) * 7
        self.frame_serial = 1


class _Grip:
    kind, jaw_rad, torque_nm, holding = "open", 1.35, 0.0, False
    grip_preload_rad, live, open_rad, coil_c, fault_code = 0.0, True, 1.35, 30, None


class _Status:
    def __init__(self, id, phase, elapsed_ms, message=None):
        self.id, self.phase, self.elapsed_ms, self.message = id, phase, elapsed_ms, message


class _Slider:
    height_m, moving, alarm, alarm_code, alarm_text = 0.205, False, False, 0, ""


class TableDaemon:
    """Plays every job on a fake clock; the left arm stops where its tool
    point reaches ``contact_z``, and the torque on J1 rises past it."""

    def __init__(self, kin, left_deg, right_deg, *, contact_z: float):
        self.kin, self.contact_z = kin, contact_z
        self.clock = {"t": 0.0}
        self.arms = {"a": _Arm(left_deg), "b": _Arm(right_deg)}
        self.uploads: List[Dict[str, Any]] = []
        self._job: Optional[Dict[str, Any]] = None
        self._id = 0

    def _z(self, a_deg) -> float:
        saved = np.array(self.kin.joints("left"), dtype=float)
        try:
            self.kin.set_joints("left", np.radians(a_deg))
            return float(tool_from_link7(*self.kin.ee_pose("left"))[0][2])
        finally:
            self.kin.set_joints("left", saved)

    @staticmethod
    def _at(points, tau):
        for (t0, a0, b0), (t1, a1, b1) in zip(points, points[1:]):
            if tau <= t1:
                f = 0.0 if t1 == t0 else (tau - t0) / (t1 - t0)
                return (np.asarray(a0) + f * (np.asarray(a1) - np.asarray(a0)),
                        np.asarray(b0) + f * (np.asarray(b1) - np.asarray(b0)))
        return np.asarray(points[-1][1], float), np.asarray(points[-1][2], float)

    def _start(self, points, route: str) -> int:
        refusal = daemon_refusal(points)
        self.uploads.append({"route": route, "points": points, "refused": refusal})
        if refusal is not None:
            raise FirmwareError("POST", "/v1/arm/trajectory/start", 400, refusal)
        block = None
        if route == "typed":                  # the contact leg
            tau, end = 0.0, points[-1][0]
            while tau <= end + 1e-9:
                if self._z(self._at(points, tau)[0]) <= self.contact_z:
                    block = tau
                    break
                tau += 0.005
        self._id += 1
        self._job = {"id": self._id, "points": points, "t0": self.clock["t"],
                     "frozen": None, "block": block}
        return self._id

    def _tau(self) -> float:
        job = self._job
        return job["frozen"] if job["frozen"] is not None else self.clock["t"] - job["t0"]

    def _advance(self) -> None:
        job = self._job
        if job is None:
            return
        tau = self._tau()
        a, b = self._at(job["points"], tau)
        torque = list(LOAD)
        fb = a
        if job["block"] is not None and tau >= job["block"]:
            fb = self._at(job["points"], job["block"])[0]
            torque[0] += 40.0 * (tau - job["block"])
        self.arms["a"] = _Arm(fb, a, torque)
        self.arms["b"] = _Arm(b)

    # typed operations (the contact leg)
    def trajectory_start(self, points, *, holder=None):
        pts = [(float(t), [float(v) for v in a], [float(v) for v in b])
               for t, a, b in points]
        return _Status(self._start(pts, "typed"), "running", 0)

    def trajectory_status(self, job):
        tau = self._tau()
        if self._job["frozen"] is not None:
            return _Status(job, "cancelled", int(tau * 1000))
        if tau >= self._job["points"][-1][0]:
            self._advance()
            self._job = None
            return _Status(job, "completed", int(tau * 1000))
        return _Status(job, "running", int(tau * 1000))

    def trajectory_cancel(self, job):
        if self._job is not None and self._job["id"] == job:
            self._job["frozen"] = self._tau()
            self._advance()
            self._job = None
        return _Status(job, "cancelled", 0)

    def arm_state(self, side):
        self._advance()
        return self.arms[side]

    def gripper_state(self, side):
        return _Grip()

    def gripper_set(self, side, closedness, *, grip=None, timeout_s=None):
        return None

    def slider_state(self):
        return _Slider()

    # the enveloped routes (ordinary legs)
    def request(self, method, path, body=None):
        if path == "/v1/arm/lease":
            return (None if method != "POST" else
                    {"holder": body["holder"], "class": body.get("class"),
                     "epoch": 1, "ttl_s": 30, "expires_in_s": 30,
                     "preempted_from": None})
        if path.endswith("/mode"):
            return None
        if path == "/v1/arm/trajectory/start":
            pts = [(float(w["t"]), list(w["a"]), list(w["b"]))
                   for w in body["waypoints"]]
            return {"id": self._start(pts, "request"), "phase": "running",
                    "elapsed_ms": 0}
        if path.endswith("/cancel"):
            if self._job is not None:
                self.trajectory_cancel(self._job["id"])
            return None
        if path.startswith("/v1/arm/trajectory/"):
            s = self.trajectory_status(int(path.split("/")[-2]))
            return {"id": s.id, "phase": s.phase, "elapsed_ms": s.elapsed_ms,
                    "message": s.message}
        raise AssertionError(f"unexpected call {method} {path}")


def _executor(daemon) -> FirmwareExecutor:
    def sleep(seconds):
        daemon.clock["t"] += float(seconds)
    return FirmwareExecutor(daemon, sleep=sleep, clock=lambda: daemon.clock["t"],
                            heartbeat=False, vel_ratio=0.15)


def _observe(robot, kin, like: WorldView = None) -> WorldView:
    """probe_trial.py's ``observe``: the world as the executor measures it."""
    state = robot.state()
    arms, grippers = [], []
    for side in SIDES:
        arm = state.arms[side]
        kin.set_joints(side, arm.q)
        p, r = tool_from_link7(*kin.ee_pose(side))
        arms.append(ArmView(side, joints=arm.q, tool_p=p, tool_r=r,
                            mode=arm.mode, error_code=arm.error_code))
        grippers.append(GripperView(side, 0.0))
    world = WorldView.of([] if like is None else list(like.objects),
                         arms=arms, grippers=grippers)
    return world if like is None else world.with_(contacts=like.contacts)


def _lift(robot, kin, world, dz):
    plan = Nudge(side="left", dz=dz, frame="base").plan(world, kin)
    assert plan.ok, str(plan)
    report = run(plan, robot, kin=kin)
    assert report.completed, report.to_json()


def _probe(robot, kin, world, max_travel_m, declare_as=""):
    probe = Probe(side="left", direction="down", max_travel_m=max_travel_m,
                  declare_as=declare_as)
    plan = probe.plan(world, kin)
    assert plan.ok, str(plan)
    report = run(plan, robot, kin=kin)
    return report, record_contacts(_observe(robot, kin, world), report, probe)


# --------------------------------------------------------------------------- #
# the replay
# --------------------------------------------------------------------------- #

@pytest.fixture(scope="module")
def session():
    """nudge_up.py --dz -0.05, then probe_trial.py: lift 5 cm, 3 x (air
    probe 30 mm, lift 3 cm), 10 x (table probe 120 mm, lift 5 cm)."""
    from manipulation_kit.arms import get_arm_kinematics
    kin = get_arm_kinematics("d1/arm", quiet=True)
    daemon = TableDaemon(get_arm_kinematics("d1/arm", quiet=True),
                         STANDOFF_LEFT_DEG, RIGHT_DEG,
                         contact_z=TABLE_Z_M + PAD_LEAD_M)
    lines = []
    with _executor(daemon) as robot:
        world = _observe(robot, kin)
        _lift(robot, kin, world, -0.05)
        world = _observe(robot, kin)
        _lift(robot, kin, world, 0.05)
        world = _observe(robot, kin, world)
        for _ in range(3):
            report, world = _probe(robot, kin, world, 0.03)
            lines.append(("air", report))
            assert report.completed, report.to_json()
            _lift(robot, kin, _observe(robot, kin, world), 0.03)
            world = _observe(robot, kin, world)
        postures = []
        for _ in range(10):
            report, world = _probe(robot, kin, world, 0.12, "table")
            lines.append(("table", report))
            if not report.completed:
                break
            _lift(robot, kin, world, 0.05)
            world = _observe(robot, kin, world)
            postures.append(np.degrees(robot.state().joints["left"]))
    return {"kin": kin, "daemon": daemon, "lines": lines, "postures": postures}


def test_the_d1_2_probe_session_replays_ten_table_probes_without_a_400(session):
    table = [r for kind, r in session["lines"] if kind == "table"]
    assert [r.completed for r in table] == [True] * 10, \
        [r.error for r in table if not r.completed]
    assert [r.contacts[-1].stopped_by for r in table] == ["contact"] * 10
    for r in table:
        assert r.contacts[-1].p_tool[2] == pytest.approx(
            TABLE_Z_M + PAD_LEAD_M, abs=0.002)
    uploads = session["daemon"].uploads
    assert uploads and all(u["refused"] is None for u in uploads)
    air = [r for kind, r in session["lines"] if kind == "air"]
    assert [r.contacts[-1].stopped_by for r in air] == ["max_travel"] * 3


def test_posture_after_ten_probe_cycles_stays_clear_of_every_box_limit(session):
    kin = session["kin"]
    lo, hi = (np.degrees(v) for v in kin.limits("left"))
    assert len(session["postures"]) == 10
    for n, q in enumerate(session["postures"]):
        margin = np.minimum(q - lo, hi - q)
        assert margin.min() > 15.0, (n, np.round(q, 1).tolist())
    # and the wrist never turned over: J5 stays on the standoff's side
    j5 = [q[4] for q in session["postures"]]
    assert max(abs(v - STANDOFF_LEFT_DEG[4]) for v in j5) < 30.0, j5


def test_no_probe_re_aim_turns_the_wrist_over(session):
    """Every ordinary upload (re-aims, lifts, relieves) moves J5 by less than
    30 deg: the live run's re-aims were ~175 deg half-turns."""
    for u in session["daemon"].uploads:
        j5 = [p[1][4] for p in u["points"]]
        assert max(j5) - min(j5) < 30.0, (u["route"], len(u["points"]))


# --------------------------------------------------------------------------- #
# the two mechanisms, pinned one by one
# --------------------------------------------------------------------------- #

def _world_at(kin, left_deg) -> WorldView:
    kin.set_joints("left", np.radians(left_deg))
    kin.set_joints("right", np.radians(RIGHT_DEG))
    arms, grippers = [], []
    for side in SIDES:
        p, r = tool_from_link7(*kin.ee_pose(side))
        arms.append(ArmView(side, joints=kin.joints(side), tool_p=p, tool_r=r,
                            mode="position"))
        grippers.append(GripperView(side, 0.0))
    return WorldView.of([], arms=arms, grippers=grippers)


def test_a_probe_from_the_standoff_keeps_the_roll_the_hand_has(d1_arm):
    """The first air probe's re-aim at fcc2087 was 104 knots, J5 -5 -> +169."""
    lifted = (-19.8, -67.8, 35.5, -85.0, -4.6, -34.2, -55.6)
    plan = Probe(side="left", direction="down", max_travel_m=0.03).plan(
        _world_at(d1_arm, lifted), d1_arm)
    assert plan.ok, str(plan)
    assert not any("jaws rolled" in n for n in plan.notes)
    q = [np.degrees(s.q) for s in plan.steps if isinstance(s, JointStep)]
    assert len(q) < 20
    assert max(abs(v[4] - lifted[4]) for v in q) < 10.0
    assert max(abs(v[6] - lifted[6]) for v in q) < 10.0


def test_a_probe_from_the_live_end_posture_is_refused_joint_limit(d1_arm):
    """J5 = 171.5 of 173 deg: the only postures left are next to the stop, so
    the probe says so instead of planning a leg the solver pins there."""
    plan = Probe(side="left", direction="down", max_travel_m=0.12,
                 declare_as="table").plan(_world_at(d1_arm, LIVE_END_LEFT_DEG),
                                          d1_arm)
    assert not plan.ok
    assert plan.reason == JOINT_LIMIT, str(plan)
    assert "J5" in plan.detail


def test_a_contact_leg_with_non_advancing_knots_is_timed_strictly_increasing():
    q0 = np.radians(np.full(7, 10.0))
    q1 = q0 + np.radians([0, 0, 0, 0, 0.5, 0, 0])      # a wrist correction
    q2 = q1 + np.radians([0, 1.0, 0, 0, 0, 0, 0])
    step = ContactStep("left", Direction((0.0, 0.0, -1.0), "base"), 0.05,
                       ContactCriterion(), waypoint=1, path=(q0, q1, q2),
                       s=(0.0, 0.0, 0.01), speed_m_s=0.01)
    times = [t for t, _q in step.timed_path()]
    assert all(b > a for a, b in zip(times, times[1:])), times
    assert step.duration_s() == times[-1]


def test_a_contact_leg_whose_distance_goes_backwards_is_refused():
    q = np.radians(np.full(7, 10.0))
    with pytest.raises(ValueError, match="must not decrease"):
        ContactStep("left", Direction((0.0, 0.0, -1.0), "base"), 0.05,
                    ContactCriterion(), waypoint=1, path=(q, q, q),
                    s=(0.0, 0.01, 0.005))


def test_a_trajectory_with_equal_consecutive_times_is_refused_before_upload():
    a, b = [10.0] * 7, [0.0] * 7
    with pytest.raises(TrajectoryInvalid) as err:
        check_waypoints([(0.0, a, b), (0.5, a, b), (0.5, a, b), (1.0, a, b)])
    assert "knot 2: t=0.500000 s does not increase after knot 1" in str(err.value)
    with pytest.raises(TrajectoryInvalid, match="non-finite joint"):
        check_waypoints([(0.0, a, b), (0.5, a[:6] + [float("nan")], b)])
    with pytest.raises(TrajectoryInvalid, match="first time must be 0"):
        check_waypoints([(0.1, a, b), (0.5, a, b)])


def test_the_contact_leg_uploader_refuses_equal_times_without_calling_the_daemon(
        d1_arm):
    daemon = TableDaemon(d1_arm, (10.0,) * 7, (0.0,) * 7, contact_z=-10.0)
    robot = _executor(daemon)
    q0 = np.radians(np.full(7, 10.0))
    q1 = q0 + np.radians([0, 1.0, 0, 0, 0, 0, 0])
    q2 = q1 + np.radians([0, 1.0, 0, 0, 0, 0, 0])
    with pytest.raises(TrajectoryInvalid, match="does not increase"):
        robot.move_until([(0.0, q0), (1.0, q1), (1.0, q2)], side="left",
                         criterion=ContactCriterion(), kin=d1_arm)
    assert daemon.uploads == []
