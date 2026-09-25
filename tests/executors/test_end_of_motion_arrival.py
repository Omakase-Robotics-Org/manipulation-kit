"""The last leg of a plan is MEASURED before the run says ``completed``.

d1-2, 2026-09-22 22:42Z (``tests/data/d1_2_approach_20260922/approach.json``):
``Approach(probe_spot, left, down, contact="pad")`` through
``LiveRobot.firmware`` and ``run()``. The plan was right — forward kinematics
of its final posture is 9 mm from the standoff — and the daemon played it; the
wrist roll (J7) stopped at about -39.5 deg against a -90 deg command, which
puts the tool point 167 mm and 53 deg off the standoff. The run reported
``completed: true, arrivals: []`` because nothing in either runner compared the
measured arm with the command after the last leg:

* ``Approach``'s one waypoint was not marked ``arrive``, so no tool gate ran;
* the trajectory runner only flushed a gated waypoint when another joint leg
  or a stroke followed it, never before a settle or at the end of the plan;
* a ``SettleStep`` checks that the arms are STATIONARY, which an arm held
  50 deg short on one joint is.

These tests replay the recorded plan into a fake daemon whose J7 stops where
the real one did, on the trajectory transport and on the generic runner.
"""

from __future__ import annotations

import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from manipulation_kit.executor import (BARRIER_FAILED, KinematicExecutor,
                                       ToolGate, run)
from manipulation_kit.executors.firmware import FirmwareExecutor
from manipulation_kit.primitives import Approach
from manipulation_kit.primitives.types import (GripStep, JointStep, Plan,
                                               PlanBinding, SettleStep,
                                               Waypoint)

FIXTURE = (Path(__file__).resolve().parents[1] / "data"
           / "d1_2_approach_20260922" / "approach.json")


def _load_fakes():
    """``test_firmware.py``'s protocol-faithful fake, loaded by path
    (``tests/`` is deliberately not a package)."""
    spec = importlib.util.spec_from_file_location(
        "_fw_fakes_end_of_motion", Path(__file__).with_name("test_firmware.py"))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


_fakes = _load_fakes()


@pytest.fixture(scope="module")
def record():
    return json.loads(FIXTURE.read_text())


def _plan(record) -> Plan:
    """The recorded plan as a value, bound to the recorded posture."""
    p = record["plan"]
    waypoints = tuple(
        Waypoint(w["label"], np.asarray(w["p"], dtype=float),
                 R.from_quat(w["quat_xyzw"]), allow_via=w["allow_via"],
                 arrive=w["arrive"])
        for w in p["waypoints"])
    steps = []
    for s in p["steps"]:
        if s["step"] == "joint":
            steps.append(JointStep(s["side"], np.asarray(s["q_rad"]),
                                   s["waypoint"]))
        elif s["step"] == "grip":
            steps.append(GripStep(s["side"], s["closedness"], s["grip"],
                                  s["waypoint"]))
        elif s["step"] == "settle":
            steps.append(SettleStep(s["timeout_s"]))
        else:
            raise AssertionError(f"unexpected step in the fixture: {s}")
    q0 = {side: tuple(np.radians(q)) for side, q in record["q0_deg"].items()}
    return Plan(p["primitive"], p["side"], waypoints, tuple(steps),
                tuple(p["notes"]), binding=PlanBinding(q0=q0))


class StuckWristClient(_fakes.FakeClient):
    """A daemon that plays every waypoint and whose left J7 stops at
    ``stuck_deg`` whenever it is sent past it (the d1-2 run, as measured).

    The command is reported as SENT; only the feedback stops short — the
    arm is stationary, error-free and in position mode, exactly the state
    ``settle`` accepts.
    """

    stuck_deg = None

    def _apply(self, point):
        super()._apply(point)
        if self.stuck_deg is None:
            return
        a = self.arms["a"]
        fb = list(a.feedback_joints)
        if fb[6] < self.stuck_deg:
            fb[6] = self.stuck_deg
        self.arms["a"] = _fakes.FakeArmState(
            mode=a.mode, feedback_joints=tuple(fb),
            command_joints=a.command_joints, stationary=True)


def _executor(record, *, stuck: bool, transport: str = "trajectory"):
    client = StuckWristClient()
    client.stuck_deg = record["stuck_joint"]["deg"] if stuck else None
    for wire, side in (("a", "left"), ("b", "right")):
        q = tuple(float(v) for v in record["q0_deg"][side])
        client.arms[wire] = _fakes.FakeArmState(feedback_joints=q,
                                                command_joints=q)
    client.grippers = {k: _fakes.FakeGripperState(kind="open", jaw_rad=1.35,
                                                  open_rad=1.35) for k in "ab"}
    client.stroke_kind = "open"
    clock = {"t": 0.0}

    def sleep(seconds):
        clock["t"] += float(seconds)

    return FirmwareExecutor(client, sleep=sleep, clock=lambda: clock["t"],
                            vel_ratio=0.15, acc_ratio=0.15,
                            transport=transport)


def _tool(kin, q_deg):
    p, _r = ToolGate._tool(kin, "left", np.radians(q_deg))
    return np.asarray(p, dtype=float)


# --------------------------------------------------------------------------- #
# the diagnosis, offline
# --------------------------------------------------------------------------- #

def test_the_recorded_plan_ends_on_its_standoff(record, d1_arm):
    """Hypothesis 1 is FALSE: the plan was consistent. Its final posture is
    within 10 mm of the standoff it was planned for."""
    final = record["live"]["plan_final_joints_deg"]["left"]
    target = record["plan"]["waypoints"][0]["p"]
    assert np.linalg.norm(_tool(d1_arm, final) - target) < 0.010


def test_a_j7_stopped_at_minus_39_5_deg_is_the_measured_miss(record, d1_arm):
    """The measured tool point is the plan's final posture with ONE joint
    changed: J7 at -39.5 deg instead of -90 (0.3 mm, and the 53 deg facing
    error to 0.1 deg). Nothing else in the live trace moves only J7."""
    final = np.asarray(record["live"]["plan_final_joints_deg"]["left"])
    measured = record["live"]["verdict"]["measured"]["ToolAt"]["measured"]
    stuck = final.copy()
    stuck[record["stuck_joint"]["index"]] = record["stuck_joint"]["deg"]
    assert np.linalg.norm(_tool(d1_arm, stuck)
                          - measured["tool_p"]) < 0.002
    assert final[6] == pytest.approx(-90.0)


# --------------------------------------------------------------------------- #
# the regression: the run must not say completed
# --------------------------------------------------------------------------- #

def test_the_recorded_approach_with_a_stuck_wrist_is_not_completed(
        record, d1_arm):
    """The live run, replayed: the trajectory transport, the recorded plan,
    the wrist stopping where it stopped. It used to report
    ``completed: True, arrivals: []``."""
    executor = _executor(record, stuck=True)
    with executor:
        report = run(_plan(record), executor, kin=d1_arm)
    assert not report.completed, report.to_json()
    assert report.stop_reason == BARRIER_FAILED
    assert report.arrivals, "a failed run with no arrival on the record"
    last = report.arrivals[-1]
    assert not last.arrived
    assert last.worst_error_rad == pytest.approx(math.radians(50.5), abs=0.01)
    # the tool number travels with the joint one, and it is the live miss
    assert last.tool_error_m == pytest.approx(0.167, abs=0.01)
    assert report.refusal is not None
    assert "end of motion" in report.error and "50.5" in report.error


def test_the_recorded_approach_with_a_stuck_wrist_is_not_completed_generic(
        record, d1_arm):
    """The same plan through the generic runner (``run_steps``), on a
    kinematic executor whose left J7 stops where d1-2's did. (The streamed
    firmware transport refuses this plan before sending anything: its READY
    ramp has 14 deg knots, over the one-POST step bound.)"""
    stuck = math.radians(record["stuck_joint"]["deg"])

    class StuckWrist(KinematicExecutor):
        def send_joints(self, q16, *, t):
            q = np.asarray(q16, dtype=float).copy()
            q[6] = max(q[6], stuck)
            super().send_joints(q, t=t)

    saved = {side: np.array(d1_arm.joints(side)) for side in ("left", "right")}
    try:
        executor = StuckWrist(d1_arm)
        for side, q in record["q0_deg"].items():
            d1_arm.set_joints(side, np.radians(q))
        report = run(_plan(record), executor, kin=d1_arm)
    finally:
        for side, q in saved.items():
            d1_arm.set_joints(side, q)
    assert not report.completed, report.to_json()
    assert report.stop_reason == BARRIER_FAILED
    assert report.arrivals and not report.arrivals[-1].arrived
    assert report.arrivals[-1].tool_error_m == pytest.approx(0.167, abs=0.01)


def test_a_faithful_daemon_completes_the_recorded_approach_with_an_arrival(
        record, d1_arm):
    """The barrier costs nothing when the arm is there — and it is ON the
    record: ``arrivals`` is never empty for a run that moved the arm."""
    executor = _executor(record, stuck=False)
    with executor:
        report = run(_plan(record), executor, kin=d1_arm)
    assert report.completed, report.to_json()
    assert report.arrivals and all(a.arrived for a in report.arrivals)
    assert report.arrivals[-1].waypoint_label == "end of motion"


def test_a_plan_that_ends_on_joint_steps_is_measured_at_the_end(d1_arm):
    """No settle, no stroke after the last leg: the generic runner measures
    anyway, on an executor whose arm does not get there."""
    q0 = np.asarray(d1_arm.home("left"), dtype=float)
    q1 = q0 + np.radians([0, 0, 0, 0, 0, 0, 2.0])
    plan = Plan("nudge", "left", (), (JointStep("left", q1, 0),))

    class Short(KinematicExecutor):
        def send_joints(self, q16, *, t):
            q = np.asarray(q16, dtype=float).copy()
            q[6] -= math.radians(10.0)      # left J7 stops 10 deg short
            super().send_joints(q, t=t)

    saved = {side: np.array(d1_arm.joints(side)) for side in ("left", "right")}
    try:
        d1_arm.set_joints("left", q0)
        report = run(plan, Short(d1_arm), kin=d1_arm, allow_unbound=True)
    finally:
        for side, q in saved.items():
            d1_arm.set_joints(side, q)
    assert not report.completed
    assert report.stop_reason == BARRIER_FAILED


# --------------------------------------------------------------------------- #
# Approach gates its standoff
# --------------------------------------------------------------------------- #

def test_approach_gates_the_tool_at_its_standoff(d1_arm, observe):
    world = observe(d1_arm)
    plan = Approach(object="red_block", side="left").plan(world, d1_arm)
    assert plan.ok, plan
    assert [w.label for w in plan.waypoints if w.arrive] == ["standoff"]


def _gated(record) -> Plan:
    """The recorded plan with its standoff marked ``arrive`` — what
    ``Approach`` emits now."""
    plan = _plan(record)
    w = plan.waypoints[0]
    return Plan(plan.primitive, plan.side,
                (Waypoint(w.label, w.p, w.r, allow_via=w.allow_via,
                          arrive=True),),
                plan.steps, plan.notes, binding=plan.binding)


def test_a_gated_last_waypoint_is_tool_gated_before_the_settle(record, d1_arm):
    """The trajectory runner used to flush-and-gate an ``arrive`` waypoint
    only when another joint leg or a stroke followed it. The last leg of an
    Approach is followed by a SETTLE, so even a gated standoff went
    unmeasured on this transport."""
    executor = _executor(record, stuck=True)
    with executor:
        report = run(_gated(record), executor, kin=d1_arm)
    assert not report.completed, report.to_json()
    assert report.stop_reason == BARRIER_FAILED
    assert report.arrivals[-1].waypoint_label == "standoff"
    assert report.settle is None, "the settle ran before the gate"


def test_a_gated_standoff_that_is_reached_passes_the_tool_gate(record, d1_arm):
    executor = _executor(record, stuck=False)
    with executor:
        report = run(_gated(record), executor, kin=d1_arm)
    assert report.completed, report.to_json()
    assert [a.waypoint_label for a in report.arrivals] == ["standoff"]
    assert report.arrivals[0].tool_error_m < 0.005
