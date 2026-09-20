"""ONE contract suite, run against every executor in the repository.

The review's first and largest complaint about the test matrix: "The pick/place
tests and demo each implement a private walker instead of proving the common
runner." A private walker proves that ITS author understood the order of a
plan's steps; it proves nothing about the transport a consumer will actually
use, and it is where "swap the executor, the loop does not change" quietly
stopped being true.

So: the same cases, parametrised over the three executors this repository
ships — the recorder, the kinematic mirror and the firmware transport over a
fake client. Each case is one sentence of the Executor contract.

The FirmwareExecutor here speaks to ``tests/executors/test_firmware.py``'s fake
client, which follows the commands it is given: its feedback moves when a
trajectory completes or a joint command arrives, and its gripper takes a
settable number of polls to finish a stroke. A fake whose state never changes
cannot tell a transport that arrives from one that stops short.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

from manipulation_kit.executor import (ARM_DOF, SIDES, KinematicExecutor, RawState,
                                       RecordingExecutor, run)
from manipulation_kit.primitives import Grasp, Lift
from manipulation_kit.primitives.types import Plan

REACHABLE = (0.38, 0.25, 0.05)

def _load_fakes():
    """The protocol-faithful fake client, from the file that documents it.

    ``tests/`` is deliberately not a package (two areas would collide on a
    basename), so it is loaded by path — and registered in ``sys.modules``
    first, because ``@dataclass`` looks its own module up there.
    """
    spec = importlib.util.spec_from_file_location(
        "_fw_fakes", Path(__file__).with_name("test_firmware.py"))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


_fakes = _load_fakes()
FakeClient, FakeArmState = _fakes.FakeClient, _fakes.FakeArmState


def _recording(world, d1_arm):
    executor = RecordingExecutor(RawState(
        joints={s: world.arm(s).joints for s in SIDES},
        grippers={s: world.gripper(s).closedness for s in SIDES}))
    executor.pretend_arrived = True
    return executor


def _kinematic(world, d1_arm):
    return KinematicExecutor(d1_arm)


def _firmware(world, d1_arm):
    from manipulation_kit.executors.firmware import FirmwareExecutor
    client = FakeClient()
    for side, wire_side in (("left", "a"), ("right", "b")):
        client.arms[wire_side] = FakeArmState(
            feedback_joints=tuple(np.degrees(world.arm(side).joints)),
            command_joints=tuple(np.degrees(world.arm(side).joints)))
    clock = {"t": 0.0}

    def sleep(seconds: float) -> None:
        clock["t"] += float(seconds)

    robot = FirmwareExecutor(client, heartbeat=False, sleep=sleep,
                             clock=lambda: clock["t"])
    robot.acquire()
    return robot


EXECUTORS = {"recording": _recording, "kinematic": _kinematic,
             "firmware": _firmware}


@pytest.fixture(params=sorted(EXECUTORS), ids=sorted(EXECUTORS))
def executor_for(request):
    return EXECUTORS[request.param], request.param


# --------------------------------------------------------------------------- #
# the contract, one case per sentence
# --------------------------------------------------------------------------- #

def test_state_reports_both_arms_in_radians(executor_for, d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE)
    build, _name = executor_for
    state = build(world, d1_arm).state()
    for side in SIDES:
        assert side in state.joints
        assert np.asarray(state.joints[side]).shape == (ARM_DOF,)
        assert np.allclose(state.joints[side], world.arm(side).joints, atol=1e-6)


def test_a_refused_plan_reaches_the_robot_not_at_all(executor_for, d1_arm,
                                                     observe):
    world = observe(d1_arm, block_p=(0.52, 0.25, 0.45))
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    assert not plan.ok
    build, _name = executor_for
    report = run(plan, build(world, d1_arm))
    assert not report.completed and report.stop_reason == "refused_plan"
    assert report.steps_sent == 0


def test_an_unbound_plan_is_refused_by_every_transport(executor_for, d1_arm,
                                                       observe):
    world = observe(d1_arm, block_p=REACHABLE)
    build, _name = executor_for
    report = run(Plan("hand_made", "left", (), ()), build(world, d1_arm))
    assert report.stop_reason == "not_bound"


def test_a_plan_checked_in_another_posture_is_refused(executor_for, d1_arm,
                                                      observe):
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    build, name = executor_for
    robot = build(world, d1_arm)
    if name == "kinematic":
        d1_arm.set_joints("right", d1_arm.joints("right") + np.radians(12.0))
    elif name == "recording":
        drifted = dict(robot.state().joints)
        drifted["right"] = drifted["right"] + np.radians(12.0)
        robot._state = RawState(joints=drifted, grippers=robot.state().grippers)
    else:
        robot.client.arms["b"] = FakeArmState(
            feedback_joints=tuple(np.degrees(world.arm("right").joints)
                                  + 12.0))
    report = run(plan, robot)
    assert report.stop_reason == "stale_binding", report.error
    assert report.steps_sent == 0


def test_every_joint_step_of_the_plan_is_commanded(executor_for, d1_arm,
                                                   observe):
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    plan = Lift(object="red_block", side="left", height_m=0.10).plan(world, d1_arm)
    assert plan.ok
    build, name = executor_for
    robot = build(world, d1_arm)
    report = run(plan, robot)
    assert report.completed, report.error
    if name == "firmware":
        uploaded = sum(len(b["waypoints"]) - 1 for b in
                       robot.client.posts("/v1/arm/trajectory/start"))
        assert uploaded == len(plan.joint_steps())
    else:
        assert len(robot.sent) == len(plan.joint_steps())


def test_the_final_commanded_posture_is_the_plans_endpoint(executor_for,
                                                           d1_arm, observe):
    """R4, as a contract rather than as a firmware detail: no transport may
    end anywhere but where the plan said."""
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    plan = Lift(object="red_block", side="left", height_m=0.10).plan(world, d1_arm)
    build, name = executor_for
    robot = build(world, d1_arm)
    assert run(plan, robot).completed
    want = np.degrees(np.asarray(plan.final_joints()["left"]))
    if name == "firmware":
        got = np.asarray(robot.client.posts(
            "/v1/arm/trajectory/start")[-1]["waypoints"][-1]["a"])
    else:
        from manipulation_kit.executor import JOINT_SLICE
        got = np.degrees(robot.sent[-1][1][JOINT_SLICE["left"]])
    assert np.allclose(got, want, atol=1e-6), (
        f"{name} ended {np.max(np.abs(got - want)):.4f} deg from the endpoint")


def test_a_dual_arm_vector_never_leaves_the_other_arm_at_zero(executor_for,
                                                              d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    plan = Lift(object="red_block", side="left").plan(world, d1_arm)
    build, name = executor_for
    robot = build(world, d1_arm)
    run(plan, robot)
    right = np.asarray(world.arm("right").joints)
    if name == "firmware":
        for body in robot.client.posts("/v1/arm/trajectory/start"):
            for point in body["waypoints"]:
                assert np.allclose(np.radians(point["b"]), right, atol=1e-6)
    else:
        from manipulation_kit.executor import JOINT_SLICE
        for _t, vector in robot.sent:
            assert np.allclose(vector[JOINT_SLICE["right"]], right, atol=1e-6)


def test_the_gripper_order_of_a_grasp_survives_the_transport(executor_for,
                                                             d1_arm, observe):
    """open, travel, close — and the close is the LAST thing."""
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    build, name = executor_for
    robot = build(world, d1_arm)
    assert run(plan, robot).completed
    if name == "firmware":
        strokes = [b["closedness"] for m, p, b in robot.client.calls
                   if p == "/v1/gripper/a/set"]
    else:
        strokes = [c for _s, c, _g in robot.grips]
    assert strokes == [0.0, 1.0]


def test_a_settle_that_fails_stops_the_run_everywhere(executor_for, d1_arm,
                                                      observe):
    from manipulation_kit.executor import SettleReport
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    build, name = executor_for
    robot = build(world, d1_arm)
    if name == "firmware":
        robot.client.settle_after = 10_000
    else:
        robot.settle = lambda timeout_s: SettleReport(  # type: ignore[assignment]
            False, timeout_s, 12.0, "still moving")
    report = run(plan, robot)
    assert not report.completed
    assert report.stop_reason == "barrier_failed"


def test_an_unsupported_step_is_a_typed_outcome_not_a_silent_skip(
        executor_for, d1_arm, observe):
    """R, section 5: "Unsupported step types raise in generic execution but
    are silently ignored in firmware execution; unify this"."""
    import dataclasses
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    broken = dataclasses.replace(plan, steps=plan.steps + ("not a step",))
    build, _name = executor_for
    report = run(broken, build(world, d1_arm))
    assert not report.completed
    assert report.stop_reason == "transport_error"
    assert "not a plan step" in report.error


def test_a_plan_serialises_losslessly_and_replays_to_the_same_commands(
        d1_arm, observe):
    """R, section 5: "``Plan.to_json()`` stores counts rather than joint
    commands ... the trace is diagnostic prose, not a replayable record"."""
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    full = plan.to_json(full=True)
    assert len(full["steps"]) == len(plan.steps)
    joints = [s for s in full["steps"] if s["step"] == "joint"]
    assert len(joints) == len(plan.joint_steps())
    for step, rendered in zip(plan.joint_steps(), joints):
        assert np.allclose(step.q, rendered["q_rad"], atol=1e-6)
    assert full["final_joints_deg"]["left"] == [
        round(float(np.degrees(v)), 3) for v in plan.final_joints()["left"]]
    # the default form SAYS it is a summary rather than pretending otherwise
    assert "steps_elided" in plan.to_json()
