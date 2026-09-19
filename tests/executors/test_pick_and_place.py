"""The whole sequence, on a fake robot, with every step measured.

``KinematicExecutor`` mirrors — it does not simulate. That is the right
fidelity for this: it proves the PLANS compose (the grasp leaves the hand where
the lift expects it, the carry leaves the block where the place expects it) and
that every verifier fires on the world the previous verb produced. Whether the
physical grasp holds is not a thing kinematics can tell you, and this test does
not pretend otherwise.
"""

from __future__ import annotations

import numpy as np

from manipulation_kit.executor import (GRIPPER_INDEX, JOINT_SLICE,
                                       KinematicExecutor, RecordingExecutor,
                                       WIRE_DIM, run, wire)
from manipulation_kit.primitives import (Carry, GoHome, GripStep, Grasp,
                                         JointStep, Lift, Place, Retreat,
                                         Verdict)

REACHABLE = (0.38, 0.25, 0.05)


class Mirror:
    """A scene where the block follows whichever hand is closed on it."""

    def __init__(self, kin, observe, block_p=REACHABLE):
        self.kin = kin
        self.observe = observe
        self.executor = KinematicExecutor(kin)
        self.executor.next_object["left"] = "red_block"
        self.block = np.array(block_p, dtype=float)

    def world(self):
        return self.observe(self.kin, block_p=tuple(self.block),
                            closed=dict(self.executor.grippers),
                            held={s: self.executor.held.get(s)
                                  for s in ("left", "right")})

    def run(self, plan) -> int:
        from manipulation_kit.primitives.approach import tool_from_link7
        state = self.executor.state()
        joints = {s: np.array(q, dtype=float) for s, q in state.joints.items()}
        grippers = dict(state.grippers)
        sent = 0
        for step in plan.steps:
            if isinstance(step, JointStep):
                joints[step.side] = np.asarray(step.q, dtype=float)
                self.executor.send_joints(wire(joints, grippers), t=sent * 0.02)
            elif isinstance(step, GripStep):
                self.executor.set_gripper(step.side, step.closedness, grip=step.grip)
                grippers[step.side] = float(step.closedness)
            sent += 1
            if self.executor.held.get("left") == "red_block":
                self.block = tool_from_link7(*self.kin.ee_pose("left"))[0].copy()
        return sent


def test_a_whole_pick_and_place_is_planned_run_and_measured(d1_arm, observe):
    mirror = Mirror(d1_arm, observe)
    verbs = [Grasp(object="red_block", side="left"),
             Lift(object="red_block", side="left", height_m=0.10),
             Carry(object="red_block", to="box", side="left"),
             Place(object="red_block", to="box", side="left"),
             Retreat(side="left", distance_m=0.10),
             GoHome()]
    for verb in verbs:
        before = mirror.world()
        plan = verb.plan(before, d1_arm)
        assert plan.ok, f"{verb.name()}: {plan}"
        mirror.run(plan)
        report = verb.verifier(before)(mirror.world())
        assert report.verdict == Verdict.TRUE, \
            f"{verb.name()} measured {report.verdict}: {report.reason}"
    box = mirror.world().find("box")
    assert box.contains(mirror.block, mirror.world().frames, pad_m=0.01)


def test_the_task_verifier_is_false_until_the_place_actually_happens(d1_arm, observe):
    """A per-primitive TRUE is not a task TRUE, and the loop must not confuse
    them — this is the distinction the trace's claimed/measured columns keep."""
    mirror = Mirror(d1_arm, observe)
    goal = Place(object="red_block", to="box", side="left")
    start = mirror.world()
    for verb in (Grasp(object="red_block", side="left"),
                 Lift(object="red_block", side="left"),
                 Carry(object="red_block", to="box", side="left")):
        plan = verb.plan(mirror.world(), d1_arm)
        assert plan.ok
        mirror.run(plan)
        assert goal.verifier(start)(mirror.world()).verdict == Verdict.FALSE
    plan = goal.plan(mirror.world(), d1_arm)
    assert plan.ok
    mirror.run(plan)
    assert goal.verifier(start)(mirror.world()).verdict == Verdict.TRUE


def _recorder(world):
    """A RecordingExecutor whose reported state is the world's measured one."""
    from manipulation_kit.executor import RawState
    return RecordingExecutor(RawState(
        joints={s: world.arm(s).joints for s in ("left", "right")},
        grippers={s: world.gripper(s).closedness for s in ("left", "right")}))


def test_run_refuses_when_the_executor_cannot_say_where_an_arm_is(d1_arm, observe):
    """The untouched arm is commanded in EVERY dual-arm vector. Defaulting it
    to zero would command a T-pose through the torso."""
    import pytest
    from manipulation_kit.executor import RawState
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    blind = RecordingExecutor(RawState(joints={"left": world.arm("left").joints}))
    with pytest.raises(ValueError, match="no joints for"):
        run(plan, blind)


def test_run_sends_one_dual_arm_vector_per_joint_step(d1_arm, observe):
    """A dual-arm robot commanded one arm at a time is two robots disagreeing
    about the posture the guard just approved."""
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    executor = _recorder(world)
    report = run(plan, executor)
    assert report.completed
    assert len(executor.sent) == len(plan.joint_steps())
    for _t, vector in executor.sent:
        assert vector.shape == (WIRE_DIM,)
        # the idle arm is carried along at its current joints, not left at zero
        assert np.allclose(vector[JOINT_SLICE["right"]], world.arm("right").joints)


def test_run_paces_the_plan_at_the_commanded_rate(d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Lift(object="red_block", side="left").plan(
        observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                held={"left": "red_block"}), d1_arm)
    assert plan.ok
    executor = _recorder(world)
    run(plan, executor, hz=50.0)
    gaps = executor.cadence()
    assert gaps.size
    assert np.allclose(gaps, 0.02, atol=1e-9)


def test_a_refused_plan_runs_nothing_and_says_why(d1_arm, observe):
    world = observe(d1_arm, block_p=(0.52, 0.25, 0.45))
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    assert not plan.ok
    executor = _recorder(world)
    report = run(plan, executor)
    assert not report.completed
    assert report.steps_sent == 0
    assert not executor.sent
    assert plan.reason in report.error


def test_the_gripper_closedness_travels_in_the_wire_vector(d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    executor = _recorder(world)
    run(plan, executor)
    assert [(s, c) for s, c, _g in executor.grips] == [("left", 0.0), ("left", 1.0)]
    assert executor.settles == [1.0]
    # after the opening stroke every commanded vector carries the open jaws
    assert all(v[GRIPPER_INDEX["left"]] == 0.0 for _t, v in executor.sent)
