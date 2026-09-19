"""The whole sequence, on a fake robot, with every step measured.

``KinematicExecutor`` mirrors — it does not simulate. That is the right
fidelity for this: it proves the PLANS compose (the grasp leaves the hand where
the lift expects it, the carry leaves the block where the place expects it) and
that every verifier fires on the world the previous verb produced. Whether the
physical grasp holds is not a thing kinematics can tell you, and this test does
not pretend otherwise.
"""

from __future__ import annotations

from manipulation_kit.primitives import verbs
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


def _recorder(world, *, arrives: bool = True):
    """A RecordingExecutor whose reported state is the world's measured one.

    ``arrives`` is the barrier switch. A recorder moves nothing, so by default
    it cannot claim an arrival and ``run`` stops at the first stroke — which
    is the behaviour ``test_a_recorder_cannot_close_the_jaws...`` pins. The
    tests that are about step ROUTING rather than about the barrier say so.
    """
    from manipulation_kit.executor import RawState
    executor = RecordingExecutor(RawState(
        joints={s: world.arm(s).joints for s in ("left", "right")},
        grippers={s: world.gripper(s).closedness for s in ("left", "right")}))
    executor.pretend_arrived = arrives
    return executor


def test_run_refuses_when_the_executor_cannot_say_where_an_arm_is(d1_arm, observe):
    """The untouched arm is commanded in EVERY dual-arm vector. Defaulting it
    to zero would command a T-pose through the torso."""
    import pytest
    from manipulation_kit.executor import RawState
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    blind = RecordingExecutor(RawState(joints={"left": world.arm("left").joints}))
    blind.pretend_arrived = True
    report = run(plan, blind)
    assert not report.completed
    # The BINDING catches it first now, and says the better thing: the posture
    # this plan was checked against cannot be confirmed. The runner's own
    # "nothing to command the untouched arm at" is the belt to that's braces
    # and still fires for an unbound plan.
    assert report.stop_reason == "stale_binding"
    assert "no joints for the right arm" in report.error
    assert not blind.sent, "nothing may be commanded on a half-known robot"

    from manipulation_kit.executor import run_steps
    unbound = run_steps(plan, blind)
    assert unbound.stop_reason == "transport_error"
    assert "no joints for" in unbound.error
    assert pytest


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


def test_a_plan_that_never_mentions_the_jaws_carries_the_COMMAND_not_the_measurement(
        d1_arm, observe):
    """F9. A Lift sends joint knots only, and every one of them still has to
    put a gripper number on the wire. That number is the command the hand is
    already under — 1.0 — and never where the jaws stopped, 0.41: commanding a
    force-limited gripper to the position it has reached tells it to stop
    squeezing, and the held object is on the table before the arm has moved.
    MEASURED on the Isaac harness, three trials out of three, 2026-09-19."""
    from manipulation_kit.executor import RawState, run_steps

    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 0.41},
                    held={"left": "red_block"})
    plan = Lift(object="red_block", side="left", height_m=0.10).plan(world, d1_arm)
    assert getattr(plan, "ok", False)
    assert not any(isinstance(s, GripStep) for s in plan.steps)

    measured = {s: np.array(d1_arm.joints(s), dtype=float) for s in ("left", "right")}

    stalled = RecordingExecutor(RawState(
        joints=measured, grippers={"left": 0.41, "right": 0.0},
        commanded_grippers={"left": 1.0, "right": 0.0}))
    stalled.pretend_arrived = True
    run_steps(plan, stalled)
    assert stalled.sent
    assert all(v[GRIPPER_INDEX["left"]] == 1.0 for _t, v in stalled.sent)

    # and an executor that cannot say what it commanded still gets a number:
    # the measurement, which is the honest fallback and is documented as one
    blind = RecordingExecutor(RawState(
        joints=measured, grippers={"left": 0.41, "right": 0.0}))
    blind.pretend_arrived = True
    run_steps(plan, blind)
    assert all(v[GRIPPER_INDEX["left"]] == 0.41 for _t, v in blind.sent)


def test_the_gripper_closedness_travels_in_the_wire_vector(d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    executor = _recorder(world)
    run(plan, executor)
    assert [(s, c) for s, c, _g in executor.grips] == [("left", 0.0), ("left", 1.0)]
    assert executor.settles == [verbs.SETTLE_S]
    # after the opening stroke every commanded vector carries the open jaws
    assert all(v[GRIPPER_INDEX["left"]] == 0.0 for _t, v in executor.sent)


def test_a_recorder_cannot_close_the_jaws_because_it_never_arrives(
        d1_arm, observe):
    """R7, as a runner-level rule. An executor that cannot MEASURE an arrival
    does not get to run a gripper stroke: the stroke's whole meaning is "the
    tool is on the object now". Before this, ``set_gripper`` was called the
    instant the previous joint step returned, whatever the arm was doing."""
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    executor = _recorder(world, arrives=False)
    report = run(plan, executor)
    assert not report.completed
    assert report.stop_reason == "barrier_failed"
    # the OPENING stroke is the first step and has no preceding joint step, so
    # what stops the run is its completion, not an arrival
    assert report.strokes and not report.strokes[0].settled
    assert len(executor.grips) == 1, "no second stroke after a failed barrier"


def test_a_plan_is_refused_against_a_posture_it_was_not_checked_in(
        d1_arm, observe):
    """R8. Between planning and running there is a model turn; the arm can
    move. The plan carries the posture it was checked in and the runner
    compares it with what the executor measures."""
    import numpy as np
    from manipulation_kit.executor import RawState

    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    assert plan.ok and plan.binding is not None

    drifted = dict({s: np.array(world.arm(s).joints, dtype=float)
                    for s in ("left", "right")})
    drifted["right"] = drifted["right"] + np.radians(12.0)
    executor = RecordingExecutor(RawState(
        joints=drifted, grippers={"left": 0.0, "right": 0.0}))
    executor.pretend_arrived = True
    report = run(plan, executor)
    assert not report.completed
    assert report.stop_reason == "stale_binding"
    assert "right arm has moved" in report.error
    assert not executor.sent, "nothing may be sent against a stale plan"


def test_an_unbound_plan_is_refused_unless_the_caller_owns_it(d1_arm, observe):
    from manipulation_kit.primitives.types import Plan

    world = observe(d1_arm, block_p=REACHABLE)
    hand_made = Plan("hand_made", "left", (), ())
    executor = _recorder(world)
    assert run(hand_made, executor).stop_reason == "not_bound"
    assert run(hand_made, executor, allow_unbound=True).completed


def test_an_unguarded_plan_is_not_an_executable_plan(d1_arm, observe):
    """R, section 5: "A guard-disabled kinematic model can still produce
    Plan.ok=True. Keep analytical unguarded planning possible, but distinguish
    it from an executable checked plan and require guard provenance at the
    hardware boundary."."""
    from manipulation_kit.executor import run

    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    assert plan.binding.guarded

    class Unguarded:
        """The kit's own model with the guard gate reporting absent."""

        class gate:
            installed = False

        def __getattr__(self, name):
            return getattr(d1_arm, name)

    unguarded = Grasp(object="red_block", side="left").plan(world, Unguarded())
    assert unguarded.ok, "analytical planning must stay possible"
    assert not unguarded.binding.guarded

    executor = _recorder(world)
    assert run(unguarded, executor).stop_reason == "unguarded_plan"
    assert not executor.sent
    assert run(unguarded, executor, allow_unguarded=True).completed


def test_a_held_hand_with_no_command_history_refuses_rather_than_releasing(
        d1_arm, observe):
    """R, section 3: "the fallback still re-commands a measured stalled
    aperture ... For a transport that uses all 16 entries this remains unsafe
    when command history is unknown."

    F9 one step further: the documented fallback (send the measurement when
    nobody retained the command) is only honest for an EMPTY hand. With
    something held and no retained command, commanding the measured aperture
    tells a force-limited gripper to stop squeezing, and the carried object is
    on the table before the arm has moved. There is no safe number, so the
    answer is to refuse and say which field would fix it."""
    from manipulation_kit.executor import RawState, run_steps

    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 0.41},
                    held={"left": "red_block"})
    plan = Lift(object="red_block", side="left", height_m=0.10).plan(world, d1_arm)
    assert getattr(plan, "ok", False)
    measured = {s: np.array(d1_arm.joints(s), dtype=float) for s in ("left", "right")}

    blind = RecordingExecutor(RawState(
        joints=measured, grippers={"left": 0.41, "right": 0.0},
        holding={"left": True, "right": False}))
    blind.pretend_arrived = True
    report = run_steps(plan, blind)
    assert not report.completed
    assert report.stop_reason == "transport_error"
    assert "commanded_grippers" in report.error
    assert not blind.sent

    # ...and with the command retained it runs, carrying the COMMAND forward
    knows = RecordingExecutor(RawState(
        joints=measured, grippers={"left": 0.41, "right": 0.0},
        holding={"left": True, "right": False},
        commanded_grippers={"left": 1.0, "right": 0.0}))
    knows.pretend_arrived = True
    assert run_steps(plan, knows).completed
    assert all(v[GRIPPER_INDEX["left"]] == 1.0 for _t, v in knows.sent)
