"""The executor state is typed, complete, and bound to the firmware it came from.

Redesign step 1 (design C.6): ``RawState`` is ``arms: {side: JointState}`` +
``hands: {side: HandState}``; a latched controller stops the run in the kit;
a plan records the firmware contract its observation came through; and no
environment variable reaches the tool revision a plan is bound to.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from manipulation_kit.executor import (ARM_DOF, SIDES, HandState, JointState,
                                       KinematicExecutor, RawState,
                                       RecordingExecutor, check_binding,
                                       read_lift, read_neck, run)
from manipulation_kit.primitives import Grasp
from manipulation_kit.primitives.orientation import tool_revision

REACHABLE = (0.38, 0.25, 0.05)
SRC = Path(__file__).resolve().parents[2] / "src"


# --------------------------------------------------------------------------- #
# the dataclass, and the one sanctioned shim
# --------------------------------------------------------------------------- #

def test_the_flat_accessors_are_views_of_arms_and_hands():
    state = RawState(
        arms={"left": JointState(q=np.ones(ARM_DOF), stationary=False),
              "right": JointState(q=np.zeros(ARM_DOF))},
        hands={"left": HandState(closedness=0.4, commanded=1.0, holding=True),
               "right": HandState()})
    assert np.allclose(state.joints["left"], 1.0)
    assert state.grippers == {"left": 0.4}          # unknown stays absent
    assert state.holding == {"left": True}
    assert state.commanded_grippers == {"left": 1.0}
    assert state.stationary is False
    assert state.vector().shape == (16,)


def test_the_transitional_flat_constructor_builds_the_typed_state():
    """d1-isaaclab's executor builds ``RawState(joints=..., grippers=...)``;
    that keeps working for one release, and lands in ``arms``/``hands``."""
    state = RawState(joints={s: np.zeros(ARM_DOF) for s in SIDES},
                     grippers={"left": 0.2, "right": 0.9},
                     commanded_grippers={"right": 1.0},
                     holding={"right": True}, stationary=True,
                     extra={"raw_grippers": {"left": 0.1}})
    assert state.arms["left"].mode == "unknown"
    assert state.hands["right"] == HandState(closedness=0.9, commanded=1.0,
                                             holding=True)
    assert state.extra == {"raw_grippers": {"left": 0.1}}
    with pytest.raises(TypeError):
        RawState(arms={}, joints={})


def test_a_joint_state_rejects_a_wrong_shape_and_an_unknown_mode():
    with pytest.raises(ValueError):
        JointState(q=np.zeros(6))
    assert JointState(q=np.zeros(ARM_DOF), mode="warp").mode == "unknown"
    assert JointState(q=np.zeros(ARM_DOF), mode="error").faulted
    assert JointState(q=np.zeros(ARM_DOF), mode="position", error_code=5).faulted
    assert not JointState(q=np.zeros(ARM_DOF), mode="position").faulted


def test_the_kinematic_mirror_says_what_it_knows_and_no_more(d1_arm):
    mirror = KinematicExecutor(d1_arm, open_gap_m=0.0605)
    state = mirror.state()
    for side in SIDES:
        arm, hand = state.arms[side], state.hands[side]
        assert arm.mode == "position" and not arm.faulted
        assert arm.torque_nm is None             # no load to measure
        assert hand.commanded == hand.closedness
        assert hand.open_gap_m == 0.0605
        assert hand.jaw_gap_m is None and hand.fault is None
    assert mirror.firmware_spec == "kinematic"
    assert read_neck(mirror) is None and read_lift(mirror) is None
    assert KinematicExecutor(d1_arm).state().hands["left"].open_gap_m is None


# --------------------------------------------------------------------------- #
# faults stop the run, in the kit
# --------------------------------------------------------------------------- #

def test_a_latched_controller_stops_the_generic_runner_before_any_byte(
        d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    assert plan.ok
    latched = RecordingExecutor(RawState(
        arms={s: JointState(q=world.arm(s).joints,
                            mode="error" if s == "right" else "position",
                            error_code=11 if s == "right" else 0)
              for s in SIDES},
        hands={s: HandState(closedness=0.0, commanded=0.0) for s in SIDES}))
    latched.pretend_arrived = True
    report = run(plan, latched)
    assert report.stop_reason == "controller_fault", report.error
    assert report.refusal is not None and report.refusal.reason == "controller_fault"
    assert report.steps_sent == 0 and not latched.sent and not latched.grips


def test_a_controller_that_latches_between_steps_stops_the_generic_runner(
        d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)

    class LatchesAfterTheFirstWaypoint(KinematicExecutor):
        def state(self):
            state = super().state()
            if len(self.sent) >= 3:
                arms = dict(state.arms)
                arms["left"] = JointState(q=arms["left"].q, mode="error",
                                          error_code=9)
                return RawState(arms, state.hands)
            return state

    robot = LatchesAfterTheFirstWaypoint(d1_arm)
    report = run(plan, robot, kin=d1_arm)
    assert report.stop_reason == "controller_fault", report.error
    assert 0 < report.steps_sent < len(plan.steps)
    assert not [g for g in robot.grips if g[1] == 1.0], (
        "the jaws closed after the controller latched")


# --------------------------------------------------------------------------- #
# the binding: tool revision and firmware spec
# --------------------------------------------------------------------------- #

def test_no_env_var_feeds_tool_revision():
    """``MKIT_DRIVEN_OPEN_GAP_M`` used to feed ``tool_revision()``, so a
    process with it set refused every plan a differently-configured process
    made. A robot's hand measurement is published by its executor now; the
    revision is a pure function of the hand description."""
    env = dict(os.environ, MKIT_DRIVEN_OPEN_GAP_M="0.0605",
               PYTHONPATH=str(SRC) + os.pathsep + os.environ.get("PYTHONPATH", ""))
    out = subprocess.run(
        [sys.executable, "-c",
         "from manipulation_kit.primitives.orientation import tool_revision;"
         "from manipulation_kit.hands.d1.parallel_gripper import description;"
         "print(tool_revision()); print(description.DRIVEN_OPEN_GAP_M)"],
        env=env, capture_output=True, text=True, check=True)
    revision, gap = out.stdout.strip().splitlines()
    assert revision == tool_revision()
    assert float(gap) == pytest.approx(0.05196)
    source = (SRC / "manipulation_kit" / "hands" / "d1" / "parallel_gripper"
              / "description.py").read_text(encoding="utf-8")
    assert "MKIT_DRIVEN_OPEN_GAP_M" not in source and "os.environ" not in source


class _SpecExecutor(KinematicExecutor):
    """A mirror that claims to drive a particular firmware contract."""

    def __init__(self, kin, spec):
        super().__init__(kin)
        self.firmware_spec = spec


def test_a_plan_bound_to_one_spec_is_refused_on_another(d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE).with_(firmware_spec="a" * 64)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    assert plan.ok and plan.binding.firmware_spec == "a" * 64
    assert plan.binding.to_json()["firmware_spec"] == "a" * 64
    assert check_binding(plan, _SpecExecutor(d1_arm, "a" * 64)) is None
    reason = check_binding(plan, _SpecExecutor(d1_arm, "b" * 64))
    assert reason is not None and "firmware client spec" in reason
    report = run(plan, _SpecExecutor(d1_arm, "b" * 64))
    assert report.stop_reason == "stale_binding" and report.steps_sent == 0
    # a dry-run's plan (bound to the mirror) is refused on hardware too
    mirror_plan = Grasp(object="red_block", side="left").plan(
        observe(d1_arm, block_p=REACHABLE).with_(firmware_spec="kinematic"),
        d1_arm)
    assert check_binding(mirror_plan, _SpecExecutor(d1_arm, "a" * 64)) is not None
    assert check_binding(mirror_plan, KinematicExecutor(d1_arm)) is None


def test_a_plan_whose_producer_named_no_spec_is_not_refused_for_it(d1_arm,
                                                                   observe):
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    assert plan.binding.firmware_spec == ""
    assert check_binding(plan, _SpecExecutor(d1_arm, "b" * 64)) is None


# --------------------------------------------------------------------------- #
# run() resolves its settings once and forwards all of them (L13)
# --------------------------------------------------------------------------- #

def test_run_forwards_its_settings_to_a_transports_own_run_plan(d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    seen = {}

    class Takes(KinematicExecutor):
        def run_plan(self, plan, *, hz=None, arrive_tol_rad=None,
                     arrive_timeout_s=None, stroke_timeout_s=None, gate=None):
            seen.update(hz=hz, gate=gate, stroke=stroke_timeout_s)
            from manipulation_kit.executor import RunReport
            return RunReport(plan.primitive, plan.side, True, 0)

    run(plan, Takes(d1_arm), hz=25.0, tool_tol_along_m=0.02,
        settle_timeout_s=0.7, stroke_timeout_s=9.0)
    assert seen["hz"] == 25.0 and seen["stroke"] == 9.0
    assert seen["gate"].tol_along_m == 0.02
    assert seen["gate"].settle_timeout_s == 0.7
