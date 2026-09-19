"""Every verb, planned against the real D1 URDF, IK and collision guard.

Not a mock anywhere in this file. ``d1_arm`` is the bundled ``d1.urdf`` driven
by the same ``solve_ee`` the teleop stack runs, with ``MotionGuard`` installed —
so a plan that comes back ``ok`` here is one whose every joint step the robot
would accept, and a refusal is the robot's own refusal.
"""

from __future__ import annotations

import numpy as np
import pytest

from manipulation_kit.primitives import (Approach, Carry, GoHome, Grasp,
                                         GripStep, Lift, Nudge, Place, Pour,
                                         Release, Retreat, SettleStep)
from manipulation_kit.primitives.types import (FRAME_STALE, GUARD_REJECT,
                                               LEARNED_POLICY_REQUIRED,
                                               NO_SUCH_OBJECT, PLAN_REASONS)

#: A block the LEFT arm can reach, measured by sweeping the bundled URDF.
REACHABLE = (0.38, 0.25, 0.05)
#: Out at the edge of the envelope and high: the IK runs out of arm.
UNREACHABLE = (0.52, 0.25, 0.45)
#: Close in front of the sternum: reachable in the abstract, refused by the
#: torso keep-out, which is the interesting refusal.
AGAINST_THE_BODY = (0.20, 0.05, 0.20)


def test_a_grasp_of_a_reachable_block_plans_a_continuous_joint_path(d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    assert plan.ok, str(plan)
    steps = plan.joint_steps()
    assert steps, "a plan with no joint steps did not move the arm"
    # continuity: no step exceeds the kit's own per-tick joint clamp
    from manipulation_kit.arms import safety
    q = world.arm("left").joints
    for step in steps:
        assert np.max(np.abs(step.q - q)) <= safety.MAX_JOINT_STEP_RAD + 1e-9
        q = step.q
    # ... and the last one puts the tool on the block
    from manipulation_kit.primitives.approach import tool_from_link7
    d1_arm.set_joints("left", steps[-1].q)
    tool = tool_from_link7(*d1_arm.ee_pose("left"))[0]
    assert np.linalg.norm(tool - np.array(REACHABLE)) < 0.01


def test_a_grasp_opens_before_it_moves_and_closes_only_at_the_object(d1_arm, observe):
    """An open-on-arrival stroke sweeps the pads through whatever is beside it."""
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Grasp(object="red_block", side="left", grip="firm").plan(world, d1_arm)
    grips = [s for s in plan.steps if isinstance(s, GripStep)]
    assert [g.closedness for g in grips] == [0.0, 1.0]
    assert all(g.grip == "firm" for g in grips)
    assert isinstance(plan.steps[0], GripStep)          # open first
    assert isinstance(plan.steps[-1], SettleStep)       # settle before measuring
    assert isinstance(plan.steps[-2], GripStep)


def test_an_out_of_reach_block_is_refused_with_a_residual(d1_arm, observe):
    world = observe(d1_arm, block_p=UNREACHABLE)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    assert not plan.ok
    assert plan.reason in PLAN_REASONS
    assert plan.waypoint_index >= 0 and plan.waypoint_label
    assert plan.residual_m > 0.0
    assert "red_block" not in str(plan) or plan.residual_m > 0


def test_a_pose_inside_the_torso_keepout_is_refused_by_the_guard(d1_arm, observe):
    world = observe(d1_arm, block_p=AGAINST_THE_BODY)
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    assert not plan.ok
    assert plan.reason == GUARD_REJECT, str(plan)
    assert "body" in plan.detail


def test_the_guard_refusal_disappears_when_the_guard_does(d1_arm, observe):
    """Proves the refusal above IS the guard and not an incidental IK failure —
    the same pose with no guard installed plans."""
    from manipulation_kit.arms import get_arm_kinematics
    unguarded = get_arm_kinematics("d1/arm", guard=None, find_ready=False,
                                   quiet=True)
    world = observe(unguarded, block_p=AGAINST_THE_BODY)
    guarded = Grasp(object="red_block", side="left").plan(
        observe(d1_arm, block_p=AGAINST_THE_BODY), d1_arm)
    free = Grasp(object="red_block", side="left").plan(world, unguarded)
    assert guarded.reason == GUARD_REJECT
    assert free.ok or free.reason != GUARD_REJECT


def test_planning_leaves_the_kinematic_model_exactly_where_it_was(d1_arm, observe):
    """A plan that moved the mirror would make the NEXT plan depend on it."""
    before = {s: d1_arm.joints(s).copy() for s in ("left", "right")}
    world = observe(d1_arm, block_p=REACHABLE)
    for verb in (Grasp(object="red_block", side="left"),
                 Grasp(object="red_block", side="left", approach="front"),
                 Grasp(object="red_block", side="right"),
                 Nudge(side="left", dz=0.03)):
        verb.plan(world, d1_arm)
        for side, q in before.items():
            assert np.allclose(d1_arm.joints(side), q), f"{verb.name()} moved {side}"


def test_a_plan_is_reproducible(d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE)
    verb = Grasp(object="red_block", side="left")
    first = verb.plan(world, d1_arm)
    second = verb.plan(world, d1_arm)
    assert len(first.joint_steps()) == len(second.joint_steps())
    for a, b in zip(first.joint_steps(), second.joint_steps()):
        assert np.allclose(a.q, b.q)


def test_naming_something_that_is_not_there_says_what_is(d1_arm, observe):
    world = observe(d1_arm)
    plan = Grasp(object="blue_sphere", side="left").plan(world, d1_arm)
    assert not plan.ok and plan.reason == NO_SUCH_OBJECT
    assert "red_block" in str(plan.unmet[0])


def test_an_object_wider_than_the_driven_jaws_is_refused_before_any_ik(d1_arm, observe):
    """51.96 mm is what the gripper on the robot opens to, not the 64 mm the
    mechanism could reach — planning against the mechanism is how a 48 mm
    object gets 2 mm of clearance instead of 8."""
    world = observe(d1_arm, block_p=REACHABLE, block_size=(0.09, 0.08, 0.05))
    verb = Grasp(object="red_block", side="left")
    codes = {u.code for u in verb.preconditions(world)}
    assert "object_too_wide" in codes
    assert not verb.plan(world, d1_arm).ok


def test_a_stale_frame_refuses_the_plan_with_frame_stale(d1_arm, observe):
    from scipy.spatial.transform import Rotation as R

    from manipulation_kit.world import Frame, FrameGraph, ObjectView
    graph = FrameGraph.of([Frame("table", "base", p=[0.3, 0.2, 0.0],
                                 r=R.identity(), stamp=0.0, max_age_s=1.0)],
                          now=60.0)
    world = observe(d1_arm, frames=graph)
    world = world.with_(objects=[ObjectView("red_block", p=(0.08, 0.05, 0.05),
                                            size=(0.05, 0.04, 0.05),
                                            frame_id="table")]
                        + [o for o in world.objects if o.name != "red_block"])
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    assert not plan.ok and plan.reason == FRAME_STALE


def test_side_auto_chooses_the_near_hand_and_says_so(d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Approach(object="red_block").plan(world, d1_arm)
    assert plan.ok, str(plan)
    assert plan.side == "left"           # the block is at +y
    assert any("automatically" in n for n in plan.notes)


def test_go_home_ramps_in_joint_space_with_no_ik(d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE)
    reach = Grasp(object="red_block", side="left").plan(world, d1_arm)
    d1_arm.set_joints("left", reach.joint_steps()[-1].q)
    moved = observe(d1_arm, block_p=REACHABLE)
    plan = GoHome().plan(moved, d1_arm)
    assert plan.ok, str(plan)
    steps = plan.joint_steps()
    assert steps
    assert np.allclose(steps[-1].q, d1_arm.home(steps[-1].side), atol=1e-6)


def test_lift_carry_place_release_chain_through_a_held_object(d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE)
    grasp = Grasp(object="red_block", side="left").plan(world, d1_arm)
    assert grasp.ok
    d1_arm.set_joints("left", grasp.joint_steps()[-1].q)
    held = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                   held={"left": "red_block"})
    for verb in (Lift(object="red_block", side="left"),
                 Carry(object="red_block", to="box", side="left"),
                 Place(object="red_block", to="box", side="left"),
                 Release(side="left"),
                 Retreat(side="left")):
        plan = verb.plan(held, d1_arm)
        assert plan.ok, f"{verb.name()}: {plan}"


def test_lift_refuses_when_nothing_is_held(d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Lift(object="red_block", side="left").plan(world, d1_arm)
    assert not plan.ok
    assert any(u.code == "not_holding" for u in plan.unmet)


def test_place_refuses_a_destination_that_is_not_a_container_or_surface(d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    plan = Place(object="red_block", to="red_block", side="left").plan(world, d1_arm)
    assert not plan.ok


def test_pour_checks_its_preconditions_and_then_refuses_to_plan(d1_arm, observe):
    """The contract, not a stub: the kit owns the preconditions and the
    verifier; the motion belongs to the policy."""
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    plan = Pour(source="red_block", target="box", side="left").plan(world, d1_arm)
    assert not plan.ok
    assert plan.reason == LEARNED_POLICY_REQUIRED
    assert "act:pourwithsmallpotjp" in plan.detail
    # and a precondition failure still beats it to the answer
    empty = observe(d1_arm, block_p=REACHABLE)
    assert Pour(source="red_block", target="box",
                side="left").plan(empty, d1_arm).reason != LEARNED_POLICY_REQUIRED


def test_pour_is_marked_as_learned(d1_arm, observe):
    from manipulation_kit.primitives import LearnedPrimitive
    assert isinstance(Pour(source="a", target="b"), LearnedPrimitive)


def test_every_verb_reports_a_reason_from_the_published_vocabulary(d1_arm, observe):
    """No verb may invent a refusal string a consumer cannot switch on."""
    world = observe(d1_arm, block_p=UNREACHABLE)
    verbs = [Approach(object="red_block"), Grasp(object="red_block"),
             Lift(object="nothing"), Carry(object="nothing", to="box"),
             Place(object="nothing", to="box"), Release(side="left"),
             Nudge(side="left"), Retreat(side="left", distance_m=99.0),
             GoHome(side="nowhere"), Pour(source="x", target="box")]
    for verb in verbs:
        result = verb.plan(world, d1_arm)
        if not result.ok:
            assert result.reason in PLAN_REASONS, f"{verb.name()}: {result.reason}"


# --------------------------------------------------------------------------- #
# the pads reach 29 mm past the tool point, and the table does not move
# --------------------------------------------------------------------------- #

def test_a_top_down_grasp_keeps_the_pad_tips_off_the_table(d1_arm, observe):
    """MEASURED, 2026-09-19 (blocks-eval, ten attempts, ten failures).

    The tool point is the pad CENTRE and the pads reach ``TIP_BELOW_TOOL_M``
    = 29 mm past it. Descending to a 40 mm cube's CENTRE therefore asked for
    the finger tips 9 mm BELOW the wagon top. The fingers jammed on the table,
    the arm stopped 17 mm high and 19 mm to the side — still 2.5 deg from the
    commanded posture after two seconds of holding it, while the same arm
    tracks a free-air posture to 0.00 deg in 0.7 s — and the jaws closed
    beside the block every time.
    """
    from manipulation_kit.primitives import approach as ap

    block = (0.38, 0.25, 0.05)                  # 50 x 40 x 50 mm, so it stands
    world = observe(d1_arm, block_p=block)      # on a surface at z = 0.025
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    assert plan.ok, str(plan)
    grasp = plan.waypoints[-1]
    assert grasp.label == "grasp"
    bottom = block[2] - 0.05 / 2
    assert grasp.p[2] == pytest.approx(
        bottom + ap.TIP_BELOW_TOOL_M + ap.SUPPORT_CLEARANCE_M)
    # over the object, not beside it, and still inside its height
    assert np.allclose(grasp.p[:2], block[:2])
    assert grasp.p[2] < block[2] + 0.05 / 2
    # the standoff is measured from the RAISED point, not the old one
    assert plan.waypoints[0].p[2] == pytest.approx(grasp.p[2] + 0.08)
    assert any("above the object's centre" in note for note in plan.notes)


def test_an_object_tall_enough_is_grasped_at_its_centre_as_before(d1_arm, observe):
    """The clearance is a floor, not an offset: nothing that already cleared
    the table moves."""
    from manipulation_kit.primitives import approach as ap

    from manipulation_kit.world import FrameGraph, ObjectView

    frames = FrameGraph()
    tall = ObjectView("tall_block", p=(0.38, 0.25, 0.12), size=(0.05, 0.04, 0.16))
    assert not ap.grasp_point(tall, "top_down", frames)[1]
    assert np.allclose(ap.grasp_point(tall, "top_down", frames)[0], tall.p)


def test_a_flat_object_is_refused_rather_than_grasped_over(d1_arm, observe):
    """When the lowest legal tool point is above the object's top face the
    pads would close on air with the tips still down. Say so."""
    world = observe(d1_arm, block_p=(0.38, 0.25, 0.006),
                    block_size=(0.05, 0.04, 0.012))
    codes = {u.code for u in Grasp(object="red_block", side="left")
             .preconditions(world)}
    assert "object_too_flat" in codes
    assert not Grasp(object="red_block", side="left").plan(world, d1_arm).ok
    # ...and coming in from the side is not refused for that reason
    side_on = {u.code for u in Grasp(object="red_block", side="left",
                                     approach="front").preconditions(world)}
    assert "object_too_flat" not in side_on


def test_a_horizontal_approach_still_aims_at_the_object_centre(d1_arm, observe):
    """The clamp is about what the object STANDS on, which only the top-down
    descent drives into."""
    from manipulation_kit.primitives import approach as ap

    from manipulation_kit.world import FrameGraph, ObjectView

    frames = FrameGraph()
    low = ObjectView("low_block", p=(0.38, 0.25, 0.02), size=(0.05, 0.04, 0.04))
    for name in ("front", "side_left", "side_right"):
        point, raised = ap.grasp_point(low, name, frames)
        assert not raised and np.allclose(point, low.p)
    assert ap.grasp_point(low, "top_down", frames)[1]
