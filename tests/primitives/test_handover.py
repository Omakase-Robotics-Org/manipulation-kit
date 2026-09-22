"""``handover``: one plan over both arms (design C.10, redesign step 8).

Nothing here is a mock of the planner: the bundled URDF, the real IK and
guard, the kinematic mirror as the executor and the agent's own
``SceneSource`` as the observation — the left hand really grasps and lifts
the cube first, and the handover starts from that posture.
"""

from __future__ import annotations

import numpy as np
import pytest

from manipulation_kit.agent.policy import (LOOK_NOT_POSSIBLE, OperatorPolicy,
                                           PolicyState)
from manipulation_kit.agent.robot import SceneSource
from manipulation_kit.executor import (BARRIER_FAILED, HOLD_NOT_CONFIRMED,
                                       KinematicExecutor, run)
from manipulation_kit.primitives import (Approach, Grasp, GripStep, Handover,
                                         JointStep, Lift, Release, reach)
from manipulation_kit.primitives.offer import candidates_for, check
from manipulation_kit.primitives.schema import decode, tool_schemas
from manipulation_kit.primitives.types import (BAD_ARGUMENT, NOT_HOLDING,
                                               UNREACHABLE_HANDOVER)

SCENE = {"objects": [
    {"name": "table", "kind": "surface", "p": [0.40, 0.0, 0.0],
     "size": [0.90, 0.80, 0.02]},
    {"name": "cube", "kind": "object", "p": [0.40, 0.15, 0.03],
     "size": [0.04, 0.04, 0.04]}]}


def _robot(kin, *, receiver_closes_on="cube"):
    executor = KinematicExecutor(kin, open_gap_m=0.0605)
    executor.next_object["left"] = "cube"
    executor.next_object["right"] = receiver_closes_on
    return executor, SceneSource.from_scene(executor, kin, SCENE)


def _do(primitive, source, executor, kin):
    world = source.observe()
    plan = check(primitive, world, kin)
    assert plan.ok, str(plan)
    report = run(plan, executor, kin=kin)
    return world, plan, report, source.observe()


def _left_holds_it(kin, **kwargs):
    executor, source = _robot(kin, **kwargs)
    for verb in (Approach(object="cube", side="left"),
                 Grasp(object="cube", side="left", grip="firm"),
                 Lift(object="cube", side="left", height_m=0.12)):
        _w, _p, report, after = _do(verb, source, executor, kin)
        assert report.completed, report.error
    assert after.gripper("left").held_object == "cube"
    return executor, source, after


def test_a_handover_ends_with_the_other_hand_holding(d1_arm):
    executor, source, holding = _left_holds_it(d1_arm)
    world0, plan, report, world1 = _do(Handover(object="cube"), source,
                                       executor, d1_arm)
    assert report.completed, report.error
    # ONE plan, both arms: the giver meets and backs out, the receiver takes
    sides = {s.side for s in plan.steps if isinstance(s, JointStep)}
    assert sides == {"left", "right"}
    assert plan.side == "right"
    # the receiver's close is the hold barrier, and it comes before the
    # giver opens
    strokes = [s for s in plan.steps if isinstance(s, GripStep)]
    close = next(i for i, s in enumerate(strokes)
                 if s.side == "right" and s.closedness >= 0.5)
    assert strokes[close].expect_hold
    opens = [i for i, s in enumerate(strokes)
             if s.side == "left" and s.closedness < 0.5]
    assert opens and min(opens) > close
    # measured afterwards, by the verb's own verifier
    verdict = Handover(object="cube").verifier(world0)(world1)
    assert verdict.verdict == "true", verdict.reason
    assert world1.gripper("right").held_object == "cube"
    assert not world1.gripper("left").holding
    cube = world1.find("cube")
    assert cube.provenance == "attached"
    assert np.linalg.norm(np.asarray(cube.p) - world1.arm("right").tool_p) < 0.01
    # the giving hand backed out along its own axis, clear of the object
    assert np.linalg.norm(world1.arm("left").tool_p - cube.p) > 0.07
    assert any("meeting at" in n for n in plan.notes)


def test_a_handover_is_refused_when_no_meeting_pose_reaches(d1_arm,
                                                           monkeypatch):
    _executor, source, holding = _left_holds_it(d1_arm)
    # one rung outside both arms' reach, one too low over the table
    monkeypatch.setattr(reach, "HANDOVER_MEETING_POINTS_M",
                        ((0.95, 0.05, 0.25), (0.40, 0.05, 0.05)))
    error = Handover(object="cube").plan(holding, d1_arm)
    assert not error.ok
    assert error.reason == UNREACHABLE_HANDOVER
    assert error.stage == "meeting_ladder"
    assert len(error.attempted) == 2
    assert "(0.95, +0.05, 0.25) m: meet refused" in error.attempted[0]
    assert "above what is under it" in error.attempted[1]
    assert "move the object" in error.detail


def test_handover_is_offered_only_when_one_hand_holds(d1_arm):
    executor, source = _robot(d1_arm)
    empty = source.observe()
    assert "handover" not in [s["name"] for s in tool_schemas(empty)]
    assert not any(c.name() == "handover" for c in candidates_for(empty))
    # still refused with the typed reason when asked for anyway
    asked = decode("handover", {"object": "cube"}, empty)
    refusal = asked.plan(empty, d1_arm)
    assert NOT_HOLDING in {u.code for u in refusal.unmet}
    _executor, _source, holding = _left_holds_it(d1_arm)
    assert "handover" in [s["name"] for s in tool_schemas(holding)]
    offered = [c for c in candidates_for(holding) if c.name() == "handover"]
    assert len(offered) == 1
    assert (offered[0].from_side, offered[0].to_side) == ("left", "right")
    assert offered[0].direction.alias() == "left"     # toward the giver
    # the schema without a world still describes every verb
    assert "handover" in [s["name"] for s in tool_schemas()]


def test_the_giver_does_not_open_when_the_receiver_holds_nothing(d1_arm):
    executor, source, holding = _left_holds_it(d1_arm, receiver_closes_on=None)
    _w, plan, report, after = _do(Handover(object="cube"), source, executor,
                                  d1_arm)
    assert not report.completed
    assert report.stop_reason == BARRIER_FAILED
    assert report.refusal.reason == HOLD_NOT_CONFIRMED
    # the left hand never got the open command
    assert after.gripper("left").holding
    assert executor.grips[-1][0] == "right"


def test_a_direction_away_from_the_giver_is_refused(d1_arm):
    _executor, _source, holding = _left_holds_it(d1_arm)
    error = Handover(object="cube", direction="right").plan(holding, d1_arm)
    assert not error.ok
    bad = [u for u in error.unmet if u.code == BAD_ARGUMENT]
    assert bad and "AWAY from the left hand" in bad[0].detail


def test_release_with_the_other_hand_holding_is_a_handover_not_a_drop(d1_arm):
    _executor, _source, holding = _left_holds_it(d1_arm)
    alone = Release(side="left").preconditions(holding)
    assert [u.code for u in alone] == ["unsupported_release"]
    both = holding.with_(grippers={
        **holding.grippers,
        "right": holding.gripper("left").__class__(
            "right", 1.0, holding=True, held_object="cube")})
    assert Release(side="left").preconditions(both) == []


def test_a_look_policy_refuses_a_handover_it_cannot_look_before():
    call = Handover(object="cube")
    _call, unmet = OperatorPolicy(look_before_stroke=True).clamp(
        call, PolicyState(), side="right")
    assert [u.code for u in unmet] == [LOOK_NOT_POSSIBLE]
    _call, unmet = OperatorPolicy(look_before_stroke=False).clamp(
        call, PolicyState(), side="right")
    assert unmet == []
