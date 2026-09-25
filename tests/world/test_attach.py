"""A held object rides the tool, and says it is inferred (design C.9, review 11)."""

from __future__ import annotations

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from manipulation_kit.agent.robot import SceneSource
from manipulation_kit.executor import KinematicExecutor, run
from manipulation_kit.primitives import Approach, Grasp, Lift
from manipulation_kit.primitives.offer import check
from manipulation_kit.world import (ArmView, ContainerView, GripperView,
                                    ObjectView, SurfaceView, WorldView,
                                    attached, grasp_transform, released,
                                    with_attached)

SCENE = {"objects": [
    {"name": "table", "kind": "surface", "p": [0.40, 0.0, 0.0],
     "size": [0.90, 0.80, 0.02]},
    {"name": "red_block", "kind": "object", "p": [0.38, 0.25, 0.035],
     "size": [0.05, 0.04, 0.05]},
    {"name": "box", "kind": "container", "p": [0.33, 0.34, 0.050],
     "size": [0.16, 0.14, 0.08], "interior": [0.14, 0.12, 0.07],
     "rim_height_m": 0.04}]}


def _live(kin):
    """Hardware-shaped: a transport that reports holding (the daemon's stall)
    without knowing WHAT it holds, and a declared scene."""
    executor = KinematicExecutor(kin)
    executor.next_object["left"] = "<the daemon does not say what>"
    return executor, SceneSource.from_scene(executor, kin, SCENE)


def _step(primitive, source, executor, kin):
    world = source.observe()
    plan = check(primitive, world, kin)
    assert plan.ok, plan
    assert run(plan, executor, kin=kin).completed
    return world, source.observe()


def test_a_lift_measures_a_rise_because_the_object_is_attached(d1_arm):
    """Review 11: ``live.py`` rebuilt the scene objects unchanged after every
    action, so ``ObjectRose`` measured ZERO rise after a real lift. The
    producer now publishes the attached pose and the rise is measured."""
    executor, source = _live(d1_arm)
    _step(Approach(object="red_block", side="left"), source, executor, d1_arm)
    _before, held = _step(Grasp(object="red_block", side="left"), source,
                          executor, d1_arm)
    block = held.find("red_block")
    assert block.provenance == "attached"
    assert held.gripper("left").held_object == "red_block"   # associated
    # the grasp does not move it: the transform is recorded AT the stroke
    assert np.allclose(block.p, SCENE["objects"][1]["p"], atol=1e-9)
    lift = Lift(object="red_block", side="left", height_m=0.10)
    world0, world1 = _step(lift, source, executor, d1_arm)
    verdict = lift.verifier(world0)(world1)
    assert verdict.verdict == "true", verdict.reason
    assert verdict.measured["rise_m"] == pytest.approx(0.10, abs=0.01)
    assert verdict.measured["provenance"] == "attached"
    assert "inferred, not sighted" in verdict.reason
    # ...and the pre-0.16 producer, which re-published the declared pose,
    # is exactly the FALSE the review measured
    stale = world1.with_(objects=world0.with_().objects)
    stale = stale.with_(objects=tuple(
        o if o.name != "red_block" else ObjectView(
            "red_block", p=SCENE["objects"][1]["p"],
            size=SCENE["objects"][1]["size"])
        for o in stale.objects))
    assert lift.verifier(world0)(stale).verdict == "false"


def test_an_attached_object_is_never_reported_as_observed(d1_arm):
    executor, source = _live(d1_arm)
    _step(Approach(object="red_block", side="left"), source, executor, d1_arm)
    _, held = _step(Grasp(object="red_block", side="left"), source, executor,
                    d1_arm)
    block = held.find("red_block")
    assert block.provenance == "attached" and block.inferred
    assert block.to_json()["provenance"] == "attached"
    assert "ATTACHED" in block.to_text(held.frames)
    assert "not sighted" in block.to_text(held.frames)
    assert attached(held, side="left") is block
    assert attached(held, side="right") is None
    # a verifier reading an inferred pose never calls a confident FALSE on it
    from manipulation_kit.primitives import Carry, Place
    far = held.with_(objects=tuple(
        o if o.name != "box" else ContainerView(
            "box", p=(0.30, -0.30, 0.05), size=(0.16, 0.14, 0.08),
            interior=(0.14, 0.12, 0.07)) for o in held.objects))
    over = Carry(object="red_block", to="box", side="left").verifier(far)(far)
    assert over.verdict == "unknown" and "inferred, not sighted" in over.reason
    # let go: the object stays where the hand had it, PREDICTED
    executor.set_gripper("left", 0.0, grip="soft")
    let_go = source.observe()
    block = let_go.find("red_block")
    assert block.provenance == "predicted" and block.inferred
    assert "PREDICTED" in block.to_text(let_go.frames)
    placed = Place(object="red_block", to="box", side="left").verifier(held)(
        let_go)
    assert placed.verdict == "unknown"
    assert "inferred, not sighted" in placed.reason
    # a statement about it (a model declaring what it sees) replaces the guess
    source.declare([ObjectView("red_block", p=block.p, size=block.size,
                               provenance="declared")])
    assert source.observe().find("red_block").provenance == "declared"


def _world(tool_p, tool_r, holding=True):
    return WorldView.of(
        [ObjectView("cube", p=(0.40, 0.0, 0.05), size=(0.04, 0.04, 0.04)),
         SurfaceView("table", p=(0.40, 0.0, 0.0), size=(0.8, 0.8, 0.02))],
        arms=[ArmView("left", joints=np.zeros(7), tool_p=tool_p,
                      tool_r=tool_r)],
        grippers=[GripperView("left", 1.0 if holding else 0.0,
                              holding=holding)])


def test_the_object_turns_with_the_wrist():
    """The old chain rule added the tool's TRANSLATION and ignored its turn."""
    down = R.from_euler("x", 180, degrees=True)
    at_stroke = _world((0.40, 0.0, 0.07), down)
    grasp = grasp_transform(at_stroke, side="left", name="cube")
    assert np.allclose(grasp.p_in_tool, down.inv().apply([0, 0, -0.02]))
    turned = down * R.from_euler("z", 90, degrees=True)
    later = with_attached(_world((0.30, 0.10, 0.20), turned), side="left",
                          name="cube", grasp=grasp)
    cube = later.find("cube")
    assert np.allclose(cube.p, [0.30, 0.10, 0.18])
    assert np.allclose((cube.r).as_matrix(),
                       (turned * grasp.r_in_tool).as_matrix())
    assert later.gripper("left").held_object == "cube"
    with pytest.raises(ValueError):
        with_attached(later, side="right", name="cube", grasp=grasp)
    with pytest.raises(LookupError):
        grasp_transform(later, side="left", name="nothing")
    assert released(later, name="cube").find("cube").provenance == "predicted"


def test_a_provenance_is_one_of_four():
    with pytest.raises(ValueError):
        ObjectView("cube", p=(0, 0, 0), size=(0.1, 0.1, 0.1),
                   provenance="guessed")
