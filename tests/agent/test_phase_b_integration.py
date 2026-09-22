"""Where steps 3, 4, 5 and 7 meet (redesign phase B integration).

Each test pins one seam the step branches could not test alone: the contact
verbs planned through the scene gate, the operator's droop margin reaching
the fingertip floor, ``allowed_directions`` covering every arriving verb,
measured contacts surviving across turns, and the d1-2 wrist placeholder
being refused on hardware.
"""

from __future__ import annotations

import json
import re
from pathlib import Path

import numpy as np
import pytest

from manipulation_kit.agent import OperatorPolicy
from manipulation_kit.agent.policy import DIRECTED_VERBS, DIRECTION_NOT_ALLOWED
from manipulation_kit.agent.robot import SceneSource, wrist_camera_from_scene
from manipulation_kit.primitives import clearance, verbs
from manipulation_kit.primitives.clearance import SceneGate, policy_of
from manipulation_kit.primitives.contact import Press, Probe
from manipulation_kit.primitives.planning import Kin, solve_path
from manipulation_kit.primitives.types import GUARD_REJECT, Waypoint
from manipulation_kit.world import ContactView

REPO = Path(__file__).resolve().parents[2]
D1_2_SCENE = REPO / "examples/agent/scenes/d1-2_tape_cup.json"


def _d1_2(kin):
    import sys
    sys.path.insert(0, str(REPO / "tests/primitives"))
    import test_clearance as tc  # noqa: PLC0415 - reuse its d1-2 world
    return tc._d1_2(kin), tc


@pytest.fixture
def restore(d1_arm):
    q = {s: np.array(d1_arm.joints(s), dtype=float) for s in ("left", "right")}
    before = getattr(d1_arm, clearance.POLICY_ATTR, None)
    yield d1_arm
    for s, v in q.items():
        d1_arm.set_joints(s, v)
    if before is None:
        if hasattr(d1_arm, clearance.POLICY_ATTR):
            delattr(d1_arm, clearance.POLICY_ATTR)
    else:
        setattr(d1_arm, clearance.POLICY_ATTR, before)


# --------------------------------------------------------------------------- #
# step 4 x step 5: the contact verbs are gated, minus what they measure
# --------------------------------------------------------------------------- #

def _hand_over_the_wagon(kin):
    """The right hand 64 mm over the d1-2 wagon top, fingertips down, clear
    of the cube and the cup (step 5's probe-leg start)."""
    from manipulation_kit.primitives import orientation as ap
    world, tc = _d1_2(kin)
    p = np.array([0.36, -0.25, 0.23])
    r = ap.align_tool("right", np.array([0.0, 0.0, -1.0]))
    with Kin(kin, world, scene=SceneGate(())) as borrowed:
        steps, error, _ = solve_path(
            borrowed, "right", [Waypoint("start", p, r, allow_via=True)],
            primitive="test")
    assert error is None, str(error)
    kin.set_joints("right", steps[-1].q)
    world, _ = _d1_2(kin)
    return world, tc


def test_a_probe_is_planned_through_the_scene_gate_minus_the_surface_it_measures(
        restore, monkeypatch):
    """``contact._contact_plan`` builds its ``Kin`` with
    ``SceneGate.for_contact``: a 150 mm probe down onto the wagon top (its
    leg ends 50 mm "through" the table, which is the point of a probe) plans
    and says the table was left out; with the ordinary gate in its place the
    SAME probe is refused naming the table — so the gate is live there."""
    kin = restore
    world, _ = _hand_over_the_wagon(kin)
    probe = Probe(side="right", direction="down", max_travel_m=0.15)
    plan = probe.plan(world, kin)
    assert plan.ok, str(plan)
    assert any("'table' it is aimed at is left out" in n for n in plan.notes), \
        plan.notes

    monkeypatch.setattr(
        SceneGate, "for_contact",
        classmethod(lambda cls, world, kin, *a, exclude=(), **k:
                    cls.of(world, kin, exclude=exclude)))
    blind = probe.plan(world, kin)
    assert not blind.ok and blind.reason == GUARD_REJECT, str(blind)
    assert "obstacle:table" in blind.attempted


def test_a_press_leaves_out_its_target_but_not_the_rest_of_the_scene(restore):
    """A press's named target is excluded by name (the leg ends past its
    face); every other declared thing still gates the plan."""
    kin = restore
    world, _ = _d1_2(kin)
    seen = {}
    real = SceneGate.for_contact.__func__

    def spy(cls, world, kin, p, d, travel, *, exclude=(), **k):
        gate = real(cls, world, kin, p, d, travel, exclude=exclude, **k)
        seen["exclude"], seen["names"] = tuple(exclude), {
            ob.name for ob in gate.obstacles}
        return gate

    import unittest.mock as mock
    with mock.patch.object(SceneGate, "for_contact", classmethod(spy)):
        Press(side="right", target="cup", direction="forward").plan(world, kin)
    assert seen["exclude"] == ("cup",)
    assert "cup" not in seen["names"] and "cube" in seen["names"]
    # NOTE (open, phase B report): the forward ray from a press standoff at
    # the cup's height is widened by the hand and the clearance, so it meets
    # the wagon top it stands on first and for_contact leaves THAT out too.


# --------------------------------------------------------------------------- #
# step 5 x step 7: the operator's droop reaches the fingertip floor
# --------------------------------------------------------------------------- #

def test_the_operator_droop_margin_reaches_the_fingertip_floor(restore):
    """``OperatorPolicy.droop_margin_m`` (d1-2 measured 0.012) is applied to
    the kinematics when a loop starts; a top-down Grasp on the d1-2 cube
    then keeps the pad tips ~15 mm (3 + 12) over the wagon, as
    ``MKIT_SUPPORT_CLEARANCE_M=0.015`` used to."""
    kin = restore
    world, _ = _d1_2(kin)

    def clearance_mm(plan):
        assert plan.ok, str(plan)
        note = [n for n in plan.notes if "keeps the pad tips" in n][0]
        return float(re.search(r"tips ([0-9.]+) mm", note).group(1))

    rigid = clearance_mm(verbs.Grasp(object="cube", side="right",
                                     direction="down").plan(world, kin))
    OperatorPolicy(droop_margin_m=0.012).apply_to(kin)
    assert policy_of(kin).droop_margin_m == 0.012
    drooped = clearance_mm(verbs.Grasp(object="cube", side="right",
                                       direction="down").plan(world, kin))
    assert rigid == pytest.approx(3.0, abs=1.5)
    assert drooped == pytest.approx(14.9, abs=1.5)
    with pytest.raises(ValueError):
        OperatorPolicy(droop_margin_m=-0.001)


def test_run_applies_the_policy_droop_to_the_robot_kinematics(
        agent_examples, restore):
    from manipulation_kit.agent import KinematicMirror, run
    from manipulation_kit.primitives import Place
    from scene import DEMO_WRIST_CAMERA, demo_scene
    world, kin = demo_scene()
    robot = KinematicMirror(kin, world, wrist_intrinsics=DEMO_WRIST_CAMERA)
    run(goal=Place(object="red_block", to="box"), robot=robot,
        policy=OperatorPolicy(droop_margin_m=0.012, max_turns=1),
        ask=lambda messages, tools: {"name": None, "arguments": {},
                                     "claimed": ""})
    assert policy_of(robot.kin).droop_margin_m == 0.012
    # and the flag / file round trip carries it
    import argparse
    parser = argparse.ArgumentParser()
    OperatorPolicy.add_arguments(parser)
    got = OperatorPolicy.from_args(parser.parse_args(["--droop-margin-m",
                                                      "0.012"]))
    assert got.droop_margin_m == 0.012
    assert OperatorPolicy.from_json(got.to_json()) == got


# --------------------------------------------------------------------------- #
# step 4 x step 7: allowed_directions covers every arriving verb
# --------------------------------------------------------------------------- #

def test_allowed_directions_restricts_every_verb_that_arrives_along_one():
    assert set(DIRECTED_VERBS) == {"approach", "grasp", "probe", "press"}
    policy = OperatorPolicy(allowed_directions=("down",))
    for call in (Probe(direction="forward"),
                 Press(target="cup", direction="forward"),
                 verbs.Approach(object="cube", direction="forward")):
        _, unmet = policy.clamp(call)
        assert [u.code for u in unmet] == [DIRECTION_NOT_ALLOWED], call
    for call in (Probe(direction="down"), verbs.Lift(object="cube")):
        _, unmet = policy.clamp(call)
        assert unmet == [], call      # a Lift's "up" is not an arrival


# --------------------------------------------------------------------------- #
# step 4 x step 7: contacts are measurements, kept across turns
# --------------------------------------------------------------------------- #

class _Still:
    """An executor whose state never changes (enough for SceneSource)."""

    def __init__(self, kin):
        from manipulation_kit.executor import KinematicExecutor
        self.inner = KinematicExecutor(kin)

    def state(self):
        return self.inner.state()


def test_the_scene_source_keeps_contacts_until_the_scene_is_restated(d1_arm):
    from manipulation_kit.agent.tools import apply_declare_scene
    from manipulation_kit.agent.robot import LiveRobot
    scene = json.loads(D1_2_SCENE.read_text(encoding="utf-8"))
    source = SceneSource.from_scene(_Still(d1_arm), d1_arm, scene)
    robot = LiveRobot(source.executor, source, d1_arm)
    touch = ContactView("right", True, np.array([0.4, -0.2, 0.166]),
                        np.array([0.4, -0.2, 0.195]), np.array([0, 0, 1.0]),
                        "contact", 0.06, 4.2, "wagon", "probe", 0.0)
    robot.remember_contacts((touch,))
    assert robot.world().contacts == (touch,)
    assert robot.world().contacts == (touch,)          # the next turn too
    apply_declare_scene(robot, {"objects": [
        {"name": "mug", "kind": "object", "p": [0.45, 0.1, 0.2],
         "size": [0.08, 0.08, 0.1]}]})
    assert robot.world().contacts == ()


# --------------------------------------------------------------------------- #
# step 7: the d1-2 wrist placeholder dry-runs, and is refused on hardware
# --------------------------------------------------------------------------- #

def test_the_d1_2_wrist_placeholder_is_for_the_mirror_only():
    scene = json.loads(D1_2_SCENE.read_text(encoding="utf-8"))
    block = scene["robot"]["wrist_camera"]
    assert block["measured"] is False and "NOT MEASURED" in block["_source"]
    assert wrist_camera_from_scene(scene) is not None          # the mirror
    assert wrist_camera_from_scene(scene, measured_only=True) is None
    scene["robot"]["wrist_camera"]["measured"] = True
    assert wrist_camera_from_scene(scene, measured_only=True) is not None
