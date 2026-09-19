"""Which hand picks the block is decided by which hand can DELIVER it.

The near-hand rule — the sign of the block's y — was the whole rule until
2026-09-19, and it cost the agent-eval harness two trials out of five: the
block was picked up cleanly by the arm nearest it and the bin it had to go in
was on the other side of the wagon, so ``Carry`` was refused after the lift
with the block already in the jaws. ``examples/agent/chain.py`` plans the whole
Approach -> Grasp -> Lift -> Carry -> Place for BOTH arms first, with the kit's
own pure ``plan()``, and the arm whose chain plans is the one that grasps.

The layout is ``blocks-eval``'s NOMINAL one (``--fixed-blocks``): the cube strip
and the wagon exactly as the scene draws them. The bins appear twice, because
the two facts are different:

* where the served scene puts them (x ~ 0.63) — measured 2026-09-19, NO
  top-down posture of either arm puts the tool past x = 0.53 at any height, so
  every chain is refused and the chooser has to SAY so rather than pick;
* and in the reachable band (x = 0.50), where the question the chooser exists
  for — "which of the two arms" — actually has an answer.

Real ``d1.urdf``, real IK, real guard, all of it.
"""

from __future__ import annotations

import numpy as np
import pytest

from manipulation_kit.primitives.approach import tool_from_link7
from manipulation_kit.primitives.types import UNREACHABLE_DESTINATION
from manipulation_kit.world import (ArmView, ContainerView, GripperView,
                                    ObjectView, SurfaceView, WorldView)

CUBE = 0.040
#: wagon top centre in the base frame, as the env server reports it
WAGON_TOP = (0.564, 0.002, 0.169)


def _uv(u, v, z):
    return (WAGON_TOP[0] + u, WAGON_TOP[1] + v, z)


#: ``blocks_eval.NOMINAL_CUBE_UV`` zipped with ``CUBE_COLOURS``: red on the
#: robot's RIGHT, blue on the centre line, yellow on the LEFT.
NOMINAL_CUBES = {
    "block_red": _uv(-0.110, -0.105, WAGON_TOP[2] + CUBE / 2),
    "block_blue": _uv(-0.110, 0.000, WAGON_TOP[2] + CUBE / 2),
    "block_yellow": _uv(-0.110, 0.105, WAGON_TOP[2] + CUBE / 2),
}
#: ``blocks_eval.NOMINAL_BOX_UV`` — red goes robot-LEFT, blue robot-RIGHT
SERVED_BOXES = {"box_red": _uv(0.060, 0.168, WAGON_TOP[2] + 0.030),
                "box_blue": _uv(0.060, -0.168, WAGON_TOP[2] + 0.030)}
#: the same two bins, moved into the arm's measured top-down workspace
REACHABLE_BOXES = {"box_red": (0.500, 0.180, WAGON_TOP[2] + 0.030),
                   "box_blue": (0.500, -0.180, WAGON_TOP[2] + 0.030)}


def _scene(kin, boxes):
    arms, grippers = [], []
    for side in ("left", "right"):
        kin.set_joints(side, kin.home(side))
        p, r = tool_from_link7(*kin.ee_pose(side))
        arms.append(ArmView(side, joints=kin.joints(side), tool_p=p, tool_r=r,
                            mode="position"))
        grippers.append(GripperView(side, 0.0))
    props = [ObjectView(name, p=p, size=(CUBE,) * 3, colour=name.split("_")[1])
             for name, p in NOMINAL_CUBES.items()]
    props += [ContainerView(name, p=p, size=(0.15, 0.15, 0.06),
                            interior=(0.146, 0.146, 0.058))
              for name, p in boxes.items()]
    props.append(SurfaceView("wagon_top", p=WAGON_TOP, size=(0.4, 0.6, 0.002)))
    return WorldView.of(props, arms=arms, grippers=grippers, stamp=0.0)


# --------------------------------------------------------------------------- #
# the choice
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize("obj,destination,expected", [
    ("block_red", "box_blue", "right"),      # both on the robot's right
    ("block_blue", "box_red", "left"),       # the bin is on the left; go left
])
def test_both_directions_pick_the_arm_that_can_deliver(d1_arm, agent_examples,
                                                       obj, destination, expected):
    from chain import choose_side

    world = _scene(d1_arm, REACHABLE_BOXES)
    pick = choose_side(world, d1_arm, obj=obj, destination=destination)
    assert pick.reachable, pick.reason
    assert pick.side == expected, pick.reason
    assert pick.chains[expected].ok
    assert pick.chains[expected].planned == 5
    assert [link.verb for link in pick.chains[expected].links] == [
        "approach", "grasp", "lift", "carry", "place"]


def test_the_hand_flips_away_from_the_near_one_when_it_cannot_deliver(
        d1_arm, agent_examples):
    """The exhibit, and it is the trial the harness lost twice.

    ``block_blue`` sits on the centre line at y = +0.002, so the near-hand rule
    says LEFT. Its bin is ``box_blue``, on the robot's right. The left arm
    grasps and lifts it perfectly well and then cannot deliver it; the right
    arm can do both, because the centre line is inside both arms' workspaces.
    The chain picks the right.
    """
    from chain import _near_hand, choose_side

    world = _scene(d1_arm, REACHABLE_BOXES)
    assert _near_hand(world, "block_blue") == "left"
    pick = choose_side(world, d1_arm, obj="block_blue", destination="box_blue")
    assert pick.reachable, pick.reason
    assert pick.side == "right", pick.reason
    assert pick.chains["right"].ok
    assert not pick.chains["left"].ok
    assert pick.chains["left"].broke_at.verb == "carry"
    assert (pick.chains["left"].broke_at.result.reason
            == UNREACHABLE_DESTINATION)


def test_a_block_and_a_bin_on_OPPOSITE_sides_is_refused_by_both_arms(
        d1_arm, agent_examples):
    """The case no arm choice can rescue, stated rather than discovered again.

    ``block_red`` is at y = -0.105 and ``box_red`` at y = +0.180. The LEFT arm
    cannot even reach the block (the guard refuses its standoff, 358 mm short);
    the RIGHT arm grasps and lifts it and then cannot reach the bin. Measured
    2026-09-19: the two arms' top-down workspaces overlap only within about
    +-40 mm of the centre line, so a block outside that band can only be
    delivered to a bin on its OWN side — or handed over, which is not in the
    verb set. A draw like this is an impossible trial, and the chooser says so
    (``reachable`` False) instead of picking a hand and letting the run
    discover it four verbs later.
    """
    from chain import choose_side

    world = _scene(d1_arm, REACHABLE_BOXES)
    pick = choose_side(world, d1_arm, obj="block_red", destination="box_red")
    assert not pick.reachable, pick.reason
    assert not pick.chains["left"].ok and not pick.chains["right"].ok
    assert pick.chains["left"].broke_at.verb == "approach"
    assert pick.chains["right"].broke_at.verb == "carry"
    assert (pick.chains["right"].broke_at.result.reason
            == UNREACHABLE_DESTINATION)


def test_the_served_bins_are_out_of_reach_for_both_arms_and_it_says_so(
        d1_arm, agent_examples):
    """The measured state of ``blocks-eval`` on 2026-09-19.

    Not a reason to pick a hand quietly: ``reachable`` is False, both chains
    are reported, and the refusal that ended each is the ladder's, with a
    residual. A chooser that returned a side and said nothing would have
    turned a scene fact into a mystery model failure.
    """
    from chain import choose_side

    world = _scene(d1_arm, SERVED_BOXES)
    for obj, destination in (("block_red", "box_blue"),
                             ("block_blue", "box_red")):
        pick = choose_side(world, d1_arm, obj=obj, destination=destination)
        assert not pick.reachable
        assert pick.side in ("left", "right")
        broke = pick.chains[pick.side].broke_at
        assert broke is not None and broke.verb == "carry"
        assert broke.result.reason == UNREACHABLE_DESTINATION
        assert np.isfinite(broke.result.residual_m)
        assert "no arm plans the whole chain" in pick.reason


# --------------------------------------------------------------------------- #
# what the chain is allowed to claim
# --------------------------------------------------------------------------- #

def test_the_chain_stops_at_the_first_refusal(d1_arm, agent_examples):
    """No link is planned against a world that never happened."""
    from chain import plan_chain

    world = _scene(d1_arm, SERVED_BOXES)
    chain = plan_chain(world, d1_arm, obj="block_red", destination="box_blue",
                       side="right")
    assert not chain.ok
    assert [link.verb for link in chain.links] == ["approach", "grasp", "lift",
                                                   "carry"]
    assert chain.planned == 3


def test_planning_a_chain_leaves_both_arms_where_it_found_them(d1_arm,
                                                               agent_examples):
    from chain import choose_side

    world = _scene(d1_arm, REACHABLE_BOXES)
    before = {s: np.array(d1_arm.joints(s)) for s in ("left", "right")}
    first = choose_side(world, d1_arm, obj="block_red", destination="box_blue")
    for side, q in before.items():
        assert np.allclose(d1_arm.joints(side), q), f"{side} arm was left moved"
    again = choose_side(world, d1_arm, obj="block_red", destination="box_blue")
    assert first.side == again.side and first.reason == again.reason


def test_the_world_the_chain_rolls_forward_holds_the_block_it_grasped(
        d1_arm, agent_examples):
    """The hypothetical is a prediction about REACH, and it is built the only
    way that makes that prediction mean anything: the arm at the joint vector
    the previous plan ended on, the object carried with the tool."""
    from chain import _grasped, _moved, _posed, _tool_of

    world = _scene(d1_arm, REACHABLE_BOXES)
    q = np.array(world.arm("right").joints) * 0.0 + np.array(d1_arm.ready("right"))
    posed = _posed(world, d1_arm, "right", q)
    assert np.allclose(posed.arm("right").joints, q)
    assert np.allclose(posed.arm("right").tool_p, _tool_of(d1_arm, "right", q)[0])
    held = _grasped(posed, "right", "block_red")
    assert held.gripper("right").holding
    assert held.gripper("right").held_object == "block_red"
    assert held.holder_of("block_red") == "right"
    moved = _moved(held, "block_red", np.array([0.0, 0.0, 0.12]))
    assert moved.find("block_red").p[2] == pytest.approx(
        world.find("block_red").p[2] + 0.12)
    # ...and none of it touched the world it was derived from
    assert not world.gripper("right").holding
    assert np.allclose(world.arm("right").joints, d1_arm.home("right"))
