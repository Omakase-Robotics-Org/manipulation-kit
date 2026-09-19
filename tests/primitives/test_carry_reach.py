"""Choosing the transit height from the reachable set — F10, against the real URDF.

What it was (2026-09-19, agent-eval run E/F): every trial that got as far as a
lift then lost ``Carry`` AND ``Place`` to ``ik_fail`` at ``over_destination``.
The waypoint was the destination's rim plus a CONSTANT 100 mm, and a constant
is a claim about the arm that nobody measured. So the transit height is now
chosen from :data:`~manipulation_kit.primitives.verbs.CARRY_CLEARANCE_LADDER_M`
— the largest rung that plans wins — floored at what the carried object itself
needs to clear the rim, and a destination no rung reaches is refused as
``unreachable_destination`` with the min residual and the rung that came
closest, rather than as a bare ``ik_fail`` a caller cannot act on.

Nothing here is a mock: the bundled ``d1.urdf``, the real DLS IK, MotionGuard,
and a holding posture produced by planning a real ``Grasp`` and ``Lift`` rather
than asserted into a world.
"""

from __future__ import annotations

import numpy as np
import pytest

from manipulation_kit.primitives import Carry, Grasp, Lift, Place
from manipulation_kit.primitives import verbs
from manipulation_kit.primitives.approach import tool_from_link7
from manipulation_kit.primitives.types import (JointStep, PRECONDITION_UNMET,
                                               UNREACHABLE_DESTINATION)
from manipulation_kit.world import (ArmView, ContainerView, GripperView,
                                    ObjectView, SurfaceView, WorldView)

CUBE = 0.040
WAGON_TOP = (0.564, 0.002, 0.169)
CUBE_P = (0.45, -0.12, WAGON_TOP[2] + CUBE / 2)

#: A bin on a SHELF, high enough that the 100 mm transit is over the right
#: arm's head and the 60 mm one is not. Measured on this URDF: at x = 0.48,
#: y = -0.15 the arm's top-down ceiling is between 0.44 and 0.46 m, and the
#: rim is at 0.42 — so rim + 100 mm and rim + 80 mm are both out and rim +
#: 60 mm is in. This is the shape of destination the ladder exists for.
SHELF_BIN_P = (0.48, -0.15, 0.36)
SHELF_BIN_SIZE = (0.15, 0.15, 0.12)
SHELF_BIN_RIM_Z = SHELF_BIN_P[2] + SHELF_BIN_SIZE[2] / 2       # 0.42

#: The served ``blocks-eval`` bins, as the env reports them (seed 7, trial 3).
#: Measured 2026-09-19: NO top-down posture of either arm puts the tool past
#: x = 0.53 at any height, so a bin whose centre is at x = 0.63 is outside the
#: workspace and no transit height can help. That is what the refusal has to
#: say out loud.
SERVED_BOX_BLUE = (0.631, -0.168, 0.200)


def _home_world(kin, props):
    arms, grippers = [], []
    for side in ("left", "right"):
        kin.set_joints(side, kin.home(side))
        p, r = tool_from_link7(*kin.ee_pose(side))
        arms.append(ArmView(side, joints=kin.joints(side), tool_p=p, tool_r=r,
                            mode="position"))
        grippers.append(GripperView(side, 0.0))
    return WorldView.of(props, arms=arms, grippers=grippers, stamp=0.0)


def _tool_of(kin, side, q):
    saved = np.array(kin.joints(side), dtype=float)
    try:
        kin.set_joints(side, np.asarray(q, dtype=float))
        return tool_from_link7(*kin.ee_pose(side))
    finally:
        kin.set_joints(side, saved)


def _last_q(plan, side, fallback):
    steps = [s for s in plan.steps if isinstance(s, JointStep) and s.side == side]
    return np.array(steps[-1].q if steps else fallback, dtype=float)


def _posed(world, kin, side, q):
    p, r = _tool_of(kin, side, q)
    arms = dict(world.arms)
    arms[side] = ArmView(side, joints=np.asarray(q, dtype=float), tool_p=p,
                         tool_r=r, mode="position")
    return world.with_(arms=arms)


def _holding(world, side, name):
    grippers = dict(world.grippers)
    grippers[side] = GripperView(side, 1.0, holding=True, held_object=name,
                                 jaw_gap_m=CUBE, grip="firm", jaw_stalled=True)
    return world.with_(grippers=grippers)


def _moved(world, name, delta):
    import dataclasses
    return world.with_(objects=tuple(
        dataclasses.replace(o, p=np.asarray(o.p, dtype=float) + np.asarray(delta))
        if o.name == name else o for o in world.objects))


def after_a_real_grasp_and_lift(kin, destination, *, side="right", lift_m=0.12):
    """A world whose right hand IS holding the cube, 120 mm up — planned, not faked.

    The posture matters: ``Carry`` starts from where the tool actually is, and
    a test that put the arm at HOME and set ``holding=True`` would be planning
    a transit from a pose the robot is never in.
    """
    world = _home_world(kin, [
        ObjectView("cube", p=CUBE_P, size=(CUBE,) * 3, colour="red"),
        destination,
        SurfaceView("wagon_top", p=WAGON_TOP, size=(0.4, 0.6, 0.002))])
    grasp = Grasp(object="cube", side=side, approach="top_down",
                  grip="firm").plan(world, kin)
    assert grasp.ok, str(grasp)
    q0 = world.arm(side).joints
    q1 = _last_q(grasp, side, q0)
    world = _holding(_posed(world, kin, side, q1), side, "cube")
    lift = Lift(object="cube", side=side, height_m=lift_m).plan(world, kin)
    assert lift.ok, str(lift)
    q2 = _last_q(lift, side, q1)
    world = _moved(world, "cube",
                   _tool_of(kin, side, q2)[0] - _tool_of(kin, side, q1)[0])
    return _posed(world, kin, side, q2)


def shelf_bin():
    return ContainerView("bin_on_shelf", p=SHELF_BIN_P, size=SHELF_BIN_SIZE,
                         interior=(0.14, 0.14, 0.116))


def served_box():
    return ContainerView("box_blue", p=SERVED_BOX_BLUE, size=(0.15, 0.15, 0.06),
                         interior=(0.146, 0.146, 0.058))


# --------------------------------------------------------------------------- #
# the ladder
# --------------------------------------------------------------------------- #

def test_a_carry_that_only_plans_at_60_mm_takes_60_mm_and_says_so(d1_arm):
    """The whole point: the height is CHOSEN, and the choice is in the record."""
    world = after_a_real_grasp_and_lift(d1_arm, shelf_bin())
    plan = Carry(object="cube", to="bin_on_shelf", side="right").plan(world, d1_arm)
    assert plan.ok, str(plan)
    assert any("transit 60 mm" in note for note in plan.notes), plan.notes
    assert any("100 mm rung is out of reach" in note for note in plan.notes)
    # ...and it ends where the chosen rung said, not where the default did
    tool = _tool_of(d1_arm, "right", _last_q(plan, "right",
                                             world.arm("right").joints))[0]
    assert tool[2] == pytest.approx(SHELF_BIN_RIM_Z + 0.060, abs=0.01)
    assert np.linalg.norm(tool[:2] - np.array(SHELF_BIN_P[:2])) < 0.01


def test_the_rungs_above_it_really_are_out_of_reach(d1_arm, monkeypatch):
    """The 60 mm plan is not luck: with the ladder cut to 100 mm alone, the
    same carry is refused — so the rung below IS what made it possible."""
    world = after_a_real_grasp_and_lift(d1_arm, shelf_bin())
    for only in ((0.10,), (0.08,)):
        monkeypatch.setattr(verbs, "CARRY_CLEARANCE_LADDER_M", only)
        error = Carry(object="cube", to="bin_on_shelf", side="right",
                      clearance_m=only[0]).plan(world, d1_arm)
        assert not error.ok, f"{only[0]} m was supposed to be out of reach"
        assert error.reason == UNREACHABLE_DESTINATION
    monkeypatch.setattr(verbs, "CARRY_CLEARANCE_LADDER_M", (0.06,))
    plan = Carry(object="cube", to="bin_on_shelf", side="right",
                 clearance_m=0.06).plan(world, d1_arm)
    assert plan.ok, str(plan)


def test_the_ladder_never_goes_below_the_rim_plus_the_object(d1_arm):
    """The floor is about the OBJECT and the rim, not about the solver.

    With the tool at the object's centre this is exactly "rim + half height +
    10 mm"; the tool is 12 mm above a 40 mm cube's centre after a top-down
    grasp, so the measured hang is what the floor is actually computed from.
    """
    world = after_a_real_grasp_and_lift(d1_arm, shelf_bin())
    tool = world.arm("right").tool_p
    carry = Carry(object="cube", to="bin_on_shelf", side="right")
    ladder = carry.ladder(world, tool)
    cube = world.find("cube")
    hang = float(tool[2]) - (float(cube.p[2]) - cube.height() / 2.0)
    assert hang == pytest.approx(CUBE / 2 + 0.012, abs=0.004)
    assert min(ladder) >= hang + verbs.RIM_MARGIN_M - 1e-9
    assert ladder == tuple(sorted(ladder, reverse=True))
    assert ladder[0] == pytest.approx(verbs.DEFAULT_CLEARANCE_M)
    # an ask BELOW the floor does not get honoured — the floor is the rung
    tiny = Carry(object="cube", to="bin_on_shelf", side="right",
                 clearance_m=0.005).ladder(world, tool)
    assert len(tiny) == 1 and tiny[0] >= hang + verbs.RIM_MARGIN_M - 1e-9


def test_an_explicit_larger_clearance_is_still_the_first_rung(d1_arm):
    world = after_a_real_grasp_and_lift(d1_arm, shelf_bin())
    tool = world.arm("right").tool_p
    ladder = Carry(object="cube", to="bin_on_shelf", side="right",
                   clearance_m=0.15).ladder(world, tool)
    assert ladder[0] == pytest.approx(0.15)
    # ...and then the fixed list, minus the rungs the object cannot fit under
    assert ladder[1:] == tuple(c for c in verbs.CARRY_CLEARANCE_LADDER_M
                               if c >= min(ladder))


# --------------------------------------------------------------------------- #
# the refusal
# --------------------------------------------------------------------------- #

def test_a_destination_no_rung_reaches_is_unreachable_destination(d1_arm):
    """The served blocks-eval bin: 10 cm outside the arm's top-down workspace.

    The refusal has to be a statement about the ARM AND THE DESTINATION —
    "try the other hand" — rather than ``ik_fail`` at one waypoint, which
    reads like "try another waypoint" and is what the model was told for
    three trials in a row.
    """
    world = after_a_real_grasp_and_lift(d1_arm, served_box())
    error = Carry(object="cube", to="box_blue", side="right").plan(world, d1_arm)
    assert not error.ok
    assert error.reason == UNREACHABLE_DESTINATION
    assert error.primitive == "carry" and error.side == "right"
    assert np.isfinite(error.residual_m) and error.residual_m > 0.05
    assert "100" in error.detail and "mm above its rim" in error.detail
    assert "the closest was" in error.detail
    assert "other arm" in error.detail


def test_place_is_refused_the_same_way_and_for_the_same_bin(d1_arm):
    world = after_a_real_grasp_and_lift(d1_arm, served_box())
    error = Place(object="cube", to="box_blue", side="right").plan(world, d1_arm)
    assert not error.ok and error.reason == UNREACHABLE_DESTINATION


def test_an_empty_hand_is_still_a_precondition_not_a_reach_problem(d1_arm):
    """The new reason must not swallow the old ones."""
    world = _home_world(d1_arm, [
        ObjectView("cube", p=CUBE_P, size=(CUBE,) * 3),
        shelf_bin(),
        SurfaceView("wagon_top", p=WAGON_TOP, size=(0.4, 0.6, 0.002))])
    error = Carry(object="cube", to="bin_on_shelf", side="right").plan(world, d1_arm)
    assert not error.ok and error.reason == PRECONDITION_UNMET


# --------------------------------------------------------------------------- #
# Place still places
# --------------------------------------------------------------------------- #

def test_place_descends_to_the_set_down_and_still_ends_by_letting_go(d1_arm):
    """The transit height moved; what "placed" means did not."""
    world = after_a_real_grasp_and_lift(d1_arm, shelf_bin())
    plan = Place(object="cube", to="bin_on_shelf", side="right").plan(world, d1_arm)
    assert plan.ok, str(plan)
    # RISE, travel, descend: the first leg exists so an object below the rim
    # does not take a diagonal into it (R10).
    assert [w.label for w in plan.waypoints][:2] == ["clearance",
                                                     "over_destination"]
    assert plan.waypoints[-1].label in ("set_down", "rim_release")
    assert any("transit" in note for note in plan.notes), plan.notes
    grips = [s for s in plan.steps if getattr(s, "closedness", None) == 0.0]
    assert grips, "a place that does not release is not a place"
    # the object is set down on the bin's own floor, not dropped from the rim
    bin_ = world.find("bin_on_shelf")
    floor = float(bin_.p[2]) - float(bin_.interior[2]) / 2.0
    drop = plan.waypoints[-1].p
    tool = world.arm("right").tool_p
    hang = float(tool[2]) - (float(world.find("cube").p[2]) - CUBE / 2.0)
    assert float(drop[2]) - hang == pytest.approx(floor + 0.01, abs=0.005)
    assert bin_.contains(np.array([drop[0], drop[1], floor + CUBE / 2]),
                         world.frames)


# --------------------------------------------------------------------------- #
# purity
# --------------------------------------------------------------------------- #

def test_walking_the_whole_ladder_leaves_the_model_where_it_found_it(d1_arm):
    """A ladder is several plans over the same mirror. If one leaked, the next
    plan would depend on the last — and ``plan()`` is a value."""
    world = after_a_real_grasp_and_lift(d1_arm, served_box())
    before = {s: np.array(d1_arm.joints(s)) for s in ("left", "right")}
    first = Carry(object="cube", to="box_blue", side="right").plan(world, d1_arm)
    for side, q in before.items():
        assert np.allclose(d1_arm.joints(side), q), f"{side} arm was left moved"
    again = Carry(object="cube", to="box_blue", side="right").plan(world, d1_arm)
    assert not first.ok and not again.ok
    assert first.to_json() == again.to_json()


def test_the_chosen_rung_is_the_same_every_time(d1_arm):
    world = after_a_real_grasp_and_lift(d1_arm, shelf_bin())
    notes = [Carry(object="cube", to="bin_on_shelf", side="right")
             .plan(world, d1_arm).notes for _ in range(2)]
    assert notes[0] == notes[1]
