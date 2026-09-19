"""Nudge is the only verb that takes numbers, so it is the only one that can
be asked for something silly. It snaps and clamps instead of refusing.

The granularity rule behind the grid is Raptor's Jev run (2026-09-19): a task
needing a 30 mm correction FAILS when the only offer is 50 mm. Coarse and fine
have to be in the same vocabulary.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from manipulation_kit.primitives import (NUDGE_GRID_M, NUDGE_MAX_YAW_RAD, Nudge,
                                         snap)


@pytest.mark.parametrize("asked_mm, expect_mm", [
    (0.0, 0), (3.0, 0), (4.9, 0),          # below half of 10 mm: nothing
    (5.1, 10), (9.0, 10), (10.0, 10), (14.0, 10),
    (23.0, 30), (30.0, 30), (41.0, 50), (50.0, 50), (400.0, 50),
    (-23.0, -30), (-9.0, -10), (-400.0, -50),
])
def test_translations_snap_to_the_ten_thirty_fifty_grid(asked_mm, expect_mm):
    assert snap(asked_mm / 1000.0) == pytest.approx(expect_mm / 1000.0)


def test_thirty_millimetres_is_reachable_in_one_offer():
    """G1, the half that matters: the grid can express the correction."""
    assert snap(0.030) == pytest.approx(0.030)
    assert 0.030 in NUDGE_GRID_M


def test_a_fifty_only_grid_could_not_express_it():
    """The other half of G1, as an executable statement of the failure."""
    coarse = (0.050,)
    nearest = min(coarse, key=lambda g: abs(0.030 - g))
    assert nearest != pytest.approx(0.030)


def test_yaw_is_clamped_to_fifteen_degrees_about_the_approach_axis():
    _, yaw = Nudge(side="left", dyaw=math.radians(90)).snapped()
    assert yaw == pytest.approx(NUDGE_MAX_YAW_RAD)
    _, yaw = Nudge(side="left", dyaw=-math.radians(90)).snapped()
    assert yaw == pytest.approx(-NUDGE_MAX_YAW_RAD)
    _, yaw = Nudge(side="left", dyaw=math.radians(5)).snapped()
    assert yaw == pytest.approx(math.radians(5))


def test_a_nudge_that_snaps_to_nothing_is_an_unmet_precondition(d1_arm, observe):
    world = observe(d1_arm)
    verb = Nudge(side="left", dx=0.002)
    codes = {u.code for u in verb.preconditions(world)}
    assert "no_motion" in codes
    assert not verb.plan(world, d1_arm).ok


def test_the_plan_says_out_loud_that_it_snapped(d1_arm, observe):
    world = observe(d1_arm)
    plan = Nudge(side="left", dz=0.023, frame="base").plan(world, d1_arm)
    assert plan.ok, str(plan)
    assert any("snapped" in note for note in plan.notes)
    assert any("30" in note for note in plan.notes)


def test_a_base_frame_nudge_moves_the_tool_along_the_base_axis(d1_arm, observe):
    from manipulation_kit.primitives.approach import tool_from_link7
    world = observe(d1_arm)
    before = world.arm("left").tool_p
    plan = Nudge(side="left", dz=0.030, frame="base").plan(world, d1_arm)
    assert plan.ok, str(plan)
    d1_arm.set_joints("left", plan.joint_steps()[-1].q)
    after = tool_from_link7(*d1_arm.ee_pose("left"))[0]
    assert np.allclose(after - before, [0.0, 0.0, 0.030], atol=0.004)


def test_a_tool_frame_nudge_moves_along_the_hands_own_axes(d1_arm, observe):
    from manipulation_kit.primitives.approach import tool_from_link7
    world = observe(d1_arm)
    arm = world.arm("left")
    plan = Nudge(side="left", dz=0.030, frame="tool").plan(world, d1_arm)
    assert plan.ok, str(plan)
    d1_arm.set_joints("left", plan.joint_steps()[-1].q)
    after = tool_from_link7(*d1_arm.ee_pose("left"))[0]
    expected = arm.tool_p + arm.tool_r.apply([0.0, 0.0, 0.030])
    assert np.allclose(after, expected, atol=0.004)


def test_an_unknown_frame_name_is_refused_rather_than_guessed(d1_arm, observe):
    world = observe(d1_arm)
    verb = Nudge(side="left", dz=0.030, frame="world")
    # ONE argument vocabulary now owns this: a value outside a published
    # domain is ``bad_argument`` whichever argument it was (R14).
    unmet = verb.preconditions(world)
    assert any(u.code == "bad_argument" and "frame" in u.detail for u in unmet)


# --------------------------------------------------------------------------- #
# the verifier's tolerance — measured against what was ASKED for
# --------------------------------------------------------------------------- #

def test_the_tolerance_scales_with_the_requested_displacement():
    """``TOOL_TOL_M`` was 20 mm and ``NUDGE_GRID_M`` starts at 10 mm, so the
    finest correction on the menu could not fail its own verifier."""
    from manipulation_kit.primitives import verifiers as V

    assert V.moved_tol([0.010, 0.0, 0.0]) == pytest.approx(0.004)
    assert V.moved_tol([0.030, 0.0, 0.0]) == pytest.approx(0.012)
    assert V.moved_tol([0.050, 0.0, 0.0]) == pytest.approx(0.020)
    # the floor: below ~3 mm a FALSE would be measuring the IK, not the robot
    assert V.moved_tol([0.001, 0.0, 0.0]) == pytest.approx(V.MIN_MOVED_TOL_M)
    # and it is a magnitude, not a per-axis budget
    assert V.moved_tol([0.030, 0.040, 0.0]) == pytest.approx(0.020)


def test_a_ten_millimetre_nudge_that_moved_nothing_is_false(d1_arm, observe):
    """The Raptor granularity lesson turned inward: the correction the fine
    grid exists for is the one the verifier has to be able to fail."""
    world = observe(d1_arm)
    verifier = Nudge(side="left", dx=0.010, frame="base").verifier(world)
    assert verifier.tol_m == pytest.approx(0.004)
    verdict = verifier(observe(d1_arm, stamp=1.0))      # nothing moved
    assert str(verdict.verdict) == "false"
    assert verdict.measured["error_m"] == pytest.approx(0.010, abs=1e-4)


def test_a_ten_millimetre_nudge_that_moved_ten_millimetres_is_true(d1_arm, observe):
    from manipulation_kit.primitives.approach import tool_from_link7

    world = observe(d1_arm)
    verb = Nudge(side="left", dx=0.010, frame="base")
    verifier = verb.verifier(world)
    plan = verb.plan(world, d1_arm)
    assert plan.ok, str(plan)
    d1_arm.set_joints("left", plan.joint_steps()[-1].q)
    after = observe(d1_arm, stamp=1.0)
    assert bool(verifier(after)), verifier(after).reason
    moved = tool_from_link7(*d1_arm.ee_pose("left"))[0] - world.arm("left").tool_p
    assert np.linalg.norm(moved - np.array([0.010, 0.0, 0.0])) <= 0.004


def test_a_fifty_millimetre_nudge_keeps_the_old_twenty_millimetre_window(d1_arm, observe):
    """Scaling is not tightening everywhere: the coarse end of the grid is
    graded exactly as it was, so nothing that used to pass now fails."""
    world = observe(d1_arm)
    assert Nudge(side="left", dz=0.050, frame="base").verifier(world).tol_m == \
        pytest.approx(0.020)
