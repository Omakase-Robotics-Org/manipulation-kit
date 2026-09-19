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
    assert any(u.code == "bad_frame" for u in verb.preconditions(world))
