"""Two geometry claims the kit had never written down.

**The standoff is measured from the grasp point.** ``Grasp`` travels to
``approach.grasp_point``, which for ``top_down`` is not the object's centre —
it is raised until the pad tips clear whatever the object is standing on.
``Approach`` stood off the CENTRE, so the two verbs disagreed about where the
descent corridor begins and the hand waited nearer the object than the number
in the call said. Shu, 2026-09-22: the jaws sat 1-2 cm over a 5 cm charger
after ``approach standoff 0.08``.

**A declared yaw turns the jaws.** ``grasp_orientation`` squares the jaws
across the object's long axis, so a box declared 30 degrees off the base axes
must produce a jaw axis 30 degrees round — on BOTH arms, whose top-down seeds
are mirror images — and the solved plan must actually put the wrist there.
That was asserted nowhere: every existing orientation test uses an
axis-aligned box, which is exactly the case a mirrored convention bug survives
(the 2026-09-08 wrist-camera incident).
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from manipulation_kit.primitives import Approach, Grasp
from manipulation_kit.primitives import approach as ap
from manipulation_kit.primitives.verbs import DEFAULT_STANDOFF_M
from manipulation_kit.world import ObjectView, WorldView

#: the declared yaw the test is about, and its degrees
DECLARED_YAW_RAD = 0.52
DECLARED_YAW_DEG = math.degrees(DECLARED_YAW_RAD)   # 29.794...


def _tool_orientation(kin, side: str, q):
    """FK the solved posture back to a TCP orientation, model restored."""
    saved = np.array(kin.joints(side), dtype=float)
    try:
        kin.set_joints(side, np.asarray(q, dtype=float))
        return ap.tool_from_link7(*kin.ee_pose(side))[1]
    finally:
        kin.set_joints(side, saved)


def _yawed(world: WorldView, name: str, yaw_rad: float) -> WorldView:
    return world.with_(objects=tuple(
        dataclasses.replace(o, r=R.from_euler("z", yaw_rad))
        if o.name == name else o for o in world.objects))


# --------------------------------------------------------------------------- #
# the standoff is measured from the grasp point
# --------------------------------------------------------------------------- #

def test_a_top_down_standoff_is_measured_from_the_grasp_point():
    """A 50 mm-tall object: the standoff tool point is grasp z + 80 mm.

    Not centre z + 80 mm. The two differ by the 7 mm ``grasp_point`` raises
    this object's descent by, and that 7 mm came off the descent.
    """
    block = ObjectView("charger", p=(0.40, -0.20, 0.025), size=(0.04, 0.03, 0.050))
    world = WorldView.of([block])
    grasp_p, raised = ap.grasp_point(block, "top_down", world.frames)
    assert raised, "a 50 mm object on a table IS raised — that is the case"

    stand = ap.standoff_pose(grasp_p, "top_down", 0.08)
    assert stand[2] == pytest.approx(float(grasp_p[2]) + 0.08, abs=1e-9)
    # and it is NOT the centre-derived number the old code produced
    centre_z = float(block.pose_in_base(world.frames)[0][2])
    assert stand[2] > centre_z + 0.08 + 1e-4


def test_approach_and_grasp_agree_about_where_the_descent_starts(d1_arm, observe):
    """The whole point of the change: ONE reference, so ``standoff_m`` is the
    millimetres of straight travel the ``Grasp`` that follows will make."""
    world = observe(d1_arm, block_p=(0.40, 0.22, 0.025),
                    block_size=(0.04, 0.03, 0.050))
    call = dict(object="red_block", side="left", approach="top_down",
                standoff_m=0.08)
    approach_stand = Approach(**call)._geometry(world)[1]
    _side, grasp_stand, grasp_p, _r, unmet = Grasp(**call)._geometry(world)
    assert not unmet
    assert np.allclose(approach_stand, grasp_stand)
    assert float(grasp_stand[2] - grasp_p[2]) == pytest.approx(0.08, abs=1e-9)


def test_the_standoff_of_a_tall_object_is_unchanged(d1_arm, observe):
    """A regression guard on the other half. An object tall enough that
    ``grasp_point`` does NOT raise it must stand off exactly as before —
    centre plus the standoff — or this change moved every existing plan."""
    world = observe(d1_arm, block_p=(0.40, 0.22, 0.08),
                    block_size=(0.04, 0.03, 0.12))
    block = world.find("red_block")
    _p_grasp, raised = ap.grasp_point(block, "top_down", world.frames)
    assert not raised
    stand = Approach(object="red_block", side="left")._geometry(world)[1]
    centre = block.pose_in_base(world.frames)[0]
    assert stand[2] == pytest.approx(float(centre[2]) + DEFAULT_STANDOFF_M,
                                     abs=1e-9)
    assert np.allclose(stand[:2], centre[:2])


# --------------------------------------------------------------------------- #
# a declared yaw turns the jaws, on both arms
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize("side", ["left", "right"])
def test_a_declared_yaw_turns_the_jaw_axis_by_the_same_angle(side):
    """30 degrees of declared box is 30 degrees of jaw axis, per arm.

    Signed in the horizontal plane, so a convention that mirrored the TURN as
    well as the seed would fail here: the two arms' seeds point opposite ways
    and the CORRECTION is the same rotation for both.
    """
    square = ObjectView("box", p=(0.40, 0.20, 0.05), size=(0.05, 0.03, 0.05))
    world = WorldView.of([square])
    yawed = _yawed(world, "box", DECLARED_YAW_RAD).find("box")

    axis0 = ap.jaw_axis(ap.grasp_orientation(side, "top_down", square,
                                             world.frames))
    axis1 = ap.jaw_axis(ap.grasp_orientation(side, "top_down", yawed,
                                             world.frames))
    turn = math.degrees(math.atan2(float(np.cross(axis0, axis1)[2]),
                                   float(np.dot(axis0, axis1))))
    assert turn == pytest.approx(DECLARED_YAW_DEG, abs=0.5)
    # both axes stay horizontal: a top-down grasp closes in the table plane
    assert abs(float(axis0[2])) < 1e-6 and abs(float(axis1[2])) < 1e-6


@pytest.mark.parametrize("side", ["left", "right"])
def test_the_solved_grasp_actually_puts_the_wrist_there(d1_arm, observe, side):
    """Not just the commanded quaternion — the SOLVED posture.

    An orientation the planner derives and the IK cannot reach is a
    derivation, not a grasp. Measured against the plan's own final joints, by
    the kit's forward kinematics: the achieved TCP rotates about its approach
    axis by the declared yaw, and the wrist roll is where that turn lives.
    """
    y = 0.20 if side == "left" else -0.20
    plans = {}
    for name, yaw in (("square", None), ("yawed", DECLARED_YAW_RAD)):
        for s in ("left", "right"):
            d1_arm.set_joints(s, d1_arm.home(s))
        world = observe(d1_arm, block_p=(0.40, y, 0.05),
                        block_size=(0.05, 0.03, 0.05))
        if yaw is not None:
            world = _yawed(world, "red_block", yaw)
        plan = Grasp(object="red_block", side=side).plan(world, d1_arm)
        assert getattr(plan, "ok", False), f"{name}: {plan}"
        plans[name] = np.asarray(plan.final_joints()[side], dtype=float)

    r0 = _tool_orientation(d1_arm, side, plans["square"])
    r1 = _tool_orientation(d1_arm, side, plans["yawed"])
    rotvec = (r0.inv() * r1).as_rotvec()
    # the whole difference is a ROLL about the tool's own approach axis (+z)
    assert math.degrees(abs(float(rotvec[2]))) == pytest.approx(
        DECLARED_YAW_DEG, abs=2.0)
    assert math.degrees(float(np.linalg.norm(rotvec[:2]))) < 2.0

    # ...and it is carried by the wrist roll, not by re-posing the shoulder
    delta = np.degrees(plans["yawed"] - plans["square"])
    assert abs(float(delta[5])) > 20.0
    assert float(np.max(np.abs(np.delete(delta, 5)))) < 10.0
