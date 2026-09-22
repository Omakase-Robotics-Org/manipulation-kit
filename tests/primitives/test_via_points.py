"""Routing around a guard-rejected knot, against the real URDF and guard.

The scene is the one that found the bug: ``blocks-eval`` on the measured JP
wagon (d1-isaaclab #53), three cubes on the top, the robot at HOME. Every
position here is a GROUND-TRUTH pose read off the served scene's ``objects``
reply in the ``dual_base`` frame (``artifacts/blocks-eval-20260919``), so
"this plans" means the thing the harness asks for plans, not a convenient
point chosen to make a test pass.

What it was: the standoff and the grasp pose are both guard-CLEAN, and the
straight line from HOME to them puts ``Link4_R`` inside ``torso_belly`` at
knot 1-3. ``solve_path`` refused at the first rejected knot, the offer filter
deleted the grasp from the model's prompt, and the first agent-eval run lost
61 of 61 turn-0 approaches to a goal the arm can hold perfectly well.

Not a mock anywhere: the bundled ``d1.urdf``, the real DLS IK, MotionGuard.
"""

from __future__ import annotations

import numpy as np
import pytest

from manipulation_kit.world.direction import ALIASES

from manipulation_kit.arms import safety
from manipulation_kit.primitives import Approach, Carry, Grasp
from manipulation_kit.primitives import planning
from manipulation_kit.primitives.orientation import tool_from_link7
from manipulation_kit.primitives.planning import Kin, solve_path
from manipulation_kit.primitives.types import GUARD_REJECT, Waypoint
from manipulation_kit.world import (ArmView, ContainerView, GripperView,
                                    ObjectView, SurfaceView, WorldView)

#: the base-frame "down" vector the orientation helpers take
DOWN = ALIASES["down"].vector()

#: The three cubes, in the base frame, as the served scene reports them.
CUBES = {"block_red": (0.4651, -0.0966, 0.195),
         "block_blue": (0.4507, 0.0051, 0.195),
         "block_yellow": (0.4708, 0.1100, 0.195)}
#: a bin on the far half of the wagon top
BOX_RED = (0.6288, 0.1576, 0.200)
WAGON_TOP = (0.5641, 0.0018, 0.169)
#: 40 mm, after d1-isaaclab sized the cube to the driven jaws
CUBE_SIZE = (0.040, 0.040, 0.040)

#: Far out and almost on the centre line, where the arm has to reach across
#: itself. Measured: of 144 candidate clearance points swept for this target,
#: NONE lets the grasp descent through, and neither does the READY re-seed.
#: It is here so "the via search fixed everything" cannot quietly become
#: "the via search says yes to everything".
NO_VIA_REACHES = (0.49, -0.01, 0.195)
#: A cube the READY re-seed can rescue with the clearance points removed —
#: the mechanism of last resort, exercised on its own.
READY_RESCUES = (0.43, 0.07, 0.195)


def _scene(kin, cubes, *, held=None, closed=None):
    """A WorldView of the blocks-eval top, read off the kinematic mirror."""
    held = held or {}
    closed = closed or {}
    arms, grippers = [], []
    for side in ("left", "right"):
        p, r = tool_from_link7(*kin.ee_pose(side))
        arms.append(ArmView(side, joints=kin.joints(side), tool_p=p, tool_r=r,
                            mode="position"))
        grippers.append(GripperView(side, closed.get(side, 0.0),
                                    holding=held.get(side) is not None,
                                    held_object=held.get(side), jaw_gap_m=None))
    props = [ObjectView(name, p=p, size=CUBE_SIZE, colour=name.split("_")[1])
             for name, p in cubes.items()]
    props.append(ContainerView("box_red", p=BOX_RED, size=(0.15, 0.15, 0.06),
                               interior=(0.146, 0.146, 0.056)))
    props.append(SurfaceView("wagon_top", p=WAGON_TOP, size=(0.4, 0.6, 0.002)))
    return WorldView.of(props, arms=arms, grippers=grippers, stamp=0.0)


def _tool(kin, side, q):
    saved = kin.joints(side)
    try:
        kin.set_joints(side, q)
        return tool_from_link7(*kin.ee_pose(side))[0]
    finally:
        kin.set_joints(side, saved)


@pytest.mark.parametrize("name", sorted(CUBES))
def test_a_top_down_grasp_over_the_wagon_plans_from_home(d1_arm, name):
    """The case the harness could not offer. All three cubes, both arms."""
    world = _scene(d1_arm, CUBES)
    plan = Grasp(object=name).plan(world, d1_arm)
    assert plan.ok, str(plan)
    steps = plan.joint_steps()
    assert steps
    # it went AROUND something, and says so in its own record
    assert any("routed via a clearance point" in note or "READY" in note
               for note in plan.notes), plan.notes
    # ...and it still ends where the plan said: a detour that does not arrive
    # is not a plan, it is a wander
    tool = _tool(d1_arm, plan.side, steps[-1].q)
    assert np.linalg.norm(tool - plan.waypoints[-1].p) < 0.01
    # the grasp point is over the cube, and lifted just clear of the table it
    # stands on rather than driven through it
    centre = np.array(CUBES[name])
    assert np.linalg.norm(tool[:2] - centre[:2]) < 0.01
    from manipulation_kit.primitives import orientation as ap
    assert tool[2] == pytest.approx(
        centre[2] - CUBE_SIZE[2] / 2 + ap.TIP_BELOW_TOOL_M
        + ap.SUPPORT_CLEARANCE_M, abs=0.005)


def test_the_same_grasp_is_refused_with_the_detour_switched_off(d1_arm):
    """The via IS what fixed it — not the scene, not the cube size, not luck.

    Same world, same waypoints, ``allow_via=False``: the straight line dies
    exactly where the 2026-09-19 run said it did, on the guard, in the first
    handful of knots.
    """
    world = _scene(d1_arm, CUBES)
    verb = Grasp(object="block_blue")
    side, p_stand, p_grasp, r_tcp, unmet = verb._geometry(world)
    assert not unmet
    waypoints = [Waypoint("standoff", p_stand, r_tcp),
                 Waypoint("grasp", p_grasp, r_tcp)]
    with Kin(d1_arm, world) as borrowed:
        steps, error, notes = solve_path(borrowed, side, waypoints,
                                         primitive="grasp", allow_via=False)
    assert error is not None and error.reason == GUARD_REJECT
    assert error.waypoint_index == 0 and error.waypoint_label == "standoff"
    assert notes == []
    # it did not even get close before the guard stopped it
    assert len(steps) < 10


def test_a_target_no_via_reaches_is_still_refused_and_says_where(d1_arm):
    """A planner that always finds a way is a planner that does not check.

    The refusal is the STRAIGHT line's — same reason, same waypoint index,
    a finite residual — so a dead end looks exactly as it did before the via
    search existed, and a caller can still tell the model why.
    """
    world = _scene(d1_arm, {"block_far": NO_VIA_REACHES})
    error = Grasp(object="block_far").plan(world, d1_arm)
    assert not error.ok
    assert error.reason == GUARD_REJECT
    assert error.waypoint_index >= 0 and error.waypoint_label in ("standoff",
                                                                 "grasp")
    assert np.isfinite(error.residual_m)
    assert "motion guard" in error.detail
    # Approach alone gets further than the descent does, which is the honest
    # answer: the standoff is routable and the grasp point is not.
    assert Approach(object="block_far").plan(world, d1_arm).ok


def test_the_ready_reseed_is_the_last_resort_when_no_clearance_point_works(
        d1_arm, monkeypatch):
    """The second mechanism, on its own.

    With the clearance points removed the only thing left is re-seeding from
    the arm's searched READY posture, and it still plans this cube — so the
    fallback is load-bearing rather than decorative.
    """
    monkeypatch.setattr(planning, "VIA_OFFSETS_M", ())
    world = _scene(d1_arm, {"block_near": READY_RESCUES})
    plan = Grasp(object="block_near").plan(world, d1_arm)
    assert plan.ok, str(plan)
    assert any("READY" in note for note in plan.notes), plan.notes
    tool = _tool(d1_arm, plan.side, plan.joint_steps()[-1].q)
    assert np.linalg.norm(tool - plan.waypoints[-1].p) < 0.01


def test_a_detour_never_exceeds_the_per_tick_caps(d1_arm):
    """The whole point of a plan is that every step is already small enough.

    ``MAX_JOINT_STEP_RAD`` is the HARD cap — it is what ``solve_ee`` clamps
    to, and it holds exactly. The Cartesian ``MAX_STEP_M`` is the knot
    SPACING the path is built from; one clamped joint step can carry the tool
    a few percent past it when the Jacobian is favourable, on a detour and on
    a straight line alike, so it is asserted with that margin stated rather
    than pretended away.
    """
    world = _scene(d1_arm, CUBES)
    for name in sorted(CUBES):
        plan = Grasp(object=name).plan(world, d1_arm)
        assert plan.ok, str(plan)
        q = world.arm(plan.side).joints
        previous = None
        for step in plan.joint_steps():
            assert np.max(np.abs(step.q - q)) <= safety.MAX_JOINT_STEP_RAD + 1e-9
            q = step.q
            tool = _tool(d1_arm, plan.side, step.q)
            if previous is not None:
                assert (np.linalg.norm(tool - previous)
                        <= safety.MAX_STEP_M * 1.1)
            previous = tool


def test_the_knot_generator_still_bounds_a_detour_leg(d1_arm):
    """The leg TO a clearance point is knotted by the same rule as any other:
    no commanded knot is further than one tick from the last."""
    from scipy.spatial.transform import Rotation as R

    p0, r0 = np.array([0.30, 0.20, 0.20]), R.identity()
    p1 = p0 + np.array([0.0, 0.25, 0.25])
    previous = p0
    for p_knot, _r_knot in planning.knots(p0, r0, p1, r0):
        assert np.linalg.norm(p_knot - previous) <= safety.MAX_STEP_M + 1e-9
        previous = p_knot
    assert np.allclose(previous, p1)


def test_planning_stays_pure_and_deterministic_through_a_detour(d1_arm):
    """A via search poses the mirror many times over. If one attempt leaked,
    the next plan would depend on the last one — and ``plan()`` is a value."""
    world = _scene(d1_arm, CUBES)
    before = {side: np.array(d1_arm.joints(side)) for side in ("left", "right")}
    first = Grasp(object="block_red").plan(world, d1_arm)
    for side, q in before.items():
        assert np.allclose(d1_arm.joints(side), q), f"{side} arm was left moved"
    again = Grasp(object="block_red").plan(world, d1_arm)
    assert first.ok and again.ok
    assert first.notes == again.notes
    assert np.allclose([s.q for s in first.joint_steps()],
                       [s.q for s in again.joint_steps()])


def test_carry_gets_the_detour_too(d1_arm):
    """Approach, Grasp and Carry share one planner, so they share one fix."""
    world = _scene(d1_arm, CUBES, held={"left": "block_blue"},
                   closed={"left": 1.0})
    plan = Carry(object="block_blue", to="box_red", side="left").plan(
        world, d1_arm)
    assert plan.ok, str(plan)
    assert plan.joint_steps()


def test_the_jaws_are_squared_to_a_yawed_cube_not_to_the_base_frame(d1_arm):
    """The grasp is planned across a FACE, whatever the cube's yaw.

    A 40 mm cube fits the driven jaws with 2 mm a side; the same cube presented
    corner-first does not, and the pads stall on two corners holding nothing —
    which is what five blocks-eval trials measured before the kit started
    reading the object's own footprint axis.
    """
    from scipy.spatial.transform import Rotation as R

    from manipulation_kit.primitives import orientation as ap

    for yaw in (0.0, 11.7, 25.0):
        cube = ObjectView("cube", p=(0.44, -0.05, 0.19), size=CUBE_SIZE,
                          r=R.from_euler("z", yaw, degrees=True), colour="red")
        world = WorldView.of(
            [cube, SurfaceView("wagon_top", p=WAGON_TOP, size=(0.4, 0.6, 0.002))],
            arms=[ArmView(s, joints=d1_arm.joints(s)) for s in ("left", "right")],
            grippers=[GripperView(s, 0.0) for s in ("left", "right")])
        r_tcp = ap.grasp_orientation("right", DOWN, cube, world.frames)
        gap = r_tcp.as_matrix()[:, 0]
        half = cube.axes_in_base(world.frames) * np.array(cube.size) / 2.0
        width = 2 * sum(abs(float(np.dot(half[:, i], gap))) for i in range(3))
        assert width == pytest.approx(CUBE_SIZE[0], abs=1e-6), (
            f"a cube yawed {yaw} deg presents {width * 1000:.1f} mm to the jaws")
        assert width <= ap.JAW_OPEN_M - 2 * ap.JAW_CLEARANCE_M
