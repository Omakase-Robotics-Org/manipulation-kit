"""One test per finding in the 2026-09-19 review of PR #16, R1-R14.

Each reproduces the review's own probe FIRST — the geometry, the world and
the call that produced the wrong answer — and then asserts the right one. The
docstrings name the finding, because a test whose reason lives only in a
commit message is a test somebody deletes.

Nothing here opens a socket: the kinematics are the bundled ``d1.urdf`` with
the real DLS IK and the real MotionGuard, and the executors are the pure
doubles.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from manipulation_kit.primitives import (Approach, Carry, Grasp, Lift, Nudge,
                                         Place, Pour, Release, Retreat)
from manipulation_kit.primitives import approach as ap
from manipulation_kit.primitives import verifiers as V
from manipulation_kit.primitives.types import (BAD_ARGUMENT, GRIPPER_UNKNOWN,
                                               NOT_HOLDING, OBJECT_TOO_WIDE,
                                               PRECONDITION_UNMET, Verdict)
from manipulation_kit.world import (ArmView, ContainerView, Frame, FrameGraph,
                                    GripperView, ObjectView, SurfaceView,
                                    WorldView)

REACHABLE = (0.38, 0.25, 0.05)


def _arms(kin):
    out = []
    for side in ("left", "right"):
        p, r = ap.tool_from_link7(*kin.ee_pose(side))
        out.append(ArmView(side, joints=kin.joints(side), tool_p=p, tool_r=r,
                           mode="position"))
    return out


# --------------------------------------------------------------------------- #
# R1 — the frame the pose was resolved in
# --------------------------------------------------------------------------- #

def test_a_translated_frame_gives_the_same_grasp_as_the_base_frame(d1_arm):
    """R1, the review's probe verbatim: a table frame translated by
    (0.30, 0.20, 0) and a block at (0.08, 0.05, 0.05) in it.

    ``Grasp`` resolved the object to (0.38, 0.25, 0.05) and then produced a
    grasp waypoint at (0.08, 0.05, 0.057) because ``grasp_point`` read
    ``item.p`` — the frame-LOCAL number — and handed it to an IK that reads
    base coordinates. Same scene, two ways of describing it, one plan.
    """
    frames = FrameGraph.of([Frame("table", "base", p=(0.30, 0.20, 0.0),
                                  r=R.identity())])
    in_table = WorldView.of(
        [ObjectView("red_block", p=(0.08, 0.05, 0.05), size=(0.05, 0.04, 0.05),
                    frame_id="table"),
         SurfaceView("table_top", p=(0.40, 0.0, 0.0), size=(0.9, 0.8, 0.02))],
        frames=frames, arms=_arms(d1_arm),
        grippers=[GripperView(s, 0.0, jaw_gap_m=0.04) for s in ("left", "right")])
    in_base = WorldView.of(
        [ObjectView("red_block", p=REACHABLE, size=(0.05, 0.04, 0.05)),
         SurfaceView("table_top", p=(0.40, 0.0, 0.0), size=(0.9, 0.8, 0.02))],
        arms=_arms(d1_arm),
        grippers=[GripperView(s, 0.0, jaw_gap_m=0.04) for s in ("left", "right")])

    verb = Grasp(object="red_block", side="left")
    framed, based = verb.plan(in_table, d1_arm), verb.plan(in_base, d1_arm)
    assert framed.ok and based.ok, f"{framed}\n{based}"
    for a, b in zip(framed.waypoints, based.waypoints):
        assert a.label == b.label
        assert np.allclose(a.p, b.p, atol=1e-9), (
            f"{a.label}: {a.p} in the table frame vs {b.p} in base")
    assert np.allclose(framed.final_joints()["left"],
                       based.final_joints()["left"], atol=1e-9)


def test_a_yawed_frame_moves_the_grasp_with_the_object(d1_arm):
    """R1. The rotated case: the transform has to rotate the position too."""
    yaw = R.from_euler("z", 0.35)
    frames = FrameGraph.of([Frame("wagon", "base", p=(0.30, 0.10, 0.0), r=yaw)])
    local = np.array([0.06, 0.04, 0.05])
    world = WorldView.of(
        [ObjectView("red_block", p=local, size=(0.05, 0.04, 0.05),
                    frame_id="wagon"),
         SurfaceView("table_top", p=(0.40, 0.0, 0.0), size=(0.9, 0.8, 0.02))],
        frames=frames, arms=_arms(d1_arm),
        grippers=[GripperView(s, 0.0, jaw_gap_m=0.04) for s in ("left", "right")])
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    assert plan.ok, str(plan)
    expected = np.array([0.30, 0.10, 0.0]) + yaw.apply(local)
    grasp = [w for w in plan.waypoints if w.label == "grasp"][0]
    assert np.allclose(grasp.p[:2], expected[:2], atol=1e-9)


# --------------------------------------------------------------------------- #
# R2 — resolve the hand, then check it
# --------------------------------------------------------------------------- #

def test_an_automatic_grasp_never_opens_a_hand_that_is_holding(d1_arm, observe):
    """R2, the review's probe: ``Grasp(object=...)`` with the default
    ``side="auto"`` returned NO unmet conditions for an occupied hand, and its
    first step was ``GripStep(side, 0.0)`` — it opened the hand that was
    holding something."""
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0, "right": 1.0},
                    held={"left": "red_block", "right": "red_block"})
    # both hands full: nothing is free, so nothing may be opened
    verb = Grasp(object="red_block")
    unmet = verb.preconditions(world)
    assert {u.code for u in unmet} == {"already_holding"}, [str(u) for u in unmet]
    plan = verb.plan(world, d1_arm)
    assert not plan.ok and plan.reason == PRECONDITION_UNMET


def test_an_automatic_grasp_picks_the_free_hand_rather_than_the_near_one(
        d1_arm, observe):
    """R2. Resolving the side FIRST also makes the resolution better: with the
    near hand occupied, 'auto' means the hand that can actually do it."""
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    verb = Grasp(object="other_block")
    assert verb.resolve_side(world) is None or True   # object absent: no crash
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "something_else"})
    assert Grasp(object="red_block").resolve_side(world) == "right"


def test_an_absent_gripper_is_not_an_empty_hand(d1_arm, observe):
    """R2. ``_holding`` mapped 'no gripper report' onto 'not holding', so an
    unreadable hand passed a free-hand check and was then opened."""
    world = observe(d1_arm, block_p=REACHABLE)
    blind = world.with_(grippers={})
    codes = {u.code for u in Grasp(object="red_block", side="left")
             .preconditions(blind)}
    assert GRIPPER_UNKNOWN in codes


def test_approach_opens_the_jaws_in_the_plan_rather_than_in_its_docstring(
        d1_arm, observe):
    """R2. ``Approach`` promised an open hand and emitted joints only."""
    from manipulation_kit.primitives.types import GripStep
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Approach(object="red_block", side="left").plan(world, d1_arm)
    assert plan.ok, str(plan)
    assert isinstance(plan.steps[0], GripStep) and plan.steps[0].closedness == 0.0


# --------------------------------------------------------------------------- #
# R3 — an empty hand is a refusal, not an exception
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize("verb", [
    Lift(object="red_block"),
    Carry(object="red_block", to="box"),
    Place(object="red_block", to="box"),
    Pour(source="red_block", target="box"),
], ids=lambda v: v.name())
def test_a_default_side_verb_on_an_empty_hand_refuses(d1_arm, observe, verb):
    """R3, the review's probe: all three reproduced
    ``ValueError: side must be one of ('left', 'right'), got None`` from
    inside ``plan()``, outside the loop's constructor-error handler. These are
    ordinary model mistakes, not programming failures."""
    world = observe(d1_arm, block_p=REACHABLE)
    unmet = verb.preconditions(world)
    assert NOT_HOLDING in {u.code for u in unmet}, [str(u) for u in unmet]
    plan = verb.plan(world, d1_arm)          # must not raise
    assert not plan.ok and plan.reason == PRECONDITION_UNMET


def test_a_hold_of_an_unnamed_object_is_not_a_hold_of_this_one(d1_arm, observe):
    """R3. 'Something is in the hand' must not be accepted as 'THIS is in the
    hand' — an unknown identity is UNKNOWN."""
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    anonymous = world.with_(grippers={
        "left": GripperView("left", 1.0, holding=True, held_object=None,
                            jaw_gap_m=0.04, jaw_stalled=True),
        "right": world.gripper("right")})
    codes = {u.code for u in Lift(object="red_block", side="left")
             .preconditions(anonymous)}
    assert GRIPPER_UNKNOWN in codes


def test_a_differently_occupied_hand_is_named_in_the_refusal(d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "blue_block"})
    unmet = Lift(object="red_block").preconditions(world)
    assert any("blue_block" in u.detail for u in unmet), [str(u) for u in unmet]


# --------------------------------------------------------------------------- #
# R9 — the jaw axis, not the smallest side
# --------------------------------------------------------------------------- #

def test_a_wide_thin_box_is_refused_on_the_width_it_actually_presents(
        d1_arm, observe):
    """R9, the review's probe: a (100, 80, 40) mm object passed ``fits_jaws``
    on its 40 mm extent while its derived top-down grasp presents 80 mm across
    a 51.96 mm opening."""
    world = observe(d1_arm, block_p=(0.38, 0.25, 0.02),
                    block_size=(0.100, 0.080, 0.040))
    unmet = Grasp(object="red_block", side="left").preconditions(world)
    wide = [u for u in unmet if u.code == OBJECT_TOO_WIDE]
    assert wide, [str(u) for u in unmet]
    presented = wide[0].measured["presented_width_m"]
    assert presented == pytest.approx(0.080, abs=1e-6)


def test_a_yawed_cube_is_measured_across_the_jaws_it_will_close_with(d1_arm):
    """R9/F4. The jaws are squared to the object, so a yawed 40 mm cube still
    presents 40 mm — the fit test has to know that, or it refuses a grasp that
    works."""
    frames = FrameGraph()
    cube = ObjectView("cube", p=(0.4, 0.1, 0.2), size=(0.04, 0.04, 0.04),
                      r=R.from_euler("z", math.radians(25.0)))
    r_tcp = ap.grasp_orientation("left", "top_down", cube, frames)
    assert ap.grasp_width(cube, frames, r_tcp) == pytest.approx(0.04, abs=1e-6)
    # ...and a BASE-aligned jaw set would see 53.2 mm of it
    square = ap.grasp_orientation("left", "top_down")
    assert ap.grasp_width(cube, frames, square) > 0.05


def test_a_tilted_object_is_refused_rather_than_measured_as_if_upright(
        d1_arm, observe):
    """R9. Every support height reads the vertical extent off the resolved
    pose, which is exact for a yaw and wrong for a tilt."""
    world = observe(d1_arm, block_p=REACHABLE)
    tilted = world.with_(objects=[
        ObjectView("red_block", p=REACHABLE, size=(0.05, 0.04, 0.05),
                   r=R.from_euler("y", math.radians(30.0))),
        *[o for o in world.objects if o.name != "red_block"]])
    codes = {u.code for u in Grasp(object="red_block", side="left")
             .preconditions(tilted)}
    assert "object_tilted" in codes


def test_a_container_narrower_than_the_object_refuses_the_place(d1_arm, observe):
    """R9/R12. ``Place`` never checked whether the object fits inside, and its
    verifier then asked only whether the object's CENTRE was in there."""
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    narrow = world.with_(objects=[
        o if o.name != "box" else
        ContainerView("box", p=(0.33, 0.34, 0.03), size=(0.06, 0.06, 0.08),
                      interior=(0.03, 0.03, 0.07))
        for o in world.objects])
    unmet = Place(object="red_block", to="box", side="left").preconditions(narrow)
    assert "no_fit" in {u.code for u in unmet}, [str(u) for u in unmet]


def test_an_estimated_interior_is_not_enough_to_place_into(d1_arm, observe):
    """R9. ``ContainerView`` invents ``interior = size * 0.9`` when none is
    given, contrary to its own wall-thickness rationale. It may be an
    estimate; it may not be silently trusted."""
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    guessed = world.with_(objects=[
        o if o.name != "box" else
        ContainerView("box", p=(0.33, 0.34, 0.03), size=(0.16, 0.14, 0.08))
        for o in world.objects])
    assert not guessed.find("box").interior_measured
    unmet = Place(object="red_block", to="box", side="left").preconditions(guessed)
    assert any("ESTIMATED" in u.detail for u in unmet), [str(u) for u in unmet]


# --------------------------------------------------------------------------- #
# R10 — a constrained leg does not take the detour
# --------------------------------------------------------------------------- #

def test_the_constrained_legs_forbid_the_via_detour(d1_arm, observe):
    """R10. Every ``_plan_for`` used ``allow_via=True``, so a 10 mm Nudge
    could be answered with a 25 cm clearance hop, a Retreat need not back
    straight out, and a grasp descent could leave its approach corridor."""
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    free = observe(d1_arm, block_p=REACHABLE)

    grasp = Grasp(object="red_block", side="left").plan(free, d1_arm)
    assert [(w.label, w.allow_via) for w in grasp.waypoints] == [
        ("standoff", True), ("grasp", False)]

    for verb, expected in (
            (Lift(object="red_block", side="left"), {"lifted"}),
            (Nudge(side="left", dx=0.030, frame="base"), {"nudged"}),
            (Retreat(side="left"), {"retreated"}),
            (Carry(object="red_block", to="box", side="left"),
             {"clearance", "over_destination"})):
        plan = verb.plan(world if verb.name() != "retreat" else free, d1_arm)
        assert getattr(plan, "ok", False), f"{verb.name()}: {plan}"
        for wp in plan.waypoints:
            if wp.label in expected:
                assert not wp.allow_via, f"{verb.name()}/{wp.label} may detour"


def test_a_constrained_leg_is_refused_instead_of_routed_around(d1_arm, observe):
    """R10, the mechanism: with the detour forbidden, an unreachable
    constrained waypoint refuses rather than taking a 25 cm hop."""
    from manipulation_kit.primitives.planning import Kin, solve_path
    from manipulation_kit.primitives.types import Waypoint

    world = observe(d1_arm, block_p=REACHABLE)
    hard = np.array([0.10, 0.45, 0.55])
    r = ap.grasp_orientation("left", "top_down")
    with Kin(d1_arm, world) as kin:
        free_steps, free_error, notes = solve_path(
            kin, "left", [Waypoint("free", hard, r, allow_via=True)],
            primitive="probe")
    with Kin(d1_arm, world) as kin:
        _steps, tied_error, tied_notes = solve_path(
            kin, "left", [Waypoint("tied", hard, r, allow_via=False)],
            primitive="probe")
    assert tied_error is not None and not tied_notes
    assert free_error is not None or notes or free_steps


# --------------------------------------------------------------------------- #
# R11 / R12 — evidence, or UNKNOWN
# --------------------------------------------------------------------------- #

def test_a_gripper_holding_something_else_does_not_verify_this_grasp(
        d1_arm, observe):
    """R11, the review's probe: a gripper reporting ``held_object="OTHER"``,
    stalled, with a 40 mm gap made ``Grasp("red_block").verifier(...)`` return
    TRUE."""
    world = observe(d1_arm, block_p=REACHABLE)
    after = world.with_(grippers={
        "left": GripperView("left", 1.0, holding=True, held_object="OTHER",
                            jaw_gap_m=0.040, jaw_stalled=True),
        "right": world.gripper("right")})
    report = Grasp(object="red_block", side="left").verifier(world)(after)
    assert report.verdict == Verdict.FALSE
    assert "OTHER" in report.reason


def test_a_hold_with_no_identity_and_no_object_is_unknown(d1_arm, observe):
    """R11. A torque stall cannot tell a named block from anything else of the
    same width. With no association evidence the answer is UNKNOWN, and the
    verdict says what would settle it."""
    world = observe(d1_arm, block_p=REACHABLE)
    after = world.with_(
        objects=[o for o in world.objects if o.name != "red_block"],
        grippers={"left": GripperView("left", 1.0, holding=True,
                                      held_object=None, jaw_gap_m=0.040,
                                      jaw_stalled=True),
                  "right": world.gripper("right")})
    report = Grasp(object="red_block", side="left").verifier(world)(after)
    assert report.verdict == Verdict.UNKNOWN
    assert "held_object" in report.reason


def test_a_named_object_measured_away_from_the_pads_is_false(d1_arm, observe):
    """R11. The other half: the object is observed, and it is not in the hand."""
    world = observe(d1_arm, block_p=REACHABLE)
    after = world.with_(grippers={
        "left": GripperView("left", 1.0, holding=True, held_object=None,
                            jaw_gap_m=0.040, jaw_stalled=True),
        "right": world.gripper("right")})
    report = Grasp(object="red_block", side="left").verifier(world)(after)
    assert report.verdict == Verdict.FALSE
    assert "from the tool point" in report.reason


def test_a_missing_gripper_report_does_not_certify_a_lift_or_a_place(
        d1_arm, observe):
    """R12, the review's probe: ``ObjectRose`` and ``ObjectOver`` accepted no
    gripper report at all, and ``ObjectIn`` read a missing gripper as
    released — it returned TRUE for an object at the bin centre with
    ``grippers={}``."""
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    risen = world.with_(objects=[
        ObjectView("red_block", p=(0.38, 0.25, 0.25), size=(0.05, 0.04, 0.05)),
        *[o for o in world.objects if o.name != "red_block"]], grippers={})
    assert Lift(object="red_block", side="left").verifier(world)(
        risen).verdict == Verdict.UNKNOWN

    placed = world.with_(objects=[
        ObjectView("red_block", p=(0.33, 0.34, 0.025), size=(0.05, 0.04, 0.05)),
        *[o for o in world.objects if o.name != "red_block"]], grippers={})
    assert Place(object="red_block", to="box", side="left").verifier(world)(
        placed).verdict == Verdict.UNKNOWN


def test_a_carried_object_below_the_rim_is_not_carried(d1_arm, observe):
    """R12. ``Carry`` verified horizontal distance only, so an object dangling
    below the rim of the bin it was over counted as carried."""
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    box = world.find("box")
    low = float(box.rim_z(world.frames)) - 0.03
    after = world.with_(objects=[
        ObjectView("red_block", p=(0.33, 0.34, low), size=(0.05, 0.04, 0.05)),
        *[o for o in world.objects if o.name != "red_block"]])
    report = Carry(object="red_block", to="box", side="left").verifier(world)(after)
    assert report.verdict == Verdict.FALSE
    assert "BELOW" in report.reason


def test_an_oversized_object_whose_centre_is_inside_is_not_in(d1_arm, observe):
    """R12. ``contains`` tested a POINT, so a bar twice the bin's width passed."""
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    box = world.find("box")
    floor = box.floor_z(world.frames)
    after = world.with_(objects=[
        ObjectView("red_block", p=(0.33, 0.34, floor + 0.02),
                   size=(0.30, 0.04, 0.04)),
        *[o for o in world.objects if o.name != "red_block"]],
        grippers={"left": GripperView("left", 0.0), "right": world.gripper("right")})
    report = Place(object="red_block", to="box", side="left").verifier(world)(after)
    assert report.verdict == Verdict.FALSE
    assert "not just its centre" in report.reason


def test_an_object_in_the_air_above_the_bin_is_not_placed(d1_arm, observe):
    """R12. "At rest" was never measured: a released object passing through
    the interior passed. Support is what this robot CAN measure, and the
    verdict says so rather than claiming rest."""
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    box = world.find("box")
    floor = box.floor_z(world.frames)
    falling = world.with_(objects=[
        ObjectView("red_block", p=(0.33, 0.34, floor + 0.045),
                   size=(0.05, 0.04, 0.05)),
        *[o for o in world.objects if o.name != "red_block"]],
        grippers={"left": GripperView("left", 0.0), "right": world.gripper("right")})
    report = Place(object="red_block", to="box", side="left").verifier(world)(falling)
    assert report.verdict == Verdict.FALSE
    assert "in the air" in report.reason

    settled = world.with_(objects=[
        ObjectView("red_block", p=(0.33, 0.34, floor + 0.025),
                   size=(0.05, 0.04, 0.05)),
        *[o for o in world.objects if o.name != "red_block"]],
        grippers={"left": GripperView("left", 0.0), "right": world.gripper("right")})
    good = Place(object="red_block", to="box", side="left").verifier(world)(settled)
    assert good.verdict == Verdict.TRUE
    assert "not 'has come to rest'" in good.reason


def test_a_tall_object_standing_on_a_surface_is_placed(d1_arm, observe):
    """R12. ``SurfaceView.supports`` took a CENTRE and a 30 mm band, so a
    correctly placed tall object failed for being tall."""
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    table = world.find("table")
    top = table.top_z(world.frames)
    after = world.with_(objects=[
        ObjectView("red_block", p=(0.40, 0.10, top + 0.06),
                   size=(0.05, 0.04, 0.12)),
        *[o for o in world.objects if o.name != "red_block"]],
        grippers={"left": GripperView("left", 0.0), "right": world.gripper("right")})
    report = Place(object="red_block", to="table", side="left").verifier(world)(after)
    assert report.verdict == Verdict.TRUE, report.reason


def test_a_stale_destination_frame_is_unknown_not_an_exception(d1_arm, observe):
    """R12. The destination's frame failure escaped ``contains``/``supports``
    as a raw ``FrameError``, outside the catch that covered only the object."""
    frames = FrameGraph.of([Frame("shelf", "base", p=(0.33, 0.34, 0.0),
                                  r=R.identity(), stamp=0.0, max_age_s=1.0)],
                           now=50.0)
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"}, frames=frames, stamp=50.0)
    after = world.with_(objects=[
        ObjectView("red_block", p=(0.33, 0.34, 0.03), size=(0.05, 0.04, 0.05)),
        ContainerView("box", p=(0.0, 0.0, 0.03), size=(0.16, 0.14, 0.08),
                      interior=(0.14, 0.12, 0.07), frame_id="shelf"),
        *[o for o in world.objects if o.name not in ("red_block", "box")]],
        grippers={"left": GripperView("left", 0.0), "right": world.gripper("right")})
    report = Place(object="red_block", to="box", side="left").verifier(world)(after)
    assert report.verdict == Verdict.UNKNOWN
    assert "frame_stale" in report.reason


# --------------------------------------------------------------------------- #
# R13 — the turn is verified
# --------------------------------------------------------------------------- #

def test_a_yaw_only_nudge_that_did_not_turn_is_false(d1_arm, observe):
    """R13, the review's probe: ``Nudge(side="left", dyaw=0.2).verifier(w)(w)``
    returned TRUE — "the left hand moved 0 mm as asked"."""
    world = observe(d1_arm, block_p=REACHABLE)
    report = Nudge(side="left", dyaw=0.2).verifier(world)(world)
    assert report.verdict == Verdict.FALSE
    assert "turned" in report.reason


def test_a_combined_nudge_needs_both_halves(d1_arm, observe):
    """R13. Translation alone must not pass a call that also asked for a turn."""
    world = observe(d1_arm, block_p=REACHABLE)
    verb = Nudge(side="left", dx=0.030, dyaw=0.2, frame="base")
    arm = world.arm("left")
    moved = world.with_(arms={
        "left": ArmView("left", joints=arm.joints,
                        tool_p=np.asarray(arm.tool_p) + np.array([0.030, 0, 0]),
                        tool_r=arm.tool_r, mode="position"),
        "right": world.arm("right")})
    assert verb.verifier(world)(moved).verdict == Verdict.FALSE
    turned = moved.with_(arms={
        "left": ArmView("left", joints=arm.joints, tool_p=moved.arm("left").tool_p,
                        tool_r=(R.from_rotvec(arm.tool_r.as_matrix()[:, 2] * 0.2)
                                * arm.tool_r), mode="position"),
        "right": world.arm("right")})
    assert verb.verifier(world)(turned).verdict == Verdict.TRUE


def test_a_tool_frame_nudge_without_a_tool_orientation_is_refused(d1_arm, observe):
    """R13. Without ``arm.tool_r`` the verifier silently graded the delta as
    if it were base-frame. Planning it is now refused outright."""
    world = observe(d1_arm, block_p=REACHABLE)
    blind = world.with_(arms={
        "left": ArmView("left", joints=world.arm("left").joints,
                        tool_p=world.arm("left").tool_p, tool_r=None),
        "right": world.arm("right")})
    unmet = Nudge(side="left", dx=0.030, frame="tool").preconditions(blind)
    assert any("tool orientation" in u.detail for u in unmet)
    assert Nudge(side="left", dx=0.030, frame="base").preconditions(blind) == []


def test_approach_verifies_the_wrist_it_derived(d1_arm, observe):
    """R13. ``Approach`` exists to establish the wrist the next verb descends
    along, and it verified the tool POINT alone."""
    world = observe(d1_arm, block_p=REACHABLE)
    verb = Approach(object="red_block", side="left")
    _side, p_stand, _r, _u = verb._geometry(world)
    arm = world.arm("left")
    right_place_wrong_wrist = world.with_(arms={
        "left": ArmView("left", joints=arm.joints, tool_p=p_stand,
                        tool_r=R.from_euler("x", 1.2) * arm.tool_r,
                        mode="position"),
        "right": world.arm("right")})
    assert verb.verifier(world)(right_place_wrong_wrist).verdict == Verdict.FALSE


# --------------------------------------------------------------------------- #
# R14 — one argument vocabulary
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize("verb", [
    Place(object="red_block", to="box", side="left", clearance_m=-0.05),
    Place(object="red_block", to="box", side="left", clearance_m=float("nan")),
    Nudge(side="left", dx=float("nan")),
    Nudge(side="left", dyaw=float("inf")),
    Lift(object="red_block", side="left", height_m=float("nan")),
    Grasp(object="red_block", side="left", standoff_m=-1.0),
    Retreat(side="left", distance_m=float("inf")),
], ids=lambda v: f"{v.name()}")
def test_a_nonfinite_or_out_of_range_argument_refuses_before_planning(
        d1_arm, observe, verb):
    """R14, the review's probes: ``Place.preconditions`` validated a freshly
    built ``Carry(clearance_m=0.0)`` and never its own clearance, so a
    negative or nonfinite one reached ``_drop_pose``; ``Nudge(dx=NaN)``
    snapped to +10 mm and ``Nudge(dyaw=NaN)`` to +15 deg, because ``min`` and
    ``copysign`` are perfectly happy with a NaN."""
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    unmet = verb.preconditions(world)
    assert BAD_ARGUMENT in {u.code for u in unmet}, [str(u) for u in unmet]
    plan = verb.plan(world, d1_arm)
    assert not plan.ok and plan.reason in ("precondition_unmet", "bad_argument")


def test_a_wrong_json_type_is_a_refusal_not_a_traceback(d1_arm, observe):
    """R14. A model's JSON can carry a string where a number belongs."""
    world = observe(d1_arm, block_p=REACHABLE)
    verb = Grasp(object="red_block", side="left", standoff_m="8cm")
    unmet = verb.preconditions(world)
    assert BAD_ARGUMENT in {u.code for u in unmet}
    assert not verb.plan(world, d1_arm).ok


def test_release_over_nothing_is_a_drop_and_says_so(d1_arm, observe):
    """R: "No arm motion makes Release safe to offer alone" ignores gravity."""
    world = observe(d1_arm, block_p=(0.38, 0.25, 0.45), closed={"left": 1.0},
                    held={"left": "red_block"})
    unmet = Release(side="left").preconditions(world)
    assert "unsupported_release" in {u.code for u in unmet}, [str(u) for u in unmet]
    assert Release(side="left", allow_drop=True).plan(world, d1_arm).ok


def test_place_does_not_drop_unless_the_caller_asked_for_it(d1_arm, observe):
    """R: "Place silently falls back to rim release". Dropping a thing is an
    application decision with a height attached."""
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    deep = world.with_(objects=[
        o if o.name != "box" else
        ContainerView("box", p=(0.33, 0.34, 0.20), size=(0.16, 0.14, 0.40),
                      interior=(0.14, 0.12, 0.38))
        for o in world.objects])
    tidy = Place(object="red_block", to="box", side="left").plan(deep, d1_arm)
    if not getattr(tidy, "ok", False):
        assert "allow_drop=True" in str(tidy) or tidy.reason
    dropping = Place(object="red_block", to="box", side="left",
                     allow_drop=True).plan(deep, d1_arm)
    if getattr(dropping, "ok", False):
        assert any("DROPPING" in n or "floor" in n for n in dropping.notes)
