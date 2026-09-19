"""The truth tables. TRUE needs evidence, UNKNOWN is a real answer, and an
executor that did nothing can never pass.

``G6`` in the design note: "every primitive's verifier returns FALSE/UNKNOWN —
never TRUE — when the executor did nothing". That is the whole reason verifiers
are built from the world BEFORE and called with the world after.
"""

from __future__ import annotations

import pytest
from scipy.spatial.transform import Rotation as R

from manipulation_kit.primitives import (Approach, Carry, GoHome, Grasp, Lift,
                                         Nudge, Place, Pour, Release, Retreat,
                                         Verdict)
from manipulation_kit.world import GripperView, ObjectView

REACHABLE = (0.38, 0.25, 0.05)

ALL_VERBS = [
    Approach(object="red_block", side="left"),
    Grasp(object="red_block", side="left"),
    Lift(object="red_block", side="left"),
    Carry(object="red_block", to="box", side="left"),
    Place(object="red_block", to="box", side="left"),
    Release(side="left"),
    Nudge(side="left", dz=0.03, frame="base"),
    Retreat(side="left"),
    GoHome(),
    Pour(source="red_block", target="box", side="left"),
]

#: Which verbs only make sense with something already in the hand. G6 has to
#: start from a world where the POSTCONDITION IS NOT ALREADY TRUE, or it tests
#: nothing: a Grasp verifier handed a hand that is already holding is right to
#: say TRUE, and a GoHome verifier handed arms already at HOME likewise.
NEEDS_A_HELD_OBJECT = {"lift", "carry", "place", "release", "pour"}


def _before(verb, d1_arm, observe):
    """The world a verb would be issued FROM, with its postcondition unmet."""
    if verb.name() == "go_home":
        for side in ("left", "right"):
            d1_arm.set_joints(side, d1_arm.home(side) + 0.3)
        return observe(d1_arm, block_p=REACHABLE)
    if verb.name() in NEEDS_A_HELD_OBJECT:
        return observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                       held={"left": "red_block"})
    return observe(d1_arm, block_p=REACHABLE)


@pytest.mark.parametrize("verb", ALL_VERBS, ids=lambda v: v.name())
def test_nothing_happened_is_never_true(d1_arm, observe, verb):
    """G6. The same world in and out: the only honest answers are FALSE and
    UNKNOWN, and a verifier that says TRUE here is grading its own homework."""
    world = _before(verb, d1_arm, observe)
    report = verb.verifier(world)(world)
    assert report.verdict in (Verdict.FALSE, Verdict.UNKNOWN), (
        f"{verb.name()} said {report.verdict} with nothing moved: {report.reason}")


@pytest.mark.parametrize("verb", ALL_VERBS, ids=lambda v: v.name())
def test_a_recording_executor_that_sent_the_plan_still_fails(d1_arm, observe, verb):
    """The same claim, made through the machinery instead of by hand: the
    RecordingExecutor accepts every step and moves nothing."""
    from manipulation_kit.executor import RecordingExecutor, run
    world = _before(verb, d1_arm, observe)
    plan = verb.plan(world, d1_arm)
    executor = RecordingExecutor()
    if getattr(plan, "ok", False):
        report = run(plan, executor)
        assert report.completed
        assert executor.sent or executor.grips, (
            f"{verb.name()} produced a plan that sent nothing at all")
    verdict = verb.verifier(world)(world).verdict
    assert verdict != Verdict.TRUE


def test_grasp_is_true_only_when_the_gripper_itself_reports_holding(d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE)
    verifier = Grasp(object="red_block", side="left").verifier(world)
    closed_empty = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0})
    assert verifier(closed_empty).verdict == Verdict.FALSE
    holding = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                      held={"left": "red_block"})
    assert verifier(holding).verdict == Verdict.TRUE


def test_a_gripper_stalled_on_itself_is_not_holding_the_object(d1_arm, observe):
    """``holding`` alone is not enough: a gripper that closed all the way is
    stalled against its own pads."""
    world = observe(d1_arm, block_p=REACHABLE)
    verifier = Grasp(object="red_block", side="left").verifier(world)
    after = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    after = after.with_(grippers={
        "left": GripperView("left", 1.0, holding=True, held_object="red_block",
                            jaw_gap_m=0.002),
        "right": after.gripper("right")})
    assert verifier(after).verdict == Verdict.FALSE
    assert "closed on itself" in verifier(after).reason


def test_a_stroke_that_never_stalled_is_not_a_grasp(d1_arm, observe):
    """Nothing arrested the jaws, so nothing is between them — whatever else
    the producer says. This is (1) of the three, and it is the half that
    replaces the closure-ratio gate F8 deleted."""
    world = observe(d1_arm, block_p=REACHABLE)
    verifier = Grasp(object="red_block", side="left").verifier(world)
    after = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"}, stalled={"left": False})
    report = verifier(after)
    assert report.verdict == Verdict.FALSE
    assert "never stopped short" in report.reason


def test_a_40mm_cube_is_held_at_a_closedness_the_old_gate_refused(d1_arm, observe):
    """F8, as a row. The env stopped the jaws at closedness 0.41 on a 40 mm
    cube and the 0.6 closure gate therefore reported nothing held — while the
    cube tracked the tool through 149.8 mm of lift. Closedness is not the
    evidence; the gap, the stall and the body between the pads are."""
    world = observe(d1_arm, block_p=REACHABLE, block_size=(0.04, 0.04, 0.04))
    verifier = Grasp(object="red_block", side="left").verifier(world)
    after = observe(d1_arm, block_p=REACHABLE, block_size=(0.04, 0.04, 0.04),
                    closed={"left": 0.41}, held={"left": "red_block"},
                    gap={"left": 0.04122})          # the measured face gap
    report = verifier(after)
    assert report.verdict == Verdict.TRUE, report.reason
    assert report.measured["closedness"] == 0.41


def test_the_gap_has_to_be_one_the_object_could_make(d1_arm, observe):
    """Either side of the window is a different, named failure."""
    from manipulation_kit.primitives.verifiers import grip_width_window

    size = (0.04, 0.04, 0.04)
    world = observe(d1_arm, block_p=REACHABLE, block_size=size)
    verifier = Grasp(object="red_block", side="left").verifier(world)
    low, high = grip_width_window(0.04)
    assert (round(low, 4), round(high, 4)) == (0.036, 0.044)

    def after(gap_m):
        return observe(d1_arm, block_p=REACHABLE, block_size=size,
                       closed={"left": 0.9}, held={"left": "red_block"},
                       gap={"left": gap_m})

    assert verifier(after(low + 0.0005)).verdict == Verdict.TRUE
    assert verifier(after(high - 0.0005)).verdict == Verdict.TRUE
    closed_through = verifier(after(0.002))
    assert closed_through.verdict == Verdict.FALSE
    assert "closed on itself" in closed_through.reason
    never_reached = verifier(after(0.060))
    assert never_reached.verdict == Verdict.FALSE
    assert "never reached it" in never_reached.reason


def test_a_10mm_bar_is_held_at_the_gap_a_10mm_bar_makes(d1_arm, observe):
    """The window is the OBJECT's, not the gripper's: the same 41 mm gap that
    holds a 40 mm cube is the jaws nowhere near a 10 mm bar."""
    bar = (0.010, 0.080, 0.010)
    world = observe(d1_arm, block_p=REACHABLE, block_size=bar)
    verifier = Grasp(object="red_block", side="left").verifier(world)
    held = observe(d1_arm, block_p=REACHABLE, block_size=bar, closed={"left": 0.86},
                   held={"left": "red_block"}, gap={"left": 0.0112})
    assert verifier(held).verdict == Verdict.TRUE
    wide = observe(d1_arm, block_p=REACHABLE, block_size=bar, closed={"left": 0.41},
                   held={"left": "red_block"}, gap={"left": 0.04122})
    assert verifier(wide).verdict == Verdict.FALSE


def test_an_empty_close_is_not_holding_however_hard_it_stalled(d1_arm, observe):
    """The control the harness ran on the sim: the same jaws close to 1.000 on
    NOTHING. Stalled, yes — on themselves."""
    world = observe(d1_arm, block_p=REACHABLE)
    verifier = Grasp(object="red_block", side="left").verifier(world)
    empty = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    gap={"left": 0.0}, stalled={"left": True})
    assert verifier(empty).verdict == Verdict.FALSE


def test_holding_falls_back_to_the_stall_when_the_width_is_unmeasurable(d1_arm, observe):
    """A producer with no pad-gap sensor still gets a verdict — from the stall
    and the body between the pads — rather than a manufactured gap."""
    world = observe(d1_arm, block_p=REACHABLE)
    verifier = Grasp(object="red_block", side="left").verifier(world)
    blind = observe(d1_arm, block_p=REACHABLE, closed={"left": 0.41},
                    held={"left": "red_block"}, gap={"left": None})
    assert verifier(blind).verdict == Verdict.TRUE
    not_stalled = observe(d1_arm, block_p=REACHABLE, closed={"left": 0.41},
                          held={"left": "red_block"}, gap={"left": None},
                          stalled={"left": False})
    assert verifier(not_stalled).verdict == Verdict.FALSE


def test_a_missing_gripper_report_is_unknown_not_false(d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE)
    verifier = Grasp(object="red_block", side="left").verifier(world)
    blind = world.with_(grippers={})
    assert verifier(blind).verdict == Verdict.UNKNOWN


def test_lift_is_false_while_the_object_is_still_standing_on_something(d1_arm, observe):
    """A rise is not a lift while the underside is still on a support. The
    block goes up 100 mm and so does the thing under it — which on a real robot
    is a drifting pose estimate or a surface that is not where it was measured,
    and either way is not an object in the air."""
    from manipulation_kit.world import SurfaceView

    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    verifier = Lift(object="red_block", side="left", height_m=0.10).verifier(world)
    high = (REACHABLE[0], REACHABLE[1], REACHABLE[2] + 0.10)
    risen = observe(d1_arm, block_p=high, closed={"left": 1.0},
                    held={"left": "red_block"})
    assert verifier(risen).verdict == Verdict.TRUE

    # the same rise, with the table's top exactly under the block's base
    base_z = high[2] - 0.05 / 2.0
    still_standing = risen.with_(objects=[
        SurfaceView("table", p=(0.40, 0.0, base_z - 0.01), size=(0.9, 0.8, 0.02))
        if o.name == "table" else o for o in risen.objects])
    report = verifier(still_standing)
    assert report.verdict == Verdict.FALSE
    assert "still on table" in report.reason
    assert report.measured["resting_on"] == "table"


def test_lift_measures_the_object_not_the_hand(d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    verifier = Lift(object="red_block", side="left", height_m=0.10).verifier(world)
    risen = observe(d1_arm, block_p=(REACHABLE[0], REACHABLE[1], REACHABLE[2] + 0.10),
                    closed={"left": 1.0}, held={"left": "red_block"})
    assert verifier(risen).verdict == Verdict.TRUE
    barely = observe(d1_arm, block_p=(REACHABLE[0], REACHABLE[1], REACHABLE[2] + 0.02),
                     closed={"left": 1.0}, held={"left": "red_block"})
    assert verifier(barely).verdict == Verdict.FALSE
    dropped = observe(d1_arm, block_p=(REACHABLE[0], REACHABLE[1], REACHABLE[2] + 0.10))
    assert verifier(dropped).verdict == Verdict.FALSE


def test_an_object_that_vanished_is_unknown(d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    verifier = Lift(object="red_block", side="left").verifier(world)
    gone = world.with_(objects=[o for o in world.objects if o.name != "red_block"])
    assert verifier(gone).verdict == Verdict.UNKNOWN


def test_place_needs_both_inside_and_let_go(d1_arm, observe):
    world = observe(d1_arm, closed={"left": 1.0}, held={"left": "red_block"})
    verifier = Place(object="red_block", to="box", side="left").verifier(world)
    inside_still_held = observe(d1_arm, block_p=(0.33, 0.34, 0.03),
                                closed={"left": 1.0}, held={"left": "red_block"})
    assert verifier(inside_still_held).verdict == Verdict.FALSE
    beside_and_released = observe(d1_arm, block_p=(0.60, 0.34, 0.03))
    assert verifier(beside_and_released).verdict == Verdict.FALSE
    done = observe(d1_arm, block_p=(0.33, 0.34, 0.03))
    assert verifier(done).verdict == Verdict.TRUE


def test_release_wants_the_jaws_open_as_well_as_empty(d1_arm, observe):
    world = observe(d1_arm, closed={"left": 1.0}, held={"left": "red_block"})
    verifier = Release(side="left").verifier(world)
    assert verifier(observe(d1_arm, closed={"left": 1.0})).verdict == Verdict.FALSE
    assert verifier(observe(d1_arm, closed={"left": 0.0})).verdict == Verdict.TRUE


def test_pour_is_unknown_once_the_tilt_happened_and_false_before(d1_arm, observe):
    """The honest verdict on a robot with no scale and no level sensor. Saying
    TRUE because the arm rotated is the exact failure measured verifiers exist
    to prevent."""
    world = observe(d1_arm, closed={"left": 1.0}, held={"left": "red_block"})
    verifier = Pour(source="red_block", target="box", side="left",
                    tilt_deg=75.0).verifier(world)
    assert verifier(world).verdict == Verdict.FALSE
    tilted = world.with_(objects=[
        ObjectView("red_block", p=(0.38, 0.25, 0.15), size=(0.05, 0.04, 0.05),
                   r=R.from_euler("y", 80, degrees=True))]
        + [o for o in world.objects if o.name != "red_block"])
    report = verifier(tilted)
    assert report.verdict == Verdict.UNKNOWN
    assert "nothing on this robot measures" in report.reason


def test_go_home_measures_against_the_kits_own_home_pose(d1_arm, observe):
    world = observe(d1_arm)
    verifier = GoHome().verifier(world)
    assert verifier(world).verdict == Verdict.TRUE      # already home
    away = world.with_(arms={
        "left": world.arm("left").__class__("left",
                                            joints=world.arm("left").joints + 0.5),
        "right": world.arm("right")})
    assert verifier(away).verdict == Verdict.FALSE


def test_every_verdict_report_carries_the_numbers_it_was_read_off(d1_arm, observe):
    world = observe(d1_arm, closed={"left": 1.0}, held={"left": "red_block"})
    for verb in ALL_VERBS:
        report = verb.verifier(world)(world)
        assert isinstance(report.measured, dict)
        assert report.reason, f"{verb.name()} gave a verdict with no reason"
