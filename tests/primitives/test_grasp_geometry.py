"""WHERE and HOW the hand meets an object: ``primitives.grasp_geometry``.

One test per promise of redesign step 3 (DESIGN C.2/C.4, D.1 row 3): the
fingertip reference, the standoff measured from the silhouette, the one roll
sweep, the tilted object grasped along its own face, the descent floor taken
from the measured support, and a fingertip plan that cannot be replayed as a
pad plan. All of it runs against the bundled URDF, the real IK and the real
guard — no mock.
"""

from __future__ import annotations

import math
import re
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from manipulation_kit.executor import KinematicExecutor, check_binding
from manipulation_kit.primitives import Approach, Grasp, decode
from manipulation_kit.primitives import grasp_geometry as gg
from manipulation_kit.primitives import orientation as ap
from manipulation_kit.primitives import reach
from manipulation_kit.primitives.offer import check
from manipulation_kit.primitives.orientation import tool_from_link7, tool_revision
from manipulation_kit.primitives.types import Plan
from manipulation_kit.world import (ALIASES, ArmView, ContainerView,
                                    Direction, FrameGraph, GripperView,
                                    ObjectView, SurfaceView, WorldView)
from manipulation_kit.world.direction import object_frame

SRC = Path(__file__).resolve().parents[2] / "src" / "manipulation_kit"
EXAMPLES = Path(__file__).resolve().parents[2] / "examples"

#: the tests' table: top face at z = 0.01, like the shared ``observe`` fixture
TABLE = SurfaceView("table", p=(0.40, 0.0, 0.0), size=(0.9, 0.8, 0.02))
TABLE_TOP = 0.01


def _world(kin, objects, *, open_gap_m=None):
    arms, grippers = [], []
    for side in ("left", "right"):
        p, r = tool_from_link7(*kin.ee_pose(side))
        arms.append(ArmView(side, joints=kin.joints(side), tool_p=p, tool_r=r,
                            mode="position"))
        grippers.append(GripperView(side, 0.0, jaw_gap_m=0.04,
                                    open_gap_m=open_gap_m))
    return WorldView.of(list(objects), arms=arms, grippers=grippers)


def _tips(p_tool, r_tcp) -> np.ndarray:
    """Where the finger TIPS are for a pad-centre tool point."""
    return np.asarray(p_tool) + r_tcp.apply(
        [0.0, 0.0, gg.TIP.offset_z_m - ap.TOOL_Z_M])


# --------------------------------------------------------------------------- #
# the fingertip reference
# --------------------------------------------------------------------------- #

def test_a_6mm_card_is_grasped_at_the_tips_and_refused_at_the_pads(d1_arm):
    """A 6 mm slab lying on a table. The pads reach 29 mm past their centre,
    so a pad grasp would close above it with the tips on the table; the TIP
    reference has nothing leading it, so the tips go to the table's clearance
    and close on the slab's edges."""
    card = ObjectView("card", p=(0.38, 0.25, TABLE_TOP + 0.003),
                      size=(0.060, 0.040, 0.006))
    world = _world(d1_arm, [card, TABLE])

    pads = {u.code for u in Grasp(object="card", side="left")
            .preconditions(world)}
    assert "object_too_flat" in pads
    refused = Grasp(object="card", side="left").plan(world, d1_arm)
    assert not refused.ok and refused.reason == "precondition_unmet"

    tips = Grasp(object="card", side="left", contact="tip")
    assert tips.preconditions(world) == []
    plan = tips.plan(world, d1_arm)
    assert isinstance(plan, Plan), str(plan)
    grasp = next(w for w in plan.waypoints if w.label == "grasp")
    at = _tips(grasp.p, grasp.r)
    # the plain descent stops the TIPS over the card's edges at the search
    # height, then a contact search takes them down to the table (0.16.0,
    # d1-2 tip trial: a fixed height closed 7 mm above the slab)
    assert at[2] == pytest.approx(TABLE_TOP + gg.TIP_SEARCH_START_M, abs=1e-9)
    end = _tips(plan.waypoints[-1].p, plan.waypoints[-1].r)
    assert plan.waypoints[-1].label == "contact_limit"
    assert end[2] == pytest.approx(TABLE_TOP - gg.CONTACT_OVERTRAVEL_M,
                                   abs=1e-9)
    assert end[2] < card.top_face_z(world.frames)
    assert np.allclose(at[:2], card.p[:2], atol=1e-9)
    # the waypoint is still the PAD CENTRE, 29 mm up the tool axis
    assert grasp.p[2] == pytest.approx(at[2] + gg.PAD.lead_m, abs=1e-9)
    assert any("contact at the tips" in n for n in plan.notes), plan.notes
    # and the model can ask for it
    decoded = decode("grasp", {"object": "card", "side": "left",
                               "contact": "tip"}, world)
    assert isinstance(decoded, Grasp) and decoded.contact == "tip"
    bad = decode("grasp", {"object": "card", "contact": "knuckle"}, world)
    assert not getattr(bad, "ok", True) and bad.reason == "bad_argument"


def test_the_tip_fit_uses_the_tip_clearance_and_the_measured_hand(d1_arm):
    """The width test is per reference (4 mm a side at the pads, 2 mm at the
    tips) and per HAND: the executor's measured opening when the world
    carries one, the description's nominal otherwise — never an env var."""
    frames = FrameGraph()
    spec = gg.GraspSpec(ALIASES["down"])
    assert gg.graspable_width_m(gg.PAD) == pytest.approx(0.05196 - 0.008)
    assert gg.graspable_width_m(gg.TIP) == pytest.approx(0.05196 - 0.004)
    assert gg.graspable_width_m(gg.PAD, 0.0605) == pytest.approx(0.0525)
    block = ObjectView("block", p=(0.38, 0.25, 0.035), size=(0.060, 0.046, 0.05))
    r_tcp = ap.grasp_orientation("left", spec.direction.vector(), block, frames)
    assert gg.fits(block, frames, spec, r_tcp).code == "object_too_wide"
    tip = gg.GraspSpec(ALIASES["down"], gg.TIP)
    assert gg.fits(block, frames, tip, r_tcp) is None
    assert gg.fits(block, frames, spec, r_tcp, open_gap_m=0.0605) is None
    # the verb reads it off the world's gripper
    wide = _world(d1_arm, [block, TABLE])
    measured = _world(d1_arm, [block, TABLE], open_gap_m=0.0605)
    assert "object_too_wide" in {u.code for u in Grasp(
        object="block", side="left").preconditions(wide)}
    assert Grasp(object="block", side="left").preconditions(measured) == []


def test_a_non_grasping_hand_shape_is_the_descriptions_closedness():
    """The ``hand`` argument itself is the contact verbs' (step 4); the
    number each shape means is the hand description's, read through here."""
    from manipulation_kit.hands.d1.parallel_gripper import description
    from manipulation_kit.primitives.arguments import ARGUMENTS
    for shape in description.HAND_POSES:
        assert gg.hand_closedness(shape) == description.HAND_CLOSEDNESS[shape]
    assert gg.hand_closedness("open") == 0.0
    assert gg.hand_closedness("closed") == 1.0
    with pytest.raises(ValueError):
        gg.hand_closedness("fist")
    assert ARGUMENTS["contact"].values == gg.CONTACTS


# --------------------------------------------------------------------------- #
# the standoff
# --------------------------------------------------------------------------- #

def test_the_standoff_clears_a_110mm_cup_rim(d1_arm):
    """C.4, Shu's "too close for a 5 cm object". The old standoff was 80 mm
    from the GRASP POINT, which a descent lifts only to 32 mm over the table,
    and the tips reach 29 mm past the tool point: 51 mm up, under a 110 mm
    rim. The standoff is now the gap between the tips and the object's
    silhouette."""
    cup = ObjectView("cup", p=(0.38, 0.25, TABLE_TOP + 0.055),
                     size=(0.042, 0.042, 0.110))
    world = _world(d1_arm, [cup, TABLE])
    rim = cup.top_face_z(world.frames)
    grasp = Grasp(object="cup", side="left")
    meet, unmet = grasp._meet(world)
    assert not unmet
    r_tcp = meet.r_tcp(meet.rolls[0])
    tips = _tips(meet.p_stand, r_tcp)
    assert tips[2] == pytest.approx(rim + gg.DEFAULT_STANDOFF_M, abs=1e-9)
    # the old rule, for the record: from the (lifted) grasp point
    old_tips = meet.p_grasp[2] + gg.DEFAULT_STANDOFF_M - gg.PAD.lead_m
    assert old_tips < rim, "the old standoff was under the rim"
    # Approach stands at exactly the point Grasp descends from
    stand = Approach(object="cup", side="left")._meet(world)[0].p_stand
    assert np.allclose(stand, meet.p_stand, atol=1e-12)
    plan = grasp.plan(world, d1_arm)
    assert isinstance(plan, Plan), str(plan)
    assert _tips(plan.waypoints[0].p, plan.waypoints[0].r)[2] > rim + 0.079
    # sideways too: the tips wait standoff_m short of the NEAR face
    fwd = Grasp(object="cup", side="left", direction="forward")._meet(world)[0]
    near = cup.p[0] - cup.size[0] / 2
    tips_fwd = _tips(fwd.p_stand, fwd.r_tcp(fwd.rolls[0]))
    assert tips_fwd[0] == pytest.approx(near - gg.DEFAULT_STANDOFF_M, abs=1e-9)


# --------------------------------------------------------------------------- #
# the one roll sweep
# --------------------------------------------------------------------------- #

#: d1-2's tape-cup scene (``examples/agent/scenes/d1-2_tape_cup.json``) as the
#: robot measured it, with the hand opening d1-2 reports (60.5 mm): the
#: squared wrist over the cube is guard-rejected for the right arm, and the
#: quarter turn plans — the run6 situation the example's two sweeps patched.
D1_2 = [SurfaceView("table", p=(0.5065, 0.0, 0.156), size=(0.4, 0.6, 0.02)),
        ObjectView("cube", p=(0.452, -0.233, 0.191), size=(0.045, 0.05, 0.05)),
        ContainerView("cup", p=(0.449, -0.113, 0.221), size=(0.09, 0.09, 0.11),
                      interior=(0.08, 0.08, 0.1), rim_height_m=0.1)]


def test_one_sweep_serves_chooser_and_loop(d1_arm, monkeypatch):
    """``roll_candidates`` is THE generator: the task chooser
    (``reach.choose_side``) and the loop's door (``decode`` -> ``check``)
    both take their roll from it, and no other sweep exists in the code."""
    # (1) no literal jaw-turn sweep anywhere in the wheel or the examples
    sweep = re.compile(r"\(\s*0(\.0)?\s*,\s*-\s*90(\.0)?\s*,\s*90(\.0)?\s*\)")
    for root in (SRC, EXAMPLES):
        for path in root.rglob("*.py"):
            if "_client" in path.parts:
                continue
            text = path.read_text(encoding="utf-8")
            assert not sweep.search(text), path
            # no roll handed to a verb from outside the kit's own sweep
            assert not re.search(r"roll_rad\s*=\s*math\.radians", text), path
    # ...and the one generator is called from the verbs' shared geometry only
    callers = sorted(str(p.relative_to(SRC)) for p in SRC.rglob("*.py")
                     if "roll_candidates(" in p.read_text(encoding="utf-8")
                     and p.name != "grasp_geometry.py")
    assert callers == ["primitives/verbs.py"], callers

    # (2) both doors go through it, and both land on the turn it offered
    world = _world(d1_arm, D1_2, open_gap_m=0.0605)
    calls = []
    real = gg.roll_candidates

    def recording(*args, **kwargs):
        out = real(*args, **kwargs)
        calls.append(out)
        return out

    monkeypatch.setattr(gg, "roll_candidates", recording)
    square = Approach(object="cube", side="right")._meet(world)[0]
    assert len(square.rolls) == 3 and square.rolls[0] == 0.0
    calls.clear()
    choice = reach.choose_side(world, d1_arm, obj="cube", destination="cup")
    assert calls, "the chooser did not ask roll_candidates"
    right = choice.chains["right"]
    approach = right.links[0].result
    assert approach.ok, str(approach)
    assert any(n.startswith("jaws rolled ") for n in approach.notes)
    calls.clear()
    call = decode("approach", {"object": "cube", "side": "right"}, world)
    loop_plan = check(call, world, d1_arm)
    assert calls, "the loop's door did not ask roll_candidates"
    assert isinstance(loop_plan, Plan), str(loop_plan)
    assert loop_plan.notes == approach.notes
    assert np.allclose(loop_plan.waypoints[0].r.as_quat(),
                       approach.waypoints[0].r.as_quat())
    # the squared wrist really is the refused one: pinned via the sweep
    monkeypatch.setattr(gg, "roll_candidates", lambda *a, **k: (0.0,))
    only_square = Approach(object="cube", side="right").plan(world, d1_arm)
    assert not only_square.ok and only_square.reason == "guard_reject"


def test_a_grasp_continues_the_roll_its_approach_took(d1_arm):
    """After an Approach that had to turn the wrist, the Grasp's first try is
    the roll the wrist already holds — the two verbs do not choose twice."""
    world = _world(d1_arm, D1_2, open_gap_m=0.0605)
    chain = reach.plan_chain(world, d1_arm, obj="cube", destination="cup",
                             side="right")
    approach, grasp = chain.links[0].result, chain.links[1].result
    assert approach.ok and grasp.ok, chain.sentence()
    assert np.allclose(grasp.waypoints[0].r.as_quat(),
                       approach.waypoints[0].r.as_quat())
    assert np.allclose(grasp.waypoints[0].p, approach.waypoints[0].p)


def test_the_roll_candidates_drop_a_turn_that_does_not_fit():
    frames = FrameGraph()
    spec = gg.GraspSpec(ALIASES["down"])
    long_ = ObjectView("bar", p=(0.4, 0.2, 0.05), size=(0.12, 0.03, 0.03))
    assert gg.roll_candidates(long_, frames, spec) == (0.0,)
    cube = ObjectView("cube", p=(0.4, 0.2, 0.05), size=(0.04, 0.04, 0.04))
    assert gg.roll_candidates(cube, frames, spec) == (
        0.0, -math.pi / 2, math.pi / 2)
    too_wide = ObjectView("slab", p=(0.4, 0.2, 0.05), size=(0.09, 0.08, 0.03))
    assert gg.roll_candidates(too_wide, frames, spec) == (0.0,)


# --------------------------------------------------------------------------- #
# yaw (and tilt) follow the object
# --------------------------------------------------------------------------- #

def _tilted_block(tilt_deg=30.0):
    """A 50 x 40 x 50 mm block tipped about y — its top face turned TOWARD the
    robot (-x), so the face-normal descent comes from the robot's side — its
    low edge on the table."""
    r = R.from_euler("y", -math.radians(tilt_deg))
    size = np.array([0.05, 0.04, 0.05])
    tall = sum(abs(r.as_matrix()[2, i]) * size[i] for i in range(3))
    return ObjectView("red_block", p=(0.38, 0.25, TABLE_TOP + tall / 2),
                      size=size, r=r)


def test_a_30deg_object_is_grasped_along_its_own_face(d1_arm):
    """Req 4. A 30 deg block on a measured table is not refused: the descent
    is its own top face's normal, ``object:red_block``, the jaws are squared
    to its footprint, and the notes say so. The explicit object-frame
    direction plans identically."""
    block = _tilted_block(30.0)
    world = _world(d1_arm, [block, TABLE])
    assert gg.support_of(world, "red_block") is TABLE or \
        gg.support_of(world, "red_block").name == "table"
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    assert isinstance(plan, Plan), str(plan)
    face = block.axes_in_base(FrameGraph())[:, 2]          # body +z, base
    grasp = plan.waypoints[-1]
    axis = grasp.r.as_matrix()[:, 2]
    assert np.allclose(axis, -face, atol=1e-9), (axis, face)
    assert math.degrees(math.acos(-float(axis[2]))) == pytest.approx(30.0)
    # the jaws close across the block's own 40 mm side
    assert ap.grasp_width(block, world.frames, grasp.r) == pytest.approx(
        0.040, abs=1e-6)
    assert any("tilted 30 deg" in n and "object:red_block" in n
               for n in plan.notes), plan.notes
    explicit = Grasp(object="red_block", side="left",
                     direction=Direction((0, 0, -1),
                                         object_frame("red_block")))
    other = explicit.plan(world, d1_arm)
    assert isinstance(other, Plan), str(other)
    for a, b in zip(plan.waypoints, other.waypoints):
        assert np.allclose(a.p, b.p, atol=1e-12)
        assert np.allclose(a.r.as_quat(), b.r.as_quat(), atol=1e-12)
    # the tips stop over the TABLE even though the descent is not vertical
    tips = _tips(grasp.p, grasp.r)
    assert tips[2] >= TABLE_TOP + ap.SUPPORT_CLEARANCE_M - 1e-9
    # below the threshold nothing changes: a 5 deg block is descended on
    # straight down
    slight = _world(d1_arm, [_tilted_block(5.0), TABLE])
    meet = Grasp(object="red_block", side="left")._meet(slight)[0]
    assert np.allclose(meet.d, (0, 0, -1))
    assert not any("tilted" in n for n in meet.notes)


# --------------------------------------------------------------------------- #
# the descent floor (L5)
# --------------------------------------------------------------------------- #

def test_the_descent_floor_is_the_support_when_one_is_known(d1_arm):
    """Astra review 4 / L5. A block declared 20 mm INTO a measured table used
    to set the floor from its own underside, so the finger tips were sent
    17 mm below the table top. The measured surface wins, and the notes say
    which floor was used; with no surface, the underside is the floor and the
    notes say that instead."""
    sunk = ObjectView("red_block", p=(0.38, 0.25, TABLE_TOP - 0.02 + 0.035),
                      size=(0.05, 0.04, 0.07))
    world = _world(d1_arm, [sunk, TABLE])
    assert gg.support_of(world, "red_block").name == "table"
    plan = Grasp(object="red_block", side="left").plan(world, d1_arm)
    assert isinstance(plan, Plan), str(plan)
    tips = _tips(plan.waypoints[-1].p, plan.waypoints[-1].r)
    assert tips[2] == pytest.approx(TABLE_TOP + ap.SUPPORT_CLEARANCE_M,
                                    abs=1e-9)
    assert any("descent floor: table's top" in n for n in plan.notes)
    # the achieved-clearance check is against the SAME floor
    solved = [n for n in plan.notes if n.startswith("the solved descent")]
    assert solved, plan.notes
    kept = float(re.search(r"tips ([-0-9.]+) mm", solved[0]).group(1))
    assert kept >= ap.MIN_ACHIEVED_CLEARANCE_M * 1000, solved
    # the old floor, for the record: 20 mm lower, i.e. into the table
    old_tips = sunk.bottom_z(world.frames) + ap.SUPPORT_CLEARANCE_M
    assert old_tips < TABLE_TOP

    # no measured surface: the object's own underside, said so
    bare = _world(d1_arm, [sunk])
    assert gg.support_of(bare, "red_block") is None
    plan = Grasp(object="red_block", side="left").plan(bare, d1_arm)
    assert isinstance(plan, Plan), str(plan)
    # the underside is 35 mm below the centre, so the centre already clears
    # it: the tool goes to the centre — 17 mm lower than with the table known
    assert np.allclose(plan.waypoints[-1].p, sunk.p, atol=1e-9)
    tips = _tips(plan.waypoints[-1].p, plan.waypoints[-1].r)
    assert tips[2] >= sunk.bottom_z(bare.frames) + ap.SUPPORT_CLEARANCE_M
    assert tips[2] < TABLE_TOP
    assert any("own declared underside" in n for n in plan.notes)
    # a surface far below an object on something unmeasured is not its floor
    floating = ObjectView("red_block", p=(0.38, 0.25, 0.10),
                          size=(0.05, 0.04, 0.05))
    assert gg.support_of(_world(d1_arm, [floating, TABLE]), "red_block") is None


# --------------------------------------------------------------------------- #
# plan binding
# --------------------------------------------------------------------------- #

def test_a_tip_plan_is_not_replayable_as_a_pad_plan(d1_arm):
    """``tool_revision`` records WHERE on the hand a grasp plan makes contact;
    ``PlanBinding.drift`` compares every field both revisions state. So a tip
    plan runs on the hand it was planned for (the executor states the hand
    only) and is refused wherever a PAD plan is expected — and vice versa."""
    block = ObjectView("red_block", p=(0.38, 0.25, TABLE_TOP + 0.025),
                       size=(0.05, 0.04, 0.05))
    world = _world(d1_arm, [block, TABLE])
    tip = Grasp(object="red_block", side="left", contact="tip").plan(world, d1_arm)
    pad = Grasp(object="red_block", side="left").plan(world, d1_arm)
    assert isinstance(tip, Plan) and isinstance(pad, Plan), (tip, pad)
    assert tip.binding.tool_revision == tool_revision("tip")
    assert pad.binding.tool_revision == tool_revision("pad")
    assert tip.binding.tool_revision != pad.binding.tool_revision
    # where a pad plan is expected, the tip plan is refused (and back)
    assert "tool configuration changed" in tip.binding.drift(
        tool_revision=tool_revision("pad"))
    assert "tool configuration changed" in pad.binding.drift(
        tool_revision=tool_revision("tip"))
    assert tip.binding.drift(tool_revision=tool_revision("tip")) is None
    # the executor states the HAND, and both run on the hand they were made for
    executor = KinematicExecutor(d1_arm)
    assert check_binding(tip, executor) is None
    assert check_binding(pad, executor) is None
    # a different hand still refuses either
    other_hand = tool_revision().replace("pad_tip=0.1290", "pad_tip=0.1435")
    assert "tool configuration changed" in tip.binding.drift(
        tool_revision=other_hand)
    # the two plans put the tool point in different places, which is WHY
    assert not np.allclose(tip.waypoints[-1].p, pad.waypoints[-1].p)
