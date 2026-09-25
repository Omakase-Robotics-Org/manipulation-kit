"""A fingertip grasp backs off the support before it closes; a press does not.

The d1-2 record (``servo-judgeonly-2``, 2026-09-24, turn 6): a top-down
``grasp(tape, right, down, soft, tip)`` whose declared centre was 55-62 mm
off the roll. The fingertip search stopped ON THE TABLE — 13.5 mm into the
search, tips at z 0.1674, 1.4 mm above the modelled 0.166 top, on a 6.84 Nm
rise — the close stroke dragged the tips across the table, the friction
stalled the jaws at 48 mm (inside the 46-54 mm window of the 50 mm
declaration) and the daemon reported a hold.

The replay below is that geometry on a contact double of the kinematic
mirror: the table is where the robot met it, the roll is where the later
looks put it (clear of the jaws), and the jaws stall on the table's friction
exactly when the tips are left touching it. Nothing else is simulated — the
double states the one physical fact the record established.

* With the ``back_off`` policy (the grasp's) the tips leave the table by 1 mm
  along minus the travel, the jaws sweep to the empty gap and the verifier
  says FALSE.
* With the back-off at 0 the stall reproduces and the verifier says TRUE —
  the false hold of the record.
* A stop ABOVE the support (on the object) does not back off; a leg that
  touched nothing is ``none``; a ``press`` keeps ``push_through``.
* The back-off runs along minus the leg's own direction, whatever it is.
"""

from __future__ import annotations

import dataclasses
import json
import math

import numpy as np
import pytest

from manipulation_kit.executor import (CONTACT, MAX_TRAVEL, SURFACE_NONE,
                                       SURFACE_OBJECT, SURFACE_SUPPORT,
                                       ContactReport, KinematicExecutor,
                                       StrokeReport, contact_outcome,
                                       contact_report, leg_posture_at,
                                       leg_ticks, run)
from manipulation_kit.primitives import (VERB_CONTACT_POLICY, ContactPolicy,
                                         ContactStep, Grasp, Plan, Press, Probe,
                                         SurfaceBackoff)
from manipulation_kit.primitives import grasp_geometry as gg
from manipulation_kit.primitives import orientation as ap
from manipulation_kit.primitives.clearance import (ClearancePolicy, policy_of,
                                                   set_policy)
from manipulation_kit.world import (ALIASES, ArmView, Direction, GripperView,
                                    ObjectView, SurfaceView, WorldView)

# --- the record, turn 6 ---------------------------------------------------- #
#: the world the grasp was planned in (``world`` of iteration 6)
TABLE = SurfaceView("table", p=(0.506, 0.0, 0.156), size=(0.4, 0.6, 0.02))
TABLE_TOP = 0.166
TAPE_DECLARED = ObjectView("tape", p=(0.351, -0.118, 0.179),
                           size=(0.05, 0.05, 0.026))
#: where the later looks put the roll (turn 8 re-declared it here)
TAPE_TRUE_XY = (0.413, -0.111)
Q0_DEG = {"left": (-52.26, 87.38, 88.3, -114.32, 86.67, -1.1, 13.05),
          "right": (32.06, -64.88, -46.06, -98.86, 63.67, 43.33, 44.08)}
OPEN_GAP_M = 0.064
#: the recorded plan's final (grasp) posture of the right arm [deg]
LIVE_GRASP_Q_DEG = (18.49, -66.814, -47.82, -98.011, 65.939, 41.761, 28.814)
#: the table as the tips MET it: the contact record's p z
TABLE_MET_Z = 0.1674
#: the recorded stall and the torque rise that ended the search
STALL_GAP_M = 0.048
RECORDED_RISE_NM = 6.843
#: the tips count as dragging on a surface this close to it [m]
DRAG_M = 0.0002

TIP_LEAD_M = gg.PAD_TIP_Z_M - ap.TOOL_Z_M


def _tips(kin, side: str, u) -> np.ndarray:
    p, _r = ap.tool_from_link7(*kin.ee_pose(side))
    return np.asarray(p, dtype=float) + TIP_LEAD_M * np.asarray(u, dtype=float)


class _Turn6(KinematicExecutor):
    """The mirror, plus the two physical facts of turn 6.

    ``move_until`` stops when the leading tips reach ``stop_z`` (the table as
    met, or an object's top above it). A close with the tips within
    :data:`DRAG_M` of the table stalls on its friction at the recorded 48 mm
    and the hand reports a hold; a close clear of it, with nothing between
    the jaws, goes to the empty gap.
    """

    def __init__(self, kin, *, stop_z: float = TABLE_MET_Z,
                 table_z: float = TABLE_MET_Z, **kw):
        super().__init__(kin, open_gap_m=OPEN_GAP_M, **kw)
        self.stop_z = stop_z
        self.table_z = table_z
        self.closed_on = None          # "table" | "nothing" after a close
        self.tips_at_close = None

    def move_until(self, path, *, side, criterion, hz=50.0, kin=None,
                   direction=None, **_):
        timed = [(float(t), np.asarray(q, dtype=float)) for t, q in path]
        q_start = np.array(self.kin.joints(side), dtype=float)
        u = np.asarray(direction, dtype=float)
        for t, q in leg_ticks(timed, hz):
            self.kin.set_joints(side, q)
            if _tips(self.kin, side, u)[2] <= self.stop_z + 1e-9:
                return contact_report(
                    self.kin, side, timed, direction=direction,
                    q_start=q_start, q_stop=q, made=True, stopped_by=CONTACT,
                    torque_nm=RECORDED_RISE_NM, joint=0, leg_t_s=t,
                    elapsed_s=t, detail="the tips met the table")
        return contact_report(
            self.kin, side, timed, direction=direction, q_start=q_start,
            q_stop=timed[-1][1], made=False, stopped_by=MAX_TRAVEL,
            leg_t_s=timed[-1][0], elapsed_s=timed[-1][0], detail="air")

    def set_gripper(self, side, closedness, *, grip):
        super().set_gripper(side, closedness, grip=grip)
        if closedness >= self.hold_at:
            tips = _tips(self.kin, side, (0.0, 0.0, -1.0))
            self.tips_at_close = float(tips[2])
            self.closed_on = ("table" if tips[2] - self.table_z <= DRAG_M
                              else "nothing")

    def wait_gripper_settled(self, side, *, timeout_s=3.0):
        stalled = self.closed_on == "table" and self.grippers[side] >= 0.5
        return StrokeReport(True, self.grippers[side], holding=stalled,
                            stalled=stalled, detail="turn-6 double")

    def gripper_view(self, side) -> GripperView:
        if self.closed_on == "table":
            # the record: 0.261 closedness, 48 mm, stalled, held (producer)
            return GripperView(side, 0.261, holding=True,
                               jaw_gap_m=STALL_GAP_M, held_object="tape",
                               grip="soft", jaw_stalled=True,
                               open_gap_m=OPEN_GAP_M)
        closed = self.grippers[side] >= 0.5
        return GripperView(side, 1.0 if closed else 0.0, holding=False,
                           jaw_gap_m=0.0 if closed else OPEN_GAP_M,
                           grip="soft", jaw_stalled=False if closed else None,
                           open_gap_m=OPEN_GAP_M)


def _world(kin, grippers=None, objects=(TABLE, TAPE_DECLARED)) -> WorldView:
    arms = []
    for side in ("left", "right"):
        p, r = ap.tool_from_link7(*kin.ee_pose(side))
        arms.append(ArmView(side, joints=kin.joints(side), tool_p=p, tool_r=r,
                            mode="position"))
    grippers = grippers or [GripperView(s, 0.0, jaw_gap_m=OPEN_GAP_M,
                                        open_gap_m=OPEN_GAP_M)
                            for s in ("left", "right")]
    return WorldView.of(list(objects), arms=arms, grippers=grippers)


def _at_turn6(kin) -> WorldView:
    for side in ("left", "right"):
        kin.set_joints(side, np.radians(Q0_DEG[side]))
    return _world(kin)


@pytest.fixture
def backoff_policy(d1_arm):
    """Restores the shared arm model's clearance policy afterwards."""
    saved = policy_of(d1_arm)
    yield d1_arm
    set_policy(d1_arm, saved)


def _turn6_plan(kin):
    world0 = _at_turn6(kin)
    plan = Grasp(object="tape", side="right", direction="down", grip="soft",
                 contact="tip").plan(world0, kin)
    assert isinstance(plan, Plan), str(plan)
    return world0, plan


def _leg(plan) -> ContactStep:
    (leg,) = [s for s in plan.steps if isinstance(s, ContactStep)]
    return leg


def _run_turn6(kin, **double):
    world0, plan = _turn6_plan(kin)
    ex = _Turn6(kin, **double)
    for side in ("left", "right"):
        ex.kin.set_joints(side, world0.arm(side).joints)
    report = run(plan, ex, kin=kin)
    world1 = _world(kin, grippers=[ex.gripper_view("left"),
                                   ex.gripper_view("right")])
    verdict = Grasp(object="tape", side="right", direction="down", grip="soft",
                    contact="tip").verifier(world0)(world1)
    return plan, ex, report, verdict


# --------------------------------------------------------------------------- #
# the plan
# --------------------------------------------------------------------------- #

def test_the_turn6_grasp_plans_a_back_off_from_the_table(backoff_policy):
    kin = backoff_policy
    _w, plan = _turn6_plan(kin)
    leg = _leg(plan)
    assert leg.policy is ContactPolicy.BACK_OFF
    assert VERB_CONTACT_POLICY["grasp"] is ContactPolicy.BACK_OFF
    b = leg.backoff
    assert b.distance_m == pytest.approx(gg.CONTACT_BACKOFF_M)
    assert b.support == "table"
    # the solved leg starts the tips ~15 mm over the modelled top
    assert b.surface_at_m == pytest.approx(gg.TIP_SEARCH_START_M, abs=5e-4)
    assert b.band_m == pytest.approx(gg.SURFACE_CONTACT_BAND_M)
    assert b.tip_lead_m == pytest.approx(0.029)
    assert any("contact policy back_off" in n for n in plan.notes)
    # the plan is the recorded one: the grasp posture the robot ran
    assert np.degrees(plan.final_joints()["right"]) == pytest.approx(
        LIVE_GRASP_Q_DEG, abs=1.0)
    # the close still comes after the leg
    kinds = [type(s).__name__ for s in plan.steps]
    assert kinds.index("ContactStep") < len(kinds) - 1 - kinds[::-1].index(
        "GripStep")
    summary = plan.to_json()["contact_steps"][0]
    assert summary["policy"] == "back_off" and summary["backoff_m"] == 0.001
    full = plan.to_json(full=True)
    assert json.dumps(full)          # the full step record serialises


def test_the_default_back_off_is_twice_what_the_executor_resolves():
    assert ClearancePolicy().contact_backoff_m == gg.CONTACT_BACKOFF_M == 0.001
    assert gg.CONTACT_BACKOFF_M >= 2 * gg.CONTACT_BACKOFF_MIN_M
    # the model's correction grid is not the arm's resolution
    from manipulation_kit.primitives.types import NUDGE_GRID_M
    assert min(NUDGE_GRID_M) > gg.CONTACT_BACKOFF_M


def test_a_back_off_below_the_resolution_is_raised_to_it(backoff_policy):
    kin = backoff_policy
    set_policy(kin, dataclasses.replace(policy_of(kin),
                                        contact_backoff_m=0.0002))
    _w, plan = _turn6_plan(kin)
    assert _leg(plan).backoff.distance_m == gg.CONTACT_BACKOFF_MIN_M
    assert any("raised from 0.20 mm" in n for n in plan.notes)


def test_an_object_too_thin_for_the_retreat_is_closed_on_where_it_stopped(
        backoff_policy):
    """An 8 mm slab and a 5 mm back-off: the retreat would lift the tips past
    half of it, so the leg stays where it stopped (and says why)."""
    kin = backoff_policy
    set_policy(kin, dataclasses.replace(policy_of(kin),
                                        contact_backoff_m=0.005))
    slab = ObjectView("tape", p=(0.351, -0.118, TABLE_TOP + 0.004),
                      size=(0.05, 0.05, 0.008))
    _at_turn6(kin)
    world0 = _world(kin, objects=(TABLE, slab))
    plan = Grasp(object="tape", side="right", contact="tip").plan(world0, kin)
    assert isinstance(plan, Plan), str(plan)
    leg = _leg(plan)
    assert leg.policy is ContactPolicy.STAY and leg.backoff is None
    assert any("no back-off after the tips touch" in n for n in plan.notes)
    # the default 1 mm fits the same slab
    set_policy(kin, dataclasses.replace(policy_of(kin),
                                        contact_backoff_m=0.001))
    plan = Grasp(object="tape", side="right", contact="tip").plan(world0, kin)
    assert _leg(plan).policy is ContactPolicy.BACK_OFF
    assert _leg(plan).backoff.band_m == pytest.approx(0.004)


# --------------------------------------------------------------------------- #
# the turn-6 replay
# --------------------------------------------------------------------------- #

def test_the_true_roll_is_clear_of_the_jaws():
    """What the double assumes: nothing but the table is between the jaws."""
    gap = math.hypot(TAPE_TRUE_XY[0] - 0.351, TAPE_TRUE_XY[1] + 0.118)
    assert gap - 0.025 > OPEN_GAP_M / 2


def test_with_the_back_off_the_jaws_sweep_to_the_empty_gap(backoff_policy):
    kin = backoff_policy
    plan, ex, report, verdict = _run_turn6(kin)
    assert report.completed, report.error
    (c,) = report.contacts
    assert c.made and c.stopped_by == CONTACT
    assert c.surface == SURFACE_SUPPORT and c.support == "table"
    # where the record stopped: 1.4 mm above the modelled top
    assert c.tip_z_before_m == pytest.approx(TABLE_MET_Z, abs=5e-4)
    assert c.height_m == pytest.approx(TABLE_MET_Z - TABLE_TOP, abs=6e-4)
    assert c.backoff_m == pytest.approx(0.001, abs=1e-9)
    # measured after the retreat: 1 mm up, along minus the (downward) travel
    assert c.backoff_measured_m == pytest.approx(0.001, abs=5e-5)
    assert c.tip_z_after_m - c.tip_z_before_m == pytest.approx(0.001, abs=5e-5)
    assert ex.closed_on == "nothing"
    assert ex.tips_at_close == pytest.approx(c.tip_z_after_m, abs=1e-6)
    assert verdict.verdict == "false", verdict.reason
    record = report.to_json()["contacts"][0]
    assert record["surface"] == "support" and record["support"] == "table"
    assert record["backoff_m"] == 0.001
    assert record["tip_z_after_m"] > record["tip_z_before_m"]


def test_without_the_back_off_the_table_drag_stall_reproduces(backoff_policy):
    kin = backoff_policy
    set_policy(kin, dataclasses.replace(policy_of(kin), contact_backoff_m=0.0))
    plan, ex, report, verdict = _run_turn6(kin)
    assert report.completed, report.error
    (c,) = report.contacts
    assert c.surface == SURFACE_SUPPORT and c.backoff_m == 0.0
    assert c.tip_z_after_m == pytest.approx(c.tip_z_before_m, abs=1e-6)
    assert ex.closed_on == "table"
    # the record's false hold: 48 mm inside the 50 mm declaration's window
    assert verdict.verdict == "true", verdict.reason
    assert verdict.measured["jaw_gap_m"] == STALL_GAP_M


def test_a_stop_on_the_object_above_the_support_does_not_back_off(
        backoff_policy):
    """The tips stop 8 mm over the table — on something standing on it,
    outside the 5 mm band — and the close happens where they stopped."""
    kin = backoff_policy
    plan, ex, report, _v = _run_turn6(kin, stop_z=TABLE_TOP + 0.008)
    assert report.completed, report.error
    (c,) = report.contacts
    assert c.made and c.surface == SURFACE_OBJECT
    assert c.height_m == pytest.approx(0.008, abs=6e-4)
    assert c.backoff_m == 0.0
    assert c.tip_z_after_m == pytest.approx(c.tip_z_before_m, abs=1e-6)
    assert ex.tips_at_close == pytest.approx(TABLE_TOP + 0.008, abs=6e-4)


def test_a_search_that_touched_nothing_is_none_and_moves_nothing(
        backoff_policy):
    kin = backoff_policy
    _w, plan = _turn6_plan(kin)
    world0 = _at_turn6(kin)
    ex = KinematicExecutor(kin, open_gap_m=OPEN_GAP_M)
    for side in ("left", "right"):
        ex.kin.set_joints(side, world0.arm(side).joints)
    report = run(plan, ex, kin=kin)
    assert report.completed, report.error
    (c,) = report.contacts
    assert not c.made and c.surface == SURFACE_NONE and c.backoff_m == 0.0


# --------------------------------------------------------------------------- #
# press and probe keep their policies
# --------------------------------------------------------------------------- #

def _fingertips_down(kin):
    kin.set_joints("left", np.radians(
        [-21.091, -68.997, 37.832, -84.469, -5.509, -34.506, -50.0]))
    return _world(kin, objects=())


class _Meets(KinematicExecutor):
    """A mirror whose leg stops ``at_m`` along its travel, on contact."""

    def __init__(self, kin, step: ContactStep, at_m: float, **kw):
        super().__init__(kin, **kw)
        self.step, self.at_m = step, at_m

    def move_until(self, path, *, side, criterion, hz=50.0, kin=None,
                   direction=None, **_):
        timed = [(float(t), np.asarray(q, dtype=float)) for t, q in path]
        q_start = np.array(self.kin.joints(side), dtype=float)
        q_stop = leg_posture_at(self.step, self.at_m)
        self.kin.set_joints(side, q_stop)
        return contact_report(self.kin, side, timed, direction=direction,
                              q_start=q_start, q_stop=q_stop, made=True,
                              stopped_by=CONTACT, torque_nm=6.5,
                              leg_t_s=self.at_m / self.step.speed_m_s)


def _button(kin):
    """A 30 mm button 120 mm in front of the right hand's HOME tool point
    (``test_contact._button_scene``)."""
    p, _r = ap.tool_from_link7(*kin.ee_pose("right"))
    return ObjectView("button", p=np.asarray(p) + np.array([0.12, 0.0, 0.0]),
                      size=(0.03, 0.03, 0.03))


def test_a_press_plan_pushes_through_and_has_no_back_off(d1_arm):
    world = _world(d1_arm, objects=(_button(d1_arm),))
    plan = Press(target="button", side="right", direction="forward",
                 force_nm=6.0, hold_s=0.5).plan(world, d1_arm)
    assert isinstance(plan, Plan), str(plan)
    leg = _leg(plan)
    assert leg.policy is ContactPolicy.PUSH_THROUGH and leg.retract
    assert leg.backoff is None
    assert VERB_CONTACT_POLICY["press"] is ContactPolicy.PUSH_THROUGH
    assert not any("back_off" in n for n in plan.notes)
    # run: it touches the face 50 mm in, is not classified, holds, and
    # returns to its standoff — no 1 mm back-off anywhere
    ex = _Meets(d1_arm, leg, 0.05)
    for side in ("left", "right"):
        ex.kin.set_joints(side, world.arm(side).joints)
    report = run(plan, ex, kin=d1_arm)
    assert report.completed, report.error
    (c,) = report.contacts
    assert c.made and c.surface == "" and c.backoff_m == 0.0
    assert "surface" not in c.to_json()
    assert contact_outcome(leg, c) is None
    standoff = next(w for w in plan.waypoints if w.label == "press_standoff")
    p, _r = ap.tool_from_link7(*d1_arm.ee_pose("right"))
    assert np.linalg.norm(np.asarray(p) - standoff.p) < 0.002
    frozen = leg_posture_at(leg, 0.05)
    held = [q for _t, q in ex.sent if np.allclose(q[8:15], frozen, atol=1e-6)]
    assert len(held) >= int(0.5 * 50), "the press did not hold its command"


def test_a_probe_plan_stays_touching(d1_arm):
    world = _fingertips_down(d1_arm)
    plan = Probe(side="left", direction="down").plan(world, d1_arm)
    assert plan.ok, str(plan)
    leg = _leg(plan)
    assert leg.policy is ContactPolicy.STAY and leg.backoff is None
    assert not leg.retract


def test_the_policy_and_the_step_must_agree(d1_arm):
    world = _fingertips_down(d1_arm)
    leg = _leg(Probe(side="left", direction="down").plan(world, d1_arm))
    with pytest.raises(ValueError, match="needs SurfaceBackoff"):
        dataclasses.replace(leg, policy=ContactPolicy.BACK_OFF)
    with pytest.raises(ValueError, match="push_through"):
        dataclasses.replace(leg, policy=ContactPolicy.PUSH_THROUGH)
    with pytest.raises(ValueError, match="takes no"):
        dataclasses.replace(leg, backoff=SurfaceBackoff(0.001, 0.01, 0.005))


# --------------------------------------------------------------------------- #
# the back-off is along minus the leg's own direction
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize("side,direction", [
    ("left", "down"), ("right", "forward"),
    ("left", Direction((0.7071, 0.0, -0.7071), "base"))])
def test_the_back_off_runs_along_minus_the_travel(d1_arm, side, direction):
    """A probe leg in each direction, given the back_off policy, stopped on
    its 'support' 12 mm in: the runner retreats 1 mm along minus that
    direction, measured at the tool."""
    world = (_fingertips_down(d1_arm) if side == "left"
             else _world(d1_arm, objects=()))
    plan = Probe(side=side, direction=direction,
                 max_travel_m=0.03).plan(world, d1_arm)
    assert plan.ok, str(plan)
    leg = _leg(plan)
    backed = dataclasses.replace(
        leg, policy=ContactPolicy.BACK_OFF,
        backoff=SurfaceBackoff(0.001, surface_at_m=0.012, band_m=0.005,
                               support="wall", tip_lead_m=TIP_LEAD_M))
    plan = dataclasses.replace(plan, steps=tuple(
        backed if s is leg else s for s in plan.steps))
    u = np.asarray(leg.direction.vector(), dtype=float)
    ex = _Meets(d1_arm, backed, 0.012)
    for s in ("left", "right"):
        ex.kin.set_joints(s, world.arm(s).joints)
    report = run(plan, ex, kin=d1_arm)
    assert report.completed, report.error
    (c,) = report.contacts
    assert c.surface == SURFACE_SUPPORT and c.support == "wall"
    assert c.backoff_measured_m == pytest.approx(0.001, abs=5e-5)
    p_after, _ = ap.tool_from_link7(*d1_arm.ee_pose(side))
    moved = np.asarray(p_after, dtype=float) - np.asarray(c.p_tool, dtype=float)
    # along minus the travel; across it no more than the leg's own knots
    # deviate from their line (the executor's 0.3 mm resolution)
    assert float(np.dot(moved, -u)) == pytest.approx(0.001, abs=5e-5)
    assert float(np.linalg.norm(moved + u * 0.001)) < 3e-4
    assert c.tip_z_after_m - c.tip_z_before_m == pytest.approx(
        -0.001 * float(u[2]), abs=5e-5)


def test_the_outcome_rule_on_its_own():
    q = [np.zeros(7), np.full(7, 0.01), np.full(7, 0.02)]
    step = ContactStep("left", ALIASES["down"], 0.02, waypoint=1, path=q,
                       s=(0.0, 0.01, 0.02), policy=ContactPolicy.BACK_OFF,
                       backoff=SurfaceBackoff(0.001, 0.015, 0.005, "table"))

    def at(travel, made=True):
        return ContactReport(made, (0, 0, 0), travel, (0, 0, 1), 5.0,
                             CONTACT if made else MAX_TRAVEL)

    on = contact_outcome(step, at(0.0135))         # 1.5 mm short: support
    assert on.surface == SURFACE_SUPPORT
    assert on.height_m == pytest.approx(0.0015)
    assert np.allclose(on.q_backoff, np.full(7, 0.0125))
    assert contact_outcome(step, at(0.018)).surface == SURFACE_SUPPORT  # past
    assert contact_outcome(step, at(0.0099)).surface == SURFACE_OBJECT
    assert contact_outcome(step, at(0.0099)).q_backoff is None
    assert contact_outcome(step, at(0.02, made=False)).surface == SURFACE_NONE
    # never behind the leg start
    early = dataclasses.replace(step, backoff=SurfaceBackoff(0.001, 0.0005,
                                                             0.005))
    assert np.allclose(contact_outcome(early, at(0.0004)).q_backoff, q[0])


# --------------------------------------------------------------------------- #
# nothing reaches the model
# --------------------------------------------------------------------------- #

def test_no_primitive_field_carries_the_contact_policy():
    for verb in (Grasp, Press, Probe):
        names = {f.name for f in dataclasses.fields(verb)}
        assert not names & {"policy", "contact_policy", "backoff",
                            "contact_backoff_m", "backoff_m"}, verb
