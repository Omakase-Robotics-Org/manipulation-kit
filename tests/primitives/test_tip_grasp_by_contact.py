"""The d1-2 fingertip trial of 2026-09-23 (02:05-02:10Z), replayed.

Two findings, one test group each:

1. THE JAW AXIS. Shu's photos show the open jaws spanning the slab's 91 mm
   side. The recorded plan put the jaws along base x (quaternion
   ``[0, 1, 0, 0]``: TCP x = base -x) across a slab DECLARED at yaw 90 deg
   (long side along base y), i.e. across its 55 mm; ``fits()`` agreed
   (55.1 mm). The kit's jaw axis, the executed posture and the gripper
   description's own prismatic finger joints all agree — pinned here — so
   the 91 mm the jaws met is the slab lying long side along base x, 90 deg
   from its declaration (the photos are taken from the robot's right, where
   "left-right" is the robot's forward axis). Declared as it lay, the kit
   squares the jaws across the 55 mm; the executed posture is refused.
2. THE HEIGHT. The plan stopped the tips 14.9 mm over the wagon (3 mm + a
   12 mm droop margin); the near arm did not sag and the jaws closed 7 mm
   above an 8 mm slab, three of three. A fingertip grasp onto a surface now
   finishes BY CONTACT: stop 15 mm up, then a ContactStep down to 5 mm past
   the modelled top that ends when the tips touch it.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from manipulation_kit.executor import (CONTACT, MAX_TRAVEL,
                                       KinematicExecutor, contact_report,
                                       leg_ticks, run)
from manipulation_kit.primitives import Approach, Grasp
from manipulation_kit.primitives import grasp_geometry as gg
from manipulation_kit.primitives.clearance import (ClearancePolicy, policy_of,
                                                   set_policy)
from manipulation_kit.primitives import orientation as ap
from manipulation_kit.primitives.types import ContactStep, JointStep, Plan
from manipulation_kit.world import (ALIASES, ArmView, GripperView, ObjectView,
                                    SurfaceView, WorldView)

SRC = Path(__file__).resolve().parents[2] / "src" / "manipulation_kit"
WHOLEBODY = SRC / "description" / "d1" / "d1_wholebody_gripper.urdf"

# --- the live trial, as tip_trial.py declared it (trace tip-20260923-100805)
TABLE_TOP = 0.166                    # wagon top in base at lift 0.205
OPEN_GAP_M = 0.0605                  # d1-2 driven-open gap
CARD_L, CARD_W, CARD_T = 0.091, 0.055, 0.008
CARD_XY = (0.403, 0.10)
#: the yaw the trial DECLARED. Not math.pi / 2: a quarter-turn slab sits
#: exactly on align_tool's half-turn fold, and the 4th decimal decides which
#: wrist the squared roll gets (exact pi/2 plans J5 = +173 deg) — this is the
#: number that reproduces the live joints to 0.002 deg.
CARD_YAW_RAD = 1.5708
#: the live plan's grasp waypoint quaternion (xyzw) and its final joints [deg]
LIVE_QUAT = (0.0, 1.0, 0.0, 0.0)
LIVE_GRASP_Q_DEG = (-12.314, -72.491, 42.606, -80.572, 2.845, -30.608, -48.299)
#: where trial 0's Grasp started: the Approach's standoff, both arms [deg]
LIVE_Q0_DEG = {"left": (-23.52, -71.13, 42.42, -84.49, -6.57, -34.72, -53.86),
               "right": (52.26, 87.38, -88.3, -114.32, -86.67, -1.1, -13.05)}

TABLE = SurfaceView("table", p=(0.503, 0.0, TABLE_TOP - 0.01),
                    size=(0.40, 0.60, 0.02))


def _card(yaw_rad: float) -> ObjectView:
    return ObjectView("card", p=(CARD_XY[0], CARD_XY[1],
                                 TABLE_TOP + CARD_T / 2),
                      size=(CARD_L, CARD_W, CARD_T),
                      r=R.from_euler("z", yaw_rad))


def _world(kin, objects):
    """The world trial 0's Grasp planned in: both arms at the live q0."""
    arms, grippers = [], []
    for side in ("left", "right"):
        kin.set_joints(side, np.radians(LIVE_Q0_DEG[side]))
        p, r = ap.tool_from_link7(*kin.ee_pose(side))
        arms.append(ArmView(side, joints=kin.joints(side), tool_p=p, tool_r=r,
                            mode="position"))
        grippers.append(GripperView(side, 0.0, jaw_gap_m=0.04,
                                    open_gap_m=OPEN_GAP_M))
    return WorldView.of(list(objects), arms=arms, grippers=grippers)


def _executed_r_tcp(kin, q_deg) -> R:
    saved = np.array(kin.joints("left"), dtype=float)
    try:
        kin.set_joints("left", np.radians(q_deg))
        return ap.tool_from_link7(*kin.ee_pose("left"))[1]
    finally:
        kin.set_joints("left", saved)


def _urdf_finger_axis(q_deg) -> np.ndarray:
    """The LEFT hand's finger travel in base, walked straight off the gripper
    description's prismatic joints — independent of ``orientation``."""
    root = ET.parse(WHOLEBODY).getroot()
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None
                                  else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None
                                  else "0 0 0").split()]
        a = j.find("axis")
        axis = ([float(v) for v in a.get("xyz").split()] if a is not None
                else [0.0, 0.0, 1.0])
        joints[j.find("child").get("link")] = (
            j.get("name"), j.find("parent").get("link"), np.array(xyz),
            R.from_euler("xyz", rpy), np.array(axis), j.get("type"))
    q = {f"Joint{i + 1}_R": math.radians(v) for i, v in enumerate(q_deg)}

    def rot(link):
        if link not in joints:
            return R.identity()
        name, parent, _xyz, r, axis, kind = joints[link]
        out = rot(parent) * r
        if kind == "revolute" and name in q:
            out = out * R.from_rotvec(axis * q[name])
        return out

    for link, (name, parent, _xyz, r, axis, kind) in joints.items():
        if kind == "prismatic" and name == "gripper_R_tcp_r_joint":
            return (rot(parent) * r).apply(axis)
    raise AssertionError("no left finger joint in the description")


# --------------------------------------------------------------------------- #
# 1. the jaw axis
# --------------------------------------------------------------------------- #

def test_the_live_tip_plan_presents_the_short_side_along_the_executed_jaw_axis(d1_arm):
    """Slab 91 x 55 at yaw 90 deg, left arm, contact=tip — the live plan.
    The presented width along the jaw axis of the EXECUTED posture is under
    the opening, and ``fits()`` measures that same axis."""
    card = _card(CARD_YAW_RAD)
    world = _world(d1_arm, [card, TABLE])
    plan = Grasp(object="card", side="left", contact="tip").plan(world, d1_arm)
    assert isinstance(plan, Plan), str(plan)
    grasp = next(w for w in plan.waypoints if w.label == "grasp")
    # the kit plans what it planned live: TCP x along base -x
    assert np.allclose(np.abs(grasp.r.as_quat()), LIVE_QUAT, atol=1e-5)

    for q_deg in (LIVE_GRASP_Q_DEG,
                  tuple(np.degrees([s for s in plan.steps
                                    if isinstance(s, JointStep)][-1].q))):
        r_exec = _executed_r_tcp(d1_arm, q_deg)
        jaw = ap.jaw_axis(r_exec)
        assert abs(float(jaw[0])) > 0.999, jaw          # base x
        # the gripper description's fingers travel along that same axis
        fingers = _urdf_finger_axis(q_deg)
        assert abs(float(np.dot(fingers, jaw))) > 0.999, (fingers, jaw)
        presented = card.extent_along(jaw, world.frames)
        assert presented == pytest.approx(CARD_W, abs=1e-3)
        assert presented <= gg.graspable_width_m(gg.TIP, OPEN_GAP_M)
        # fits() asks the same question about the same axis
        spec = gg.GraspSpec(ALIASES["down"], gg.TIP)
        assert ap.grasp_width(card, world.frames, r_exec) == pytest.approx(
            presented, abs=1e-12)
        assert gg.fits(card, world.frames, spec, r_exec,
                       open_gap_m=OPEN_GAP_M, support=TABLE) is None


def test_the_slab_as_it_lay_is_refused_at_the_live_posture_and_never_taken_across_91mm(d1_arm):
    """The photos: the jaws spanned the 91 mm side, i.e. the slab lay long
    side along base x (yaw 0). At the executed live posture that slab
    presents 91 mm and ``fits()`` refuses it. Declared as it lay, no roll the
    planner may take presents 91 mm: the squared roll turns the jaws across
    base y — which the left wrist cannot reach from here (coupled J7 limit),
    so the plan is REFUSED rather than run across the long side."""
    lay = _card(0.0)
    r_exec = _executed_r_tcp(d1_arm, LIVE_GRASP_Q_DEG)
    spec = gg.GraspSpec(ALIASES["down"], gg.TIP)
    frames = _world(d1_arm, [lay, TABLE]).frames
    assert ap.grasp_width(lay, frames, r_exec) == pytest.approx(CARD_L, abs=1e-3)
    refused = gg.fits(lay, frames, spec, r_exec, open_gap_m=OPEN_GAP_M,
                      support=TABLE)
    assert refused is not None and refused.code == "object_too_wide"

    world = _world(d1_arm, [lay, TABLE])
    rolls = gg.roll_candidates(lay, world.frames, spec, open_gap_m=OPEN_GAP_M)
    for roll in rolls:
        r = ap.grasp_orientation("left", [0, 0, -1], lay, world.frames,
                                 roll_rad=roll)
        assert abs(float(ap.jaw_axis(r)[1])) > 0.999   # across base y
        assert ap.grasp_width(lay, world.frames, r) == pytest.approx(
            CARD_W, abs=1e-3)
    plan = Grasp(object="card", side="left", contact="tip").plan(world, d1_arm)
    assert not plan.ok and plan.reason == "joint_limit", str(plan)


# --------------------------------------------------------------------------- #
# 2. the height: a fingertip grasp onto a surface finishes by contact
# --------------------------------------------------------------------------- #

@pytest.fixture
def operator_droop(d1_arm):
    """The live trial's policy: a 12 mm droop margin, restored afterwards
    (the arm model is shared by the whole session)."""
    saved = policy_of(d1_arm)
    set_policy(d1_arm, ClearancePolicy(droop_margin_m=0.012))
    yield d1_arm
    set_policy(d1_arm, saved)


def _contact_plan(kin):
    world = _world(kin, [_card(CARD_YAW_RAD), TABLE])
    return world, Grasp(object="card", side="left", contact="tip").plan(world, kin)


def test_a_tip_grasp_on_a_surface_descends_by_contact(operator_droop):
    d1_arm = operator_droop
    world, plan = _contact_plan(d1_arm)
    assert isinstance(plan, Plan), str(plan)
    legs = [s for s in plan.steps if isinstance(s, ContactStep)]
    assert len(legs) == 1
    leg = legs[0]
    assert leg.direction.vector()[2] == pytest.approx(-1.0, abs=1e-9)
    assert leg.criterion.joint_torque_nm == gg.TIP_CONTACT_NM
    # it starts with the tips TIP_SEARCH_START_M up — NOT 3 mm + the
    # 12 mm droop margin — and may travel 5 mm past the modelled top
    grasp = next(w for w in plan.waypoints if w.label == "grasp")
    tips = grasp.p + grasp.r.apply([0, 0, gg.TIP.offset_z_m - ap.TOOL_Z_M])
    assert tips[2] == pytest.approx(TABLE_TOP + gg.TIP_SEARCH_START_M,
                                    abs=1e-9)
    assert leg.max_travel_m == pytest.approx(
        gg.TIP_SEARCH_START_M + gg.CONTACT_OVERTRAVEL_M, abs=1e-9)
    # the solved leg really goes that far down
    assert leg.s[-1] == pytest.approx(leg.max_travel_m, abs=2e-3)
    # the close comes AFTER the search
    kinds = [type(s).__name__ for s in plan.steps]
    assert kinds.index("ContactStep") < max(
        i for i, s in enumerate(plan.steps)
        if type(s).__name__ == "GripStep")
    assert any("SEARCHES down by contact" in n for n in plan.notes)
    assert not any("droop" in n and "margin" in n and "mm" in n
                   and "keeps the pad tips" in n for n in plan.notes)


def test_a_pad_grasp_keeps_the_fixed_height_and_the_droop_margin(operator_droop):
    d1_arm = operator_droop
    block = ObjectView("block", p=(0.40, 0.10, TABLE_TOP + 0.025),
                       size=(0.04, 0.04, 0.05))
    world = _world(d1_arm, [block, TABLE])
    pad = Grasp(object="block", side="left").plan(world, d1_arm)
    assert isinstance(pad, Plan), str(pad)
    assert not any(isinstance(s, ContactStep) for s in pad.steps)
    grasp = next(w for w in pad.waypoints if w.label == "grasp")
    tips = grasp.p + grasp.r.apply([0, 0, gg.PAD.lead_m])
    # the pad path keeps 3 mm + the 12 mm droop margin over the table
    assert tips[2] == pytest.approx(TABLE_TOP + ap.SUPPORT_CLEARANCE_M + 0.012,
                                    abs=1e-9)
    assert not Grasp(object="block", side="left").by_contact(world)
    assert Grasp(object="block", side="left", contact="tip").by_contact(world)


def test_the_kinematic_mirror_plans_and_runs_it_and_says_no_contact_was_measured(operator_droop):
    d1_arm = operator_droop
    world, plan = _contact_plan(d1_arm)
    assert isinstance(plan, Plan), str(plan)
    ex = KinematicExecutor(d1_arm, open_gap_m=OPEN_GAP_M)
    for side in ("left", "right"):
        ex.kin.set_joints(side, world.arm(side).joints)
    report = run(plan, ex, kin=d1_arm)
    assert report.completed, report.error
    assert len(report.contacts) == 1
    c = report.contacts[0]
    assert c.made is False and c.stopped_by == MAX_TRAVEL


class _TouchesAt(KinematicExecutor):
    """A mirror whose tips meet a surface ``found_m`` along the leg."""

    def __init__(self, kin, found_m, **kw):
        super().__init__(kin, **kw)
        self.found_m = found_m

    def move_until(self, path, *, side, criterion, hz=50.0, kin=None,
                   direction=None, **_):
        timed = [(float(t), np.asarray(q, dtype=float)) for t, q in path]
        q_start = np.array(self.kin.joints(side), dtype=float)
        stop = timed[-1]
        p0 = ap.tool_from_link7(*self.kin.ee_pose(side))[0]
        for t, q in leg_ticks(timed, hz):
            self.kin.set_joints(side, q)
            p = ap.tool_from_link7(*self.kin.ee_pose(side))[0]
            if float(np.dot(p - p0, direction)) >= self.found_m:
                stop = (t, q)
                break
        self.kin.set_joints(side, stop[1])
        return contact_report(self.kin, side, timed, direction=direction,
                              q_start=q_start, q_stop=stop[1], made=True,
                              stopped_by=CONTACT, leg_t_s=timed[-1][0],
                              elapsed_s=stop[0], torque_nm=3.2,
                              detail="touched")


def test_the_jaws_close_where_the_tips_touched(operator_droop):
    """The live case: the real top is where the scene says and the tips
    touch it 15 mm into the search; the close happens there, not 7 mm up."""
    d1_arm = operator_droop
    world, plan = _contact_plan(d1_arm)
    ex = _TouchesAt(d1_arm, gg.TIP_SEARCH_START_M, open_gap_m=OPEN_GAP_M)
    for side in ("left", "right"):
        ex.kin.set_joints(side, world.arm(side).joints)
    report = run(plan, ex, kin=d1_arm)
    assert report.completed, report.error
    c = report.contacts[0]
    assert c.made and c.stopped_by == CONTACT
    p, r = ap.tool_from_link7(*d1_arm.ee_pose("left"))
    tips = p + r.apply([0, 0, gg.TIP.offset_z_m - ap.TOOL_Z_M])
    # the tips are on the table (within the leg's knot spacing), below the
    # 8 mm slab's top — not 14.9 mm up
    assert abs(tips[2] - TABLE_TOP) < 0.003, tips
    assert tips[2] < TABLE_TOP + CARD_T
    assert ex.grippers["left"] > 0.9       # and the jaws closed there
