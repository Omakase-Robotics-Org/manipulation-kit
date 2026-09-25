"""The D1 wrist roll J7 reaches less far the more the wrist is pitched (J6).

Measured by hand on d1-2, 2026-09-22 23:37-23:44Z (Shu): the hand, camera
plate and cables catch on the J6 link, so |J7| stops at 65 deg with J6 = 30
deg and at 39 deg with J6 = 55 deg, the same for either sign. The box in
``home_pose.json`` (J6 +/-60, J7 +/-90) let the live Approach of 22:42Z solve
J7 = -90 at J6 = 55.2; the wrist stopped at -39.5 and the tool ended 167 mm
off (``tests/data/d1_2_approach_20260922``). These tests pin the coupled
limit as data, in the IK, in the posture check, and on that Approach.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from manipulation_kit.agent.robot import KinematicMirror, load_scene
from manipulation_kit.arms.coupled_limits import (wrist_roll_limit,
                                                  wrist_roll_limit_deg)
from manipulation_kit.arms.kinematics import JOINT_LIMIT as IK_JOINT_LIMIT
from manipulation_kit.primitives import Approach, Kin, joint_ramp
from manipulation_kit.primitives.clearance import (POLICY_ATTR,
                                                   ClearancePolicy)
from manipulation_kit.primitives.types import (JOINT_LIMIT, PLAN_REASONS,
                                               JointStep)
from manipulation_kit.world import ArmView, GripperView, WorldView
from manipulation_kit.primitives.orientation import tool_from_link7

REPO = Path(__file__).resolve().parents[2]
RECORD = json.loads((REPO / "tests" / "data" / "d1_2_approach_20260922"
                     / "approach.json").read_text())
#: the live Approach's planned final posture (J7 = -90 at J6 = 55.2)
LIVE_FINAL_DEG = [8.33, -61.29, -6.32, -84.47, 9.74, 55.22, -90.0]
#: the d1-2 probe spot of scratchpad pose_for_probe.py: 0.10 m from the
#: wagon's near edge (chassis front 0.303 + gap 0) and 0.10 m left of centre
PROBE_SPOT_XY = (0.403, 0.10)


def _j67(q):
    return float(np.degrees(q[5])), float(np.degrees(q[6]))


def _within(q, tol_deg=1e-6):
    j6, j7 = _j67(q)
    return abs(j7) <= wrist_roll_limit_deg(j6) + tol_deg


# -- the table ------------------------------------------------------------- #

def test_the_two_measured_points_are_respected_with_the_margin():
    lim = wrist_roll_limit()
    for sign in (1.0, -1.0):
        assert lim.hardware_limit_deg(sign * 30.0) == pytest.approx(65.0)
        assert lim.hardware_limit_deg(sign * 55.0) == pytest.approx(39.0)
        assert wrist_roll_limit_deg(sign * 30.0) == pytest.approx(60.0)
        assert wrist_roll_limit_deg(sign * 55.0) == pytest.approx(34.0)
    assert lim.margin_deg == 5.0
    assert "measured d1-2 2026-09-22 by hand, two points" in lim.provenance
    # linear between them: 26 deg of roll lost over 25 deg of pitch
    assert wrist_roll_limit_deg(42.5) == pytest.approx(52.0 - 5.0)


def test_below_the_measured_range_is_extrapolated_capped_and_flagged():
    lim = wrist_roll_limit()
    assert not lim.measured(0.0) and not lim.measured(29.0)
    assert lim.measured(30.0) and lim.measured(-55.0)
    assert wrist_roll_limit_deg(0.0) == pytest.approx(90.0)   # the box binds
    assert wrist_roll_limit_deg(20.0) == pytest.approx(65 + 10 * 26 / 25 - 5)
    assert "EXTRAPOLATED" in lim.describe_source(10.0)


def test_joint_limit_is_a_plan_reason():
    assert JOINT_LIMIT == IK_JOINT_LIMIT == "joint_limit"
    assert JOINT_LIMIT in PLAN_REASONS


# -- the posture check ----------------------------------------------------- #

def test_the_recorded_live_final_posture_is_refused(d1_arm):
    q = np.radians(LIVE_FINAL_DEG)
    why = d1_arm.posture_violation("left", q)
    assert why is not None and "coupled wrist_roll limit" in why, why
    assert "J6=+55.2" in why and "J7=-90.0" in why
    # the same posture as a joint-space goal is refused with the typed reason
    world = _world(d1_arm, {"left": RECORD["q0_deg"]["left"],
                            "right": RECORD["q0_deg"]["right"]})
    with Kin(d1_arm, world) as kin:
        steps, error = joint_ramp(kin, "left", q, primitive="test",
                                  label="the live final posture")
    assert steps == [] and error is not None
    assert error.reason == JOINT_LIMIT and "wrist_roll" in error.detail


def test_the_measured_points_themselves_are_postures_inside_the_margin(d1_arm):
    for j6, j7_ok, j7_bad in ((30.0, 59.5, 61.0), (55.0, 33.5, 35.0)):
        for s6 in (1.0, -1.0):
            for s7 in (1.0, -1.0):
                q = np.radians([8.3, -61.3, -6.3, -84.5, 9.7, s6 * j6, s7 * j7_ok])
                assert d1_arm.posture_violation("left", q) is None
                q[6] = np.radians(s7 * j7_bad)
                assert d1_arm.posture_violation("left", q) is not None


def test_a_posture_at_j6_zero_with_j7_80_is_allowed(d1_arm):
    q = np.radians([8.3, -61.3, -6.3, -84.5, 9.7, 0.0, 80.0])
    assert d1_arm.posture_violation("left", q) is None
    q[6] = np.radians(-80.0)
    assert d1_arm.posture_violation("left", q) is None


# -- the IK ---------------------------------------------------------------- #

def test_ik_does_not_return_the_live_roll(d1_arm):
    """Asked for the pose the live plan's final posture reaches, seeded AT that
    posture (where a box-only solver is converged before it starts), the
    solver either finds a posture inside the coupled limit or refuses with
    ``joint_limit`` naming it — never J7 = -90 at J6 = 55."""
    q_live = np.radians(LIVE_FINAL_DEG)
    with d1_arm.lock:
        d1_arm.set_joints("left", q_live)
        p, r = d1_arm.ee_pose("left")
        result = d1_arm.solve_ee("left", p, r, q0=q_live)
    if result.ok:
        assert _within(result.q), _j67(result.q)
    else:
        assert result.reason == IK_JOINT_LIMIT, result
        assert "wrist_roll" in result.detail
        np.testing.assert_allclose(d1_arm.joints("left"), q_live)


def test_a_box_only_model_would_have_returned_the_live_roll():
    """The test above would have failed at fd17a5c: the same solve on the box
    alone hands back J7 = -90 at J6 = 55.2."""
    from manipulation_kit.arms.d1.arm.kinematics import build_kinematics
    box = build_kinematics(guard=None, find_ready=False, coupled=(), quiet=True)
    q_live = np.radians(LIVE_FINAL_DEG)
    box.set_joints("left", q_live)
    p, r = box.ee_pose("left")
    result = box.solve_ee("left", p, r, q0=q_live)
    assert result.ok and not _within(result.q)


def test_ik_solutions_near_the_measured_points_stay_inside(d1_arm):
    """Targets that a box-only solve reaches with |J7| past the coupled limit
    (J6 at the two measured pitches, J7 10 deg past the stop): every accepted
    solve is inside the limit."""
    base = [8.3, -61.3, -6.3, -84.5, 9.7]
    for j6, stop in ((30.0, 65.0), (55.0, 39.0)):
        for s in (1.0, -1.0):
            q_t = np.radians(base + [j6, s * (stop + 10.0)])
            q_seed = np.radians(base + [j6, s * (stop - 20.0)])
            with d1_arm.lock:
                d1_arm.set_joints("left", q_t)
                p, r = d1_arm.ee_pose("left")
                d1_arm.set_joints("left", q_seed)
                result = d1_arm.solve_ee("left", p, r, q0=q_seed)
            if result.ok:
                assert _within(result.q), (j6, s, _j67(result.q))
            else:
                assert result.reason in (IK_JOINT_LIMIT, "ik_fail",
                                         "guard_reject"), result


# -- the d1-2 probe-spot Approach ----------------------------------------- #

def _world(kin, q_deg):
    saved = {s: np.array(kin.joints(s)) for s in ("left", "right")}
    arms, grippers = [], []
    try:
        for side, q in q_deg.items():
            kin.set_joints(side, np.radians(q))
            p, r = tool_from_link7(*kin.ee_pose(side))
            arms.append(ArmView(side, joints=np.radians(q), tool_p=p, tool_r=r,
                                mode="position", error_code=0))
            grippers.append(GripperView(side, 1.0, holding=False,
                                        open_gap_m=0.0605))
    finally:
        for side, q in saved.items():
            kin.set_joints(side, q)
    return WorldView.of([], arms=arms, grippers=grippers)


def _probe_spot_plan(kin, q0_deg, xy=PROBE_SPOT_XY):
    """``pose_for_probe.py``'s Approach on the d1-2 tape/cup scene, offline."""
    scene = load_scene(REPO / "examples" / "agent" / "scenes"
                       / "d1-2_tape_cup.json",
                       profile=REPO / "tests" / "data"
                       / "d1-2.camera_calibration.json")
    scene["objects"].append({"name": "probe_spot", "kind": "object",
                             "frame_id": "base", "p": [xy[0], xy[1], 0.171],
                             "size": [0.04, 0.04, 0.01], "yaw_rad": 0.0})
    for side, q in q0_deg.items():
        kin.set_joints(side, np.radians(q))
    world = KinematicMirror(kin, scene=scene).world()
    return Approach(object="probe_spot", side="left", direction="down",
                    contact="pad").plan(world, kin)


@pytest.mark.parametrize("start", ["recorded_q0", "home"])
def test_the_d1_2_probe_spot_approach_plans_inside_the_coupled_limit(
        d1_arm, monkeypatch, start):
    monkeypatch.setattr(d1_arm, POLICY_ATTR,
                        ClearancePolicy(droop_margin_m=0.012), raising=False)
    q0 = (RECORD["q0_deg"] if start == "recorded_q0" else
          {s: list(np.degrees(d1_arm.home(s))) for s in ("left", "right")})
    plan = _probe_spot_plan(d1_arm, q0)
    assert plan.ok, plan
    for step in plan.steps:
        if isinstance(step, JointStep) and step.side == "left":
            assert _within(step.q), _j67(step.q)
    final = np.asarray(plan.final_joints()["left"])
    assert _within(final)
    # the live plan ended at J7 = -90; this one does not
    assert abs(_j67(final)[1]) < 85.0
    # it sits on the (margin-reduced) limit, and the plan says so
    assert any("near the coupled wrist_roll limit" in n for n in plan.notes), \
        plan.notes
