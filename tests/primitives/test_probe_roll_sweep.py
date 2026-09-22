"""Probe tries the planner's rolls, not only the wrist's current one.

d1-2, 2026-09-22: every ``Probe`` from a real posture was refused at
``probe_start`` with ``ik_fail`` (position residual 0.5 mm, orientation
residual 0.25-1.2 rad). Probe turned the hand fingertips-down IN PLACE while
keeping the wrist's current jaw axis; Grasp and Approach try
``grasp_geometry.roll_candidates``. The live postures themselves stay
refused — the hand was 167 mm and 53 deg off where the Approach should have
left it (see ``tests/executors/test_end_of_motion_arrival.py``) — but a
tilted hand whose own roll cannot be turned down in place is not a reason to
refuse when a quarter turn plans.
"""

from __future__ import annotations

import numpy as np
import pytest

import manipulation_kit.primitives.contact as contact
from manipulation_kit.primitives import Probe
from manipulation_kit.primitives.orientation import tool_from_link7
from manipulation_kit.world import ArmView, GripperView, WorldView

def _right_home_deg():
    """The right arm stood at HOME during the live run; read it from the
    canonical file rather than inlining the angles (test_description_consistency)."""
    import json as _json
    from pathlib import Path as _Path
    cfg = _Path(__file__).resolve().parents[2] / "src" / "manipulation_kit" / "config" / "home_pose.json"
    pose = _json.load(open(cfg))["home_pose"]
    return [round(float(v), 2) for v in pose[7:14]]


RIGHT_Q0_DEG = _right_home_deg()
#: a tilted left hand over the wagon whose own roll does not plan down
TILTED_DEG = [17.4, -75.4, 10.6, -68.9, -10.0, 59.0, -79.0]
#: d1-2 after the Approach: the plan's final posture with J7 at -39.5 deg
LIVE_AFTER_APPROACH_DEG = [8.333, -61.29, -6.316, -84.468, 9.737, 55.223, -39.5]


def _world(kin, left_deg):
    saved = {s: np.array(kin.joints(s)) for s in ("left", "right")}
    arms, grippers = [], []
    try:
        for side, q in (("left", left_deg), ("right", RIGHT_Q0_DEG)):
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


def test_the_rolls_are_the_wrists_own_first_then_the_quarter_turns():
    assert contact.PROBE_ROLLS_RAD[0] == 0.0
    assert set(np.round(np.degrees(contact.PROBE_ROLLS_RAD))) == {
        0.0, 90.0, -90.0, 180.0}


def test_a_tilted_hand_probes_after_a_quarter_turn(d1_arm, monkeypatch):
    world = _world(d1_arm, TILTED_DEG)
    probe = Probe(side="left", direction="down", max_travel_m=0.03)
    monkeypatch.setattr(contact, "PROBE_ROLLS_RAD", (0.0,))
    assert not probe.plan(world, d1_arm).ok, "the own-roll-only probe plans"
    monkeypatch.undo()
    plan = probe.plan(world, d1_arm)
    assert plan.ok, plan
    assert any("rolled" in note for note in plan.notes)


def test_the_live_post_approach_posture_is_still_refused(d1_arm):
    """Not a roll problem: from where the hand actually was, fingertips-down
    in place does not plan at any of the four rolls. The fix for that is the Approach
    arriving, not the Probe."""
    plan = Probe(side="left", direction="down", max_travel_m=0.03).plan(
        _world(d1_arm, LIVE_AFTER_APPROACH_DEG), d1_arm)
    assert not plan.ok
    assert plan.reason in ("ik_fail", "guard_reject")
