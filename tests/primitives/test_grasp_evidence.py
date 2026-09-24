"""A grasp's hold is graded on the stroke, not on the jaw gap alone.

d1-2, 2026-09-24 (``servo-judgeonly-2``, turn 6): ``grasp(tape, right, down,
soft, tip)`` stalled the jaws at 48 mm, inside the 46-54 mm window of a 50 mm
declaration, and the verifier said ``holding``. The roll never left the table:
the wrist locates after the lift (turns 9 and 10) put it on the table, and
the live run's head locate found it where it had been. The declared centre
was 55-57 mm short of the roll (the wrist-locate bug), so the jaws closed on
the roll's near edge.

What the record itself says about the fingers: ``tool_p`` is the PAD CENTRE
(z 0.196), and the tips are 29 mm further along the approach, at z 0.167 —
the table, which the fingertip search touched 1.4 mm above its modelled top.
Against the DECLARED roll (top at 0.192) that is 25 mm of insertion: at
closure, nothing the robot measured distinguished this from a hold, and the
closure-time criteria pass on it (test 1). A rim pinch in the sense of tips
stopped ON the roll is caught (tests 2-4); the hold in this run is refuted
after the lift, by the look (``tests/agent/test_hold_sighting.py``).
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from manipulation_kit.agent.robot import objects_from
from manipulation_kit.executor import ArrivalReport, ContactReport, RunReport
from manipulation_kit.primitives import Grasp
from manipulation_kit.primitives import grasp_geometry as gg
from manipulation_kit.primitives.verifiers import SEARCH_STOP_TOL_M
from manipulation_kit.world import (ArmView, ContactView, FrameGraph,
                                    GripperView, ObjectView, WorldView)

DATA = (Path(__file__).resolve().parents[1] / "data"
        / "grasp_rim_pinch_d1_2_20260924.json")
PADS_DOWN = R.from_quat([0.7071068, 0.7071068, 0.0, 0.0])


def _load():
    return json.loads(DATA.read_text(encoding="utf-8"))


def world_of(doc) -> WorldView:
    """A trace record's ``world`` / ``observation_after`` -> a WorldView."""
    stamp = float(doc["stamp"])
    arms = [ArmView(a["side"], joints=np.radians(a["joints_deg"]),
                    tool_p=a.get("tool_p"),
                    tool_r=(R.from_quat(a["tool_quat_xyzw"])
                            if a.get("tool_quat_xyzw") else None),
                    mode=a.get("mode", "unknown"),
                    stationary=a.get("stationary", True))
            for a in doc["arms"]]
    grippers = [GripperView(g["side"], g["closedness"],
                            holding=g.get("holding", False),
                            jaw_gap_m=g.get("jaw_gap_m"),
                            held_object=g.get("held_object"),
                            grip=g.get("grip", "firm"),
                            jaw_stalled=g.get("jaw_stalled"),
                            open_gap_m=g.get("open_gap_m"))
                for g in doc["grippers"]]
    contacts = []
    for c in doc.get("contacts") or ():
        travel = -np.asarray(c["normal"], dtype=float)
        contacts.append(ContactView(
            c["side"], c["made"], c["p"],
            np.asarray(c["p"]) - travel * gg.PAD.lead_m, c["normal"],
            c["stopped_by"], c.get("travel_m", float("nan")),
            c.get("torque_nm", float("nan")), verb=c.get("verb", ""),
            stamp=stamp))
    world = WorldView.of(objects_from({"objects": doc["objects"]}),
                         frames=FrameGraph(now=stamp), arms=arms,
                         grippers=grippers, stamp=stamp,
                         revision=int(doc.get("revision", 0)))
    return world.with_(contacts=tuple(contacts)) if contacts else world


def run_of(doc) -> RunReport:
    """A trace record's ``run`` -> the RunReport the executor returned."""
    nan = float("nan")
    arrivals = tuple(ArrivalReport(
        bool(a["arrived"]), math.radians(a.get("worst_error_deg") or 0.0),
        waited_s=a.get("waited_s", 0.0), detail=a.get("detail", ""),
        tool_error_m=nan if a.get("tool_error_m") is None else a["tool_error_m"],
        waypoint_label=a.get("waypoint_label", ""),
        tool_along_m=nan if a.get("tool_along_m") is None else a["tool_along_m"])
        for a in doc["arrivals"])
    contacts = tuple(ContactReport(
        bool(c["made"]), c["p_tool"], c["travel_m"], c["normal_hint"],
        c["torque_nm"], c["stopped_by"], side=c["side"])
        for c in doc.get("contacts") or ())
    return RunReport(doc["primitive"], doc["side"], bool(doc["completed"]),
                     int(doc["steps_sent"]), stop_reason=doc["stop_reason"],
                     stopped_at=doc["stopped_at"], arrivals=arrivals,
                     contacts=contacts, error=doc.get("error", ""))


def turn6():
    doc = _load()["grasp"]
    grasp = Grasp(**doc["choice"]["arguments"])
    return (grasp, world_of(doc["world_before"]), world_of(doc["world_after"]),
            run_of(doc["run"]), doc)


def _moved_tool(world: WorldView, side: str, tool_z: float) -> WorldView:
    """``world`` with ``side``'s measured tool point at height ``tool_z``
    (and its grasp contact moved with it)."""
    arms = {}
    for s, arm in world.arms.items():
        if s == side:
            p = np.array(arm.tool_p, dtype=float)
            p[2] = tool_z
            arm = ArmView(s, joints=arm.joints, tool_p=p, tool_r=arm.tool_r)
        arms[s] = arm
    contacts = tuple(
        ContactView(c.side, c.made,
                    [c.p[0], c.p[1], tool_z - gg.PAD.lead_m],
                    [c.p_tool[0], c.p_tool[1], tool_z], c.normal,
                    c.stopped_by, c.travel_m, c.torque_nm, verb=c.verb,
                    stamp=c.stamp)
        if c.side == side else c for c in world.contacts)
    return world.with_(arms=arms, contacts=contacts)


def _run_stopped_at(run: RunReport, tool_z: float) -> RunReport:
    import dataclasses
    return dataclasses.replace(run, contacts=tuple(
        dataclasses.replace(c, p_tool=[c.p_tool[0], c.p_tool[1], tool_z])
        for c in run.contacts))


# --------------------------------------------------------------------------- #
# the recorded turn 6
# --------------------------------------------------------------------------- #

def test_turn6_as_recorded_the_tips_reached_the_table():
    """The recorded numbers, replayed: tips 25 mm into the DECLARED roll, the
    search stopped on the table. Every closure-time criterion passes, and the
    record now says so criterion by criterion — the refutation of this hold
    comes after the lift, from the look."""
    grasp, w0, w1, run, doc = turn6()
    assert doc["verdict_recorded"]["verdict"] == "true"
    report = grasp.verifier(w0)(w1, run)
    m = report.measured
    assert report.verdict == "true", report.reason
    assert m["tip_z_m"] == pytest.approx(0.167, abs=0.001)
    assert m["object_top_z_m"] == pytest.approx(0.192, abs=0.001)
    assert m["insertion_m"] == pytest.approx(0.025, abs=0.001)
    assert m["insertion_min_m"] == pytest.approx(0.0065, abs=1e-4)
    assert m["approach_completed"] is True
    assert {k: v["verdict"] for k, v in m["checks"].items()} == {
        "jaw_gap": "pass", "insertion": "pass", "approach": "pass"}
    approach = m["checks"]["approach"]
    assert approach["search_contact"] is True
    assert approach["search_stop_above_floor_m"] == pytest.approx(0.0014,
                                                                  abs=5e-4)
    # the measured-over-declared width rule is untouched
    assert m["fit"] == "held" and m["matches_declaration"] is True
    assert m["width_window_m"] == [0.046, 0.054]


def test_tips_stopped_on_the_roll_are_a_rim_pinch_not_a_hold():
    """The rim pinch the jaw gap cannot see: the same 48 mm stall in the
    same window, with the FINGER TIPS at z 0.196 — on the roll, not in it."""
    grasp, w0, w1, run, _ = turn6()
    tool_z = 0.196 + gg.PAD.lead_m
    report = grasp.verifier(w0)(_moved_tool(w1, "right", tool_z),
                                _run_stopped_at(run, tool_z))
    m = report.measured
    assert report.verdict == "false"
    assert m["tip_z_m"] == pytest.approx(0.196, abs=1e-3)
    assert m["insertion_m"] == pytest.approx(-0.004, abs=1e-3)
    checks = {k: v["verdict"] for k, v in m["checks"].items()}
    # WHICH one failed, and which did not
    assert checks == {"jaw_gap": "pass", "insertion": "fail",
                      "approach": "fail"}
    assert "stalled at 48 mm" in report.reason
    assert "collision, not a grasp" in report.reason


def test_insertion_alone_refuses_the_rim_pinch():
    """No run report and no contact record — the world-only call a
    simulator makes: the tip height alone decides."""
    grasp, w0, w1, _run, _ = turn6()
    moved = _moved_tool(w1, "right", 0.196 + gg.PAD.lead_m)
    report = grasp.verifier(w0)(moved.with_(contacts=()))
    m = report.measured
    assert report.verdict == "false"
    assert m["checks"]["approach"]["verdict"] == "unmeasured"
    assert m["approach_completed"] is None
    assert m["checks"]["insertion"]["verdict"] == "fail"
    assert "stalled at 48 mm with the finger tips -4 mm past 'tape''s top" \
        in report.reason
    assert "(insertion -4.0 mm; a hold needs 6.5 mm): a pinch on its rim" \
        in report.reason


def test_a_search_that_stopped_on_the_object_is_a_collision():
    """The fingertip search stopped by contact at the roll's top, with the
    tips (by some other error) still inside the minimum: the approach
    criterion refuses it on its own."""
    grasp, w0, w1, run, _ = turn6()
    import dataclasses
    tool_z = 0.166 + SEARCH_STOP_TOL_M + 0.006 + gg.PAD.lead_m
    stopped = dataclasses.replace(run, contacts=tuple(
        dataclasses.replace(c, p_tool=[c.p_tool[0], c.p_tool[1], tool_z])
        for c in run.contacts))
    report = grasp.verifier(w0)(w1, stopped)
    m = report.measured
    assert report.verdict == "false"
    assert m["checks"]["insertion"]["verdict"] == "pass"
    assert m["checks"]["approach"]["verdict"] == "fail"
    assert m["approach_completed"] is False
    assert "stopped by contact 16 mm above table's top" in report.reason


def test_an_arrival_short_of_the_grasp_point_is_not_a_hold():
    grasp, w0, w1, run, _ = turn6()
    import dataclasses
    arrivals = tuple(dataclasses.replace(a, tool_along_m=-0.015)
                     if a.waypoint_label == "grasp" else a
                     for a in run.arrivals)
    report = grasp.verifier(w0)(w1, dataclasses.replace(run,
                                                        arrivals=arrivals))
    assert report.verdict == "false"
    assert report.measured["checks"]["approach"]["verdict"] == "fail"
    assert "15 mm short of the grasp point" in report.reason


def test_a_run_that_stopped_is_not_a_hold():
    grasp, w0, w1, run, _ = turn6()
    import dataclasses
    report = grasp.verifier(w0)(w1, dataclasses.replace(
        run, completed=False, stop_reason="barrier_failed",
        error="the right tool point is 27 mm from the grasp pose"))
    assert report.verdict == "false"
    assert "barrier_failed" in report.reason


def test_no_tool_pose_is_unknown_never_true():
    grasp, w0, w1, run, _ = turn6()
    bare = w1.with_(arms={s: ArmView(s, joints=a.joints)
                          for s, a in w1.arms.items()})
    report = grasp.verifier(w0)(bare, run)
    assert report.verdict == "unknown"
    assert report.measured["checks"]["insertion"]["verdict"] == "unmeasured"


def test_a_genuine_pad_grasp_is_true(d1_arm):
    """A 40 mm cube taken at the pad centre, tool point at its centre: the
    tips 49 mm past its top, jaws stalled at its width, the run arrived."""
    from manipulation_kit.world import SurfaceView
    table = SurfaceView("table", p=[0.45, 0.0, 0.156], size=[0.4, 0.6, 0.02],
                        plane_source="declared")
    cube = ObjectView("cube", p=[0.40, -0.12, 0.186], size=[0.04] * 3,
                      provenance="observed")
    w0 = WorldView.of([table, cube], frames=FrameGraph(),
                      arms=[ArmView("right", joints=np.zeros(7),
                                    tool_p=[0.40, -0.12, 0.30],
                                    tool_r=PADS_DOWN)],
                      grippers=[GripperView("right", 0.0, open_gap_m=0.064)])
    w1 = w0.with_(arms={"right": ArmView("right", joints=np.zeros(7),
                                         tool_p=[0.40, -0.12, 0.186],
                                         tool_r=PADS_DOWN)},
                  grippers={"right": GripperView(
                      "right", 0.4, holding=True, jaw_gap_m=0.0405,
                      jaw_stalled=True, held_object="cube",
                      open_gap_m=0.064)})
    run = RunReport("grasp", "right", True, 6, arrivals=(
        ArrivalReport(True, 0.0, waypoint_label="standoff", tool_along_m=0.0),
        ArrivalReport(True, 0.0, waypoint_label="grasp",
                      tool_along_m=-0.002)))
    report = Grasp(object="cube", side="right").verifier(w0)(w1, run)
    m = report.measured
    assert report.verdict == "true", report.reason
    assert m["insertion_m"] == pytest.approx(0.049, abs=1e-3)
    assert m["insertion_min_m"] == pytest.approx(gg.GRASP_DEPTH_MIN_M)
    assert all(c["verdict"] == "pass" for c in m["checks"].values())


def test_a_handover_receiver_is_graded_on_the_jaws_alone():
    """``Holding`` without a stroke (a handover's receiving hand) carries no
    insertion or approach criterion — and is not made UNKNOWN by them."""
    from manipulation_kit.primitives import verifiers as V
    cube = ObjectView("cube", p=[0.4, 0.0, 0.3], size=[0.04] * 3)
    w = WorldView.of([cube], frames=FrameGraph(), grippers=[GripperView(
        "left", 0.4, holding=True, jaw_gap_m=0.04, jaw_stalled=True,
        held_object="cube")])
    report = V.Holding("handover", w, "left", cube)(w)
    assert report.verdict == "true"
    assert set(report.measured["checks"]) == {"jaw_gap"}


# --------------------------------------------------------------------------- #
# the geometry
# --------------------------------------------------------------------------- #

def test_fingertip_point_leads_the_pad_centre_along_the_approach():
    tip = gg.fingertip_point([0.35, -0.12, 0.196], PADS_DOWN)
    assert tip == pytest.approx([0.35, -0.12, 0.196 - 0.029], abs=1e-6)
    sideways = R.from_euler("y", 90, degrees=True)      # TCP +z -> base +x
    tip = gg.fingertip_point([0.30, 0.0, 0.25], sideways)
    assert tip == pytest.approx([0.329, 0.0, 0.25], abs=1e-6)


def test_upright_cylinder_top_is_its_near_face_for_a_descent():
    """The tape roll: 50 mm across, 26 mm tall, standing on the table
    (centre z 0.179, top 0.192)."""
    frames = FrameGraph()
    roll = ObjectView("tape", p=[0.351, -0.118, 0.179],
                      size=[0.05, 0.05, 0.026])
    down = [0.0, 0.0, -1.0]
    assert gg.near_face_along(roll, frames, down) == pytest.approx(-0.192)
    assert gg.insertion_along(roll, frames, down,
                              [0.351, -0.118, 0.167]) == pytest.approx(0.025)
    assert gg.insertion_along(roll, frames, down,
                              [0.351, -0.118, 0.196]) == pytest.approx(-0.004)
    # 8 mm, capped at a quarter of 26 mm
    assert gg.min_insertion_m(roll, frames, down) == pytest.approx(0.0065)


def test_cylinder_lying_down_presents_its_diameter_and_its_length():
    """A 200 mm bottle, 60 mm across, lying along x and yawed 30 degrees:
    from above it is 60 mm deep; from the side (travel +y) it is
    200 sin 30 + 60 cos 30 = 152 mm long."""
    frames = FrameGraph()
    bottle = ObjectView("bottle", p=[0.4, 0.0, 0.196],
                        size=[0.2, 0.06, 0.06], r=R.from_euler("z", 30,
                                                               degrees=True))
    assert gg.near_face_along(bottle, frames, [0, 0, -1]) == pytest.approx(
        -0.226)
    assert gg.min_insertion_m(bottle, frames, [0, 0, -1]) == pytest.approx(
        gg.GRASP_DEPTH_MIN_M)
    side = [0.0, 1.0, 0.0]
    extent = 0.2 * 0.5 + 0.06 * math.cos(math.radians(30))
    assert gg.near_face_along(bottle, frames, side) == pytest.approx(
        -extent / 2.0)
    assert gg.insertion_along(bottle, frames, side,
                              [0.4, -extent / 2.0 + 0.01, 0.196]) \
        == pytest.approx(0.01)


def test_box_on_its_side_and_a_thin_card():
    frames = FrameGraph()
    # 100x40x20 box rolled 90 degrees about x: it now stands 40 mm tall
    box = ObjectView("box", p=[0.4, 0.0, 0.186], size=[0.1, 0.04, 0.02],
                     r=R.from_euler("x", 90, degrees=True))
    assert gg.near_face_along(box, frames, [0, 0, -1]) == pytest.approx(
        -0.206)
    # a 6 mm card needs a quarter of its thickness, not 8 mm
    card = ObjectView("card", p=[0.4, 0.0, 0.169], size=[0.09, 0.06, 0.006])
    assert gg.min_insertion_m(card, frames, [0, 0, -1]) == pytest.approx(
        0.0015)
    assert gg.insertion_along(card, frames, [0, 0, -1],
                              [0.4, 0.0, 0.1674]) == pytest.approx(0.0046)
