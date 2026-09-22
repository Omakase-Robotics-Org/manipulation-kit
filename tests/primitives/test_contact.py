"""probe / press: plans, the default ``move_until``, and the scene they feed.

Everything here runs against :class:`SurfaceRig`, a kinematic mirror with ONE
rigid plane in it and a scripted torque: a commanded posture whose leading
fingertip would be past the plane is NOT reached — the measured joints stay
where the fingertip met it — and the blocked joint reports a torque rise
proportional to how far the command is past the surface. That is the whole
physics a position-controlled arm pressing on a stiff table has, and it is
enough to tell a report measured from the state from one read off the
command.
"""

from __future__ import annotations

import math
import re
from typing import Dict, List, Optional, Tuple

import numpy as np
import pytest

from manipulation_kit.executor import (CONTROLLER_FAULT, TRANSPORT_ERROR,
                                       HandState, JointState,
                                       KinematicExecutor, RawState,
                                       RecordingExecutor, StreamingContact,
                                       run)
from manipulation_kit.primitives import (BY_VERB, ContactStep, Press, Probe,
                                         decode, fit_plane, record_contacts,
                                         tool_schemas)
from manipulation_kit.primitives.grasp_geometry import PAD
from manipulation_kit.primitives.orientation import tool_from_link7
from manipulation_kit.world import (ALIASES, ArmView, GripperView, ObjectView,
                                    SurfaceView, WorldView)

SIDES = ("left", "right")


# --------------------------------------------------------------------------- #
# the rig
# --------------------------------------------------------------------------- #

def _tip(kin, side: str, q) -> np.ndarray:
    saved = np.array(kin.joints(side), dtype=float)
    try:
        kin.set_joints(side, np.asarray(q, dtype=float))
        p, r = tool_from_link7(*kin.ee_pose(side))
    finally:
        kin.set_joints(side, saved)
    return p + r.as_matrix()[:, 2] * PAD.lead_m


class SurfaceRig(StreamingContact, KinematicExecutor):
    """A mirror with one rigid plane, a scripted torque, and the kit's default
    ``move_until`` (it streams and it reports torque, so it opts in)."""

    HZ = 50.0

    def __init__(self, kin, *, point, normal, stiffness_nm_per_m: float = 2000.0,
                 joint: int = 1, fault_after: Optional[int] = None):
        super().__init__(kin)
        self.point = np.asarray(point, dtype=float)
        n = np.asarray(normal, dtype=float)
        self.normal = n / np.linalg.norm(n)
        self.k = float(stiffness_nm_per_m)
        self.joint = int(joint)
        #: a gravity-like load the arm carries all the time, so the watch
        #: has to subtract a baseline to see anything
        self.load = np.array([0.3, 5.0, -0.8, 2.1, 0.0, 0.4, 0.0])
        self.measured: Dict[str, np.ndarray] = {
            s: np.array(kin.joints(s), dtype=float) for s in SIDES}
        self.velocity: Dict[str, np.ndarray] = {s: np.zeros(7) for s in SIDES}
        self.rise: Dict[str, float] = {s: 0.0 for s in SIDES}
        self.commanded: Dict[str, np.ndarray] = dict(self.measured)
        self.fault_after = fault_after
        self.sends = 0

    def penetration(self, side: str, q) -> float:
        return float(np.dot(self.point - _tip(self.kin, side, q), self.normal))

    def send_joints(self, q16, *, t: float) -> None:
        q16 = np.asarray(q16, dtype=float).reshape(16)
        self.sent.append((float(t), q16.copy()))
        self.sends += 1
        for side, part in (("left", slice(0, 7)), ("right", slice(8, 15))):
            q = q16[part]
            self.commanded[side] = q.copy()
            depth = self.penetration(side, q)
            before = self.measured[side]
            if depth <= 0.0:
                self.measured[side] = q.copy()
                self.rise[side] = 0.0
            else:
                # BLOCKED: the arm stays where the fingertip met the plane
                # and the controller's effort grows with the command's lead.
                self.rise[side] = self.k * depth
            self.velocity[side] = (self.measured[side] - before) * self.HZ
            self.kin.set_joints(side, self.measured[side])
        self.grippers["left"] = float(q16[7])
        self.grippers["right"] = float(q16[15])

    def state(self) -> RawState:
        faulted = self.fault_after is not None and self.sends >= self.fault_after
        arms = {}
        for side in SIDES:
            torque = self.load.copy()
            torque[self.joint] += self.rise[side]
            arms[side] = JointState(
                q=self.measured[side], qd=self.velocity[side],
                torque_nm=torque, mode="error" if faulted else "position",
                error_code=7 if faulted else 0,
                stationary=not np.any(np.abs(self.velocity[side]) > 1e-6))
        hands = {s: HandState(closedness=self.grippers[s],
                              commanded=self.grippers[s], holding=False)
                 for s in SIDES}
        return RawState(arms, hands)


def observe(kin, objects=(), contacts=()) -> WorldView:
    arms, grippers = [], []
    for side in SIDES:
        p, r = tool_from_link7(*kin.ee_pose(side))
        arms.append(ArmView(side, joints=kin.joints(side), tool_p=p, tool_r=r,
                            mode="position"))
        grippers.append(GripperView(side, 0.0))
    world = WorldView.of(list(objects), arms=arms, grippers=grippers)
    return world.with_(contacts=tuple(contacts)) if contacts else world


def tool_point(kin, side: str) -> np.ndarray:
    return tool_from_link7(*kin.ee_pose(side))[0]


def probe_tip_start(kin, side: str, direction) -> np.ndarray:
    """Where a probe's leading fingertip starts: the tool point, re-aimed."""
    d = np.asarray(ALIASES[direction].v, dtype=float)
    return tool_point(kin, side) + d * PAD.lead_m


def probe_and_record(kin, rig, verb, world0):
    plan = verb.plan(world0, kin)
    assert plan.ok, str(plan)
    report = run(plan, rig, kin=kin)
    world1 = record_contacts(observe(kin, world0.objects, world0.contacts),
                             report, verb)
    return plan, report, world1


# --------------------------------------------------------------------------- #
# the verbs, measured
# --------------------------------------------------------------------------- #

def test_a_probe_down_measures_a_table(d1_arm):
    table_z = float(probe_tip_start(d1_arm, "left", "down")[2]) - 0.060
    rig = SurfaceRig(d1_arm, point=(0.0, 0.0, table_z), normal=(0, 0, 1))
    world0 = observe(d1_arm)
    verb = Probe(side="left", direction="down", declare_as="table")
    plan, report, world1 = probe_and_record(d1_arm, rig, verb, world0)

    assert report.completed, report.error
    (contact,) = report.contacts
    assert contact.made and contact.stopped_by == "contact"
    assert contact.normal_hint == pytest.approx([0, 0, 1], abs=1e-3)
    # MEASURED where the table is, to well inside the +-3 mm gate
    touched = world1.contacts[-1]
    assert touched.p[2] == pytest.approx(table_z, abs=0.001)
    assert 0.055 <= contact.travel_m <= 0.062
    table = world1.find("table")
    assert isinstance(table, SurfaceView) and table.plane_source == "contact"
    assert table.top_z(world1.frames) == pytest.approx(table_z, abs=0.001)
    verdict = verb.verifier(world0)(world1)
    assert verdict.verdict == "true", verdict.reason


def test_a_probe_forward_measures_a_wall(d1_arm):
    wall_x = float(probe_tip_start(d1_arm, "right", "forward")[0]) + 0.050
    rig = SurfaceRig(d1_arm, point=(wall_x, 0.0, 0.0), normal=(-1, 0, 0))
    world0 = observe(d1_arm)
    verb = Probe(side="right", direction="forward", declare_as="wall")
    _plan, report, world1 = probe_and_record(d1_arm, rig, verb, world0)

    (contact,) = report.contacts
    assert contact.made and contact.stopped_by == "contact"
    # the surface pushes back along -x: the hint is the wall's normal
    assert contact.normal_hint == pytest.approx([-1, 0, 0], abs=1e-3)
    assert world1.contacts[-1].p[0] == pytest.approx(wall_x, abs=0.001)
    wall = world1.find("wall")
    assert wall.top_normal(world1.frames) == pytest.approx([-1, 0, 0], abs=1e-6)
    assert wall.plane_offset(world1.contacts[-1].p, world1.frames) == \
        pytest.approx(0.0, abs=1e-6)
    assert verb.verifier(world0)(world1).verdict == "true"


def _button_scene(kin):
    """A 30 mm button 120 mm in front of the right hand's HOME tool point."""
    p = tool_point(kin, "right") + np.array([0.12, 0.0, 0.0])
    button = ObjectView("button", p=p, size=(0.03, 0.03, 0.03))
    face_x = float(p[0]) - 0.015
    return button, face_x


def test_a_press_returns_to_its_standoff(d1_arm):
    button, face_x = _button_scene(d1_arm)
    rig = SurfaceRig(d1_arm, point=(face_x, 0.0, 0.0), normal=(-1, 0, 0))
    world0 = observe(d1_arm, [button])
    verb = Press(side="right", target="button", direction="forward",
                 force_nm=6.0, hold_s=0.5)
    plan, report, world1 = probe_and_record(d1_arm, rig, verb, world0)

    assert report.completed, report.error
    (contact,) = report.contacts
    assert contact.made
    assert contact.torque_nm >= 6.0
    # it pressed ON the face (the tip stopped there) ...
    assert world1.contacts[-1].p[0] == pytest.approx(face_x, abs=0.001)
    # ... HELD it (the frozen command was repeated for hold_s) ...
    held = [q for _t, q in rig.sent
            if rig.penetration("right", q[8:15]) > 0.0]
    assert len(held) >= int(0.5 * SurfaceRig.HZ)
    # ... and came back out to where it stood off
    standoff = next(w for w in plan.waypoints if w.label == "press_standoff")
    assert np.linalg.norm(tool_point(d1_arm, "right") - standoff.p) < 0.002
    assert rig.rise["right"] == 0.0
    assert verb.verifier(world0)(world1).verdict == "true"


def test_contact_is_measured_from_state_not_from_the_command(d1_arm):
    table_z = float(probe_tip_start(d1_arm, "left", "down")[2]) - 0.040
    rig = SurfaceRig(d1_arm, point=(0.0, 0.0, table_z), normal=(0, 0, 1))
    world0 = observe(d1_arm)
    verb = Probe(side="left", direction="down")
    plan = verb.plan(world0, d1_arm)
    step = next(s for s in plan.steps if isinstance(s, ContactStep))
    frozen: List[np.ndarray] = []
    original = rig.move_until

    def watch(path, **kwargs):
        out = original(path, **kwargs)
        frozen.append(rig.commanded["left"].copy())   # the command at the stop
        return out

    rig.move_until = watch            # type: ignore[assignment]
    report = run(plan, rig, kin=d1_arm)
    (contact,) = report.contacts
    command_tip = _tip(d1_arm, "left", frozen[0])
    measured_tip = contact.p_tool + np.array([0, 0, -PAD.lead_m])
    # the COMMAND was past the table by the lead the controller was pushing
    # with (4 Nm / 2000 Nm/m = 2 mm) ...
    assert table_z - command_tip[2] >= 0.0015
    # ... and the REPORT is where the arm was, on the table
    assert measured_tip[2] == pytest.approx(table_z, abs=0.0005)
    assert contact.q_stop is not None
    assert not np.allclose(contact.q_stop, frozen[0])
    # and the probe stopped pushing: re-commanded where it measured
    assert rig.rise["left"] == 0.0
    assert step.criterion.joint_torque_nm == 4.0


def test_a_controller_fault_during_a_probe_stops_with_fault(d1_arm):
    table_z = float(probe_tip_start(d1_arm, "left", "down")[2]) - 0.080
    rig = SurfaceRig(d1_arm, point=(0.0, 0.0, table_z), normal=(0, 0, 1))
    world0 = observe(d1_arm)
    plan = Probe(side="left", direction="down").plan(world0, d1_arm)
    standoff = sum(1 for s in plan.steps if type(s).__name__ == "JointStep")
    rig.fault_after = standoff + 40          # 40 ticks into the leg
    report = run(plan, rig, kin=d1_arm)
    assert not report.completed
    assert report.stop_reason == CONTROLLER_FAULT
    (contact,) = report.contacts
    assert contact.stopped_by == "fault" and not contact.made
    assert report.refusal is not None and report.refusal.reason == "controller_fault"
    # nothing more was sent after the fault was seen
    assert rig.sends == standoff + 40


def test_a_kinematic_executor_reports_no_contact_and_the_verifier_says_so(d1_arm):
    """C.3: the mirror returns ``made=False, stopped_by="max_travel"`` — a
    dry-run keeps working, and the verdict says nothing was touched."""
    mirror = KinematicExecutor(d1_arm)
    world0 = observe(d1_arm)
    verb = Probe(side="left", direction="down", max_travel_m=0.10,
                 declare_as="table")
    plan, report, world1 = probe_and_record(d1_arm, mirror, verb, world0)
    assert report.completed, report.error
    (contact,) = report.contacts
    assert not contact.made and contact.stopped_by == "max_travel"
    assert contact.travel_m == pytest.approx(0.10, abs=0.004)
    assert world1.find("table") is None          # nothing measured, nothing published
    verdict = verb.verifier(world0)(world1)
    assert verdict.verdict == "false"
    assert "max_travel" in verdict.reason


def test_an_executor_without_move_until_fails_a_contact_step_loudly(d1_arm):
    world0 = observe(d1_arm)
    plan = Probe(side="left", direction="down").plan(world0, d1_arm)
    recorder = RecordingExecutor(RawState(
        {s: JointState(q=d1_arm.joints(s), mode="position") for s in SIDES},
        {s: HandState(closedness=0.0, commanded=0.0, holding=False)
         for s in SIDES}))
    recorder.pretend_arrived = True
    report = run(plan, recorder, kin=d1_arm)
    assert report.stop_reason == TRANSPORT_ERROR
    assert "move_until" in report.error
    step = next(s for s in plan.steps if isinstance(s, ContactStep))
    # the leg's knots never went out as ordinary joint commands
    leg = {tuple(np.round(q, 9)) for q in step.path[1:]}
    assert not any(tuple(np.round(q[0:7], 9)) in leg for _t, q in recorder.sent)


def test_three_probes_fit_a_plane_with_a_real_normal(d1_arm):
    """Three contacts on a table tilted 4 deg about x: the published surface
    carries the TABLE's normal, not the probes' straight-down hint."""
    tilt = math.radians(4.0)
    normal = np.array([0.0, -math.sin(tilt), math.cos(tilt)])
    start = probe_tip_start(d1_arm, "left", "down")
    origin = start - np.array([0.0, 0.0, 0.050])
    rig = SurfaceRig(d1_arm, point=origin, normal=normal)
    verb = Probe(side="left", direction="down", declare_as="table")
    world = observe(d1_arm)
    for offset in ((0.0, 0.0), (0.05, 0.0), (0.0, -0.05)):
        # put the hand over the next spot, 50 mm above where it starts
        q = _ik_down(d1_arm, "left", start + np.array([offset[0], offset[1], 0.0]))
        for s, qs in (("left", q), ("right", d1_arm.joints("right"))):
            d1_arm.set_joints(s, qs)
            rig.measured[s] = np.array(qs, dtype=float)
        before = observe(d1_arm, world.objects, world.contacts)
        _plan, report, world = probe_and_record(d1_arm, rig, verb, before)
        assert report.contacts[-1].made, report.contacts[-1].detail
    table = world.find("table")
    assert len([c for c in world.contacts if c.surface == "table"]) == 3
    n = table.top_normal(world.frames)
    assert math.degrees(math.acos(min(1.0, float(np.dot(n, normal))))) < 0.3
    # and not the hint: the fit found the tilt
    assert math.degrees(math.acos(float(n[2]))) == pytest.approx(4.0, abs=0.3)
    for c in world.contacts:
        assert table.plane_offset(c.p, world.frames) == pytest.approx(0, abs=5e-4)


def _ik_down(kin, side: str, tip) -> np.ndarray:
    """A posture whose fingertip is at ``tip`` pointing down, via a plan."""
    from manipulation_kit.primitives import Kin, Waypoint, align_tool, solve_path
    world = observe(kin)
    r = align_tool(side, (0, 0, -1))
    with Kin(kin, world) as borrowed:
        steps, error, _ = solve_path(borrowed, side, [Waypoint(
            "spot", np.asarray(tip) - np.array([0, 0, -PAD.lead_m]), r,
            allow_via=False)], primitive="test")
    assert error is None, str(error)
    return np.array(steps[-1].q if steps else kin.joints(side), dtype=float)


# --------------------------------------------------------------------------- #
# the plane fit, alone
# --------------------------------------------------------------------------- #

def test_one_or_two_probes_publish_the_hint_not_a_fitted_normal():
    up = (0, 0, 1)
    one = fit_plane([(0.4, 0.1, 0.7)], [up])
    assert not one.fitted and one.normal == pytest.approx([0, 0, 1])
    two = fit_plane([(0.4, 0.1, 0.70), (0.5, 0.1, 0.71)], [up, up])
    assert not two.fitted
    # perpendicular to the line through the two, closest to the hint
    assert float(np.dot(two.normal, [0.1, 0.0, 0.01])) == pytest.approx(0, abs=1e-9)
    line = fit_plane([(0.4, 0.1, 0.7), (0.45, 0.1, 0.7), (0.5, 0.1, 0.7)],
                     [up] * 3)
    assert not line.fitted


# --------------------------------------------------------------------------- #
# the model surface
# --------------------------------------------------------------------------- #

DIRECTION_WORDS = re.compile(r"(down|up|forward|backward|left|right|top|side|"
                             r"front|back)", re.IGNORECASE)


def test_no_verb_name_contains_a_direction():
    offenders = [verb for verb in BY_VERB if DIRECTION_WORDS.search(verb)]
    assert offenders == []
    assert "touch_down" not in BY_VERB
    assert {"probe", "press"} <= set(BY_VERB)


def test_contact_thresholds_are_model_bindable_only_inside_their_bounds():
    schemas = {s["name"]: s for s in tool_schemas()}
    probe = schemas["probe"]["parameters"]["properties"]
    press = schemas["press"]["parameters"]["properties"]
    assert (probe["contact_nm"]["minimum"], probe["contact_nm"]["maximum"]) == (1.5, 6.0)
    assert (press["force_nm"]["minimum"], press["force_nm"]["maximum"]) == (2.0, 8.0)
    for name in ("max_travel_m", "contact_nm", "hand", "declare_as", "direction"):
        assert name in probe
    for name in ("depth_m", "force_nm", "hold_s", "hand", "target", "direction"):
        assert name in press
    assert decode("probe", {"contact_nm": 12.0}).reason == "bad_argument"
    assert decode("press", {"target": "x", "force_nm": 0.5}).reason == "bad_argument"
    assert decode("probe", {"hand": "fist"}).reason == "bad_argument"
    ok = decode("probe", {"direction": "forward", "contact_nm": 3.0})
    assert isinstance(ok, Probe) and ok.direction == ALIASES["forward"]


def test_a_press_target_can_be_anything_the_world_names(d1_arm):
    button, _ = _button_scene(d1_arm)
    world = observe(d1_arm, [button, SurfaceView("panel", p=(0.6, 0, 1.0),
                                                 size=(0.02, 0.4, 0.4))])
    for name in ("button", "panel"):
        call = decode("press", {"target": name}, world)
        assert isinstance(call, Press), str(call)
    schema = {s["name"]: s for s in tool_schemas(world)}["press"]
    assert set(schema["parameters"]["properties"]["target"]["enum"]) == {
        "button", "panel"}


def test_declare_as_may_not_rename_a_movable_thing(d1_arm):
    world = observe(d1_arm, [ObjectView("cup", p=(0.4, 0.1, 0.1),
                                        size=(0.05, 0.05, 0.08))])
    refused = Probe(side="left", declare_as="cup").plan(world, d1_arm)
    assert not refused.ok and refused.reason == "precondition_unmet"
    assert refused.unmet[0].code == "bad_argument"


def test_an_upward_probe_is_a_typed_refusal(d1_arm):
    refused = Probe(side="left", direction="up").plan(observe(d1_arm), d1_arm)
    assert not refused.ok
    assert refused.unmet[0].code == "bad_argument"


def test_a_probe_verifier_on_its_own_world_is_not_true(d1_arm):
    world = observe(d1_arm)
    verb = Probe(side="left", declare_as="table")
    assert verb.verifier(world)(world).verdict != "true"
