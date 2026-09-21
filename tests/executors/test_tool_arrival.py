"""The tool-space arrival barrier: measured at the jaws, corrected in place.

WHAT THIS IS ABOUT, in one measurement (blocks-eval, seed 7, 2026-09-21). A
``grasp`` run straight from HOME succeeded once in three. Every failure looked
the same: ``run.arrivals[0].worst_error_deg`` 2.2-2.8, which PASSES the
executor's ``ARRIVE_TOL_RAD`` of 3 deg — a tolerance that exists because the
real arm droops about a degree under gravity (F16) — and then an 80 mm descent
and a stroke that closed the jaws beside the block ("stalled at 13.5 mm inside
block_red's 43.8 mm"). 2.8 deg at a half-metre reach is centimetres at the
tool, and the joint-space barrier cannot see it.

So the barrier is asked the question the verb is about: is the JAW POCKET
where the plan put it? Same two postures, the kit's own forward kinematics, no
new RPC. The fakes here are the smallest thing that reproduces the failure: a
transport that lands every command with a constant joint offset, which is what
a droop, a controller deadband and a warm harmonic drive all look like from
outside.
"""

from __future__ import annotations

import math

import numpy as np

from manipulation_kit.executor import (ARRIVE_TOL_ALONG_M, ARRIVE_TOL_M,
                                       ARRIVE_TOL_RAD,
                                       JOINT_SLICE, SIDES, ArrivalReport,
                                       RawState, SettleReport, StrokeReport,
                                       ToolGate, run)
from manipulation_kit.primitives import Approach, Grasp, Place

REACHABLE = (0.38, 0.25, 0.05)


# --------------------------------------------------------------------------- #
# the fake
# --------------------------------------------------------------------------- #

class DroopingExecutor:
    """A transport that lands every command with a constant joint offset.

    Joint-space it is INSIDE tolerance and says so honestly: 2.5 deg against a
    3 deg barrier is an arrival by the only measure the old runner had. At the
    tool it is two centimetres out, and that is the whole point.

    ``jams_after_gate`` is the arm that stops responding once the barrier
    starts talking to it — the F5 failure, fingers on the table, ten grasps out
    of ten — so a correction that cannot work is seen not to work.
    """

    def __init__(self, world, *, offset_rad: float = math.radians(2.5),
                 side: str = "left", jams_after_gate: bool = False,
                 flips: bool = False):
        self.start = {s: np.array(world.arm(s).joints, dtype=float)
                      for s in SIDES}
        self.grippers = {s: float(world.gripper(s).closedness) for s in SIDES}
        self.offset = float(offset_rad)
        self.side = side
        self.jams_after_gate = bool(jams_after_gate)
        #: backlash: every correction is answered by a droop the OTHER way,
        #: so the tool error never shrinks however politely it is asked
        self.flips = bool(flips)
        self.commanded = None
        self.jammed_at = None
        self.gated = False
        self.sent = []
        self.grips = []
        self.settles = []

    # -- what the robot is doing ------------------------------------------- #
    def measured(self):
        if self.commanded is None:
            return {s: np.array(q) for s, q in self.start.items()}
        base = (self.jammed_at if self.jammed_at is not None else self.commanded)
        out = {s: np.array(base[JOINT_SLICE[s]], dtype=float) for s in SIDES}
        out[self.side] = out[self.side].copy()
        out[self.side][0] += self.offset      # J1, where the droop lives
        return out

    def state(self) -> RawState:
        return RawState(joints=self.measured(), grippers=dict(self.grippers),
                        commanded_grippers=dict(self.grippers),
                        stationary=True)

    def send_joints(self, q16, *, t: float) -> None:
        q = np.asarray(q16, dtype=float).reshape(16)
        self.sent.append((float(t), q.copy()))
        if self.jams_after_gate and self.gated:
            if self.jammed_at is None:
                self.jammed_at = np.array(self.commanded, dtype=float)
            return                              # commanded, and nothing moves
        if self.flips and self.gated:
            self.offset = -self.offset
        self.commanded = q.copy()

    def set_gripper(self, side: str, closedness: float, *, grip: str) -> None:
        self.grips.append((side, float(closedness), grip))
        self.grippers[side] = float(closedness)

    def settle(self, timeout_s: float) -> SettleReport:
        self.settles.append(float(timeout_s))
        return SettleReport(True, 0.0, 0.0, "the fake is always settled")

    # -- barriers ----------------------------------------------------------- #
    def wait_arrived(self, q16, *, tol_rad: float = ARRIVE_TOL_RAD,
                     timeout_s: float = 3.0) -> ArrivalReport:
        self.gated = True
        want = np.asarray(q16, dtype=float).reshape(16)
        measured = self.measured()
        worst = max(float(np.max(np.abs(measured[s] - want[JOINT_SLICE[s]])))
                    for s in SIDES)
        if worst <= tol_rad:
            return ArrivalReport(True, worst, 0.0,
                                 "inside the joint-space tolerance")
        return ArrivalReport(False, worst, float(timeout_s),
                             f"{math.degrees(worst):.2f} deg short")

    def wait_gripper_settled(self, side: str, *, timeout_s: float = 3.0
                             ) -> StrokeReport:
        return StrokeReport(True, self.grippers.get(side, 0.0), holding=False,
                            stalled=False, detail="the fake's stroke is instant")


def _grasp(world, d1_arm, side="left"):
    plan = Grasp(object="red_block", side=side).plan(world, d1_arm)
    assert getattr(plan, "ok", False), plan
    return plan


def _strokes(executor):
    return [c for _s, c, _g in executor.grips]


# --------------------------------------------------------------------------- #
# the barrier
# --------------------------------------------------------------------------- #

def test_a_droop_inside_the_joint_tolerance_is_centimetres_at_the_tool(
        d1_arm, observe):
    """The measurement this whole change is about, isolated: the joint barrier
    passes and the tool is nowhere near."""
    world = observe(d1_arm, block_p=REACHABLE)
    plan = _grasp(world, d1_arm)
    q_cmd = np.asarray(plan.final_joints()["left"], dtype=float)
    q_off = q_cmd.copy()
    q_off[0] += math.radians(2.5)
    gate = ToolGate(kin=d1_arm)
    p_cmd = gate._tool(d1_arm, "left", q_cmd)[0]
    p_off = gate._tool(d1_arm, "left", q_off)[0]
    assert math.degrees(2.5 * math.pi / 180) < 3.0     # joint barrier passes
    assert float(np.linalg.norm(p_off - p_cmd)) > ARRIVE_TOL_M * 2, (
        "2.5 deg on J1 has to be well outside the 5 mm tool tolerance for "
        "these tests to be about anything")


def test_without_correction_the_stroke_is_refused_with_arrived_off_by(
        d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE)
    plan = _grasp(world, d1_arm)
    robot = DroopingExecutor(world)
    report = run(plan, robot, kin=d1_arm, correct_arrival=False)

    assert not report.completed
    assert report.stop_reason == "barrier_failed"
    assert report.refusal is not None
    assert report.refusal.reason == "arrived_off_by"
    assert report.refusal.waypoint_label == "standoff", (
        "the miss is caught BEFORE the descent, which is the leg that may not "
        "be re-routed")
    assert report.refusal.residual_m > ARRIVE_TOL_M
    # the jaws never closed: the opening stroke is the only one
    assert _strokes(robot) == [0.0]
    arrival = report.arrivals[-1]
    assert arrival.tool_error_m > ARRIVE_TOL_M
    assert np.isfinite(arrival.tool_rot_error_rad)
    assert arrival.corrections == ()
    # ...and it is all in the trace, by name
    rendered = report.to_json()
    assert rendered["refusal"]["schema"] == "manipulation_kit.refusal/2"
    assert rendered["refusal"]["reason"] == "arrived_off_by"
    assert rendered["arrivals"][-1]["tool_error_m"] > ARRIVE_TOL_M


def test_with_correction_the_tool_lands_and_the_stroke_proceeds(d1_arm,
                                                                observe):
    """1.2 deg of droop, not the 2.5 above, and the difference is a property
    of the SOLVER rather than of the barrier: this scene's standoff is one of
    the plateaus ``planning`` documents ("the last four knots of a HOME ->
    standoff travel at (0.38, 0.25) sit at 4.7-9.1 mm however many solves they
    are given"), where a re-solve takes out about half the miss and stops. 1.2
    deg is 8 mm at the tool, which one round lands inside 5 mm; 2.5 deg is
    17 mm, which it cannot, and THAT is the refusal the test above pins."""
    world = observe(d1_arm, block_p=REACHABLE)
    plan = _grasp(world, d1_arm)
    robot = DroopingExecutor(world, offset_rad=math.radians(1.2))
    report = run(plan, robot, kin=d1_arm)

    assert report.completed, report.error
    assert _strokes(robot) == [0.0, 1.0], "open, descend, close"
    for arrival in report.arrivals:
        assert arrival.arrived
        assert arrival.tool_across_m <= ARRIVE_TOL_M, (
            f"{arrival.waypoint_label}: {arrival.tool_across_m * 1000:.1f} mm")
        assert abs(arrival.tool_along_m) <= ARRIVE_TOL_ALONG_M
    corrected = [a for a in report.arrivals if a.corrections]
    assert corrected, "a droop of 8 mm at the tool has to have been corrected"
    for arrival in corrected:
        assert len(arrival.corrections) <= 2
        first = arrival.corrections[0]
        assert first.tool_error_before_m > ARRIVE_TOL_M
        assert arrival.corrections[-1].tool_error_after_m <= ARRIVE_TOL_M
        assert first.sent


def test_a_fake_that_never_converges_fails_after_exactly_two_rounds(d1_arm,
                                                                    observe):
    """An arm with backlash answers every correction with the opposite droop.

    Two rounds, then the truth. Not three, not a loop: the correction feeds a
    MEASURED offset forward, so an arm that has not answered the first two is
    not going to answer the tenth, and the model is owed the refusal rather
    than an arm nodding at a block while the turn budget runs out.
    """
    world = observe(d1_arm, block_p=REACHABLE)
    plan = _grasp(world, d1_arm)
    robot = DroopingExecutor(world, offset_rad=math.radians(1.2), flips=True)
    report = run(plan, robot, kin=d1_arm)

    assert not report.completed
    assert report.stop_reason == "barrier_failed"
    assert report.refusal.reason == "arrived_off_by"
    arrival = report.arrivals[-1]
    assert len(arrival.corrections) == 2, [c.to_json() for c in
                                           arrival.corrections]
    assert all(c.sent for c in arrival.corrections)
    assert arrival.tool_across_m > ARRIVE_TOL_M
    assert "after 2 corrections" in arrival.detail
    assert _strokes(robot) == [0.0], "no stroke on an unverified pose"
    assert len(report.refusal.attempted) == 2, "the rounds travel with it"


def test_an_arm_that_stops_responding_is_named_as_that_and_not_as_a_miss(
        d1_arm, observe):
    """The other way a correction fails: it is commanded and nothing moves
    (F5, fingers jammed on the table, ten grasps out of ten). Still
    ``arrived_off_by``, and the detail says the corrected posture was never
    reached rather than blaming the geometry."""
    world = observe(d1_arm, block_p=REACHABLE)
    plan = _grasp(world, d1_arm)
    robot = DroopingExecutor(world, offset_rad=math.radians(1.0),
                             jams_after_gate=True)
    report = run(plan, robot, kin=d1_arm)

    assert not report.completed
    assert report.refusal.reason == "arrived_off_by"
    arrival = report.arrivals[-1]
    assert 1 <= len(arrival.corrections) <= 2
    assert "not reached" in arrival.detail
    assert _strokes(robot) == [0.0]


def test_a_corrected_posture_the_guard_rejects_is_refused_not_sent(d1_arm,
                                                                   observe):
    """The correction is a posture NOBODY planned. It gets the same guard the
    plan got, and a refusal is a refusal — not a smaller correction."""
    from manipulation_kit.arms.guard import GuardGate

    world = observe(d1_arm, block_p=REACHABLE)
    plan = _grasp(world, d1_arm)
    robot = DroopingExecutor(world)

    class RefuseEverything:
        """A guard that refuses every posture it is shown."""

        def __init__(self):
            self.seen = 0

        def check(self, joints_a_deg, joints_b_deg):
            self.seen += 1
            return type("Report", (), {"ok": False, "violations": ("stub",),
                                       "min_body_clearance": 0.0})()

    guard = RefuseEverything()
    gate = ToolGate(kin=d1_arm)
    original = d1_arm._gate
    d1_arm._gate = GuardGate(guard)
    try:
        report = run(plan, robot, kin=d1_arm)
    finally:
        d1_arm._gate = original

    assert not report.completed
    assert report.refusal.reason == "arrived_off_by"
    assert guard.seen, "the guard was consulted about the corrected posture"
    arrival = report.arrivals[-1]
    assert len(arrival.corrections) == 1
    assert arrival.corrections[0].sent is False
    assert "guard" in arrival.corrections[0].detail
    assert "guard" in report.error
    # the plan's own knots were sent; the correction was NOT
    commanded = [q for _t, q in robot.sent]
    assert commanded, "the plan itself still ran up to the barrier"
    assert _strokes(robot) == [0.0]


def test_an_unguardable_correction_is_refused_before_it_is_solved(d1_arm,
                                                                  observe):
    """A guarded plan may not be corrected by an unguarded model. The refusal
    says which, because "it did not move" is not a diagnosis."""
    from manipulation_kit.arms.guard import GuardGate

    world = observe(d1_arm, block_p=REACHABLE)
    plan = _grasp(world, d1_arm)
    robot = DroopingExecutor(world)
    original = d1_arm._gate
    d1_arm._gate = GuardGate(None)
    try:
        report = run(plan, robot, kin=d1_arm)
    finally:
        d1_arm._gate = original
    assert not report.completed
    assert report.refusal.reason == "arrived_off_by"
    assert "guard" in report.error and "none installed" in report.error
    assert report.arrivals[-1].corrections == ()


def test_a_transport_that_cannot_say_where_the_arm_is_fails_the_barrier(
        d1_arm, observe):
    """No measurement, no arrival. The gate does not assume one."""
    world = observe(d1_arm, block_p=REACHABLE)
    plan = _grasp(world, d1_arm)

    class Mute(DroopingExecutor):
        """It claims the arrival its transport can see and cannot show it."""

        def measured(self):
            if self.commanded is None:        # the binding still has to pass
                return {s: np.array(q) for s, q in self.start.items()}
            return {}

        def wait_arrived(self, q16, *, tol_rad=ARRIVE_TOL_RAD, timeout_s=3.0):
            self.gated = True
            return ArrivalReport(True, 0.0, 0.0, "the transport says so")

    report = run(plan, Mute(world), kin=d1_arm)
    assert not report.completed
    assert report.stop_reason == "barrier_failed"
    assert report.refusal.reason == "arrival_unknown"
    assert "no measured joints" in report.error


def test_a_descent_that_stopped_short_is_refused_rather_than_shoved(d1_arm,
                                                                    observe):
    """Depth is what CONTACT takes, and pushing into it is F5.

    A descent deliberately ends with the fingertips 3 mm off the surface
    (``approach.SUPPORT_CLEARANCE_M``), so an arm that parks a centimetre high
    with the jaws still lined up has met something. The barrier says so and
    stops; it does not command the same pose deeper, which is how ten grasps
    out of ten jammed their fingers on the table.
    """
    from manipulation_kit.primitives.approach import link7_from_tool

    world = observe(d1_arm, block_p=REACHABLE)
    plan = _grasp(world, d1_arm)

    class StopsShort(DroopingExecutor):
        """Every command lands 12 mm short ALONG the approach axis — 2.5 deg
        in joints, inside the joint barrier and outside the tool one."""

        short_m = 0.012

        def measured(self):
            if self.commanded is None:
                return {s: np.array(q) for s, q in self.start.items()}
            out = {s: np.array(self.commanded[JOINT_SLICE[s]], dtype=float)
                   for s in SIDES}
            q = out[self.side]
            gate = ToolGate(kin=d1_arm)
            p, r = gate._tool(d1_arm, self.side, q)
            axis = np.asarray(r.as_matrix()[:, 2], dtype=float)
            p7, r7 = link7_from_tool(p - axis * self.short_m, r)
            d1_arm.set_joints(self.side, q)
            result = d1_arm.solve_ee(self.side, p7, r7)
            if result.ok:
                out[self.side] = np.array(result.q, dtype=float)
            d1_arm.set_joints(self.side, q)
            return out

    robot = StopsShort(world, offset_rad=0.0)
    report = run(plan, robot, kin=d1_arm)
    assert not report.completed
    assert report.refusal.reason == "arrived_off_by"
    arrival = report.arrivals[-1]
    assert arrival.corrections == (), "a shove is not a correction"
    assert arrival.tool_across_m <= ARRIVE_TOL_M, "the jaws are lined up"
    assert abs(arrival.tool_along_m) > ARRIVE_TOL_ALONG_M
    assert "stopped" in arrival.detail and "deeper" in arrival.detail
    assert _strokes(robot) == [0.0]


def test_an_arm_that_has_not_stopped_is_not_measured_at_all(d1_arm, observe):
    """The correction feeds a STEADY-STATE offset forward. A tool point read
    while the arm is still converging is where it was passing, not where it is
    going to be — measured on blocks-eval, reading at the instant the 3 deg
    joint gate passed gave 23.8 -> 14.9 -> 9.1 mm, three rounds chasing the
    same settle. So the barrier stops the arm first, and an arm that will not
    stop is its own typed refusal."""
    world = observe(d1_arm, block_p=REACHABLE)
    plan = _grasp(world, d1_arm)

    class NeverStops(DroopingExecutor):
        def settle(self, timeout_s: float) -> SettleReport:
            self.settles.append(float(timeout_s))
            return SettleReport(False, float(timeout_s), 12.0,
                                "still moving at 12.0 deg/s")

    robot = NeverStops(world)
    report = run(plan, robot, kin=d1_arm)
    assert not report.completed
    assert report.stop_reason == "barrier_failed"
    assert report.refusal.reason == "not_settled"
    arrival = report.arrivals[-1]
    assert arrival.settled is False
    assert arrival.corrections == (), "nothing to feed forward from a blur"
    assert _strokes(robot) == [0.0]
    assert robot.settles[0] == 2.0, "the gate's own settle budget"


def test_the_barrier_runs_once_per_gated_waypoint_and_not_per_knot(d1_arm,
                                                                   observe):
    """A 25 cm travel is a dozen knots. It is ONE arrival."""
    world = observe(d1_arm, block_p=REACHABLE)
    plan = _grasp(world, d1_arm)
    robot = DroopingExecutor(world, offset_rad=0.0)
    report = run(plan, robot, kin=d1_arm)
    assert report.completed, report.error
    assert [a.waypoint_label for a in report.arrivals] == ["standoff", "grasp"]
    assert len(plan.joint_steps()) > len(report.arrivals)


def test_a_plan_with_no_gated_waypoint_keeps_the_barrier_it_always_had(
        d1_arm, observe):
    """``Approach`` is unchanged: its own verifier measures the tool point, and
    its opening stroke happens before the arm moves at all."""
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Approach(object="red_block", side="left").plan(world, d1_arm)
    assert getattr(plan, "ok", False)
    assert [w.arrive for w in plan.waypoints] == [False]
    robot = DroopingExecutor(world)
    report = run(plan, robot, kin=d1_arm)
    assert report.completed, report.error
    assert report.arrivals == (), "no stroke follows a joint step here"


# --------------------------------------------------------------------------- #
# what the plans ask for
# --------------------------------------------------------------------------- #

def test_grasp_marks_the_standoff_and_the_grasp(d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE)
    plan = _grasp(world, d1_arm)
    assert [(w.label, w.arrive) for w in plan.waypoints] == [
        ("standoff", True), ("grasp", True)]
    assert plan.to_json()["waypoints"][0]["arrive"] is True


def test_place_marks_the_transit_and_the_set_down(d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    plan = Place(object="red_block", to="box", side="left").plan(world, d1_arm)
    assert getattr(plan, "ok", False), plan
    marked = {w.label: w.arrive for w in plan.waypoints}
    assert marked["over_destination"] is True
    assert marked["set_down"] is True
    assert marked["clearance"] is False, (
        "the rise is free space; the tool promise starts over the destination")


def test_approach_is_unchanged(d1_arm, observe):
    world = observe(d1_arm, block_p=REACHABLE)
    plan = Approach(object="red_block", side="left").plan(world, d1_arm)
    assert [(w.label, w.arrive) for w in plan.waypoints] == [
        ("standoff", False)]


def test_a_waypoint_defaults_to_no_tool_barrier(d1_arm, observe):
    """Every other verb keeps the barrier it had: the gate is spent where the
    tool point IS the promise, not everywhere."""
    from manipulation_kit.primitives import Carry, GoHome, Lift, Nudge, Retreat
    world = observe(d1_arm, block_p=REACHABLE, closed={"left": 1.0},
                    held={"left": "red_block"})
    for verb in (Lift(object="red_block", side="left"),
                 Carry(object="red_block", to="box", side="left"),
                 Nudge(side="left", dz=-0.01), Retreat(side="left"),
                 GoHome(side="left")):
        plan = verb.plan(world, d1_arm)
        if not getattr(plan, "ok", False):
            continue
        assert not any(w.arrive for w in plan.waypoints), verb.name()
