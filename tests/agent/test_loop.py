"""``manipulation_kit.agent.loop``: the look before a stroke, the stops, the trace."""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest

from manipulation_kit.agent import (KinematicMirror, ObservationError,
                                    OperatorPolicy, run)
from manipulation_kit.agent.trace import DecisionRecord, DecisionTrace
from manipulation_kit.primitives import Place


class Script:
    """A model that plays a fixed list of calls, then stops."""

    def __init__(self, *calls):
        self.calls = list(calls)
        self.seen = []

    def __call__(self, messages, tools):
        self.seen.append(messages[-1])
        if not self.calls:
            return {"name": None, "arguments": {}, "claimed": ""}
        name, arguments = self.calls.pop(0)
        return {"name": name, "arguments": arguments, "claimed": "",
                "call_id": f"c{len(self.seen)}"}


def _mirror(wrist=True):
    from scene import DEMO_WRIST_CAMERA, demo_scene
    world, kin = demo_scene()
    return KinematicMirror(kin, world,
                           wrist_intrinsics=DEMO_WRIST_CAMERA if wrist else None)


GRASP = ("grasp", {"object": "red_block", "side": "left", "direction": "down"})
APPROACH = ("approach", {"object": "red_block", "side": "left",
                         "direction": "down"})
GOAL = Place(object="red_block", to="box")


def test_a_stroke_is_preceded_by_a_look(agent_examples):
    """Requirement 2 (``look_before_stroke``): no grasp stroke runs until the
    hand has looked at the object from the posture it closes from — and a look
    from HOME, where the wrist camera does not see it, does not count."""
    robot = _mirror()
    model = Script(GRASP, APPROACH, GRASP, GRASP)
    trace = run(goal=GOAL, robot=robot, policy=OperatorPolicy(max_turns=4),
                ask=model)
    home, approach, look, grasp = trace.records
    # a grasp from HOME: the object is not in the wrist frame, nothing ran
    assert home.look["visible"] is False and home.run is None
    assert home.refused[0]["unmet"][0]["code"] == "look_required"
    assert approach.run["completed"]
    # after the approach the look SEES it, tells the model where, runs nothing
    assert look.look["visible"] is True and look.run is None
    assert 0 <= look.look["u"] <= 640 and 0 <= look.look["v"] <= 480
    assert "WRIST LOOK" in look.refused[0]["detail"]  # what the model reads
    # and only then does the stroke run, and hold
    assert grasp.look is None and grasp.run["completed"]
    assert grasp.verdict["verdict"] == "true"


def test_without_a_wrist_camera_a_look_policy_refuses_to_start(agent_examples):
    """Fail closed, before anything moves."""
    trace = run(goal=GOAL, robot=_mirror(wrist=False),
                policy=OperatorPolicy(), ask=Script(GRASP))
    assert trace.stop == "look_unavailable"
    assert "robot.wrist_camera" in trace.stop_detail
    assert len(trace.records) == 1 and trace.records[0].run is None
    # the operator can own the blind grasp, explicitly
    trace = run(goal=GOAL, robot=_mirror(wrist=False),
                policy=OperatorPolicy(look_before_stroke=False, max_turns=1),
                ask=Script(GRASP))
    assert trace.records[0].run["completed"]


def test_a_wrist_locate_corrects_the_object_and_the_hand(agent_examples):
    """The correction half: ``locate`` on the wrist camera re-measures the
    target (a sighting — ``observed``) and moves the hand by the difference
    with the kit's own Nudge, within the nudge budget."""
    robot = _mirror()
    probe = run(goal=GOAL, robot=robot, policy=OperatorPolicy(max_turns=2),
                ask=Script(APPROACH, GRASP))
    look = probe.records[1].look
    before = robot.world().find("red_block").p.copy()
    # the model sees it 40 px further right in the wrist image than declared
    model = Script(("locate", {"camera": "left_wrist", "u": look["u"] + 40,
                               "v": look["v"]}), GRASP)
    trace = run(goal=GOAL, robot=robot, policy=OperatorPolicy(max_turns=2),
                ask=model)
    correct, grasp = trace.records
    # the world the stroke was planned in: the wrist's sighting
    after = next(o for o in grasp.world["objects"] if o["name"] == "red_block")
    assert after["provenance"] == "observed"
    assert np.linalg.norm(np.array(after["p"][:2]) - before[:2]) > 0.005
    assert correct.plan["primitive"] == "nudge" and correct.run["completed"]
    assert "re-measured" in correct.observation_after["answer"]
    # the correction counts as the look: the stroke runs
    assert grasp.look is None and grasp.run is not None


def test_a_latched_controller_ends_the_loop(agent_examples):
    """The kit stops the run; the loop stops the task (step 1 moved it here)."""
    robot = _mirror()
    inner = robot.executor.state

    def faulted():
        state = inner()
        arms = dict(state.arms)
        arms["left"] = dataclasses.replace(arms["left"], mode="error",
                                           error_code=2)
        return type(state)(arms=arms, hands=state.hands, stamp=state.stamp)

    robot.executor.state = faulted
    trace = run(goal=GOAL, robot=robot,
                policy=OperatorPolicy(look_before_stroke=False),
                ask=Script(APPROACH, APPROACH))
    assert trace.stop == "controller_fault"
    assert len(trace.records) == 1


def test_no_fresh_observation_no_turn(agent_examples):
    def blind(turn, world):
        raise ObservationError("the head camera returned exit 1")

    trace = run(goal=GOAL, robot=_mirror(), policy=OperatorPolicy(),
                ask=Script(APPROACH), observe=blind)
    assert trace.stop == "observation_failed"
    assert trace.records[0].choice is None and trace.records[0].run is None


def test_the_request_is_recorded_verbatim_and_the_effect_beside_it(
        agent_examples):
    """Astra review 15: the grip cap used to rewrite the recorded request."""
    policy = OperatorPolicy(max_grip="soft", look_before_stroke=False,
                            max_turns=2)
    firm = ("grasp", {"object": "red_block", "side": "left",
                      "direction": "down", "grip": "strong"})
    trace = run(goal=GOAL, robot=_mirror(), policy=policy,
                ask=Script(APPROACH, firm))
    record = trace.records[1]
    assert record.choice["arguments"]["grip"] == "strong"
    assert record.effective["arguments"]["grip"] == "soft"


def test_old_photos_are_not_resent(agent_examples):
    parts = [{"type": "input_image", "image_url": "data:x", "_file": "f.jpg"}]
    model = Script(APPROACH, ("nudge", {"side": "left", "dz": 0.01}))
    run(goal=GOAL, robot=_mirror(), policy=OperatorPolicy(max_turns=2),
        ask=model, observe=lambda turn, world: [dict(parts[0])])
    first, second = model.seen
    assert isinstance(second["content"], list)
    assert any(p.get("type") == "input_image" for p in second["content"])
    # the first observation's photo was replaced by its label
    assert not any(p.get("type") == "input_image" for p in first["content"]
                   if isinstance(first["content"], list))


def test_the_decision_trace_separates_the_claim_from_the_measurement():
    tracker = DecisionTrace()
    tracker.write(DecisionRecord(0, world={}, claimed="done",
                                 goal_verdict={"verdict": "false",
                                               "reason": "not in the box"}))
    tracker.write(DecisionRecord(1, world={}, claimed=None,
                                 goal_verdict={"verdict": "true", "reason": "in"}))
    assert len(tracker.disagreements()) == 1
    assert tracker.summary()["claimed_but_unmeasured"] == 1


def test_run_wants_a_measurable_goal(agent_examples):
    with pytest.raises(TypeError):
        run(goal=None, robot=_mirror(), policy=OperatorPolicy(),
            ask=Script())


def test_a_grasp_that_measured_the_width_corrects_the_world(agent_examples):
    """d1-2 2026-09-23 record 6, end to end: the hand holds, stalled at a gap
    wider than declared. The grasp is TRUE, and the world the NEXT turn plans
    in carries the MEASURED width along the jaws (``size_provenance``), with
    the correction in the trace and in what the model is told."""
    from scene import DEMO_WRIST_CAMERA, demo_scene

    from manipulation_kit.primitives.orientation import jaw_axis
    world, kin = demo_scene()
    robot = KinematicMirror(kin, world, wrist_intrinsics=DEMO_WRIST_CAMERA,
                            open_gap_m=0.0605)
    source, extra = robot.source, 0.006
    inner = source._robot_half

    def robot_half(state):
        # the mirror measures no gap; this stands in for the daemon's
        # jaw_rad: the declared extent along the jaws plus 6 mm, stalled
        arms, grippers = inner(state)
        out = []
        for g in grippers:
            arm = next(a for a in arms if a.side == g.side)
            if g.holding:
                block = source.objects["red_block"]
                declared = block.extent_along(jaw_axis(arm.tool_r),
                                              source._frames(0.0))
                if block.size_provenance != "measured":
                    robot_half.gap = declared + extra
                g = dataclasses.replace(g, jaw_gap_m=robot_half.gap,
                                        jaw_stalled=True)
            out.append(g)
        return arms, out

    source._robot_half = robot_half
    model = Script(APPROACH, GRASP, GRASP, ("go_home", {"side": "left"}))
    seen = []

    def ask(messages, tools):
        seen[:] = [str(m.get("output", m.get("content", ""))) for m in messages]
        return model(messages, tools)

    trace = run(goal=GOAL, robot=robot, policy=OperatorPolicy(max_turns=4),
                ask=ask)
    grasp = trace.records[2]
    assert grasp.choice["name"] == "grasp" and grasp.run is not None
    assert grasp.verdict["verdict"] == "true", grasp.verdict["reason"]
    assert "width corrected" in grasp.verdict["reason"]
    fix, = grasp.corrections
    assert fix["applied"] and fix["field"] == "width_along_jaw_axis"
    assert fix["measured_m"] == pytest.approx(robot_half.gap, abs=1e-4)
    # the next turn's world: the measured width, on the object in the hand
    later = trace.records[3].world
    block = next(o for o in later["objects"] if o["name"] == "red_block")
    assert block["size_provenance"] == "measured"
    assert block["provenance"] == "attached"
    measured = robot.source.objects["red_block"]
    axis = np.asarray(fix["jaw_axis"])
    assert measured.extent_along(axis, robot.world().frames) == pytest.approx(
        robot_half.gap, abs=1e-3)
    assert any("grasp MEASURED" in m for m in seen), \
        "the model was not told its declaration was corrected"
