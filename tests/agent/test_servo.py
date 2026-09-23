"""``manipulation_kit.agent.servo``: the look before a stroke answered by a
judge that chooses, the kit stepping the hand on its own grid."""

from __future__ import annotations

import numpy as np
import pytest

from manipulation_kit.agent import (KinematicMirror, OperatorPolicy, Servo,
                                    geometry_judge, run)
from manipulation_kit.agent.servo import (ALIGNED, BUDGET, CHOICES,
                                          ServoLook, image_direction_in_base)
from manipulation_kit.primitives import Place


class Script:
    """A model that plays a fixed list of calls, then stops."""

    def __init__(self, *calls):
        self.calls = list(calls)
        self.seen = []

    def __call__(self, messages, tools):
        self.seen.append(messages[-1])
        self.history = list(messages)
        if not self.calls:
            return {"name": None, "arguments": {}, "claimed": ""}
        name, arguments = self.calls.pop(0)
        return {"name": name, "arguments": arguments, "claimed": "",
                "call_id": f"c{len(self.seen)}"}


GRASP = ("grasp", {"object": "red_block", "side": "left", "direction": "down"})
APPROACH = ("approach", {"object": "red_block", "side": "left",
                         "direction": "down"})
GOAL = Place(object="red_block", to="box")


def _mirror():
    from scene import DEMO_WRIST_CAMERA, demo_scene
    world, kin = demo_scene()
    return KinematicMirror(kin, world, wrist_intrinsics=DEMO_WRIST_CAMERA)


def _no_frame(side):
    return None


def _misplaced(robot, dy_m: float):
    """The TRUE red block is ``dy_m`` (base y) from where the scene declares
    it: the judge sees the truth, the kit believes the declaration."""
    declared = robot.world().find("red_block").p.copy()
    truth = declared + np.array([0.0, dy_m, 0.0])
    return declared, truth, geometry_judge({"red_block": truth})


def test_the_servo_aligns_the_hand_and_the_stroke_runs_in_the_same_turn(
        agent_examples):
    """Declared 40 mm off along base y. The servo steps toward the truth
    (coarse 30 mm, then fine 10 mm), re-declares the block as a sighting, and
    the grasp the model asked for runs on that turn — no wrist-look text."""
    robot = _mirror()
    declared, truth, judge = _misplaced(robot, 0.040)
    servo = Servo(_no_frame, judge)
    model = Script(APPROACH, GRASP)
    trace = run(goal=GOAL, robot=robot, policy=OperatorPolicy(max_turns=2),
                ask=model, servo=servo)
    approach, grasp = trace.records
    assert approach.servo is None and approach.run["completed"]
    assert grasp.servo["outcome"] == ALIGNED
    steps = [s for s in grasp.servo["steps"] if s["step_m"] is not None]
    assert 1 <= len(steps) <= 3
    for step in steps:
        assert step["run"]["completed"] and step["plan"]["primitive"] == "nudge"
    # every judgement is in the trace with its distribution
    assert all(set(s["distribution"]) == set(CHOICES)
               for s in grasp.servo["steps"])
    assert grasp.distribution["on"] == 1.0
    # the stroke ran, on the re-declared (sighted) block
    assert grasp.run["completed"] and grasp.verdict["verdict"] == "true"
    block = next(o for o in grasp.world["objects"] if o["name"] == "red_block")
    assert block["provenance"] == "observed"
    assert abs(block["p"][1] - truth[1]) < abs(declared[1] - truth[1])
    assert abs(block["p"][1] - truth[1]) <= 0.015
    # and the model never read a WRIST LOOK question
    assert not any("WRIST LOOK" in str(m.get("content")) for m in model.seen)


def test_the_nudge_budget_ends_the_servo_and_the_stroke_does_not_run(
        agent_examples):
    """Declared far off: three corrections (the default budget) are not
    enough, the servo stops on the policy's budget, the model is told."""
    robot = _mirror()
    _declared, _truth, judge = _misplaced(robot, 0.20)
    model = Script(APPROACH, GRASP)
    trace = run(goal=GOAL, robot=robot,
                policy=OperatorPolicy(max_turns=3, max_nudges_per_target=2),
                ask=model, servo=Servo(_no_frame, judge))
    grasp = trace.records[1]
    assert grasp.servo["outcome"] == BUDGET
    assert sum(1 for s in grasp.servo["steps"] if s["step_m"]) == 2
    assert grasp.run is None
    assert grasp.refused and "could not align" in grasp.refused[0]["detail"]
    assert any("could not align" in str(m.get("content"))
               for m in model.history)


def test_an_unsure_judge_makes_no_blind_step(agent_examples):
    robot = _mirror()
    run(goal=GOAL, robot=robot, policy=OperatorPolicy(max_turns=1),
        ask=Script(APPROACH))

    def shrug(look: ServoLook):
        return {"left": 0.4, "right": 0.35, "on": 0.25}

    trace = run(goal=GOAL, robot=robot, policy=OperatorPolicy(max_turns=1),
                ask=Script(GRASP), servo=Servo(_no_frame, shrug))
    record = trace.records[0]
    assert record.servo["outcome"] == "unsure"
    assert record.servo["steps"][0]["step_m"] is None
    assert record.run is None


def test_a_judge_answering_off_the_menu_is_an_error(agent_examples):
    robot = _mirror()
    run(goal=GOAL, robot=robot, policy=OperatorPolicy(max_turns=1),
        ask=Script(APPROACH))
    with pytest.raises(ValueError, match="did not offer"):
        run(goal=GOAL, robot=robot, policy=OperatorPolicy(max_turns=1),
            ask=Script(GRASP),
            servo=Servo(_no_frame, lambda look: {"sideways": 1.0}))


@pytest.mark.parametrize("side", ["left", "right"])
def test_image_directions_recover_the_sign_of_a_base_offset(agent_examples,
                                                            side):
    """For both hands' cameras: a point displaced along a base axis projects
    to one side of the mark, and that image choice maps back to a table-plane
    direction with a positive component along the same axis — the sign the
    servo steps with."""
    from scipy.spatial.transform import Rotation as R

    from manipulation_kit.perception import WristCamera
    from scene import DEMO_WRIST_CAMERA
    # the hand pointing straight down at the table, as at a top-down standoff
    camera = WristCamera.from_flange(side, np.array([0.4, 0.2, 0.3]),
                                     R.from_euler("x", 180, degrees=True),
                                     **DEMO_WRIST_CAMERA)
    # a point 0.25 m in front of the lens
    p = camera.p + camera.r.apply([0.0, 0.0, 0.25])
    mark = camera.project_point(p)
    assert mark.visible
    for axis in (np.array([0.0, 0.05, 0.0]), np.array([0.05, 0.0, 0.0])):
        seen = camera.project_point(p + axis)
        du, dv = seen.u - mark.u, seen.v - mark.v
        choice = (("left" if du < 0 else "right") if abs(du) >= abs(dv)
                  else ("above" if dv < 0 else "below"))
        direction = image_direction_in_base(camera, choice)
        assert direction is not None
        assert float(np.dot(direction, axis)) > 0.0, (side, choice, direction)


def test_without_a_servo_the_look_is_still_a_question(agent_examples):
    """The default path is untouched."""
    robot = _mirror()
    trace = run(goal=GOAL, robot=robot, policy=OperatorPolicy(max_turns=2),
                ask=Script(APPROACH, GRASP))
    look = trace.records[1]
    assert look.servo is None and look.run is None
    assert "WRIST LOOK" in look.refused[0]["detail"]


# --------------------------------------------------------------------------- #
# the example (C.12: the part that knows a model exists, and nothing else)
# --------------------------------------------------------------------------- #

def test_the_jev_servo_example_runs_dry_and_aligns(agent_examples, tmp_path,
                                                   capsys):
    import jev_servo
    assert jev_servo.main(["--dry-run", "--misplace-mm", "40",
                           "--trace", str(tmp_path / "t.jsonl")]) == 0
    out = capsys.readouterr().out
    assert "[servo aligned, 1 step(s)]" in out
    assert '"stop": "goal_verified"' in out
    import json
    records = [json.loads(line) for line in
               (tmp_path / "t.jsonl").read_text().splitlines()]
    servo = next(r["servo"] for r in records if r["servo"])
    assert servo["outcome"] == ALIGNED
    assert servo["steps"][0]["choice"] in ("left", "right", "above", "below")
    assert servo["steps"][-1]["choice"] == "on"


def test_the_jev_servo_example_is_short_and_carries_no_policy(agent_examples):
    from pathlib import Path
    import jev_servo
    text = Path(jev_servo.__file__).read_text(encoding="utf-8")
    assert len(text.splitlines()) < 200
    assert "os.environ" not in text and "getenv" not in text
    assert "manipulation_kit.primitives.orientation" not in text
    # no direction, step or budget decided here: the kit's ids are the menu
    from manipulation_kit.agent.servo import CHOICES
    assert set(jev_servo.LABELS) == set(CHOICES)
    options = {opt for action in jev_servo.build_parser()._actions
               for opt in action.option_strings}
    assert {"--judge", "--misplace-mm", "--max-nudges-per-target",
            "--no-look-before-stroke", "--snapshot-cmd"} <= options


def test_the_jev_judge_refuses_to_judge_without_a_photo(agent_examples):
    import jev_servo
    look = ServoLook(side="left", object="cup", camera=None, u=1.0, v=1.0,
                     depth_m=0.2, image=None, photo=None)
    with pytest.raises(RuntimeError, match="snapshot-cmd"):
        jev_servo.JevJudge()(look)


def test_the_servo_needs_a_place_goal_like_the_loop(agent_examples):
    robot = _mirror()
    with pytest.raises(TypeError):
        run(goal="put it there", robot=robot, policy=OperatorPolicy(),
            ask=Script(), servo=Servo(_no_frame, geometry_judge({})))
    assert isinstance(GOAL, Place)
