"""``manipulation_kit.agent.servo``: the look before a stroke answered by a
judge that chooses, the kit stepping the hand on its own grid."""

from __future__ import annotations

import numpy as np
import pytest

from manipulation_kit.agent import (DecisionTrace, KinematicMirror,
                                    OperatorPolicy, Servo, geometry_judge, run)
from manipulation_kit.agent.servo import (ALIGNED, BUDGET, CHOICES,
                                          ServoLook, image_direction_in_base,
                                          photoless)
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
    # a JUDGEMENT, not a sighting: the classifier chose, nobody measured
    assert block["provenance"] == "judged"
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
                ask=Script(GRASP), servo=Servo(_no_frame, photoless(shrug)))
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
            servo=Servo(_no_frame, photoless(lambda look: {"sideways": 1.0})))


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
    assert "servo aligned: on 1.00" in out and "after 1 step(s)" in out
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
            "--no-look-before-stroke", "--snapshot-cmd", "--judge-url",
            "--judge-only", "--refine", "--rejudge"} <= options
    for helper in ("jev_judge.py", "jev_judge_server.py"):
        text = (Path(jev_servo.__file__).parent / helper).read_text()
        assert len(text.splitlines()) < 200, helper
        assert "os.environ" not in text and "getenv" not in text


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


# --------------------------------------------------------------------------- #
# provenance "judged": a statement, like "declared" — never a sighting
# --------------------------------------------------------------------------- #

def _held_world(provenance: str, *, gap):
    """The left hand stalled closed at the block, the block's pose ``provenance``;
    no ``held_object`` from the producer, so only position can associate."""
    from manipulation_kit.world import (ArmView, GripperView, ObjectView,
                                        WorldView)
    block = ObjectView("red_block", p=(0.4, 0.2, 0.05), size=(0.05, 0.04, 0.05),
                       provenance=provenance, confidence=0.8)
    arm = ArmView("left", joints=np.zeros(7), tool_p=(0.4, 0.2, 0.05))
    hand = GripperView("left", 1.0, holding=True, jaw_stalled=True,
                       jaw_gap_m=gap, open_gap_m=0.052)
    return WorldView.of([block], arms=[arm], grippers=[hand])


@pytest.mark.parametrize("provenance", ["judged", "declared"])
def test_a_judged_pose_is_a_statement_not_a_sighting(provenance):
    from manipulation_kit.primitives.verifiers import Holding
    from manipulation_kit.world import INFERRED, STATED
    assert "judged" in STATED and "judged" not in INFERRED
    world = _held_world(provenance, gap=None)
    block = world.find("red_block")
    assert not block.inferred
    if provenance == "judged":
        assert "JUDGED" in block.to_text(world.frames)
        assert "confidence 0.80" in block.to_text(world.frames)
    # no measured gap: the stated position is where the grasp aimed, so it
    # ties nothing to the stall — UNKNOWN, never TRUE
    verdict = Holding("grasp", world, "left", obj=block).measure(world)
    assert verdict.verdict == "unknown", verdict.reason
    assert provenance in verdict.reason
    # a measured gap that fits the object decides (grip_fit)
    fitted = _held_world(provenance, gap=0.041)
    verdict = Holding("grasp", fitted, "left",
                      obj=fitted.find("red_block")).measure(fitted)
    assert verdict.verdict == "true", verdict.reason
    assert verdict.measured["association"] == "stated_position+grip_fit"
    assert verdict.measured["provenance"] == provenance


def _align(robot, servo):
    """One servo alignment on the left hand after an approach."""
    from manipulation_kit.agent.policy import PolicyState
    from manipulation_kit.executor import run as run_plan
    return servo.align(robot=robot, policy=OperatorPolicy(), state=PolicyState(),
                       side="left", name="red_block", settings={},
                       run_plan=run_plan)


def _approached():
    robot = _mirror()
    run(goal=GOAL, robot=robot, policy=OperatorPolicy(max_turns=1),
        ask=Script(APPROACH))
    return robot


def test_the_servo_carries_the_judges_confidence(agent_examples):
    robot = _approached()
    _declared, _truth, geometry = _misplaced(robot, 0.030)
    answers = []

    def seventy_percent(look):
        (choice, _p), = geometry(look).items()
        answers.append(choice)
        return {choice: 0.7, "not_visible": 0.3}

    report = _align(robot, Servo(_no_frame, photoless(seventy_percent)))
    assert report.outcome == ALIGNED and answers[0] != "on"
    block = robot.world().find("red_block")
    assert block.provenance == "judged"
    assert block.confidence == pytest.approx(0.7)


# --------------------------------------------------------------------------- #
# refine: the residual under 5 mm without moving the hand
# --------------------------------------------------------------------------- #

def test_refine_adds_judgements_but_no_motion(agent_examples):
    reports = []
    for refine in (False, True):
        robot = _approached()
        _declared, _truth, judge = _misplaced(robot, 0.040)
        reports.append(_align(robot, Servo(_no_frame, judge, refine=refine)))
    rough, fine = reports
    assert rough.outcome == fine.outcome == ALIGNED
    moves = [sum(1 for s in r.steps if s.step_m) for r in reports]
    assert moves == [1, 1]                   # refinement adds no motion
    refines = [s for s in fine.steps if s.kind == "refine"]
    assert 2 <= len(refines) <= 9
    assert all(s.step_m is None and s.shift_m is not None for s in refines)
    assert "refined: declaration moved" in fine.detail
    assert "hand not moved" in fine.detail


def test_refine_lands_within_refine_m_of_the_truth(agent_examples):
    """A perfect judge: the 30 mm step leaves 10 mm (inside the tolerance);
    the refinement grid brings the declaration within 5 mm."""
    robot = _approached()
    _declared, truth, judge = _misplaced(robot, 0.040)
    report = _align(robot, Servo(_no_frame, judge, refine=True))
    assert report.outcome == ALIGNED
    block = robot.world().find("red_block")
    off = float(np.hypot(*(block.p[:2] - truth[:2])))
    assert off <= 0.0051, off
    assert block.provenance == "judged"


def test_refine_never_takes_a_worse_mark(agent_examples):
    """A judge that rates the unshifted mark best: the declaration stays."""
    robot = _approached()
    before = robot.world().find("red_block").p.copy()

    def centred(look):
        here = np.allclose(look.declared_p[:2], before[:2], atol=1e-6)
        on = 0.9 if here else 0.6
        return {"on": on, "left": 1.0 - on}

    report = _align(robot, Servo(_no_frame, photoless(centred), refine=True))
    assert report.outcome == ALIGNED
    assert "unshifted mark is best" in report.detail
    assert np.allclose(robot.world().find("red_block").p, before)


# --------------------------------------------------------------------------- #
# judge-only: recorded, never stepping
# --------------------------------------------------------------------------- #

def test_judge_only_records_the_distribution_and_never_steps(agent_examples):
    robot = _mirror()
    _declared, _truth, judge = _misplaced(robot, 0.040)
    model = Script(APPROACH, GRASP)
    trace = run(goal=GOAL, robot=robot, policy=OperatorPolicy(max_turns=2),
                ask=model, servo=Servo(_no_frame, judge, observe_only=True))
    approach, look = trace.records
    joints = [a for a in approach.observation_after["arms"] if a["side"] == "left"]
    assert look.servo["outcome"] == "judged_only"
    assert [s["step_m"] for s in look.servo["steps"]] == [None]
    assert look.distribution["left"] == 1.0          # the judge's answer, kept
    assert look.run is None and look.plan is None
    # the model got the ordinary look question, the declaration is untouched
    assert "WRIST LOOK" in look.refused[0]["detail"]
    block = robot.world().find("red_block")
    assert block.provenance != "judged"
    now = [a for a in robot.world().to_json()["arms"] if a["side"] == "left"]
    assert now[0]["joints_deg"] == joints[0]["joints_deg"]


def test_the_jev_servo_example_runs_judge_only_to_the_goal(agent_examples,
                                                            tmp_path, capsys):
    import jev_servo
    assert jev_servo.main(["--dry-run", "--misplace-mm", "40", "--judge-only",
                           "--trace", str(tmp_path / "t.jsonl")]) == 0
    out = capsys.readouterr().out
    assert "servo judged: left 1.00 (" in out and "— recorded only" in out
    assert '"stop": "goal_verified"' in out


# --------------------------------------------------------------------------- #
# the remote judge seam and the offline re-judge
# --------------------------------------------------------------------------- #

class FakeJev:
    """``predict`` with Jev-Omni's signature: 'inside the green box' wins."""

    loaded = True

    def __init__(self):
        self.calls = []

    def predict(self, *, state, question, options, media, modality):
        from pathlib import Path
        assert modality == "image" and Path(media).stat().st_size > 0
        self.calls.append(question)
        return {"probabilities": {o: (0.8 if "entirely inside" in o
                                      else 0.2 / (len(options) - 1))
                                  for o in options}}


@pytest.fixture
def judge_server():
    import threading
    import jev_judge_server
    fake = FakeJev()
    server = jev_judge_server.make_server(fake, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_address[1]}", fake
    finally:
        server.shutdown()
        server.server_close()


def _photo(path):
    from PIL import Image
    Image.new("RGB", (640, 480), (128, 128, 128)).save(path)
    return path


def test_the_remote_judge_answers_over_http(agent_examples, tmp_path,
                                            judge_server):
    import json
    import urllib.request
    from jev_judge import RemoteJudge
    url, fake = judge_server
    look = ServoLook(side="left", object="red_block", camera=None, u=320.0,
                     v=240.0, depth_m=0.2, image=_photo(tmp_path / "m.jpg"),
                     photo=None)
    dist = RemoteJudge(url)(look)
    assert set(dist) == set(CHOICES) and max(dist, key=dist.get) == "on"
    # one question, asked about each of the four views of the photo
    assert fake.calls == ["How does the red block sit relative to the green box?"] * 4
    with urllib.request.urlopen(url + "/health") as reply:
        assert json.loads(reply.read())["ok"] is True
    bad = urllib.request.Request(url + "/judge", data=b'{"options": []}',
                                 headers={"Content-Type": "application/json"})
    with pytest.raises(urllib.error.HTTPError) as err:
        urllib.request.urlopen(bad)
    assert err.value.code == 400


def test_the_servo_runs_with_the_remote_judge(agent_examples, tmp_path,
                                              judge_server):
    from jev_judge import RemoteJudge
    url, fake = judge_server
    robot = _mirror()
    trace = run(goal=GOAL, robot=robot, policy=OperatorPolicy(max_turns=2),
                ask=Script(APPROACH, GRASP),
                servo=Servo(lambda side: _photo(tmp_path / f"{side}.jpg"),
                            RemoteJudge(url), out_dir=tmp_path))
    grasp = trace.records[1]
    assert grasp.servo["outcome"] == ALIGNED and grasp.run["completed"]
    assert len(fake.calls) == 4
    # the marked photo, beside the servo's out_dir, named by the turn
    assert list(tmp_path.glob("turn1_servo_left.png"))
    # the recorded servo judgement is re-judged offline from its own photo
    import json
    from jev_judge import rejudge
    from scene import DEMO_WRIST_CAMERA
    (tmp_path / "trace.jsonl").write_text(
        "\n".join(json.dumps(r.to_json()) for r in trace.records) + "\n")
    said = []
    assert rejudge(tmp_path, RemoteJudge(url), kin=robot.kin,
                   wrist={"left": DEMO_WRIST_CAMERA}, say=said.append) == 1
    assert said[0].startswith("turn 1 servo step 0 (was on 0.80)")


def test_rejudge_replays_a_runs_wrist_photos(agent_examples, tmp_path,
                                             judge_server, capsys):
    """A finished run (here the dry run) plus the wrist photos
    ``--snapshot-cmd`` saved: every photo is marked where the trace's
    declared objects project through the robot profile's lens and judged
    again, nothing moved."""
    import jev_servo
    from pathlib import Path
    url, fake = judge_server
    run_dir = tmp_path / "run"
    assert jev_servo.main(["--dry-run", "--trace",
                           str(run_dir / "trace.jsonl")]) == 0
    for turn in range(2):
        for side in ("left", "right"):
            _photo(run_dir / f"turn{turn}_{side}_wrist_0_rgb.jpg")
    capsys.readouterr()
    profile = Path(__file__).resolve().parents[1] / "data/d1-2.camera_calibration.json"
    assert jev_servo.main(["--rejudge", str(run_dir), "--robot-profile",
                           str(profile), "--judge-url", url]) == 0
    out = capsys.readouterr().out
    assert "turn 1 left_wrist red_block: mark (" in out
    assert "-> on 0.80" in out
    assert fake.calls and list((run_dir / "rejudge").glob("servo*_*.png"))


def test_rejudge_without_a_lens_is_a_usage_error(agent_examples, tmp_path):
    import jev_servo
    with pytest.raises(SystemExit) as stop:
        jev_servo.main(["--rejudge", str(tmp_path), "--robot-profile",
                        str(tmp_path / "missing.json"),
                        "--judge-url", "http://127.0.0.1:1"])
    assert stop.value.code == 2


# --------------------------------------------------------------------------- #
# before EVERY stroke: the model's own wrist locate does not replace the servo
# (d1-2 2026-09-24 01:47Z, --judge-only: locate(right_wrist) -> grasp, twice;
# twelve records, servo: null on every one)
# --------------------------------------------------------------------------- #

def _wrist_pixel(robot, side="left", name="red_block"):
    """Where the declared object projects in the wrist camera right now —
    the pixel a model reading the photo would locate on."""
    world = robot.world()
    seen = robot.cameras(world)[f"{side}_wrist"].project_object(
        world.find(name), world.frames)
    assert seen.visible
    return float(seen.u), float(seen.v)


def _locate(u, v, side="left"):
    return ("locate", {"camera": f"{side}_wrist", "u": u, "v": v,
                       "size": [0.05, 0.04, 0.05]})


def _photo_judge(answer):
    """A judge that READS the photo (the default): asserts it got one."""
    seen = []

    def judge(look):
        assert look.image is not None and look.image.exists()
        seen.append(look)
        return dict(answer)
    judge.seen = seen
    return judge


def test_the_servo_judges_a_grasp_the_model_already_located(agent_examples,
                                                             tmp_path):
    """Judge-only, the d1-2 shape: the model's wrist locate satisfies the
    policy's look, the grasp's stroke runs — and the servo judged it first,
    its marked photo beside the trace as ``turn{N}_servo_{side}.png``."""
    robot = _approached()
    u, v = _wrist_pixel(robot)
    judge = _photo_judge({"on": 0.81, "left": 0.12, "right": 0.04,
                          "above": 0.02, "below": 0.01})
    run_dir = tmp_path / "run"
    trace = run(goal=GOAL, robot=robot, policy=OperatorPolicy(max_turns=2),
                ask=Script(_locate(u, v), GRASP),
                trace=DecisionTrace(run_dir / "trace.jsonl"),
                servo=Servo(lambda side: _photo(tmp_path / f"{side}.jpg"),
                            judge, observe_only=True))
    located, grasp = trace.records
    assert located.servo is None                  # a locate is not a stroke
    assert grasp.servo is not None
    assert grasp.servo["outcome"] == "judged_only"
    assert grasp.servo["steps"][0]["distribution"]["on"] == pytest.approx(0.81)
    assert len(judge.seen) == 1
    marked = run_dir / "turn1_servo_left.png"
    assert marked.exists()
    assert grasp.servo["steps"][0]["look"]["image"] == str(marked)
    # recorded only: the model's locate was the look, the stroke ran
    assert grasp.run is not None and grasp.run["completed"]
    assert [s["step_m"] for s in grasp.servo["steps"]] == [None]
    from manipulation_kit.agent.servo import servo_line
    assert servo_line(grasp.servo) == (
        "servo judged: on 0.81 (left 0.12, right 0.04, above 0.02, below "
        "0.01, not_visible 0.00) — recorded only")


def test_the_servo_aligns_a_grasp_the_model_already_located(agent_examples):
    """Live: the model locates on the DECLARED pixel (it believes the
    declaration), the true block is 30 mm off. The servo still judges, steps
    the hand 30 mm and the stroke runs in the same turn, aligned within the
    tolerance of the truth."""
    from manipulation_kit.agent.servo import DEFAULT_TOLERANCE_M
    robot = _approached()
    u, v = _wrist_pixel(robot)
    # the truth is 30 mm from the declaration the STROKE would plan to — the
    # one the model's locate left, which the servo's first look is drawn for
    truth = {}
    geometry = geometry_judge(truth)

    def judge(look):
        truth.setdefault("red_block",
                         np.array(look.declared_p) + [0.0, 0.030, 0.0])
        return geometry(look)

    trace = run(goal=GOAL, robot=robot, policy=OperatorPolicy(max_turns=2),
                ask=Script(_locate(u, v), GRASP),
                servo=Servo(_no_frame, photoless(judge)))
    truth = truth["red_block"]
    located, grasp = trace.records
    assert located.servo is None
    assert grasp.servo["outcome"] == ALIGNED
    steps = [s for s in grasp.servo["steps"] if s["step_m"] is not None]
    assert len(steps) == 1
    assert float(np.hypot(*steps[0]["step_m"])) == pytest.approx(0.030)
    # the stroke ran on the JUDGED declaration, within tolerance of the truth
    block = next(o for o in grasp.world["objects"] if o["name"] == "red_block")
    assert block["provenance"] == "judged"
    assert float(np.hypot(*(np.array(block["p"][:2]) - truth[:2]))) <= (
        DEFAULT_TOLERANCE_M + 1e-4)
    assert grasp.run["completed"] and grasp.verdict["verdict"] == "true"
    assert grasp.look["visible"] is True


def test_a_photo_judge_without_a_wrist_frame_is_recorded_as_skipped(
        agent_examples):
    """No snapshotter: the photo judge is not asked, and the record says so
    (never a silent null); the model's own look rule follows."""
    robot = _approached()
    judge = _photo_judge({"on": 1.0})
    trace = run(goal=GOAL, robot=robot, policy=OperatorPolicy(max_turns=2),
                ask=Script(GRASP, GRASP), servo=Servo(_no_frame, judge))
    look, grasp = trace.records
    for record in (look, grasp):
        assert record.servo["skipped"] == "no wrist frame"
        assert record.servo["steps"] == []
    assert judge.seen == []
    # the model's look, then its grasp
    assert "WRIST LOOK" in look.refused[0]["detail"] and look.run is None
    assert grasp.run["completed"]
    from manipulation_kit.agent.servo import servo_line
    assert servo_line(grasp.servo) == "servo skipped: no wrist frame"


def test_the_d1_2_trace_shape_never_leaves_the_servo_null(agent_examples,
                                                         tmp_path):
    """The shape of d1-2's judge-only run (servo-judgeonly-1): approach, then
    locate on the wrist before each grasp, twice. Every grasp record carries
    the servo's judgement."""
    robot = _approached()
    u, v = _wrist_pixel(robot)
    calls = [_locate(u, v), GRASP, _locate(u + 1, v - 14), GRASP,
             _locate(u + 6, v - 14),
             ("retreat", {"side": "left", "distance_m": 0.1,
                          "direction": "up"}),
             ("go_home", {"side": "left"})]
    judge = _photo_judge({"left": 0.7, "on": 0.3})
    trace = run(goal=GOAL, robot=robot,
                policy=OperatorPolicy(max_turns=len(calls)),
                ask=Script(*calls),
                servo=Servo(lambda side: _photo(tmp_path / f"{side}.jpg"),
                            judge, observe_only=True, out_dir=tmp_path))
    strokes = [r for r in trace.records
               if (r.choice or {}).get("name") == "grasp"]
    assert len(strokes) == 2
    for record in strokes:
        assert record.servo is not None, record.iteration
        assert record.servo["outcome"] == "judged_only"
    others = [r for r in trace.records
              if (r.choice or {}).get("name") not in ("grasp",)]
    assert all(r.servo is None for r in others)


def test_the_servo_verbs_are_explicit():
    from manipulation_kit.agent.policy import DIRECTED_VERBS
    from manipulation_kit.agent.servo import SERVO_VERBS
    assert SERVO_VERBS == {"grasp": "object", "press": "target"}
    assert set(SERVO_VERBS) <= set(DIRECTED_VERBS)
    for verb in ("approach", "probe", "handover"):
        assert verb not in SERVO_VERBS


# --------------------------------------------------------------------------- #
# the inner loop: photo -> judge -> accumulate -> step or stop, at the judge
# rate (d1-2 2026-09-24: one photo per turn, "below" at 0.36 / 0.28 under a
# 0.50 gate, no step, the grasp refused)
# --------------------------------------------------------------------------- #

def _lukewarm(top, p, on, rest_to=None):
    """A distribution the live run would give: ``top`` at ``p``, ``on`` at
    ``on``, the rest spread evenly."""
    others = [c for c in CHOICES if c not in (top, "on")]
    out = {c: (1.0 - p - on) / len(others) for c in others}
    out.update({top: p, "on": on})
    return out


def test_decide_accumulates_and_never_steps_on_a_contradiction():
    from manipulation_kit.agent.servo import accumulate, decide
    one = _lukewarm("below", 0.31, 0.24)          # d1-2 turn 6
    two = _lukewarm("below", 0.36, 0.14)          # d1-2 turn 8
    assert decide(one, 1, window=3, margin=0.10)[0] == "more"
    acc = accumulate([one, two])
    assert acc["below"] == pytest.approx(0.335)
    assert decide(acc, 2, window=3, margin=0.10)[0] == "below"
    # left and right both high: the object is not on two sides at once
    torn = {"left": 0.4, "right": 0.35, "on": 0.25}
    assert decide(accumulate([torn] * 3), 3, window=3, margin=0.10)[0] == "flat"
    assert decide(accumulate([torn]), 1, window=3, margin=0.10)[0] == "more"
    assert decide({"on": 0.5, "left": 0.3, "below": 0.2}, 1, window=3,
                  margin=0.10)[0] == "on"
    assert decide({"not_visible": 0.7, "on": 0.3}, 1, window=3,
                  margin=0.10)[0] == "not_visible"


def test_a_flat_judge_takes_window_photos_and_no_step(agent_examples):
    robot = _approached()
    asked = []

    def flat(look):
        asked.append(look)
        return {c: 1.0 / len(CHOICES) for c in CHOICES}

    lines = []
    report = _align(robot, Servo(_no_frame, photoless(flat), log=lines.append))
    assert report.outcome == "unsure" and len(asked) == 3
    assert all(s.step_m is None for s in report.steps)
    assert [s.frames for s in report.steps] == [1, 2, 3]
    assert "flat over 3 photos" in report.detail
    assert len(lines) == 4 and lines[-1].startswith("t=+")
    assert "unsure" in lines[-1] and "3 photos, 0 steps" in lines[-1]


def test_a_consistent_lukewarm_judge_steps_until_on(agent_examples):
    """The judge never exceeds 0.36, but keeps pointing the same way: the
    loop steps on the grid until the truth is inside the tolerance, and the
    stroke may run."""
    robot = _approached()
    _declared, truth, geometry = _misplaced(robot, 0.040)

    def lukewarm(look):
        (choice, _p), = geometry(look).items()
        return (_lukewarm("left", 0.20, 0.40) if choice == "on"
                else _lukewarm(choice, 0.36, 0.20))

    lines = []
    report = _align(robot, Servo(_no_frame, photoless(lukewarm),
                                 log=lines.append))
    assert report.outcome == ALIGNED, report.detail
    steps = [s for s in report.steps if s.step_m is not None]
    assert 1 <= len(steps) <= 3
    from manipulation_kit.primitives.types import NUDGE_GRID_M
    for s in steps:
        assert min(abs(float(np.hypot(*s.step_m)) - g) for g in NUDGE_GRID_M) < 1e-9
    block = robot.world().find("red_block")
    assert abs(block.p[1] - truth[1]) <= 0.0101
    assert any("-> step +30mm along image-" in line for line in lines)
    assert report.iterations == len(report.steps)


def test_the_wall_clock_budget_ends_the_loop(agent_examples):
    """A judge that keeps asking for another look (never decisive, window
    never full) runs into the loop's own clock."""
    from manipulation_kit.agent.servo import TIMEOUT
    robot = _approached()
    now = [100.0]

    def clock():
        now[0] += 1.0                 # every reading: a second later
        return now[0]

    def flat(look):
        return {c: 1.0 / len(CHOICES) for c in CHOICES}

    report = _align(robot, Servo(_no_frame, photoless(flat), window=50,
                                 budget_s=4.0, clock=clock, log=lambda s: None))
    assert report.outcome == TIMEOUT
    assert 1 <= report.iterations <= 3
    report = _align(robot, Servo(_no_frame, photoless(flat), window=50,
                                 max_iterations=5, log=lambda s: None))
    assert report.outcome == TIMEOUT and report.iterations == 5


def test_the_policy_budget_still_ends_a_lukewarm_loop(agent_examples):
    robot = _approached()
    _declared, _truth, geometry = _misplaced(robot, 0.20)

    def lukewarm(look):
        (choice, _p), = geometry(look).items()
        return _lukewarm(choice, 0.36, 0.20)

    from manipulation_kit.agent.policy import PolicyState
    from manipulation_kit.executor import run as run_plan
    report = Servo(_no_frame, photoless(lukewarm), log=lambda s: None).align(
        robot=robot, policy=OperatorPolicy(max_nudges_per_target=2),
        state=PolicyState(), side="left", name="red_block", settings={},
        run_plan=run_plan)
    assert report.outcome == BUDGET
    assert sum(1 for s in report.steps if s.step_m) == 2


def test_a_photo_written_before_the_move_is_not_judged(agent_examples,
                                                       tmp_path):
    """After a correction the next photo must be written after the move
    ended: a frame source that keeps handing an old file stops the loop as
    ``stale_frame`` instead of judging the old pose."""
    import os
    from manipulation_kit.agent.servo import STALE
    robot = _approached()
    _declared, _truth, geometry = _misplaced(robot, 0.040)
    old = _photo(tmp_path / "old.jpg")
    os.utime(old, (1.0, 1.0))                    # written long ago
    judge = _photo_judge({})
    judge_answers = []

    def reads(look):
        judge_answers.append(look)
        (choice, _p), = geometry(look).items()
        return {choice: 1.0}

    report = _align(robot, Servo(lambda side: old, reads, settle_s=0.0,
                                 log=lambda s: None))
    assert report.outcome == STALE
    assert sum(1 for s in report.steps if s.step_m) == 1
    assert len(judge_answers) == 1              # the old photo judged once


def test_the_iteration_line_says_what_was_seen_and_done():
    from manipulation_kit.agent.servo import (ServoReport, ServoStep,
                                              exit_line, iteration_line)
    step = ServoStep(look={}, distribution=_lukewarm("below", 0.41, 0.30),
                     choice="below", confidence=0.41, step_m=(-0.010, 0.0),
                     iteration=3, t_s=0.83, accumulated=_lukewarm("below", 0.38, 0.29),
                     frames=2, frame_age_s=0.21,
                     timing_s={"frame": 0.21, "judge": 0.17, "step": 0.44})
    line = iteration_line(step, side="right")
    assert line.startswith("t=+0.83s iter 3 side=right judge below 0.41 "
                           "(on 0.30, ")
    assert "acc[2] below 0.38" in line
    assert ("-> step +10mm along image-below -> base (dx,dy)=(-0.010,+0.000)"
            " m") in line
    assert line.endswith("[frame 0.21s judge 0.17s step 0.44s]")
    report = ServoReport(side="right", object="tape", outcome="aligned",
                         detail="on the mark at 0.61 over 2 photos",
                         steps=[step], elapsed_s=2.1, iterations=4)
    assert exit_line(report) == (
        "t=+2.10s servo right 'tape': aligned — on the mark at 0.61 over 2 "
        "photos (4 photos, 1 step, 0.53 s/iteration)")


def test_the_trace_keeps_every_iteration(agent_examples):
    robot = _mirror()
    _declared, _truth, judge = _misplaced(robot, 0.040)
    trace = run(goal=GOAL, robot=robot, policy=OperatorPolicy(max_turns=2),
                ask=Script(APPROACH, GRASP),
                servo=Servo(_no_frame, judge, log=lambda s: None))
    servo = trace.records[1].servo
    assert servo["iterations"] == len(servo["steps"]) >= 2
    assert servo["elapsed_s"] >= 0.0
    for i, step in enumerate(servo["steps"], start=1):
        assert step["iteration"] == i
        assert set(step["accumulated"]) == set(CHOICES)
        assert step["decision"] and "judge" in step["timing_s"]
