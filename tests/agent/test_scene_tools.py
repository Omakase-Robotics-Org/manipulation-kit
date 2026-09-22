"""``declare_scene`` and ``locate``: the model as the detector.

The kit owns the verbs and refuses to own the observation — nothing in
``manipulation_kit`` opens a camera — and the scene file was somebody with a
tape measure. These two tools are the third option: the model looks at the
same photograph the loop does and says where things are, and a deterministic
function turns any pixel it picks into base-frame metres.

What is worth pinning about them, and what these check:

* they are OFFERED, every turn, alongside the motion verbs;
* ``declare_scene`` goes through the SAME reader a hand-written scene file
  does (``live.objects_from``), so a model cannot declare something a person
  could not have written;
* a declaration that arrives at turn 0 into an EMPTY scene makes the arm
  choice possible, which it was not before;
* ``locate`` is exact, carries the uncertainty the nominal head mount has,
  and refuses rather than extrapolating when there is no plane or the ray
  climbs above the horizon;
* nothing in the kit gates on ``confidence``, so a 0.3 declaration is still
  plannable — which is what makes declare-then-nudge possible.

No network, no robot: the model is a stub and the executor is the kinematic
mirror.
"""

from __future__ import annotations

import json

import numpy as np
import pytest


@pytest.fixture
def loop_module(agent_examples):
    import astra_loop
    return astra_loop


@pytest.fixture
def camera(agent_examples):
    from manipulation_kit.perception import HeadCamera
    return HeadCamera.from_robot(width=640, height=480, neck_pitch=0.512,
                                 lift_m=0.205)


def _empty_scene():
    """A perceived scene with a table and nothing on it — the default."""
    return {"objects": [{"name": "table", "kind": "surface",
                         "frame_id": "base", "p": [0.55, 0.0, 0.156],
                         "size": [0.40, 0.60, 0.02], "yaw_rad": 0.0,
                         "confidence": 0.2}],
            "frames": []}


def _world(scene, kin):
    import dataclasses
    import time

    from live import frames_from, objects_from
    from scene import demo_scene
    world, _ = demo_scene()
    return dataclasses.replace(world, objects=tuple(objects_from(scene)),
                               frames=frames_from(scene, now=time.time()))


def _robot(loop_module, kin, scene):
    return loop_module.build_robot("kinematic", kin, "http://unused", scene,
                                   world0=_world(scene, kin), obj="cup")


DECLARED = {"objects": [
    {"name": "cup", "kind": "container", "p": [0.50, -0.14, 0.221],
     "size": [0.09, 0.09, 0.11], "interior": [0.08, 0.08, 0.10],
     "confidence": 0.6},
    {"name": "charger", "kind": "object", "p": [0.38, -0.17, 0.191],
     "size": [0.045, 0.02, 0.05], "confidence": 0.5}]}


# --------------------------------------------------------------------------- #
# the schemas
# --------------------------------------------------------------------------- #

def test_both_tools_are_offered_beside_the_motion_verbs(loop_module, camera,
                                                        d1_arm):
    from manipulation_kit.primitives.schema import tool_schemas
    world = _world(_empty_scene(), d1_arm)
    names = {t["name"] for t in tool_schemas(world)
             + loop_module.scene_tools(camera)}
    assert {"declare_scene", "locate"} <= names
    assert "nudge" in names, "the motion verbs are still there"


def test_locate_is_not_offered_without_a_camera(loop_module):
    """A tool that sometimes answers from geometry and sometimes from nothing
    is worse than a missing tool."""
    names = {t["name"] for t in loop_module.scene_tools(None)}
    assert names == {"declare_scene"}


def test_the_declare_schema_asks_for_base_metres_and_a_confidence(loop_module):
    schema = loop_module.DECLARE_SCHEMA["parameters"]
    item = schema["properties"]["objects"]["items"]
    assert item["required"] == ["name", "kind", "p", "size"]
    assert item["additionalProperties"] is False
    assert "base metres" in item["properties"]["p"]["description"]
    assert "CENTRE" in item["properties"]["p"]["description"]
    assert item["properties"]["confidence"]["maximum"] == 1.0
    # a container's INSIDE is a separate measurement and has to be sayable,
    # or `place` is unreachable for every scene the model declares
    assert item["properties"]["interior"]["minItems"] == 3


# --------------------------------------------------------------------------- #
# declare_scene
# --------------------------------------------------------------------------- #

def test_a_declaration_becomes_the_world_the_next_turn_is_planned_in(
        loop_module, d1_arm):
    scene = _empty_scene()
    robot = _robot(loop_module, d1_arm, scene)
    assert {o.name for o in robot.world().objects} == {"table"}
    answer = loop_module.apply_declare_scene(robot, DECLARED)
    assert "cup" in answer and "0.500" in answer
    names = {o.name for o in robot.world().objects}
    assert names == {"table", "cup", "charger"}


def test_a_declaration_replaces_by_name_and_leaves_the_rest_alone(
        loop_module, d1_arm):
    robot = _robot(loop_module, d1_arm, _empty_scene())
    loop_module.apply_declare_scene(robot, DECLARED)
    loop_module.apply_declare_scene(robot, {"objects": [
        {"name": "cup", "kind": "container", "p": [0.52, -0.10, 0.221],
         "size": [0.09, 0.09, 0.11], "confidence": 0.9}]})
    objects = {o.name: o for o in robot.world().objects}
    assert set(objects) == {"table", "cup", "charger"}
    assert objects["cup"].p[1] == pytest.approx(-0.10)
    assert objects["charger"].p[1] == pytest.approx(-0.17)


def test_a_stated_interior_is_a_measurement_and_a_missing_one_is_not(
        loop_module, d1_arm):
    """``Place`` refuses to drop into an interior nobody stated. A model that
    can see into a cup has to be able to say what it sees, or the verb is
    unreachable for every scene it declares."""
    robot = _robot(loop_module, d1_arm, _empty_scene())
    loop_module.apply_declare_scene(robot, DECLARED)
    cup = next(o for o in robot.world().objects if o.name == "cup")
    assert cup.interior_measured is True
    assert np.allclose(cup.interior, [0.08, 0.08, 0.10])

    loop_module.apply_declare_scene(robot, {"objects": [
        {"name": "mug", "kind": "container", "p": [0.45, 0.10, 0.20],
         "size": [0.09, 0.09, 0.10]}]})
    mug = next(o for o in robot.world().objects if o.name == "mug")
    assert mug.interior_measured is False


def test_a_malformed_declaration_changes_nothing_and_says_why(loop_module,
                                                              d1_arm):
    robot = _robot(loop_module, d1_arm, _empty_scene())
    before = [o.name for o in robot.world().objects]
    for bad in ({"objects": [{"name": "x", "kind": "object", "p": [0, 0],
                              "size": [0.05, 0.05, 0.05]}]},
                {"objects": [{"name": "x", "kind": "object", "p": [0, 0, 0],
                              "size": [0.05, 0.0, 0.05]}]},
                {"objects": [{"name": "x", "kind": "teapot", "p": [0, 0, 0],
                              "size": [0.05, 0.05, 0.05]}]}):
        answer = loop_module.apply_declare_scene(robot, bad)
        assert "malformed" in answer, answer
    assert [o.name for o in robot.world().objects] == before


def test_a_declared_confidence_reaches_the_text_the_model_reads(loop_module,
                                                                d1_arm):
    """It has been a field on ``ObjectView`` since the first WorldView and
    never reached ``to_text``, so the one honest thing about a detector's
    number was erased on the way to the only consumer that could act on it."""
    robot = _robot(loop_module, d1_arm, _empty_scene())
    loop_module.apply_declare_scene(robot, DECLARED)
    text = robot.world().to_text()
    assert "confidence 0.60" in text
    assert "confidence 0.50" in text
    # the demo scene's own objects are measured, and stay unadorned
    from scene import demo_scene
    assert "confidence" not in demo_scene()[0].to_text()


# --------------------------------------------------------------------------- #
# locate
# --------------------------------------------------------------------------- #

def test_locate_round_trips_a_pixel_through_the_declared_plane(loop_module,
                                                               camera,
                                                               d1_arm):
    world = _world(_empty_scene(), d1_arm)
    answer = loop_module.apply_locate(camera, world, {"u": 400, "v": 380})
    assert "m in base" in answer and "+-" in answer
    # the plane it used is the TOP of the declared surface, not its centre
    assert "z=0.166" in answer
    # and it agrees with the camera model called directly
    point = camera.locate(400, 380, plane_z=0.166).p
    assert f"{point[0]:.3f}" in answer


def test_locate_says_which_surface_and_how_sure_it_is(loop_module, camera,
                                                      d1_arm):
    world = _world(_empty_scene(), d1_arm)
    answer = loop_module.apply_locate(camera, world, {"u": 320, "v": 400})
    assert "'table'" in answer and "confidence 0.2" in answer


def test_locate_refuses_rather_than_extrapolating(loop_module, camera,
                                                  d1_arm):
    from manipulation_kit.perception import HeadCamera
    world = _world(_empty_scene(), d1_arm)
    # at this neck angle EVERY pixel in frame still meets the table; tip the
    # head up and the top of the frame stops being a place.
    level = HeadCamera.from_robot(width=640, height=480, neck_pitch=-0.30)
    assert "above the horizon" in loop_module.apply_locate(
        level, world, {"u": 320, "v": 0})
    assert "not usable" in loop_module.apply_locate(camera, world,
                                                    {"u": "left-ish", "v": 0})
    empty = _world({"objects": [], "frames": []}, d1_arm)
    assert "no table in the scene" in loop_module.apply_locate(
        camera, empty, {"u": 320, "v": 400})
    assert "no camera model" in loop_module.apply_locate(None, world,
                                                         {"u": 1, "v": 1})


def test_locate_takes_the_highest_surface(loop_module, camera, d1_arm):
    """A wagon standing on a floor: the thing on top is the one a pixel lands
    on, and the answer says which."""
    scene = _empty_scene()
    scene["objects"].append({"name": "floor", "kind": "surface",
                             "frame_id": "base", "p": [0.6, 0.0, -0.72],
                             "size": [2.0, 2.0, 0.02], "yaw_rad": 0.0})
    world = _world(scene, d1_arm)
    assert "'table'" in loop_module.apply_locate(camera, world,
                                                 {"u": 320, "v": 400})


# --------------------------------------------------------------------------- #
# the loop, with an empty scene and a model that fills it in
# --------------------------------------------------------------------------- #

class _Declaring:
    """locate, then declare_scene, then play a correct pick-and-place."""

    def __init__(self):
        self.turn = 0
        self.saw_tools = []
        self.script = [
            {"name": "locate", "arguments": {"u": 470, "v": 350}},
            {"name": "declare_scene", "arguments": DECLARED},
            {"name": "grasp", "arguments": {"object": "charger",
                                            "side": "right",
                                            "direction": "down"}},
            {"name": "lift", "arguments": {"object": "charger",
                                           "side": "right",
                                           "height_m": 0.1}},
        ]

    def __call__(self, messages, tools):
        self.saw_tools.append({t["name"] for t in tools})
        if self.turn >= len(self.script):
            return {"name": None, "arguments": {}, "claimed": ""}
        call = self.script[self.turn]
        self.turn += 1
        return dict(call, claimed="", call_id=f"c{self.turn}")


def test_the_loop_starts_with_an_empty_scene_and_the_model_fills_it_in(
        loop_module, camera, d1_arm, tmp_path):
    """The zero-shot path end to end, with no scene number anywhere.

    Before this the loop planned the arm choice at turn zero and refused
    ``unreachable_task`` when the object was not in the scene — which is every
    run that has not measured the furniture first.
    """
    scene = _empty_scene()
    robot = _robot(loop_module, d1_arm, scene)
    model = _Declaring()
    trace = loop_module.loop(model, robot, task="put the charger in the cup",
                             max_turns=6, trace_path=tmp_path / "t.jsonl",
                             world0=_world(scene, d1_arm), kin=d1_arm,
                             obj="charger", destination="cup", camera=camera)
    assert trace.stop != "unreachable_task"
    names = [(r.choice or {}).get("name") for r in trace.records]
    assert names[:2] == ["locate", "declare_scene"]
    # both observation tools were offered on the FIRST turn, before anything
    # was declared
    assert {"declare_scene", "locate"} <= model.saw_tools[0]
    # the answers are correlated back with the call that asked
    first = trace.records[0].observation_after
    assert first["tool"] == "locate" and "m in base" in first["answer"]
    second = trace.records[1].observation_after
    assert "the right arm" in second["answer"] or "arm" in second["answer"]
    # ...and the motion verbs then ran against the declared scene
    assert "grasp" in names
    assert trace.records[2].verdict is not None


def test_a_model_that_stops_without_declaring_has_not_succeeded(loop_module,
                                                                camera,
                                                                d1_arm):
    """There is no goal verifier to ask before the things exist, and 'I
    stopped before saying where anything was' is not a measured success."""
    class Quits:
        def __call__(self, messages, tools):
            return {"name": None, "arguments": {}, "claimed": "done"}

    scene = _empty_scene()
    trace = loop_module.loop(Quits(), _robot(loop_module, d1_arm, scene),
                             max_turns=3, world0=_world(scene, d1_arm),
                             kin=d1_arm, obj="charger", destination="cup",
                             camera=camera)
    assert trace.stop == "model_stopped"
    assert "without declaring" in trace.stop_detail
    assert trace.records[-1].goal_verdict is None


def test_a_motion_verb_before_anything_is_declared_is_answered_not_crashed_on(
        loop_module, camera, d1_arm):
    class Impatient:
        def __call__(self, messages, tools):
            return {"name": "grasp", "claimed": "", "call_id": "x",
                    "arguments": {"object": "charger", "side": "right"}}

    scene = _empty_scene()
    trace = loop_module.loop(Impatient(), _robot(loop_module, d1_arm, scene),
                             max_turns=2, world0=_world(scene, d1_arm),
                             kin=d1_arm, obj="charger", destination="cup",
                             camera=camera)
    assert len(trace.records) == 2
    assert all(r.verdict is None for r in trace.records)


def test_the_model_is_told_the_camera_and_both_tool_points(loop_module,
                                                           camera, d1_arm):
    """Its scale reference: the lens's own pose, and the robot's two hands at
    known base-frame positions in the same photograph."""
    seen = {}

    class Listening(_Declaring):
        def __call__(self, messages, tools):
            seen.setdefault("first", [str(m.get("content")) for m in messages])
            return super().__call__(messages, tools)

    scene = _empty_scene()
    loop_module.loop(Listening(), _robot(loop_module, d1_arm, scene),
                     max_turns=1, world0=_world(scene, d1_arm), kin=d1_arm,
                     obj="charger", destination="cup", camera=camera)
    text = "\n".join(seen["first"])
    assert "HEAD CAMERA" in text and "fx 606" in text
    assert "Lens at" in text
    assert "left arm: mode position tool at" in text
    assert "right arm: mode position tool at" in text
    assert "YOU ARE THE DETECTOR" in text
    assert "declare_scene" in text and "locate(u, v)" in text


def test_the_camera_travels_in_the_scene_file(loop_module, agent_examples,
                                              tmp_path):
    """Not as a second set of loop flags: a neck that has moved since the
    frame was taken is a different camera, and re-typing its angle on two
    command lines is how those two quietly stop matching."""
    import perceive
    frames = agent_examples.parents[1] / "tests" / "data" / "perceive"
    args = perceive.build_parser().parse_args(
        ["--image", str(frames / "frame_D_run2_start.jpg"),
         "--neck-pitch", "0.512", "--lift", "0.205"])
    scene = perceive.perceive(args)
    camera = loop_module.camera_from_scene(scene)
    assert camera is not None
    assert camera.neck_pitch == pytest.approx(0.512)
    assert camera.calibrated is False
    from manipulation_kit.perception import HeadCamera
    direct = HeadCamera.from_robot(width=640, height=480, neck_pitch=0.512)
    # the file rounds; a millimetre and a hundred-thousandth of a quaternion
    assert np.allclose(camera.p, direct.p, atol=1e-4)
    assert np.allclose(camera.r.as_quat(), direct.r.as_quat(), atol=1e-4)
    assert loop_module.camera_from_scene(None) is None
    assert loop_module.camera_from_scene({"objects": []}) is None


def test_perceive_writes_the_scene_beside_the_trace(loop_module,
                                                    agent_examples, tmp_path):
    frames = agent_examples.parents[1] / "tests" / "data" / "perceive"
    scene = loop_module.perceived_scene(
        str(frames / "frame_D_run2_start.jpg"),
        trace_path=tmp_path / "trace.jsonl", obj="charger", destination="cup",
        options="--neck-pitch 0.512 --lift 0.205")
    assert [o["name"] for o in scene["objects"]] == ["table"]
    assert json.loads((tmp_path / "scene_perceived.json").read_text()) == scene


def test_scene_and_perceive_are_mutually_exclusive(loop_module,
                                                   agent_examples):
    frames = agent_examples.parents[1] / "tests" / "data" / "perceive"
    with pytest.raises(SystemExit):
        loop_module.main(["--scene", str(frames / "frame_D_run2_start.jpg"),
                          "--perceive", str(frames / "frame_D_run2_start.jpg"),
                          "--dry-run"])


def test_a_missing_frame_is_a_message_not_a_traceback(loop_module):
    with pytest.raises(SystemExit) as caught:
        loop_module.perceived_scene("/no/such/frame.jpg", trace_path=None,
                                    obj="a", destination="b")
    assert "no such frame" in str(caught.value)


def test_snapshot_mode_needs_the_hook_and_the_trace(loop_module, monkeypatch):
    monkeypatch.delenv("ASTRA_SNAPSHOT_CMD", raising=False)
    with pytest.raises(SystemExit) as caught:
        loop_module.perceived_scene("snapshot", trace_path=None, obj="a",
                                    destination="b")
    assert "--trace" in str(caught.value)
