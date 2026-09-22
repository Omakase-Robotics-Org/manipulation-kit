"""The example side of perception: the ``perceive.py`` CLI, the Astra
detector (``detector.py``) and the scene file through the loop's own reader.

The numerics these call are ``manipulation_kit.perception``'s and are pinned
in ``tests/perception/test_perceive.py``; what is pinned HERE is the wiring —
the default path takes no scene number, the flags reach the kit, the Astra
reply is parsed strictly, and a perceived scene loads through ``live.py``.
Moved out of the old ``tests/agent/test_perceive.py`` with the code split.
"""

from __future__ import annotations

import json

import numpy as np
import pytest

from manipulation_kit.perception.measure import build_scene, measure
from manipulation_kit.perception.plane import project_corners

FRAMES = {
    "A": "frame_A_original.jpg",
    "B": "frame_B_moved.jpg",
    "C": "frame_C_rotated.jpg",
    "D": "frame_D_run2_start.jpg",
    "E": "frame_E_run2_turn2.jpg",
}
TRUE_WIDTH_M = 0.60

MASK_OBJECTS = ({"name": "charger", "kind": "object", "colour": "white"},
                {"name": "cup", "kind": "container", "colour": "brown"})


@pytest.fixture
def perceive(agent_examples):
    pytest.importorskip("PIL", reason="perceive.py needs an image decoder")
    import perceive as module
    return module


@pytest.fixture
def detector(agent_examples):
    import detector as module
    return module


@pytest.fixture
def frames(agent_examples):
    return agent_examples.parents[1] / "tests" / "data" / "perceive"


@pytest.fixture
def camera_module():
    from manipulation_kit.perception import head
    return head


def _fit(perceive, frames, key):
    from manipulation_kit.perception import fit_table_plane
    image = perceive.load_image(frames / FRAMES[key])
    return image, fit_table_plane(image, table_width_m=TRUE_WIDTH_M)


def _camera(camera_module, plane, *, neck_pitch=0.512):
    height, width = plane.shape
    return camera_module.HeadCamera.from_robot(
        width=width, height=height, fx=plane.fx, neck_pitch=neck_pitch)


def _scene_of(perceive, camera_module, frames, key, *, table_z=None,
              objects=()):
    """A frame through the KIT's pieces, the way ``perceive_frame`` does."""
    from manipulation_kit.perception import detect_objects_mask, table_in_base
    image, plane = _fit(perceive, frames, key)
    camera = _camera(camera_module, plane)
    table = table_in_base(plane, camera, table_z=table_z)
    detections = (detect_objects_mask(image, plane, list(objects))
                  if objects else [])
    return camera, plane, table, build_scene(camera, table, detections)


def test_the_default_path_takes_no_scene_number_at_all(perceive,
                                                       camera_module, frames):
    """The rule, as a test. Nothing about the furniture goes in."""
    args = perceive.build_parser().parse_args(
        ["--image", str(frames / FRAMES["D"]), "--neck-pitch", "0.512"])
    assert args.table_width is None and args.table_z is None
    assert args.detector == "model"
    scene = perceive.perceive(args)
    assert scene["_perceive"]["table"]["height_source"] == "provisional"
    assert scene["_perceive"]["camera"]["calibrated"] is False
    # the table IS there, in the right shape, at a height nobody measured
    table = scene["objects"][0]
    assert table["kind"] == "surface"
    assert table["confidence"] <= 0.2
    assert "PROVISIONAL" in " ".join(table["measurement"]["notes"])
    # ...and nothing was detected: the model does that
    assert len(scene["objects"]) == 1


def test_a_declared_height_is_used_as_given(perceive, frames):
    args = perceive.build_parser().parse_args(
        ["--image", str(frames / FRAMES["D"]), "--neck-pitch", "0.512",
         "--table-z", "0.166"])
    scene = perceive.perceive(args)
    assert scene["_perceive"]["table"]["height_source"] == "declared"
    assert scene["objects"][0]["measurement"]["top_z_base_m"] == pytest.approx(
        0.166, abs=1e-6)


def _world_from(scene, kin):
    import dataclasses
    import time

    from live import frames_from, objects_from
    from scene import demo_scene
    world, _ = demo_scene()
    return dataclasses.replace(world, objects=tuple(objects_from(scene)),
                               frames=frames_from(scene, now=time.time()))


def test_the_scene_loads_through_the_loops_own_reader(perceive, camera_module,
                                                      frames, tmp_path,
                                                      d1_arm):
    from live import load_scene, objects_from
    _, _, _, scene = _scene_of(perceive, camera_module, frames, "D",
                               table_z=0.166, objects=MASK_OBJECTS)
    path = tmp_path / "perceived.json"
    path.write_text(json.dumps(scene, indent=1), encoding="utf-8")
    objects = objects_from(load_scene(path))
    assert {o.name for o in objects} == {"table", "charger", "cup"}
    cup = next(o for o in objects if o.name == "cup")
    # A PERCEIVED interior is a guess, and the flag the kit reads has to say
    # so — `Place` refuses to drop into a guessed interior.
    assert cup.interior_measured is False
    # ...and the confidence SURVIVES the file, into the text a model reads.
    assert cup.confidence < 1.0
    assert "confidence" in cup.to_text()


def test_a_low_confidence_object_still_plans_an_approach_and_a_grasp(
        perceive, camera_module, frames, d1_arm):
    """Checked against the kit rather than assumed: NOTHING in
    ``manipulation_kit`` gates on ``ObjectView.confidence`` (it is carried and
    reported and never compared), so a declaration the model is only 30 %
    sure of is plannable, which is what makes declare-then-nudge possible.
    If that ever changes, this fails and the relaxation has to be argued for.
    """
    import dataclasses

    from manipulation_kit.primitives import Approach, Grasp
    from manipulation_kit.primitives.offer import check
    _, _, _, scene = _scene_of(perceive, camera_module, frames, "D",
                               table_z=0.166, objects=MASK_OBJECTS)
    for item in scene["objects"]:
        item["confidence"] = 0.3
        if item["name"] == "charger":
            # the tape's size, so this test is about CONFIDENCE and not about
            # the two extents a single view cannot see (tested separately)
            item["size"] = [0.045, 0.02, 0.05]
    world = _world_from(scene, d1_arm)
    assert all(o.confidence == pytest.approx(0.3) for o in world.objects)
    for verb in (Approach(object="charger", side="right"),
                 Grasp(object="charger", side="right")):
        plan = check(verb, world, d1_arm)
        assert plan.ok, f"{verb.name()} refused at confidence 0.3: {plan}"
    assert dataclasses.is_dataclass(world.objects[0])


def test_the_measured_scene_picks_the_right_arm_and_says_what_stops_it(
        perceive, camera_module, frames, d1_arm):
    """The honest outcome, pinned.

    Frame D's charger MEASURES 50 mm across its footprint and the driven jaws
    take 44 mm, so the chain Shu's hand-made scene planned does not plan off
    the measurement — his file declared the charger 20 mm across y, which is a
    number a single view cannot produce. The right arm is still the one
    chosen, and the refusal is ``object_too_wide`` with both numbers in it.
    """
    from manipulation_kit.primitives.reach import choose_side
    _, _, _, scene = _scene_of(perceive, camera_module, frames, "D",
                               table_z=0.166, objects=MASK_OBJECTS)
    world = _world_from(scene, d1_arm)
    choice = choose_side(world, d1_arm, obj="charger", destination="cup",
                         direction="down")
    assert choice.side == "right"
    assert not choice.reachable
    sentence = choice.chains["right"].sentence()
    assert "object_too_wide" in sentence
    assert "44 mm" in sentence


def test_the_two_escape_hatches_are_parsed_or_refused(perceive):
    got = perceive.parse_extents(["cup=0.08,0.08,0.10"], "--interior")
    assert got == {"cup": [0.08, 0.08, 0.10]}
    for bad in (["cup"], ["cup=0.08,0.08"], ["cup=a,b,c"], ["cup=0,1,1"]):
        with pytest.raises(SystemExit):
            perceive.parse_extents(bad, "--size")


class _FakeResponse:
    def __init__(self, text):
        self.output_text = text


class _FakeClient:
    """One ``responses.create`` that replays a list of canned bodies."""

    def __init__(self, *replies):
        self.replies = list(replies)
        self.calls = []
        self.responses = self

    def create(self, *, model, input):      # noqa: A002 - the API's own name
        self.calls.append({"model": model, "input": input})
        return _FakeResponse(self.replies[min(len(self.calls) - 1,
                                              len(self.replies) - 1)])


GOOD_REPLY = json.dumps({"objects": [
    {"name": "cup", "bbox": [440, 245, 535, 350], "base_px": [487, 349],
     "upright": True, "shape": "cylinder", "kind": "container",
     "confidence": 0.8},
    {"name": "charger", "bbox": [502, 387, 570, 438], "base_px": [536, 438],
     "upright": False, "shape": "box", "kind": "object", "confidence": 0.7}]})

BOTH = [{"name": "cup", "kind": "container", "colour": None},
        {"name": "charger", "kind": "object", "colour": None}]


def test_the_astra_detector_sends_the_frame_and_parses_the_reply(perceive, detector,
                                                                 frames):
    image = perceive.load_image(frames / FRAMES["D"])
    client = _FakeClient(GOOD_REPLY)
    found = {d.name: d for d in detector.detect_objects(
        image, BOTH, model="gpt-6-astra", client=client)}
    assert set(found) == {"cup", "charger"}
    assert found["cup"].shape == "cylinder" and found["cup"].kind == "container"
    assert found["charger"].upright is False
    assert found["cup"].base_px == (487.0, 349.0)
    assert found["cup"].source == "astra"

    sent = client.calls[0]
    assert sent["model"] == "gpt-6-astra"
    parts = sent["input"][0]["content"]
    assert parts[0]["type"] == "input_text"
    assert "base_px" in parts[0]["text"]
    assert "640" in parts[0]["text"] and "480" in parts[0]["text"]
    assert "cup, charger" in parts[0]["text"]
    assert parts[1]["type"] == "input_image"
    assert parts[1]["detail"] == "high"
    assert parts[1]["image_url"].startswith("data:image/jpeg;base64,")


def test_a_fenced_or_chatty_reply_is_still_read(perceive, detector, frames):
    image = perceive.load_image(frames / FRAMES["D"])
    chatty = "Sure! Here is the JSON:\n```json\n" + GOOD_REPLY + "\n```\n"
    found = detector.detect_objects(image, BOTH, client=_FakeClient(chatty))
    assert {d.name for d in found} == {"cup", "charger"}


def test_the_detector_retries_once_and_then_stops(perceive, detector, frames):
    image = perceive.load_image(frames / FRAMES["D"])
    client = _FakeClient("not json at all", GOOD_REPLY)
    found = detector.detect_objects(image, BOTH, client=client)
    assert len(client.calls) == 2
    assert {d.name for d in found} == {"cup", "charger"}
    # the failure is FED BACK, not silently retried with the same prompt
    assert any("not usable" in str(m.get("content", ""))
               for m in client.calls[1]["input"])

    hopeless = _FakeClient("still not json")
    with pytest.raises(ValueError):
        detector.detect_objects(image, BOTH[:1], client=hopeless)
    assert len(hopeless.calls) == 2


def test_a_reply_that_omits_a_requested_object_is_an_error(perceive, detector, frames):
    image = perceive.load_image(frames / FRAMES["D"])
    partial = json.dumps({"objects": [
        {"name": "cup", "bbox": [440, 245, 535, 350], "base_px": [487, 349]}]})
    with pytest.raises(ValueError) as caught:
        detector.detect_objects(image, BOTH, client=_FakeClient(partial))
    assert "charger" in str(caught.value)


def test_a_bbox_that_is_not_four_numbers_is_not_recovered_from(detector):
    with pytest.raises(ValueError):
        detector.parse_reply(json.dumps({"objects": [
            {"name": "cup", "bbox": [1, 2, 3]}]}), BOTH[:1])


def test_an_astra_detection_measures_through_the_same_geometry(perceive,
                                                               detector,
                                                               camera_module,
                                                               frames):
    """A bbox with no contact band still produces a scene object — measured a
    little worse, and saying so."""
    image, plane = _fit(perceive, frames, "D")
    camera = _camera(camera_module, plane)
    table = project_corners(plane, camera, 0.166, source="declared")
    found = detector.detect_objects(image, BOTH[:1],
                                    client=_FakeClient(GOOD_REPLY))
    item = measure(found[0], camera, table)
    assert item["measurement"]["contact_band_px"] is None
    # ...and measured a little worse: a bbox's bottom corners are a cup's rim
    # and its far side, so the footprint it implies is too wide and the centre
    # it implies drifts. 40 mm here against the mask detector's 17 mm.
    assert abs(item["p"][0] - 0.517) < 0.04
    assert abs(item["p"][1] + 0.165) < 0.04


def test_the_cli_writes_a_scene_and_a_debug_frame(perceive, frames, tmp_path,
                                                  capsys):
    out = tmp_path / "scenes" / "live.json"
    debug = tmp_path / "fit.png"
    code = perceive.main([
        "--image", str(frames / FRAMES["D"]), "--neck-pitch", "0.512",
        "--lift", "0.205", "--fx", "606",
        "--objects", "charger:object,cup:container", "--detector", "mask",
        "--table-z", "0.166", "--out", str(out), "--debug", str(debug)])
    assert code == 0
    assert debug.is_file() and debug.stat().st_size > 1000
    scene = json.loads(out.read_text(encoding="utf-8"))
    assert [o["name"] for o in scene["objects"]] == ["table", "charger", "cup"]
    assert scene["_perceive"]["camera"]["calibrated"] is False
    assert scene["_perceive"]["table_height_above_floor_m"] == pytest.approx(
        0.884, abs=0.005)
    printed = capsys.readouterr()
    assert json.loads(printed.out)["objects"]
    assert "table top z" in printed.err


def test_the_cli_reports_a_refused_fit_rather_than_a_traceback(perceive,
                                                               tmp_path):
    blank = tmp_path / "blank.png"
    perceive.write_png(blank, np.zeros((64, 64, 3), dtype=np.uint8))
    assert perceive.main(["--image", str(blank)]) == 2


def test_the_object_spec_takes_kinds_and_colours(perceive):
    got = perceive._requested("charger:object,cup:container:brown,plate")
    assert got[0] == {"name": "charger", "kind": "object", "colour": "white"}
    assert got[1] == {"name": "cup", "kind": "container", "colour": "brown"}
    assert got[2] == {"name": "plate", "kind": "object", "colour": None}
    with pytest.raises(SystemExit):
        perceive._requested("cup:saucer")


def test_the_default_detector_leaves_the_things_to_the_model(perceive,
                                                             frames, capsys):
    """``--detector model`` names them and finds none: the loop's own model
    is the detector, looking at this same frame."""
    args = perceive.build_parser().parse_args(
        ["--image", str(frames / FRAMES["D"]), "--neck-pitch", "0.512",
         "--objects", "charger:object,cup:container"])
    scene = perceive.perceive(args)
    assert [o["name"] for o in scene["objects"]] == ["table"]
    assert scene["_perceive"]["diagnostics"]["objects_left_to_the_model"] == [
        "charger", "cup"]

