"""The servo's v2 references and actions: the jaw opening drawn at the
object's depth, the object's real-shape outline, the typed readings, and the
fixed priority retreat -> turn -> xy -> approach."""

from __future__ import annotations

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from manipulation_kit.agent import KinematicMirror, OperatorPolicy, Servo
from manipulation_kit.agent import jaws as J
from manipulation_kit.agent.servo import (ALIGNED, BUDGET, Choice, Reading,
                                          ServoLook, accumulate, as_reading,
                                          choose, geometry_judge, photoless,
                                          turn_sign)
from manipulation_kit.perception import WristCamera
from manipulation_kit.world import FrameGraph, ObjectView

DEMO = {"fx": 320.0, "fy": 320.0, "cx": 320.0, "cy": 240.0, "width": 640,
        "height": 480}


def _down_camera(side="right", yaw_deg=0.0):
    """A wrist camera on a hand pointing straight down (approach -z)."""
    r = R.from_euler("z", yaw_deg, degrees=True) * R.from_euler("x", 180,
                                                                 degrees=True)
    return WristCamera.from_flange(side, np.array([0.4, 0.0, 0.40]), r, **DEMO)


# --------------------------------------------------------------------------- #
# geometry: the jaw opening
# --------------------------------------------------------------------------- #

def test_the_camera_carries_the_flange_it_rides_on():
    camera = _down_camera()
    assert np.allclose(camera.flange_p, [0.4, 0.0, 0.4])
    assert np.allclose(camera.flange_r.as_matrix()[:, 2], [0, 0, -1], atol=1e-9)


def test_the_jaw_opening_is_two_pads_around_the_approach_axis():
    camera = _down_camera()
    opening = J.jaw_opening(camera, gap_m=0.06, depth_m=0.25)
    axis_point = camera.flange_p + 0.25 * np.array([0.0, 0.0, -1.0])
    assert np.allclose(opening.point, axis_point)
    u, v = camera.project(axis_point)
    assert opening.centre == pytest.approx((u, v))
    # the two pads sit on either side of the centre along the jaw travel, and
    # the gap between their inner edges is the jaw gap at that depth
    x = camera.flange_r.as_matrix()[:, 0]
    inner = [camera.project(axis_point + s * 0.03 * x) for s in (1.0, -1.0)]
    width_px = math.dist(*inner)
    local_depth = float(camera.r.inv().apply(axis_point - camera.p)[2])
    assert width_px == pytest.approx(camera.fx * 0.06 / local_depth, rel=1e-6)
    pad_centres = [np.mean(p, axis=0) for p in opening.pads]
    along = [np.dot(np.subtract(c, opening.centre),
                    np.subtract(inner[0], opening.centre)) for c in pad_centres]
    assert along[0] > 0 > along[1]
    for pad in opening.pads:
        d = [math.dist(p, opening.centre) for p in pad]
        assert min(d) >= 0.5 * width_px - 1e-6
    assert opening.gap_m == 0.06 and opening.depth_m == 0.25


def test_the_jaw_opening_shift_and_gap_sources():
    camera = _down_camera()
    base = J.jaw_opening(camera, gap_m=0.05, depth_m=0.2)
    moved = J.jaw_opening(camera, gap_m=0.05, depth_m=0.2, shift_m=(0.01, 0.0))
    x = camera.flange_r.as_matrix()[:, 0]
    assert np.allclose(np.subtract(moved.point, base.point), 0.01 * x)
    from manipulation_kit.hands.d1.parallel_gripper.description import (
        DRIVEN_OPEN_GAP_M)
    from manipulation_kit.world import GripperView
    assert J.jaw_gap(None) == (DRIVEN_OPEN_GAP_M, "nominal")
    assert J.jaw_gap(GripperView("left", 0.0, open_gap_m=0.06)) == (
        0.06, "driven_open")
    assert J.jaw_gap(GripperView("left", 0.0, jaw_gap_m=0.041,
                                 open_gap_m=0.06)) == (0.041, "measured")
    # a camera without a flange pose draws no jaws
    bare = WristCamera(fx=320.0, cx=320.0, cy=240.0, width=640, height=480,
                       p=np.zeros(3), r=R.identity())
    assert J.jaw_opening(bare, gap_m=0.05, depth_m=0.2) is None
    assert J.approach_depth(bare, [0, 0, 1]) is None


def test_the_pad_width_is_the_composed_urdfs_jaw_box():
    from manipulation_kit.description.d1.tools import generate_d1_urdf as g
    from manipulation_kit.hands.d1.parallel_gripper.description import (
        PAD_WIDTH_M)
    lo, hi = g.GRIPPER_JAW_BOX["r"]
    assert PAD_WIDTH_M == pytest.approx(hi[1] - lo[1])


@pytest.mark.parametrize("side", ["left", "right"])
def test_a_positive_yaw_turns_the_jaws_the_way_turn_sign_says(side):
    """The camera rides the hand, so in the wrist photo the drawn jaws never
    turn — the OBJECT does, the other way. The judged turn is the jaws'
    turn RELATIVE TO THE OBJECT, clockwise in the photo; turn_sign maps it to
    the hand's yaw: a yaw of +10 deg x turn_sign turns the jaw travel 10 deg
    clockwise against a fixed line on the table."""
    from manipulation_kit.primitives.orientation import roll_tool
    camera = _down_camera(side)
    p = camera.flange_p + [0.0, 0.0, -0.25]
    line = (p - [0.02, 0.01, 0.0], p + [0.02, 0.01, 0.0])

    def relative(cam):
        a, b = (cam.project(x) for x in line)
        seen = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
        return J.jaw_opening(cam, gap_m=0.05, depth_m=0.25).jaw_angle_deg - seen
    dyaw = math.radians(10.0) * turn_sign(camera)
    turned = WristCamera.from_flange(side, camera.flange_p,
                                     roll_tool(camera.flange_r, dyaw), **DEMO)
    assert J.wrap_turn(relative(turned) - relative(camera)) == pytest.approx(
        10.0, abs=0.5)


# --------------------------------------------------------------------------- #
# geometry: the object's outline
# --------------------------------------------------------------------------- #

def test_declared_outlines_have_their_shapes():
    camera = _down_camera()
    p = camera.flange_p + [0.0, 0.0, -0.25]
    frames = FrameGraph()
    tape = ObjectView("tape", p=p, size=(0.05, 0.05, 0.026))
    ring = J.declared_outline(camera, tape, frames, "cylinder")
    assert ring.source == "declared:cylinder" and ring.aspect < 1.15
    assert not ring.elongated
    for yaw in (0.0, 30.0, -60.0):
        bar = ObjectView("bar", p=p, size=(0.12, 0.03, 0.03),
                         r=R.from_euler("z", yaw, degrees=True))
        box = J.declared_outline(camera, bar, frames, "box")
        assert box.elongated
        # the image long axis follows the declared yaw (seen from above,
        # through a camera turned pi about the approach axis)
        a, b = camera.project(p - 0.05 * bar.r.apply([1, 0, 0])), \
            camera.project(p + 0.05 * bar.r.apply([1, 0, 0]))
        expected = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
        assert J.wrap_turn(box.long_axis_deg - expected) == pytest.approx(0.0, abs=2.0)
    foot = J.declared_outline(camera, bar, frames, "other")
    assert foot.source == "declared:other" and foot.elongated
    with pytest.raises(ValueError):
        J.declared_outline(camera, tape, frames, "sphere")


def test_hull_axis_and_turn_helpers():
    square = [(0, 0), (10, 0), (10, 10), (0, 10), (5, 5)]
    assert len(J.convex_hull(square)) == 4
    angle, aspect = J.principal_axis([(0, 0), (40, 0), (40, 10), (0, 10)])
    assert angle == pytest.approx(0.0) and aspect == pytest.approx(4.0)
    angle, _ = J.principal_axis([(0, 0), (10, 10), (8, 12), (-2, 2)])
    assert angle == pytest.approx(45.0)
    assert J.wrap_turn(135.0) == -45.0 and J.wrap_turn(-90.0) == 90.0
    # a turn split between a quarter turn either way IS a quarter turn
    assert abs(J.axial_mean([-90, -45, 0, 45, 90], [0.5, 0, 0, 0, 0.5])) == 90.0
    assert J.axial_mean([-90, -45, 0, 45, 90], [0, 0.2, 0.6, 0.2, 0]) == \
        pytest.approx(0.0, abs=1e-9)
    assert J.axial_mean([-90, -45, 0, 45, 90], [0, 0, 0, 1, 0]) == \
        pytest.approx(45.0)


def test_the_drawing_marks_the_photo(tmp_path):
    from PIL import Image
    photo = tmp_path / "p.png"
    Image.new("RGB", (640, 480), (128, 128, 128)).save(photo)
    camera = _down_camera()
    opening = J.jaw_opening(camera, gap_m=0.06, depth_m=0.25)
    tape = ObjectView("tape", p=opening.point, size=(0.05, 0.05, 0.026))
    ring = J.declared_outline(camera, tape, FrameGraph(), "cylinder")
    out = J.draw(photo, tmp_path / "d.png", jaws=opening, outline=ring)
    image = np.asarray(Image.open(out))
    green = np.all(image == J.GREEN, axis=-1)
    magenta = np.all(image == J.MAGENTA, axis=-1)
    assert green.sum() > 50 and magenta.sum() > 50


# --------------------------------------------------------------------------- #
# the typed reading and the fixed priority
# --------------------------------------------------------------------------- #

def _look(elongated=False):
    return ServoLook(side="left", object="x", camera=None, u=0.0, v=0.0,
                     depth_m=0.2, image=None, photo=None, elongated=elongated)


def _r(direction, **kw):
    return as_reading(Reading(direction=direction, **kw))


def test_readings_normalise_and_accumulate():
    one = _r({"left": 2.0, "on": 2.0}, depth={"ahead": 3.0, "between": 1.0},
             occluded=0.2)
    assert one.direction["left"] == 0.5 and one.depth["ahead"] == 0.75
    with pytest.raises(ValueError):
        as_reading(Reading({"sideways": 1.0}))
    with pytest.raises(ValueError):
        as_reading(Reading({"on": 1.0}, depth={"under": 1.0}))
    two = _r({"left": 1.0}, occluded=0.6, turn_deg=30.0)
    acc = accumulate([one, two])
    assert acc.direction["left"] == pytest.approx(0.75)
    assert acc.occluded == pytest.approx(0.4)
    assert acc.turn_deg == pytest.approx(30.0)          # only one carried it
    assert acc.depth["ahead"] == pytest.approx(0.75)
    # bare distributions stay distributions (the v1 judges)
    assert accumulate([{"on": 1.0}, {"left": 1.0}])["on"] == 0.5


@pytest.mark.parametrize("reading,elongated,room,expected", [
    # occlusion first, whatever else is said
    (dict(direction={"left": 1.0}, occluded=0.9, turn_deg=45.0), True, 0.02,
     ("retreat", -1.0)),
    # a hidden-or-not near 0.5 is not evidence (occlusion_margin)
    (dict(direction={"left": 1.0}, occluded=0.7), False, 0.02, ("xy", 0.0)),
    # then the turn, for an elongated object, before any xy step
    (dict(direction={"left": 1.0}, occluded=0.1, turn_deg=45.0), True, 0.02,
     ("turn", 1.0)),
    (dict(direction={"left": 1.0}, turn_deg=-40.0), True, 0.02, ("turn", -1.0)),
    # not elongated: the turn answer is recorded, not acted on
    (dict(direction={"left": 1.0}, turn_deg=45.0), False, 0.02, ("xy", 0.0)),
    # a turn under half a step is not taken
    (dict(direction={"on": 1.0}, turn_deg=5.0), True, 0.0, ("on", 0.0)),
    # centred and still ahead: approach while there is room
    (dict(direction={"on": 1.0}, depth={"ahead": 1.0}), False, 0.02,
     ("approach", 1.0)),
    (dict(direction={"on": 1.0}, depth={"ahead": 1.0}), False, 0.0, ("on", 0.0)),
    (dict(direction={"on": 1.0}, depth={"behind": 1.0}), False, 0.0,
     ("approach", -1.0)),
    (dict(direction={"on": 1.0}, depth={"between": 1.0}), False, 0.02,
     ("on", 0.0)),
    # an xy step comes before the approach
    (dict(direction={"below": 1.0}, depth={"ahead": 1.0}), False, 0.02,
     ("xy", 0.0)),
])
def test_the_priority_is_retreat_turn_xy_approach(reading, elongated, room,
                                                  expected):
    chosen = choose(_r(**reading), 1, look=_look(elongated), window=3,
                    margin=0.1, approach_room_m=room)
    assert (chosen.kind, chosen.sign) == expected, chosen
    assert chosen.why


def test_the_turn_budget_hands_over_to_xy():
    chosen = choose(_r({"left": 1.0}, turn_deg=45.0), 1, look=_look(True),
                    window=3, margin=0.1, turned_rad=math.radians(90.0))
    assert chosen == Choice("xy", chosen.why, direction="left")


# --------------------------------------------------------------------------- #
# the loop, on the kinematic mirror
# --------------------------------------------------------------------------- #

def _mirror():
    from scene import DEMO_WRIST_CAMERA, demo_scene
    world, kin = demo_scene()
    return KinematicMirror(kin, world, wrist_intrinsics=DEMO_WRIST_CAMERA)


def _approached():
    from manipulation_kit.agent import run
    from manipulation_kit.primitives import Place
    robot = _mirror()

    class Script:
        calls = [("approach", {"object": "red_block", "side": "left",
                               "direction": "down"})]

        def __call__(self, messages, tools):
            if not self.calls:
                return {"name": None, "arguments": {}, "claimed": ""}
            name, arguments = self.calls.pop(0)
            return {"name": name, "arguments": arguments, "claimed": "",
                    "call_id": "c1"}
    run(goal=Place(object="red_block", to="box"), robot=robot,
        policy=OperatorPolicy(max_turns=1), ask=Script())
    return robot


def _align(robot, servo, policy=None):
    from manipulation_kit.agent.policy import PolicyState
    from manipulation_kit.executor import run as run_plan
    return servo.align(robot=robot, policy=policy or OperatorPolicy(),
                       state=PolicyState(), side="left", name="red_block",
                       settings={}, run_plan=run_plan)


def _tool(robot):
    arm = robot.world().arm("left")
    return np.asarray(arm.tool_p), arm.tool_r


def test_the_reference_is_the_jaw_opening_and_on_declares_the_jaw_point(
        agent_examples):
    """Declared 40 mm off: the look's reference is the jaw point (the hand),
    not the declaration; the servo steps the HAND onto the truth and "on"
    re-declares the object at the jaw point."""
    robot = _approached()
    declared = robot.world().find("red_block").p.copy()
    truth = declared + [0.0, 0.04, 0.0]
    report = _align(robot, Servo(lambda s: None, geometry_judge(
        {"red_block": truth}), log=lambda s: None))
    assert report.outcome == ALIGNED, report.detail
    first = report.steps[0].look
    assert first["jaws"] is not None and first["reference_p"] is not None
    assert first["jaws"]["gap_source"] in ("measured", "driven_open", "nominal")
    block = robot.world().find("red_block")
    assert block.provenance == "judged"
    assert float(np.hypot(*(block.p[:2] - truth[:2]))) <= 0.0101
    tool_p, _r = _tool(robot)
    assert float(np.hypot(*(tool_p[:2] - truth[:2]))) <= 0.0101
    assert all(s.action["kind"] == "xy" for s in report.steps if s.action)
    assert report.to_json()["actions"]["xy"] >= 1


def test_occlusion_backs_off_along_the_approach_axis_first(agent_examples):
    robot = _approached()
    p0, r0 = _tool(robot)
    truth = robot.world().find("red_block").p.copy()
    geometry = geometry_judge({"red_block": truth})
    asked = []

    def judge(look):
        asked.append(look)
        reading = geometry(look)
        return Reading(reading.direction, occluded=0.9 if len(asked) == 1
                       else 0.0)

    lines = []
    report = _align(robot, Servo(lambda s: None, photoless(judge),
                                 log=lines.append))
    assert report.outcome == ALIGNED
    moved = [s for s in report.steps if s.action]
    assert moved[0].action["kind"] == "retreat"
    assert moved[0].action["nudge"]["dz"] == pytest.approx(-0.03)
    assert "occluded 0.90" in moved[0].action["reason"]
    p1, _ = _tool(robot)
    back = r0.as_matrix()[:, 2] * -0.03
    assert np.allclose(p1 - p0, back, atol=2e-3)
    assert any("-> retreat -30mm along the approach axis" in line
               for line in lines)


def test_an_elongated_object_is_turned_before_the_xy_step(agent_examples):
    """A judge that says "turn 45 deg" for an elongated outline: yaw steps
    of 15 deg about the approach axis (the nudge's cap) until the turn is
    done, the object's declaration turned with the hand, then on."""
    robot = _approached()
    truth = robot.world().find("red_block").p.copy()
    geometry = geometry_judge({"red_block": truth})
    turned = []

    def judge(look):
        reading = geometry(look)
        left = 45.0 - 15.0 * len(turned)
        return Reading(reading.direction, turn_deg=left)

    def elongated(photo, look):
        return J.Outline(points=((0, 0), (40, 0), (40, 10), (0, 10)),
                         source="segmented:test", long_axis_deg=0.0,
                         aspect=4.0)

    servo = Servo(lambda s: None, photoless(judge), object_outline=elongated,
                  log=lambda s: None)
    original = servo._correct

    def counting(*args, **kwargs):
        out = original(*args, **kwargs)
        step = args[9]
        if step.action["kind"] == "turn":
            turned.append(step.action["nudge"]["dyaw"])
        return out
    servo._correct = counting
    r_before = robot.world().find("red_block").pose_in_base(
        robot.world().frames)[1]
    _p, tool_r0 = _tool(robot)
    report = _align(robot, servo,
                    policy=OperatorPolicy(max_nudges_per_target=5))
    assert report.outcome == ALIGNED, report.detail
    assert len(turned) == 3
    assert all(abs(abs(t) - math.radians(15.0)) < 1e-9 for t in turned)
    _p, tool_r1 = _tool(robot)
    z = tool_r0.as_matrix()[:, 2]
    rel = tool_r1 * tool_r0.inv()
    assert abs(rel.magnitude() - math.radians(45.0)) < math.radians(1.0)
    assert abs(abs(float(np.dot(rel.as_rotvec() / rel.magnitude(), z))) - 1.0) < 1e-3
    r_after = robot.world().find("red_block").pose_in_base(
        robot.world().frames)[1]
    assert (r_after * r_before.inv()).magnitude() == pytest.approx(
        math.radians(45.0), abs=1e-6)
    assert report.to_json()["actions"]["turn"] == 3


def test_the_approach_stops_above_the_object_and_its_support(agent_examples):
    """A depth answer of "ahead" brings the hand along the approach axis in
    fine steps, never past the servo's cap or the fingertip clearance above
    the object's top."""
    from manipulation_kit.hands.d1.parallel_gripper.description import (
        PAD_TIP_Z_M)
    robot = _approached()
    truth = robot.world().find("red_block").p.copy()
    p0, r0 = _tool(robot)
    servo = Servo(lambda s: None, geometry_judge({"red_block": truth},
                                                 depth=True),
                  max_approach_m=0.10, log=lambda s: None)
    report = _align(robot, servo, policy=OperatorPolicy(max_nudges_per_target=12))
    assert report.outcome == ALIGNED, report.detail
    kinds = [s.action["kind"] for s in report.steps if s.action]
    assert kinds and set(kinds) == {"approach"}
    p1, _ = _tool(robot)
    came = float(np.dot(p1 - p0, r0.as_matrix()[:, 2]))
    assert 0.0 < came <= 0.10 + 1e-3
    # the fingertips (tool point = pad centre; tip 29 mm further) stay the
    # clearance above the block's top
    from manipulation_kit.hands.d1.parallel_gripper.description import (
        PAD_CENTRE_Z_M)
    block = robot.world().find("red_block")
    tip_z = p1[2] - (PAD_TIP_Z_M - PAD_CENTRE_Z_M)
    top = block.p[2] + block.size[2] / 2.0
    assert tip_z >= top + servo.approach_clearance_m - 2e-3
    assert "no approach room" in report.detail or report.steps[-1].decision


def test_the_nudge_budget_counts_every_action_kind(agent_examples):
    robot = _approached()

    def always_hidden(look):
        return Reading({"on": 1.0}, occluded=1.0)

    report = _align(robot, Servo(lambda s: None, photoless(always_hidden),
                                 log=lambda s: None),
                    policy=OperatorPolicy(max_nudges_per_target=2))
    assert report.outcome == BUDGET
    assert report.to_json()["actions"]["retreat"] == 2


def test_a_missing_segmentation_falls_back_to_the_declared_shape(agent_examples):
    robot = _approached()
    looks = []

    def judge(look):
        looks.append(look)
        return {"on": 1.0}

    servo = Servo(lambda s: None, photoless(judge),
                  object_outline=lambda photo, look: None,
                  shapes={"red_block": "other"}, log=lambda s: None)
    assert _align(robot, servo).outcome == ALIGNED
    assert looks[0].outline.source == "declared:other"
    assert looks[0].outline.extra["fallback"] == "not found"
    assert looks[0].to_json()["outline"]["fallback"] == "not found"
    with pytest.raises(ValueError, match="shapes"):
        Servo(lambda s: None, judge, shapes={"red_block": "sphere"})


# --------------------------------------------------------------------------- #
# the examples: the v2 questions, the segmenter seam and its server
# --------------------------------------------------------------------------- #

def test_the_jaws_formulation_reads_a_typed_batch(agent_examples):
    from manipulation_kit.agent.judge import read
    from jev_questions import FORMULATIONS, questions
    items = questions("tape", "jaws")
    assert [q.id for q in items] == ["letters", "distance", "depth",
                                     "occluded", "jaw_turn", "fit"]
    probs = {"letters": [0.1, 0.6, 0.1, 0.1, 0.05, 0.05],
             "distance": [0.1, 0.2, 0.6, 0.1], "depth": [0.7, 0.2, 0.1],
             "occluded": [0.2, 0.8], "jaw_turn": [0.45, 0.05, 0.0, 0.05, 0.45],
             "fit": [0.9, 0.1]}
    answers = {q.id: read(q, probs[q.id]) for q in items}
    reading = FORMULATIONS["jaws"].to_reading(answers)
    assert max(reading.direction, key=reading.direction.get) == "right"
    assert reading.depth["ahead"] == pytest.approx(0.7)
    assert reading.occluded == pytest.approx(0.2)
    assert abs(reading.turn_deg) == pytest.approx(90.0)
    assert reading.fit == pytest.approx(0.9)
    assert reading.distance == pytest.approx(1.7)
    words = [q.id for q in questions("tape", "jaws_words")]
    assert words[0] == "where" and words[1:] == [q.id for q in items][1:]


@pytest.fixture
def segment_server():
    import threading
    import segment_server as S

    class Fake:
        name, device = "fake-sam", "cpu"

        def segment(self, image, names, threshold):
            assert image.size == (64, 48)
            return {n: ({"found": True, "score": 0.8,
                         "polygon": [[10, 10], [30, 10], [30, 20], [10, 20]],
                         "bbox": [10, 10, 30, 20], "area_px": 200}
                        if n == "roll of tape" else {"found": False})
                    for n in names}
    server = S.make_server(Fake(), "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield f"http://127.0.0.1:{server.server_address[1]}"
    server.shutdown()
    server.server_close()


def test_the_remote_segmenter_outlines_over_http(agent_examples, segment_server,
                                                 tmp_path):
    from PIL import Image
    from segmenter import RemoteSegmenter
    photo = tmp_path / "w.jpg"
    Image.new("RGB", (64, 48)).save(photo)
    seg = RemoteSegmenter(segment_server, prompts={"tape": "roll of tape"})
    assert seg.health()["model"] == "fake-sam"
    look = _look()
    outline = seg(photo, ServoLook(**{**look.__dict__, "object": "tape"}))
    assert outline.source == "segmented:fake-sam" and outline.score == 0.8
    assert len(outline.points) == 4 and outline.aspect == pytest.approx(2.0)
    assert seg(photo, look) is None                  # "x" is not found
    assert seg(None, look) is None
    assert RemoteSegmenter("http://127.0.0.1:9", timeout_s=0.5)(photo, look) is None


def test_the_segment_servers_rle_round_trips(agent_examples):
    import segment_server as S
    mask = np.zeros((5, 7), dtype=bool)
    mask[1:3, 2:6] = True
    mask[0, 0] = True
    rle = S.rle_encode(mask)
    assert rle["size"] == [5, 7]
    assert np.array_equal(S.rle_decode(rle), mask)
    assert S.rle_encode(np.zeros((2, 2), bool))["counts"] == [4]


def test_the_outline_flags_build_servo_options(agent_examples, tmp_path):
    import json
    import jev_servo
    from segmenter import RemoteSegmenter, outline_options
    scene = tmp_path / "scene.json"
    scene.write_text(json.dumps({"objects": [{"name": "tape",
                                              "shape": "cylinder"}]}))
    args = jev_servo.build_parser().parse_args(
        ["--scene", str(scene), "--object-shape", "bar=box",
         "--segment-url", "http://127.0.0.1:8767"])
    options = outline_options(args)
    assert options["shapes"] == {"tape": "cylinder", "bar": "box"}
    assert isinstance(options["object_outline"], RemoteSegmenter)
    plain = outline_options(jev_servo.build_parser().parse_args([]))
    assert plain == {"shapes": {}}
