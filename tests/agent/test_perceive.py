"""``examples/agent/perceive.py``: one head frame -> a scene, no scene calib.

Five real frames of the d1-2 JP wagon are in ``tests/data/perceive``, and the
numbers below are not round: they are what a throw-away OpenCV script produced
on the night the loop's first hand-made scene was written (2026-09-22), plus
what Shu then measured with a tape. A rewrite that quietly moves a corner by
20 px or a cup by 4 cm still produces a tidy JSON file, so the corners, the
depths and the footprints are all pinned.

THE RULE THESE ENFORCE, more than any single number: the only calibration that
goes in is the ROBOT's — intrinsics and the camera pose from the neck joints.
``--table-width`` is optional and nothing else about the furniture is an input
at all. The default path is checked with no scene number whatsoever.

No network, no Isaac, no robot. The Astra box detector is exercised against a
canned reply through a fake client; the mask fallback runs for real and is now
kept mainly so the geometry has something to be tested against.
"""

from __future__ import annotations

import json
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

FRAMES = {
    "A": "frame_A_original.jpg",
    "B": "frame_B_moved.jpg",
    "C": "frame_C_rotated.jpg",
    "D": "frame_D_run2_start.jpg",
    "E": "frame_E_run2_turn2.jpg",
}

#: Far corners (image-left, image-right) in pixels, from ``wagon_plane.py``
#: run under ``cv2.fitLine(DIST_HUBER)`` on these exact frames. The rewrite
#: here uses a consensus fit in numpy instead, and has to agree with it.
REFERENCE_CORNERS = {
    "A": ((104.1, 190.1), (533.1, 184.4)),
    "B": ((96.2, 217.0), (542.8, 203.3)),
    "C": ((96.2, 217.0), (542.8, 203.5)),
    "D": ((123.7, 188.2), (549.9, 179.9)),
    "E": ((123.0, 188.6), (548.7, 179.5)),
}

#: The wagon top Shu measured with a tape: 0.40 m deep, 0.60 m wide.
TRUE_DEPTH_M = 0.40
TRUE_WIDTH_M = 0.60


@pytest.fixture
def perceive(agent_examples):
    pytest.importorskip("PIL", reason="perceive.py needs an image decoder")
    import perceive as module
    return module


@pytest.fixture
def frames(agent_examples):
    return agent_examples.parents[1] / "tests" / "data" / "perceive"


@pytest.fixture
def camera_module(agent_examples):
    import camera as module
    return module


def _fit(perceive, frames, key, **kwargs):
    image = perceive.load_image(frames / FRAMES[key])
    return image, perceive.fit_table_plane(image, table_width_m=TRUE_WIDTH_M,
                                           **kwargs)


def _camera(camera_module, plane, *, neck_pitch=0.512, **kwargs):
    height, width = plane.shape
    return camera_module.HeadCamera.from_robot(
        width=width, height=height, fx=plane.fx, neck_pitch=neck_pitch,
        **kwargs)


def _scene_of(perceive, camera_module, frames, key, *, table_width=None,
              table_z=None, objects=(), neck_pitch=0.512, **kwargs):
    """Perceive one frame the way the CLI does, in-process."""
    image, plane = _fit(perceive, frames, key)
    camera = _camera(camera_module, plane, neck_pitch=neck_pitch)
    if table_z is not None:
        z, source = float(table_z), "declared"
    elif table_width is not None:
        z = perceive.table_z_from_known_length(plane, camera, table_width)
        source = "known-length"
    else:
        z, source = camera_module.provisional_table_z(), "provisional"
    table = perceive.project_corners(plane, camera, z, source=source)
    detections = []
    if objects:
        detections = perceive.detect_objects_mask(image, plane, list(objects))
    return camera, plane, table, perceive.build_scene(camera, table,
                                                      detections, **kwargs)


# --------------------------------------------------------------------------- #
# the plane
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize("key", ["A", "B", "C", "D"])
def test_the_far_corners_land_where_opencv_put_them(perceive, frames, key):
    _, plane = _fit(perceive, frames, key)
    for got, want in zip(plane.far_px, REFERENCE_CORNERS[key]):
        assert np.hypot(got[0] - want[0], got[1] - want[1]) < 6.0, (
            f"frame {key}: far corner {np.round(got, 1)} vs {want}")


@pytest.mark.parametrize("key", ["A", "D"])
def test_the_near_edge_measures_the_table_depth(perceive, frames, key):
    """A and D show the whole top, so the depth is MEASURED, not assumed."""
    _, plane = _fit(perceive, frames, key)
    assert plane.depth_m is not None, "the near edge was not found"
    assert abs(plane.depth_m - TRUE_DEPTH_M) < 0.015, (
        f"frame {key}: {plane.depth_m*1000:.0f} mm deep, tape says "
        f"{TRUE_DEPTH_M*1000:.0f}")


@pytest.mark.parametrize("key", ["B", "C"])
def test_a_clipped_near_edge_is_not_measured(perceive, frames, key):
    """The wagon runs out of the bottom of these two. A depth invented from
    the image border would look exactly like a measurement."""
    _, plane = _fit(perceive, frames, key)
    assert plane.depth_m is None
    assert plane.near_px is None


def test_the_arm_in_view_frame_still_fits_the_same_table(perceive, frames):
    """Frame E is two turns into run2 with the right arm over the wagon.

    Either the fit survives it — same corners — or it REFUSES with
    ``occlusion``. What it must never do is return a plausible different
    plane, which is the failure ``manipulation_kit.world.frames`` exists to
    name.
    """
    try:
        _, plane = _fit(perceive, frames, "E")
    except perceive.PlaneFitError as exc:
        assert exc.reason == "occlusion"
        assert "occl" in str(exc).lower() or "standing on" in str(exc)
        return
    _, reference = _fit(perceive, frames, "D")
    for got, want in zip(plane.far_px, reference.far_px):
        assert np.hypot(got[0] - want[0], got[1] - want[1]) < 10.0


def _occlude_right_edge(image, *, rows, amplitude=0.0):
    """Paint a dark, RAGGED shape over the wagon's right-hand edge.

    Ragged on purpose: a rectangle would replace the table's straight edge
    with another straight edge and no edge-based check could tell, which is a
    real limit of this method and not something to pretend away. A forearm's
    silhouette is not straight.
    """
    blocked = np.array(image, copy=True)
    for y in range(*rows):
        x0 = int(470 + amplitude * math.sin(y / 7.0))
        blocked[y, x0:] = (40, 40, 45)
    return blocked


def test_part_of_a_side_edge_may_be_hidden_without_moving_the_fit(perceive,
                                                                  frames):
    """A forearm across part of the right edge. The consensus fit takes the
    line the rest of it agrees on, which is the whole reason it is a consensus
    fit and not a Huber fit (see ``fit_edge``).

    Note how little of this table's right edge there is to spend: it leaves
    the frame two-thirds of the way down, so only ~54 scan rows ever reach it
    and an arm over 80 image rows takes half of them — which is the next test.
    """
    image, plane = _fit(perceive, frames, "D")
    blocked = _occlude_right_edge(image, rows=(250, 292), amplitude=25.0)
    partial = perceive.fit_table_plane(blocked, table_width_m=TRUE_WIDTH_M)
    for got, want in zip(partial.far_px, plane.far_px):
        assert np.hypot(got[0] - want[0], got[1] - want[1]) < 6.0


def test_an_occluded_side_edge_is_refused_not_averaged(perceive, frames):
    """Most of the right edge hidden behind the same ragged shape. There is
    no longer a line to agree on, and a plane fitted to the occluder's outline
    would still evaluate and still be wrong by centimetres."""
    image, _ = _fit(perceive, frames, "D")
    blocked = _occlude_right_edge(image, rows=(200, 480), amplitude=25.0)
    with pytest.raises(perceive.PlaneFitError) as caught:
        perceive.fit_table_plane(blocked, table_width_m=TRUE_WIDTH_M)
    assert caught.value.reason == "occlusion"
    assert ("standing on" in str(caught.value)
            or "outline" in str(caught.value)
            or "straight line" in str(caught.value))


def test_a_frame_with_no_table_says_so(perceive):
    with pytest.raises(perceive.PlaneFitError) as caught:
        perceive.fit_table_plane(np.zeros((64, 64, 3), dtype=np.uint8))
    assert caught.value.reason == "no_table"




# --------------------------------------------------------------------------- #
# the camera model: pixels <-> base metres, from robot facts alone
# --------------------------------------------------------------------------- #

def test_a_pixel_round_trips_through_the_camera_model(camera_module):
    """The whole of ``locate``: ray, plane, point, and back to the pixel."""
    camera = camera_module.HeadCamera.from_robot(width=640, height=480,
                                                 neck_pitch=0.45)
    for u, v in ((320, 400), (120, 300), (560, 470), (240, 260)):
        located = camera.locate(u, v, plane_z=0.166)
        assert located.p[2] == pytest.approx(0.166)
        back = camera.project(located.p)
        assert back[0] == pytest.approx(u, abs=1e-6)
        assert back[1] == pytest.approx(v, abs=1e-6)


def test_the_camera_pose_is_the_kits_head_camera_frame(camera_module):
    """No second copy of the geometry: it is the URDF's, through the kit."""
    from manipulation_kit.description.head_camera import head_camera_pose
    camera = camera_module.HeadCamera.from_robot(width=640, height=480,
                                                 neck_pitch=0.3, neck_yaw=0.1)
    p, r = head_camera_pose(neck_pitch=0.3, neck_yaw=0.1)
    assert np.allclose(camera.p, p)
    assert np.allclose(camera.r.as_quat(), r.as_quat())
    assert camera.calibrated is False


def test_the_neck_state_sign_is_flipped_once(camera_module):
    a = camera_module.HeadCamera.from_neck_state({"pitch": -0.3, "yaw": 0.1},
                                                 width=64, height=48)
    b = camera_module.HeadCamera.from_robot(width=64, height=48,
                                            neck_pitch=0.3, neck_yaw=0.1)
    assert np.allclose(a.p, b.p) and np.allclose(a.r.as_quat(), b.r.as_quat())


def test_looking_down_moves_the_intersection_toward_the_robot(camera_module):
    near = camera_module.HeadCamera.from_robot(width=640, height=480,
                                               neck_pitch=0.60)
    far = camera_module.HeadCamera.from_robot(width=640, height=480,
                                              neck_pitch=0.10)
    assert (near.locate(320, 240, plane_z=0.166).p[0]
            < far.locate(320, 240, plane_z=0.166).p[0])


def test_a_pixel_above_the_horizon_is_refused_not_extrapolated(camera_module):
    """A ray that climbs never meets the table, and the intersection behind
    the camera is a perfectly plausible-looking number."""
    camera = camera_module.HeadCamera.from_robot(width=640, height=480,
                                                 neck_pitch=0.0)
    with pytest.raises(camera_module.NotOnThePlane):
        camera.locate(320, 0, plane_z=0.166)


def test_the_uncertainty_grows_toward_the_horizon(camera_module):
    """It is PROPAGATED from the mount's documented slop, not quoted. A flat
    number would have hidden that a pixel near the far edge is worth half as
    much as one under the robot's nose."""
    camera = camera_module.HeadCamera.from_robot(width=640, height=480,
                                                 neck_pitch=0.512)
    near = camera.locate(320, 460, plane_z=0.166)
    far = camera.locate(320, 200, plane_z=0.166)
    assert 0.01 < near.uncertainty_m < far.uncertainty_m
    assert far.uncertainty_m > 1.5 * near.uncertainty_m
    assert "+-" in near.to_text() and "base" in near.to_text()
    # ...and with the neck level, the bottom of the frame is a grazing ray
    # and says so: half a metre of table for one pixel of aim.
    level = camera_module.HeadCamera.from_robot(width=640, height=480,
                                                neck_pitch=0.0)
    grazing = level.locate(320, 280, plane_z=0.166)
    assert grazing.grazing and grazing.uncertainty_m > 0.2
    assert "GRAZING" in grazing.to_text()


def test_the_lift_moves_the_floor_and_not_the_camera(camera_module):
    low = camera_module.HeadCamera.from_robot(width=64, height=48, lift_m=0.0)
    high = camera_module.HeadCamera.from_robot(width=64, height=48,
                                               lift_m=0.30)
    assert np.allclose(low.p, high.p)
    assert high.floor_z() == pytest.approx(low.floor_z() - 0.30)
    assert camera_module.HeadCamera.from_robot(width=64,
                                               height=48).floor_z() is None


def test_intrinsics_are_read_in_the_layouts_that_turn_up(camera_module,
                                                         tmp_path):
    direct = tmp_path / "a.json"
    direct.write_text(json.dumps({"fx": 612.5, "ppx": 317.0, "ppy": 241.0}))
    assert camera_module.read_intrinsics(direct) == pytest.approx(
        {"fx": 612.5, "cx": 317.0, "cy": 241.0})
    ros = tmp_path / "b.json"
    ros.write_text(json.dumps({"camera_matrix": {"data": [
        608.0, 0, 319.0, 0, 608.0, 239.0, 0, 0, 1]}}))
    assert camera_module.read_intrinsics(ros)["fx"] == pytest.approx(608.0)
    assert camera_module.read_intrinsics(ros)["cx"] == pytest.approx(319.0)
    empty = tmp_path / "c.json"
    empty.write_text("{}")
    with pytest.raises(ValueError):
        camera_module.read_intrinsics(empty)


# --------------------------------------------------------------------------- #
# the geometry, against a camera whose answer is known exactly
# --------------------------------------------------------------------------- #

def _synthetic(perceive, *, fx=600.0, shape=(480, 640), width_m=0.6,
               pitch_deg=25.0, height_m=0.55):
    """A camera looking down at a rectangle, and the pixels it would see.

    Built forwards — place the table, project it — so that recovering it
    backwards is a real test and not a rearrangement of the same expression.
    """
    height, width = shape
    K = np.array([[fx, 0.0, width / 2.0], [0.0, fx, height / 2.0],
                  [0.0, 0.0, 1.0]])
    tilt = R.from_euler("x", pitch_deg, degrees=True)
    across = tilt.apply([1.0, 0.0, 0.0])          # far edge, image-left -> right
    away = tilt.apply([0.0, 0.0, 1.0])            # side edges, away from camera
    up = np.cross(across, away)                   # out of the table
    up = up / np.linalg.norm(up)
    far_left = (tilt.apply([0.0, height_m, 0.0]) - across * (width_m / 2.0)
                + away * 0.85)

    def project(point):
        pixel = K @ np.asarray(point, dtype=float)
        return pixel[:2] / pixel[2]

    return dict(K=K, fx=fx, shape=shape, width_m=width_m, across=across,
                away=away, up=up, far_left=far_left, project=project,
                c1=project(far_left),
                c2=project(far_left + across * width_m),
                vanish=project(away * 1e6))


def test_the_metric_solve_recovers_a_camera_it_was_given(perceive):
    """Three pixel observations plus one known length -> the plane, to 1 mm.

    This is the OPTIONAL path (``--table-width``); it is kept and tested
    because the closed-form height solve is derived from it.
    """
    truth = _synthetic(perceive)
    plane = perceive.solve_plane(truth["c1"], truth["c2"], truth["vanish"],
                                 fx=truth["fx"], shape=truth["shape"],
                                 table_width_m=truth["width_m"])
    assert np.allclose(plane.P1, truth["far_left"], atol=1e-3)
    assert abs(float(np.linalg.norm(plane.E)) - truth["width_m"]) < 1e-9
    assert float(np.linalg.norm(plane.D - truth["away"])) < 1e-6
    assert abs(abs(float(plane.up @ truth["up"])) - 1.0) < 1e-6


def test_to_plane_round_trips_known_points_to_under_a_millimetre(perceive):
    truth = _synthetic(perceive)
    plane = perceive.solve_plane(truth["c1"], truth["c2"], truth["vanish"],
                                 fx=truth["fx"], shape=truth["shape"],
                                 table_width_m=truth["width_m"])
    for across_m in (0.0, 0.13, 0.31, 0.6):
        for depth_m in (0.0, 0.08, 0.27, 0.45):
            point = (truth["far_left"] + truth["across"] * across_m
                     + -truth["away"] * depth_m)
            u, v = truth["project"](point)
            got_across, got_depth = plane.to_plane(u, v)
            assert abs(got_across - across_m) < 1e-3
            assert abs(got_depth - depth_m) < 1e-3


def test_the_camera_distance_does_not_depend_on_the_camera_aim(perceive):
    """Why a known length fixes the height: rotating the camera cannot move
    the perpendicular distance from its centre to the table."""
    a = _synthetic(perceive, pitch_deg=25.0)
    b = _synthetic(perceive, pitch_deg=40.0)
    first = perceive.solve_plane(a["c1"], a["c2"], a["vanish"], fx=a["fx"],
                                 shape=a["shape"], table_width_m=a["width_m"])
    second = perceive.solve_plane(b["c1"], b["c2"], b["vanish"], fx=b["fx"],
                                  shape=b["shape"], table_width_m=b["width_m"])
    assert abs(first.camera_distance_m - second.camera_distance_m) < 1e-6


def test_a_height_above_a_base_point_round_trips(perceive, camera_module):
    """A cup's rim: a point 100 mm above a point on the plane, recovered."""
    camera = camera_module.HeadCamera.from_robot(width=640, height=480,
                                                 neck_pitch=0.45)
    base = camera.locate(360, 380, plane_z=0.150).p
    for lift in (0.02, 0.10, 0.25):
        u, v = camera.project(base + np.array([0.0, 0.0, lift]))
        assert perceive.height_above(camera, base, u, v) == pytest.approx(
            lift, abs=1e-6)


# --------------------------------------------------------------------------- #
# the height: the one thing one camera cannot measure
# --------------------------------------------------------------------------- #

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


def test_a_known_length_fixes_the_height_in_closed_form(perceive,
                                                        camera_module, frames):
    """``--table-width``: the one optional scene number, and what it buys.

    Shu's tape says the wagon top is 0.166 m above ``base``. The nominal head
    mount puts it ~14 mm low, consistently, on every frame — that is the lens
    position inside a 90 mm housing, and it is a BIAS, not noise.
    """
    image, plane = _fit(perceive, frames, "D")
    camera = _camera(camera_module, plane)
    z = perceive.table_z_from_known_length(plane, camera, TRUE_WIDTH_M)
    assert abs(z - 0.166) < 0.03, z
    table = perceive.project_corners(plane, camera, z, source="known-length")
    depth, width, yaw = table.extent()
    assert width == pytest.approx(TRUE_WIDTH_M, abs=0.002)
    assert depth == pytest.approx(TRUE_DEPTH_M, abs=0.015)
    assert abs(yaw) < math.radians(5.0)


@pytest.mark.parametrize("key", ["A", "B", "C", "D", "E"])
def test_the_known_length_height_is_the_same_on_every_frame(perceive,
                                                            camera_module,
                                                            frames, key):
    image, plane = _fit(perceive, frames, key)
    pitch = perceive.neck_pitch_that_levels(plane)
    camera = _camera(camera_module, plane, neck_pitch=pitch)
    z = perceive.table_z_from_known_length(plane, camera, TRUE_WIDTH_M)
    assert 0.14 < z < 0.17, f"frame {key}: {z}"


def test_the_whole_scene_scales_with_the_height_and_only_with_it(
        perceive, camera_module, frames):
    """The consequence of "one camera cannot measure the plane's height",
    made explicit: get it wrong by 10 % and everything is wrong by 10 %,
    TOGETHER — which is why one declared number from the model fixes it."""
    image, plane = _fit(perceive, frames, "D")
    camera = _camera(camera_module, plane)
    cam_z = float(camera.p[2])
    a = perceive.project_corners(plane, camera, 0.166).extent()
    b = perceive.project_corners(plane, camera, 0.066).extent()
    ratio = (cam_z - 0.066) / (cam_z - 0.166)
    assert b[0] / a[0] == pytest.approx(ratio, rel=1e-6)
    assert b[1] / a[1] == pytest.approx(ratio, rel=1e-6)


def test_the_level_correction_is_a_scale_free_diagnostic(perceive,
                                                         camera_module,
                                                         frames):
    """It needs no length at all, and it is what tells you the neck angle you
    passed is wrong — including its sign."""
    image, plane = _fit(perceive, frames, "D")
    right = _camera(camera_module, plane, neck_pitch=0.512)
    assert abs(perceive.level_correction_deg(plane, right)) < 1.0
    wrong_sign = _camera(camera_module, plane, neck_pitch=-0.512)
    assert abs(perceive.level_correction_deg(plane, wrong_sign)) > 40.0
    assert perceive.neck_pitch_that_levels(plane) == pytest.approx(0.512,
                                                                   abs=0.02)


def test_a_declared_height_is_used_as_given(perceive, frames):
    args = perceive.build_parser().parse_args(
        ["--image", str(frames / FRAMES["D"]), "--neck-pitch", "0.512",
         "--table-z", "0.166"])
    scene = perceive.perceive(args)
    assert scene["_perceive"]["table"]["height_source"] == "declared"
    assert scene["objects"][0]["measurement"]["top_z_base_m"] == pytest.approx(
        0.166, abs=1e-6)


# --------------------------------------------------------------------------- #
# the mask detector: no longer the point, still the thing the geometry is
# checked against with no network
# --------------------------------------------------------------------------- #

#: What Shu used for run2, from frame D, quoted to +-20 mm. The scene there
#: was pinned by a tape on the far edge; here only the height is declared and
#: the rest comes from the robot's own camera pose, so the agreement is the
#: measurement of BOTH.
FRAME_D_BASE = {"charger": (0.416, -0.207), "cup": (0.517, -0.165)}

MASK_OBJECTS = ({"name": "charger", "kind": "object", "colour": "white"},
                {"name": "cup", "kind": "container", "colour": "brown"})


def test_the_mask_detector_finds_both_things_on_frame_d(perceive,
                                                        camera_module, frames):
    camera, plane, table, scene = _scene_of(
        perceive, camera_module, frames, "D", table_z=0.166,
        objects=MASK_OBJECTS)
    by_name = {o["name"]: o for o in scene["objects"]}
    assert set(by_name) == {"table", "charger", "cup"}
    for name, (x, y) in FRAME_D_BASE.items():
        p = by_name[name]["p"]
        assert abs(p[0] - x) < 0.02, f"{name} x {p[0]}"
        assert abs(p[1] - y) < 0.02, f"{name} y {p[1]}"


def test_the_detector_searches_the_table_and_not_the_room(perceive, frames):
    """There is a white printer and a white wall socket behind the wagon; the
    script this came from fitted its colour windows around them by hand."""
    image, plane = _fit(perceive, frames, "D")
    detection = perceive.detect_objects_mask(image, plane, [MASK_OBJECTS[0]])[0]
    x0, y0, x1, y1 = detection.bbox
    assert perceive._table_polygon_mask(plane)[int(y1) - 2,
                                               int((x0 + x1) / 2)]
    assert y0 > plane.far_px[0][1], "the blob is above the far edge"


def test_a_cup_is_measured_at_its_rim_and_a_box_at_its_footprint(
        perceive, camera_module, frames):
    """The two shapes are measured differently ON PURPOSE (see ``measure``):
    a cylinder's silhouette is its diameter, a box's is wider than it is."""
    _, _, _, scene = _scene_of(perceive, camera_module, frames, "D",
                               table_z=0.166, objects=MASK_OBJECTS)
    by_name = {o["name"]: o for o in scene["objects"]}
    cup, charger = by_name["cup"], by_name["charger"]
    # a 110 mm paper cup, 90 mm across the rim; a charger about 45 mm square
    assert 0.085 < cup["size"][0] < 0.115
    assert 0.09 < cup["size"][2] < 0.125
    assert 0.04 < charger["size"][0] < 0.06
    assert cup["measurement"]["silhouette_width_m"] >= cup["size"][0]
    assert charger["size"][0] < charger["measurement"]["silhouette_width_m"]


def test_a_provisional_height_drags_every_confidence_down_with_it(
        perceive, camera_module, frames):
    """The honest consequence of not knowing the scale, in the one field a
    model reads: everything measured against a guessed plane is a guess."""
    _, _, _, guessed = _scene_of(perceive, camera_module, frames, "D",
                                 objects=MASK_OBJECTS)
    _, _, _, known = _scene_of(perceive, camera_module, frames, "D",
                               table_z=0.166, objects=MASK_OBJECTS)
    for item in guessed["objects"]:
        assert item["confidence"] <= 0.2, item["name"]
        assert item["measurement"].get("table_height_source",
                                       "provisional") == "provisional"
    assert max(o["confidence"] for o in known["objects"]) > 0.2


def test_an_unknown_colour_is_refused_rather_than_guessed(perceive, frames):
    image, plane = _fit(perceive, frames, "D")
    with pytest.raises(SystemExit):
        perceive.detect_objects_mask(image, plane, [
            {"name": "thing", "kind": "object", "colour": "chartreuse"}])
    with pytest.raises(SystemExit):
        perceive.detect_objects_mask(image, plane, [
            {"name": "thing", "kind": "object", "colour": None}])


# --------------------------------------------------------------------------- #
# the scene file, and the loop that has to consume it
# --------------------------------------------------------------------------- #

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


def test_a_declared_size_and_interior_say_they_were_declared(perceive,
                                                             camera_module,
                                                             frames):
    image, plane = _fit(perceive, frames, "D")
    camera = _camera(camera_module, plane)
    table = perceive.project_corners(plane, camera, 0.166, source="declared")
    detections = perceive.detect_objects_mask(image, plane, list(MASK_OBJECTS))
    scene = perceive.build_scene(camera, table, detections,
                                 sizes={"charger": [0.045, 0.02, 0.05]},
                                 interiors={"cup": [0.08, 0.08, 0.10]})
    by_name = {o["name"]: o for o in scene["objects"]}
    assert by_name["charger"]["size"] == [0.045, 0.02, 0.05]
    assert by_name["charger"]["measurement"]["size_y_measured"] is True
    assert 0.045 < by_name["charger"]["measurement"]["size_measured_m"][0] < 0.06
    assert by_name["cup"]["interior_measured"] is True


def test_every_perceived_number_carries_its_uncertainty(perceive,
                                                        camera_module, frames):
    _, _, _, scene = _scene_of(perceive, camera_module, frames, "D",
                               table_z=0.166, objects=MASK_OBJECTS)
    for item in scene["objects"]:
        assert item["confidence"] < 1.0, item["name"]
        assert item["measurement"], item["name"]
    cup = next(o for o in scene["objects"] if o["name"] == "cup")
    assert cup["measurement"]["interior_measured"] is False
    assert cup["measurement"]["size_y_measured"] is False
    assert cup["measurement"]["yaw_measured"] is False
    assert cup["yaw_rad"] == 0.0
    assert scene["_perceive"]["camera"]["calibrated"] is False


def test_an_unmeasured_depth_is_declared_rather_than_defaulted(perceive,
                                                               camera_module,
                                                               frames):
    """Frame B's near edge is out of shot, so the table's depth is the image
    border's and the file has to say which."""
    _, _, table, scene = _scene_of(perceive, camera_module, frames, "B",
                                   table_z=0.166)
    assert table.depth_measured is False
    assert any("near edge is outside the frame" in note
               for note in scene["objects"][0]["measurement"]["notes"])
    _, _, deep, _ = _scene_of(perceive, camera_module, frames, "D",
                              table_z=0.166)
    assert deep.depth_measured is True


# --------------------------------------------------------------------------- #
# the Astra box detector: kept for comparison, exercised with no network
# --------------------------------------------------------------------------- #

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


def test_the_astra_detector_sends_the_frame_and_parses_the_reply(perceive,
                                                                 frames):
    image = perceive.load_image(frames / FRAMES["D"])
    client = _FakeClient(GOOD_REPLY)
    found = {d.name: d for d in perceive.detect_objects(
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


def test_a_fenced_or_chatty_reply_is_still_read(perceive, frames):
    image = perceive.load_image(frames / FRAMES["D"])
    chatty = "Sure! Here is the JSON:\n```json\n" + GOOD_REPLY + "\n```\n"
    found = perceive.detect_objects(image, BOTH, client=_FakeClient(chatty))
    assert {d.name for d in found} == {"cup", "charger"}


def test_the_detector_retries_once_and_then_stops(perceive, frames):
    image = perceive.load_image(frames / FRAMES["D"])
    client = _FakeClient("not json at all", GOOD_REPLY)
    found = perceive.detect_objects(image, BOTH, client=client)
    assert len(client.calls) == 2
    assert {d.name for d in found} == {"cup", "charger"}
    # the failure is FED BACK, not silently retried with the same prompt
    assert any("not usable" in str(m.get("content", ""))
               for m in client.calls[1]["input"])

    hopeless = _FakeClient("still not json")
    with pytest.raises(ValueError):
        perceive.detect_objects(image, BOTH[:1], client=hopeless)
    assert len(hopeless.calls) == 2


def test_a_reply_that_omits_a_requested_object_is_an_error(perceive, frames):
    image = perceive.load_image(frames / FRAMES["D"])
    partial = json.dumps({"objects": [
        {"name": "cup", "bbox": [440, 245, 535, 350], "base_px": [487, 349]}]})
    with pytest.raises(ValueError) as caught:
        perceive.detect_objects(image, BOTH, client=_FakeClient(partial))
    assert "charger" in str(caught.value)


def test_a_bbox_that_is_not_four_numbers_is_not_recovered_from(perceive):
    with pytest.raises(ValueError):
        perceive._parse_astra(json.dumps({"objects": [
            {"name": "cup", "bbox": [1, 2, 3]}]}), BOTH[:1])


def test_an_astra_detection_measures_through_the_same_geometry(perceive,
                                                               camera_module,
                                                               frames):
    """A bbox with no contact band still produces a scene object — measured a
    little worse, and saying so."""
    image, plane = _fit(perceive, frames, "D")
    camera = _camera(camera_module, plane)
    table = perceive.project_corners(plane, camera, 0.166, source="declared")
    found = perceive.detect_objects(image, BOTH[:1],
                                    client=_FakeClient(GOOD_REPLY))
    item = perceive.measure(found[0], camera, table)
    assert item["measurement"]["contact_band_px"] is None
    # ...and measured a little worse: a bbox's bottom corners are a cup's rim
    # and its far side, so the footprint it implies is too wide and the centre
    # it implies drifts. 40 mm here against the mask detector's 17 mm.
    assert abs(item["p"][0] - 0.517) < 0.04
    assert abs(item["p"][1] + 0.165) < 0.04


# --------------------------------------------------------------------------- #
# the CLI
# --------------------------------------------------------------------------- #

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
