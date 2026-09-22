"""``examples/agent/perceive.py``: one head frame -> a measured scene.

Five real frames of the d1-2 JP wagon are in ``tests/data/perceive`` and the
numbers below are not round: they are what a throw-away OpenCV script produced
on the night the loop's first hand-made scene was written (2026-09-22), plus
what Shu then measured with a tape. A rewrite that quietly moves a corner by
20 px or a cup by 4 cm still produces a tidy JSON file, so the corners, the
depths and the footprints are all pinned.

No network, no Isaac, no robot. The Astra detector is exercised against a
canned reply through a fake client; the mask fallback runs for real.
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


def _fit(perceive, frames, key, **kwargs):
    image = perceive.load_image(frames / FRAMES[key])
    return image, perceive.fit_table_plane(image, table_width_m=TRUE_WIDTH_M,
                                           **kwargs)


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
# the geometry, against a camera whose answer is known exactly
# --------------------------------------------------------------------------- #

def _synthetic(perceive, *, fx=600.0, shape=(480, 640), width_m=0.6,
               depth_m=0.45, pitch_deg=25.0, height_m=0.55):
    """A camera looking down at a rectangle, and the pixels it would see.

    Built forwards — place the table, project it — so that recovering it
    backwards is a real test and not a rearrangement of the same expression.
    """
    height, width = shape
    K = np.array([[fx, 0.0, width / 2.0], [0.0, fx, height / 2.0],
                  [0.0, 0.0, 1.0]])
    # camera at the origin, optical +z forward, +y image-down; the table is
    # below and in front, tipped by the camera's pitch.
    tilt = R.from_euler("x", pitch_deg, degrees=True)
    across = tilt.apply([1.0, 0.0, 0.0])          # far edge, image-left -> right
    away = tilt.apply([0.0, 0.0, 1.0])            # side edges, away from camera
    up = np.cross(across, away)          # out of the table, toward the camera
    up = up / np.linalg.norm(up)
    far_left = (tilt.apply([0.0, height_m, 0.0]) - across * (width_m / 2.0)
                + away * 0.85)

    def project(point):
        pixel = K @ np.asarray(point, dtype=float)
        return pixel[:2] / pixel[2]

    c1 = project(far_left)
    c2 = project(far_left + across * width_m)
    vanish = project(away * 1e6)
    return dict(K=K, fx=fx, shape=shape, width_m=width_m, depth_m=depth_m,
                across=across, away=away, up=up, far_left=far_left,
                c1=c1, c2=c2, vanish=vanish, project=project)


def test_the_metric_solve_recovers_a_camera_it_was_given(perceive):
    """Three pixel observations plus one known length -> the plane, to 1 mm."""
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


def test_a_lifted_slice_and_a_height_round_trip_too(perceive):
    """The rim of a cup: a point 100 mm above the table, measured there."""
    truth = _synthetic(perceive)
    plane = perceive.solve_plane(truth["c1"], truth["c2"], truth["vanish"],
                                 fx=truth["fx"], shape=truth["shape"],
                                 table_width_m=truth["width_m"])
    base = truth["far_left"] + truth["across"] * 0.30 - truth["away"] * 0.20
    top = base + truth["up"] * 0.10
    u, v = truth["project"](top)
    assert abs(plane.height_above(base, u, v) - 0.10) < 1e-3
    assert np.allclose(plane.point(u, v, 0.10), top, atol=1e-3)


def test_the_camera_distance_does_not_depend_on_the_camera_aim(perceive):
    """The claim ``--anchor camera`` rests on: rotating the camera cannot move
    the perpendicular distance from its centre to the table."""
    truth = _synthetic(perceive, pitch_deg=25.0)
    other = _synthetic(perceive, pitch_deg=40.0)
    a = perceive.solve_plane(truth["c1"], truth["c2"], truth["vanish"],
                             fx=truth["fx"], shape=truth["shape"],
                             table_width_m=truth["width_m"])
    b = perceive.solve_plane(other["c1"], other["c2"], other["vanish"],
                             fx=other["fx"], shape=other["shape"],
                             table_width_m=other["width_m"])
    assert abs(a.camera_distance_m - b.camera_distance_m) < 1e-6


# --------------------------------------------------------------------------- #
# the mask detector, on the frame the live run started from
# --------------------------------------------------------------------------- #

#: (across, depth) in plane metres of each footprint on frame D, from the
#: numbers Shu used for run2. He quoted them to +-2 cm.
FRAME_D_FOOTPRINTS = {"charger": (0.47, 0.376), "cup": (0.44, 0.26)}


def test_the_mask_detector_finds_both_footprints_on_frame_d(perceive, frames):
    image, plane = _fit(perceive, frames, "D")
    detections = perceive.detect_objects_mask(image, plane, [
        {"name": "charger", "kind": "object", "colour": "white"},
        {"name": "cup", "kind": "container", "colour": "brown"}])
    found = {d.name: d for d in detections}
    assert set(found) == {"charger", "cup"}
    anchor = perceive.anchor_far_edge(plane, far_edge_x=0.76, table_z=0.166)
    for name, (across, depth) in FRAME_D_FOOTPRINTS.items():
        item = perceive.measure(found[name], plane, anchor)
        got_across, got_depth = item["measurement"]["footprint_plane_m"]
        assert abs(got_across - across) < 0.03, f"{name} across {got_across}"
        assert abs(got_depth - depth) < 0.03, f"{name} depth {got_depth}"


def test_the_detector_searches_the_table_and_not_the_room(perceive, frames):
    """There is a white printer and a white wall socket behind the wagon; the
    script this came from fitted its colour windows around them by hand."""
    image, plane = _fit(perceive, frames, "D")
    detections = perceive.detect_objects_mask(image, plane, [
        {"name": "charger", "kind": "object", "colour": "white"}])
    x0, y0, x1, y1 = detections[0].bbox
    inside = perceive._table_polygon_mask(plane)
    assert inside[int(y1) - 2, int((x0 + x1) / 2)], "the blob is off the table"
    assert y0 > plane.far_px[0][1], "the blob is above the far edge"


def test_a_cup_is_measured_at_its_rim_and_a_box_at_its_footprint(perceive,
                                                                 frames):
    """The two shapes are measured differently ON PURPOSE (see ``measure``):
    a cylinder's silhouette is its diameter, a box's is wider than it is."""
    image, plane = _fit(perceive, frames, "D")
    anchor = perceive.anchor_far_edge(plane, far_edge_x=0.76, table_z=0.166)
    found = {d.name: d for d in perceive.detect_objects_mask(image, plane, [
        {"name": "charger", "kind": "object", "colour": "white"},
        {"name": "cup", "kind": "container", "colour": "brown"}])}
    cup = perceive.measure(found["cup"], plane, anchor)
    charger = perceive.measure(found["charger"], plane, anchor)
    # a 110 mm paper cup, 90 mm across the rim; a charger 50 x 50 mm
    assert 0.09 < cup["size"][0] < 0.115
    assert 0.10 < cup["size"][2] < 0.125
    assert 0.04 < charger["size"][0] < 0.06
    assert 0.03 < charger["size"][2] < 0.06
    assert cup["measurement"]["silhouette_width_m"] >= cup["size"][0]
    # the box is NOT measured at its silhouette, which is wider than it is
    assert charger["size"][0] < charger["measurement"]["silhouette_width_m"]


def test_an_unknown_colour_is_refused_rather_than_guessed(perceive, frames):
    image, plane = _fit(perceive, frames, "D")
    with pytest.raises(SystemExit):
        perceive.detect_objects_mask(image, plane, [
            {"name": "thing", "kind": "object", "colour": "chartreuse"}])
    with pytest.raises(SystemExit):
        perceive.detect_objects_mask(image, plane, [
            {"name": "thing", "kind": "object", "colour": None}])


# --------------------------------------------------------------------------- #
# anchors
# --------------------------------------------------------------------------- #

def test_the_far_edge_anchor_reproduces_the_hand_measured_scene(perceive,
                                                                frames):
    """Shu's hand-made run2 scene: far edge x 0.76, top z 0.166, cup at
    (0.50, -0.14), charger at (0.38, -0.17), all quoted +-2 cm."""
    image, plane = _fit(perceive, frames, "D")
    anchor = perceive.anchor_far_edge(plane, far_edge_x=0.76, centre_y=0.0,
                                      table_z=0.166)
    detections = perceive.detect_objects_mask(image, plane, [
        {"name": "charger", "kind": "object", "colour": "white"},
        {"name": "cup", "kind": "container", "colour": "brown"}])
    scene = perceive.build_scene(plane, anchor, detections)
    by_name = {o["name"]: o for o in scene["objects"]}
    assert by_name["table"]["measurement"]["top_z_base_m"] == pytest.approx(
        0.166, abs=1e-6)
    for name, (x, y) in (("cup", (0.50, -0.14)), ("charger", (0.38, -0.17))):
        p = by_name[name]["p"]
        assert abs(p[0] - x) < 0.02, f"{name} x {p[0]}"
        assert abs(p[1] - y) < 0.02, f"{name} y {p[1]}"


def test_the_camera_anchor_measures_a_table_height_nobody_typed(perceive,
                                                                frames):
    """The zero-shot claim. Shu's tape: the wagon top is 0.884 m off the floor
    and 0.166 m above ``base`` with the column at 0.205.

    The neck angle for these frames was not written down, so the fit's own
    answer (``neck_pitch_that_levels_rad``) is used — which is the workflow
    the mode documents when the daemon's neck state is not to hand.
    """
    from manipulation_kit.description.head_camera import floor_to_base_m

    _, plane = _fit(perceive, frames, "D")
    pitch = perceive.neck_pitch_that_levels(plane)
    assert pitch is not None and -0.35 < pitch < 0.65
    anchor = perceive.anchor_camera(plane, neck_pitch=pitch)
    assert anchor.calibrated is False
    assert abs(anchor.level_correction_deg) < 1.0
    assert abs(anchor.table_z - 0.166) < 0.03, anchor.table_z
    floor = anchor.table_z + floor_to_base_m(0.205)
    assert abs(floor - 0.884) < 0.03, floor


def test_the_far_edge_anchor_demands_the_height_it_cannot_measure(perceive,
                                                                  frames):
    parser = perceive.build_parser()
    args = parser.parse_args(["--image", str(frames / FRAMES["D"]),
                              "--anchor", "far-edge-x=0.76"])
    with pytest.raises(SystemExit) as caught:
        perceive.perceive(args)
    assert "--table-z" in str(caught.value)


def test_the_anchor_spec_is_parsed_or_refused(perceive):
    assert perceive.parse_anchor("camera") == {"mode": "camera"}
    got = perceive.parse_anchor("far-edge-x=0.76,centre-y=-0.03")
    assert got["far_edge_x"] == pytest.approx(0.76)
    assert got["centre_y"] == pytest.approx(-0.03)
    for bad in ("far-edge-x", "centre-y=0.0", "elbow=1"):
        with pytest.raises(SystemExit):
            perceive.parse_anchor(bad)


# --------------------------------------------------------------------------- #
# the scene file, and the loop that has to consume it
# --------------------------------------------------------------------------- #

@pytest.fixture
def frame_d_scene(perceive, frames):
    image, plane = _fit(perceive, frames, "D")
    anchor = perceive.anchor_far_edge(plane, far_edge_x=0.76, table_z=0.166)
    detections = perceive.detect_objects_mask(image, plane, [
        {"name": "charger", "kind": "object", "colour": "white"},
        {"name": "cup", "kind": "container", "colour": "brown"}])
    return perceive.build_scene(plane, anchor, detections)


def _world_from(scene, kin):
    import dataclasses
    import time

    from live import frames_from, objects_from
    from scene import demo_scene
    world, _ = demo_scene()
    return dataclasses.replace(world, objects=tuple(objects_from(scene)),
                               frames=frames_from(scene, now=time.time()))


def test_the_scene_loads_through_the_loops_own_reader(perceive, frame_d_scene,
                                                      tmp_path, d1_arm):
    from live import load_scene, objects_from
    path = tmp_path / "perceived.json"
    path.write_text(json.dumps(frame_d_scene, indent=1), encoding="utf-8")
    objects = objects_from(load_scene(path))
    assert {o.name for o in objects} == {"table", "charger", "cup"}
    cup = next(o for o in objects if o.name == "cup")
    # A PERCEIVED interior is a guess, and the flag the kit reads has to say
    # so — `Place` refuses to drop into a guessed interior.
    assert cup.interior_measured is False


def test_the_measured_scene_picks_the_right_arm_and_says_what_stops_it(
        perceive, frame_d_scene, d1_arm):
    """The honest outcome, pinned.

    Frame D's charger MEASURES 50 mm across its footprint and the driven jaws
    take 44 mm, so the chain that Shu's hand-made scene planned does not plan
    off the measurement — his file declared the charger 20 mm across y, which
    is a number a single view cannot produce. The right arm is still the one
    chosen and still the one that gets furthest, and the refusal is
    ``object_too_wide`` with both numbers in it.
    """
    from manipulation_kit.primitives.reach import choose_side
    world = _world_from(frame_d_scene, d1_arm)
    choice = choose_side(world, d1_arm, obj="charger", destination="cup",
                         approach="top_down")
    assert choice.side == "right"
    assert not choice.reachable
    right = choice.chains["right"]
    assert right.planned >= 1
    assert "object_too_wide" in right.sentence()
    assert "50 mm" in right.sentence() and "44 mm" in right.sentence()


def test_the_geometry_is_right_and_only_the_two_unseeable_numbers_are_not(
        perceive, frame_d_scene, d1_arm):
    """The other half of the previous test, and the one that says the PLANE is
    good: put back exactly the two things a single view cannot see — the
    charger's depth (Shu's tape: 20 mm) and the cup's real interior — and the
    right arm plans Approach, Grasp, Lift, Carry and Place off the perceived
    positions, heights and table."""
    import copy

    from manipulation_kit.primitives.reach import choose_side
    scene = copy.deepcopy(frame_d_scene)
    for item in scene["objects"]:
        if item["name"] == "charger":
            item["size"] = [item["size"][0], 0.02, item["size"][2]]
        if item["name"] == "cup":
            item["interior_measured"] = True
    world = _world_from(scene, d1_arm)
    choice = choose_side(world, d1_arm, obj="charger", destination="cup",
                         approach="top_down")
    assert choice.reachable, choice.reason
    assert choice.side == "right"


def test_the_zero_shot_anchor_also_plans_a_chain(perceive, frames, d1_arm):
    """``--anchor camera`` with the two declared numbers, end to end.

    Nothing here was measured by hand except the wagon's width, the two
    extents a single view cannot see, and fx. The table height, the far edge's
    distance and both objects come out of the picture — and the right arm
    plans the whole chain off them.

    It plans a DIFFERENT approach from the far-edge anchor (front, not
    top_down), which is the 2-3 cm the nominal head-camera mount is off: this
    anchor puts the wagon ~20 mm further away and ~30 mm to the right. Pinned
    so that a calibration that closes that gap is visible here as a change.
    """
    from manipulation_kit.primitives.reach import choose_side
    image, plane = _fit(perceive, frames, "D")
    anchor = perceive.anchor_camera(
        plane, neck_pitch=perceive.neck_pitch_that_levels(plane))
    detections = perceive.detect_objects_mask(image, plane, [
        {"name": "charger", "kind": "object", "colour": "white"},
        {"name": "cup", "kind": "container", "colour": "brown"}])
    scene = perceive.build_scene(plane, anchor, detections,
                                 interiors={"cup": [0.085, 0.085, 0.10]},
                                 sizes={"charger": [0.045, 0.02, 0.05]})
    world = _world_from(scene, d1_arm)
    plans = [a for a in ("top_down", "front", "side_right", "side_left")
             if choose_side(world, d1_arm, obj="charger", destination="cup",
                            approach=a).reachable]
    assert plans, "no approach plans off the zero-shot anchor"
    assert choose_side(world, d1_arm, obj="charger", destination="cup",
                       approach=plans[0]).side == "right"


def test_a_declared_size_says_it_was_declared(perceive, frames):
    """The escape hatch records what the FRAME said as well as what you did."""
    image, plane = _fit(perceive, frames, "D")
    anchor = perceive.anchor_far_edge(plane, far_edge_x=0.76, table_z=0.166)
    detections = perceive.detect_objects_mask(image, plane, [
        {"name": "charger", "kind": "object", "colour": "white"}])
    scene = perceive.build_scene(plane, anchor, detections,
                                 sizes={"charger": [0.045, 0.02, 0.05]})
    charger = next(o for o in scene["objects"] if o["name"] == "charger")
    assert charger["size"] == [0.045, 0.02, 0.05]
    assert charger["measurement"]["size_y_measured"] is True
    measured = charger["measurement"]["size_measured_m"]
    assert 0.045 < measured[0] < 0.06, measured
    assert any("--size" in n for n in charger["measurement"]["notes"])


def test_a_declared_interior_is_marked_measured(perceive, frames):
    image, plane = _fit(perceive, frames, "D")
    anchor = perceive.anchor_far_edge(plane, far_edge_x=0.76, table_z=0.166)
    detections = perceive.detect_objects_mask(image, plane, [
        {"name": "cup", "kind": "container", "colour": "brown"}])
    scene = perceive.build_scene(plane, anchor, detections,
                                 interiors={"cup": [0.08, 0.08, 0.10]})
    cup = next(o for o in scene["objects"] if o["name"] == "cup")
    assert cup["interior"] == [0.08, 0.08, 0.1]
    assert cup["interior_measured"] is True


def test_every_perceived_number_carries_its_uncertainty(perceive,
                                                        frame_d_scene):
    for item in frame_d_scene["objects"]:
        assert item["confidence"] < 1.0, item["name"]
        assert item["measurement"], item["name"]
    cup = next(o for o in frame_d_scene["objects"] if o["name"] == "cup")
    assert cup["measurement"]["interior_measured"] is False
    assert cup["measurement"]["size_y_measured"] is False
    assert cup["measurement"]["yaw_measured"] is False
    assert cup["yaw_rad"] == 0.0


def test_an_unmeasured_depth_is_declared_rather_than_defaulted(perceive,
                                                               frames):
    """Frame B's near edge is out of shot, so the 0.40 m in the file is an
    assumption and the file has to say which one."""
    image, plane = _fit(perceive, frames, "B")
    anchor = perceive.anchor_far_edge(plane, far_edge_x=0.76, table_z=0.166)
    scene = perceive.build_scene(plane, anchor, [])
    table = scene["objects"][0]
    assert table["size"][0] == pytest.approx(0.40)
    assert "ASSUMED" in table["measurement"]["depth_m"]


# --------------------------------------------------------------------------- #
# the Astra detector: the prompt and the parse, with no network
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


def test_the_astra_detector_sends_the_frame_and_parses_the_reply(perceive,
                                                                 frames):
    image = perceive.load_image(frames / FRAMES["D"])
    client = _FakeClient(GOOD_REPLY)
    found = {d.name: d for d in perceive.detect_objects(
        image, [{"name": "cup", "kind": "container", "colour": None},
                {"name": "charger", "kind": "object", "colour": None}],
        model="gpt-6-astra", client=client)}
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
    found = perceive.detect_objects(
        image, [{"name": "cup", "kind": "container", "colour": None},
                {"name": "charger", "kind": "object", "colour": None}],
        client=_FakeClient(chatty))
    assert {d.name for d in found} == {"cup", "charger"}


def test_the_detector_retries_once_and_then_stops(perceive, frames):
    image = perceive.load_image(frames / FRAMES["D"])
    client = _FakeClient("not json at all", GOOD_REPLY)
    found = perceive.detect_objects(
        image, [{"name": "cup", "kind": "container", "colour": None},
                {"name": "charger", "kind": "object", "colour": None}],
        client=client)
    assert len(client.calls) == 2
    assert {d.name for d in found} == {"cup", "charger"}
    # the failure is FED BACK, not silently retried with the same prompt
    retried = client.calls[1]["input"]
    assert any("not usable" in str(m.get("content", "")) for m in retried)

    hopeless = _FakeClient("still not json")
    with pytest.raises(ValueError):
        perceive.detect_objects(image, [{"name": "cup", "kind": "container",
                                         "colour": None}], client=hopeless)
    assert len(hopeless.calls) == 2


def test_a_reply_that_omits_a_requested_object_is_an_error(perceive, frames):
    image = perceive.load_image(frames / FRAMES["D"])
    partial = json.dumps({"objects": [
        {"name": "cup", "bbox": [440, 245, 535, 350], "base_px": [487, 349]}]})
    with pytest.raises(ValueError) as caught:
        perceive.detect_objects(
            image, [{"name": "cup", "kind": "container", "colour": None},
                    {"name": "charger", "kind": "object", "colour": None}],
            client=_FakeClient(partial))
    assert "charger" in str(caught.value)


def test_a_bbox_that_is_not_four_numbers_is_not_recovered_from(perceive):
    with pytest.raises(ValueError):
        perceive._parse_astra(json.dumps({"objects": [
            {"name": "cup", "bbox": [1, 2, 3]}]}),
            [{"name": "cup", "kind": "container", "colour": None}])


def test_an_astra_detection_measures_through_the_same_geometry(perceive,
                                                               frames):
    """A bbox with no contact band still produces a scene object — measured a
    little worse, and saying so."""
    image, plane = _fit(perceive, frames, "D")
    found = perceive.detect_objects(
        image, [{"name": "cup", "kind": "container", "colour": None}],
        client=_FakeClient(GOOD_REPLY))
    anchor = perceive.anchor_far_edge(plane, far_edge_x=0.76, table_z=0.166)
    item = perceive.measure(found[0], plane, anchor)
    assert item["measurement"]["contact_band_px"] is None
    assert abs(item["p"][0] - 0.50) < 0.03
    assert abs(item["p"][1] + 0.14) < 0.03


# --------------------------------------------------------------------------- #
# the CLI, end to end
# --------------------------------------------------------------------------- #

def test_the_cli_writes_a_scene_and_a_debug_frame(perceive, frames, tmp_path,
                                                  capsys):
    out = tmp_path / "scenes" / "live.json"
    debug = tmp_path / "fit.png"
    code = perceive.main([
        "--image", str(frames / FRAMES["D"]), "--table-width", "0.60",
        "--fx", "606", "--anchor", "far-edge-x=0.76", "--table-z", "0.166",
        "--objects", "charger:object,cup:container", "--detector", "mask",
        "--out", str(out), "--debug", str(debug)])
    assert code == 0
    assert debug.is_file() and debug.stat().st_size > 1000
    scene = json.loads(out.read_text(encoding="utf-8"))
    assert [o["name"] for o in scene["objects"]] == ["table", "charger", "cup"]
    assert scene["_perceive"]["anchor"]["calibrated"] is False
    printed = capsys.readouterr()
    assert json.loads(printed.out)["objects"]
    assert "table top z" in printed.err


def test_the_cli_reports_a_refused_fit_rather_than_a_traceback(perceive,
                                                               tmp_path):
    blank = tmp_path / "blank.png"
    perceive.write_png(blank, np.zeros((64, 64, 3), dtype=np.uint8))
    assert perceive.main(["--image", str(blank), "--anchor", "camera"]) == 2


def test_the_two_escape_hatches_are_parsed_or_refused(perceive):
    got = perceive.parse_extents(["cup=0.08,0.08,0.10"], "--interior")
    assert got == {"cup": [0.08, 0.08, 0.10]}
    for bad in (["cup"], ["cup=0.08,0.08"], ["cup=a,b,c"], ["cup=0,1,1"]):
        with pytest.raises(SystemExit):
            perceive.parse_extents(bad, "--size")


def test_the_object_spec_takes_kinds_and_colours(perceive):
    got = perceive._requested("charger:object,cup:container:brown,plate")
    assert got[0] == {"name": "charger", "kind": "object", "colour": "white"}
    assert got[1] == {"name": "cup", "kind": "container", "colour": "brown"}
    assert got[2] == {"name": "plate", "kind": "object", "colour": None}
    with pytest.raises(SystemExit):
        perceive._requested("cup:saucer")


def test_fx_comes_from_an_intrinsics_file_when_one_is_given(perceive,
                                                            tmp_path):
    direct = tmp_path / "a.json"
    direct.write_text(json.dumps({"fx": 612.5, "fy": 612.5}))
    assert perceive.read_fx(direct) == pytest.approx(612.5)
    ros = tmp_path / "b.json"
    ros.write_text(json.dumps({"camera_matrix": {"data": [
        608.0, 0, 320, 0, 608.0, 240, 0, 0, 1]}}))
    assert perceive.read_fx(ros) == pytest.approx(608.0)
    with pytest.raises(SystemExit):
        empty = tmp_path / "c.json"
        empty.write_text("{}")
        perceive.read_fx(empty)


# --------------------------------------------------------------------------- #
# the loop's --perceive hook
# --------------------------------------------------------------------------- #

def test_the_loop_perceives_a_scene_and_keeps_it_beside_the_trace(
        agent_examples, frames, tmp_path, perceive):
    import astra_loop
    scene = astra_loop.perceived_scene(
        str(frames / FRAMES["D"]), trace_path=tmp_path / "trace.jsonl",
        obj="charger", destination="cup",
        options="--table-width 0.60 --anchor far-edge-x=0.76 --table-z 0.166 "
                "--detector mask")
    assert [o["name"] for o in scene["objects"]] == ["table", "charger", "cup"]
    written = json.loads((tmp_path / "scene_perceived.json").read_text())
    assert written == scene


def test_scene_and_perceive_are_mutually_exclusive(agent_examples, frames,
                                                   perceive):
    import astra_loop
    with pytest.raises(SystemExit):
        astra_loop.main(["--scene", str(frames / FRAMES["D"]),
                         "--perceive", str(frames / FRAMES["D"]),
                         "--dry-run"])


def test_a_missing_frame_is_a_message_not_a_traceback(agent_examples, perceive):
    import astra_loop
    with pytest.raises(SystemExit) as caught:
        astra_loop.perceived_scene("/no/such/frame.jpg", trace_path=None,
                                   obj="a", destination="b")
    assert "no such frame" in str(caught.value)


def test_snapshot_mode_needs_the_hook_and_the_trace(agent_examples,
                                                    monkeypatch, perceive):
    import astra_loop
    monkeypatch.delenv("ASTRA_SNAPSHOT_CMD", raising=False)
    with pytest.raises(SystemExit) as caught:
        astra_loop.perceived_scene("snapshot", trace_path=None, obj="a",
                                   destination="b")
    assert "--trace" in str(caught.value)


def test_the_loop_runs_a_whole_scripted_task_off_a_perceived_scene(
        agent_examples, frames, tmp_path, perceive, d1_arm):
    """End to end with no robot: perceive frame D, hand the file to the loop,
    and let the scripted model play the chain on the kinematic mirror.

    The two numbers a single view cannot see are supplied the way a user would
    supply them — ``--interior`` on the command line, and the charger's depth
    edited into the file — which is also the documented workaround."""
    import astra_loop
    scene = astra_loop.perceived_scene(
        str(frames / FRAMES["D"]), trace_path=tmp_path / "t.jsonl",
        obj="charger", destination="cup",
        options="--table-width 0.60 --anchor far-edge-x=0.76 --table-z 0.166 "
                "--detector mask --interior cup=0.085,0.085,0.10")
    for item in scene["objects"]:
        if item["name"] == "charger":
            item["size"] = [item["size"][0], 0.02, item["size"][2]]
    path = tmp_path / "edited.json"
    path.write_text(json.dumps(scene), encoding="utf-8")
    code = astra_loop.main(["--scene", str(path), "--object", "charger",
                            "--destination", "cup", "--dry-run",
                            "--task", "put the charger in the cup"])
    assert code == 0
