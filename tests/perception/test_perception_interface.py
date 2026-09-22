"""``manipulation_kit.perception`` as an INTERFACE: the rules it now owns.

Design D.1 step 6 (C.7; Astra review items 7, 8, 9; L5, L9, L15, B11). Each
test names a behaviour that used to be prose, an example re-implementation or
missing, and would fail on e1dce97 — where there was no package at all, the
neck sign was flipped in three places, the interior fraction had two values,
the declared-object lift ignored the surface's frame, the mount and plane
height were not separated, and a plane above the lens returned a point
behind the camera.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

REPO = Path(__file__).resolve().parents[2]
SRC = REPO / "src" / "manipulation_kit"
EXAMPLES = REPO / "examples"


def _head(**kwargs):
    from manipulation_kit.perception import HeadCamera
    kwargs.setdefault("neck_pitch", 0.512)
    return HeadCamera.from_robot(width=640, height=480, **kwargs)


def _python_files(*roots):
    for root in roots:
        for path in sorted(root.rglob("*.py")):
            if "_client" in path.parts or "__pycache__" in path.parts:
                continue
            yield path


# --------------------------------------------------------------------------- #
# review 7: a contact point is not a centre, and the kit converts it
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize("yaw_deg", [0.0, 30.0])
def test_a_contact_point_is_converted_to_a_centre(yaw_deg):
    """Place a box, look at the bottom-middle of its NEAR face — the pixel a
    silhouette's bottom gives — and the conversion must land on the box's
    centre: half the footprint further away, half the height up."""
    from manipulation_kit.perception import contact_to_centre
    camera = _head()
    table_z = 0.166
    size = np.array([0.06, 0.04, 0.10])
    centre = np.array([0.50, -0.12, table_z + size[2] / 2.0])
    yaw = math.radians(yaw_deg)
    away = centre[:2] - camera.p[:2]
    away = away / np.linalg.norm(away)
    ax = np.array([math.cos(yaw), math.sin(yaw)])
    ay = np.array([-math.sin(yaw), math.cos(yaw)])
    half_depth = 0.5 * (abs(away @ ax) * size[0] + abs(away @ ay) * size[1])
    near_edge = np.array([*(centre[:2] - away * half_depth), table_z])
    u, v = camera.project(near_edge)

    contact = camera.locate(u, v, plane_z=table_z)
    assert contact.kind == "contact"
    assert "CONTACT" in contact.to_text()
    assert np.allclose(contact.p, near_edge, atol=1e-9)

    got = contact_to_centre(contact, size=size, viewpoint=camera.p,
                            yaw_rad=yaw)
    assert got.kind == "centre"
    assert np.allclose(got.p, centre, atol=1e-9), got.p - centre
    assert got.uncertainty_m == contact.uncertainty_m
    assert "CENTRE" in got.to_text()
    # a centre is not converted twice, and a bad size is refused
    with pytest.raises(ValueError):
        contact_to_centre(got, size=size, viewpoint=camera.p)
    with pytest.raises(ValueError):
        contact_to_centre(contact, size=(0.05, 0.0, 0.1), viewpoint=camera.p)


def test_the_loops_locate_tool_does_the_conversion_given_a_size(
        agent_examples, d1_arm):
    """The prompt sentence that asked the model to shift its declaration by
    half an object is gone; the tool does it when told the size."""
    import dataclasses
    import time

    import astra_loop
    from live import frames_from, objects_from
    from scene import demo_scene
    assert "move\nthe declared centre about half" not in astra_loop.SYSTEM
    assert "size" in astra_loop.LOCATE_SCHEMA["parameters"]["properties"]
    scene = {"objects": [{"name": "table", "kind": "surface",
                          "p": [0.55, 0.0, 0.156], "size": [0.4, 0.6, 0.02],
                          "confidence": 0.2}], "frames": []}
    world = dataclasses.replace(demo_scene()[0],
                                objects=tuple(objects_from(scene)),
                                frames=frames_from(scene, now=time.time()))
    camera = _head()
    contact = astra_loop.apply_locate(camera, world, {"u": 400, "v": 380})
    centre = astra_loop.apply_locate(camera, world, {
        "u": 400, "v": 380, "size": [0.05, 0.05, 0.10]})
    assert "CONTACT" in contact and "CENTRE" in centre
    assert "z=0.166" in centre


# --------------------------------------------------------------------------- #
# L15: one interior fraction, one neck sign flip
# --------------------------------------------------------------------------- #

def test_the_interior_fraction_has_one_value():
    from manipulation_kit.perception import measure as measure_module
    from manipulation_kit.world import views
    assert measure_module.INTERIOR_FRACTION is views.INTERIOR_FRACTION
    kwdefaults = measure_module.measure.__kwdefaults__ or {}
    assert kwdefaults.get("interior_fraction") == views.INTERIOR_FRACTION
    definitions, literals = [], []
    for path in _python_files(SRC, EXAMPLES):
        text = path.read_text(encoding="utf-8")
        for number, line in enumerate(text.splitlines(), 1):
            if re.search(r"\bINTERIOR_FRACTION\s*=", line.split("#", 1)[0]):
                definitions.append(f"{path.relative_to(REPO)}:{number}")
    # a second LITERAL for it, anywhere perception, the world views or the
    # agent example could put one (0.85 was perceive.py's, `size * 0.9`
    # the view's own)
    for path in _python_files(SRC / "perception", SRC / "world",
                              EXAMPLES / "agent"):
        for number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            code = line.split("#", 1)[0]
            if re.search(r"(?<![\d.])0\.85(?!\d)", code) or \
                    re.search(r"\*\s*0\.9\b", code):
                literals.append(f"{path.relative_to(REPO)}:{number}: {line}")
    assert definitions == ["src/manipulation_kit/world/views.py:"
                           + definitions[0].split(":")[-1]], definitions
    assert not literals, literals
    # ...and what the operator is told is the number the file is written at
    preflight = (EXAMPLES / "preflight.py").read_text(encoding="utf-8")
    assert "90%" not in preflight and "INTERIOR_FRACTION" in preflight


def test_a_perceived_interior_uses_the_kits_fraction():
    from manipulation_kit.perception import Detection, TableInBase
    from manipulation_kit.perception.measure import measure
    from manipulation_kit.world.views import INTERIOR_FRACTION
    camera = _head()
    table = TableInBase(z=0.166, source="declared", uncertainty_m=0.005)
    cup = Detection(name="cup", kind="container", shape="cylinder",
                    bbox=(420.0, 280.0, 500.0, 380.0), base_px=(460.0, 380.0),
                    confidence=0.6)
    item = measure(cup, camera, table)
    width = item["size"][0]
    assert item["interior"][0] == pytest.approx(width * INTERIOR_FRACTION,
                                                abs=1e-4)
    assert item["interior_measured"] is False


#: a unary minus applied to something called pitch, or a negate() call
_FLIP = re.compile(
    r"(?<![\w)\]])-\s*(?:float\()?\s*[\w.]*\[?\s*['\"]?\w*pitch|"
    r"neck_pitch\s*=\s*-|\bnegate\w*\(", re.IGNORECASE)


def test_the_neck_sign_flip_has_one_implementation():
    """The daemon's logical pitch -> URDF joint flip lives in
    ``description/head_camera.py`` and nowhere else (it was also in
    ``camera.py:160-164`` and ``astra_loop.py:1049``)."""
    allowed = SRC / "description" / "head_camera.py"
    hits = []
    for path in _python_files(SRC, EXAMPLES):
        if path == allowed:
            continue
        for number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            code = line.split("#", 1)[0]
            if "pitch" in code.lower() and _FLIP.search(code):
                hits.append(f"{path.relative_to(REPO)}:{number}: {line}")
    assert not hits, hits
    assert _FLIP.search(allowed.read_text(encoding="utf-8"))
    # ...and the one implementation is the one the camera uses
    from manipulation_kit.description.head_camera import (
        head_camera_pose, neck_joints_from_state)
    from manipulation_kit.perception import HeadCamera
    assert neck_joints_from_state({"pitch": -0.3, "yaw": 0.1}) == (0.3, 0.1)
    live = HeadCamera.from_neck_state(_Neck(-0.3, 0.1), width=64, height=48)
    p, r = head_camera_pose(neck_pitch=0.3, neck_yaw=0.1)
    assert np.allclose(live.p, p) and np.allclose(live.r.as_quat(),
                                                  r.as_quat())


def test_no_raw_http_neck_or_slider_read_in_the_examples():
    for path in _python_files(EXAMPLES):
        text = path.read_text(encoding="utf-8")
        assert "urllib" not in text, path
        assert "/v1/neck/state" not in text.replace(
            "GET /v1/neck/state", ""), path
    assert not (EXAMPLES / "agent" / "camera.py").exists()


# --------------------------------------------------------------------------- #
# review 9: a typed head-camera config that fails closed
# --------------------------------------------------------------------------- #

@dataclass
class _Neck:                     # the step-1 NeckState contract, as a fake
    pitch_rad: float
    yaw_rad: float
    enabled: Optional[bool] = True
    moving: Optional[bool] = False


@dataclass
class _Lift:
    height_m: float
    moving: Optional[bool] = False
    alarm: Optional[str] = None


def test_the_head_camera_takes_a_typed_config_and_fails_closed():
    from manipulation_kit.perception import (HeadCamera, HeadCameraConfig,
                                             HeadPoseUnknown, LiftStateLike,
                                             NeckStateLike, read_head_state)
    assert isinstance(_Neck(0.0, 0.0), NeckStateLike)
    assert isinstance(_Lift(0.2), LiftStateLike)
    config = HeadCameraConfig(fx=606.0, fy=604.0, cx=321.0, cy=239.0,
                              width=640, height=480,
                              neck=_Neck(-0.512, 0.0), lift=_Lift(0.205))
    camera = HeadCamera.from_config(config)
    joints = _head(neck_pitch=0.512)
    assert np.allclose(camera.p, joints.p)
    assert camera.neck_pitch == pytest.approx(0.512)
    assert camera.fy == 604.0 and camera.K[1, 1] == 604.0
    assert camera.lift_m == pytest.approx(0.205)
    with pytest.raises(HeadPoseUnknown):
        HeadCamera.from_config(HeadCameraConfig(
            fx=606.0, fy=606.0, cx=320.0, cy=240.0, width=640, height=480))
    with pytest.raises(HeadPoseUnknown):
        HeadCamera.from_config(HeadCameraConfig(
            fx=606.0, fy=606.0, cx=320.0, cy=240.0, width=640, height=480,
            neck=_Neck(-0.5, 0.0, moving=True)))
    with pytest.raises(ValueError):
        HeadCameraConfig(fx=float("nan"), fy=606.0, cx=320.0, cy=240.0,
                         width=640, height=480)
    with pytest.raises(ValueError):
        HeadCamera.from_config(HeadCameraConfig(
            fx=606.0, fy=606.0, cx=320.0, cy=240.0, width=640, height=480,
            neck=_Neck(float("inf"), 0.0)))

    class _Executor:
        def neck_state(self):
            return _Neck(-0.4, 0.05)

        def lift_state(self):
            return _Lift(0.1)

    class _Kinematic:
        def neck_state(self):
            return None

    neck, lift = read_head_state(_Executor())
    assert neck.pitch_rad == -0.4 and lift.height_m == 0.1
    with pytest.raises(HeadPoseUnknown):
        read_head_state(_Kinematic())
    with pytest.raises(HeadPoseUnknown):
        read_head_state(object())


def test_a_live_neck_and_a_neck_flag_are_two_answers(agent_examples, tmp_path):
    pytest.importorskip("PIL")
    import perceive
    frame = REPO / "tests" / "data" / "perceive" / "frame_D_run2_start.jpg"
    args = perceive.build_parser().parse_args(
        ["--image", str(frame), "--neck-pitch", "0.512"])
    with pytest.raises(SystemExit):
        perceive.perceive(args, neck=_Neck(-0.512, 0.0))
    live = perceive.perceive(perceive.build_parser().parse_args(
        ["--image", str(frame)]), neck=_Neck(-0.512, 0.0), lift=_Lift(0.205))
    flags = perceive.perceive(perceive.build_parser().parse_args(
        ["--image", str(frame), "--neck-pitch", "0.512", "--lift", "0.205"]))
    assert live["objects"] == flags["objects"]
    assert live["_perceive"]["camera"] == flags["_perceive"]["camera"]


# --------------------------------------------------------------------------- #
# review 9: non-finite inputs and intersections behind the lens
# --------------------------------------------------------------------------- #

def test_a_ray_behind_the_lens_is_refused():
    from manipulation_kit.perception import NotOnThePlane
    camera = _head()
    above_lens = float(camera.p[2]) + 0.20
    with pytest.raises(NotOnThePlane) as caught:
        camera.locate(320, 400, plane_z=above_lens)
    assert "behind" in str(caught.value).lower()
    # a ray at the horizon is refused rather than extrapolated (kept)
    level = _head(neck_pitch=-0.30)
    with pytest.raises(NotOnThePlane):
        level.locate(320, 0, plane_z=0.166)
    for bad in ((float("nan"), 400.0, 0.166), (320.0, float("inf"), 0.166),
                (320.0, 400.0, float("nan"))):
        with pytest.raises(ValueError):
            camera.locate(bad[0], bad[1], plane_z=bad[2])
    with pytest.raises(ValueError):
        camera.locate(320, 400, plane_z=0.166, plane_uncertainty_m=-0.01)
    with pytest.raises(ValueError):
        camera.project([float("nan"), 0.0, 0.0])


# --------------------------------------------------------------------------- #
# review 8: uncertainty, propagated in independent parts, carried to the world
# --------------------------------------------------------------------------- #

def test_mount_and_height_uncertainty_are_propagated_separately():
    camera = _head()
    u, v, z = 360.0, 330.0, 0.166
    exact = camera.locate(u, v, plane_z=z)
    assert exact.height_uncertainty_m == 0.0
    assert exact.uncertainty_m == pytest.approx(exact.mount_uncertainty_m)

    unsure = camera.locate(u, v, plane_z=z, plane_uncertainty_m=0.10,
                           plane_source="provisional")
    assert unsure.mount_uncertainty_m == pytest.approx(
        exact.mount_uncertainty_m)                   # the mount did not move
    higher = camera.locate(u, v, plane_z=z + 0.10).p
    lower = camera.locate(u, v, plane_z=z - 0.10).p
    assert unsure.height_uncertainty_m == pytest.approx(max(
        np.linalg.norm(higher - exact.p), np.linalg.norm(lower - exact.p)))
    assert unsure.uncertainty_m == pytest.approx(math.hypot(
        unsure.mount_uncertainty_m, unsure.height_uncertainty_m))
    assert "from the plane's height" in unsure.to_text()

    # INDEPENDENT translation and aim: the combined figure covers every pair,
    # so it is at least the worst of each alone and of the old coupled
    # samples (lens and aim moved together along the same axis).
    from manipulation_kit.perception.camera import _intersect
    spread = camera.mount_spread(u, v, plane_z=z)
    local = camera._local_ray(u, v)
    tilt = math.radians(camera.aim_uncertainty_deg)
    coupled = 0.0
    for axis in np.eye(3):
        for sign in (1.0, -1.0):
            moved = _intersect(camera.p + axis * sign * camera.lens_uncertainty_m,
                               (R.from_rotvec(axis * sign * tilt)
                                * camera.r).as_matrix(), local, z)
            coupled = max(coupled, float(np.linalg.norm(moved - exact.p)))
    assert spread.translation_m == pytest.approx(camera.lens_uncertainty_m,
                                                 rel=0.05)
    assert spread.combined_m >= max(spread.translation_m, spread.aim_m,
                                    coupled) - 1e-12
    assert spread.combined_m > coupled                  # it was an undercount
    assert exact.mount_uncertainty_m == pytest.approx(spread.combined_m)


def test_plane_source_and_height_uncertainty_reach_the_world():
    """A perceived table's height provenance used to stop at the file."""
    from manipulation_kit.perception import (ScenePerceiver, TableInBase,
                                             table_object)
    from manipulation_kit.world import SurfaceView, WorldView
    table = TableInBase(z=0.166, source="provisional", uncertainty_m=0.10,
                        corners=np.array([[0.75, 0.3, 0.166],
                                          [0.75, -0.3, 0.166],
                                          [0.35, -0.3, 0.166],
                                          [0.35, 0.3, 0.166]]))
    item = table_object(table, _head())
    assert item["plane_source"] == "provisional"
    assert item["height_uncertainty_m"] == pytest.approx(0.10)
    surface = SurfaceView(item["name"], p=item["p"], size=item["size"],
                          plane_source=item["plane_source"],
                          height_uncertainty_m=item["height_uncertainty_m"],
                          confidence=item["confidence"])
    assert "top height provisional +-100mm" in surface.to_text()
    assert surface.to_json()["plane_source"] == "provisional"
    perceiver = ScenePerceiver({"head": _head()}, WorldView.of([surface]))
    located = perceiver.locate("head", 360, 330)
    assert located.plane_z == pytest.approx(0.166)
    assert "provisional" in located.plane_source
    assert located.height_uncertainty_m > 0.05
    with pytest.raises(ValueError):
        SurfaceView("t", p=(0, 0, 0), size=(1, 1, 0.02),
                    height_uncertainty_m=float("nan"))


def test_a_scene_file_surface_keeps_its_provenance_through_the_reader(
        agent_examples):
    from live import objects_from
    views = objects_from({"objects": [
        {"name": "table", "kind": "surface", "p": [0.55, 0, 0.156],
         "size": [0.4, 0.6, 0.02], "plane_source": "known-length",
         "height_uncertainty_m": 0.02}]})
    assert views[0].plane_source == "known-length"
    assert views[0].height_uncertainty_m == pytest.approx(0.02)


# --------------------------------------------------------------------------- #
# L5 / L15: a declared object is lifted onto its support, frame resolved
# --------------------------------------------------------------------------- #

def _wagon_world(yaw_deg=30.0):
    from manipulation_kit.world import Frame, FrameGraph, SurfaceView, WorldView
    frames = FrameGraph.of([Frame("wagon", "base", p=(0.55, 0.0, 0.10),
                                  r=R.from_euler("z", yaw_deg, degrees=True))])
    # the wagon top is 50 mm up IN THE WAGON FRAME, i.e. 0.16 m in base
    top = SurfaceView("wagon_top", p=(0.0, 0.0, 0.05), size=(0.4, 0.6, 0.02),
                      frame_id="wagon", plane_source="declared")
    return WorldView.of([top], frames=frames), top


def test_a_declared_object_is_lifted_onto_its_support_with_the_frame_resolved():
    from manipulation_kit.perception import ScenePerceiver, lift_onto_support
    from manipulation_kit.world import ObjectView
    world, top = _wagon_world()
    assert top.top_z(world.frames) == pytest.approx(0.16)
    # the open-coded rule would have put the top at p[2] + size[2]/2 = 0.06
    sunk = ObjectView("cube", p=(0.56, 0.02, 0.03), size=(0.05, 0.05, 0.05))
    in_wagon = ObjectView("block", p=(0.05, -0.1, 0.0),
                          size=(0.04, 0.04, 0.04), frame_id="wagon",
                          r=R.from_euler("z", 0.4))
    held_up = ObjectView("floating", p=(0.55, 0.0, 0.40),
                         size=(0.05, 0.05, 0.05))
    lifted, notes = lift_onto_support([sunk, in_wagon, held_up], [top],
                                      world.frames)
    by = {o.name: o for o in lifted}
    for name in ("cube", "block"):
        assert by[name].bottom_z(world.frames) == pytest.approx(0.16)
        assert top.supports_object(by[name], world.frames), name
    # the one declared in the wagon frame stays in it, moved along base +z
    assert by["block"].frame_id == "wagon"
    assert np.allclose(by["block"].pose_in_base(world.frames)[0][:2],
                       in_wagon.pose_in_base(world.frames)[0][:2])
    assert by["floating"].p[2] == pytest.approx(0.40)   # never lowered
    assert any("cube raised" in n and "'wagon_top'" in n for n in notes)

    perceiver = ScenePerceiver(world=world)
    after = perceiver.declare([sunk])
    assert after.find("cube").bottom_z(after.frames) == pytest.approx(0.16)
    assert after.find("wagon_top") is not None
    assert perceiver.notes


def test_the_loops_declare_scene_uses_the_kit_rule(agent_examples, d1_arm):
    """``apply_declare_scene`` no longer open-codes the lift."""
    source = (EXAMPLES / "agent" / "astra_loop.py").read_text(encoding="utf-8")
    assert "lift_onto_support" in source
    assert "float(s.p[2]) + float(s.size[2]) / 2.0" not in source
    assert "float(o.p[2]) + float(o.size[2]) / 2.0" not in source


# --------------------------------------------------------------------------- #
# the wrist camera model
# --------------------------------------------------------------------------- #

_WRIST = dict(fx=400.0, fy=400.0, cx=320.0, cy=240.0, width=640, height=480)


@pytest.mark.parametrize("side", ["left", "right"])
def test_the_wrist_camera_sees_its_own_jaws_at_the_bottom_of_the_image(
        side, d1_arm):
    """The real wrist streams show the jaws at the BOTTOM edge; the mount is
    the kit's plate constants, composed with the arm's FK."""
    from manipulation_kit.hands.d1.parallel_gripper.description import (
        PAD_TIP_Z_M)
    from manipulation_kit.perception import WristCamera
    from manipulation_kit.primitives.approach import TCP_P, TCP_R
    camera = WristCamera.from_kin(d1_arm, side, **_WRIST)
    p7, r7 = d1_arm.ee_pose(side)
    flange_r = r7 * TCP_R
    flange_p = np.asarray(p7) + r7.apply(TCP_P)
    tips = flange_p + flange_r.apply([0.0, 0.0, PAD_TIP_Z_M])
    seen = camera.project_point(tips)
    assert seen.depth_m is not None and seen.depth_m > 0.0
    assert seen.v > _WRIST["cy"], seen
    # ...and a point well behind the flange is not in front of the lens
    behind = flange_p - flange_r.apply([0.0, 0.0, 0.30])
    assert not camera.project_point(behind).visible


def test_the_wrist_camera_says_whether_an_object_is_in_frame(d1_arm):
    from manipulation_kit.perception import WristCamera
    from manipulation_kit.primitives.approach import TCP_P, TCP_R
    from manipulation_kit.world import FrameGraph, ObjectView
    camera = WristCamera.from_kin(d1_arm, "right", **_WRIST)
    optical_axis = camera.r.apply([0.0, 0.0, 1.0])
    ahead = ObjectView("cube", p=camera.p + optical_axis * 0.25,
                       size=(0.04, 0.04, 0.04))
    seen = camera.project_object(ahead, FrameGraph())
    assert seen.visible and seen.u == pytest.approx(_WRIST["cx"])
    beside = ObjectView("cube", p=camera.p + camera.r.apply([1.0, 0.0, 0.2]),
                        size=(0.04, 0.04, 0.04))
    assert not camera.project_object(beside, FrameGraph()).visible
    assert TCP_P is not None and TCP_R is not None


# --------------------------------------------------------------------------- #
# L9: the example is a thin wrapper, and the perceiver protocol is satisfied
# --------------------------------------------------------------------------- #

def test_the_example_perceive_cli_is_a_thin_wrapper():
    text = (EXAMPLES / "agent" / "perceive.py").read_text(encoding="utf-8")
    assert len(text.splitlines()) < 300
    assert "manipulation_kit.perception" in text
    for numerics in ("np.linalg", "ndimage", "scipy", "def fit_", "def measure",
                     "def solve_plane", "INTERIOR_FRACTION", "0.85",
                     "def locate", "Rotation"):
        assert numerics not in text, numerics
    detector = (EXAMPLES / "agent" / "detector.py").read_text(encoding="utf-8")
    assert len(detector.splitlines()) < 220
    assert "class AstraDetector(ScenePerceiver)" in detector


def test_the_astra_detector_is_a_perceiver(agent_examples):
    import detector
    from manipulation_kit.perception import Perceiver, ScenePerceiver
    assert isinstance(detector.AstraDetector(), Perceiver)
    assert isinstance(ScenePerceiver(), Perceiver)


def test_the_kit_opens_no_image_without_the_extra():
    """``import manipulation_kit.perception`` needs numpy and scipy only."""
    decoder = re.compile(r"^\s*(?:from|import)\s+(?:PIL|cv2|imageio)\b",
                         re.MULTILINE)
    for path in (SRC / "perception").glob("*.py"):
        assert not decoder.search(path.read_text(encoding="utf-8")), path
