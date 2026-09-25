"""A look at a held object can refute the hold (``manipulation_kit.agent.hold``).

d1-2, 2026-09-24 (``servo-judgeonly-2``): after a grasp the verifier called
a hold and a lift that "rose 150 mm" by attachment alone, the model located
the tape on the right wrist camera twice (turns 9 and 10). Both pixels are on
the roll standing on the table, 54 and 76 px off where a roll in the jaws
would appear — the kit re-declared it at the attached height and nudged the
holding hand instead of saying the hold was false.
"""

from __future__ import annotations

import json
import warnings
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from manipulation_kit.agent.hold import hold_sighting, silhouette_px
from manipulation_kit.agent.robot import objects_from
from manipulation_kit.world import ArmView, FrameGraph, GripperView, WorldView

DATA = Path(__file__).resolve().parents[1] / "data"


def _world(doc) -> WorldView:
    stamp = float(doc["stamp"])
    return WorldView.of(
        objects_from({"objects": doc["objects"]}), frames=FrameGraph(now=stamp),
        arms=[ArmView(a["side"], joints=np.radians(a["joints_deg"]),
                      tool_p=a["tool_p"],
                      tool_r=R.from_quat(a["tool_quat_xyzw"]))
              for a in doc["arms"]],
        grippers=[GripperView(g["side"], g["closedness"],
                              holding=g.get("holding", False),
                              jaw_gap_m=g.get("jaw_gap_m"),
                              held_object=g.get("held_object"),
                              jaw_stalled=g.get("jaw_stalled"))
                  for g in doc["grippers"]], stamp=stamp)


@pytest.fixture
def d1_2_wrist():
    from manipulation_kit.agent.robot import wrist_camera_from_scene
    from manipulation_kit.description.robot_profile import (RobotProfile,
                                                            with_profile)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")        # the fixture's WARN gates
        profile = RobotProfile.resolve(DATA / "d1-2.camera_calibration.json")
        return wrist_camera_from_scene(with_profile(None, profile),
                                       measured_only=True)


def _camera_at(d1_arm, wrist, world, side="right"):
    from manipulation_kit.perception import WristCamera
    saved = np.array(d1_arm.joints(side), dtype=float)
    try:
        d1_arm.set_joints(side, world.arm(side).joints)
        return WristCamera.from_kin(d1_arm, side, **wrist[side])
    finally:
        d1_arm.set_joints(side, saved)


def _recorded():
    doc = json.loads((DATA / "grasp_rim_pinch_d1_2_20260924.json")
                     .read_text(encoding="utf-8"))
    grasped = _world(doc["grasp"]["world_before"]).find("tape")
    return grasped, doc["post_lift_locates"]


@pytest.mark.parametrize("index", [0, 1], ids=["turn9", "turn10"])
def test_the_recorded_post_lift_locates_contradict_the_hold(
        d1_arm, d1_2_wrist, index):
    from manipulation_kit.agent.tools import locate
    grasped, looks = _recorded()
    look = looks[index]
    world = _world(look["world"])
    held = world.find("tape")
    assert held.provenance == "attached"
    camera = _camera_at(d1_arm, d1_2_wrist, world)
    args = look["choice"]["arguments"]
    seen = locate({"right_wrist": camera}, world, args)
    got = hold_sighting(camera_name="right_wrist", camera=camera,
                        pixel=(args["u"], args["v"]), seen_p=seen.p,
                        held=held, grasped=grasped, frames=world.frames,
                        side="right")
    m = got.measured
    assert got.holding_verified is False, got.reason
    assert m["px_off_held"] > 40            # 54 / 76 px below the pixel
    assert m["px_off_grasped"] <= 12        # on the roll's silhouette
    assert m["seen_to_grasp_site_m"] <= 0.08
    assert m["held_rise_m"] == pytest.approx(0.15, abs=0.005)
    assert "on the table" in got.reason
    assert got.to_text().startswith("HOLD CONTRADICTED")


def test_a_pixel_on_the_held_silhouette_corroborates_the_hold(
        d1_arm, d1_2_wrist):
    from manipulation_kit.agent.tools import locate
    grasped, looks = _recorded()
    world = _world(looks[0]["world"])
    held = world.find("tape")
    camera = _camera_at(d1_arm, d1_2_wrist, world)
    box = silhouette_px(camera, held, world.frames)
    u, v = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0
    seen = locate({"right_wrist": camera}, world,
                  {"camera": "right_wrist", "u": u, "v": v,
                   "size": [0.05, 0.05, 0.026]})
    got = hold_sighting(camera_name="right_wrist", camera=camera,
                        pixel=(u, v), seen_p=seen.p, held=held,
                        grasped=grasped, frames=world.frames, side="right")
    assert got.holding_verified is True, got.reason
    assert got.measured["px_off_held"] == 0.0


def test_a_view_that_cannot_separate_the_two_is_inconclusive():
    """A camera looking straight down the lift sees the held object and the
    grasp site on the same pixels: no verdict either way."""
    from manipulation_kit.perception import PinholeCamera
    from manipulation_kit.world import ObjectView
    camera = PinholeCamera(p=np.array([0.4, 0.0, 0.8]),
                           r=R.from_euler("x", 180, degrees=True),
                           fx=600.0, fy=600.0, cx=320.0, cy=240.0,
                           width=640, height=480)
    grasped = ObjectView("tape", p=[0.4, 0.0, 0.179], size=[0.05, 0.05, 0.026],
                         provenance="observed")
    held = ObjectView("tape", p=[0.4, 0.0, 0.329], size=[0.05, 0.05, 0.026],
                      provenance="attached")
    u, v = camera.project([0.4, 0.0, 0.179])
    got = hold_sighting(camera_name="head", camera=camera, pixel=(u, v),
                        seen_p=[0.4, 0.0, 0.179], held=held, grasped=grasped,
                        frames=FrameGraph(), side="right")
    assert got.holding_verified is None
    assert "cannot tell them apart" in got.reason


# --------------------------------------------------------------------------- #
# in the loop
# --------------------------------------------------------------------------- #

class _Ask:
    """Plays calls; a callable entry is built at call time from the robot."""

    def __init__(self, robot, *calls):
        self.robot, self.calls = robot, list(calls)

    def __call__(self, messages, tools):
        if not self.calls:
            return {"name": None, "arguments": {}, "claimed": ""}
        call = self.calls.pop(0)
        name, arguments = call(self.robot) if callable(call) else call
        return {"name": name, "arguments": arguments, "claimed": "",
                "call_id": f"c{len(self.calls)}"}


def _left_behind_pixel(site):
    """A left-wrist locate on where the block was grasped, from wherever the
    hand is now: the model pointing at the block still on the table."""
    def call(robot):
        world = robot.world()
        camera = robot.cameras(world)["left_wrist"]
        block = world.find("red_block")
        bottom = np.array([site[0], site[1], site[2] - block.size[2] / 2.0])
        u, v = camera.project(bottom)
        return "locate", {"camera": "left_wrist", "u": u, "v": v,
                          "size": [float(c) for c in block.size]}
    return call


def test_the_loop_records_a_contradicted_hold_and_does_not_nudge(
        agent_examples):
    from manipulation_kit.agent import KinematicMirror, OperatorPolicy, run
    from manipulation_kit.primitives import Place
    from scene import DEMO_WRIST_CAMERA, demo_scene
    world, kin = demo_scene()
    robot = KinematicMirror(kin, world, wrist_intrinsics=DEMO_WRIST_CAMERA)
    site = robot.world().find("red_block").p.copy()
    grasp = ("grasp", {"object": "red_block", "side": "left",
                       "direction": "down"})
    model = _Ask(robot,
                 ("approach", {"object": "red_block", "side": "left",
                               "direction": "down"}),
                 grasp, grasp,
                 ("lift", {"object": "red_block", "side": "left",
                           "height_m": 0.15}),
                 _left_behind_pixel(site))
    trace = run(goal=Place(object="red_block", to="box"), robot=robot,
                policy=OperatorPolicy(max_turns=5), ask=model)
    approach, look, stroke, lift, located = trace.records
    # the grasp was graded with the run's evidence, criterion by criterion
    checks = stroke.verdict["measured"]["checks"]
    assert stroke.verdict["verdict"] == "true", stroke.verdict["reason"]
    # (the mirror has no pad-gap sensor: the gap is unmeasured, not passed)
    assert {k: v["verdict"] for k, v in checks.items()} == {
        "jaw_gap": "unmeasured", "insertion": "pass", "approach": "pass"}
    assert checks["approach"]["run_completed"] is True
    assert lift.verdict["verdict"] == "true"
    # the look after the lift puts the block back at the grasp site
    evidence = located.hold_evidence
    assert evidence is not None and evidence["holding_verified"] is False, \
        evidence
    answer = located.observation_after["answer"]
    assert answer.startswith("HOLD CONTRADICTED")
    # and the holding hand was not "corrected" toward it
    assert located.plan is None and located.run is None
    after = robot.world().find("red_block")
    assert after.provenance == "attached"
