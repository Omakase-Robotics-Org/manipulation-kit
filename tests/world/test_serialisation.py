"""The world description is a PROMPT, and a prompt has a budget.

``WorldView.to_text()`` is what a typed-choice model is shown instead of an
image. If it grows with the scene it silently stops fitting, the model starts
losing the earliest objects, and nothing anywhere says so. So the budget is a
test, not a hope.

The token estimate is the conservative ~4 characters per token that holds for
English prose with numbers in it; the assertion has room to spare either way.
"""

from __future__ import annotations

import json

from scipy.spatial.transform import Rotation as R

from manipulation_kit.world import (ContainerView, FrameGraph, GripperView,
                                    ArmView, Frame, ObjectView, SurfaceView,
                                    WorldView)

#: characters per token, conservative
CHARS_PER_TOKEN = 4
BUDGET_TOKENS = 1500


def _four_object_world():
    import numpy as np
    return WorldView.of(
        [ObjectView("red_block", p=(0.40, 0.10, 0.05), size=(0.05, 0.04, 0.05),
                    colour="red"),
         ObjectView("green_cup", p=(0.45, -0.05, 0.06), size=(0.07, 0.07, 0.09),
                    colour="green", r=R.from_euler("z", 30, degrees=True)),
         ContainerView("box", p=(0.33, 0.34, 0.03), size=(0.16, 0.14, 0.08),
                       interior=(0.14, 0.12, 0.07)),
         SurfaceView("table", p=(0.40, 0.0, 0.0), size=(0.90, 0.80, 0.02))],
        arms=[ArmView(s, joints=np.zeros(7), tool_p=(0.29, 0.21, 0.20),
                      mode="position") for s in ("left", "right")],
        grippers=[GripperView(s, 0.0, jaw_gap_m=0.052) for s in ("left", "right")])


def test_four_objects_and_both_arms_fit_well_inside_the_prompt_budget():
    text = _four_object_world().to_text()
    tokens = len(text) / CHARS_PER_TOKEN
    assert tokens < BUDGET_TOKENS, (
        f"{tokens:.0f} estimated tokens for a four-object world; the budget is "
        f"{BUDGET_TOKENS}. A world description that grows without bound is a "
        f"prompt that silently stops fitting.\n\n{text}")


def test_every_object_appears_in_the_text_by_name():
    world = _four_object_world()
    text = world.to_text()
    for name in world.names():
        assert name in text


def test_the_text_states_the_frame_convention_once():
    text = _four_object_world().to_text()
    assert "+x forward" in text and "+y robot-left" in text


def test_stale_frames_are_announced_rather_than_quietly_dropped():
    """A model that cannot see WHY an object is unusable will keep asking."""
    import numpy as np
    graph = FrameGraph.of([Frame("table", "base", p=[0.5, 0, 0], r=R.identity(),
                                 stamp=0.0, max_age_s=1.0)], now=40.0)
    world = WorldView.of(
        [ObjectView("red_block", p=(0.1, 0, 0), size=(0.05,) * 3,
                    frame_id="table")],
        arms=[ArmView("left", joints=np.zeros(7))], frames=graph)
    assert "stale frames" in world.to_text()
    assert "table" in world.to_text()


def test_to_json_is_json_and_round_trips_through_the_serialiser():
    world = _four_object_world()
    payload = json.loads(world.to_json_str())
    assert [o["name"] for o in payload["objects"]] == list(world.names())
    assert payload["objects"][0]["size"] == [0.05, 0.04, 0.05]
    assert {a["side"] for a in payload["arms"]} == {"left", "right"}


def test_the_json_form_carries_the_interior_a_place_verifier_needs():
    payload = _four_object_world().to_json()
    box = next(o for o in payload["objects"] if o["name"] == "box")
    assert box["interior"] == [0.14, 0.12, 0.07]
