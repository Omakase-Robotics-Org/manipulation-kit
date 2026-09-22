"""``Direction``: a unit vector and a frame — resolution, parsing, rendering."""

from __future__ import annotations

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from manipulation_kit.world import (ALIASES, TOOL, ArmView, Direction,
                                    FrameError, ObjectView, WorldView,
                                    parse_direction, toward)
from manipulation_kit.world.frames import UNKNOWN_FRAME


def _arms_world(objects=()):
    """Two arms whose tool frames differ — one pads down, one tipped 30 deg."""
    left = R.from_euler("xyz", (180.0, 0.0, 90.0), degrees=True)
    right = R.from_euler("xyz", (150.0, 0.0, -90.0), degrees=True)
    return WorldView.of(list(objects), arms=[
        ArmView("left", joints=np.zeros(7), tool_p=(0.3, 0.2, 0.2), tool_r=left),
        ArmView("right", joints=np.zeros(7), tool_p=(0.3, -0.2, 0.2),
                tool_r=right)])


def test_a_direction_is_normalised_and_compares_by_value():
    assert Direction((0, 0, -2)) == ALIASES["down"]
    assert Direction((0, 0, -2)).v == (0.0, 0.0, -1.0)
    assert hash(Direction((3, 0, 0))) == hash(ALIASES["forward"])
    assert Direction((0, 0, 1), TOOL) != ALIASES["up"]      # same vector, other frame
    assert ALIASES["up"].opposite() == ALIASES["down"]
    with pytest.raises(ValueError):
        Direction((0, 0, 0))
    with pytest.raises(ValueError):
        Direction((0, float("nan"), 1))
    with pytest.raises(ValueError):
        Direction((1, 0, 0), frame="world")


def test_the_aliases_are_the_documented_vectors():
    expect = {"down": (0, 0, -1), "up": (0, 0, 1), "forward": (1, 0, 0),
              "backward": (-1, 0, 0), "left": (0, 1, 0), "right": (0, -1, 0),
              "along_tool": (0, 0, 1)}
    assert set(ALIASES) == set(expect)
    for name, v in expect.items():
        assert ALIASES[name].v == pytest.approx(v)
        assert ALIASES[name].frame == (TOOL if name == "along_tool" else "base")
        assert ALIASES[name].label() == name


def test_a_tool_frame_direction_resolves_per_side():
    """``along_tool`` is each hand's OWN +z — mirrored hands, different vectors."""
    world = _arms_world()
    along = ALIASES["along_tool"]
    for side in ("left", "right"):
        expect = world.arm(side).tool_r.as_matrix()[:, 2]
        assert np.allclose(along.resolve(world, side=side), expect)
    left = along.resolve(world, side="left")
    right = along.resolve(world, side="right")
    assert not np.allclose(left, right)
    # a tool frame with no hand named is unknowable, not "base"
    with pytest.raises(FrameError) as exc:
        along.resolve(world)
    assert exc.value.reason == UNKNOWN_FRAME
    # and the planner may hand in the tool orientation it plans from
    r = R.from_euler("x", 90.0, degrees=True)
    assert np.allclose(along.resolve(world, side="left", tool_r=r),
                       r.as_matrix()[:, 2])
    # the base frame ignores the hand entirely
    assert np.allclose(ALIASES["down"].resolve(world, side="left"), (0, 0, -1))


def test_an_unknown_object_frame_is_a_FrameError_not_base():
    cup = ObjectView("cup", p=(0.4, 0.0, 0.1), size=(0.08, 0.08, 0.1),
                     r=R.from_euler("z", 90.0, degrees=True))
    world = _arms_world([cup])
    # a known object's own +x, yawed 90 deg, is base +y
    assert np.allclose(Direction((1, 0, 0), "object:cup").resolve(world),
                       (0, 1, 0), atol=1e-12)
    with pytest.raises(FrameError) as exc:
        Direction((1, 0, 0), "object:ghost").resolve(world)
    assert exc.value.reason == UNKNOWN_FRAME
    assert exc.value.frame_id == "object:ghost"


def test_parse_direction_takes_an_alias_a_vector_or_axis_and_frame():
    assert parse_direction("down") is ALIASES["down"]
    assert parse_direction([0, 0, -5]) == ALIASES["down"]
    assert parse_direction({"axis": [0, 0, 1], "frame": "tool"}) == ALIASES["along_tool"]
    assert parse_direction({"axis": [1, 0, 0]}) == ALIASES["forward"]
    d = Direction((1, 1, 0))
    assert parse_direction(d) is d
    for bad in ("top_down", "front", "side_left", {"axis": [1, 0, 0], "x": 1},
                {"frame": "base"}, [1, 0], [0, 0, 0], None, True):
        with pytest.raises(ValueError):
            parse_direction(bad)


def test_labels_and_json():
    assert ALIASES["along_tool"].opposite().label() == "-along_tool"
    free = Direction((1, 1, 0))
    assert "in base" in free.label()
    assert free.as_argument() == {"axis": [round(1 / math.sqrt(2), 6)] * 2 + [0.0],
                                  "frame": "base"}
    assert ALIASES["left"].as_argument() == "left"
    assert ALIASES["down"].to_json() == {"axis": [0.0, 0.0, -1.0],
                                         "frame": "base", "label": "down"}
    # -0.0 never leaks into JSON
    assert ALIASES["along_tool"].opposite().as_argument()["axis"] == [0.0, 0.0, -1.0]


def test_toward_points_at_the_object_centre():
    cup = ObjectView("cup", p=(0.5, 0.0, 0.1), size=(0.08, 0.08, 0.1))
    world = _arms_world([cup])
    d = toward(world, name="cup", from_p=(0.3, 0.0, 0.1))
    assert d == ALIASES["forward"] and d.label() == "forward"
    d = toward(world, name="cup", from_p=(0.3, 0.2, 0.1))
    assert d.label() == "toward cup"
    with pytest.raises(FrameError):
        toward(world, name="ghost", from_p=(0, 0, 0))
