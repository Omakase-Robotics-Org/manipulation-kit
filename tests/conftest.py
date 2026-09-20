"""Fixtures shared by the world / primitive / executor / agent suites.

Building the D1 arm model runs a 200-restart READY-seed search per side and
costs a couple of seconds, so it is built ONCE per session. Everything that
borrows it must put the joints back — which is what
:class:`manipulation_kit.primitives.Kin` does, and which ``d1_arm`` re-asserts
between tests so a leak shows up as a failure here rather than as a mystery
three files later.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
#: the agent examples live OUTSIDE the wheel (Shu, 2026-09-19), so the suite
#: that tests them puts that directory on the path rather than importing a
#: package that deliberately does not exist
EXAMPLES = REPO / "examples" / "agent"


@pytest.fixture(scope="session")
def _arm_model():
    from manipulation_kit.arms import get_arm_kinematics
    return get_arm_kinematics("d1/arm", quiet=True)


@pytest.fixture
def d1_arm(_arm_model):
    """The shared model, restored to HOME before and after every test."""
    for side in ("left", "right"):
        _arm_model.set_joints(side, _arm_model.home(side))
    yield _arm_model
    for side in ("left", "right"):
        _arm_model.set_joints(side, _arm_model.home(side))


@pytest.fixture
def agent_examples():
    """``examples/agent`` on ``sys.path``, removed again afterwards."""
    path = str(EXAMPLES)
    sys.path.insert(0, path)
    try:
        yield EXAMPLES
    finally:
        if path in sys.path:
            sys.path.remove(path)
        for name in ("menu", "mirror", "live", "trace", "scene",
                     "astra_loop", "jev_menu"):
            sys.modules.pop(name, None)


@pytest.fixture
def observe():
    """The scene producer, as a fixture.

    A fixture rather than an importable helper because ``tests/`` is not a
    package (no ``__init__.py``, by the repo's own pytest convention) and
    ``from tests.conftest import ...`` only works by accident of namespace
    packages on some interpreters. The suite has to run on 3.9 and 3.12.
    """
    return _observe


def _observe(kin, *, block_p=(0.38, 0.25, 0.05), box_p=(0.33, 0.34, 0.03),
            closed=None, held=None, block_size=(0.05, 0.04, 0.05), stamp=0.0,
            frames=None, gap=None, stalled=None):
    """A WorldView read off the kinematic mirror — the tests' scene producer.

    A gripper reporting ``holding`` reports the two measurements that go with
    it: a pad gap the block's own width could make, and a stalled stroke. The
    producer in the harness (and the firmware on the robot) publishes all
    three, so a fixture that published only the flag would be testing a
    verifier against a world no producer emits. ``gap`` / ``stalled`` override
    them per side for the tests that are about exactly that.
    """
    from manipulation_kit.primitives.approach import tool_from_link7
    from manipulation_kit.world import (ArmView, ContainerView, GripperView,
                                        ObjectView, SurfaceView, WorldView)
    closed = closed or {}
    held = held or {}
    arms, grippers = [], []
    for side in ("left", "right"):
        p, r = tool_from_link7(*kin.ee_pose(side))
        arms.append(ArmView(side, joints=kin.joints(side), tool_p=p, tool_r=r,
                            mode="position"))
        has = held.get(side) is not None
        grippers.append(GripperView(
            side, closed.get(side, 0.0), holding=has, held_object=held.get(side),
            jaw_gap_m=(gap or {}).get(side, 0.04) if gap is not None else 0.04,
            jaw_stalled=(stalled or {}).get(side, has) if stalled is not None else has))
    return WorldView.of(
        [ObjectView("red_block", p=block_p, size=block_size, colour="red"),
         ContainerView("box", p=box_p, size=(0.16, 0.14, 0.08),
                       interior=(0.14, 0.12, 0.07)),
         SurfaceView("table", p=(0.40, 0.0, 0.0), size=(0.9, 0.8, 0.02))],
        arms=arms, grippers=grippers, frames=frames, stamp=stamp)
