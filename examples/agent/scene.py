"""One small scene the runnable examples share, so they can be compared.

A 50x40x50 mm block on a table, a box to put it in, both arms at HOME. The
numbers are inside the D1's measured reachable envelope — the point of the
examples is the LOOP, not a reachability puzzle.
"""

from __future__ import annotations

from typing import Tuple

from manipulation_kit.arms import get_arm_kinematics
from manipulation_kit.primitives.orientation import tool_from_link7
from manipulation_kit.world import (ArmView, ContainerView, GripperView,
                                    ObjectView, SurfaceView, WorldView)

BLOCK_P = (0.38, 0.25, 0.05)
BOX_P = (0.33, 0.34, 0.03)
#: The mirror's wrist-camera INTRINSICS — a 640x480 placeholder, because the
#: demo robot has a wrist-camera model (so the policy's look before a stroke
#: runs) and no photograph. Not any real lens: on a robot these come from the
#: stream, in the scene's ``robot.wrist_camera``.
DEMO_WRIST_CAMERA = {"fx": 320.0, "fy": 320.0, "cx": 320.0, "cy": 240.0,
                     "width": 640, "height": 480}


def observe(kin, *, block_p=BLOCK_P, closed=None, held=None) -> WorldView:
    """Build a WorldView from the kinematic mirror — the shape a real producer
    (``d1-inference/scene``, the Isaac env server) fills in from a camera."""
    closed = closed or {"left": 0.0, "right": 0.0}
    held = held or {}
    arms, grippers = [], []
    for side in ("left", "right"):
        p, r = tool_from_link7(*kin.ee_pose(side))
        arms.append(ArmView(side, joints=kin.joints(side), tool_p=p, tool_r=r,
                            mode="position"))
        grippers.append(GripperView(side, closed.get(side, 0.0),
                                    holding=held.get(side) is not None,
                                    held_object=held.get(side),
                                    jaw_gap_m=0.04))
    return WorldView.of(
        [ObjectView("red_block", p=block_p, size=(0.05, 0.04, 0.05), colour="red"),
         ContainerView("box", p=BOX_P, size=(0.16, 0.14, 0.08),
                       interior=(0.14, 0.12, 0.07)),
         SurfaceView("table", p=(0.40, 0.0, 0.0), size=(0.9, 0.8, 0.02))],
        arms=arms, grippers=grippers)


def demo_scene() -> Tuple[WorldView, object]:
    kin = get_arm_kinematics("d1/arm", quiet=True)
    return observe(kin), kin
