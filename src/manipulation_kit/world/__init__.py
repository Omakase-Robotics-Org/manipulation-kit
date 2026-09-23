"""manipulation_kit.world — the perception RESULT types the primitives read.

The kit never imports a camera. This package is the narrow contract between
whatever produced an estimate (``d1-inference/scene``, the Isaac env server, an
ArUco detector, a hand-written script) and the primitives that plan against it.

    from manipulation_kit.world import ObjectView, WorldView, ArmView

    world = WorldView.of(
        [ObjectView("red_block", p=(0.40, 0.10, 0.78), size=(0.05, 0.05, 0.05))],
        arms=[ArmView("left", joints=q_left), ArmView("right", joints=q_right)])
    print(world.to_text())

See :mod:`.frames` for why a pose carries its frame and what happens when that
frame is stale, and :mod:`.views` for why ``size`` has no default.
"""

from .frames import (BASE, FRAME_STALE, FUTURE_TOL_S, UNKNOWN_FRAME, Frame,
                     FrameError, FrameGraph)
from .direction import (ALIASES, FRAMES, OBJECT_PREFIX, TOOL, Direction,
                        frame_rotation, object_frame, parse_direction, toward)
from .views import (INFERRED, PROVENANCES, UPRIGHT_TOL_RAD, ArmView,
                    ContactView, ContainerView, GripperView, ObjectView,
                    SurfaceView, WorldView)
from .attach import (GraspTransform, attached, grasp_transform, released,
                     with_attached, with_measured_width)

__all__ = [
    "ALIASES", "Direction", "FRAMES", "OBJECT_PREFIX", "TOOL",
    "frame_rotation", "object_frame", "parse_direction", "toward",
    "BASE", "FRAME_STALE", "FUTURE_TOL_S", "UNKNOWN_FRAME", "UPRIGHT_TOL_RAD",
    "Frame", "FrameError", "FrameGraph",
    "ObjectView", "ContainerView", "SurfaceView",
    "ArmView", "GripperView", "WorldView", "ContactView",
    "INFERRED", "PROVENANCES",
    "GraspTransform", "attached", "grasp_transform", "released",
    "with_attached", "with_measured_width",
]
