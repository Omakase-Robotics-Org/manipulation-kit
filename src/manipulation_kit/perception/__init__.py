"""manipulation_kit.perception — cameras as geometry, and the Perceiver contract.

The kit still opens no camera and decodes no image: a PERCEIVER (a model, a
RealSense, an ArUco rig) does, and hands the kit pixels and declarations. What
lives here is everything that needs no model to be right:

    camera.py    PinholeCamera, CameraPose, Located (kind: contact | centre),
                 contact_to_centre, the propagated uncertainty
    head.py      HeadCamera on description.head_camera, from a typed
                 HeadCameraConfig (neck / lift state from the executor)
    wrist.py     WristCamera on the parallel gripper's plate mount
    plane.py     the table plane from one frame; the height policy
    measure.py   detections -> scene objects; the scene file
    protocol.py  Perceiver, ScenePerceiver, lift_onto_support

    from manipulation_kit.perception import HeadCamera
    camera = HeadCamera.from_robot(width=640, height=480, neck_pitch=0.512)
    camera.locate(320, 400, plane_z=0.166).to_text()

Everything is NOMINAL (``calibrated: false``) and says so; see
:mod:`.camera` for the one thing a single view cannot measure.
"""

from .camera import (AIM_UNCERTAINTY_DEG, DEFAULT_FX, GRAZING_SIN,
                     LENS_UNCERTAINTY_M, LOCATED_KINDS,
                     PROVISIONAL_UNCERTAINTY_M, CameraPose, Located,
                     MountSpread, NotOnThePlane, PinholeCamera,
                     contact_to_centre, provisional_table_z, read_intrinsics)
from .head import HeadCamera, HeadCameraConfig, HeadPoseUnknown, read_head_state
# ``measure`` the FUNCTION is not re-exported: ``perception.measure`` is the
# module, and a package attribute that is sometimes one and sometimes the
# other is a trap. ``from manipulation_kit.perception.measure import measure``.
from .measure import (ConfidenceLadder, Detection, FrameResult, build_scene,
                      detect_objects_mask, perceive_frame, requested_objects,
                      table_object)
from .plane import (HEIGHT_SOURCES, PlaneFitError, TableInBase, TablePlane,
                    fit_table_plane, project_corners, table_in_base,
                    table_z_from_known_length)
from .protocol import (CameraModel, LiftStateLike, NeckStateLike, NoSupport,
                       Perceiver, ScenePerceiver, lift_onto_support,
                       support_plane)
from .wrist import InFrame, WristCamera

__all__ = [
    "AIM_UNCERTAINTY_DEG", "DEFAULT_FX", "GRAZING_SIN", "HEIGHT_SOURCES",
    "LENS_UNCERTAINTY_M", "LOCATED_KINDS", "PROVISIONAL_UNCERTAINTY_M",
    "CameraModel", "CameraPose", "ConfidenceLadder", "Detection",
    "FrameResult", "HeadCamera", "HeadCameraConfig", "HeadPoseUnknown",
    "InFrame", "LiftStateLike", "Located", "MountSpread", "NeckStateLike",
    "NoSupport", "NotOnThePlane", "Perceiver", "PinholeCamera",
    "PlaneFitError", "ScenePerceiver", "TableInBase", "TablePlane",
    "WristCamera", "build_scene", "contact_to_centre", "detect_objects_mask",
    "fit_table_plane", "lift_onto_support", "perceive_frame",
    "project_corners", "provisional_table_z", "read_head_state",
    "read_intrinsics", "requested_objects", "support_plane", "table_in_base",
    "table_object", "table_z_from_known_length",
]
