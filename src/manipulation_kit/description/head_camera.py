"""Where the head camera is, in the frame plans are written in.

The D1's head camera has had a FRAME in this package since the whole-body
URDF gained cameras: ``head_camera_link`` on the neck-tilt link and its ROS
optical child ``head_camera_optical_frame`` (see the CAMERAS block in
``d1/tools/generate_d1_urdf.py`` for where the direction and the mount tilt
come from). What it did not have is a way to ASK for that frame from Python
without loading a URDF yourself and knowing which two joints to set, so every
consumer that wanted "the camera pose in base" wrote its own chain — and a
hand-written chain is exactly the thing that silently disagrees with the asset
after a head revision.

This module is that accessor and nothing more. It reads the committed
whole-body URDF through the package's own parser
(:class:`manipulation_kit.guard.urdf_model.UrdfModel`), so it CANNOT drift
from the shipped geometry: change the head part, regenerate, and this moves
with it.

    >>> from manipulation_kit.description.head_camera import head_camera_pose
    >>> p, r = head_camera_pose(neck_pitch=0.30)
    >>> round(float(p[2]), 3)            # metres above the torso platform
    0.718

WHAT THE NUMBERS ARE, AND ARE NOT
---------------------------------
NOMINAL. The frame is vendor geometry plus the head part's DESIGN tilt
(:data:`manipulation_kit.description.HEAD_CAMERA_TILT_DEG`); it is not a
calibrated extrinsic, and two things in it are known-unknown — the lens
position inside the 90 mm multi-sensor housing (good to a couple of
centimetres) and the roll about the lens axis. The d1-3 head ArUco fit sits
~2.5 deg off the nominal pitch. Anything that needs pixel accuracy must use
that robot's own ``cameras_<robot>.json``; anything that says "calibrated"
while using this is lying, which is why every consumer in this repository
that uses it stamps ``calibrated: false`` on its output.

LIFT DOES NOT MOVE THE CAMERA IN ``base``
-----------------------------------------
:data:`manipulation_kit.world.BASE` is ``dual_base``, the torso platform, and
the ``lift`` joint sits BELOW it (``base_footprint`` -> ``dual_base``). So the
column raises the base and the camera together and the pose returned here is
independent of the lift — a fact worth stating, because the obvious
expectation is the opposite and a caller that "corrects" for the lift is
wrong by up to 300 mm. The lift only matters when you want a height above the
FLOOR, which is what :func:`floor_to_base_m` is for.

SIGN OF ``neck_pitch``
----------------------
The URDF ``neck_tilt`` joint is the MOTOR frame: POSITIVE PITCHES THE HEAD
DOWN, limits [-0.35, +0.65] rad. That is the same convention as the neck
safety source of truth (``omakase_neck/src/limits.py``) and the OPPOSITE sign
of the *logical* pitch ``omakase_neck/config/config.yaml`` exposes
(logical = -motor). ``d1-firmwared``'s ``GET /v1/neck/state`` reports the
stack's logical pitch, so pass ``neck_pitch=-state["pitch"]`` — or call
:func:`pose_from_neck_state`, which does it for you and is the reason this
module would rather you did not do the arithmetic by hand.
"""
from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from . import (DEFAULT_HARDWARE_REVISION, WHOLEBODY_GRIPPER_URDF,
               head_camera_tilt_deg)

#: The frame plans are written in; the root this module reports poses in.
BASE_LINK = "dual_base"

#: The camera body frame (lens looks along its +x) and its ROS optical child
#: (+z out of the lens, +x image right, +y image down).
CAMERA_LINK = "head_camera_link"
OPTICAL_LINK = "head_camera_optical_frame"

#: URDF joint names this module actuates. Everything else in the chain is
#: fixed geometry.
PAN_JOINT = "neck_pan"
TILT_JOINT = "neck_tilt"

_MODEL_CACHE: Dict[str, object] = {}


def _model(urdf_path: Optional[str] = None):
    """The parsed whole-body URDF, once per path.

    Parsing costs a few milliseconds and a perception example asks for the
    pose per frame, so it is cached — but keyed by PATH, because a consumer
    building its own asset for another hardware revision passes that asset in
    and must not be served this one.
    """
    from ..guard.urdf_model import UrdfModel  # noqa: PLC0415

    key = str(urdf_path or WHOLEBODY_GRIPPER_URDF)
    if key not in _MODEL_CACHE:
        _MODEL_CACHE[key] = UrdfModel(key)
    return _MODEL_CACHE[key]


def _relative(model, parent: str, child: str, q_rad: Dict[str, float]):
    """``child``'s pose expressed in ``parent``, as ``(p, R)``."""
    for link in (parent, child):
        if link not in model.links:
            raise KeyError(
                f"{link!r} is not a link of {model.name!r} — this URDF does "
                f"not carry the head camera frame. The whole-body variants do "
                f"(cameras are deliberately absent from the guard model).")
    tfs = model.link_transforms(q_rad)
    a, b = tfs[parent], tfs[child]
    ra = np.array(a.R, dtype=float)
    rb = np.array(b.R, dtype=float)
    rel_r = ra.T @ rb
    rel_p = ra.T @ (np.array(b.t, dtype=float) - np.array(a.t, dtype=float))
    return rel_p, R.from_matrix(rel_r)


def head_camera_pose(neck_pitch: float = 0.0, neck_yaw: float = 0.0,
                     *, optical: bool = True,
                     urdf_path: Optional[str] = None) -> Tuple[np.ndarray, R]:
    """The head camera's pose in :data:`BASE_LINK`, from the committed URDF.

    ``neck_pitch`` / ``neck_yaw`` are the URDF ``neck_tilt`` / ``neck_pan``
    joint values in RADIANS (motor sign: positive pitch looks DOWN). With
    ``optical=True`` — the default, and what a pinhole model wants — the
    rotation is the ROS optical convention: +z out of the lens, +x image
    right, +y image down. With ``optical=False`` you get the camera BODY
    frame, whose +x is the lens axis.

    NOMINAL, not calibrated; see the module docstring.
    """
    model = _model(urdf_path)
    q = {TILT_JOINT: float(neck_pitch), PAN_JOINT: float(neck_yaw)}
    return _relative(model, BASE_LINK,
                     OPTICAL_LINK if optical else CAMERA_LINK, q)


def pose_from_neck_state(state: Dict[str, float], *, optical: bool = True,
                         urdf_path: Optional[str] = None
                         ) -> Tuple[np.ndarray, R]:
    """:func:`head_camera_pose` from a ``GET /v1/neck/state`` body.

    The daemon reports the stack's LOGICAL pitch, which is the negative of the
    URDF joint. Doing that flip in one place is the point: a consumer that
    gets it wrong aims the camera 2x the neck angle away and the error looks
    like a bad calibration rather than like a sign.
    """
    return head_camera_pose(neck_pitch=-float(state.get("pitch", 0.0)),
                            neck_yaw=float(state.get("yaw", 0.0)),
                            optical=optical, urdf_path=urdf_path)


def floor_to_base_m(lift_m: float = 0.0, *,
                    urdf_path: Optional[str] = None) -> float:
    """Height of :data:`BASE_LINK` above the floor for a lift extension.

    The one thing the lift DOES change. ``lift_m`` is the slider extension in
    metres (``GET /v1/slider/state`` -> ``height_m``), and the return is what
    you add to a z in ``base`` to get a z above the ground: a table measured
    at z=0.18 in base with the column at 0.205 stands
    0.18 + 0.513 + 0.205 = 0.898 m off the floor.
    """
    model = _model(urdf_path)
    joint = next((j for j in model.joints.values() if j.name == "lift"), None)
    if joint is None:
        raise KeyError(f"{model.name!r} has no 'lift' joint")
    return float(joint.origin.t[2]) + float(lift_m)


def nominal_tilt_rad(revision: str = DEFAULT_HARDWARE_REVISION) -> float:
    """The head part's design tilt in radians, for the report line that says
    how far a measured correction has moved off nominal."""
    return math.radians(head_camera_tilt_deg(revision))


__all__ = ["BASE_LINK", "CAMERA_LINK", "OPTICAL_LINK", "PAN_JOINT",
           "TILT_JOINT", "head_camera_pose", "pose_from_neck_state",
           "floor_to_base_m", "nominal_tilt_rad"]
