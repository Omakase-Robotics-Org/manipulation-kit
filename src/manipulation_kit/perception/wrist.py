"""The D1 wrist camera: a :class:`~.camera.PinholeCamera` on the gripper.

The mount is ALREADY in the kit — the V2.0 arm-end plate in
:mod:`manipulation_kit.hands.d1.parallel_gripper.description`
(``CAMERA_MOUNT_XYZ_M``, ``CAMERA_TILT_RAD``, ``CAMERA_ARM_YAW_RAD``) and the
``wrist_camera_optical`` frame of ``gripper_with_camera.urdf`` — so this module
only composes it with the arm's forward kinematics:

    link7 --TCP_P/TCP_R--> flange (+z = approach) --yaw(side)--> gripper
    base_link --mount xyz, Rx(15 deg)--> wrist_camera --Rz(pi)--> optical

The per-arm clocking is robot composition, not URDF: both cameras sit on top
of the wrist, so the physical LEFT arm mounts the description yawed pi.

What it is for (design C.7, requirement 2): after an ``Approach``,
:meth:`WristCamera.project_object` says whether the object is in the wrist
frame and where — the "look before the stroke" a policy can require
(step 7) — and :meth:`~.camera.PinholeCamera.locate` on the same model turns
a wrist pixel into a base-frame correction.

NOMINAL, like the head camera: vendor plate geometry, a placeholder lens
position inside the module (see the URDF's comment), no per-robot extrinsic.
The intrinsics are NOT defaulted — a wide-angle UVC module's focal length is
the stream's, and nobody has written it down for this one; pass it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from .camera import NotOnThePlane, PinholeCamera

#: The wrist module's own slop: the lens sits somewhere inside a placeholder
#: 30 g box on the plate (``gripper_with_camera.urdf``), and the plate's
#: 15 deg face is machined but the module is clamped by hand.
WRIST_LENS_UNCERTAINTY_M = 0.010
WRIST_AIM_UNCERTAINTY_DEG = 3.0


def optical_in_flange(side: str) -> Tuple[np.ndarray, R]:
    """``(p, r)`` of the wrist camera's OPTICAL frame in the arm's flange
    (TCP) frame, for ``side``. Composition of the kit's own mount constants."""
    from ..hands.d1.parallel_gripper.description import (  # noqa: PLC0415
        CAMERA_ARM_YAW_RAD, CAMERA_MOUNT_XYZ_M, CAMERA_TILT_RAD)
    if side not in CAMERA_ARM_YAW_RAD:
        raise ValueError(f"side must be one of {sorted(CAMERA_ARM_YAW_RAD)}, "
                         f"got {side!r}")
    clock = R.from_euler("z", CAMERA_ARM_YAW_RAD[side])
    mount = R.from_euler("x", CAMERA_TILT_RAD) * R.from_euler("z", math.pi)
    return (clock.apply(np.asarray(CAMERA_MOUNT_XYZ_M, dtype=float)),
            clock * mount)


@dataclass(frozen=True)
class InFrame:
    """Where a point or an object lands in a camera image, if it does."""

    visible: bool
    u: Optional[float]
    v: Optional[float]
    depth_m: Optional[float]
    reason: str = ""

    def to_json(self) -> Dict[str, Any]:
        return {"visible": self.visible,
                "u": None if self.u is None else round(float(self.u), 1),
                "v": None if self.v is None else round(float(self.v), 1),
                "depth_m": None if self.depth_m is None
                else round(float(self.depth_m), 4),
                "reason": self.reason}


@dataclass
class WristCamera(PinholeCamera):
    """A wrist camera at the pose the arm's FK puts it."""

    side: str = "right"

    @classmethod
    def from_flange(cls, side: str, flange_p, flange_r: R, *, fx: float,
                    fy: float, cx: float, cy: float, width: int,
                    height: int) -> "WristCamera":
        """From the flange (TCP) pose in base, as ``tool_from_link7`` minus
        the tool offset gives it."""
        p_off, r_off = optical_in_flange(side)
        flange_p = np.asarray(flange_p, dtype=float).reshape(3)
        return cls(fx=float(fx), fy=float(fy), cx=float(cx), cy=float(cy),
                   width=int(width), height=int(height),
                   p=flange_p + flange_r.apply(p_off), r=flange_r * r_off,
                   side=side,
                   lens_uncertainty_m=WRIST_LENS_UNCERTAINTY_M,
                   aim_uncertainty_deg=WRIST_AIM_UNCERTAINTY_DEG,
                   notes=("NOMINAL wrist-camera frame: the V2.0 plate's mount "
                          "constants composed with the arm's FK; not a "
                          "per-robot extrinsic.",))

    @classmethod
    def from_kin(cls, kin, side: str, **intrinsics) -> "WristCamera":
        """At the arm's CURRENT joints in ``kin`` (an ``ArmKinematics``)."""
        from ..primitives.orientation import TCP_P, TCP_R  # noqa: PLC0415
        p7, r7 = kin.ee_pose(side)
        flange_p = np.asarray(p7, dtype=float) + r7.apply(TCP_P)
        return cls.from_flange(side, flange_p, r7 * TCP_R, **intrinsics)

    def project_point(self, p_base) -> InFrame:
        """Where a base-frame point lands, and whether that is in the image."""
        try:
            u, v = self.project(p_base)
        except NotOnThePlane:
            return InFrame(False, None, None, None, "behind the camera")
        depth = float(self.r.inv().apply(np.asarray(p_base, dtype=float)
                                         - self.p)[2])
        if not self.in_image(u, v):
            return InFrame(False, u, v, depth, "outside the image")
        return InFrame(True, u, v, depth, "")

    def project_object(self, obj, frames) -> InFrame:
        """Is this object in the wrist frame, and where is its centre?

        ``visible`` means the CENTRE projects inside the image in front of
        the lens. Occlusion is not modelled — this is geometry, and the
        photograph is what confirms it.
        """
        p, _r = obj.pose_in_base(frames)
        return self.project_point(p)

    def to_json(self) -> Dict[str, Any]:
        out = super().to_json()
        out["side"] = self.side
        return out


__all__ = ["InFrame", "WRIST_AIM_UNCERTAINTY_DEG", "WRIST_LENS_UNCERTAINTY_M",
           "WristCamera", "optical_in_flange"]
