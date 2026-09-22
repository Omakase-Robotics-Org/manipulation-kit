"""The D1 wrist camera: a :class:`~.camera.PinholeCamera` on the gripper —
with the FISHEYE model its lens actually has, when it has been measured.

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
the stream's; pass it, from the robot's profile
(:class:`manipulation_kit.description.robot_profile.WristIntrinsics`).

THE LENS MODEL. The d1 wrist module is a ~150-170 deg fisheye. A pinhole
model of it is ~10 % off at 30 deg off-axis and far worse at the edge — which
is exactly where a look before the stroke looks. With ``model="fisheye"`` the
camera projects and unprojects through the OpenCV ``cv2.fisheye`` model
(equidistant: ``theta_d = theta (1 + k1 theta^2 + k2 theta^4 + k3 theta^6 +
k4 theta^8)``), implemented here in numpy — the kit takes no OpenCV
dependency — with a fixed-point inverse for unprojection. ``valid_radius_px``
is how far from the principal point the calibration's board coverage
reached: a pixel outside it is reported as not in the image rather than
trusted.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from .camera import NotOnThePlane, PinholeCamera, _finite

#: The wrist module's own slop: the lens sits somewhere inside a placeholder
#: 30 g box on the plate (``gripper_with_camera.urdf``), and the plate's
#: 15 deg face is machined but the module is clamped by hand.
WRIST_LENS_UNCERTAINTY_M = 0.010
WRIST_AIM_UNCERTAINTY_DEG = 3.0


#: fixed-point iterations of the fisheye inverse; the d1-2 lenses converge to
#: 1e-12 rad in under ten
FISHEYE_ITERATIONS = 30


def fisheye_distort(theta, k) -> np.ndarray:
    """The equidistant model's distorted angle for an incidence ``theta``."""
    t = np.asarray(theta, dtype=float)
    t2 = t * t
    k1, k2, k3, k4 = (float(c) for c in k)
    return t * (1.0 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4))))


def fisheye_undistort(theta_d, k) -> np.ndarray:
    """The incidence angle that distorts to ``theta_d`` — fixed point, as
    OpenCV's ``undistortPoints`` for ``cv2.fisheye`` does."""
    td = np.asarray(theta_d, dtype=float)
    theta = td.copy()
    k1, k2, k3, k4 = (float(c) for c in k)
    for _ in range(FISHEYE_ITERATIONS):
        t2 = theta * theta
        theta = td / (1.0 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4))))
    return theta


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
    #: ``"pinhole"`` or ``"fisheye"`` (OpenCV ``cv2.fisheye``, equidistant)
    model: str = "pinhole"
    #: the fisheye's k1..k4 (empty for a pinhole)
    k: Tuple[float, ...] = ()
    #: pixels farther than this from the principal point are not trusted
    valid_radius_px: Optional[float] = None

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.model not in ("pinhole", "fisheye"):
            raise ValueError(f"model must be pinhole or fisheye, got "
                             f"{self.model!r}")
        self.k = tuple(float(c) for c in self.k)
        if self.model == "fisheye" and len(self.k) != 4:
            raise ValueError(f"a fisheye needs k = [k1, k2, k3, k4], got "
                             f"{list(self.k)}")
        _finite("fisheye coefficients", *self.k)

    @classmethod
    def from_flange(cls, side: str, flange_p, flange_r: R, *, fx: float,
                    fy: float, cx: float, cy: float, width: int,
                    height: int, model: str = "pinhole",
                    k: Tuple[float, ...] = (),
                    valid_radius_px: Optional[float] = None,
                    **_ignored: Any) -> "WristCamera":
        """From the flange (TCP) pose in base, as ``tool_from_link7`` minus
        the tool offset gives it. The intrinsics are a
        :meth:`~manipulation_kit.description.robot_profile.WristIntrinsics.camera_kwargs`
        (extra provenance keys such as ``source`` are ignored)."""
        p_off, r_off = optical_in_flange(side)
        flange_p = np.asarray(flange_p, dtype=float).reshape(3)
        lens = ("pinhole intrinsics" if model == "pinhole" else
                "MEASURED fisheye intrinsics (cv2.fisheye, equidistant)")
        return cls(fx=float(fx), fy=float(fy), cx=float(cx), cy=float(cy),
                   width=int(width), height=int(height),
                   p=flange_p + flange_r.apply(p_off), r=flange_r * r_off,
                   side=side, model=str(model), k=tuple(k),
                   valid_radius_px=(None if valid_radius_px is None
                                    else float(valid_radius_px)),
                   lens_uncertainty_m=WRIST_LENS_UNCERTAINTY_M,
                   aim_uncertainty_deg=WRIST_AIM_UNCERTAINTY_DEG,
                   notes=("NOMINAL wrist-camera frame: the V2.0 plate's mount "
                          "constants composed with the arm's FK; not a "
                          "per-robot extrinsic.", f"lens: {lens}."))

    # -- the lens ------------------------------------------------------------
    def _local_ray(self, u: float, v: float) -> np.ndarray:
        if self.model != "fisheye":
            return super()._local_ray(u, v)
        _finite("a pixel", u, v)
        xd = (float(u) - self.cx) / self.fx
        yd = (float(v) - self.cy) / float(self.fy)
        theta_d = math.hypot(xd, yd)
        if theta_d < 1e-12:
            return np.array([0.0, 0.0, 1.0])
        theta = float(fisheye_undistort(theta_d, self.k))
        if not 0.0 <= theta < math.pi / 2.0 - 1e-9:
            raise NotOnThePlane(f"pixel ({u:.0f}, {v:.0f}) is at or past 90 "
                                f"deg off the lens axis")
        scale = math.tan(theta) / theta_d
        return np.array([xd * scale, yd * scale, 1.0])

    def project(self, p_base) -> Tuple[float, float]:
        if self.model != "fisheye":
            return super().project(p_base)
        point = np.asarray(p_base, dtype=float).reshape(3)
        if not np.all(np.isfinite(point)):
            raise ValueError(f"cannot project a non-finite point {point}")
        local = self.r.inv().apply(point - self.p)
        if local[2] <= 1e-6:
            raise NotOnThePlane("that point is behind the camera")
        a, b = local[0] / local[2], local[1] / local[2]
        radius = math.hypot(a, b)
        scale = (1.0 if radius < 1e-12 else
                 float(fisheye_distort(math.atan(radius), self.k)) / radius)
        return (self.cx + self.fx * a * scale,
                self.cy + float(self.fy) * b * scale)

    def in_calibrated_radius(self, u: float, v: float) -> bool:
        if self.valid_radius_px is None:
            return True
        return math.hypot(float(u) - self.cx,
                          float(v) - self.cy) <= float(self.valid_radius_px)

    def in_image(self, u: float, v: float) -> bool:
        return super().in_image(u, v) and self.in_calibrated_radius(u, v)

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
        if not PinholeCamera.in_image(self, u, v):
            return InFrame(False, u, v, depth, "outside the image")
        if not self.in_calibrated_radius(u, v):
            return InFrame(False, u, v, depth,
                           f"outside the calibrated radius "
                           f"({self.valid_radius_px:.0f} px): the lens model "
                           f"is not trusted there")
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
        out["model"] = self.model
        if self.model == "fisheye":
            out["k"] = [float(c) for c in self.k]
        if self.valid_radius_px is not None:
            out["valid_radius_px"] = float(self.valid_radius_px)
        return out


__all__ = ["FISHEYE_ITERATIONS", "InFrame", "WRIST_AIM_UNCERTAINTY_DEG",
           "WRIST_LENS_UNCERTAINTY_M", "WristCamera", "fisheye_distort",
           "fisheye_undistort", "optical_in_flange"]
