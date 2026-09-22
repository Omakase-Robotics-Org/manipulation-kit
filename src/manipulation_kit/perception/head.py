"""The D1 head camera: a :class:`~.camera.PinholeCamera` on the neck.

The pose comes from :mod:`manipulation_kit.description.head_camera`, read out
of the committed URDF, so it moves when the robot's description does. The
one fact consumers get wrong — the daemon's LOGICAL neck pitch is the negative
of the URDF ``neck_tilt`` joint — is resolved THERE
(:func:`~manipulation_kit.description.head_camera.neck_joints_from_state`) and
nowhere else; this module calls it.

Two ways in, and they are different on purpose:

* :meth:`HeadCamera.from_config` — the LIVE path. A typed
  :class:`HeadCameraConfig` carrying the stream's intrinsics and the neck and
  lift states the firmware executor reports (``neck_state()`` /
  ``lift_state()``). It FAILS CLOSED: no neck state, or a neck that is moving,
  is not a camera pose, and a default of "level" is exactly the silent
  override Astra review item 9 found.
* :meth:`HeadCamera.from_robot` — joint space, for a RECORDED frame whose
  neck angle somebody wrote down (the committed d1-2 fixtures, the CLI).

THE LIFT DOES NOT MOVE THE CAMERA IN ``base``: it sits below ``dual_base`` and
raises the base and the camera together. It only changes the floor's height,
:meth:`HeadCamera.floor_z`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from .camera import (AIM_UNCERTAINTY_DEG, DEFAULT_FX, LENS_UNCERTAINTY_M,
                     PinholeCamera)
from .protocol import LiftStateLike, NeckStateLike


class HeadPoseUnknown(ValueError):
    """The head camera's pose cannot be established from what was reported."""


def _check_finite(what: str, value) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{what} must be finite, got {value!r}")
    return number


@dataclass(frozen=True)
class HeadCameraConfig:
    """Everything a head camera is, typed: the stream and the robot's joints.

    ``neck`` / ``lift`` are the firmware executor's ``neck_state()`` /
    ``lift_state()`` (any object with those attributes — see
    :class:`~.protocol.NeckStateLike`); ``neck.pitch_rad`` is the daemon's
    LOGICAL pitch and is passed through UNFLIPPED.
    """

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    neck: Optional[NeckStateLike] = None
    lift: Optional[LiftStateLike] = None

    def __post_init__(self) -> None:
        for name in ("fx", "fy", "cx", "cy"):
            _check_finite(name, getattr(self, name))
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("fx and fy must be positive")
        if int(self.width) <= 0 or int(self.height) <= 0:
            raise ValueError("width and height must be positive")

    @classmethod
    def from_intrinsics(cls, intrinsics: Mapping[str, Any], *, width: int,
                        height: int, neck: Optional[NeckStateLike] = None,
                        lift: Optional[LiftStateLike] = None
                        ) -> "HeadCameraConfig":
        """From a ``read_intrinsics`` dict; a missing principal point is the
        image centre, a missing ``fy`` is ``fx`` (square pixels)."""
        fx = float(intrinsics.get("fx") or DEFAULT_FX)
        fy = intrinsics.get("fy")
        cx, cy = intrinsics.get("cx"), intrinsics.get("cy")
        return cls(fx=fx, fy=fx if fy is None else float(fy),
                   cx=float(width) / 2.0 if cx is None else float(cx),
                   cy=float(height) / 2.0 if cy is None else float(cy),
                   width=int(width), height=int(height), neck=neck, lift=lift)


def read_head_state(executor) -> Tuple[NeckStateLike, Optional[LiftStateLike]]:
    """The neck and lift states from an executor, or a refusal.

    Uses the executor's typed ``neck_state()`` / ``lift_state()`` capability —
    thin calls on the d1-firmwared OpenAPI client — and NEVER a raw HTTP read.
    An executor without the capability (``KinematicExecutor`` returns
    ``None``) has no neck to report, and the answer is a refusal rather than
    a level head: a default here is a camera aimed somewhere it is not.
    The lift is optional (it does not move the camera in ``base``).
    """
    reader = getattr(executor, "neck_state", None)
    if reader is None:
        raise HeadPoseUnknown(
            f"{type(executor).__name__} has no neck_state(): the head camera's "
            f"pose cannot be read from it (d1-firmwared's GET /v1/neck/state "
            f"through the generated client is the source)")
    neck = reader()
    if neck is None:
        raise HeadPoseUnknown(f"{type(executor).__name__}.neck_state() "
                              f"reported nothing; there is no neck to aim by")
    lift_reader = getattr(executor, "lift_state", None)
    lift = lift_reader() if lift_reader is not None else None
    return neck, lift


@dataclass
class HeadCamera(PinholeCamera):
    """The D1 head camera: intrinsics plus the lens pose for these joints."""

    neck_pitch: float = 0.0          # URDF neck_tilt, MOTOR sign (+ = down)
    neck_yaw: float = 0.0            # URDF neck_pan
    lift_m: Optional[float] = None

    # -- construction ------------------------------------------------------
    @classmethod
    def from_robot(cls, *, width: int, height: int, fx: float = DEFAULT_FX,
                   fy: Optional[float] = None,
                   cx: Optional[float] = None, cy: Optional[float] = None,
                   neck_pitch: float = 0.0, neck_yaw: float = 0.0,
                   lift_m: Optional[float] = None,
                   urdf_path: Optional[str] = None) -> "HeadCamera":
        """The head camera of a D1 whose neck JOINTS are where you say.

        ``neck_pitch`` is the URDF ``neck_tilt`` joint in radians, MOTOR sign:
        positive looks DOWN. A daemon state is the other sign — use
        :meth:`from_config` or :meth:`from_neck_state` for one.
        """
        from ..description.head_camera import (  # noqa: PLC0415
            head_camera_pose, nominal_tilt_rad)

        neck_pitch = _check_finite("neck_pitch", neck_pitch)
        neck_yaw = _check_finite("neck_yaw", neck_yaw)
        p, r = head_camera_pose(neck_pitch=neck_pitch, neck_yaw=neck_yaw,
                                urdf_path=urdf_path)
        notes = (
            f"NOMINAL head-camera frame: vendor geometry plus the head part's "
            f"{math.degrees(nominal_tilt_rad()):.0f} deg design tilt, lens at "
            f"the housing's front face. Not a per-robot extrinsic.",
            f"every point is +-{LENS_UNCERTAINTY_M * 1000:.0f} mm of lens "
            f"position and +-{AIM_UNCERTAINTY_DEG:.0f} deg of aim.",
            "the lift does not enter: it sits below `base` and raises the "
            "base and the camera together.")
        return cls(fx=float(fx), fy=None if fy is None else float(fy),
                   cx=float(width) / 2.0 if cx is None else float(cx),
                   cy=float(height) / 2.0 if cy is None else float(cy),
                   width=int(width), height=int(height),
                   p=np.asarray(p, dtype=float), r=r,
                   neck_pitch=neck_pitch, neck_yaw=neck_yaw,
                   lift_m=None if lift_m is None
                   else _check_finite("lift_m", lift_m),
                   notes=notes)

    @classmethod
    def from_neck_state(cls, state, **kwargs) -> "HeadCamera":
        """From a daemon neck state (a ``NeckState`` or a ``GET
        /v1/neck/state`` body); the sign flip is the description's."""
        from ..description.head_camera import (  # noqa: PLC0415
            neck_joints_from_state)
        tilt, pan = neck_joints_from_state(state)
        return cls.from_robot(neck_pitch=tilt, neck_yaw=pan, **kwargs)

    @classmethod
    def from_config(cls, config: HeadCameraConfig, *,
                    urdf_path: Optional[str] = None) -> "HeadCamera":
        """The LIVE head camera, from a typed config. Fails closed."""
        if config.neck is None:
            raise HeadPoseUnknown(
                "no neck state: the head camera's pose is unknown, and a "
                "default neck angle would aim it somewhere it is not")
        if getattr(config.neck, "moving", None) is True:
            raise HeadPoseUnknown(
                "the neck is MOVING: a frame taken now has no single pose")
        lift_m = None
        if config.lift is not None:
            lift_m = _check_finite("lift.height_m", config.lift.height_m)
        return cls.from_neck_state(
            config.neck, width=config.width, height=config.height,
            fx=config.fx, fy=config.fy, cx=config.cx, cy=config.cy,
            lift_m=lift_m, urdf_path=urdf_path)

    @classmethod
    def from_json(cls, block: Mapping[str, Any]) -> "HeadCamera":
        """Rebuild the camera a perceived scene recorded (:meth:`to_json`)."""
        width, height = block["image"]
        return cls(fx=float(block["fx"]),
                   fy=float(block.get("fy", block["fx"])),
                   cx=float(block["cx"]), cy=float(block["cy"]),
                   width=int(width), height=int(height),
                   p=np.asarray(block["p_base"], dtype=float),
                   r=R.from_quat(block["quat_xyzw"]),
                   neck_pitch=float(block.get("neck_pitch_rad", 0.0)),
                   neck_yaw=float(block.get("neck_yaw_rad", 0.0)),
                   lift_m=block.get("lift_m"),
                   calibrated=bool(block.get("calibrated", False)),
                   notes=tuple(block.get("notes", ())),
                   lens_uncertainty_m=float(block.get("lens_uncertainty_m",
                                                      LENS_UNCERTAINTY_M)),
                   aim_uncertainty_deg=float(block.get("aim_uncertainty_deg",
                                                       AIM_UNCERTAINTY_DEG)))

    # -- reporting ---------------------------------------------------------
    def to_json(self) -> Dict[str, Any]:
        out = super().to_json()
        out.update({"neck_pitch_rad": round(self.neck_pitch, 4),
                    "neck_yaw_rad": round(self.neck_yaw, 4),
                    "lift_m": self.lift_m})
        return out

    def to_text(self) -> str:
        """The camera, for a model's own prompt: all robot facts."""
        axis = self.r.as_matrix()[:, 2]
        down = math.degrees(math.asin(max(-1.0, min(1.0, -axis[2]))))
        return (
            f"HEAD CAMERA (nominal, not calibrated): {self.width}x"
            f"{self.height}, fx {self.fx:.0f} px, principal point "
            f"({self.cx:.0f}, {self.cy:.0f}). Lens at ({self.p[0]:.3f}, "
            f"{self.p[1]:.3f}, {self.p[2]:.3f}) m in base, looking "
            f"{down:.0f} deg below horizontal along ({axis[0]:+.2f}, "
            f"{axis[1]:+.2f}, {axis[2]:+.2f}) in base axes "
            f"(neck pitch {self.neck_pitch:+.2f} rad, yaw "
            f"{self.neck_yaw:+.2f}). Image +x is right, +y is down. "
            f"Positions derived from it are worth about "
            f"+-{self.lens_uncertainty_m * 1000:.0f} mm.")

    def floor_z(self) -> Optional[float]:
        """The floor's z in ``base``, when the lift height is known."""
        if self.lift_m is None:
            return None
        from ..description.head_camera import (  # noqa: PLC0415
            floor_to_base_m)
        return -floor_to_base_m(self.lift_m)


__all__ = ["HeadCamera", "HeadCameraConfig", "HeadPoseUnknown",
           "read_head_state"]
