"""The head camera as a model: pixels <-> base-frame points, and nothing else.

THE ONLY CALIBRATION IN THIS DIRECTORY IS ROBOT-SPECIFIC. That is the rule
Shu set on 2026-09-22 after reading the first version of this work
(「中途半端にこっちでシーンごとの calib をするのは消したい」): a number that
belongs to *this robot* — its head-camera intrinsics, its neck joints, its
lift, the head part's mount tilt — is a calibration and may live here. A
number that belongs to *the scene in front of it* — how wide that wagon is,
how far away its far edge is, how high its top sits — is not a calibration,
it is a measurement somebody has to redo every time the furniture moves, and
it must not be an input.

So this file knows the camera and the robot, and knows nothing about tables.

    fx, cx, cy          the stream's intrinsics
    neck_pitch/yaw      where the head is pointing (GET /v1/neck/state)
    lift                the column, for a FLOOR-relative answer only

and from ``manipulation_kit.description.head_camera``, the pose of the lens in
``base`` for those joints — read out of the committed URDF, so it moves when
the robot's description does.

What that buys, and it is the whole of it: a pixel is a RAY in ``base``, and a
ray plus a HORIZONTAL PLANE AT A KNOWN HEIGHT is a point. The height is the
one thing this file will not invent — it is an argument, and the caller says
where it got it (the model declared it, a known length on the table fixed it,
or it is the robot-derived provisional one below, which is flagged as such).

WHAT A SINGLE VIEW CANNOT DO, stated once, here, because every caller inherits
it: one camera cannot measure the HEIGHT of the plane it is looking at. The
image of a horizontal plane is identical for every height once the scale is
free — twice as far and twice as big is the same picture — so the scale has to
come from somewhere outside the geometry. In this loop it comes from the
model, which is shown the robot's own hands at known base-frame positions in
the same picture and can say how high the table is relative to them.

THE POSE IS NOMINAL. It is vendor geometry plus the head part's design tilt,
not a per-robot extrinsic (see ``manipulation_kit.description.head_camera``),
so every point this returns carries an UNCERTAINTY computed from the two
documented unknowns — the lens position inside its housing and the aim — and
everything built on it says ``calibrated: false``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

#: The d1 head camera at 640x480. A focal length belongs to the STREAM, not to
#: the robot, so it is a default and not a constant — pass ``--fx`` or an
#: intrinsics file for anything else.
DEFAULT_FX = 606.0

#: How far the real lens may sit from where the nominal frame puts it. The
#: head housing is a 90 mm multi-sensor bar and the frame is placed at the
#: centre of its front face, so "a couple of centimetres" is the documented
#: figure (``description/d1/tools/generate_d1_urdf.py``, CAMERAS).
LENS_UNCERTAINTY_M = 0.020

#: ...and how far the aim may be off. The d1-3 head ArUco calibration put the
#: optical axis 12.2-12.8 deg below head_link forward against a 15 deg design
#: tilt, with -1.3...-1.9 deg of lateral yaw; the neck's motor zero moves
#: after a home on top of that.
AIM_UNCERTAINTY_DEG = 3.0

#: Below this angle between the ray and the plane, a pixel stops being a
#: position. 20 degrees, where 3 degrees of aim slop is already a quarter of a
#: metre on the table — the far edge of a wagon seen with the neck level. The
#: flag exists because the number alone does not say WHY it got big.
GRAZING_SIN = math.sin(math.radians(20.0))


@dataclass(frozen=True)
class Located:
    """One pixel, turned into a base-frame point, with what it is worth."""

    p: np.ndarray                    # base metres
    uncertainty_m: float             # 1-sigma-ish, from the nominal mount
    plane_z: float
    plane_source: str
    grazing: bool = False            # the ray meets the plane nearly flat

    def to_json(self) -> Dict[str, Any]:
        return {"p": [round(float(v), 4) for v in self.p],
                "uncertainty_m": round(float(self.uncertainty_m), 4),
                "plane_z": round(float(self.plane_z), 4),
                "plane_source": self.plane_source,
                "grazing": bool(self.grazing)}

    def to_text(self) -> str:
        note = (" — GRAZING: the ray meets the plane at a shallow angle, so a "
                "pixel of error is centimetres of position" if self.grazing
                else "")
        return (f"({self.p[0]:.3f}, {self.p[1]:.3f}, {self.p[2]:.3f}) m in "
                f"base, +-{self.uncertainty_m * 1000:.0f} mm, on the plane "
                f"z={self.plane_z:.3f} ({self.plane_source}){note}")


class NotOnThePlane(ValueError):
    """The pixel's ray never reaches the plane — it is above the horizon."""


@dataclass
class HeadCamera:
    """Intrinsics plus the lens pose in ``base``. Robot facts only."""

    fx: float
    cx: float
    cy: float
    width: int
    height: int
    p: np.ndarray                    # lens origin in base
    r: R                             # base <- optical rotation
    neck_pitch: float = 0.0
    neck_yaw: float = 0.0
    lift_m: Optional[float] = None
    calibrated: bool = False
    notes: Tuple[str, ...] = field(default_factory=tuple)

    # -- construction ------------------------------------------------------
    @classmethod
    def from_robot(cls, *, width: int, height: int, fx: float = DEFAULT_FX,
                   cx: Optional[float] = None, cy: Optional[float] = None,
                   neck_pitch: float = 0.0, neck_yaw: float = 0.0,
                   lift_m: Optional[float] = None,
                   urdf_path: Optional[str] = None) -> "HeadCamera":
        """The head camera of a D1 whose neck is where you say it is.

        ``neck_pitch`` is the URDF ``neck_tilt`` joint in radians, MOTOR sign:
        positive looks DOWN. ``GET /v1/neck/state`` reports the stack's
        LOGICAL pitch, which is its negative — use
        :meth:`from_neck_state` rather than doing that flip by hand.
        """
        from manipulation_kit.description.head_camera import (  # noqa: PLC0415
            head_camera_pose, nominal_tilt_rad)

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
        return cls(fx=float(fx),
                   cx=float(width) / 2.0 if cx is None else float(cx),
                   cy=float(height) / 2.0 if cy is None else float(cy),
                   width=int(width), height=int(height),
                   p=np.asarray(p, dtype=float), r=r,
                   neck_pitch=float(neck_pitch), neck_yaw=float(neck_yaw),
                   lift_m=None if lift_m is None else float(lift_m),
                   notes=notes)

    @classmethod
    def from_neck_state(cls, state: Dict[str, float], **kwargs) -> "HeadCamera":
        """From a ``GET /v1/neck/state`` body, sign flip included."""
        return cls.from_robot(neck_pitch=-float(state.get("pitch", 0.0)),
                              neck_yaw=float(state.get("yaw", 0.0)), **kwargs)

    # -- the model ---------------------------------------------------------
    @property
    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx],
                         [0.0, self.fx, self.cy],
                         [0.0, 0.0, 1.0]])

    def ray(self, u: float, v: float) -> np.ndarray:
        """Unit direction in BASE of the ray through pixel ``(u, v)``."""
        direction = self.r.apply([(float(u) - self.cx) / self.fx,
                                  (float(v) - self.cy) / self.fx, 1.0])
        return direction / float(np.linalg.norm(direction))

    def project(self, p_base) -> Tuple[float, float]:
        """The pixel a base-frame point lands on (no clipping, no distortion)."""
        local = self.r.inv().apply(np.asarray(p_base, dtype=float) - self.p)
        if local[2] <= 1e-6:
            raise NotOnThePlane("that point is behind the camera")
        return (self.cx + self.fx * local[0] / local[2],
                self.cy + self.fx * local[1] / local[2])

    def locate(self, u: float, v: float, *, plane_z: float,
               plane_source: str = "declared") -> Located:
        """Pixel -> the base-frame point where its ray meets ``z = plane_z``.

        Deterministic, and the only thing it will not do is guess the height.
        The uncertainty is PROPAGATED rather than quoted: the lens is moved by
        :data:`LENS_UNCERTAINTY_M` and the aim by :data:`AIM_UNCERTAINTY_DEG`
        in the directions that move this particular pixel the most, and the
        answer is how far the point goes. A pixel near the horizon therefore
        reports metres, which is correct and is the thing a flat number would
        have hidden.
        """
        direction = self.ray(u, v)
        if direction[2] > -1e-6:
            raise NotOnThePlane(
                f"pixel ({u:.0f}, {v:.0f}) looks at or above the horizon; its "
                f"ray never reaches the plane z={plane_z:.3f}")
        distance = (float(plane_z) - float(self.p[2])) / direction[2]
        point = self.p + direction * distance
        spread = self._spread(u, v, plane_z=float(plane_z))
        return Located(p=point, uncertainty_m=spread, plane_z=float(plane_z),
                       plane_source=plane_source,
                       grazing=bool(abs(direction[2]) < GRAZING_SIN))

    def _spread(self, u: float, v: float, *, plane_z: float) -> float:
        """How far :meth:`locate` moves when the nominal mount is wrong."""
        base = _intersect(self.p, self.r, u, v, plane_z,
                          fx=self.fx, cx=self.cx, cy=self.cy)
        if base is None:
            return float("inf")
        worst = 0.0
        tilt = math.radians(AIM_UNCERTAINTY_DEG)
        for axis in np.eye(3):
            for sign in (1.0, -1.0):
                moved = _intersect(self.p + axis * sign * LENS_UNCERTAINTY_M,
                                   R.from_rotvec(axis * sign * tilt) * self.r,
                                   u, v, plane_z,
                                   fx=self.fx, cx=self.cx, cy=self.cy)
                if moved is None:
                    return float("inf")
                worst = max(worst, float(np.linalg.norm(moved - base)))
        return worst

    # -- reporting ---------------------------------------------------------
    def to_json(self) -> Dict[str, Any]:
        return {"fx": round(self.fx, 2), "cx": round(self.cx, 2),
                "cy": round(self.cy, 2),
                "image": [self.width, self.height],
                "p_base": [round(float(v), 4) for v in self.p],
                "quat_xyzw": [round(float(v), 5) for v in self.r.as_quat()],
                "neck_pitch_rad": round(self.neck_pitch, 4),
                "neck_yaw_rad": round(self.neck_yaw, 4),
                "lift_m": self.lift_m,
                "calibrated": self.calibrated,
                "lens_uncertainty_m": LENS_UNCERTAINTY_M,
                "aim_uncertainty_deg": AIM_UNCERTAINTY_DEG,
                "notes": list(self.notes)}

    def to_text(self) -> str:
        """The camera, for the model's own prompt.

        The model is a geometry engine with a picture; this is the rest of
        what it needs to turn one into the other, and the numbers are all
        robot facts.
        """
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
            f"+-{LENS_UNCERTAINTY_M * 1000:.0f} mm.")

    def floor_z(self) -> Optional[float]:
        """The floor's z in ``base``, when the lift height is known."""
        if self.lift_m is None:
            return None
        from manipulation_kit.description.head_camera import (  # noqa: PLC0415
            floor_to_base_m)
        return -floor_to_base_m(self.lift_m)


def _intersect(p, r: R, u: float, v: float, plane_z: float, *, fx: float,
               cx: float, cy: float):
    """One ray-plane intersection, with the pose passed in.

    A free function rather than a method so :meth:`HeadCamera._spread` can
    intersect with a MOVED pose without building six ``HeadCamera`` objects
    per pixel.
    """
    direction = r.apply([(u - cx) / fx, (v - cy) / fx, 1.0])
    direction = direction / float(np.linalg.norm(direction))
    if direction[2] > -1e-6:
        return None
    p = np.asarray(p, dtype=float)
    return p + direction * ((plane_z - float(p[2])) / direction[2])


#: A table height to start from when NOBODY has said one yet: the z of the
#: arms' HOME tool points. It is a robot fact — where this robot's hands rest
#: — and it is the right order of magnitude for a surface it is built to work
#: on (0.201 m; the JP wagon the loop was written against is 0.166 m). It is
#: NOT a measurement of anything in front of the camera, it is flagged
#: ``provisional`` everywhere it appears, and the model's first job in the
#: loop is to replace it with ``declare_scene``.
PROVISIONAL_UNCERTAINTY_M = 0.10


def provisional_table_z(kin=None) -> float:
    """Where to put the plane before anyone has measured it. See above."""
    from manipulation_kit.primitives.orientation import (  # noqa: PLC0415
        tool_from_link7)
    if kin is None:
        from manipulation_kit.arms import get_arm_kinematics  # noqa: PLC0415
        kin = get_arm_kinematics("d1/arm", quiet=True)
    heights = []
    for side in ("left", "right"):
        saved = np.array(kin.joints(side), dtype=float)
        try:
            kin.set_joints(side, kin.home(side))
            heights.append(float(tool_from_link7(*kin.ee_pose(side))[0][2]))
        finally:
            kin.set_joints(side, saved)
    return float(np.mean(heights))


def read_intrinsics(path) -> Dict[str, float]:
    """``fx`` / ``cx`` / ``cy`` out of a camera JSON, in the shapes that turn up.

    d1-inference writes ``{"fx": …, "cx": …}``; a ROS ``camera_info`` dump
    writes ``{"K": [...9]}`` or ``{"camera_matrix": {"data": [...9]}}``. Only
    those three are read — no distortion model, because nothing here undoes
    one and pretending otherwise would be worse than the 2 mm it costs at this
    field of view.
    """
    import json  # noqa: PLC0415
    from pathlib import Path  # noqa: PLC0415

    blob = json.loads(Path(path).read_text(encoding="utf-8"))
    out: Dict[str, float] = {}
    for key in ("fx", "focal_length_px"):
        if isinstance(blob.get(key), (int, float)):
            out["fx"] = float(blob[key])
            break
    for key, name in (("cx", "cx"), ("cy", "cy"),
                      ("ppx", "cx"), ("ppy", "cy")):
        if isinstance(blob.get(key), (int, float)):
            out.setdefault(name, float(blob[key]))
    if "fx" not in out:
        for key in ("K", "camera_matrix", "intrinsic_matrix"):
            value = blob.get(key)
            if isinstance(value, dict):
                value = value.get("data")
            if isinstance(value, list) and len(value) >= 9:
                flat = np.asarray(value, dtype=float).ravel()
                out["fx"] = float(flat[0])
                out.setdefault("cx", float(flat[2]))
                out.setdefault("cy", float(flat[5]))
                break
    if "fx" not in out:
        raise ValueError(f"{path}: no fx, K or camera_matrix in this file")
    return out


__all__ = ["DEFAULT_FX", "LENS_UNCERTAINTY_M", "AIM_UNCERTAINTY_DEG",
           "GRAZING_SIN",
           "PROVISIONAL_UNCERTAINTY_M", "HeadCamera", "Located",
           "NotOnThePlane", "provisional_table_z", "read_intrinsics"]
