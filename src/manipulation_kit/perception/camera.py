"""A pinhole camera as a model: pixels <-> base-frame points, and nothing else.

THE ONLY CALIBRATION IN THIS PACKAGE IS ROBOT-SPECIFIC. That is the rule
Shu set on 2026-09-22 after reading the first version of this work
(「中途半端にこっちでシーンごとの calib をするのは消したい」): a number that
belongs to *this robot* — a camera's intrinsics, its neck joints, its lift,
the head part's mount tilt, the wrist plate — is a calibration and may live
here. A number that belongs to *the scene in front of it* — how wide that
wagon is, how far away its far edge is, how high its top sits — is not a
calibration, it is a measurement somebody has to redo every time the
furniture moves, and it must not be an input.

So this module knows cameras, and knows nothing about tables.

What that buys, and it is the whole of it: a pixel is a RAY in ``base``, and a
ray plus a HORIZONTAL PLANE AT A KNOWN HEIGHT is a point. The height is the
one thing this module will not invent — it is an argument, and the caller says
where it got it (the model declared it, a known length on the table fixed it,
or it is the robot-derived provisional one below, which is flagged as such)
and how sure it is.

WHAT A SINGLE VIEW CANNOT DO, stated once, here, because every caller inherits
it: one camera cannot measure the HEIGHT of the plane it is looking at. The
image of a horizontal plane is identical for every height once the scale is
free — twice as far and twice as big is the same picture — so the scale has to
come from somewhere outside the geometry.

THE POSE IS NOMINAL. A camera built from the kit's description is vendor
geometry plus a design tilt, not a per-robot extrinsic, so every point this
returns carries an UNCERTAINTY propagated from the documented unknowns — the
lens position inside its housing, the aim, and the height of the plane — and
everything built on it says ``calibrated: false``. The uncertainty is a
propagated worst case over those perturbations, not a calibrated error bound.

The head camera is :mod:`.head`; the wrist camera is :mod:`.wrist`. This
module is the part they share.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Any, Dict, NamedTuple, Optional, Sequence, Tuple

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

#: A table height to start from when NOBODY has said one yet: the z of the
#: arms' HOME tool points. It is a robot fact — where this robot's hands rest
#: — and it is the right order of magnitude for a surface it is built to work
#: on (0.201 m; the JP wagon the loop was written against is 0.166 m). It is
#: NOT a measurement of anything in front of the camera, it is flagged
#: ``provisional`` everywhere it appears, and a model's first job is to
#: replace it with a declaration.
PROVISIONAL_UNCERTAINTY_M = 0.10

#: What a :class:`Located` point IS. ``contact``: where the pixel's ray meets
#: the plane — for the bottom of a silhouette that is the object's NEAR EDGE
#: on the surface, not its middle. ``centre``: an object centre, which only
#: :func:`contact_to_centre` produces, from a contact point and a size.
LOCATED_KINDS = ("contact", "centre")


class NotOnThePlane(ValueError):
    """The pixel's ray never reaches the plane in front of the lens.

    Either it looks at or above the horizon, or the plane is above the lens
    and the only intersection is BEHIND the camera — a perfectly
    plausible-looking number that is not a place.
    """


@dataclass(frozen=True)
class Located:
    """One pixel, turned into a base-frame point, with what it is worth.

    ``uncertainty_m`` is the TOTAL, ``hypot(mount, height)``: the mount part
    is how far the point moves when the nominal lens position and aim are
    wrong (independently, every combination), the height part is how far it
    moves when the plane is ``plane_uncertainty_m`` higher or lower. They are
    kept apart because they are corrected by different things — a per-robot
    extrinsic fixes the first, one declared table height fixes the second.
    """

    p: np.ndarray                    # base metres
    uncertainty_m: float             # 1-sigma-ish total, see above
    plane_z: float
    plane_source: str
    grazing: bool = False            # the ray meets the plane nearly flat
    kind: str = "contact"            # contact (a surface point) | centre
    mount_uncertainty_m: float = 0.0
    height_uncertainty_m: float = 0.0

    def __post_init__(self) -> None:
        if self.kind not in LOCATED_KINDS:
            raise ValueError(f"Located.kind must be one of {LOCATED_KINDS}, "
                             f"got {self.kind!r}")
        p = np.array(self.p, dtype=float).reshape(3)
        if not np.all(np.isfinite(p)):
            raise ValueError(f"Located.p must be finite, got {p.tolist()}")
        p.setflags(write=False)
        object.__setattr__(self, "p", p)

    def to_json(self) -> Dict[str, Any]:
        return {"p": [round(float(v), 4) for v in self.p],
                "kind": self.kind,
                "uncertainty_m": round(float(self.uncertainty_m), 4),
                "mount_uncertainty_m": round(float(self.mount_uncertainty_m), 4),
                "height_uncertainty_m": round(float(self.height_uncertainty_m),
                                              4),
                "plane_z": round(float(self.plane_z), 4),
                "plane_source": self.plane_source,
                "grazing": bool(self.grazing)}

    def to_text(self) -> str:
        note = (" — GRAZING: the ray meets the plane at a shallow angle, so a "
                "pixel of error is centimetres of position" if self.grazing
                else "")
        split = ""
        if self.height_uncertainty_m > 0.0:
            split = (f" ({self.mount_uncertainty_m * 1000:.0f} mm from the "
                     f"camera mount, {self.height_uncertainty_m * 1000:.0f} mm "
                     f"from the plane's height)")
        what = ("the CONTACT point where the ray meets the surface — for the "
                "bottom of a silhouette that is the object's near edge, not "
                "its centre" if self.kind == "contact"
                else "an object CENTRE, converted from a contact point")
        return (f"({self.p[0]:.3f}, {self.p[1]:.3f}, {self.p[2]:.3f}) m in "
                f"base, +-{self.uncertainty_m * 1000:.0f} mm{split}, on the "
                f"plane z={self.plane_z:.3f} ({self.plane_source}); {what}"
                f"{note}")


class MountSpread(NamedTuple):
    """How far one located point moves under each family of mount error."""

    translation_m: float     # lens position alone
    aim_m: float             # aim alone
    combined_m: float        # every (translation, aim) pair, independently


@dataclass(frozen=True)
class CameraPose:
    """Where a lens is: origin and ``base <- optical`` rotation."""

    p: np.ndarray
    r: R


def _finite(what: str, *values: float) -> None:
    for value in values:
        if not math.isfinite(float(value)):
            raise ValueError(f"{what} must be finite, got {value!r}")


@dataclass
class PinholeCamera:
    """Intrinsics plus the lens pose in ``base``. Robot facts only.

    ROS optical convention: +z out of the lens, +x image right, +y image down.
    No distortion model — nothing here undoes one, and pretending otherwise
    would be worse than the 2 mm it costs at the head camera's field of view.
    """

    fx: float
    cx: float
    cy: float
    width: int
    height: int
    p: np.ndarray                    # lens origin in base
    r: R                             # base <- optical rotation
    fy: Optional[float] = None       # None = square pixels (fy = fx)
    calibrated: bool = False
    notes: Tuple[str, ...] = field(default_factory=tuple)
    lens_uncertainty_m: float = LENS_UNCERTAINTY_M
    aim_uncertainty_deg: float = AIM_UNCERTAINTY_DEG

    def __post_init__(self) -> None:
        if self.fy is None:
            self.fy = self.fx
        _finite("camera intrinsics", self.fx, self.fy, self.cx, self.cy)
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError(f"focal lengths must be positive, got fx="
                             f"{self.fx!r} fy={self.fy!r}")
        if int(self.width) <= 0 or int(self.height) <= 0:
            raise ValueError("the image must have a positive size")
        self.p = np.array(self.p, dtype=float).reshape(3)
        if not np.all(np.isfinite(self.p)):
            raise ValueError(f"the lens position must be finite, got "
                             f"{self.p.tolist()}")
        if not isinstance(self.r, R):
            raise TypeError("r must be a scipy Rotation (base <- optical)")

    # -- the model ---------------------------------------------------------
    @property
    def pose(self) -> CameraPose:
        return CameraPose(p=np.array(self.p), r=self.r)

    @property
    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx],
                         [0.0, self.fy, self.cy],
                         [0.0, 0.0, 1.0]])

    def _local_ray(self, u: float, v: float) -> np.ndarray:
        _finite("a pixel", u, v)
        return np.array([(float(u) - self.cx) / self.fx,
                         (float(v) - self.cy) / self.fy, 1.0])

    def ray(self, u: float, v: float) -> np.ndarray:
        """Unit direction in BASE of the ray through pixel ``(u, v)``."""
        direction = self.r.apply(self._local_ray(u, v))
        return direction / float(np.linalg.norm(direction))

    def project(self, p_base) -> Tuple[float, float]:
        """The pixel a base-frame point lands on (no clipping, no distortion)."""
        point = np.asarray(p_base, dtype=float).reshape(3)
        if not np.all(np.isfinite(point)):
            raise ValueError(f"cannot project a non-finite point {point}")
        local = self.r.inv().apply(point - self.p)
        if local[2] <= 1e-6:
            raise NotOnThePlane("that point is behind the camera")
        return (self.cx + self.fx * local[0] / local[2],
                self.cy + self.fy * local[1] / local[2])

    def in_image(self, u: float, v: float) -> bool:
        return 0.0 <= float(u) < float(self.width) and \
            0.0 <= float(v) < float(self.height)

    def locate(self, u: float, v: float, *, plane_z: float,
               plane_source: str = "declared",
               plane_uncertainty_m: float = 0.0) -> Located:
        """Pixel -> the base-frame CONTACT point where its ray meets
        ``z = plane_z``.

        Deterministic, and the only thing it will not do is guess the height.
        It REFUSES, with :class:`NotOnThePlane`, a ray at or above the horizon
        and a plane above the lens (the intersection would be behind the
        camera), and with ``ValueError`` any non-finite input.

        The uncertainty is PROPAGATED rather than quoted, in two independent
        parts (see :class:`Located`): the mount — the lens moved by
        ``lens_uncertainty_m`` and the aim by ``aim_uncertainty_deg`` along
        each axis, every combination of the two — and the plane's height,
        moved by ``plane_uncertainty_m`` up and down. A pixel near the horizon
        therefore reports metres, which is correct and is the thing a flat
        number would have hidden.
        """
        _finite("locate's inputs", u, v, plane_z, plane_uncertainty_m)
        if plane_uncertainty_m < 0.0:
            raise ValueError("plane_uncertainty_m cannot be negative")
        direction = self.ray(u, v)
        if direction[2] > -1e-6:
            raise NotOnThePlane(
                f"pixel ({u:.0f}, {v:.0f}) looks at or above the horizon; its "
                f"ray never reaches the plane z={plane_z:.3f}")
        point = _intersect(self.p, self.r.as_matrix(), self._local_ray(u, v),
                           float(plane_z))
        if point is None:
            raise NotOnThePlane(
                f"the plane z={plane_z:.3f} is above the lens (z="
                f"{float(self.p[2]):.3f}); pixel ({u:.0f}, {v:.0f}) would meet "
                f"it only BEHIND the camera")
        mount = self.mount_spread(u, v, plane_z=float(plane_z)).combined_m
        height = self._height_spread(u, v, float(plane_z),
                                     float(plane_uncertainty_m), point)
        return Located(p=point, uncertainty_m=float(math.hypot(mount, height)),
                       plane_z=float(plane_z), plane_source=plane_source,
                       grazing=bool(abs(direction[2]) < GRAZING_SIN),
                       kind="contact", mount_uncertainty_m=float(mount),
                       height_uncertainty_m=float(height))

    def mount_spread(self, u: float, v: float, *, plane_z: float) -> MountSpread:
        """How far :meth:`locate` moves when the nominal mount is wrong.

        The lens is moved by ±``lens_uncertainty_m`` along each base axis and
        the aim turned by ±``aim_uncertainty_deg`` about each base axis, and
        the two are INDEPENDENT unknowns: the combined figure is the worst
        over every (translation, aim) pair including "only one of them",
        not the six samples that moved both at once along the same axis.
        """
        local = self._local_ray(u, v)
        base = _intersect(self.p, self.r.as_matrix(), local, plane_z)
        if base is None:
            inf = float("inf")
            return MountSpread(inf, inf, inf)
        shifts = [np.zeros(3)] + [axis * sign * self.lens_uncertainty_m
                                  for axis in np.eye(3) for sign in (1.0, -1.0)]
        tilt = math.radians(self.aim_uncertainty_deg)
        turns = [self.r.as_matrix()] + [
            (R.from_rotvec(axis * sign * tilt) * self.r).as_matrix()
            for axis in np.eye(3) for sign in (1.0, -1.0)]
        translation = aim = combined = 0.0
        for i, shift in enumerate(shifts):
            for j, turn in enumerate(turns):
                if i == 0 and j == 0:
                    continue
                moved = _intersect(self.p + shift, turn, local, plane_z)
                if moved is None:
                    inf = float("inf")
                    return MountSpread(inf, inf, inf)
                distance = float(np.linalg.norm(moved - base))
                combined = max(combined, distance)
                if j == 0:
                    translation = max(translation, distance)
                if i == 0:
                    aim = max(aim, distance)
        return MountSpread(translation, aim, combined)

    def _height_spread(self, u: float, v: float, plane_z: float, sigma: float,
                       point: np.ndarray) -> float:
        """How far the point moves when the plane is ``sigma`` higher or
        lower — the part of the error one declared height removes."""
        if sigma <= 0.0:
            return 0.0
        local = self._local_ray(u, v)
        worst = 0.0
        for dz in (sigma, -sigma):
            moved = _intersect(self.p, self.r.as_matrix(), local, plane_z + dz)
            if moved is None:
                return float("inf")
            worst = max(worst, float(np.linalg.norm(moved - point)))
        return worst

    # -- reporting ---------------------------------------------------------
    def to_json(self) -> Dict[str, Any]:
        return {"fx": round(self.fx, 2), "fy": round(float(self.fy), 2),
                "cx": round(self.cx, 2), "cy": round(self.cy, 2),
                "image": [self.width, self.height],
                "p_base": [round(float(v), 4) for v in self.p],
                "quat_xyzw": [round(float(v), 5) for v in self.r.as_quat()],
                "calibrated": self.calibrated,
                "lens_uncertainty_m": self.lens_uncertainty_m,
                "aim_uncertainty_deg": self.aim_uncertainty_deg,
                "notes": list(self.notes)}


def _intersect(p, rotation: np.ndarray, local_ray: np.ndarray,
               plane_z: float) -> Optional[np.ndarray]:
    """One ray-plane intersection IN FRONT of the lens, with the pose passed in.

    A free function over plain matrices so :meth:`PinholeCamera.mount_spread`
    can intersect 48 moved poses per pixel without building camera objects.
    ``None`` when the ray does not descend or the plane is behind the lens.
    """
    direction = rotation @ local_ray
    direction = direction / float(np.linalg.norm(direction))
    if direction[2] > -1e-6:
        return None
    p = np.asarray(p, dtype=float)
    distance = (plane_z - float(p[2])) / direction[2]
    if not distance > 0.0:
        return None
    return p + direction * distance


def contact_to_centre(contact: Located, *, size: Sequence[float],
                      viewpoint, yaw_rad: float = 0.0,
                      normal: Sequence[float] = (0.0, 0.0, 1.0),
                      image_up: Optional[Sequence[float]] = None) -> Located:
    """A CONTACT point at the bottom of a silhouette -> the object's CENTRE.

    The bottom of an object's silhouette meets the surface at the footprint's
    edge that lies furthest DOWN THE IMAGE. For a camera looking forward and
    down past the object (the head) that is the NEAR edge — the side facing
    the camera; for a camera ahead of the object looking back and down at it
    (a wrist camera past the standoff) image-down runs AWAY from the camera
    and the bottom of the silhouette is the FAR edge. So the centre is:

    * half the footprint's extent along the viewing direction, in the
      surface plane (the extent of a ``size[0] x size[1]`` rectangle yawed by
      ``yaw_rad``, measured along that direction) — FURTHER from the camera,
      unless ``image_up`` (the surface-plane direction the image's up axis
      runs at the contact pixel) points back toward it, and then toward it;
    * half the object's height UP the support normal.

    d1-2, 2026-09-24: the right wrist camera at x = 0.488 m looked back at a
    tape roll at 0.41 m; the pixel of the silhouette's bottom landed on the
    roll's far edge (0.38 m) and walking a further half-size away declared
    it at 0.353 m — 55 mm short of the photo, the grasp closed beside it.

    This is what a prompt sentence used to ask a model to do in its head
    (Astra review, item 7). It is a typed conversion now: a ``centre`` cannot
    be converted again, and a caller that declares a contact point as a
    centre has to say so by not calling this.
    """
    if contact.kind != "contact":
        raise ValueError(f"only a contact point converts to a centre; this "
                         f"one is already a {contact.kind!r}")
    extent = np.array(size, dtype=float).reshape(3)
    if not np.all(np.isfinite(extent)) or np.any(extent <= 0.0):
        raise ValueError(f"size must be three positive metres, got {size!r}")
    _finite("yaw_rad", yaw_rad)
    up = np.array(normal, dtype=float).reshape(3)
    up = up / float(np.linalg.norm(up))
    eye = np.array(viewpoint, dtype=float).reshape(3)
    away = contact.p - eye
    away = away - up * float(away @ up)
    norm = float(np.linalg.norm(away))
    if norm < 1e-9:
        # Looking straight down the normal: the silhouette's bottom is not an
        # edge of anything, and there is no "away" to walk.
        raise ValueError("the camera is directly above this point; a contact "
                         "edge has no direction to be corrected along")
    away = away / norm
    if image_up is not None:
        up_image = np.array(image_up, dtype=float).reshape(3)
        up_image = up_image - up * float(up_image @ up)
        if float(up_image @ away) < 0.0:
            # image-down runs away from the camera: the silhouette's bottom
            # is the far edge, the centre is back toward the camera
            away = -away
    # the object's own horizontal axes, yawed about the support normal
    turn = R.from_rotvec(up * float(yaw_rad))
    along_x = turn.apply([1.0, 0.0, 0.0])
    along_y = turn.apply([0.0, 1.0, 0.0])
    half_depth = 0.5 * (abs(float(away @ along_x)) * extent[0]
                        + abs(float(away @ along_y)) * extent[1])
    centre = contact.p + away * half_depth + up * (extent[2] / 2.0)
    return replace(contact, p=centre, kind="centre")


def provisional_table_z(kin=None) -> float:
    """Where to put the plane before anyone has measured it. See
    :data:`PROVISIONAL_UNCERTAINTY_M`."""
    from ..primitives.orientation import tool_from_link7  # noqa: PLC0415
    if kin is None:
        from ..arms import get_arm_kinematics  # noqa: PLC0415
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
    """``fx`` / ``fy`` / ``cx`` / ``cy`` out of a camera JSON, in the shapes
    that turn up.

    d1-inference writes ``{"fx": …, "cx": …}``; a ROS ``camera_info`` dump
    writes ``{"K": [...9]}`` or ``{"camera_matrix": {"data": [...9]}}``. Only
    those are read — no distortion model, because nothing here undoes one.
    """
    import json  # noqa: PLC0415
    from pathlib import Path  # noqa: PLC0415

    blob = json.loads(Path(path).read_text(encoding="utf-8"))
    out: Dict[str, float] = {}
    for key in ("fx", "focal_length_px"):
        if isinstance(blob.get(key), (int, float)):
            out["fx"] = float(blob[key])
            break
    for key, name in (("fy", "fy"), ("cx", "cx"), ("cy", "cy"),
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
                out.setdefault("fy", float(flat[4]))
                out.setdefault("cx", float(flat[2]))
                out.setdefault("cy", float(flat[5]))
                break
    if "fx" not in out:
        raise ValueError(f"{path}: no fx, K or camera_matrix in this file")
    return out


__all__ = ["DEFAULT_FX", "LENS_UNCERTAINTY_M", "AIM_UNCERTAINTY_DEG",
           "GRAZING_SIN", "PROVISIONAL_UNCERTAINTY_M", "LOCATED_KINDS",
           "CameraPose", "Located", "MountSpread", "NotOnThePlane",
           "PinholeCamera", "contact_to_centre", "provisional_table_z",
           "read_intrinsics"]
