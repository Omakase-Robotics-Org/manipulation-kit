"""One robot's MEASURED numbers, typed — read from the file the ROBOT holds.

The description in this package is the D1 as DESIGNED. A particular D1 is not
quite that: its head camera sits a few centimetres and a few degrees off the
URDF nominal, its gripper opens a little wider than the hand description, its
wrist lenses have their own focal lengths and fisheye distortion. A
:class:`RobotProfile` is those numbers in the shape the kit consumes:

``hand``              :class:`HandMeasurement` — the driven-open pad gap. On
                      hardware the daemon's own ``open_rad`` wins; this is the
                      value for a transport that cannot report it.
``head_mount_delta``  :class:`HeadMountDelta` — the head camera's fitted mount
                      as a correction of the nominal it was measured against
                      (``T = T_nominal @ T_delta``, rotation URDF fixed-axis
                      rpy), the nominal recorded with it.
                      :class:`manipulation_kit.perception.HeadCamera` applies
                      it and says ``calibrated: true`` only then.
``wrist_cameras``     side -> :class:`WristIntrinsics` (``model:
                      fisheye|pinhole``, ``k: [k1..k4]``, ``valid_radius_px``).
``wrist_mounts``      side -> :class:`WristMount` — the wrist camera's MEASURED
                      optical frame in its gripper's camera plate
                      (``gripper_R_camera_plate`` for the logical left arm,
                      ``gripper_L_camera_plate`` for the right).
                      :class:`manipulation_kit.perception.WristCamera` uses it
                      in place of the URDF nominal and says ``mount:
                      measured`` only then.

**The kit owns the schema and the reader; the robot holds the values.** No
profile ships in this wheel. A robot keeps ONE ``omakase.camera_calibration/2``
file (:mod:`manipulation_kit.description.camera_calibration`, default
``~/.config/omakase/camera_calibration.json``), and a profile is built from it
(:meth:`RobotProfile.load` / :meth:`RobotProfile.from_camera_calibration`):
the head mount's ABSOLUTE ``head_link -> optical`` pose becomes the delta on
the nominal it records, the ``left_wrist`` / ``right_wrist`` lenses become the
kit's per-side intrinsics and their mounts the per-side plate -> optical
transforms, ``hand`` becomes :class:`HandMeasurement`. A layer
whose gate FAILED is refused unless overridden (``allow_failed_gate``). A scene
file's ``robot`` block may name a file (``"robot": {"profile": "PATH"}``,
relative to the scene) and override any part of it (:func:`scene_robot_block`).
Nothing here is ever defaulted: a field nobody measured is ``None``, and a
consumer that needs it refuses.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Tuple, Union

from ..arms.sides import URDF_SUFFIX
from . import camera_calibration as cc

#: the lens models a wrist intrinsics block may name
WRIST_MODELS = ("pinhole", "fisheye")

#: the link a head mount is expressed in
HEAD_PARENT_LINK = "head_link"

#: logical side -> the link a wrist mount is expressed in: the gripper's
#: camera plate, which IS the gripper base (the tool flange, yawed pi on the
#: logical left arm). NOTE the crossed suffix (:mod:`manipulation_kit.arms.sides`).
WRIST_PARENT_LINKS = {side: f"gripper_{suffix}_camera_plate"
                      for side, suffix in URDF_SUFFIX.items()}


class NotACalibrationFile(LookupError):
    """``--robot-profile`` got something that is not a calibration file."""


def _finite(what: str, *values: float) -> None:
    for value in values:
        if not math.isfinite(float(value)):
            raise ValueError(f"{what} must be finite, got {value!r}")


@dataclass(frozen=True)
class HandMeasurement:
    """The gripper as measured on this robot."""

    #: the driven-open pad gap [m]
    open_gap_m: float
    source: str = ""

    def __post_init__(self) -> None:
        _finite("open_gap_m", self.open_gap_m)
        if not 0.0 < float(self.open_gap_m) < 0.2:
            raise ValueError(f"open_gap_m must be a gap in metres, got "
                             f"{self.open_gap_m!r}")

    def to_json(self) -> Dict[str, Any]:
        return {"open_gap_m": float(self.open_gap_m), "source": self.source}


@dataclass(frozen=True)
class HeadMountDelta:
    """The head camera's fitted mount, as a correction of the nominal it was
    MEASURED AGAINST.

    ``xyz_m`` / ``rpy_deg`` are the correction, in that nominal optical
    frame's axes, composing on the right (``T_headlink_optical = T_nominal @
    T_delta``, rotation URDF fixed-axis roll-pitch-yaw, scipy ``"xyz"``) —
    d1-inference ``head_aruco``'s convention. ``nominal_xyz_m`` /
    ``nominal_quat_xyzw`` are the optical frame in ``head_link`` the fit was
    made against, and they are REQUIRED: a correction means nothing without
    its nominal, and the kit's own nominal has moved since (the head part's
    design tilt went from 17.25 to 15 deg, 2026-09-20). So the camera pose is
    rebuilt ABSOLUTELY — ``base <- head_link`` from the URDF at the neck
    joints, then the measured ``head_link <- optical`` — and a later change of
    the URDF's nominal cannot silently shift it.
    """

    xyz_m: Tuple[float, float, float]
    rpy_deg: Tuple[float, float, float]
    nominal_xyz_m: Tuple[float, float, float]
    nominal_quat_xyzw: Tuple[float, float, float, float]
    rms_px: Optional[float] = None
    source: str = ""

    def __post_init__(self) -> None:
        for name, size in (("xyz_m", 3), ("rpy_deg", 3), ("nominal_xyz_m", 3),
                           ("nominal_quat_xyzw", 4)):
            value = tuple(float(v) for v in getattr(self, name))
            if len(value) != size:
                raise ValueError(f"head mount {name} needs {size} numbers, "
                                 f"got {list(value)}")
            _finite(f"head mount {name}", *value)
            object.__setattr__(self, name, value)

    def head_link_to_optical(self):
        """``(p, r)`` of the MEASURED optical frame in ``head_link``."""
        import numpy as np  # noqa: PLC0415
        from scipy.spatial.transform import Rotation as R  # noqa: PLC0415
        r_nom = R.from_quat(self.nominal_quat_xyzw)
        delta = R.from_euler("xyz", self.rpy_deg, degrees=True)
        p = (np.asarray(self.nominal_xyz_m, dtype=float)
             + r_nom.apply(np.asarray(self.xyz_m, dtype=float)))
        return p, r_nom * delta

    def to_json(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {"xyz_m": list(self.xyz_m),
                               "rpy_deg": list(self.rpy_deg),
                               "nominal_xyz_m": list(self.nominal_xyz_m),
                               "nominal_quat_xyzw": list(self.nominal_quat_xyzw),
                               "source": self.source}
        if self.rms_px is not None:
            out["rms_px"] = float(self.rms_px)
        return out
    @classmethod
    def from_mount(cls, mount: "cc.Mount") -> "HeadMountDelta":
        """The delta of a v2 file's ABSOLUTE mount on the nominal it records:
        ``T_delta = T_nominal^-1 @ T_measured``, so :meth:`head_link_to_optical`
        gives back ``mount.T_parent_camera``."""
        import numpy as np  # noqa: PLC0415
        from scipy.spatial.transform import Rotation as R  # noqa: PLC0415
        if mount.parent_link != HEAD_PARENT_LINK:
            raise ValueError(f"the head mount must be expressed in "
                             f"{HEAD_PARENT_LINK!r}, the file says "
                             f"{mount.parent_link!r}")
        r_nom = R.from_quat(mount.nominal.quat_xyzw)
        r_meas = R.from_quat(mount.T_parent_camera.quat_xyzw)
        xyz = r_nom.inv().apply(np.asarray(mount.T_parent_camera.xyz_m)
                                - np.asarray(mount.nominal.xyz_m))
        rpy = (r_nom.inv() * r_meas).as_euler("xyz", degrees=True)
        prov = mount.provenance
        return cls(tuple(float(v) for v in xyz), tuple(float(v) for v in rpy),
                   tuple(mount.nominal.xyz_m), tuple(mount.nominal.quat_xyzw),
                   rms_px=prov.rms_px,
                   source=f"{prov.describe()}; gate {mount.gate.verdict}")


@dataclass(frozen=True)
class WristIntrinsics:
    """One wrist lens: pinhole intrinsics, and the fisheye model when it is one.

    ``k`` are OpenCV ``cv2.fisheye`` coefficients (equidistant: ``theta_d =
    theta (1 + k1 theta^2 + k2 theta^4 + k3 theta^6 + k4 theta^8)``);
    ``valid_radius_px`` is how far from the principal point the calibration's
    board coverage reaches — pixels outside it are not trusted.
    """

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    model: str = "pinhole"
    k: Tuple[float, ...] = ()
    valid_radius_px: Optional[float] = None
    rms_px: Optional[float] = None
    source: str = ""

    def __post_init__(self) -> None:
        _finite("wrist intrinsics", self.fx, self.fy, self.cx, self.cy)
        if self.model not in WRIST_MODELS:
            raise ValueError(f"wrist camera model must be one of "
                             f"{WRIST_MODELS}, got {self.model!r}")
        object.__setattr__(self, "k", tuple(float(v) for v in self.k))
        if self.model == "fisheye" and len(self.k) != 4:
            raise ValueError(f"a fisheye lens needs k = [k1, k2, k3, k4], got "
                             f"{list(self.k)}")
        _finite("wrist distortion", *self.k)

    def camera_kwargs(self) -> Dict[str, Any]:
        """What :meth:`manipulation_kit.perception.WristCamera.from_flange`
        takes."""
        return {"fx": float(self.fx), "fy": float(self.fy),
                "cx": float(self.cx), "cy": float(self.cy),
                "width": int(self.width), "height": int(self.height),
                "model": self.model, "k": tuple(self.k),
                "valid_radius_px": self.valid_radius_px}

    def to_json(self) -> Dict[str, Any]:
        out = dict(self.camera_kwargs(), k=list(self.k), source=self.source)
        if self.rms_px is not None:
            out["rms_px"] = float(self.rms_px)
        return out
    @classmethod
    def from_calibration(cls, camera: "cc.Camera") -> "WristIntrinsics":
        """A v2 file's wrist camera: ``kannala_brandt`` is the kit's
        ``fisheye``; ``none`` (or a Brown-Conrady model with all-zero
        coefficients) is ``pinhole``. A non-zero Brown-Conrady distortion is
        refused — the wrist camera model has no such lens, and dropping the
        coefficients would project through numbers nobody measured."""
        lens = camera.intrinsics
        if lens is None:
            raise ValueError(f"camera {camera.slot!r} has no intrinsics layer")
        dist = lens.distortion
        if dist.model == "kannala_brandt":
            model, k = "fisheye", tuple(dist.coefficients)
        elif dist.model == "none" or dist.is_zero:
            model, k = "pinhole", ()
        else:
            raise ValueError(
                f"camera {camera.slot!r}: {dist.model} distortion "
                f"{list(dist.coefficients)} — the kit's wrist camera models "
                f"kannala_brandt (fisheye) and undistorted pinhole lenses only")
        return cls(fx=lens.fx, fy=lens.fy, cx=lens.cx, cy=lens.cy,
                   width=camera.width, height=camera.height, model=model, k=k,
                   valid_radius_px=lens.valid_radius_px,
                   rms_px=lens.provenance.rms_px,
                   source=f"{camera.slot}: {lens.provenance.describe()}; "
                          f"gate {lens.gate.verdict}")


@dataclass(frozen=True)
class WristMount:
    """One wrist camera's MEASURED mount: the ABSOLUTE pose of its optical
    frame (ROS: x right, y down, z forward) in the gripper's camera plate
    (:data:`WRIST_PARENT_LINKS`), with the URDF nominal it was fitted against.

    It is absolute on purpose, like the head's: ``base <- plate`` comes from
    the arm's FK, then this measured ``plate <- optical`` — a later change of
    the URDF's nominal cannot silently move a measured camera. The nominal is
    carried for the trace (how far the measurement moved the lens).
    """

    side: str
    xyz_m: Tuple[float, float, float]
    quat_xyzw: Tuple[float, float, float, float]
    parent_link: str
    nominal_xyz_m: Optional[Tuple[float, float, float]] = None
    nominal_quat_xyzw: Optional[Tuple[float, float, float, float]] = None
    #: the mount gate's verdict as the file recorded it (never FAIL unless
    #: overridden — the reader refuses that)
    gate: str = ""
    rms_px: Optional[float] = None
    source: str = ""

    def __post_init__(self) -> None:
        if self.side not in WRIST_PARENT_LINKS:
            raise ValueError(f"wrist mount side must be one of "
                             f"{sorted(WRIST_PARENT_LINKS)}, got {self.side!r}")
        expected = WRIST_PARENT_LINKS[self.side]
        if self.parent_link != expected:
            raise ValueError(
                f"the {self.side} wrist camera's mount must be expressed in "
                f"{expected!r} (logical {self.side} = URDF suffix "
                f"_{URDF_SUFFIX[self.side]}), the file says "
                f"{self.parent_link!r}")
        for name, size in (("xyz_m", 3), ("quat_xyzw", 4),
                           ("nominal_xyz_m", 3), ("nominal_quat_xyzw", 4)):
            raw = getattr(self, name)
            if raw is None and name.startswith("nominal"):
                continue
            value = tuple(float(v) for v in raw)
            if len(value) != size:
                raise ValueError(f"wrist mount {name} needs {size} numbers, "
                                 f"got {list(value)}")
            _finite(f"wrist mount {name}", *value)
            object.__setattr__(self, name, value)
        norm = math.sqrt(sum(q * q for q in self.quat_xyzw))
        if abs(norm - 1.0) > 1e-6:
            raise ValueError(f"wrist mount quat_xyzw must be a unit "
                             f"quaternion, norm is {norm:.9f}")

    @classmethod
    def from_mount(cls, side: str, mount: "cc.Mount") -> "WristMount":
        """A v2 file's ``cameras.<side>_wrist.mount``."""
        prov = mount.provenance
        return cls(side=side, xyz_m=mount.T_parent_camera.xyz_m,
                   quat_xyzw=mount.T_parent_camera.quat_xyzw,
                   parent_link=mount.parent_link,
                   nominal_xyz_m=mount.nominal.xyz_m,
                   nominal_quat_xyzw=mount.nominal.quat_xyzw,
                   gate=mount.gate.verdict, rms_px=prov.rms_px,
                   source=f"{cc.WRIST_SLOTS[side]}: {prov.describe()}; "
                          f"gate {mount.gate.verdict}")

    @classmethod
    def from_json(cls, side: str, block: Mapping[str, Any]) -> "WristMount":
        """The block :meth:`to_json` writes (a scene's
        ``robot.wrist_camera.<side>.mount``)."""
        missing = [k for k in ("parent_link", "xyz_m", "quat_xyzw")
                   if k not in block]
        if missing:
            raise ValueError(f"scene robot.wrist_camera.{side}.mount needs "
                             f"parent_link, xyz_m, quat_xyzw; missing {missing}")
        return cls(side=side, xyz_m=block["xyz_m"],
                   quat_xyzw=block["quat_xyzw"],
                   parent_link=str(block["parent_link"]),
                   nominal_xyz_m=block.get("nominal_xyz_m"),
                   nominal_quat_xyzw=block.get("nominal_quat_xyzw"),
                   gate=str(block.get("gate", "")),
                   rms_px=(None if block.get("rms_px") is None
                           else float(block["rms_px"])),
                   source=str(block.get("source", "")))

    def to_json(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {"parent_link": self.parent_link,
                               "xyz_m": list(self.xyz_m),
                               "quat_xyzw": list(self.quat_xyzw),
                               "gate": self.gate, "source": self.source}
        if self.nominal_xyz_m is not None:
            out["nominal_xyz_m"] = list(self.nominal_xyz_m)
        if self.nominal_quat_xyzw is not None:
            out["nominal_quat_xyzw"] = list(self.nominal_quat_xyzw)
        if self.rms_px is not None:
            out["rms_px"] = float(self.rms_px)
        return out


@dataclass(frozen=True)
class RobotProfile:
    """Everything measured about ONE robot that the kit consumes."""

    name: str
    hand: Optional[HandMeasurement] = None
    head_mount_delta: Optional[HeadMountDelta] = None
    wrist_cameras: Dict[str, WristIntrinsics] = field(default_factory=dict)
    wrist_mounts: Dict[str, WristMount] = field(default_factory=dict)
    #: the file it was built from, typed (``None`` for a hand-built profile)
    calibration: Optional["cc.CameraCalibration"] = field(default=None,
                                                         compare=False,
                                                         repr=False)

    @classmethod
    def from_camera_calibration(
            cls, doc: Union["cc.CameraCalibration", Mapping[str, Any], str, Path],
            *, allow_failed_gate: bool = False) -> "RobotProfile":
        """A profile from an ``omakase.camera_calibration/2`` file — a path,
        the parsed JSON, or an already-read
        :class:`~manipulation_kit.description.camera_calibration.CameraCalibration`.

        A layer whose gate FAILED is refused
        (:class:`~manipulation_kit.description.camera_calibration.FailedCalibrationGate`)
        unless the file overrides it or ``allow_failed_gate`` — a wrist
        mount's gate included (WARN is accepted and warned about). A wrist
        mount becomes that side's :class:`WristMount`; a wrist mount
        expressed in any link but the side's camera plate is refused."""
        if isinstance(doc, (str, Path)):
            calib = cc.load(doc, allow_failed_gate=allow_failed_gate)
        elif isinstance(doc, cc.CameraCalibration):
            calib = doc
        else:
            calib = cc.parse(doc, allow_failed_gate=allow_failed_gate)
        head = calib.camera(cc.HEAD_SLOT)
        wrists: Dict[str, WristIntrinsics] = {}
        mounts: Dict[str, WristMount] = {}
        for side, slot in cc.WRIST_SLOTS.items():
            camera = calib.camera(slot)
            if camera is None:
                continue
            if camera.mount is not None:
                try:
                    mounts[side] = WristMount.from_mount(side, camera.mount)
                except ValueError as exc:
                    raise ValueError(f"{calib.source}: camera {slot!r}: "
                                     f"{exc}") from None
            if camera.intrinsics is not None:
                wrists[side] = WristIntrinsics.from_calibration(camera)
        hand = calib.hand
        return cls(
            name=calib.robot,
            hand=None if hand is None else HandMeasurement(
                hand.open_gap_m,
                "" if hand.provenance is None else hand.provenance.describe()),
            head_mount_delta=(None if head is None or head.mount is None
                              else HeadMountDelta.from_mount(head.mount)),
            wrist_cameras=wrists, wrist_mounts=mounts, calibration=calib)

    @classmethod
    def load(cls, path: Union[str, Path], *,
             allow_failed_gate: bool = False) -> "RobotProfile":
        """The profile in one robot's calibration file (an old
        ``manipulation_kit.robot_profile/1`` file is refused with the
        migration)."""
        return cls.from_camera_calibration(Path(path).expanduser(),
                                           allow_failed_gate=allow_failed_gate)

    @classmethod
    def resolve(cls, ref: Union[str, Path, "RobotProfile", None], *,
                relative_to: Optional[Path] = None,
                allow_failed_gate: bool = False) -> Optional["RobotProfile"]:
        """A profile from what a flag or a scene says: a profile, or a PATH to
        a calibration file (tried as given, then relative to ``relative_to``).
        ``None`` stays ``None``. A bare robot name is an error: the kit no
        longer carries any robot's numbers."""
        if ref is None or isinstance(ref, RobotProfile):
            return ref
        path = Path(ref).expanduser()
        candidates = [path]
        if relative_to is not None and not path.is_absolute():
            candidates.append(Path(relative_to) / path)
        for candidate in candidates:
            if candidate.is_file():
                return cls.load(candidate, allow_failed_gate=allow_failed_gate)
        if path.suffix == "" and len(path.parts) == 1:
            raise NotACalibrationFile(
                f"robot profile {str(ref)!r} is a name, not a file: the kit "
                f"carries no robot's numbers any more. {cc.MIGRATION_HINT}")
        raise FileNotFoundError(
            f"no calibration file at {' or '.join(str(c) for c in candidates)}")

    def scene_block(self) -> Dict[str, Any]:
        """The profile as a scene file's ``robot`` block, every value marked
        measured — what the agent's scene readers consume."""
        out: Dict[str, Any] = {"profile_name": self.name}
        if self.hand is not None:
            out["hand"] = {"open_gap_m": float(self.hand.open_gap_m),
                           "_source": self.hand.source}
        if self.wrist_cameras:
            out["wrist_camera"] = {}
            for side, w in sorted(self.wrist_cameras.items()):
                one = dict(w.to_json(), measured=True)
                if side in self.wrist_mounts:
                    one["mount"] = self.wrist_mounts[side].to_json()
                out["wrist_camera"][side] = one
        if self.head_mount_delta is not None:
            out["head_mount_delta"] = self.head_mount_delta.to_json()
        return out


def scene_robot_block(scene: Optional[Mapping[str, Any]], *,
                      profile: Union[RobotProfile, str, Path, None] = None,
                      relative_to: Optional[Path] = None,
                      allow_failed_gate: bool = False) -> Dict[str, Any]:
    """A scene's ``robot`` block with its profile folded in.

    The profile is ``profile`` when given (a profile or a calibration file's
    path), else the file the block names (``"profile": "PATH"``, relative to
    ``relative_to`` — the scene file's directory). The scene's own keys
    OVERRIDE the profile's, key by key at the top level (``hand``,
    ``wrist_camera``) — a scene written for one session may restate a number,
    and it says so where it does. ``{}`` when there is neither.
    """
    block = dict(((scene or {}).get("robot") or {}))
    ref = block.pop("profile", None)
    if profile is None:
        profile, where = ref, relative_to
    else:
        where = None
    resolved = RobotProfile.resolve(profile, relative_to=where,
                                    allow_failed_gate=allow_failed_gate)
    merged = resolved.scene_block() if resolved is not None else {}
    merged.update(block)
    return merged


def with_profile(scene: Optional[Mapping[str, Any]],
                 profile: Union[RobotProfile, str, Path, None], *,
                 relative_to: Optional[Path] = None,
                 allow_failed_gate: bool = False) -> Dict[str, Any]:
    """``scene`` (possibly ``None``) with its ``robot`` block resolved against
    ``profile`` / the file it names — a plain scene dict every reader
    understands."""
    out = dict(scene or {})
    block = scene_robot_block(out, profile=profile, relative_to=relative_to,
                              allow_failed_gate=allow_failed_gate)
    if block:
        out["robot"] = block
    return out


__all__ = ["HEAD_PARENT_LINK", "HandMeasurement", "HeadMountDelta",
           "NotACalibrationFile", "RobotProfile", "WRIST_MODELS",
           "WRIST_PARENT_LINKS", "WristIntrinsics", "WristMount",
           "scene_robot_block", "with_profile"]
