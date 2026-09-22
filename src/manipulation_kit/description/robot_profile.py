"""One robot's MEASURED numbers, typed, in one file per robot.

The description in this package is the D1 as DESIGNED. A particular D1 is not
quite that: its head camera sits a few centimetres and a few degrees off the
URDF nominal, its gripper opens a little wider than the hand description, its
wrist lenses have their own focal lengths and fisheye distortion. Until
0.16.0 those numbers lived in three ad-hoc places (a scene file's
``robot.hand.open_gap_m``, a scene's ``robot.wrist_camera``, and nothing at
all for the head mount). A :class:`RobotProfile` is the one place:

``hand``              :class:`HandMeasurement` — the driven-open pad gap. On
                      hardware the daemon's own ``open_rad`` wins; this is the
                      value for a transport that cannot report it.
``head_mount_delta``  :class:`HeadMountDelta` — the head camera's fitted mount
                      as a correction of the nominal it was measured against,
                      in that nominal OPTICAL frame's axes (d1-inference
                      ``head_aruco``'s ``cameras_<robot>.head*.json``
                      convention: ``T = T_nominal @ T_delta``, rotation URDF
                      fixed-axis rpy), the nominal recorded with it.
                      :class:`manipulation_kit.perception.HeadCamera` applies
                      it and says ``calibrated: true`` only then.
``wrist_cameras``     side -> :class:`WristIntrinsics` — d1-inference
                      ``d1-calibrate-wrist``'s ``wrist_<side>_intrinsics.json``
                      (``model: fisheye|pinhole``, ``k: [k1..k4]``,
                      ``valid_radius_px``).

A profile is loaded from JSON (:meth:`RobotProfile.load`, or by name for the
ones committed under ``description/profiles/`` — :meth:`RobotProfile.named`)
or assembled from the d1-inference artefacts VERBATIM
(:meth:`RobotProfile.from_files`). A scene file's ``robot`` block names one
(``"robot": {"profile": "d1-2"}``) and may override any part of it
(:func:`scene_robot_block`). Nothing here is ever defaulted: a field nobody
measured is ``None``, and a consumer that needs it refuses.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Tuple, Union

#: where the committed per-robot profiles live, by name (``<name>.json``)
PROFILES = Path(__file__).resolve().parent / "profiles"

#: the lens models a wrist intrinsics block may name
WRIST_MODELS = ("pinhole", "fisheye")


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
    def from_json(cls, block: Mapping[str, Any]) -> "HeadMountDelta":
        return cls(tuple(block["xyz_m"]), tuple(block["rpy_deg"]),
                   tuple(block["nominal_xyz_m"]),
                   tuple(block["nominal_quat_xyzw"]),
                   rms_px=block.get("rms_px"),
                   source=str(block.get("source", "")))

    @classmethod
    def from_head_calibration(cls, path: Union[str, Path]) -> "HeadMountDelta":
        """d1-inference ``head_aruco``'s ``cameras_<robot>.head*.json``,
        verbatim: ``cameras.head.position_m`` / ``rotation_xyz_deg`` (the
        delta that file writes), the nominal from
        ``provenance.nominal_head_camera_optical_in_head_link``, RMS from
        ``provenance.scores``."""
        from scipy.spatial.transform import Rotation as R  # noqa: PLC0415
        doc = json.loads(Path(path).read_text(encoding="utf-8"))
        head = doc["cameras"]["head"]
        if "rotation_matrix" in head or "parent_link" in head:
            raise ValueError(
                f"{path}: this head entry is an ABSOLUTE head_link pose "
                f"(d1-isaaclab's merged form), not the delta a head_aruco "
                f"solve writes; pass the solver's own output")
        provenance = doc.get("provenance") or {}
        nominal = provenance.get("nominal_head_camera_optical_in_head_link")
        if not nominal:
            raise ValueError(f"{path}: no provenance.nominal_head_camera_"
                             f"optical_in_head_link — a correction without "
                             f"the nominal it corrects cannot be applied")
        quat = R.from_matrix(nominal["rotation_matrix"]).as_quat()
        scores = provenance.get("scores") or {}
        return cls(tuple(head["position_m"]), tuple(head["rotation_xyz_deg"]),
                   tuple(nominal["position_m"]), tuple(float(q) for q in quat),
                   rms_px=scores.get("rms_px"),
                   source=f"{Path(path).name} ({provenance.get('tool', 'head_aruco')}, "
                          f"{provenance.get('measured_at', 'undated')})")


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
    def from_json(cls, block: Mapping[str, Any]) -> "WristIntrinsics":
        """A profile's / scene's block, or d1-inference ``d1-calibrate-wrist``'s
        ``wrist_<side>_intrinsics.json`` verbatim (``k`` or ``distortion``)."""
        k = block.get("k")
        if k is None:
            k = block.get("distortion") or ()
        return cls(fx=float(block["fx"]), fy=float(block["fy"]),
                   cx=float(block["cx"]), cy=float(block["cy"]),
                   width=int(block["width"]), height=int(block["height"]),
                   model=str(block.get("model", "pinhole")),
                   k=tuple(k) if block.get("model") == "fisheye" else (),
                   valid_radius_px=(None if block.get("valid_radius_px") is None
                                    else float(block["valid_radius_px"])),
                   rms_px=block.get("rms_px"),
                   source=str(block.get("source", block.get("_source", ""))))


@dataclass(frozen=True)
class RobotProfile:
    """Everything measured about ONE robot that the kit consumes."""

    name: str
    hand: Optional[HandMeasurement] = None
    head_mount_delta: Optional[HeadMountDelta] = None
    wrist_cameras: Dict[str, WristIntrinsics] = field(default_factory=dict)
    notes: Tuple[str, ...] = ()

    # -- reading --------------------------------------------------------- #
    @classmethod
    def from_json(cls, doc: Mapping[str, Any]) -> "RobotProfile":
        hand = doc.get("hand")
        head = doc.get("head_mount_delta")
        wrists = doc.get("wrist_cameras") or {}
        return cls(
            name=str(doc["name"]),
            hand=None if not hand else HandMeasurement(
                float(hand["open_gap_m"]), str(hand.get("source", ""))),
            head_mount_delta=None if not head else HeadMountDelta.from_json(head),
            wrist_cameras={side: WristIntrinsics.from_json(block)
                           for side, block in wrists.items() if block},
            notes=tuple(doc.get("notes", ())))

    @classmethod
    def load(cls, path: Union[str, Path]) -> "RobotProfile":
        return cls.from_json(json.loads(Path(path).read_text(encoding="utf-8")))

    @classmethod
    def named(cls, name: str) -> "RobotProfile":
        """A profile committed with the kit (``description/profiles/``)."""
        path = PROFILES / f"{name}.json"
        if not path.is_file():
            known = sorted(p.stem for p in PROFILES.glob("*.json"))
            raise LookupError(f"no robot profile called {name!r}; the kit "
                              f"carries {known}")
        return cls.load(path)

    @classmethod
    def resolve(cls, ref: Union[str, Path, "RobotProfile", None],
                *, relative_to: Optional[Path] = None
                ) -> Optional["RobotProfile"]:
        """A profile from what a flag or a scene says: a profile, a path to a
        JSON file (relative to ``relative_to`` when given), or a committed
        name. ``None`` stays ``None``."""
        if ref is None or isinstance(ref, RobotProfile):
            return ref
        path = Path(ref)
        candidates = [path] + ([relative_to / path] if relative_to else [])
        for candidate in candidates:
            if candidate.suffix == ".json" and candidate.is_file():
                return cls.load(candidate)
        return cls.named(str(ref))

    @classmethod
    def from_files(cls, name: str, *,
                   head_calibration: Union[str, Path, None] = None,
                   wrist: Optional[Mapping[str, Union[str, Path]]] = None,
                   hand_open_gap_m: Optional[float] = None,
                   hand_source: str = "") -> "RobotProfile":
        """A profile from the d1-inference artefacts, read verbatim: the head
        solve's ``cameras_<robot>.head*.json`` and ``d1-calibrate-wrist``'s
        ``wrist_<side>_intrinsics.json`` per side."""
        wrists: Dict[str, WristIntrinsics] = {}
        for side, path in (wrist or {}).items():
            doc = json.loads(Path(path).read_text(encoding="utf-8"))
            if doc.get("side") not in (None, side):
                raise ValueError(f"{path} is the {doc['side']} wrist, not "
                                 f"the {side}")
            wrists[side] = WristIntrinsics.from_json(doc)
        return cls(
            name=name,
            hand=(None if hand_open_gap_m is None
                  else HandMeasurement(float(hand_open_gap_m), hand_source)),
            head_mount_delta=(None if head_calibration is None else
                              HeadMountDelta.from_head_calibration(
                                  head_calibration)),
            wrist_cameras=wrists)

    # -- writing / consuming --------------------------------------------- #
    def to_json(self) -> Dict[str, Any]:
        return {"schema": "manipulation_kit.robot_profile/1", "name": self.name,
                "hand": None if self.hand is None else self.hand.to_json(),
                "head_mount_delta": (None if self.head_mount_delta is None
                                     else self.head_mount_delta.to_json()),
                "wrist_cameras": {s: w.to_json()
                                  for s, w in sorted(self.wrist_cameras.items())},
                "notes": list(self.notes)}

    def scene_block(self) -> Dict[str, Any]:
        """The profile as a scene file's ``robot`` block, every value marked
        measured — what the agent's scene readers consume."""
        out: Dict[str, Any] = {"profile_name": self.name}
        if self.hand is not None:
            out["hand"] = {"open_gap_m": float(self.hand.open_gap_m),
                           "_source": self.hand.source}
        if self.wrist_cameras:
            out["wrist_camera"] = {
                side: dict(w.to_json(), measured=True)
                for side, w in sorted(self.wrist_cameras.items())}
        if self.head_mount_delta is not None:
            out["head_mount_delta"] = self.head_mount_delta.to_json()
        return out


def scene_robot_block(scene: Optional[Mapping[str, Any]], *,
                      profile: Optional[RobotProfile] = None,
                      relative_to: Optional[Path] = None) -> Dict[str, Any]:
    """A scene's ``robot`` block with its profile folded in.

    The profile is ``profile`` when given, else whatever the block names
    (``"profile": "d1-2"`` or a path). The scene's own keys OVERRIDE the
    profile's, key by key at the top level (``hand``, ``wrist_camera``) — a
    scene written for one session may restate a number, and it says so where
    it does. ``{}`` when there is neither.
    """
    block = dict(((scene or {}).get("robot") or {}))
    ref = block.pop("profile", None)
    if profile is None and ref is not None:
        profile = RobotProfile.resolve(ref, relative_to=relative_to)
    merged = profile.scene_block() if profile is not None else {}
    merged.update(block)
    return merged


def with_profile(scene: Optional[Mapping[str, Any]],
                 profile: Optional[RobotProfile], *,
                 relative_to: Optional[Path] = None) -> Dict[str, Any]:
    """``scene`` (possibly ``None``) with its ``robot`` block resolved against
    ``profile`` / the profile it names — a plain scene dict every reader
    understands."""
    out = dict(scene or {})
    block = scene_robot_block(out, profile=profile, relative_to=relative_to)
    if block:
        out["robot"] = block
    return out


__all__ = ["HandMeasurement", "HeadMountDelta", "PROFILES", "RobotProfile",
           "WRIST_MODELS", "WristIntrinsics", "scene_robot_block",
           "with_profile"]
