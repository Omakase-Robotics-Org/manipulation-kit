"""The reader of ``omakase.camera_calibration/2`` — one robot's cameras, one file.

**The kit owns the schema and the reader; the robot holds the values.** No
per-robot number ships in this wheel. Each robot keeps ONE file,
:data:`DEFAULT_PATH` (``~/.config/omakase/camera_calibration.json``), written
by seiryu-calib (the camera layers) and read here. The JSON Schema is package
data (:data:`SCHEMA_PATH`); this module checks the same rules by hand, so the
kit takes no ``jsonschema`` dependency.

The file, in short::

    {"schema": "omakase.camera_calibration/2", "robot": "d1-2",
     "cameras": {"head" | "left_wrist" | "right_wrist" | ...: {
         "stream": {"width", "height"},
         "intrinsics": null | {fx, fy, cx, cy,             # layer 1: the lens
                               "distortion": {"model", "coefficients"},
                               "valid_radius_px", "gate", "provenance"},
         "mount": null | {"parent_link",                    # layer 2: the mount
                          "T_parent_camera": {"xyz_m", "quat_xyzw"},
                          "nominal": {"T_parent_camera": {...}},
                          "gate", "provenance"}}},
     "hand": null | {"open_gap_m", "provenance"}}

``T_parent_camera`` is the ABSOLUTE pose of the camera's OPTICAL frame (ROS:
x right, y down, z forward) in ``parent_link``; ``nominal`` is the pose it was
fitted against. The distortion vocabulary is seiryu-stream's
(:data:`DISTORTION_COEFFICIENTS`); ``kannala_brandt`` is OpenCV
``cv2.fisheye``.

**Gates.** Every layer carries ``gate.verdict`` ``PASS | WARN | FAIL``. The
reader REFUSES a FAIL layer (:class:`FailedCalibrationGate`, naming the
camera, the layer and the gate's reasons) unless the file's gate records an
``override`` with a reason, or the caller passes ``allow_failed_gate=True``
(the CLIs' ``--allow-failed-calibration``). WARN is accepted and warned about.
"""

from __future__ import annotations

import json
import math
import warnings
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Tuple, Union

#: the one schema string this reader accepts
SCHEMA = "omakase.camera_calibration/2"

#: the kit's pre-2026-09-23 per-robot file, refused with a migration pointer
LEGACY_ROBOT_PROFILE_SCHEMA = "manipulation_kit.robot_profile/1"

#: where a robot keeps its file
DEFAULT_PATH = Path("~/.config/omakase/camera_calibration.json").expanduser()

#: the JSON Schema this module implements, shipped as package data
SCHEMA_PATH = (Path(__file__).resolve().parent / "schemas"
               / "omakase.camera_calibration-2.schema.json")

#: seiryu-stream DistortionModel -> number of coefficients, in its order
DISTORTION_COEFFICIENTS = {
    "none": 0,
    "brown_conrady": 5,             # k1 k2 p1 p2 k3
    "inverse_brown_conrady": 5,
    "brown_conrady_rational": 8,    # k1 k2 p1 p2 k3 k4 k5 k6
    "kannala_brandt": 4,            # k1..k4 (cv2.fisheye)
}

VERDICTS = ("PASS", "WARN", "FAIL")
CHECK_VERDICTS = VERDICTS + ("SKIP",)
LAYERS = ("intrinsics", "mount")

#: the kit's logical arm side -> the file's camera slot
WRIST_SLOTS = {"left": "left_wrist", "right": "right_wrist"}
HEAD_SLOT = "head"

MIGRATION_HINT = (
    "per-robot values now live ON THE ROBOT in one omakase.camera_calibration/2 "
    f"file (default {DEFAULT_PATH}); build it with `seiryu-calib migrate` "
    "from the d1-inference sources (wrist_<side>_intrinsics.json, "
    "cameras_<robot>.head*.json), or from seiryu-calib's own solves, and pass "
    "it with --robot-profile PATH")


class CalibrationFormatError(ValueError):
    """The document is not a valid ``omakase.camera_calibration/2`` file."""


class FailedCalibrationGate(ValueError):
    """A layer's gate says FAIL and nothing overrode it."""

    def __init__(self, source: str, camera: str, layer: str,
                 reasons: Tuple[str, ...]):
        self.camera, self.layer, self.reasons = camera, layer, reasons
        why = "; ".join(reasons) or "no reason recorded"
        super().__init__(
            f"{source}: camera {camera!r} {layer} failed its calibration gate "
            f"({why}). Re-calibrate, or install it deliberately: record a "
            f"gate.override with a reason in the file, or pass "
            f"allow_failed_gate=True / --allow-failed-calibration")


# --------------------------------------------------------------------------- #
# validation — the schema's rules, by hand
# --------------------------------------------------------------------------- #

def _is_number(value: Any) -> bool:
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(float(value)))


class _Checker:
    def __init__(self) -> None:
        self.problems: List[str] = []

    def fail(self, where: str, what: str) -> None:
        self.problems.append(f"{where}: {what}")

    def obj(self, where: str, value: Any, required: Tuple[str, ...] = (),
            allowed: Optional[Tuple[str, ...]] = None) -> bool:
        if not isinstance(value, Mapping):
            self.fail(where, f"must be an object, got {type(value).__name__}")
            return False
        for key in required:
            if key not in value:
                self.fail(where, f"missing required key {key!r}")
        if allowed is not None:
            for key in value:
                if key not in allowed:
                    self.fail(where, f"unknown key {key!r}")
        return True

    def number(self, where: str, value: Any, *, positive: bool = False,
               nullable: bool = False) -> None:
        if value is None and nullable:
            return
        if not _is_number(value):
            self.fail(where, f"must be a finite number, got {value!r}")
        elif positive and float(value) <= 0.0:
            self.fail(where, f"must be > 0, got {value!r}")

    def string(self, where: str, value: Any, *, nullable: bool = False,
               nonempty: bool = False) -> None:
        if value is None and nullable:
            return
        if not isinstance(value, str) or (nonempty and not value):
            self.fail(where, f"must be a{' non-empty' if nonempty else ''} "
                             f"string, got {value!r}")

    def numbers(self, where: str, value: Any, size: int) -> bool:
        if (not isinstance(value, list) or len(value) != size
                or not all(_is_number(v) for v in value)):
            self.fail(where, f"must be {size} finite numbers, got {value!r}")
            return False
        return True

    # -- blocks ------------------------------------------------------------ #
    def pose(self, where: str, value: Any) -> None:
        if not self.obj(where, value, ("xyz_m", "quat_xyzw"),
                        ("xyz_m", "quat_xyzw")):
            return
        self.numbers(f"{where}.xyz_m", value.get("xyz_m"), 3)
        if self.numbers(f"{where}.quat_xyzw", value.get("quat_xyzw"), 4):
            norm = math.sqrt(sum(float(q) ** 2 for q in value["quat_xyzw"]))
            if abs(norm - 1.0) > 1e-6:
                self.fail(f"{where}.quat_xyzw", f"must be a unit quaternion, "
                                                f"norm is {norm:.9f}")

    def gate(self, where: str, value: Any) -> None:
        if not self.obj(where, value, ("verdict", "checks"),
                        ("verdict", "checks", "reasons", "override")):
            return
        if value.get("verdict") not in VERDICTS:
            self.fail(f"{where}.verdict", f"must be one of {VERDICTS}, got "
                                          f"{value.get('verdict')!r}")
        checks = value.get("checks")
        if not isinstance(checks, list) or not checks:
            self.fail(f"{where}.checks", "must be a non-empty array (a gate "
                                         "needs at least one check)")
        else:
            for i, check in enumerate(checks):
                at = f"{where}.checks[{i}]"
                if not self.obj(at, check, ("name", "verdict"),
                                ("name", "value", "warn", "fail", "direction",
                                 "verdict", "detail")):
                    continue
                self.string(f"{at}.name", check.get("name"))
                if check.get("verdict") not in CHECK_VERDICTS:
                    self.fail(f"{at}.verdict", f"must be one of "
                                               f"{CHECK_VERDICTS}")
                for key in ("value", "warn", "fail"):
                    self.number(f"{at}.{key}", check.get(key), nullable=True)
                if check.get("direction") not in ("below", "above", None):
                    self.fail(f"{at}.direction", "must be below|above|null")
        reasons = value.get("reasons", [])
        if not isinstance(reasons, list) or not all(isinstance(r, str)
                                                    for r in reasons):
            self.fail(f"{where}.reasons", "must be an array of strings")
        override = value.get("override")
        if override is not None and self.obj(f"{where}.override", override,
                                             ("reason",), ("reason", "by", "at")):
            self.string(f"{where}.override.reason", override.get("reason"),
                        nonempty=True)

    def provenance(self, where: str, value: Any) -> None:
        if not self.obj(where, value, ("tool", "date")):
            return
        self.string(f"{where}.tool", value.get("tool"))
        self.string(f"{where}.date", value.get("date"))
        for key in ("rms_px", "square_mm_measured", "marker_mm_measured"):
            self.number(f"{where}.{key}", value.get(key), nullable=True)

    def intrinsics(self, where: str, value: Any) -> None:
        keys = ("fx", "fy", "cx", "cy", "distortion", "gate", "provenance")
        if not self.obj(where, value, keys, keys + ("valid_radius_px",)):
            return
        for key in ("fx", "fy"):
            self.number(f"{where}.{key}", value.get(key), positive=True)
        for key in ("cx", "cy"):
            self.number(f"{where}.{key}", value.get(key))
        self.number(f"{where}.valid_radius_px", value.get("valid_radius_px"),
                    positive=True, nullable=True)
        dist = value.get("distortion")
        at = f"{where}.distortion"
        if self.obj(at, dist, ("model", "coefficients"),
                    ("model", "coefficients")):
            model = dist.get("model")
            if model not in DISTORTION_COEFFICIENTS:
                self.fail(f"{at}.model", f"must be one of "
                                         f"{tuple(DISTORTION_COEFFICIENTS)}, "
                                         f"got {model!r}")
            else:
                self.numbers(f"{at}.coefficients", dist.get("coefficients"),
                             DISTORTION_COEFFICIENTS[model])
        if "gate" in value:
            self.gate(f"{where}.gate", value["gate"])
        if "provenance" in value:
            self.provenance(f"{where}.provenance", value["provenance"])

    def mount(self, where: str, value: Any) -> None:
        keys = ("parent_link", "T_parent_camera", "nominal", "gate",
                "provenance")
        if not self.obj(where, value, keys, keys):
            return
        self.string(f"{where}.parent_link", value.get("parent_link"),
                    nonempty=True)
        if "T_parent_camera" in value:
            self.pose(f"{where}.T_parent_camera", value["T_parent_camera"])
        nominal = value.get("nominal")
        if nominal is not None and self.obj(
                f"{where}.nominal", nominal, ("T_parent_camera",),
                ("T_parent_camera", "urdf", "kit_version")):
            if "T_parent_camera" in nominal:
                self.pose(f"{where}.nominal.T_parent_camera",
                          nominal["T_parent_camera"])
            for key in ("urdf", "kit_version"):
                self.string(f"{where}.nominal.{key}", nominal.get(key),
                            nullable=True)
        if "gate" in value:
            self.gate(f"{where}.gate", value["gate"])
        if "provenance" in value:
            self.provenance(f"{where}.provenance", value["provenance"])

    def camera(self, where: str, value: Any) -> None:
        if not self.obj(where, value, ("stream",),
                        ("device", "stream", "intrinsics", "mount")):
            return
        stream = value.get("stream")
        if self.obj(f"{where}.stream", stream, ("width", "height"),
                    ("width", "height")):
            for key in ("width", "height"):
                size = stream.get(key)
                if (not isinstance(size, int) or isinstance(size, bool)
                        or size < 1):
                    self.fail(f"{where}.stream.{key}",
                              f"must be a positive integer, got {size!r}")
        device = value.get("device")
        if device is not None:
            self.obj(f"{where}.device", device, (),
                     ("kind", "serial", "by_path", "model", "asset_tag",
                      "seiryu_node"))
        intrinsics = value.get("intrinsics")
        if intrinsics is not None:
            self.intrinsics(f"{where}.intrinsics", intrinsics)
            if (isinstance(stream, Mapping) and isinstance(intrinsics, Mapping)
                    and all(_is_number(stream.get(k)) for k in ("width", "height"))
                    and all(_is_number(intrinsics.get(k)) for k in ("cx", "cy"))
                    and not (0.0 <= float(intrinsics["cx"]) <= stream["width"]
                             and 0.0 <= float(intrinsics["cy"]) <= stream["height"])):
                self.fail(f"{where}.intrinsics",
                          f"principal point ({intrinsics['cx']}, "
                          f"{intrinsics['cy']}) is outside the "
                          f"{stream['width']}x{stream['height']} stream")
        if value.get("mount") is not None:
            self.mount(f"{where}.mount", value["mount"])


def validate(doc: Any, *, source: str = "camera calibration") -> None:
    """Raise :class:`CalibrationFormatError` listing every structural problem
    of ``doc`` (the schema string, required keys, coefficient counts per
    distortion model, finite numbers, unit quaternions)."""
    if isinstance(doc, Mapping) and doc.get("schema") == LEGACY_ROBOT_PROFILE_SCHEMA:
        raise CalibrationFormatError(
            f"{source} is a {LEGACY_ROBOT_PROFILE_SCHEMA} file (the kit's old "
            f"committed profile), which is no longer read: {MIGRATION_HINT}")
    check = _Checker()
    if check.obj("$", doc, ("schema", "robot", "cameras"),
                 ("schema", "robot", "updated", "cameras", "hand",
                  "kinematics")):
        if doc.get("schema") != SCHEMA:
            check.fail("$.schema", f"must be {SCHEMA!r}, got "
                                   f"{doc.get('schema')!r}")
        check.string("$.robot", doc.get("robot"), nonempty=True)
        check.string("$.updated", doc.get("updated", ""))
        cameras = doc.get("cameras")
        if check.obj("$.cameras", cameras):
            for slot, camera in cameras.items():
                check.camera(f"$.cameras.{slot}", camera)
        hand = doc.get("hand")
        if hand is not None and check.obj("$.hand", hand, ("open_gap_m",),
                                          ("open_gap_m", "provenance")):
            gap = hand.get("open_gap_m")
            check.number("$.hand.open_gap_m", gap, positive=True)
            if _is_number(gap) and float(gap) >= 0.2:
                check.fail("$.hand.open_gap_m", f"must be < 0.2 m, got {gap!r}")
            if "provenance" in hand:
                check.provenance("$.hand.provenance", hand["provenance"])
        kinematics = doc.get("kinematics")
        if kinematics is not None:
            check.obj("$.kinematics", kinematics, (), ("neck_compensation",))
    if check.problems:
        raise CalibrationFormatError(
            f"{source} is not a valid {SCHEMA} document:\n  "
            + "\n  ".join(check.problems))


# --------------------------------------------------------------------------- #
# the typed document
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class GateCheck:
    name: str
    verdict: str
    value: Optional[float] = None
    warn: Optional[float] = None
    fail: Optional[float] = None
    direction: Optional[str] = None
    detail: Optional[str] = None


@dataclass(frozen=True)
class Gate:
    """A layer's acceptance verdict, as the writer's gate decided it."""

    verdict: str
    checks: Tuple[GateCheck, ...] = ()
    reasons: Tuple[str, ...] = ()
    #: ``{"reason", "by", "at"}`` when a FAIL was installed anyway
    override: Optional[Mapping[str, Any]] = None

    @property
    def overridden(self) -> bool:
        return self.override is not None

    @classmethod
    def from_json(cls, block: Mapping[str, Any]) -> "Gate":
        return cls(verdict=str(block["verdict"]),
                   checks=tuple(GateCheck(**dict(c)) for c in block["checks"]),
                   reasons=tuple(block.get("reasons") or ()),
                   override=(None if block.get("override") is None
                             else dict(block["override"])))


@dataclass(frozen=True)
class Provenance:
    tool: str
    date: str
    #: the whole block, verbatim (session, board, views, rms_px, notes, ...)
    raw: Mapping[str, Any] = field(default_factory=dict, compare=False)

    @property
    def rms_px(self) -> Optional[float]:
        value = self.raw.get("rms_px")
        return None if value is None else float(value)

    @property
    def notes(self) -> Tuple[str, ...]:
        return tuple(self.raw.get("notes") or ())

    def describe(self) -> str:
        """One line: the tool, the date, the session when there is one."""
        session = self.raw.get("session")
        return f"{self.tool}, {self.date}" + (f", session {session}"
                                               if session else "")

    @classmethod
    def from_json(cls, block: Mapping[str, Any]) -> "Provenance":
        return cls(tool=str(block["tool"]), date=str(block["date"]),
                   raw=dict(block))


@dataclass(frozen=True)
class Distortion:
    model: str
    coefficients: Tuple[float, ...] = ()

    @property
    def is_zero(self) -> bool:
        return all(c == 0.0 for c in self.coefficients)


@dataclass(frozen=True)
class Intrinsics:
    """Layer 1: the lens, for the image as seiryu publishes it."""

    fx: float
    fy: float
    cx: float
    cy: float
    distortion: Distortion
    gate: Gate
    provenance: Provenance
    valid_radius_px: Optional[float] = None


@dataclass(frozen=True)
class Pose:
    xyz_m: Tuple[float, float, float]
    quat_xyzw: Tuple[float, float, float, float]

    @classmethod
    def from_json(cls, block: Mapping[str, Any]) -> "Pose":
        return cls(tuple(float(v) for v in block["xyz_m"]),
                   tuple(float(v) for v in block["quat_xyzw"]))


@dataclass(frozen=True)
class Mount:
    """Layer 2: the ABSOLUTE optical-frame pose in ``parent_link``, with the
    nominal it was fitted against."""

    parent_link: str
    T_parent_camera: Pose
    nominal: Pose
    gate: Gate
    provenance: Provenance
    nominal_urdf: Optional[str] = None
    nominal_kit_version: Optional[str] = None


@dataclass(frozen=True)
class Camera:
    slot: str
    width: int
    height: int
    intrinsics: Optional[Intrinsics] = None
    mount: Optional[Mount] = None
    device: Optional[Mapping[str, Any]] = field(default=None, compare=False)


@dataclass(frozen=True)
class Hand:
    open_gap_m: float
    provenance: Optional[Provenance] = None


@dataclass(frozen=True)
class CameraCalibration:
    """One robot's ``omakase.camera_calibration/2`` file, gate-checked."""

    robot: str
    cameras: Dict[str, Camera]
    hand: Optional[Hand] = None
    updated: str = ""
    #: where it was read from (for messages)
    source: str = ""

    def camera(self, slot: str) -> Optional[Camera]:
        return self.cameras.get(slot)


def _intrinsics(block: Mapping[str, Any]) -> Intrinsics:
    dist = block["distortion"]
    return Intrinsics(
        fx=float(block["fx"]), fy=float(block["fy"]),
        cx=float(block["cx"]), cy=float(block["cy"]),
        distortion=Distortion(str(dist["model"]),
                              tuple(float(c) for c in dist["coefficients"])),
        gate=Gate.from_json(block["gate"]),
        provenance=Provenance.from_json(block["provenance"]),
        valid_radius_px=(None if block.get("valid_radius_px") is None
                         else float(block["valid_radius_px"])))


def _mount(block: Mapping[str, Any]) -> Mount:
    nominal = block["nominal"]
    return Mount(parent_link=str(block["parent_link"]),
                 T_parent_camera=Pose.from_json(block["T_parent_camera"]),
                 nominal=Pose.from_json(nominal["T_parent_camera"]),
                 gate=Gate.from_json(block["gate"]),
                 provenance=Provenance.from_json(block["provenance"]),
                 nominal_urdf=nominal.get("urdf"),
                 nominal_kit_version=nominal.get("kit_version"))


def _enforce_gate(source: str, slot: str, layer: str, gate: Gate, *,
                  allow_failed_gate: bool) -> None:
    if gate.verdict == "FAIL":
        if gate.overridden:
            warnings.warn(f"{source}: camera {slot!r} {layer} FAILED its gate "
                          f"and is installed under an override "
                          f"({gate.override.get('reason')})", stacklevel=4)
            return
        if allow_failed_gate:
            warnings.warn(f"{source}: camera {slot!r} {layer} FAILED its gate "
                          f"({'; '.join(gate.reasons) or 'no reason'}); "
                          f"accepted because the caller allowed a failed gate",
                          stacklevel=4)
            return
        raise FailedCalibrationGate(source, slot, layer, gate.reasons)
    if gate.verdict == "WARN":
        warnings.warn(f"{source}: camera {slot!r} {layer} gate is WARN "
                      f"({'; '.join(gate.reasons) or 'no reason'})",
                      stacklevel=4)


def parse(doc: Mapping[str, Any], *, allow_failed_gate: bool = False,
          source: str = "camera calibration") -> CameraCalibration:
    """``doc`` validated (:func:`validate`), gate-checked, typed."""
    validate(doc, source=source)
    cameras: Dict[str, Camera] = {}
    for slot, block in doc["cameras"].items():
        intrinsics = (None if block.get("intrinsics") is None
                      else _intrinsics(block["intrinsics"]))
        mount = None if block.get("mount") is None else _mount(block["mount"])
        for layer, value in (("intrinsics", intrinsics), ("mount", mount)):
            if value is not None:
                _enforce_gate(source, slot, layer, value.gate,
                              allow_failed_gate=allow_failed_gate)
        cameras[slot] = Camera(slot=slot, width=int(block["stream"]["width"]),
                               height=int(block["stream"]["height"]),
                               intrinsics=intrinsics, mount=mount,
                               device=block.get("device"))
    hand = doc.get("hand")
    return CameraCalibration(
        robot=str(doc["robot"]), cameras=cameras, updated=str(doc.get("updated", "")),
        hand=None if hand is None else Hand(
            float(hand["open_gap_m"]),
            None if hand.get("provenance") is None
            else Provenance.from_json(hand["provenance"])),
        source=source)


def load(path: Union[str, Path], *,
         allow_failed_gate: bool = False) -> CameraCalibration:
    """Read, validate and gate-check one robot's calibration file."""
    path = Path(path).expanduser()
    try:
        doc = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise FileNotFoundError(f"no camera calibration at {path}") from None
    except json.JSONDecodeError as exc:
        raise CalibrationFormatError(f"{path} is not JSON: {exc}") from None
    return parse(doc, allow_failed_gate=allow_failed_gate, source=str(path))


def installed_path() -> Optional[Path]:
    """:data:`DEFAULT_PATH` when this machine has one (a robot), else
    ``None`` — the default an entry point's ``--robot-profile`` falls back to.
    Library calls take a path."""
    return DEFAULT_PATH if DEFAULT_PATH.is_file() else None


__all__ = ["CalibrationFormatError", "Camera", "CameraCalibration",
           "DEFAULT_PATH", "DISTORTION_COEFFICIENTS", "Distortion",
           "FailedCalibrationGate", "Gate", "GateCheck", "HEAD_SLOT", "Hand",
           "Intrinsics", "LEGACY_ROBOT_PROFILE_SCHEMA", "MIGRATION_HINT",
           "Mount", "Pose", "Provenance", "SCHEMA", "SCHEMA_PATH",
           "WRIST_SLOTS", "installed_path", "load", "parse", "validate"]
