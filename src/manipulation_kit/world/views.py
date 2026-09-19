"""What the kit is willing to be told about the world, and nothing more.

These are PERCEPTION RESULTS, not perception. The kit never imports a camera:
``d1-inference/scene`` (table homography, ArUco wagon frame, the detectors),
the Isaac env server and hand-written scripts are the producers; everything
here only reads.

Three rules, each of them a bug somebody shipped:

1. **``size`` is mandatory.** Every clearance a primitive computes — how far
   to stand off, how deep to descend, whether the jaws even close on this —
   comes out of the object's extent. An optional size is how a grasp silently
   becomes a collision, so there is no default.
2. **``frame_id`` travels with the pose.** See :mod:`.frames`. A pose is a
   number plus the frame it was measured in, and resolving it can FAIL.
3. **The measured half and the intended half are different fields.**
   :class:`GripperView.holding` is what the torque-stop mechanism reports;
   ``target_closedness`` is what somebody asked for. Verifiers read the first
   one only.

Serialisation is part of the contract, not a debug aid: :meth:`WorldView.to_text`
is what a typed-choice model (Jev) is shown instead of an image, and
:meth:`WorldView.to_json` is what a trace record stores. Both are stable and
both are tested for size — a world description that grows without bound is a
prompt that silently stops fitting.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from .frames import BASE, FrameGraph

_SIDES = ("left", "right")


def _vec3(value, what: str) -> np.ndarray:
    array = np.asarray(value, dtype=float).reshape(3)
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{what} must be three finite numbers, got {value!r}")
    return array


def _rot(value, what: str) -> R:
    if value is None:
        return R.identity()
    if not isinstance(value, R):
        raise TypeError(f"{what} must be a scipy Rotation")
    return value


def _round(values: Iterable[float], places: int = 3) -> list:
    return [round(float(v), places) for v in values]


@dataclass(frozen=True)
class ObjectView:
    """One named thing with a measured extent, in the frame it was seen in."""

    name: str
    p: np.ndarray
    size: np.ndarray            # measured l, w, h in the object's own axes [m]
    r: R = field(default_factory=R.identity)
    frame_id: str = BASE
    kind: str = "object"        # object | container | surface
    colour: Optional[str] = None
    confidence: float = 1.0
    stamp: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "p", _vec3(self.p, f"{self.name}.p"))
        size = _vec3(self.size, f"{self.name}.size")
        if np.any(size <= 0.0):
            raise ValueError(f"{self.name}.size must be positive in every axis, "
                             f"got {size.tolist()} — an object with no measured "
                             f"extent cannot be grasped safely")
        object.__setattr__(self, "size", size)
        object.__setattr__(self, "r", _rot(self.r, f"{self.name}.r"))
        if not self.name:
            raise ValueError("an object needs a name the model can say")

    # -- geometry ---------------------------------------------------------- #
    def pose_in_base(self, frames: FrameGraph) -> Tuple[np.ndarray, R]:
        """``(p, r)`` in :data:`~manipulation_kit.world.frames.BASE`.

        Raises :class:`~manipulation_kit.world.frames.FrameError`; callers in
        :mod:`manipulation_kit.primitives` turn that into a typed refusal.
        """
        return frames.to_base(self.p, self.r, frame_id=self.frame_id)

    def axes_in_base(self, frames: FrameGraph) -> np.ndarray:
        """The object's three body axes as columns, in the base frame."""
        return self.pose_in_base(frames)[1].as_matrix()

    def principal_axis(self, frames: FrameGraph, *,
                       tie_m: float = 0.005) -> Optional[np.ndarray]:
        """Unit vector along the LONGEST horizontal body axis, base frame.

        ``None`` when the two horizontal extents differ by less than ``tie_m``:
        a square footprint has no principal axis, and inventing one rolls the
        wrist for nothing. Primitives then keep the approach set's own default
        jaw orientation.
        """
        axes = self.axes_in_base(frames)
        horizontal = []
        for i in range(3):
            axis = axes[:, i]
            span = float(self.size[i]) * math.hypot(axis[0], axis[1])
            horizontal.append((span, axis))
        horizontal.sort(key=lambda item: item[0], reverse=True)
        if horizontal[0][0] - horizontal[1][0] < tie_m:
            return None
        axis = np.array([horizontal[0][1][0], horizontal[0][1][1], 0.0])
        norm = float(np.linalg.norm(axis))
        return None if norm < 1e-9 else axis / norm

    def min_horizontal_extent(self) -> float:
        """Smallest of the three measured extents — what the jaws must span.

        Deliberately the smallest of ALL THREE rather than of the two
        horizontal ones: a parallel gripper approaching from any direction in
        the kit's approach set closes across one of them, and the conservative
        reading of "does this fit" is the one that does not depend on which.
        """
        return float(np.min(self.size))

    def height(self) -> float:
        return float(self.size[2])

    def to_json(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "name": self.name, "kind": self.kind, "frame_id": self.frame_id,
            "p": _round(self.p), "size": _round(self.size),
            "quat_xyzw": _round(self.r.as_quat(), 4),
            "confidence": round(float(self.confidence), 3),
            "stamp": round(float(self.stamp), 3),
        }
        if self.colour:
            out["colour"] = self.colour
        return out

    def to_text(self) -> str:
        colour = f" {self.colour}" if self.colour else ""
        yaw = math.degrees(self.r.as_euler("xyz")[2])
        return (f"{self.name}{colour}: at ({self.p[0]:.3f}, {self.p[1]:.3f}, "
                f"{self.p[2]:.3f}) in {self.frame_id}, "
                f"{self.size[0] * 1000:.0f}x{self.size[1] * 1000:.0f}x"
                f"{self.size[2] * 1000:.0f}mm, yaw {yaw:+.0f}deg")


@dataclass(frozen=True)
class ContainerView(ObjectView):
    """An object with an INSIDE — a box, a cup, a tray.

    ``interior`` is the usable inner extent (l, w, h) about the container's own
    centre, and it is what :class:`~manipulation_kit.primitives.Place`'s
    verifier tests membership of. It is separate from ``size`` because a wall
    thickness of 8 mm is the difference between "in the box" and "balanced on
    the rim".
    """

    kind: str = "container"
    interior: Optional[np.ndarray] = None
    #: height of the rim above the container's centre [m]; ``None`` = size/2
    rim_height_m: Optional[float] = None

    def __post_init__(self) -> None:
        super().__post_init__()
        interior = (self.size * 0.9 if self.interior is None
                    else _vec3(self.interior, f"{self.name}.interior"))
        if np.any(interior <= 0.0) or np.any(interior > self.size + 1e-9):
            raise ValueError(f"{self.name}.interior must be positive and no "
                             f"larger than size")
        object.__setattr__(self, "interior", interior)

    def rim_z(self, frames: FrameGraph) -> float:
        """World z of the rim — where a carried object must clear."""
        p, _ = self.pose_in_base(frames)
        lift = (float(self.size[2]) / 2.0 if self.rim_height_m is None
                else float(self.rim_height_m))
        return float(p[2]) + lift

    def contains(self, point, frames: FrameGraph, *, pad_m: float = 0.0) -> bool:
        """Is ``point`` (base frame) inside the interior AABB, in the container's axes?"""
        p, r = self.pose_in_base(frames)
        local = r.inv().apply(np.asarray(point, dtype=float).reshape(3) - p)
        half = np.asarray(self.interior, dtype=float) / 2.0 + float(pad_m)
        return bool(np.all(np.abs(local) <= half))

    def to_json(self) -> Dict[str, Any]:
        out = super().to_json()
        out["interior"] = _round(self.interior)
        return out

    def to_text(self) -> str:
        return (super().to_text() + f", interior "
                f"{self.interior[0] * 1000:.0f}x{self.interior[1] * 1000:.0f}"
                f"x{self.interior[2] * 1000:.0f}mm")


@dataclass(frozen=True)
class SurfaceView(ObjectView):
    """A table, a shelf, a wagon top — something to put things ON."""

    kind: str = "surface"

    def top_z(self, frames: FrameGraph) -> float:
        p, _ = self.pose_in_base(frames)
        return float(p[2]) + float(self.size[2]) / 2.0

    def supports(self, point, frames: FrameGraph, *, pad_m: float = 0.0) -> bool:
        """Is ``point`` over this surface's footprint and within 30 mm of its top?"""
        p, r = self.pose_in_base(frames)
        local = r.inv().apply(np.asarray(point, dtype=float).reshape(3) - p)
        half = np.asarray(self.size, dtype=float) / 2.0 + float(pad_m)
        return bool(abs(local[0]) <= half[0] and abs(local[1]) <= half[1]
                    and -0.005 <= local[2] - half[2] <= 0.030 + float(pad_m))


@dataclass(frozen=True)
class ArmView:
    """One arm as MEASURED: its joints, and where its tool point ended up."""

    side: str
    joints: np.ndarray                 # 7 joint angles [rad]
    tool_p: Optional[np.ndarray] = None    # base frame [m]; None = not reported
    tool_r: Optional[R] = None
    mode: str = "unknown"              # firmware ArmMode, or "unknown"
    error_code: int = 0
    stationary: bool = True

    def __post_init__(self) -> None:
        if self.side not in _SIDES:
            raise ValueError(f"side must be one of {_SIDES}, got {self.side!r}")
        joints = np.asarray(self.joints, dtype=float).reshape(-1)
        if joints.size != 7 or not np.all(np.isfinite(joints)):
            raise ValueError(f"{self.side} arm: joints must be 7 finite radians")
        object.__setattr__(self, "joints", joints)
        if self.tool_p is not None:
            object.__setattr__(self, "tool_p", _vec3(self.tool_p, "tool_p"))

    def to_json(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "side": self.side, "joints_deg": _round(np.degrees(self.joints), 1),
            "mode": self.mode, "error_code": int(self.error_code),
            "stationary": bool(self.stationary),
        }
        if self.tool_p is not None:
            out["tool_p"] = _round(self.tool_p)
        return out

    def to_text(self) -> str:
        where = ("" if self.tool_p is None else
                 f" tool at ({self.tool_p[0]:.3f}, {self.tool_p[1]:.3f}, "
                 f"{self.tool_p[2]:.3f})")
        return f"{self.side} arm: mode {self.mode}{where}"


@dataclass(frozen=True)
class GripperView:
    """One gripper as MEASURED — the torque-stop verdict, not the command.

    ``holding`` is ``GripperReport.holding`` off the wire: the closing stroke
    met something and stopped squeezing at the preset's torque. It is the only
    field a :class:`~manipulation_kit.primitives.Grasp` verifier is allowed to
    believe, and ``held_object`` is the caller's own bookkeeping beside it.
    """

    side: str
    closedness: float                  # 0 open .. 1 closed, measured
    holding: bool = False
    jaw_gap_m: Optional[float] = None  # measured pad-to-pad gap, when known
    held_object: Optional[str] = None
    grip: str = "firm"                 # soft | firm | strong

    def __post_init__(self) -> None:
        if self.side not in _SIDES:
            raise ValueError(f"side must be one of {_SIDES}, got {self.side!r}")
        if not math.isfinite(self.closedness) or not 0.0 <= self.closedness <= 1.0:
            raise ValueError(f"{self.side} gripper: closedness must be in [0, 1]")

    def to_json(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "side": self.side, "closedness": round(float(self.closedness), 3),
            "holding": bool(self.holding), "grip": self.grip,
        }
        if self.jaw_gap_m is not None:
            out["jaw_gap_m"] = round(float(self.jaw_gap_m), 4)
        if self.held_object:
            out["held_object"] = self.held_object
        return out

    def to_text(self) -> str:
        what = f" ({self.held_object})" if self.held_object else ""
        gap = "" if self.jaw_gap_m is None else f", gap {self.jaw_gap_m * 1000:.0f}mm"
        return (f"{self.side} gripper: "
                f"{'HOLDING' + what if self.holding else 'empty'}, "
                f"closedness {self.closedness:.2f}{gap}")


@dataclass(frozen=True)
class WorldView:
    """One observation: the frames, the things, and both arms — that is all.

    It is a VALUE. A verifier is handed the world before a primitive ran and
    called with the world after, and the difference between the two is the
    measurement. Nothing here can be asked to refresh itself, on purpose: a
    "world" that can go and look again is a world whose verdict depends on
    when you asked.
    """

    frames: FrameGraph = field(default_factory=FrameGraph)
    objects: Tuple[ObjectView, ...] = ()
    arms: Mapping[str, ArmView] = field(default_factory=dict)
    grippers: Mapping[str, GripperView] = field(default_factory=dict)
    stamp: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "objects", tuple(self.objects))
        names = [o.name for o in self.objects]
        duplicates = sorted({n for n in names if names.count(n) > 1})
        if duplicates:
            raise ValueError(f"two things answer to the same name: {duplicates} "
                             f"— a model naming one of them cannot be obeyed")
        object.__setattr__(self, "arms", dict(self.arms))
        object.__setattr__(self, "grippers", dict(self.grippers))

    @classmethod
    def of(cls, objects: Sequence[ObjectView] = (), *,
           frames: Optional[FrameGraph] = None,
           arms: Sequence[ArmView] = (),
           grippers: Sequence[GripperView] = (),
           stamp: float = 0.0) -> "WorldView":
        return cls(frames=frames if frames is not None else FrameGraph(now=stamp),
                   objects=tuple(objects),
                   arms={a.side: a for a in arms},
                   grippers={g.side: g for g in grippers},
                   stamp=stamp)

    # -- lookup ------------------------------------------------------------ #
    def find(self, name: str) -> Optional[ObjectView]:
        for item in self.objects:
            if item.name == name:
                return item
        return None

    def containers(self) -> Tuple[ContainerView, ...]:
        return tuple(o for o in self.objects if isinstance(o, ContainerView))

    def surfaces(self) -> Tuple[SurfaceView, ...]:
        return tuple(o for o in self.objects if isinstance(o, SurfaceView))

    def names(self) -> Tuple[str, ...]:
        return tuple(o.name for o in self.objects)

    def arm(self, side: str) -> Optional[ArmView]:
        return self.arms.get(side)

    def gripper(self, side: str) -> Optional[GripperView]:
        return self.grippers.get(side)

    def holder_of(self, name: str) -> Optional[str]:
        """Which side's gripper reports holding ``name``, if any."""
        for side, gripper in self.grippers.items():
            if gripper.holding and gripper.held_object == name:
                return side
        return None

    def with_(self, **changes: Any) -> "WorldView":
        """A copy with fields replaced — how a test writes "and then"."""
        return WorldView(
            frames=changes.get("frames", self.frames),
            objects=tuple(changes.get("objects", self.objects)),
            arms=changes.get("arms", self.arms),
            grippers=changes.get("grippers", self.grippers),
            stamp=changes.get("stamp", self.stamp))

    # -- serialisation ----------------------------------------------------- #
    def to_json(self) -> Dict[str, Any]:
        return {
            "stamp": round(float(self.stamp), 3),
            "frames": {f.frame_id: {"parent": f.parent, "p": _round(f.p),
                                    "quat_xyzw": _round(f.r.as_quat(), 4),
                                    "stamp": round(float(f.stamp), 3),
                                    "max_age_s": f.max_age_s, "valid": f.valid}
                       for f in self.frames.frames.values()},
            "objects": [o.to_json() for o in self.objects],
            "arms": [self.arms[s].to_json() for s in _SIDES if s in self.arms],
            "grippers": [self.grippers[s].to_json() for s in _SIDES
                         if s in self.grippers],
        }

    def to_text(self) -> str:
        """The world as a typed-choice model is shown it: line per thing.

        Kept flat, metric and short on purpose. This string is a prompt, and a
        prompt that grows with the scene is one that silently stops fitting —
        ``tests/world/test_serialisation.py`` pins the budget.
        """
        lines = ["WORLD (base frame: +x forward, +y robot-left, +z up)"]
        if self.objects:
            lines.append("things:")
            lines += [f"  - {o.to_text()}" for o in self.objects]
        else:
            lines.append("things: none detected")
        for side in _SIDES:
            if side in self.arms:
                lines.append(f"  - {self.arms[side].to_text()}")
            if side in self.grippers:
                lines.append(f"  - {self.grippers[side].to_text()}")
        stale = [f.frame_id for f in self.frames.frames.values()
                 if not f.fresh(self.frames.now)]
        if stale:
            lines.append("stale frames (nothing measured in them can be acted on): "
                         + ", ".join(sorted(stale)))
        return "\n".join(lines)

    def to_json_str(self) -> str:
        return json.dumps(self.to_json(), separators=(",", ":"), sort_keys=False)
