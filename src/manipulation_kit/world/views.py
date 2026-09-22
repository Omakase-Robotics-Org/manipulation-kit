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
from types import MappingProxyType
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from .frames import BASE, FrameError, FrameGraph

_SIDES = ("left", "right")


def _vec3(value, what: str) -> np.ndarray:
    """A frozen 3-vector. The array is COPIED and made read-only.

    ``np.asarray`` alone hands back the caller's own buffer, so a producer
    that reuses one scratch array mutates every observation it ever emitted —
    including the ``before`` world a verifier is holding. A world is a value;
    this is what makes that true rather than merely documented.
    """
    array = np.array(value, dtype=float).reshape(3)
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{what} must be three finite numbers, got {value!r}")
    array.setflags(write=False)
    return array


def _extent_along(axis, size, rot: R) -> float:
    """The object's full extent along a unit ``axis``, in the BASE frame.

    ``sum(|axis . body_axis_i| * size_i)`` — the support width of the oriented
    box along that direction. It reduces to ``size_i`` for an axis-aligned box
    and it is the only reading that survives a yaw or a tilt.

    This replaces the old ``min(size)``, which was called conservative and was
    not: a 100x80x40 mm box passed a 51.96 mm jaw check on its 40 mm extent
    while its derived top-down grasp presented the 80 mm one (R9).
    """
    axis = np.asarray(axis, dtype=float).reshape(3)
    norm = float(np.linalg.norm(axis))
    if norm < 1e-12:
        raise ValueError("extent_along needs a non-degenerate axis")
    axis = axis / norm
    columns = rot.as_matrix()
    return float(sum(abs(float(np.dot(axis, columns[:, i]))) * float(size[i])
                     for i in range(3)))


def _tilt_rad(rot: R) -> float:
    """How far this pose is from having ONE body axis straight up.

    Zero for any yaw about z (a turned block is not a tilted one); the angle
    to the nearest upright otherwise. The primitives refuse above
    :data:`UPRIGHT_TOL_RAD` rather than computing a support height that
    assumes a level box.
    """
    columns = rot.as_matrix()
    best = max(abs(float(columns[2, i])) for i in range(3))
    return float(math.acos(max(0.0, min(1.0, best))))


#: How far from upright a box may sit and still be planned against. Above it
#: the vertical extent, the support height and the jaw geometry all become
#: statements about a shape this v1 does not model, and the honest answer is a
#: refusal rather than a number (R9).
UPRIGHT_TOL_RAD = math.radians(10.0)


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
        a square footprint has no LONG axis, and picking one at random would
        roll the wrist for nothing. It still has faces, though — see
        :meth:`footprint_axis`, which is what a parallel gripper has to square
        itself to.
        """
        span, axis = self._horizontal_axes(frames)[0]
        second = self._horizontal_axes(frames)[1][0]
        if span - second < tie_m:
            return None
        return axis

    def _horizontal_axes(self, frames: FrameGraph):
        """The body axes by horizontal span, longest first, each flattened
        into the ground plane and normalised. Degenerate (vertical) axes drop
        out, because a jaw gap along them means nothing."""
        axes = self.axes_in_base(frames)
        out = []
        for i in range(3):
            axis = axes[:, i]
            flat = np.array([axis[0], axis[1], 0.0])
            norm = float(np.linalg.norm(flat))
            if norm < 1e-9:
                continue
            out.append((float(self.size[i]) * math.hypot(axis[0], axis[1]),
                        flat / norm))
        out.sort(key=lambda item: item[0], reverse=True)
        while len(out) < 2:
            out.append((0.0, np.array([1.0, 0.0, 0.0])))
        return out

    def footprint_axis(self, frames: FrameGraph) -> Optional[np.ndarray]:
        """A horizontal body axis to square the jaws to, base frame.

        The same axis as :meth:`principal_axis` when there IS a long one, and
        the widest horizontal axis when the footprint is square — because a
        square prism has no preferred grasp but it certainly has a wrong one.
        MEASURED, 2026-09-19: a 40 mm cube yawed 11.7 deg on the blocks-eval
        wagon presents **47.3 mm** across base-frame-aligned jaws (53.2 mm at
        the arrangement's 25 deg limit) against a driven opening of 51.96 mm
        and a 43.96 mm graspable width. The jaws closed on two corners, stalled
        at a 46 mm gap and the env reported nothing held — five trials in a
        row. Keeping "the approach set's own default" is only harmless when
        the object happens to be axis-aligned.
        """
        return self._horizontal_axes(frames)[0][1]

    def min_horizontal_extent(self) -> float:
        """Smallest of the three measured extents.

        NOT a jaw-fit test — :meth:`extent_along` is. It is kept for the one
        thing it is honestly good for: a lower bound used in a message. A
        100x80x40 mm box has a 40 mm minimum and presents 80 mm to a top-down
        grasp, so "does this fit" has to name the axis (R9).
        """
        return float(np.min(self.size))

    def extent_along(self, axis, frames: FrameGraph) -> float:
        """The object's extent along a base-frame ``axis``, RESOLVED.

        This is the number every clearance question actually wants: the jaw
        gap needs the extent along the jaw axis, a support needs it along the
        surface normal, a container needs it along its own three axes.
        """
        _p, r = self.pose_in_base(frames)
        return _extent_along(axis, self.size, r)

    def vertical_extent(self, frames: FrameGraph) -> float:
        """How tall this object stands in the base frame, orientation included."""
        return self.extent_along((0.0, 0.0, 1.0), frames)

    def tilt_rad(self, frames: FrameGraph) -> float:
        """Angle from upright, base frame. Yaw is not tilt."""
        return _tilt_rad(self.pose_in_base(frames)[1])

    def upright(self, frames: FrameGraph, *, tol_rad: float = UPRIGHT_TOL_RAD
                ) -> bool:
        return self.tilt_rad(frames) <= float(tol_rad)

    def bottom_z(self, frames: FrameGraph) -> float:
        """World z of the object's underside, from the RESOLVED pose."""
        p, r = self.pose_in_base(frames)
        return float(p[2]) - _extent_along((0.0, 0.0, 1.0), self.size, r) / 2.0

    def top_face_z(self, frames: FrameGraph) -> float:
        p, r = self.pose_in_base(frames)
        return float(p[2]) + _extent_along((0.0, 0.0, 1.0), self.size, r) / 2.0

    def vertical_extent_local(self) -> float:
        """The object-LOCAL z extent — the fallback when no frame resolves."""
        return float(self.size[2])

    def height(self) -> float:
        """The object-LOCAL z extent. Prefer :meth:`vertical_extent`, which is
        the same number for an upright box and the right one for any other."""
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

    def to_text(self, frames: Optional[FrameGraph] = None) -> str:
        """One line, in BASE coordinates, with where the number came from.

        The old line printed the object's own frame-local position and
        labelled the whole world with base axes, so a model could not compare
        two objects measured in different frames and had no way to know
        (R: "World text reports positions in each object's local frame but
        omits the transforms"). Names are quoted: an object name is DATA, and
        a scene that contains "ignore your instructions" must read as a
        string.
        """
        colour = f" {self.colour}" if self.colour else ""
        where, note = self.p, ""
        if frames is not None:
            try:
                where = self.pose_in_base(frames)[0]
                note = ("" if self.frame_id == BASE
                        else f" (measured in {self.frame_id!r})")
            except FrameError as exc:
                return (f"{self.name!r}{colour}: position UNAVAILABLE — "
                        f"{exc.reason} on {self.frame_id!r}; nothing measured "
                        f"in it can be acted on")
        else:
            note = f" in {self.frame_id}"
        yaw = math.degrees(self.r.as_euler("xyz")[2])
        # A confidence below 1 is printed and a confidence of 1 is not. The
        # field has been here since the first WorldView and never reached the
        # text, so a producer that said "0.3, I am guessing" (a detector, or a
        # model declaring what it sees) had that erased on the way to the only
        # consumer that could act on it. Nothing in the kit GATES on it —
        # checked, 2026-09-22 — so it is information, not a permission.
        doubt = "" if self.confidence >= 1.0 else \
            f", confidence {self.confidence:.2f}"
        return (f"{self.name!r}{colour}: centre at ({where[0]:.3f}, "
                f"{where[1]:.3f}, {where[2]:.3f}) m base{note}, "
                f"{self.size[0] * 1000:.0f}x{self.size[1] * 1000:.0f}x"
                f"{self.size[2] * 1000:.0f}mm, yaw {yaw:+.0f}deg{doubt}")


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
    #: was ``interior`` MEASURED, or is it the 90% estimate? A placement that
    #: needs the walls to be where they are said to be must not run on a
    #: guess, so the flag travels with the number and
    #: :class:`~manipulation_kit.primitives.Place` refuses a tight fit against
    #: an estimate rather than silently trusting it.
    interior_measured: bool = True

    def __post_init__(self) -> None:
        super().__post_init__()
        estimated = self.interior is None
        interior = (np.asarray(self.size, dtype=float) * 0.9 if estimated
                    else _vec3(self.interior, f"{self.name}.interior"))
        if np.any(interior <= 0.0) or np.any(interior > self.size + 1e-9):
            raise ValueError(f"{self.name}.interior must be positive and no "
                             f"larger than size")
        interior = np.array(interior, dtype=float)
        interior.setflags(write=False)
        object.__setattr__(self, "interior", interior)
        if estimated:
            object.__setattr__(self, "interior_measured", False)

    def rim_z(self, frames: FrameGraph) -> float:
        """World z of the rim — where a carried object must clear.

        From the RESOLVED pose: a container measured in a table frame that
        sits 30 mm up has its rim 30 mm higher, and reading ``self.p`` here is
        the same frame mistake as R1 one class along.
        """
        p, r = self.pose_in_base(frames)
        lift = (_extent_along((0.0, 0.0, 1.0), self.size, r) / 2.0
                if self.rim_height_m is None else float(self.rim_height_m))
        return float(p[2]) + lift

    def floor_z(self, frames: FrameGraph) -> float:
        """World z of the inner floor a placed object comes to rest on."""
        p, r = self.pose_in_base(frames)
        return float(p[2]) - _extent_along((0.0, 0.0, 1.0), self.interior, r) / 2.0

    def contains(self, point, frames: FrameGraph, *, pad_m: float = 0.0) -> bool:
        """Is ``point`` (base frame) inside the interior AABB, in the container's axes?"""
        p, r = self.pose_in_base(frames)
        local = r.inv().apply(np.asarray(point, dtype=float).reshape(3) - p)
        half = np.asarray(self.interior, dtype=float) / 2.0 + float(pad_m)
        return bool(np.all(np.abs(local) <= half))

    def contains_object(self, obj: "ObjectView", frames: FrameGraph, *,
                        pad_m: float = 0.0) -> bool:
        """Is the whole of ``obj`` — its extent, not its centre — inside?

        A 120 mm bar whose centre sits over a 100 mm bin is not in the bin.
        The centre test passed it (R12); this one measures the object's own
        support width along each of the container's axes.
        """
        p, r = self.pose_in_base(frames)
        op, orot = obj.pose_in_base(frames)
        local = r.inv().apply(np.asarray(op, dtype=float).reshape(3) - p)
        half = np.asarray(self.interior, dtype=float) / 2.0 + float(pad_m)
        columns = r.as_matrix()
        for i in range(3):
            reach = _extent_along(columns[:, i], obj.size, orot) / 2.0
            if abs(float(local[i])) + reach > float(half[i]):
                return False
        return True

    def fits_inside(self, obj: "ObjectView", frames: FrameGraph, *,
                    pad_m: float = 0.0) -> bool:
        """Could ``obj`` fit at all, wherever it were put? Horizontal only."""
        _p, r = self.pose_in_base(frames)
        _op, orot = obj.pose_in_base(frames)
        columns = r.as_matrix()
        for i in (0, 1):
            if (_extent_along(columns[:, i], obj.size, orot)
                    > float(self.interior[i]) + 2 * float(pad_m)):
                return False
        return True

    def to_json(self) -> Dict[str, Any]:
        out = super().to_json()
        out["interior"] = _round(self.interior)
        out["interior_measured"] = bool(self.interior_measured)
        if self.rim_height_m is not None:
            out["rim_height_m"] = round(float(self.rim_height_m), 4)
        return out

    def to_text(self, frames: Optional[FrameGraph] = None) -> str:
        how = "measured" if self.interior_measured else "ESTIMATED at 90% of size"
        return (super().to_text(frames) + f", interior "
                f"{self.interior[0] * 1000:.0f}x{self.interior[1] * 1000:.0f}"
                f"x{self.interior[2] * 1000:.0f}mm ({how})")


@dataclass(frozen=True)
class SurfaceView(ObjectView):
    """A table, a shelf, a wagon top — something to put things ON."""

    kind: str = "surface"

    def top_z(self, frames: FrameGraph) -> float:
        """World z of the top face, from the RESOLVED pose (R1/R9)."""
        p, r = self.pose_in_base(frames)
        return float(p[2]) + _extent_along((0.0, 0.0, 1.0), self.size, r) / 2.0

    def normal(self, frames: FrameGraph) -> np.ndarray:
        """The surface's own up axis in the base frame."""
        _p, r = self.pose_in_base(frames)
        columns = r.as_matrix()
        i = int(np.argmax([abs(float(columns[2, k])) for k in range(3)]))
        axis = np.array(columns[:, i], dtype=float)
        return axis if float(axis[2]) >= 0.0 else -axis

    def level(self, frames: FrameGraph, *, tol_rad: float = UPRIGHT_TOL_RAD
              ) -> bool:
        return _tilt_rad(self.pose_in_base(frames)[1]) <= float(tol_rad)

    def over(self, point, frames: FrameGraph, *, pad_m: float = 0.0) -> bool:
        """Is ``point`` inside the footprint, ignoring height?"""
        p, r = self.pose_in_base(frames)
        local = r.inv().apply(np.asarray(point, dtype=float).reshape(3) - p)
        half = np.asarray(self.size, dtype=float) / 2.0 + float(pad_m)
        return bool(abs(local[0]) <= half[0] + float(pad_m)
                    and abs(local[1]) <= half[1] + float(pad_m))

    def supports(self, point, frames: FrameGraph, *, pad_m: float = 0.0) -> bool:
        """Is ``point`` over the footprint and within 30 mm of the top face?

        Takes a POINT, so a caller that hands it an object CENTRE is asking
        whether the centre floats near the top — which failed a correctly
        placed 80 mm box (R12). :meth:`supports_object` is the one that knows
        about undersides.
        """
        p, r = self.pose_in_base(frames)
        local = r.inv().apply(np.asarray(point, dtype=float).reshape(3) - p)
        half = np.asarray(self.size, dtype=float) / 2.0 + float(pad_m)
        return bool(abs(local[0]) <= half[0] and abs(local[1]) <= half[1]
                    and -0.005 <= local[2] - half[2] <= 0.030 + float(pad_m))

    def supports_object(self, obj: "ObjectView", frames: FrameGraph, *,
                        pad_m: float = 0.0, tol_m: float = 0.005) -> bool:
        """Is ``obj`` standing ON this surface — its UNDERSIDE on the top face?

        Measured along the surface's own normal, and the footprint test uses
        the object's own extent rather than its centre, so an 80 mm box whose
        centre is 40 mm up still passes and a box hanging half off the edge
        does not.
        """
        op, orot = obj.pose_in_base(frames)
        p, r = self.pose_in_base(frames)
        axis = self.normal(frames)
        reach = _extent_along(axis, obj.size, orot) / 2.0
        top = float(np.dot(axis, np.asarray(p, dtype=float))) + \
            _extent_along(axis, self.size, r) / 2.0
        under = float(np.dot(axis, np.asarray(op, dtype=float))) - reach
        if not (-float(tol_m) - float(pad_m) <= under - top
                <= float(tol_m) + float(pad_m)):
            return False
        local = r.inv().apply(np.asarray(op, dtype=float).reshape(3) - p)
        half = np.asarray(self.size, dtype=float) / 2.0 + float(pad_m)
        return bool(abs(local[0]) <= half[0] and abs(local[1]) <= half[1])


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
        if self.tool_r is not None:
            out["tool_quat_xyzw"] = _round(self.tool_r.as_quat(), 4)
        return out

    def approach_axis(self):
        """The tool's own +z in the base frame — the direction it closes along.

        ``None`` when the producer reports no orientation, which is the case a
        ``Nudge`` in the TOOL frame cannot be verified in (R13).
        """
        return None if self.tool_r is None else self.tool_r.as_matrix()[:, 2]

    def to_text(self) -> str:
        where = ("" if self.tool_p is None else
                 f" tool at ({self.tool_p[0]:.3f}, {self.tool_p[1]:.3f}, "
                 f"{self.tool_p[2]:.3f})")
        axis = ""
        if self.tool_r is not None:
            z = self.tool_r.as_matrix()[:, 2]
            axis = (f", closing along ({z[0]:+.2f}, {z[1]:+.2f}, {z[2]:+.2f}) "
                    f"in base axes")
        return f"{self.side} arm: mode {self.mode}{where}{axis}"


@dataclass(frozen=True)
class GripperView:
    """One gripper as MEASURED — the physical verdict, not the command.

    ``holding`` is the PRODUCER's measured verdict, and the producer is the only
    party that can see the half of it the kit cannot: whether a body is actually
    between the two pad faces. On the robot that is ``GripperReport.holding``
    (the closing stroke met something and stopped squeezing at the preset's stop
    torque); in the Isaac harness it is the env's own geometric test. It is
    never a command echoed back, and never a closedness threshold: an object
    thicker than the threshold's implied gap can never be reported held by one,
    which cost the agent-eval harness five trials out of five on 2026-09-19
    (F8 — a 0.6 closure gate on 70 mm pads is a 28 mm ceiling, and the cube is
    40 mm).

    Three MEASURED numbers travel beside it so the kit can check the verdict
    against the object it asked for rather than believing it:

    ``closedness``  0 open .. 1 closed, as the hand reports it.
    ``jaw_gap_m``   the pad-FACE separation [m] — the gap an object of a known
                    width has to be able to make. ``None`` when the producer
                    cannot measure it, and then the width test is skipped
                    rather than manufactured from the stroke.
    ``jaw_stalled`` the jaws were commanded to close and have STOPPED, short of
                    the commanded target — which for a force- or torque-limited
                    drive IS the stop. ``None`` = not measured.

    ``held_object`` is the caller's own bookkeeping beside all of it.
    """

    side: str
    closedness: float                  # 0 open .. 1 closed, measured
    holding: bool = False
    jaw_gap_m: Optional[float] = None  # measured pad-FACE gap, when known
    held_object: Optional[str] = None
    grip: str = "firm"                 # soft | firm | strong
    #: commanded closed AND stopped short of the target; None = not measured
    jaw_stalled: Optional[bool] = None
    #: the pad gap THIS hand reaches driven fully open [m], as its producer
    #: reports it (``HandState.open_gap_m``); None = use the hand
    #: description's nominal driven opening. What a grasp's fit is judged by.
    open_gap_m: Optional[float] = None

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
        if self.jaw_stalled is not None:
            out["jaw_stalled"] = bool(self.jaw_stalled)
        if self.open_gap_m is not None:
            out["open_gap_m"] = round(float(self.open_gap_m), 4)
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
    #: The producer's own observation counter. Two worlds with the same
    #: revision are the same look at the world; a plan is BOUND to the
    #: revision it was checked against and an executor refuses to run it
    #: against a different one (R8). Producers that do not count simply leave
    #: it at 0 and the binding falls back to the stamp and the frame set.
    revision: int = 0
    #: Which firmware contract the ROBOT half of this observation came through
    #: (the executor's ``firmware_spec``: an OpenAPI sha256, ``"kinematic"``),
    #: "" when the producer does not say. Recorded in every plan's binding.
    firmware_spec: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "objects", tuple(self.objects))
        names = [o.name for o in self.objects]
        duplicates = sorted({n for n in names if names.count(n) > 1})
        if duplicates:
            raise ValueError(f"two things answer to the same name: {duplicates} "
                             f"— a model naming one of them cannot be obeyed")
        # COPIES. A producer that keeps its own dict and mutates it in place
        # would otherwise be editing every world it has ever emitted,
        # including the ``before`` a verifier is holding.
        object.__setattr__(self, "arms", MappingProxyType(dict(self.arms)))
        object.__setattr__(self, "grippers", MappingProxyType(dict(self.grippers)))
        if not math.isfinite(float(self.stamp)):
            raise ValueError("WorldView.stamp must be finite")

    @classmethod
    def of(cls, objects: Sequence[ObjectView] = (), *,
           frames: Optional[FrameGraph] = None,
           arms: Sequence[ArmView] = (),
           grippers: Sequence[GripperView] = (),
           stamp: float = 0.0, revision: int = 0,
           firmware_spec: str = "") -> "WorldView":
        return cls(frames=frames if frames is not None else FrameGraph(now=stamp),
                   objects=tuple(objects),
                   arms={a.side: a for a in arms},
                   grippers={g.side: g for g in grippers},
                   stamp=stamp, revision=revision,
                   firmware_spec=str(firmware_spec or ""))

    # -- identity ---------------------------------------------------------- #
    def observation_id(self) -> Tuple[Any, ...]:
        """What makes this observation THIS observation.

        Revision, stamp, the frame set and the object poses. It is what a
        :class:`~manipulation_kit.primitives.types.PlanBinding` records, so a
        plan cannot be run against a different look at the world.
        """
        return (int(self.revision), round(float(self.stamp), 6),
                self.frames.revision(),
                tuple((o.name, round(float(o.stamp), 6),
                       tuple(round(float(v), 6) for v in o.p)) for o in self.objects))

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
        """A copy with fields replaced — how a test writes "and then".

        A new ``stamp`` with no explicit ``frames`` RE-CLOCKS the graph, so
        the transforms age with the world instead of staying young forever
        (the ``with_(stamp=...)`` hole the review found).
        """
        stamp = float(changes.get("stamp", self.stamp))
        if "frames" in changes:
            frames = changes["frames"]
        elif stamp != float(self.stamp):
            frames = self.frames.copy(now=stamp)
        else:
            frames = self.frames
        revision = changes.get("revision", self.revision)
        if "revision" not in changes and stamp != float(self.stamp):
            revision = int(self.revision) + 1
        return WorldView(
            frames=frames,
            objects=tuple(changes.get("objects", self.objects)),
            arms=changes.get("arms", self.arms),
            grippers=changes.get("grippers", self.grippers),
            stamp=stamp, revision=int(revision),
            firmware_spec=changes.get("firmware_spec", self.firmware_spec))

    # -- serialisation ----------------------------------------------------- #
    def to_json(self) -> Dict[str, Any]:
        return {
            "stamp": round(float(self.stamp), 3),
            "revision": int(self.revision),
            "frames_now": round(float(self.frames.now), 3),
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
        lines = ["WORLD (base frame: +x forward, +y robot-left, +z up; "
                 "metres, radians; every position below is a CENTRE)"]
        if self.objects:
            lines.append("things:")
            lines += [f"  - {o.to_text(self.frames)}" for o in self.objects]
        else:
            lines.append("things: none detected")
        holds = [f"{side} hand holds {g.held_object!r}"
                 for side, g in sorted(self.grippers.items())
                 if g.holding and g.held_object]
        if holds:
            lines.append("held: " + "; ".join(holds))
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
