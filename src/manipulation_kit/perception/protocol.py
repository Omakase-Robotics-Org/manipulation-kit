"""What a camera stack owes the kit — and the kit rules it must not re-derive.

The kit still opens no camera and decodes no image. A PERCEIVER is whatever
does (a model looking at a photograph, a RealSense, an ArUco rig, a person
with a tape); this module is the shape of what it hands over, so a customer
with a different camera stack implements four methods instead of forking an
example:

    cameras()                      the camera MODELS it has (pixel <-> base)
    locate(camera, u, v, plane=)   a pixel -> a base-frame contact point
    declare(objects, support=)     objects -> a WorldView, LIFTED onto what
                                   they stand on
    attached(side)                 what a hand is carrying, if anything

:class:`ScenePerceiver` is the reference implementation over a camera
mapping and a :class:`~manipulation_kit.world.WorldView`; the kit rules it
applies — :func:`support_plane` and :func:`lift_onto_support` — are plain
functions so an example that is not a perceiver uses the SAME rule instead of
open-coding ``p[2] + size[2] / 2`` (which ignores ``frame_id`` and rotation,
design L5/L15).

The neck and lift states are typed here as Protocols with the attribute names
the firmware executor's ``neck_state()`` / ``lift_state()`` return.
"""

from __future__ import annotations

import dataclasses
from typing import (TYPE_CHECKING, Dict, List, Mapping, Optional, Sequence,
                    Tuple)

import numpy as np

try:                                    # 3.8+: typing; kept explicit for 3.9
    from typing import Protocol, runtime_checkable
except ImportError:                     # pragma: no cover
    from typing_extensions import Protocol, runtime_checkable  # type: ignore

from ..world.frames import BASE, FrameError, FrameGraph
from ..world.views import ObjectView, SurfaceView, WorldView

if TYPE_CHECKING:                       # pragma: no cover
    from .camera import Located

# The neck and lift a head camera is posed by are the executor's own typed
# state, ``manipulation_kit.executor.NeckState`` / ``LiftState`` (read through
# the d1-firmwared OpenAPI client by ``FirmwareExecutor``); perception has no
# parallel description of them.


@runtime_checkable
class CameraModel(Protocol):
    """A camera as geometry: what :class:`~.camera.PinholeCamera` is."""

    width: int
    height: int

    def ray(self, u: float, v: float) -> np.ndarray: ...

    def project(self, p_base) -> Tuple[float, float]: ...

    def locate(self, u: float, v: float, *, plane_z: float,
               plane_source: str = ...,
               plane_uncertainty_m: float = ...) -> "Located": ...


@runtime_checkable
class Perceiver(Protocol):
    """What a camera stack owes the kit. The kit still opens no camera."""

    def cameras(self) -> Mapping[str, CameraModel]: ...

    def locate(self, camera: str, u: float, v: float, *,
               plane: Optional[SurfaceView] = None) -> "Located": ...

    def declare(self, objects: Sequence[ObjectView], *,
                support: Optional[SurfaceView] = None) -> WorldView: ...

    def attached(self, side: str) -> Optional[ObjectView]: ...


class NoSupport(LookupError):
    """There is no surface for a pixel to land on."""


def support_plane(surfaces: Sequence[SurfaceView], frames: FrameGraph
                  ) -> Optional[Tuple[SurfaceView, float]]:
    """The HIGHEST surface whose frame resolves, and its top z in base.

    The thing on top is the one a head-camera pixel meets first (a wagon on a
    floor). ``top_z`` resolves the surface's frame and rotation; a surface
    whose frame does not resolve is skipped, not guessed at.
    """
    best: Optional[Tuple[SurfaceView, float]] = None
    for surface in surfaces:
        try:
            top = surface.top_z(frames)
        except FrameError:
            continue
        if best is None or top > best[1]:
            best = (surface, top)
    return best


def _support_for(obj: ObjectView, surfaces: Sequence[SurfaceView],
                 frames: FrameGraph) -> Optional[Tuple[SurfaceView, float]]:
    """The highest surface this object stands OVER (footprint included)."""
    try:
        centre = obj.pose_in_base(frames)[0]
    except FrameError:
        return None
    reach = 0.5 * float(max(obj.size[0], obj.size[1]))
    under = []
    for surface in surfaces:
        try:
            if surface.over(centre, frames, pad_m=reach):
                under.append(surface)
        except FrameError:
            continue
    return support_plane(under, frames)


def lift_onto_support(objects: Sequence[ObjectView],
                      surfaces: Sequence[SurfaceView], frames: FrameGraph, *,
                      support: Optional[SurfaceView] = None,
                      tol_m: float = 0.002
                      ) -> Tuple[Tuple[ObjectView, ...], List[str]]:
    """Declared objects, with any that sink into their support lifted onto it.

    THE DESCENT FLOOR TRUSTS THE OBJECT'S BOTTOM, so a declared bottom below
    the table top would send the pad tips into the table (design review 4,
    design L5). An object whose RESOLVED underside
    (:meth:`ObjectView.bottom_z`) is more than ``tol_m`` below its support's
    RESOLVED top (:meth:`SurfaceView.top_z`) is raised, along base +z, until
    it stands on it. Both resolve ``frame_id`` and rotation — a wagon frame
    that is yawed and offset is exactly where ``p[2] + size[2]/2`` was wrong.

    The support is ``support`` when given, otherwise the highest of
    ``surfaces`` the object is over. An object over no known surface is left
    where it was declared, and the note says so. Objects are never LOWERED:
    something above the table may be held, or on something else.

    Returns the objects (surfaces unchanged) and one note per correction.
    """
    notes: List[str] = []
    out: List[ObjectView] = []
    pool = [s for s in surfaces if isinstance(s, SurfaceView)]
    for obj in objects:
        if isinstance(obj, SurfaceView):
            out.append(obj)
            continue
        if support is not None:
            try:
                found: Optional[Tuple[SurfaceView, float]] = (
                    support, support.top_z(frames))
            except FrameError as exc:
                raise NoSupport(f"the support {support.name!r} does not "
                                f"resolve: {exc}") from exc
        else:
            found = _support_for(obj, pool, frames)
        if found is None:
            if pool:
                notes.append(f"{obj.name} is not over any known surface; "
                             f"left where it was declared")
            out.append(obj)
            continue
        surface, top = found
        try:
            bottom = obj.bottom_z(frames)
        except FrameError as exc:
            notes.append(f"{obj.name}: its frame does not resolve ({exc}); "
                         f"not lifted")
            out.append(obj)
            continue
        if bottom < top - float(tol_m):
            dz = top - bottom
            lift = np.array([0.0, 0.0, dz])
            if obj.frame_id != BASE:
                # base +z, expressed in the frame the object was declared in
                lift = frames.pose_in_base(obj.frame_id)[1].inv().apply(lift)
            obj = dataclasses.replace(obj, p=np.asarray(obj.p) + lift)
            notes.append(f"{obj.name} raised {dz * 1000:.0f} mm so its bottom "
                         f"sits on the top of {surface.name!r} ({top:.3f} m)")
        out.append(obj)
    return tuple(out), notes


def replace_by_name(world: WorldView, objects: Sequence[ObjectView]
                    ) -> WorldView:
    """``world`` with ``objects`` replacing any of the same name, appended
    otherwise, in the world's own order."""
    incoming: Dict[str, ObjectView] = {o.name: o for o in objects}
    kept = [incoming.pop(o.name, o) for o in world.objects]
    kept += [o for o in objects if o.name in incoming]
    return world.with_(objects=tuple(kept))


class ScenePerceiver:
    """The reference :class:`Perceiver`: camera models over a world.

    It does no detection at all — that is what a subclass (the examples'
    detector) or a model adds. What it does is the kit's share:
    turn a pixel into a contact point on the right plane with the plane's
    OWN provenance and height uncertainty, and turn declared objects into a
    world with the support rule applied.
    """

    def __init__(self, cameras: Optional[Mapping[str, CameraModel]] = None,
                 world: Optional[WorldView] = None):
        self._cameras: Dict[str, CameraModel] = dict(cameras or {})
        self.world: WorldView = world if world is not None else WorldView()
        #: the corrections :meth:`declare` made last time, for the caller to
        #: report back (a model is told when its declaration was moved).
        self.notes: List[str] = []

    def cameras(self) -> Mapping[str, CameraModel]:
        return dict(self._cameras)

    def locate(self, camera: str, u: float, v: float, *,
               plane: Optional[SurfaceView] = None) -> "Located":
        """A pixel -> the contact point on ``plane`` (default: the highest
        surface in the world), carrying that surface's provenance."""
        model = self._cameras.get(camera)
        if model is None:
            raise KeyError(f"no camera {camera!r}; this perceiver has "
                           f"{sorted(self._cameras)}")
        if plane is not None:
            top = plane.top_z(self.world.frames)
        else:
            found = support_plane(self.world.surfaces(), self.world.frames)
            if found is None:
                raise NoSupport("there is no surface in the world for a pixel "
                                "to land on")
            plane, top = found
        return model.locate(u, v, plane_z=top,
                            plane_source=plane_source_of(plane),
                            plane_uncertainty_m=float(
                                plane.height_uncertainty_m or 0.0))

    def declare(self, objects: Sequence[ObjectView], *,
                support: Optional[SurfaceView] = None) -> WorldView:
        surfaces = [o for o in objects if isinstance(o, SurfaceView)]
        surfaces += list(self.world.surfaces())
        lifted, self.notes = lift_onto_support(objects, surfaces,
                                               self.world.frames,
                                               support=support)
        self.world = replace_by_name(self.world, lifted)
        return self.world

    def attached(self, side: str) -> Optional[ObjectView]:
        """The object the ``side`` gripper REPORTS holding, at its last
        declared pose. Step 7 (``world/attach.py``) gives it the attached
        pose; until then this is identity only, never a moved position."""
        gripper = self.world.gripper(side)
        if gripper is None or not gripper.holding or not gripper.held_object:
            return None
        return self.world.find(gripper.held_object)


def plane_source_of(surface: SurfaceView) -> str:
    """How a surface's height is known, in the words a model reads."""
    how = surface.plane_source or "declared"
    return (f"the top of {surface.name!r}, {how}, confidence "
            f"{surface.confidence:.1f}")


__all__ = ["CameraModel", "NoSupport",
           "Perceiver", "ScenePerceiver", "lift_onto_support",
           "plane_source_of", "replace_by_name", "support_plane"]
