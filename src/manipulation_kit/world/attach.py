"""A held object rides the tool. One grasp-transform rule, for everybody.

Astra review 11: the live adapter rebuilt its scene objects unchanged after
every action, so ``ObjectRose`` measured ZERO rise after a real lift and every
later ``Carry``/``Place`` planned against a pose the object had left. The
planner's hypothetical chain (``primitives.reach``) had its own version of the
opposite mistake — translating the object by the tool's displacement and
ignoring the tool's rotation. Both are the same question, "where is the thing
in the hand now?", and this module is the one answer (design C.9):

    grasp = grasp_transform(world, side="left", name="cube")   # at the stroke
    later = with_attached(world_later, side="left", name="cube", grasp=grasp)
    later.find("cube").provenance                               # "attached"

The pose is the tool pose composed with the object's pose IN THE TOOL FRAME,
recorded once, at the stroke. It is an INFERENCE — a rigid grasp is assumed,
slip is not modelled — and the view says so: ``provenance="attached"``, which
:mod:`manipulation_kit.primitives.verifiers` reads, so an inferred pose is
never reported as a sighting. When the hand lets go, :func:`released` leaves
the object where the hand last had it, ``provenance="predicted"``, until
somebody looks.

Nothing here decides WHICH object is in the hand. The producer does (the
gripper report plus its own association rule) and hands the name in.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from typing import Any, Dict, Optional

import numpy as np
from scipy.spatial.transform import Rotation as R

from .frames import BASE
from .views import GripperView, ObjectView, WorldView

ATTACHED = "attached"
PREDICTED = "predicted"


@dataclass(frozen=True)
class GraspTransform:
    """The object's pose in the TOOL frame of ``side``, recorded at the stroke.

    ``p_in_tool`` / ``r_in_tool`` are what stays constant while the hand
    carries the object; everything else about the object (size, kind,
    interior) is carried by the view itself.
    """

    side: str
    name: str
    p_in_tool: np.ndarray
    r_in_tool: R
    stamp: float = 0.0

    def to_json(self) -> Dict[str, Any]:
        return {"side": self.side, "name": self.name,
                "p_in_tool": [round(float(v), 4) for v in self.p_in_tool],
                "quat_in_tool_xyzw": [round(float(v), 4)
                                      for v in self.r_in_tool.as_quat()],
                "stamp": round(float(self.stamp), 3)}


def _tool(world: WorldView, side: str):
    arm = world.arm(side)
    if arm is None or arm.tool_p is None or arm.tool_r is None:
        raise LookupError(f"the {side} arm reports no tool pose, so nothing "
                          f"can ride it")
    return np.asarray(arm.tool_p, dtype=float), arm.tool_r


def grasp_transform(world: WorldView, *, side: str, name: str
                    ) -> GraspTransform:
    """``name``'s pose in ``side``'s tool frame, as ``world`` has them both.

    Call it with the world AT THE STROKE — the tool on the object, the object
    where it was last seen — and keep the result for as long as the hand holds.
    Raises ``LookupError`` when the object or the tool pose is missing, or the
    object's frame does not resolve.
    """
    item = world.find(name)
    if item is None:
        raise LookupError(f"{name!r} is not in this world")
    p_tool, r_tool = _tool(world, side)
    p_obj, r_obj = item.pose_in_base(world.frames)
    inv = r_tool.inv()
    return GraspTransform(side, name,
                          np.asarray(inv.apply(np.asarray(p_obj) - p_tool),
                                     dtype=float),
                          inv * r_obj, float(world.stamp))


def with_attached(world: WorldView, *, side: str, name: str,
                  grasp: Optional[GraspTransform] = None) -> WorldView:
    """The held object rides the tool: its pose is recomputed from the tool
    pose and the grasp transform recorded at the stroke, and it is marked
    ``provenance='attached'`` so it can never be mistaken for an observation.

    ``grasp=None`` records the transform from ``world`` itself — i.e. ``world``
    IS the stroke instant, and the object's pose does not change on this call.
    The object is re-expressed in ``base`` (it is no longer where its old frame
    put it), and the ``side`` gripper, when present, names it as held.
    """
    if grasp is None:
        grasp = grasp_transform(world, side=side, name=name)
    if grasp.side != side or grasp.name != name:
        raise ValueError(f"that grasp is {grasp.name!r} in the {grasp.side} "
                         f"hand, not {name!r} in the {side}")
    item = world.find(name)
    if item is None:
        raise LookupError(f"{name!r} is not in this world")
    p_tool, r_tool = _tool(world, side)
    p = p_tool + r_tool.apply(grasp.p_in_tool)
    r = r_tool * grasp.r_in_tool
    moved = dataclasses.replace(item, p=p, r=r, frame_id=BASE,
                                provenance=ATTACHED, stamp=float(world.stamp))
    objects = tuple(moved if o.name == name else o for o in world.objects)
    grippers = dict(world.grippers)
    held = grippers.get(side)
    if held is not None and held.holding and held.held_object != name:
        grippers[side] = dataclasses.replace(held, held_object=name)
    return world.with_(objects=objects, grippers=grippers)


def attached(world: WorldView, *, side: str) -> Optional[ObjectView]:
    """The object ``side`` holds, IF its pose is the attached one.

    ``None`` when the gripper reports nothing held, names nothing, or the named
    object's pose is anything other than ``attached`` — an identity with a
    stale pose is exactly what this module exists to stop handing out.
    """
    gripper: Optional[GripperView] = world.gripper(side)
    if gripper is None or not gripper.holding or not gripper.held_object:
        return None
    item = world.find(gripper.held_object)
    if item is None or item.provenance != ATTACHED:
        return None
    return item


def with_measured_width(item: ObjectView, frames: Any, axis,
                        width_m: float) -> ObjectView:
    """``item`` with its extent along the base-frame ``axis`` (the jaw axis of
    the grasp that measured it) set to ``width_m``, and
    ``size_provenance="measured"``.

    A stalled grasp is a MEASUREMENT of the object's width along the jaws, and
    it outranks the declared size (d1-2, 2026-09-23: a tape roll declared
    50 mm was held at 57.1 mm). The object-frame axis most aligned with
    ``axis`` absorbs the difference, so :meth:`ObjectView.extent_along` of the
    result along ``axis`` IS ``width_m`` (for an axis-aligned grasp: exactly
    that side). The other two sides are not measured and stay as declared.
    Raises ``LookupError`` when ``item``'s frame does not resolve.
    """
    a = np.asarray(axis, dtype=float).reshape(3)
    norm = float(np.linalg.norm(a))
    if not norm > 0.0 or not float(width_m) > 0.0:
        raise ValueError(f"a measured width needs a jaw axis and a positive "
                         f"width, got axis={axis!r}, width={width_m!r}")
    _p, r = item.pose_in_base(frames)
    local = np.abs(r.inv().apply(a / norm))
    k = int(np.argmax(local))
    size = np.array(item.size, dtype=float)
    rest = float(sum(size[i] * local[i] for i in range(3) if i != k))
    side = (float(width_m) - rest) / float(local[k])
    size[k] = side if side > 1e-4 else float(width_m)
    return dataclasses.replace(item, size=size, size_provenance="measured")


def released(world: WorldView, *, name: str,
             provenance: str = PREDICTED) -> WorldView:
    """``name`` let go where the hand last had it: the pose stays, and it is a
    PREDICTION (nothing has looked since) unless the caller is a simulator
    whose world is the truth and says ``provenance='observed'``."""
    objects = tuple(dataclasses.replace(o, provenance=provenance)
                    if o.name == name else o for o in world.objects)
    return world.with_(objects=objects)


__all__ = ["ATTACHED", "GraspTransform", "PREDICTED", "attached",
           "grasp_transform", "released", "with_attached",
           "with_measured_width"]
