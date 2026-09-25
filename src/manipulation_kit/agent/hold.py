"""Is the held object in the hand? A look after the grasp can say no.

While a gripper reports holding, the world source publishes the object's
pose as ``provenance="attached"``: the tool pose composed with the grasp
recorded at the stroke — an inference (:mod:`manipulation_kit.world.attach`).
Every later verdict about that object (``lift``'s rise, ``place``'s
underside) is computed from it, so a false hold stays true as long as nobody
looks. d1-2, 2026-09-24: a rim pinch passed as a grasp, the lift "rose
150 mm" by attachment alone, and the model's next two wrist locates pointed
at the roll still on the table.

A ``locate`` of the held object is a look. :func:`hold_sighting` tests the
pixel against the two hypotheses the kit can project into the same camera:

* **held** — the object at its attached pose, riding the hand;
* **left behind** — the object where it was when the grasp was planned.

The pixel on the held object's silhouette corroborates the hold. A pixel off
that silhouette whose table-plane position is at the grasp site contradicts
it (``holding_verified: false``). A view in which the two silhouettes overlap
at the pixel (a camera looking straight down the lift) cannot tell them apart,
and says so. Nothing here moves anything or takes a photo: it reads the look
the loop was already given.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np

from ..world import FrameGraph, ObjectView

#: How far outside a projected silhouette's bounding box a pixel may be and
#: still be ON it [px]: a model's point on a silhouette's edge, not a
#: measurement to the pixel. The recorded contradiction (turn 9) is 54 px
#: off the held silhouette.
SIGHTING_MARGIN_PX = 12.0
#: How near the grasp site a table-plane sighting has to be to be the object
#: left behind, rather than something else on the table [m]: the verifiers'
#: association distance (``verifiers.ASSOCIATION_TOL_M``). The recorded
#: declaration was 57 mm off the roll the jaws closed on.
LEFT_BEHIND_M = 0.08

#: :attr:`HoldSighting.holding_verified` values, by name
CORROBORATED, CONTRADICTED, INCONCLUSIVE = True, False, None

Box = Tuple[float, float, float, float]


def silhouette_px(camera: Any, item: ObjectView,
                  frames: FrameGraph) -> Optional[Box]:
    """``(u_min, v_min, u_max, v_max)`` of ``item``'s eight box corners in
    ``camera``, or ``None`` when a corner is behind the lens or the pose does
    not resolve."""
    try:
        p, r = item.pose_in_base(frames)
        half = np.asarray(item.size, dtype=float).reshape(3) / 2.0
        pixels = [camera.project(np.asarray(p) + r.apply(half * np.array(s)))
                  for s in itertools.product((-1.0, 1.0), repeat=3)]
    except (LookupError, ValueError):
        return None
    us, vs = zip(*pixels)
    return float(min(us)), float(min(vs)), float(max(us)), float(max(vs))


def _outside_px(box: Optional[Box], u: float, v: float) -> Optional[float]:
    """How far ``(u, v)`` is outside ``box`` [px]; 0 inside, None: no box."""
    if box is None:
        return None
    du = max(box[0] - u, 0.0, u - box[2])
    dv = max(box[1] - v, 0.0, v - box[3])
    return float(np.hypot(du, dv))


@dataclass(frozen=True)
class HoldSighting:
    """One look at a held object, against the hold."""

    object: str
    side: str
    camera: str
    pixel: Tuple[float, float]
    #: ``False`` the look contradicts the hold, ``True`` it corroborates it,
    #: ``None`` this view cannot tell
    holding_verified: Optional[bool]
    reason: str
    measured: Dict[str, Any] = field(default_factory=dict)

    def to_json(self) -> Dict[str, Any]:
        return {"object": self.object, "side": self.side,
                "camera": self.camera,
                "pixel": [round(float(c), 1) for c in self.pixel],
                "holding_verified": self.holding_verified,
                "reason": self.reason, "measured": dict(self.measured)}

    def to_text(self) -> str:
        if self.holding_verified is False:
            return (f"HOLD CONTRADICTED: {self.reason}. The {self.side} "
                    f"gripper's hold on {self.object!r} is not verified; its "
                    f"attached pose (and any lift measured from it) is an "
                    f"inference the photo refutes. Open the hand and grasp "
                    f"again")
        return self.reason


def hold_sighting(*, camera_name: str, camera: Any, pixel: Sequence[float],
                  seen_p: Sequence[float], held: ObjectView,
                  grasped: ObjectView, frames: FrameGraph, side: str,
                  margin_px: float = SIGHTING_MARGIN_PX,
                  left_behind_m: float = LEFT_BEHIND_M) -> HoldSighting:
    """Grade one ``locate`` of the object ``side`` holds.

    ``pixel`` is the model's pixel in ``camera``; ``seen_p`` where the
    kit's locate put it on the table plane; ``held`` the object at its
    ATTACHED pose; ``grasped`` the object as the grasp was planned against
    (the world before the stroke).
    """
    u, v = float(pixel[0]), float(pixel[1])
    held_box = silhouette_px(camera, held, frames)
    grasped_box = silhouette_px(camera, grasped, frames)
    off_held = _outside_px(held_box, u, v)
    off_grasped = _outside_px(grasped_box, u, v)
    at_site = float(np.linalg.norm(
        np.asarray(seen_p, dtype=float)[:2]
        - np.asarray(grasped.pose_in_base(frames)[0], dtype=float)[:2]))
    rise = float(held.pose_in_base(frames)[0][2]
                 - grasped.pose_in_base(frames)[0][2])
    measured = {
        "seen_p": [round(float(c), 4) for c in seen_p],
        "grasped_p": [round(float(c), 4)
                      for c in grasped.pose_in_base(frames)[0]],
        "held_p": [round(float(c), 4) for c in held.pose_in_base(frames)[0]],
        "held_provenance": held.provenance,
        "seen_to_grasp_site_m": round(at_site, 4),
        "held_rise_m": round(rise, 4),
        "held_silhouette_px": (None if held_box is None
                               else [round(c) for c in held_box]),
        "grasped_silhouette_px": (None if grasped_box is None
                                  else [round(c) for c in grasped_box]),
        "px_off_held": None if off_held is None else round(off_held, 1),
        "px_off_grasped": (None if off_grasped is None
                           else round(off_grasped, 1)),
        "margin_px": margin_px, "left_behind_m": left_behind_m}

    def result(verified, reason):
        return HoldSighting(held.name, side, camera_name, (u, v), verified,
                            reason, measured)

    on_held = off_held is not None and off_held <= margin_px
    on_grasped = off_grasped is not None and off_grasped <= margin_px
    if on_held and on_grasped:
        return result(INCONCLUSIVE, (
            f"from the {camera_name}, {held.name!r} held and {held.name!r} "
            f"left at the grasp site both cover pixel ({u:.0f}, {v:.0f}): "
            f"this view cannot tell them apart"))
    if on_held:
        return result(CORROBORATED, (
            f"the {camera_name} pixel ({u:.0f}, {v:.0f}) is on {held.name!r} "
            f"as the {side} hand holds it"))
    if at_site <= left_behind_m:
        where = ("" if held_box is None else
                 f", {off_held:.0f} px off where the held one would appear")
        return result(CONTRADICTED, (
            f"the {camera_name} sees {held.name!r} on the table at "
            f"({seen_p[0]:.3f}, {seen_p[1]:.3f}), {at_site * 1000:.0f} mm "
            f"from where it was grasped{where}, while its attached pose is "
            f"{rise * 1000:.0f} mm up in the {side} hand"))
    return result(INCONCLUSIVE, (
        f"the {camera_name} pixel ({u:.0f}, {v:.0f}) is neither on "
        f"{held.name!r} in the {side} hand nor at the grasp site "
        f"({at_site * 1000:.0f} mm away): not evidence about the hold"))


__all__ = ["CONTRADICTED", "CORROBORATED", "HoldSighting", "INCONCLUSIVE",
           "LEFT_BEHIND_M", "SIGHTING_MARGIN_PX", "hold_sighting",
           "silhouette_px"]
