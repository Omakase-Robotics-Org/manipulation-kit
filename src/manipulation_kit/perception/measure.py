"""Pixels -> scene objects in ``base`` metres, and the scene file itself.

Moved from ``examples/agent/perceive.py`` (design C.7, L9). A detector —
whatever produces :class:`Detection` s: a model, the colour-mask fallback
here, a person clicking — ends at the IMAGE; this module begins there.

Whatever produces them, the geometry is the same: the bottom of a silhouette
is where the object meets the plane, moved half a footprint back because that
bottom is the footprint's NEAR edge; the height comes from intersecting the ray
through the top with the vertical above the footprint; a cylinder is measured
at its widest slice and a box at its contact patch.

WHAT IS NOT MEASURED, AND SAYS SO
---------------------------------
Every number carries a ``confidence`` below 1 (:class:`ConfidenceLadder`) and a
note when it was inferred rather than seen: the plane's height, a container's
interior (:data:`~manipulation_kit.world.views.INTERIOR_FRACTION` of the
outside; ``Place`` refuses to drop into a guessed one, by design), an object's
extent along the unseen horizontal axis, and yaw — an axis-aligned silhouette
carries no orientation, so yaw is 0 and the width is written on BOTH
horizontal axes, which makes that 0 harmless rather than a quiet lie.

The colour-mask detector (:func:`detect_objects_mask`) knows two colours and
is not a detector; it is here so the whole pipeline runs and is TESTED with no
network.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy import ndimage

from ..world.views import INTERIOR_FRACTION
from .plane import (PlaneFitError, TableInBase, TablePlane, _dilate, _erode,
                    fit_table_plane, height_above, level_correction_deg,
                    neck_pitch_that_levels, table_in_base, to_hsv)


@dataclass(frozen=True)
class ConfidenceLadder:
    """How much each kind of evidence is allowed to claim, 0..1.

    POLICY, not measurement, so it is an explicit input to :func:`measure`
    and :func:`table_object` rather than literals inside them.
    """

    #: a detected object never claims more than this, however sure the
    #: detector says it is: one view, nominal mount
    detected_cap: float = 0.8
    #: ...and nothing measured against a PROVISIONAL plane claims more than
    #: this, because its every distance scales with a height nobody measured
    provisional_cap: float = 0.2
    #: a caller-declared size adds this much, up to ``declared_size_cap``
    declared_size_bonus: float = 0.2
    declared_size_cap: float = 0.9
    #: the fitted table top: provisional height vs a declared/known one
    surface_provisional: float = 0.2
    surface_measured: float = 0.6


DEFAULT_LADDER = ConfidenceLadder()

#: Colour priors the ``mask`` detector knows, as HSV windows in the same
#: ranges: (h_lo, h_hi, s_lo, s_hi, v_lo, v_hi). These are a FALLBACK — two
#: objects on one wagon — and the reason a model detector exists. They are
#: WIDER than the throw-away script's were (its white was ``V > 200``, which
#: misses the same charger standing in the cup's shadow two frames later);
#: what makes them work is that they are applied only to pixels that already
#: differ from the TABLE's own measured colour, which the script replaced with
#: hand-typed pixel windows.
COLOURS: Dict[str, Tuple[int, int, int, int, int, int]] = {
    "white": (0, 180, 0, 60, 120, 255),
    "brown": (8, 35, 60, 255, 60, 255),
}

#: How far a pixel must differ from the table top's OWN median saturation to
#: count as something standing on it, and how much darker than the top it must
#: be to count on brightness alone. The second one exists to let a dark object
#: through without letting a SHADOW through: a shadow on this wagon is 30-45
#: units darker than the top and keeps its (very low) saturation.
OBJECT_S_MARGIN = 20
OBJECT_V_DROP = 70

#: Name fragments that pick a colour prior when the caller did not name one.
NAME_COLOUR_HINTS = (("cup", "brown"), ("mug", "brown"), ("charger", "white"),
                     ("cube", "white"), ("block", "white"), ("box", "white"))

#: A container's interior, as a fraction of its measured outside, is
#: :data:`manipulation_kit.world.views.INTERIOR_FRACTION` — ONE number, owned
#: by the view that also invents it when a producer gives none (it was 0.85
#: here and 0.9 there, and the operator was told "90 %" about a file written
#: at 85, design L15). NOT measured: a wall thickness is not visible from a
#: single view above and in front, so every object built with it says
#: ``interior_measured: false``.
#: How far below the rim the usable interior starts, metres.
RIM_DROP_M = 0.01

# --------------------------------------------------------------------------- #
# detections
# --------------------------------------------------------------------------- #

@dataclass
class Detection:
    """One named thing in the frame, in PIXELS. No metres here on purpose:
    the detector's job ends at the image and the geometry's begins."""

    name: str
    bbox: Tuple[float, float, float, float]     # x0, y0, x1, y1
    base_px: Tuple[float, float]                # where the footprint touches
    #: ``(u_left, u_right, v)`` of the CONTACT BAND — the bottom few rows of
    #: the silhouette, which is where the object meets the table. A detector
    #: that returns only a bounding box leaves this ``None`` and the bbox's
    #: bottom row is used instead, which over-measures a round or a tilted
    #: footprint; see :func:`from_silhouette`.
    contact_px: Optional[Tuple[float, float, float]] = None
    #: ``(u_left, u_right, v)`` of the WIDEST row of the silhouette — a cup's
    #: rim, not its base. ``None`` when the detector gave only a box, and then
    #: the box's own width is used at half the object's height.
    widest_px: Optional[Tuple[float, float, float]] = None
    #: ``(u, v)`` of the TOP of the silhouette, for the height measurement.
    top_px: Optional[Tuple[float, float]] = None
    kind: str = "object"
    upright: bool = True
    shape: str = "other"                        # box | cylinder | other
    colour: Optional[str] = None
    confidence: float = 0.5
    source: str = "mask"
    notes: List[str] = field(default_factory=list)


def requested_objects(spec: str) -> List[Dict[str, str]]:
    """``"cube:object,cup:container:brown"`` -> the list the detectors take.

    ``name:kind[:colour]``; a missing colour is guessed from the name
    (:data:`NAME_COLOUR_HINTS`) and may stay ``None``.
    """
    out = []
    for token in spec.split(","):
        token = token.strip()
        if not token:
            continue
        parts = token.split(":")
        name = parts[0].strip()
        kind = parts[1].strip() if len(parts) > 1 and parts[1].strip() else "object"
        colour = parts[2].strip() if len(parts) > 2 and parts[2].strip() else None
        if kind not in ("object", "container"):
            raise ValueError(f"{token!r}: {kind!r} is not object|container")
        if colour is None:
            colour = next((c for frag, c in NAME_COLOUR_HINTS
                           if frag in name.lower()), None)
        out.append({"name": name, "kind": kind, "colour": colour})
    if not out:
        raise ValueError("the object list names nothing")
    return out


def _table_polygon_mask(plane: TablePlane) -> np.ndarray:
    """Pixels that are ON the table top: inside the fitted quadrilateral.

    The detectors search here and nowhere else, which is what replaces the
    hand-tuned pixel windows the throw-away script used. A white printer
    behind the wagon is outside the quad and stops being a candidate.
    """
    height, width = plane.shape
    ys, xs = np.mgrid[0:height, 0:width]
    inside = np.ones((height, width), dtype=bool)
    (c1, c2) = plane.far_px
    if plane.near_px is not None:
        corners = [c1, c2, plane.near_px[1], plane.near_px[0]]
    else:                            # near edge out of frame: use the frame's
        corners = [c1, c2, np.array([width - 1.0, height - 1.0]),
                   np.array([0.0, height - 1.0])]
    ordered = np.array(corners, dtype=float)
    centroid = ordered.mean(axis=0)
    for i in range(len(ordered)):
        a, b = ordered[i], ordered[(i + 1) % len(ordered)]
        edge = b - a
        side = edge[0] * (ys - a[1]) - edge[1] * (xs - a[0])
        reference = edge[0] * (centroid[1] - a[1]) - edge[1] * (centroid[0] - a[0])
        inside &= (side * reference) >= 0
    return inside


def table_top_colour(hsv: np.ndarray, plane: TablePlane) -> Tuple[float, float]:
    """The fitted top's own median (S, V) — measured, not assumed.

    This is what makes the fallback portable between frames: the JP wagon's
    top reads S=3 V=186 in one frame and S=4 V=180 in another taken minutes
    later, and every threshold below is relative to it.
    """
    on_top = _table_polygon_mask(plane) & np.asarray(plane.mask, dtype=bool)
    if not on_top.any():
        raise PlaneFitError("no_table", "the fitted top has no pixels inside "
                                        "its own quadrilateral")
    return (float(np.median(hsv[..., 1][on_top])),
            float(np.median(hsv[..., 2][on_top])))


def from_silhouette(blob: np.ndarray, **fields) -> "Detection":
    """A :class:`Detection` from a binary silhouette, contact band and all.

    THE CONTACT BAND is the part worth reading. Where an object meets the
    table is the bottom of its silhouette — and for anything that is not a
    slab seen face-on, that is NOT the bottom corners of its bounding box: a
    cup's bbox bottom-left and bottom-right are its rim and its far side,
    110 mm apart, while the base it actually stands on is 48 mm across.
    Measuring the bottom few rows instead gives the footprint the robot has to
    reach into.
    """
    ys, xs = np.nonzero(blob)
    y0, y1 = float(ys.min()), float(ys.max())
    band = ys >= (y1 - max(2.0, 0.08 * (y1 - y0)))
    contact = xs[band]
    rows = np.flatnonzero(blob.any(axis=1))
    left = np.array([np.flatnonzero(blob[y]).min() for y in rows], dtype=float)
    right = np.array([np.flatnonzero(blob[y]).max() for y in rows], dtype=float)
    widest = int(np.argmax(right - left))
    top = np.flatnonzero(blob[int(rows[0])])
    return Detection(bbox=(float(xs.min()), y0, float(xs.max()), y1),
                     base_px=(float(contact.mean()), y1),
                     contact_px=(float(contact.min()), float(contact.max()), y1),
                     widest_px=(left[widest], right[widest],
                                float(rows[widest])),
                     top_px=(float(top.mean()), y0),
                     **fields)


def detect_objects_mask(image_rgb: np.ndarray, plane: TablePlane,
                        requested: Sequence[Dict[str, str]]) -> List[Detection]:
    """Colour-prior blobs standing on the fitted table. The no-key fallback.

    It knows two colours and it is not a detector; it is here so the whole
    pipeline — plane, footprints, scene file, loop — runs and is TESTED with
    no network. A model detector is the one that generalises.

    Two things it does that the script it replaces did not, both of which are
    what let it run on a frame it was not tuned on:

    * it searches INSIDE THE FITTED TABLE only, instead of inside hand-typed
      pixel windows, so the white printer behind the wagon is not a candidate;
    * it requires a pixel to differ from the TABLE'S OWN measured colour
      before a colour window is even consulted, so the cup's shadow — which is
      the table's colour, 40 units darker — stops being part of the cup. With
      the shadow in, the cup's silhouette was 116 px wide instead of 98 and
      its footprint centre sat 25 mm to the right of the cup.
    """
    hsv = to_hsv(image_rgb)
    inside = _table_polygon_mask(plane)
    s_top, v_top = table_top_colour(hsv, plane)
    hue, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    foreground = inside & ((np.abs(sat - s_top) > OBJECT_S_MARGIN)
                           | (val < v_top - OBJECT_V_DROP))
    out: List[Detection] = []
    for item in requested:
        colour = item.get("colour")
        if colour is None:
            raise ValueError(
                f"the mask detector needs a colour for {item['name']!r}: write "
                f"{item['name']}:{item['kind']}:white|brown, or use a real "
                f"detector")
        if colour not in COLOURS:
            raise ValueError(f"the mask detector knows {sorted(COLOURS)}, "
                             f"not {colour!r}")
        h_lo, h_hi, s_lo, s_hi, v_lo, v_hi = COLOURS[colour]
        hit = (foreground
               & (hue >= h_lo) & (hue <= h_hi)
               & (sat >= s_lo) & (sat <= s_hi)
               & (val >= v_lo) & (val <= v_hi))
        hit = _dilate(_erode(hit, 3), 3)
        labels, count = ndimage.label(hit, np.ones((3, 3), bool))
        if count == 0:
            raise PlaneFitError(
                "no_table", f"nothing {colour} is standing on the table for "
                            f"{item['name']!r}")
        sizes = ndimage.sum_labels(hit, labels, index=np.arange(1, count + 1))
        blob = labels == (int(np.argmax(sizes)) + 1)
        detection = from_silhouette(
            blob, name=item["name"], kind=item["kind"], colour=colour,
            source="mask", confidence=0.4,
            shape="cylinder" if colour == "brown" else "box")
        detection.notes.append(
            "a colour blob, not a recognised object; a model detector is the "
            "one that knows what it is looking at")
        out.append(detection)
    return out

def measure(detection: Detection, camera, table: TableInBase, *,
            interior: Optional[Sequence[float]] = None,
            size: Optional[Sequence[float]] = None,
            interior_fraction: float = INTERIOR_FRACTION,
            ladder: "ConfidenceLadder" = None) -> Dict[str, Any]:
    """One detection -> one scene object, in ``base`` metres.

    ``interior_fraction`` and ``ladder`` are the two POLICIES in here, and
    they are explicit inputs: the fraction of the outside an unmeasured
    interior is taken to be (default: the kit's one
    :data:`~manipulation_kit.world.views.INTERIOR_FRACTION`), and the
    confidence each kind of evidence is allowed (:class:`ConfidenceLadder`).

    Four measurements, and what each one is worth:

    FOOTPRINT CENTRE. The bottom of a silhouette is where the object meets the
    table — ``camera.locate`` puts that pixel on the plane — but it is the
    footprint's NEAR edge, not its middle, because the near side of a cup's
    base is a radius closer to the camera than the base's centre. So the point
    is moved half a footprint away from the camera, along the plane. Without
    that the cup landed 26 mm in front of itself, which is most of a gripper's
    tolerance.

    FOOTPRINT WIDTH, between the two ends of the contact band (or the bounding
    box's bottom corners when the detector gave no band), both lifted onto the
    plane. Real: both ends lie ON the surface.

    HEIGHT, by intersecting the ray through the TOP of the silhouette with the
    vertical through the footprint's far side. "Pixel extent times range over
    fx" over-reports by a fifth, because the top of the object is nearer the
    camera than its base.

    WIDTH, at the height where the silhouette is WIDEST, on a plane lifted to
    that height — a cup's rim, not its base. Measuring a rim against the table
    top made a 95 mm cup 110 mm wide, and 85 % of the wrong one is an interior
    a 50 mm charger does not fit into.

    UNSEEN HORIZONTAL EXTENT: there isn't one. A single view gives one
    silhouette, so the width is written on BOTH horizontal axes and yaw is
    0 — a square footprint is rotation-invariant, which makes that zero
    harmless instead of an invented orientation. ``size_y_measured: false``
    says so in the file.

    EVERY ONE OF THESE INHERITS THE PLANE'S HEIGHT. If ``table.source`` is
    ``provisional``, so is all of this, in proportion — which is exactly why
    the loop's first move is to let the model declare the scene instead.
    """
    x0, y0, x1, y1 = detection.bbox
    base_u, base_v = detection.base_px
    if detection.contact_px is not None:
        left_u, right_u, contact_v = detection.contact_px
    else:
        left_u, right_u, contact_v = x0, x1, base_v

    def on_plane(u, v, lift=0.0):
        return camera.locate(u, v, plane_z=table.z + lift,
                             plane_source=table.source).p

    edge = on_plane(base_u, base_v)
    footprint = float(np.linalg.norm(on_plane(right_u, contact_v)[:2]
                                     - on_plane(left_u, contact_v)[:2]))
    # The bottom of a silhouette is the footprint's NEAR edge. Its centre is
    # half a footprint FURTHER AWAY from the camera, along the plane, and its
    # far side another half beyond that — which is where the TOP of the
    # silhouette stands. (Signs matter here more than anywhere else in the
    # file: this ray is at ~45 deg, so 47 mm of horizontal error in the base
    # point is 41 mm of height error.)
    away = edge[:2] - np.asarray(camera.p, dtype=float)[:2]
    norm = float(np.linalg.norm(away))
    step = away / norm * (footprint / 2.0) if norm > 1e-6 else np.zeros(2)
    contact = np.array(edge, dtype=float)
    contact[:2] = contact[:2] + step
    far_side = np.array(contact, dtype=float)
    far_side[:2] = far_side[:2] + step

    top_u, top_v = detection.top_px or ((x0 + x1) / 2.0, y0)
    height = height_above(camera, far_side, top_u, top_v)
    if not (0.0 < height < 2.0):
        raise PlaneFitError(
            "degenerate",
            f"{detection.name}: the ray through the top of the silhouette "
            f"does not meet the vertical above its footprint (height "
            f"{height:.3f} m). The neck angle or the plane height is wrong.")

    # WIDTH depends on the shape, and honestly so. A cylinder's silhouette is
    # its diameter from every angle, so its widest row IS the measurement.
    # A BOX seen from a corner shows two faces at once and its silhouette is
    # WIDER than it is (the d1-2 charger: 64 mm of silhouette for a 50 mm
    # footprint), so for anything not a cylinder the contact width is the
    # better number and the silhouette is only recorded.
    if detection.widest_px is not None:
        wide_l, wide_r, wide_v = detection.widest_px
        slice_h = min(max(height_above(camera, contact,
                                       (wide_l + wide_r) / 2.0, wide_v), 0.0),
                      height)
        silhouette = float(np.linalg.norm(
            on_plane(wide_r, wide_v, slice_h)[:2]
            - on_plane(wide_l, wide_v, slice_h)[:2]))
    else:
        slice_h = height / 2.0
        silhouette = float(np.linalg.norm(on_plane(x1, base_v, slice_h)[:2]
                                          - on_plane(x0, base_v, slice_h)[:2]))
    width = max(footprint, silhouette) if detection.shape == "cylinder" \
        else footprint

    notes = list(detection.notes)
    notes.append("yaw is 0 and the width is written on BOTH horizontal axes: "
                 "one view gives one silhouette, so a nonzero yaw here would "
                 "be invented")
    notes.append(f"the footprint centre is {footprint / 2 * 1000:.0f} mm "
                 f"behind the bottom of the silhouette, which is the "
                 f"footprint's NEAR edge, not its middle")
    notes.append(f"the table height this is measured against is "
                 f"{table.source} (z = {table.z:.3f} m)")
    measured_size = [round(width, 4), round(width, 4), round(height, 4)]
    if size is not None:
        height = float(size[2])
        width = max(float(size[0]), float(size[1]))
    centre = np.array(contact, dtype=float)
    centre[2] = table.z + height / 2.0
    ladder = DEFAULT_LADDER if ladder is None else ladder
    confidence = min(detection.confidence, ladder.detected_cap)
    if table.source == "provisional":
        confidence = min(confidence, ladder.provisional_cap)
    item: Dict[str, Any] = {
        "name": detection.name, "kind": detection.kind,
        "frame_id": "base",
        "p": [round(float(v), 4) for v in centre],
        "size": ([round(float(v), 4) for v in size] if size is not None
                 else measured_size),
        "yaw_rad": 0.0,
        "confidence": round(confidence, 2),
        "measurement": {
            "source": detection.source,
            "bbox_px": [round(v, 1) for v in detection.bbox],
            "footprint_px": [round(base_u, 1), round(base_v, 1)],
            "contact_band_px": (None if detection.contact_px is None else
                                [round(v, 1) for v in detection.contact_px]),
            "widest_row_px": (None if detection.widest_px is None else
                              [round(v, 1) for v in detection.widest_px]),
            "footprint_width_m": round(footprint, 4),
            "silhouette_width_m": round(silhouette, 4),
            "widest_slice_height_m": round(float(slice_h), 4),
            "table_z_m": round(float(table.z), 4),
            "table_height_source": table.source,
            "upright": bool(detection.upright),
            "shape": detection.shape,
            "size_y_measured": size is not None,
            "size_measured_m": measured_size,
            "yaw_measured": False,
            "notes": notes}}
    if size is not None:
        item["confidence"] = round(min(confidence + ladder.declared_size_bonus,
                                       ladder.declared_size_cap), 2)
        item["measurement"]["notes"].append(
            f"size declared ({detection.name}="
            f"{','.join(format(float(v), 'g') for v in size)}); this frame "
            f"measured {measured_size}")
    if detection.colour:
        item["colour"] = detection.colour
    if detection.kind == "container":
        if interior is not None:
            item["interior"] = [round(float(v), 4) for v in interior]
            item["rim_height_m"] = round(float(interior[2]), 4)
            item["interior_measured"] = True
            item["measurement"]["interior_measured"] = True
            item["measurement"]["notes"].append(
                "interior declared by the caller; nothing in this frame "
                "checked it")
        else:
            interior_h = max(height - RIM_DROP_M, 0.005)
            item["interior"] = [round(width * interior_fraction, 4),
                                round(width * interior_fraction, 4),
                                round(interior_h, 4)]
            item["rim_height_m"] = round(interior_h, 4)
            # THE FLAG THE KIT READS, not just a note. `Place` refuses a
            # placement into a container whose interior is a guess, and a
            # perceived interior IS a guess: a wall thickness is not visible
            # from one view above and in front.
            item["interior_measured"] = False
            item["measurement"]["interior_measured"] = False
            item["measurement"]["notes"].append(
                f"interior = {interior_fraction:g} x the measured outside in "
                f"xy and {RIM_DROP_M * 1000:.0f} mm below the rim in z. "
                f"manipulation_kit's Place REFUSES a placement into an "
                f"estimated interior: measure this once and declare it "
                f"(e.g. {detection.name}=0.08,0.08,0.10)")
    if not detection.upright:
        item["measurement"]["notes"].append(
            "the detector says this object is NOT upright; its footprint and "
            "height are read as if it were, so both are wrong")
    return item


def table_object(table: TableInBase, camera, *, name: str = "table",
                 thickness_m: float = 0.02,
                 depth_m: Optional[float] = None,
                 ladder: "ConfidenceLadder" = None) -> Dict[str, Any]:
    """The fitted top as a ``SurfaceView`` item.

    ``plane_source`` and ``height_uncertainty_m`` are TOP-LEVEL fields of the
    item, not only notes: they are what ``SurfaceView`` carries into the
    world, so a pixel located on this surface later inherits them (Astra
    review 8 — they used to stop at the file).
    """
    ladder = DEFAULT_LADDER if ladder is None else ladder
    extent = table.extent()
    if extent is None:
        raise PlaneFitError("no_table", "no fitted rectangle to place")
    depth, width, yaw = extent
    centre = table.centre
    if depth_m is not None:
        depth = float(depth_m)
    return {
        "name": name, "kind": "surface", "frame_id": "base",
        "p": [round(float(centre[0]), 4), round(float(centre[1]), 4),
              round(float(table.z) - thickness_m / 2.0, 4)],
        "size": [round(float(depth), 4), round(float(width), 4), thickness_m],
        "yaw_rad": round(float(yaw), 4),
        "confidence": (ladder.surface_provisional
                       if table.source == "provisional"
                       else ladder.surface_measured),
        "plane_source": table.source,
        "height_uncertainty_m": round(float(table.uncertainty_m), 4),
        "measurement": {
            "top_z_base_m": round(float(table.z), 4),
            "table_height_source": table.source,
            "height_uncertainty_m": round(float(table.uncertainty_m), 4),
            "depth_measured": table.depth_measured,
            "corners_base": [[round(float(v), 4) for v in c]
                             for c in table.corners],
            "notes": list(table.notes)}}


def build_scene(camera, table: TableInBase,
                detections: Sequence[Detection] = (), *,
                table_name: str = "table",
                table_depth_m: Optional[float] = None,
                interiors: Optional[Dict[str, Sequence[float]]] = None,
                sizes: Optional[Dict[str, Sequence[float]]] = None,
                source_image: Optional[str] = None,
                diagnostics: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """The scene file the agent example's ``--scene`` / ``live.py`` loads.

    With no detector it contains the CAMERA and the table and no things, which
    is the default and is not an empty file: it is everything the model needs
    to fill in the things itself (``astra_loop``'s ``declare_scene`` and
    ``locate``).
    """
    objects: List[Dict[str, Any]] = []
    if table.corners is not None:
        objects.append(table_object(table, camera, name=table_name,
                                    depth_m=table_depth_m))
    for detection in detections:
        objects.append(measure(detection, camera, table,
                               interior=(interiors or {}).get(detection.name),
                               size=(sizes or {}).get(detection.name)))
    readme = [
        "MEASURED FROM ONE HEAD FRAME by manipulation_kit.perception. The only "
        "calibration in it is ROBOT-specific: the head camera's intrinsics "
        "and its pose in `base` from the neck joints and the kit's own "
        "head-camera frame. No scene was measured to produce this.",
        f"table height: {table.source} (z = {table.z:.3f} m, "
        f"+-{table.uncertainty_m * 1000:.0f} mm). ONE CAMERA CANNOT MEASURE "
        f"THE HEIGHT OF THE PLANE IT IS LOOKING AT — twice as far and twice "
        f"as big is the same picture — so this number came from outside the "
        f"geometry and everything in the file scales with it.",
        "base = dual_base (torso platform), +x forward, +y robot LEFT, +z up.",
        "Nothing here is a calibrated extrinsic; the head-camera frame is "
        "nominal geometry plus the head part's design tilt.",
        "Every object carries `confidence` and a `measurement` block saying "
        "what was seen and what was assumed. Read them before trusting a "
        "number to a millimetre.",
    ] + [f"table note: {note}" for note in table.notes]
    if source_image:
        readme.insert(1, f"source frame: {source_image}")
    return {"_README": readme,
            "_perceive": {"camera": camera.to_json(),
                          "table": table.to_json(),
                          "image": source_image,
                          "diagnostics": dict(diagnostics or {})},
            "frames": [],
            "objects": objects}


# --------------------------------------------------------------------------- #
# the debug frame
# --------------------------------------------------------------------------- #

def debug_image(image_rgb: np.ndarray, plane: Optional[TablePlane],
                detections: Sequence[Detection]) -> np.ndarray:
    """The frame with the fit drawn on it. Look at this before believing the
    numbers: a plane fit that latched onto the floor still produces a tidy
    JSON file."""
    out = np.array(image_rgb, dtype=np.uint8, copy=True)
    height, width = out.shape[:2]

    def dot(u, v, colour, radius=4):
        u, v = int(round(u)), int(round(v))
        y0, y1 = max(0, v - radius), min(height, v + radius + 1)
        x0, x1 = max(0, u - radius), min(width, u + radius + 1)
        out[y0:y1, x0:x1] = colour

    def segment(a, b, colour):
        steps = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2) + 1
        for t in np.linspace(0.0, 1.0, steps):
            u, v = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
            if 0 <= int(v) < height and 0 <= int(u) < width:
                dot(u, v, colour, radius=1)

    if plane is not None:
        c1, c2 = plane.far_px
        segment(c1, c2, (0, 200, 255))
        if plane.near_px is not None:
            n1, n2 = plane.near_px
            segment(n1, n2, (0, 200, 255))
            segment(c1, n1, (0, 160, 255))
            segment(c2, n2, (0, 160, 255))
        for corner in (c1, c2) + (plane.near_px or ()):
            dot(corner[0], corner[1], (0, 255, 0), radius=5)
    for detection in detections:
        x0, y0, x1, y1 = detection.bbox
        for pair in (((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
                     ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))):
            segment(pair[0], pair[1], (255, 0, 0))
        dot(detection.base_px[0], detection.base_px[1], (255, 0, 255), radius=5)
    return out


@dataclass
class FrameResult:
    """What :func:`perceive_frame` saw: the scene, and the evidence for it."""

    scene: Dict[str, Any]
    plane: TablePlane
    table: TableInBase
    detections: List[Detection] = field(default_factory=list)


#: ``detector(image_rgb, plane, requested) -> [Detection]``
Detector = Callable[[np.ndarray, TablePlane, Sequence[Dict[str, Any]]],
                    List[Detection]]


def perceive_frame(image_rgb: np.ndarray, camera, *,
                   table_z: Optional[float] = None,
                   known_length_m: Optional[float] = None,
                   provisional_z: Optional[float] = None,
                   requested: Sequence[Dict[str, Any]] = (),
                   detector: Optional[Detector] = None,
                   table_name: str = "table",
                   table_depth_m: Optional[float] = None,
                   interiors: Optional[Dict[str, Sequence[float]]] = None,
                   sizes: Optional[Dict[str, Sequence[float]]] = None,
                   source_image: Optional[str] = None) -> FrameResult:
    """One head frame -> a scene, the whole pipeline.

    Fit the table plane (:func:`~.plane.fit_table_plane`), apply the height
    policy (:func:`~.plane.table_in_base`), hand the frame to ``detector``
    for ``requested`` things — or, with no detector, record that the things
    were LEFT TO THE MODEL — and serialise (:func:`build_scene`).

    Raises :class:`~.plane.PlaneFitError` rather than returning a plausible
    plane.
    """
    plane = fit_table_plane(image_rgb, fx=camera.fx)
    diagnostics: Dict[str, Any] = {
        "level_correction_deg": round(level_correction_deg(plane, camera), 2),
        "edge_fit": dict(plane.quality)}
    implied = neck_pitch_that_levels(plane,
                                     neck_yaw=getattr(camera, "neck_yaw", 0.0))
    if implied is not None:
        diagnostics["neck_pitch_that_levels_rad"] = round(implied, 4)
    table = table_in_base(plane, camera, table_z=table_z,
                          known_length_m=known_length_m,
                          provisional_z=provisional_z)
    detections: List[Detection] = []
    if requested and detector is not None:
        detections = list(detector(image_rgb, plane, list(requested)))
    elif requested:
        diagnostics["objects_left_to_the_model"] = [i["name"]
                                                    for i in requested]
    scene = build_scene(camera, table, detections, table_name=table_name,
                        table_depth_m=table_depth_m, interiors=interiors,
                        sizes=sizes, source_image=source_image,
                        diagnostics=diagnostics)
    floor = camera.floor_z() if hasattr(camera, "floor_z") else None
    if floor is not None:
        scene["_perceive"]["table_height_above_floor_m"] = round(
            float(table.z) - floor, 4)
    return FrameResult(scene=scene, plane=plane, table=table,
                       detections=detections)


__all__ = ["COLOURS", "ConfidenceLadder", "DEFAULT_LADDER", "Detection",
           "Detector", "FrameResult", "INTERIOR_FRACTION", "NAME_COLOUR_HINTS",
           "OBJECT_S_MARGIN", "OBJECT_V_DROP", "RIM_DROP_M", "build_scene",
           "debug_image", "detect_objects_mask", "from_silhouette", "measure",
           "perceive_frame", "requested_objects", "table_object",
           "table_top_colour"]
