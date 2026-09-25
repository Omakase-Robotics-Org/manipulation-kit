"""Where the jaws will close, and what the object looks like, in the wrist
photo: the two references the servo draws for its judge.

THE JAW OPENING (:func:`jaw_opening`). The judge is asked about the object
RELATIVE TO THE FINGERS, not relative to the kit's belief: a box drawn around
the declared object only says where the kit thinks the object is, and "on the
box" is then "the belief agrees with itself" whenever the declaration moved
with the hand. The closing region is the hand's own geometry — the two pads
at the current jaw gap — carried along the approach axis to the object's
depth and projected through the wrist camera (fisheye included). It is drawn
as what it is: two pad faces and the gap between them.

    flange frame (the camera's ``flange_p`` / ``flange_r``): +z approach,
    +x the jaw travel (``primitives.orientation.jaw_axis``), +y across it
    pads: x in +-[gap/2, gap/2 + PAD_DRAW_THICKNESS_M], y in +-PAD_WIDTH_M/2,
    at z = the object's depth along the approach axis

The gap is the MEASURED one when the executor reports it
(``GripperView.jaw_gap_m``, d1-firmwared's ``jaw_rad`` through the hand's
kinematic map), else the hand's driven-open gap, else the description's
nominal driven opening — and the look records which.

THE OBJECT'S OUTLINE (:func:`declared_outline`, :class:`Outline`). Phase 1
needs no model: the declared object projected with its SHAPE — a cylinder as
the silhouette of its top and footprint circles at the declared diameter, a
box as the silhouette of its eight corners at its declared yaw, anything else
as its declared footprint. Phase 2 is any :data:`ObjectOutline` (an HTTP
segmenter in the examples); both give the same :class:`Outline`, so the
drawing and the questions do not know which drew it.

Pure geometry and pillow drawing; nothing here knows a model.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import (Any, Callable, Dict, List, Mapping, Optional, Sequence,
                    Tuple)

import numpy as np

#: how thick the pads are drawn across the jaw travel [m]. A drawing
#: constant, not the jaw's collision box (40 mm, carriage included): the
#: judge is shown two pad FACES and the gap, not the carriage behind them
PAD_DRAW_THICKNESS_M = 0.010

#: points sampled along each straight edge before projection, so a fisheye
#: draws a curved edge as curved
EDGE_SAMPLES = 8

#: the shapes a declaration can carry (:func:`declared_outline`)
SHAPES: Tuple[str, ...] = ("box", "cylinder", "other")

#: an object whose longer horizontal extent is at least this many times the
#: shorter is ELONGATED: which way the jaws close across it matters
ELONGATED_RATIO = 1.3

Point = Tuple[float, float]


@dataclass(frozen=True)
class JawOpening:
    """The closing region in the image: two pad polygons, the gap polygon
    between them, the image point of the approach axis at the object's depth
    (``centre``) and that point in base (``point``)."""

    centre: Point
    pads: Tuple[Tuple[Point, ...], Tuple[Point, ...]]
    gap: Tuple[Point, ...]
    point: Tuple[float, float, float]
    depth_m: float                   # along the approach axis, from the flange
    gap_m: float
    gap_source: str
    #: the jaw travel's direction in the image at ``centre`` (degrees,
    #: image x right, y down; 0 = the jaws close left-right)
    jaw_angle_deg: float

    def bbox(self) -> Tuple[float, float, float, float]:
        pts = [p for poly in self.pads for p in poly] + list(self.gap)
        us = [p[0] for p in pts]
        vs = [p[1] for p in pts]
        return (min(us), min(vs), max(us), max(vs))

    def to_json(self) -> Dict[str, Any]:
        r = (lambda poly: [[round(u, 1), round(v, 1)] for u, v in poly])
        return {"centre": [round(self.centre[0], 1), round(self.centre[1], 1)],
                "pads": [r(p) for p in self.pads], "gap": r(self.gap),
                "point": [round(float(x), 5) for x in self.point],
                "depth_m": round(self.depth_m, 4),
                "gap_m": round(self.gap_m, 4), "gap_source": self.gap_source,
                "jaw_angle_deg": round(self.jaw_angle_deg, 1)}


@dataclass(frozen=True)
class Outline:
    """An object's outline in the image: a closed polygon, where it came
    from (``declared:<shape>`` or a segmenter's name), and the segmenter's
    score when it gave one."""

    points: Tuple[Point, ...]
    source: str
    score: Optional[float] = None
    #: the long axis of the outline in the image (degrees, as
    #: ``JawOpening.jaw_angle_deg``) and the ratio of its extents along and
    #: across it (1.0 = round)
    long_axis_deg: float = 0.0
    aspect: float = 1.0
    extra: Dict[str, Any] = field(default_factory=dict)

    @property
    def elongated(self) -> bool:
        return self.aspect >= ELONGATED_RATIO

    def bbox(self) -> Tuple[float, float, float, float]:
        us = [p[0] for p in self.points]
        vs = [p[1] for p in self.points]
        return (min(us), min(vs), max(us), max(vs))

    def centre(self) -> Point:
        pts = np.asarray(self.points, dtype=float)
        return (float(pts[:, 0].mean()), float(pts[:, 1].mean()))

    def to_json(self) -> Dict[str, Any]:
        out = {"source": self.source, "n": len(self.points),
               "points": [[round(u, 1), round(v, 1)] for u, v in self.points],
               "long_axis_deg": round(self.long_axis_deg, 1),
               "aspect": round(self.aspect, 3)}
        if self.score is not None:
            out["score"] = round(float(self.score), 4)
        out.update(self.extra)
        return out


def outline_from_points(points: Sequence[Point], source: str, *,
                        score: Optional[float] = None,
                        extra: Optional[Mapping[str, Any]] = None,
                        hull: bool = True) -> Optional[Outline]:
    """An :class:`Outline`: the convex hull of ``points`` (``hull=False``:
    the polygon as given, e.g. a segmenter's contour), its long axis and
    aspect from the hull's minimum-area rectangle; None for fewer than three
    distinct points."""
    convex = convex_hull(points)
    if len(convex) < 3:
        return None
    angle, aspect = principal_axis(convex)
    kept = convex if hull else tuple((float(u), float(v)) for u, v in points)
    return Outline(points=kept, source=source, score=score,
                   long_axis_deg=angle, aspect=aspect,
                   extra=dict(extra or {}))


def convex_hull(points: Sequence[Point]) -> Tuple[Point, ...]:
    """Andrew's monotone chain, counter-clockwise in image axes."""
    pts = sorted(set((float(u), float(v)) for u, v in points))
    if len(pts) < 3:
        return tuple(pts)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower: List[Point] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper: List[Point] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return tuple(lower[:-1] + upper[:-1])


def principal_axis(polygon: Sequence[Point]) -> Tuple[float, float]:
    """``(angle_deg, aspect)`` of a polygon: the direction of its longest
    extent (degrees in (-90, 90], image axes) and the ratio of its extents
    along and across that direction. Found by rotating calipers over the
    hull's edges (the minimum-area bounding rectangle)."""
    hull = convex_hull(polygon)
    if len(hull) < 3:
        return 0.0, 1.0
    pts = np.asarray(hull, dtype=float)
    best = None
    for i in range(len(pts)):
        edge = pts[(i + 1) % len(pts)] - pts[i]
        n = float(np.hypot(*edge))
        if n < 1e-9:
            continue
        a = edge / n
        b = np.array([-a[1], a[0]])
        la = pts @ a
        lb = pts @ b
        ext_a, ext_b = float(la.max() - la.min()), float(lb.max() - lb.min())
        area = ext_a * ext_b
        if best is None or area < best[0] - 1e-9:
            best = (area, a, b, ext_a, ext_b)
    _area, a, b, ext_a, ext_b = best
    long = a if ext_a >= ext_b else b
    angle = math.degrees(math.atan2(long[1], long[0]))
    if angle <= -90.0:
        angle += 180.0
    elif angle > 90.0:
        angle -= 180.0
    short = min(ext_a, ext_b)
    return angle, (max(ext_a, ext_b) / short if short > 1e-9 else float("inf"))


def wrap_turn(deg: float) -> float:
    """An angle between two AXES (not directions), wrapped to (-90, 90]."""
    x = (float(deg) + 90.0) % 180.0 - 90.0
    return 90.0 if x == -90.0 else x


def axial_mean(values_deg: Sequence[float], weights: Sequence[float]) -> float:
    """The weighted mean of AXIS angles (degrees; +90 and -90 are the same
    turn): the doubled-angle circular mean, wrapped to (-90, 90]. The plain
    expected value of a turn answer split between +90 and -90 is 0 — the one
    turn it certainly is not."""
    s = sum(float(w) * math.sin(math.radians(2.0 * float(v)))
            for v, w in zip(values_deg, weights))
    c = sum(float(w) * math.cos(math.radians(2.0 * float(v)))
            for v, w in zip(values_deg, weights))
    if abs(s) < 1e-12 and abs(c) < 1e-12:
        return 0.0
    return wrap_turn(0.5 * math.degrees(math.atan2(s, c)))


def _project_many(camera, points_base) -> Optional[List[Point]]:
    from ..perception import NotOnThePlane  # noqa: PLC0415
    out = []
    for p in points_base:
        try:
            u, v = camera.project(np.asarray(p, dtype=float))
        except NotOnThePlane:
            return None
        out.append((float(u), float(v)))
    return out


def _edge_loop(corners: Sequence[np.ndarray], n: int = EDGE_SAMPLES
               ) -> List[np.ndarray]:
    """``corners`` of a planar polygon with every edge sampled ``n`` times."""
    out = []
    for i in range(len(corners)):
        a, b = np.asarray(corners[i]), np.asarray(corners[(i + 1) % len(corners)])
        for t in np.linspace(0.0, 1.0, n, endpoint=False):
            out.append(a + t * (b - a))
    return out


def approach_depth(camera, p_base) -> Optional[float]:
    """How far along the approach axis (from the flange) ``p_base`` lies;
    None when the camera carries no flange pose."""
    if getattr(camera, "flange_p", None) is None:
        return None
    z = camera.flange_r.as_matrix()[:, 2]
    return float(np.dot(np.asarray(p_base, dtype=float) - camera.flange_p, z))


def jaw_opening(camera, *, gap_m: float, depth_m: float,
                gap_source: str = "given",
                shift_m: Tuple[float, float] = (0.0, 0.0),
                pad_width_m: Optional[float] = None,
                pad_thickness_m: float = PAD_DRAW_THICKNESS_M
                ) -> Optional[JawOpening]:
    """The closing region at ``depth_m`` along the approach axis, projected
    through ``camera`` (a wrist camera carrying its flange pose). ``shift_m``
    moves it along the jaw axis / across it (the servo's refinement grid).
    None when the camera has no flange pose or a corner is behind the lens."""
    if getattr(camera, "flange_p", None) is None:
        return None
    if pad_width_m is None:
        from ..hands.d1.parallel_gripper.description import (  # noqa: PLC0415
            PAD_WIDTH_M)
        pad_width_m = PAD_WIDTH_M
    m = camera.flange_r.as_matrix()
    x, y, z = m[:, 0], m[:, 1], m[:, 2]
    origin = (np.asarray(camera.flange_p, dtype=float) + float(depth_m) * z
              + float(shift_m[0]) * x + float(shift_m[1]) * y)
    g, w, t = float(gap_m) / 2.0, float(pad_width_m) / 2.0, float(pad_thickness_m)

    def rect(x0, x1):
        return [origin + x0 * x - w * y, origin + x1 * x - w * y,
                origin + x1 * x + w * y, origin + x0 * x + w * y]
    polys = []
    for x0, x1 in ((g, g + t), (-g - t, -g), (-g, g)):
        projected = _project_many(camera, _edge_loop(rect(x0, x1)))
        if projected is None:
            return None
        polys.append(tuple(projected))
    centre = _project_many(camera, [origin, origin + 0.01 * x])
    if centre is None:
        return None
    (cu, cv), (xu, xv) = centre
    return JawOpening(centre=(cu, cv), pads=(polys[0], polys[1]), gap=polys[2],
                      point=tuple(float(c) for c in origin), depth_m=float(depth_m),
                      gap_m=float(gap_m), gap_source=gap_source,
                      jaw_angle_deg=math.degrees(math.atan2(xv - cv, xu - cu)))


def jaw_gap(gripper) -> Tuple[float, str]:
    """The gap to draw, and where it came from: the measured pad gap, else
    the hand's driven-open gap, else the description's nominal opening."""
    if gripper is not None and getattr(gripper, "jaw_gap_m", None) is not None:
        return float(gripper.jaw_gap_m), "measured"
    if gripper is not None and getattr(gripper, "open_gap_m", None) is not None:
        return float(gripper.open_gap_m), "driven_open"
    from ..hands.d1.parallel_gripper.description import (  # noqa: PLC0415
        DRIVEN_OPEN_GAP_M)
    return DRIVEN_OPEN_GAP_M, "nominal"


def declared_outline(camera, item, frames, shape: str = "box", *,
                     p: Optional[np.ndarray] = None,
                     samples: int = 24) -> Optional[Outline]:
    """Phase 1: the declared object's outline with its declared ``shape``
    (:data:`SHAPES`), projected. ``cylinder``: the silhouette of the top and
    footprint circles, diameter = the mean of the two horizontal extents;
    ``box``: the silhouette of the eight corners at the declared yaw;
    ``other``: the declared footprint (the bottom face). ``p`` overrides the
    centre (a lab placing the declaration elsewhere)."""
    if shape not in SHAPES:
        raise ValueError(f"shape must be one of {SHAPES}, got {shape!r}")
    centre, r = item.pose_in_base(frames)
    centre = np.asarray(centre if p is None else p, dtype=float)
    half = np.asarray(item.size, dtype=float).reshape(3) / 2.0
    points: List[np.ndarray] = []
    if shape == "cylinder":
        radius = (half[0] + half[1]) / 2.0
        for sz in (-1.0, 1.0):
            for k in range(samples):
                a = 2.0 * math.pi * k / samples
                local = np.array([radius * math.cos(a), radius * math.sin(a),
                                  sz * half[2]])
                points.append(centre + r.apply(local))
    elif shape == "box":
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                for sz in (-1.0, 1.0):
                    points.append(centre + r.apply(half * [sx, sy, sz]))
        # sample the edges so a fisheye bends them
        corners = [np.array([sx, sy, sz]) for sx, sy in
                   ((-1, -1), (1, -1), (1, 1), (-1, 1)) for sz in (-1.0, 1.0)]
        for sz in (-1.0, 1.0):
            face = [centre + r.apply(half * c) for c in corners if c[2] == sz]
            points.extend(_edge_loop(face))
    else:
        face = [centre + r.apply(half * np.array([sx, sy, -1.0]))
                for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        points.extend(_edge_loop(face))
    projected = _project_many(camera, points)
    if projected is None:
        return None
    return outline_from_points(projected, f"declared:{shape}")


#: phase 1 or phase 2, one signature: the photo (may be None) and the look
#: -> the object's outline in that photo, or None ("not found": the servo
#: then draws the declared shape and records that it did)
ObjectOutline = Callable[[Optional[Path], Any], Optional[Outline]]


def declared(image: Optional[Path], look) -> Optional[Outline]:
    """The phase-1 :data:`ObjectOutline`: the declared shape the look was
    built with (``look.declared_outline``)."""
    return getattr(look, "declared_outline", None)


GREEN = (0, 230, 0)
MAGENTA = (255, 0, 220)


def draw(photo: Path, out: Path, *, jaws: Optional[JawOpening] = None,
         outline: Optional[Outline] = None, width: int = 3,
         arm_px: int = 10) -> Path:
    """The photo with the jaw opening (two green pad faces, the gap between
    them outlined, a small cross on the approach axis) and the object's
    outline (magenta)."""
    try:
        from PIL import Image, ImageDraw  # noqa: PLC0415
    except ImportError as exc:  # pragma: no cover - environment
        raise RuntimeError("drawing the wrist mark needs pillow (the "
                           "`perception` extra)") from exc
    with Image.open(photo) as source:
        image = source.convert("RGB")
    canvas = ImageDraw.Draw(image)
    if outline is not None and len(outline.points) >= 3:
        canvas.line(list(outline.points) + [outline.points[0]], fill=MAGENTA,
                    width=width, joint="curve")
    if jaws is not None:
        canvas.line(list(jaws.gap) + [jaws.gap[0]], fill=GREEN, width=1)
        for pad in jaws.pads:
            canvas.polygon(list(pad), outline=GREEN, fill=None, width=width + 1)
        u, v = jaws.centre
        canvas.line([(u - arm_px, v), (u + arm_px, v)], fill=GREEN, width=2)
        canvas.line([(u, v - arm_px), (u, v + arm_px)], fill=GREEN, width=2)
    out.parent.mkdir(parents=True, exist_ok=True)
    image.save(out)
    return out


__all__ = ["axial_mean", "EDGE_SAMPLES", "ELONGATED_RATIO", "JawOpening", "ObjectOutline",
           "Outline", "PAD_DRAW_THICKNESS_M", "SHAPES", "approach_depth",
           "convex_hull", "declared", "declared_outline", "draw", "jaw_gap",
           "jaw_opening", "outline_from_points", "principal_axis", "wrap_turn"]
