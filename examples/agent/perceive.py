"""One head frame -> a MEASURED scene, with no tape measure on the table.

``--scene`` is the first-hour path: you measure the table and the objects by
hand and the loop consumes the file. It works — d1-2 run2 (2026-09-22) made
two real grasps off a hand-made scene — and it does not scale, because every
one of those numbers is a person with a ruler and every one of them is stale
the moment the wagon is nudged.

This is the same file, MEASURED from one photograph. The only priors are

  * ONE known length on the table plane — the rectangular top's WIDTH
    (``--table-width``); any known edge would do, and
  * the camera's focal length in pixels (``--fx``).

No ArUco board, no marker on the objects, no table height, no depth camera.
Shu's phrasing on 2026-09-22 was "テーブルとかの高さとか知らずにできるべき", and
the table height is the interesting one: it comes out of the fit rather than
out of a constant, which is what makes this zero-shot rather than
pre-measured-in-a-different-place.

HOW THE PLANE IS RECOVERED, in one paragraph
--------------------------------------------
Mask the table top (it is the bright, low-saturation region; the JP wagon top
renders near-white/pink). Fit three straight lines to the largest blob: the
FAR edge, and the two SIDE edges. The side edges are parallel ON THE TABLE, so
where they meet in the IMAGE is their vanishing point ``v``, and
``D = normalise(K^-1 v)`` is their 3-D direction. Back-project the two far
corners as rays ``r1``, ``r2`` and solve the two ranges ``d1``, ``d2`` that
make ``E = d2 r2 - d1 r1`` perpendicular to ``D`` and exactly
``--table-width`` long — one known length, two unknowns, one constraint each.
That fixes the far edge in metres, and ``n = E x D`` is the plane's normal.
Everything else is a ray-plane intersection.

The table is then a rectangle in plane coordinates: ``across`` from the
image-left far corner along the far edge, ``depth`` toward the camera. The
NEAR edge is fitted the same way when it is inside the frame, so the table's
depth is measured too rather than assumed.

WHERE THAT PLANE SITS ON THE ROBOT -- ``--anchor``
--------------------------------------------------
The fit lives in CAMERA coordinates. Turning it into a ``base`` pose needs the
camera pose, and there are two honest ways to get one:

``--anchor far-edge-x=0.76[,centre-y=0.0]``
    You know where the table's far edge is in front of the robot (a single
    measurement that survives the objects being moved around). ``--table-z``
    is then REQUIRED, because nothing in this mode measures height.
    This is what Shu did by hand on 2026-09-22.

``--anchor camera``
    The zero-shot one. The camera pose comes from the kit's own model
    (``manipulation_kit.description.head_camera``, new in this change) given
    the neck joints and, for a floor-relative report, the lift. THE TABLE
    HEIGHT IS THEN MEASURED: the perpendicular distance from the camera
    centre to the fitted plane does not depend on the camera's orientation at
    all, so ``table_z = camera_z_in_base - distance`` is as good as the
    camera's HEIGHT, which is the best-known part of a nominal mount.

    The camera's ORIENTATION is the worst-known part (the frame is nominal
    geometry plus a design tilt, uncalibrated, and the neck's motor zero moves
    after a home), so this mode does not trust it blindly: with
    ``--assume-level`` (the default) the fitted plane is taken to be
    horizontal — it is a wagon on a floor — and the camera rotation is
    corrected by the MINIMAL rotation that makes it so. The correction is
    reported as ``level_correction_deg`` and is a direct measurement of how
    far the nominal aim is off. Everything this mode writes is stamped
    ``calibrated: false``.

WHAT IS NOT MEASURED, AND SAYS SO
---------------------------------
Every number in the output carries a ``confidence`` below 1 and a note when it
was inferred rather than seen: a container's interior (a rim thickness is not
visible from above-and-in-front), an object's extent along the unseen
horizontal axis, and yaw — an axis-aligned bounding box carries no
orientation, so yaw is 0 and the footprint width is written on BOTH horizontal
axes, which makes that 0 harmless rather than a quiet lie.

    # the exact shape of the thing, with no key and no robot:
    python examples/agent/perceive.py --image head.jpg --table-width 0.60 \
        --anchor far-edge-x=0.76 --table-z 0.166 \
        --objects charger:object,cup:container --detector mask \
        --out scenes/live.json --debug /tmp/perceived.png

    # zero-shot, on the robot, with the neck where the daemon says it is:
    python examples/agent/perceive.py --image turn0_base_0_rgb.jpg \
        --table-width 0.60 --anchor camera --neck-pitch 0.52 --lift 0.205 \
        --objects charger:object,cup:container --detector astra

``--detector mask`` is the colour fallback (white / brown, the two things on
the d1-2 wagon) and needs nothing but numpy, scipy and an image decoder.
``--detector astra`` is one Responses-API call with the frame attached; the
``openai`` package is imported lazily and is not a dependency of anything
here.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy import ndimage
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, str(Path(__file__).resolve().parent))

#: The d1 head camera at 640x480. A focal length is a property of the stream,
#: not of the robot, so it is a default and not a constant.
DEFAULT_FX = 606.0

#: Table-top mask thresholds, OpenCV HSV ranges (S and V in 0..255). The JP
#: wagon top is a bright, almost unsaturated pink; the brick wall behind it and
#: the black trolley below it are neither.
TOP_S_MAX = 45
TOP_V_MIN = 140

#: Colour priors the ``mask`` detector knows, as HSV windows in the same
#: ranges: (h_lo, h_hi, s_lo, s_hi, v_lo, v_hi). These are a FALLBACK — two
#: objects on one wagon — and the reason ``--detector astra`` exists. They are
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

#: A container's interior, as a fraction of its measured outside. NOT
#: measured: the wall thickness is not visible from a single view above and in
#: front, so it is a documented assumption and every object built with it says
#: ``interior_measured: false``.
INTERIOR_FRACTION = 0.85
#: How far below the rim the usable interior starts, metres.
RIM_DROP_M = 0.01


# --------------------------------------------------------------------------- #
# failures with a reason, not a stack trace
# --------------------------------------------------------------------------- #

class PlaneFitError(RuntimeError):
    """The table plane could not be recovered from this frame.

    ``reason`` is one of a small closed vocabulary so a caller can branch
    without parsing English:

    ``no_table``        nothing bright and unsaturated enough to be a top
    ``far_edge``        the far edge did not fit a line
    ``side_edge``       one or both side edges did not fit a line
    ``occlusion``       the top is there but something is standing on the
                        edges the fit needs — the robot's own arm is the case
                        this was written for
    ``degenerate``      the geometry solved to something impossible (the side
                        edges parallel in the image, a corner behind the
                        camera, a plane through the camera centre)
    """

    def __init__(self, reason: str, detail: str = ""):
        self.reason = reason
        self.detail = detail
        super().__init__(f"{reason}: {detail}" if detail else reason)


# --------------------------------------------------------------------------- #
# image in, HSV out — no OpenCV required
# --------------------------------------------------------------------------- #

def load_image(path) -> np.ndarray:
    """An ``(H, W, 3)`` uint8 RGB array from a file.

    Pillow first because it is small and pure wheels exist for every Python
    this repository supports; OpenCV second because a robot that has
    ``d1-inference`` installed already has it. One of the two, and the error
    says so rather than dying inside an import.
    """
    try:
        from PIL import Image  # noqa: PLC0415
    except ImportError:
        pass
    else:
        with Image.open(path) as handle:
            return np.asarray(handle.convert("RGB"), dtype=np.uint8)
    try:
        import cv2  # noqa: PLC0415
    except ImportError as exc:      # pragma: no cover - environment specific
        raise RuntimeError(
            "reading an image needs Pillow or OpenCV: pip install pillow "
            "(or opencv-python-headless)") from exc
    bgr = cv2.imread(str(path))     # pragma: no cover - environment specific
    if bgr is None:
        raise FileNotFoundError(path)
    return np.ascontiguousarray(bgr[:, :, ::-1])


def to_hsv(rgb: np.ndarray) -> np.ndarray:
    """OpenCV's 8-bit HSV: H in 0..179, S and V in 0..255.

    Written out rather than imported so the thresholds above keep meaning what
    they meant in the script they came from, on a machine with no OpenCV.
    """
    arr = np.asarray(rgb, dtype=np.float32)
    r, g, b = arr[..., 0], arr[..., 1], arr[..., 2]
    v = arr.max(axis=-1)
    low = arr.min(axis=-1)
    span = v - low
    s = np.where(v > 0, span / np.maximum(v, 1e-6) * 255.0, 0.0)
    h = np.zeros_like(v)
    nz = span > 0
    with np.errstate(invalid="ignore", divide="ignore"):
        rmax = nz & (v == r)
        gmax = nz & (v == g) & ~rmax
        bmax = nz & ~rmax & ~gmax
        h[rmax] = 60.0 * (g[rmax] - b[rmax]) / span[rmax]
        h[gmax] = 120.0 + 60.0 * (b[gmax] - r[gmax]) / span[gmax]
        h[bmax] = 240.0 + 60.0 * (r[bmax] - g[bmax]) / span[bmax]
    h = np.mod(h, 360.0) / 2.0
    return np.stack([h, s, v], axis=-1)


def _erode(mask: np.ndarray, k: int) -> np.ndarray:
    """Erosion that does NOT eat the image border.

    ``scipy.ndimage`` pads with zeros by default, which shaves ``k//2`` pixels
    off every edge of the mask — and the near edge of this table runs along
    the bottom of the frame, so that padding alone moved the measured depth by
    a centimetre. ``border_value=1`` is OpenCV's behaviour and the right one.
    """
    return ndimage.binary_erosion(mask, np.ones((k, k), bool), border_value=1)


def _dilate(mask: np.ndarray, k: int) -> np.ndarray:
    return ndimage.binary_dilation(mask, np.ones((k, k), bool), border_value=0)


def table_top_mask(hsv: np.ndarray, *, s_max: int = TOP_S_MAX,
                   v_min: int = TOP_V_MIN, open_k: int = 7,
                   close_k: int = 15) -> np.ndarray:
    """The largest bright, unsaturated blob: the table top.

    Only the LARGEST connected component survives, which the script this came
    from did not do — it fitted its edges to every bright pixel in the frame,
    including the printer and the wall sockets behind the wagon. It got away
    with it because those sit above the far edge; a white object ON the table
    would have broken it.
    """
    raw = (hsv[..., 1] < s_max) & (hsv[..., 2] > v_min)
    clean = _dilate(_erode(raw, open_k), open_k)         # open
    clean = _erode(_dilate(clean, close_k), close_k)     # close
    labels, count = ndimage.label(clean, np.ones((3, 3), bool))
    if count == 0:
        raise PlaneFitError(
            "no_table", f"no region with S < {s_max} and V > {v_min}; is this "
            f"a table top, and is the exposure the one the thresholds were "
            f"chosen for?")
    sizes = ndimage.sum_labels(clean, labels, index=np.arange(1, count + 1))
    return labels == (int(np.argmax(sizes)) + 1)


# --------------------------------------------------------------------------- #
# robust straight lines
# --------------------------------------------------------------------------- #

def fit_line(points: np.ndarray, *, weights: Optional[np.ndarray] = None,
             huber_c: float = 1.345,
             iterations: int = 20) -> Tuple[np.ndarray, np.ndarray]:
    """``(direction, point)`` of the line through ``points``, Huber-weighted.

    A total-least-squares fit, then iteratively reweighted with ``w = 1``
    inside ``huber_c`` pixels and ``w = c / r`` outside it. Total least
    squares matters — these are near-horizontal AND near-vertical edges in the
    same function, and an ordinary y-on-x fit cannot do the second.

    Huber alone is NOT enough here, which is why :func:`fit_edge` exists; see
    its docstring for the frame that proved it.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) < 2:
        raise PlaneFitError("degenerate", f"only {len(pts)} points to fit")
    w = np.ones(len(pts)) if weights is None else np.asarray(weights, float)
    direction = centre = None
    for _ in range(iterations):
        total = w.sum()
        centre = (w[:, None] * pts).sum(axis=0) / total
        delta = pts - centre
        cov = (w[:, None] * delta).T @ delta / total
        angle = 0.5 * math.atan2(2.0 * cov[0, 1], cov[0, 0] - cov[1, 1])
        new = np.array([math.cos(angle), math.sin(angle)])
        if direction is not None and abs(abs(new @ direction) - 1.0) < 1e-9:
            direction = new
            break
        direction = new
        residual = np.abs(delta[:, 0] * direction[1] - delta[:, 1] * direction[0])
        w = np.where(residual < huber_c, 1.0,
                     huber_c / np.maximum(residual, 1e-9))
    return direction, centre


def fit_edge(points: np.ndarray, *, inlier_px: float = 2.0,
             max_hypotheses: int = 60
             ) -> Tuple[Tuple[np.ndarray, np.ndarray], np.ndarray]:
    """``((direction, point), inliers)`` — consensus first, then Huber.

    THE FRAME THAT FORCED THIS. On the d1-2 run2 start frame the wagon's right
    edge leaves the picture two-thirds of the way down, so every row below
    that contributes the NEAR edge's rightmost pixel instead — a fifth of the
    scan, 200+ px away, in a tight cluster of its own. A Huber fit seeded by
    ordinary least squares starts halfway between the two clusters, and
    reweighting never climbs out: it returned a line with a 40 px median
    residual and a far corner 300 px from where OpenCV's ``fitLine`` put it.
    ``cv2.fitLine`` survives it only because it is quietly a RANSAC — twenty
    restarts from random ten-point subsets — which is not something to depend
    on without saying so.

    So: take the line two points agree on that the most other points also
    agree with (deterministically, over an evenly spaced subsample, so the
    same frame always gives the same plane), then Huber-refit on that
    consensus. The inlier mask is returned because "how much of this edge was
    actually visible" is the occlusion evidence the caller needs.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) < 2:
        raise PlaneFitError("degenerate", f"only {len(pts)} points to fit")
    if len(pts) == 2:
        return fit_line(pts), np.ones(2, dtype=bool)
    step = max(1, len(pts) // max_hypotheses)
    seeds = np.arange(0, len(pts), step)
    best_count, best_cost, best = -1, np.inf, None
    for a in range(len(seeds)):
        for b in range(a + 1, len(seeds)):
            delta = pts[seeds[b]] - pts[seeds[a]]
            norm = float(np.hypot(*delta))
            if norm < 1e-9:
                continue
            direction = delta / norm
            offset = pts - pts[seeds[a]]
            residual = np.abs(offset[:, 0] * direction[1]
                              - offset[:, 1] * direction[0])
            inliers = residual <= inlier_px
            count = int(inliers.sum())
            cost = float(np.minimum(residual, inlier_px).sum())
            if count > best_count or (count == best_count and cost < best_cost):
                best_count, best_cost, best = count, cost, inliers
    if best is None or best_count < 2:
        raise PlaneFitError("degenerate", "no two points agree on a line")
    return fit_line(pts[best]), best


def line_residuals(points: np.ndarray, line: Tuple[np.ndarray, np.ndarray]
                   ) -> np.ndarray:
    direction, centre = line
    delta = np.asarray(points, dtype=float).reshape(-1, 2) - centre
    return np.abs(delta[:, 0] * direction[1] - delta[:, 1] * direction[0])


def _homogeneous(line: Tuple[np.ndarray, np.ndarray]) -> np.ndarray:
    direction, centre = line
    return np.cross(np.append(centre, 1.0), np.append(centre + direction, 1.0))


def _meet(a: np.ndarray, b: np.ndarray, what: str) -> np.ndarray:
    point = np.cross(a, b)
    if abs(point[2]) < 1e-9:
        raise PlaneFitError("degenerate", f"{what} do not meet in the image")
    return point / point[2]


# --------------------------------------------------------------------------- #
# the plane
# --------------------------------------------------------------------------- #

@dataclass
class TablePlane:
    """A rectangular table top, in CAMERA coordinates, from one frame.

    ``P1`` is the far corner on the image LEFT, ``E`` the far edge from it to
    the far corner on the image right (so ``|E|`` is the known width), ``D``
    the unit direction of the side edges pointing AWAY from the camera, and
    ``n = E x D`` the plane normal pointing back at the camera.
    """

    fx: float
    width_m: float
    shape: Tuple[int, int]                 # (H, W)
    P1: np.ndarray
    P2: np.ndarray
    E: np.ndarray
    D: np.ndarray
    n: np.ndarray
    far_px: Tuple[np.ndarray, np.ndarray]
    near_px: Optional[Tuple[np.ndarray, np.ndarray]] = None
    depth_m: Optional[float] = None
    mask: Optional[np.ndarray] = field(default=None, repr=False)
    quality: Dict[str, float] = field(default_factory=dict)

    # -- rays and points ---------------------------------------------------
    @property
    def K(self) -> np.ndarray:
        height, width = self.shape
        return np.array([[self.fx, 0.0, width / 2.0],
                         [0.0, self.fx, height / 2.0],
                         [0.0, 0.0, 1.0]])

    def ray(self, u: float, v: float) -> np.ndarray:
        return np.linalg.inv(self.K) @ np.array([float(u), float(v), 1.0])

    def point(self, u: float, v: float, height_m: float = 0.0) -> np.ndarray:
        """Where the pixel ray meets the table, in camera coordinates.

        ``height_m`` lifts the plane: the widest slice of a cup is 100 mm off
        the table and a ray through its rim meets the TABLE half a cup
        further away, so measuring a rim against the table top over-reports
        it by a fifth. Same plane, same normal, different offset.
        """
        r = self.ray(u, v)
        denominator = r @ self.n
        if abs(denominator) < 1e-9:
            raise PlaneFitError("degenerate",
                                f"pixel ({u:.0f}, {v:.0f}) runs parallel to "
                                f"the table plane")
        origin = self.P1 + self.up * float(height_m)
        return ((origin @ self.n) / denominator) * r

    def height_above(self, base_point: np.ndarray, u: float,
                     v: float) -> float:
        """How high above the table the pixel ``(u, v)`` is, ASSUMING it sits
        on the vertical line through ``base_point``.

        That assumption is what a single view has: the top of an upright cup
        is above its base, so the ray through the rim and the vertical through
        the footprint meet at the rim's height. It is exact for a symmetric
        upright object and wrong in proportion to how far the point really is
        off that axis — which is why the height it produces is reported with a
        confidence below 1 and not as a measurement of a leaning object.
        """
        ray = self.ray(u, v)
        matrix = np.column_stack([self.up, -ray])
        solution, *_ = np.linalg.lstsq(matrix, -np.asarray(base_point, float),
                                       rcond=None)
        return float(solution[0])

    def to_plane(self, u: float, v: float) -> Tuple[float, float]:
        """``(across, depth)`` in metres: across from the image-LEFT far
        corner along the far edge, depth toward the camera from the far edge."""
        offset = self.point(u, v) - self.P1
        return float(offset @ (self.E / self.width_m)), float(offset @ -self.D)

    @property
    def camera_distance_m(self) -> float:
        """Perpendicular distance from the camera centre to the table top.

        Orientation-free, which is the whole reason ``--anchor camera`` can
        MEASURE a table height off an uncalibrated mount: rotating the camera
        does not move this number, and the camera's height above the base is
        the part of a nominal frame that is actually reliable.
        """
        return float(abs(self.P1 @ self.n))

    @property
    def up(self) -> np.ndarray:
        """Plane normal pointing from the table toward the camera."""
        return self.n if (self.P1 @ self.n) < 0 else -self.n

    def corners_plane(self) -> Dict[str, Tuple[float, float]]:
        depth = self.depth_m if self.depth_m is not None else 0.0
        return {"far_left": (0.0, 0.0), "far_right": (self.width_m, 0.0),
                "near_left": (0.0, depth), "near_right": (self.width_m, depth)}


def _edge_points(mask: np.ndarray, *, band: Tuple[float, float] = (0.25, 0.75),
                 step: int = 4, top: bool = True) -> np.ndarray:
    """Top- (or bottom-) most mask pixel per column, over a central band."""
    height, width = mask.shape
    lo, hi = int(width * band[0]), int(width * band[1])
    out = []
    for x in range(lo, hi, step):
        rows = np.flatnonzero(mask[:, x])
        if rows.size:
            y = rows[0] if top else rows[-1]
            if top or y < height - 2:      # a clipped near edge is not an edge
                out.append((x, y))
    return np.array(out, dtype=float)


def _side_points(mask: np.ndarray, y0: int, *, step: int = 3,
                 min_run: int = 50) -> Tuple[np.ndarray, np.ndarray]:
    """Left- and right-most mask pixel per row, below ``y0``.

    Rows whose extreme pixel sits ON the image border are dropped: the table
    runs out of frame there and its edge is the frame's, not the table's.
    """
    height, width = mask.shape
    left, right = [], []
    for y in range(y0, height - 5, step):
        cols = np.flatnonzero(mask[y, :])
        if cols.size < min_run:
            continue
        if 3 < cols[0] < width - 4:
            left.append((cols[0], y))
        if 3 < cols[-1] < width - 4:
            right.append((cols[-1], y))
    return np.array(left, dtype=float), np.array(right, dtype=float)


def solve_plane(c1, c2, vanish, *, fx: float, shape: Tuple[int, int],
                table_width_m: float) -> TablePlane:
    """The whole metric step, from three pixel observations and one length.

    ``c1`` / ``c2`` are the far corners (image left, image right) and
    ``vanish`` is where the two side edges meet in the image. Separated from
    the mask work above so it can be tested against a camera whose answer is
    known exactly, rather than only against photographs of a wagon.
    """
    height, width = shape
    inverse_k = np.linalg.inv(np.array([[fx, 0.0, width / 2.0],
                                        [0.0, fx, height / 2.0],
                                        [0.0, 0.0, 1.0]]))
    c1 = np.asarray(c1, dtype=float).ravel()
    c2 = np.asarray(c2, dtype=float).ravel()
    vanish = np.asarray(vanish, dtype=float).ravel()
    c1 = c1 if c1.size == 3 else np.append(c1, 1.0)
    c2 = c2 if c2.size == 3 else np.append(c2, 1.0)
    vanish = vanish if vanish.size == 3 else np.append(vanish, 1.0)
    direction = inverse_k @ vanish
    direction = direction / np.linalg.norm(direction)
    if direction[2] < 0:            # the side edges run AWAY from the camera
        direction = -direction
    r1, r2 = inverse_k @ c1, inverse_k @ c2
    if abs(r2 @ direction) < 1e-9:
        raise PlaneFitError("degenerate", "a far corner ray is perpendicular "
                                          "to the side-edge direction")
    ratio = (r1 @ direction) / (r2 @ direction)
    span = r1 - ratio * r2
    norm = float(np.linalg.norm(span))
    if norm < 1e-9:
        raise PlaneFitError("degenerate", "the two far corners solved to the "
                                          "same ray")
    d1 = table_width_m / norm
    d2 = ratio * d1
    if d1 <= 0 or d2 <= 0:
        raise PlaneFitError("degenerate", f"a far corner solved BEHIND the "
                                          f"camera (ranges {d1:.3f}, {d2:.3f})")
    p1, p2 = d1 * r1, d2 * r2
    edge = p2 - p1
    normal = np.cross(edge, direction)
    normal = normal / np.linalg.norm(normal)
    return TablePlane(fx=float(fx), width_m=float(table_width_m),
                      shape=(height, width), P1=p1, P2=p2, E=edge,
                      D=direction, n=normal, far_px=(c1[:2], c2[:2]),
                      quality={"far_range_m": round(float(d1), 4),
                               "far_range_right_m": round(float(d2), 4)})


def fit_table_plane(image_rgb: np.ndarray, *, fx: float = DEFAULT_FX,
                    table_width_m: float = 0.60, s_max: int = TOP_S_MAX,
                    v_min: int = TOP_V_MIN,
                    max_edge_residual_px: float = 3.0,
                    min_visible_fraction: float = 0.5,
                    min_side_rows: int = 30) -> TablePlane:
    """Recover the table plane from one frame. See the module docstring.

    Raises :class:`PlaneFitError` rather than returning a plausible plane — a
    homography that still evaluates after the scene changed is the exact
    failure ``manipulation_kit.world.frames`` exists to refuse.
    """
    image = np.asarray(image_rgb)
    if image.ndim != 3 or image.shape[2] != 3:
        raise PlaneFitError("no_table", f"expected an (H, W, 3) image, got "
                                        f"{image.shape}")
    height, width = image.shape[:2]
    mask = table_top_mask(to_hsv(image), s_max=s_max, v_min=v_min)

    def edge(points: np.ndarray, name: str, minimum: int):
        """Fit one edge, and REFUSE rather than average over an occluder."""
        if len(points) < minimum:
            raise PlaneFitError(
                "occlusion" if name != "far" else "far_edge",
                f"only {len(points)} scan lines reach the {name} edge (need "
                f"{minimum}); something is standing on it — the robot's own "
                f"arm across the table does exactly this — so the plane is "
                f"not determined by this frame")
        line, inliers = fit_edge(points)
        seen = float(inliers.mean())
        if seen < min_visible_fraction:
            raise PlaneFitError(
                "occlusion",
                f"only {seen*100:.0f}% of the {name} edge's scan lines agree "
                f"on a straight line ({int(inliers.sum())} of {len(points)}); "
                f"the mask is following something that is not the table's "
                f"outline")
        residual = float(np.median(line_residuals(points[inliers], line)))
        if residual > max_edge_residual_px:
            raise PlaneFitError(
                "occlusion",
                f"the {name} edge does not fit a straight line (median "
                f"residual {residual:.1f} px > {max_edge_residual_px:.1f})")
        return line, residual, seen

    far_points = _edge_points(mask, top=True)
    far_line, far_residual, far_seen = edge(far_points, "far", 10)

    y0 = int(far_points[:, 1].mean()) + 25
    left_points, right_points = _side_points(mask, y0)
    left_line, left_residual, left_seen = edge(left_points, "left side",
                                               min_side_rows)
    right_line, right_residual, right_seen = edge(right_points, "right side",
                                                  min_side_rows)
    side_residual = max(left_residual, right_residual)

    far_h = _homogeneous(far_line)
    left_h, right_h = _homogeneous(left_line), _homogeneous(right_line)
    vanish = _meet(left_h, right_h, "the two side edges")
    c1 = _meet(far_h, left_h, "the far edge and the left side edge")
    c2 = _meet(far_h, right_h, "the far edge and the right side edge")

    plane = solve_plane(c1, c2, vanish, fx=fx, shape=(height, width),
                        table_width_m=table_width_m)
    plane.mask = mask
    plane.quality.update({"far_edge_residual_px": round(far_residual, 2),
                          "side_edge_residual_px": round(side_residual, 2),
                          "far_edge_visible": round(far_seen, 2),
                          "left_edge_visible": round(left_seen, 2),
                          "right_edge_visible": round(right_seen, 2)})

    near_points = _edge_points(mask, top=False)
    if len(near_points) >= 10:
        near_line, _ = fit_edge(near_points)
        near_h = _homogeneous(near_line)
        try:
            n1 = _meet(near_h, left_h, "the near edge and the left side edge")
            n2 = _meet(near_h, right_h, "the near edge and the right side edge")
        except PlaneFitError:
            pass
        else:
            depths = [plane.to_plane(*n1[:2])[1], plane.to_plane(*n2[:2])[1]]
            if min(depths) > 0.02:
                plane.near_px = (n1[:2], n2[:2])
                plane.depth_m = float(np.mean(depths))
    return plane


# --------------------------------------------------------------------------- #
# where the plane sits on the robot
# --------------------------------------------------------------------------- #

@dataclass
class Anchor:
    """A camera -> ``base`` transform, and how much to believe it."""

    p: np.ndarray                  # camera origin in base
    r: R                           # camera (optical) rotation in base
    mode: str
    calibrated: bool
    notes: List[str] = field(default_factory=list)
    level_correction_deg: Optional[float] = None
    implied_neck_pitch: Optional[float] = None
    table_z: Optional[float] = None

    def to_base(self, p_cam) -> np.ndarray:
        return self.p + self.r.apply(np.asarray(p_cam, dtype=float))

    def to_json(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "mode": self.mode, "calibrated": self.calibrated,
            "camera_p_base": [round(float(v), 4) for v in self.p],
            "camera_quat_xyzw": [round(float(v), 5) for v in self.r.as_quat()],
            "notes": list(self.notes)}
        if self.level_correction_deg is not None:
            out["level_correction_deg"] = round(self.level_correction_deg, 2)
        if self.implied_neck_pitch is not None:
            out["neck_pitch_that_levels_rad"] = round(self.implied_neck_pitch, 4)
        return out


def anchor_far_edge(plane: TablePlane, *, far_edge_x: float,
                    centre_y: float = 0.0, table_z: float) -> Anchor:
    """Pin the plane by the table's far-edge x in ``base``.

    The plane's own axes are ``across`` (image-left far corner -> image-right
    along the far edge), ``depth`` (toward the camera) and ``up``. In ``base``
    those are ``-y``, ``-x`` and ``+z``: the robot looks along ``+x``, its
    ``+y`` is its own left, and image-left IS robot-left.
    """
    across = plane.E / plane.width_m
    toward = -plane.D
    up = plane.up
    cam_axes = np.column_stack([across, toward, up])
    base_axes = np.column_stack([(0.0, -1.0, 0.0), (-1.0, 0.0, 0.0),
                                 (0.0, 0.0, 1.0)])
    rotation = base_axes @ cam_axes.T
    if abs(np.linalg.det(rotation) - 1.0) > 1e-6:
        raise PlaneFitError("degenerate", f"the plane axes are not a rotation "
                                          f"(det {np.linalg.det(rotation):.4f})")
    origin = np.array([float(far_edge_x),
                       float(centre_y) + plane.width_m / 2.0, float(table_z)])
    # base = origin + rotation @ (P - P1)  =>  the camera sits where P = 0 maps
    p = origin + rotation @ (-plane.P1)
    return Anchor(p=p, r=R.from_matrix(rotation), mode="far-edge-x",
                  calibrated=False, table_z=float(table_z),
                  notes=["the table height came from --table-z, not from this "
                         "frame; --anchor camera measures it instead"])


def anchor_camera(plane: TablePlane, *, neck_pitch: float = 0.0,
                  neck_yaw: float = 0.0, assume_level: bool = True,
                  urdf_path: Optional[str] = None) -> Anchor:
    """The camera pose from the kit's own head-camera frame.

    See the module docstring for why the height that comes out of this is a
    measurement and the aim that goes into it is not.
    """
    from manipulation_kit.description.head_camera import (  # noqa: PLC0415
        head_camera_pose, nominal_tilt_rad)

    p, rotation = head_camera_pose(neck_pitch=neck_pitch, neck_yaw=neck_yaw,
                                   urdf_path=urdf_path)
    notes = [
        "the head camera frame is NOMINAL geometry (design tilt "
        f"{math.degrees(nominal_tilt_rad()):.0f} deg, lens placed at the "
        "housing's front face), not a calibrated extrinsic",
        "lift does not enter: the lift joint is below `base`, so it raises "
        "the base and the camera together"]
    correction = None
    if assume_level:
        normal = rotation.apply(plane.up)
        axis = np.cross(normal, (0.0, 0.0, 1.0))
        sine = float(np.linalg.norm(axis))
        cosine = float(normal @ np.array([0.0, 0.0, 1.0]))
        angle = math.atan2(sine, cosine)
        if sine > 1e-9:
            rotation = R.from_rotvec(axis / sine * angle) * rotation
        correction = math.degrees(angle)
        notes.append(
            f"--assume-level: the fitted top was taken to be horizontal and "
            f"the nominal camera aim was rotated {correction:.1f} deg to make "
            f"it so; that angle IS the nominal mount's error plus the neck's "
            f"zero offset for this frame")
    table_z = float(p[2]) - plane.camera_distance_m
    implied = neck_pitch_that_levels(plane, neck_yaw=neck_yaw,
                                     urdf_path=urdf_path)
    if implied is not None:
        notes.append(
            f"the fitted top is level at neck_tilt = {implied:+.3f} rad "
            f"({math.degrees(implied):+.1f} deg, motor sign, positive looks "
            f"down). If you do not know what the neck was doing when this "
            f"frame was taken, that is the table's own answer — and if you DO "
            f"know it and it disagrees, the difference is the uncalibrated "
            f"mount")
    return Anchor(p=np.asarray(p, dtype=float), r=rotation, mode="camera",
                  calibrated=False, notes=notes,
                  level_correction_deg=correction, table_z=table_z,
                  implied_neck_pitch=implied)


def neck_pitch_that_levels(plane: TablePlane, *, neck_yaw: float = 0.0,
                           urdf_path: Optional[str] = None,
                           bounds: Tuple[float, float] = (-0.35, 0.65)
                           ) -> Optional[float]:
    """The ``neck_tilt`` at which the NOMINAL frame already sees this top as
    horizontal — the table telling you where the head was pointing.

    It is a diagnostic, not a calibration: it folds the mount's real error and
    the neck's zero offset into one number, and it cannot separate them. What
    it is good for is a frame whose neck angle nobody wrote down, and for
    noticing that the two disagree. ``None`` when no angle inside the joint's
    own limits does it.
    """
    from manipulation_kit.description.head_camera import (  # noqa: PLC0415
        head_camera_pose)

    def tilt_error(pitch: float) -> float:
        _, rot = head_camera_pose(neck_pitch=pitch, neck_yaw=neck_yaw,
                                  urdf_path=urdf_path)
        normal = rot.apply(plane.up)
        return math.atan2(float(np.linalg.norm(np.cross(normal, (0, 0, 1.0)))),
                          float(normal[2]))

    lo, hi = bounds
    # The error is monotone in the tilt over the joint's range (one joint, one
    # axis), so a bisection on its DERIVATIVE-free sign is not available —
    # minimise instead, with a golden section, and refuse if the minimum is
    # not actually zero.
    phi = (math.sqrt(5.0) - 1.0) / 2.0
    a, b = lo, hi
    for _ in range(60):
        c, d = b - phi * (b - a), a + phi * (b - a)
        if tilt_error(c) < tilt_error(d):
            b = d
        else:
            a = c
    best = (a + b) / 2.0
    if tilt_error(best) > math.radians(1.0) or not lo < best < hi:
        return None
    return float(best)


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


def _requested(spec: str) -> List[Dict[str, str]]:
    """``"cube:object,cup:container:brown"`` -> the list the detectors take."""
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
            raise SystemExit(f"--objects: {kind!r} is not object|container")
        if colour is None:
            colour = next((c for frag, c in NAME_COLOUR_HINTS
                           if frag in name.lower()), None)
        out.append({"name": name, "kind": kind, "colour": colour})
    if not out:
        raise SystemExit("--objects named nothing")
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
    no network. ``--detector astra`` is the one that generalises.

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
            raise SystemExit(
                f"--detector mask needs a colour for {item['name']!r}: write "
                f"{item['name']}:{item['kind']}:white|brown, or use "
                f"--detector astra")
        if colour not in COLOURS:
            raise SystemExit(f"--detector mask knows {sorted(COLOURS)}, not "
                             f"{colour!r}")
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
            "a colour blob, not a recognised object; --detector astra is the "
            "one that knows what it is looking at")
        out.append(detection)
    return out


ASTRA_PROMPT = """You are given ONE photograph from a robot's head camera,
looking down and forward at a table.

For each object named below, report where it is IN PIXELS. Reply with STRICT
JSON and nothing else - no prose, no markdown fence:

{"objects": [{"name": "<the requested name, verbatim>",
              "bbox": [x0, y0, x1, y1],
              "base_px": [u, v],
              "upright": true,
              "shape": "box" | "cylinder" | "other",
              "kind": "object" | "container",
              "confidence": 0.0-1.0}]}

  bbox     the object's tight pixel bounding box, x right, y down, origin at
           the top-left of the image.
  base_px  the pixel where the object's FOOTPRINT touches the table: the
           bottom-centre of its contact patch, NOT the bottom of the bounding
           box if the object overhangs, and NOT the centre of the object.
           This single pixel decides where the robot reaches, so look at it.
  upright  false if the object has tipped over or is lying on its side.
  kind     "container" if the robot could put something INSIDE it.

Omit an object you cannot see; do not invent one. The image is %(width)d x
%(height)d pixels.

The objects: %(names)s"""


def _parse_astra(text: str, requested: Sequence[Dict[str, str]]
                 ) -> List[Detection]:
    """Strict JSON, leniently extracted, strictly validated.

    Models fence their JSON, prefix it with "Here is", and occasionally return
    the list without the wrapper object. All three are recoverable and none of
    them is a reason to lose a robot run; a bbox that is not four numbers is
    NOT recoverable and raises.
    """
    payload = text.strip()
    fence = re.search(r"```(?:json)?\s*(.*?)```", payload, re.S)
    if fence:
        payload = fence.group(1).strip()
    start = min((i for i in (payload.find("{"), payload.find("[")) if i >= 0),
                default=-1)
    if start < 0:
        raise ValueError(f"no JSON in the reply: {text[:200]!r}")
    decoded, _ = json.JSONDecoder().raw_decode(payload[start:])
    items = decoded["objects"] if isinstance(decoded, dict) else decoded
    if not isinstance(items, list):
        raise ValueError(f"expected a list of objects, got {type(items)}")
    wanted = {item["name"]: item for item in requested}
    out: List[Detection] = []
    for raw in items:
        name = str(raw.get("name", ""))
        if name not in wanted:
            continue
        bbox = [float(v) for v in raw["bbox"]]
        if len(bbox) != 4:
            raise ValueError(f"{name}: bbox must be 4 numbers, got {raw['bbox']}")
        x0, y0, x1, y1 = bbox
        base = raw.get("base_px") or [(x0 + x1) / 2.0, y1]
        notes = []
        if not raw.get("base_px"):
            notes.append("no base_px in the reply; the bbox's bottom-centre "
                         "was used and an overhang would bias it")
        out.append(Detection(
            name=name, kind=str(raw.get("kind") or wanted[name]["kind"]),
            bbox=(min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)),
            base_px=(float(base[0]), float(base[1])),
            upright=bool(raw.get("upright", True)),
            shape=str(raw.get("shape", "other")),
            colour=wanted[name].get("colour"),
            confidence=float(raw.get("confidence", 0.6)),
            source="astra", notes=notes))
    missing = sorted(set(wanted) - {d.name for d in out})
    if missing:
        raise ValueError(f"the reply does not contain {missing}")
    return out


def detect_objects(image, names: Sequence[Any], *, model: str = "gpt-6-astra",
                   client=None, order: str = "rgb",
                   retries: int = 1) -> List[Detection]:
    """Name the objects in one frame with one Responses-API call.

    ``names`` may be plain strings or the ``{"name", "kind", "colour"}`` dicts
    the CLI builds. ``order`` says whether ``image`` is RGB (this module's
    convention, and Pillow's) or BGR (OpenCV's) — getting it wrong is silent
    and turns a brown cup blue, so it is a parameter rather than a guess.

    One retry on a reply that is not parseable, with the parse error fed back;
    a second failure raises, because a detector that keeps inventing scenes
    until one parses is worse than one that stops.
    """
    requested = [{"name": n, "kind": "object", "colour": None}
                 if isinstance(n, str) else dict(n) for n in names]
    array = np.asarray(image)
    if order.lower() == "bgr":
        array = array[:, :, ::-1]
    height, width = array.shape[:2]
    prompt = ASTRA_PROMPT % {"width": width, "height": height,
                             "names": ", ".join(i["name"] for i in requested)}
    if client is None:                          # pragma: no cover - network
        from openai import OpenAI  # noqa: PLC0415
        client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))
    content: List[Dict[str, Any]] = [
        {"type": "input_text", "text": prompt},
        {"type": "input_image", "detail": "high",
         "image_url": "data:image/jpeg;base64," + _jpeg_base64(array)}]
    messages = [{"role": "user", "content": content}]
    last = None
    for attempt in range(retries + 1):
        response = client.responses.create(model=model, input=messages)
        text = getattr(response, "output_text", "") or ""
        try:
            return _parse_astra(text, requested)
        except (ValueError, KeyError, TypeError, IndexError) as exc:
            last = exc
            if attempt >= retries:
                break
            messages = messages + [
                {"role": "assistant", "content": text},
                {"role": "user", "content":
                    f"That was not usable: {exc}. Reply with the JSON object "
                    f"described above and nothing else."}]
    raise ValueError(f"the detector never returned usable JSON: {last}")


def _jpeg_base64(rgb: np.ndarray) -> str:
    import base64  # noqa: PLC0415
    import io  # noqa: PLC0415

    from PIL import Image  # noqa: PLC0415
    buffer = io.BytesIO()
    Image.fromarray(np.asarray(rgb, dtype=np.uint8)).save(
        buffer, format="JPEG", quality=92)
    return base64.b64encode(buffer.getvalue()).decode("ascii")


# --------------------------------------------------------------------------- #
# pixels -> a scene file the loop consumes
# --------------------------------------------------------------------------- #

def measure(detection: Detection, plane: TablePlane, anchor: Anchor, *,
            interior: Optional[Sequence[float]] = None,
            size: Optional[Sequence[float]] = None) -> Dict[str, Any]:
    """One detection -> one scene object, in ``base`` metres.

    Four measurements, and what each one is worth:

    FOOTPRINT WIDTH, on the plane, between the two ends of the contact band
    (or the bounding box's bottom corners when the detector gave no band).
    This one is real: both ends lie ON the surface whose geometry was just
    solved.

    FOOTPRINT CENTRE. The contact band's bottom row is the NEAR edge of the
    footprint, not its middle — the near side of a cup's base is a radius
    closer to the camera than the base's centre. So the centre is moved half a
    footprint back, along the plane, away from the camera. Without that the
    cup landed 26 mm in front of itself, which is most of a gripper's
    tolerance.

    HEIGHT, by intersecting the ray through the TOP of the silhouette with the
    vertical line through the footprint. The obvious "pixel extent times range
    over fx" over-reports by the same fifth a rim does, for the same reason:
    the top of the object is nearer the camera than its base.

    WIDTH, at the height where the silhouette is WIDEST and on a plane lifted
    to that height — a cup's rim, not its base. Measuring a rim against the
    table top made a 95 mm cup 110 mm wide, and 85% of the wrong one is an
    interior a 50 mm charger does not fit into.

    UNSEEN HORIZONTAL EXTENT: there isn't one. A single view gives one
    silhouette, so the width is written on BOTH horizontal axes and yaw is
    0 — a square footprint is rotation-invariant, which makes that zero
    harmless instead of an invented orientation. ``size_y_measured: false``
    says so in the file.
    """
    x0, y0, x1, y1 = detection.bbox
    base_u, base_v = detection.base_px
    if detection.contact_px is not None:
        left_u, right_u, contact_v = detection.contact_px
    else:
        left_u, right_u, contact_v = x0, x1, base_v
    footprint = float(np.linalg.norm(plane.point(right_u, contact_v)
                                     - plane.point(left_u, contact_v)))
    across, depth = plane.to_plane(base_u, base_v)
    centre_depth = max(depth - footprint / 2.0, 0.0)
    contact = _plane_point(plane, across, centre_depth)
    range_m = float(np.linalg.norm(contact))

    # The TOP of the silhouette is the object's FAR top edge (the back of a
    # cup's rim), not a point above its centre — so the ray is intersected
    # with the vertical through the far side of the footprint. Through the
    # centre instead it overshot the cup by 18 mm, which is a sixth of a cup.
    far = _plane_point(plane, across, max(centre_depth - footprint / 2.0, 0.0))
    top_u, top_v = detection.top_px or ((x0 + x1) / 2.0, y0)
    height = plane.height_above(far, top_u, top_v)
    if not (0.0 < height < 2.0):        # a ray that never meets the vertical
        height = float((base_v - y0) * range_m / plane.fx)

    # WIDTH depends on the shape, and honestly so. A cylinder's silhouette is
    # its diameter from every angle, so its widest row IS the measurement —
    # a cup's rim, 100 mm above the base and therefore measured on a plane
    # lifted to that height. A BOX seen from a corner shows two faces at once
    # and its silhouette is WIDER than it is (the d1-2 charger: 64 mm of
    # silhouette for a 50 mm footprint), so for anything not a cylinder the
    # contact width is the better number and the silhouette is only recorded.
    slice_h = 0.0
    width = footprint
    if detection.widest_px is not None:
        wide_l, wide_r, wide_v = detection.widest_px
        slice_h = min(max(plane.height_above(contact, (wide_l + wide_r) / 2.0,
                                             wide_v), 0.0), height)
        silhouette = float(np.linalg.norm(
            plane.point(wide_r, wide_v, slice_h)
            - plane.point(wide_l, wide_v, slice_h)))
    else:
        slice_h = height / 2.0
        silhouette = float(np.linalg.norm(plane.point(x1, base_v, slice_h)
                                          - plane.point(x0, base_v, slice_h)))
    if detection.shape == "cylinder":
        width = max(width, silhouette)

    notes = list(detection.notes)
    notes.append("yaw is 0 and the width is written on BOTH horizontal axes: "
                 "one view gives one silhouette, so a nonzero yaw here would "
                 "be invented")
    notes.append(f"the footprint centre is {footprint/2*1000:.0f} mm behind "
                 f"the bottom of the silhouette, which is the footprint's "
                 f"NEAR edge, not its middle")
    if height <= 0.005 or width <= 0.005:
        notes.append(f"suspiciously small ({width*1000:.0f} x "
                     f"{height*1000:.0f} mm); check the footprint pixel")
    measured_size = [round(width, 4), round(width, 4), round(height, 4)]
    if size is not None:
        # The one number a single view cannot produce (see the docstring) and
        # the one Shu's hand-made run2 file carried: the charger's 20 mm
        # depth. Declaring it does not touch WHERE the object was measured to
        # be — only how big it is said to be — and the file records both.
        height = float(size[2])
        width = max(float(size[0]), float(size[1]))
    p = anchor.to_base(contact + plane.up * (height / 2.0))
    item: Dict[str, Any] = {
        "name": detection.name, "kind": detection.kind,
        "frame_id": "base",
        "p": [round(float(v), 4) for v in p],
        "size": ([round(float(v), 4) for v in size] if size is not None
                 else measured_size),
        "yaw_rad": 0.0,
        "confidence": round(min(detection.confidence, 0.8), 2),
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
            "footprint_plane_m": [round(float(across), 4),
                                  round(float(centre_depth), 4)],
            "silhouette_edge_plane_m": [round(float(across), 4),
                                        round(float(depth), 4)],
            "range_m": round(range_m, 4),
            "upright": bool(detection.upright),
            "shape": detection.shape,
            "size_y_measured": size is not None,
            "size_measured_m": measured_size,
            "yaw_measured": False,
            "notes": notes}}
    if size is not None:
        item["confidence"] = round(min(detection.confidence + 0.2, 0.9), 2)
        item["measurement"]["notes"].append(
            f"size declared on the command line (--size "
            f"{detection.name}={','.join(format(float(v), 'g') for v in size)}"
            f"); this frame measured {measured_size}")
    if detection.colour:
        item["colour"] = detection.colour
    if detection.kind == "container":
        if interior is not None:
            item["interior"] = [round(float(v), 4) for v in interior]
            item["rim_height_m"] = round(float(interior[2]), 4)
            item["interior_measured"] = True
            item["measurement"]["interior_measured"] = True
            item["measurement"]["notes"].append(
                "interior declared MEASURED on the command line "
                "(--interior); nothing in this frame checked it")
        else:
            interior_h = max(height - RIM_DROP_M, 0.005)
            item["interior"] = [round(width * INTERIOR_FRACTION, 4),
                                round(width * INTERIOR_FRACTION, 4),
                                round(interior_h, 4)]
            item["rim_height_m"] = round(interior_h, 4)
            # THE FLAG THE KIT READS, not just a note. `Place` refuses a
            # placement into a container whose interior is a guess, and a
            # perceived interior IS a guess: a wall thickness is not visible
            # from one view above and in front. Saying so in a comment while
            # letting `ContainerView` default to `interior_measured=True`
            # would be the quiet version of lying.
            item["interior_measured"] = False
            item["measurement"]["interior_measured"] = False
            item["measurement"]["notes"].append(
                f"interior = {INTERIOR_FRACTION:g} x the measured outside in "
                f"xy and {RIM_DROP_M*1000:.0f} mm below the rim in z. "
                f"manipulation_kit's Place REFUSES a placement into an "
                f"estimated interior: measure this cup once and pass "
                f"--interior {detection.name}=0.08,0.08,0.10")
    if not detection.upright:
        item["measurement"]["notes"].append(
            "the detector says this object is NOT upright; its footprint and "
            "height are read as if it were, so both are wrong")
    return item


def build_scene(plane: TablePlane, anchor: Anchor,
                detections: Sequence[Detection], *, table_name: str = "table",
                table_depth_m: Optional[float] = None,
                table_thickness_m: float = 0.02,
                interiors: Optional[Dict[str, Sequence[float]]] = None,
                sizes: Optional[Dict[str, Sequence[float]]] = None,
                source_image: Optional[str] = None) -> Dict[str, Any]:
    """The ``--scene`` file ``examples/agent/live.py`` loads, measured."""
    depth = table_depth_m if table_depth_m is not None else plane.depth_m
    depth_note = "measured from the fitted near edge"
    if table_depth_m is not None:
        depth_note = "given on the command line (--table-depth)"
    elif depth is None:
        depth = 0.40
        depth_note = ("ASSUMED: the near edge is outside the frame, so the "
                      "table's depth was not measured. Pass --table-depth, or "
                      "retake the frame with the whole top visible")
    top_z = anchor.to_base(plane.P1)[2]
    corners = {
        name: [round(float(v), 4)
               for v in anchor.to_base(_plane_point(plane, across, along))]
        for name, (across, along) in
        (("far_left", (0.0, 0.0)), ("far_right", (plane.width_m, 0.0)),
         ("near_left", (0.0, depth)), ("near_right", (plane.width_m, depth)))}
    centre = np.mean([corners["far_left"], corners["far_right"],
                      corners["near_left"], corners["near_right"]], axis=0)
    table = {
        "name": table_name, "kind": "surface", "frame_id": "base",
        "p": [round(float(centre[0]), 4), round(float(centre[1]), 4),
              round(float(top_z) - table_thickness_m / 2.0, 4)],
        "size": [round(float(depth), 4), round(plane.width_m, 4),
                 table_thickness_m],
        "yaw_rad": 0.0,
        "confidence": 0.6,
        "measurement": {
            "top_z_base_m": round(float(top_z), 4),
            "width_m": "PRIOR (--table-width); the one known length",
            "depth_m": depth_note,
            "corners_base": corners,
            "camera_to_top_m": round(plane.camera_distance_m, 4),
            "far_corner_px": [[round(float(v), 1) for v in plane.far_px[0]],
                              [round(float(v), 1) for v in plane.far_px[1]]],
            "near_corner_px": (None if plane.near_px is None else
                               [[round(float(v), 1) for v in plane.near_px[0]],
                                [round(float(v), 1) for v in plane.near_px[1]]]),
            "fit": dict(plane.quality)}}
    readme = [
        "MEASURED FROM ONE HEAD FRAME by examples/agent/perceive.py. Priors: "
        f"the table top's width ({plane.width_m:.3f} m) and fx "
        f"({plane.fx:.1f} px). Nothing else was measured by hand.",
        f"anchor: {anchor.mode}; calibrated: {str(anchor.calibrated).lower()}. "
        "Nothing in this file is a calibrated extrinsic.",
        "base = dual_base (torso platform), +x forward, +y robot LEFT, +z up.",
        "Every object carries `confidence` and a `measurement` block saying "
        "what was seen and what was assumed. Read them before trusting a "
        "number to a millimetre.",
    ] + [f"anchor note: {note}" for note in anchor.notes]
    if source_image:
        readme.insert(1, f"source frame: {source_image}")
    return {"_README": readme,
            "_perceive": {"anchor": anchor.to_json(),
                          "fx": plane.fx, "table_width_m": plane.width_m,
                          "image": source_image},
            "frames": [],
            "objects": [table] + [
                measure(d, plane, anchor,
                        interior=(interiors or {}).get(d.name),
                        size=(sizes or {}).get(d.name))
                for d in detections]}


def _plane_point(plane: TablePlane, across: float, along: float) -> np.ndarray:
    """A point on the table from plane coordinates, in camera coordinates."""
    return plane.P1 + (plane.E / plane.width_m) * across + (-plane.D) * along


# --------------------------------------------------------------------------- #
# the debug frame
# --------------------------------------------------------------------------- #

def debug_image(image_rgb: np.ndarray, plane: TablePlane,
                detections: Sequence[Detection]) -> np.ndarray:
    """The frame with the fit drawn on it. Look at this before believing the
    numbers: a plane fit that latched onto the floor still produces a tidy
    JSON file."""
    out = np.array(image_rgb, dtype=np.uint8, copy=True)
    height, width = plane.shape

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
        for pair in ((( x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
                     ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))):
            segment(pair[0], pair[1], (255, 0, 0))
        dot(detection.base_px[0], detection.base_px[1], (255, 0, 255), radius=5)
    return out


def write_png(path, rgb: np.ndarray) -> None:
    try:
        from PIL import Image  # noqa: PLC0415
    except ImportError:             # pragma: no cover - environment specific
        import cv2  # noqa: PLC0415
        cv2.imwrite(str(path), np.asarray(rgb)[:, :, ::-1])
        return
    Image.fromarray(np.asarray(rgb, dtype=np.uint8)).save(str(path))


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def read_fx(path) -> float:
    """``fx`` out of an intrinsics JSON, in the shapes that turn up.

    d1-inference writes ``{"fx": ..., "fy": ...}``; a ROS camera_info dump
    writes ``{"K": [...9]}`` or ``{"camera_matrix": {"data": [...9]}}``. Only
    ``fx`` is used — the principal point is taken at the image centre, which
    is what the plane fit assumes and what these frames support.
    """
    blob = json.loads(Path(path).read_text(encoding="utf-8"))
    for key in ("fx", "focal_length_px"):
        if isinstance(blob.get(key), (int, float)):
            return float(blob[key])
    for key in ("K", "camera_matrix", "intrinsic_matrix"):
        value = blob.get(key)
        if isinstance(value, dict):
            value = value.get("data")
        if isinstance(value, list) and value:
            flat = np.asarray(value, dtype=float).ravel()
            if flat.size >= 1:
                return float(flat[0])
    raise SystemExit(f"{path}: no fx, K or camera_matrix in this file")


def parse_extents(specs: Sequence[str], flag: str) -> Dict[str, List[float]]:
    """``["cup=0.08,0.08,0.10"]`` -> ``{"cup": [0.08, 0.08, 0.10]}``.

    Shared by ``--interior`` and ``--size``, which are the two escape hatches
    for the two things one view cannot see.
    """
    out: Dict[str, List[float]] = {}
    for spec in specs or ():
        if "=" not in spec:
            raise SystemExit(f"{flag}: {spec!r} is not NAME=LX,LY,LZ")
        name, values = spec.split("=", 1)
        try:
            numbers = [float(v) for v in values.split(",")]
        except ValueError:
            raise SystemExit(f"{flag} {name}: {values!r} is not three "
                             f"numbers") from None
        if len(numbers) != 3 or min(numbers) <= 0:
            raise SystemExit(f"{flag} {name}: need three positive metres, "
                             f"got {values!r}")
        out[name.strip()] = numbers
    return out


def parse_anchor(spec: str) -> Dict[str, Any]:
    """``camera`` or ``far-edge-x=0.76[,centre-y=0.0]``."""
    spec = spec.strip()
    if spec == "camera":
        return {"mode": "camera"}
    out: Dict[str, Any] = {"mode": "far-edge-x"}
    for token in spec.split(","):
        if "=" not in token:
            raise SystemExit(f"--anchor: {token!r} is not key=value")
        key, value = token.split("=", 1)
        key = key.strip().replace("_", "-")
        if key not in ("far-edge-x", "centre-y", "center-y"):
            raise SystemExit(f"--anchor: unknown key {key!r}; use "
                             f"far-edge-x=<m>[,centre-y=<m>] or 'camera'")
        out["centre_y" if key.endswith("-y") else "far_edge_x"] = float(value)
    if "far_edge_x" not in out:
        raise SystemExit("--anchor far-edge-x=<m> needs the x")
    out.setdefault("centre_y", 0.0)
    return out


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="perceive.py", description=__doc__.splitlines()[0],
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--image", type=Path, required=True,
                        help="one head-camera frame (jpg/png)")
    parser.add_argument("--table-width", type=float, default=0.60,
                        help="the ONE prior: the table top's width, metres")
    parser.add_argument("--fx", type=float, default=DEFAULT_FX,
                        help="focal length in pixels; principal point is "
                             "taken at the image centre")
    parser.add_argument("--intrinsics", type=Path, default=None,
                        help="read fx from a camera JSON instead of --fx")
    parser.add_argument("--anchor", default="camera",
                        help="'camera' (zero-shot, measures the table height) "
                             "or far-edge-x=<m>[,centre-y=<m>]")
    parser.add_argument("--table-z", type=float, default=None,
                        help="table-top z in base; REQUIRED by "
                             "--anchor far-edge-x, ignored by --anchor camera")
    parser.add_argument("--table-depth", type=float, default=None,
                        help="override the measured near-to-far depth, metres")
    parser.add_argument("--table-name", default="table")
    parser.add_argument("--interior", action="append", default=[],
                        metavar="NAME=LX,LY,LZ",
                        help="declare a container's interior as MEASURED "
                             "(metres). Without it a perceived interior is an "
                             "estimate and manipulation_kit's Place refuses "
                             "to put anything in it, which is the rule and "
                             "not a bug. Repeatable.")
    parser.add_argument("--size", action="append", default=[],
                        metavar="NAME=LX,LY,LZ",
                        help="declare an object's size as MEASURED (metres), "
                             "overriding the fit. The escape hatch for the "
                             "extent along the axis a single view cannot see "
                             "— the d1-2 charger is 50 mm of silhouette and "
                             "20 mm of charger. Repeatable.")
    parser.add_argument("--neck-pitch", type=float, default=0.0,
                        help="URDF neck_tilt, radians, POSITIVE LOOKS DOWN "
                             "(= -pitch from GET /v1/neck/state)")
    parser.add_argument("--neck-yaw", type=float, default=0.0,
                        help="URDF neck_pan, radians (GET /v1/neck/state)")
    parser.add_argument("--lift", type=float, default=None,
                        help="slider height_m; does NOT move the camera in "
                             "base, only reports the table's floor height")
    parser.add_argument("--assume-level", dest="assume_level",
                        action="store_true", default=True,
                        help="--anchor camera: take the top to be horizontal "
                             "and correct the nominal aim to match")
    parser.add_argument("--no-assume-level", dest="assume_level",
                        action="store_false",
                        help="trust the nominal camera aim as it is")
    parser.add_argument("--objects", default="",
                        help="name:kind[:colour] list, e.g. "
                             "'charger:object:white,cup:container:brown'")
    parser.add_argument("--detector", choices=("astra", "mask"),
                        default="mask")
    parser.add_argument("--model", default=os.environ.get("OPENAI_MODEL",
                                                          "gpt-6-astra"))
    parser.add_argument("--out", type=Path, default=None,
                        help="write the scene here as well as printing it")
    parser.add_argument("--debug", type=Path, default=None,
                        help="write an annotated PNG of the fit here")
    return parser


def perceive(args: argparse.Namespace) -> Dict[str, Any]:
    """The whole pipeline, as a function, so the loop can call it."""
    fx = read_fx(args.intrinsics) if args.intrinsics else args.fx
    image = load_image(args.image)
    plane = fit_table_plane(image, fx=fx, table_width_m=args.table_width)
    spec = parse_anchor(args.anchor)
    if spec["mode"] == "camera":
        anchor = anchor_camera(plane, neck_pitch=args.neck_pitch,
                               neck_yaw=args.neck_yaw,
                               assume_level=args.assume_level)
    else:
        if args.table_z is None:
            raise SystemExit(
                "--anchor far-edge-x measures no height: pass --table-z <m> "
                "(the table top's z in base), or use --anchor camera, which "
                "measures it from the fit")
        anchor = anchor_far_edge(plane, far_edge_x=spec["far_edge_x"],
                                 centre_y=spec["centre_y"],
                                 table_z=args.table_z)
    requested = _requested(args.objects) if args.objects else []
    if not requested:
        detections: List[Detection] = []
    elif args.detector == "mask":
        detections = detect_objects_mask(image, plane, requested)
    else:
        detections = detect_objects(image, requested, model=args.model)
    scene = build_scene(plane, anchor, detections,
                        table_name=args.table_name,
                        table_depth_m=args.table_depth,
                        interiors=parse_extents(args.interior, "--interior"),
                        sizes=parse_extents(args.size, "--size"),
                        source_image=str(args.image))
    if args.lift is not None:
        from manipulation_kit.description.head_camera import (  # noqa: PLC0415
            floor_to_base_m)
        top_z = scene["objects"][0]["measurement"]["top_z_base_m"]
        scene["_perceive"]["table_height_above_floor_m"] = round(
            top_z + floor_to_base_m(args.lift), 4)
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(scene, indent=1) + "\n",
                                  encoding="utf-8")
    if args.debug:
        Path(args.debug).parent.mkdir(parents=True, exist_ok=True)
        write_png(args.debug, debug_image(image, plane, detections))
    return scene


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        scene = perceive(args)
    except PlaneFitError as exc:
        print(f"[perceive] the plane fit refused ({exc.reason}): {exc.detail}",
              file=sys.stderr)
        return 2
    print(json.dumps(scene, indent=1))
    table = scene["objects"][0]
    print(f"\ntable top z = {table['measurement']['top_z_base_m']:+.3f} m in "
          f"base, {table['size'][0]:.3f} deep x {table['size'][1]:.3f} wide "
          f"({table['measurement']['depth_m']})", file=sys.stderr)
    for item in scene["objects"][1:]:
        print(f"{item['name']:>12}  p {np.round(item['p'], 3)}  size "
              f"{np.round(item['size'], 3)}  confidence {item['confidence']}",
              file=sys.stderr)
    if args.out:
        print(f"\nwrote {args.out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
