"""One head frame -> the table's plane and rectangle, with NO scene calibration.

Moved from ``examples/agent/perceive.py`` (design C.7, L9): robust line and
plane fitting and the metric solve, with no D1 and no model in them.

WHAT THIS CAN AND CANNOT DO
---------------------------
:mod:`.camera` turns a pixel into a RAY in ``base`` exactly, from robot facts
alone. A ray plus a HORIZONTAL PLANE AT A KNOWN HEIGHT is a point, and
everything here is that one operation. The catch, stated once and inherited
everywhere: **one camera cannot measure the height of the plane it is looking
at.** Twice as far and twice as big is the same picture. The scale has to come
from outside the geometry, and there are exactly three honest places it can
come from (:data:`HEIGHT_SOURCES`):

``declared``      somebody, or something, says so — in the agent loop, the
                  MODEL, looking at the same frame with the robot's own hands
                  at known base-frame positions in it for scale.
``known-length``  a known length across the far edge — OPTIONAL, and the only
                  scene number this module accepts. It solves the height in
                  closed form (:func:`table_z_from_known_length`).
``provisional``   nobody has said. The plane goes at the z of the arms' HOME
                  tool points — a robot fact, not a measurement of anything in
                  front of the camera — flagged ``provisional`` in every
                  object it touches.

The rest of the fit is scale-free and therefore always available: masking the
top, fitting the far edge and the two side edges, and projecting those lines
onto the plane gives the table's RECTANGLE — where it is, how big, which way
it is turned — correct in proportion for whatever the height turns out to be.

``level_correction_deg`` falls out for free and is worth reading: it is how far
the nominal camera aim is from seeing the fitted top as horizontal, it needs no
known length, and it is large when the neck angle you passed is wrong or its
sign is.

The table-top mask (:func:`table_top_mask`) is a colour threshold tuned for
the d1-2 JP wagon's bright, unsaturated top; its thresholds are arguments.
Nothing here decodes an image: pass an ``(H, W, 3)`` uint8 RGB array.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from scipy import ndimage

from .camera import DEFAULT_FX, PROVISIONAL_UNCERTAINTY_M

#: Table-top mask thresholds, OpenCV HSV ranges (S and V in 0..255). The JP
#: wagon top is a bright, almost unsaturated pink; the brick wall behind it and
#: the black trolley below it are neither.
TOP_S_MAX = 45
TOP_V_MIN = 140

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
# HSV, and the table-top mask — no OpenCV required
# --------------------------------------------------------------------------- #

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

        Orientation-free: rotating the camera does not move it. That is what
        makes it a usable cross-check against ``camera.p[2] - table_z`` when a
        known length HAS fixed the scale, and it is also why it cannot supply
        the scale itself — it is a distance in whatever units the fit was
        solved in.
        """
        return float(abs(self.P1 @ self.n))

    @property
    def up(self) -> np.ndarray:
        """Plane normal pointing from the table toward the camera."""
        return self.n if (self.P1 @ self.n) < 0 else -self.n


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
# the table in BASE: a horizontal plane at a height somebody is accountable for
# --------------------------------------------------------------------------- #

#: Where a table's height can come from, worst to best. The string travels
#: with every number derived from it, into the scene file and into the text
#: the model reads, because "0.166" and "0.166, guessed" plan the same and
#: fail differently.
HEIGHT_SOURCES = ("provisional", "known-length", "declared")


@dataclass
class TableInBase:
    """The plane the loop actually uses: horizontal, at ``z``, in ``base``.

    ``z`` is the ONLY thing a single view cannot supply (see
    :mod:`manipulation_kit.perception.camera`), so it arrives with a ``source`` and an
    uncertainty and never without them. Everything else here — the rectangle's
    centre, extent and yaw — IS measured, by projecting the fitted edge lines
    onto that plane through the robot's own camera model.
    """

    z: float
    source: str
    uncertainty_m: float
    corners: Optional[np.ndarray] = None        # 4x3 base, far-L far-R near-R near-L
    depth_measured: bool = False
    notes: List[str] = field(default_factory=list)

    @property
    def centre(self) -> Optional[np.ndarray]:
        return None if self.corners is None else self.corners.mean(axis=0)

    def extent(self) -> Optional[Tuple[float, float, float]]:
        """``(depth, width, yaw)`` of the rectangle, metres and radians."""
        if self.corners is None:
            return None
        far = self.corners[1] - self.corners[0]          # image-left -> right
        side = self.corners[3] - self.corners[0]         # far -> near
        width = float(np.linalg.norm(far[:2]))
        depth = float(np.linalg.norm(side[:2]))
        # ``size`` is (length, width, thickness) along the object's OWN axes,
        # and its length axis is the DEPTH one, so the yaw is the angle of
        # near->far: a wagon squarely in front of the robot reads 0, not 180.
        yaw = float(math.atan2(-side[1], -side[0]))
        return depth, width, (yaw + math.pi) % (2 * math.pi) - math.pi

    def to_json(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "z": round(float(self.z), 4),
            "height_source": self.source,
            "height_uncertainty_m": round(float(self.uncertainty_m), 4),
            "depth_measured": bool(self.depth_measured),
            "notes": list(self.notes)}
        if self.corners is not None:
            out["corners_base"] = [[round(float(v), 4) for v in c]
                                   for c in self.corners]
        return out


def project_corners(plane: TablePlane, camera, table_z: float,
                    *, source: str = "provisional") -> TableInBase:
    """The fitted edge pixels, put on the horizontal plane at ``table_z``.

    This is the half of the fit that survives having no known length: the far
    and side edges say WHERE the rectangle is and which way it is turned, and
    the camera model turns those pixels into base-frame metres as soon as a
    height exists. Get the height wrong by 10 % and the rectangle is 10 % too
    big and 10 % too far away — wrong together, and correctable by one number.
    """
    height, width = plane.shape
    far_left, far_right = plane.far_px
    if plane.near_px is not None:
        near_left, near_right = plane.near_px
        depth_measured = True
    else:
        # The top runs out of the bottom of the frame. Its near edge is the
        # frame's, which is not a measurement of the table — say so.
        near_left = np.array([far_left[0], float(height - 1)])
        near_right = np.array([far_right[0], float(height - 1)])
        depth_measured = False
    corners = []
    for pixel in (far_left, far_right, near_right, near_left):
        corners.append(camera.locate(pixel[0], pixel[1], plane_z=table_z,
                                     plane_source=source).p)
    notes = []
    if not depth_measured:
        notes.append("the near edge is outside the frame, so the table's "
                     "DEPTH is the frame's edge and not the table's")
    return TableInBase(z=float(table_z), source=source,
                       uncertainty_m=0.0, corners=np.array(corners),
                       depth_measured=depth_measured, notes=notes)


def table_z_from_known_length(plane: TablePlane, camera, length_m: float,
                              *, start_z: Optional[float] = None) -> float:
    """Solve the plane height that makes the far edge ``length_m`` long.

    OPTIONAL REFINEMENT, and the only place in this file a scene number is
    allowed in at all. It is closed form, not a search: the projection of a
    fixed image line onto a horizontal plane scales linearly with the camera's
    height above that plane, so one evaluation gives the constant.

        width(z) = width(z0) * (cam_z - z) / (cam_z - z0)

    Pass a known length if you happen to know one edge of the table. Do not
    go and measure one: the point of this file is that you should not have to.
    """
    cam_z = float(camera.p[2])
    z0 = float(cam_z - 0.5 if start_z is None else start_z)
    if abs(cam_z - z0) < 1e-6:
        raise PlaneFitError("degenerate", "the camera is on the plane")
    reference = project_corners(plane, camera, z0).extent()
    if reference is None or reference[1] < 1e-6:
        raise PlaneFitError("degenerate",
                            "the far edge projects to nothing; no length to "
                            "scale")
    return cam_z - (cam_z - z0) * float(length_m) / reference[1]


def level_correction_deg(plane: TablePlane, camera) -> float:
    """How far the NOMINAL camera aim is from seeing this top as horizontal.

    A diagnostic, and a good one: it is a scale-free measurement, so it works
    with no known length at all. Large means the neck angle you passed is
    wrong, the sign of it is wrong, or the mount is further off than its
    2-3 deg of documented slop. It cannot tell you WHICH.
    """
    normal = camera.r.apply(plane.up)
    return math.degrees(math.atan2(
        float(np.linalg.norm(np.cross(normal, (0.0, 0.0, 1.0)))),
        float(normal[2])))


def neck_pitch_that_levels(plane: TablePlane, *, neck_yaw: float = 0.0,
                           urdf_path: Optional[str] = None,
                           bounds: Tuple[float, float] = (-0.35, 0.65)
                           ) -> Optional[float]:
    """The ``neck_tilt`` at which the NOMINAL frame already sees this top as
    horizontal — the table telling you where the head was pointing.

    It is a diagnostic, not a calibration: it folds the mount's real error and
    the neck's zero offset into one number and cannot separate them. What it
    is good for is a frame whose neck angle nobody wrote down, and for
    noticing that the two disagree. ``None`` when no angle inside the joint's
    own limits does it.
    """
    from .head import HeadCamera  # noqa: PLC0415

    height, width = plane.shape

    def error(pitch: float) -> float:
        camera = HeadCamera.from_robot(width=width, height=height,
                                       fx=plane.fx, neck_pitch=pitch,
                                       neck_yaw=neck_yaw, urdf_path=urdf_path)
        return abs(level_correction_deg(plane, camera))

    lo, hi = bounds
    phi = (math.sqrt(5.0) - 1.0) / 2.0
    a, b = lo, hi
    for _ in range(60):
        c, d = b - phi * (b - a), a + phi * (b - a)
        if error(c) < error(d):
            b = d
        else:
            a = c
    best = (a + b) / 2.0
    if error(best) > 1.0 or not lo < best < hi:
        return None
    return float(best)


def height_above(camera, base_point: np.ndarray, u: float, v: float) -> float:
    """How high above ``base_point`` the pixel ``(u, v)`` is.

    ASSUMING it sits on the vertical line through ``base_point`` — which is
    what a single view has: the top of an upright cup is above its base, so
    the ray through the rim and the vertical through the footprint meet at the
    rim's height. Exact for a symmetric upright object, wrong in proportion to
    how far the point really is off that axis, which is why what it produces
    is reported with a confidence below 1 and never as a measurement of a
    leaning object.
    """
    ray = camera.ray(u, v)
    matrix = np.column_stack([np.array([0.0, 0.0, 1.0]), -ray])
    solution, *_ = np.linalg.lstsq(
        matrix, np.asarray(camera.p, dtype=float) - np.asarray(base_point,
                                                               dtype=float),
        rcond=None)
    return float(solution[0])


#: How sure each height source is, metres. A height the model (or a person)
#: DECLARED is taken as given to a few millimetres; one solved from a known
#: length inherits the nominal mount's ~14 mm bias on the d1-2 frames, so it
#: is quoted at 20; a provisional one is :data:`PROVISIONAL_UNCERTAINTY_M`.
DECLARED_HEIGHT_UNCERTAINTY_M = 0.005
KNOWN_LENGTH_HEIGHT_UNCERTAINTY_M = 0.02


def table_in_base(plane: TablePlane, camera, *,
                  table_z: Optional[float] = None,
                  known_length_m: Optional[float] = None,
                  provisional_z: Optional[float] = None) -> TableInBase:
    """The height policy, once: declared, else known-length, else provisional.

    Returns the projected rectangle with its ``source``, its height
    uncertainty and a note saying where the height came from — the three
    things every number derived from it inherits. ``provisional_z`` defaults
    to :func:`~.camera.provisional_table_z` (the arms' HOME tool height).
    """
    if table_z is not None:
        z, source = float(table_z), "declared"
        uncertainty = DECLARED_HEIGHT_UNCERTAINTY_M
    elif known_length_m is not None:
        z = table_z_from_known_length(plane, camera, float(known_length_m))
        source, uncertainty = "known-length", KNOWN_LENGTH_HEIGHT_UNCERTAINTY_M
    else:
        if provisional_z is None:
            from .camera import provisional_table_z  # noqa: PLC0415
            provisional_z = provisional_table_z()
        z, source = float(provisional_z), "provisional"
        uncertainty = PROVISIONAL_UNCERTAINTY_M
    table = project_corners(plane, camera, z, source=source)
    table.uncertainty_m = uncertainty
    if source == "provisional":
        table.notes.insert(0, (
            "PROVISIONAL HEIGHT: nobody has measured this table. It is the z "
            "of the arms' HOME tool points — a robot fact, not a measurement "
            "of what is in front of the camera — and every distance in this "
            "file scales with it. The loop's model is expected to replace it "
            "with declare_scene on turn 0."))
    elif source == "known-length":
        table.notes.insert(0, (
            f"height solved from a known length of {known_length_m:.3f} m "
            f"across the far edge; that is the one scene number perception "
            f"accepts and it is optional"))
    return table


__all__ = ["DECLARED_HEIGHT_UNCERTAINTY_M", "HEIGHT_SOURCES",
           "KNOWN_LENGTH_HEIGHT_UNCERTAINTY_M", "PlaneFitError",
           "TOP_S_MAX", "TOP_V_MIN", "TableInBase", "TablePlane",
           "fit_edge", "fit_line", "fit_table_plane", "height_above",
           "level_correction_deg", "line_residuals", "neck_pitch_that_levels",
           "project_corners", "solve_plane", "table_in_base",
           "table_top_mask", "table_z_from_known_length", "to_hsv"]
