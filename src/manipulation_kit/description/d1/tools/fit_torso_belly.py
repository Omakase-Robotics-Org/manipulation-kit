"""Fit record of the belly band (``torso_belly``): how far the guard's shape
is from the torso shell, per height and azimuth.

The shell is the CAD torso mesh (``meshes/body_hifi/torso_column_*.obj``, the
colour regions of one mesh, in the ``torso_column`` = ``dual_base`` frame),
sliced every 10 mm over the band's height (z 0.195 .. 0.435 m). The OUTER
outline of each slice is taken per 0.5 deg of azimuth about the band's
centre.

Two outlines are compared, because the CAD is not the built robot:

* ``cad`` — the CAD outline as it is. It is 7-13 mm wider per side than the
  built robot (the measured faces are y +/-0.110, the CAD reaches +/-0.113 at
  the bottom of the band and +/-0.131 at its top), so against it every shape
  whose faces are the measured ones "under-covers" the flanks by up to ~21 mm
  — a statement about the CAD, not about the robot.
* ``measured`` — the CAD outline with each quadrant moved onto the measured
  faces (x -0.130 / +0.135, y +/-0.110): the built robot is taken to have the
  CAD's corner shape on its own faces. This is the outline the corner radii
  are fitted to and the one ``under`` (shell outside the shape: the shape is
  INSIDE the real shell) is bounded against.

Per outline: ``under_mm`` — the largest distance of the shell outside the
shape; ``over_front_mm`` / ``over_back_mm`` — the largest distance of the
shape's boundary outside the shell at the front / back corners (|y| > 50 mm,
|x| > 50 mm), i.e. air the guard treats as body; ``over_any_mm`` anywhere.

Needs the CAD (``MKIT_ASSETS_DIR`` or ``mkit-urdf fetch-assets``)::

    python -m manipulation_kit.description.d1.tools.fit_torso_belly
"""
from __future__ import annotations

import sys
from typing import Dict, Optional, Tuple

import numpy as np

COLOURS = ("white", "navy", "dark", "silver", "red")
Z_SLICES = tuple(round(0.195 + 0.01 * i, 3) for i in range(25))   # 0.195 .. 0.435
AZIMUTH_BINS = 720


def mesh_paths():
    from manipulation_kit import assets  # noqa: PLC0415

    paths = [assets.resolve(f"description/d1/meshes/body_hifi/torso_column_{c}.obj")
             for c in COLOURS]
    return None if any(p is None for p in paths) else [str(p) for p in paths]


def load_triangles(paths) -> np.ndarray:
    tris = []
    for path in paths:
        v, f = [], []
        with open(path) as fh:
            for line in fh:
                if line.startswith("v "):
                    v.append([float(x) for x in line.split()[1:4]])
                elif line.startswith("f "):
                    f.append([int(t.split("/")[0]) - 1 for t in line.split()[1:4]])
        tris.append(np.asarray(v)[np.asarray(f)])
    return np.vstack(tris)


def slice_points(tris: np.ndarray, z0: float, step: float = 0.001) -> np.ndarray:
    """Points (x, y) of the mesh's cross-section at height ``z0``."""
    z = tris[:, :, 2]
    above = z > z0
    cut = tris[above.any(1) & ~above.all(1)]
    out = []
    for tri in cut:
        pts = []
        for i in range(3):
            a, b = tri[i], tri[(i + 1) % 3]
            if (a[2] > z0) != (b[2] > z0):
                pts.append(a + (z0 - a[2]) / (b[2] - a[2]) * (b - a))
        if len(pts) == 2:
            n = max(2, int(np.linalg.norm(pts[1] - pts[0]) / step) + 1)
            out.append(np.linspace(pts[0], pts[1], n)[:, :2])
    return np.vstack(out)


def outer_outline(pts: np.ndarray, centre) -> np.ndarray:
    """The outermost point per azimuth bin about ``centre``."""
    d = pts - centre
    r = np.hypot(d[:, 0], d[:, 1])
    b = ((np.degrees(np.arctan2(d[:, 1], d[:, 0])) % 360) // (360 / AZIMUTH_BINS)).astype(int)
    out = []
    for i in range(AZIMUTH_BINS):
        m = np.flatnonzero(b == i)
        if len(m):
            out.append(pts[m[np.argmax(r[m])]])
    return np.asarray(out)


def onto_faces(outline: np.ndarray, lo, hi, centre) -> np.ndarray:
    """Each quadrant of ``outline`` moved so its extreme sits on the measured
    faces ``lo`` / ``hi`` (x, y)."""
    q = outline.copy()
    front = outline[:, 0] > centre[0]
    left = outline[:, 1] > centre[1]
    q[:, 0] += np.where(front, hi[0] - outline[:, 0].max(), lo[0] - outline[:, 0].min())
    q[:, 1] += np.where(left, hi[1] - outline[:, 1].max(), lo[1] - outline[:, 1].min())
    return q


def rounded_sd(p: np.ndarray, lo, hi, rf: float, rb: float, centre) -> np.ndarray:
    """Signed xy distance of points ``p`` to the box ``lo``..``hi`` with its
    vertical edges rounded (front radius ``rf``, back ``rb``); > 0 outside."""
    x, y = p[:, 0], p[:, 1]
    r = np.where(x > centre[0], rf, rb)
    cx = np.clip(x, lo[0] + r, hi[0] - r)
    cy = np.clip(y, lo[1] + r, hi[1] - r)
    d = np.hypot(x - cx, y - cy) - r
    inner = (x > lo[0] + r) & (x < hi[0] - r) & (y > lo[1] + r) & (y < hi[1] - r)
    face = np.minimum(np.minimum(x - lo[0], hi[0] - x), np.minimum(y - lo[1], hi[1] - y))
    return np.where(inner, -face, d)


def rounded_boundary(lo, hi, rf: float, rb: float, n: int = 400) -> np.ndarray:
    pts = []
    for x, r in ((hi[0], rf), (lo[0], rb)):
        ys = np.linspace(lo[1] + r, hi[1] - r, n)
        pts.append(np.c_[np.full_like(ys, x), ys])
    for y in (lo[1], hi[1]):
        xs = np.linspace(lo[0] + rb, hi[0] - rf, n)
        pts.append(np.c_[xs, np.full_like(xs, y)])
    for sx, sy, r in ((1, 1, rf), (1, -1, rf), (-1, 1, rb), (-1, -1, rb)):
        a = np.linspace(0.0, np.pi / 2, n)
        cx = hi[0] - r if sx > 0 else lo[0] + r
        cy = hi[1] - r if sy > 0 else lo[1] + r
        pts.append(np.c_[cx + sx * r * np.cos(a), cy + sy * r * np.sin(a)])
    return np.vstack(pts)


def _inside_outline(points: np.ndarray, outline: np.ndarray, centre) -> np.ndarray:
    d = outline - centre
    th = np.arctan2(d[:, 1], d[:, 0])
    order = np.argsort(th)
    rs = np.hypot(d[:, 0], d[:, 1])[order]
    e = points - centre
    r_shell = np.interp(np.arctan2(e[:, 1], e[:, 0]), th[order], rs, period=2 * np.pi)
    return np.hypot(e[:, 0], e[:, 1]) <= r_shell + 1e-9


def coverage(outlines: Dict[float, np.ndarray], lo, hi, rf: float, rb: float
             ) -> Dict[str, float]:
    centre = np.array([(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0])
    under = over_f = over_b = over_any = 0.0
    boundary = rounded_boundary(lo, hi, max(rf, 1e-9), max(rb, 1e-9))
    corner = np.abs(boundary[:, 1] - centre[1]) > 0.05
    front = corner & (boundary[:, 0] > centre[0] + 0.05)
    back = corner & (boundary[:, 0] < centre[0] - 0.05)
    for outline in outlines.values():
        under = max(under, float(rounded_sd(outline, lo, hi, max(rf, 1e-9),
                                            max(rb, 1e-9), centre).max()))
        gap = np.min(np.linalg.norm(boundary[:, None, :] - outline[None, :, :], axis=2), axis=1)
        gap = np.where(_inside_outline(boundary, outline, centre), 0.0, gap)
        over_f = max(over_f, float(gap[front].max()))
        over_b = max(over_b, float(gap[back].max()))
        over_any = max(over_any, float(gap.max()))
    return {"under_mm": under * 1e3, "over_front_mm": over_f * 1e3,
            "over_back_mm": over_b * 1e3, "over_any_mm": over_any * 1e3}


def outlines(paths=None) -> Optional[Tuple[Dict[float, np.ndarray], Dict[float, np.ndarray]]]:
    """``(cad, measured)`` outlines per slice height, or ``None`` without CAD."""
    from manipulation_kit.description.d1.tools import generate_d1_urdf as gen  # noqa: PLC0415

    paths = paths or mesh_paths()
    if paths is None:
        return None
    tris = load_triangles(paths)
    lo, hi = gen.BELLY_LO, gen.BELLY_HI
    centre = np.array([(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0])
    cad, measured = {}, {}
    for z in Z_SLICES:
        o = outer_outline(slice_points(tris, z), centre)
        cad[z] = o
        measured[z] = onto_faces(o, lo, hi, centre)
    return cad, measured


def main() -> int:
    from manipulation_kit.description.d1.tools import generate_d1_urdf as gen  # noqa: PLC0415

    got = outlines()
    if got is None:
        print("the torso CAD mesh is absent (set MKIT_ASSETS_DIR)", file=sys.stderr)
        return 2
    cad, measured = got
    lo, hi = gen.BELLY_LO, gen.BELLY_HI
    for label, rf, rb in (("square box", 0.0, 0.0),
                          ("rounded (generator)", gen.BELLY_FRONT_EDGE_R, gen.BELLY_BACK_EDGE_R)):
        for which, o in (("measured", measured), ("cad", cad)):
            c = coverage(o, lo, hi, rf, rb)
            print(f"{label:22s} vs {which:8s}: " + ", ".join(f"{k} {v:.1f}" for k, v in c.items()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
