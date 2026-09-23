"""Fit the guard's arm capsules to the vendor arm meshes, and record the fit.

The motion guard models each arm link as one or two capsules in the link's own
frame (``ARM_CAPSULES`` in ``generate_d1_urdf.py``). They used to be one
joint-to-joint capsule per link with a hand-picked radius, which was up to
23 mm THINNER than the real arm (the Link4 elbow housing) and, on links whose
housing is one-sided, would have to be far FATTER than the real tube to cover
it. This tool derives them from the vendor meshes instead, so the model is the
arm's real shell and the guard's margin is real air:

* every link gets a **tube** capsule and a **housing** capsule;
* links with a joint-to-joint segment (Link2, Link4, Link7) put the tube on
  that segment and the housing on the link's own joint axis at the origin;
  links whose mesh runs back along their own joint axis (Link1, Link3, Link5,
  Link6) put the tube on that axis and the housing on the child joint's axis;
* the segment ends and the housing half-length come from a small fixed grid,
  and the two radii are the pair of minimum total volume for which **every
  mesh vertex lies inside one of the two capsules** (exact: the radius of one
  is swept over its sorted vertex distances and the other takes whatever is
  left);
* the fit is done on the right arm's meshes and checked on the left arm's,
  which are mirror parts in identical link frames; radii are rounded UP and
  endpoints to 0.1 mm, and coverage is re-checked after rounding.

Run it with the CAD present (``MKIT_ASSETS_DIR`` or ``mkit-urdf
fetch-assets``)::

    python -m manipulation_kit.description.d1.tools.fit_arm_capsules [--write]

It prints the ``ARM_CAPSULES`` literal and, with ``--write``, rewrites
``arm_capsule_fit.json`` — the fit record that lets CI (which has no CAD)
check that the committed capsules are the fitted ones and that no vertex
protrudes. ``tests/guard/test_arm_capsule_fit.py`` holds both halves of that
check.
"""
from __future__ import annotations

import hashlib
import itertools
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RECORD = os.path.join(HERE, "..", "arm_capsule_fit.json")
LINKS = ("Base", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6", "Link7")
CHILD = {"Base": "Link1", "Link1": "Link2", "Link2": "Link3", "Link3": "Link4",
         "Link4": "Link5", "Link5": "Link6", "Link6": "Link7", "Link7": "TCP_Link"}
#: Grid resolution of the rounded record, metres.
ROUND_M = 1e-4


def mesh_path(link, side):
    from manipulation_kit import assets  # noqa: PLC0415

    sub = {"R": "right", "L": "left"}[side]
    return assets.resolve(f"description/d1_arm/{sub}/meshes/{link}_{side}.STL")


def mesh_vertices(path):
    from manipulation_kit.description.d1.tools.generate_d1_urdf import _stl_points  # noqa: PLC0415

    pts = np.array(_stl_points(path), float)
    return np.unique(np.round(pts, 7), axis=0)


def sha256(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def seg_dist(P, a, b):
    """Distance of every row of ``P`` to the segment ``a``-``b``."""
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    ab = b - a
    L = ab @ ab
    t = np.clip(((P - a) @ ab) / L, 0, 1) if L > 1e-18 else np.zeros(len(P))
    return np.linalg.norm(P - (a + np.outer(t, ab)), axis=1)


def protrusion(P, capsules):
    """Per-vertex signed distance outside the union of ``capsules`` (m)."""
    return np.min(np.stack([seg_dist(P, a, b) - r for _, a, b, r in capsules]), axis=0)


def _volume(a, b, r):
    return math.pi * r * r * float(np.linalg.norm(np.subtract(b, a))) + 4 / 3 * math.pi * r ** 3


def _two_radii(P, seg0, seg1):
    """The radii pair for two fixed segments that HUGS the mesh.

    Every vertex must be inside at least one capsule. Among the pairs that
    achieve that (capsule 0's radius swept over quantiles of its vertex
    distances, capsule 1 taking whatever is left), the one with the smallest
    mean slack — how far, on average, a vertex sits inside the surface that
    covers it — wins. Minimising volume instead rewards short fat capsules
    whose caps overshoot the mesh; minimising slack keeps each surface on
    the shell it stands for.
    """
    d0 = seg_dist(P, *seg0)
    d1 = seg_dist(P, *seg1)
    best = None
    for r0 in np.unique(np.r_[np.quantile(d0, np.linspace(0.0, 1.0, 201)), 0.0]):
        out = d0 > r0
        r1 = float(d1[out].max()) if out.any() else 0.0
        slack = np.maximum(np.where(d0 <= r0, r0 - d0, -np.inf),
                           np.where(d1 <= r1, r1 - d1, -np.inf))
        score = float(slack.mean())
        if best is None or score < best[0]:
            best = (score, float(r0), r1)
    return best


def _joint(child, side="R"):
    from manipulation_kit.guard.urdf_model import UrdfModel  # noqa: PLC0415

    model = UrdfModel()
    j = model.joints[f"{child}_{side}"]
    return np.array(j.origin.t, float), np.array(j.origin.R, float) @ np.array(j.axis, float)


def fit_link(link, P):
    """``[(part, a, b, r), ...]`` for one link from its vertices ``P``."""
    z = np.array([0.0, 0.0, 1.0])
    if link == "Base":
        # Exempt from every guard stage (it is bolted into the shoulder
        # sleeve); one joint-to-joint capsule that covers the mesh.
        seg = (np.zeros(3), np.array([0.0, 0.0, 0.1586]))
        return [("tube", *seg, float(seg_dist(P, *seg).max()))]
    co, cax = _joint(CHILD[link])
    L = float(np.linalg.norm(co))
    # Both tube ends are trimmed on the grid: a capsule's cap reaches a
    # radius past its segment, so a segment that runs to the end of the mesh
    # overshoots it by that much (into the next joint, where it would read as
    # a same-arm collision that the real links never make).
    trims = np.linspace(0.0, 0.05, 6)
    candidates = []
    if L > 0.05:
        u = co / L
        tmax = min(float((P @ u).max()), L)
        for s, e, h, dc in itertools.product(np.linspace(0.02, 0.10, 9), trims,
                                             np.linspace(0.0, 0.05, 6), (-0.01, 0.0, 0.01)):
            if tmax - e <= s:
                continue
            tube = (u * s, u * (tmax - e))
            housing = (u * dc - z * h, u * dc + z * h)
            candidates.append((tube, housing))
    else:
        zmin = float(P[:, 2].min())
        for s, e, h in itertools.product(np.linspace(zmin, 0.0, 9), trims, np.linspace(0.0, 0.05, 6)):
            if zmin + e >= s:
                continue
            tube = (z * (zmin + e), z * s)
            housing = (co - cax * h, co + cax * h)
            candidates.append((tube, housing))
    best = None
    for tube, housing in candidates:
        v, rt, rh = _two_radii(P, tube, housing)
        if best is None or v < best[0]:
            best = (v, tube, housing, rt, rh)
    _, tube, housing, rt, rh = best
    out = []
    for part, (a, b), r in (("tube", tube, rt), ("housing", housing, rh)):
        if r > 0.0:
            out.append((part, a, b, r))
    return out


def rounded(capsules, clouds):
    """Round endpoints to the grid and radii up, then re-cover every cloud."""
    out = []
    for part, a, b, r in capsules:
        a = tuple(round(float(x) / ROUND_M) * ROUND_M for x in a)
        b = tuple(round(float(x) / ROUND_M) * ROUND_M for x in b)
        out.append([part, tuple(round(x, 4) + 0.0 for x in a), tuple(round(x, 4) + 0.0 for x in b),
                    math.ceil(r / ROUND_M - 1e-9) * ROUND_M])
    for P in clouds:
        while protrusion(P, out).max() > 0.0:
            worst = int(np.argmax(protrusion(P, out)))
            d = [seg_dist(P[worst:worst + 1], a, b)[0] - r for _, a, b, r in out]
            k = int(np.argmin(d))
            out[k][3] += ROUND_M
    return [(part, a, b, round(r, 4)) for part, a, b, r in out]


def fit_all():
    table, record = {}, {"_comment": (
        "Fit record of the guard's arm capsules (ARM_CAPSULES in "
        "generate_d1_urdf.py), written by tools/fit_arm_capsules.py from the "
        "vendor arm meshes in manipulation-kit-assets. max_protrusion_mm is the "
        "largest distance of any mesh vertex OUTSIDE the link's capsules "
        "(<= 0: every vertex is inside). CI checks the committed capsules "
        "against this record; with the CAD present it re-measures the meshes."),
        "links": {}}
    for link in LINKS:
        clouds, meta = [], {}
        for side in ("R", "L"):
            path = mesh_path(link, side)
            if path is None:
                raise SystemExit("the vendor arm meshes are not here: set MKIT_ASSETS_DIR "
                                 "or run `mkit-urdf fetch-assets`")
            P = mesh_vertices(str(path))
            clouds.append(P)
            meta[side] = {"sha256": sha256(str(path)), "vertices": int(len(P))}
        caps = rounded(fit_link(link, clouds[0]), clouds)
        table[link] = caps
        for side, P in zip(("R", "L"), clouds):
            meta[side]["max_protrusion_mm"] = round(float(protrusion(P, caps).max()) * 1000, 3)
        record["links"][link] = {"capsules": [[p, list(a), list(b), r] for p, a, b, r in caps],
                                 "meshes": meta}
    return table, record


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    table, record = fit_all()
    print("ARM_CAPSULES = {")
    for link, caps in table.items():
        print(f"    {link!r}: (")
        for part, a, b, r in caps:
            print(f"        ({part!r}, {a!r}, {b!r}, {r!r}),")
        print("    ),")
    print("}")
    for link, entry in record["links"].items():
        print(link, {s: m["max_protrusion_mm"] for s, m in entry["meshes"].items()})
    if "--write" in argv:
        with open(RECORD, "w") as f:
            json.dump(record, f, indent=1)
            f.write("\n")
        print("wrote", os.path.normpath(RECORD))


if __name__ == "__main__":
    main()
