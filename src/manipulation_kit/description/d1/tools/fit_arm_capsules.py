"""Fit the guard's arm model to the vendor arm meshes, and record the fit.

The motion guard models each arm link as a tube, plus a joint housing where
the link carries one, in the link's own frame (``ARM_CAPSULES`` in
``generate_d1_urdf.py``), built the way the arm is built (Shu, 2026-09-23):

* the **tube** runs along the link at the radius of the link's tube section
  (the joint housings excluded, a radius quantile of the vertices there):
  upper arm ~42.5 mm, forearm ~46-49 mm, wrist ~34-35 mm. Its segment is
  trimmed so its rounded ends stop at the mesh's ends instead of doming a
  radius past them;
* the shoulder (J2, on Link2) and elbow (J4, on Link3 and Link4) **housings**
  are the measured 97 mm cylinders, as capsules of radius ``HOUSING_R`` on
  the joint axis, trimmed the same way so their caps end at the housing's
  faces (at HOME the J4 axis points at the torso: an untrimmed capsule would
  put a 48.5 mm dome in the air beside the flat face).

This does **not** contain every vertex: the housing cover-plate rims (a
capsule cannot fit a squat cylinder's edge) and the tubes' flat ends stick
out. Covering them took ~200 offset capsules per arm, which Shu rejected for
the guard's call rate; the residual is recorded instead, per link and side,
in ``arm_capsule_fit.json`` (``max_protrusion_mm``, up to ~24 mm on Link4),
and ``tests/guard/test_arm_capsule_fit.py`` fails if a mesh ever sticks out
further than its recorded residual + 1 mm. A true cylinder primitive may
replace the housing capsules later.

``Base`` and ``Link1`` sit in the shoulder sleeve and are exempt from the
guard's body and arm-arm checks; each keeps one covering capsule.

The fit is done on the right arm's meshes and measured on both arms' (mirror
parts in identical link frames). Radii are rounded up and endpoints rounded
to 0.1 mm.

Run it with the CAD present (``MKIT_ASSETS_DIR`` or ``mkit-urdf
fetch-assets``)::

    python -m manipulation_kit.description.d1.tools.fit_arm_capsules [--write]

It prints the ``ARM_CAPSULES`` literal and, with ``--write``, rewrites the
fit record.
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


#: The shoulder (J2) and elbow (J4) housings: 97 mm cylinders (Shu, 2026-09-23).
HOUSING_R = 0.0485
#: Slack the mesh test allows on top of a link's recorded residual.
TOLERANCE_M = 0.001

Z = np.array([0.0, 0.0, 1.0])
Y = np.array([0.0, 1.0, 0.0])
#: Per link: the tube segment before trimming (link frame), the axial
#: fraction of it that is pure tube (housings excluded) and the radius
#: quantile taken there, and the housings as (name, centre, axis).
SPEC = {
    "Link2": dict(tube=((0.0, 0.0485, 0.0), (0.0, 0.264, 0.0)), pure=(0.27, 0.5), q=0.9,
                  housings=(("housing_j2", (0.0, 0.0, 0.0), Z),)),
    "Link3": dict(tube=((0.0, 0.0, -0.139), (0.0, 0.0, 0.0)), pure=(0.0, 0.6), q=0.9,
                  housings=(("housing_j4", (0.018, 0.0, 0.0), -Y),)),
    "Link4": dict(tube=((0.0, -0.0485, 0.0), (0.018, -0.264, 0.0)), pure=(0.23, 0.4), q=0.6,
                  housings=(("housing_j4", (0.0, 0.0, 0.0), Z),)),
    "Link5": dict(tube=((0.0, 0.0, -0.169), (0.0, 0.0, 0.0)), pure=(0.0, 0.8), q=0.9, housings=()),
    "Link6": dict(tube=((0.0, 0.02, 0.0), (0.0, -0.02, 0.0)), pure=(0.0, 1.0), q=0.9, housings=()),
    "Link7": dict(tube=((0.0, 0.0, 0.0), (0.0, -0.087, 0.0)), pure=(0.3, 1.0), q=0.9, housings=()),
}


def _trimmed(P, a, b, r):
    """The segment a-b shortened so a radius-r capsule on it ends where the
    vertices end along it (never lengthened; a point if nothing is left)."""
    L = float(np.linalg.norm(b - a))
    if L < 1e-12:
        return a, b
    u = (b - a) / L
    t = (P - a) @ u
    t0, t1 = max(0.0, float(t.min()) + r), min(L, float(t.max()) - r)
    if t0 > t1:
        t0 = t1 = 0.5 * (t0 + t1)
    return a + u * t0, a + u * t1


def fit_link(link, P):
    """``[(part, a, b, r), ...]`` for one link from its vertices ``P``."""
    if link == "Base":
        seg = (np.zeros(3), np.array([0.0, 0.0, 0.1586]))
        return [("tube", *seg, float(seg_dist(P, *seg).max()))]
    if link == "Link1":
        seg = (np.array([0.0, 0.0, -0.077]), np.array([0.0, 0.0, 0.011]))
        return [("tube", *seg, float(seg_dist(P, *seg).max()))]
    spec = SPEC[link]
    caps = []
    for name, centre, axis in spec["housings"]:
        c = np.asarray(centre, float)
        t = (P - c) @ axis
        near = np.linalg.norm((P - c) - np.outer(t, axis), axis=1) <= HOUSING_R
        a, b = _trimmed(P[near], c + axis * t[near].min(), c + axis * t[near].max(), HOUSING_R)
        caps.append((name, a, b, HOUSING_R))
    a, b = (np.asarray(v, float) for v in spec["tube"])
    ab = b - a
    t = ((P - a) @ ab) / (ab @ ab)
    pure = (t >= spec["pure"][0]) & (t <= spec["pure"][1])
    r = float(np.quantile(seg_dist(P[pure], a, b), spec["q"]))
    caps.append(("tube", *_trimmed(P, a, b, r), r))
    return caps


def rounded(capsules):
    """Endpoints to the 0.1 mm grid, radii up to it."""
    return [(part,
             tuple(round(round(float(x) / ROUND_M) * ROUND_M, 4) + 0.0 for x in a),
             tuple(round(round(float(x) / ROUND_M) * ROUND_M, 4) + 0.0 for x in b),
             round(math.ceil(r / ROUND_M - 1e-9) * ROUND_M, 4))
            for part, a, b, r in capsules]


def fit_all():
    table, record = {}, {"_comment": (
        "Fit record of the guard's arm capsules (ARM_CAPSULES in "
        "generate_d1_urdf.py), written by tools/fit_arm_capsules.py from the "
        "vendor arm meshes in manipulation-kit-assets: a tube per link at the "
        "link's real tube radius and the 97 mm J2/J4 housings, trimmed to the "
        "mesh ends. max_protrusion_mm is the largest distance of any mesh "
        "vertex OUTSIDE the link's capsules: the housing cover-plate rims and "
        "tube ends the model does not cover, accepted by Shu 2026-09-23. CI "
        "checks the committed capsules against this record; with the CAD "
        "present it re-measures the meshes and fails past residual + 1 mm."),
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
        caps = rounded(fit_link(link, clouds[0]))
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
