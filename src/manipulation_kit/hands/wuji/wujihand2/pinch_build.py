"""Solve the pinch table (``pinch_table.json``) from the vendor's Wuji URDF.

Maintainer tool. The URDF and its STL meshes are the vendor's and are not in
this repository (the same opt-in as ``tests/hands/test_wujihand2.py``), so the
table is solved once, here, and committed; a consumer only reads it
(:mod:`.pinch`)::

    python -m manipulation_kit.hands.wuji.wujihand2.pinch_build \\
        <wuji_v2>/urdf/right.urdf [--out pinch_table.json]

What one grid point is
----------------------
For an index finger at ``(S1, 0, S3, S4)`` (no spread), the thumb's four
joints ``q`` minimise, with ``scipy.optimize.least_squares`` inside the
joints' limits less :data:`MARGIN_FRAC` of each span::

    | thumb_tip(q) - (index_tip + R_index_distal @ (0, offset, 0)) |^2  (mm)
      + POSTURE_WEIGHT * | q - POSTURE |^2                             (rad)

``thumb_tip`` and ``index_tip`` are the URDF's ``*_tip`` frames (the ends of
the last segments); ``(0, offset, 0)`` in the index's distal frame points from
the index tip towards its pad (the palm side). ``offset`` starts at
:data:`OFFSET_MM` and grows by :data:`OFFSET_STEP_MM` while any vertex of the
thumb's last two segments' meshes lies inside the convex hull of one of the
index's segment meshes, up to :data:`OFFSET_MAX_MM`. The posture term picks
one thumb out of the many that reach the same point (four joints, three
coordinates), the same one at every grid point, so neighbouring grid points
get neighbouring thumbs.

Points are solved in grid order (S1 outer, S4 inner), each started from the
previous solution. Everything is deterministic: the same URDF and meshes give
the same file.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np

from .axes import JOINT_LIMITS_RAD
from .pinch import INDEX_AXES, SCHEMA, TABLE_FILE, THUMB_AXES

#: The grid, rad: index S1, S3, S4. Recorded UDCAP pinches reach index S1
#: 1.07, S3 1.03, S4 0.82; past the grid the table reads its edge.
GRID: Dict[str, Tuple[float, ...]] = {
    "index_S1": tuple(round(0.1 * i, 4) for i in range(13)),   # 0 .. 1.2
    "index_S3": tuple(round(0.1 * i, 4) for i in range(14)),   # 0 .. 1.3
    "index_S4": tuple(round(0.1 * i, 4) for i in range(12)),   # 0 .. 1.1
}
#: Kept clear of each thumb joint's limits, as a fraction of its span.
MARGIN_FRAC = 0.02
#: The posture the solve leans towards, thumb S1..S4 rad, and how hard (mm^2/rad^2).
POSTURE = (1.0, -0.3, 0.3, 0.3)
POSTURE_WEIGHT = 16.0
#: The thumb tip's target, from the index tip towards its pad, mm.
OFFSET_MM = 8.0
OFFSET_STEP_MM = 2.0
OFFSET_MAX_MM = 24.0

#: URDF joint names on the right hand, per digit (S1..S4, then the tip).
_CHAINS = {
    "thumb": ("r_thumb_cmc_flex", "r_thumb_cmc_abd", "r_thumb_mcp", "r_thumb_ip",
              "r_thumb_tip_fixed"),
    "index": ("r_index_finger_mcp_flex", "r_index_finger_mcp_abd", "r_index_finger_pip",
              "r_index_finger_dip", "r_index_finger_tip_fixed"),
}
#: The thumb's segments checked against the index (its last two).
_THUMB_CHECKED = ("r_thumb_middle", "r_thumb_distal")


def _rpy(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = (np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y))
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _rot(axis: np.ndarray, q: float) -> np.ndarray:
    a = axis / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * k + (1 - np.cos(q)) * k @ k


def _floats(text: str) -> List[float]:
    return [float(v) for v in text.split()]


def load_stl(path: Path) -> np.ndarray:
    """An STL file's unique vertices, (N, 3) in the file's unit."""
    data = path.read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        v = [_floats(line.split("vertex", 1)[1]) for line in data.decode().splitlines()
             if line.strip().startswith("vertex")]
        return np.unique(np.array(v, float), axis=0)
    n = int(np.frombuffer(data[80:84], "<u4")[0])
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype(
        [("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return np.unique(rec["v"].reshape(-1, 3).astype(float), axis=0)


class Hand:
    """The thumb and index chains of a Wuji URDF, with their visual meshes."""

    def __init__(self, urdf: Path, meshes: bool = True):
        root = ET.parse(urdf).getroot()
        self.joints = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            a = j.find("axis")
            self.joints[j.get("name")] = dict(
                child=j.find("child").get("link"), type=j.get("type"),
                xyz=np.array(_floats(o.get("xyz") or "0 0 0")),
                R=_rpy(*_floats(o.get("rpy") or "0 0 0")),
                axis=np.array(_floats(a.get("xyz"))) if a is not None else None)
        self.mesh: Dict[str, np.ndarray] = {}
        if not meshes:
            return
        for link in root.findall("link"):
            m = link.find("visual/geometry/mesh")
            if m is None:
                continue
            o = link.find("visual/origin")
            xyz = np.array(_floats(o.get("xyz") or "0 0 0")) if o is not None else np.zeros(3)
            rpy = _floats(o.get("rpy") or "0 0 0") if o is not None else [0.0] * 3
            v = load_stl((urdf.parent / m.get("filename")).resolve())
            self.mesh[link.get("name")] = v @ _rpy(*rpy).T + xyz

    def frames(self, digit: str, q4: Sequence[float]):
        """(link, R, p) after each joint of ``digit`` and its tip, wrist frame."""
        rot, pos, out = np.eye(3), np.zeros(3), []
        for k, name in enumerate(_CHAINS[digit]):
            j = self.joints[name]
            pos = pos + rot @ j["xyz"]
            rot = rot @ j["R"]
            if j["type"] == "revolute":
                rot = rot @ _rot(j["axis"], q4[k])
            out.append((j["child"], rot.copy(), pos.copy()))
        return out

    def tip(self, digit: str, q4: Sequence[float]) -> np.ndarray:
        return self.frames(digit, q4)[-1][2]

    def points(self, digit: str, q4: Sequence[float], links=None) -> Dict[str, np.ndarray]:
        return {link: self.mesh[link] @ rot.T + pos for link, rot, pos in self.frames(digit, q4)
                if link in self.mesh and (links is None or link in links)}


def solve_point(hand: Hand, index_q: Sequence[float], start: Sequence[float]):
    """``(thumb q, offset mm)`` for one index pose (see the module docstring)."""
    from scipy.optimize import least_squares  # noqa: PLC0415
    from scipy.spatial import Delaunay  # noqa: PLC0415

    lim = np.array(JOINT_LIMITS_RAD[:4])
    span = lim[:, 1] - lim[:, 0]
    lo, hi = lim[:, 0] + MARGIN_FRAC * span, lim[:, 1] - MARGIN_FRAC * span
    posture, w = np.array(POSTURE), np.sqrt(POSTURE_WEIGHT)
    _, rot, tip = hand.frames("index", index_q)[-1]
    hulls = [Delaunay(v) for v in hand.points("index", index_q).values()]
    q, offset = np.clip(np.asarray(start, float), lo, hi), OFFSET_MM
    while True:
        target = tip + rot @ np.array([0.0, offset / 1000.0, 0.0])
        res = least_squares(
            lambda x: np.concatenate([1000.0 * (hand.tip("thumb", x) - target), w * (x - posture)]),
            q, bounds=(lo, hi), xtol=1e-10, ftol=1e-10, gtol=1e-10, max_nfev=400)
        q = res.x
        thumb = np.vstack(list(hand.points("thumb", q, _THUMB_CHECKED).values()))
        inside = any((h.find_simplex(thumb) >= 0).any() for h in hulls)
        if not inside or offset >= OFFSET_MAX_MM - 1e-9:
            return q, offset
        offset += OFFSET_STEP_MM


def build(urdf: Path) -> dict:
    """The table document for ``urdf`` (a right-hand Wuji Hand 2 URDF)."""
    hand = Hand(urdf)
    g1, g3, g4 = (GRID[n] for n in INDEX_AXES)
    thumb = [[[None] * len(g4) for _ in g3] for _ in g1]
    offsets = []
    q = np.array(POSTURE)
    for a, s1 in enumerate(g1):
        for b, s3 in enumerate(g3):
            for c, s4 in enumerate(g4):
                start = (thumb[a][b][c - 1] if c else thumb[a][b - 1][0] if b
                         else thumb[a - 1][0][0] if a else q)
                q, off = solve_point(hand, (s1, 0.0, s3, s4), start)
                thumb[a][b][c] = [round(float(v), 4) for v in q]
                offsets.append(off)
    return {
        "schema": SCHEMA,
        "source": {
            "urdf": urdf.name,
            "urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
            "solver": "manipulation_kit.hands.wuji.wujihand2.pinch_build",
            "margin_frac": MARGIN_FRAC,
            "posture": list(POSTURE),
            "posture_weight": POSTURE_WEIGHT,
            "offset_mm": [OFFSET_MM, OFFSET_STEP_MM, OFFSET_MAX_MM],
            "offset_raised_at": sum(1 for o in offsets if o > OFFSET_MM),
        },
        "thumb_axes": list(THUMB_AXES),
        "grid": {n: list(GRID[n]) for n in INDEX_AXES},
        "thumb": thumb,
    }


def dumps(doc: dict) -> str:
    """The table as committed: one grid row (S4 run) per line."""
    head = {k: v for k, v in doc.items() if k != "thumb"}
    rows = []
    for plane in doc["thumb"]:
        rows.append("  [\n" + ",\n".join("   " + json.dumps(r) for r in plane) + "\n  ]")
    text = json.dumps(head, indent=1)[:-2] + ',\n "thumb": [\n' + ",\n".join(rows) + "\n ]\n}\n"
    json.loads(text)
    return text


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("urdf", type=Path, help="the vendor's wuji_v2/urdf/right.urdf")
    ap.add_argument("--out", type=Path, default=TABLE_FILE)
    a = ap.parse_args(argv)
    a.out.write_text(dumps(build(a.urdf)))
    print(f"wrote {a.out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
