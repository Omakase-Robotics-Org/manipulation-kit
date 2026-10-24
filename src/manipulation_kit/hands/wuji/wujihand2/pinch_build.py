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
from .pinch import GRID_AXES, SCHEMA, TABLE_FILE, TARGET_FINGERS, THUMB_AXES, finger_axes

#: The grid, rad, per finger: S1, S3, S4. It covers what the UDCAP glove
#: reads for a finger (the vendor solver stops at 1.40, 1.75 and 1.40 rad), so
#: a finger curled past a pinch is not read at the grid's edge.
GRID: Dict[str, Tuple[float, ...]] = {
    "S1": tuple(round(0.1 * i, 4) for i in range(16)),   # 0 .. 1.5
    "S3": tuple(round(0.1 * i, 4) for i in range(19)),   # 0 .. 1.8
    "S4": tuple(round(0.1 * i, 4) for i in range(15)),   # 0 .. 1.4
}
#: Added to a grid point's pad gap when the thumb still overlaps the finger at
#: the largest offset, so a consumer that fades the blend by the gap never
#: aims the thumb into the finger.
OVERLAP_GAP_MM = 100.0
#: Where the solve changes solution branch between two neighbouring grid
#: points (some thumb joint differs by more than JUMP_MAX_RAD), the point with
#: the deeper finger curl (larger S1 + S3 + S4) is marked out of reach
#: (OVERLAP_GAP_MM added to its pad gap), until no two in-reach neighbours
#: differ by that much: a consumer that fades the blend by the gap then never
#: swings the thumb across the jump. The branches change only where the
#: finger is curled deep (S3 past about 1.5 rad), towards a fist.
JUMP_MAX_RAD = 0.5
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
    **{f: tuple(f"r_{f}{'' if f == 'pinky' else '_finger'}_{j}"
                for j in ("mcp_flex", "mcp_abd", "pip", "dip", "tip_fixed"))
       for f in ("index", "middle", "ring", "pinky")},
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


def solve_point(hand: Hand, finger: str, finger_q: Sequence[float], start: Sequence[float]):
    """``(thumb q, offset mm, residual mm, still inside)`` for one pose of
    ``finger`` (see the module docstring); the residual is the thumb tip's
    distance to its target, and ``still inside`` says the thumb's last two
    segments still overlap the finger at the largest offset."""
    from scipy.optimize import least_squares  # noqa: PLC0415
    from scipy.spatial import Delaunay  # noqa: PLC0415

    lim = np.array(JOINT_LIMITS_RAD[:4])
    span = lim[:, 1] - lim[:, 0]
    lo, hi = lim[:, 0] + MARGIN_FRAC * span, lim[:, 1] - MARGIN_FRAC * span
    posture, w = np.array(POSTURE), np.sqrt(POSTURE_WEIGHT)
    _, rot, tip = hand.frames(finger, finger_q)[-1]
    hulls = [Delaunay(v) for v in hand.points(finger, finger_q).values()]
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
            return q, offset, 1000.0 * float(np.linalg.norm(hand.tip("thumb", q) - target)), inside
        offset += OFFSET_STEP_MM


def build_finger(urdf: Path, finger: str) -> dict:
    """One finger's part of the table document for ``urdf``."""
    hand = Hand(urdf)
    g1, g3, g4 = (GRID[n] for n in GRID_AXES)
    thumb = [[[None] * len(g4) for _ in g3] for _ in g1]
    resid = [[[None] * len(g4) for _ in g3] for _ in g1]
    offsets, inside = [], 0
    q = np.array(POSTURE)
    for a, s1 in enumerate(g1):
        for b, s3 in enumerate(g3):
            for c, s4 in enumerate(g4):
                start = (thumb[a][b][c - 1] if c else thumb[a][b - 1][0] if b
                         else thumb[a - 1][0][0] if a else q)
                q, off, r, ins = solve_point(hand, finger, (s1, 0.0, s3, s4), start)
                thumb[a][b][c] = [round(float(v), 4) for v in q]
                # the thumb tip's distance to the pad point (offset + residual), mm,
                # and OVERLAP_GAP_MM more where the thumb still overlaps the finger
                resid[a][b][c] = round(off + r + (OVERLAP_GAP_MM if ins else 0.0), 1)
                offsets.append(off)
                inside += bool(ins)
    part = {"axes": list(finger_axes(finger)),
            "grid": {n: list(GRID[n]) for n in GRID_AXES},
            "offset_raised_at": sum(1 for o in offsets if o > OFFSET_MM),
            "overlap_left_at": inside,
            "thumb": thumb, "pad_gap_mm": resid}
    return mark_jumps(part)


def _neighbours(shape, a, b, c):
    for da, db, dc in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)):
        x, y, z = a + da, b + db, c + dc
        if 0 <= x < shape[0] and 0 <= y < shape[1] and 0 <= z < shape[2]:
            yield x, y, z


def mark_jumps(part: dict) -> dict:
    """Mark branch changes out of reach (see :data:`JUMP_MAX_RAD`), in place."""
    thumb, gap = part["thumb"], part["pad_gap_mm"]
    g1, g3, g4 = (part["grid"][n] for n in GRID_AXES)
    shape = (len(g1), len(g3), len(g4))
    marked = 0
    while True:
        changed = 0
        for a in range(shape[0]):
            for b in range(shape[1]):
                for c in range(shape[2]):
                    if gap[a][b][c] >= OVERLAP_GAP_MM:
                        continue
                    for i, j, k in _neighbours(shape, a, b, c):
                        if gap[i][j][k] >= OVERLAP_GAP_MM:
                            continue
                        if max(abs(x - y) for x, y in zip(thumb[a][b][c], thumb[i][j][k])) <= JUMP_MAX_RAD:
                            continue
                        deeper = (a, b, c) if g1[a] + g3[b] + g4[c] >= g1[i] + g3[j] + g4[k] else (i, j, k)
                        x, y, z = deeper
                        gap[x][y][z] = round(gap[x][y][z] + OVERLAP_GAP_MM, 1)
                        changed += 1
                        if deeper == (a, b, c):
                            break
        marked += changed
        if not changed:
            break
    part["jump_marked_at"] = marked
    return part


def _build_one(job):
    return job[1], build_finger(*job)


def build(urdf: Path, jobs: int = 1) -> dict:
    """The table document for ``urdf`` (a right-hand Wuji Hand 2 URDF)."""
    work = [(urdf, f) for f in TARGET_FINGERS]
    if jobs > 1:
        from multiprocessing import Pool  # noqa: PLC0415
        with Pool(min(jobs, len(work))) as pool:
            parts = dict(pool.map(_build_one, work))
    else:
        parts = dict(map(_build_one, work))
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
            "overlap_gap_mm": OVERLAP_GAP_MM,
            "jump_max_rad": JUMP_MAX_RAD,
        },
        "thumb_axes": list(THUMB_AXES),
        "fingers": {f: parts[f] for f in TARGET_FINGERS},
    }


def dumps(doc: dict) -> str:
    """The table as committed: one grid row (S4 run) per line."""
    def grid(rows):
        return "[\n" + ",\n".join(
            "    [\n" + ",\n".join("     " + json.dumps(r) for r in plane) + "\n    ]"
            for plane in rows) + "\n   ]"
    parts = []
    for f, part in doc["fingers"].items():
        head = {k: v for k, v in part.items() if k not in ("thumb", "pad_gap_mm")}
        body = json.dumps(head)[:-1]
        parts.append(f'  {json.dumps(f)}: {body},\n   "thumb": {grid(part["thumb"])},\n'
                     f'   "pad_gap_mm": {grid(part["pad_gap_mm"])}}}')
    head = {k: v for k, v in doc.items() if k != "fingers"}
    text = json.dumps(head, indent=1)[:-2] + ',\n "fingers": {\n' + ",\n".join(parts) + "\n }\n}\n"
    json.loads(text)
    return text


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("urdf", type=Path, help="the vendor's wuji_v2/urdf/right.urdf")
    ap.add_argument("--out", type=Path, default=TABLE_FILE)
    ap.add_argument("--jobs", type=int, default=1, help="fingers solved in parallel")
    ap.add_argument("--mark-jumps", type=Path,
                    help="only mark the branch changes of this already-solved table file")
    a = ap.parse_args(argv)
    if a.mark_jumps:
        doc = json.loads(a.mark_jumps.read_text())
        doc["source"]["jump_max_rad"] = JUMP_MAX_RAD
        doc["fingers"] = {f: mark_jumps(doc["fingers"][f]) for f in TARGET_FINGERS}
        a.out.write_text(dumps(doc))
        print(f"wrote {a.out}", file=sys.stderr)
        return 0
    a.out.write_text(dumps(build(a.urdf, a.jobs)))
    print(f"wrote {a.out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
