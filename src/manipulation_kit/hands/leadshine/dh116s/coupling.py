"""DH116S passive/active joint coupling tables.

The DH116S has 11 DoF driven by 6 active axes; the passive knuckle and
fingertip joints follow the active drive linkage through a nonlinear worm-gear
linkage.  The vendor manual (§3.4) ships per-finger lookup tables which are
bundled here as CSVs (``data/coupling_*.csv``, regenerate with
``tools/extract_coupling.py``):

- ``linkage_to_knuckle``: active drive/linkage angle (deg) → knuckle angle
- ``knuckle_to_fingertip``: knuckle angle (deg) → fingertip angle

The thumb ships only a knuckle→fingertip table.  All mappings are strictly
monotonic, so inverses are plain flipped interpolations.

Typical uses: URDF/MJCF mimic-joint approximation for sim, and converting
recorded active-axis positions into full 11-DoF hand poses.
"""

from __future__ import annotations

import csv
from functools import lru_cache
from pathlib import Path
from typing import Dict, Tuple

import numpy as np

DATA_DIR = Path(__file__).resolve().parent / "data"

FINGERS = ("thumb", "index", "middle", "ring", "pinky")
PAIRS = ("linkage_to_knuckle", "knuckle_to_fingertip")

# Fingers that ship a linkage→knuckle table (the thumb does not).
_HAS_LINKAGE = ("index", "middle", "ring", "pinky")


class CouplingTable:
    """A monotonic 1-D lookup table with clamped linear interpolation."""

    def __init__(self, x: np.ndarray, y: np.ndarray, name: str = ""):
        if len(x) < 2:
            raise ValueError(f"{name}: need at least 2 rows")
        order = np.argsort(x)
        self.x = np.asarray(x, dtype=float)[order]
        self.y = np.asarray(y, dtype=float)[order]
        if np.any(np.diff(self.x) <= 0):
            raise ValueError(f"{name}: input column not strictly monotonic")
        if np.any(np.diff(self.y) <= 0):
            raise ValueError(f"{name}: output column not strictly monotonic")
        self.name = name

    def __call__(self, deg):
        return np.interp(deg, self.x, self.y)

    def inverse(self, deg):
        return np.interp(deg, self.y, self.x)

    @property
    def input_range(self) -> Tuple[float, float]:
        return float(self.x[0]), float(self.x[-1])

    @property
    def output_range(self) -> Tuple[float, float]:
        return float(self.y[0]), float(self.y[-1])


def _load_csv(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    xs, ys = [], []
    with path.open() as f:
        for row in csv.DictReader(f):
            xs.append(float(row["input_deg"]))
            ys.append(float(row["output_deg"]))
    return np.array(xs), np.array(ys)


@lru_cache(maxsize=None)
def load_table(finger: str, pair: str) -> CouplingTable:
    """Load a bundled coupling table, e.g. ``load_table("index", "linkage_to_knuckle")``."""
    if finger not in FINGERS:
        raise ValueError(f"unknown finger {finger!r} (choose from {FINGERS})")
    if pair not in PAIRS:
        raise ValueError(f"unknown pair {pair!r} (choose from {PAIRS})")
    if pair == "linkage_to_knuckle" and finger == "thumb":
        raise ValueError("the vendor ships no linkage_to_knuckle table for the thumb")
    path = DATA_DIR / f"coupling_{finger}_{pair}.csv"
    x, y = _load_csv(path)
    return CouplingTable(x, y, name=f"{finger}/{pair}")


def load_all() -> Dict[str, Dict[str, CouplingTable]]:
    out: Dict[str, Dict[str, CouplingTable]] = {}
    for finger in FINGERS:
        out[finger] = {}
        for pair in PAIRS:
            if pair == "linkage_to_knuckle" and finger not in _HAS_LINKAGE:
                continue
            out[finger][pair] = load_table(finger, pair)
    return out


# -- convenience mappings ------------------------------------------------------


def linkage_to_knuckle(finger: str, linkage_deg):
    return load_table(finger, "linkage_to_knuckle")(linkage_deg)


def knuckle_to_linkage(finger: str, knuckle_deg):
    return load_table(finger, "linkage_to_knuckle").inverse(knuckle_deg)


def knuckle_to_fingertip(finger: str, knuckle_deg):
    return load_table(finger, "knuckle_to_fingertip")(knuckle_deg)


def fingertip_to_knuckle(finger: str, fingertip_deg):
    return load_table(finger, "knuckle_to_fingertip").inverse(fingertip_deg)


def linkage_to_fingertip(finger: str, linkage_deg):
    """Compose linkage→knuckle→fingertip (flexion fingers only)."""
    return knuckle_to_fingertip(finger, linkage_to_knuckle(finger, linkage_deg))


def fingertip_to_linkage(finger: str, fingertip_deg):
    return knuckle_to_linkage(finger, fingertip_to_knuckle(finger, fingertip_deg))
