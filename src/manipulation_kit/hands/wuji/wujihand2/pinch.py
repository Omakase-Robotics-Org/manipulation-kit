"""Pinch-aware thumb for the Wuji Hand 2: where the thumb goes to meet the index.

A glove's thumb angles copied 1:1 onto the Wuji's thumb do not pinch. The
Wuji's thumb base sits elsewhere and its segments have other proportions than a
person's, so the same angles put its tip 80 mm from the index tip: forward
kinematics of the vendor's right-hand URDF, over the operator's recorded
pinches (a UDCAP glove, thumb ``0.42 / 0.61 / 0.57 / 0.70`` rad median). The
Wuji pinches with a different shape: ``thumb_S1`` (``cmc_flex``) near its top,
``thumb_S2`` (``cmc_abd``) NEGATIVE, ``thumb_S3`` and ``thumb_S4`` small.

The pinch-aware thumb (:attr:`RetargetConfig.thumb_pinch
<manipulation_kit.hands.wuji.wujihand2.retarget.RetargetConfig.thumb_pinch>`)
blends two thumbs per joint::

    o      = clamp((opposition_angle - pinch_open_rad)
                   / (pinch_closed_rad - pinch_open_rad), 0, 1)
    thumb  = (1 - o) * the 1:1 thumb  +  o * PINCH_TABLE(index S1, S3, S4)

``opposition_angle`` is a glove channel (``pinch_channel``, default
``thumb_cm_pitch``); the index joints are the targets the same map computes for
the index finger on that call, so the thumb aims at where the Wuji's index IS
being sent, not where the wearer's is.

The table
---------
``pinch_table.json`` (beside this module) holds, for every point of a grid over
the index's ``S1``, ``S3`` and ``S4``, the four thumb joints that put the thumb
tip on the index's pad. It was solved offline by
:mod:`manipulation_kit.hands.wuji.wujihand2.pinch_build` from the vendor's
right-hand URDF and its meshes, which are not part of this repository; the
file records the URDF's SHA-256 and every parameter of the solve. Read with
trilinear interpolation, so a thumb target moves continuously as the index
curls; an index outside the grid reads the grid's edge.

Per operator: where opposition starts and ends
----------------------------------------------
``pinch_open_rad`` and ``pinch_closed_rad`` are the opposition angles that read
as "not pinching" (``o = 0``) and "pinching" (``o = 1``). They are fields of
the binding's ``retarget_config``, so an operator's own values go in their
binding. They are where a fourth, "pinch", calibration pose would write: the
operator holds an open hand and then a pinch, and the two readings of
``pinch_channel`` become ``pinch_open_rad`` and ``pinch_closed_rad``. Nothing
here takes that pose yet.
"""
from __future__ import annotations

import json
import math
from functools import lru_cache
from pathlib import Path
from typing import Optional, Sequence, Tuple

#: The table file, beside this module (package data).
TABLE_FILE = Path(__file__).with_name("pinch_table.json")

#: The schema the table file declares.
SCHEMA = "manipulation_kit.wuji_pinch_table/1"

#: The index joints the table is indexed by, in table axis order.
INDEX_AXES: Tuple[str, ...] = ("index_S1", "index_S3", "index_S4")

#: The thumb joints the table holds, in value order.
THUMB_AXES: Tuple[str, ...] = ("thumb_S1", "thumb_S2", "thumb_S3", "thumb_S4")

#: Per thumb joint (S1..S4), how much faster than ``o`` its blend runs:
#: joint ``n`` uses ``min(1, BLEND_LEAD[n] * o)``. ``thumb_S2`` completes its
#: blend halfway: the 1:1 thumb's positive ``S2`` (towards the index) is what
#: pushes a half-opposed thumb into the index finger, so it leaves first.
BLEND_LEAD: Tuple[float, ...] = (1.0, 2.0, 1.0, 1.0)


class PinchTable:
    """Index (S1, S3, S4) -> thumb (S1..S4) in radians, trilinear on a grid."""

    def __init__(self, doc: dict):
        if doc.get("schema") != SCHEMA:
            raise ValueError(f"pinch table: schema {doc.get('schema')!r}, want {SCHEMA!r}")
        grid = doc["grid"]
        self.axes = tuple(tuple(float(v) for v in grid[name]) for name in INDEX_AXES)
        for name, axis in zip(INDEX_AXES, self.axes):
            if len(axis) < 2 or any(b <= a for a, b in zip(axis, axis[1:])):
                raise ValueError(f"pinch table: {name} grid must rise, got {axis}")
        self.thumb = doc["thumb"]
        n1, n3, n4 = (len(a) for a in self.axes)
        if len(self.thumb) != n1 or any(len(p) != n3 for p in self.thumb) or any(
                len(r) != n4 or any(len(q) != 4 for q in r) for p in self.thumb for r in p):
            raise ValueError(f"pinch table: thumb must be {n1} x {n3} x {n4} x 4")
        self.meta = {k: v for k, v in doc.items() if k not in ("grid", "thumb")}

    @staticmethod
    def _cell(axis: Sequence[float], x: float) -> Tuple[int, float]:
        x = min(max(x, axis[0]), axis[-1])
        i = 0
        while i < len(axis) - 2 and x >= axis[i + 1]:
            i += 1
        return i, (x - axis[i]) / (axis[i + 1] - axis[i])

    def thumb_for(self, s1: float, s3: float, s4: float) -> Tuple[float, float, float, float]:
        """The thumb joints that meet an index at ``(S1, S3, S4)`` rad."""
        (i, a), (j, b), (k, c) = (self._cell(ax, x) for ax, x in zip(self.axes, (s1, s3, s4)))
        out = [0.0] * 4
        for di, wi in ((0, 1.0 - a), (1, a)):
            for dj, wj in ((0, 1.0 - b), (1, b)):
                for dk, wk in ((0, 1.0 - c), (1, c)):
                    w = wi * wj * wk
                    if w == 0.0:
                        continue
                    q = self.thumb[i + di][j + dj][k + dk]
                    for n in range(4):
                        out[n] += w * q[n]
        return tuple(out)  # type: ignore[return-value]


@lru_cache(maxsize=1)
def pinch_table() -> PinchTable:
    """The shipped table (``pinch_table.json``), read once."""
    return PinchTable(json.loads(TABLE_FILE.read_text()))


def opposition(angle: Optional[float], open_rad: float, closed_rad: float) -> Optional[float]:
    """``o`` in [0, 1] from the opposition channel's angle, or ``None``."""
    if angle is None or not math.isfinite(angle):
        return None
    return min(1.0, max(0.0, (angle - open_rad) / (closed_rad - open_rad)))


__all__ = ["BLEND_LEAD", "INDEX_AXES", "SCHEMA", "TABLE_FILE", "THUMB_AXES", "PinchTable",
           "opposition", "pinch_table"]
