"""Pinch-aware thumb for the Wuji Hand 2: where the thumb goes to meet a finger.

A glove's thumb angles copied 1:1 onto the Wuji's thumb do not pinch. The
Wuji's thumb base sits elsewhere and its segments have other proportions than a
person's, so the same angles put its tip 80 mm from the index tip: forward
kinematics of the vendor's right-hand URDF, over the operator's recorded
pinches (a UDCAP glove, thumb ``0.42 / 0.61 / 0.57 / 0.70`` rad median). The
Wuji pinches with a different shape: ``thumb_S1`` (``cmc_flex``) near its top,
``thumb_S2`` (``cmc_abd``) NEGATIVE, ``thumb_S3`` and ``thumb_S4`` small for
the index, larger for the fingers further from the thumb.

Two ways to say how far, and towards which finger, the thumb is opposed
-----------------------------------------------------------------------

The pinch-aware thumb (:attr:`RetargetConfig.thumb_pinch
<manipulation_kit.hands.wuji.wujihand2.retarget.RetargetConfig.thumb_pinch>`)
blends, per joint, the 1:1 thumb with the thumbs that meet the fingers::

    thumb = (1 - sum(w)) * the 1:1 thumb  +  sum over f of  w[f] * MEET[f](finger f's S1, S3, S4)

``MEET[f]`` reads finger ``f``'s table (below) at the targets the same map
computes for that finger on that call, so the thumb aims at where the Wuji's
finger IS being sent, not where the wearer's is. The weights ``w`` come from

* a :class:`ThumbModel` (:attr:`RetargetConfig.thumb_model`), when the binding
  carries one: a softmax over ``none`` and the four fingers, read from the
  glove's angles and fitted per operator from a calibration session. The
  UDCAP glove has no sensor for thumb opposition (its ``thumb_cm_yaw`` is
  computed from ``thumb_cm_pitch``), so which finger the thumb is at is read
  from the whole hand: the thumb's three measured angles together with which
  finger is curled. A finger's probability below :attr:`ThumbModel.gate_lo`
  gives it no weight, above :attr:`ThumbModel.gate_hi` all of it, linear in
  between (the weights are then scaled down to sum to at most 1);
* otherwise one opposition angle for the index only (the first version):
  ``w[index] = clamp((pinch_channel - pinch_open_rad) / (pinch_closed_rad -
  pinch_open_rad), 0, 1)``.

The tables
----------
``pinch_table.json`` (beside this module) holds, per finger, for every point
of a grid over that finger's ``S1``, ``S3`` and ``S4`` (its spread at 0), the
four thumb joints that put the thumb tip on the finger's pad. It was solved
offline by :mod:`manipulation_kit.hands.wuji.wujihand2.pinch_build` from the
vendor's right-hand URDF and its meshes, which are not part of this
repository; the file records the URDF's SHA-256 and every parameter of the
solve, and per grid point how far the thumb tip stayed from the pad
(``pad_gap_mm``). Read with trilinear interpolation, so a thumb target moves
continuously as the finger curls; a finger outside the grid reads the grid's
edge.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

#: The table file, beside this module (package data).
TABLE_FILE = Path(__file__).with_name("pinch_table.json")

#: The schema the table file declares.
SCHEMA = "manipulation_kit.wuji_pinch_table/2"

#: The fingers the thumb can be sent to meet, one table each.
TARGET_FINGERS: Tuple[str, ...] = ("index", "middle", "ring", "pinky")

#: The joints of a finger a table is indexed by (spread is not), in table axis order.
GRID_AXES: Tuple[str, ...] = ("S1", "S3", "S4")


def finger_axes(finger: str) -> Tuple[str, ...]:
    """The joint names a finger's table is indexed by (``index_S1`` ...)."""
    return tuple(f"{finger}_{a}" for a in GRID_AXES)


#: The index joints the index table is indexed by, in table axis order.
INDEX_AXES: Tuple[str, ...] = finger_axes("index")

#: The thumb joints the tables hold, in value order.
THUMB_AXES: Tuple[str, ...] = ("thumb_S1", "thumb_S2", "thumb_S3", "thumb_S4")

#: Per thumb joint (S1..S4), how much faster than the total weight its blend
#: runs: joint ``n`` uses ``min(1, BLEND_LEAD[n] * sum(w))``. ``thumb_S2``
#: completes its blend halfway: the 1:1 thumb's positive ``S2`` (towards the
#: index) is what pushes a half-opposed thumb into the index finger, so it
#: leaves first.
BLEND_LEAD: Tuple[float, ...] = (1.0, 2.0, 1.0, 1.0)


class PinchTable:
    """One finger (S1, S3, S4) -> thumb (S1..S4) in radians, trilinear on a grid."""

    def __init__(self, part: Mapping[str, Any], finger: str = "index"):
        self.finger = finger
        grid = part["grid"]
        self.axes = tuple(tuple(float(v) for v in grid[name]) for name in GRID_AXES)
        for name, axis in zip(GRID_AXES, self.axes):
            if len(axis) < 2 or any(b <= a for a, b in zip(axis, axis[1:])):
                raise ValueError(f"pinch table {finger}: {name} grid must rise, got {axis}")
        self.thumb = part["thumb"]
        n1, n3, n4 = (len(a) for a in self.axes)
        if len(self.thumb) != n1 or any(len(p) != n3 for p in self.thumb) or any(
                len(r) != n4 or any(len(q) != 4 for q in r) for p in self.thumb for r in p):
            raise ValueError(f"pinch table {finger}: thumb must be {n1} x {n3} x {n4} x 4")
        self.pad_gap_mm = part.get("pad_gap_mm")
        self.meta = {k: v for k, v in part.items() if k not in ("grid", "thumb", "pad_gap_mm")}

    @staticmethod
    def _cell(axis: Sequence[float], x: float) -> Tuple[int, float]:
        x = min(max(x, axis[0]), axis[-1])
        i = 0
        while i < len(axis) - 2 and x >= axis[i + 1]:
            i += 1
        return i, (x - axis[i]) / (axis[i + 1] - axis[i])

    def _read(self, grid, s1: float, s3: float, s4: float, width: int) -> List[float]:
        (i, a), (j, b), (k, c) = (self._cell(ax, x) for ax, x in zip(self.axes, (s1, s3, s4)))
        out = [0.0] * width
        for di, wi in ((0, 1.0 - a), (1, a)):
            for dj, wj in ((0, 1.0 - b), (1, b)):
                for dk, wk in ((0, 1.0 - c), (1, c)):
                    w = wi * wj * wk
                    if w == 0.0:
                        continue
                    q = grid[i + di][j + dj][k + dk]
                    if width == 1:
                        out[0] += w * q
                    else:
                        for n in range(width):
                            out[n] += w * q[n]
        return out

    def thumb_for(self, s1: float, s3: float, s4: float) -> Tuple[float, float, float, float]:
        """The thumb joints that meet the finger at ``(S1, S3, S4)`` rad."""
        return tuple(self._read(self.thumb, s1, s3, s4, 4))  # type: ignore[return-value]

    def gap_for(self, s1: float, s3: float, s4: float) -> Optional[float]:
        """How far (mm) that thumb's tip stays from the pad, interpolated, or ``None``."""
        if self.pad_gap_mm is None:
            return None
        return self._read(self.pad_gap_mm, s1, s3, s4, 1)[0]


class PinchTables:
    """The table file: one :class:`PinchTable` per finger of :data:`TARGET_FINGERS`."""

    def __init__(self, doc: Mapping[str, Any]):
        if doc.get("schema") != SCHEMA:
            raise ValueError(f"pinch table: schema {doc.get('schema')!r}, want {SCHEMA!r}")
        fingers = doc.get("fingers") or {}
        missing = [f for f in TARGET_FINGERS if f not in fingers]
        if missing:
            raise ValueError(f"pinch table: no table for {missing}")
        self.tables = {f: PinchTable(fingers[f], f) for f in TARGET_FINGERS}
        self.meta = {k: v for k, v in doc.items() if k != "fingers"}

    def __getitem__(self, finger: str) -> PinchTable:
        return self.tables[finger]


@lru_cache(maxsize=1)
def pinch_tables() -> PinchTables:
    """The shipped tables (``pinch_table.json``), read once."""
    return PinchTables(json.loads(TABLE_FILE.read_text()))


def pinch_table(finger: str = "index") -> PinchTable:
    """One finger's shipped table."""
    return pinch_tables()[finger]


def opposition(angle: Optional[float], open_rad: float, closed_rad: float) -> Optional[float]:
    """``o`` in [0, 1] from the opposition channel's angle, or ``None``."""
    if angle is None or not math.isfinite(angle):
        return None
    return min(1.0, max(0.0, (angle - open_rad) / (closed_rad - open_rad)))


# --- the per-operator opposition model ---------------------------------------------

#: The schema a :class:`ThumbModel` table declares.
MODEL_SCHEMA = "manipulation_kit.wuji_thumb_model/1"

#: The model's classes: ``none`` (the 1:1 thumb) first, then the fingers.
MODEL_CLASSES: Tuple[str, ...] = ("none",) + TARGET_FINGERS


@dataclass(frozen=True)
class ThumbModel:
    """Which finger the thumb is at, from the glove's angles: a softmax
    regression ``p = softmax(weights @ (x / scale) + bias)`` over
    :data:`MODEL_CLASSES`, where ``x`` is the glove's angle on each of
    ``channels`` (radians).

    Fitted per operator (the bench calibration session writes it into the
    binding as ``[bind.retarget_config.thumb_model]``); nothing here fits it.
    ``weights`` has one row per class and one column per channel.
    ``fitted_from`` is free text saying what it was fitted from."""

    channels: Tuple[str, ...]
    scale: Tuple[float, ...]
    weights: Tuple[Tuple[float, ...], ...]
    bias: Tuple[float, ...]
    gate_lo: float = 0.3
    gate_hi: float = 0.7
    #: Per finger of :data:`TARGET_FINGERS` (empty = none): ``(S1, S3, S4)``
    #: rad added to that finger's target in proportion to its weight. The
    #: Wuji's thumb cannot reach every curl a wearer pinches with (a nearly
    #: straight little finger is 40 mm out of its reach); the shift moves
    #: the operator's own pinch curl onto the nearest curl the thumb meets.
    finger_shift: Tuple[Tuple[float, ...], ...] = ()
    #: The most the 1:1 thumb's ``thumb_S2`` may be, rad, before it is
    #: blended (``None`` = the joint's own limit). The UDCAP glove's
    #: ``thumb_cm_yaw`` is not measured (it is ``min(2.5 * thumb_cm_pitch,
    #: 0.611)``), and at 0.611 rad the Wuji's thumb lies inside the index and
    #: middle fingers' first segments for an open hand with the thumb bent.
    thumb_s2_max: Optional[float] = None
    #: A finger's weight fades out as the thumb's reach to it does: full up
    #: to a pad gap (:meth:`PinchTable.gap_for`) of ``reach_mm[0]``, none
    #: from ``reach_mm[1]``, linear between. The tables add 100 mm to the gap
    #: where the thumb would still overlap the finger, so a finger curled out
    #: of the thumb's reach (into the palm) is not aimed at.
    reach_mm: Tuple[float, float] = (15.0, 30.0)
    fitted_from: str = ""
    schema: str = MODEL_SCHEMA

    def __post_init__(self):
        if self.schema != MODEL_SCHEMA:
            raise ValueError(f"thumb_model: schema {self.schema!r}, want {MODEL_SCHEMA!r}")
        n, k = len(self.channels), len(MODEL_CLASSES)
        if n == 0 or len(set(self.channels)) != n:
            raise ValueError("thumb_model: channels must be distinct and not empty")
        if len(self.scale) != n or not all(math.isfinite(s) and s > 0 for s in self.scale):
            raise ValueError(f"thumb_model: scale needs {n} positive numbers")
        if len(self.weights) != k or any(len(r) != n for r in self.weights):
            raise ValueError(f"thumb_model: weights must be {k} rows "
                             f"({', '.join(MODEL_CLASSES)}) of {n} columns")
        if len(self.bias) != k:
            raise ValueError(f"thumb_model: bias needs {k} numbers")
        if not all(math.isfinite(v) for r in self.weights for v in r) or \
                not all(math.isfinite(v) for v in self.bias):
            raise ValueError("thumb_model: weights and bias must be finite")
        if self.finger_shift and (len(self.finger_shift) != len(TARGET_FINGERS) or any(
                len(r) != len(GRID_AXES) or not all(math.isfinite(v) for v in r)
                for r in self.finger_shift)):
            raise ValueError(f"thumb_model: finger_shift needs {len(TARGET_FINGERS)} rows "
                             f"({', '.join(TARGET_FINGERS)}) of {len(GRID_AXES)} finite numbers")
        if len(self.reach_mm) != 2 or not (0.0 <= self.reach_mm[0] < self.reach_mm[1]):
            raise ValueError(f"thumb_model: reach_mm needs 0 <= near < far, got {self.reach_mm}")
        if self.thumb_s2_max is not None and not math.isfinite(self.thumb_s2_max):
            raise ValueError("thumb_model: thumb_s2_max must be finite")
        if not (0.0 <= self.gate_lo < self.gate_hi <= 1.0):
            raise ValueError(f"thumb_model: need 0 <= gate_lo < gate_hi <= 1, "
                             f"got {self.gate_lo}, {self.gate_hi}")

    def probabilities(self, angles) -> Optional[List[float]]:
        """The softmax over :data:`MODEL_CLASSES`, or ``None`` when a channel
        has no finite reading."""
        x = []
        for ch, s in zip(self.channels, self.scale):
            v = angles.get(ch)
            if v is None or not math.isfinite(v):
                return None
            x.append(v / s)
        z = [b + sum(w * v for w, v in zip(row, x)) for row, b in zip(self.weights, self.bias)]
        top = max(z)
        e = [math.exp(v - top) for v in z]
        total = sum(e)
        return [v / total for v in e]

    def reach(self, gap_mm: Optional[float]) -> float:
        """How much of a finger's weight its pad gap leaves, in [0, 1]."""
        if gap_mm is None:
            return 1.0
        near, far = self.reach_mm
        return min(1.0, max(0.0, (far - gap_mm) / (far - near)))

    def finger_weights(self, angles) -> Optional[Dict[str, float]]:
        """Finger -> blend weight (gated, summing to at most 1), or ``None``."""
        p = self.probabilities(angles)
        if p is None:
            return None
        span = self.gate_hi - self.gate_lo
        w = {f: min(1.0, max(0.0, (pf - self.gate_lo) / span))
             for f, pf in zip(TARGET_FINGERS, p[1:])}
        total = sum(w.values())
        if total > 1.0:
            w = {f: v / total for f, v in w.items()}
        return w

    @classmethod
    def from_table(cls, value: Any, where: str, problems: List[str]) -> Optional["ThumbModel"]:
        """A binding's ``thumb_model`` table, or ``None`` with ``problems``."""
        from ....gloves.channels import spec_of  # noqa: PLC0415

        if not isinstance(value, Mapping):
            problems.append(f"{where}: expected a table")
            return None
        known = ("schema", "classes", "channels", "scale", "weights", "bias", "gate_lo",
                 "gate_hi", "finger_shift", "thumb_s2_max", "reach_mm", "fitted_from")
        before = len(problems)
        for key in value:
            if key not in known:
                problems.append(f"{where}: unknown key {key!r} ({', '.join(known)})")
        if list(value.get("classes", MODEL_CLASSES)) != list(MODEL_CLASSES):
            problems.append(f"{where}.classes: must be {list(MODEL_CLASSES)}, "
                            f"got {value.get('classes')!r}")
        channels = value.get("channels")
        if not isinstance(channels, list) or not all(isinstance(c, str) for c in channels):
            problems.append(f"{where}.channels: expected an array of glove channel names")
            return None
        for c in channels:
            if spec_of(c) is None:
                problems.append(f"{where}.channels: {c!r} is not a glove channel")

        def numbers(key: str, v: Any) -> Optional[Tuple[float, ...]]:
            if not isinstance(v, list) or not all(
                    isinstance(x, (int, float)) and not isinstance(x, bool) for x in v):
                problems.append(f"{where}.{key}: expected an array of numbers")
                return None
            return tuple(float(x) for x in v)

        scale = numbers("scale", value.get("scale"))
        bias = numbers("bias", value.get("bias"))
        rows = value.get("weights")
        weights: Optional[Tuple[Optional[Tuple[float, ...]], ...]] = None
        if not isinstance(rows, list):
            problems.append(f"{where}.weights: expected an array of arrays of numbers")
        else:
            weights = tuple(numbers(f"weights[{i}]", r) for i, r in enumerate(rows))
        kw: Dict[str, Any] = {}
        if "reach_mm" in value:
            got = numbers("reach_mm", value["reach_mm"])
            if got is not None:
                kw["reach_mm"] = got
        if "finger_shift" in value:
            rows = value["finger_shift"]
            if not isinstance(rows, list):
                problems.append(f"{where}.finger_shift: expected an array of arrays of numbers")
            else:
                shift = tuple(numbers(f"finger_shift[{i}]", r) for i, r in enumerate(rows))
                if all(r is not None for r in shift):
                    kw["finger_shift"] = shift
        for key in ("gate_lo", "gate_hi", "thumb_s2_max"):
            if key in value:
                v = value[key]
                if not isinstance(v, (int, float)) or isinstance(v, bool):
                    problems.append(f"{where}.{key}: expected a number")
                else:
                    kw[key] = float(v)
        for key in ("schema", "fitted_from"):
            if key in value:
                if not isinstance(value[key], str):
                    problems.append(f"{where}.{key}: expected a string")
                else:
                    kw[key] = value[key]
        if len(problems) > before or scale is None or bias is None or weights is None \
                or any(r is None for r in weights):
            return None
        try:
            return cls(channels=tuple(channels), scale=scale, weights=weights,  # type: ignore[arg-type]
                       bias=bias, **kw)
        except ValueError as exc:
            problems.append(f"{where}: {exc}")
            return None


__all__ = ["BLEND_LEAD", "GRID_AXES", "INDEX_AXES", "MODEL_CLASSES", "MODEL_SCHEMA", "SCHEMA",
           "TABLE_FILE", "TARGET_FINGERS", "THUMB_AXES", "PinchTable", "PinchTables", "ThumbModel",
           "finger_axes", "opposition", "pinch_table", "pinch_tables"]
