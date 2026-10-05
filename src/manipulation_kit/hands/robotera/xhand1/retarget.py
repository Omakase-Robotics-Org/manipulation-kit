"""Anatomical flexion channels → XHAND1 12-joint retarget map.

Hand knowledge only: WHICH anatomical channels drive WHICH XHAND1 joint, and
over which part of its range. The input is the canonical hand pose of
:mod:`manipulation_kit.gloves` (flexion in [0, 1] per channel, 0 = extended,
or ``None``); the output is twelve targets in RADIANS, in the order and
within the limits of :data:`~.axes.JOINTS` — what ``d1-firmwared``'s
``POST /v1/end_effectors/{side}/joints`` takes for this hand.

The map
-------
Every joint is a weighted mean of one or more channels, scaled linearly onto
an output interval and clamped to the joint's limits::

    v   = sum(w_i * flex_i) / sum(w_i)          clamp to [0, 1]
    v   = 1 - v                                  if invert
    rad = lo + v * (hi - lo)                     clamp to the joint limits

====================  ==================================  =================
joint                 default source(s)                   default lo .. hi
====================  ==================================  =================
``thumb_bend``        ``thumb_cm_yaw``                    0 .. 90 deg
``thumb_rota1``       ``thumb_cm_pitch``                  0 .. 90 deg
``thumb_rota2``       ``thumb_mp_pitch``                  0 .. 90 deg
``index_bend``        ``index_mp_swing``                  -5 .. 5 deg
``<finger>_j1``       ``<finger>_mp_pitch``               0 .. 110 deg
``<finger>_j2``       ``<finger>_pip_pitch``              0 .. 110 deg
====================  ==================================  =================

(``<finger>`` is index, middle, ring, pinky; the XHAND1 calls the middle
finger ``mid``.) A joint is ``None`` when any of its non-zero-weighted
channels has no reading: the consumer holds it.

Why these defaults, and what is NOT verified
--------------------------------------------
* **The fingers** follow the DH116S map's reasoning: ``j1`` is the knuckle
  and ``j2`` the coupled middle/last joint, so ``j2`` reads the PIP and
  the DIP adds noise rather than information (weight 0, configurable). The
  open end is 0 rad on every flexion joint (the daemon's ``open`` preset sends
  0 rad to all twelve), so ``invert`` is off.
* **The index spread** reads a BIPOLAR channel, which is 0.5 when the finger
  is neutral. The output interval is symmetric about 0 rad so that neutral
  lands on 0 rad; it uses only ±5 degrees because the manual's range is
  -5 .. 17 degrees and a symmetric interval cannot go further on the negative
  side. WHICH WAY positive spreads the finger has not been checked on a hand:
  flip ``invert`` if it spreads the wrong way. Note that the UDCAP glove
  declares its finger spreads UNCALIBRATED, so with that glove this joint is
  always held.
* **The thumb NEEDS ON-HAND TUNING.** The XHAND1's thumb has three joints and
  the joint names do not say which motion each is. The defaults read the
  thumb's anatomical chain in order (rotation towards the palm, then base
  flexion, then the second joint) and keep ``thumb_rota1`` out of its
  negative range (-60 .. 0 degrees), whose meaning is not documented. Until a
  hand has been watched, treat the thumb mapping as a starting point. Two
  facts about the gloves bear on it: the LitchiBot glove declares
  ``thumb_cm_pitch`` UNCALIBRATED (so ``thumb_rota1`` holds with it
  until it is re-sourced), and its ``thumb_mp_pitch`` slot carries the
  thumb's LAST joint (v1's thumb fields are one joint off on that product),
  while the UDCAP glove measures all three (``thumb_ip_pitch`` included).

Everything above is :class:`RetargetConfig` data, so bring-up changes no code.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar, Dict, List, Mapping, Optional, Tuple

import numpy as np

from ...joints import JointSpec

from math import radians as _rad

from .axes import AXIS_NAMES, JOINT_LIMITS_RAD, JOINTS, NUM_AXES  # noqa: F401


@dataclass(frozen=True)
class JointMap:
    """How one joint is driven: weighted channels -> [lo, hi] rad.

    Shared with :mod:`manipulation_kit.hands.robotera.xhand1_lite`."""

    #: ``(channel, weight)`` pairs. A weight of 0 reads nothing.
    sources: Tuple[Tuple[str, float], ...]
    #: Radians at flexion 0.0 (after ``invert``).
    lo: float
    #: Radians at flexion 1.0 (after ``invert``).
    hi: float
    invert: bool = False


_FINGERS = (("index", "index"), ("mid", "middle"), ("ring", "ring"),
            ("pinky", "pinky"))


def default_joint_maps() -> Dict[str, JointMap]:
    """The defaults tabled in the module docstring, keyed by joint name."""
    maps = {
        "thumb_bend": JointMap((("thumb_cm_yaw", 1.0),), 0.0, _rad(90)),
        "thumb_rota1": JointMap((("thumb_cm_pitch", 1.0),), 0.0, _rad(90)),
        "thumb_rota2": JointMap((("thumb_mp_pitch", 1.0),), 0.0, _rad(90)),
        "index_bend": JointMap((("index_mp_swing", 1.0),), _rad(-5), _rad(5)),
    }
    for joint, finger in _FINGERS:
        maps[f"{joint}_j1"] = JointMap(((f"{finger}_mp_pitch", 1.0),), 0.0, _rad(110))
        maps[f"{joint}_j2"] = JointMap(
            ((f"{finger}_pip_pitch", 1.0), (f"{finger}_dip_pitch", 0.0)), 0.0, _rad(110))
    return maps


@dataclass
class RetargetConfig:
    """Per-joint maps plus output smoothing.

    ``joints`` must name every joint in :attr:`JOINTS` exactly once, and
    every ``lo``/``hi`` must lie inside that joint's limits: a map that asks
    for an angle the hand cannot reach is refused when the config is built,
    not clamped silently on every sample.

    The XHAND1 Lite reuses this class with its own :attr:`JOINTS` and
    defaults (a subclass), so the two hands share one checked map shape."""

    #: The hand this config is for; a subclass names another hand.
    JOINTS: ClassVar[Tuple[JointSpec, ...]] = JOINTS

    joints: Dict[str, JointMap] = field(default_factory=default_joint_maps)
    #: exponential smoothing on the outputs (0 = off), as in the DH116S map.
    ema_alpha: float = 0.0

    def __post_init__(self):
        names = [j.name for j in self.JOINTS]
        missing = set(names) - set(self.joints)
        extra = set(self.joints) - set(names)
        if missing or extra:
            raise ValueError(f"{type(self).__module__} RetargetConfig: missing joints "
                             f"{sorted(missing)}, unknown joints {sorted(extra)}")
        for spec in self.JOINTS:
            name, jlo, jhi = spec.name, spec.min, spec.max
            m = self.joints[name]
            for bound in (m.lo, m.hi):
                if not (jlo - 1e-12 <= bound <= jhi + 1e-12):
                    raise ValueError(f"{name}: output {m.lo}..{m.hi} rad leaves "
                                     f"the joint's limits {jlo}..{jhi}")
            if any(w < 0 for _, w in m.sources):
                raise ValueError(f"{name}: negative weight in {m.sources}")
            if not any(w > 0 for _, w in m.sources):
                raise ValueError(f"{name}: no channel with a positive weight")
        if not 0.0 <= self.ema_alpha < 1.0:
            raise ValueError(f"ema_alpha must be in [0, 1), got {self.ema_alpha}")


class _MappingPose:
    """Adapter so a plain ``{channel: flex}`` dict can stand in for a HandPose."""

    def __init__(self, values: Mapping[str, Optional[float]]):
        self._values = values

    def get(self, channel: str) -> Optional[float]:
        return self._values.get(channel)


class Retargeter:
    """Canonical hand pose → 12 XHAND1 joint targets in radians.

    Version 2 of the retarget contract (:mod:`manipulation_kit.hands.joints`):
    :data:`JOINTS`, :meth:`required_channels`, :meth:`joint_targets`. For the
    version-1 seam it is also callable with a ``flex(channel) -> float``
    callable; on this hand the "wire unit" that call returns is radians,
    because that is what the XHAND1's own protocol carries."""

    JOINTS = JOINTS
    #: The config class; :meth:`__init__` refuses a config for another hand.
    CONFIG = RetargetConfig

    def __init__(self, config: Optional[RetargetConfig] = None):
        self.cfg = config if config is not None else self.CONFIG()
        if self.cfg.JOINTS != self.JOINTS:
            raise ValueError(f"{type(self).__module__}: config is for another hand "
                             f"({type(self.cfg).__module__})")
        self._ema: Optional[np.ndarray] = None

    def required_channels(self) -> Tuple[str, ...]:
        out: List[str] = []
        for spec in self.JOINTS:
            for ch, w in self.cfg.joints[spec.name].sources:
                if w > 0 and ch not in out:
                    out.append(ch)
        return tuple(out)

    def joint_targets(self, pose) -> List[Optional[float]]:
        """One canonical hand pose → twelve ``rad`` targets or ``None``."""
        out: List[Optional[float]] = []
        for spec in self.JOINTS:
            m = self.cfg.joints[spec.name]
            total, acc = 0.0, 0.0
            for ch, w in m.sources:
                if w <= 0:
                    continue
                v = pose.get(ch)
                if v is None:
                    acc = None
                    break
                acc += w * v
                total += w
            if acc is None:
                out.append(None)
                continue
            v = min(1.0, max(0.0, acc / total))
            if m.invert:
                v = 1.0 - v
            out.append(spec.clamp(m.lo + v * (m.hi - m.lo)))
        return self._smooth(out)

    def _smooth(self, targets: List[Optional[float]]) -> List[Optional[float]]:
        a = self.cfg.ema_alpha
        if a <= 0.0:
            return targets
        if self._ema is None:
            self._ema = np.array([np.nan if t is None else t for t in targets])
            return targets
        out: List[Optional[float]] = []
        for i, t in enumerate(targets):
            if t is None:
                out.append(None)
                continue
            prev = self._ema[i]
            self._ema[i] = t if np.isnan(prev) else (1.0 - a) * t + a * prev
            out.append(float(self._ema[i]))
        return out

    def __call__(self, flex) -> np.ndarray:
        """Version-1 seam: ``flex(channel) -> float`` in, 12 radians out."""
        values = {ch: float(flex(ch)) for ch in self.required_channels()}
        return np.array(self.joint_targets(_MappingPose(values)), dtype=float)

    def reset(self) -> None:
        self._ema = None


def build_retarget(config: Optional[RetargetConfig] = None) -> Retargeter:
    """Standard cross-hand retarget constructor (see ``manipulation_kit.hands.get_retarget``)."""
    return Retargeter(config=config)
