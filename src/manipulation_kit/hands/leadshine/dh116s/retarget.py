"""Anatomical flexion channels → DH116S 6-active-axis retarget map.

This module owns the HAND-model knowledge of retargeting: WHICH anatomical
channels drive WHICH DH116S axis, with what weights. It is deliberately
glove-agnostic — the input is a ``flex`` callable ``(channel_name) -> float``
returning per-channel flexion in [0, 1] (0 = open, 1 = closed) for the
anatomical channel names in :data:`CHANNELS`. Glove transport and per-user
range calibration stay in the teleop stack (e.g. d1-vr-teleop wraps its
``RangeCalibration`` + sample into a ``flex`` closure per tick).

(Migrated from d1-vr-teleop ``hand_retarget.py`` 2026-07 — behaviour is
verbatim; only the input contract changed from HandSample+RangeCalibration
to the ``flex`` callable.)

Output order matches the driver axis order (:data:`AXIS_NAMES`, re-exported
from ``.axes``):

    0 thumb lateral swing   1 thumb flexion
    2 index flexion   3 middle flexion   4 ring flexion   5 pinky flexion

Mapping:

- finger flexion (index/middle/ring/pinky): weighted average of the MP and
  PIP pitch flexions. The DIP is anatomically coupled to the PIP and the
  DH116S drives one flexion linkage per finger, so DIP adds noise, not
  information (weight 0 by default, configurable).
- thumb flexion: weighted average of thumb CM pitch and MP pitch flexions.
- thumb lateral swing: thumb CM yaw flexion, blended with CM roll — on the
  glove the thumb's toward-palm motion appears on BOTH yaw and roll (axes
  tumble as the CM rotates); the blend weight NEEDS ON-HAND TUNING and is
  deliberately a parameter (default: yaw only).

Two outputs, one map
--------------------
* ``mapper(flex)`` — retarget contract version 1, unchanged: a
  ``flex(channel) -> float`` callable in, six stroke fractions 0..10000 (the
  DH116S wire unit, :data:`POS_MAX`) out.
* ``mapper.joint_targets(pose)`` — version 2
  (:mod:`manipulation_kit.hands.joints`): a
  :class:`~manipulation_kit.gloves.HandPose` in, six ``float | None`` out in
  ``d1-firmwared``'s end-effector descriptor order and unit (:data:`JOINTS`:
  ``fraction`` 0.0 .. 1.0, i.e. version 1's value / 10000). An axis is
  ``None`` when any channel it reads with a non-zero weight has no reading;
  the consumer holds that axis. :meth:`Retargeter.required_channels` lists
  those channels for the current config.

Both paths apply the same weights, EMA, ``invert`` and ``out_lo``/``out_hi``
clamps, and agree to within version 1's integer rounding when every channel
has a reading.

The thumb's second lateral channel
----------------------------------
:attr:`RetargetConfig.thumb_swing_blend_channel` names the channel the swing
blends against the yaw. It defaults to ``thumb_cm_roll``, the name this map
always read. Under the glove vocabulary
(:mod:`manipulation_kit.gloves.channels`) that name is an AXIAL ROLL, which
the UDCAP glove measures; the LitchiBot glove's thumb ABDUCTION, which older
glove adapters used to answer under ``thumb_cm_roll``, is now declared as
``thumb_cm_swing``. With the default blend of 0.0 neither is read. A blend
tuned on one glove is a different quantity on the other: set the channel
explicitly when tuning it.

All weights live in :class:`RetargetConfig`; per-axis ``invert`` and
output ``lo/hi`` clamps allow flipping/limiting axes during hardware
bring-up without code changes.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from .axes import AXIS_NAMES, JOINTS, POS_MAX  # noqa: F401  (re-exported)

_FINGERS = ("index", "middle", "ring", "pinky")

#: Anatomical input channels the ``flex`` callable must answer for.
CHANNELS = (
    "thumb_cm_yaw", "thumb_cm_roll", "thumb_cm_pitch", "thumb_mp_pitch",
) + tuple(f"{f}_{j}_pitch" for f in _FINGERS for j in ("mp", "pip", "dip"))


@dataclass
class RetargetConfig:
    """Tunable weights — the thumb entries in particular need on-hand tuning."""

    # finger flexion = w_mp*MP + w_pip*PIP + w_dip*DIP (normalized below)
    w_mp: float = 0.5
    w_pip: float = 0.5
    w_dip: float = 0.0
    # thumb flexion = w_cm_pitch*CM_pitch + w_mp_pitch*MP_pitch
    thumb_w_cm_pitch: float = 0.5
    thumb_w_mp_pitch: float = 0.5
    # thumb swing = (1-roll_blend)*CM_yaw + roll_blend*CM_roll
    # TUNE ON HARDWARE: 2 active thumb DoA vs 4 glove thumb angles.
    thumb_swing_roll_blend: float = 0.0
    # the channel the blend reads (see "The thumb's second lateral channel")
    thumb_swing_blend_channel: str = "thumb_cm_roll"
    # exponential smoothing on the 6 outputs (0 = off, 0.3 ≈ light smoothing
    # at 50 Hz). The One-Euro treatment lives on the wrist path; hand channels
    # are already vendor-filtered, keep this gentle.
    ema_alpha: float = 0.0
    # per-axis post-processing (AXIS_NAMES order)
    invert: tuple = (False,) * 6
    out_lo: tuple = (0,) * 6            # stroke-fraction clamp low
    out_hi: tuple = (POS_MAX,) * 6      # stroke-fraction clamp high


class Retargeter:
    """Stateful (EMA only) anatomical-flexion→DH116S mapper.

    Call per sample at ≤50 Hz with a ``flex(channel_name) -> float`` callable
    returning flexion in [0, 1] for each name in :data:`CHANNELS`."""

    def __init__(self, config: RetargetConfig | None = None):
        self.cfg = config if config is not None else RetargetConfig()
        self._ema: np.ndarray | None = None
        # the version-2 path keeps its own smoothing state: it may hold an
        # axis (NaN = not seeded yet), which the version-1 arithmetic cannot
        self._ema_v2: np.ndarray | None = None

    # ------------------------------------------------------------------ #
    def fractions(self, flex) -> np.ndarray:
        """One sample → 6 flexions in [0, 1] (AXIS_NAMES order)."""
        cfg = self.cfg

        # thumb swing: yaw (+ optional roll blend)
        b = float(np.clip(cfg.thumb_swing_roll_blend, 0.0, 1.0))
        swing = ((1.0 - b) * flex("thumb_cm_yaw")
                 + b * flex(cfg.thumb_swing_blend_channel))

        # thumb flexion: CM + MP pitch
        tw = cfg.thumb_w_cm_pitch + cfg.thumb_w_mp_pitch
        thumb = ((cfg.thumb_w_cm_pitch * flex("thumb_cm_pitch")
                  + cfg.thumb_w_mp_pitch * flex("thumb_mp_pitch")) / tw
                 if tw > 0 else 0.0)

        # finger flexions: MP + PIP (+ optional DIP)
        fw = cfg.w_mp + cfg.w_pip + cfg.w_dip
        fingers = []
        for f in _FINGERS:
            v = (cfg.w_mp * flex(f + "_mp_pitch")
                 + cfg.w_pip * flex(f + "_pip_pitch")
                 + cfg.w_dip * flex(f + "_dip_pitch")) / fw if fw > 0 else 0.0
            fingers.append(v)

        out = np.array([swing, thumb] + fingers)
        if cfg.ema_alpha > 0.0:
            if self._ema is None:
                self._ema = out.copy()
            else:
                a = cfg.ema_alpha
                self._ema = (1.0 - a) * out + a * self._ema
            out = self._ema
        return np.clip(out, 0.0, 1.0)

    def __call__(self, flex) -> np.ndarray:
        """One sample → 6 stroke fractions, int 0..10000."""
        frac = self.fractions(flex)
        cfg = self.cfg
        out = np.empty(6, dtype=int)
        for i in range(6):
            v = 1.0 - frac[i] if cfg.invert[i] else frac[i]
            out[i] = int(round(np.clip(v * POS_MAX, cfg.out_lo[i], cfg.out_hi[i])))
        return out

    def reset(self) -> None:
        self._ema = None
        self._ema_v2 = None

    # ------------------------------------------- retarget contract v2 -- #
    #: The joints :meth:`joint_targets` answers, in descriptor order.
    JOINTS = JOINTS

    def _terms(self) -> List[Tuple[Tuple[float, str], ...]]:
        """Per axis, the ``(weight, channel)`` terms of its weighted mean —
        the same arithmetic :meth:`fractions` does, written as data so the
        version-2 path can tell which channels an axis depends on."""
        cfg = self.cfg
        b = float(np.clip(cfg.thumb_swing_roll_blend, 0.0, 1.0))
        terms = [((1.0 - b, "thumb_cm_yaw"), (b, cfg.thumb_swing_blend_channel)),
                 ((cfg.thumb_w_cm_pitch, "thumb_cm_pitch"),
                  (cfg.thumb_w_mp_pitch, "thumb_mp_pitch"))]
        for f in _FINGERS:
            terms.append(((cfg.w_mp, f + "_mp_pitch"), (cfg.w_pip, f + "_pip_pitch"),
                          (cfg.w_dip, f + "_dip_pitch")))
        return terms

    def required_channels(self) -> Tuple[str, ...]:
        """Every channel read with a non-zero weight under the current config,
        in axis order, each once. Check them against the glove's declaration
        at bind time."""
        out: List[str] = []
        for axis in self._terms():
            for w, ch in axis:
                if w != 0.0 and ch not in out:
                    out.append(ch)
        return tuple(out)

    def joint_targets(self, pose) -> List[Optional[float]]:
        """One canonical hand pose → six ``fraction`` targets or ``None``.

        ``pose`` is a :class:`~manipulation_kit.gloves.HandPose` (anything with
        ``get(channel) -> float | None``). An axis with a missing input is
        ``None`` and leaves its EMA state untouched, so a sensor that drops
        out and returns does not drag the axis through a stale average."""
        cfg = self.cfg
        raw: List[Optional[float]] = []
        for axis in self._terms():
            total = sum(w for w, _ in axis)
            value = 0.0
            for w, ch in axis:
                if w == 0.0:
                    continue
                v = pose.get(ch)
                if v is None:
                    value = None
                    break
                value += w * v
            raw.append(None if value is None else (value / total if total > 0 else 0.0))

        if cfg.ema_alpha > 0.0:
            if self._ema_v2 is None:
                # a held axis has nothing to seed with yet: NaN until it does
                self._ema_v2 = np.array([np.nan if v is None else v for v in raw])
            else:
                a = cfg.ema_alpha
                for i, v in enumerate(raw):
                    if v is None:
                        continue
                    prev = self._ema_v2[i]
                    self._ema_v2[i] = v if np.isnan(prev) else (1.0 - a) * v + a * prev
            raw = [None if v is None else float(self._ema_v2[i])
                   for i, v in enumerate(raw)]

        out: List[Optional[float]] = []
        for i, v in enumerate(raw):
            if v is None:
                out.append(None)
                continue
            v = float(np.clip(v, 0.0, 1.0))
            if cfg.invert[i]:
                v = 1.0 - v
            v = float(np.clip(v, cfg.out_lo[i] / POS_MAX, cfg.out_hi[i] / POS_MAX))
            out.append(JOINTS[i].clamp(v))
        return out


def build_retarget(config: RetargetConfig | None = None) -> Retargeter:
    """Standard cross-hand retarget constructor (see ``manipulation_kit.hands.get_retarget``)."""
    return Retargeter(config=config)
