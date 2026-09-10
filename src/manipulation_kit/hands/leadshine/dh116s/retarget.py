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

Everything returns stroke fractions 0..10000 (the DH116S wire unit,
:data:`POS_MAX` — what ``Dh116sDriver.set_positions`` takes).
All weights live in :class:`RetargetConfig`; per-axis ``invert`` and
output ``lo/hi`` clamps allow flipping/limiting axes during hardware
bring-up without code changes.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .axes import AXIS_NAMES, POS_MAX  # noqa: F401  (re-exported)

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

    # ------------------------------------------------------------------ #
    def fractions(self, flex) -> np.ndarray:
        """One sample → 6 flexions in [0, 1] (AXIS_NAMES order)."""
        cfg = self.cfg

        # thumb swing: yaw (+ optional roll blend)
        b = float(np.clip(cfg.thumb_swing_roll_blend, 0.0, 1.0))
        swing = (1.0 - b) * flex("thumb_cm_yaw") + b * flex("thumb_cm_roll")

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


def build_retarget(config: RetargetConfig | None = None) -> Retargeter:
    """Standard cross-hand retarget constructor (see ``manipulation_kit.hands.get_retarget``)."""
    return Retargeter(config=config)
