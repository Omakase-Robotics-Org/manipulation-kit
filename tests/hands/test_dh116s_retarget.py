"""Retarget map tests — pure numpy, no glove, no mujoco, no hardware.

The input contract is a ``flex(channel_name) -> [0, 1]`` callable; tests
build synthetic ones from dicts."""

import numpy as np
import pytest

from manipulation_kit.hands.leadshine.dh116s.retarget import (
    AXIS_NAMES,
    CHANNELS,
    POS_MAX,
    RetargetConfig,
    Retargeter,
    build_retarget,
)


def _flex(values=None, default=0.0):
    """flex callable from a {channel: flexion} dict (missing → default)."""
    values = values or {}

    def flex(name):
        assert name in CHANNELS, f"retarget asked for unknown channel {name!r}"
        return values.get(name, default)

    return flex


def test_axis_names_order():
    assert AXIS_NAMES == ("thumb_swing", "thumb_flex", "index_flex",
                          "middle_flex", "ring_flex", "pinky_flex")


def test_full_open_all_zero():
    out = Retargeter()(_flex(default=0.0))
    assert out.tolist() == [0] * 6


def test_full_close_all_pos_max():
    out = Retargeter()(_flex(default=1.0))
    assert out.tolist() == [POS_MAX] * 6


def test_axis_ordering_channel_to_axis():
    """Each finger's MP+PIP drive exactly its own axis; thumb channels theirs."""
    r = Retargeter()
    for finger, axis in (("index", 2), ("middle", 3), ("ring", 4), ("pinky", 5)):
        out = r(_flex({f"{finger}_mp_pitch": 1.0, f"{finger}_pip_pitch": 1.0}))
        expect = [0] * 6
        expect[axis] = POS_MAX
        assert out.tolist() == expect, f"{finger} should drive axis {axis} only"
    # thumb swing (axis 0) ← CM yaw only by default (roll blend 0)
    assert r(_flex({"thumb_cm_yaw": 1.0})).tolist() == [POS_MAX, 0, 0, 0, 0, 0]
    assert r(_flex({"thumb_cm_roll": 1.0})).tolist() == [0] * 6
    # thumb flexion (axis 1) ← CM pitch + MP pitch
    out = r(_flex({"thumb_cm_pitch": 1.0, "thumb_mp_pitch": 1.0}))
    assert out.tolist() == [0, POS_MAX, 0, 0, 0, 0]


def test_finger_weights_average_mp_pip():
    # default w_mp = w_pip = 0.5, w_dip = 0 → MP-only closure reads 50 %
    out = Retargeter()(_flex({"index_mp_pitch": 1.0}))
    assert out[2] == POS_MAX // 2
    # DIP is ignored by default (coupled to PIP on the glove — noise)
    out = Retargeter()(_flex({"index_dip_pitch": 1.0}))
    assert out[2] == 0


def test_thumb_swing_roll_blend():
    cfg = RetargetConfig(thumb_swing_roll_blend=0.25)
    out = Retargeter(cfg)(_flex({"thumb_cm_roll": 1.0}))
    assert out[0] == int(round(0.25 * POS_MAX))


def test_invert_flips_axis():
    cfg = RetargetConfig(invert=(True,) + (False,) * 5)
    r = Retargeter(cfg)
    out = r(_flex(default=0.0))           # full open
    assert out.tolist() == [POS_MAX, 0, 0, 0, 0, 0]
    out = r(_flex(default=1.0))           # full close
    assert out.tolist() == [0] + [POS_MAX] * 5


def test_out_lo_hi_clamp():
    cfg = RetargetConfig(out_lo=(1000,) * 6, out_hi=(9000,) * 6)
    r = Retargeter(cfg)
    assert r(_flex(default=0.0)).tolist() == [1000] * 6
    assert r(_flex(default=1.0)).tolist() == [9000] * 6


def test_ema_smoothing_and_reset():
    a = 0.5
    r = Retargeter(RetargetConfig(ema_alpha=a))
    # first sample seeds the EMA (no lag)
    assert r(_flex(default=1.0)).tolist() == [POS_MAX] * 6
    # step to open: ema' = (1-a)*new + a*old = 0.5 → half stroke
    out = r(_flex(default=0.0))
    assert out.tolist() == [int(round((1 - a) * POS_MAX))] * 6
    # converges toward the held input
    out = r(_flex(default=0.0))
    assert np.all(out < (1 - a) * POS_MAX)
    # reset drops the EMA state → next sample is instantaneous again
    r.reset()
    assert r(_flex(default=1.0)).tolist() == [POS_MAX] * 6


def test_ema_off_is_stateless():
    r = Retargeter()  # ema_alpha = 0
    r(_flex(default=1.0))
    assert r(_flex(default=0.0)).tolist() == [0] * 6


def test_build_retarget_factory():
    r = build_retarget()
    assert isinstance(r, Retargeter)
    cfg = RetargetConfig(ema_alpha=0.3)
    assert build_retarget(cfg).cfg is cfg


def test_output_dtype_and_range():
    out = Retargeter()(_flex(default=0.5))
    assert out.shape == (6,)
    assert out.dtype.kind == "i"
    assert np.all((out >= 0) & (out <= POS_MAX))
