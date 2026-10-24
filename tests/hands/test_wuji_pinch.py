"""The Wuji Hand 2's pinch-aware thumb: the table, the blend, the step bound, the binding."""
import hashlib
import json
import math
import os
import random
import textwrap
from dataclasses import replace
from pathlib import Path

import pytest

from manipulation_kit.binding import load_binding
from manipulation_kit.binding.retarget_config import build_retarget_config
from manipulation_kit.gloves import HandAngles
from manipulation_kit.hands.wuji.wujihand2 import AXIS_NAMES, JOINT_LIMITS_RAD
from manipulation_kit.hands.wuji.wujihand2.pinch import (
    BLEND_LEAD,
    INDEX_AXES,
    SCHEMA,
    TABLE_FILE,
    THUMB_AXES,
    PinchTable,
    opposition,
    pinch_table,
)
from manipulation_kit.hands.wuji.wujihand2.retarget import RetargetConfig, Retargeter
from manipulation_kit.hands.wuji.wujihand2.udcap import udcap_config

ROOT = Path(__file__).resolve().parents[2]
PINCHES = ROOT / "tests" / "data" / "gloves" / "udcap_right_pinch.jsonl"
EXAMPLE = ROOT / "docs" / "binding-examples" / "udcap-wuji-right.toml"
FINGERS = ("index", "middle", "ring", "pinky")
THUMB = [AXIS_NAMES.index(n) for n in THUMB_AXES]
INDEX = [AXIS_NAMES.index(n) for n in INDEX_AXES]


def _angles(**kw):
    ch = {}
    for f in FINGERS:
        ch.update({f"{f}_mp_pitch": 0.0, f"{f}_pip_pitch": 0.0, f"{f}_dip_pitch": 0.0})
    ch.update({"thumb_cm_pitch": 0.0, "thumb_cm_yaw": 0.0, "thumb_mp_pitch": 0.0,
               "thumb_ip_pitch": 0.0, "index_mp_swing": 0.0, "ring_mp_swing": 0.0,
               "pinky_mp_swing": 0.0})
    ch.update(kw)
    return HandAngles("right", ch)


def _thumb(out):
    return [out[i] for i in THUMB]


# --- the table ---------------------------------------------------------------

def test_the_shipped_table_is_a_full_grid_inside_the_thumbs_limits():
    t = pinch_table()
    doc = json.loads(TABLE_FILE.read_text())
    assert doc["schema"] == SCHEMA and doc["thumb_axes"] == list(THUMB_AXES)
    assert [len(a) for a in t.axes] == [13, 14, 12]
    assert [(a[0], a[-1]) for a in t.axes] == [(0.0, 1.2), (0.0, 1.3), (0.0, 1.1)]
    for plane in t.thumb:
        for row in plane:
            for q in row:
                for v, (lo, hi) in zip(q, JOINT_LIMITS_RAD[:4]):
                    assert lo < v < hi
    # 64 hex digits of the vendor URDF the table was solved from.
    assert len(doc["source"]["urdf_sha256"]) == 64


def test_neighbouring_grid_points_have_neighbouring_thumbs():
    # One grid step of the index (0.1 rad) moves no thumb joint by more than
    # 0.27 rad: the posture term keeps one solution branch across the grid.
    t = pinch_table()
    worst = 0.0
    n1, n3, n4 = (len(a) for a in t.axes)
    for a in range(n1):
        for b in range(n3):
            for c in range(n4):
                q = t.thumb[a][b][c]
                for da, db, dc in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
                    if a + da < n1 and b + db < n3 and c + dc < n4:
                        r = t.thumb[a + da][b + db][c + dc]
                        worst = max(worst, max(abs(x - y) for x, y in zip(q, r)))
    assert worst < 0.27


def test_the_table_reads_its_grid_points_and_interpolates_between_them():
    t = pinch_table()
    g1, g3, g4 = t.axes
    assert t.thumb_for(g1[3], g3[5], g4[2]) == pytest.approx(t.thumb[3][5][2], abs=1e-12)
    mid = t.thumb_for((g1[3] + g1[4]) / 2, g3[5], g4[2])
    want = [(x + y) / 2 for x, y in zip(t.thumb[3][5][2], t.thumb[4][5][2])]
    assert mid == pytest.approx(want, abs=1e-12)
    # Past the grid it reads the edge.
    assert t.thumb_for(9.0, -1.0, 9.0) == pytest.approx(t.thumb[-1][0][-1], abs=1e-12)


def test_the_table_is_continuous_along_any_index_curl():
    t = pinch_table()
    rng = random.Random(7)
    for _ in range(50):
        p = [rng.uniform(0, 1.2), rng.uniform(0, 1.3), rng.uniform(0, 1.1)]
        d = [rng.uniform(-1, 1) for _ in range(3)]
        prev = t.thumb_for(*p)
        for k in range(1, 101):
            q = t.thumb_for(*(x + 1e-3 * k * dx for x, dx in zip(p, d)))
            # 1.7 mrad of index motion: at most 3.5 rad/rad of thumb motion.
            assert max(abs(a - b) for a, b in zip(q, prev)) < 0.006
            prev = q


def test_a_malformed_table_is_refused():
    doc = json.loads(TABLE_FILE.read_text())
    with pytest.raises(ValueError, match="schema"):
        PinchTable({**doc, "schema": "x"})
    with pytest.raises(ValueError, match="thumb must be"):
        PinchTable({**doc, "thumb": doc["thumb"][:-1]})
    grid = dict(doc["grid"], index_S1=list(reversed(doc["grid"]["index_S1"])))
    with pytest.raises(ValueError, match="must rise"):
        PinchTable({**doc, "grid": grid})


def test_opposition_ramps_between_the_two_angles():
    assert opposition(0.10, 0.15, 0.30) == 0.0
    assert opposition(0.225, 0.15, 0.30) == pytest.approx(0.5)
    assert opposition(0.9, 0.15, 0.30) == 1.0
    assert opposition(None, 0.15, 0.30) is None
    assert opposition(float("nan"), 0.15, 0.30) is None


# --- the config --------------------------------------------------------------

def test_the_default_config_has_no_pinch_and_udcap_turns_it_on():
    assert not RetargetConfig().thumb_pinch
    cfg = udcap_config()
    assert cfg.thumb_pinch and cfg.pinch_channel == "thumb_cm_pitch"
    assert (cfg.pinch_open_rad, cfg.pinch_closed_rad, cfg.pinch_max_step_rad) == (0.15, 0.30, 0.1)


def test_a_pinch_without_the_absolute_thumb_is_refused():
    with pytest.raises(ValueError, match="thumb_pinch needs"):
        RetargetConfig(thumb_pinch=True)
    with pytest.raises(ValueError, match="thumb_pinch needs"):
        replace(udcap_config(), mapping="range")
    with pytest.raises(ValueError, match="below"):
        udcap_config(pinch_open_rad=0.3, pinch_closed_rad=0.3)
    with pytest.raises(ValueError, match=">= 0"):
        udcap_config(pinch_max_step_rad=-0.1)


def test_the_pinch_channel_is_required():
    assert "thumb_cm_pitch" in Retargeter(udcap_config()).required_channels()
    r = Retargeter(udcap_config(pinch_channel="thumb_cm_roll"))
    assert "thumb_cm_roll" in r.required_channels()
    held = udcap_config(hold=(True,) * 4 + udcap_config().hold[4:], pinch_channel="thumb_cm_roll")
    assert "thumb_cm_roll" not in Retargeter(held).required_channels()


# --- the blend ---------------------------------------------------------------

def test_below_the_open_angle_the_thumb_is_the_one_to_one_thumb():
    a = _angles(thumb_cm_pitch=0.12, thumb_cm_yaw=0.3, thumb_mp_pitch=0.8,
                thumb_ip_pitch=0.2, index_mp_pitch=0.7, index_pip_pitch=0.6)
    pinch = Retargeter(udcap_config()).joint_targets(None, angles=a)
    plain = Retargeter(udcap_config(thumb_pinch=False)).joint_targets(None, angles=a)
    assert pinch == plain
    assert _thumb(plain) == [0.12, 0.3, 0.8, 0.2]


def test_the_fingers_never_change():
    rng = random.Random(3)
    pinch = Retargeter(udcap_config())
    plain = Retargeter(udcap_config(thumb_pinch=False))
    for _ in range(200):
        a = _angles(**{f"{f}_{j}": rng.uniform(0, 1.6) for f in FINGERS
                       for j in ("mp_pitch", "pip_pitch", "dip_pitch")},
                    thumb_cm_pitch=rng.uniform(0, 1.4), thumb_cm_yaw=rng.uniform(0, 0.6))
        x = pinch.joint_targets(None, angles=a)
        y = plain.joint_targets(None, angles=a)
        assert x[4:] == y[4:]


def test_fully_opposed_the_thumb_is_the_tables_thumb_for_the_commanded_index():
    r = Retargeter(udcap_config(pinch_max_step_rad=0.0))
    out = r.joint_targets(None, angles=_angles(
        thumb_cm_pitch=0.42, thumb_cm_yaw=0.61, thumb_mp_pitch=0.57, thumb_ip_pitch=0.70,
        index_mp_pitch=0.72, index_pip_pitch=0.58, index_dip_pitch=0.46))
    want = pinch_table().thumb_for(0.72, 0.58, 0.46)
    assert _thumb(out) == pytest.approx(want, abs=1e-12)
    # The Wuji pinches with its base flexed and abducted towards the palm.
    assert out[THUMB[0]] > 1.0 and out[THUMB[1]] < 0.0


def test_the_index_the_thumb_aims_at_is_the_commanded_one():
    # zero_rad moves the commanded index; the thumb follows the command.
    zero = [0.0] * 20
    zero[INDEX[1]] = 0.2
    r = Retargeter(udcap_config(pinch_max_step_rad=0.0, zero_rad=tuple(zero)))
    out = r.joint_targets(None, angles=_angles(
        thumb_cm_pitch=1.0, index_mp_pitch=0.7, index_pip_pitch=0.8, index_dip_pitch=0.5))
    assert out[INDEX[1]] == pytest.approx(0.6)
    assert _thumb(out) == pytest.approx(pinch_table().thumb_for(0.7, 0.6, 0.5), abs=1e-12)


def test_half_opposed_the_thumb_is_between_and_s2_leads():
    one = [0.2, 0.5, 0.6, 0.4]
    a = _angles(thumb_cm_pitch=0.2, thumb_cm_yaw=0.5, thumb_mp_pitch=0.6, thumb_ip_pitch=0.4,
                index_mp_pitch=0.7, index_pip_pitch=0.6, index_dip_pitch=0.48)
    # thumb_cm_pitch 0.2 is one third of the way from 0.15 to 0.30; thumb_S1
    # reads that angle, so its 1:1 value is 0.2.
    out = Retargeter(udcap_config(pinch_max_step_rad=0.0)).joint_targets(None, angles=a)
    tab = pinch_table().thumb_for(0.7, 0.6, 0.48)
    for n, lead in enumerate(BLEND_LEAD):
        o = min(1.0, lead / 3)
        assert out[THUMB[n]] == pytest.approx((1 - o) * one[n] + o * tab[n], abs=1e-9)


def test_each_thumb_joint_moves_at_most_the_step_per_call_and_reset_forgets():
    r = Retargeter(udcap_config())
    open_ = _angles(index_mp_pitch=0.7, index_pip_pitch=0.6, index_dip_pitch=0.48)
    pinch = _angles(thumb_cm_pitch=0.5, index_mp_pitch=0.7, index_pip_pitch=0.6,
                    index_dip_pitch=0.48)
    prev = _thumb(r.joint_targets(None, angles=open_))
    target = pinch_table().thumb_for(0.7, 0.6, 0.48)
    for _ in range(30):
        cur = _thumb(r.joint_targets(None, angles=pinch))
        assert max(abs(a - b) for a, b in zip(cur, prev)) <= 0.1 + 1e-12
        prev = cur
    assert cur == pytest.approx(target, abs=1e-12)
    r.joint_targets(None, angles=open_)
    r.reset()
    assert _thumb(r.joint_targets(None, angles=pinch)) == pytest.approx(target, abs=1e-12)


def test_no_opposition_reading_holds_the_thumb_only():
    a = _angles(index_mp_pitch=0.7)
    del a.radians["thumb_cm_pitch"]
    out = Retargeter(udcap_config()).joint_targets(None, angles=a)
    assert _thumb(out) == [None] * 4
    assert out[INDEX[0]] == 0.7


def test_a_recorded_pinch_session_pinches_and_stays_bounded():
    # 299 frames (every fifth) of 30 s of an operator pinching thumb and index
    # with a UDCAP glove, then opening the hand, on 2026-10-24.
    rows = [json.loads(line) for line in PINCHES.read_text().splitlines()]
    assert len(rows) == 299
    r = Retargeter(udcap_config())
    plain = Retargeter(udcap_config(thumb_pinch=False))
    prev = None
    pinching = []
    for row in rows:
        a = HandAngles("right", row["radians"])
        out = r.joint_targets(None, angles=a)
        assert out[4:] == plain.joint_targets(None, angles=a)[4:]
        th = _thumb(out)
        for v, (lo, hi) in zip(th, JOINT_LIMITS_RAD[:4]):
            assert lo <= v <= hi
        if prev is not None:
            assert max(abs(x - y) for x, y in zip(th, prev)) <= 0.1 + 1e-12
        prev = th
        rad = row["radians"]
        if rad["thumb_cm_pitch"] >= 0.30 and rad["index_mp_pitch"] + rad["index_pip_pitch"] > 0.9:
            pinching.append(th)
    # Pinching, the thumb has left the positive S2 the glove's yaw gives it
    # (0.611 rad on every one of these frames) for the Wuji's pinch shape.
    assert len(pinching) > 50
    s1 = sorted(t[0] for t in pinching)
    s2 = sorted(t[1] for t in pinching)
    assert s1[len(s1) // 2] > 0.9 and s2[len(s2) // 2] < 0.0


# --- the binding -------------------------------------------------------------

def test_the_pinch_fields_bind_from_a_binding_file():
    cfg, problems = build_retarget_config("wuji/wujihand2", {
        "mapping": "absolute", "thumb_absolute": True, "thumb_pinch": True,
        "pinch_open_rad": 0.1, "pinch_closed_rad": 0.4, "pinch_max_step_rad": 0.05})
    assert problems == []
    assert (cfg.pinch_open_rad, cfg.pinch_closed_rad, cfg.pinch_max_step_rad) == (0.1, 0.4, 0.05)
    _, problems = build_retarget_config("wuji/wujihand2", {
        "mapping": "absolute", "thumb_absolute": True, "thumb_pinch": True,
        "pinch_channel": "thumb_opposition"})
    assert problems and "not a glove channel" in problems[0]


def test_the_example_binding_carries_the_udcap_pinch():
    built = load_binding(EXAMPLE).binds[0].build_retarget().cfg
    want = udcap_config()
    for f in ("thumb_pinch", "pinch_channel", "pinch_open_rad", "pinch_closed_rad",
              "pinch_max_step_rad"):
        assert getattr(built, f) == getattr(want, f), f


# --- regenerating the table (opt-in: the vendor's URDF) ------------------------

URDF_DIR = os.environ.get("MKIT_WUJI_URDF_DIR")


@pytest.mark.skipif(not URDF_DIR, reason="MKIT_WUJI_URDF_DIR not set (the vendor's wuji_v2/urdf)")
def test_the_table_is_what_the_solver_gives_for_the_vendor_urdf():
    import numpy as np

    from manipulation_kit.hands.wuji.wujihand2 import pinch_build as pb

    urdf = Path(URDF_DIR) / "right.urdf"
    doc = json.loads(TABLE_FILE.read_text())
    assert hashlib.sha256(urdf.read_bytes()).hexdigest() == doc["source"]["urdf_sha256"]
    hand = pb.Hand(urdf)
    t = pinch_table()
    g1, g3, g4 = t.axes
    for a, b, c in ((0, 0, 1), (7, 6, 5), (7, 6, 6), (10, 9, 8), (12, 13, 11)):
        start = t.thumb[a][b][c - 1]
        q, _ = pb.solve_point(hand, (g1[a], 0.0, g3[b], g4[c]), start)
        assert np.allclose(q, t.thumb[a][b][c], atol=2e-4), (a, b, c, q, t.thumb[a][b][c])
    # Where the recorded pinches put the index (median, and a deeper and a
    # shallower one), the table's thumb tip is 14.2, 7.6 and 18.2 mm from the
    # index tip: the target is 8 mm or more towards the index pad.
    for iq, mm in (((0.72, 0.58, 0.46), 14.2), ((0.9, 0.8, 0.64), 7.6),
                   ((0.6, 0.6, 0.48), 18.2)):
        tip = hand.tip("index", (iq[0], 0.0, iq[1], iq[2]))
        th = hand.tip("thumb", t.thumb_for(*iq))
        assert 1000 * np.linalg.norm(th - tip) == pytest.approx(mm, abs=0.1)
