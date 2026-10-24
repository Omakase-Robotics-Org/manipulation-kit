"""The Wuji Hand 2's pinch-aware thumb: the tables, the blend, the step bound, the
per-operator thumb model, the binding."""
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
    MODEL_CLASSES,
    SCHEMA,
    TABLE_FILE,
    TARGET_FINGERS,
    THUMB_AXES,
    PinchTable,
    PinchTables,
    ThumbModel,
    finger_axes,
    opposition,
    pinch_table,
    pinch_tables,
)
from manipulation_kit.hands.wuji.wujihand2.retarget import RetargetConfig, Retargeter
from manipulation_kit.hands.wuji.wujihand2.udcap import udcap_config

ROOT = Path(__file__).resolve().parents[2]
PINCHES = ROOT / "tests" / "data" / "gloves" / "udcap_right_pinch.jsonl"
SESSION = ROOT / "tests" / "data" / "gloves" / "udcap_right_thumb_session.json"
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

def test_the_shipped_tables_are_full_grids_inside_the_thumbs_limits():
    doc = json.loads(TABLE_FILE.read_text())
    assert doc["schema"] == SCHEMA and doc["thumb_axes"] == list(THUMB_AXES)
    assert list(doc["fingers"]) == list(TARGET_FINGERS)
    for f in TARGET_FINGERS:
        t = pinch_table(f)
        assert doc["fingers"][f]["axes"] == list(finger_axes(f))
        assert [len(a) for a in t.axes] == [16, 19, 15]
        assert [(a[0], a[-1]) for a in t.axes] == [(0.0, 1.5), (0.0, 1.8), (0.0, 1.4)]
        for plane in t.thumb:
            for row in plane:
                for q in row:
                    for v, (lo, hi) in zip(q, JOINT_LIMITS_RAD[:4]):
                        assert lo < v < hi
        # The pad gap: never closer than the 8 mm target, never unset.
        gaps = [g for plane in t.pad_gap_mm for row in plane for g in row]
        assert len(gaps) == 16 * 19 * 15 and min(gaps) >= 7.9
    # 64 hex digits of the vendor URDF the tables were solved from.
    assert len(doc["source"]["urdf_sha256"]) == 64


@pytest.mark.parametrize("finger", TARGET_FINGERS)
def test_neighbouring_in_reach_grid_points_have_neighbouring_thumbs(finger):
    # One grid step of the finger (0.1 rad) between two points the thumb
    # reaches (pad gap under 100 mm) moves no thumb joint by more than 0.5
    # rad: where the solve changes branch, the deeper-curled point is marked
    # out of reach (pinch_build.JUMP_MAX_RAD), so the fade by gap never swings
    # the thumb across.
    t = pinch_table(finger)
    got = 0.0
    n1, n3, n4 = (len(a) for a in t.axes)
    for a in range(n1):
        for b in range(n3):
            for c in range(n4):
                if t.pad_gap_mm[a][b][c] >= 100:
                    continue
                q = t.thumb[a][b][c]
                for da, db, dc in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
                    x, y, z = a + da, b + db, c + dc
                    if x < n1 and y < n3 and z < n4 and t.pad_gap_mm[x][y][z] < 100:
                        r = t.thumb[x][y][z]
                        got = max(got, max(abs(u - v) for u, v in zip(q, r)))
    assert got <= 0.5


def test_the_table_reads_its_grid_points_and_interpolates_between_them():
    for f in TARGET_FINGERS:
        t = pinch_table(f)
        g1, g3, g4 = t.axes
        assert t.thumb_for(g1[3], g3[5], g4[2]) == pytest.approx(t.thumb[3][5][2], abs=1e-12)
        mid = t.thumb_for((g1[3] + g1[4]) / 2, g3[5], g4[2])
        want = [(x + y) / 2 for x, y in zip(t.thumb[3][5][2], t.thumb[4][5][2])]
        assert mid == pytest.approx(want, abs=1e-12)
        assert t.gap_for(g1[3], g3[5], g4[2]) == pytest.approx(t.pad_gap_mm[3][5][2], abs=1e-12)
        # Past the grid it reads the edge.
        assert t.thumb_for(9.0, -1.0, 9.0) == pytest.approx(t.thumb[-1][0][-1], abs=1e-12)


def test_the_table_is_continuous_along_any_finger_curl():
    rng = random.Random(7)
    for f in TARGET_FINGERS:
        t = pinch_table(f)
        for _ in range(50):
            p = [rng.uniform(0, 1.0), rng.uniform(0, 1.2), rng.uniform(0, 1.0)]
            d = [rng.uniform(-1, 1) for _ in range(3)]
            prev = t.thumb_for(*p)
            for k in range(1, 101):
                q = t.thumb_for(*(x + 1e-3 * k * dx for x, dx in zip(p, d)))
                # 1.7 mrad of finger motion: at most 3.5 rad/rad of thumb motion.
                assert max(abs(a - b) for a, b in zip(q, prev)) < 0.006
                prev = q


def test_the_thumb_cannot_reach_a_straight_little_finger():
    # Why finger_shift exists: the Wuji's thumb tip stays 40 mm from the pad
    # of a little finger curled as an operator curls it to pinch (S1 0.31,
    # S3 0.54, S4 0.43), and meets it once the finger curls further.
    t = pinch_table("pinky")
    assert t.gap_for(0.31, 0.54, 0.43) > 35.0
    assert t.gap_for(0.8, 1.0, 0.5) < 10.0


def test_a_malformed_table_is_refused():
    doc = json.loads(TABLE_FILE.read_text())
    part = doc["fingers"]["index"]
    with pytest.raises(ValueError, match="schema"):
        PinchTables({**doc, "schema": "x"})
    with pytest.raises(ValueError, match="no table for"):
        PinchTables({**doc, "fingers": {"index": part}})
    with pytest.raises(ValueError, match="thumb must be"):
        PinchTable({**part, "thumb": part["thumb"][:-1]})
    grid = dict(part["grid"], S1=list(reversed(part["grid"]["S1"])))
    with pytest.raises(ValueError, match="must rise"):
        PinchTable({**part, "grid": grid})


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
        q, _, _, _ = pb.solve_point(hand, "index", (g1[a], 0.0, g3[b], g4[c]), start)
        assert np.allclose(q, t.thumb[a][b][c], atol=2e-4), (a, b, c, q, t.thumb[a][b][c])
    for f in ("middle", "ring", "pinky"):
        tf = pinch_table(f)
        a, b, c = 7, 9, 6
        q, _, _, _ = pb.solve_point(hand, f, (g1[a], 0.0, g3[b], g4[c]), tf.thumb[a][b][c - 1])
        assert np.allclose(q, tf.thumb[a][b][c], atol=2e-4), (f, q, tf.thumb[a][b][c])
    # Where the recorded pinches put the index (median, and a deeper and a
    # shallower one), the table's thumb tip is 14.2, 7.6 and 18.2 mm from the
    # index tip: the target is 8 mm or more towards the index pad.
    for iq, mm in (((0.72, 0.58, 0.46), 14.2), ((0.9, 0.8, 0.64), 7.6),
                   ((0.6, 0.6, 0.48), 18.2)):
        tip = hand.tip("index", (iq[0], 0.0, iq[1], iq[2]))
        th = hand.tip("thumb", t.thumb_for(*iq))
        assert 1000 * np.linalg.norm(th - tip) == pytest.approx(mm, abs=0.1)


# --- the per-operator thumb model ----------------------------------------------

def _session():
    return json.loads(SESSION.read_text())


def _model(**kw):
    problems = []
    m = ThumbModel.from_table({**_session()["model"], **kw}, "thumb_model", problems)
    assert problems == [] and m is not None
    return m


def _pose(name, **kw):
    doc = _session()
    ch = dict(zip(doc["channels"], doc["poses"][name]))
    ch.update(kw)
    return HandAngles("right", ch)


def _toy(**kw):
    """A two-channel model: index weight from index_mp_pitch, middle from middle_mp_pitch."""
    base = dict(channels=("index_mp_pitch", "middle_mp_pitch"), scale=(1.0, 1.0),
                weights=((0.0, 0.0), (20.0, 0.0), (0.0, 20.0), (0.0, 0.0), (0.0, 0.0)),
                bias=(8.0, 0.0, 0.0, -50.0, -50.0))
    base.update(kw)
    return ThumbModel(**base)


def test_the_model_gates_and_normalises_its_weights():
    m = _toy()
    assert list(MODEL_CLASSES) == ["none", "index", "middle", "ring", "pinky"]
    # index 0.2 rad: logit 4 against none's 8 -> p 0.018, below gate_lo: no weight.
    assert m.finger_weights(_angles(index_mp_pitch=0.2)) == {f: 0.0 for f in TARGET_FINGERS}
    # index 0.7: logit 14 against 8 -> p 0.9975, above gate_hi: all of it.
    w = m.finger_weights(_angles(index_mp_pitch=0.7))
    assert w["index"] == pytest.approx(1.0) and w["middle"] == 0.0
    # index and middle both 0.7: half each in probability, 0.5 -> (0.5-0.3)/0.4 = 0.5 each.
    w = m.finger_weights(_angles(index_mp_pitch=0.7, middle_mp_pitch=0.7))
    assert w["index"] == pytest.approx(0.5, abs=0.01) and w["middle"] == pytest.approx(0.5, abs=0.01)
    a = _angles()
    del a.radians["index_mp_pitch"]
    assert m.finger_weights(a) is None


def test_a_malformed_model_is_refused():
    with pytest.raises(ValueError, match="weights must be 5 rows"):
        _toy(weights=((1.0, 1.0),))
    with pytest.raises(ValueError, match="scale"):
        _toy(scale=(1.0, 0.0))
    with pytest.raises(ValueError, match="gate_lo"):
        _toy(gate_lo=0.8, gate_hi=0.7)
    with pytest.raises(ValueError, match="finger_shift"):
        _toy(finger_shift=((0.0, 0.0, 0.0),))
    with pytest.raises(ValueError, match="schema"):
        _toy(schema="x")
    problems = []
    assert ThumbModel.from_table({**_session()["model"], "channels": ["thumb_opposition"] * 15,
                                  "colour": 1}, "m", problems) is None
    assert any("not a glove channel" in p for p in problems)
    assert any("unknown key 'colour'" in p for p in problems)
    problems = []
    assert ThumbModel.from_table({**_session()["model"], "classes": ["none", "index"]}, "m",
                                 problems) is None
    assert "classes" in problems[0]
    with pytest.raises(ValueError, match="thumb_model needs thumb_pinch"):
        udcap_config(thumb_pinch=False, thumb_model=_toy())


def test_with_a_model_the_thumb_meets_the_finger_the_model_names():
    r = Retargeter(udcap_config(thumb_model=_toy(), pinch_max_step_rad=0.0))
    a = _angles(thumb_cm_pitch=0.3, thumb_cm_yaw=0.6, thumb_mp_pitch=0.5, thumb_ip_pitch=0.4,
                middle_mp_pitch=0.7, middle_pip_pitch=1.0, middle_dip_pitch=0.8)
    out = r.joint_targets(None, angles=a)
    want = pinch_table("middle").thumb_for(0.7, 1.0, 0.8)
    assert _thumb(out) == pytest.approx(want, abs=1e-6)
    # Nothing opposed: the 1:1 thumb, whatever thumb_cm_pitch reads.
    out = r.joint_targets(None, angles=_angles(thumb_cm_pitch=0.9, thumb_cm_yaw=0.6,
                                               thumb_mp_pitch=0.5, thumb_ip_pitch=0.4))
    assert _thumb(out) == pytest.approx([0.9, 0.6, 0.5, 0.4])


def test_two_fingers_blend_by_their_weights_and_s2_leads():
    r = Retargeter(udcap_config(thumb_model=_toy(), pinch_max_step_rad=0.0))
    a = _angles(thumb_cm_pitch=0.2, thumb_cm_yaw=0.5, thumb_mp_pitch=0.6, thumb_ip_pitch=0.4,
                index_mp_pitch=0.7, index_pip_pitch=0.6, index_dip_pitch=0.5,
                middle_mp_pitch=0.7, middle_pip_pitch=0.9, middle_dip_pitch=0.7)
    w = _toy().finger_weights(a)
    total = w["index"] + w["middle"]
    ti = pinch_table("index").thumb_for(0.7, 0.6, 0.5)
    tm = pinch_table("middle").thumb_for(0.7, 0.9, 0.7)
    out = r.joint_targets(None, angles=a)
    one = [0.2, 0.5, 0.6, 0.4]
    for n, lead in enumerate(BLEND_LEAD):
        on = min(1.0, lead * total)
        aim = (w["index"] * ti[n] + w["middle"] * tm[n]) / total
        assert out[THUMB[n]] == pytest.approx((1 - on) * one[n] + on * aim, abs=1e-9)


def test_the_model_reads_its_channels_and_a_missing_one_holds_the_thumb():
    r = Retargeter(udcap_config(thumb_model=_model()))
    req = r.required_channels()
    for ch in _model().channels:
        assert ch in req
    a = _pose("pinch_index")
    del a.radians["ring_dip_pitch"]
    out = r.joint_targets(None, angles=a)
    assert _thumb(out) == [None] * 4


def test_the_one_to_one_thumbs_s2_is_capped():
    m = _toy(thumb_s2_max=0.2)
    out = Retargeter(udcap_config(thumb_model=m)).joint_targets(None, angles=_angles(
        thumb_cm_pitch=0.5, thumb_cm_yaw=0.611, thumb_mp_pitch=0.5))
    assert out[THUMB[1]] == pytest.approx(0.2)
    assert out[THUMB[0]] == pytest.approx(0.5)


def test_finger_shift_moves_the_named_finger_by_its_weight_a_step_at_a_time():
    shift = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.4, 0.3, 0.0))
    m = _toy(weights=((0.0, 0.0),) * 4 + ((20.0, 0.0),), bias=(8.0, -50.0, -50.0, -50.0, 0.0),
             finger_shift=shift)
    a = _angles(index_mp_pitch=0.7, pinky_mp_pitch=0.3, pinky_pip_pitch=0.5, pinky_dip_pitch=0.4)
    pinky = [AXIS_NAMES.index(n) for n in finger_axes("pinky")]
    r = Retargeter(udcap_config(thumb_model=m))
    seen = []
    for _ in range(6):
        out = r.joint_targets(None, angles=a)
        seen.append([out[i] for i in pinky])
    # 0.1 rad per call until the whole shift is in.
    assert seen[0] == pytest.approx([0.4, 0.6, 0.4])
    assert seen[1] == pytest.approx([0.5, 0.7, 0.4])
    assert seen[-1] == pytest.approx([0.7, 0.8, 0.4])
    # The thumb aims at the shifted little finger.
    r2 = Retargeter(udcap_config(thumb_model=m, pinch_max_step_rad=0.0))
    out = r2.joint_targets(None, angles=a)
    assert _thumb(out) == pytest.approx(pinch_table("pinky").thumb_for(0.7, 0.8, 0.4), abs=1e-9)
    # Without the model's weight on the little finger, it is not moved.
    plain = Retargeter(udcap_config(thumb_model=m)).joint_targets(None, angles=_angles(
        pinky_mp_pitch=0.3, pinky_pip_pitch=0.5, pinky_dip_pitch=0.4))
    assert [plain[i] for i in pinky] == pytest.approx([0.3, 0.5, 0.4])


def test_the_operators_session_poses_through_the_fitted_model():
    # An operator's 16-pose calibration session (2026-10-24, UDCAP right
    # glove), per pose the median v1 angles, and the model fitted from it.
    doc = _session()
    m = _model()
    for f in TARGET_FINGERS:
        w = m.finger_weights(_pose(f"pinch_{f}"))
        assert w[f] > 0.9, (f, w)
        assert max(v for g, v in w.items() if g != f) < 0.05
    for pose in ("flat", "spread", "fist", "thumb_radial", "thumb_palmar", "thumb_across",
                 "key_pinch", "flat_check", "thumb_circle", "thumb_ip_cycle"):
        w = m.finger_weights(_pose(pose))
        assert max(w.values()) < 0.1, (pose, w)
    # Through the map: each pinch pose sends the thumb to that finger's table
    # thumb (at the shifted curl), held long enough for the step bound.
    for f in TARGET_FINGERS:
        r = Retargeter(udcap_config(thumb_model=m))
        for _ in range(40):
            out = r.joint_targets(None, angles=_pose(f"pinch_{f}"))
        curl = [out[AXIS_NAMES.index(n)] for n in finger_axes(f)]
        want = pinch_table(f).thumb_for(*curl)
        w = m.finger_weights(_pose(f"pinch_{f}"))[f]
        for n in range(4):
            assert abs(_thumb(out)[n] - want[n]) < 0.15 * (1 - w) + 0.02, (f, n, _thumb(out), want)
        assert pinch_table(f).gap_for(*curl) < 10.5, (f, curl)
    assert doc["model"]["fitted_from"].startswith("session 20261024T064600Z")


def test_the_thumb_model_binds_from_a_binding_file(tmp_path):
    m = _session()["model"]
    lines = ["[bind.retarget_config.thumb_model]"]
    for k, v in m.items():
        lines.append(f"{k} = {json.dumps(v)}")
    text = EXAMPLE.read_text() + "\n" + "\n".join(lines) + "\n"
    path = tmp_path / "b.toml"
    path.write_text(text)
    cfg = load_binding(path).binds[0].build_retarget().cfg
    assert cfg.thumb_model == _model()
    _, problems = build_retarget_config("wuji/wujihand2", {
        "mapping": "absolute", "thumb_absolute": True, "thumb_pinch": True,
        "thumb_model": {**m, "weights": [[0.0]]}})
    assert problems and "weights" in problems[0]
