"""Coupling-table tests against known rows from the vendor xlsx."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import coupling  # noqa: E402


def test_all_bundled_tables_load_and_are_monotonic():
    tables = coupling.load_all()
    assert set(tables) == set(coupling.FINGERS)
    assert "linkage_to_knuckle" not in tables["thumb"]  # vendor ships none
    for finger, pairs in tables.items():
        for pair, t in pairs.items():
            assert len(t.x) > 100, f"{finger}/{pair} suspiciously small"


def test_known_rows_index_linkage_to_knuckle():
    # Last row of 食指-连杆-指节运动关系-20260115.xlsx: 77.0 → 78.2340495031668
    assert coupling.linkage_to_knuckle("index", 77.0) == pytest.approx(78.2340, abs=1e-3)
    # Near-zero row maps ~0 → ~0
    assert coupling.linkage_to_knuckle("index", 0.0147) == pytest.approx(0.0229, abs=1e-3)


def test_known_rows_middle():
    # 中指 tables (800-row version): linkage 78.0 → knuckle 77.3582229673355
    assert coupling.linkage_to_knuckle("middle", 78.0) == pytest.approx(77.3582, abs=1e-3)
    # knuckle 77.3582 → fingertip 116.466962561067
    assert coupling.knuckle_to_fingertip("middle", 77.3582) == pytest.approx(116.467, abs=1e-2)


def test_known_rows_thumb_fingertip():
    # 大拇指-指节-指尖: knuckle 80.0 → fingertip 67.0828260842519
    assert coupling.knuckle_to_fingertip("thumb", 80.0) == pytest.approx(67.083, abs=1e-2)


def test_composition_linkage_to_fingertip():
    # pinky: linkage 78 → knuckle 79.7988 → fingertip 110.833
    assert coupling.linkage_to_fingertip("pinky", 78.0) == pytest.approx(110.833, abs=5e-2)


def test_inverse_roundtrip():
    for finger in ("index", "middle", "ring", "pinky"):
        for deg in (5.0, 20.0, 40.0, 70.0):
            k = coupling.linkage_to_knuckle(finger, deg)
            assert coupling.knuckle_to_linkage(finger, k) == pytest.approx(deg, abs=1e-6)
            tip = coupling.knuckle_to_fingertip(finger, k)
            assert coupling.fingertip_to_knuckle(finger, tip) == pytest.approx(k, abs=1e-6)


def test_extrapolation_is_clamped():
    t = coupling.load_table("index", "linkage_to_knuckle")
    lo, hi = t.output_range
    assert t(-100.0) == pytest.approx(lo)
    assert t(1e6) == pytest.approx(hi)


def test_thumb_linkage_table_absent():
    with pytest.raises(ValueError, match="thumb"):
        coupling.load_table("thumb", "linkage_to_knuckle")


def test_unknown_finger_rejected():
    with pytest.raises(ValueError):
        coupling.load_table("palm", "linkage_to_knuckle")
