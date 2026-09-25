"""The omakaseos gesture CSV contract, pinned against the reader of record
(omakase-core ``firmware_session.trajectory_from_csv``)."""
from __future__ import annotations

import math

import numpy as np
import pytest

from manipulation_kit.teach import (HEADER, Gesture, GestureFormatError, Keyframe,
                                    load_home, parse_csv, to_csv, trajectory_points)
from manipulation_kit.teach.gesture_csv import UNSAFE_TAG, sample

HOME = load_home()


def _gesture(*rows):
    return Gesture([Keyframe(d, q) for d, q in rows])


def test_home_is_the_kits_file_in_csv_order():
    # A1..A7 then B1..B7; the arms are a mirrored pair (J1/J3/J5/J7 flip)
    assert len(HOME) == 14
    for j in (0, 2, 4, 6):
        assert HOME[j] == pytest.approx(-HOME[7 + j])
    for j in (1, 3, 5):
        assert HOME[j] == pytest.approx(HOME[7 + j])


def test_the_header_is_duration_then_r1_to_l7():
    assert HEADER == "duration,R1,R2,R3,R4,R5,R6,R7,L1,L2,L3,L4,L5,L6,L7"


def test_written_rows_are_tocsv_format_and_round_trip():
    pose = [round(v + 3.0, 4) for v in HOME]
    g = _gesture((0.05, HOME), (1.25, pose), (0.8, HOME))
    g.meta = {"name": "wave", "source": "teach"}
    text = to_csv(g)
    lines = text.splitlines()
    assert lines[0].startswith("# D1 dual-arm gesture (omakaseos keyframe format")
    assert "# mkit-teach: name=wave" in lines
    assert lines[lines.index(HEADER) + 1].startswith("0.0500,-52.2600,87.3800")
    back = parse_csv(text)
    assert back.meta == {"name": "wave", "source": "teach"}
    assert back.array() == pytest.approx(g.array())
    assert to_csv(back) == text


def test_a_gripper_column_is_refused_because_the_player_refuses_it():
    text = HEADER + ",G\n" + "0.5," + ",".join(["0"] * 15) + "\n"
    with pytest.raises(GestureFormatError, match="exactly 15"):
        parse_csv(text)


@pytest.mark.parametrize("bad", ["0.0", "-1", "nan"])
def test_a_non_positive_or_non_finite_duration_is_refused(bad):
    with pytest.raises(GestureFormatError):
        parse_csv(HEADER + "\n" + bad + "," + ",".join(["0"] * 14) + "\n")


def test_kp_kd_rows_and_comments_are_skipped_like_the_readers_do():
    row = "0.5," + ",".join(f"{v}" for v in HOME)
    text = "# c\nduration,x\nkp,1,2\nkd,1,2\n" + row + "\n" + row + "\n"
    assert len(parse_csv(text).keyframes) == 2


def test_the_unsafe_tag_survives_a_round_trip():
    g = _gesture((0.05, HOME), (1.0, HOME))
    g.unsafe = ["arm A link Link2_R within 0.010 m of body box torso_belly"]
    back = parse_csv(to_csv(g))
    assert back.unsafe == g.unsafe
    assert f"# mkit-teach: {UNSAFE_TAG}=" in to_csv(g)


def test_the_player_pins_home_at_zero_and_ignores_row_zero_duration():
    """firmware_session.trajectory_from_csv: rows[0] and rows[-1] become HOME,
    the spline starts at HOME at t=0, row k lands at the cumulative sum of
    durations 1..k — row 0's duration plays no part."""
    off = [v + 5.0 for v in HOME]
    mid = [v + 2.0 for v in HOME]
    g = _gesture((9.0, off), (1.0, mid), (0.5, off))
    points = trajectory_points(g, HOME)
    assert points[0] == {"t": 0.0, "a": HOME[:7], "b": HOME[7:]}
    assert [p["t"] for p in points] == [0.0, 1.0, 1.5]
    assert points[1]["a"] == mid[:7] and points[1]["b"] == mid[7:]
    assert points[-1]["a"] == HOME[:7] and points[-1]["b"] == HOME[7:]


def test_the_spline_passes_through_every_knot():
    mid = [v + 4.0 for v in HOME]
    points = trajectory_points(_gesture((0.05, HOME), (1.0, mid), (2.0, HOME)), HOME)
    for p in points:
        assert sample(points, p["t"]) == pytest.approx(np.array(p["a"] + p["b"]))
    # linear time + Catmull-Rom: the midpoint of a symmetric bump overshoots
    # the chord toward the peak, never away from it
    q = sample(points, 0.5)
    assert np.all(q >= np.array(HOME) - 1e-9)
    assert math.isfinite(float(q.sum()))
