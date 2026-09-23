"""The gesture_record reduction, ported: smoothing, reduction, HOME pinning,
idle trimming and the playability limiter on the daemon's spline."""
from __future__ import annotations

import numpy as np
import pytest

from manipulation_kit.teach import (KeyframeOptions, Keyframe, Gesture, load_home,
                                    keyframes_from_poses, keyframes_from_samples,
                                    limit_joint_dynamics, smooth_samples, trim_idle,
                                    trajectory_points)
from manipulation_kit.teach.process import (WRIST_JOINTS, enforce_min_spacing,
                                            reduce_collinear,
                                            reduce_douglas_peucker, segment_peaks)

HOME = np.array(load_home())


def _ramp(corners, n_per=20):
    """Piecewise-linear dense samples through ``corners`` (list of 14-vectors)."""
    out = []
    for a, b in zip(corners, corners[1:]):
        for k in range(n_per):
            out.append(np.asarray(a) + (np.asarray(b) - a) * k / n_per)
    out.append(np.asarray(corners[-1]))
    return np.array(out)


def test_smoothing_rejects_a_spike_and_keeps_the_sample_count():
    q = np.tile(HOME, (21, 1))
    q[10, 1] += 30.0                       # one encoder outlier
    out = smooth_samples(q, 5)
    assert out.shape == q.shape
    assert np.max(np.abs(out - HOME)) < 1e-9   # the median removed it outright
    assert smooth_samples(q, 1) == pytest.approx(q)   # window <= 1 is off


def test_an_even_window_is_rounded_up_to_odd():
    q = np.tile(HOME, (9, 1)) + np.arange(9)[:, None]
    assert smooth_samples(q, 4) == pytest.approx(smooth_samples(q, 5))


@pytest.mark.parametrize("reduce", [reduce_collinear, reduce_douglas_peucker])
def test_reduction_keeps_exactly_the_corners_of_a_piecewise_linear_path(reduce):
    c1 = HOME + np.r_[10.0, np.zeros(13)]
    c2 = HOME + np.r_[10.0, 8.0, np.zeros(12)]
    q = _ramp([HOME, c1, c2, HOME])
    kept = reduce(q, 0.05)
    assert kept == [0, 20, 40, 60]


def test_min_spacing_drops_crowded_interior_keyframes():
    times = np.arange(10) * 0.05
    assert enforce_min_spacing([0, 1, 2, 3, 9], times, 0.1) == [0, 2, 9]


def test_a_stream_pins_home_and_locks_the_wrist():
    c1 = HOME + np.r_[0, 12.0, 0, 0, 20.0, 5.0, 9.0, 0, 0, 0, 0, 0, 0, 0]
    q = _ramp([HOME + 3.0, c1, HOME + 2.0])
    times = np.arange(len(q)) / 20.0
    g = keyframes_from_samples(times, q, HOME)
    rows = g.array()
    assert rows[0] == pytest.approx(HOME) and rows[-1] == pytest.approx(HOME)
    for j in WRIST_JOINTS:
        assert rows[:, j] == pytest.approx(np.full(len(rows), HOME[j]))
    free = keyframes_from_samples(times, q, HOME, KeyframeOptions(lock_wrist=False))
    assert np.max(free.array()[:, 4]) > HOME[4] + 10.0


def test_limiting_stretches_time_only_and_meets_the_caps():
    far = HOME + np.r_[0, 30.0, 0, -25.0, np.zeros(10)]
    g = Gesture([Keyframe(0.05, HOME), Keyframe(0.2, far), Keyframe(0.2, HOME)])
    out = limit_joint_dynamics(g, HOME)
    assert out.array() == pytest.approx(g.array())          # poses untouched
    assert all(b.duration >= a.duration for a, b in zip(g.keyframes, out.keyframes))
    for vel, acc in segment_peaks(trajectory_points(out, HOME)):
        assert vel <= 25.0 * 1.02 and acc <= 120.0 * 1.02
    again = limit_joint_dynamics(out, HOME)                 # idempotent
    assert [k.duration for k in again.keyframes] == pytest.approx(
        [k.duration for k in out.keyframes], abs=1e-3)


def test_trim_idle_caps_only_keyframes_that_do_not_move():
    moved = HOME + np.r_[5.0, np.zeros(13)]
    g = Gesture([Keyframe(0.05, HOME), Keyframe(1.0, moved), Keyframe(3.0, moved),
                 Keyframe(1.0, HOME)])
    out = trim_idle(g, 0.5)
    assert [k.duration for k in out.keyframes] == [0.05, 1.0, 0.5, 1.0]
    assert trim_idle(g, 0.0).keyframes[2].duration == 3.0     # Off keeps pauses


def test_stepped_keyframes_get_home_around_them():
    pose = HOME + np.r_[0, 10.0, np.zeros(12)]
    g = keyframes_from_poses([pose], HOME, 1.5)
    rows = g.array()
    assert len(rows) == 3
    assert rows[0] == pytest.approx(HOME) and rows[-1] == pytest.approx(HOME)
    assert rows[1][1] == pytest.approx(pose[1])
