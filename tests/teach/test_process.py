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


# -- HOME rules for a hand-guided take (Shu, 2026-09-23) --------------------- #
from manipulation_kit.teach import check_gesture  # noqa: E402
from manipulation_kit.teach.gesture_csv import sample_path  # noqa: E402
from manipulation_kit.teach.process import (HOME_SPEED_DEG_S,  # noqa: E402
                                            segment_peaks_per_joint, settle_index)


def _sagged_take(rate_hz=20.0):
    """Brakes open at HOME: A J2 and J4 drop 3 / 2 deg in 0.2 s, the operator
    catches it back to HOME by 0.4 s, holds, then swings J2 -10 deg and back
    (1.0 .. 5.0 s)."""
    times = np.arange(0.0, 6.0 + 1e-9, 1.0 / rate_hz)
    q = np.tile(HOME, (len(times), 1))
    for i, t in enumerate(times):
        dip = np.sin(np.pi * min(t, 0.4) / 0.4) if t < 0.4 else 0.0
        q[i, 1] += 3.0 * dip
        q[i, 3] += 2.0 * dip
        if 1.0 <= t <= 5.0:
            q[i, 1] -= 10.0 * np.sin(np.pi * (t - 1.0) / 4.0)
    return times, q


def test_the_release_sag_is_found_and_cut():
    times, q = _sagged_take()
    start = settle_index(times, q)
    assert 0.3 <= times[start] <= 0.5
    still = np.tile(HOME, (40, 1))
    assert settle_index(np.arange(40) / 20.0, still) == 0      # no sag, no cut


def test_a_sagged_take_exports_from_home_without_the_dip_or_a_spike():
    times, q = _sagged_take()
    g = keyframes_from_samples(times, q, HOME)
    rows = g.array()
    assert rows[0] == pytest.approx(HOME) and rows[-1] == pytest.approx(HOME)
    points = trajectory_points(g, HOME)
    t, poses = sample_path(points, 0.01)
    early = poses[t <= 0.6]
    # the +3 deg dip is gone (the swing that follows is -J2)
    assert np.max(early[:, 1] - HOME[1]) < 0.3
    assert np.max(np.abs(early[:, 3] - HOME[3])) < 0.6
    # continuous and capped: no segment is a spike
    for vel, acc in segment_peaks_per_joint(points):
        assert np.max(vel) <= 25.0 * 1.03 and np.max(acc) <= 120.0 * 1.03
    assert check_gesture(g, HOME, step_s=0.05).ok
    # without the cut the dip IS played
    raw = keyframes_from_samples(times, q, HOME, KeyframeOptions(sag_max_s=0.0))
    _, raw_poses = sample_path(trajectory_points(raw, HOME), 0.01)
    assert np.max(raw_poses[:, 1] - HOME[1]) > 1.5


def _ends_away(offset_deg):
    """A stream that ends held ``offset_deg`` from HOME on A J2."""
    times = np.arange(0.0, 3.0 + 1e-9, 0.05)
    q = np.tile(HOME, (len(times), 1))
    ramp = np.clip((times - 0.5) / 1.5, 0.0, 1.0)
    q[:, 1] -= offset_deg * ramp
    return times, q


@pytest.mark.parametrize("offset", [6.0, 12.0, 24.0])
def test_the_return_to_home_is_appended_at_a_constant_speed(offset):
    times, q = _ends_away(offset)
    o = KeyframeOptions(speed_limit=False)
    g = keyframes_from_samples(times, q, HOME, o)
    rows = g.array()
    assert rows[-1] == pytest.approx(HOME)
    assert rows[-2][1] == pytest.approx(HOME[1] - offset, abs=0.05)  # pose kept
    assert g.keyframes[-1].duration == pytest.approx(offset / HOME_SPEED_DEG_S,
                                                     rel=0.01)


def test_return_duration_is_proportional_to_distance_after_the_limiter_too():
    durations = []
    for offset in (8.0, 16.0, 32.0):
        times, q = _ends_away(offset)
        durations.append(keyframes_from_samples(times, q, HOME).keyframes[-1].duration)
    assert durations[0] < durations[1] < durations[2]
    assert durations[2] / durations[1] == pytest.approx(2.0, rel=0.2)


def test_a_keyframe_take_returns_home_at_the_same_speed():
    far = HOME + np.r_[0, -20.0, np.zeros(12)]
    g = keyframes_from_poses([far], HOME, 1.5, KeyframeOptions(speed_limit=False))
    assert g.keyframes[-1].duration == pytest.approx(20.0 / HOME_SPEED_DEG_S)
