"""The gesture_record reduction, ported: smoothing, reduction, HOME pinning,
idle trimming and the playability limiter on the daemon's spline."""
from __future__ import annotations

import math

import numpy as np
import pytest

from manipulation_kit.teach import (KeyframeOptions, Keyframe, Gesture, load_home,
                                    keyframes_from_poses, keyframes_from_samples,
                                    limit_joint_dynamics, smooth_samples, trim_idle,
                                    trajectory_points)
from manipulation_kit.teach.process import (DEFAULT_SPEED, HOME_SPEED_DEG_S,
                                            HOME_SPEED_MAX_DEG_S, LEGACY_SPEED,
                                            WRIST_JOINTS, reduce_samples,
                                            enforce_min_spacing,
                                            reduce_collinear,
                                            reduce_douglas_peucker, segment_peaks)
from manipulation_kit.teach.process import HomeReturn, reduce_poses
from manipulation_kit.teach.gesture_csv import parse_csv, sample, to_csv

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


def test_a_stream_pins_home_and_keeps_the_taught_wrist():
    """d1-2 task4 (2026-09-23): L7 35.7 deg taught, 0.0 exported under the
    old default pin. The wrist is free by default; ``pin_wrist`` pins it;
    a wrist joint that moved under 2 deg (sag/noise) is pinned either way."""
    c1 = HOME + np.r_[0, 12.0, 0, 0, 20.0, 5.0, 9.0, 0, 0, 0, 0, 0, 0, 1.2]
    q = _ramp([HOME + 3.0, c1, HOME + 2.0])
    times = np.arange(len(q)) / 20.0
    red = reduce_samples(times, q, HOME)
    rows = red.gesture.array()
    assert rows[0] == pytest.approx(HOME) and rows[-1] == pytest.approx(HOME)
    assert np.max(rows[:, 4]) > HOME[4] + 10.0              # R5 kept as taught
    assert np.ptp(rows[:, 6]) > 5.0                          # R7 kept
    # L7 only drifted +3 -> +1.2 -> +2 (range 1.8 deg, under 2): sag/noise,
    # pinned to HOME; L5/L6 span the 3 deg start offset and are kept
    assert red.pinned == [13] and not red.pin_all
    assert rows[:, 13] == pytest.approx(np.full(len(rows), HOME[13]))
    pinned = reduce_samples(times, q, HOME, KeyframeOptions(pin_wrist=True))
    assert set(pinned.pinned) == set(WRIST_JOINTS) and pinned.pin_all
    for j in WRIST_JOINTS:
        assert pinned.gesture.array()[:, j] == pytest.approx(
            np.full(len(pinned.gesture.keyframes), HOME[j]))
    lines = "\n".join(pinned.lines())
    assert "R5 " in lines and "(pinned: --pin-wrist)" in lines


@pytest.mark.parametrize("speed", [DEFAULT_SPEED, LEGACY_SPEED])
def test_limiting_stretches_time_only_and_meets_the_caps(speed):
    far = HOME + np.r_[0, 30.0, 0, -25.0, np.zeros(10)]
    g = Gesture([Keyframe(0.05, HOME), Keyframe(0.1, far), Keyframe(0.1, HOME)])
    out = limit_joint_dynamics(g, HOME, speed)
    assert out.array() == pytest.approx(g.array())          # poses untouched
    assert out.played_s > g.played_s
    assert all(b.duration >= a.duration for a, b in zip(g.keyframes, out.keyframes))
    for vel, acc in segment_peaks(trajectory_points(out, HOME)):
        assert vel <= speed.max_joint_vel_deg_s * 1.02
        assert acc <= speed.max_joint_acc_deg_s2 * 1.02
    again = limit_joint_dynamics(out, HOME, speed)          # idempotent
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
    red = reduce_poses([pose], HOME, 1.5)
    rows = red.gesture.array()
    # HOME, the pose, then the return: a dwell at the pose + min-jerk knots
    assert len(rows) == 2 + red.return_frames
    assert rows[0] == pytest.approx(HOME) and rows[-1] == pytest.approx(HOME)
    assert rows[1][1] == pytest.approx(pose[1]) and rows[2][1] == pytest.approx(pose[1])


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
        assert np.max(vel) <= DEFAULT_SPEED.max_joint_vel_deg_s * 1.03
        assert np.max(acc) <= DEFAULT_SPEED.max_joint_acc_deg_s2 * 1.03
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


def _home_leg(red):
    """``(points, first segment index)`` of the return on the daemon's spline."""
    points = trajectory_points(red.gesture, HOME)
    return points, len(points) - 1 - red.return_frames


def _leg_peaks(red):
    points, first = _home_leg(red)
    peaks = segment_peaks_per_joint(points)[first:]
    return (max(float(np.max(v)) for v, _ in peaks),
            max(float(np.max(a)) for _, a in peaks))


def _arrival_deg_s(red):
    points = trajectory_points(red.gesture, HOME)
    end, h = points[-1]["t"], 1e-3
    return float(np.max(np.abs(sample(points, end) - sample(points, end - h))) / h)


def test_home_return_duration_law():
    p = HomeReturn()
    assert (p.peak_vel_deg_s, p.peak_acc_deg_s2, p.min_s) == (40.0, 90.0, 2.0)
    assert p.duration_s(5.0) == 2.0                                # the floor
    assert p.duration_s(60.0) == pytest.approx(15 / 8 * 60 / 40)   # velocity-bound
    steep = HomeReturn(peak_vel_deg_s=400.0, peak_acc_deg_s2=20.0, min_s=0.0)
    assert steep.duration_s(60.0) == pytest.approx(math.sqrt(10 / math.sqrt(3) * 60 / 20))
    assert p.within(LEGACY_SPEED) == HomeReturn(25.0, 90.0)       # never over the ceiling
    with pytest.raises(ValueError):
        HomeReturn(peak_vel_deg_s=0.0)


@pytest.mark.parametrize("offset", [6.0, 12.0, 24.0, 60.0, 90.0])
def test_the_return_to_home_is_a_min_jerk_leg_within_its_profile(offset):
    times, q = _ends_away(offset)
    red = reduce_samples(times, q, HOME)
    g = red.gesture
    rows = g.array()
    assert red.ret and red.return_frames >= 3
    assert rows[-1] == pytest.approx(HOME)
    tail = rows[-red.return_frames - 1:]
    assert tail[0][1] == pytest.approx(HOME[1] - offset, abs=0.05)   # pose kept
    assert tail[1] == pytest.approx(tail[0])                          # the dwell
    # the min-jerk leg lasts exactly the profile's duration
    leg = sum(k.duration for k in g.keyframes[-red.return_frames + 1:])
    assert leg == pytest.approx(HomeReturn().duration_s(offset), rel=1e-6)
    vel, acc = _leg_peaks(red)
    assert vel <= 40.0 * 1.03 and acc <= 90.0 * 1.03
    assert _arrival_deg_s(red) < 1.0                                  # at rest at HOME
    assert red.return_s == pytest.approx(
        sum(k.duration for k in g.keyframes[-red.return_frames:]))
    assert check_gesture(g, HOME, step_s=0.05).ok


def test_the_return_ignores_how_fast_the_gesture_was_taught():
    """A fast take used to return at its own peak speed (up to 90 deg/s, one
    knot: 113 deg/s on the spline, 45 deg/s at HOME). The return's duration
    now depends on the distance only."""
    def take(seconds):
        times = np.arange(0.0, seconds + 2.0, 0.05)
        s = np.clip((times - 0.5) / seconds, 0, 1)
        q = np.tile(HOME, (len(times), 1))
        q[:, 7] += 60.0 * s * s * s * (10 - 15 * s + 6 * s * s)   # out, and stays
        return times, q
    slow, fast = reduce_samples(*take(4.0), HOME), reduce_samples(*take(0.6), HOME)
    assert fast.home_speed_deg_s == HOME_SPEED_MAX_DEG_S      # HOME-in unchanged
    assert slow.home_speed_deg_s < fast.home_speed_deg_s
    for red in (slow, fast):
        leg = sum(k.duration for k in red.gesture.keyframes[-red.return_frames + 1:])
        assert leg == pytest.approx(HomeReturn().duration_s(60.0), rel=1e-6)
        vel, _ = _leg_peaks(red)
        assert vel <= 40.0 * 1.03
    custom = reduce_samples(*take(0.6), HOME,
                            KeyframeOptions(home_return=HomeReturn(30.0, 60.0, 3.0)))
    assert custom.return_s > fast.return_s and _leg_peaks(custom)[0] <= 30.0 * 1.03


def test_the_dwell_brings_a_moving_end_to_rest_within_the_profile():
    """A take that stops recording mid-motion: the dwell segment carries the
    last body tangent and must still be within the return's profile."""
    times = np.arange(0.0, 2.0 + 1e-9, 0.05)
    q = np.tile(HOME, (len(times), 1))
    q[:, 1] -= 30.0 * np.clip((times - 0.5) / 1.5, 0.0, 1.0)     # still moving at the end
    red = reduce_samples(times, q, HOME)
    points, first = _home_leg(red)
    vel, acc = segment_peaks_per_joint(points)[first]
    assert float(np.max(vel)) <= 40.0 * 1.03 and float(np.max(acc)) <= 90.0 * 1.03
    assert check_gesture(red.gesture, HOME, step_s=0.05).ok


def test_idle_trim_leaves_the_return_alone():
    times, q = _ends_away(24.0)
    plain = reduce_samples(times, q, HOME)
    trimmed = reduce_samples(times, q, HOME, KeyframeOptions(max_idle_s=0.05))
    n = plain.return_frames
    assert [k.duration for k in trimmed.gesture.keyframes[-n:]] == pytest.approx(
        [k.duration for k in plain.gesture.keyframes[-n:]])


def test_a_keyframe_take_returns_home_on_the_same_profile():
    far = HOME + np.r_[0, -20.0, np.zeros(12)]
    red = reduce_poses([far], HOME, 1.5, KeyframeOptions(speed_limit=False))
    leg = sum(k.duration for k in red.gesture.keyframes[-red.return_frames + 1:])
    assert leg == pytest.approx(HomeReturn().duration_s(20.0))
    assert _leg_peaks(red)[0] <= 40.0 * 1.03 and _arrival_deg_s(red) < 1.0


def test_the_csv_declares_the_return_and_check_reports_it_on_its_own():
    times, q = _ends_away(60.0)
    red = reduce_samples(times, q, HOME)
    text = to_csv(red.gesture)
    assert "# mkit-teach: home_return_vel=40" in text
    assert "# mkit-teach: home_return_acc=90" in text
    assert f"# mkit-teach: home_return_frames={red.return_frames}" in text
    g = parse_csv(text)
    rep = check_gesture(g, HOME, step_s=0.05)
    assert rep.ok and rep.home_return_declared
    assert rep.home_return_segments == red.return_frames
    assert rep.home_return_s == pytest.approx(red.return_s, abs=1e-4 * red.return_frames)
    assert max(rep.home_return_peak_vel_deg_s) <= 40.0 * 1.03
    assert rep.home_return_arrival_deg_s < 1.0
    line = rep.home_return_line()
    assert line.startswith(f"  HOME return: {red.return_s:.2f} s over {red.return_frames} "
                           f"segment(s), peak velocity ")
    assert "profile 40 deg/s, 90 deg/s^2" in line and line in rep.summary()
    # a CSV that declares a slower profile than its return is a HARD failure
    g.meta[HomeReturn.VEL_KEY] = "10"
    bad = check_gesture(g, HOME, step_s=0.05)
    assert not bad.ok and any("HOME return velocity" in v for v in bad.violations)


def test_check_warns_about_a_legacy_one_knot_return():
    """The return as it was exported before the profile: one knot at the
    take's clamped peak speed. Warned about, not refused."""
    far = HOME + np.r_[0, -60.0, np.zeros(12)]
    g = Gesture([Keyframe(0.05, HOME), Keyframe(1.5, far), Keyframe(0.5, far),
                 Keyframe(60.0 / 90.0, HOME)],
                meta={"max_joint_vel": "150", "max_joint_acc": "600"})
    rep = check_gesture(g, HOME, step_s=0.05)
    assert rep.ok and not rep.home_return_declared and rep.home_return_segments == 1
    assert max(rep.home_return_peak_vel_deg_s) > 100.0        # ~113 deg/s
    assert rep.home_return_arrival_deg_s > 40.0               # ~45 deg/s at HOME
    assert any("predates the HOME-return profile" in w for w in rep.warnings)
    assert "(default; not declared by this CSV)" in rep.summary()


def test_the_breakdown_adds_up_and_shows_a_lost_joint():
    times = np.arange(0.0, 3.0, 0.05)
    q = np.tile(HOME, (len(times), 1))
    q[:, 7] += 40.0 * np.sin(np.pi * np.clip((times - 0.8) / 1.0, 0.0, 1.0))
    q[:, 13] += 30.0 * np.sin(np.pi * np.clip((times - 0.8) / 1.0, 0.0, 1.0))
    red = reduce_samples(times, q, HOME)
    assert red.body_s + red.connect_s + red.return_s == pytest.approx(red.gesture.played_s)
    count, added = red.stretched_knots()
    assert count >= 1 and added > 0                           # acc cap at corners
    text = red.breakdown()
    assert text.startswith(f"recorded {times[-1]:.2f} s -> body ")
    assert f"= {red.gesture.played_s:.2f} s" in text and f"+{added:.2f} s" in text
    ranges = red.range_lines()[0]
    assert "L1 40.0 ->" in ranges and "L7 30.0 ->" in ranges and "LOST" not in ranges
    pinned = reduce_samples(times, q, HOME, KeyframeOptions(pin_wrist=True))
    assert "L7 30.0 -> 0.0 (pinned: --pin-wrist)" in pinned.range_lines()[0]


def _fast_wrist_swing(spike: bool = False):
    """L7 swings -70 deg and back in 0.3 s, at 20 Hz (d1-2 task7's shape)."""
    times = np.arange(0.0, 2.0, 0.05)
    q = np.tile(HOME, (len(times), 1))
    s = np.clip((times - 0.8) / 0.3, 0.0, 1.0)
    q[:, 13] -= 70.0 * np.sin(np.pi * s)
    if spike:
        q[5, 1] += 6.0                     # one encoder spike on R2, flat around it
    return times, q


def test_a_fast_wrist_peak_survives_smoothing_and_reduction():
    """task7: recorded L7 -70.0 deg, exported -58.4 (the 5-sample median ->
    mean shaved 12 deg). The taught extreme must survive within 2 deg."""
    times, q = _fast_wrist_swing()
    red = reduce_samples(times, q, HOME)
    t, poses = sample_path(trajectory_points(red.gesture, HOME), 0.005)
    recorded = float(np.min(q[:, 13]))
    exported = float(np.min(poses[:, 13]))
    assert abs(exported - recorded) <= 2.0, (recorded, exported)
    assert "peak shaved" not in red.range_lines()[0]


def test_a_single_sample_spike_is_still_rejected():
    times, q = _fast_wrist_swing(spike=True)
    red = reduce_samples(times, q, HOME)
    _, poses = sample_path(trajectory_points(red.gesture, HOME), 0.005)
    assert np.max(poses[:, 1] - HOME[1]) < 1.5


def test_a_shaved_peak_is_reported():
    """A joint whose exported range is > 3 deg under the recorded one is
    flagged, so a flattened turn is visible in export's output."""
    import manipulation_kit.teach.process as process
    times, q = _fast_wrist_swing()
    red = reduce_samples(times, q, HOME)
    low = list(HOME)
    low[13] -= 60.0                                   # 10 deg short of -70
    red.gesture = process.Gesture([process.Keyframe(0.05, list(HOME)),
                                   process.Keyframe(1.0, low),
                                   process.Keyframe(1.0, list(HOME))])
    assert "(peak shaved 10.0 deg)" in red.range_lines()[0]
