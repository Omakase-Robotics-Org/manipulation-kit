"""d1-2 task6 (2026-09-23): the CSV was right (L7 70.6 deg, peak 117.8
deg/s) but it played slow and smoothed with J7 barely moving — position mode
at the fixed 0.15 lets the controller track ~21 deg/s. ``play`` now derives
the ratio from the gesture's own peak; the CSV's SpeedPolicy stays the gate."""
from __future__ import annotations

import numpy as np
import pytest

from manipulation_kit.teach import (Gesture, Keyframe, SpeedPolicy, limit_joint_dynamics,
                                    load_home)
from manipulation_kit.teach.play import APPROACH_RATIO, play, playback_ratio

HOME = list(load_home())


def _swing(peak_cap: float) -> Gesture:
    """L7 out 60 deg and back, limited to ``peak_cap`` deg/s: its peak on
    the played spline is the cap."""
    out = list(np.array(HOME) + np.r_[np.zeros(13), 60.0])
    g = Gesture([Keyframe(0.05, HOME), Keyframe(0.2, out), Keyframe(0.2, HOME)])
    return limit_joint_dynamics(g, HOME, SpeedPolicy(peak_cap, 1e6))


def test_a_118_deg_s_gesture_plays_at_full_ratio():
    ratio, why = playback_ratio(_swing(118.0), HOME)
    assert ratio == 1.0 and "peak 11" in why


def test_a_legacy_25_deg_s_gesture_plays_at_the_floor():
    ratio, _ = playback_ratio(_swing(25.0), HOME)
    assert ratio == pytest.approx(0.3)


def test_a_mid_speed_gesture_gets_headroom_over_its_peak():
    ratio, _ = playback_ratio(_swing(70.0), HOME)
    assert ratio == pytest.approx(1.3 * 70.0 / 140.0, rel=0.03)


def _mode_ratios(daemon):
    return [b["vel_ratio"] for m, p, b in daemon.calls
            if m == "POST" and p.endswith("/mode") and b.get("mode") == "position"]


def test_play_approaches_calmly_then_plays_at_the_derived_ratio(daemon, robot_factory):
    daemon.pose["b"][3] += 10.0                     # away from HOME: an approach
    said = []
    with robot_factory(vel_ratio=APPROACH_RATIO, acc_ratio=APPROACH_RATIO) as robot:
        report = play(robot, _swing(118.0), HOME, no_safety=True, announce=said.append)
    assert report.ok, report.detail
    ratios = _mode_ratios(daemon)
    assert ratios[:2] == [APPROACH_RATIO, APPROACH_RATIO]     # entry, both arms
    assert ratios[-2:] == [1.0, 1.0]                          # before the gesture
    calls = [(m, p) for m, p, _ in daemon.calls if m == "POST"]
    starts = [i for i, c in enumerate(calls) if c[1] == "/v1/arm/trajectory/start"]
    last_mode = max(i for i, c in enumerate(calls) if c[1].endswith("/mode"))
    assert starts[0] < last_mode < starts[1]                  # approach, ratio, gesture
    assert any("playback ratio 1.00" in s and "HOME approach at 0.30" in s for s in said)


def test_vel_ratio_overrides_the_derived_one(daemon, robot_factory):
    with robot_factory() as robot:
        report = play(robot, _swing(118.0), HOME, no_safety=True, vel_ratio=0.5)
    assert report.ok and _mode_ratios(daemon)[-2:] == [0.5, 0.5]
    assert any("(--vel-ratio)" in n for n in report.notes)


def test_the_cli_play_default_derives_the_ratio():
    from manipulation_kit.teach.cli import build_parser
    assert build_parser().parse_args(["play", "x.csv"]).vel_ratio is None
    assert build_parser().parse_args(["record"]).vel_ratio == 0.15
