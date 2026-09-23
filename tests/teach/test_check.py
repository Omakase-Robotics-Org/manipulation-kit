"""``check``: guard, joint limits (incl. the coupled wrist limit) and rates,
on the spline the daemon actually plays."""
from __future__ import annotations

import numpy as np
import pytest

from manipulation_kit.teach import (Gesture, Keyframe, ascii_preview,
                                    check_gesture, load_home)

HOME = load_home()


@pytest.fixture(scope="module")
def guard():
    from manipulation_kit.guard import MotionGuard
    return MotionGuard(clamp_limits=False)


def _bump(delta, *, seconds=4.0):
    pose = list(np.array(HOME) + np.asarray(delta, dtype=float))
    return Gesture([Keyframe(0.05, HOME), Keyframe(seconds, pose),
                    Keyframe(seconds, HOME)])


def test_a_gentle_bump_from_home_is_ok(guard):
    report = check_gesture(_bump(np.r_[0, -6.0, np.zeros(12)]), HOME, guard=guard,
                           step_s=0.05)
    assert report.ok, report.summary()
    assert report.duration_s == pytest.approx(8.0)
    assert set(report.flange_range_m) == {"left", "right"}
    assert max(report.peak_vel_deg_s) < 25.0


def test_an_arm_swung_into_the_torso_is_a_violation(guard):
    """make_joint_test_gesture's --unsafe-demo: A J2 +30 deg into the belly."""
    report = check_gesture(_bump(np.r_[0, 30.0, np.zeros(12)]), HOME, guard=guard,
                           step_s=0.05)
    assert not report.ok
    assert any("body box" in v or "chest" in v for v in report.violations), report.violations


def test_the_coupled_wrist_limit_is_checked_where_the_box_would_pass(guard):
    """J6 = 55 deg allows |J7| <= 34 deg (39 measured minus 5 margin) though
    the box says 90. Arm B: J6 -1.1 -> 55, J7 -13 -> -60."""
    delta = np.zeros(14)
    delta[12] = 56.1
    delta[13] = -47.0
    report = check_gesture(_bump(delta), HOME, guard=guard, step_s=0.05)
    assert any("coupled wrist_roll" in v for v in report.violations), report.violations


def test_a_joint_limit_is_a_violation_not_a_clip(guard):
    delta = np.zeros(14)
    delta[12] = 70.0                      # B J6 past +/-60
    report = check_gesture(_bump(delta, seconds=8.0), HOME, guard=guard, step_s=0.05)
    assert any("outside" in v for v in report.violations), report.violations


def test_too_fast_is_a_violation(guard):
    report = check_gesture(_bump(np.r_[0, -6.0, np.zeros(12)], seconds=0.1), HOME,
                           guard=guard, step_s=0.01)
    assert any("velocity" in v for v in report.violations)


def test_a_row_the_player_will_replace_is_warned_about(guard):
    off = list(np.array(HOME) + 5.0)
    g = Gesture([Keyframe(1.0, off), Keyframe(2.0, HOME), Keyframe(1.0, HOME)])
    report = check_gesture(g, HOME, guard=guard, step_s=0.1)
    assert any("first row" in w for w in report.warnings)


def test_the_ascii_preview_draws_one_strip_per_joint():
    text = ascii_preview(_bump(np.r_[0, -6.0, np.zeros(12)]), HOME, width=40)
    lines = text.splitlines()
    assert len(lines) == 15 and lines[2].startswith(" R2 ")
    assert "─" in lines[1]            # R1 does not move: a flat line
