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


def test_an_arm_swung_into_the_torso_is_an_advisory_warning_not_a_failure(guard):
    """make_joint_test_gesture's --unsafe-demo: A J2 +30 deg into the belly.
    For a taught gesture the guard is advisory: a WARNING with the closest
    distance, the frames and the time — and the hard checks decide ``ok``."""
    report = check_gesture(_bump(np.r_[0, 30.0, np.zeros(12)]), HOME, guard=guard,
                           step_s=0.05)
    assert report.ok and not report.violations, report.summary()
    assert not report.guard_clear
    body = [f for f in report.guard_findings if f.kind in ("body", "chest")]
    assert body, report.guard_findings
    worst = body[0]
    assert worst.clearance_m < 0.03                    # under the 30 mm margin
    assert worst.clearance_m == pytest.approx(report.min_body_clearance_m, abs=1e-9) \
        or worst.kind == "chest"
    assert "arm A link" in worst.detail and "Link" in worst.detail
    assert 0.0 < worst.t < report.duration_s and worst.samples >= 1
    text = report.summary()
    assert "WARNING: guard (advisory)" in text and "mm at t=" in text
    assert "VIOLATION" not in text
    assert "body" in report.clearance_note() and "below margin" in report.clearance_note()


def test_non_increasing_times_are_a_hard_failure(guard):
    g = Gesture([Keyframe(0.05, HOME), Keyframe(1.0, HOME), Keyframe(1.0, HOME)])
    g.keyframes[1].duration = 0.0                 # built in code, not parsed
    report = check_gesture(g, HOME, guard=guard, step_s=0.05)
    assert not report.ok
    assert any(v.startswith("timing:") and "does not increase" in v
               for v in report.violations)


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
    report = check_gesture(_bump(np.r_[0, -20.0, np.zeros(12)], seconds=0.1), HOME,
                           guard=guard, step_s=0.01)
    assert any("velocity" in v and "150 deg/s cap" in v for v in report.violations)


def test_the_ceiling_is_the_csvs_own_unless_overridden(guard):
    """A CSV exported at a higher ceiling carries it, and check holds it to
    that one — not the default; an explicit policy still wins."""
    from manipulation_kit.teach import SpeedPolicy
    fast = _bump(np.r_[0, -20.0, np.zeros(12)], seconds=0.1)
    fast.meta.update(SpeedPolicy(1000.0, 100000.0).meta())
    assert check_gesture(fast, HOME, guard=guard, step_s=0.01).ok
    strict = check_gesture(fast, HOME, guard=guard, step_s=0.01,
                           speed=SpeedPolicy(25.0, 120.0))
    assert any("25 deg/s cap" in v for v in strict.violations)


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
