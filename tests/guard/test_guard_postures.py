"""The postures the default guard must accept and refuse (2026-09-23).

Joint vectors from the guard-margin study (IK onto the named body poses).
With the model at the real shell, HOME must clear the body with room to
spare, the arm hanging at the side must pass, and arms crossed at the chest
must not.
"""
import pytest

from manipulation_kit.guard import MotionGuard

HOME = ([-52.26, 87.38, 88.3, -114.32, 86.67, -1.1, 13.05],
        [52.26, 87.38, -88.3, -114.32, -86.67, -1.1, -13.05])
MIRROR = (-1, 1, -1, 1, -1, 1, -1)


def both(qa):
    return qa, [a * s for a, s in zip(qa, MIRROR)]


PASS = {
    "P1 arm hanging straight down": both([-89.88, 89.1, 96.83, -6.86, 86.67, -1.1, 13.05]),
    "P1b elbow at the side, forearm forward": both([-87.23, 89.05, 89.95, -96.76, 86.7, 0.21, -3.83]),
    "P1c elbow at the side, tips 30 deg in": both([-86.66, 89.21, 86.68, -97.29, 94.84, -39.24, 0.71]),
}
P2 = ([-136.35, 101.22, 134.69, -86.76, 101.01, -54.44, -14.24],
      [125.75, 99.37, -136.56, -78.99, -96.71, -45.66, 4.09])


def test_home_clears_the_body_by_the_margin_plus_5mm():
    guard = MotionGuard(clamp_limits=False)
    report = guard.check(*HOME)
    assert report.ok, report
    need = guard.body_margin_m + 0.005
    assert report.min_body_clearance >= need, report
    assert report.min_chest_clearance >= need, report


@pytest.mark.parametrize("name", sorted(PASS))
def test_arm_at_the_side_passes(name):
    report = MotionGuard(clamp_limits=False).check(*PASS[name])
    assert report.ok, f"{name}: {report}"


def test_arms_crossed_at_the_chest_are_refused():
    report = MotionGuard(clamp_limits=False).check(*P2)
    assert not report.ok
    assert any("torso_belly" in v for v in report.violations), report
