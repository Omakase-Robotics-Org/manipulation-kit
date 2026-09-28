"""The named poses, and HOME_CLEAR measured against the kit's own guard.

Every verdict here is the kit's: ``GuardedArm.posture_violation`` (the joint
box and the coupled wrist-roll limit J7(J6)) and then the motion guard on the
full two-arm posture (``GuardGate.report``), exactly what planning and the
executor ask before a joint vector is sent. Nothing is re-derived.

Why HOME_CLEAR exists: at HOME the upper arm (``Link2``) is the binding link,
36.3 mm (arm B, logical right) and 30.3 mm (arm A, logical left) from the
``torso_belly`` box against the 30 mm body margin, so a small +J2 command from
HOME is refused and most of HOME's +/-0.1 rad band is not sendable. Lowering J2
by 10 deg swings ``Link2`` clear; what binds then is the shoulder sphere
``Link1``, which is fixed at the mount and so is the best ANY posture reaches
against this collision model.
"""

from __future__ import annotations

import numpy as np
import pytest

from manipulation_kit.arms.d1.arm import kinematics as mk

#: A1..A7 -> B1..B7 joint-by-joint mirror (B = A * sign), as home_pose.json.
MIRROR_SIGN = np.array([-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0])
BAND_RAD = 0.1
BAND_SAMPLES = 1024


@pytest.fixture(scope="module")
def kin():
    arm = mk.build_kinematics(find_ready=False, quiet=True)
    if not arm.gate.installed:
        pytest.skip("the motion guard is not available")
    return arm


def _verdict(kin, side, q_left, q_right):
    """The kit's answer for one two-arm posture [rad]: ``(ok, reason)``."""
    why = kin.posture_violation(side, q_left if side == "left" else q_right)
    if why is not None:
        return False, why
    rep = kin.gate.report(q_left, q_right)
    return bool(rep.ok), "; ".join(rep.violations)


def _body_clearance_m(kin, side, q):
    """One arm's minimum capsule-to-body-box clearance [m], from the guard."""
    deg = list(np.degrees(q))
    rep = kin.gate.guard.check(deg, None) if side == "left" \
        else kin.gate.guard.check(None, deg)
    return rep.min_body_clearance


def _band_fraction(kin, side, pose, *, seed=0):
    """Clean fraction of the +/-BAND_RAD cube around ``pose[side]``, every
    joint sampled independently, the other arm at its ``pose`` value."""
    rng = np.random.default_rng(seed)
    clean = 0
    for d in rng.uniform(-BAND_RAD, BAND_RAD, (BAND_SAMPLES, 7)):
        q = {**pose, side: pose[side] + d}
        clean += _verdict(kin, side, q["left"], q["right"])[0]
    return clean / BAND_SAMPLES


def test_named_poses_load_and_home_is_load_home():
    home = mk.load_home(quiet=True)
    named = mk.load_named_pose("home")
    for side in ("left", "right"):
        np.testing.assert_array_equal(named[side], home[side])
    with pytest.raises(ValueError, match="unknown named pose"):
        mk.load_named_pose("daran")


@pytest.mark.parametrize("name", sorted(mk.NAMED_POSES))
def test_every_named_pose_is_mirror_symmetric(name):
    pose = mk.load_named_pose(name)
    np.testing.assert_allclose(pose["right"], pose["left"] * MIRROR_SIGN, atol=1e-12)


def test_home_clear_is_home_with_both_j2_ten_degrees_lower():
    home, clear = mk.load_named_pose("home"), mk.load_named_pose("home_clear")
    for side in ("left", "right"):
        delta = np.degrees(clear[side] - home[side])
        np.testing.assert_allclose(delta, [0, -10.0, 0, 0, 0, 0, 0], atol=1e-9)


def test_home_clear_is_within_the_box_and_the_coupled_limit(kin):
    clear = mk.load_named_pose("home_clear")
    for side in ("left", "right"):
        assert kin.posture_violation(side, clear[side]) is None


def test_home_clear_body_clearance_reaches_the_shoulder_floor(kin):
    home, clear = mk.load_named_pose("home"), mk.load_named_pose("home_clear")
    margin = kin.gate.guard.body_margin_m
    got = {s: _body_clearance_m(kin, s, clear[s]) for s in ("left", "right")}
    was = {s: _body_clearance_m(kin, s, home[s]) for s in ("left", "right")}
    # HOME, the finding: Link2 at 36.3 / 30.3 mm against a 30 mm margin.
    assert was["right"] == pytest.approx(0.0363, abs=5e-4)
    assert was["left"] == pytest.approx(0.0303, abs=5e-4)
    # HOME_CLEAR: 43.1 / 38.1 mm, 13.1 / 8.1 mm over the margin.
    assert got["right"] == pytest.approx(0.0431, abs=5e-4)
    assert got["left"] == pytest.approx(0.0381, abs=5e-4)
    assert min(got.values()) - margin > 0.008
    # ...and that is the floor: no posture of either arm does better, because
    # the binding capsule (Link1, at the mount) does not move with the joints.
    rng = np.random.default_rng(1)
    lo, hi = kin.limits("right")
    for side in ("left", "right"):
        best = max(_body_clearance_m(kin, side, q)
                   for q in rng.uniform(lo, hi, (300, 7)))
        assert best <= got[side] + 1e-4, (side, best, got[side])


def test_home_clear_band_is_kit_clean_and_homes_is_not(kin):
    home, clear = mk.load_named_pose("home"), mk.load_named_pose("home_clear")
    for side in ("left", "right"):
        assert _verdict(kin, side, clear["left"], clear["right"]) == (True, "")
        assert _band_fraction(kin, side, clear) >= 0.99, side
    # HOME's own band, for the record: 75.7 % right, 48.7 % left at 4096
    # samples. Pinned loosely so the reason for HOME_CLEAR stays visible.
    assert _band_fraction(kin, "right", home) < 0.85
    assert _band_fraction(kin, "left", home) < 0.60


def test_straight_move_home_to_home_clear_is_guard_clean(kin):
    """Both arms together, sampled at <= 1 deg per joint, as the daemon's
    ``move_joints_both`` checks a single-shot move."""
    home, clear = mk.load_named_pose("home"), mk.load_named_pose("home_clear")
    largest = max(np.abs(clear[s] - home[s]).max() for s in ("left", "right"))
    n = max(1, int(np.ceil(largest / np.radians(1.0))))
    assert n >= 10  # a 10 deg move; float noise may add one sample
    for k in range(1, n + 1):
        t = k / n
        q = {s: home[s] + t * (clear[s] - home[s]) for s in ("left", "right")}
        for side in ("left", "right"):
            ok, why = _verdict(kin, side, q["left"], q["right"])
            assert ok, f"sample {k}/{n}: {why}"
