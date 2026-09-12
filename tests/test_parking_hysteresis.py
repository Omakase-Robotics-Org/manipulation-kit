"""The parking deadband has TWO edges — the twitches at the END of a move.

Field report, Japan team 2026-08-16: in VR teleop the arm moves in 2-3 small
steps AFTER the operator has stopped their hand (クイクイ). It is the same
mechanism as the slow-motion プルプル that :data:`manipulation_kit.arms.safety.PARK_POS_M`
was added for, one deadband edge further out: the One-Euro filter runs its
lowest cutoff at low speed, so it keeps converging for several ticks after the
hand itself has stopped, and the tail of that convergence arrives at the clutch
as a handful of discrete steps just OVER 2.5 mm. Each one is a fresh IK solve
and each solve may dither the joints inside ``IK_POS_TOL``.

A single-edged deadband cannot swallow that, because its edge is exactly where
the stopping hand comes to rest. So leaving the deadband now costs
``park_release_mult`` x entering it, and that is what is pinned here.

The filter is switched OFF in these tests on purpose: the deadband arithmetic is
what is under test, and with smoothing in the loop no tick has an exactly known
deviation. The filter's contribution (that the residual exists at all) is
covered by dx-vr-teleop's ``test_clutch.py``.
"""

from __future__ import annotations

import dataclasses

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")
from scipy.spatial.transform import Rotation as R  # noqa: E402

from manipulation_kit.arms import safety  # noqa: E402
from manipulation_kit.arms.targeting import ArmClutch, ClutchTuning  # noqa: E402

#: engage anchors: controller at the origin, EE inside the workspace box, both
#: orientations identity — so a deviation in the assertions below IS the
#: quantity the deadband compares.
CTRL0 = np.zeros(3)
Q0 = np.array([0.0, 0.0, 0.0, 1.0])
EE0_P = np.array([0.30, 0.20, 0.30])

PARK_POS = safety.PARK_POS_M
PARK_ROT = safety.PARK_ROT_RAD
RELEASE = safety.PARK_RELEASE_MULT


def _clutch(**overrides) -> ArmClutch:
    tuning = dataclasses.replace(ClutchTuning(workspace=safety.WORKSPACE),
                                 **overrides)
    c = ArmClutch(tuning=tuning)
    c.engage(CTRL0, Q0, EE0_P, R.identity())
    c.filter_enabled = False
    return c


def _ctrl_p(dx: float) -> np.ndarray:
    """The controller position whose unfiltered target sits ``dx`` m along +x.

    WebXR -z is robot +x and the clutch amplifies by ``POS_SCALE``.
    """
    return np.array([0.0, 0.0, -dx / safety.POS_SCALE])


def _ctrl_q(angle: float) -> np.ndarray:
    """A controller orientation ``angle`` rad from the anchor.

    ``WEBXR2ROBOT`` is a proper rotation and the frame change conjugates by it,
    so the angle survives into the robot frame unchanged.
    """
    return R.from_rotvec([0.0, angle, 0.0]).as_quat()


def _park(c: ArmClutch, t: float = 0.02) -> None:
    """Drive one tick well inside the entry deadband, so the clutch parks."""
    assert c.target(_ctrl_p(0.2 * PARK_POS), Q0, t) is None, "did not park"


# --------------------------------------------------------------------------- #
# the release edge
# --------------------------------------------------------------------------- #
def test_a_parked_hand_holds_through_deviations_inside_the_release_band():
    """THE BUG. 1.5x the entry threshold is over the old single edge and under
    the new release edge: it is what the filter's convergence tail looks like
    after the hand stops, and it must not reach the IK."""
    c = _clutch()
    _park(c)
    for i in range(5):
        assert c.target(_ctrl_p(1.5 * PARK_POS), Q0, 0.04 + 0.02 * i) is None, (
            f"tick {i}: a {1.5 * PARK_POS * 1000:.2f} mm residual re-issued a "
            f"target — the deadband still has one edge")
    # and the hold pose is untouched: the arm is holding, not creeping
    np.testing.assert_allclose(c.last_target_p, EE0_P, rtol=0, atol=0)


def test_a_parked_hand_is_released_once_the_deviation_clears_the_band():
    """Hysteresis must not become a freeze: real motion still gets through, and
    the target it gets is the full deviation, not a clipped one."""
    c = _clutch()
    _park(c)
    assert c.target(_ctrl_p(1.5 * PARK_POS), Q0, 0.04) is None
    dx = 1.2 * RELEASE * PARK_POS
    tgt = c.target(_ctrl_p(dx), Q0, 0.06)
    assert tgt is not None, "held past the release threshold — the arm would freeze"
    np.testing.assert_allclose(tgt[0], EE0_P + [dx, 0.0, 0.0], rtol=0, atol=1e-12)
    # released state: the next small deviation is measured against the ENTRY
    # threshold again, so a re-park needs the hand to be still
    c.accept(*tgt)
    assert c.target(_ctrl_p(dx + 1.5 * PARK_POS), Q0, 0.08) is not None


def test_the_release_band_applies_to_rotation_too():
    """A wrist that keeps rotating inside the band is the same failure; the
    release is an OR, so either quantity clearing its band releases."""
    c = _clutch()
    _park(c)
    assert c.target(CTRL0, _ctrl_q(1.5 * PARK_ROT), 0.04) is None
    tgt = c.target(CTRL0, _ctrl_q(1.2 * RELEASE * PARK_ROT), 0.06)
    assert tgt is not None
    assert abs(float(np.linalg.norm(tgt[1].as_rotvec()))
               - 1.2 * RELEASE * PARK_ROT) < 1e-9


def test_either_band_alone_is_enough_to_release():
    """The two bands are ORed on release exactly as they are ANDed on entry: a
    translation clearing its band releases a hold whose rotation has not moved."""
    c = _clutch()
    _park(c)
    tgt = c.target(_ctrl_p(1.2 * RELEASE * PARK_POS), _ctrl_q(0.2 * PARK_ROT), 0.04)
    assert tgt is not None


# --------------------------------------------------------------------------- #
# the state, and the semantics it must not break
# --------------------------------------------------------------------------- #
def test_a_reject_leaves_the_parked_state_so_the_boundary_keeps_retrying():
    """Pre-existing rule: after a guard reject the clutch re-issues the held
    target so the arm resumes the moment the boundary clears. The parked STATE
    must not survive a reject and re-freeze it."""
    c = _clutch()
    _park(c)
    assert c._parked
    c.reject()
    tgt = c.target(_ctrl_p(0.2 * PARK_POS), Q0, 0.04)
    assert tgt is not None, "parked while rejected — the arm would freeze"
    assert not c._parked


def test_engage_and_disengage_clear_the_parked_state():
    """A release threshold left over from the last session must not decide the
    first motion of the next one."""
    c = _clutch()
    _park(c)
    assert c._parked
    c.disengage()
    assert not c._parked
    c.engage(CTRL0, Q0, EE0_P, R.identity())
    assert not c._parked


def test_a_release_multiplier_of_one_restores_the_single_edged_deadband():
    """The escape hatch, and the reason the migration's byte-identical gates
    still have something to compare against: at 1.0 the two edges coincide and
    the behaviour is the pre-hysteresis one."""
    c = _clutch(park_release_mult=1.0)
    _park(c)
    assert c.target(_ctrl_p(1.5 * PARK_POS), Q0, 0.04) is not None


def test_the_multiplier_is_a_tuning_field_defaulting_to_the_shared_constant():
    tun = ClutchTuning(workspace=safety.WORKSPACE)
    assert tun.park_release_mult == safety.PARK_RELEASE_MULT == 2.0


# --------------------------------------------------------------------------- #
# the whole point, end to end
# --------------------------------------------------------------------------- #
def test_a_hand_that_stops_issues_fewer_targets_than_it_used_to():
    """The filter back ON, a hand that moves slowly and then stops DEAD: count
    the targets issued after the stop, single-edged vs hysteretic.

    This is the reported symptom in one number. Note WHERE the saving comes
    from: the first ticks after the stop carry a residual of several mm and are
    real tracking (the filter catching up to where the hand actually is) — those
    still get through, and should. What the second edge removes is the ticks
    AFTER the clutch has parked, where a sub-threshold residual accumulates
    against a frozen hold pose until it trips the single edge again, and each
    trip is one visible twitch of a hand that stopped moving long ago.
    """
    def _issued_after_stop(mult):
        c = _clutch(park_release_mult=mult)
        c.filter_enabled = True
        t = 0.0
        hand = np.zeros(3)
        for i in range(25):                            # 0.5 s of slow motion
            t += 0.02
            hand = np.array([0.0, 0.0, -0.08 * t])     # 0.08 m/s toward robot +x
            tgt = c.target(hand, Q0, t)
            if tgt is not None:
                c.accept(*tgt)
        issued = 0
        for i in range(50):                            # 1 s perfectly still
            t += 0.02
            tgt = c.target(hand, Q0, t)
            if tgt is not None:
                issued += 1
                c.accept(*tgt)
        return issued

    before = _issued_after_stop(1.0)
    after = _issued_after_stop(safety.PARK_RELEASE_MULT)
    assert before >= 2, (
        f"the fixture no longer reproduces the post-stop twitching ({before} "
        f"targets after the stop; the field report is 2-3 movements)")
    assert after < before, (
        f"hysteresis did not reduce the post-stop solves ({after} vs {before})")
