"""The swept-path guard: d1-firmwared's single-shot joint-move check, in the kit.

``move_joints_both`` samples the straight joint-space line from the arms'
pose to the targets at 1 degree per joint and refuses when any sample is
inside a guard margin. ``GuardedArm.swept_path_violation`` is that check with
the kit's verdict (``posture_violation`` + the motion guard) at every sample.
The endpoint verdict alone cannot see a line that dips into a margin between
two clean postures; these tests pin one such pair.
"""

from __future__ import annotations

import numpy as np
import pytest

from manipulation_kit.arms import safety
from manipulation_kit.arms.kinematics import (swept_path_samples,
                                              swept_sample_count)
from manipulation_kit.arms.d1.arm import kinematics as mk

#: Right-arm (arm B) endpoints [deg], both kit-clean (19.7 / 26.0 mm from
#: torso_belly against the 5 mm margin), whose straight line brings Link5_L
#: within 2 mm of torso_belly from its third sample. Found by random search
#: around HOME; integer degrees so the fixture is exact.
CROSSING_FROM_DEG = [12.0, 71.0, -108.0, -124.0, -83.0, 23.0, 36.0]
CROSSING_TO_DEG = [-1.0, 84.0, -122.0, -113.0, -75.0, 26.0, 23.0]


@pytest.fixture(scope="module")
def kin():
    arm = mk.build_kinematics(find_ready=False, quiet=True)
    if not arm.gate.installed:
        pytest.skip("the motion guard is not available")
    return arm


def _pose(kin, right_deg=None, left=None):
    return {"left": kin.home("left") if left is None else np.asarray(left, float),
            "right": (kin.home("right") if right_deg is None
                      else np.radians(right_deg))}


def _endpoint_clean(kin, pose):
    """The per-send endpoint verdict the swept check extends."""
    return (kin.posture_violation("right", pose["right"]) is None
            and kin.posture_violation("left", pose["left"]) is None
            and kin.gate.ok(pose["left"], pose["right"]))


# --------------------------------------------------------------------------- #
# step-count arithmetic (no guard)
# --------------------------------------------------------------------------- #

def _move(deg_left=(), deg_right=()):
    q0 = {"left": np.zeros(7), "right": np.zeros(7)}
    q1 = {"left": np.zeros(7), "right": np.zeros(7)}
    for j, d in deg_left:
        q1["left"][j] = np.radians(d)
    for j, d in deg_right:
        q1["right"][j] = np.radians(d)
    return q0, q1


@pytest.mark.parametrize("move, step, n", [
    (_move(), 1.0, 1),                                  # no motion: the target is still checked
    (_move(deg_right=[(1, 0.4)]), 1.0, 1),
    (_move(deg_right=[(1, 1.0)]), 1.0, 1),
    (_move(deg_right=[(1, 1.01)]), 1.0, 2),
    (_move(deg_left=[(0, -7.5)]), 1.0, 8),
    (_move(deg_left=[(0, 3.0)], deg_right=[(6, -12.2)]), 1.0, 13),  # the larger arm decides
    (_move(deg_right=[(3, 10.0)]), 0.5, 20),
])
def test_sample_count_is_ceil_of_the_largest_joint_delta(move, step, n):
    q0, q1 = move
    assert swept_sample_count(q0, q1, step_deg=step) == n
    samples = swept_path_samples(q0, q1, step_deg=step)
    assert len(samples) == n
    for side in ("left", "right"):
        # the target last, the start not included, and no joint moves more
        # than one step between consecutive checked postures
        np.testing.assert_allclose(samples[-1][side], q1[side])
        chain = [q0[side]] + [s[side] for s in samples]
        assert np.degrees(np.abs(np.diff(chain, axis=0))).max() <= step + 1e-9


def test_the_default_step_is_the_daemons_one_degree():
    assert safety.SWEPT_PATH_STEP_DEG == 1.0
    q0, q1 = _move(deg_right=[(2, 14.3)])
    assert swept_sample_count(q0, q1) == 15


@pytest.mark.parametrize("bad", [0.0, -1.0, float("nan"), float("inf")])
def test_a_nonsense_step_is_refused(bad):
    q0, q1 = _move(deg_right=[(0, 5.0)])
    with pytest.raises(ValueError, match="step_deg"):
        swept_sample_count(q0, q1, step_deg=bad)


# --------------------------------------------------------------------------- #
# the verdict
# --------------------------------------------------------------------------- #

def test_clean_endpoints_whose_line_crosses_the_belly_margin_are_refused(kin):
    a = _pose(kin, CROSSING_FROM_DEG)
    b = _pose(kin, CROSSING_TO_DEG)
    # the endpoint verdict passes both ...
    assert _endpoint_clean(kin, a) and _endpoint_clean(kin, b)
    # ... and the swept path does not, in either direction
    why = kin.swept_path_violation(a, b)
    assert why is not None
    assert why.startswith("path at sample 3/14 (")
    assert "arm B J1=+9.2" in why and "arm A" not in why   # only moving joints
    assert "Link5_L" in why and "torso_belly" in why
    assert not kin.swept_path_ok(a, b)
    assert not kin.swept_path_ok(b, a)


def test_a_clean_line_passes(kin):
    home = _pose(kin)
    out = dict(home, right=home["right"] + np.radians([0, -5, 0, 0, 0, 0, 0]))
    assert kin.swept_path_violation(home, out) is None
    assert kin.swept_path_ok(out, home)
    # a coarser step is a weaker check, never a different API
    assert kin.swept_path_ok(home, out, step_deg=5.0)


def test_an_unclean_target_is_refused_at_the_sample_it_enters(kin):
    home = _pose(kin)
    deeper = dict(home, right=home["right"] + np.radians([0, 10, 0, 0, 0, 0, 0]))
    why = kin.swept_path_violation(home, deeper)
    assert why.startswith("path at sample 9/10 (arm B J2=+96.4)"), why
    assert "Link4_L" in why and "torso_belly" in why


def test_a_target_outside_the_joint_box_is_refused_before_sampling(kin):
    home = _pose(kin)
    far = dict(home, right=home["right"].copy())
    far["right"][1] = np.radians(130.0)            # J2 box is +/-118 deg
    why = kin.swept_path_violation(home, far)
    assert why.startswith("target outside joint limits: arm B J2=+130.0")


def test_the_escape_rule_lets_an_arm_inside_a_margin_move_out_but_not_deeper(kin):
    home = _pose(kin)
    inside = dict(home, right=home["right"] + np.radians([0, 9, 0, 0, 0, 0, 0]))
    assert not _endpoint_clean(kin, inside)
    # out, towards HOME: every sample is still inside at first, but no
    # clearance gets smaller than at the start
    assert kin.swept_path_violation(inside, home) is None
    # deeper: refused, and the reason says why the escape did not apply
    deeper = dict(inside, right=inside["right"] + np.radians([0, 1, 0, 0, 0, 0, 0]))
    why = kin.swept_path_violation(inside, deeper)
    assert why.startswith("path at sample 1/1 (arm B J2=+97.4): the arm starts "
                          "inside the guard's margins"), why
    assert "moves the body clearance closer" in why


def test_the_endpoint_verdict_is_the_one_sample_case(kin):
    """Where there is no line to sweep (a zero move), the swept check is the
    endpoint verdict itself."""
    for right in (CROSSING_FROM_DEG, CROSSING_TO_DEG):
        p = _pose(kin, right)
        assert kin.swept_path_ok(p, p) == _endpoint_clean(kin, p)
    mid = _pose(kin, list((np.array(CROSSING_FROM_DEG) + CROSSING_TO_DEG) / 2))
    assert not _endpoint_clean(kin, mid)


def test_the_samples_are_what_the_verdict_visits(kin):
    """``swept_path_samples`` is the mirror's input: the kit's per-sample
    verdict over it reproduces ``swept_path_violation``."""
    a = _pose(kin, CROSSING_FROM_DEG)
    b = _pose(kin, CROSSING_TO_DEG)
    first_bad = next(k for k, q in enumerate(swept_path_samples(a, b), start=1)
                     if not _endpoint_clean(kin, q))
    assert kin.swept_path_violation(a, b).startswith(f"path at sample {first_bad}/")
