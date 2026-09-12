"""``ok`` must mean the solution REACHES the target — the d1-3 press_hand bug.

On 2026-08-25 a press_hand primitive on d1-3 asked for a ~90 deg wrist
reorientation. :func:`manipulation_kit.arms.ik.solve_ik` reported success; the joints it
returned put the end-effector 16 cm and 81 deg away from what was asked for,
with J6 sitting exactly on its -1.0472 limit. Every motion primitive in
d1-inference trusts ``ok`` for every step, so the arm drove confidently to the
wrong pose. Small teleop steps never showed it, which is why it survived this
long.

The mechanism was that the DLS loop iterated an UNBOUNDED joint vector, tested
convergence on it, and only then clipped the winner into the joint limits on the
way out — moving joints AFTER the test that blessed them, and never re-checking
where the moved posture actually points. This module pins the fix at both
layers: the solver only ever converges at a feasible posture, and
:meth:`~manipulation_kit.arms.kinematics.GuardedArm.solve_ee` independently FK-verifies
whatever it is handed before it commits.

WHAT IS NOT BEING FIXED HERE: ``step_clamped``. A solution further than
``MAX_JOINT_STEP_RAD`` is committed PARTIALLY on purpose so fast motion tracks at
bounded joint speed instead of freezing; the consumer loops and re-asks. That is
a different claim from ``ok`` — ``ok`` says the requested pose is achievable,
``step_clamped`` says the arm is not there yet — and both are load-bearing. The
partial step is asserted to still behave exactly as before.
"""

from __future__ import annotations

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")
from scipy.spatial.transform import Rotation as R  # noqa: E402

from manipulation_kit.arms import safety  # noqa: E402
from manipulation_kit.arms.ik import DEFAULT_TUNING, solve_ik  # noqa: E402
from manipulation_kit.arms.kinematics import IK_FAIL, INFEASIBLE, OK  # noqa: E402
from manipulation_kit.arms.d1.arm import kinematics as mk  # noqa: E402


@pytest.fixture(scope="module")
def arm(substrate):
    if not mk.default_urdf().exists():
        pytest.skip(f"no d1.urdf at {mk.default_urdf()}")
    # guard=None / find_ready=False: this file is about the SOLVER's honesty,
    # which must not depend on whether pyguard imports. Same fixture style as
    # test_d1_arm_kinematics.py in this directory. Parametrised over the
    # substrates because the d1-3 bug this file pins was in the SOLVER, and the
    # solver must be honest on whatever chain it is handed.
    return mk.build_kinematics(guard=None, find_ready=False, quiet=True,
                               chain=substrate)


# --------------------------------------------------------------------------- #
# the reference implementation this change replaced
# --------------------------------------------------------------------------- #
def _old_solve_ik(chain, p_target, r_target, q0, *, q_ref=None,
                  tuning=DEFAULT_TUNING):
    """``solve_ik`` exactly as it stood before this change (origin/main).

    Kept verbatim rather than as recorded numbers so the "interior solutions are
    unchanged" claim is re-derived from the algorithm on every run, on whatever
    URDF is installed, instead of being frozen against one machine's output.
    The ONLY difference from the current solver is where the limits are applied:
    here the iterate is unbounded and the winner is clipped on the way out.
    """
    q = np.array(q0, dtype=float)
    q_ref = np.asarray(q0 if q_ref is None else q_ref, dtype=float)
    n = chain.ndof
    eye_n = np.eye(n)
    eye6 = np.eye(6)
    for _ in range(tuning.iters):
        chain.set_joints(q)
        p, r = chain.ee_pose()
        e_p = np.asarray(p_target, dtype=float) - np.asarray(p, dtype=float)
        e_r = (r_target * r.inv()).as_rotvec()
        err = np.concatenate([e_p, e_r])
        if (np.linalg.norm(e_p) < tuning.pos_tol
                and np.linalg.norm(e_r) < tuning.rot_tol):
            lo, hi = chain.limits()
            return np.clip(q, lo, hi)          # <-- the bug
        J = chain.jacobian()
        dq = J.T @ np.linalg.solve(J @ J.T + tuning.damping * eye6, err)
        pull = np.clip(q_ref - q, -tuning.pull_clip, tuning.pull_clip)
        dq = dq + tuning.posture_gain * (eye_n - np.linalg.pinv(J) @ J) @ pull
        q = q + np.clip(dq, -tuning.dq_clip, tuning.dq_clip)
    chain.set_joints(np.array(q0, dtype=float))
    return None


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def _pose_error(chain, q, p_t, r_t):
    """``(position error [m], rotation error [rad])`` of ``q`` against a target."""
    chain.set_joints(np.asarray(q, dtype=float))
    p, r = chain.ee_pose()
    return (float(np.linalg.norm(np.asarray(p_t, dtype=float) - p)),
            float(np.linalg.norm((r_t * r.inv()).as_rotvec())))


def _within_limits(chain, q):
    lo, hi = chain.limits()
    return bool(np.all(np.asarray(q) >= lo) and np.all(np.asarray(q) <= hi))


def _reaches(chain, q, p_t, r_t):
    """The whole contract in one predicate: feasible AND on target."""
    e_p, e_r = _pose_error(chain, q, p_t, r_t)
    return (e_p < DEFAULT_TUNING.pos_tol and e_r < DEFAULT_TUNING.rot_tol
            and _within_limits(chain, q))


def _pinned(chain, q):
    """Indices (1-based, as the joints are named on the robot) sitting exactly
    on a limit — the fingerprint of the clip that used to happen on the way out."""
    lo, hi = chain.limits()
    q = np.asarray(q, dtype=float)
    return [j + 1 for j in range(len(q))
            if abs(q[j] - lo[j]) < 1e-9 or abs(q[j] - hi[j]) < 1e-9]


def _unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


#: The live d1-3 press_hand request, 2026-08-25. Seed joints as read off the
#: arm; the target is the hand rotated to press, built from its own axes so the
#: matrix is exactly orthonormal rather than a transcribed approximation.
LIVE_SEED = np.array([-2.79, 0.98, 2.81, -1.65, 0.3, 1.02, -0.62])
LIVE_P = np.array([0.36, 0.22, 0.34])


def _live_rotation():
    x_e = _unit((0.0, 0.43, 0.90))
    z_e = _unit((0.0, -0.90, 0.43))
    y_e = np.cross(z_e, x_e)
    m = np.column_stack([x_e, y_e, z_e])
    assert np.isclose(np.linalg.det(m), 1.0), "target frame must be right-handed"
    np.testing.assert_allclose(m.T @ m, np.eye(3), atol=1e-12)
    return R.from_matrix(m)


# --------------------------------------------------------------------------- #
# 1. the live case
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("side", ["left", "right"])
def test_the_live_press_hand_request_is_never_answered_with_a_lie(arm, side):
    """The d1-3 pose, at both layers, on both arms.

    Deliberately a DISJUNCTION rather than a fixed expected outcome: whether
    this particular pose is reachable is a property of the URDF and the joint
    limits, and those move. What must never move is that a reported success is
    a real one. (As of the d1-sdk this runs against, the left arm reaches it and
    the right arm does not — both branches are live.)
    """
    r_t = _live_rotation()
    chain = arm._chains[side]

    q = solve_ik(chain, LIVE_P, r_t, LIVE_SEED, q_ref=arm.ready(side))
    assert q is None or _reaches(chain, q, LIVE_P, r_t), (
        f"solve_ik reported success at {_pose_error(chain, q, LIVE_P, r_t)} "
        f"(pos [m], rot [rad]) with joints pinned at {_pinned(chain, q)} — this "
        f"is the d1-3 bug")

    arm.set_joints(side, LIVE_SEED)
    res = arm.solve_ee(side, LIVE_P, r_t)
    assert not res.ok or _within_limits(chain, res.q)
    if res.ok and not res.step_clamped:
        # not clamped => the committed vector IS the solution, so it must reach
        assert _reaches(chain, res.q, LIVE_P, r_t)


def test_the_live_case_is_a_real_regression_on_the_old_algorithm(arm):
    """Guards the test above from quietly becoming a tautology.

    If a future URDF made every pose in this file comfortably interior, the
    disjunctions would still pass while proving nothing. This asserts that the
    algorithm that was replaced is still demonstrably wrong SOMEWHERE on this
    chain — see the sweep at the bottom for the full accounting.
    """
    side = "left"
    chain = arm._chains[side]
    lo, hi = chain.limits()
    rng = np.random.default_rng(0)
    lies = 0
    for _ in range(300):
        q0 = rng.uniform(lo, hi)
        chain.set_joints(rng.uniform(lo, hi))
        p_t, r_t = chain.ee_pose()
        q_old = _old_solve_ik(chain, p_t, r_t, q0, q_ref=q0)
        if q_old is not None and not _reaches(chain, q_old, p_t, r_t):
            lies += 1
    assert lies > 0, ("the pre-fix algorithm no longer misreports on this chain; "
                      "the regression tests here have gone vacuous")


# --------------------------------------------------------------------------- #
# 2. interior solutions are untouched
# --------------------------------------------------------------------------- #
def test_interior_solutions_are_bit_identical_to_the_previous_algorithm(arm):
    """Where the limits never bind, the projection is the identity.

    This is the compatibility claim the whole change rests on: teleop following
    moves a few hundredths of a rad per tick, deep inside the joint box, so it
    must be BIT-identical — not close. Compared against the old loop inlined
    above rather than against recorded numbers, so it re-derives on any URDF.
    """
    side = "left"
    chain = arm._chains[side]
    q_mid = arm.home(side)
    rng = np.random.default_rng(5)

    compared = 0
    for _ in range(40):
        # a small delta, the size a teleop tick actually produces
        chain.set_joints(q_mid + rng.normal(0.0, 0.02, chain.ndof))
        p_t, r_t = chain.ee_pose()

        chain.set_joints(q_mid)
        q_new = solve_ik(chain, p_t, r_t, q_mid, q_ref=q_mid)
        chain.set_joints(q_mid)
        q_old = _old_solve_ik(chain, p_t, r_t, q_mid, q_ref=q_mid)

        assert (q_new is None) == (q_old is None)
        if q_new is None:
            continue
        if _pinned(chain, q_old):
            continue            # limits were active — not an interior solve
        np.testing.assert_array_equal(
            q_new, q_old,
            err_msg="an interior solve changed; the limit projection must be "
                    "inactive away from the limits")
        compared += 1
    assert compared >= 30, f"only {compared} interior solves compared"


# --------------------------------------------------------------------------- #
# 3. a target blocked by a joint limit is refused, not faked
# --------------------------------------------------------------------------- #
@pytest.fixture(scope="module")
def blocked_target(arm):
    """A pose that genuinely CANNOT be held inside the joint limits.

    Built by posing the chain with one joint driven past its limit and taking
    the resulting end-effector pose. That alone is not enough — a 7-DoF arm has
    a null space and usually reaches the same pose on another branch — so the
    candidate is only accepted once the solver has failed to find it from 80
    restarts spread across the whole joint box. That makes ``None`` the CORRECT
    answer here rather than a local minimum, which is what the old algorithm's
    limit-pinned "success" was papering over.
    """
    side = "left"
    chain = arm._chains[side]
    lo, hi = chain.limits()
    home = arm.home(side)
    rng = np.random.default_rng(11)

    for _ in range(400):
        j = int(rng.integers(0, chain.ndof))
        over = float(rng.uniform(0.4, 1.4))
        q_out = np.clip(home + rng.normal(0.0, 0.5, chain.ndof), lo, hi)
        q_out[j] = hi[j] + over if rng.random() < 0.5 else lo[j] - over
        chain.set_joints(q_out)
        p_t, r_t = chain.ee_pose()
        q0 = np.clip(home + rng.normal(0.0, 0.25, chain.ndof), lo, hi)

        # the old algorithm must have called this reachable, and been wrong —
        # otherwise the case does not exercise the bug
        q_old = _old_solve_ik(chain, p_t, r_t, q0, q_ref=home)
        if q_old is None or _reaches(chain, q_old, p_t, r_t):
            continue
        if solve_ik(chain, p_t, r_t, q0, q_ref=home) is not None:
            continue
        probe = np.random.default_rng(0)
        if any(solve_ik(chain, p_t, r_t, s, q_ref=s) is not None
               for s in (probe.uniform(lo, hi) for _ in range(80))):
            continue           # reachable on some other branch — not blocked
        return side, q0, p_t, r_t, q_old
    pytest.skip("no limit-blocked pose found on this URDF")


def test_a_limit_blocked_target_is_refused(arm, blocked_target):
    side, q0, p_t, r_t, _ = blocked_target
    chain = arm._chains[side]
    assert solve_ik(chain, p_t, r_t, q0, q_ref=arm.home(side)) is None

    arm.set_joints(side, q0)
    res = arm.solve_ee(side, p_t, r_t)
    assert not res.ok and res.reason in (IK_FAIL, INFEASIBLE)
    assert res.q is None
    np.testing.assert_array_equal(arm.joints(side), q0)


def test_the_old_algorithm_faked_that_same_target(arm, blocked_target):
    """The other half: this is a REGRESSION test, so show what it regressed."""
    side, _, p_t, r_t, q_old = blocked_target
    chain = arm._chains[side]
    e_p, e_r = _pose_error(chain, q_old, p_t, r_t)
    assert e_p >= DEFAULT_TUNING.pos_tol or e_r >= DEFAULT_TUNING.rot_tol
    assert _pinned(chain, q_old), (
        "the old answer should be sitting on a limit — that clip IS the bug")


# --------------------------------------------------------------------------- #
# 4. the partial step is untouched
# --------------------------------------------------------------------------- #
def test_a_large_feasible_move_still_commits_a_partial_step(arm):
    """``step_clamped`` streaming semantics must survive this change verbatim.

    A far but reachable target: accepted, flagged clamped, and the committed
    vector is the SCALED step along ``q_start -> q_solution`` — not the solution,
    and not a refusal.
    """
    side = "left"
    chain = arm._chains[side]
    q0 = arm.home(side)
    # FEASIBLE is the whole point of this case, so the posture the target is
    # taken from is clipped into the limits FIRST. Taking it from a raw
    # `q0 + 0.9` put J2 at 2.425 rad against a 2.06 limit, i.e. the "large
    # feasible move" was a move to a pose the arm cannot hold — and once the
    # solver stopped lying about those, this test started failing for the
    # reason it was written to prove. (Both these step_clamped cases are red on
    # dx-manipulator PR #31 as filed; fixed here, in the fixture, not the code.)
    lo, hi = chain.limits()
    arm.set_joints(side, np.clip(q0 + 0.9, lo, hi))
    p_t, r_t = arm.ee_pose(side)

    arm.set_joints(side, q0)
    res = arm.solve_ee(side, p_t, r_t)
    assert res.ok and res.reason == OK
    assert res.step_clamped
    assert np.isclose(np.max(np.abs(res.q - q0)), arm.max_joint_step)
    np.testing.assert_array_equal(arm.joints(side), res.q)

    # the committed vector is a partial step toward a solution that DOES reach
    q_full = solve_ik(chain, p_t, r_t, q0, q_ref=arm.ready(side))
    assert q_full is not None and _reaches(chain, q_full, p_t, r_t)
    scale = safety.MAX_JOINT_STEP_RAD / np.max(np.abs(q_full - q0))
    np.testing.assert_allclose(res.q, q0 + (q_full - q0) * scale, rtol=0, atol=1e-12)


def test_a_partial_step_is_not_expected_to_reach_the_target_yet(arm):
    """Nails down what ``ok`` does NOT claim, so a future reader does not
    'fix' the partial step by making solve_ee refuse it."""
    side = "left"
    chain = arm._chains[side]
    q0 = arm.home(side)
    lo, hi = chain.limits()          # feasible target — see the test above
    arm.set_joints(side, np.clip(q0 + 0.9, lo, hi))
    p_t, r_t = arm.ee_pose(side)
    arm.set_joints(side, q0)
    res = arm.solve_ee(side, p_t, r_t)
    assert res.ok and res.step_clamped
    e_p, _ = _pose_error(chain, res.q, p_t, r_t)
    assert e_p > DEFAULT_TUNING.pos_tol, (
        "this case is meant to be mid-flight; if it now lands in one step it no "
        "longer distinguishes ok from step_clamped")


# --------------------------------------------------------------------------- #
# 5. the second wall, on its own
# --------------------------------------------------------------------------- #
def test_solve_ee_refuses_a_solution_that_does_not_reach(arm, monkeypatch):
    """The FK check in ``solve_ee`` must stand up without the solver's help.

    With the projection in place nothing downstream can normally produce an
    unreachable ``q``, so the only way to test the second wall is to inject one
    — which is exactly the future this check exists for: a different solver, an
    analytic seed, a cache handing back something stale.
    """
    side = "left"
    q0 = arm.home(side)
    arm.set_joints(side, q0)
    p_t, r_t = arm.ee_pose(side)

    monkeypatch.setattr("manipulation_kit.arms.kinematics.solve_ik",
                        lambda *a, **kw: q0 + 0.8)
    res = arm.solve_ee(side, p_t, r_t)
    assert not res.ok and res.reason == INFEASIBLE and res.q is None
    np.testing.assert_array_equal(arm.joints(side), q0)


def test_solve_ee_refuses_a_solution_outside_the_joint_limits(arm, monkeypatch):
    side = "left"
    chain = arm._chains[side]
    q0 = arm.home(side)
    arm.set_joints(side, q0)
    p_t, r_t = arm.ee_pose(side)
    lo, _ = chain.limits()

    bad = q0.copy()
    bad[3] = lo[3] - 0.5           # on-target only if the elbow leaves its range
    monkeypatch.setattr("manipulation_kit.arms.kinematics.solve_ik", lambda *a, **kw: bad)
    res = arm.solve_ee(side, p_t, r_t)
    assert not res.ok and res.reason == INFEASIBLE
    np.testing.assert_array_equal(arm.joints(side), q0)


# --------------------------------------------------------------------------- #
# 6. the property, swept
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("side", ["left", "right"])
def test_a_reported_solution_always_reaches_the_target(arm, side):
    """THE contract, over a wide sweep, with the old algorithm scored beside it.

    Both solvers are driven on identical (seed, target) pairs. The assertion is
    one-sided on purpose — the new solver is allowed to find FEWER solutions
    (refusing is the honest answer to a pose it cannot reach) but is allowed
    ZERO wrong ones.
    """
    chain = arm._chains[side]
    lo, hi = chain.limits()
    rng = np.random.default_rng(0)

    new_ok = new_bad = old_ok = old_bad = 0
    for _ in range(600):
        q0 = rng.uniform(lo, hi)
        chain.set_joints(rng.uniform(lo, hi))
        p_t, r_t = chain.ee_pose()

        chain.set_joints(q0)
        q_new = solve_ik(chain, p_t, r_t, q0, q_ref=q0)
        chain.set_joints(q0)
        q_old = _old_solve_ik(chain, p_t, r_t, q0, q_ref=q0)

        if q_new is not None:
            new_ok += 1
            if not _reaches(chain, q_new, p_t, r_t):
                new_bad += 1
        if q_old is not None:
            old_ok += 1
            if not _reaches(chain, q_old, p_t, r_t):
                old_bad += 1

    assert new_bad == 0, (
        f"solve_ik reported {new_bad}/{new_ok} solutions that do not reach "
        f"their target")
    assert old_bad > 0, "the sweep no longer reproduces the old bug"
    # the projection also CONVERGES better: keeping the iterate feasible stops
    # it wandering off into postures the arm cannot hold and never coming back
    assert new_ok - new_bad > old_ok - old_bad, (
        f"honest solutions regressed: {new_ok - new_bad} now vs "
        f"{old_ok - old_bad} before")
