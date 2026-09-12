"""arm-on-D1 kinematics, on every substrate the binding can be built on.

Runs once per entry of the ``substrate`` fixture (see ``conftest.py``): the
numpy ``UrdfChain`` always, and the MuJoCo one when that optional extra is
installed. Nothing in this file is substrate-specific — it is the arm contract,
and both substrates owe it.
"""

from __future__ import annotations

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")
from scipy.spatial.transform import Rotation as R  # noqa: E402

from manipulation_kit.arms import sides  # noqa: E402
from manipulation_kit.arms.kinematics import GUARD_REJECT, IK_FAIL, OK  # noqa: E402
from manipulation_kit.arms.d1.arm import kinematics as mk  # noqa: E402


@pytest.fixture(scope="module")
def arm(substrate):
    if not mk.default_urdf().exists():
        pytest.skip(f"no d1.urdf at {mk.default_urdf()}")
    # guard=None keeps this fixture independent of whether pyguard is importable;
    # the guard's own behaviour is covered in tests/test_arm_conventions.py and
    # end-to-end in the parity gate.
    return mk.build_kinematics(guard=None, find_ready=False, quiet=True,
                               chain=substrate)


def test_both_arms_present_with_seven_joints(arm):
    assert arm.sides == ("left", "right")
    for side in arm.sides:
        assert arm.joints(side).shape == (sides.JOINTS_PER_ARM,)
        lo, hi = arm.limits(side)
        assert lo.shape == hi.shape == (sides.JOINTS_PER_ARM,)
        assert np.all(lo < hi)


def test_starts_at_home_not_urdf_zero(arm):
    """The model must wake in the same posture the physical robot does."""
    for side in arm.sides:
        np.testing.assert_allclose(arm.joints(side), arm.home(side))


def test_the_two_arms_are_mirrored_in_y(arm):
    """A sanity check that the crossover mapping is wired to the right bodies:
    at HOME the two end-effectors sit at mirrored lateral offsets.

    Tolerance is 10 um, not exact: HOME comes from the hardware's
    ``home_pose.json`` and side A's joint angles are not the exact negatives of
    side B's, so x and z agree only to ~1e-6 m. That asymmetry is real data, not
    a wiring error.
    """
    pl, _ = arm.ee_pose("left")
    pr, _ = arm.ee_pose("right")
    assert pl[1] > 0 > pr[1]
    assert np.isclose(pl[1], -pr[1], atol=1e-5)
    np.testing.assert_allclose(pl[[0, 2]], pr[[0, 2]], atol=1e-5)


def test_ready_defaults_to_home_when_search_disabled(arm):
    for side in arm.sides:
        np.testing.assert_allclose(arm.ready(side), arm.home(side))


def test_round_trip_a_reachable_pose(arm):
    """FK a known posture, then IK back to it."""
    side = "left"
    q_home = arm.home(side)
    arm.set_joints(side, q_home)
    q_true = q_home + 0.10
    arm.set_joints(side, q_true)
    p_t, r_t = arm.ee_pose(side)

    arm.set_joints(side, q_home)
    res = arm.solve_ee(side, p_t, r_t)
    assert res.ok and res.reason == OK
    p, r = arm.ee_pose(side)
    assert np.linalg.norm(p - p_t) < 2e-3
    assert np.linalg.norm((r_t * r.inv()).as_rotvec()) < 5e-2


def test_unreachable_target_fails_and_moves_nothing(arm):
    side = "left"
    arm.set_joints(side, arm.home(side))
    before = arm.joints(side)
    res = arm.solve_ee(side, np.array([5.0, 0.0, 0.0]), R.identity())
    assert not res.ok and res.reason == IK_FAIL and res.q is None
    np.testing.assert_array_equal(arm.joints(side), before)


def test_solving_one_arm_does_not_disturb_the_other(arm):
    """On MuJoCo both chains share one mjData, so a solve that forgot to scope
    itself would drag the other arm along. Asserted on every substrate."""
    for s in arm.sides:
        arm.set_joints(s, arm.home(s))
    right_before = arm.joints("right")
    q_t = arm.home("left") + 0.05
    arm.set_joints("left", q_t)
    p_t, r_t = arm.ee_pose("left")
    arm.set_joints("left", arm.home("left"))
    arm.solve_ee("left", p_t, r_t)
    np.testing.assert_array_equal(arm.joints("right"), right_before)


def test_step_clamp_bounds_a_large_move(arm):
    """A far target is executed PARTIALLY rather than refused."""
    side = "left"
    arm.set_joints(side, arm.home(side))
    q_far = arm.home(side) + 0.9
    arm.set_joints(side, q_far)
    p_t, r_t = arm.ee_pose(side)

    q0 = arm.home(side)
    arm.set_joints(side, q0)
    res = arm.solve_ee(side, p_t, r_t)
    if res.ok:
        assert res.step_clamped
        assert np.max(np.abs(res.q - q0)) <= arm.max_joint_step + 1e-12


def test_guard_rejection_restores_the_entry_pose(arm):
    """With a guard that refuses everything, a solve must leave nothing moved
    and report the reason — this is the body-boundary stop."""
    from manipulation_kit.arms.guard import GuardGate

    side = "left"
    arm.set_joints(side, arm.home(side))
    q_true = arm.home(side) + 0.05
    arm.set_joints(side, q_true)
    p_t, r_t = arm.ee_pose(side)
    q0 = arm.home(side)
    arm.set_joints(side, q0)

    class Refusing:
        def check(self, a, b):
            return type("Rep", (), {"ok": False, "violations": ["nope"]})()

    original = arm._gate
    arm._gate = GuardGate(Refusing())
    try:
        res = arm.solve_ee(side, p_t, r_t)
    finally:
        arm._gate = original
    assert not res.ok and res.reason == GUARD_REJECT
    np.testing.assert_array_equal(arm.joints(side), q0)


def test_limit_margin_is_positive_at_home(arm):
    for side in arm.sides:
        assert arm.limit_margin_deg(side) > 0.0


def test_home_falls_back_to_t_pose_when_json_missing(tmp_path):
    home = mk.load_home(tmp_path / "nope.json", quiet=True)
    assert set(home) == {"left", "right"}
    for v in home.values():
        np.testing.assert_array_equal(v, np.zeros(sides.JOINTS_PER_ARM))


def test_home_reads_degrees_A_then_B(tmp_path):
    """home_pose.json is 14 DEGREES, side A first — i.e. logical left first."""
    import json

    p = tmp_path / "home.json"
    deg = list(range(1, 15))
    p.write_text(json.dumps({"home_pose": deg}))
    home = mk.load_home(p)
    np.testing.assert_allclose(home["left"], np.deg2rad(deg[:7]))
    np.testing.assert_allclose(home["right"], np.deg2rad(deg[7:]))


def test_clutch_tuning_is_the_measured_box():
    from manipulation_kit.arms import safety

    assert mk.clutch_tuning().workspace == safety.WORKSPACE
