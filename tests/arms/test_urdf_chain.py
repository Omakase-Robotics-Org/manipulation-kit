"""The numpy IK substrate, on its own terms — no MuJoCo anywhere in this file.

That is the point of the file as much as its contents: these are the checks that
say the shipped default works, and they must run on a machine that has never
heard of a physics engine. The cross-check AGAINST MuJoCo lives next door in
``test_urdf_chain_parity.py`` and skips when it is absent.

The Jacobian is verified against FINITE DIFFERENCES of this chain's own FK
rather than against another implementation, so it stands up even where there is
nothing to compare with: a Jacobian that does not differentiate the FK beside it
is wrong no matter what any other library says.
"""

from __future__ import annotations

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

from manipulation_kit.arms import sides  # noqa: E402
from manipulation_kit.arms.ik import KinematicChain, solve_ik  # noqa: E402
from manipulation_kit.arms.urdf_chain import UrdfChain  # noqa: E402
from manipulation_kit.arms.d1.arm import kinematics as mk  # noqa: E402
from manipulation_kit.guard.urdf_model import UrdfModel  # noqa: E402


@pytest.fixture(scope="module")
def urdf_model():
    if not mk.default_urdf().exists():
        pytest.skip(f"no d1.urdf at {mk.default_urdf()}")
    return UrdfModel(str(mk.default_urdf()))


def _chain(urdf_model, side="left"):
    return UrdfChain(urdf_model, ee_body=sides.EE_BODY[side],
                     joint_names=sides.ARM_JOINTS[side])


def _fd_jacobian(chain, q, h=1e-6):
    """Central-difference 6 x ndof Jacobian of the chain's own FK at ``q``."""
    J = np.zeros((6, chain.ndof))
    for i in range(chain.ndof):
        qp, qm = np.array(q, dtype=float), np.array(q, dtype=float)
        qp[i] += h
        qm[i] -= h
        chain.set_joints(qp)
        p_p, r_p = chain.ee_pose()
        chain.set_joints(qm)
        p_m, r_m = chain.ee_pose()
        J[:3, i] = (p_p - p_m) / (2 * h)
        J[3:, i] = (r_p * r_m.inv()).as_rotvec() / (2 * h)
    chain.set_joints(q)
    return J


# --------------------------------------------------------------------------- #
# 1. the protocol
# --------------------------------------------------------------------------- #
def test_it_is_a_kinematic_chain(urdf_model):
    assert isinstance(_chain(urdf_model), KinematicChain)


@pytest.mark.parametrize("side", ["left", "right"])
def test_shape_and_limits_come_from_the_urdf(urdf_model, side):
    chain = _chain(urdf_model, side)
    assert chain.ndof == sides.JOINTS_PER_ARM
    assert chain.joint_names == sides.ARM_JOINTS[side]
    lo, hi = chain.limits()
    assert lo.shape == hi.shape == (sides.JOINTS_PER_ARM,)
    assert np.all(lo < hi)
    # the values are the URDF's, not a guess: J6 is the tight +-60 deg wrist
    assert np.isclose(lo[5], -1.0472) and np.isclose(hi[5], 1.0472)


def test_set_joints_rejects_the_wrong_length(urdf_model):
    with pytest.raises(ValueError, match="expected 7 joints"):
        _chain(urdf_model).set_joints(np.zeros(6))


def test_a_joint_name_that_is_not_on_the_path_is_refused(urdf_model):
    """Caught at construction, where it is a typo, not at runtime, where it is
    a silently wrong Jacobian."""
    bad = list(sides.ARM_JOINTS["left"])
    bad[3] = "Joint4_L"          # the OTHER arm's elbow
    with pytest.raises(ValueError, match="actuated joints on the path"):
        UrdfChain(urdf_model, ee_body=sides.EE_BODY["left"], joint_names=bad)


def test_an_unknown_link_is_refused(urdf_model):
    with pytest.raises(ValueError, match="no link named"):
        UrdfChain(urdf_model, ee_body="Link7_Q",
                  joint_names=sides.ARM_JOINTS["left"])


# --------------------------------------------------------------------------- #
# 2. FK
# --------------------------------------------------------------------------- #
def test_fk_is_the_urdf_walk_the_guard_already_does(urdf_model):
    """The guard's own stdlib FK and this chain's numpy FK must agree — they
    describe the same robot, and if they drift the solver and the collision
    check stop talking about the same arm."""
    chain = _chain(urdf_model)
    rng = np.random.default_rng(4)
    lo, hi = chain.limits()
    for _ in range(20):
        q = rng.uniform(lo, hi)
        chain.set_joints(q)
        p, _ = chain.ee_pose()
        tfs = urdf_model.link_transforms(
            dict(zip(sides.ARM_JOINTS["left"], q)))
        np.testing.assert_allclose(p, tfs[sides.EE_BODY["left"]].t, atol=1e-12)


def test_the_two_arms_are_mirrored_in_y(urdf_model):
    left, right = _chain(urdf_model, "left"), _chain(urdf_model, "right")
    home = mk.load_home(quiet=True)
    left.set_joints(home["left"])
    right.set_joints(home["right"])
    pl, pr = left.ee_pose()[0], right.ee_pose()[0]
    assert pl[1] > 0 > pr[1]
    assert np.isclose(pl[1], -pr[1], atol=1e-5)


def test_body_xpos_reaches_links_the_ready_probe_scores(urdf_model):
    """``_probe`` asks for the shoulder and the elbow to score a branch."""
    chain = _chain(urdf_model)
    chain.set_joints(np.zeros(7))
    sh = chain.body_xpos(sides.SHOULDER_BODY["left"])
    el = chain.body_xpos(sides.ELBOW_BODY["left"])
    assert sh.shape == el.shape == (3,)
    assert not np.allclose(sh, el)
    # a link OFF this chain resolves too (MuJoCo's shared mjData gives that for
    # free, and the probe must not care which substrate it got)
    assert chain.body_xpos("head_link").shape == (3,)


# --------------------------------------------------------------------------- #
# 3. the Jacobian, against finite differences of the FK above
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("side", ["left", "right"])
def test_jacobian_differentiates_its_own_fk(urdf_model, side):
    chain = _chain(urdf_model, side)
    lo, hi = chain.limits()
    rng = np.random.default_rng(9)
    worst = 0.0
    for _ in range(25):
        q = rng.uniform(lo + 0.1, hi - 0.1)
        chain.set_joints(q)
        worst = max(worst, float(np.max(np.abs(chain.jacobian()
                                               - _fd_jacobian(chain, q)))))
    assert worst < 1e-6, f"analytic vs finite-difference Jacobian off by {worst}"


def test_linear_rows_are_on_top(urdf_model):
    """``solve_ik`` concatenates [position error, rotation error]; a Jacobian
    with the blocks the other way round converges to nonsense."""
    chain = _chain(urdf_model)
    chain.set_joints(np.full(7, 0.3))
    J = chain.jacobian()
    p, _ = chain.ee_pose()
    # the angular rows are unit axes; the linear rows are r x omega, which on
    # this arm are metres-scale, not unit
    assert np.allclose(np.linalg.norm(J[3:], axis=0), 1.0, atol=1e-12)
    assert np.linalg.norm(p) > 0.1


# --------------------------------------------------------------------------- #
# 4. the shared solver drives it
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("side", ["left", "right"])
def test_the_shared_solver_converges_on_this_chain(urdf_model, side):
    chain = _chain(urdf_model, side)
    lo, hi = chain.limits()
    home = mk.load_home(quiet=True)[side]
    q_true = np.clip(home + 0.1, lo, hi)
    chain.set_joints(q_true)
    p_t, r_t = chain.ee_pose()
    q = solve_ik(chain, p_t, r_t, np.clip(home, lo, hi))
    assert q is not None
    chain.set_joints(q)
    p, r = chain.ee_pose()
    assert np.linalg.norm(p - p_t) < 1e-3
    assert np.linalg.norm((r_t * r.inv()).as_rotvec()) < 1e-2


# --------------------------------------------------------------------------- #
# 5. prismatic joints — none on the D1 arm, but the substrate is generic
# --------------------------------------------------------------------------- #
PRISMATIC_URDF = """<?xml version="1.0"?>
<robot name="slider">
  <link name="base"/>
  <link name="lift"/>
  <link name="tool"/>
  <joint name="j_lift" type="prismatic">
    <parent link="base"/><child link="lift"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="0.0" upper="0.4" effort="1" velocity="1"/>
  </joint>
  <joint name="j_yaw" type="revolute">
    <parent link="lift"/><child link="tool"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-1.0" upper="1.0" effort="1" velocity="1"/>
  </joint>
</robot>
"""


@pytest.fixture()
def prismatic(tmp_path):
    p = tmp_path / "slider.urdf"
    p.write_text(PRISMATIC_URDF)
    return UrdfChain(str(p), ee_body="tool", joint_names=("j_lift", "j_yaw"))


def test_prismatic_limits_are_read_in_metres(prismatic):
    lo, hi = prismatic.limits()
    np.testing.assert_allclose(lo, [0.0, -1.0])
    np.testing.assert_allclose(hi, [0.4, 1.0])


def test_a_prismatic_joint_translates_and_does_not_rotate(prismatic):
    prismatic.set_joints([0.0, 0.0])
    p0, r0 = prismatic.ee_pose()
    prismatic.set_joints([0.25, 0.0])
    p1, r1 = prismatic.ee_pose()
    np.testing.assert_allclose(p1 - p0, [0.0, 0.0, 0.25], atol=1e-12)
    np.testing.assert_allclose((r1 * r0.inv()).as_rotvec(), np.zeros(3), atol=1e-12)


def test_prismatic_jacobian_matches_finite_differences(prismatic):
    q = np.array([0.2, 0.4])
    prismatic.set_joints(q)
    J = prismatic.jacobian()
    np.testing.assert_allclose(J[:, 0], [0, 0, 1, 0, 0, 0], atol=1e-12)
    np.testing.assert_allclose(J, _fd_jacobian(prismatic, q), atol=1e-7)


def test_an_unsupported_joint_type_is_refused(tmp_path):
    p = tmp_path / "floaty.urdf"
    p.write_text(PRISMATIC_URDF.replace('type="prismatic"', 'type="floating"'))
    with pytest.raises(ValueError, match="unsupported joint type"):
        UrdfChain(str(p), ee_body="tool", joint_names=("j_yaw",))
