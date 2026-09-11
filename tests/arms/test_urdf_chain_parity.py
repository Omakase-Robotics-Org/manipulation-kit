"""UrdfChain vs MujocoChain — the substrate swap must be a NON-EVENT.

``get_arm_kinematics("d1/arm")`` used to be able to answer only through MuJoCo;
it now walks the URDF in numpy by default, and MuJoCo is one optional substrate
of two. That is only a safe default if the two describe the SAME arm, so this
file measures the difference rather than asserting it away:

* forward kinematics over 500 random in-limit postures per side,
* the 6x7 geometric Jacobian at each of them,
* the joint limits, and the link positions the READY-seed probe scores,
* the deterministic 200-restart READY search, whose result is the posture every
  later solve is biased toward,
* ``solve_ee`` over the same 80-tick trajectory the dx-vr-teleop parity gate
  uses, which deliberately leaves the reachable set so accepted ticks, IK
  failures and guard rejections are all compared.

Tolerances are 1e-6 (m / rad), roughly nine orders of magnitude looser than what
is measured — see the PR for the numbers — because the claim is "same model",
not "same floating-point rounding". Skips when MuJoCo is absent: it is the
OPTIONAL side of this comparison now.
"""

from __future__ import annotations

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")
mujoco = pytest.importorskip("mujoco")
from scipy.spatial.transform import Rotation as R  # noqa: E402

from manipulation_kit.arms import sides  # noqa: E402
from manipulation_kit.arms.urdf_chain import UrdfChain  # noqa: E402
from manipulation_kit.arms.d1.arm import kinematics as mk  # noqa: E402
from manipulation_kit.arms.d1.arm.mujoco_chain import MujocoChain  # noqa: E402
from manipulation_kit.guard.urdf_model import UrdfModel  # noqa: E402

#: how far apart the two substrates may be. Measured worst case on this URDF is
#: ~5e-16 m / ~1.4e-15 rad / ~3e-15 on the Jacobian, i.e. double-precision noise.
TOL = 1e-6

#: postures per side in the FK/Jacobian sweep
SAMPLES = 500


@pytest.fixture(scope="module")
def pair():
    """``side -> (UrdfChain, MujocoChain)`` over one and the same ``d1.urdf``."""
    urdf = mk.default_urdf()
    if not urdf.exists():
        pytest.skip(f"no d1.urdf at {urdf}")
    model = mujoco.MjModel.from_xml_string(urdf.read_text())
    data = mujoco.MjData(model)
    parsed = UrdfModel(str(urdf))
    return {
        side: (UrdfChain(parsed, ee_body=sides.EE_BODY[side],
                         joint_names=sides.ARM_JOINTS[side]),
               MujocoChain(mujoco, model, data, ee_body=sides.EE_BODY[side],
                           joint_names=sides.ARM_JOINTS[side]))
        for side in sides.SIDES
    }


@pytest.mark.parametrize("side", ["left", "right"])
def test_joint_limits_identical(pair, side):
    urdf_chain, mj_chain = pair[side]
    lo_u, hi_u = urdf_chain.limits()
    lo_m, hi_m = mj_chain.limits()
    np.testing.assert_allclose(lo_u, lo_m, rtol=0, atol=0)
    np.testing.assert_allclose(hi_u, hi_m, rtol=0, atol=0)


@pytest.mark.parametrize("side", ["left", "right"])
def test_fk_and_jacobian_agree_over_the_joint_box(pair, side):
    """The whole claim, swept: same end-effector pose, same Jacobian."""
    urdf_chain, mj_chain = pair[side]
    lo, hi = mj_chain.limits()
    rng = np.random.default_rng(0)
    worst_p = worst_r = worst_j = 0.0
    for _ in range(SAMPLES):
        q = rng.uniform(lo, hi)
        urdf_chain.set_joints(q)
        mj_chain.set_joints(q)
        p_u, r_u = urdf_chain.ee_pose()
        p_m, r_m = mj_chain.ee_pose()
        worst_p = max(worst_p, float(np.linalg.norm(p_u - p_m)))
        worst_r = max(worst_r, float(np.linalg.norm((r_m * r_u.inv()).as_rotvec())))
        worst_j = max(worst_j, float(np.max(np.abs(urdf_chain.jacobian()
                                                   - mj_chain.jacobian()))))
    assert worst_p < TOL, f"FK position diverged by {worst_p} m over {SAMPLES}"
    assert worst_r < TOL, f"FK rotation diverged by {worst_r} rad over {SAMPLES}"
    assert worst_j < TOL, f"Jacobian diverged by {worst_j} over {SAMPLES}"


@pytest.mark.parametrize("side", ["left", "right"])
def test_probe_link_positions_agree(pair, side):
    """``body_xpos`` feeds the READY-seed clearance score, so it is load-bearing
    even though it is not part of the KinematicChain protocol."""
    urdf_chain, mj_chain = pair[side]
    lo, hi = mj_chain.limits()
    rng = np.random.default_rng(1)
    for _ in range(20):
        q = rng.uniform(lo, hi)
        urdf_chain.set_joints(q)
        mj_chain.set_joints(q)
        for body in (sides.SHOULDER_BODY[side], sides.ELBOW_BODY[side]):
            np.testing.assert_allclose(urdf_chain.body_xpos(body),
                                       mj_chain.body_xpos(body), atol=TOL)


# --------------------------------------------------------------------------- #
# end to end: the same arm object, built on each substrate
# --------------------------------------------------------------------------- #
@pytest.fixture(scope="module")
def arms():
    if not mk.default_urdf().exists():
        pytest.skip(f"no d1.urdf at {mk.default_urdf()}")
    return {kind: mk.build_kinematics(chain=kind, quiet=True)
            for kind in ("urdf", "mujoco")}


@pytest.mark.parametrize("side", ["left", "right"])
def test_the_ready_seed_search_lands_on_the_same_posture(arms, side):
    """The strongest assertion here: reproducing it needs the limits, the RNG
    seed and draw order, the solver and its tolerances, the collision guard and
    the clearance/forward-jut scoring to agree across both substrates."""
    np.testing.assert_allclose(arms["urdf"].ready(side), arms["mujoco"].ready(side),
                               atol=1e-9)


def _trajectory(side, n=80, seed=7):
    """The dx-vr-teleop parity trajectory: a deterministic walk that leaves the
    reachable set on purpose.

    Mirrored in y per side, unlike dx-vr-teleop's, which starts both arms at
    +0.20: the right arm cannot reach across the body, so the unmirrored walk
    is refused on all 80 ticks and compares nothing but refusals.
    """
    rng = np.random.default_rng(seed)
    p = np.array([0.30, sides.MOUNT_Y_SIGN[side] * 0.20, 0.30])
    r = R.from_euler("xyz", [0.0, 0.0, 0.0])
    out = []
    for _ in range(n):
        p = p + rng.normal(0.0, 0.02, 3)
        r = R.from_rotvec(rng.normal(0.0, 0.05, 3)) * r
        out.append((p.copy(), r))
    return out


@pytest.mark.parametrize("side", ["left", "right"])
def test_guarded_solves_take_the_same_decisions_and_joints(arms, side):
    for arm in arms.values():
        for s in sides.SIDES:
            arm.set_joints(s, arm.home(s))

    accepted = refused = 0
    worst = 0.0
    for i, (p, r) in enumerate(_trajectory(side)):
        res = {k: a.solve_ee(side, p, r) for k, a in arms.items()}
        assert res["urdf"].ok == res["mujoco"].ok, (
            f"tick {i}: urdf={res['urdf'].reason} mujoco={res['mujoco'].reason}")
        assert res["urdf"].reason == res["mujoco"].reason
        np.testing.assert_allclose(arms["urdf"].joints(side),
                                   arms["mujoco"].joints(side), atol=1e-4,
                                   err_msg=f"tick {i}: joints diverged")
        if res["urdf"].ok:
            accepted += 1
            worst = max(worst, float(np.max(np.abs(res["urdf"].q - res["mujoco"].q))))
        else:
            refused += 1
    assert accepted > 0 and refused > 0, (
        f"trajectory covered only one outcome (ok={accepted} refused={refused})")
    assert worst < 1e-4, f"committed joints diverged by {worst} rad"
