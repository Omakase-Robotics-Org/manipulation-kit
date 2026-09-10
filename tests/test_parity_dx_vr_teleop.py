"""THE DIVERGENCE CHECK — is ``manipulation_kit.arms`` still the same code as dx-vr-teleop?

Phase 1 of this migration deliberately leaves the arm code duplicated: it lands
here, and dx-vr-teleop's ``server/`` copy stays untouched. That is only safe if
"have the two copies diverged?" is a QUESTION WITH AN ANSWER rather than a
matter of opinion. This module is that answer.

It runs both implementations against each other on the same D1 description, the
same collision guard and the same input sequences, and asserts they agree
exactly — not approximately. It is the gate for phase 2: deleting
dx-vr-teleop's copy is permitted once this passes on the robot host.

HOW TO RUN IT (it SKIPS silently otherwise, because dx-vr-teleop is a separate
private repo and is not a dependency of this one — the same pattern dx-vr-teleop
already uses in the other direction with ``importorskip("manipulation_kit.hands")``)::

    D1_SDK_DIR=~/d1-sdk \\
    D1_TELEOP_GUARD_CONFIG=~/d1-vr-teleop/guard_config.json \\
    DX_VR_TELEOP_DIR=~/d1-vr-teleop \\
    pytest tests/test_parity_dx_vr_teleop.py -v

Note that only the LEGACY env var name is set above, deliberately: both copies
must respond to the same environment while the duplication lasts, and
:func:`manipulation_kit.arms.safety.envf` honours the ``D1_TELEOP_*`` names for exactly
that reason. If a future edit breaks that, ``test_guard_config_is_shared`` fails.

Baseline recorded at migration time: dx-vr-teleop ``master`` @ ``de8d781``
(= PR #41 merged, tree byte-identical to its tip ``043d521``).
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")
pytest.importorskip("mujoco")
from scipy.spatial.transform import Rotation as R  # noqa: E402

#: dx-vr-teleop checkout to compare against
VRT_DIR = os.environ.get("DX_VR_TELEOP_DIR")

pytestmark = pytest.mark.skipif(
    not VRT_DIR,
    reason="set DX_VR_TELEOP_DIR=<dx-vr-teleop checkout> to run the parity gate",
)

#: the reference commit this gate was written against; reported on failure so a
#: divergence caused by dx-vr-teleop moving on is distinguishable from one
#: caused by an edit here
BASELINE_REF = "de8d781"


# --------------------------------------------------------------------------- #
# fixtures: both implementations, side by side
# --------------------------------------------------------------------------- #
@pytest.fixture(scope="module")
def vrt():
    """dx-vr-teleop's ``server/`` package, importable."""
    server = Path(VRT_DIR) / "server"
    if not (server / "backends.py").exists():
        pytest.skip(f"no server/backends.py under {VRT_DIR}")
    sys.path.insert(0, str(server))
    import backends  # noqa: PLC0415
    import clutch  # noqa: PLC0415
    import filters  # noqa: PLC0415
    return {"backends": backends, "clutch": clutch, "filters": filters}


@pytest.fixture(scope="module")
def theirs(vrt):
    """dx-vr-teleop's SimBackend (the reference implementation)."""
    if not Path(os.environ.get("D1_SDK_DIR", "~/d1-sdk")).expanduser().exists():
        pytest.skip("D1_SDK_DIR does not exist — no d1.urdf to load")
    return vrt["backends"].SimBackend()


@pytest.fixture(scope="module")
def ours():
    """The migrated kinematics."""
    from manipulation_kit.arms import get_arm_kinematics  # noqa: PLC0415
    return get_arm_kinematics("d1/arm", quiet=True)


# --------------------------------------------------------------------------- #
# 1. the model, the guard and the postures
# --------------------------------------------------------------------------- #
def test_guard_config_is_shared(theirs, ours):
    """Both copies must see the SAME collision guard.

    If one has a guard and the other does not, every later comparison is
    meaningless — and in production it would mean the two disagree about what
    is safe.
    """
    assert (theirs._guard is not None) == ours.gate.installed, (
        "one implementation has a collision guard and the other does not; "
        "check D1_SDK_DIR / D1_TELEOP_GUARD_CONFIG")


def test_home_pose_identical(theirs, ours):
    for side in ("left", "right"):
        np.testing.assert_array_equal(theirs._home[side], ours.home(side))


def test_ready_seed_identical(theirs, ours):
    """The 200-restart deterministic READY search must land on the same posture.

    This is the strongest single assertion in the file: reproducing it requires
    the joint limits, the RNG seed and draw order, the IK solver and all its
    tolerances, the guard, and the clearance/forward-jut scoring to ALL match.
    """
    for side in ("left", "right"):
        np.testing.assert_allclose(
            theirs._ready[side], ours.ready(side), rtol=0, atol=0,
            err_msg=f"READY seed diverged for {side} (baseline {BASELINE_REF})")


def test_ee_pose_and_limit_margin_identical(theirs, ours):
    for side in ("left", "right"):
        p_t, r_t = theirs.ee_pose(side)
        p_o, r_o = ours.ee_pose(side)
        np.testing.assert_allclose(p_t, p_o, rtol=0, atol=0)
        np.testing.assert_allclose(r_t.as_quat(), r_o.as_quat(), rtol=0, atol=0)
        assert theirs._limit_margin_deg(side) == ours.limit_margin_deg(side)


# --------------------------------------------------------------------------- #
# 2. the guarded IK, over a trajectory
# --------------------------------------------------------------------------- #
def _trajectory(n=80, seed=7):
    """A deterministic walk that deliberately leaves the reachable set, so the
    comparison covers accepted ticks, IK failures AND guard rejections."""
    rng = np.random.default_rng(seed)
    p = np.array([0.30, 0.20, 0.30])
    r = R.from_euler("xyz", [0.0, 0.0, 0.0])
    out = []
    for _ in range(n):
        p = p + rng.normal(0.0, 0.02, 3)
        r = R.from_rotvec(rng.normal(0.0, 0.05, 3)) * r
        out.append((p.copy(), r))
    return out


@pytest.mark.parametrize("side", ["left", "right"])
def test_guarded_ik_decisions_and_joints_identical(theirs, ours, side):
    """Every tick: same accept/reject, and on accept the same joint vector.

    Both are re-seeded to HOME first so they start from identical state, and
    each tick's outcome feeds the next (the mirror carries state), so a single
    divergence anywhere cascades and cannot be hidden by averaging.
    """
    for s in ("left", "right"):
        theirs._set_joints(s, theirs._home[s])
        ours.set_joints(s, ours.home(s))

    accepted = refused = 0
    for i, (p, r) in enumerate(_trajectory()):
        got_t = bool(theirs.try_move_ee(side, p, r))
        got_o = ours.solve_ee(side, p, r)
        assert got_t == got_o.ok, (
            f"tick {i}: dx-vr-teleop {'accepted' if got_t else 'refused'} but "
            f"manipulation_kit.arms {'accepted' if got_o.ok else 'refused'} "
            f"(reason={got_o.reason}, baseline {BASELINE_REF})")
        np.testing.assert_allclose(
            theirs.joints(side), ours.joints(side), rtol=0, atol=0,
            err_msg=f"tick {i}: joints diverged after a {'pass' if got_t else 'refusal'}")
        accepted += bool(got_t)
        refused += (not got_t)
    # the trajectory must actually exercise both paths, or this proves little
    assert accepted > 0 and refused > 0, (
        f"trajectory did not cover both outcomes (ok={accepted} refused={refused})")


# --------------------------------------------------------------------------- #
# 3. filters
# --------------------------------------------------------------------------- #
def test_one_euro_identical(vrt):
    from manipulation_kit.arms.filters import OneEuro  # noqa: PLC0415
    rng = np.random.default_rng(1)
    a = vrt["filters"].OneEuro(0.5, 3.0)
    b = OneEuro(0.5, 3.0)
    for i in range(200):
        x = rng.normal(0, 1, 3)
        t = i * 0.02
        np.testing.assert_allclose(a(x, t), b(x, t), rtol=0, atol=0)


def test_pose_filter_identical(vrt):
    from manipulation_kit.arms.filters import PoseFilter  # noqa: PLC0415
    rng = np.random.default_rng(2)
    a, b = vrt["filters"].PoseFilter(), PoseFilter()
    for i in range(200):
        p = rng.normal(0, 0.1, 3)
        q = rng.normal(0, 1, 4)
        q = q / np.linalg.norm(q)
        if i % 17 == 0:
            q = -q          # exercise the sign-continuity branch
        pa, qa = a(p, q, i * 0.02)
        pb, qb = b(p, q, i * 0.02)
        np.testing.assert_allclose(pa, pb, rtol=0, atol=0)
        np.testing.assert_allclose(qa, qb, rtol=0, atol=0)


def test_joint_filter_identical(vrt):
    from manipulation_kit.arms.filters import JointFilter  # noqa: PLC0415
    rng = np.random.default_rng(3)
    a, b = vrt["filters"].JointFilter(), JointFilter()
    for i in range(200):
        q = rng.normal(0, 0.3, 7)
        np.testing.assert_allclose(a(q, i * 0.02), b(q, i * 0.02), rtol=0, atol=0)


# --------------------------------------------------------------------------- #
# 4. frame conventions and the clutch
# --------------------------------------------------------------------------- #
def test_frame_conversion_identical(vrt):
    from manipulation_kit.arms.frames import webxr_quat_to_robot, webxr_to_robot  # noqa: PLC0415
    rng = np.random.default_rng(4)
    for _ in range(50):
        p = rng.normal(0, 1, 3)
        q = rng.normal(0, 1, 4)
        q = q / np.linalg.norm(q)
        np.testing.assert_allclose(vrt["clutch"].webxr_to_robot(p),
                                   webxr_to_robot(p), rtol=0, atol=0)
        np.testing.assert_allclose(
            vrt["clutch"].webxr_quat_to_robot(q).as_matrix(),
            webxr_quat_to_robot(q).as_matrix(), rtol=0, atol=0)


def test_clutch_constants_identical(vrt):
    """The clamps are safety limits — a silent difference here is the bug this
    whole migration exists to make impossible."""
    from manipulation_kit.arms import safety  # noqa: PLC0415
    c = vrt["clutch"]
    assert c.MAX_STEP_M == safety.MAX_STEP_M
    assert c.MAX_STEP_RAD == safety.MAX_STEP_RAD
    assert c.PARK_POS_M == safety.PARK_POS_M
    assert c.PARK_ROT_RAD == safety.PARK_ROT_RAD
    assert c.POS_SCALE == safety.POS_SCALE
    assert c.WORKSPACE == safety.WORKSPACE


def test_max_joint_step_identical(vrt):
    """Agreement with master, NOT the literal 0.25 — that is pinned separately in
    ``tests/test_arm_conventions.py``.

    The division is deliberate. dx-vr-teleop ``master`` is the reference
    implementation, so this test's job is to notice when the two sides stop
    agreeing (including when master legitimately moves and we must follow).
    Hard-coding 0.25 here would make it pass while master drifted, destroying
    exactly the signal it exists to give.
    """
    from manipulation_kit.arms import safety  # noqa: PLC0415
    assert vrt["backends"].SimBackend.MAX_JOINT_STEP == safety.MAX_JOINT_STEP_RAD


@pytest.mark.parametrize("strapped", [False, True])
def test_clutch_target_sequence_identical(vrt, strapped):
    """Engage both clutches at one anchor and drive the same controller path.

    Covers the box clamp + anchor-union latch, the step clamp, the parking
    deadband and its release hysteresis (returns None), the accept/reject
    bookkeeping, and — when ``strapped`` — the mount-rotation conjugation.
    """
    from manipulation_kit.arms.targeting import ArmClutch, ClutchTuning  # noqa: PLC0415
    from manipulation_kit.arms import safety  # noqa: PLC0415

    mount = R.from_euler("xyz", [0.3, -0.2, 0.9]) if strapped else None
    a = vrt["clutch"].ArmClutch()
    b = ArmClutch(tuning=ClutchTuning(workspace=safety.WORKSPACE))
    a.mount_rot = mount
    b.mount_rot = mount

    # anchor OUTSIDE the workspace box on purpose, to exercise the union/latch
    ee_p = np.array([0.05, 0.30, 0.20])
    ee_r = R.from_euler("xyz", [0.1, 0.2, 0.3])
    a.engage([0.0, 0.0, 0.0], [0, 0, 0, 1], ee_p, ee_r)
    b.engage([0.0, 0.0, 0.0], [0, 0, 0, 1], ee_p, ee_r)

    rng = np.random.default_rng(11)
    parked = moved = 0
    cp = np.zeros(3)
    cr = R.identity()
    for i in range(300):
        # Mostly near-still (so the parking deadband engages), occasionally a
        # big jump (so the box clamp and the step clamp engage). The rotation
        # must be INCREMENTAL for the same reason the position is: a fresh
        # random orientation every tick never parks.
        big = (i % 9 == 0)
        cp = cp + (rng.normal(0, 0.25, 3) if big else rng.normal(0, 0.0004, 3))
        cr = R.from_rotvec(rng.normal(0, 0.4 if big else 0.0008, 3)) * cr
        qq = cr.as_quat()
        if i % 17 == 0:
            qq = -qq        # exercise quaternion sign continuity
        t = i * 0.02
        ta = a.target(cp, qq, t)
        tb = b.target(cp, qq, t)
        assert (ta is None) == (tb is None), f"tick {i}: parking disagreed"
        if ta is None:
            parked += 1
            continue
        moved += 1
        np.testing.assert_allclose(ta[0], tb[0], rtol=0, atol=0,
                                   err_msg=f"tick {i}: target position diverged")
        np.testing.assert_allclose(ta[1].as_quat(), tb[1].as_quat(), rtol=0, atol=0,
                                   err_msg=f"tick {i}: target rotation diverged")
        # alternate accept/reject so the hold-pose bookkeeping is exercised
        if i % 5:
            a.accept(*ta)
            b.accept(*tb)
        else:
            assert (a.reject() is None) == (b.reject() is None)
            assert a.rejected_streak == b.rejected_streak
    assert parked > 0 and moved > 0, f"parked={parked} moved={moved}"
