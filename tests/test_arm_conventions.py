"""The side crossover, the safety constants, and the guard's fail-closed rule.

These are the numbers and conventions that were duplicated across repos. Pinning
them here is what makes the duplication detectable rather than silent.
"""

from __future__ import annotations

import pytest

from manipulation_kit.arms import safety, sides


# --------------------------------------------------------------------------- #
# the crossover
# --------------------------------------------------------------------------- #
def test_logical_left_is_urdf_R_is_sdk_A():
    """The trap, pinned. Logical left = URDF ``_R`` = SDK side A = physical LEFT.

    Every consumer has got this wrong at least once. If someone "fixes" the
    inversion because it looks like a typo, this test fails loudly.
    """
    assert sides.URDF_SUFFIX["left"] == "R"
    assert sides.URDF_SUFFIX["right"] == "L"
    assert sides.SDK_SIDE["left"] == "A"
    assert sides.SDK_SIDE["right"] == "B"
    assert sides.EE_BODY["left"] == "Link7_R"
    assert sides.EE_BODY["right"] == "Link7_L"


def test_wire_order_is_A_then_B():
    assert sides.SIDES == ("left", "right")
    assert [sides.SDK_SIDE[s] for s in sides.SIDES] == ["A", "B"]


def test_joint_names_are_proximal_to_distal():
    assert sides.ARM_JOINTS["left"] == tuple(f"Joint{i}_R" for i in range(1, 8))
    assert sides.ARM_JOINTS["right"] == tuple(f"Joint{i}_L" for i in range(1, 8))
    assert all(len(v) == sides.JOINTS_PER_ARM for v in sides.ARM_JOINTS.values())


def test_shoulder_and_elbow_bodies_follow_the_same_suffix():
    assert sides.SHOULDER_BODY["left"] == "Link2_R"
    assert sides.ELBOW_BODY["right"] == "Link4_L"


def test_mount_y_sign_is_plus_for_logical_left():
    """+y is robot-left in the base frame, and logical left is physical left."""
    assert sides.MOUNT_Y_SIGN["left"] == 1.0
    assert sides.MOUNT_Y_SIGN["right"] == -1.0


def test_other_and_check():
    assert sides.other("left") == "right"
    assert sides.other("right") == "left"
    assert sides.check("left") == "left"
    with pytest.raises(ValueError, match="side must be one of"):
        sides.check("A")
    with pytest.raises(ValueError):
        sides.other("middle")


# --------------------------------------------------------------------------- #
# the safety numbers
# --------------------------------------------------------------------------- #
def test_clamp_constants_pinned():
    assert safety.MAX_STEP_M == 0.03
    assert safety.MAX_STEP_RAD == 0.12
    assert safety.PARK_POS_M == 0.0025
    assert safety.PARK_ROT_RAD == 0.012
    assert safety.PARK_RELEASE_MULT == 2.0
    assert safety.POS_SCALE == 1.5
    assert safety.MAX_JOINT_STEP_RAD == 0.25


def test_workspace_box_pinned():
    """The x-floor is 0.15, NOT 0 — reaching x<0.15 near the midline commands
    the hand INTO the torso keep-out and the guard rejects every such tick."""
    assert safety.WORKSPACE == {"x": (0.15, 0.55),
                                "y": (-0.72, 0.72),
                                "z": (0.05, 0.85)}


def test_ik_and_seed_constants_pinned():
    assert (safety.IK_ITERS, safety.IK_POS_TOL, safety.IK_ROT_TOL) == (60, 2e-3, 5e-2)
    assert safety.IK_DAMPING == 1e-4
    assert safety.IK_POSTURE_GAIN == 0.15
    assert (safety.IK_PULL_CLIP, safety.IK_DQ_CLIP) == (0.1, 0.2)
    assert safety.READY_SEED_SAMPLES == 200
    assert safety.READY_SEED_RNG == 0
    assert safety.READY_SEED_IK_ITERS == 50
    assert safety.READY_SEED_FWD_W == 0.6


def test_filter_constants_pinned():
    assert (safety.POS_MIN_CUTOFF, safety.POS_BETA) == (0.5, 3.0)
    assert (safety.ROT_MIN_CUTOFF, safety.ROT_BETA) == (1.0, 1.5)
    assert (safety.JOINT_MIN_CUTOFF, safety.JOINT_BETA) == (1.0, 10.0)
    assert safety.D_CUTOFF == 1.0


# --------------------------------------------------------------------------- #
# env resolution — both copies must respond to the SAME environment
# --------------------------------------------------------------------------- #
def test_envf_prefers_new_name_then_legacy_then_default(monkeypatch):
    monkeypatch.delenv("OMAKASE_ARM_T", raising=False)
    monkeypatch.delenv("D1_TELEOP_T", raising=False)
    assert safety.envf("OMAKASE_ARM_T", 1.0, legacy="D1_TELEOP_T") == 1.0

    monkeypatch.setenv("D1_TELEOP_T", "2.5")
    assert safety.envf("OMAKASE_ARM_T", 1.0, legacy="D1_TELEOP_T") == 2.5

    monkeypatch.setenv("OMAKASE_ARM_T", "3.5")
    assert safety.envf("OMAKASE_ARM_T", 1.0, legacy="D1_TELEOP_T") == 3.5


def test_envf_without_legacy_name():
    assert safety.envf("OMAKASE_ARM_DEFINITELY_UNSET_XYZ", 7.0) == 7.0


# --------------------------------------------------------------------------- #
# the guard gate
# --------------------------------------------------------------------------- #
def test_guard_gate_fails_closed_on_exception():
    pytest.importorskip("numpy")
    from manipulation_kit.arms.guard import GuardGate

    class Exploding:
        def check(self, a, b):
            raise RuntimeError("API drift")

    gate = GuardGate(Exploding())
    assert gate.installed
    assert gate.ok([0.0] * 7, [0.0] * 7) is False, "guard must fail CLOSED"
    assert gate.clearance([0.0] * 7, [0.0] * 7) is None


def test_guard_gate_converts_radians_to_degrees_once():
    """The rad->deg seam is in exactly one place. Passing radians straight
    through once made the guard evaluate a near-zero-degree posture and let an
    elbow-through-torso command pass."""
    import numpy as np

    from manipulation_kit.arms.guard import GuardGate

    seen = {}

    class Recorder:
        def check(self, a, b):
            seen["a"], seen["b"] = np.asarray(a), np.asarray(b)
            return type("Rep", (), {"ok": True, "min_body_clearance": 0.05})()

    gate = GuardGate(Recorder())
    qa = np.full(7, np.pi / 2)
    assert gate.ok(qa, np.zeros(7)) is True
    np.testing.assert_allclose(seen["a"], np.full(7, 90.0))
    np.testing.assert_allclose(seen["b"], np.zeros(7))


def test_no_guard_means_allowed_but_not_installed():
    from manipulation_kit.arms.guard import GuardGate

    gate = GuardGate(None)
    assert not gate.installed
    assert gate.ok([0.0] * 7, [0.0] * 7) is True
    assert gate.report([0.0] * 7, [0.0] * 7) is None


def test_guard_records_the_rejection_reason():
    from manipulation_kit.arms.guard import GuardGate

    class Rejecting:
        def check(self, a, b):
            return type("Rep", (), {"ok": False,
                                    "violations": ["torso_belly vs Link4_R"]})()

    gate = GuardGate(Rejecting())
    assert gate.ok([0.0] * 7, [0.0] * 7) is False
    assert gate.last_reject is not None
    assert "torso_belly" in gate.last_reject[0]


# --------------------------------------------------------------------------- #
# clutch tuning
# --------------------------------------------------------------------------- #
def test_clutch_tuning_requires_an_explicit_workspace():
    """A safety box must never silently default to another robot's numbers."""
    pytest.importorskip("scipy")
    from manipulation_kit.arms.targeting import ClutchTuning

    with pytest.raises(TypeError):
        ClutchTuning()                      # type: ignore[call-arg]

    tun = ClutchTuning(workspace=safety.WORKSPACE)
    assert tun.bounds() == [(0.15, 0.55), (-0.72, 0.72), (0.05, 0.85)]
    assert tun.max_step_m == safety.MAX_STEP_M
