"""MotionGuard behaviour: known-safe poses pass, known-colliding poses are
rejected, limits clamp/reject, margins configurable."""
import json
import os

import pytest

import manipulation_kit
from manipulation_kit.guard import MotionGuard, GuardViolation

HERE = os.path.dirname(os.path.abspath(__file__))
# manipulation-kit resolves its assets INSIDE the package (no $D1_SDK_DIR, no
# sibling checkout), so the tests do too — importing the anchor rather than
# counting ".." keeps them working from a wheel as well as a checkout.
KIT = os.path.dirname(os.path.abspath(manipulation_kit.__file__))
CONFIG = os.path.join(KIT, "config")
#: The gesture CSV moved into the test suite with this file: it is fixture
#: data, and d1-sdk's copy sat next to the C++ playback example that consumed
#: it, which did not come here.
DATA = os.path.join(HERE, "..", "data")

# --- fixture poses (deg, SDK order J1..J7) --------------------------------
# found by randomized / deterministic FK search over the joint limits and
# verified numerically against the URDF FK; kept fixed so tests are
# deterministic.
POSE_BODY_HIT = [-73.4, 113.3, -132.1, -65.6, 89.0, -41.8, -2.0]   # Link3 -> torso_belly
# YUBI hand folded back so it overlaps its OWN proximal arm structure (the
# EE capsules penetrate Link1..Link4 by ~2 cm, FK-verified).  Under the
# EE-exclusion policy this is ALLOWED — the hand is a working surface — so
# the full guard PASSES it (before the fix the same-arm self check rejected
# any wrist-into-arm fold).
POSE_EE_ONTO_OWN_ARM = [91.03, 30.72, 131.75, -142.98, 99.94, 56.74, 84.64]
# Bimanual "hands together in front of the torso": the two YUBI hands are
# brought to the midline ~1 cm apart (FK-verified), arm STRUCTURE stays
# clear.  Under the EE-exclusion policy this MUST pass (field failure #1:
# the old guard rejected it because the finger capsules touched).
POSE_HANDS_TOUCH_A = [3.11, -41.45, 2.29, -115.48, -6.24, -22.87, -20.34]
POSE_HANDS_TOUCH_B = [-3.11, -41.45, -2.29, -115.48, 6.24, -22.87, 20.34]
# Elbow / forearm driven into the upper-chest band (field failure #2): the
# forearm (Link4) penetrates the real trunk at z~0.59 m, a zone the measured
# torso/head boxes MISS (min body clearance stays > body_margin) but the new
# per-arm chest keep-out catches.  FK-verified: Link4_R at ~(0.23, 0.10, 0.59).
POSE_ELBOW_INTO_TORSO = [-20.8, -106.6, -3.6, -95.7, 24.0, -38.2, -63.9]
POSE_AA_A = [158.8, 87.8, -134.1, -66.6, 89.8, -20.3, 90.0]        # arm-arm crossing
POSE_AA_B = [-158.8, 87.8, 134.1, -66.6, -89.8, -20.3, 90.0]
POSE_SAFE = [-90.7, 10.4, -45.0, -30.3, 43.5, -52.1, -87.6]


@pytest.fixture(scope="module")
def guard():
    return MotionGuard()


@pytest.fixture(scope="module")
def home():
    with open(os.path.join(CONFIG, "home_pose.json")) as f:
        hp = json.load(f)["home_pose"]
    return hp[:7], hp[7:]


def test_home_pose_passes(guard, home):
    qa, qb = home
    rep = guard.check(qa, qb)
    assert rep.ok, str(rep)
    assert rep.min_body_clearance >= 0.03
    assert rep.min_chest_clearance >= 0.03   # HOME clean of the chest keep-out
    assert rep.min_self_clearance > 0.0


def test_readme_example_pose_passes(guard):
    q = [0, -70, 0, -40, 0, 0, 0]
    assert guard.check(q, q).ok


def test_safe_random_pose_passes(guard):
    assert guard.check(POSE_SAFE, None).ok


def test_shipped_gesture_csv_passes(guard):
    """Every keyframe of the repo's teach-recorded gesture must pass with
    the default guard config (it already passes the C++ playback gate)."""
    path = os.path.join(DATA, "test_gesture_motion.csv")
    rows = [r.strip() for r in open(path)
            if r.strip() and not r.strip().startswith("#")]
    assert rows[0].startswith("duration,")
    checked = 0
    for row in rows[1:]:
        vals = [float(v) for v in row.split(",")]
        q = vals[1:15]
        rep = guard.check(q[:7], q[7:14])
        assert rep.ok, f"keyframe {checked}: {rep}"
        checked += 1
    assert checked >= 10


def test_elbow_into_torso_rejected(guard):
    rep = guard.check(POSE_BODY_HIT, None)
    assert not rep.ok
    assert any("body box" in v for v in rep.violations), rep.violations
    assert rep.min_body_clearance < 0.03


def test_ee_folded_onto_own_arm_passes(guard):
    """The YUBI hand folded back until it overlaps its own proximal arm is
    ALLOWED (the EE is a working surface, excluded from the self check).
    The full guard passes and the reported structural self clearance stays
    positive because the EE capsules are not part of it."""
    rep = guard.check(POSE_EE_ONTO_OWN_ARM, None)
    assert rep.ok, str(rep)
    assert rep.min_self_clearance > 0.0


def test_self_check_runs_and_is_positive_at_home(guard, home):
    """Structural (non-EE) arm links cannot self-intersect within the joint
    limits (verified by search), so the self check reports positive
    clearance for reachable poses; this pins that it is still wired in and
    computing a finite clearance rather than being silently disabled."""
    qa, qb = home
    rep = guard.check(qa, qb)
    assert 0.0 < rep.min_self_clearance < 1e9


# --- field failure #1: EE-exclusion collision policy ----------------------
def test_bimanual_hands_touching_passes(guard):
    """Both YUBI hands brought together in front of the torso (~1 cm apart)
    with the arm structure clear MUST pass: the end-effectors are excluded
    from every collision check so bimanual contact is allowed.  (Before the
    EE-exclusion fix the finger capsules tripped the arm-arm check.)"""
    rep = guard.check(POSE_HANDS_TOUCH_A, POSE_HANDS_TOUCH_B)
    assert rep.ok, str(rep)
    # the arm STRUCTURE stays clear even though the hands touch
    assert rep.min_arm_arm >= 0.06


def test_ee_bodies_excluded_from_all_checks(guard):
    """The tool-side bodies (TCP flange + YUBI hand) are absent from every
    checked capsule set, on both arms."""
    from manipulation_kit.guard.guard import is_ee_body
    for side, q in (("A", POSE_HANDS_TOUCH_A), ("B", POSE_HANDS_TOUCH_B)):
        caps, _ = guard._arm_capsules(side, q)
        assert caps, "expected some structural capsules"
        assert not any(is_ee_body(c.link) for c in caps), \
            [c.link for c in caps if is_ee_body(c.link)]
    # boundary: Link7 (proximal of the flange) is arm structure, kept;
    # TCP_Link / yubi_* (distal of the flange) are tool, dropped.
    assert not is_ee_body("Link7_R")
    assert is_ee_body("TCP_Link_R")
    assert is_ee_body("yubi_R_hand_root")


# --- field failure #2: elbow-into-torso caught by the chest keep-out ------
def test_elbow_into_upper_chest_rejected(guard):
    """The forearm driven into the upper-chest band must be rejected by the
    per-arm chest keep-out, naming the offending link."""
    rep = guard.check(POSE_ELBOW_INTO_TORSO, None)
    assert not rep.ok
    assert any("chest keep-out" in v for v in rep.violations), rep.violations
    assert rep.min_chest_clearance < 0.0
    # The chest keep-out is what PENETRATES here; the body boxes only come
    # within 0.022 m. Before the vendor head box (urdf2026072302) replaced the
    # CAD one the nearest body box was >= 0.03 away, i.e. outside the guard
    # margin — the taller-but-shallower CAD head missed this band entirely.
    assert 0.0 < rep.min_body_clearance < 0.03


def test_chest_keepout_can_be_disabled():
    """chest_keepout=None removes the chest keep-out (and only that): the
    elbow-into-upper-chest pose is then no longer caught by a chest rule."""
    off = MotionGuard(chest_keepout=None)
    rep = off.check(POSE_ELBOW_INTO_TORSO, None)
    assert not any("chest keep-out" in v for v in rep.violations)


def test_arm_arm_crossing_rejected(guard):
    # each arm alone is fine...
    assert guard.check(POSE_AA_A, None).ok
    assert guard.check(None, POSE_AA_B).ok
    # ...but together they cross the midline into each other
    rep = guard.check(POSE_AA_A, POSE_AA_B)
    assert not rep.ok
    assert any("arm A" in v and "arm B" in v for v in rep.violations)


def test_limit_clamp_default(guard):
    clamped, changed = guard.clamp("A", [200, 0, 0, 0, 0, 0, 0])
    assert changed
    assert abs(clamped[0] - 173.0) < 0.1
    # J4 asymmetric limit
    clamped, _ = guard.clamp("A", [0, 0, 0, 90, 0, 0, 0])
    assert abs(clamped[3] - 45.0) < 0.1
    clamped, _ = guard.clamp("A", [0, 0, 0, -170, 0, 0, 0])
    assert abs(clamped[3] + 145.0) < 0.1
    # check() clamps (does not reject) by default
    rep = guard.check([200, -70, 0, -40, 0, 0, 0], None)
    assert rep.clamped


def test_limit_reject_mode():
    strict = MotionGuard(clamp_limits=False)
    rep = strict.check([200, 0, 0, 0, 0, 0, 0], None)
    assert not rep.ok
    assert "outside" in rep.violations[0]


def test_margin_configurable():
    """A pose that passes with default margins must fail when the body
    margin is cranked up."""
    hp = json.load(open(os.path.join(CONFIG,
                                     "home_pose.json")))["home_pose"]
    assert MotionGuard().check(hp[:7], hp[7:]).ok
    paranoid = MotionGuard(body_margin_m=0.10)
    assert not paranoid.check(hp[:7], hp[7:]).ok


def _aabb(guard, name):
    """The (lo, hi) of one body keep-out box, by its URDF primitive name."""
    return next((lo, hi) for n, lo, hi in guard.body_aabbs if n == name)


def test_box_pad_inflates_named_box(guard):
    """box_pad_m grows ONE named box on every face and leaves the others at
    their URDF size (the pad is a per-box geometry correction, not a global
    margin)."""
    pad = 0.02
    lo0, hi0 = _aabb(guard, "torso_belly")
    keep0 = _aabb(guard, "head_shell")
    padded = MotionGuard(box_pad_m={"torso_belly": pad})
    lo1, hi1 = _aabb(padded, "torso_belly")
    assert all(abs(b - (a - pad)) < 1e-12 for a, b in zip(lo0, lo1))
    assert all(abs(b - (a + pad)) < 1e-12 for a, b in zip(hi0, hi1))
    assert _aabb(padded, "head_shell") == keep0


def test_negative_box_pad_shrinks_named_box(guard):
    """A negative pad shrinks the box (lo rises, hi falls) — used where the
    CAD primitive is more conservative than the real hardware."""
    pad = -0.01
    lo0, hi0 = _aabb(guard, "torso_belly")
    shrunk = MotionGuard(box_pad_m={"torso_belly": pad})
    lo1, hi1 = _aabb(shrunk, "torso_belly")
    assert all(b > a for a, b in zip(lo0, lo1))
    assert all(b < a for a, b in zip(hi0, hi1))
    assert all(abs(b - (a - pad)) < 1e-12 for a, b in zip(lo0, lo1))
    assert all(abs(b - (a + pad)) < 1e-12 for a, b in zip(hi0, hi1))


def test_unknown_box_pad_name_rejected():
    """A misspelled box name pads nothing, so it must fail loudly rather than
    silently leave the operator with an unpadded guard."""
    with pytest.raises(ValueError) as exc:
        MotionGuard(box_pad_m={"torso_bely": 0.02})
    msg = str(exc.value)
    assert "torso_bely" in msg
    assert "torso_belly" in msg        # the known names are listed


def test_box_pad_reaches_the_body_check():
    """The padded geometry is what check() measures against: inflating the
    belly box by 5 cm turns a default-passing pose into a body violation
    naming that box."""
    assert MotionGuard().check(POSE_SAFE, None).ok
    rep = MotionGuard(box_pad_m={"torso_belly": 0.05}).check(POSE_SAFE, None)
    assert not rep.ok
    assert any("torso_belly" in v for v in rep.violations)


def test_checks_can_be_disabled():
    """Disabling check_body also disables the chest keep-out (it is a
    body-keep-out); with every check off, otherwise-colliding poses pass."""
    off = MotionGuard(check_body=False, check_self=False,
                      check_arm_arm=False)
    assert off.check(POSE_BODY_HIT, POSE_ELBOW_INTO_TORSO).ok


def test_guard_raises(guard):
    with pytest.raises(GuardViolation):
        guard.guard(POSE_BODY_HIT, None)
    qa, qb = guard.guard([0, -70, 0, -40, 0, 0, 0], None)
    assert qa == [0, -70, 0, -40, 0, 0, 0]
    assert qb is None


def test_input_validation(guard):
    with pytest.raises(ValueError):
        guard.clamp("C", [0] * 7)
    with pytest.raises(ValueError):
        guard.clamp("A", [0] * 6)
