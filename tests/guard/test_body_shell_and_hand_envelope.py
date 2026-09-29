"""The belly band's rounded edges and the hand envelope, against the guard.

* The belly band (``torso_belly``) is the measured torso box with its vertical
  edges rounded (front 80 mm, back 40 mm), given to URDF as three boxes and
  four vertical cylinders. The guard's distance to that union must be the
  distance to the rounded box, and every reason must name ``torso_belly``.
* With the CAD present, the fit record of
  ``description/d1/tools/fit_torso_belly.py`` is re-measured: the shape stays
  within 5 mm of the shell everywhere and covers far less air at the front
  corners than the square box did.
* The hand envelope covers every collision box of the parallel-gripper
  description (jaws over their whole travel, the camera plate and riser, the
  wrist-camera housing) and is checked against the body, not against its own
  arm and not against the other hand.
"""
from __future__ import annotations

import math
import os

import numpy as np
import pytest

from manipulation_kit.description.d1.tools import generate_d1_urdf as gen
from manipulation_kit.guard import MotionGuard, body_name, is_hand_envelope
from manipulation_kit.guard import geometry as g
from manipulation_kit.guard.urdf_model import UrdfModel

#: description/d1 (the generator lives in its tools/)
D1 = os.path.dirname(os.path.dirname(gen.__file__))

HOME_A = [-52.26, 87.38, 88.3, -114.32, 86.67, -1.1, 13.05]
HOME_B = [52.26, 87.38, -88.3, -114.32, -86.67, -1.1, -13.05]
#: Right arm (SDK B) with a jaw-tip corner of the gripper ON the belly shell
#: (0 mm from the rounded band) while every arm link is >= 24 mm clear of the
#: body. Found by minimising the jaw-tip distance from random postures near
#: HOME. The capsule-only guard of 0.19 accepted it (Link7 11 mm from the
#: square box); this is the contact seen in teleop.
JAW_TIP_ON_THE_BELLY = [-0.31, 107.62, -85.08, -126.62, -81.75, -29.69, -24.95]
#: The pad-tip corners in the gripper base_link frame (open jaws).
PAD_TIPS = [(sx * 0.072, sy * 0.019, 0.129) for sx in (-1, 1) for sy in (-1, 1)]


@pytest.fixture(scope="module")
def guard():
    return MotionGuard(clamp_limits=False)


def _rounded_distance(p, rf=gen.BELLY_FRONT_EDGE_R, rb=gen.BELLY_BACK_EDGE_R):
    (x0, y0, z0), (x1, y1, z1) = gen.BELLY_LO, gen.BELLY_HI
    cx0 = (x0 + x1) / 2.0
    r = rf if p[0] > cx0 else rb
    qx = min(max(p[0], x0 + r), x1 - r)
    qy = min(max(p[1], y0 + r), y1 - r)
    dxy = max(math.hypot(p[0] - qx, p[1] - qy) - r, 0.0)
    dz = max(z0 - p[2], 0.0, p[2] - z1)
    return math.hypot(dxy, dz)


def _belly_distance(guard, p):
    d = [g.point_aabb_distance(p, lo, hi) for n, lo, hi in guard.body_aabbs
         if body_name(n) == "torso_belly"]
    d += [g.point_vcyl_distance(p, *c[1:]) for c in guard.body_cylinders
          if body_name(c[0]) == "torso_belly"]
    return min(d)


# --------------------------------------------------------------------------- #
# the belly band
# --------------------------------------------------------------------------- #

def test_the_belly_parts_are_three_boxes_and_four_vertical_cylinders(guard):
    boxes = sorted(n for n, _, _ in guard.body_aabbs if body_name(n) == "torso_belly")
    cyls = sorted(c[0] for c in guard.body_cylinders)
    assert boxes == ["torso_belly", "torso_belly__back", "torso_belly__front"]
    assert cyls == ["torso_belly__back_left", "torso_belly__back_right",
                    "torso_belly__front_left", "torso_belly__front_right"]


def test_the_union_is_the_rounded_box(guard):
    """Distance from random points to the union of parts equals the distance
    to the analytic rounded box (to 1e-12), outside and at 0 inside."""
    rng = np.random.default_rng(3)
    lo, hi = np.array(gen.BELLY_LO), np.array(gen.BELLY_HI)
    for p in rng.uniform(lo - 0.08, hi + 0.08, (4000, 3)):
        assert _belly_distance(guard, p) == pytest.approx(_rounded_distance(p), abs=1e-12)


def test_the_segment_distance_to_the_band_is_exact(guard):
    """The guard's capsule-body distance (boxes exact, cylinders by a convex
    search) against a dense sample of the segment."""
    rng = np.random.default_rng(4)
    c = type("C", (), {})
    for _ in range(150):
        a, b = rng.uniform(-0.3, 0.3, 3) + (0, 0, 0.3), rng.uniform(-0.3, 0.3, 3) + (0, 0, 0.3)
        c.a, c.b, c.r = tuple(a), tuple(b), 0.0
        got = guard._body_distances(c)["torso_belly"]
        dense = min(_rounded_distance(a + t * (b - a)) for t in np.linspace(0, 1, 4001))
        assert got <= dense + 1e-12
        assert got >= dense - 2e-4 * float(np.linalg.norm(b - a))   # sampling slack


def test_a_refusal_names_the_band_not_a_part(guard):
    rep = guard.check(None, JAW_TIP_ON_THE_BELLY)
    assert not rep.ok
    assert all("torso_belly__" not in v for v in rep.violations)
    assert any("body box torso_belly " in v for v in rep.violations)


def test_the_belly_fits_the_shell_where_the_cad_says():
    from manipulation_kit.description.d1.tools import fit_torso_belly as fit
    got = fit.outlines()
    if got is None:
        pytest.skip("the torso CAD mesh is absent (set MKIT_ASSETS_DIR)")
    _cad, measured = got
    lo, hi = gen.BELLY_LO, gen.BELLY_HI
    square = fit.coverage(measured, lo, hi, 0.0, 0.0)
    rounded = fit.coverage(measured, lo, hi, gen.BELLY_FRONT_EDGE_R, gen.BELLY_BACK_EDGE_R)
    # the finding: the square box covered ~40 mm of air at the front corners
    assert square["over_front_mm"] > 35.0
    assert square["under_mm"] < 0.5
    # the rounded band: never more than 5 mm inside the shell, and < 10 mm of
    # air at the front corners, < 25 mm at the back
    assert rounded["under_mm"] <= 5.0
    assert rounded["over_front_mm"] < 10.0
    assert rounded["over_back_mm"] < 25.0


# --------------------------------------------------------------------------- #
# the hand envelope
# --------------------------------------------------------------------------- #

def _gripper_points(side):
    """Dense points of every gripper collision box, in the gripper base_link
    frame, the jaws over their whole travel."""
    m = UrdfModel(os.path.join(D1, "d1_wholebody_gripper.urdf"))
    tfs = m.link_transforms({})

    def mat(tf):
        t = np.eye(4)
        t[:3, :3] = np.array(tf.R)
        t[:3, 3] = tf.t
        return t

    base = np.linalg.inv(mat(tfs[f"gripper_{side}_base_link"]))
    joints = {j.child: j for j in (m.joints.values() if isinstance(m.joints, dict) else m.joints)}
    out = []
    for link in m.links:
        if not link.startswith(f"gripper_{side}"):
            continue
        for prim in m.collisions[link]:
            if prim.kind != "box":
                continue
            h = np.array(prim.size) / 2.0
            grid = np.array([[x, y, z, 1.0] for x in np.linspace(-h[0], h[0], 7)
                             for y in np.linspace(-h[1], h[1], 7)
                             for z in np.linspace(-h[2], h[2], 7)])
            pts = (base @ mat(tfs[link]) @ mat(prim.origin) @ grid.T).T[:, :3]
            j = joints.get(link)
            if j is not None and j.type == "prismatic":
                axis = base[:3, :3] @ (mat(tfs[link])[:3, :3] @ np.array(j.axis))
                pts = np.vstack([pts + q * axis for q in np.linspace(j.lower, j.upper, 9)])
            out.append(pts)
    return np.vstack(out)


@pytest.mark.parametrize("side", ["R", "L"])
def test_the_envelope_covers_the_gripper_description(side):
    pts = _gripper_points(side)
    assert len(pts) > 1000

    def seg(p, a, b):
        a, b = np.asarray(a), np.asarray(b)
        d = b - a
        t = np.clip(((p - a) @ d) / (d @ d), 0.0, 1.0)
        return np.linalg.norm(p - (a + t[:, None] * d), axis=1)

    outside = np.min([seg(pts, a, b) - r for _, a, b, r in gen.HAND_ENVELOPE], axis=0)
    assert outside.max() <= 0.001, outside.max()      # 0.74 mm at the pad-tip corner
    # and the jaw tip is inside it, not merely near
    assert np.max(np.min([seg(np.array(PAD_TIPS), a, b) - r
                          for _, a, b, r in gen.HAND_ENVELOPE], axis=0)) <= 0.001


def test_the_jaw_tip_on_the_belly_is_refused(guard):
    caps, tfs = guard._arm_capsules("B", JAW_TIP_ON_THE_BELLY)
    tip = min(_belly_distance(guard, tfs["hand_envelope_L"].apply(t)) for t in PAD_TIPS)
    assert tip == pytest.approx(0.0, abs=5e-4)
    structure = min(d for c in caps
                    if not is_hand_envelope(c.link) and c.link not in guard.body_exempt_links
                    for d in guard._body_distances(c).values())
    assert structure > 0.020                          # the arm itself is clear
    rep = guard.check(HOME_A, JAW_TIP_ON_THE_BELLY)
    assert not rep.ok
    assert rep.violations and all("hand_envelope_L" in v and "torso_belly" in v
                                  for v in rep.violations), rep.violations


def test_home_leaves_the_hands_clear_of_the_body(guard):
    rep = guard.check(HOME_A, HOME_B)
    assert rep.ok, rep
    for side, q in (("A", HOME_A), ("B", HOME_B)):
        caps, _ = guard._arm_capsules(side, q)
        hand = min(d for c in caps if is_hand_envelope(c.link)
                   for d in guard._body_distances(c).values())
        assert hand > 0.080, (side, hand)


def test_the_envelope_is_not_checked_against_its_own_arm_or_the_other_hand(guard):
    """Hands meeting in front of the torso pass, the structure keeps the
    arm-arm margin, and no violation is a same-arm or hand-hand pair."""
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "tg", os.path.join(os.path.dirname(__file__), "test_guard.py"))
    tg = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tg)
    rep = guard.check(tg.POSE_HANDS_TOUCH_A, tg.POSE_HANDS_TOUCH_B)
    assert rep.ok, rep
    assert rep.min_arm_arm >= guard.arm_arm_margin_m
    assert rep.min_hand_arm >= guard.body_margin_m
