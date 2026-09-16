"""URDF parse + forward kinematics against known reference poses.

The reference numbers were cross-validated against the vendor-chain C++ FK
d1-sdk include/omakase_arm/collision_model.h::armPoints — agreement < 1e-8 m
over random configs. That live comparison (``test_cpp_crosscheck.py``) did NOT
come to manipulation-kit: the C++ collision model stayed in d1-sdk and is being
retired in favour of d1-firmwared's Rust port, which is itself validated
against 426 golden vectors generated from this guard. The numbers below are
the frozen result of the original crosscheck.
"""
import json
import math
import os

import pytest

import manipulation_kit
from manipulation_kit.guard.urdf_model import UrdfModel, DEFAULT_URDF

HERE = os.path.dirname(os.path.abspath(__file__))
# manipulation-kit resolves its assets INSIDE the package (no $D1_SDK_DIR, no
# sibling checkout), so the tests do too — importing the anchor rather than
# counting ".." keeps them working from a wheel as well as a checkout.
KIT = os.path.dirname(os.path.abspath(manipulation_kit.__file__))
CONFIG = os.path.join(KIT, "config")


@pytest.fixture(scope="module")
def model():
    return UrdfModel()


def _close(p, q, tol=1e-5):
    return all(abs(a - b) < tol for a, b in zip(p, q))


def test_urdf_exists_and_parses(model):
    assert model.name == "d1"
    assert model.root == "dual_base"


def test_structure(model):
    rev = model.revolute_joints()
    # 7 per arm + 2 fingers per hand
    assert len(rev) == 18
    for suf in ("R", "L"):
        for i in range(1, 8):
            assert f"Link{i}_{suf}" in model.links
        assert f"TCP_Link_{suf}" in model.links
        assert f"yubi_{suf}_hand_root" in model.links
    for body in ("torso_column", "head_link", "chassis_link"):
        assert body in model.links


def test_limits_match_safety_zones_json(model):
    with open(os.path.join(CONFIG, "safety_zones.json")) as f:
        zones = json.load(f)["joint_limits_deg"]
    lims = model.limits_deg([f"Joint{i}_R" for i in range(1, 8)])
    for i, (lo, hi) in enumerate(lims):
        zlo, zhi = zones[f"J{i + 1}"]
        assert abs(lo - zlo) < 0.2, f"J{i + 1} lower {lo} != {zlo}"
        assert abs(hi - zhi) < 0.2, f"J{i + 1} upper {hi} != {zhi}"


def test_left_right_symmetric_limits(model):
    r = model.limits_deg([f"Joint{i}_R" for i in range(1, 8)])
    l = model.limits_deg([f"Joint{i}_L" for i in range(1, 8)])
    assert r == l


def test_fk_zero_pose(model):
    """All joints zero = T-pose: arms straight out laterally at shoulder
    height; chain y-offsets are mount 0.037 + Base 0.1586 + 2x 0.264 +
    TCP 0.087."""
    tfs = model.link_transforms({})
    assert _close(tfs["Link1_R"].t, (0.0, 0.1956, 0.5))
    assert _close(tfs["Link3_R"].t, (0.0, 0.4596, 0.5))
    assert _close(tfs["Link7_R"].t, (0.0, 0.7236, 0.5))
    assert _close(tfs["TCP_Link_R"].t, (0.0, 0.8106, 0.5))
    assert _close(tfs["Link7_L"].t, (0.0, -0.7236, 0.5))
    assert _close(tfs["TCP_Link_L"].t, (0.0, -0.8106, 0.5))
    # yubi flange offset: +0.055 along tool axis, 0.019 lateral
    assert _close(tfs["yubi_R_hand_root"].t, (0.0, 0.8656, 0.519))


def test_fk_home_pose(model):
    """Hardware HOME pose (config/home_pose.json): wrist-forward 'ready',
    left/right mirror-symmetric."""
    with open(os.path.join(CONFIG, "home_pose.json")) as f:
        hp = json.load(f)["home_pose"]
    q = {f"Joint{i + 1}_R": math.radians(hp[i]) for i in range(7)}
    q.update({f"Joint{i + 1}_L": math.radians(hp[7 + i]) for i in range(7)})
    tfs = model.link_transforms(q)
    # Pinned against the wrist-up home pose (config/home_pose.json). The FK
    # math itself is validated number-for-number against the C++ model in
    # test_cpp_crosscheck; these values pin the measured pose geometry.
    assert _close(tfs["Link4_R"].t, (-0.147177, 0.208201, 0.280454), 1e-4)
    assert _close(tfs["Link7_R"].t, (0.105498, 0.211328, 0.201932), 1e-4)
    assert _close(tfs["TCP_Link_R"].t, (0.192492, 0.210477, 0.201456), 1e-4)
    # mirror symmetry of the measured home pose
    for r, l in (("Link4_R", "Link4_L"), ("Link7_R", "Link7_L"),
                 ("TCP_Link_R", "TCP_Link_L")):
        pr, pl = tfs[r].t, tfs[l].t
        assert _close((pr[0], -pr[1], pr[2]), pl, 1e-4)


def test_default_urdf_is_the_repo_description(model):
    assert DEFAULT_URDF.endswith(os.path.join("description", "d1", "d1.urdf"))
    assert os.path.exists(DEFAULT_URDF)


def test_body_boxes_are_static_and_axis_aligned(model):
    from manipulation_kit.guard.urdf_model import prim_to_world
    tfs = model.link_transforms({})
    for link in ("torso_column", "head_link", "chassis_link"):
        for prim in model.collisions[link]:
            kind, shape = prim_to_world(prim, link, tfs[link])
            assert kind == "aabb"
