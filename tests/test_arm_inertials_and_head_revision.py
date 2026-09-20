"""Two facts about the robot that used to be reinstated by every consumer.

**Arm inertials.** The generated D1 models used to give every arm link the
family's 0.5 kg / 1e-3 diagonal placeholder while the vendor D1 arm URDFs,
committed in this very repository, carry the CAD masses, COMs and tensors.
Consumers noticed and patched them back in downstream (d1-isaaclab's URDF
builder injected exactly these elements into every asset it composed), which
puts a robot fact outside the robot description. These tests pin the inertials
to the vendor files so the description is the one place that says what an arm
link weighs.

**Head-camera mount tilt.** 15 deg is a DESIGN value of one head part, not a
constant of the D1: d1-1..d1-3 wear it and the next units are built at 20 deg
(Shu, 2026-09-20). So it is a named hardware revision, the committed URDFs say
which revision they are, and a consumer composing its own asset selects one by
name. What a robot's camera ACTUALLY points at is its per-robot ArUco
calibration, which is an absolute head_link -> camera extrinsic and must never
be folded back into this shared description.
"""
import math
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import pytest

from manipulation_kit.description import (
    DEFAULT_HARDWARE_REVISION, HEAD_CAMERA_TILT_DEG, ROOT, head_camera_tilt_deg)

GENERATED = ("d1.urdf", "d1_wholebody.urdf", "d1_wholebody_gripper.urdf")
SIDES = {"R": "right", "L": "left"}
ARM_LINKS = ["Base"] + [f"Link{i}" for i in range(1, 8)] + ["TCP_Link"]


def _links(path):
    return {link.get("name"): link
            for link in ET.parse(path).getroot().findall("link")}


def _inertial(link):
    element = link.find("inertial")
    assert element is not None, f"{link.get('name')} has no <inertial>"
    origin = element.find("origin")
    inertia = element.find("inertia")
    return (origin.get("xyz"), origin.get("rpy"),
            element.find("mass").get("value"),
            tuple(inertia.get(k)
                  for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")))


@pytest.fixture(scope="module")
def vendor():
    out = {}
    for side, folder in SIDES.items():
        path = ROOT / "d1_arm" / folder / f"d1_arm_{folder}.urdf"
        out[side] = {name: _inertial(link)
                     for name, link in _links(path).items()}
    return out


@pytest.mark.parametrize("filename", GENERATED)
def test_every_arm_link_carries_the_vendor_cad_inertial(filename, vendor):
    """Verbatim, attribute string for attribute string.

    The chain is byte-derived from the vendor URDFs (see
    ``test_arm_chain_identical_across_all_models_and_sides``), so the link
    frames are identical and the inertial transfers with no transform. Copying
    it through a float format would let a rounding difference creep in between
    the description and the CAD it claims to quote.
    """
    links = _links(ROOT / "d1" / filename)
    for side in SIDES:
        for stem in ARM_LINKS:
            name = f"{stem}_{side}"
            assert _inertial(links[name]) == vendor[side][name], name


@pytest.mark.parametrize("filename", GENERATED)
def test_no_arm_link_is_left_on_the_placeholder(filename):
    """The 0.5 kg / 1e-3 diagonal is what a link with no known inertial gets.

    An arm link is not in that position, and a placeholder that looks like a
    number is how a 3x mass error survives review.
    """
    links = _links(ROOT / "d1" / filename)
    for side in SIDES:
        for stem in ARM_LINKS[:-1]:            # TCP_Link is genuinely massless
            mass = float(_inertial(links[f"{stem}_{side}"])[2])
            assert mass != 0.5, f"{stem}_{side} still on the placeholder"
            assert mass > 0.3, f"{stem}_{side} mass {mass} is not CAD"


def test_the_tool_flange_stays_massless():
    """``TCP_Link`` is a pure frame at the end of the chain in the vendor
    files, and inventing a mass for it would put 50 g on every tool reading."""
    links = _links(ROOT / "d1" / "d1_wholebody_gripper.urdf")
    for side in SIDES:
        assert float(_inertial(links[f"TCP_Link_{side}"])[2]) == 0.0


def test_arm_mass_per_side_is_the_vendor_total(vendor):
    """One side of the arm chain weighs what the vendor says it does."""
    links = _links(ROOT / "d1" / "d1_wholebody_gripper.urdf")
    for side in SIDES:
        total = sum(float(_inertial(links[f"{stem}_{side}"])[2])
                    for stem in ARM_LINKS)
        expected = sum(float(vendor[side][f"{stem}_{side}"][2])
                       for stem in ARM_LINKS)
        assert total == pytest.approx(expected)
        assert total == pytest.approx(8.08, abs=0.01)


# ------------------------------------------------------------- head revision
def test_the_revision_table_says_which_head_part_each_unit_wears():
    assert head_camera_tilt_deg("rev1") == 15.0
    assert head_camera_tilt_deg("rev2") == 20.0
    assert head_camera_tilt_deg() == HEAD_CAMERA_TILT_DEG[
        DEFAULT_HARDWARE_REVISION]


def test_an_unknown_revision_is_an_error_not_a_default():
    """Silently falling back to 15 deg for an unrecognised revision is how a
    robot gets simulated with the wrong head part and nobody is told."""
    with pytest.raises(ValueError, match="unknown D1 hardware revision"):
        head_camera_tilt_deg("rev0")


@pytest.mark.parametrize("filename", ("d1_wholebody.urdf",
                                      "d1_wholebody_gripper.urdf"))
def test_the_committed_urdfs_are_the_default_revision(filename):
    root = ET.parse(ROOT / "d1" / filename).getroot()
    joint = root.find("joint[@name='head_camera_mount']")
    yaw = float(joint.find("origin").get("rpy").split()[2])
    assert yaw == pytest.approx(
        math.radians(head_camera_tilt_deg()), abs=1e-6)


def test_building_another_revision_moves_the_head_camera_and_nothing_else():
    """The revision is a one-number knob, which is the point of naming it: a
    consumer that asks for rev2 must not silently get a different robot."""
    generator = ROOT / "d1" / "tools" / "generate_d1_urdf.py"
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run(
            [sys.executable, str(generator), "--out-dir", tmp,
             "--only", "d1_wholebody_gripper.urdf",
             "--hardware-revision", "rev2"], check=True, capture_output=True)
        built = ET.parse(os.path.join(tmp, "d1_wholebody_gripper.urdf")).getroot()
    committed = ET.parse(ROOT / "d1" / "d1_wholebody_gripper.urdf").getroot()

    def mount(root):
        return root.find("joint[@name='head_camera_mount']/origin")

    assert float(mount(built).get("rpy").split()[2]) == pytest.approx(
        math.radians(20.0), abs=1e-6)
    assert mount(built).get("xyz") == mount(committed).get("xyz")

    def stripped(root):
        # Comments are excluded on purpose: the generator stamps the revision
        # into the header and into the head-camera note, and those SHOULD
        # differ. Everything a loader acts on must not.
        root.find("joint[@name='head_camera_mount']/origin").set("rpy", "")
        return {(el.tag, el.get("name")): ET.tostring(el)
                for el in root if isinstance(el.tag, str)}

    built_elements, committed_elements = stripped(built), stripped(committed)
    assert set(built_elements) == set(committed_elements)
    assert built_elements == committed_elements
