"""The prebuilt exports under ``dist/`` must match the description they came from.

``dist/`` exists so a consumer can grab an asset without installing or running
anything. That convenience is worth exactly as much as its freshness: a stale
``dist/`` is a second, forked copy of the robot's geometry, which is the failure
mode this whole repo is organised against.

So the same ``--check`` a consumer runs in ITS CI runs here, against our own
committed copies, on every push.
"""
import json
import os

import pytest

from manipulation_kit.description.tools import export_description as ex

HERE = os.path.dirname(os.path.abspath(__file__))
DIST = os.path.normpath(os.path.join(HERE, "..", "dist"))

#: Committed directory -> exporter flavour.
PREBUILT = {
    "d1-collision": "d1_collision",
    "d1-wholebody-gripper": "d1_wholebody_gripper",
}


@pytest.mark.parametrize("name,flavour", sorted(PREBUILT.items()))
def test_prebuilt_export_is_current(name, flavour):
    dest = os.path.join(DIST, name)
    if not os.path.isdir(dest):
        pytest.skip(f"{dest} not present (installed wheel, not a checkout)")
    assert ex.check(flavour, dest, "") == 0, (
        f"dist/{name} has drifted — regenerate with "
        f"`mkit-urdf export {name} --dest dist/{name}`")


@pytest.mark.parametrize("name", sorted(PREBUILT))
def test_prebuilt_export_carries_provenance(name):
    path = os.path.join(DIST, name, "PROVENANCE.json")
    if not os.path.isfile(path):
        pytest.skip("dist/ not present (installed wheel, not a checkout)")
    with open(path) as f:
        manifest = json.load(f)
    assert manifest["source_repo"].endswith("manipulation-kit")
    assert manifest["files"], "an export with no files is not an export"
    assert "absent_optional" in manifest, (
        "the manifest must declare which optional meshes are absent, or a "
        "reader cannot tell 'not shipped' from 'lost'")


def test_the_collision_export_needs_no_assets_at_all():
    """d1-collision is THE variant for a consumer with zero mesh assets — a
    planner, a bare host, a firmware-side test rig. One file, no meshes."""
    dest = os.path.join(DIST, "d1-collision")
    if not os.path.isdir(dest):
        pytest.skip("dist/ not present")
    files = {f for f in os.listdir(dest)}
    assert files == {"d1.urdf", "PROVENANCE.json"}, sorted(files)
