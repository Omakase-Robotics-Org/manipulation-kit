"""This repository must not contain Omakase or vendor CAD.

manipulation-kit is meant to become public (LICENSE-STATUS.md). The code, the
URDFs and the third-party descriptions that carry a licence can go; the CAD
geometry cannot, because nobody has yet written down that we may redistribute
it. The geometry was removed from this repository AND from its history on
2026-09-10 and moved to the private ``manipulation-kit-assets``.

That decision survives exactly as long as something checks it. A mesh comes
back the way it left — somebody copies a directory in to make a sim run and
commits the lot — so this is a test rather than a paragraph in a README.

It is deliberately a test over the WORK TREE, not over the URDFs: the URDFs
are supposed to keep naming these files (a reference is not a redistribution),
and ``mkit-urdf fetch-assets`` is supposed to be able to put them on disk. What
must not happen is a fetched mesh becoming a TRACKED file, which is what
:func:`test_no_external_cad_is_tracked` catches.
"""
import os
import subprocess

import pytest

from manipulation_kit import assets

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, ".."))


def _tracked():
    out = subprocess.run(["git", "-C", REPO, "ls-files"],
                         capture_output=True, text=True)
    if out.returncode != 0:
        pytest.skip("not a git checkout")
    return out.stdout.split()


#: Geometry suffixes. A licensed third-party mesh is allowed (see
#: :func:`test_only_licensed_third_party_geometry_is_tracked`); an unlicensed
#: one is not, and the difference is WHERE it sits.
GEOMETRY = (".STL", ".stl", ".obj", ".dae", ".ply", ".glb", ".step", ".stp")

#: The only geometry that may be committed here, and why. Both are Apache-2.0
#: and both keep their upstream ``package.xml`` / README stating so.
LICENSED_GEOMETRY_DIRS = {
    "src/manipulation_kit/description/d1_arm/yubi_description/meshes":
        "YUBI parallel-jaw hand — Apache-2.0 (Toyota / AIRoA)",
    "src/manipulation_kit/description/d1_yubi_description_v2/"
    "yubi_description/meshes":
        "YUBI parallel-jaw hand — Apache-2.0 (Toyota / AIRoA)",
    "src/manipulation_kit/hands/leadshine/dh116s/descriptions/meshes":
        "DH116S hand — Apache-2.0 (sorrowfeng/leadtron_hand_descriptions)",
}


def test_only_licensed_third_party_geometry_is_tracked():
    """Every committed mesh must sit in a directory whose licence is known."""
    stray = [f for f in _tracked() if f.endswith(GEOMETRY)
             and os.path.dirname(f) not in LICENSED_GEOMETRY_DIRS]
    assert not stray, (
        "geometry with no recorded licence is committed here:\n  "
        + "\n  ".join(stray)
        + "\n\nThis repository ships URDFs, not CAD (LICENSE-STATUS.md). If "
        "this is Omakase/vendor CAD it belongs in manipulation-kit-assets; if "
        "it is third-party under a licence that permits redistribution, add "
        "its directory to LICENSED_GEOMETRY_DIRS with the licence, and to "
        "LICENSE-STATUS.md.")


def test_no_external_cad_is_tracked():
    """`mkit-urdf fetch-assets` writes into the package; .gitignore is what
    keeps the result out of a commit. Prove the two agree.

    Geometry only: ``meshes/body/merged_robot.urdf`` is the vendor's own URDF
    and stays committed. Shipping a description that NAMES a part is not
    shipping the part.
    """
    tracked = set(_tracked())
    for rel in assets.EXTERNAL_ASSET_DIRS:
        prefix = f"src/manipulation_kit/{rel}/"
        inside = sorted(f for f in tracked if f.startswith(prefix)
                        and f.endswith(GEOMETRY))
        assert not inside, (
            f"{prefix} is withheld CAD but these are committed: {inside}. "
            "Check .gitignore — a fetched asset must never become tracked.")


@pytest.mark.parametrize("rel", sorted(assets.EXTERNAL_ASSET_DIRS))
def test_every_withheld_directory_is_gitignored(rel):
    """The ignore rule is the whole safety net, so name it in a test rather
    than trusting that nobody tidies .gitignore."""
    probe = os.path.join(REPO, "src", "manipulation_kit", rel, "probe.STL")
    out = subprocess.run(["git", "-C", REPO, "check-ignore", "-q", probe])
    assert out.returncode == 0, (
        f"src/manipulation_kit/{rel}/*.STL is not ignored — a fetched mesh "
        "could be committed by accident")
