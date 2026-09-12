"""The two console scripts, end to end.

``mkit-urdf`` and ``mkit-toolconfig`` are the public face of this repo — a
second-development customer meets them before they read a line of the library —
so they get the same treatment as the library: the happy path, the refusals, and
the one message that has to be genuinely useful (the O30).
"""
import json
import subprocess
import sys

import pytest

from manipulation_kit.description import cli as urdf_cli
from manipulation_kit.description.variants import (
    VARIANTS, VariantNotBuildable, resolve)
from manipulation_kit.hands import cli as tool_cli


# ------------------------------------------------------------- mkit-urdf
def test_variants_lists_every_registered_variant(capsys):
    assert urdf_cli.main(["variants"]) == 0
    out = capsys.readouterr().out
    for name in VARIANTS:
        assert name in out


def test_export_writes_a_loadable_export(tmp_path):
    dest = tmp_path / "d1-collision"
    assert urdf_cli.main(["export", "d1-collision", "--dest", str(dest)]) == 0
    assert (dest / "d1.urdf").is_file()
    manifest = json.loads((dest / "PROVENANCE.json").read_text())
    assert manifest["flavour"] == "d1_collision"
    assert manifest["files"]["d1.urdf"]
    assert manifest["source_repo"].endswith("manipulation-kit")


def test_export_check_passes_on_a_fresh_export(tmp_path):
    dest = tmp_path / "d1-collision"
    urdf_cli.main(["export", "d1-collision", "--dest", str(dest)])
    assert urdf_cli.main(
        ["export", "d1-collision", "--dest", str(dest), "--check"]) == 0


def test_export_check_catches_a_hand_edit(tmp_path):
    """The whole point of the provenance manifest: an edited vendored copy is
    caught, not merged into whatever the consumer's simulator happens to load."""
    dest = tmp_path / "d1-collision"
    urdf_cli.main(["export", "d1-collision", "--dest", str(dest)])
    urdf = dest / "d1.urdf"
    urdf.write_text(urdf.read_text().replace("<robot", "<robot "))
    assert urdf_cli.main(
        ["export", "d1-collision", "--dest", str(dest), "--check"]) == 1


def test_o30_variant_refuses_with_the_actual_reason(capsys):
    """Registered-but-unbuildable must explain itself. "Unknown variant" would
    read like a typo and send someone looking in the wrong repository."""
    assert urdf_cli.main(
        ["export", "d1-wholebody-o30", "--dest", "/tmp/nope"]) == 2
    err = capsys.readouterr().err
    assert "O30 CAD" in err and "measured TCP" in err
    with pytest.raises(VariantNotBuildable):
        resolve("d1-wholebody-o30")


def test_fetch_visuals_refuses_a_directory_without_the_meshes(tmp_path, capsys):
    assert urdf_cli.main(
        ["fetch-visuals", "--from-d1-sdk", str(tmp_path)]) == 2
    assert "no visual meshes" in capsys.readouterr().err


def test_build_reproduces_the_committed_urdfs(tmp_path):
    """``mkit-urdf build`` must be a no-op on a clean tree — the URDFs are
    generated, so a byte difference means either the generator drifted from its
    output or somebody hand-edited a .urdf."""
    from manipulation_kit.description import ROOT

    out = subprocess.run(
        [sys.executable, str(ROOT / "d1" / "tools" / "generate_d1_urdf.py"),
         "--out-dir", str(tmp_path)], check=True, capture_output=True)
    assert out.returncode == 0
    for name in ("d1.urdf", "d1_wholebody.urdf", "d1_wholebody_gripper.urdf"):
        assert (tmp_path / name).read_bytes() == (ROOT / "d1" / name).read_bytes(), \
            f"{name} differs from the generator — run `mkit-urdf build`"


# --------------------------------------------------------- mkit-toolconfig
def test_toolconfig_list_covers_every_hand(capsys):
    assert tool_cli.main(["list"]) == 0
    out = capsys.readouterr().out
    for model in tool_cli.MODELS:
        assert model in out


def test_toolconfig_export_round_trips(tmp_path):
    out = tmp_path / "grip.json"
    assert tool_cli.main(["export", "d1/parallel_gripper", str(out)]) == 0
    doc = json.loads(out.read_text())
    assert doc["model"] == "d1/parallel_gripper"
    assert len(doc["kinematics"]) == 6 and len(doc["dynamics"]) == 10


def test_toolconfig_export_to_stdout(capsys):
    assert tool_cli.main(["export", "linkerbot/o30", "-"]) == 0
    assert json.loads(capsys.readouterr().out)["model"] == "linkerbot/o30"
