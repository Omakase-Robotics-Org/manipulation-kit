"""``mkit-teach`` offline: the golden take -> CSV, check, force, register."""
from __future__ import annotations

from pathlib import Path

import pytest

from manipulation_kit.teach import load_csv
from manipulation_kit.teach.cli import main
from manipulation_kit.teach.registry import entry_yaml, register

DATA = Path(__file__).resolve().parents[1] / "data" / "teach"


def test_the_golden_take_exports_byte_for_byte(tmp_path):
    """tests/data/teach/wave_take.json (8 s, jitter, one spike) through the
    default pipeline is exactly wave_motion.csv. A change to smoothing,
    reduction, HOME pinning, the limiter or the writer shows up here."""
    out = tmp_path / "wave_motion.csv"
    assert main(["export", str(DATA / "wave_take.json"), str(out), "--name", "wave",
                 "--usage", "filler", "ending"]) == 0
    assert out.read_text() == (DATA / "wave_motion.csv").read_text()


def test_check_passes_the_golden_and_fails_an_unsafe_file(tmp_path, capsys):
    assert main(["check", str(DATA / "wave_motion.csv"), "--ascii",
                 "--step-s", "0.05"]) == 0
    assert " R1 " in capsys.readouterr().out
    bad = tmp_path / "bad_motion.csv"
    rows = (DATA / "wave_motion.csv").read_text().splitlines()
    head = [r for r in rows if not r[0].isdigit()]
    data = [r.split(",") for r in rows if r[0].isdigit()]
    data[2][2] = f"{float(data[2][2]) + 30.0:.4f}"         # A J2 into the belly
    bad.write_text("\n".join(head + [",".join(r) for r in data]) + "\n")
    assert main(["check", str(bad), "--step-s", "0.05"]) == 1


def test_export_refuses_unsafe_unless_forced_and_force_marks_it(tmp_path):
    bad = tmp_path / "src.csv"
    rows = (DATA / "wave_motion.csv").read_text().splitlines()
    data = [r.split(",") for r in rows if r[0].isdigit()]
    data[2][2] = f"{float(data[2][2]) + 30.0:.4f}"
    bad.write_text("\n".join(["duration,R1"] + [",".join(r) for r in data]) + "\n")
    out = tmp_path / "out_motion.csv"
    assert main(["export", str(bad), str(out), "--no-speed-limit"]) == 1
    assert not out.exists()
    assert main(["export", str(bad), str(out), "--force"]) == 0
    assert load_csv(out).unsafe
    assert main(["play", str(out), "--dry-run"]) == 1        # refused offline


def test_register_splices_the_entry_like_omakase_core_did(tmp_path):
    yml = tmp_path / "gesture.yaml"
    yml.write_text("robot_vendor: omakase\nrobot_name: d1\ncsv_base_dir: ./csv\n"
                   "gestures:\n- name: d1_gentle_nod\n  csv: gentle_nod_motion.csv\n"
                   "  sentiment: positive\n  usage:\n  - filler\n  generated: true\n")
    assert register(yml, "wave") == "added"
    assert register(yml, "wave", sentiment="positive", usage=["opening"]) == "replaced"
    text = yml.read_text()
    assert text.count("- name: d1_wave") == 1
    assert text.endswith(entry_yaml("wave", sentiment="positive", usage=["opening"]))
    assert "d1_gentle_nod" in text and "generated: true" in text


@pytest.mark.parametrize("name", ["", "Wave", "a.b", "_x", "wave!"])
def test_names_follow_the_panel_rule(name):
    with pytest.raises(ValueError):
        entry_yaml(name)
