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


def _edited(tmp_path, name, column, delta, *, header=True):
    rows = (DATA / "wave_motion.csv").read_text().splitlines()
    head = [r for r in rows if not r[0].isdigit()] if header else ["duration,R1"]
    data = [r.split(",") for r in rows if r[0].isdigit()]
    data[2][column] = f"{float(data[2][column]) + delta:.4f}"
    path = tmp_path / name
    path.write_text("\n".join(head + [",".join(r) for r in data]) + "\n")
    return path


def test_check_passes_the_golden_and_fails_a_limit(tmp_path, capsys):
    assert main(["check", str(DATA / "wave_motion.csv"), "--ascii",
                 "--step-s", "0.05"]) == 0
    assert " R1 " in capsys.readouterr().out
    # B J6 (column 13) +70 deg: past its limit — a HARD failure
    bad = _edited(tmp_path, "bad_motion.csv", 13, 70.0)
    assert main(["check", str(bad), "--step-s", "0.05"]) == 1
    assert "VIOLATION" in capsys.readouterr().out


def test_check_only_warns_about_the_guard(tmp_path, capsys):
    """A J2 +20 deg into the belly: the guard's finding is a WARNING and the
    exit status is 0 — the operator guided the arm there by hand."""
    belly = _edited(tmp_path, "belly_motion.csv", 2, 20.0)
    assert main(["check", str(belly), "--step-s", "0.05"]) == 0
    out = capsys.readouterr().out
    assert "WARNING: guard (advisory)" in out and "VIOLATION" not in out


def test_export_of_a_guard_only_finding_is_not_unsafe(tmp_path):
    belly = _edited(tmp_path, "src.csv", 2, 20.0, header=False)
    out = tmp_path / "out_motion.csv"
    assert main(["export", str(belly), str(out), "--no-speed-limit"]) == 0
    written = load_csv(out)
    assert not written.unsafe
    assert "below margin" in written.meta["min_clearance"]
    assert written.meta["guard_advisory"].startswith("guard (advisory)")
    assert main(["play", str(out), "--dry-run"]) == 0        # the kit lets it by


def test_export_refuses_unsafe_unless_forced_and_force_marks_it(tmp_path):
    bad = _edited(tmp_path, "src.csv", 13, 70.0, header=False)   # B J6 limit
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


def test_record_defaults_to_brake_release_with_one_holding_for_all_arms(capsys):
    from manipulation_kit.teach.cli import _guide, build_parser, confirm_holding
    parse = build_parser().parse_args
    assert _guide(parse(["record", "t.json"])) == "brake"
    assert _guide(parse(["record", "t.json", "--compliance"])) == "compliance"
    assert _guide(parse(["record", "t.json", "--no-brake"])) == "idle"
    with pytest.raises(SystemExit):
        _guide(parse(["record", "t.json", "--compliance", "--no-brake"]))
    asked = []
    args = parse(["record", "t.json"])
    assert confirm_holding(args, ("left", "right"),
                           ask=lambda prompt: asked.append(prompt) or "HOLDING")
    assert len(asked) == 1 and "left/right" in asked[0]      # once, not per arm
    assert "BRAKES RELEASED" in capsys.readouterr().out
    assert not confirm_holding(args, ("left",), ask=lambda prompt: "yes")


def test_play_defaults_to_the_speed_only_guard():
    from manipulation_kit.teach.cli import build_parser
    parse = build_parser().parse_args
    assert parse(["play", "x.csv"]).guard == "speed_only"
    assert parse(["play", "x.csv", "--guard", "full"]).guard == "full"
    with pytest.raises(SystemExit):
        parse(["play", "x.csv", "--guard", "off"])
