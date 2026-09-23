"""The operator flow of ``mkit-teach`` after d1-2 take2 (2026-09-23):
named takes, the CSV written beside the take, the speed ceiling carried in
the CSV, and the "Next:" command every subcommand ends with. Offline: no
robot, no daemon; stdin is scripted."""
from __future__ import annotations

import io
import json
import re
from pathlib import Path

import numpy as np
import pytest

from manipulation_kit.teach import (DEFAULT_SPEED, SpeedPolicy, check_gesture,
                                    load_csv, load_home)
from manipulation_kit.teach.cli import (ask_labels, build_parser, main,
                                        next_steps, resolve_take)

DATA = Path(__file__).resolve().parents[1] / "data" / "teach"
HOME = np.array(load_home())


def _take(tmp_path, name="wave", *, doc=None, file="take2.json"):
    doc = doc or json.loads((DATA / "wave_take.json").read_text())
    doc["meta"]["name"] = name
    path = tmp_path / file
    path.write_text(json.dumps(doc))
    return path


def _fast_take(tmp_path):
    """Arm B J1 swings 40 deg out and back in 1 s: ~113 deg/s taught."""
    t = np.arange(0.0, 3.0, 0.05)
    q = np.tile(HOME, (len(t), 1))
    q[:, 7] += 40.0 * np.sin(np.pi * np.clip((t - 0.8) / 1.0, 0.0, 1.0))
    doc = json.loads((DATA / "wave_take.json").read_text())
    doc.update(times=[round(float(v), 6) for v in t],
               samples=[[round(float(v), 4) for v in row] for row in q],
               grippers=[{"left": None, "right": None}] * len(t))
    return _take(tmp_path, "swing", doc=doc)


@pytest.fixture
def no_tty(monkeypatch):
    monkeypatch.setattr("sys.stdin", io.StringIO(""))


def _tty(monkeypatch, text):
    stdin = io.StringIO(text)
    stdin.isatty = lambda: True
    monkeypatch.setattr("sys.stdin", stdin)


# -- (1) the CSV lands beside the take, named, absolute ---------------------- #
def test_export_defaults_to_the_take_dir_and_the_recorded_name(tmp_path, no_tty,
                                                               monkeypatch, capsys):
    take = _take(tmp_path)
    monkeypatch.chdir(tmp_path.parent)          # NOT the take's dir (take2)
    assert main(["export", str(take)]) == 0
    csv = tmp_path / "wave_motion.csv"
    out = capsys.readouterr().out
    assert csv.exists() and f"[saved] {csv}:" in out
    assert load_csv(csv).meta["name"] == "wave"
    assert f"mkit-teach check {csv} --ascii" in out
    assert f"mkit-teach play {csv} --dry-run" in out


def test_saved_and_check_report_the_same_duration(tmp_path, no_tty, capsys):
    take = _take(tmp_path)
    assert main(["export", str(take)]) == 0
    saved = re.search(r"\[saved\] .*?, ([\d.]+) s on the daemon's spline",
                      capsys.readouterr().out).group(1)
    assert main(["check", str(tmp_path / "wave_motion.csv")]) == 0
    checked = re.search(r"keyframes, ([\d.]+) s on the daemon's spline",
                        capsys.readouterr().out).group(1)
    assert saved == checked


def test_export_without_any_name_says_how(tmp_path, no_tty):
    doc = json.loads((DATA / "wave_take.json").read_text())
    path = tmp_path / "anon.json"
    path.write_text(json.dumps(doc))
    with pytest.raises(SystemExit, match="give --name"):
        main(["export", str(path)])


def test_export_asks_sentiment_and_usage_only_on_a_terminal(tmp_path, monkeypatch):
    take = _take(tmp_path)
    _tty(monkeypatch, "cheerful\npositive\nopening ending\n")   # bad, then good
    assert main(["export", str(take)]) == 0
    meta = load_csv(tmp_path / "wave_motion.csv").meta
    assert meta["sentiment"] == "positive" and meta["usage"] == "opening ending"
    args = build_parser().parse_args(["export", str(take)])
    assert ask_labels(args, interactive=False) == ("neutral", ["filler"])
    args = build_parser().parse_args(["export", str(take), "--sentiment", "thoughtful",
                                      "--usage", "ending"])
    assert ask_labels(args, ask=lambda _p: pytest.fail("asked"),
                      interactive=True) == ("thoughtful", ["ending"])


# -- record: the name first, the take at <teach dir>/<name>.json -------------- #
def _record_args(*argv):
    return build_parser().parse_args(["record", *argv])


def test_record_asks_the_name_and_puts_the_take_in_the_teach_dir(tmp_path, monkeypatch):
    monkeypatch.setenv("MKIT_TEACH_DIR", str(tmp_path))
    answers = iter(["Wave!", "wave"])
    path, name = resolve_take(_record_args(), ask=lambda _p: next(answers),
                              interactive=True)
    assert (path, name) == (tmp_path / "wave.json", "wave")


def test_an_existing_take_is_overwritten_only_when_asked(tmp_path, monkeypatch):
    monkeypatch.setenv("MKIT_TEACH_DIR", str(tmp_path))
    (tmp_path / "wave.json").write_text("{}")
    answers = iter(["wave", "n", "wave2"])
    assert resolve_take(_record_args(), ask=lambda _p: next(answers),
                        interactive=True) == (tmp_path / "wave2.json", "wave2")
    answers = iter(["wave", "o"])
    assert resolve_take(_record_args(), ask=lambda _p: next(answers),
                        interactive=True) == (tmp_path / "wave.json", "wave")
    with pytest.raises(SystemExit, match="exists"):
        resolve_take(_record_args("--name", "wave"), interactive=False)
    assert resolve_take(_record_args("--name", "wave", "--yes"),
                        interactive=False) == (tmp_path / "wave.json", "wave")


def test_record_without_a_terminal_needs_a_name_or_a_path(tmp_path, monkeypatch):
    monkeypatch.setenv("MKIT_TEACH_DIR", str(tmp_path))
    with pytest.raises(SystemExit, match="--name"):
        resolve_take(_record_args(), interactive=False)
    path, name = resolve_take(_record_args(str(tmp_path / "x" / "take1.json")),
                              interactive=False)
    assert path == tmp_path / "x" / "take1.json" and name == "take1"


def test_the_default_teach_dir_is_home_teach(monkeypatch):
    from manipulation_kit.teach.cli import teach_dir
    monkeypatch.delenv("MKIT_TEACH_DIR", raising=False)
    assert teach_dir() == Path("~/teach").expanduser().resolve()


def test_record_keeps_the_name_in_the_recording(daemon, robot_factory):
    from manipulation_kit.teach import record
    clock = daemon.fake_clock
    with robot_factory() as robot:
        rec = record(robot, home=list(HOME), duration_s=0.5, countdown_s=0,
                     name="wave", sleep=clock.sleep, clock=clock)
    assert rec.meta["name"] == "wave"


# -- (2) speed: said when it stretched, carried in the CSV, honoured ---------- #
def test_the_default_ceiling_is_shus_150_and_600():
    assert (DEFAULT_SPEED.max_joint_vel_deg_s, DEFAULT_SPEED.max_joint_acc_deg_s2) == (150, 600)


def test_export_says_how_much_it_slowed_the_gesture(tmp_path, no_tty, capsys):
    take = _fast_take(tmp_path)
    assert main(["export", str(take), "--max-joint-vel", "25",
                 "--max-joint-acc", "120"]) == 0
    out = capsys.readouterr().out
    line = next(l for l in out.splitlines() if l.startswith("speed: stretched"))
    assert "to meet 25 deg/s, 120 deg/s^2" in line and "L1 peak 113 deg/s" in line
    assert "--max-joint-vel" in line
    meta = load_csv(tmp_path / "swing_motion.csv").meta
    assert (meta["max_joint_vel"], meta["max_joint_acc"]) == ("25", "120")
    assert meta["speed_stretch"].startswith("stretched 2.95 s ->")


def test_nothing_is_said_about_speed_when_nothing_was_stretched(tmp_path, no_tty, capsys):
    assert main(["export", str(_take(tmp_path))]) == 0
    assert "speed:" not in capsys.readouterr().out
    assert "speed_stretch" not in load_csv(tmp_path / "wave_motion.csv").meta


def test_check_and_play_hold_a_csv_to_its_own_ceiling(tmp_path, no_tty, capsys):
    take = _fast_take(tmp_path)
    assert main(["export", str(take), "--no-speed-limit",
                 "--max-joint-vel", "1000", "--max-joint-acc", "100000"]) == 0
    csv = tmp_path / "swing_motion.csv"
    assert SpeedPolicy.of(load_csv(csv)) == SpeedPolicy(1000, 100000)
    assert main(["check", str(csv)]) == 0                    # its own ceiling
    assert main(["play", str(csv), "--dry-run"]) == 0
    assert main(["check", str(csv), "--max-joint-vel", "25"]) == 1   # override
    assert main(["play", str(csv), "--dry-run", "--max-joint-vel", "25"]) == 1
    # a CSV without the keys is held to the default
    bare = tmp_path / "bare_motion.csv"
    bare.write_text("\n".join(l for l in csv.read_text().splitlines()
                              if "max_joint_" not in l) + "\n")
    assert not check_gesture(load_csv(bare), list(HOME), step_s=0.05).ok


# -- Next: every subcommand ends with the command to run ---------------------- #
def test_next_steps_walk_the_whole_flow(tmp_path):
    take, csv = tmp_path / "wave.json", tmp_path / "wave_motion.csv"
    assert next_steps("record", take=take, name="wave") == [
        f"mkit-teach export {take}   # writes {csv}"]
    assert next_steps("export", csv=csv, name="wave") == [
        f"mkit-teach check {csv} --ascii", f"mkit-teach play {csv} --dry-run"]
    assert next_steps("check", csv=csv) == [f"mkit-teach play {csv} --dry-run"]
    assert next_steps("play", csv=csv, dry_run=True) == [f"mkit-teach play {csv}"]
    done = next_steps("play", csv=csv, name="wave")
    assert done[0].startswith("mkit-teach register wave --yaml ")
    assert str(csv) in done[0] and done[1].startswith("mkit-teach record ")


def test_a_failure_prints_the_command_that_fixes_it(tmp_path):
    csv = tmp_path / "wave_motion.csv"
    fix = next_steps("record", ok=False, take=tmp_path / "wave.json",
                     url="http://127.0.0.1:4750", unrecovered=("b",))
    assert "Arms -> Recover" in fix[0]
    assert fix[1] == ("curl -X POST http://127.0.0.1:4750/v1/arm/b/recover "
                      "-H 'Content-Type: application/json' -d '{}'")
    assert fix[-1].startswith(f"mkit-teach record {tmp_path / 'wave.json'}")
    assert next_steps("check", ok=False, csv=csv)[0].startswith("mkit-teach export ")
    assert next_steps("play", ok=False, csv=csv, dry_run=True) == [
        f"mkit-teach check {csv} --ascii   # see what was refused"]


def test_every_offline_subcommand_prints_next(tmp_path, no_tty, capsys):
    take = _take(tmp_path)
    csv = tmp_path / "wave_motion.csv"
    for argv in (["keyframes", str(take), str(tmp_path / "wave.keys.json")],
                 ["export", str(take)], ["check", str(csv)],
                 ["play", str(csv), "--dry-run"], ["register", "wave"]):
        main(argv)
        out = capsys.readouterr().out
        assert "\nNext:\n  mkit-teach " in out, argv
