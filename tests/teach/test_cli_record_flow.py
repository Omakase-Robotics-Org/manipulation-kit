"""``mkit-teach record``'s own operator flow after the live run of ba9a642
on d1-2 (2026-09-23): which arm(s) is asked; a Ctrl-C while recording keeps
the take; ``To fix:`` only when something really needs fixing; and no hint
ever suggests ``--yes`` (it skips the typed HOLDING confirmation)."""
from __future__ import annotations

import contextlib
import itertools
from pathlib import Path

import numpy as np
import pytest

from manipulation_kit.teach import load_home, record
from manipulation_kit.teach import cli
from manipulation_kit.teach.cli import (build_parser, confirm_holding, next_steps,
                                        resolve_arms, retry_record)
from manipulation_kit.teach.record import RecordAborted, Recording

HOME = list(load_home())


def _args(*argv):
    return build_parser().parse_args(["record", *argv])


# -- (1) which arm(s) ------------------------------------------------------- #
def test_the_arms_are_asked_after_the_name(capsys):
    answers = iter(["arm", "r"])
    assert resolve_arms(_args(), ask=lambda _p: next(answers), interactive=True) == "right"
    assert "not 'arm'" in capsys.readouterr().err
    for said, meant in (("", "both"), ("b", "both"), ("l", "left"), ("left", "left"),
                        ("RIGHT", "right")):
        assert resolve_arms(_args(), ask=lambda _p, s=said: s, interactive=True) == meant


def test_arms_on_the_command_line_or_no_terminal_asks_nothing():
    never = lambda _p: pytest.fail("asked")                  # noqa: E731
    assert resolve_arms(_args("--arms", "left"), ask=never, interactive=True) == "left"
    assert resolve_arms(_args(), ask=never, interactive=False) == "both"
    assert resolve_arms(_args("--yes"), ask=never) == "both"


def test_the_contract_names_only_the_chosen_arm(capsys):
    args = _args("--arms", "right")
    assert confirm_holding(args, cli.ARMS["right"], ask=lambda _p: "HOLDING")
    out = capsys.readouterr().out
    assert "every released arm (right)" in out and "left" not in out


def test_cmd_record_asks_name_then_arms_before_the_contract(tmp_path, monkeypatch):
    monkeypatch.setenv("MKIT_TEACH_DIR", str(tmp_path))
    monkeypatch.setattr(cli, "_interactive", lambda args=None: True)
    order = []
    answers = {"Gesture name": "wave", "Arms to teach": "l", "Type HOLDING": "no"}

    def ask(prompt):
        key = next(k for k in answers if prompt.startswith(k))
        order.append(key)
        return answers[key]
    monkeypatch.setattr("builtins.input", ask)
    assert cli.main(["record"]) == 2              # not confirmed: nothing moved
    assert order == ["Gesture name", "Arms to teach", "Type HOLDING"]


# -- (2) never --yes ---------------------------------------------------------- #
def test_no_hint_ever_suggests_yes(tmp_path):
    take, csv = tmp_path / "wave.json", tmp_path / "wave_motion.csv"
    lines = []
    for step, ok, dry, unrec, name, arms in itertools.product(
            ("record", "keyframes", "export", "check", "play", "register"),
            (True, False), (True, False), ((), ("b",)), (None, "wave"),
            (None, "left")):
        lines += next_steps(step, ok=ok, take=take, csv=csv, name=name, arms=arms,
                            unrecovered=unrec, dry_run=dry)
    lines += [retry_record(take, "wave", "left"), retry_record(None, None, None)]
    assert lines and not [line for line in lines if "--yes" in line]


def test_the_retry_command_is_prefilled_from_the_take(tmp_path, monkeypatch):
    monkeypatch.setenv("MKIT_TEACH_DIR", str(tmp_path))
    assert retry_record(tmp_path / "task3.json", "task3", "right").startswith(
        "mkit-teach record --name task3 --arms right")
    elsewhere = tmp_path / "x" / "t.json"
    assert retry_record(elsewhere, "t", None).startswith(f"mkit-teach record {elsewhere}")


# -- (3) Ctrl-C keeps the take; To fix only when needed ------------------------ #
def test_ctrl_c_while_recording_is_a_stop_that_keeps_the_take(daemon, robot_factory):
    clock = daemon.fake_clock
    n = {"k": 0}

    def stop():
        n["k"] += 1
        if n["k"] == 6:
            raise KeyboardInterrupt
        return False
    with robot_factory() as robot:
        rec = record(robot, home=HOME, arms=("right",), countdown_s=0, stop=stop,
                     sleep=clock.sleep, clock=clock)
    assert len(rec.samples) == 5 and rec.meta["stopped_by"] == "interrupt"
    assert rec.meta["exit_problems"] == []
    assert daemon.brakes == {"a": False, "b": False} and daemon.modes["b"] == "position"


def test_ctrl_c_before_two_samples_discards_the_take(daemon, robot_factory):
    clock = daemon.fake_clock

    def stop():
        raise KeyboardInterrupt
    with robot_factory() as robot:
        with pytest.raises(RecordAborted) as caught:
            record(robot, home=HOME, arms=("right",), countdown_s=0, stop=stop,
                   sleep=clock.sleep, clock=clock)
    assert caught.value.recording.meta["exit_problems"] == []
    assert daemon.posts("/b/brake_engage")


def _run_cmd(monkeypatch, tmp_path, outcome):
    """cmd_record with the robot replaced by ``outcome`` (a Recording to
    return, or an exception to raise)."""
    monkeypatch.setattr(cli, "_executor",
                        lambda args, lease_class: contextlib.nullcontext("robot"))

    def fake_record(robot, **kw):
        if isinstance(outcome, BaseException):
            raise outcome
        return outcome
    monkeypatch.setattr(cli, "record", fake_record)
    return cli.main(["record", str(tmp_path / "task3.json"), "--arms", "right",
                     "--no-brake"])


def _rec(samples, problems=()):
    rec = Recording(mode="stream", guide="brake", arms=["right"], rate_hz=20.0,
                    home=HOME, meta={"name": "task3", "exit_problems": list(problems)})
    for k in range(samples):
        rec.times.append(k / 20.0)
        rec.samples.append(HOME)
        rec.grippers.append({"left": None, "right": None})
    return rec


UNRECOVERED = ("arm b was not put back in a position hold and is left IDLE with "
               "its holding brakes ENGAGED (safe): the DAEMON REFUSED the recover")


def test_a_clean_stop_prints_recorded_and_next_and_no_fix(tmp_path, monkeypatch, capsys):
    assert _run_cmd(monkeypatch, tmp_path, _rec(40)) == 0
    out = capsys.readouterr().out
    take = tmp_path / "task3.json"
    assert f"[recorded] {take}: 40 samples" in out
    assert f"Next:\n  mkit-teach export {take}" in out and "To fix:" not in out
    assert take.exists()


def test_an_unrecovered_arm_is_the_only_thing_to_fix(tmp_path, monkeypatch, capsys):
    assert _run_cmd(monkeypatch, tmp_path, _rec(40, [UNRECOVERED])) == 0
    out = capsys.readouterr().out
    assert "To fix:" in out and "/v1/arm/b/recover" in out and "/v1/arm/a/" not in out
    assert "Next:\n  mkit-teach export" in out


def test_a_too_short_take_is_discarded_and_says_how_to_record_again(
        tmp_path, monkeypatch, capsys):
    code = _run_cmd(monkeypatch, tmp_path, RecordAborted("not enough samples (1)", _rec(1)))
    captured = capsys.readouterr()
    assert code == 1 and "take discarded, nothing saved" in captured.err
    assert "To fix:" not in captured.out
    assert f"Next:\n  mkit-teach record {tmp_path / 'task3.json'} --arms right" in captured.out
    assert not (tmp_path / "task3.json").exists()


def test_an_interrupt_before_recording_fixes_only_what_is_broken(
        tmp_path, monkeypatch, capsys):
    clean = KeyboardInterrupt()
    clean.teach_recording = _rec(0)
    assert _run_cmd(monkeypatch, tmp_path, clean) == 130
    out = capsys.readouterr().out
    assert "To fix:" not in out and "mkit-teach record" in out
    broken = KeyboardInterrupt()
    broken.teach_recording = _rec(0, [UNRECOVERED])
    assert _run_cmd(monkeypatch, tmp_path, broken) == 130
    out = capsys.readouterr().out
    assert "To fix:" in out and "/v1/arm/b/recover" in out and "--yes" not in out
