"""The first live ``mkit-teach record`` on d1-2 (2026-09-23 20:16Z), as tests.

What the daemon journal showed: the idle request answered in 0 ms, the brake
release 5 ms later was refused because the controller still reported
``position`` (it switched 11 ms after the request); the recover 2 ms after the
brake engage was refused while the mode flapped idle/error; and an arm the
operator had left idle 40 degrees from its command killed the session in
``FirmwareExecutor.__enter__``. The fake here reports a requested mode only
after ``mode_lag`` state reads, refuses recovers on request, and carries a
commanded-vs-measured gap — the three things the live run had and the old
fake did not.
"""
from __future__ import annotations

import pytest

from manipulation_kit.executors.firmware import FirmwareUnavailable, ModeUnconfirmed
from manipulation_kit.executors.firmware.executor import (RECOVER_ATTEMPTS,
                                                          RECOVER_SPACING_S)
from manipulation_kit.teach import load_csv, record, save_csv
from manipulation_kit.teach.gesture_csv import Gesture, Keyframe
from manipulation_kit.teach.play import play

from .conftest import HOME


def _record(robot, daemon, **kw):
    clock = daemon.fake_clock
    kw.setdefault("duration_s", 0.5)
    kw.setdefault("countdown_s", 0)
    kw.setdefault("arms", ("right",))
    return record(robot, home=HOME, sleep=clock.sleep, clock=clock, **kw)


def _reported_before_release(daemon):
    """Each brake_release, with the mode the arm REPORTED at that moment."""
    seen = []
    real = daemon.brake_release

    def release(wire, **kw):
        seen.append((wire, daemon.modes[wire]))
        return real(wire, **kw)
    daemon.brake_release = release
    return seen


# (a) --------------------------------------------------------------------- #
def test_brake_release_waits_for_the_reported_idle(daemon, robot_factory):
    """Attempt A, fixed: the idle request is reported only 6 reads later; the
    release goes out after the arm REPORTS idle, never on the acceptance."""
    seen = _reported_before_release(daemon)
    with robot_factory() as robot:
        daemon.mode_lag = 6
        rec = _record(robot, daemon)
    assert seen == [("b", "idle")]
    calls = [(m, p) for m, p, _ in daemon.calls]
    idle_at = calls.index(("POST", "/v1/arm/b/mode"))
    release_at = calls.index(("POST", "/v1/arm/b/brake_release"))
    reads = [c for c in calls[idle_at:release_at] if c == ("GET", "/v1/arm/b/state")]
    assert len(reads) >= 7
    assert len(rec.samples) >= 2 and rec.meta["exit_problems"] == []


def test_idle_then_confirm_then_countdown_then_release(daemon, robot_factory):
    """The order the operator sees: servos off (confirmed; brakes holding),
    3-2-1, and the release lands on "0", which is t = 0 of the take."""
    events = []
    real = daemon.brake_release
    daemon.brake_release = lambda wire, **kw: (events.append(("release", daemon.modes[wire])),
                                               real(wire, **kw))[1]
    real_mode = daemon.arm_mode
    daemon.arm_mode = lambda wire, mode, **kw: (events.append(("mode", mode)),
                                                real_mode(wire, mode, **kw))[1]
    with robot_factory() as robot:
        daemon.mode_lag = 3
        rec = _record(robot, daemon, countdown_s=3,
                      on_state=lambda s, m: events.append((s, m)))
    order = [e for e in events if e[0] in ("mode", "release") or e[1] in ("3", "2", "1")]
    assert order == [("mode", "idle"), ("starting", "3"), ("starting", "2"),
                     ("starting", "1"), ("release", "idle")]
    assert rec.times[0] == 0.0


# (b) --------------------------------------------------------------------- #
def test_a_mode_never_reported_times_out_and_never_releases(daemon, robot_factory):
    with robot_factory() as robot:
        daemon.stuck = ("b",)
        with pytest.raises(ModeUnconfirmed, match=r"arm b: waited 3 s for mode "
                                                  r"idle; it last reported mode 'position'"):
            _record(robot, daemon)
    assert not daemon.posts("brake_release")
    assert daemon.brakes == {"a": False, "b": False}


def test_an_error_after_the_idle_request_is_a_fault_not_a_wait(daemon, robot_factory):
    with robot_factory() as robot:
        real_mode = daemon.arm_mode

        def fault(wire, mode, **kw):
            real_mode(wire, mode, **kw)
            daemon._due[wire] = ["error", 0]
        daemon.arm_mode = fault
        with pytest.raises(ModeUnconfirmed, match="reported mode 'error'"):
            _record(robot, daemon)
    assert not daemon.posts("brake_release")


# (c) --------------------------------------------------------------------- #
def test_teardown_retries_a_refused_recover_one_second_apart(daemon, robot_factory):
    with robot_factory() as robot:
        daemon.recover_refusals = 2
        rec = _record(robot, daemon)
    assert len(daemon.posts("/b/recover")) == 3
    gaps = [b - a for a, b in zip(daemon.recover_times, daemon.recover_times[1:])]
    assert all(g >= RECOVER_SPACING_S for g in gaps), gaps
    assert daemon.modes["b"] == "position"
    assert rec.meta["exit_problems"] == []
    # brakes first, and the recover only after the engage
    posts = [p for m, p, _ in daemon.calls if m == "POST"]
    assert posts.index("/v1/arm/b/brake_engage") < posts.index("/v1/arm/b/recover")


def test_teardown_waits_for_a_steady_mode_before_recovering(daemon, robot_factory):
    """After the engage the arm flaps idle/error; no recover goes out until
    one mode has held, stationary, for 0.3 s."""
    flaps = iter(["error", "idle", "error", "idle"])
    real_engage = daemon.brake_engage
    engaged_at = {}

    def engage(wire):
        real_engage(wire)
        engaged_at[wire] = daemon.fake_clock()
        real_state = daemon.arm_state

        def flapping(w):
            state = real_state(w)
            word = next(flaps, None)
            if w == wire and word is not None:
                state.mode = word
            return state
        daemon.arm_state = flapping
    daemon.brake_engage = engage
    with robot_factory() as robot:
        rec = _record(robot, daemon)
    assert daemon.recover_times[0] - engaged_at["b"] >= 0.3
    assert rec.meta["exit_problems"] == []


def test_teardown_refused_three_times_leaves_the_arm_safe_and_says_how(
        daemon, robot_factory):
    lines = []
    with robot_factory() as robot:
        daemon.recover_refusals = 99
        rec = _record(robot, daemon, on_state=lambda s, m: lines.append(m))
    assert len(daemon.posts("/b/recover")) == RECOVER_ATTEMPTS == 3
    [problem] = rec.meta["exit_problems"]
    assert "IDLE with its holding brakes ENGAGED" in problem
    assert "the DAEMON REFUSED the recover (3 attempt(s))" in problem
    assert "KIT GAVE UP" not in problem
    assert "Arms -> Recover" in problem and "POST /v1/arm/b/recover" in problem
    assert daemon.brakes["b"] is False and daemon.modes["b"] == "idle"
    assert any(line.startswith("WARNING: arm b was not put back") for line in lines)


# (d) --------------------------------------------------------------------- #
def test_record_recovers_an_idle_arm_far_from_its_command(daemon, robot_factory):
    """Attempt B, fixed: arm b idle, 40.2 deg from its command. Teach
    recovers it (announced), THEN engages position and goes to HOME."""
    daemon.modes["b"], daemon.command_gap["b"] = "idle", 40.2
    said = []
    with robot_factory(recover_on_entry=True, announce=said.append) as robot:
        rec = _record(robot, daemon)
    assert said == ["recovering arm b (idle, 40.2 deg from its command)"]
    assert rec.meta["entry_recoveries"] == said
    posts = [p for m, p, _ in daemon.calls if m == "POST"]
    assert posts[:2] == ["/v1/arm/lease", "/v1/arm/b/recover"]
    assert posts.index("/v1/arm/b/recover") < posts.index("/v1/arm/b/mode")
    assert [b["mode"] for _, b in daemon.posts("/b/mode")][0] == "position"


def test_an_arm_already_in_position_is_not_recovered(daemon, robot_factory):
    said = []
    with robot_factory(recover_on_entry=True, announce=said.append) as robot:
        rec = _record(robot, daemon)
    assert said == [] and rec.meta["entry_recoveries"] == []
    # the only recover is the teardown's, after the take
    assert len(daemon.posts("/b/recover")) == 1
    assert not daemon.posts("/a/recover")


def test_the_untaught_arm_is_recovered_too(daemon, robot_factory):
    """Position mode and the HOME move drive BOTH arms, so an idle arm a
    blocks a right-arm teach just the same."""
    daemon.modes["a"], daemon.command_gap["a"] = "error", 12.0
    with robot_factory(recover_on_entry=True) as robot:
        _record(robot, daemon, arms=("right",))
    assert len(daemon.posts("/a/recover")) == 1


def test_the_generic_executor_still_refuses_a_stale_idle_arm(daemon, robot_factory):
    """No silent recover for an agent loop: without recover_on_entry the
    executor refuses, as before, and hands the lease back."""
    daemon.modes["b"], daemon.command_gap["b"] = "idle", 40.2
    with pytest.raises(FirmwareUnavailable, match="40.2 deg from the measured one"):
        with robot_factory():
            pass
    assert not daemon.posts("/recover")
    assert ("DELETE", "/v1/arm/lease") in [(m, p) for m, p, _ in daemon.calls]


# (e) --------------------------------------------------------------------- #
def test_play_recovers_an_idle_arm_first(daemon, robot_factory, tmp_path):
    gesture = Gesture([Keyframe(1.0, list(HOME)), Keyframe(1.0, list(HOME))])
    path = tmp_path / "still_motion.csv"
    save_csv(path, gesture)
    daemon.modes["a"], daemon.command_gap["a"] = "idle", 25.0
    said = []
    with robot_factory(recover_on_entry=True, announce=said.append) as robot:
        report = play(robot, load_csv(path), HOME)
    assert report.ok, report.detail
    assert said == ["recovering arm a (idle, 25.0 deg from its command)"]
    posts = [p for m, p, _ in daemon.calls if m == "POST"]
    assert posts.index("/v1/arm/a/recover") < posts.index("/v1/arm/trajectory/start")


def test_the_cli_executor_opts_into_recover_on_entry(monkeypatch):
    import argparse

    import manipulation_kit.executors.firmware as firmware
    from manipulation_kit.teach import cli
    seen = {}
    monkeypatch.setattr(firmware, "FirmwareExecutor",
                        lambda **kw: seen.update(kw) or "robot")
    args = argparse.Namespace(url="http://fake:4750", vel_ratio=0.15)
    assert cli._executor(args, lease_class="operator") == "robot"
    assert seen["recover_on_entry"] is True and callable(seen["announce"])


def test_a_client_timeout_is_not_retried_and_says_the_kit_gave_up(daemon, robot_factory):
    """d1-2 2026-09-23 20:46Z: the recover was cut by the CLIENT. Such a
    recover may still be running in the daemon, so no second one is sent,
    and the operator is told who gave up."""
    from manipulation_kit.executors.firmware import ClientTimeout
    real = daemon.arm_recover

    def slow(wire, **kw):
        daemon.recover_times.append(daemon.fake_clock())
        raise ClientTimeout("POST", f"/v1/arm/{wire}/recover", 65.0)
    with robot_factory() as robot:
        daemon.arm_recover = slow
        rec = _record(robot, daemon)
        daemon.arm_recover = real
    assert len(daemon.recover_times) == 1
    [problem] = rec.meta["exit_problems"]
    assert "the KIT GAVE UP waiting for the daemon's answer" in problem
    assert "not a refusal" in problem and "REFUSED" not in problem
    assert "IDLE with its holding brakes ENGAGED" in problem


def test_a_slow_recover_is_one_call_and_succeeds(daemon, robot_factory):
    """A recover that takes 3 s (the daemon's confirm loop) is one call and a
    success: the executor never re-issues while one is in flight."""
    real = daemon.arm_recover

    def slow(wire, **kw):
        daemon.fake_clock.sleep(3.0)
        return real(wire, **kw)
    with robot_factory() as robot:
        daemon.arm_recover = slow
        rec = _record(robot, daemon)
    assert len(daemon.posts("/b/recover")) == 1
    assert rec.meta["exit_problems"] == [] and daemon.modes["b"] == "position"
