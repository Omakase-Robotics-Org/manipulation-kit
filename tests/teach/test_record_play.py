"""record -> keyframes -> export -> play, against a fake daemon.

The hand-guiding calls are checked as DATA against what ``gesture_record``
did on the vendor SDK (compliance parameters, the settle, the position hold
on every exit), and the brake path against d1-firmware PR #92's contract."""
from __future__ import annotations

import math

import numpy as np
import pytest

from manipulation_kit.teach import (export, keyframes_from_samples, load_home,
                                    parse_csv, record, to_csv, trajectory_points)
from manipulation_kit.teach.gesture_csv import Gesture, Keyframe
from manipulation_kit.teach.play import play
from manipulation_kit.teach.record import COMPLIANCE, HOLD_RATIO, Recording

HOME = load_home()


def wave(t):
    """The operator swings arm A's J2 8 deg outward (away from the belly) and
    back over 4 s."""
    a = list(HOME[:7])
    a[1] -= 8.0 * math.sin(math.pi * min(max(t - 1.5, 0.0), 4.0) / 4.0)
    return {"a": a, "b": list(HOME[7:])}


def _record(robot, daemon, **kw):
    clock = daemon.fake_clock
    kw.setdefault("duration_s", 6.0)
    kw.setdefault("countdown_s", 0)
    return record(robot, home=HOME, sleep=clock.sleep, clock=clock, **kw)


def test_compliance_record_is_gesture_records_sequence(daemon, robot_factory):
    daemon.hand = wave
    with robot_factory(lease_class="operator") as robot:
        rec = _record(robot, daemon, arms=("left",), guide="compliance")
    lease = [b for m, p, b in daemon.calls if p == "/v1/arm/lease" and m == "POST"]
    assert lease[0]["class"] == "operator"
    modes = daemon.posts("/a/mode")
    soft = [b for _, b in modes if b["mode"] == "force_compliance"]
    assert len(soft) == 1
    for key, value in COMPLIANCE.items():
        assert soft[0][key] == value, key
    assert soft[0]["holder"] == robot.holder
    assert not daemon.posts("/b/mode")[1:], "the untaught arm stays in position"
    assert daemon.posts("/a/recover") == [("/v1/arm/a/recover",
                                           {"vel_ratio": HOLD_RATIO, "acc_ratio": HOLD_RATIO})]
    assert not daemon.posts("brake_release")
    # sampled at 20 Hz for the 6 s window, timestamps from the executor clock
    assert 110 <= len(rec.samples) <= 121
    assert rec.times == sorted(rec.times)
    assert min(s[1] for s in rec.samples) == pytest.approx(HOME[1] - 8.0, abs=0.1)
    assert rec.grippers[0] == {"left": pytest.approx(0.0), "right": pytest.approx(0.0)}
    assert ("DELETE", "/v1/arm/lease") in [(m, p) for m, p, _ in daemon.calls]


def test_the_teach_starts_by_moving_to_home_unrecorded(daemon, robot_factory):
    daemon.pose["a"][1] += 20.0
    with robot_factory() as robot:
        _record(robot, daemon, duration_s=1.0)
    starts = [b for _, b in daemon.posts("/v1/arm/trajectory/start")]
    assert len(starts) == 1
    assert starts[0]["waypoints"][-1]["a"] == HOME[:7]


def test_brake_guide_idles_then_releases_and_always_engages(daemon, robot_factory):
    with robot_factory() as robot:
        with pytest.raises(RuntimeError, match="injected"):
            daemon.hand = wave
            calls = {"n": 0}

            def stop():
                calls["n"] += 1
                if calls["n"] == 5:
                    daemon.fail_on = "/v1/arm/a/state"
                return False
            _record(robot, daemon, stop=stop, duration_s=None)   # default guide
    daemon.fail_on = None
    order = [p for m, p, b in daemon.calls if m == "POST" and "/a/" in p]
    assert order.index("/v1/arm/a/mode") < order.index("/v1/arm/a/brake_release")
    release = daemon.posts("brake_release")[0][1]
    assert release["confirm"] == "RELEASE_BRAKE" and 1 <= release["seconds"] <= 120
    # every exit path: engaged, then a measured-pose position hold
    assert order.index("/v1/arm/a/brake_engage") < order.index("/v1/arm/a/recover")
    assert daemon.brakes == {"a": False, "b": False}


def test_the_default_guide_is_brake_release(daemon, robot_factory):
    with robot_factory() as robot:
        rec = _record(robot, daemon, duration_s=0.5)
    assert rec.guide == "brake"
    assert [p for p, _ in daemon.posts("brake_release")] == [
        "/v1/arm/a/brake_release", "/v1/arm/b/brake_release"]
    assert [b["mode"] for _, b in daemon.posts("/mode")][-2:] == ["idle", "idle"]


def test_countdown_then_release_then_t0(daemon, robot_factory):
    """3, 2, 1 -> brakes open -> the first sample IS t = 0: nothing before
    the release is part of the take."""
    events = []
    real_release = daemon.brake_release

    def release(wire, **kw):
        events.append(("release", daemon.fake_clock()))
        return real_release(wire, **kw)
    daemon.brake_release = release
    with robot_factory() as robot:
        rec = _record(robot, daemon, duration_s=0.5, countdown_s=3,
                      on_state=lambda state, msg: events.append((state, msg)))
    ticks = [m for s, m in events if s == "starting" and m in ("3", "2", "1")]
    assert ticks == ["3", "2", "1"]
    first_release = next(i for i, e in enumerate(events) if e[0] == "release")
    assert events.index(("starting", "1")) < first_release
    assert rec.times[0] == 0.0
    # the 3 s countdown is not in the take
    assert rec.times[-1] == pytest.approx(0.5, abs=0.06)


def test_stop_engages_the_brakes_before_anything_else(daemon, robot_factory):
    stops = {"n": 0}

    def stop():
        stops["n"] += 1
        return stops["n"] > 4
    with robot_factory() as robot:
        _record(robot, daemon, stop=stop, duration_s=None)
    posts = [p for m, p, _ in daemon.calls if m == "POST" or m == "DELETE"]
    first_engage = next(i for i, p in enumerate(posts) if p.endswith("brake_engage"))
    last_release = max(i for i, p in enumerate(posts) if p.endswith("brake_release"))
    assert last_release < first_engage
    # the two engages come first, before any recover / mode / lease change
    assert posts[first_engage:first_engage + 2] == [
        "/v1/arm/a/brake_engage", "/v1/arm/b/brake_engage"]
    assert posts[first_engage + 2].endswith("/recover")


def test_the_keeper_cannot_release_after_the_engage():
    """halt() waits out a renewal already on the wire; after it no renewal
    goes out, so an engage sent after halt() is the last brake write."""
    import threading

    from manipulation_kit.teach.record import _BrakeKeeper
    wire_log, entered, gate = [], threading.Event(), threading.Event()

    class Client:
        def brake_release(self, wire, **_kw):
            entered.set()
            gate.wait(2.0)
            wire_log.append(("release", wire))
    keeper = _BrakeKeeper(Client(), ["a"], 0.03, "me")
    keeper.start()
    assert entered.wait(2.0)            # a renewal is in flight
    halted = threading.Thread(target=keeper.halt)
    halted.start()
    gate.set()
    halted.join(2.0)
    wire_log.append(("engage", "a"))
    keeper.join(2.0)
    assert wire_log[-1] == ("engage", "a")
    assert not keeper.is_alive()


def test_the_take_must_start_at_home(daemon, robot_factory):
    daemon.pose["a"][1] += 5.0
    with robot_factory() as robot:
        with pytest.raises(RuntimeError, match="not at HOME at Start"):
            _record(robot, daemon, home_start=False)
    soft = [b for _, b in daemon.posts("/mode") if b["mode"] != "position"]
    assert not soft and not daemon.posts("brake_release")


def test_idle_guide_touches_no_brake(daemon, robot_factory):
    with robot_factory() as robot:
        _record(robot, daemon, guide="idle", duration_s=0.5)
    assert not daemon.posts("brake_release") and not daemon.posts("brake_engage")
    assert [b["mode"] for _, b in daemon.posts("/a/mode")][-1] == "idle"


def test_compliance_on_a_bare_flange_is_refused_before_going_soft(daemon, robot_factory):
    daemon.tool_source = "none"
    with robot_factory() as robot:
        with pytest.raises(RuntimeError, match="NO tool registered"):
            _record(robot, daemon, guide="compliance")
    assert not [b for _, b in daemon.posts("/mode") if b["mode"] == "force_compliance"]


def test_stationary_auto_stop(daemon, robot_factory):
    with robot_factory() as robot:
        rec = _record(robot, daemon, duration_s=None, stationary_s=1.0)
    assert rec.times[-1] == pytest.approx(1.0, abs=0.1)


def test_keyframe_mode_captures_one_pose_per_request(daemon, robot_factory):
    poses = iter([True, True, False])
    with robot_factory() as robot:
        rec = _record(robot, daemon, mode="keyframe",
                      next_keyframe=lambda: next(poses))
    assert rec.mode == "keyframe" and len(rec.samples) == 2


def test_a_recording_round_trips_as_json(tmp_path, daemon, robot_factory):
    with robot_factory() as robot:
        rec = _record(robot, daemon, duration_s=0.5)
    rec.save(tmp_path / "take.json")
    back = Recording.load(tmp_path / "take.json")
    assert back.samples == rec.samples and back.guide == "brake"


def test_end_to_end_the_taught_wave_exports_checks_and_plays(daemon, robot_factory):
    daemon.hand = wave
    with robot_factory(lease_class="operator") as robot:
        rec = _record(robot, daemon, arms=("left",), duration_s=7.0)
    g = keyframes_from_samples(rec.times, rec.samples, HOME)
    out, report = export(g, HOME, name="wave", check_kwargs={"step_s": 0.05})
    assert report.ok, report.summary()
    text = to_csv(out)
    assert "# mkit-teach: name=wave" in text
    parsed = parse_csv(text)
    assert parsed.array()[0] == pytest.approx(HOME)
    assert min(r[1] for r in parsed.array()) == pytest.approx(HOME[1] - 8.0, abs=0.3)

    daemon.calls.clear()
    daemon.hand = None
    with robot_factory() as robot:
        result = play(robot, parsed, HOME, check_kwargs={"step_s": 0.05})
    assert result.ok, result.detail
    lease = [b for m, p, b in daemon.calls if p == "/v1/arm/lease" and m == "POST"]
    assert lease[0]["class"] == "policy"
    uploads = [b for _, b in daemon.posts("/v1/arm/trajectory/start")]
    assert len(uploads) == 1, "already at HOME: no approach move"
    sent = uploads[0]["waypoints"]
    expected = trajectory_points(parsed, HOME)
    assert [w["t"] for w in sent] == pytest.approx([p["t"] for p in expected])
    assert sent[0]["a"] == pytest.approx(HOME[:7])      # re-anchored to measured
    assert uploads[0]["holder"] == robot.holder


def test_a_force_saved_unsafe_csv_does_not_play_without_no_safety(daemon, robot_factory):
    g = Gesture([Keyframe(0.05, HOME), Keyframe(2.0, HOME)],
                unsafe=["arm A link Link2_R within 0.010 m of body box torso_belly"])
    with robot_factory() as robot:
        result = play(robot, parse_csv(to_csv(g)), HOME)
    assert not result.ok and "UNSAFE" in result.detail
    assert not daemon.posts("/v1/arm/trajectory/start")


def test_a_gesture_that_fails_the_check_moves_nothing(daemon, robot_factory):
    delta = np.zeros(14)
    delta[12] = 70.0                              # B J6 past its +/-60 limit
    pose = list(np.array(HOME) + delta)
    g = Gesture([Keyframe(0.05, HOME), Keyframe(8.0, pose), Keyframe(8.0, HOME)])
    with robot_factory() as robot:
        result = play(robot, g, HOME, check_kwargs={"step_s": 0.05})
    assert not result.ok and "pre-flight" in result.detail
    assert not daemon.posts("/v1/arm/trajectory/start")


def test_a_guard_only_finding_warns_before_playing_and_is_not_a_refusal(
        daemon, robot_factory):
    """A J2 +30 deg into the belly: the kit's guard objects, but a taught pose
    was reached by hand, so the kit warns and hands it to the daemon (which
    runs its own guard; this fake does not)."""
    pose = list(np.array(HOME) + np.r_[0, 30.0, np.zeros(12)])
    g = Gesture([Keyframe(0.05, HOME), Keyframe(3.0, pose), Keyframe(3.0, HOME)])
    said = []
    real_start = daemon.request

    def request(method, path, body=None):
        if path == "/v1/arm/trajectory/start":
            said.append("UPLOAD")
        return real_start(method, path, body)
    daemon.request = request
    with robot_factory() as robot:
        result = play(robot, g, HOME, check_kwargs={"step_s": 0.05},
                      announce=said.append)
    assert result.ok, result.detail
    assert said[0].startswith("WARNING: guard (advisory) body")
    assert said.index("UPLOAD") > 0                # warned BEFORE the upload
    assert any("guard (advisory)" in n for n in result.notes)


def test_play_approaches_home_first_when_the_arms_are_elsewhere(daemon, robot_factory):
    daemon.pose["b"][3] += 10.0
    g = Gesture([Keyframe(0.05, HOME), Keyframe(2.0, HOME)])
    with robot_factory() as robot:
        result = play(robot, g, HOME, no_safety=True)
    assert result.ok and result.approached
    assert len(daemon.posts("/v1/arm/trajectory/start")) == 2
