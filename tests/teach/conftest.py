"""A d1-firmwared-shaped fake for the teach suite. Nothing opens a socket.

It speaks what :class:`FirmwareExecutor` and :mod:`manipulation_kit.teach`
call — the enveloped ``request`` (lease, mode, trajectory), the generated-model
reads (``arm_state``, ``gripper_state``, ``arm_tool_state``) and the teach
verbs (``arm_mode``, ``arm_recover``, ``brake_release``, ``brake_engage``) —
and records every call as data. The arms FOLLOW: a completed trajectory leaves
them at its last knot, and while an arm is soft (``force_compliance`` /
``idle``) its feedback is whatever ``hand(t)`` — the operator — says.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any, Callable, Dict, List, Optional, Tuple

import pytest

from manipulation_kit.executors.firmware import FirmwareExecutor
from manipulation_kit.teach import load_home

HOME = load_home()


@dataclass
class FakeDaemon:
    pose: Dict[str, List[float]] = field(default_factory=lambda: {
        "a": list(HOME[:7]), "b": list(HOME[7:])})
    modes: Dict[str, str] = field(default_factory=lambda: {"a": "position", "b": "position"})
    #: the operator's hands: t -> {"a": 7 deg, "b": 7 deg} for SOFT arms
    hand: Optional[Callable[[float], Dict[str, List[float]]]] = None
    clock: Callable[[], float] = lambda: 0.0
    tool_source: str = "config"
    open_rad: Optional[float] = 1.16
    jaw_rad: float = 1.16
    calls: List[Tuple[str, str, Any]] = field(default_factory=list)
    brakes: Dict[str, bool] = field(default_factory=lambda: {"a": False, "b": False})
    fail_on: Optional[str] = None
    _job: Optional[list] = None
    spec_sha256: str = "f" * 64
    #: the document's TrajectoryGuard values; () = a daemon before PR #102
    guards: Tuple[str, ...] = ("full", "speed_only")

    def _log(self, method, path, body=None):
        self.calls.append((method, path, body))
        if self.fail_on and self.fail_on in path:
            raise RuntimeError(f"injected failure on {path}")

    # -- enveloped REST ----------------------------------------------------- #
    def request(self, method: str, path: str, body: Any = None) -> Any:
        self._log(method, path, body)
        if path == "/v1/arm/lease" and method == "POST":
            return {"holder": body["holder"], "class": body.get("class"),
                    "epoch": 1, "ttl_s": body.get("ttl_s", 30),
                    "expires_in_s": 30, "preempted_from": None}
        if path == "/v1/arm/lease":
            return None
        if path.endswith("/mode"):
            self.modes[path.split("/")[3]] = body["mode"]
            return None
        if path == "/v1/arm/trajectory/start":
            self._job = body["waypoints"]
            return {"id": 9, "phase": "running", "elapsed_ms": 0,
                    "guard": body.get("guard", "full")}
        if path.endswith("/cancel"):
            self._job = None
            return None
        if path.startswith("/v1/arm/trajectory/"):
            if self._job:
                last = self._job[-1]
                self.pose = {"a": list(last["a"]), "b": list(last["b"])}
                self._job = None
            return {"id": 9, "phase": "completed", "elapsed_ms": 1}
        raise AssertionError(f"unexpected {method} {path}")

    def trajectory_guards(self) -> Tuple[str, ...]:
        return tuple(self.guards)

    # -- generated-model reads ---------------------------------------------- #
    def arm_state(self, wire: str):
        self._log("GET", f"/v1/arm/{wire}/state")
        if self.hand is not None and self.modes[wire] in ("force_compliance", "idle"):
            self.pose[wire] = list(self.hand(self.clock())[wire])
        q = tuple(self.pose[wire])
        mode = {"force_compliance": "torque"}.get(self.modes[wire], self.modes[wire])
        return SimpleNamespace(mode=mode, error_code=0, feedback_joints=q,
                               command_joints=q, feedback_velocity=(0.0,) * 7,
                               feedback_torque=(0.0,) * 7, stationary=True)

    def gripper_state(self, wire: str):
        return SimpleNamespace(kind="open", jaw_rad=self.jaw_rad, torque_nm=0.0,
                               holding=False, open_rad=self.open_rad,
                               target_closedness=None, fault_code=None)

    def arm_tool_state(self, wire: str):
        return SimpleNamespace(source=SimpleNamespace(value=self.tool_source))

    # -- teach verbs --------------------------------------------------------- #
    def arm_mode(self, wire, mode, *, holder=None, **fields):
        self._log("POST", f"/v1/arm/{wire}/mode", dict(fields, mode=mode, holder=holder))
        self.modes[wire] = mode

    def arm_recover(self, wire, *, vel_ratio=None, acc_ratio=None):
        self._log("POST", f"/v1/arm/{wire}/recover",
                  {"vel_ratio": vel_ratio, "acc_ratio": acc_ratio})
        if self.brakes[wire]:
            raise AssertionError("recover while the brakes are released")
        self.modes[wire] = "position"

    def brake_release(self, wire, *, seconds, holder=None):
        self._log("POST", f"/v1/arm/{wire}/brake_release",
                  {"confirm": "RELEASE_BRAKE", "seconds": seconds, "holder": holder})
        if self.modes[wire] not in ("idle", "error"):
            raise AssertionError("brake_release on an arm whose servos are on")
        self.brakes[wire] = True

    def brake_engage(self, wire):
        self._log("POST", f"/v1/arm/{wire}/brake_engage", None)
        self.brakes[wire] = False

    def posts(self, suffix: str):
        return [(p, b) for m, p, b in self.calls if m == "POST" and p.endswith(suffix)]


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += max(0.0, float(seconds))


@pytest.fixture
def daemon():
    clock = FakeClock()
    fake = FakeDaemon(clock=clock)
    fake.fake_clock = clock
    return fake


@pytest.fixture
def robot_factory(daemon):
    def make(**kw):
        clock = daemon.fake_clock
        return FirmwareExecutor(client=daemon, heartbeat=False, sleep=clock.sleep,
                                clock=clock, **kw)
    return make
