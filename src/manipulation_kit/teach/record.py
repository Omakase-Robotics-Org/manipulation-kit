"""Record a gesture by hand through d1-firmwared — the ``gesture_record`` port.

What was lost, and what this replaces
-------------------------------------
Hand-teaching used to be d1-sdk ``devices/omakase_arm/example/
gesture_record.cpp``, spawned by omakase-core's ``/d1_teach`` panel
(``status_server/d1/teach.py``). It opened the arm controller's UDP link
itself. Since the firmwared migration (omakase-core 2050ca49 "play D1 gestures
through firmware trajectories", faa32a71 "firmware is the default arm
transport", 2026-09-08..10) ``d1-firmwared`` owns that link, playback was
ported to daemon trajectories — and teach was not: ``gesture_record`` cannot
connect while the daemon runs (d1-sdk PR #103 makes it refuse outright), so
the panel still renders and every recording fails. This module is the same
recorder over the daemon's REST API, consumed only through the generated
client (:class:`~manipulation_kit.executors.firmware.FirmwareClient`) and the
kit's :class:`~manipulation_kit.executors.firmware.FirmwareExecutor` (lease,
position mode, trajectories, arrival barrier).

The capture, step by step (Shu's operator flow, 2026-09-23):

1. **Straight to HOME first** in position mode (not recorded), so the teach
   starts wrist-up rather than from wherever the arm sagged. The take ALWAYS
   starts at HOME: with ``--no-home-start`` the operator must have put the
   arms there, and the start is refused when any taught joint is more than
   ``HOME_TOL_DEG`` from HOME.
2. **Countdown** ``3, 2, 1`` (``countdown_s``), printed through ``on_state``.
3. **Go soft** — the guide mode, per taught arm:

   ``brake`` (default: hand guiding)
       mode ``idle`` then ``POST /v1/arm/{side}/brake_release`` (d1-firmware
       PR #92) with the confirm word ``RELEASE_BRAKE``: servos OFF, holding
       brakes forced open for a timed window the daemon closes itself;
       re-sent every third of the window while recording, so the daemon's
       own timer — not this process — is what closes them if this process
       dies. **The arm drops under gravity unless someone holds it**; the
       CLI prints the contract and takes ONE typed ``HOLDING`` per session,
       for all taught arms. Shu, 2026-09-23: moving the arm in compliance
       was hard; brakes-off is the teaching default.
   ``compliance`` (``--compliance``, gesture_record's own)
       ``POST /v1/arm/{side}/mode`` ``force_compliance`` with
       ``ForceComplianceConfig::xAxisCompliance(2.0)``: force direction
       ``[1,0,0,0,0,0]``, target force 0, force type 0, adjustment limit
       2 mm, ratios 0.05 (``setForceComplianceMode(c, 5, 5)``), measured pose
       anchored. Servos ON, the arm yields to the hand. Then 1.5 s to settle
       before sampling, so the switch transient is not recorded.
   ``idle`` (``--no-brake``)
       mode ``idle`` only, for an arm that moves by hand when idle.

4. **Sample** ``GET /v1/arm/{side}/state`` (both arms, degrees) at ``rate_hz``
   with timestamps, plus each gripper's closedness where the daemon publishes
   ``open_rad`` (kept in the recording; the CSV has no gripper column).
   ``t = 0`` is the instant the (last) release was acknowledged — the brakes
   are open from the first sample on, and nothing before it is motion. Stop
   on the caller's stop signal, after ``duration_s``, or after
   ``stationary_s`` of < 0.3 deg motion — or, in ``keyframe`` mode, capture
   one pose per operator request.
5. **Stop = brakes engaged FIRST**: the renewal is stopped under a lock (no
   release can land after the engage), ``brake_engage`` goes out on every
   released arm, and only then ``POST /v1/arm/{side}/recover`` at 0.05 —
   measured-pose position hold, the daemon's ``lockCurrentPositionMode(5,
   5)`` — and the take is finalised. On EVERY exit path.

The raw recording is kept raw (no wrist lock, no smoothing): the reduction is
:mod:`~manipulation_kit.teach.process`, run by ``mkit-teach keyframes`` /
``export``, so it can be re-run with other settings without re-teaching.
"""
from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence

import numpy as np

from ..arms import sides
from ..executor import JOINT_SLICE, WIRE_DIM

SCHEMA = "manipulation_kit.teach.recording/1"
GUIDES = ("brake", "compliance", "idle")
DEFAULT_GUIDE = "brake"
MODES = ("stream", "keyframe")

DEFAULT_RATE_HZ = 20.0
#: gesture_record: ``xAxisCompliance(adj_limit_mm = 2.0)``, ratios 5 %
COMPLIANCE = {"force_direction": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              "adjustment_limit_mm": 2.0, "target_force": 0.0,
              "force_type": 0, "anchor_command_pose": True,
              "vel_ratio": 0.05, "acc_ratio": 0.05}
#: gesture_record sleeps this long after entering compliance
SETTLE_S = 1.5
#: ``lockCurrentPositionMode(5, 5)``
HOLD_RATIO = 0.05
#: gesture_record's stationary test: max joint change per sample [deg]
STATIONARY_DEG = 0.3
#: brake window granted per release [s] (the daemon: 1..120, default 30)
BRAKE_WINDOW_S = 20.0
#: joint distance [deg] under which the arm counts as AT HOME (omakase-core
#: firmware_session.HOME_TOLERANCE_DEG); also the start-at-HOME gate
HOME_TOL_DEG = 2.0
#: seconds counted down (3, 2, 1) before the arms go soft
COUNTDOWN_S = 3

#: The lifecycle the old /d1_teach panel showed, kept as the CLI's vocabulary.
IDLE, STARTING, RECORDING, RECORDED, PREVIEWING, SAVED, DISCARDED = (
    "idle", "starting", "recording", "recorded", "previewing", "saved", "discarded")

BRAKE_CONTRACT = """\
HAND GUIDING WITH THE HOLDING BRAKES RELEASED (d1-firmware PR #92)
  * The servos are OFF and the brakes are forced OPEN: the arm DROPS under
    gravity the instant they open unless a person is holding it.
  * A person must hold every released arm ({arms}) BEFORE the release and
    until the recording ends. Keep hands and faces clear of the fall path.
  * After a 3-2-1 countdown the brakes open and recording starts at once.
    Stop (Enter) engages the brakes FIRST, then the arm is held in position.
  * The release is timed: the daemon engages the brakes itself when its
    window ({window:g} s, re-sent while recording) runs out.
  * Brakes re-engage on: end of recording (this tool), window elapsed,
    POST /v1/arm/<side>/brake_engage, estop, soft kill, daemon shutdown.
  * While released the daemon refuses mode/recover/trajectory on that arm.
  * The kit's brake path is unverified on hardware as of 2026-09-23."""


@dataclass
class Recording:
    """A raw teach capture. ``samples[k]`` is 14 degrees, CSV column order
    (A = physical LEFT first), taken at ``times[k]`` seconds."""

    mode: str
    guide: str
    arms: List[str]
    rate_hz: float
    home: List[float]
    times: List[float] = field(default_factory=list)
    samples: List[List[float]] = field(default_factory=list)
    #: per sample, side -> closedness in [0, 1] or None (daemon published no
    #: ``open_rad``, or the gripper could not be read)
    grippers: List[Dict[str, Optional[float]]] = field(default_factory=list)
    meta: Dict[str, Any] = field(default_factory=dict)

    def to_json(self) -> Dict[str, Any]:
        return {"schema": SCHEMA, **asdict(self)}

    @classmethod
    def from_json(cls, doc: Dict[str, Any]) -> "Recording":
        if doc.get("schema") != SCHEMA:
            raise ValueError(f"not a {SCHEMA} document: {doc.get('schema')!r}")
        fields = {k: doc[k] for k in ("mode", "guide", "arms", "rate_hz", "home",
                                      "times", "samples", "grippers", "meta")
                  if k in doc}
        return cls(**fields)

    def save(self, path: Path) -> None:
        Path(path).write_text(json.dumps(self.to_json(), indent=1), encoding="utf-8")

    @classmethod
    def load(cls, path: Path) -> "Recording":
        return cls.from_json(json.loads(Path(path).read_text(encoding="utf-8")))


def _wire(side: str) -> str:
    return sides.SDK_SIDE[sides.check(side)].lower()


def pose_deg(state) -> List[float]:
    """The 14 measured degrees of a :class:`RawState`, CSV order."""
    joints = state.joints
    return [float(v) for side in sides.SIDES
            for v in np.degrees(np.asarray(joints[side], dtype=float))]


def _q16(home_deg: Sequence[float]) -> np.ndarray:
    q = np.zeros(WIRE_DIM, dtype=float)
    q[JOINT_SLICE["left"]] = np.radians(home_deg[:7])
    q[JOINT_SLICE["right"]] = np.radians(home_deg[7:])
    return q


def move_to(robot, target_deg: Sequence[float], *, tol_deg: float = HOME_TOL_DEG):
    """Bring both arms to ``target_deg`` in position mode (a two-knot
    trajectory timed at the executor's own schedule rate), then the arrival
    barrier. Returns the executor's ``ArrivalReport``, or ``None`` when the
    arms were already within ``tol_deg``."""
    from ..executors.firmware.executor import (INTERPOLATION_S,  # noqa: PLC0415
                                               schedule_rate_deg_s)
    current = pose_deg(robot.state())
    span = max(abs(a - b) for a, b in zip(current, target_deg))
    if span <= tol_deg:
        return None
    duration = max(span / schedule_rate_deg_s(robot.vel_ratio), INTERPOLATION_S)
    robot.play_waypoints([
        {"t": 0.0, "a": current[:7], "b": current[7:]},
        {"t": round(duration, 6), "a": list(target_deg[:7]), "b": list(target_deg[7:])}])
    return robot.wait_arrived(_q16(target_deg))


class _BrakeKeeper(threading.Thread):
    """Re-sends the timed release every third of the window while recording,
    so the daemon's own timer — not this process — is what closes the brakes
    if this process dies."""

    def __init__(self, client, wires, window_s, holder):
        super().__init__(name="mkit-teach-brake", daemon=True)
        self.client, self.wires, self.window_s, self.holder = client, wires, window_s, holder
        self.stop_event = threading.Event()
        #: held for every renewal; :meth:`halt` takes it, so once halt()
        #: returns no release is in flight and none can follow
        self.lock = threading.Lock()
        self.error: Optional[BaseException] = None

    def run(self) -> None:
        while not self.stop_event.wait(self.window_s / 3.0):
            with self.lock:
                if self.stop_event.is_set():
                    return
                try:
                    for wire in self.wires:
                        self.client.brake_release(wire, seconds=self.window_s,
                                                  holder=self.holder)
                except BaseException as exc:  # noqa: BLE001 - reported by record()
                    self.error = exc
                    return

    def halt(self) -> None:
        """Stop renewing, and wait out a renewal already on the wire."""
        self.stop_event.set()
        with self.lock:
            pass


def record(robot, *, home: Sequence[float], guide: str = DEFAULT_GUIDE,
           arms: Sequence[str] = ("left", "right"), mode: str = "stream",
           rate_hz: float = DEFAULT_RATE_HZ, duration_s: Optional[float] = None,
           stationary_s: float = 0.0, stop: Optional[Callable[[], bool]] = None,
           next_keyframe: Optional[Callable[[], bool]] = None,
           home_start: bool = True, brake_window_s: float = BRAKE_WINDOW_S,
           adj_limit_mm: float = COMPLIANCE["adjustment_limit_mm"],
           allow_bare_flange: bool = False, countdown_s: int = COUNTDOWN_S,
           home_tol_deg: float = HOME_TOL_DEG,
           on_state: Callable[[str, str], None] = lambda state, msg: None,
           sleep=time.sleep, clock=time.monotonic) -> Recording:
    """Capture a teach on an ENTERED :class:`FirmwareExecutor` (lease held).

    ``stop()`` is polled every sample (stream mode); ``next_keyframe()``
    blocks until the operator asks for a pose and returns ``False`` when they
    are done (keyframe mode). ``on_state(state, message)`` reports the
    lifecycle.
    """
    if guide not in GUIDES:
        raise ValueError(f"guide must be one of {GUIDES}, got {guide!r}")
    if mode not in MODES:
        raise ValueError(f"mode must be one of {MODES}, got {mode!r}")
    if mode == "keyframe" and next_keyframe is None:
        raise ValueError("keyframe mode needs next_keyframe()")
    if not (math.isfinite(rate_hz) and rate_hz > 0):
        raise ValueError("rate_hz must be positive")
    arms = [sides.check(a) for a in arms]
    wires = [_wire(a) for a in arms]
    client, holder = robot.client, robot.holder
    rec = Recording(mode=mode, guide=guide, arms=list(arms), rate_hz=float(rate_hz),
                    home=[float(v) for v in home],
                    meta={"holder": holder,
                          "firmware_spec": getattr(robot, "firmware_spec", None),
                          "started_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                          "adj_limit_mm": adj_limit_mm if guide == "compliance" else None,
                          "brake_window_s": brake_window_s if guide == "brake" else None})
    on_state(STARTING, "connecting")
    if home_start:
        on_state(STARTING, "moving to straight HOME (not recorded)")
        arrival = move_to(robot, rec.home)
        if arrival is not None and not arrival.arrived:
            raise RuntimeError(f"did not reach HOME before the teach: {arrival.detail}")
    # The take starts at HOME, always (the exported motion starts there).
    start = pose_deg(robot.state())
    off = [(i, start[i] - rec.home[i]) for side in arms
           for i in (range(0, 7) if side == "left" else range(7, 14))
           if abs(start[i] - rec.home[i]) > home_tol_deg]
    if off:
        worst = max(off, key=lambda item: abs(item[1]))
        raise RuntimeError(
            f"the taught arm(s) are not at HOME at Start ({len(off)} joint(s) "
            f"beyond {home_tol_deg:g} deg; worst index {worst[0]} off by "
            f"{worst[1]:+.1f} deg). The take must start at HOME: drop "
            f"--no-home-start, or bring the arms to HOME first.")
    rec.meta["start_pose"] = [round(v, 4) for v in start]
    if guide == "compliance":
        # gesture_record registered the tool before compliance: an
        # unregistered payload is gravity-compensated as an empty flange and
        # the wrist sags into the recording. The daemon registers its
        # configured end effector itself; report what it holds, and refuse
        # to go soft on a bare flange unless told to.
        for wire in wires:
            tool = _tool_source(client, wire)
            rec.meta.setdefault("tool", {})[wire] = tool
            if tool == "none" and not allow_bare_flange:
                raise RuntimeError(
                    f"arm {wire}: the daemon has NO tool registered (GET "
                    f"/v1/arm/{wire}/tool source=none), so compliance would "
                    f"gravity-compensate an empty flange and the wrist sags. "
                    f"Configure [arm] end_effector on the daemon, or pass "
                    f"allow_bare_flange / --allow-bare-flange if nothing is mounted.")
    for remaining in range(int(countdown_s), 0, -1):
        on_state(STARTING, f"{remaining}")
        sleep(1.0)
    entered: List[str] = []
    released: List[str] = []
    keeper: Optional[_BrakeKeeper] = None
    problems: List[str] = []
    try:
        for wire in wires:
            if guide == "compliance":
                params = dict(COMPLIANCE, adjustment_limit_mm=float(adj_limit_mm))
                client.arm_mode(wire, "force_compliance", holder=holder, **params)
            else:
                client.arm_mode(wire, "idle", holder=holder)
            entered.append(wire)
            if guide == "brake":
                released.append(wire)      # engaged on exit even if this fails
                client.brake_release(wire, seconds=brake_window_s, holder=holder)
        if guide == "brake":
            keeper = _BrakeKeeper(client, wires, brake_window_s, holder)
            keeper.start()
        if guide == "compliance":
            on_state(STARTING, "stabilizing (not recorded)")
            sleep(SETTLE_S)
        on_state(RECORDING, f"{guide}: move the {'/'.join(arms)} arm(s) by hand")
        # t = 0 is NOW: the release was just acknowledged (brake guide).
        _capture(robot, rec, mode=mode, duration_s=duration_s,
                 stationary_s=stationary_s, stop=stop, next_keyframe=next_keyframe,
                 keeper=keeper, sleep=sleep, clock=clock)
    finally:
        # Brakes FIRST: nothing else happens before they are engaged.
        if keeper is not None:
            keeper.halt()
        for wire in released:
            try:
                client.brake_engage(wire)
            except Exception as exc:  # noqa: BLE001 - keep going: recover
                problems.append(f"brake_engage {wire}: {exc}")
        if keeper is not None:
            keeper.join(timeout=2.0)
        for wire in entered:
            try:
                client.arm_recover(wire, vel_ratio=HOLD_RATIO, acc_ratio=HOLD_RATIO)
            except Exception as exc:  # noqa: BLE001
                problems.append(f"recover {wire}: {exc}")
        rec.meta["exit_problems"] = problems
    if keeper is not None and keeper.error is not None:
        problems.append(f"brake window renewal failed: {keeper.error}")
    if len(rec.samples) < 2:
        raise RuntimeError(f"not enough samples to build a gesture "
                           f"({len(rec.samples)}); {'; '.join(problems)}")
    on_state(RECORDED, f"{len(rec.samples)} samples, "
                       f"{rec.times[-1] - rec.times[0]:.1f} s"
             + (f"; exit problems: {'; '.join(problems)}" if problems else ""))
    return rec


def _tool_source(client, wire: str) -> str:
    """The daemon's ``ArmToolStatus.source`` word, ``"unknown"`` when this
    client cannot read it (an older document, or a fake)."""
    read = getattr(client, "arm_tool_state", None)
    if read is None:
        return "unknown"
    try:
        status = read(wire)
    except Exception:  # noqa: BLE001 - unknown, not none
        return "unknown"
    source = getattr(status, "source", None)
    return str(getattr(source, "value", source) or "unknown")


def _grip(state) -> Dict[str, Optional[float]]:
    out: Dict[str, Optional[float]] = {}
    for side in sides.SIDES:
        hand = state.hands.get(side) if getattr(state, "hands", None) else None
        out[side] = None if hand is None or hand.closedness is None else float(hand.closedness)
    return out


def _take(robot, rec: Recording, t: float):
    state = robot.state()
    pose = pose_deg(state)
    rec.times.append(round(float(t), 6))
    rec.samples.append([round(v, 4) for v in pose])
    rec.grippers.append(_grip(state))
    return pose


def _capture(robot, rec: Recording, *, mode, duration_s, stationary_s, stop,
             next_keyframe, keeper, sleep, clock) -> None:
    t0 = clock()
    if mode == "keyframe":
        while True:
            if keeper is not None and keeper.error is not None:
                raise RuntimeError(f"brake window renewal failed: {keeper.error}")
            if not next_keyframe():
                return
            _take(robot, rec, clock() - t0)
    period = 1.0 / rec.rate_hz
    still = 0.0
    prev: Optional[List[float]] = None
    while True:
        tick = clock()
        elapsed = tick - t0
        if duration_s is not None and duration_s > 0 and elapsed >= duration_s:
            return
        if stop is not None and stop():
            return
        if keeper is not None and keeper.error is not None:
            raise RuntimeError(f"brake window renewal failed: {keeper.error}")
        pose = _take(robot, rec, elapsed)
        if stationary_s > 0 and prev is not None:
            if max(abs(a - b) for a, b in zip(pose, prev)) < STATIONARY_DEG:
                still += period
                if still >= stationary_s:
                    return
            else:
                still = 0.0
        prev = pose
        wait = (tick + period) - clock()
        if wait > 0:
            sleep(wait)
