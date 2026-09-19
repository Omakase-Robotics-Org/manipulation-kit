"""Run a :class:`~manipulation_kit.primitives.Plan` on a real D1, through d1-firmwared.

THE TRANSPORT CHOICE, stated once and argued
--------------------------------------------
Two ways exist to put a joint path on the arms, and the daemon offers both:

``POST /v1/arm/move_joints_both``, streamed at 50 Hz
    What VR teleop does, because teleop has no path — it has whatever the
    operator's hand did in the last 20 ms. Each POST is one guarded dual-arm
    pose. The cost is that the timing lives on THIS side of the socket: a GC
    pause, a scheduler hiccup or a slow reply lands as a gap in the command
    stream, the controller's ``frame_miss_count`` climbs, and a post-stall
    flush arrives as one large step (which is why teleop carries an anti-lunge
    clamp at all).

``POST /v1/arm/trajectory/start``, one upload
    What a PLAN is. The daemon takes up to 10 000 absolute-time waypoints,
    plays them on its own 1 ms tokio ticker, re-checks EVERY sampled pose
    against its own Rust motion guard, enforces a 350 deg/s slew limit at
    runtime, refuses to start unless the first pose is within 3 degrees of
    measured feedback and the arm is in a clean position or torque hold, and
    cancels cleanly — including automatically, when the arm lease is preempted
    out from under us.

**This executor's default is ``trajectory``**, because the thing that makes
streaming necessary — a target that is not known until the moment it is sent —
is exactly what a pre-checked plan does not have. Every joint step in a
:class:`Plan` was already solved, step-clamped and guard-checked before the
first byte moved; handing all of them to the daemon at once moves the timing to
the component with a real-time loop and adds a second, independent guard pass
over the whole path. Streaming stays available (``transport="stream"``) for the
case that genuinely is a stream — a corrective ``Nudge`` inside a visual servo
loop, where the next target depends on the last frame — and it is exercised by
the tests, so the fallback is not decorative.

Everything else here is the part consumers kept re-deriving:

* the **arm lease** at class ``policy`` (teleop and the console claim
  ``operator`` and outrank us — a person at the robot always wins), heartbeat
  while we hold it, released on exit, and ``epoch`` watched so a preemption we
  survived is not mistaken for uninterrupted possession;
* **position mode with the run's own velocity/acceleration ratios**, re-sent
  even when the arm is already holding position, because ``recover`` leaves its
  own ratios behind and a policy served at those drove the right arm to ~90 Nm
  on d1-2 (2026-09-10);
* the **140 deg/s joint rate clamp** dx-vr-teleop runs (7 deg per tick at its
  20 Hz keepalive), applied here per command interval;
* **grip presets, not numbers**: the preset owns the stop torque (d1-firmware
  PR #55), so ``soft``/``firm``/``strong`` is the whole vocabulary.

Install: ``pip install -e '.[firmware]'``. The extra names ``d1fw-client``;
where it comes from is the consumer's pin, e.g.::

    d1fw-client @ git+ssh://git@github.com/Omakase-Robotics-Org/d1-firmware-client-py.git@bfd6a678
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from ..arms import sides
from ..executor import (JOINT_SLICE, SIDES, RawState, RunReport,
                        SettleReport, WIRE_DIM)
from ..primitives.types import GripStep, JointStep, Plan, SettleStep

# --------------------------------------------------------------------------- #
# constants, each with the measurement behind it
# --------------------------------------------------------------------------- #

#: Command rate. ``dx-vr-teleop``'s ``HOME_RATE_HZ = 50.0`` ("ramp step rate —
#: VR tick parity"): the rate the arms are actually driven at on this robot.
STREAM_HZ = 50.0

#: Per-joint rate ceiling [deg/s]. ``dx-vr-teleop``'s anti-lunge clamp is
#: ``JOINT_DELTA_CLAMP_DEG = 7.0`` per tick at its 20 Hz keepalive, i.e.
#: 140 deg/s — "~2-3x the fastest plausible human teleop joint speed", so
#: continuous motion passes through untouched while a single-tick warp is
#: spread over several commands. Expressed as a RATE here because this executor
#: runs at a different tick than the keepalive it was measured at, and a
#: per-tick copy of the number would mean something different at 50 Hz.
MAX_JOINT_RATE_DEG_S = 140.0

#: Interpolation window at the start of a trajectory [s]. The daemon refuses a
#: trajectory whose first waypoint is more than 3 degrees from measured
#: feedback, so every upload begins AT the measured pose; this is how long it
#: is given to reach the plan's first knot. Three 50 Hz ticks — long enough
#: that the controller interpolates into the motion instead of stepping into
#: it, short enough not to feel like a pause.
INTERPOLATION_S = 0.060

#: Velocity / acceleration ratios for position mode, as FRACTIONS in [0, 1].
#: The daemon clamps anything above 1, so passing the vendor SDK's percent
#: integers unconverted asks for 800% and gets 100% — every firmware run on
#: d1-2 up to 2026-09-10 drove the arms flat out and faulted the right arm at
#: the grasp. 0.15 is d1-inference's default.
DEFAULT_VEL_RATIO = 0.15
DEFAULT_ACC_RATIO = 0.15

#: Lease class. ``operator`` is a person at the robot (teleop, the console) and
#: outranks us; ``ambient`` is the un-classed default nobody can be displaced
#: by. A planned primitive is policy-grade motion, so: ``policy``.
LEASE_CLASS = "policy"
DEFAULT_HOLDER = "manipulation-kit"
DEFAULT_TTL_S = 30

#: Largest gap [deg] between the commanded and the measured pose at which
#: position control may be engaged from idle. Beyond it the controller would
#: snap to a stale command — on d1-2 (2026-09-10) one joint's command sat 48
#: degrees from its measurement.
ANCHOR_GAP_DEG = 3.0


def _wire_side(side: str) -> str:
    """Logical side -> the daemon's path segment. The crossover lives in
    :mod:`manipulation_kit.arms.sides`; this is the only place it is spelt."""
    return sides.SDK_SIDE[sides.check(side)].lower()


class FirmwareUnavailable(RuntimeError):
    """``d1fw-client`` is not installed, or the daemon refused a setup step."""


def _client_class():
    try:
        from d1fw_client import FirmwareClient  # noqa: PLC0415
    except ImportError as exc:  # pragma: no cover - depends on the venv
        raise FirmwareUnavailable(
            "manipulation_kit.executors.firmware needs the d1fw-client package: "
            "pip install -e '.[firmware]' (and pin the client by commit, see "
            "the module docstring)") from exc
    return FirmwareClient


@dataclass(frozen=True)
class Lease:
    """The daemon's published view of who holds the arms."""

    holder: str
    lease_class: str
    epoch: int
    ttl_s: int
    expires_in_s: int
    preempted_from: Optional[str] = None

    @classmethod
    def parse(cls, data: Any) -> "Lease":
        return cls(holder=str(data["holder"]),
                   lease_class=str(data.get("class", "ambient")),
                   epoch=int(data.get("epoch", 0)),
                   ttl_s=int(data.get("ttl_s", 0)),
                   expires_in_s=int(data.get("expires_in_s", 0)),
                   preempted_from=data.get("preempted_from"))


class FirmwareExecutor:
    """An :class:`~manipulation_kit.executor.Executor` over ``d1fw-client``.

    Use it as a context manager: the lease is taken on entry and handed back on
    exit, including on an exception, because a crashed consumer that keeps the
    arms until its TTL expires is a robot nobody can drive.

        with FirmwareExecutor(base_url="http://d1-2:4750") as robot:
            run(plan, robot)

    ``client`` may be injected — that is how the tests drive a fake with no
    socket, and how a consumer that already has a connection reuses it.
    """

    def __init__(self, client: Any = None, *,
                 base_url: str = "http://127.0.0.1:4750",
                 holder: str = DEFAULT_HOLDER,
                 lease_class: str = LEASE_CLASS,
                 ttl_s: int = DEFAULT_TTL_S,
                 transport: str = "trajectory",
                 hz: float = STREAM_HZ,
                 vel_ratio: float = DEFAULT_VEL_RATIO,
                 acc_ratio: float = DEFAULT_ACC_RATIO,
                 max_joint_rate_deg_s: float = MAX_JOINT_RATE_DEG_S,
                 grip: str = "soft",
                 sleep=time.sleep, clock=time.monotonic):
        if transport not in ("trajectory", "stream"):
            raise ValueError("transport must be 'trajectory' or 'stream', "
                             f"got {transport!r}")
        for name, ratio in (("vel_ratio", vel_ratio), ("acc_ratio", acc_ratio)):
            if not 0.0 < float(ratio) <= 1.0:
                raise ValueError(f"{name} is a FRACTION in (0, 1]; the daemon "
                                 f"clamps anything above 1, so {ratio!r} asks "
                                 f"for full speed. Divide the percent by 100.")
        self.client = client if client is not None else _client_class()(base_url)
        self.holder = holder
        self.lease_class = lease_class
        self.ttl_s = int(ttl_s)
        self.transport = transport
        self.hz = float(hz)
        self.period = 1.0 / float(hz)
        self.vel_ratio = float(vel_ratio)
        self.acc_ratio = float(acc_ratio)
        self.max_rate = float(max_joint_rate_deg_s)
        self.grip = grip
        self._sleep = sleep
        self._clock = clock
        self.lease: Optional[Lease] = None
        self._last_sent_deg: Dict[str, np.ndarray] = {}
        self._t0: Optional[float] = None

    # -- lease ------------------------------------------------------------- #
    def acquire(self) -> Lease:
        """Take (or heartbeat) the arm lease. Raises if somebody senior holds it."""
        data = self.client.request("POST", "/v1/arm/lease", {
            "holder": self.holder, "class": self.lease_class, "ttl_s": self.ttl_s})
        lease = Lease.parse(data)
        if self.lease is not None and lease.epoch != self.lease.epoch:
            # A fresh grant means we LOST the arms in between: mode, tool
            # registration and the last-sent pose are all stale now.
            self._last_sent_deg.clear()
        self.lease = lease
        return lease

    def release(self) -> None:
        """Hand the arms back. Never raises — a failed release must not mask
        the exception that is on its way out of the ``with`` block."""
        if self.lease is None:
            return
        try:
            self.client.request("DELETE", "/v1/arm/lease", {"holder": self.holder})
        except Exception:  # noqa: BLE001 - see the docstring
            pass
        finally:
            self.lease = None

    def __enter__(self) -> "FirmwareExecutor":
        self.acquire()
        self.position_mode()
        return self

    def __exit__(self, *_exc) -> None:
        self.release()

    def _holder_body(self, body: Dict[str, Any]) -> Dict[str, Any]:
        return dict(body, holder=self.holder) if self.lease is not None else dict(body)

    # -- setup ------------------------------------------------------------- #
    def position_mode(self) -> None:
        """Put both arms in position control at THIS run's ratios.

        Sent even when the arm already reports ``position``: the ratios are
        part of the request, the daemon's own ``recover`` leaves different ones
        behind, and a policy served at those is the d1-2 90 Nm incident.
        """
        for side in SIDES:
            wire = _wire_side(side)
            state = self.client.arm_state(wire)
            if state.mode != "position":
                gap = max(abs(c - f) for c, f in
                          zip(state.command_joints, state.feedback_joints))
                if state.error_code or gap > ANCHOR_GAP_DEG:
                    raise FirmwareUnavailable(
                        f"arm {wire} is in mode {state.mode!r} with error "
                        f"{state.error_code} and its commanded pose is "
                        f"{gap:.1f} deg from the measured one; engaging "
                        f"position control now would snap the arm to a stale "
                        f"command. Recover it explicitly first.")
            self.client.request("POST", f"/v1/arm/{wire}/mode", self._holder_body({
                "mode": "position", "vel_ratio": self.vel_ratio,
                "acc_ratio": self.acc_ratio}))

    # -- Executor protocol ------------------------------------------------- #
    def state(self) -> RawState:
        joints: Dict[str, np.ndarray] = {}
        grippers: Dict[str, float] = {}
        holding: Dict[str, bool] = {}
        stationary = True
        for side in SIDES:
            wire = _wire_side(side)
            arm = self.client.arm_state(wire)
            joints[side] = np.radians(np.asarray(arm.feedback_joints, dtype=float))
            stationary = stationary and bool(arm.stationary)
            try:
                report = self.client.gripper_state(wire)
            except Exception:  # noqa: BLE001 - a gripper that cannot be read is
                continue       # UNKNOWN, not "open"; leave it out of the dict
            holding[side] = bool(report.holding)
            if report.open_rad:
                grippers[side] = max(0.0, min(1.0, 1.0 - report.jaw_rad / report.open_rad))
        return RawState(joints=joints, grippers=grippers, holding=holding,
                        stationary=stationary, stamp=self._clock())

    def send_joints(self, q16, *, t: float) -> None:
        """One guarded dual-arm POST, paced to ``t`` and rate-clamped.

        ``t`` is PLAN time (seconds from the first step), so a caller that
        stalls between steps does not accumulate lag: the sleep is computed
        against the wall clock at the first send, and a late command is sent
        immediately rather than after a sleep that has already elapsed.
        """
        q = np.asarray(q16, dtype=float).reshape(WIRE_DIM)
        now = self._clock()
        if self._t0 is None:
            self._t0 = now
        wait = (self._t0 + float(t)) - now
        if wait > 0:
            self._sleep(wait)
        targets = {side: np.degrees(q[JOINT_SLICE[side]]) for side in SIDES}
        targets = {side: self._clamp_rate(side, deg) for side, deg in targets.items()}
        self.client.request("POST", "/v1/arm/move_joints_both", self._holder_body({
            "a": [float(v) for v in targets["left"]],
            "b": [float(v) for v in targets["right"]],
            "wait": False}))
        self._last_sent_deg = targets

    def _clamp_rate(self, side: str, target_deg: np.ndarray) -> np.ndarray:
        """Bound the per-command joint change to :data:`MAX_JOINT_RATE_DEG_S`.

        Relative to the LAST SENT value, not to feedback: this exists to stop a
        buffered flush arriving as one enormous step, and feedback lags the
        command by design.
        """
        last = self._last_sent_deg.get(side)
        if last is None:
            return np.asarray(target_deg, dtype=float)
        limit = self.max_rate * self.period
        return last + np.clip(np.asarray(target_deg, dtype=float) - last,
                              -limit, limit)

    def set_gripper(self, side: str, closedness: float, *, grip: str = "") -> None:
        self.client.gripper_set(_wire_side(side), float(closedness),
                                grip=(grip or self.grip) or None)

    def settle(self, timeout_s: float) -> SettleReport:
        """Poll until BOTH arms report stationary, or give up and say so.

        ``ArmState.stationary`` is the daemon's own judgement, from the same
        feedback frame it reports the velocities in. A verifier run while the
        arm is still moving measures the middle of the motion.
        """
        started = self._clock()
        worst = float("nan")
        while True:
            states = [self.client.arm_state(_wire_side(s)) for s in SIDES]
            worst = max(max(abs(v) for v in st.feedback_velocity) for st in states)
            if all(st.stationary for st in states):
                return SettleReport(True, self._clock() - started, worst)
            waited = self._clock() - started
            if waited >= float(timeout_s):
                return SettleReport(
                    False, waited, worst,
                    f"still moving after {waited:.1f}s, worst joint "
                    f"{worst:.1f} deg/s")
            self._sleep(min(self.period, max(0.0, float(timeout_s) - waited)))

    # -- the preferred path: one upload ------------------------------------ #
    def run_plan(self, plan: Plan) -> RunReport:
        """Play a pre-checked plan, trajectory-first. Used by
        :func:`manipulation_kit.executor.run` when this executor is passed."""
        if not getattr(plan, "ok", False):
            return RunReport(getattr(plan, "primitive", "?"),
                             getattr(plan, "side", ""), False, 0, error=str(plan))
        if self.transport == "stream":
            # run_steps, NOT run: run() delegates to run_plan, which is this.
            from ..executor import run_steps  # noqa: PLC0415
            return run_steps(plan, self, hz=self.hz)
        sent = 0
        settle: Optional[SettleReport] = None
        batch: List[JointStep] = []
        for step in plan.steps:
            if isinstance(step, JointStep):
                batch.append(step)
                continue
            sent += self._play(batch)
            batch = []
            if isinstance(step, GripStep):
                self.set_gripper(step.side, step.closedness, grip=step.grip)
                sent += 1
            elif isinstance(step, SettleStep):
                settle = self.settle(step.timeout_s)
                sent += 1
        sent += self._play(batch)
        return RunReport(plan.primitive, plan.side, True, sent, settle)

    def _play(self, steps: Sequence[JointStep]) -> int:
        """Upload one contiguous run of joint steps as a trajectory job."""
        if not steps:
            return 0
        waypoints = self._waypoints(steps)
        status = self.client.request("POST", "/v1/arm/trajectory/start",
                                     self._holder_body({"waypoints": waypoints}))
        job = int(status["id"])
        deadline = self._clock() + waypoints[-1]["t"] + 5.0
        while True:
            status = self.client.request(
                "GET", f"/v1/arm/trajectory/{job}/status")
            phase = str(status["phase"])
            if phase == "completed":
                return len(waypoints)
            if phase in ("failed", "cancelled"):
                raise FirmwareUnavailable(
                    f"trajectory {job} {phase}: {status.get('message')}")
            if self._clock() > deadline:
                self.client.request("POST", f"/v1/arm/trajectory/{job}/cancel", {})
                raise FirmwareUnavailable(
                    f"trajectory {job} was still running "
                    f"{waypoints[-1]['t'] + 5.0:.1f}s in; cancelled")
            self._sleep(self.period)

    def _waypoints(self, steps: Sequence[JointStep]) -> List[Dict[str, Any]]:
        """Absolute-time dual-arm waypoints, starting AT the measured pose.

        Two things happen here and both are required by the daemon. The first
        waypoint is the arms where they actually are, because a trajectory
        whose first pose is more than 3 degrees from feedback is refused. And
        every segment is stretched until no joint exceeds
        :data:`MAX_JOINT_RATE_DEG_S`, so the daemon's own 350 deg/s runtime
        slew gate has margin rather than being the thing that catches us.
        """
        measured = self.state().joints
        current = {side: np.degrees(measured[side]) for side in SIDES}
        points = [{"t": 0.0,
                   "a": [float(v) for v in current["left"]],
                   "b": [float(v) for v in current["right"]]}]
        for step in steps:
            current = dict(current)
            current[step.side] = np.degrees(np.asarray(step.q, dtype=float))
            previous = points[-1]
            span = max(max(abs(a - b) for a, b in zip(current["left"], previous["a"])),
                       max(abs(a - b) for a, b in zip(current["right"], previous["b"])))
            dt = max(self.period, span / self.max_rate)
            if previous["t"] == 0.0:
                dt = max(dt, INTERPOLATION_S)
            # Round the INTERVAL up, never the timestamp: rounding a timestamp
            # down shortens the segment and puts the commanded rate back over
            # the ceiling the interval was computed to respect.
            dt = math.ceil(dt * 1e6) / 1e6
            points.append({"t": round(previous["t"] + dt, 6),
                           "a": [float(v) for v in current["left"]],
                           "b": [float(v) for v in current["right"]]})
        return points
