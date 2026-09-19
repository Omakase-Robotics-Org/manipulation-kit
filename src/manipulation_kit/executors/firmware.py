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
import os
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from ..arms import safety, sides
from ..executor import (ARRIVE_TIMEOUT_S, ARRIVE_TOL_RAD, BARRIER_FAILED,
                        JOINT_SLICE, SIDES, STROKE_TIMEOUT_S,
                        TRANSPORT_ERROR, ArrivalReport, RawState, RunReport,
                        SettleReport, StrokeReport, WIRE_DIM)
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
DEFAULT_TTL_S = 30
#: Fraction of the TTL after which the lease is renewed. A third: two renewals
#: are missed before the daemon can take the arms back.
HEARTBEAT_FRACTION = 1.0 / 3.0


def default_holder() -> str:
    """A holder string unique to THIS session.

    Every process defaulting to the literal ``"manipulation-kit"`` means the
    daemon cannot tell two of them apart: a second one renews the first one's
    lease, and a ``DELETE`` from either hands back the arms the other is
    driving. The name still starts with the package so a person reading
    ``GET /v1/arm/lease`` knows what has the robot.
    """
    return f"manipulation-kit/{os.getpid()}-{uuid.uuid4().hex[:8]}"


#: Kept for consumers that pinned the old literal. It is NOT the default.
DEFAULT_HOLDER = "manipulation-kit"

#: The largest single-command joint change this transport will put on the
#: wire [deg]. One POST is one absolute target the controller interpolates
#: to, so a knot bigger than the planner's own per-tick cap is a lunge —
#: and the answer is to REFUSE it, not to clip it into something the geometry
#: no longer describes (R4).
MAX_COMMAND_STEP_DEG = math.degrees(safety.MAX_JOINT_STEP_RAD)

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


class LeasePreempted(FirmwareUnavailable):
    """Somebody senior took the arms while we were using them.

    A new lease EPOCH means we lost possession and got it back. Everything we
    believed in between — the mode, the ratios, the last commanded pose, the
    trajectory that was playing — is stale, and continuing is continuing from
    a posture nobody checked. So it raises, and the caller re-observes and
    replans. The old code cleared a cache and carried on.
    """


class RateRefused(FirmwareUnavailable):
    """A knot is too large for one command, and clipping it is not the fix.

    The stream used to clip each component against the last transmitted
    vector, which does three separate wrong things: it changes the checked
    geometry, it can change the PATH (componentwise clipping is not a
    rescaling), and it silently stops short of the endpoint — a real
    ``Grasp(red_block, left)`` ended 6.5283 degrees from its final target
    while ``RunReport.completed`` was ``True``, and the close stroke followed
    the truncated travel (R4).
    """


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
                 holder: Optional[str] = None,
                 lease_class: str = LEASE_CLASS,
                 ttl_s: int = DEFAULT_TTL_S,
                 transport: str = "trajectory",
                 hz: float = STREAM_HZ,
                 vel_ratio: float = DEFAULT_VEL_RATIO,
                 acc_ratio: float = DEFAULT_ACC_RATIO,
                 max_joint_rate_deg_s: float = MAX_JOINT_RATE_DEG_S,
                 grip: str = "soft",
                 heartbeat: bool = True,
                 arrive_tol_rad: float = ARRIVE_TOL_RAD,
                 arrive_timeout_s: float = ARRIVE_TIMEOUT_S,
                 stroke_timeout_s: float = STROKE_TIMEOUT_S,
                 sleep=time.sleep, clock=time.monotonic):
        if transport not in ("trajectory", "stream"):
            raise ValueError("transport must be 'trajectory' or 'stream', "
                             f"got {transport!r}")
        for name, ratio in (("vel_ratio", vel_ratio), ("acc_ratio", acc_ratio)):
            if not 0.0 < float(ratio) <= 1.0:
                raise ValueError(f"{name} is a FRACTION in (0, 1]; the daemon "
                                 f"clamps anything above 1, so {ratio!r} asks "
                                 f"for full speed. Divide the percent by 100.")
        # Positive and FINITE, all of them. A NaN hz makes every period a NaN,
        # every sleep a no-op and every deadline unreachable, and nothing
        # downstream would have said so (R14).
        for name, value in (("hz", hz), ("max_joint_rate_deg_s",
                                         max_joint_rate_deg_s),
                            ("arrive_timeout_s", arrive_timeout_s),
                            ("stroke_timeout_s", stroke_timeout_s),
                            ("arrive_tol_rad", arrive_tol_rad)):
            if not (math.isfinite(float(value)) and float(value) > 0.0):
                raise ValueError(f"{name} must be a positive finite number, "
                                 f"got {value!r}")
        if not (math.isfinite(float(ttl_s)) and int(ttl_s) >= 1):
            raise ValueError(f"ttl_s must be at least 1 second, got {ttl_s!r}")
        self.client = client if client is not None else _client_class()(base_url)
        self.holder = holder if holder is not None else default_holder()
        self.lease_class = lease_class
        self.ttl_s = int(ttl_s)
        self.transport = transport
        self.hz = float(hz)
        self.period = 1.0 / float(hz)
        self.vel_ratio = float(vel_ratio)
        self.acc_ratio = float(acc_ratio)
        self.max_rate = float(max_joint_rate_deg_s)
        self.grip = grip
        self.arrive_tol_rad = float(arrive_tol_rad)
        self.arrive_timeout_s = float(arrive_timeout_s)
        self.stroke_timeout_s = float(stroke_timeout_s)
        self._sleep = sleep
        self._clock = clock
        self.lease: Optional[Lease] = None
        self._last_sent_deg: Dict[str, np.ndarray] = {}
        self._t0: Optional[float] = None
        self._last_lease_at: float = float("-inf")
        self._epoch: Optional[int] = None
        self._want_heartbeat = bool(heartbeat)
        self._beat: Optional[threading.Thread] = None
        self._stop_beat = threading.Event()
        #: trajectory jobs we started and have not seen finish. Cancelled on
        #: the way out of any failure, so an interrupted consumer does not
        #: leave the arms playing a path nobody is watching.
        self._jobs: List[int] = []

    # -- lease ------------------------------------------------------------- #
    def acquire(self, *, strict: bool = False) -> Lease:
        """Take (or heartbeat) the arm lease.

        ``strict`` is for a renewal DURING a run: a changed epoch then means
        we were preempted and got the arms back, so anything in flight is
        stale and the run must stop. At entry a changed epoch is just news.
        """
        data = self.client.request("POST", "/v1/arm/lease", {
            "holder": self.holder, "class": self.lease_class, "ttl_s": self.ttl_s})
        lease = Lease.parse(data)
        self._last_lease_at = self._clock()
        changed = self._epoch is not None and lease.epoch != self._epoch
        self._epoch = lease.epoch
        self.lease = lease
        if changed:
            # A fresh grant means we LOST the arms in between: mode, tool
            # registration and the last-sent pose are all stale now.
            self._last_sent_deg.clear()
            self._t0 = None
            if strict:
                raise LeasePreempted(
                    f"the arm lease epoch moved to {lease.epoch} while this "
                    f"run was in progress"
                    + (f" (preempted from {lease.preempted_from})"
                       if lease.preempted_from else "")
                    + ": the arms were held by somebody else in between, so "
                      "the mode, the ratios and the posture this plan was "
                      "checked against are all stale. Re-observe and replan.")
        return lease

    def renew(self) -> None:
        """Heartbeat if the lease is due. Safe to call from anywhere hot.

        Called from every polling loop and every send, so the lease survives a
        long trajectory, a slow settle and a model turn without anything
        having to remember to renew it. ``acquire`` was the only renewal
        implementation and NOTHING called it periodically (R6).
        """
        if self.lease is None:
            return
        due = self.ttl_s * HEARTBEAT_FRACTION
        if self._clock() - self._last_lease_at >= due:
            self.acquire(strict=True)

    def _heartbeat_loop(self) -> None:
        interval = max(0.05, self.ttl_s * HEARTBEAT_FRACTION)
        while not self._stop_beat.wait(interval):
            try:
                self.acquire()
            except Exception:  # noqa: BLE001 - the run's own calls will see it
                return

    def release(self) -> None:
        """Hand the arms back. Never raises — a failed release must not mask
        the exception that is on its way out of the ``with`` block."""
        self._stop_beat.set()
        beat, self._beat = self._beat, None
        if beat is not None:
            beat.join(timeout=1.0)
        self.cancel_jobs()
        if self.lease is None:
            return
        try:
            self.client.request("DELETE", "/v1/arm/lease", {"holder": self.holder})
        except Exception:  # noqa: BLE001 - see the docstring
            pass
        finally:
            self.lease = None

    def cancel_jobs(self) -> None:
        """Best-effort cancel of every trajectory we started and did not see end.

        ``_play`` used to cancel only on its own deadline, so a status-poll
        exception, a preemption or a Ctrl-C left the daemon playing a path
        with nobody watching it (R6).
        """
        for job in list(self._jobs):
            try:
                self.client.request("POST", f"/v1/arm/trajectory/{job}/cancel", {})
            except Exception:  # noqa: BLE001 - best effort, by definition
                pass
        self._jobs.clear()

    def __enter__(self) -> "FirmwareExecutor":
        self.acquire()
        try:
            self.position_mode()
        except BaseException:
            # HAND THE ARMS BACK. A failed setup used to leave the lease
            # taken until its TTL expired, and the probe confirmed no DELETE
            # was ever sent (R6): a robot nobody can drive for 30 seconds,
            # because the second arm's mode call raised.
            self.release()
            raise
        if self._want_heartbeat:
            self._stop_beat.clear()
            self._beat = threading.Thread(target=self._heartbeat_loop,
                                          name="mkit-lease", daemon=True)
            self._beat.start()
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

    def begin_run(self, plan: Plan = None) -> None:
        """A new plan starts a new clock.

        ``_t0`` was set once, at the first send of the FIRST plan, while
        ``run_steps`` restarts ``t`` at zero for every plan. The second plan's
        points were therefore all overdue the moment they were computed and
        went out in a burst: measured on a fake clock, plan one sent at 0.00 /
        0.02 / 0.04 s and plan two sent all three at 1.04 s after a one-second
        model turn. A per-command 2.8 deg bound is not a 140 deg/s bound when
        the commands have no time between them (R5).
        """
        self._t0 = None

    def end_run(self, plan: Plan = None) -> None:
        self._t0 = None

    def send_joints(self, q16, *, t: float) -> None:
        """One guarded dual-arm POST, paced to ``t``. The TARGET IS NOT ALTERED.

        ``t`` is PLAN time, seconds from the start of THIS run. A caller that
        stalls does not get a burst: the schedule is re-anchored so the points
        after the stall keep their spacing rather than all being overdue at
        once.

        A knot too large for one command is REFUSED
        (:class:`RateRefused`), never clipped. Clipping changed the checked
        geometry, could change the path, and lost the endpoint (R4).
        """
        q = np.asarray(q16, dtype=float).reshape(WIRE_DIM)
        self.renew()
        targets = {side: np.degrees(q[JOINT_SLICE[side]]) for side in SIDES}
        self._check_step(targets)
        now = self._clock()
        if self._t0 is None:
            self._t0 = now
        wait = (self._t0 + float(t)) - now
        if wait > 0:
            self._sleep(wait)
        elif wait < -self.period:
            # LATE. Re-anchor rather than flush: the remaining points keep
            # their planned spacing instead of arriving together.
            self._t0 = now - float(t)
        self.client.request("POST", "/v1/arm/move_joints_both", self._holder_body({
            "a": [float(v) for v in targets["left"]],
            "b": [float(v) for v in targets["right"]],
            "wait": False}))
        self._last_sent_deg = targets

    def _check_step(self, targets: Dict[str, np.ndarray]) -> None:
        """Refuse a command that would be a lunge. Never modify one."""
        for side, deg in targets.items():
            last = self._last_sent_deg.get(side)
            if last is None:
                continue
            step = float(np.max(np.abs(np.asarray(deg, dtype=float) - last)))
            if step > MAX_COMMAND_STEP_DEG + 1e-9:
                raise RateRefused(
                    f"one command would move a {side} joint {step:.2f} deg, "
                    f"over the {MAX_COMMAND_STEP_DEG:.2f} deg this transport "
                    f"puts on the wire in a single POST. Clipping it would "
                    f"send a posture the guard never checked and stop short "
                    f"of the endpoint; re-plan with smaller knots, or upload "
                    f"the plan as a trajectory (the default transport), which "
                    f"stretches the TIME instead of shrinking the motion.")

    def stream_schedule(self, plan: Plan) -> List[float]:
        """Plan time for every joint step, honouring the rate ceiling.

        EVERY KNOT, AT ITS OWN TIME. The geometry is what the guard approved;
        the only thing this is allowed to change is WHEN each knot is sent.
        A segment that would exceed :data:`MAX_JOINT_RATE_DEG_S` at the base
        period gets more time, so the ceiling is met by stretching rather than
        by shortening the motion (R4).

        Raises :class:`RateRefused` before anything is sent when a knot is too
        large for one command at all.
        """
        # Anchored on the posture the plan was CHECKED in, not on whatever
        # the robot reports at this instant: the binding is what guarantees
        # the two agree, and a schedule that silently re-anchors would hide a
        # drift the binding is there to refuse (R8).
        binding = getattr(plan, "binding", None)
        if binding is not None and binding.q0:
            start = {side: np.asarray(q, dtype=float)
                     for side, q in binding.q0.items()}
        else:
            start = self.state().joints
        current = {side: np.degrees(np.asarray(start[side], dtype=float))
                   for side in SIDES if side in start}
        missing = [s for s in SIDES if s not in current]
        if missing:
            raise FirmwareUnavailable(
                f"no measured joints for {missing}; a stream cannot be "
                f"anchored to a posture the robot will not report")
        times: List[float] = []
        t = 0.0
        for index, step in enumerate(plan.joint_steps()):
            nxt = dict(current)
            nxt[step.side] = np.degrees(np.asarray(step.q, dtype=float))
            span = max(float(np.max(np.abs(nxt[side] - current[side])))
                       for side in SIDES)
            if span > MAX_COMMAND_STEP_DEG + 1e-9:
                raise RateRefused(
                    f"the plan contains a {span:.2f} deg joint step, over the "
                    f"{MAX_COMMAND_STEP_DEG:.2f} deg one streamed command may "
                    f"carry. Refused BEFORE anything was sent, because the "
                    f"alternatives are clipping the geometry or truncating "
                    f"the path.")
            dt = max(self.period, span / self.max_rate)
            if index == 0:
                # The same interpolation window the trajectory upload gives
                # its first segment: long enough that the controller
                # interpolates into the motion rather than stepping into it.
                dt = max(dt, INTERPOLATION_S)
            t += dt
            times.append(round(t, 6))
            current = nxt
        return times

    def set_gripper(self, side: str, closedness: float, *, grip: str = "") -> None:
        self.renew()
        self.client.gripper_set(_wire_side(side), float(closedness),
                                grip=(grip or self.grip) or None)

    # -- barriers ---------------------------------------------------------- #
    def wait_arrived(self, q16, *, tol_rad: float = None,
                     timeout_s: float = None) -> ArrivalReport:
        """Poll the MEASURED joints until they reach the commanded posture.

        Transport completion is not arrival. The trajectory phase says the
        daemon played its last waypoint; this says the arm is there. A stopped
        arm can be short of the target — F5, ten grasps out of ten, the arm
        holding 2.5 deg from the commanded posture with the fingers jammed on
        the table.
        """
        tol = self.arrive_tol_rad if tol_rad is None else float(tol_rad)
        deadline_s = (self.arrive_timeout_s if timeout_s is None
                      else float(timeout_s))
        want = np.asarray(q16, dtype=float).reshape(WIRE_DIM)
        started = self._clock()
        worst = float("nan")
        while True:
            self.renew()
            measured = self.state().joints
            missing = [s for s in SIDES if s not in measured]
            if missing:
                return ArrivalReport(False, worst, self._clock() - started,
                                     f"no measured joints for {missing}")
            worst = max(float(np.max(np.abs(
                np.asarray(measured[side], dtype=float) - want[JOINT_SLICE[side]])))
                for side in SIDES)
            if worst <= tol:
                return ArrivalReport(True, worst, self._clock() - started,
                                     "measured at the commanded posture")
            waited = self._clock() - started
            if waited >= deadline_s:
                return ArrivalReport(
                    False, worst, waited,
                    f"the worst joint is still {math.degrees(worst):.2f} deg "
                    f"from the commanded posture after {waited:.1f}s "
                    f"(tolerance {math.degrees(tol):.2f} deg)")
            self._sleep(min(self.period, max(0.0, deadline_s - waited)))

    def wait_gripper_settled(self, side: str, *, timeout_s: float = None
                             ) -> StrokeReport:
        """Poll the gripper until the stroke reaches a TERMINAL state.

        Terminal is one of: the jaws stopped moving (two consecutive identical
        readings), or the producer reports ``holding``. ``gripper_set`` posted
        the command and threw the reply away, and the in-repo fake establishes
        nothing about whether the pinned client blocks — so this does not
        assume it does. F13 is what reading the evidence too early costs.
        """
        deadline_s = (self.stroke_timeout_s if timeout_s is None
                      else float(timeout_s))
        started = self._clock()
        wire = _wire_side(side)
        previous: Optional[float] = None
        while True:
            self.renew()
            try:
                report = self.client.gripper_state(wire)
            except Exception as exc:  # noqa: BLE001
                return StrokeReport(False, waited_s=self._clock() - started,
                                    detail=f"the gripper cannot be read: {exc}")
            jaw = float(getattr(report, "jaw_rad", float("nan")))
            open_rad = float(getattr(report, "open_rad", 0.0) or 0.0)
            closedness = (float("nan") if open_rad <= 0.0
                          else max(0.0, min(1.0, 1.0 - jaw / open_rad)))
            holding = bool(getattr(report, "holding", False))
            stopped = previous is not None and abs(jaw - previous) <= 1e-6
            if holding or stopped:
                return StrokeReport(True, closedness, holding,
                                    stalled=holding or None,
                                    waited_s=self._clock() - started,
                                    detail=("the producer reports a hold"
                                            if holding else
                                            "the jaws stopped moving"))
            previous = jaw
            waited = self._clock() - started
            if waited >= deadline_s:
                return StrokeReport(
                    False, closedness, holding, None, waited,
                    f"the stroke had not reached a terminal state after "
                    f"{waited:.1f}s (jaw {jaw:.3f} rad, still moving)")
            self._sleep(min(self.period, max(0.0, deadline_s - waited)))

    def settle(self, timeout_s: float) -> SettleReport:
        """Poll until BOTH arms report stationary, or give up and say so.

        ``ArmState.stationary`` is the daemon's own judgement, from the same
        feedback frame it reports the velocities in. A verifier run while the
        arm is still moving measures the middle of the motion.
        """
        started = self._clock()
        worst = float("nan")
        while True:
            self.renew()
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
    def run_plan(self, plan: Plan, *, arrive_tol_rad: float = None,
                 arrive_timeout_s: float = None,
                 stroke_timeout_s: float = None) -> RunReport:
        """Play a pre-checked plan, trajectory-first. Used by
        :func:`manipulation_kit.executor.run` when this executor is passed.

        The BARRIERS ARE THE SAME ONES the generic runner enforces: the arm
        must be measurably at the commanded posture before a stroke, the
        stroke must reach a terminal state, and a failed settle ends the run.
        Transport ``completed`` is not any of those.
        """
        if not getattr(plan, "ok", False):
            from ..executor import REFUSED_PLAN  # noqa: PLC0415
            return RunReport(getattr(plan, "primitive", "?"),
                             getattr(plan, "side", ""), False, 0,
                             error=str(plan), stop_reason=REFUSED_PLAN)
        tol = self.arrive_tol_rad if arrive_tol_rad is None else arrive_tol_rad
        arrive_s = (self.arrive_timeout_s if arrive_timeout_s is None
                    else arrive_timeout_s)
        stroke_s = (self.stroke_timeout_s if stroke_timeout_s is None
                    else stroke_timeout_s)
        self.begin_run(plan)
        if self.transport == "stream":
            # run_steps, NOT run: run() delegates to run_plan, which is this.
            from ..executor import run_steps  # noqa: PLC0415
            try:
                schedule = self.stream_schedule(plan)
            except FirmwareUnavailable as exc:
                # REFUSED BEFORE ANYTHING WAS SENT. That is the whole point:
                # a plan this transport cannot carry is not half-carried.
                return RunReport(plan.primitive, plan.side, False, 0,
                                 error=str(exc), stop_reason=TRANSPORT_ERROR)
            return run_steps(plan, self, hz=self.hz, schedule=schedule,
                             arrive_tol_rad=tol, arrive_timeout_s=arrive_s,
                             stroke_timeout_s=stroke_s)
        sent = 0
        settle: Optional[SettleReport] = None
        arrivals: List[ArrivalReport] = []
        strokes: List[StrokeReport] = []
        batch: List[JointStep] = []
        last: Dict[str, np.ndarray] = {}

        def report(index: int, reason: str, detail: str) -> RunReport:
            self.cancel_jobs()
            return RunReport(plan.primitive, plan.side, False, sent, settle,
                             error=detail, stop_reason=reason,
                             stopped_at=index, arrivals=tuple(arrivals),
                             strokes=tuple(strokes))

        try:
            for index, step in enumerate(plan.steps):
                if isinstance(step, JointStep):
                    batch.append(step)
                    last[step.side] = np.asarray(step.q, dtype=float)
                    continue
                sent += self._play(batch)
                batch = []
                if isinstance(step, GripStep):
                    if last:
                        arrival = self.wait_arrived(self._vector(last),
                                                    tol_rad=tol,
                                                    timeout_s=arrive_s)
                        arrivals.append(arrival)
                        if not arrival.arrived:
                            return report(index, BARRIER_FAILED, arrival.detail)
                    self.set_gripper(step.side, step.closedness, grip=step.grip)
                    stroke = self.wait_gripper_settled(step.side,
                                                       timeout_s=stroke_s)
                    strokes.append(stroke)
                    if not stroke.settled:
                        return report(index, BARRIER_FAILED, stroke.detail)
                    sent += 1
                elif isinstance(step, SettleStep):
                    settle = self.settle(step.timeout_s)
                    sent += 1
                    if not settle.settled:
                        return report(index, BARRIER_FAILED, settle.detail)
                else:
                    # The firmware runner used to skip an unknown step in
                    # silence while the generic one raised. Same closed set,
                    # same answer.
                    return report(index, TRANSPORT_ERROR,
                                  f"not a plan step: {step!r}")
            sent += self._play(batch)
        except FirmwareUnavailable as exc:
            return report(-1, TRANSPORT_ERROR, str(exc))
        finally:
            self.end_run(plan)
        return RunReport(plan.primitive, plan.side, True, sent, settle,
                         arrivals=tuple(arrivals), strokes=tuple(strokes))

    def _vector(self, per_side: Dict[str, np.ndarray]) -> np.ndarray:
        """A 16-vector of the last commanded joints, filled from measurement.

        The arrival barrier compares BOTH arms, so the arm this plan never
        touched is compared against where it actually is.
        """
        measured = self.state().joints
        out = np.zeros(WIRE_DIM, dtype=float)
        for side in SIDES:
            q = per_side.get(side, measured.get(side))
            if q is not None:
                out[JOINT_SLICE[side]] = np.asarray(q, dtype=float)
        return out

    def _play(self, steps: Sequence[JointStep]) -> int:
        """Upload one contiguous run of joint steps as a trajectory job."""
        if not steps:
            return 0
        self.renew()
        waypoints = self._waypoints(steps)
        status = self.client.request("POST", "/v1/arm/trajectory/start",
                                     self._holder_body({"waypoints": waypoints}))
        job = int(status["id"])
        self._jobs.append(job)
        deadline = self._clock() + waypoints[-1]["t"] + 5.0
        try:
            while True:
                self.renew()
                status = self.client.request(
                    "GET", f"/v1/arm/trajectory/{job}/status")
                phase = str(status["phase"])
                if phase == "completed":
                    self._jobs.remove(job)
                    return len(waypoints)
                if phase in ("failed", "cancelled"):
                    self._jobs.remove(job)
                    raise FirmwareUnavailable(
                        f"trajectory {job} {phase}: {status.get('message')}")
                if self._clock() > deadline:
                    raise FirmwareUnavailable(
                        f"trajectory {job} was still running "
                        f"{waypoints[-1]['t'] + 5.0:.1f}s in; cancelled")
                self._sleep(self.period)
        except BaseException:
            # ANY way out that is not "completed" cancels the job. A polling
            # exception, a preemption, a KeyboardInterrupt: the daemon must
            # not be left playing a path nobody is watching (R6).
            self.cancel_jobs()
            raise

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
