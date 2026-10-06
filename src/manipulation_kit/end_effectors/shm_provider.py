"""A reference provider of ``end_effector.shm/1`` over the iceoryx2 0.9.3
Python bindings.

A *provider* is the process that drives an end effector ``d1-firmwared`` does
not drive itself: the robot's daemon configuration declares the side
``driver = "shm"``, the provider announces what it drives, and every client
reaches it through the daemon's ``/v1/end_effectors/{side}`` resource with
the daemon's validation, timestamp rule, soft kill and stop in front of it.

This module is the runtime a Python provider is built on, and two simulated
end effectors that show it end to end:

* :class:`Provider` — what a provider implements: its declaration, ``set``,
  ``joints``, ``read``, ``stop``, and optionally ``clear_fault``,
  ``registers``, ticks and a hook for the daemon's presence.
* :func:`serve` — owns every iceoryx2 port on the calling thread and applies
  the contract's rules before a provider method is called: a ``set`` for
  another instance or descriptor generation or past its start deadline is
  answered ``stale_target`` and not executed; an undeclared grip, torque or
  ``extra`` key is refused; a ``joints`` sample older than 200 ms, for
  another instance or out of limits is dropped and the newest valid one is
  delivered one at a time; a rise of the daemon's stop generation calls
  ``stop`` (the first value seen from a daemon is the baseline); the
  heartbeat, the descriptor republish and the state read run on their
  periods. Provider methods that may block run on worker threads, so a long
  stroke never delays a heartbeat or a stop.
* :class:`SimulatedGripper` (detail ``sim.gripper/1``) and
  :class:`SimulatedHand` (detail ``hand.state/1``) — simulated devices, and
  :func:`main`, the ``mkit-ee-provider`` command that serves one.

The bindings are the optional ``[shm]`` extra; nothing here imports them
until :func:`serve` runs. The contract's text is
``spec/end_effector.shm/1.md`` in the d1-firmware repository.
"""

from __future__ import annotations

import abc
import argparse
import concurrent.futures
import logging
import os
import queue
import signal
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple

from . import shm_contract as c

LOG = logging.getLogger("manipulation_kit.end_effectors.shm_provider")

#: ``ProviderError.kind`` to the response status the contract defines.
STATUS_OF_KIND = {
    "refused": c.STATUS_REFUSED,
    "unprocessable": c.STATUS_UNPROCESSABLE,
    "unavailable": c.STATUS_UNAVAILABLE,
    "device": c.STATUS_DEVICE,
    "timeout": c.STATUS_TIMEOUT,
    "stale_target": c.STATUS_STALE_TARGET,
    "invalid": c.STATUS_INVALID,
}


class ProviderError(Exception):
    """A provider's refusal or failure, of one of the contract's kinds."""

    def __init__(self, kind: str, message: str) -> None:
        if kind not in STATUS_OF_KIND:
            raise ValueError(f"unknown error kind {kind!r}")
        super().__init__(message)
        self.kind = kind
        self.message = message


class Provider(abc.ABC):
    """An end effector served over the contract.

    ``set``, ``joints``, ``stop``, ``clear_fault`` and ``registers`` are
    called from worker threads and may block; ``read``, ``tick`` and
    ``daemon_presence`` are called from the thread that owns the ports and
    must return at once. Raise :class:`ProviderError` to refuse or fail.
    """

    @abc.abstractmethod
    def descriptor(self) -> c.DescriptorSpec:
        """What this provider drives. Read once, when serving starts."""

    @abc.abstractmethod
    def set(self, command: c.SetCommand) -> None:
        """Drive to an openness; with ``wait`` and ``stroke_completion``,
        return when the stroke concludes."""

    @abc.abstractmethod
    def joints(self, targets: List[float], timestamp_us: int) -> None:
        """Drive every joint to its target; never wait for motion."""

    @abc.abstractmethod
    def read(self) -> c.StateSample:
        """The current state."""

    @abc.abstractmethod
    def stop(self) -> None:
        """Stop and take the actuators' authority away. Never opens;
        idempotent (a soft kill may deliver it twice)."""

    def clear_fault(self) -> Dict[str, Any]:
        """``{"ok", "fault", "details"}``, for a provider declaring
        ``clear_fault``."""
        raise ProviderError("refused", "this provider does not clear faults")

    def registers(self, ids: Optional[List[str]]) -> Dict[str, Any]:
        """``{"entries": [{"id", "name", "value", "details"}], "details"}``,
        for a provider declaring ``registers``."""
        raise ProviderError("refused", "this provider does not read registers")

    def tick(self) -> Optional[c.TickSample]:
        """The newest control-loop tick, for a provider declaring ``ticks``;
        published whenever its ``seq`` is new."""
        return None

    def daemon_presence(self, present: bool) -> None:  # noqa: B027 - optional hook
        """The daemon appeared, or its ``control`` stopped arriving. Keep
        holding what is held and start no motion while it is away."""


# --------------------------------------------------------------------------
# The rules, as functions of their inputs
# --------------------------------------------------------------------------


def check_set(descriptor: c.DescriptorSpec, command: c.SetCommand) -> Optional[ProviderError]:
    """The refusal a ``set`` earns against a declaration, or ``None``."""
    caps = descriptor.capabilities
    if not caps.openness:
        return ProviderError("refused", "this end effector does not take an openness")
    if command.grip is not None and command.grip not in caps.grip_presets:
        return ProviderError("unprocessable", f"no grip preset {command.grip!r}")
    if command.torque_nm is not None and not caps.grip_torque:
        return ProviderError("unprocessable", "no explicit torque_nm")
    for key in (command.extra or {}):
        if key not in caps.set_extra:
            return ProviderError("unprocessable", f"no extra parameter {key!r}")
    return None


def check_joints(descriptor: c.DescriptorSpec, targets: List[float]) -> Optional[str]:
    """Why joint targets do not fit a declaration, or ``None``."""
    if not descriptor.capabilities.joints:
        return "this end effector does not take joint targets"
    if len(targets) != len(descriptor.joints):
        return f"{len(targets)} targets for {len(descriptor.joints)} joints"
    for joint, target in zip(descriptor.joints, targets):
        if not joint.min <= target <= joint.max:
            return f"joint {joint.name} target {target} is outside [{joint.min}, {joint.max}]"
    return None


def triage_command(envelope: c.CommandEnvelope, instance_id: int, generation: int,
                   descriptor: c.DescriptorSpec,
                   now_ns: int) -> Optional[ProviderError]:
    """The answer a ``command`` request gets without running, or ``None`` to
    run it. A stop always runs."""
    if envelope.command is None:
        return None
    if envelope.target_instance_id != instance_id:
        return ProviderError("stale_target", "the request is for another provider instance")
    if envelope.descriptor_generation != generation:
        return ProviderError("stale_target",
                             "the request was validated against another generation")
    if now_ns > envelope.start_by_mono_ns:
        return ProviderError("stale_target", "the request arrived after its start deadline")
    return check_set(descriptor, envelope.command)


def joints_usable(targets: c.JointTargets, instance_id: int, generation: int,
                  descriptor: c.DescriptorSpec, now_ns: int) -> bool:
    """Whether a ``joints`` sample may reach the provider."""
    return (targets.target_instance_id == instance_id
            and targets.descriptor_generation == generation
            and now_ns - targets.issued_mono_ns <= c.JOINTS_MAX_AGE_MS * 1_000_000
            and check_joints(descriptor, targets.targets) is None)


class StopGenerations:
    """The daemon's stop generation, as a provider follows it: the first
    value seen from a daemon instance is the baseline, and only a rise
    after it is a stop."""

    def __init__(self) -> None:
        self.daemon_instance = 0
        self.generation = 0

    def observe(self, daemon_instance: int, generation: int) -> bool:
        """Record one ``control`` word; ``True`` when it asks for a stop."""
        if daemon_instance != self.daemon_instance:
            self.daemon_instance = daemon_instance
            self.generation = generation
            return False
        if generation > self.generation:
            self.generation = generation
            return True
        return False


# --------------------------------------------------------------------------
# The runtime
# --------------------------------------------------------------------------


@dataclass
class ServeOptions:
    """How a provider is served."""

    #: The side's base service name; the daemon's ``shm_service``.
    service: str = "omakase/ee/b"
    root_path: str = c.DEFAULT_ROOT_PATH
    prefix: str = c.DEFAULT_PREFIX
    node_name: str = "ee-provider-py"
    state_period_s: float = 0.02
    heartbeat_period_s: float = 0.05
    daemon_stale_after_s: float = c.DEFAULT_STALE_AFTER_MS / 1000
    #: How long the loop sleeps between polls of its ports.
    poll_s: float = 0.002


def _iceoryx2():
    try:
        import iceoryx2  # noqa: PLC0415 - optional, imported only to serve
    except ImportError as error:  # pragma: no cover - environment dependent
        raise ImportError(
            "serving a provider needs the iceoryx2 Python bindings 0.9.3: "
            "pip install 'manipulation-kit[shm]'") from error
    return iceoryx2


class _JointsWorker(threading.Thread):
    """Hands the newest joint targets to the provider, one call at a time."""

    def __init__(self, provider: Provider) -> None:
        super().__init__(name="ee-provider-joints", daemon=True)
        self._provider = provider
        self._slot: Optional[c.JointTargets] = None
        self._ready = threading.Condition()
        self._done = False

    def offer(self, targets: c.JointTargets) -> None:
        with self._ready:
            self._slot = targets
            self._ready.notify()

    def close(self) -> None:
        with self._ready:
            self._done = True
            self._ready.notify()

    def run(self) -> None:
        while True:
            with self._ready:
                while self._slot is None and not self._done:
                    self._ready.wait()
                if self._done:
                    return
                targets, self._slot = self._slot, None
            try:
                self._provider.joints(targets.targets, targets.timestamp_us)
            except ProviderError as error:
                LOG.debug("joint targets refused by the provider: %s", error.message)
            except Exception:  # noqa: BLE001 - a provider's defect must not end serving
                LOG.exception("joints raised")


class ShmProvider:
    """One provider bound to one side's services."""

    def __init__(self, provider: Provider, options: ServeOptions) -> None:
        self.provider = provider
        self.options = options
        self.descriptor = provider.descriptor()
        self.descriptor.validate()
        if not 0 < options.heartbeat_period_s <= c.MAX_HEARTBEAT_PERIOD_MS / 1000:
            raise ValueError("heartbeat_period_s must be above 0 and at most 0.1 s")
        if not 0 < options.state_period_s <= 1.0:
            raise ValueError("state_period_s must be above 0 and at most 1 s")
        self.instance_id = c.new_instance_id()
        self.generation = 1
        self._seq: Dict[str, int] = {}
        self._ports: Dict[str, Any] = {}
        self._node = None
        self._results: "queue.Queue[Tuple[str, int, Any]]" = queue.Queue()
        self._parked: Dict[str, Dict[int, Any]] = {"command": {}, "fault": {}}
        self._next_id = 0
        self._set_pool = concurrent.futures.ThreadPoolExecutor(1, "ee-provider-set")
        self._stop_pool = concurrent.futures.ThreadPoolExecutor(1, "ee-provider-stop")
        self._fault_pool = concurrent.futures.ThreadPoolExecutor(1, "ee-provider-fault")
        self._joints = _JointsWorker(provider)
        self._stops = StopGenerations()
        self._daemon_seen: Optional[float] = None
        self._daemon_present = False
        self._last_tick_seq: Optional[int] = None

    # -- ports ----------------------------------------------------------------

    def _header(self, service: str) -> c.Header:
        self._seq[service] = self._seq.get(service, 0) + 1
        return c.header(self.instance_id, self._seq[service])

    def open(self) -> None:
        """Create the node and every port. Raises when the root cannot be
        created (the daemon's runtime directory does not exist yet) or when
        another provider already serves the side."""
        iox2 = _iceoryx2()
        os.makedirs(self.options.root_path, exist_ok=True)
        config = iox2.config.default()
        config.global_cfg.root_path = iox2.Path.new(
            self.options.root_path if self.options.root_path.endswith("/")
            else self.options.root_path + "/")
        config.global_cfg.prefix = iox2.FileName.new(self.options.prefix)
        self._node = (iox2.NodeBuilder.new().config(config)
                      .name(iox2.NodeName.new(self.options.node_name))
                      .create(iox2.ServiceType.Ipc))
        layout = {service["suffix"]: service for service in c.load_layout()["services"]}

        def topic(suffix: str, payload):
            spec = layout[suffix]
            return (self._node.service_builder(
                iox2.ServiceName.new(f"{self.options.service}/{suffix}"))
                .publish_subscribe(payload)
                .history_size(spec["history_size"])
                .subscriber_max_buffer_size(spec["subscriber_max_buffer_size"])
                .enable_safe_overflow(spec["enable_safe_overflow"])
                .max_publishers(spec["max_publishers"])
                .max_subscribers(spec["max_subscribers"])
                .open_or_create())

        def rpc(suffix: str, request, response):
            spec = layout[suffix]
            return (self._node.service_builder(
                iox2.ServiceName.new(f"{self.options.service}/{suffix}"))
                .request_response(request, response)
                .max_servers(spec["max_servers"])
                .max_clients(spec["max_clients"])
                .max_active_requests_per_client(spec["max_active_requests_per_client"])
                .max_response_buffer_size(spec["max_response_buffer_size"])
                .enable_safe_overflow_for_requests(spec["enable_safe_overflow_for_requests"])
                .enable_safe_overflow_for_responses(spec["enable_safe_overflow_for_responses"])
                .open_or_create())

        try:
            self._ports["descriptor"] = topic(c.DESCRIPTOR, c.Descriptor) \
                .publisher_builder().create()
        except Exception as error:
            raise RuntimeError(f"cannot publish {self.options.service}/descriptor (is another "
                               f"provider serving this side?): {error}") from error
        self._ports["heartbeat"] = topic(c.HEARTBEAT, c.Heartbeat).publisher_builder().create()
        self._ports["state"] = topic(c.STATE, c.State).publisher_builder().create()
        self._ports["tick"] = topic(c.TICK, c.Tick).publisher_builder().create()
        self._ports["joints"] = topic(c.JOINTS, c.JointsCommand).subscriber_builder().create()
        self._ports["control"] = topic(c.CONTROL, c.Control).subscriber_builder().create()
        self._ports["command"] = rpc(c.COMMAND, c.CommandRequest, c.CommandResponse) \
            .server_builder().create()
        self._ports["fault"] = rpc(c.FAULT, c.FaultRequest, c.FaultResponse) \
            .server_builder().create()
        self._joints.start()
        LOG.info("%s provider serving %s (root %s, prefix %s, instance %#x): %s %s",
                 c.CONTRACT, self.options.service, self.options.root_path, self.options.prefix,
                 self.instance_id, self.descriptor.driver, self.descriptor.model)

    def close(self) -> None:
        """Close every port and stop the workers."""
        self._joints.close()
        for pool in (self._set_pool, self._stop_pool, self._fault_pool):
            pool.shutdown(wait=False, cancel_futures=True)
        for parked in self._parked.values():
            for request in parked.values():
                request.delete()
            parked.clear()
        for port in self._ports.values():
            port.delete()
        self._ports.clear()
        self._node = None

    # -- the loop ---------------------------------------------------------------

    def run(self, stop: threading.Event) -> None:
        """Serve until ``stop`` is set."""
        now = time.monotonic()
        due = {"heartbeat": now, "descriptor": now, "state": now}
        while not stop.is_set():
            now = time.monotonic()
            self._send_results()
            if now >= due["descriptor"]:
                due["descriptor"] = now + c.LATCHED_REPUBLISH_MS / 1000
                self._ports["descriptor"].send_copy(
                    self.descriptor.to_wire(self.generation, self._header("descriptor")))
            if now >= due["heartbeat"]:
                due["heartbeat"] = now + self.options.heartbeat_period_s
                self._ports["heartbeat"].send_copy(c.heartbeat(
                    self.generation, int(self.options.state_period_s * 1000),
                    self._header("heartbeat")))
            if now >= due["state"]:
                due["state"] = now + self.options.state_period_s
                self._publish_state()
            self._publish_tick()
            self._receive_control(now)
            self._receive_joints()
            self._receive_commands()
            self._receive_faults()
            time.sleep(self.options.poll_s)

    def _publish_state(self) -> None:
        try:
            sample = self.provider.read()
        except ProviderError as error:
            sample = c.StateSample(fault=f"state read failed: {error.message}")
        except Exception as error:  # noqa: BLE001 - published, not fatal
            sample = c.StateSample(fault=f"state read raised: {error}")
        try:
            message = sample.to_wire(self.generation, self._header("state"))
        except c.ContractError as error:
            LOG.warning("the provider's state does not encode; not sent: %s", error)
            return
        self._ports["state"].send_copy(message)

    def _publish_tick(self) -> None:
        if not self.descriptor.capabilities.ticks:
            return
        sample = self.provider.tick()
        if sample is None or sample.seq == self._last_tick_seq:
            return
        self._last_tick_seq = sample.seq
        try:
            self._ports["tick"].send_copy(sample.to_wire(self.generation, self._header("tick")))
        except c.ContractError as error:
            LOG.warning("the provider's tick does not encode; not sent: %s", error)

    def _receive_control(self, now: float) -> None:
        while True:
            sample = self._ports["control"].receive()
            if sample is None:
                break
            try:
                daemon, generation = c.read_control(sample.payload().contents)
            except c.ContractError as error:
                LOG.warning("control refused: %s", error)
                continue
            finally:
                sample.delete()
            self._daemon_seen = now
            if self._stops.observe(daemon, generation):
                self._stop_pool.submit(self._call_stop, None)
        present = (self._daemon_seen is not None
                   and now - self._daemon_seen <= self.options.daemon_stale_after_s)
        if present != self._daemon_present:
            self._daemon_present = present
            LOG.info("d1-firmwared %s", "present" if present else "absent")
            self.provider.daemon_presence(present)

    def _receive_joints(self) -> None:
        newest = None
        while True:
            sample = self._ports["joints"].receive()
            if sample is None:
                break
            try:
                newest = c.read_joints(sample.payload().contents)
            except c.ContractError as error:
                LOG.warning("joint targets refused: %s", error)
            finally:
                sample.delete()
        if newest is not None and joints_usable(newest, self.instance_id, self.generation,
                                                self.descriptor, c.monotonic_ns()):
            self._joints.offer(newest)

    def _park(self, lane: str, request: Any) -> int:
        self._next_id += 1
        self._parked[lane][self._next_id] = request
        return self._next_id

    def _call_set(self, request_id: int, command: c.SetCommand) -> None:
        self._results.put(("command", request_id,
                           _outcome(lambda: self.provider.set(command))))

    def _call_fault(self, request_id: int, op: int, ids: Optional[List[str]]) -> None:
        if op == c.OP_CLEAR_FAULT:
            outcome = _outcome(self.provider.clear_fault)
        else:
            outcome = _outcome(lambda: self.provider.registers(ids))
        self._results.put(("fault", request_id, outcome))

    def _call_stop(self, request_id: Optional[int]) -> None:
        outcome = _outcome(self.provider.stop)
        if request_id is not None:
            self._results.put(("command", request_id, outcome))

    def _receive_commands(self) -> None:
        server = self._ports["command"]
        while True:
            request = server.receive()
            if request is None:
                break
            try:
                envelope = c.read_command(request.payload().contents)
            except c.ContractError as error:
                self._respond_command(request, ProviderError("invalid", str(error)))
                continue
            refusal = triage_command(envelope, self.instance_id, self.generation,
                                     self.descriptor, c.monotonic_ns())
            if refusal is not None:
                self._respond_command(request, refusal)
                continue
            request_id = self._park("command", request)
            if envelope.command is None:
                self._stop_pool.submit(self._call_stop, request_id)
            else:
                self._set_pool.submit(self._call_set, request_id, envelope.command)

    def _receive_faults(self) -> None:
        server = self._ports["fault"]
        while True:
            request = server.receive()
            if request is None:
                break
            try:
                target, op, ids = c.read_fault_request(request.payload().contents)
            except c.ContractError as error:
                self._respond_fault(request, ProviderError("invalid", str(error)))
                continue
            if target != self.instance_id:
                self._respond_fault(request, ProviderError(
                    "stale_target", "the request is for another provider instance"))
                continue
            request_id = self._park("fault", request)
            self._fault_pool.submit(self._call_fault, request_id, op, ids)

    def _send_results(self) -> None:
        while True:
            try:
                lane, request_id, outcome = self._results.get_nowait()
            except queue.Empty:
                return
            request = self._parked[lane].pop(request_id, None)
            if request is None:
                continue
            if lane == "command":
                self._respond_command(request, outcome)
            else:
                self._respond_fault(request, outcome)

    def _respond_command(self, request: Any, outcome: Any) -> None:
        if isinstance(outcome, ProviderError):
            response = c.command_response(STATUS_OF_KIND[outcome.kind], outcome.message,
                                          self._header("command"))
        else:
            response = c.command_response(c.STATUS_OK, "", self._header("command"))
        request.send_copy(response)
        request.delete()

    def _respond_fault(self, request: Any, outcome: Any) -> None:
        if isinstance(outcome, ProviderError):
            response = c.fault_response(STATUS_OF_KIND[outcome.kind], outcome.message, None,
                                        self._header("fault"))
        else:
            response = c.fault_response(c.STATUS_OK, "", outcome, self._header("fault"))
        request.send_copy(response)
        request.delete()


def _outcome(call: Callable[[], Any]) -> Any:
    """``call``'s result, or the :class:`ProviderError` it raised (any other
    exception becomes a ``device`` error)."""
    try:
        return call()
    except ProviderError as error:
        return error
    except Exception as error:  # noqa: BLE001 - answered, not fatal
        LOG.exception("provider call raised")
        return ProviderError("device", f"the provider raised: {error}")


def serve(provider: Provider, options: ServeOptions,
          stop: Optional[threading.Event] = None, retry_s: Optional[float] = 1.0) -> None:
    """Serve ``provider`` until ``stop`` is set.

    With ``retry_s``, a root that cannot be opened yet (a provider started
    before the daemon) is retried at that interval instead of failing.
    """
    stop = stop or threading.Event()
    while not stop.is_set():
        runtime = ShmProvider(provider, options)
        try:
            runtime.open()
        except Exception as error:  # noqa: BLE001 - retried or raised below
            runtime.close()
            if retry_s is None:
                raise
            LOG.warning("cannot open %s yet (%s); retrying", options.service, error)
            stop.wait(retry_s)
            continue
        try:
            runtime.run(stop)
        finally:
            runtime.close()
        return


# --------------------------------------------------------------------------
# Simulated end effectors
# --------------------------------------------------------------------------


class _Motion:
    """A position that moves toward a target at a fixed speed, in a
    background thread, with an optional object that stops a closing
    motion."""

    def __init__(self, count: int, speed: float, start: float = 1.0) -> None:
        self.positions = [start] * count
        self.targets = [start] * count
        self.speed = speed
        self.block_at: Optional[float] = None
        self.lock = threading.Lock()
        self.tick_seq = 0
        self.phase = c.PHASE_IDLE
        threading.Thread(target=self._run, name="ee-sim-motion", daemon=True).start()

    def _run(self) -> None:
        step_s = 0.005
        while True:
            time.sleep(step_s)
            with self.lock:
                moved = False
                for index, target in enumerate(self.targets):
                    if self.block_at is not None and target < self.block_at:
                        target = self.block_at
                    position = self.positions[index]
                    delta = max(-self.speed * step_s, min(self.speed * step_s, target - position))
                    if delta:
                        self.positions[index] = position + delta
                        moved = True
                if moved:
                    self.tick_seq += 1

    def arrived(self) -> bool:
        with self.lock:
            return all(abs(p - (max(t, self.block_at) if self.block_at is not None else t))
                       < 1e-6 for p, t in zip(self.positions, self.targets))


class SimulatedGripper(Provider):
    """A simulated two-finger parallel gripper (detail ``sim.gripper/1``).

    One joint, ``jaw``, as a fraction of the stroke (``1.0`` open). ``set``
    moves at ``speed`` strokes per second and, with ``wait``, returns when the
    jaw arrives or stops on the simulated object (``object_at``, a jaw
    fraction, or ``None``). Its detail, schema ``sim.gripper/1``, carries
    ``model``, ``kind``, ``torque_cap_nm`` and ``stroke_interrupted``.
    """

    #: The schema id of this provider's detail, its own.
    DETAIL_SCHEMA = "sim.gripper/1"
    CAPS_NM = {"soft": 0.35, "firm": 1.0, "strong": 1.5}

    def __init__(self, model: str = "sim/two-finger", speed: float = 2.0,
                 object_at: Optional[float] = None) -> None:
        self.model = model
        self.motion = _Motion(1, speed)
        self.motion.block_at = object_at
        self.kind = "empty"
        self.cap_nm: Optional[float] = None
        self.interrupted = False
        self.fault: Optional[str] = None
        self._stopped = threading.Event()

    def descriptor(self) -> c.DescriptorSpec:
        return c.DescriptorSpec(
            driver="sim-gripper", detail_schema=self.DETAIL_SCHEMA, model=self.model,
            node_id=None,
            joints=[c.JointSpec("jaw", "fraction", 0.0, 1.0)],
            capabilities=c.Capabilities(
                openness=True, joints=True, stroke_completion=True,
                grip_presets=["soft", "firm", "strong"], force_sensing="estimated",
                contact_sensing=True, clear_fault=True, registers=True, ticks=True,
                max_command_hz=100),
            set_timeout_ms=3000)

    def set(self, command: c.SetCommand) -> None:
        if self.fault:
            raise ProviderError("refused", f"faulted: {self.fault}")
        self._stopped.clear()
        closing = command.openness < self.motion.positions[0]
        self.cap_nm = self.CAPS_NM[command.grip or "firm"] if closing else self.cap_nm
        with self.motion.lock:
            self.motion.targets = [command.openness]
            self.motion.phase = c.PHASE_CLOSING if closing else c.PHASE_OPENING
        self.interrupted = False
        if not command.wait:
            return
        deadline = time.monotonic() + 3.0
        while not self.motion.arrived():
            if self._stopped.is_set():
                self.interrupted = True
                raise ProviderError("refused", "the stroke was stopped")
            if time.monotonic() > deadline:
                raise ProviderError("timeout", "the stroke did not conclude")
            time.sleep(0.005)
        blocked = self.motion.block_at is not None and command.openness < self.motion.block_at
        self.kind = ("grasp" if blocked else "empty") if closing else "open"
        with self.motion.lock:
            self.motion.phase = c.PHASE_HOLD if blocked else c.PHASE_IDLE

    def joints(self, targets: List[float], timestamp_us: int) -> None:
        with self.motion.lock:
            self.motion.targets = list(targets)

    def read(self) -> c.StateSample:
        with self.motion.lock:
            position = self.motion.positions[0]
        holding = self.kind == "grasp"
        return c.StateSample(
            openness=max(0.0, min(1.0, position)), joints=[position], live=True,
            tracking=False, holding=holding, contact=holding,
            cap_reached=holding if self.cap_nm is not None else None,
            torque_nm=self.cap_nm if holding else 0.0, fault=self.fault,
            detail={"schema": self.DETAIL_SCHEMA, "data": {
                "model": self.model, "kind": self.kind, "torque_cap_nm": self.cap_nm,
                "stroke_interrupted": self.interrupted, "jaw_fraction": position}})

    def stop(self) -> None:
        self._stopped.set()
        with self.motion.lock:
            self.motion.targets = list(self.motion.positions)
            self.motion.phase = c.PHASE_IDLE

    def clear_fault(self) -> Dict[str, Any]:
        cleared, self.fault = self.fault, None
        return {"ok": True, "fault": None, "details": {"cleared": cleared}}

    def registers(self, ids: Optional[List[str]]) -> Dict[str, Any]:
        known = {"position": lambda: self.motion.positions[0], "kind": lambda: self.kind}
        wanted = ids or list(known)
        unknown = [i for i in wanted if i not in known]
        if unknown:
            raise ProviderError("unprocessable", f"no register {unknown[0]!r}")
        return {"entries": [{"id": i, "name": i, "value": known[i](), "details": {}}
                            for i in wanted], "details": {"simulated": True}}

    def tick(self) -> Optional[c.TickSample]:
        with self.motion.lock:
            return c.TickSample(seq=self.motion.tick_seq, timestamp_us=c.wall_clock_us(),
                                measured=list(self.motion.positions),
                                commanded=list(self.motion.targets), phase=self.motion.phase)


class SimulatedHand(Provider):
    """A simulated six-axis hand (detail ``hand.state/1``).

    Six axes as fractions of their range (``0.0`` extended); the openness is
    one minus the mean flexion. Its detail is a ``HandState``, the schema
    d1-firmware publishes for its own hands as ``hand.state/1``; a client
    that reads that schema reads this hand the same way.
    """

    #: The schema id of this provider's detail.
    DETAIL_SCHEMA = "hand.state/1"

    AXES = ("thumb_swing", "thumb", "index", "middle", "ring", "little")

    def __init__(self, model: str = "sim/six-axis", side: str = "b") -> None:
        self.model = model
        self.side = side
        self.motion = _Motion(len(self.AXES), 4.0, start=0.0)
        self.enabled = False

    def descriptor(self) -> c.DescriptorSpec:
        return c.DescriptorSpec(
            driver="sim-hand", detail_schema=self.DETAIL_SCHEMA, model=self.model,
            joints=[c.JointSpec(name, "fraction", 0.0, 1.0) for name in self.AXES],
            capabilities=c.Capabilities(openness=True, joints=True, max_command_hz=100))

    def set(self, command: c.SetCommand) -> None:
        flexion = 1.0 - command.openness
        self.enabled = True
        with self.motion.lock:
            self.motion.targets = [0.0] + [flexion] * (len(self.AXES) - 1)

    def joints(self, targets: List[float], timestamp_us: int) -> None:
        self.enabled = True
        with self.motion.lock:
            self.motion.targets = list(targets)

    def read(self) -> c.StateSample:
        with self.motion.lock:
            positions = list(self.motion.positions)
        openness = 1.0 - sum(positions[1:]) / (len(positions) - 1)
        return c.StateSample(
            openness=max(0.0, min(1.0, openness)), joints=positions, live=True,
            detail={"schema": self.DETAIL_SCHEMA, "data": {
                "model": self.model, "side": self.side, "positions": positions,
                "positions_wire": [round(p * 10_000) for p in positions],
                "enabled": [self.enabled] * len(positions), "all_enabled": self.enabled,
                "error_code": 0, "faults": [], "live": True, "features": []}})

    def stop(self) -> None:
        self.enabled = False
        with self.motion.lock:
            self.motion.targets = list(self.motion.positions)


# --------------------------------------------------------------------------
# The command
# --------------------------------------------------------------------------


def main(argv: Optional[List[str]] = None) -> int:
    """``mkit-ee-provider``: serve a simulated end effector."""
    parser = argparse.ArgumentParser(
        prog="mkit-ee-provider",
        description=f"Serve a simulated end effector to d1-firmwared ({c.CONTRACT}).")
    parser.add_argument("--model", choices=("gripper", "hand"), default="gripper")
    parser.add_argument("--service", default="omakase/ee/b",
                        help="the side's base service name (the daemon's shm_service)")
    parser.add_argument("--root", default=c.DEFAULT_ROOT_PATH,
                        help="the shared-memory root (the daemon's shm_root_path)")
    parser.add_argument("--prefix", default=c.DEFAULT_PREFIX,
                        help="the shared-memory prefix (the daemon's shm_prefix)")
    parser.add_argument("--object-at", type=float, default=None,
                        help="gripper: a simulated object stops a close at this jaw fraction")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    provider: Provider = (SimulatedGripper(object_at=args.object_at) if args.model == "gripper"
                          else SimulatedHand())
    stop = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stop.set())
    serve(provider, ServeOptions(service=args.service, root_path=args.root,
                                 prefix=args.prefix), stop)
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
