"""The reference provider: its rules as functions, then the runtime over
real iceoryx2 shared memory against a raw client of the contract (skipped
without the ``[shm]`` extra), then — with ``D1_FIRMWARE_DIR`` naming a
d1-firmware checkout — the daemon's own Rust driver against this provider.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

import pytest

from manipulation_kit.end_effectors import shm_contract as c
from manipulation_kit.end_effectors.shm_provider import (
    ServeOptions,
    SimulatedGripper,
    SimulatedHand,
    StopGenerations,
    check_set,
    joints_usable,
    serve,
    triage_command,
)

GRIPPER = SimulatedGripper().descriptor()


def _envelope(**overrides) -> c.CommandEnvelope:
    values = dict(target_instance_id=7, descriptor_generation=1,
                  start_by_mono_ns=c.monotonic_ns() + 10**9,
                  command=c.SetCommand(openness=0.5))
    values.update(overrides)
    return c.CommandEnvelope(**values)


def test_a_set_runs_only_for_this_instance_generation_and_deadline():
    now = c.monotonic_ns()
    assert triage_command(_envelope(), 7, 1, GRIPPER, now) is None
    for envelope in (_envelope(target_instance_id=8), _envelope(descriptor_generation=2),
                     _envelope(start_by_mono_ns=now - 1)):
        assert triage_command(envelope, 7, 1, GRIPPER, now).kind == "stale_target"
    stop = _envelope(target_instance_id=1, descriptor_generation=9, start_by_mono_ns=0,
                     command=None)
    assert triage_command(stop, 7, 1, GRIPPER, now) is None, "a stop always runs"


def test_undeclared_set_parameters_are_refused():
    assert check_set(GRIPPER, c.SetCommand(0.5, grip="strong")) is None
    assert check_set(GRIPPER, c.SetCommand(0.5, extra={"i_des": 0.1})).kind == "unprocessable"
    assert check_set(GRIPPER, c.SetCommand(0.5, torque_nm=1.0)).kind == "unprocessable"
    assert check_set(SimulatedHand().descriptor(),
                     c.SetCommand(0.5, grip="soft")).kind == "unprocessable"


def test_joint_targets_must_be_fresh_addressed_and_in_limits():
    now = c.monotonic_ns()

    def targets(**overrides):
        values = dict(target_instance_id=7, descriptor_generation=1, issued_mono_ns=now,
                      timestamp_us=1, targets=[0.5])
        values.update(overrides)
        return c.JointTargets(**values)

    assert joints_usable(targets(), 7, 1, GRIPPER, now)
    assert not joints_usable(targets(target_instance_id=8), 7, 1, GRIPPER, now)
    assert not joints_usable(targets(targets=[1.5]), 7, 1, GRIPPER, now)
    old = now - (c.JOINTS_MAX_AGE_MS + 1) * 1_000_000
    assert not joints_usable(targets(issued_mono_ns=old), 7, 1, GRIPPER, now)


def test_the_first_stop_generation_of_a_daemon_is_its_baseline():
    stops = StopGenerations()
    assert not stops.observe(daemon_instance=1, generation=5)
    assert not stops.observe(1, 5)
    assert stops.observe(1, 6)
    assert not stops.observe(2, 9), "a restarted daemon's generation is a new baseline"
    assert stops.observe(2, 10)


# --------------------------------------------------------------------------
# Over shared memory
# --------------------------------------------------------------------------

iox2 = pytest.importorskip("iceoryx2", reason="needs the [shm] extra (iceoryx2 0.9.3)")


class _Root:
    """A root directory and prefix of the test's own; removed afterwards
    with the segments iceoryx2 left under the prefix."""

    def __init__(self) -> None:
        self.path = tempfile.mkdtemp(prefix="mk-ee-shm-") + "/"
        self.prefix = f"mk{os.getpid()}x{time.monotonic_ns() % 10**9}_"

    def remove(self) -> None:
        shutil.rmtree(self.path, ignore_errors=True)
        for name in os.listdir("/dev/shm"):
            if name.startswith(self.prefix):
                try:
                    os.remove(os.path.join("/dev/shm", name))
                except OSError:
                    pass


@pytest.fixture
def root():
    value = _Root()
    yield value
    value.remove()


class _RawDaemon:
    """The daemon's side of the contract, written against the same ctypes
    messages: a client of the provider's services."""

    def __init__(self, root: _Root, service: str) -> None:
        config = iox2.config.default()
        config.global_cfg.root_path = iox2.Path.new(root.path)
        config.global_cfg.prefix = iox2.FileName.new(root.prefix)
        self.node = (iox2.NodeBuilder.new().config(config).name(iox2.NodeName.new("raw-daemon"))
                     .create(iox2.ServiceType.Ipc))
        layout = {s["suffix"]: s for s in c.load_layout()["services"]}

        def topic(suffix, payload):
            spec = layout[suffix]
            return (self.node.service_builder(iox2.ServiceName.new(f"{service}/{suffix}"))
                    .publish_subscribe(payload).history_size(spec["history_size"])
                    .subscriber_max_buffer_size(spec["subscriber_max_buffer_size"])
                    .enable_safe_overflow(spec["enable_safe_overflow"])
                    .max_publishers(spec["max_publishers"])
                    .max_subscribers(spec["max_subscribers"]).open_or_create())

        def rpc(suffix, request, response):
            spec = layout[suffix]
            return (self.node.service_builder(iox2.ServiceName.new(f"{service}/{suffix}"))
                    .request_response(request, response).max_servers(spec["max_servers"])
                    .max_clients(spec["max_clients"])
                    .max_active_requests_per_client(spec["max_active_requests_per_client"])
                    .max_response_buffer_size(spec["max_response_buffer_size"])
                    .enable_safe_overflow_for_requests(False)
                    .enable_safe_overflow_for_responses(False).open_or_create())

        self.descriptor = topic(c.DESCRIPTOR, c.Descriptor).subscriber_builder().create()
        self.state = topic(c.STATE, c.State).subscriber_builder().create()
        self.joints = topic(c.JOINTS, c.JointsCommand).publisher_builder().create()
        self.control = topic(c.CONTROL, c.Control).publisher_builder().create()
        self.command = rpc(c.COMMAND, c.CommandRequest, c.CommandResponse) \
            .client_builder().create()
        self.fault = rpc(c.FAULT, c.FaultRequest, c.FaultResponse).client_builder().create()
        self.instance = c.new_instance_id()
        self.seq = 0

    def header(self) -> c.Header:
        self.seq += 1
        return c.header(self.instance, self.seq)

    def target(self):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            sample = self.descriptor.receive()
            if sample is not None:
                message = sample.payload().contents
                result = (message.header.instance_id, message.generation,
                          c.get_str(message.model, "model"))
                sample.delete()
                return result
            time.sleep(0.005)
        raise AssertionError("no descriptor")

    def request(self, client, message, response_type):
        pending = client.send_copy(message)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            response = pending.receive()
            if response is not None:
                value = response_type.from_buffer_copy(response.payload().contents)
                response.delete()
                return value
            time.sleep(0.002)
        raise AssertionError("no answer")

    def set(self, instance, generation, openness, start_by=None, wait=True):
        message = c.CommandRequest()
        message.header = self.header()
        message.target_instance_id = instance
        message.descriptor_generation = generation
        message.start_by_mono_ns = (c.monotonic_ns() + 250_000_000 if start_by is None
                                    else start_by)
        message.op = c.OP_SET
        message.flags = c.SET_WAIT if wait else 0
        message.openness = openness
        return self.request(self.command, message, c.CommandResponse)

    def send_control(self, stop_generation):
        self.control.send_copy(c.Control(header=self.header(),
                                         stop_generation=stop_generation, period_ms=100))

    def close(self):
        for port in (self.descriptor, self.state, self.joints, self.control, self.command,
                     self.fault):
            port.delete()


class _Serving:
    def __init__(self, provider, root, service="omakase/ee/b"):
        self.stop = threading.Event()
        options = ServeOptions(service=service, root_path=root.path, prefix=root.prefix)
        self.thread = threading.Thread(target=serve, args=(provider, options, self.stop),
                                       kwargs={"retry_s": None}, daemon=True)
        self.thread.start()

    def close(self):
        self.stop.set()
        self.thread.join(5)


def test_the_provider_serves_a_raw_client_of_the_contract(root):
    provider = SimulatedGripper(speed=20.0)
    stops = []
    original_stop = provider.stop
    provider.stop = lambda: (stops.append(time.monotonic()), original_stop())[1]
    serving = _Serving(provider, root)
    daemon = _RawDaemon(root, "omakase/ee/b")
    try:
        instance, generation, model = daemon.target()
        assert model == "sim/two-finger"

        response = daemon.set(instance, generation, 0.2)
        assert response.status == c.STATUS_OK, c.get_str(response.message, "m")
        assert abs(provider.motion.positions[0] - 0.2) < 1e-6

        for args in ((instance + 1, generation, 0.5), (instance, generation + 1, 0.5)):
            assert daemon.set(*args).status == c.STATUS_STALE_TARGET
        late = daemon.set(instance, generation, 0.5, start_by=c.monotonic_ns() - 1)
        assert late.status == c.STATUS_STALE_TARGET
        assert abs(provider.motion.positions[0] - 0.2) < 1e-6, "nothing ran late"

        # The first control word is the baseline; a rise is a stop.
        daemon.send_control(4)
        time.sleep(0.1)
        assert stops == []
        daemon.send_control(5)
        deadline = time.monotonic() + 2
        while not stops and time.monotonic() < deadline:
            time.sleep(0.01)
        assert len(stops) == 1

        message = c.JointsCommand(header=daemon.header(), target_instance_id=instance,
                                  descriptor_generation=generation, timestamp_us=1,
                                  joint_count=1)
        message.targets[0] = 0.7
        daemon.joints.send_copy(message)
        deadline = time.monotonic() + 2
        while abs(provider.motion.positions[0] - 0.7) > 1e-6 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert abs(provider.motion.positions[0] - 0.7) < 1e-6

        request = c.FaultRequest(header=daemon.header(), target_instance_id=instance,
                                 op=c.OP_REGISTERS)
        response = daemon.request(daemon.fault, request, c.FaultResponse)
        assert response.status == c.STATUS_OK
        body = c.get_json(response.body, response.body_len, "body")
        assert [entry["id"] for entry in body["entries"]] == ["position", "kind"]
    finally:
        daemon.close()
        serving.close()


def test_a_second_provider_cannot_bind_a_side_that_has_one(root):
    serving = _Serving(SimulatedGripper(), root)
    try:
        time.sleep(0.3)
        from manipulation_kit.end_effectors.shm_provider import ShmProvider
        second = ShmProvider(SimulatedHand(), ServeOptions(
            service="omakase/ee/b", root_path=root.path, prefix=root.prefix))
        with pytest.raises(RuntimeError, match="another provider"):
            second.open()
        second.close()
    finally:
        serving.close()


def test_the_command_serves_a_simulated_hand_until_it_is_terminated(root):
    process = subprocess.Popen(
        [sys.executable, "-m", "manipulation_kit.end_effectors.shm_provider", "--model", "hand",
         "--service", "vendor/ee/left", "--root", root.path, "--prefix", root.prefix],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    daemon = None
    try:
        deadline = time.monotonic() + 10
        while daemon is None:
            try:
                daemon = _RawDaemon(root, "vendor/ee/left")
            except Exception:  # noqa: BLE001 - the provider may not have created it yet
                assert time.monotonic() < deadline, "the provider never served"
                time.sleep(0.05)
        instance, generation, model = daemon.target()
        assert model == "sim/six-axis"
        assert daemon.set(instance, generation, 0.0).status == c.STATUS_OK
    finally:
        if daemon is not None:
            daemon.close()
        process.terminate()
        output, _ = process.communicate(timeout=10)
    assert process.returncode == 0, output
    assert "provider serving vendor/ee/left" in output


# --------------------------------------------------------------------------
# Against the daemon's driver
# --------------------------------------------------------------------------


@pytest.mark.skipif(not os.environ.get("D1_FIRMWARE_DIR") or shutil.which("cargo") is None,
                    reason="needs D1_FIRMWARE_DIR (a d1-firmware checkout) and cargo")
@pytest.mark.parametrize("model", ["gripper", "hand"])
def test_the_daemons_driver_drives_this_provider(model):
    """d1-firmware's ``an_external_provider_command`` test, run against
    ``python -m manipulation_kit.end_effectors.shm_provider``: the daemon's
    own driver fits this provider, drives every verb it declares, stops it,
    and sees it go when it is killed."""
    command = f"{sys.executable} -m manipulation_kit.end_effectors.shm_provider --model {model}"
    result = subprocess.run(
        ["cargo", "test", "--locked", "-p", "d1fw-ee-shm", "--test", "shm_driver",
         "an_external_provider_command", "--", "--ignored", "--exact"],
        cwd=Path(os.environ["D1_FIRMWARE_DIR"]),
        env={**os.environ, "D1FW_EE_SHM_PROVIDER_CMD": command},
        capture_output=True, text=True, timeout=900)
    assert result.returncode == 0, result.stdout[-4000:] + result.stderr[-4000:]
    assert "1 passed" in result.stdout, result.stdout[-2000:]
