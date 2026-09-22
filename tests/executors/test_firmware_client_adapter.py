"""The adapter talks to a daemon-shaped HTTP server, end to end.

``tests/executors/test_firmware.py`` drives the executor against an injected
fake, which is the right way to test judgement — but it means nothing in the
suite ever put bytes on a socket. This does: a loopback server that serves the
bundled OpenAPI document at ``/openapi.json`` and the daemon's own envelope
(``{"status": …, "data": …, "message": …}``) everywhere else, and a real
:class:`FirmwareClient` built against it — generated tree resolved, ``httpx``
session and all.

It needs the ``[firmware]`` extra (httpx) and Python 3.10+, and skips loudly
without them.
"""
from __future__ import annotations

import json
import math
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from manipulation_kit.executors.firmware import ensure
from manipulation_kit.executors.firmware.errors import (DeviceUnavailable,
                                                        FirmwareError,
                                                        ProtocolError)

pytest.importorskip("httpx", reason="the [firmware] extra is not installed")
pytestmark = pytest.mark.skipif(
    sys.version_info < ensure.MIN_PYTHON,
    reason="the generated client imports on Python 3.10+")

#: Shaped like d1-firmwared 0.3.0's answers (the document the bundled client
#: is generated from): every required field, plus the optional ones the daemon
#: fills on d1-2.
ARM_STATE = {
    "mode": "position", "error_code": 0,
    "feedback_joints": [0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "command_joints": [0.0] * 7,
    "feedback_velocity": [0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "feedback_torque": [0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0],
    "feedback_temperature": [30.0] * 7,
    "frame_serial": 12, "stationary": True,
    "frame_miss_count": 0, "max_frame_miss_count": 3,
    "sys_cycle_miss_count": 0, "protocol_mismatch": False,
    "controller_version": 100341,
}
GRIPPER_STATE = {
    "kind": "grasp", "jaw_rad": 0.9, "torque_nm": -0.35, "holding": True,
    "grip_preload_rad": 0.05, "live": False, "open_rad": 1.35, "coil_c": 31,
    "fault_code": None, "overload_released": False,
    "target_closedness": None, "tracking": False,
}
NECK_STATE = {"pitch": -0.4, "yaw": 0.1, "pitch_velocity": 0.0,
              "yaw_velocity": 0.0, "moving": False, "pitch_torque": 0.2,
              "yaw_torque": 0.0, "enabled": True}
SLIDER_STATE = {"alarm": False, "alarm_code": 0, "alarm_text": "",
                "comms_ok": True, "height_m": 0.205, "moving": False,
                "travel_max_m": 0.30, "zero_reference": "commissioned"}


class Daemon:
    """A d1-firmwared shaped server: the document, and enveloped `/v1`."""

    def __init__(self, routes):
        self.routes = routes
        self.seen = []
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def _answer(self, body: bytes, status: int = 200):
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):  # noqa: N802
                if self.path == ensure.SPEC_ROUTE:
                    self._answer(ensure.bundled_spec_bytes())
                    return
                outer.seen.append(("GET", self.path, None))
                self._reply()

            def do_POST(self):  # noqa: N802
                length = int(self.headers.get("Content-Length") or 0)
                raw = self.rfile.read(length) if length else b""
                outer.seen.append(("POST", self.path,
                                   json.loads(raw) if raw else None))
                self._reply()

            def _reply(self):
                handler = outer.routes.get(self.path)
                if handler is None:
                    self._answer(json.dumps(
                        {"status": "error", "data": None,
                         "message": "no such route"}).encode(), 404)
                    return
                status, payload = handler()
                self._answer(payload if isinstance(payload, bytes)
                             else json.dumps(payload).encode(), status)

            def log_message(self, *_args):
                pass

        self._server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self._thread = threading.Thread(target=self._server.serve_forever,
                                        daemon=True)

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *_exc):
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=5)

    @property
    def url(self) -> str:
        host, port = self._server.server_address[:2]
        return f"http://{host}:{port}"


def ok(data):
    return lambda: (200, {"status": "ok", "data": data, "message": None})


@pytest.fixture
def client_factory(tmp_path):
    from manipulation_kit.executors.firmware.client import FirmwareClient

    def make(daemon):
        return FirmwareClient(daemon.url, cache_dir=tmp_path)
    return make


def test_a_connect_resolves_the_client_and_then_reads_the_arms(client_factory):
    with Daemon({"/v1/arm/a/state": ok(ARM_STATE)}) as daemon:
        with client_factory(daemon) as client:
            assert client.spec_sha256 == ensure.bundled_spec_sha256()
            assert client.tree.source == "bundled"
            state = client.arm_state("a")
    # The GENERATED model, not a hand-written parse of the keys.
    assert type(state).__name__ == "ArmState"
    assert type(state).__module__.startswith(ensure.BUNDLED_MODULE)
    assert state.mode == "position" and state.frame_serial == 12
    assert state.controller_version == 100341


def test_the_arm_reading_becomes_a_joint_state_field_for_field(client_factory):
    from manipulation_kit.executors.firmware.client import joint_state

    with Daemon({"/v1/arm/a/state": ok(ARM_STATE)}) as daemon:
        with client_factory(daemon) as client:
            arm = joint_state(client.arm_state("a"))
    assert arm.mode == "position" and arm.error_code == 0 and arm.stationary
    assert arm.q[1] == pytest.approx(math.radians(10.0))
    assert arm.qd[1] == pytest.approx(math.radians(5.0))
    assert arm.torque_nm[1] == pytest.approx(1.5)


def test_the_gripper_reading_comes_back_typed(client_factory):
    from manipulation_kit.executors.firmware.client import hand_state
    from manipulation_kit.hands.d1.parallel_gripper.description import (
        gap_from_motor_rad)

    with Daemon({"/v1/gripper/b/state": ok(GRIPPER_STATE)}) as daemon:
        with client_factory(daemon) as client:
            report = client.gripper_state("b")
    assert type(report).__name__ == "GripperReport"
    assert report.holding is True and report.open_rad == 1.35
    assert report.kind == "grasp"
    hand = hand_state(report)
    assert hand.holding is True and hand.stalled is True and hand.fault is None
    assert hand.closedness == pytest.approx(1.0 - 0.9 / 1.35)
    # the daemon's open_rad, through the description's kinematic map:
    # d1-2's 1.35 rad is its measured 60.5 mm opening
    assert hand.open_gap_m == pytest.approx(gap_from_motor_rad(1.35))
    assert hand.open_gap_m == pytest.approx(0.0605, abs=0.0005)
    assert hand.jaw_gap_m == pytest.approx(gap_from_motor_rad(0.9))
    assert hand.torque_nm == pytest.approx(-0.35)


def test_a_field_the_document_leaves_out_is_none_not_a_number(client_factory):
    from manipulation_kit.executors.firmware.client import hand_state

    bare = {k: GRIPPER_STATE[k] for k in ("kind", "jaw_rad", "torque_nm",
                                          "holding", "grip_preload_rad")}
    with Daemon({"/v1/gripper/a/state": ok(bare)}) as daemon:
        with client_factory(daemon) as client:
            hand = hand_state(client.gripper_state("a"))
    assert hand.open_gap_m is None and hand.closedness is None
    assert hand.commanded is None and hand.fault is None


def test_a_faulted_gripper_reports_its_fault_code(client_factory):
    from manipulation_kit.executors.firmware.client import hand_state

    faulted = dict(GRIPPER_STATE, kind="fault", holding=False,
                   fault_code="coil_over_temperature")
    with Daemon({"/v1/gripper/a/state": ok(faulted)}) as daemon:
        with client_factory(daemon) as client:
            hand = hand_state(client.gripper_state("a"))
    assert hand.fault == "coil_over_temperature"


def test_the_neck_and_the_lift_are_read_through_generated_operations(
        client_factory):
    from manipulation_kit.executors.firmware.client import (lift_state,
                                                            neck_state)

    with Daemon({"/v1/neck/state": ok(NECK_STATE),
                 "/v1/slider/state": ok(SLIDER_STATE)}) as daemon:
        with client_factory(daemon) as client:
            neck = neck_state(client.neck_state())
            lift = lift_state(client.slider_state())
    assert neck.pitch_rad == pytest.approx(-0.4)   # the daemon's sign, unflipped
    assert neck.yaw_rad == pytest.approx(0.1) and neck.enabled and not neck.moving
    assert lift.height_m == pytest.approx(0.205) and lift.alarm is None


def test_the_blocking_stroke_is_bounded_by_the_documents_own_timeout(
        client_factory):
    with Daemon({}) as daemon:
        with client_factory(daemon) as client:
            documented = client.operation_timeout_s("POST",
                                                    "/v1/gripper/{side}/set")
    spec = json.loads(ensure.bundled_spec_bytes())
    assert documented == spec["paths"]["/v1/gripper/{side}/set"]["post"][
        "x-timeout-seconds"]
    assert documented == 40.0


def test_a_device_the_daemon_could_not_read_is_a_device_failure(client_factory):
    """A degraded slot is ``{"error": …}`` inside a 200, not a broken contract."""
    with Daemon({"/v1/gripper/a/state": ok({"error": "CAN bus timeout"})}) as daemon:
        with client_factory(daemon) as client:
            with pytest.raises(DeviceUnavailable, match="CAN bus timeout"):
                client.gripper_state("a")


def test_a_refusal_carries_the_daemons_own_words(client_factory):
    def refused():
        return 409, {"status": "error", "data": None,
                     "message": "lease held by somebody else"}

    with Daemon({"/v1/arm/a/mode": refused}) as daemon:
        with client_factory(daemon) as client:
            with pytest.raises(FirmwareError, match="lease held by somebody else"):
                client.request("POST", "/v1/arm/a/mode", {"mode": "position"})


def test_an_answer_that_is_not_the_advertised_shape_is_a_protocol_error(
        client_factory):
    with Daemon({"/v1/arm/a/state": lambda: (200, b"<html>nope</html>")}) as daemon:
        with client_factory(daemon) as client:
            with pytest.raises(ProtocolError, match="invalid JSON"):
                client.request("GET", "/v1/arm/a/state")


def test_setting_the_gripper_sends_the_preset_not_a_torque(client_factory):
    with Daemon({"/v1/gripper/a/set": ok(None)}) as daemon:
        with client_factory(daemon) as client:
            client.gripper_set("a", 1.0, grip="firm")
        assert daemon.seen[-1] == ("POST", "/v1/gripper/a/set",
                                   {"closedness": 1.0, "grip": "firm"})


@pytest.mark.parametrize("closedness,grip", [(1.5, None), (float("nan"), None),
                                             (0.5, "crushing")])
def test_an_impossible_grip_never_reaches_the_wire(client_factory, closedness,
                                                   grip):
    with Daemon({"/v1/gripper/a/set": ok(None)}) as daemon:
        with client_factory(daemon) as client:
            with pytest.raises(ValueError):
                client.gripper_set("a", closedness, grip=grip)
        assert daemon.seen == []


def test_the_daemons_spec_version_is_readable_without_the_envelope(client_factory):
    with Daemon({}) as daemon:
        with client_factory(daemon) as client:
            assert client.daemon_spec_version() == ensure.snapshot()["spec_version"]


def test_a_base_url_with_credentials_is_refused():
    from manipulation_kit.executors.firmware.client import FirmwareClient

    with pytest.raises(ValueError, match="without credentials"):
        FirmwareClient("http://user:secret@d1-2:4750")


def test_the_generated_operations_are_reachable_from_the_adapter(client_factory):
    """The four verbs are a convenience, not a ceiling."""
    with Daemon({"/v1/arm/a/state": ok(ARM_STATE)}) as daemon:
        with client_factory(daemon) as client:
            arm = client.api_module("arm.arm_state")
            response = arm.sync_detailed(side="a", client=client.api_client)
    assert response.status_code == 200
