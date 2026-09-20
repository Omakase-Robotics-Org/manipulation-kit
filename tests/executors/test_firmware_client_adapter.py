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

ARM_STATE = {
    "mode": "position", "error_code": 0,
    "feedback_joints": [0.0] * 7, "command_joints": [0.0] * 7,
    "feedback_velocity": [0.0] * 7, "feedback_torque": [0.0] * 7,
    "feedback_temperature": [30.0] * 7,
    "frame_serial": 12, "stationary": True,
}
GRIPPER_STATE = {
    "kind": "parallel", "jaw_rad": 0.1, "torque_nm": 0.2, "holding": False,
    "grip_preload_rad": 0.0, "live": True, "open_rad": 0.8, "coil_c": 31,
}


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
    assert state.mode == "position"
    assert state.feedback_joints == (0.0,) * 7
    assert state.stationary is True and state.frame_serial == 12


def test_the_gripper_reading_comes_back_typed(client_factory):
    with Daemon({"/v1/gripper/b/state": ok(GRIPPER_STATE)}) as daemon:
        with client_factory(daemon) as client:
            report = client.gripper_state("b")
    assert report.holding is False and report.open_rad == 0.8
    assert report.kind == "parallel"


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
