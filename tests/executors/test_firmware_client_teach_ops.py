"""The verbs ``manipulation_kit.teach`` adds to FirmwareClient, on the wire.

A loopback daemon serving the BUNDLED document (0.3.0 with d1-firmware PRs
#92, #102 and #106, spec ``3b354c25…``): ``arm_mode``, ``arm_recover`` and the holding-brake
routes go out as the generated ``ArmModeCommand`` / ``ArmRecoverRequest`` /
``ArmBrakeReleaseBody`` bodies; an operation a document does not publish is
refused as :class:`OperationUnavailable` with no request sent, never
hand-built.
"""
from __future__ import annotations

import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from manipulation_kit.executors.firmware import OperationUnavailable, ensure

pytest.importorskip("httpx", reason="the [firmware] extra is not installed")
pytestmark = pytest.mark.skipif(sys.version_info < ensure.MIN_PYTHON,
                                reason="the generated client imports on Python 3.10+")


class _Daemon:
    def __init__(self, data, delay=None):
        self.seen = []
        #: path -> seconds the daemon works before it answers
        self.delay = dict(delay or {})
        #: path -> True once the answer was written to an open connection,
        #: False when the client had hung up (the daemon cancels then)
        self.answered = {}
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
                n = int(self.headers.get("Content-Length") or 0)
                raw = self.rfile.read(n) if n else b""
                outer.seen.append(("POST", self.path, json.loads(raw) if raw else None))
                self._reply()

            def _reply(self):
                if self.path in outer.delay:
                    time.sleep(outer.delay[self.path])
                try:
                    self._answer(json.dumps({"status": "ok",
                                             "data": data.get(self.path),
                                             "message": None}).encode())
                    self.wfile.flush()
                    outer.answered[self.path] = True
                except OSError:
                    outer.answered[self.path] = False

            def log_message(self, *_a):
                pass

        self._server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)

    def __enter__(self):
        self._thread.start()
        host, port = self._server.server_address[:2]
        self.url = f"http://{host}:{port}"
        return self

    def __exit__(self, *_exc):
        self._server.shutdown()
        self._server.server_close()


@pytest.fixture
def connect(tmp_path):
    from manipulation_kit.executors.firmware.client import FirmwareClient
    return lambda d: FirmwareClient(d.url, cache_dir=tmp_path)


def test_force_compliance_goes_out_as_the_generated_body(connect):
    with _Daemon({}) as daemon, connect(daemon) as client:
        client.arm_mode("a", "force_compliance", holder="me",
                        force_direction=[1.0, 0, 0, 0, 0, 0], adjustment_limit_mm=2.0,
                        target_force=0.0, force_type=0, anchor_command_pose=True,
                        vel_ratio=0.05, acc_ratio=0.05)
    method, path, body = daemon.seen[-1]
    assert (method, path) == ("POST", "/v1/arm/a/mode")
    assert body == {"mode": "force_compliance", "holder": "me",
                    "force_direction": [1.0, 0, 0, 0, 0, 0],
                    "adjustment_limit_mm": 2.0, "target_force": 0.0, "force_type": 0,
                    "anchor_command_pose": True, "vel_ratio": 0.05, "acc_ratio": 0.05}


def test_an_unknown_mode_or_field_is_refused_before_the_wire(connect):
    with _Daemon({}) as daemon, connect(daemon) as client:
        with pytest.raises(ValueError):
            client.arm_mode("a", "limp")
        with pytest.raises(TypeError):
            client.arm_mode("a", "idle", stiffnesss=[1] * 7)
    assert not [s for s in daemon.seen if s[0] == "POST"]


def test_recover_sends_the_hold_ratios(connect):
    report = {"side": "b", "mode": "position", "error_code": 0,
              "cleared_error": False, "anchored": True}
    with _Daemon({"/v1/arm/b/recover": report}) as daemon, connect(daemon) as client:
        client.arm_recover("b", vel_ratio=0.05, acc_ratio=0.05)
    assert daemon.seen[-1] == ("POST", "/v1/arm/b/recover",
                               {"vel_ratio": 0.05, "acc_ratio": 0.05})


def test_brake_release_goes_out_as_the_generated_body(connect):
    report = {"side": "a", "released": True, "window_s": 20.0, "remaining_s": 20.0,
              "holder": "me", "last_engage_reason": None}
    with _Daemon({"/v1/arm/a/brake_release": report}) as daemon, \
            connect(daemon) as client:
        got = client.brake_release("a", seconds=20, holder="me")
    assert daemon.seen[-1] == ("POST", "/v1/arm/a/brake_release",
                               {"confirm": "RELEASE_BRAKE", "seconds": 20.0,
                                "holder": "me"})
    assert got.released is True and got.window_s == 20.0


def test_brake_engage_and_state_are_generated_operations(connect):
    engaged = {"side": "b", "released": False, "window_s": None, "remaining_s": None,
               "holder": None, "last_engage_reason": "operator"}
    with _Daemon({"/v1/arm/b/brake_engage": engaged, "/v1/arm/b/brake": engaged}) \
            as daemon, connect(daemon) as client:
        assert client.brake_engage("b").released is False
        state = client.brake_state("b")
    assert daemon.seen[-2] == ("POST", "/v1/arm/b/brake_engage", None)
    assert daemon.seen[-1] == ("GET", "/v1/arm/b/brake", None)
    assert state.last_engage_reason.value == "operator"


def test_brake_release_refuses_a_window_outside_the_documents_bounds(connect):
    with _Daemon({}) as daemon, connect(daemon) as client:
        for seconds in (0.5, 121, float("nan")):
            with pytest.raises(ValueError):
                client.brake_release("a", seconds=seconds)
    assert not [s for s in daemon.seen if "brake" in s[1]]


def test_an_operation_the_document_lacks_is_refused_before_the_wire(connect):
    with _Daemon({}) as daemon, connect(daemon) as client:
        with pytest.raises(OperationUnavailable, match="not in the OpenAPI"):
            client.operation("arm.arm_no_such_route", "POST /v1/arm/{side}/nope")
    assert not [s for s in daemon.seen if s[0] == "POST"]


def test_every_request_waits_the_documents_bound_for_its_route(connect):
    spec = json.loads(ensure.bundled_spec_bytes())["paths"]
    with _Daemon({}) as daemon, connect(daemon) as client:
        assert client.timeout == 2.0
        for route, concrete in (("/v1/arm/{side}/recover", "/v1/arm/b/recover"),
                                ("/v1/arm/{side}/mode", "/v1/arm/a/mode"),
                                ("/v1/arm/{side}/brake_release", "/v1/arm/a/brake_release"),
                                ("/v1/arm/{side}/brake_engage", "/v1/arm/b/brake_engage"),
                                ("/v1/arm/move_joints_both", "/v1/arm/move_joints_both"),
                                ("/v1/gripper/{side}/set", "/v1/gripper/a/set")):
            documented = spec[route]["post"]["x-timeout-seconds"]
            assert client.timeout_for("POST", concrete) == max(2.0, documented), route
        assert client.timeout_for("POST", "/v1/arm/b/recover") == 65.0
        # a bound SHORTER than the client's floor never shortens it
        assert spec["/v1/arm/lease"]["post"]["x-timeout-seconds"] < 2.0
        assert client.timeout_for("POST", "/v1/arm/lease") == 2.0
        # an undocumented route gets the floor
        assert client.timeout_for("GET", "/v1/arm/a/state") == 2.0
        assert client.timeout_for("POST", "/v1/arm/trajectory/start") == 2.0


def test_a_recover_that_takes_three_seconds_is_waited_for_not_cancelled(connect):
    """One request, answered on an open connection, decoded — not cut at
    2 s (which the daemon logs as phase=cancelled)."""
    report = {"side": "b", "recovered": True, "elapsed_ms": 3000,
              "initial_state": None, "state": None}
    with _Daemon({"/v1/arm/b/recover": report},
                 delay={"/v1/arm/b/recover": 3.0}) as daemon, connect(daemon) as client:
        started = time.monotonic()
        client.arm_recover("b", vel_ratio=0.05, acc_ratio=0.05)
        took = time.monotonic() - started
    assert took >= 3.0
    assert [s for s in daemon.seen if s[1] == "/v1/arm/b/recover"] == [
        ("POST", "/v1/arm/b/recover", {"vel_ratio": 0.05, "acc_ratio": 0.05})]
    assert daemon.answered["/v1/arm/b/recover"] is True


def test_running_out_of_the_bound_is_a_client_timeout_not_a_refusal(tmp_path):
    from manipulation_kit.executors.firmware import ClientTimeout, FirmwareError
    from manipulation_kit.executors.firmware.client import FirmwareClient
    with _Daemon({}, delay={"/v1/arm/a/state": 1.0}) as daemon:
        with FirmwareClient(daemon.url, cache_dir=tmp_path, timeout=0.2) as client:
            with pytest.raises(ClientTimeout, match="gave up waiting") as caught:
                client.request("GET", "/v1/arm/a/state")
    assert not isinstance(caught.value, FirmwareError)
    assert caught.value.timeout_s == pytest.approx(0.2)
