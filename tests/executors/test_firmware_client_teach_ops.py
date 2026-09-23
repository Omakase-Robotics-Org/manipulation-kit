"""The verbs ``manipulation_kit.teach`` adds to FirmwareClient, on the wire.

A loopback daemon serving the BUNDLED document (0.3.0): ``arm_mode`` and
``arm_recover`` go out as the generated ``ArmModeCommand`` /
``ArmRecoverRequest`` bodies, and the holding-brake routes — which that
document does not publish (they arrived in d1-firmware PR #92) — are refused
as :class:`OperationUnavailable` with no request sent, never hand-built.
"""
from __future__ import annotations

import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from manipulation_kit.executors.firmware import OperationUnavailable, ensure

pytest.importorskip("httpx", reason="the [firmware] extra is not installed")
pytestmark = pytest.mark.skipif(sys.version_info < ensure.MIN_PYTHON,
                                reason="the generated client imports on Python 3.10+")


class _Daemon:
    def __init__(self, data):
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
                n = int(self.headers.get("Content-Length") or 0)
                raw = self.rfile.read(n) if n else b""
                outer.seen.append(("POST", self.path, json.loads(raw) if raw else None))
                self._reply()

            def _reply(self):
                self._answer(json.dumps({"status": "ok", "data": data.get(self.path),
                                         "message": None}).encode())

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


def test_the_brake_routes_are_refused_when_the_document_lacks_them(connect):
    with _Daemon({}) as daemon, connect(daemon) as client:
        for call in (lambda: client.brake_release("a", seconds=20),
                     lambda: client.brake_engage("a"),
                     lambda: client.brake_state("a")):
            with pytest.raises(OperationUnavailable, match="brake"):
                call()
    assert not [s for s in daemon.seen if "brake" in s[1]]
