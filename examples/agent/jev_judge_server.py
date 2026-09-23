"""Jev-Omni behind HTTP, for a robot that cannot hold it: the 12B classifier
held 46.9 GB of GPU memory when measured and a D1's Jetson does not have it.
Run this on a workstation and point ``jev_servo.py --judge-url`` at it::

    python examples/agent/jev_judge_server.py --host 127.0.0.1 --port 8766
    # on the Tailscale address instead of loopback, so a robot can reach it:
    python examples/agent/jev_judge_server.py --host 100.x.y.z --port 8766

``POST /judge`` takes ``{"state", "question", "options": [...],
"image_jpeg_b64"}`` and answers ``{"probabilities": {option: p}, "ms"}``;
``GET /health`` answers ``{"ok": true, "loaded": bool}``. The classifier is
loaded once, at start (``--lazy``: on the first request), and judgements are
serialised: one GPU, one model. Measured on an RTX PRO 6000 (6000-us,
2026-09-23, three marked mirror frames): 700 ms for the first judgement of a
process, 82-139 ms warm at the server, +4-6 ms round trip through an ssh
tunnel for one ~9 kB JPEG (the PR's in-process runs: 757-833 / 160-194 ms).
Needs Jev's environment only (torch, torchvision, transformers,
huggingface_hub), not this kit. No authentication: bind to loopback or to a
private (Tailscale) address, never to a public interface.
"""

from __future__ import annotations

import argparse
import base64
import json
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Optional, Sequence

#: a marked 640x480 JPEG is ~50 kB; refuse anything absurd
MAX_BODY_BYTES = 8 * 1024 * 1024


def make_server(classifier: Any, host: str = "127.0.0.1", port: int = 8766
                ) -> ThreadingHTTPServer:
    """An HTTP server answering ``/judge`` with ``classifier.predict(state=,
    question=, options=, media=, modality="image")`` — Jev-Omni, or a fake
    with the same method (the tests)."""
    lock = threading.Lock()

    class Handler(BaseHTTPRequestHandler):
        def _send(self, code: int, body: Any) -> None:
            data = json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self) -> None:  # noqa: N802
            if self.path != "/health":
                return self._send(404, {"error": "GET /health or POST /judge"})
            self._send(200, {"ok": True, "loaded": classifier.loaded})

        def do_POST(self) -> None:  # noqa: N802
            if self.path != "/judge":
                return self._send(404, {"error": "POST /judge"})
            size = int(self.headers.get("Content-Length") or 0)
            if not 0 < size <= MAX_BODY_BYTES:
                return self._send(413, {"error": f"body of {size} bytes"})
            try:
                ask = json.loads(self.rfile.read(size))
                image = base64.b64decode(ask["image_jpeg_b64"])
                options = [str(o) for o in ask["options"]]
                if not 2 <= len(options) <= 256:
                    raise ValueError("2-256 options")
            except (KeyError, TypeError, ValueError) as exc:
                return self._send(400, {"error": f"bad request: {exc}"})
            with tempfile.NamedTemporaryFile(suffix=".jpg") as photo:
                photo.write(image)
                photo.flush()
                started = time.time()
                with lock:
                    result = classifier.predict(
                        state=str(ask.get("state", "")),
                        question=str(ask.get("question", "")),
                        options=options, media=photo.name, modality="image")
            self._send(200, {"probabilities": {
                str(k): float(v) for k, v in result["probabilities"].items()},
                "ms": round((time.time() - started) * 1000.0)})

        def log_message(self, fmt: str, *args: Any) -> None:
            print(f"[jev_judge_server] {self.address_string()} {fmt % args}")

    return ThreadingHTTPServer((host, int(port)), Handler)


class Lazy:
    """Jev-Omni, loaded now or on the first ``predict``."""

    def __init__(self, eager: bool = True):
        self._jev = None
        if eager:
            self._load()

    @property
    def loaded(self) -> bool:
        return self._jev is not None

    def _load(self):
        # standalone on purpose: the server host needs Jev's own environment
        # (torch, transformers), not manipulation_kit
        if self._jev is None:
            import sys  # noqa: PLC0415
            from huggingface_hub import snapshot_download  # noqa: PLC0415
            sys.path.insert(0, snapshot_download("akhilaaa3/Jev-Omni"))
            from jev_omni import load_jev_omni  # noqa: PLC0415
            self._jev = load_jev_omni()
        return self._jev

    def predict(self, **kwargs: Any):
        return self._load().predict(**kwargs)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1",
                        help="bind address: loopback (default) or a private "
                             "Tailscale address; there is no authentication")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--lazy", action="store_true",
                        help="load the classifier on the first request")
    args = parser.parse_args(argv)
    server = make_server(Lazy(eager=not args.lazy), args.host, args.port)
    print(f"[jev_judge_server] listening on http://{args.host}:{args.port}",
          flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
