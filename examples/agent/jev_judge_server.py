"""Jev-Omni behind HTTP, for a robot that cannot hold it: the 12B classifier
held 46.9 GB of GPU memory when measured and a D1's Jetson does not have it.
Run this on a workstation and point ``jev_servo.py --judge-url`` at it::

    python examples/agent/jev_judge_server.py --host 127.0.0.1 --port 8766
    # on the Tailscale address instead of loopback, so a robot can reach it:
    python examples/agent/jev_judge_server.py --host 100.x.y.z --port 8766

``POST /judge`` takes ``{"state", "question", "options": [...],
"image_jpeg_b64"}`` and answers ``{"probabilities": {option: p}, "ms"}``;
``POST /judge_batch`` takes ``{"state", "image_jpeg_b64", "items":
[{"question", "options"}, ...]}`` — several questions about ONE photo — and
answers ``{"answers": [{"probabilities", "ms"}, ...], "ms"}``: one upload and
one decode for all of them. Jev-Omni has a single classification head and
answers one question per forward pass, so the model time still grows with
the number of questions; what the batch saves is the request, the upload and
the queueing between them. ``GET /health`` answers ``{"ok": true, "loaded":
bool, "batch": true}``. The classifier is
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
            self._send(200, {"ok": True, "loaded": classifier.loaded,
                             "batch": True})

        def do_POST(self) -> None:  # noqa: N802
            if self.path not in ("/judge", "/judge_batch"):
                return self._send(404, {"error": "POST /judge or /judge_batch"})
            size = int(self.headers.get("Content-Length") or 0)
            if not 0 < size <= MAX_BODY_BYTES:
                return self._send(413, {"error": f"body of {size} bytes"})
            try:
                ask = json.loads(self.rfile.read(size))
                image = base64.b64decode(ask["image_jpeg_b64"])
                items = (ask["items"] if self.path == "/judge_batch"
                         else [ask])
                if not isinstance(items, list) or not 1 <= len(items) <= 64:
                    raise ValueError("1-64 items")
                items = [(str(item.get("question", "")),
                          [str(o) for o in item["options"]]) for item in items]
                if any(not 2 <= len(options) <= 256 for _q, options in items):
                    raise ValueError("2-256 options")
            except (KeyError, TypeError, ValueError, AttributeError) as exc:
                return self._send(400, {"error": f"bad request: {exc}"})
            answers, started = [], time.time()
            with tempfile.NamedTemporaryFile(suffix=".jpg") as photo:
                photo.write(image)
                photo.flush()
                with lock:
                    for question, options in items:
                        one = time.time()
                        result = classifier.predict(
                            state=str(ask.get("state", "")),
                            question=question, options=options,
                            media=photo.name, modality="image")
                        answers.append({"probabilities": {
                            str(k): float(v)
                            for k, v in result["probabilities"].items()},
                            "ms": round((time.time() - one) * 1000.0)})
            ms = round((time.time() - started) * 1000.0)
            if self.path == "/judge":
                return self._send(200, {**answers[0], "ms": ms})
            self._send(200, {"answers": answers, "ms": ms})

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
