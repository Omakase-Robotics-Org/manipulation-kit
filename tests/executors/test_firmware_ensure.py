"""The first connect decides which client to use, against a real HTTP server.

CI reaches no robot, so the daemon here is an actual ``http.server`` on a
loopback port that serves an OpenAPI document — the bundled one when the test
wants agreement, a mutated one when it wants drift. That is enough to exercise
the whole decision, because the decision is made from the document's bytes and
nothing else.

The generator is the other thing CI does not have. Where a test needs one it
passes ``runner=`` — a stub that writes a two-file package where
``openapi-python-client`` would have written three hundred. What is under test
is the *control flow* (fetch → hash → compare → cache → generate → import),
not the generator, which is pinned and tested by its own project.

The three policies, in one sentence each:

* ``auto`` regenerates on drift, and warns and falls back when it cannot;
* ``strict`` refuses rather than fall back;
* ``bundled`` never even asks the daemon.
"""
from __future__ import annotations

import hashlib
import json
import logging
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from manipulation_kit.executors.firmware import _gen, ensure
from manipulation_kit.executors.firmware.errors import ClientUnavailable

needs_310 = pytest.mark.skipif(
    sys.version_info < ensure.MIN_PYTHON,
    reason="importing a generated client tree needs Python 3.10+")


# --------------------------------------------------------------------------- #
# a daemon that serves one document
# --------------------------------------------------------------------------- #
class FakeDaemon:
    """Serves ``body`` at ``/openapi.json`` and counts what was asked of it."""

    def __init__(self, body: bytes):
        self.body = body
        self.hits = []
        daemon = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):  # noqa: N802 - BaseHTTPRequestHandler's spelling
                daemon.hits.append(self.path)
                if self.path != ensure.SPEC_ROUTE:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(daemon.body)))
                self.end_headers()
                self.wfile.write(daemon.body)

            def log_message(self, *_args):  # keep pytest output clean
                pass

        self._server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self._thread = threading.Thread(target=self._server.serve_forever,
                                        daemon=True)

    def __enter__(self) -> "FakeDaemon":
        self._thread.start()
        return self

    def __exit__(self, *_exc) -> None:
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=5)

    @property
    def url(self) -> str:
        host, port = self._server.server_address[:2]
        return f"http://{host}:{port}"


@pytest.fixture
def bundled_bytes() -> bytes:
    return ensure.bundled_spec_bytes()


@pytest.fixture
def drifted_bytes(bundled_bytes) -> bytes:
    """The same document with one more route — a firmware that moved ahead."""
    document = json.loads(bundled_bytes)
    document["paths"]["/v1/invented/route"] = {
        "get": {"operationId": "invented", "responses": {"200": {
            "description": "a route this release has never seen"}}}}
    return json.dumps(document).encode()


@pytest.fixture
def stub_generator(tmp_path):
    """An argv prefix that writes a tiny package where the generator would."""
    stub = tmp_path / "stub_generator.py"
    stub.write_text(
        "import pathlib, sys\n"
        "pkg = pathlib.Path.cwd() / 'd1fw_api'\n"
        "pkg.mkdir(parents=True, exist_ok=True)\n"
        "(pkg / '__init__.py').write_text('from .client import Client\\n')\n"
        "(pkg / 'client.py').write_text('class Client:\\n"
        "    GENERATED_BY_STUB = True\\n')\n",
        encoding="utf-8")
    return [sys.executable, str(stub)]


def sha(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


# --------------------------------------------------------------------------- #
# agreement
# --------------------------------------------------------------------------- #
def test_a_matching_daemon_uses_the_bundled_client(bundled_bytes, tmp_path):
    with FakeDaemon(bundled_bytes) as daemon:
        resolution = ensure.resolve_client(daemon.url, cache_dir=tmp_path)
    assert resolution.source == "bundled"
    assert resolution.path == ensure.BUNDLED_PACKAGE
    assert resolution.spec_sha256 == sha(bundled_bytes)
    assert resolution.daemon_sha256 == resolution.spec_sha256
    assert resolution.note == ""
    assert daemon.hits == [ensure.SPEC_ROUTE], (
        "one GET, and nothing else, on a connect that agrees")


def test_the_logged_line_names_the_sha_the_source_and_the_path(bundled_bytes,
                                                               tmp_path, caplog):
    with FakeDaemon(bundled_bytes) as daemon, caplog.at_level(logging.INFO):
        tree_line = ensure.resolve_client(daemon.url, cache_dir=tmp_path).line()
    assert sha(bundled_bytes)[:8] in tree_line
    assert "source=bundled" in tree_line
    assert str(ensure.BUNDLED_PACKAGE) in tree_line


def test_policy_bundled_does_not_ask_the_daemon_at_all(drifted_bytes, tmp_path):
    """The escape hatch for an air-gapped box: no socket, no surprise."""
    with FakeDaemon(drifted_bytes) as daemon:
        resolution = ensure.resolve_client(daemon.url, cache_dir=tmp_path,
                                           policy="bundled")
    assert daemon.hits == []
    assert resolution.source == "bundled"
    assert "policy=bundled" in resolution.note


def test_an_unreachable_daemon_is_not_drift(tmp_path):
    """Nothing to compare against, so the bundled tree — and say so."""
    with FakeDaemon(b"{}") as daemon:
        dead = daemon.url
    resolution = ensure.resolve_client(dead, cache_dir=tmp_path, timeout_s=0.5)
    assert resolution.source == "bundled"
    assert "unreachable" in resolution.note
    assert resolution.daemon_sha256 is None


def test_a_daemon_that_serves_something_else_is_treated_as_unreachable(tmp_path):
    with FakeDaemon(b'{"not": "an openapi document"}') as daemon:
        resolution = ensure.resolve_client(daemon.url, cache_dir=tmp_path)
    assert resolution.source == "bundled"
    assert "not an OpenAPI document" in resolution.note


# --------------------------------------------------------------------------- #
# drift
# --------------------------------------------------------------------------- #
def test_drift_regenerates_into_the_cache(drifted_bytes, tmp_path,
                                          stub_generator):
    with FakeDaemon(drifted_bytes) as daemon:
        resolution = ensure.resolve_client(daemon.url, cache_dir=tmp_path,
                                           runner=stub_generator)
    expected = sha(drifted_bytes)
    assert resolution.source == "regenerated"
    assert resolution.spec_sha256 == expected
    assert resolution.path == tmp_path / expected / "d1fw_api"
    assert (resolution.path / "client.py").is_file()
    # The document that produced it is kept beside it, so the next process can
    # see what this tree was built from without asking the daemon again.
    assert (tmp_path / expected / "openapi.json").read_bytes() == drifted_bytes
    assert json.loads((tmp_path / expected / "SNAPSHOT.json").read_text()
                      )["spec_sha256"] == expected


def test_a_second_connect_reuses_the_cached_tree(drifted_bytes, tmp_path,
                                                 stub_generator):
    with FakeDaemon(drifted_bytes) as daemon:
        first = ensure.resolve_client(daemon.url, cache_dir=tmp_path,
                                      runner=stub_generator)
        # A runner that would fail if it ran: the cache has to answer this one.
        second = ensure.resolve_client(daemon.url, cache_dir=tmp_path,
                                       runner=[sys.executable, "-c", "raise SystemExit(1)"])
    assert first.source == "regenerated"
    assert second.source == "cache"
    assert second.path == first.path
    assert "differs from the bundled" in second.note


@needs_310
def test_the_regenerated_tree_is_what_gets_imported(drifted_bytes, tmp_path,
                                                    stub_generator, caplog):
    with FakeDaemon(drifted_bytes) as daemon, caplog.at_level(logging.INFO):
        tree = ensure.ensure_client(daemon.url, cache_dir=tmp_path,
                                    runner=stub_generator)
    assert tree.source == "regenerated"
    assert tree.module.Client.GENERATED_BY_STUB is True
    # Not `d1fw_api`: d1-inference generates a top-level package by that name
    # and a consumer may hold both in one interpreter.
    assert tree.module.__name__.startswith("_mkit_d1fw_api_")
    assert tree.module.__name__ != "d1fw_api"
    assert any("source=regenerated" in record.getMessage()
               for record in caplog.records)


@needs_310
def test_a_matching_daemon_imports_the_bundled_package(bundled_bytes, tmp_path):
    with FakeDaemon(bundled_bytes) as daemon:
        tree = ensure.ensure_client(daemon.url, cache_dir=tmp_path)
    assert tree.module.__name__ == ensure.BUNDLED_MODULE
    assert hasattr(tree.module, "Client")


# --------------------------------------------------------------------------- #
# drift on a machine that cannot regenerate
# --------------------------------------------------------------------------- #
@pytest.fixture
def no_generator(monkeypatch):
    """Neither ``uv`` nor ``openapi-python-client`` on PATH — the robot at 3 a.m."""
    monkeypatch.setattr(_gen.shutil, "which", lambda _name: None)


def test_without_a_generator_auto_warns_and_uses_the_bundled_client(
        drifted_bytes, tmp_path, no_generator, caplog):
    with FakeDaemon(drifted_bytes) as daemon, caplog.at_level(logging.WARNING):
        resolution = ensure.resolve_client(daemon.url, cache_dir=tmp_path)
    assert resolution.source == "bundled"
    assert resolution.daemon_sha256 == sha(drifted_bytes)
    assert "regeneration unavailable" in resolution.note
    warnings = [r.getMessage() for r in caplog.records
                if r.levelno >= logging.WARNING]
    assert warnings, "falling back to a client that may not match must be LOUD"
    assert "uv" in warnings[0] and "may not match this firmware" in warnings[0]


def test_without_a_generator_strict_refuses(drifted_bytes, tmp_path,
                                            no_generator):
    with FakeDaemon(drifted_bytes) as daemon:
        with pytest.raises(ClientUnavailable) as caught:
            ensure.resolve_client(daemon.url, cache_dir=tmp_path, policy="strict")
    message = str(caught.value)
    assert sha(drifted_bytes)[:8] in message
    assert ensure.bundled_spec_sha256()[:8] in message


def test_strict_is_happy_when_the_daemon_matches(bundled_bytes, tmp_path,
                                                 no_generator):
    """strict is about mismatch, not about having a generator installed."""
    with FakeDaemon(bundled_bytes) as daemon:
        resolution = ensure.resolve_client(daemon.url, cache_dir=tmp_path,
                                           policy="strict")
    assert resolution.source == "bundled"


def test_an_unknown_policy_is_refused_before_any_socket_opens(tmp_path):
    with pytest.raises(ValueError, match="policy must be one of"):
        ensure.resolve_client("http://127.0.0.1:1", cache_dir=tmp_path,
                              policy="whatever")


def test_a_generator_that_produces_nothing_is_not_silently_accepted(
        drifted_bytes, tmp_path):
    """An exit-0 stub that writes no package must not look like success."""
    with FakeDaemon(drifted_bytes) as daemon:
        resolution = ensure.resolve_client(
            daemon.url, cache_dir=tmp_path,
            runner=[sys.executable, "-c", "pass"])
    assert resolution.source == "bundled"
    assert "regeneration unavailable" in resolution.note
