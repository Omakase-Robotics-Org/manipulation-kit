"""Get a d1-firmwared client that matches the daemon in front of us.

THE PROBLEM THIS SOLVES
-----------------------
The firmware ships as a **binary plus an OpenAPI document**. Consumers never
have a firmware checkout; what they have is a robot on the network and whatever
client they installed months ago. The failure that follows is quiet: the
firmware gains a field, changes an enum or moves a route, the old client keeps
"working" against the parts that did not move, and the mismatch surfaces as a
wrong body on an arm command rather than as an error.

So this package does not depend on a published client package at all. It
**bundles a generated one** (``_client/d1fw_api``, produced by
``openapi-python-client`` from the document under ``_client/openapi/``) and, at
connect time, asks the daemon for its own document, hashes it, and compares.
Identical → use the bundled tree, which costs one HTTP GET. Different → the
client is regenerated **on the spot** from the daemon's document into
``~/.cache/manipulation-kit/d1fw/<sha>/`` and imported from there. A firmware
that is newer than this release therefore self-heals on the next connect,
exactly once, instead of being a support ticket.

This is the mechanism `d1-inference` already runs (``tools/ensure_client.sh``),
brought inside the package because a wheel has no ``tools/`` directory.

WHAT AN OUTSIDER SEES ON A FIRST CONNECT
----------------------------------------
One INFO line, always, naming the spec hash, where the client came from and
where it lives::

    d1fw client: spec 4a91c0e2 source=bundled path=.../_client/d1fw_api

and on a newer firmware::

    d1fw client: spec 77bd51aa source=regenerated path=~/.cache/manipulation-kit/d1fw/77bd…/d1fw_api

REGENERATION NEEDS TOOLS THIS PACKAGE DOES NOT DEPEND ON
--------------------------------------------------------
``openapi-python-client`` needs Python ≥ 3.11 to run; robots run 3.10. The
generator is therefore invoked through ``uv`` (``uv tool run --python 3.11``),
which provisions that interpreter itself — and ``uv`` is an **optional runtime
tool, never a pip dependency of this package**. Without it, a drifted daemon
falls back to the bundled client with a loud WARNING (``policy="auto"``, the
default) or refuses outright (``policy="strict"``). Nothing here ever installs
anything on its own.

POLICIES
--------
``"auto"`` (default)
    Fetch, compare, regenerate on drift, and on a machine that cannot
    regenerate warn and use the bundled tree.
``"bundled"``
    Do not even ask. For an air-gapped box, a replay, or a consumer that has
    pinned a firmware version and wants no surprises.
``"strict"``
    Same as ``auto`` until regeneration is impossible; then raise
    :class:`~manipulation_kit.executors.firmware.errors.ClientUnavailable`
    rather than drive arms with a client that provably does not match.

A daemon that cannot be reached at all is **not** drift: there is nothing to
compare against, so every policy falls back to the bundled tree and says so in
the ``note``. Refusing there would make the package unusable offline, which is
where most of its tests and all of its planning live.
"""
from __future__ import annotations

import hashlib
import importlib
import importlib.util
import json
import logging
import os
import shutil
import sys
import tempfile
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Sequence

from . import _gen
from .errors import ClientUnavailable

log = logging.getLogger("manipulation_kit.executors.firmware")

_HERE = Path(__file__).resolve().parent
#: The committed snapshot: the generated package plus the document it came from.
BUNDLED_DIR = _HERE / "_client"
BUNDLED_PACKAGE = BUNDLED_DIR / _gen.PACKAGE_NAME
BUNDLED_SPEC = BUNDLED_DIR / "openapi" / "d1-firmwared.v1.json"
SNAPSHOT_PATH = BUNDLED_DIR / "SNAPSHOT.json"
#: Importable name of the committed tree (it is a real subpackage of the wheel).
BUNDLED_MODULE = f"{__package__}._client.{_gen.PACKAGE_NAME}"

#: The route the daemon publishes its own document on. Outside ``/v1`` and the
#: one response that is not enveloped — it *is* the document.
SPEC_ROUTE = "/openapi.json"
#: A connect must not hang on a robot that is off. Two seconds, then bundled.
SPEC_TIMEOUT_S = 2.0
#: Default origin, matching d1-firmwared's own and ``$D1_FIRMWARE_URL``.
DEFAULT_BASE_URL = "http://127.0.0.1:4750"
#: The generated tree uses PEP 604 unions at module scope; 3.9 cannot import it.
MIN_PYTHON = (3, 10)

POLICIES = ("auto", "bundled", "strict")


def default_cache_dir() -> Path:
    """``$XDG_CACHE_HOME/manipulation-kit/d1fw``, else ``~/.cache/...``."""
    root = os.environ.get("XDG_CACHE_HOME") or ""
    base = Path(root) if root else Path.home() / ".cache"
    return base / "manipulation-kit" / "d1fw"


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def snapshot() -> Dict[str, Any]:
    """The committed ``_client/SNAPSHOT.json``: what was generated, from what."""
    with SNAPSHOT_PATH.open(encoding="utf-8") as handle:
        return json.load(handle)


def bundled_spec_bytes() -> bytes:
    """The exact bytes of the document the bundled client was generated from."""
    return BUNDLED_SPEC.read_bytes()


def bundled_spec_sha256() -> str:
    """sha256 of :func:`bundled_spec_bytes`, computed — not read from metadata.

    The recorded hash in ``SNAPSHOT.json`` is checked against this by a test;
    at runtime the file on disk is the authority, so a hand-edited snapshot
    cannot make a stale client look current.
    """
    return sha256_bytes(bundled_spec_bytes())


def document_bytes(resolution: "Resolution") -> bytes:
    """The OpenAPI document the tree ``resolution`` names was generated FROM.

    The bundled snapshot keeps it under ``_client/openapi/``; a regenerated
    tree keeps a copy beside itself in the cache (:func:`_regenerate`). The
    generator does not carry vendor extensions such as ``x-timeout-seconds``
    into the code, so this is how an adapter reads them — from the same
    contract, never from a constant.
    """
    if resolution.source == "bundled":
        return bundled_spec_bytes()
    return (Path(resolution.path).parent / "openapi.json").read_bytes()


def fetch_spec(base_url: str, *, timeout_s: float = SPEC_TIMEOUT_S) -> bytes:
    """``GET <base_url>/openapi.json``. Raises :class:`OSError` when it cannot.

    Returns the raw bytes, because the hash has to be over exactly what the
    daemon sent — re-serialising the JSON would change the digest and every
    connect would look like drift.
    """
    url = base_url.rstrip("/") + SPEC_ROUTE
    with urllib.request.urlopen(url, timeout=timeout_s) as response:  # noqa: S310
        raw = response.read()
    if not raw:
        raise OSError(f"{url} returned an empty body")
    try:
        document = json.loads(raw)
    except ValueError as exc:
        raise OSError(f"{url} did not return JSON: {exc}") from exc
    if not isinstance(document, dict) or "openapi" not in document:
        raise OSError(f"{url} returned JSON that is not an OpenAPI document")
    return raw


@dataclass(frozen=True)
class Resolution:
    """Which client tree to import, and how that was decided.

    Produced by :func:`resolve_client` without importing anything, so the
    decision can be tested — and logged — on an interpreter too old to import
    the generated code.
    """

    #: sha256 of the document the chosen tree was generated from.
    spec_sha256: str
    #: ``bundled`` | ``cache`` | ``regenerated``.
    source: str
    #: Directory of the ``d1fw_api`` package to import.
    path: Path
    #: Name to import it under.
    module_name: str
    #: Why, in one clause, when it was not the obvious answer.
    note: str = ""
    #: The daemon's own sha, when we got one and it differed.
    daemon_sha256: Optional[str] = None

    @property
    def short_sha(self) -> str:
        return self.spec_sha256[:8]

    def line(self) -> str:
        """The single line this package logs about the client it is using."""
        tail = f" ({self.note})" if self.note else ""
        return (f"d1fw client: spec {self.short_sha} source={self.source} "
                f"path={self.path}{tail}")


def _bundled(note: str = "", daemon_sha: Optional[str] = None) -> Resolution:
    return Resolution(spec_sha256=bundled_spec_sha256(), source="bundled",
                      path=BUNDLED_PACKAGE, module_name=BUNDLED_MODULE,
                      note=note, daemon_sha256=daemon_sha)


def resolve_client(base_url: Optional[str] = None, *,
                   cache_dir: Optional[Path] = None,
                   policy: str = "auto",
                   timeout_s: float = SPEC_TIMEOUT_S,
                   runner: Optional[Sequence[str]] = None) -> Resolution:
    """Decide which generated tree matches ``base_url``, regenerating if needed.

    Does everything except the import: fetch, hash, compare, and on drift look
    in the cache and then run the generator. See the module docstring for the
    policies and for why an unreachable daemon is not drift.

    ``runner`` overrides the generator argv (the tests pass a stub).
    """
    if policy not in POLICIES:
        raise ValueError(f"policy must be one of {POLICIES}, got {policy!r}")
    if policy == "bundled":
        return _bundled("policy=bundled, daemon not consulted")

    origin = base_url or os.environ.get("D1_FIRMWARE_URL") or DEFAULT_BASE_URL
    try:
        raw = fetch_spec(origin, timeout_s=timeout_s)
    except (OSError, urllib.error.URLError) as exc:
        return _bundled(f"daemon unreachable at {origin.rstrip('/')}{SPEC_ROUTE}: "
                        f"{exc}; the client was not verified against it")

    daemon_sha = sha256_bytes(raw)
    if daemon_sha == bundled_spec_sha256():
        return _bundled(daemon_sha=daemon_sha)

    # --- drift. The daemon is the authority; the bundled tree is stale. ---
    cache_root = Path(cache_dir) if cache_dir is not None else default_cache_dir()
    tree = cache_root / daemon_sha / _gen.PACKAGE_NAME
    if (tree / "client.py").is_file():
        return Resolution(spec_sha256=daemon_sha, source="cache", path=tree,
                          module_name=_cache_module_name(daemon_sha),
                          note=f"daemon spec differs from the bundled "
                               f"{bundled_spec_sha256()[:8]}",
                          daemon_sha256=daemon_sha)

    try:
        generated = _regenerate(raw, daemon_sha, cache_root, runner=runner)
    except _gen.GeneratorUnavailable as exc:
        message = (
            f"the daemon at {origin} serves OpenAPI {daemon_sha[:8]}, but this "
            f"release bundles a client generated from {bundled_spec_sha256()[:8]}, "
            f"and it cannot be regenerated here: {exc}")
        if policy == "strict":
            raise ClientUnavailable(message) from exc
        log.warning("%s — falling back to the bundled client, which may not "
                    "match this firmware", message)
        return _bundled(f"regeneration unavailable ({exc})", daemon_sha=daemon_sha)

    return Resolution(spec_sha256=daemon_sha, source="regenerated", path=generated,
                      module_name=_cache_module_name(daemon_sha),
                      note=f"daemon spec differs from the bundled "
                           f"{bundled_spec_sha256()[:8]}",
                      daemon_sha256=daemon_sha)


def _cache_module_name(spec_sha: str) -> str:
    """A name per spec, so two daemons in one process cannot shadow each other.

    Not ``d1fw_api``: d1-inference generates a top-level package by that name
    and a consumer may well import both in the same interpreter.
    """
    return f"_mkit_d1fw_api_{spec_sha[:12]}"


def _regenerate(raw_spec: bytes, spec_sha: str, cache_root: Path, *,
                runner: Optional[Sequence[str]] = None) -> Path:
    """Generate a client for ``raw_spec`` into ``cache_root/<sha>/``."""
    destination = cache_root / spec_sha
    destination.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        spec_path = Path(tmp) / "openapi.json"
        spec_path.write_bytes(raw_spec)
        log.info("d1fw client: regenerating for spec %s into %s",
                 spec_sha[:8], destination)
        tree = _gen.generate(spec_path, destination, runner=runner)
        shutil.copyfile(spec_path, destination / "openapi.json")
    (destination / "SNAPSHOT.json").write_text(
        json.dumps({"spec_sha256": spec_sha,
                    "generated_by": "manipulation_kit.executors.firmware.ensure",
                    "generator": _gen.GENERATOR_SPEC}, indent=2) + "\n",
        encoding="utf-8")
    return tree


def import_tree(resolution: Resolution):
    """Import the package ``resolution`` names and return the module."""
    if sys.version_info < MIN_PYTHON:
        raise ClientUnavailable(
            f"the generated d1-firmwared client needs Python "
            f"{MIN_PYTHON[0]}.{MIN_PYTHON[1]} or newer (it is emitted with PEP 604 "
            f"unions at module scope); this interpreter is "
            f"{sys.version_info[0]}.{sys.version_info[1]}. Planning works on 3.9; "
            f"execution does not.")
    if resolution.source == "bundled":
        return importlib.import_module(resolution.module_name)

    name = resolution.module_name
    existing = sys.modules.get(name)
    if existing is not None:
        return existing
    init = resolution.path / "__init__.py"
    spec = importlib.util.spec_from_file_location(
        name, init, submodule_search_locations=[str(resolution.path)])
    if spec is None or spec.loader is None:  # pragma: no cover - defensive
        raise ClientUnavailable(f"cannot import a d1fw client from {resolution.path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    try:
        spec.loader.exec_module(module)
    except BaseException:
        sys.modules.pop(name, None)
        raise
    return module


@dataclass(frozen=True)
class ClientTree:
    """An imported generated client, with the provenance it was chosen by."""

    module: Any
    resolution: Resolution

    @property
    def spec_sha256(self) -> str:
        return self.resolution.spec_sha256

    @property
    def source(self) -> str:
        return self.resolution.source

    @property
    def path(self) -> Path:
        return self.resolution.path


def ensure_client(base_url: Optional[str] = None, *,
                  cache_dir: Optional[Path] = None,
                  policy: str = "auto",
                  timeout_s: float = SPEC_TIMEOUT_S,
                  runner: Optional[Sequence[str]] = None) -> ClientTree:
    """Resolve, import and return the client tree that matches ``base_url``.

    This is what :class:`~manipulation_kit.executors.firmware.executor.FirmwareExecutor`
    calls at construction, and what a consumer calls directly to reach the
    generated API surface (all of it — every operation the daemon publishes)::

        tree = ensure_client("http://d1-2:4750")
        from_ = tree.module          # the generated `d1fw_api` package
        client = from_.Client(base_url="http://d1-2:4750")

    Emits exactly one INFO line naming the spec hash, the source and the path.
    """
    resolution = resolve_client(base_url, cache_dir=cache_dir, policy=policy,
                                timeout_s=timeout_s, runner=runner)
    log.info("%s", resolution.line())
    return ClientTree(module=import_tree(resolution), resolution=resolution)
