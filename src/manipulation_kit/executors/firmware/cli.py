"""``mkit-firmware-client`` — refresh the committed d1-firmwared client snapshot.

A **maintainer** tool, not something a consumer runs: a consumer's client
self-heals at connect time into ``~/.cache`` (see
:mod:`manipulation_kit.executors.firmware.ensure`). This is how that cached
regeneration stops being needed — by moving the new spec, and the client
generated from it, into the repository so the next release ships them.

    mkit-firmware-client refresh --url http://d1-2:4750
    mkit-firmware-client refresh --spec ~/d1-firmware/openapi/d1-firmwared.v1.json
    mkit-firmware-client check            # the committed snapshot is self-consistent
    mkit-firmware-client check --url http://d1-2:4750   # …and matches that daemon

``refresh`` rewrites three things together, because they are one fact:
``_client/openapi/d1-firmwared.v1.json`` (the document), ``_client/d1fw_api/``
(the client generated from it) and ``_client/SNAPSHOT.json`` (its sha256 and
provenance). Afterwards, bump ``project.version`` — the version gate will
insist, and it is right to: a consumer whose pip does not reinstall keeps
talking to the daemon with the old client.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import List, Optional

from . import _gen, ensure


def _snapshot_metadata(raw: bytes, *, origin: str,
                       previous: Optional[dict] = None) -> dict:
    document = json.loads(raw)
    source = {"origin": origin}
    if previous and isinstance(previous.get("spec_source"), dict):
        # Keep the provenance lines a human wrote (firmware repo, commit)
        # unless this refresh can improve on them.
        carried = {k: v for k, v in previous["spec_source"].items()
                   if k not in ("origin",)}
        source = {**carried, **source}
    return {
        "spec_sha256": ensure.sha256_bytes(raw),
        "spec_version": str(document.get("info", {}).get("version", "unknown")),
        "spec_source": source,
        "package": _gen.PACKAGE_NAME,
        "generator": _gen.GENERATOR_SPEC,
        "generator_python": _gen.GENERATOR_PYTHON,
        "post_process": ("manipulation_kit/executors/firmware/_gen/compat.py "
                         "— StrEnum/Self lowered to Python 3.10"),
    }


def refresh(*, url: Optional[str], spec: Optional[Path]) -> int:
    if (url is None) == (spec is None):
        print("give exactly one of --url or --spec", file=sys.stderr)
        return 2
    if url is not None:
        origin = url.rstrip("/") + ensure.SPEC_ROUTE
        print(f"fetching {origin}")
        raw = ensure.fetch_spec(url, timeout_s=10.0)
    else:
        origin = str(Path(spec).resolve())
        raw = Path(spec).read_bytes()
        json.loads(raw)  # fail here rather than inside the generator

    previous = ensure.snapshot() if ensure.SNAPSHOT_PATH.is_file() else None
    metadata = _snapshot_metadata(raw, origin=origin, previous=previous)
    if previous and previous.get("spec_sha256") == metadata["spec_sha256"]:
        print(f"already at spec {metadata['spec_sha256'][:8]} "
              f"({metadata['spec_version']}); regenerating anyway to prove it")

    ensure.BUNDLED_SPEC.parent.mkdir(parents=True, exist_ok=True)
    ensure.BUNDLED_SPEC.write_bytes(raw)
    print(f"generating {_gen.PACKAGE_NAME} with {_gen.GENERATOR_SPEC}")
    tree = _gen.generate(ensure.BUNDLED_SPEC, ensure.BUNDLED_DIR)
    metadata["files"] = sum(1 for _ in tree.rglob("*.py"))
    ensure.SNAPSHOT_PATH.write_text(json.dumps(metadata, indent=2) + "\n",
                                    encoding="utf-8")
    if previous and previous.get("spec_sha256") != metadata["spec_sha256"]:
        print("NOTE: spec_source carries repository/commit from the previous "
              "snapshot. If the document moved, update those lines by hand — "
              "nothing here can read a private firmware repository.")
    print(f"snapshot: spec {metadata['spec_sha256'][:8]} "
          f"(info.version {metadata['spec_version']}), "
          f"{metadata['files']} generated files under {tree}")
    print("now bump project.version in pyproject.toml and add a CHANGELOG entry")
    return 0


def check(*, url: Optional[str]) -> int:
    data = ensure.snapshot()
    on_disk = ensure.bundled_spec_sha256()
    print(f"bundled spec  {on_disk}  (info.version {data.get('spec_version')})")
    if data.get("spec_sha256") != on_disk:
        print(f"MISMATCH: SNAPSHOT.json records {data.get('spec_sha256')}",
              file=sys.stderr)
        return 1
    if not (ensure.BUNDLED_PACKAGE / "client.py").is_file():
        print(f"MISSING: no client.py under {ensure.BUNDLED_PACKAGE}", file=sys.stderr)
        return 1
    if url:
        try:
            raw = ensure.fetch_spec(url, timeout_s=10.0)
        except OSError as exc:
            print(f"daemon at {url} unreachable: {exc}", file=sys.stderr)
            return 1
        live = ensure.sha256_bytes(raw)
        print(f"daemon spec   {live}")
        if live != on_disk:
            print("the daemon serves a document this snapshot was not generated "
                  "from; run `mkit-firmware-client refresh --url ...`",
                  file=sys.stderr)
            return 1
        print("match")
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="mkit-firmware-client", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    run = sub.add_parser("refresh", help="regenerate the committed snapshot")
    source = run.add_mutually_exclusive_group(required=True)
    source.add_argument("--url", help="a running daemon, e.g. http://d1-2:4750")
    source.add_argument("--spec", type=Path, help="an OpenAPI document on disk")

    look = sub.add_parser("check", help="the snapshot is self-consistent")
    look.add_argument("--url", help="also compare it against this daemon")

    args = parser.parse_args(argv)
    if args.command == "refresh":
        return refresh(url=args.url, spec=args.spec)
    return check(url=args.url)


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
