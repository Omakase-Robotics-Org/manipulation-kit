"""Drive ``openapi-python-client`` over a d1-firmwared OpenAPI document.

One function, :func:`generate`, and it is the *only* place in this package that
runs the code generator. Both users go through it:

* :mod:`manipulation_kit.executors.firmware.ensure`, at connect time, when the
  daemon in front of us serves a spec the bundled snapshot was not built from;
* ``mkit-firmware-client refresh``, the maintainer tool that rewrites the
  committed snapshot under ``_client/``.

so a tree produced on a robot at 3 a.m. and a tree committed by a maintainer
are byte-identical for the same spec. That is the property the snapshot test
leans on, and it is why the generator version, the interpreter it runs on and
the config file are pinned here rather than passed in.

**The generator is not a dependency of this package**, and must not become one:
it needs Python ≥ 3.11 to *run* while the robots run 3.10, and the base install
of manipulation-kit deliberately carries nothing but numpy and scipy. It is
located at call time, in this order:

1. ``uv tool run --python 3.11 openapi-python-client==0.29.1`` — the preferred
   route, because ``uv`` provisions the 3.11 interpreter itself and the pinned
   version keeps the output stable across machines;
2. ``openapi-python-client`` already on ``$PATH``, for a developer box that has
   it installed;

and when neither is there, :class:`GeneratorUnavailable` is raised — which the
caller turns into a warning and a fall back to the bundled tree, never into a
silent stale client.
"""
from __future__ import annotations

import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

from . import compat

#: The generator version every snapshot in this repository was produced with.
#: Bump it and the committed ``_client/`` has to be refreshed in the same
#: commit, or ``tests/executors/test_firmware_client_snapshot.py`` fails.
GENERATOR = "openapi-python-client"
GENERATOR_VERSION = "0.29.1"
GENERATOR_SPEC = f"{GENERATOR}=={GENERATOR_VERSION}"
#: The interpreter ``uv`` provisions for it. Not the one that runs this file.
GENERATOR_PYTHON = "3.11"
#: The package name the generator emits (``package_name_override`` in config.yaml).
PACKAGE_NAME = "d1fw_api"

_HERE = Path(__file__).resolve().parent
CONFIG = _HERE / "config.yaml"


class GeneratorUnavailable(RuntimeError):
    """No usable ``openapi-python-client`` on this machine.

    Not an error on its own: the caller decides whether a machine that cannot
    regenerate should fall back to the bundled client (the default) or refuse
    to talk to a daemon it does not match (``policy="strict"``).
    """


def generator_command() -> Tuple[List[str], str]:
    """The argv prefix that runs the generator, and a word naming which route.

    Raises :class:`GeneratorUnavailable` when there is none.
    """
    if shutil.which("uv"):
        return (["uv", "tool", "run", "--python", GENERATOR_PYTHON,
                 GENERATOR_SPEC], "uv")
    on_path = shutil.which(GENERATOR)
    if on_path:
        return ([on_path], "path")
    raise GeneratorUnavailable(
        f"neither `uv` nor `{GENERATOR}` is on PATH. The generator needs "
        f"Python {GENERATOR_PYTHON}; `uv` provisions it on the fly. Install "
        f"it with:  curl -LsSf https://astral.sh/uv/install.sh | sh")


def generate(spec_path: Path, dest_parent: Path, *,
             runner: Optional[Sequence[str]] = None) -> Path:
    """Generate ``d1fw_api`` from ``spec_path`` into ``dest_parent``.

    Returns the path of the finished package (``dest_parent/d1fw_api``). The
    tree is built in a temporary directory, stripped of generator caches,
    lowered to Python 3.10 by :mod:`.compat`, and only then moved into place —
    so a failed or interrupted generation never leaves a half-written client
    where the next process would import it.

    ``runner`` overrides the argv prefix (the tests pass a stub through it).
    """
    spec_path = Path(spec_path)
    dest_parent = Path(dest_parent)
    argv_prefix = list(runner) if runner is not None else generator_command()[0]

    with tempfile.TemporaryDirectory() as tmp:
        work = Path(tmp) / "gen"
        work.mkdir()
        proc = subprocess.run(
            [*argv_prefix, "generate", "--path", str(spec_path),
             "--meta", "none", "--config", str(CONFIG), "--overwrite"],
            cwd=str(work), capture_output=True, text=True)
        produced = work / PACKAGE_NAME
        if not (produced / "client.py").is_file():
            raise GeneratorUnavailable(
                f"{' '.join(argv_prefix)} produced no {PACKAGE_NAME}/client.py "
                f"(exit {proc.returncode}): "
                f"{(proc.stderr or proc.stdout or '').strip()[:400]}")
        # The generator leaves caches behind that would make the output
        # non-deterministic; the snapshot test compares trees byte for byte.
        shutil.rmtree(produced / ".ruff_cache", ignore_errors=True)
        for cache in produced.rglob("__pycache__"):
            shutil.rmtree(cache, ignore_errors=True)
        compat.process_tree(produced)

        dest = dest_parent / PACKAGE_NAME
        dest_parent.mkdir(parents=True, exist_ok=True)
        staged = dest_parent / f".{PACKAGE_NAME}.incoming"
        shutil.rmtree(staged, ignore_errors=True)
        shutil.move(str(produced), str(staged))
        shutil.rmtree(dest, ignore_errors=True)
        staged.rename(dest)
        return dest


def generator_version(argv_prefix: Sequence[str]) -> str:
    """``<name> <version>`` as the located generator reports it, or ``unknown``."""
    try:
        out = subprocess.run([*argv_prefix, "--version"], capture_output=True,
                             text=True, timeout=120)
    except (OSError, subprocess.SubprocessError):  # pragma: no cover
        return "unknown"
    text = (out.stdout or out.stderr or "").strip().splitlines()
    return text[-1].strip() if text else "unknown"


def main(argv: Optional[List[str]] = None) -> int:  # pragma: no cover - manual
    """``python -m manipulation_kit.executors.firmware._gen <spec> <out-dir>``."""
    args = list(sys.argv[1:] if argv is None else argv)
    if len(args) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    print(generate(Path(args[0]), Path(args[1])))
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
