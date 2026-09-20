#!/usr/bin/env python3
"""Lower a freshly generated ``d1fw_api/`` tree from Python 3.11 to 3.10.

``openapi-python-client`` 0.29.1 needs Python **3.11 to run** and emits code
for a modern interpreter: string enums as ``enum.StrEnum`` (new in 3.11) and
``Self`` imported from ``typing`` (new in 3.11). Every D1 runs Python 3.10,
where both are an import-time failure, so the generator's raw output cannot be
shipped as-is. This pass lowers exactly those two constructs and nothing else:

* ``from enum import StrEnum`` → ``from enum import Enum``, and every
  ``class X(StrEnum):`` → ``class X(str, Enum):`` — the pre-3.11 idiom the
  generator itself emitted before it adopted ``StrEnum``, behaviourally
  identical for a plain string enum;
* ``from typing import Self`` → ``from typing_extensions import Self``.

It does **not** lower PEP 604 unions (``X | None``), which the generator uses
at module scope in ``types.py`` and in unguarded function signatures. That is
why the generated client's floor is 3.10 and not this package's own 3.9 —
see ``manipulation_kit.executors.firmware.ensure``.

The transforms are pure, line-oriented substitutions, so *generate → compat* is
deterministic and idempotent. That is what lets the snapshot test assert the
committed tree is what a regeneration produces.

    python -m manipulation_kit.executors.firmware._gen.compat path/to/d1fw_api

Derived from ``tools/py310_compat.py`` in Omakase-Robotics-Org/d1-inference,
which runs the same lowering for the same reason.
"""
from __future__ import annotations

import sys
from pathlib import Path

#: The lowest interpreter the lowered tree imports on.
CLIENT_MIN_PYTHON = (3, 10)


def _lower_line(line: str) -> str:
    stripped = line.strip()
    # 1) StrEnum import -> Enum import, collapsing the mixed forms too.
    if stripped == "from enum import StrEnum":
        return line.replace("from enum import StrEnum", "from enum import Enum")
    if stripped in ("from enum import Enum, StrEnum", "from enum import StrEnum, Enum"):
        return line[: len(line) - len(line.lstrip())] + "from enum import Enum\n"
    # 2) class bases: StrEnum -> (str, Enum).
    if stripped.startswith("class ") and "(StrEnum)" in line:
        return line.replace("(StrEnum)", "(str, Enum)")
    # 3) `Self` is 3.11+ in `typing`; pull it out of any single-line
    #    `from typing import ...` and take it from typing_extensions instead
    #    (already a runtime dependency of the generated models).
    if stripped.startswith("from typing import "):
        names = [n.strip() for n in stripped[len("from typing import "):].split(",")]
        if "Self" in names:
            indent = line[: len(line) - len(line.lstrip())]
            newline = "\n" if line.endswith("\n") else ""
            rest = [n for n in names if n != "Self"]
            out = ""
            if rest:
                out += f"{indent}from typing import {', '.join(rest)}{newline}"
            out += f"{indent}from typing_extensions import Self{newline}"
            return out
    return line


def process_file(path: Path) -> bool:
    """Lower one file in place. True when it changed."""
    original = path.read_text(encoding="utf-8")
    lowered = "".join(_lower_line(line) for line in original.splitlines(keepends=True))
    if lowered != original:
        path.write_text(lowered, encoding="utf-8")
        return True
    return False


def process_tree(pkg: Path) -> int:
    """Lower every ``*.py`` under ``pkg``. Returns the number changed."""
    return sum(1 for py in sorted(pkg.rglob("*.py")) if process_file(py))


def main(argv: list) -> int:
    if len(argv) != 2:
        print("usage: compat.py <d1fw_api dir>", file=sys.stderr)
        return 2
    pkg = Path(argv[1])
    if not pkg.is_dir():
        print(f"compat: {pkg} is not a directory", file=sys.stderr)
        return 2
    print(f"compat: lowered StrEnum/Self in {process_tree(pkg)} file(s) under {pkg}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
