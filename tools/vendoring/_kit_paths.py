"""Where the installed ``manipulation_kit`` package lives.

These tools used to sit INSIDE the package and anchor their output with
``Path(__file__).parents[n]``. They live outside the wheel now (they are not
needed to use the kit, and they need CAD that is not in this repository), so
the anchor has to come from the import instead of from the file's own
location.

Works from a checkout and from an installed wheel alike, which is the same
promise the assets themselves make.
"""

from __future__ import annotations

from pathlib import Path


def kit_root() -> Path:
    """The installed ``manipulation_kit`` package directory."""
    import manipulation_kit  # noqa: PLC0415 — the whole point of this module

    return Path(manipulation_kit.__file__).resolve().parent


def description_d1() -> Path:
    """``manipulation_kit/description/d1`` — the D1 description tree."""
    return kit_root() / "description" / "d1"


def parallel_gripper() -> Path:
    """``manipulation_kit/hands/d1/parallel_gripper``."""
    return kit_root() / "hands" / "d1" / "parallel_gripper"
