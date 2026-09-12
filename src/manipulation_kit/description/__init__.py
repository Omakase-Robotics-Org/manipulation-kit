"""manipulation_kit.description — the D1 robot's geometry, and only geometry.

This package is the **canonical home of the D1's shape**. Every URDF of the D1
or its D1 arms used anywhere in the Omakase stack either lives
here or is a provenance-tracked export of what lives here (``mkit-urdf
export``). Do not hand-copy a URDF out of it, and do not hand-edit a copy in a
consumer repository.

It came from d1-sdk ``description/`` unchanged in substance; the full
institutional notes are kept verbatim in ``docs/description-README.md``,
``docs/d1-description-README.md`` and ``docs/d1-arm-notes.md`` — read
d1-arm-notes before touching anything that looks like a left/right
asymmetry.

Two things changed in the move, both to make the package self-contained:

* **No ``$D1_SDK_DIR``.** Assets resolve package-relative, so an installed
  wheel behaves exactly like a checkout. :data:`ROOT` is the anchor.
* **No duplicated mesh trees.** ``d1_yubi_description_v2`` used to carry a
  byte-identical second copy of 27 MB of arm STLs so ROS could resolve
  ``package://`` per package; the exporter now aliases those paths onto the
  single ``d1_arm/`` copy and produces byte-identical output.

The generated URDFs are GENERATED. Edit ``d1/tools/generate_d1_urdf.py`` (or
``d1_yubi_description_v2/tools/assemble_d1_yubi.py``), never the ``.urdf`` —
a test regenerates and diffs.
"""
from __future__ import annotations

from pathlib import Path

#: Absolute path to this description tree (the anchor for every asset).
ROOT = Path(__file__).resolve().parent

#: The mesh-free whole-body collision model the motion guard parses.
GUARD_URDF = ROOT / "d1" / "d1.urdf"

#: The authoritative whole-body asset: D1 wearing the stock parallel gripper.
WHOLEBODY_GRIPPER_URDF = ROOT / "d1" / "d1_wholebody_gripper.urdf"

__all__ = ["ROOT", "GUARD_URDF", "WHOLEBODY_GRIPPER_URDF"]
