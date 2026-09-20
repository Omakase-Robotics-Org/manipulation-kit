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

#: Downward tilt of the head-camera mount face, DEGREES below horizontal, per
#: D1 HARDWARE REVISION. This is a DESIGN value of the head part — the D435 is
#: bolted to a machined face, not aimed — so it changes when the part changes,
#: and it is the one number in this description that is not the same on every
#: robot that will exist.
#:
#: * ``"rev1"`` — 15 deg. The head part on d1-1, d1-2 and d1-3, read off the
#:   head-part CAD section Shu supplied 2026-09-17 and confirmed by Shu
#:   2026-09-20 as by design rather than as a build tolerance.
#: * ``"rev2"`` — 20 deg. The head part the NEXT units are built with
#:   (Shu, 2026-09-20). No robot wears it yet, so no URDF is committed for it;
#:   generate one with ``mkit-urdf build --hardware-revision rev2``.
#:
#: NOMINAL, AND NOT A SUBSTITUTE FOR CALIBRATION. What a simulator or a
#: perception stack consumes is the PER-ROBOT extrinsic in that robot's
#: ``cameras_<robot>.json``, which is an absolute ``head_link`` -> camera
#: transform fitted from ArUco. d1-3's fit sits ~2.3 deg off this nominal;
#: that deviation belongs to d1-3, not to the head part, and must never be
#: folded back into this table or into a shared URDF.
HEAD_CAMERA_TILT_DEG = {"rev1": 15.0, "rev2": 20.0}

#: The revision the COMMITTED URDFs in this package describe.
DEFAULT_HARDWARE_REVISION = "rev1"


def head_camera_tilt_deg(revision: str = DEFAULT_HARDWARE_REVISION) -> float:
    """Head-camera mount tilt for one hardware revision, degrees below horizontal.

    Consumers that compose their own asset off this package (d1-isaaclab's
    ``robot/build_urdf.py``) name the revision they are building for, so that
    "which head part is this robot wearing" is an explicit choice rather than
    whatever literal the generator last had in it.
    """
    try:
        return HEAD_CAMERA_TILT_DEG[revision]
    except KeyError:
        raise ValueError(
            f"unknown D1 hardware revision {revision!r}; known revisions: "
            + ", ".join(sorted(HEAD_CAMERA_TILT_DEG))) from None


__all__ = ["ROOT", "GUARD_URDF", "WHOLEBODY_GRIPPER_URDF",
           "HEAD_CAMERA_TILT_DEG", "DEFAULT_HARDWARE_REVISION",
           "head_camera_tilt_deg"]
