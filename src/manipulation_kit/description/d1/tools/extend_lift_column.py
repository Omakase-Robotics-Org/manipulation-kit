#!/usr/bin/env python3
"""Derive the EXTENDED moving-cover visual from the maintained column mesh.

    python3 description/d1/tools/extend_lift_column.py [SOURCE] [--output OBJ]

``mkit-urdf fetch-visuals`` (and ``fetch-assets``, when it finds the optional
visual layer) runs this for you, so a normal checkout never has to think about
it.

WHY THIS EXISTS.  The D1 lift is telescoping, and the built robot's moving
cover is 53.829712 mm longer than the CAD that every visual mesh was split out
of — see the MOVING LIFT COLUMN EXTENSION block in ``generate_d1_urdf.py`` for
the measurement.  The vest regions bolted to the cover are simply carried up by
a visual origin in the URDF.  The cover itself cannot be: its bottom lip has to
stay exactly where it is (11 mm above the fixed AMR cover at full down) while
its straight walls get longer at the top.  That is a change to the GEOMETRY, so
it needs a mesh.

WHAT IT DOES.  Every vertex at or below the rounded bottom lip stays put; every
vertex above it rises by the extension.  The wall triangles that straddle the
cut therefore stretch, which is exactly the straight-walled section getting
longer.  Faces, normals, materials and all x/y coordinates are untouched — the
cover's cross-section is unchanged, only its length.

The source mesh is NOT edited and NOT replaced: it stays the maintained CAD
split, and this writes a second file beside it.  d1-isaaclab derives the same
mesh the same way (``scripts/extend_lift_column.py``, PR #43).
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path

#: Everything at or below this height in ``torso_column`` is the cover's
#: rounded bottom lip and does not move.  Just above
#: :data:`~generate_d1_urdf.MOVING_COLUMN_BOTTOM_Z` (0.079), inside the
#: straight-walled section, so the cut passes through wall faces rather than
#: through the curvature of the lip.
LIP_TOP_Z = 0.082001

#: Kept here rather than imported: this module is standalone on purpose (the
#: CLI runs it against a fetched tree, with no generator import), and the two
#: are pinned together by ``tests/test_lift_column_extension.py``.
EXTENSION = 0.053829712

#: Source and result, relative to ``description/d1/``.
SOURCE_REL = "meshes/body_hifi/torso_column_white.obj"
OUTPUT_REL = "meshes/body_hifi/torso_column_white_extended.obj"


def extend(source, output, extension: float = EXTENSION) -> int:
    """Write ``source`` to ``output`` with the cover's walls lengthened.

    Returns the number of vertices that moved, so a caller can tell a real
    extension from a no-op on an unexpected mesh.
    """
    source, output = Path(source), Path(output)
    moved, lines = 0, []
    for line in source.read_text().splitlines():
        if line.startswith("v "):
            x, y, z = (float(v) for v in line.split()[1:4])
            if z > LIP_TOP_Z:
                z += extension
                moved += 1
            line = f"v {x:.9f} {y:.9f} {z:.9f}"
        lines.append(line)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines) + "\n")
    return moved


def extend_in(visual_dir) -> "Path | None":
    """Derive the extended cover inside a ``meshes/body_hifi`` directory.

    ``None`` when the source is not there — the visual layer is optional (see
    ``manipulation_kit.assets``), and a missing decorative mesh is a normal
    state, not a failure.
    """
    visual_dir = Path(visual_dir)
    source = visual_dir / Path(SOURCE_REL).name
    if not source.is_file():
        return None
    output = visual_dir / Path(OUTPUT_REL).name
    extend(source, output)
    return output


def main(argv=None) -> int:
    here = Path(__file__).resolve().parents[1]
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("source", nargs="?", type=Path, default=here / SOURCE_REL,
                    help=f"the maintained cover mesh (default: {SOURCE_REL})")
    ap.add_argument("--output", type=Path, default=here / OUTPUT_REL)
    args = ap.parse_args(argv)
    if not args.source.is_file():
        print(f"extend_lift_column: no {args.source} — the visual layer is "
              "optional and absent (`mkit-urdf fetch-visuals`)")
        return 2
    moved = extend(args.source, args.output)
    print(f"wrote {os.path.normpath(args.output)} ({moved} vertices raised by "
          f"{EXTENSION * 1000:.6f} mm)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
