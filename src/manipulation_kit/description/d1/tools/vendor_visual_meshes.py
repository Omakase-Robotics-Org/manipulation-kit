#!/usr/bin/env python3
"""Import the vendor BODY CAD into description/d1/meshes/body/.

    unzip urdf20260725.zip -d /tmp/body
    python3 description/d1/tools/vendor_visual_meshes.py \
        --vendor-meshes /tmp/body/merged_robot/meshes

This is the only mesh set the `d1/` package has to carry itself. The others the
whole-body URDFs reference are already in this repo and are used IN PLACE, at
full resolution, so nothing is duplicated:

    arms  ../d1_arm/{right,left}/meshes/     (13.5 MB per side)
    yubi  ../d1_yubi_description_v2/yubi_description/meshes/
    grip  meshes/gripper/   (from dx-manipulator, see its own README)

NOT DECIMATED, deliberately. An earlier attempt simplified the arm and body
CAD to a 4 k-triangle budget: 27 MB of arm meshes came down to 7.5 MB, but the
sampled surface deviation was 1.5 to 2.9 mm mean and up to 48 mm worst case
(`slider_Link` lost 22 % of its surface area, `Link7_R` 42 %). These meshes are
dense CAD with fine features and small components; the simplifier cannot halve
them without visibly wrecking them, and the entire point of a visual mesh is to
look like the part. So they are carried verbatim. The number to know: this adds
**8.1 MB** to the repo, and d1-sdk has no Git LFS configured, so it is a plain
binary commit like the arm and YUBI meshes already are.

BODY PROVENANCE
---------------
Vendor SolidWorks package ``urdf20260725`` (`merged_robot`, generated
2026-07-25; Drive ``D1/URDF/urdf20260725.zip``) — the same robot as the
``urdf2026072302`` export this generator already takes the lift, neck and floor
height from, and the same assembly as the STEP in ``d1-face/d1_face.step`` that
the torso/chassis boxes were measured from in the first place.

It was first vendored into d1-isaaclab (its PR #7, "give the body real CAD, not
measured primitives"), and this tool used to read that copy with a
``--d1-isaaclab`` flag. It no longer does: d1-isaaclab now resolves d1-sdk by
path and its ``assets/d1/d1_body/`` copy is gone (its PR "read the robot
description from d1-sdk"), so the source of a re-import is the VENDOR PACKAGE
itself, which is where the geometry came from in the first place. Pointing this
tool at a downstream consumer's copy was a cycle — d1-sdk is the authoritative
source, and an authoritative source should not import from the repo that
imports from it.

Three of the vendor STLs (``rarmbase_Link``, ``larmbase_Link``, ``lidar_Link``)
are 80-byte files with ZERO triangles and are skipped. So are the vendor
inertias, which are junk (``lidar_Link``: 0.4 kg with Ixx = 12.7 and its CoM
0.55 m off-link; both arm plates carry the same copy-pasted tensor). Only
geometry is taken; every mass in the generated URDFs comes from elsewhere.
"""
from __future__ import annotations

import argparse
import hashlib
import pathlib
import shutil
import struct
import sys

HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE.parent / "meshes" / "body"

#: Vendor STLs to carry. Anything not hosted by a link in the generated URDFs
#: (see BODY_MESH_HOSTS there) is not copied.
WANTED = ("base_link", "rwheel_Link", "lwheel_Link", "carcamer_Link",
          "slider_Link", "fcamer_Link", "backcamer_Link", "dhead_Link",
          "uphead_Link", "headcamera_Link")


def triangles(path: pathlib.Path) -> int:
    blob = path.read_bytes()
    if len(blob) < 84:
        return 0
    return struct.unpack("<I", blob[80:84])[0]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--vendor-meshes", required=True, type=pathlib.Path,
                    help="the extracted vendor package's meshes/ directory "
                         "(urdf20260725.zip -> merged_robot/meshes)")
    args = ap.parse_args(argv)

    src = args.vendor_meshes.expanduser()
    if not src.is_dir():
        raise SystemExit(f"no body meshes at {src}")
    OUT.mkdir(parents=True, exist_ok=True)
    for stale in OUT.glob("*.STL"):
        stale.unlink()

    total = 0
    for stem in WANTED:
        path = src / f"{stem}.STL"
        n = triangles(path)
        if n == 0:
            print(f"  {path.name:24s} SKIP (zero-triangle vendor placeholder)")
            continue
        shutil.copy(path, OUT / path.name)
        size = (OUT / path.name).stat().st_size
        total += size
        print(f"  {path.name:24s} {n:6d} tris  {size // 1024:5d} KB")
    print(f"\n{total / 1e6:.1f} MB in {OUT}")
    print("sha256:")
    for path in sorted(OUT.glob("*.STL")):
        print(f"  {hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
