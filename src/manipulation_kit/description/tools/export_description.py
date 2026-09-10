#!/usr/bin/env python3
"""Export D1 description assets to a consumer repository, with provenance.

``manipulation_kit/description/`` is the single source of truth for the D1
robot's geometry (see docs/description-README.md).  Consumers that need local
mesh files at runtime (simulators: d1-manip-sim, d1-isaaclab, ...) do not
hand-copy URDFs; they vendor them with this tool, which

  1. copies the requested model flavour into the consumer directory,
  2. rewrites ``package://<pkg>/`` mesh URIs to paths relative to the
     exported URDF (simulators like MuJoCo / Genesis / Isaac resolve
     relative mesh paths against the URDF's own directory — never bake in
     an absolute path, it is a per-machine site fact),
  3. writes ``PROVENANCE.json`` next to the exported files: the manipulation-kit
     commit, a dirty flag, and a sha256 per exported file.

The consumer then commits the exported files AND the manifest, and its test
suite verifies the files still match the manifest (no silent hand-edits).
Re-running the export against a newer manipulation-kit is the only sanctioned
way to change a vendored copy.

Usage (the CLI ``mkit-urdf export`` is the front door; this stays runnable):
    python3 -m manipulation_kit.description.tools.export_description \
        <flavour> --dest DIR [--mesh-prefix PREFIX] [--check]

Flavours:
    d1_yubi          mesh-bearing dual-arm robot (d1_yubi_description_v2):
                     d1_yubi.urdf + d1_arm_yubi_description/meshes
                     + yubi_description/meshes
    d1_collision     the generated GUARD model, d1.urdf alone — the one file in
                     the family with no meshes at all, so the export is a single
                     self-contained file.  It used to carry d1_wholebody.urdf
                     too; that stopped being correct the moment the whole-body
                     variants gained real CAD visuals, because copying the URDF
                     without its meshes produced 32 dangling <mesh> references.
                     Use d1_wholebody_gripper for a mesh-bearing whole body.
    d1_wholebody_gripper
                     the AUTHORITATIVE whole-body asset, d1_wholebody_gripper
                     .urdf, WITH every mesh it references (body CAD, both arms,
                     the gripper) collected next to it and the paths rewritten
                     so they resolve from the exported directory.
    d1_arm  vendor per-arm packages, copied verbatim
                     (URDF + STL meshes, left and right)

--check re-derives the export in a temp dir and fails (exit 1) if the
consumer's committed copy differs — run it from consumer CI with a manipulation-kit
checkout available to detect drift from the source of truth.
"""
from __future__ import annotations

import argparse
import filecmp
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from datetime import date

HERE = os.path.dirname(os.path.abspath(__file__))
DESCRIPTION = os.path.normpath(os.path.join(HERE, ".."))
KIT_ROOT = os.path.normpath(os.path.join(DESCRIPTION, ".."))

PKG_V2 = "d1_yubi_description_v2"

#: Mesh reference prefixes that may be absent from an export (see the module
#: docstring).  Rendering-only geometry, never collision, never kinematics.
OPTIONAL_MESH_PREFIXES = ("meshes/body_hifi/",)

#: ``d1_yubi.urdf`` addresses the ARM STLs through this package's own ROS
#: package name, because ROS resolves ``package://`` per package.  d1-sdk
#: satisfied that with a byte-identical SECOND COPY of 27 MB of arm meshes;
#: manipulation-kit keeps ONE copy in ``d1_arm/`` and resolves the v2
#: paths through this alias table instead.  The exported bytes are unchanged —
#: what changed is that the two arm trees can no longer fork, because there is
#: only one.
#:
#: The 576 KB ``yubi_description/meshes`` copy is deliberately NOT aliased and
#: stays on disk: ``d1_wholebody.urdf`` reaches it by a RELATIVE path
#: (``../d1_yubi_description_v2/yubi_description/meshes/...``), not through
#: ``package://``, so aliasing it would either dangle that URDF or change its
#: bytes.  Half a megabyte is not worth either.
YUBI_MESH_SOURCES = {
    "d1_arm_yubi_description/meshes/d1_arm_l": "d1_arm/left/meshes",
    "d1_arm_yubi_description/meshes/d1_arm_r": "d1_arm/right/meshes",
}


def is_optional(ref: str) -> bool:
    """True if a missing ``ref`` is the optional visual layer, not a defect."""
    return ref.startswith(OPTIONAL_MESH_PREFIXES)


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def git(*args: str) -> str:
    """Best-effort git query against the manipulation-kit checkout.

    Returns ``""`` when there is no checkout — an installed wheel has no ``.git``
    and a provenance record that says "unknown commit" is honest, whereas
    crashing the export would make the tool unusable exactly where consumers
    most want it (a CI container with a pip install and no repo).
    """
    try:
        out = subprocess.run(["git", "-C", KIT_ROOT, *args],
                             capture_output=True, text=True, check=True)
    except (subprocess.CalledProcessError, OSError):
        return ""
    return out.stdout.strip()


def copy_tree(src: str, dest: str, rel: str, files: dict) -> None:
    for root, _dirs, names in os.walk(src):
        for name in sorted(names):
            s = os.path.join(root, name)
            r = os.path.join(rel, os.path.relpath(s, src))
            d = os.path.join(dest, r)
            os.makedirs(os.path.dirname(d), exist_ok=True)
            shutil.copy2(s, d)
            files[r] = sha256(d)


def export_d1_yubi(dest: str, mesh_prefix: str, files: dict) -> None:
    src_urdf = os.path.join(DESCRIPTION, PKG_V2, "urdf", "d1_yubi.urdf")
    with open(src_urdf) as f:
        text = f.read()
    text = text.replace(f"package://{PKG_V2}/", mesh_prefix)
    out = os.path.join(dest, "d1_yubi.urdf")
    os.makedirs(dest, exist_ok=True)
    with open(out, "w") as f:
        f.write(text)
    files["d1_yubi.urdf"] = sha256(out)
    # The arm meshes live once, under d1_arm/; see YUBI_MESH_SOURCES.
    for rel, src in YUBI_MESH_SOURCES.items():
        copy_tree(os.path.join(DESCRIPTION, *src.split("/")),
                  dest, rel, files)
    copy_tree(os.path.join(DESCRIPTION, PKG_V2, "yubi_description", "meshes"),
              dest, "yubi_description/meshes", files)


def export_d1_collision(dest: str, _mesh_prefix: str, files: dict) -> None:
    """The guard model, d1.urdf — mesh-free, so one file is the whole export.

    d1_wholebody.urdf is deliberately NOT here.  It carries real CAD visuals,
    and copying just the .urdf left every one of its 32 <mesh> references
    dangling: a consumer got a file that parses and then fails to load.  A
    mesh-bearing whole body is the ``d1_wholebody_gripper`` flavour, which
    collects the meshes too.
    """
    os.makedirs(dest, exist_ok=True)
    src = os.path.join(DESCRIPTION, "d1", "d1.urdf")
    d = os.path.join(dest, "d1.urdf")
    shutil.copy2(src, d)
    files["d1.urdf"] = sha256(d)
    if [m.get("filename") for m in ET.parse(d).getroot().iter("mesh")]:
        raise SystemExit(
            "d1.urdf has gained mesh references — it is supposed to be the "
            "mesh-free guard model. Either that is a regression in the "
            "generator, or this flavour now needs to collect meshes.")


def export_d1_wholebody_gripper(dest: str, _mesh_prefix: str,
                                files: dict) -> None:
    """d1_wholebody_gripper.urdf and every mesh it references.

    The source URDF points at meshes with paths relative to ITSELF, some of
    them reaching out of ``description/d1/`` into the arm packages
    (``../d1_arm/right/meshes/Link3_R.STL``).  A flat copy would
    dangle, so each reference is resolved against the source URDF, copied
    under a path that stays inside the export, and rewritten to match.  The
    rewrite only strips ``../`` segments, so a reader can still see which
    package a mesh came from.

    Every reference is verified to resolve before returning: an export that
    parses but cannot load is the failure mode this flavour exists to fix.
    """
    src_urdf = os.path.join(DESCRIPTION, "d1", "d1_wholebody_gripper.urdf")
    src_dir = os.path.dirname(src_urdf)
    os.makedirs(dest, exist_ok=True)
    tree = ET.parse(src_urdf)
    absent: list = []
    for mesh in tree.getroot().iter("mesh"):
        ref = mesh.get("filename")
        src = os.path.normpath(os.path.join(src_dir, ref))
        # "../d1_arm/right/meshes/x.STL" -> "d1_arm/right/..."
        rel_ref = "/".join(p for p in os.path.normpath(ref).split(os.sep)
                           if p != "..")
        if not os.path.exists(src):
            if is_optional(ref):
                absent.append(rel_ref)
                mesh.set("filename", rel_ref)
                continue
            raise SystemExit(f"d1_wholebody_gripper.urdf references {ref}, "
                             f"which does not exist at {src}")
        rel = rel_ref
        d = os.path.join(dest, *rel.split("/"))
        os.makedirs(os.path.dirname(d), exist_ok=True)
        shutil.copy2(src, d)
        files[rel] = sha256(d)
        mesh.set("filename", rel)
    out = os.path.join(dest, "d1_wholebody_gripper.urdf")
    tree.write(out, encoding="unicode", xml_declaration=True)
    files["d1_wholebody_gripper.urdf"] = sha256(out)

    missing = [m.get("filename") for m in ET.parse(out).getroot().iter("mesh")
               if not os.path.exists(os.path.join(dest, m.get("filename")))
               and m.get("filename") not in set(absent)]
    if missing:
        raise SystemExit("exported URDF still has dangling mesh references: "
                         + ", ".join(sorted(set(missing))))
    return sorted(set(absent))


def export_d1_arm(dest: str, _mesh_prefix: str, files: dict) -> None:
    for side in ("left", "right"):
        src = os.path.join(DESCRIPTION, "d1_arm", side)
        rel = os.path.join("d1_arm", side)
        urdf = f"d1_arm_{side}.urdf"
        d = os.path.join(dest, rel, urdf)
        os.makedirs(os.path.dirname(d), exist_ok=True)
        shutil.copy2(os.path.join(src, urdf), d)
        files[os.path.join(rel, urdf)] = sha256(d)
        copy_tree(os.path.join(src, "meshes"), dest,
                  os.path.join(rel, "meshes"), files)


FLAVOURS = {
    "d1_yubi": export_d1_yubi,
    "d1_collision": export_d1_collision,
    "d1_wholebody_gripper": export_d1_wholebody_gripper,
    "d1_arm": export_d1_arm,
}


def export(flavour: str, dest: str, mesh_prefix: str) -> dict:
    files: dict = {}
    absent = FLAVOURS[flavour](dest, mesh_prefix, files) or []
    manifest = {
        "source_repo": "Omakase-Robotics-Org/manipulation-kit",
        "source_path": "src/manipulation_kit/description/",
        "source_commit": git("rev-parse", "HEAD") or "unknown (no checkout)",
        "source_dirty": bool(git("status", "--porcelain", ".")),
        "exported": date.today().isoformat(),
        "tool": "manipulation_kit.description.tools.export_description",
        "flavour": flavour,
        "mesh_prefix": mesh_prefix,
        # Rendering-only meshes this kit deliberately does not carry; the URDF
        # still references them. See the module docstring.
        "absent_optional": absent,
        "files": files,
    }
    with open(os.path.join(dest, "PROVENANCE.json"), "w") as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
        f.write("\n")
    return manifest


def check(flavour: str, dest: str, mesh_prefix: str) -> int:
    """Re-derive into a temp dir and diff content files against dest."""
    with tempfile.TemporaryDirectory() as tmp:
        fresh = export(flavour, tmp, mesh_prefix)
        bad = []
        stale = os.path.join(dest, "PROVENANCE.json")
        if os.path.exists(stale):
            with open(stale) as stream:
                committed = json.load(stream)
            if sorted(committed.get("absent_optional", [])) != sorted(
                    fresh["absent_optional"]):
                bad.append(
                    "ABSENT-SET  optional visual layer differs: committed "
                    f"{sorted(committed.get('absent_optional', []))} vs "
                    f"{sorted(fresh['absent_optional'])}")
        for rel in sorted(fresh["files"]):
            a, b = os.path.join(tmp, rel), os.path.join(dest, rel)
            if not os.path.exists(b):
                bad.append(f"MISSING  {rel}")
            elif not filecmp.cmp(a, b, shallow=False):
                bad.append(f"DIFFERS  {rel}")
        if bad:
            print(f"export drift vs manipulation_kit/description/ "
                  f"({len(bad)} item(s)):")
            for line in bad:
                print("  " + line)
            return 1
    note = (f", {len(fresh['absent_optional'])} optional visual mesh(es) "
            f"absent by design" if fresh["absent_optional"] else "")
    print(f"{flavour}: {len(fresh['files'])} files match "
          f"manipulation_kit/description/{note}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("flavour", choices=sorted(FLAVOURS))
    ap.add_argument("--dest", required=True, help="consumer directory to export into")
    ap.add_argument("--mesh-prefix", default="",
                    help="replacement for 'package://%s/' in mesh URIs "
                         "(default: empty = relative to the exported URDF)" % PKG_V2)
    ap.add_argument("--check", action="store_true",
                    help="verify an existing export instead of writing it")
    args = ap.parse_args()
    dest = os.path.abspath(args.dest)
    if args.check:
        return check(args.flavour, dest, args.mesh_prefix)
    manifest = export(args.flavour, dest, args.mesh_prefix)
    print(f"exported {args.flavour}: {len(manifest['files'])} files -> {dest}")
    print(f"source commit {manifest['source_commit']}"
          + (" (DIRTY description/)" if manifest["source_dirty"] else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
