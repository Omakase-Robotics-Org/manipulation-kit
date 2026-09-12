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
                     the AUTHORITATIVE whole-body asset,
                     d1_wholebody_gripper.urdf, with every mesh reference
                     resolved and rewritten so it loads from the exported
                     directory.
    d1_arm           the per-arm packages (URDF, left and right)

WHAT AN EXPORT CARRIES, AND WHAT IT DOES NOT
--------------------------------------------
The CAD geometry is NOT in this repository (see LICENSE-STATUS.md and
:mod:`manipulation_kit.assets`), so by default an export is MESH-FREE: the
URDFs are complete and unedited, every ``<mesh>`` reference is kept verbatim,
and each one this repository cannot supply is listed in ``PROVENANCE.json``
under ``absent_external``.  A reader can therefore always tell "not shipped"
from "lost", which is the whole job of the manifest.

``--with-assets`` collects the CAD too, from a fetched checkout or from
``$MKIT_ASSETS_DIR``.  It is a deliberate second command rather than an
automatic upgrade: an export must not change shape because of what happens to
be on the exporting machine, or the committed ``dist/`` would drift between
two people running the same line.

That is also why ``--check`` is honest in both directions.  It compares what
the flavour SAYS it exports, so a mesh-free ``dist/`` verifies on a machine
with the assets fetched, and a full export verifies without them only if it
was made with them.

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

from manipulation_kit import assets

HERE = os.path.dirname(os.path.abspath(__file__))
DESCRIPTION = os.path.normpath(os.path.join(HERE, ".."))
KIT_ROOT = os.path.normpath(os.path.join(DESCRIPTION, ".."))

PKG_V2 = "d1_yubi_description_v2"

#: Where the CAD an export declares ``absent_external`` can be obtained.
#: Written into every manifest that withholds something, so a consumer holding
#: only the export knows where the rest of the robot is without asking.
ASSETS_REPO = "Omakase-Robotics-Org/manipulation-kit-assets (private)"

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


def is_external(src: str) -> bool:
    """True if ``src`` is CAD this repository does not carry.

    Distinct from :func:`is_optional`, and the difference is the reason both
    exist.  Optional means *we chose not to ship 50 MB of decoration*; external
    means *we are not sure we are allowed to ship this at all*
    (LICENSE-STATUS.md).  A consumer reading ``PROVENANCE.json`` needs to know
    which of the two it is looking at, so they are separate lists.
    """
    return assets.is_external(src)


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


def _rewrite(tree, dest, out_name, refs, files):
    """Write the URDF with each ``<mesh>`` pointed at its in-export path."""
    for mesh, (rel, _src) in refs.items():
        mesh.set("filename", rel)
    out = os.path.join(dest, out_name)
    os.makedirs(os.path.dirname(out) or dest, exist_ok=True)
    tree.write(out, encoding="unicode", xml_declaration=True)
    files[out_name] = sha256(out)
    return out


def _collect_refs(refs, dest, files, with_assets):
    """Place every referenced mesh, returning (absent_optional, absent_external).

    The two absent lists are derived from the URDF, never from a directory
    listing, and that is deliberate: an export must describe the same robot on
    a machine with the CAD fetched and on one without it, or ``--check`` would
    pass or fail depending on the operator rather than on the files.
    """
    absent_opt, absent_ext = [], []
    for rel, src in sorted(refs.values()):
        if is_external(src):
            if not with_assets:
                absent_ext.append(rel)
                continue
            found = assets.resolve(
                os.path.relpath(os.path.abspath(src), assets.PACKAGE_ROOT))
            if found is None:
                raise SystemExit(
                    f"--with-assets, but {rel} is not available.\n  "
                    + assets.NO_ASSETS_REASON)
            src = str(found)
        elif not os.path.exists(src):
            if is_optional(rel):
                absent_opt.append(rel)
                continue
            raise SystemExit(
                f"{rel} is referenced but missing from this repository "
                f"(looked in {src})")
        d = os.path.join(dest, *rel.split("/"))
        os.makedirs(os.path.dirname(d), exist_ok=True)
        shutil.copy2(src, d)
        files[rel] = sha256(d)
    return sorted(set(absent_opt)), sorted(set(absent_ext))


def _relative_refs(tree, src_dir):
    """Mesh element -> (path inside the export, path in this repository).

    For URDFs whose references are RELATIVE to their own file.  Some reach out
    of ``description/d1/`` into the arm packages
    (``../d1_arm/right/meshes/Link3_R.STL``); the export keeps the shape of
    the path and only strips the ``..`` segments, so a reader can still see
    which package a mesh came from.
    """
    out = {}
    for mesh in tree.getroot().iter("mesh"):
        ref = mesh.get("filename")
        rel = "/".join(part for part in os.path.normpath(ref).split(os.sep)
                       if part != "..")
        out[mesh] = (rel, os.path.normpath(os.path.join(src_dir, ref)))
    return out


def _package_refs(tree, prefix, resolve_pkg):
    """Mesh element -> (path inside the export, path in this repository).

    For URDFs that address meshes by ROS ``package://`` URI.  ``resolve_pkg``
    maps the part after the prefix onto a real path under ``description/``.
    """
    out = {}
    for mesh in tree.getroot().iter("mesh"):
        ref = mesh.get("filename")
        if not ref.startswith(prefix):
            raise SystemExit(f"unexpected mesh URI {ref} (want {prefix}...)")
        rel = ref[len(prefix):]
        out[mesh] = (rel, resolve_pkg(rel))
    return out


def export_d1_yubi(dest, mesh_prefix, files, with_assets=False):
    """The mesh-bearing dual-arm robot, d1_yubi.urdf.

    ``d1_yubi.urdf`` addresses the ARM STLs through its own ROS package name
    (ROS resolves ``package://`` per package).  d1-sdk satisfied that with a
    byte-identical SECOND COPY of 27 MB of arm meshes; this kit keeps one tree
    and resolves the v2 paths through :data:`YUBI_MESH_SOURCES`, so the two
    can no longer fork.  The exported layout is unchanged.
    """
    src_urdf = os.path.join(DESCRIPTION, PKG_V2, "urdf", "d1_yubi.urdf")
    tree = ET.parse(src_urdf)

    def resolve_pkg(rel):
        for dst, src in YUBI_MESH_SOURCES.items():
            if rel.startswith(dst + "/"):
                return os.path.join(DESCRIPTION, *src.split("/"),
                                    *rel[len(dst) + 1:].split("/"))
        return os.path.join(DESCRIPTION, PKG_V2, *rel.split("/"))

    refs = _package_refs(tree, f"package://{PKG_V2}/", resolve_pkg)
    if mesh_prefix:
        for mesh, (rel, src) in list(refs.items()):
            refs[mesh] = (mesh_prefix + rel, src)
    absent = _collect_refs(refs, dest, files, with_assets)
    _rewrite(tree, dest, "d1_yubi.urdf", refs, files)
    return absent


def export_d1_collision(dest, _mesh_prefix, files, with_assets=False):
    """The guard model, d1.urdf — mesh-free, so one file is the whole export.

    d1_wholebody.urdf is deliberately NOT here.  It carries real CAD visuals,
    and copying just the .urdf left every one of its 32 <mesh> references
    dangling: a consumer got a file that parses and then fails to load.  A
    mesh-bearing whole body is the ``d1_wholebody_gripper`` flavour.

    This is also the one flavour ``--with-assets`` cannot change, and the
    reason it is the recommended export for anyone outside the org: there is
    no CAD in it to be unsure about.
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
    return [], []


def export_d1_wholebody_gripper(dest, _mesh_prefix, files, with_assets=False):
    """d1_wholebody_gripper.urdf and every mesh it references.

    Every reference is accounted for before returning — copied, or named in
    one of the two absent lists.  An export that parses but cannot load, with
    nothing in the manifest to explain why, is the failure mode this flavour
    exists to prevent.
    """
    src_urdf = os.path.join(DESCRIPTION, "d1", "d1_wholebody_gripper.urdf")
    tree = ET.parse(src_urdf)
    refs = _relative_refs(tree, os.path.dirname(src_urdf))
    os.makedirs(dest, exist_ok=True)
    absent_opt, absent_ext = _collect_refs(refs, dest, files, with_assets)
    out = _rewrite(tree, dest, "d1_wholebody_gripper.urdf", refs, files)

    declared = set(absent_opt) | set(absent_ext)
    missing = [m.get("filename") for m in ET.parse(out).getroot().iter("mesh")
               if m.get("filename") not in declared
               and not os.path.exists(os.path.join(dest, m.get("filename")))]
    if missing:
        raise SystemExit("exported URDF still has dangling mesh references: "
                         + ", ".join(sorted(set(missing))))
    return absent_opt, absent_ext


def export_d1_arm(dest, _mesh_prefix, files, with_assets=False):
    """The per-arm packages, left and right, as the vendor shaped them."""
    absent_opt, absent_ext = [], []
    for side in ("left", "right"):
        src_dir = os.path.join(DESCRIPTION, "d1_arm", side)
        urdf = f"d1_arm_{side}.urdf"
        tree = ET.parse(os.path.join(src_dir, urdf))
        pkg = f"package://d1_arm_{side}/"
        refs = _package_refs(
            tree, pkg,
            lambda rel, d=src_dir: os.path.join(d, *rel.split("/")))
        rel_root = f"d1_arm/{side}"
        for mesh, (rel, src) in list(refs.items()):
            refs[mesh] = (f"{rel_root}/{rel}", src)
        opt, ext = _collect_refs(refs, dest, files, with_assets)
        absent_opt += opt
        absent_ext += ext
        out_name = f"{rel_root}/{urdf}"
        # References inside the export are relative to the EXPORT ROOT, the
        # way every other flavour writes them, so one `--mesh-prefix`-free
        # convention holds across the tool.
        _rewrite(tree, dest, out_name, refs, files)
    return sorted(set(absent_opt)), sorted(set(absent_ext))


FLAVOURS = {
    "d1_yubi": export_d1_yubi,
    "d1_collision": export_d1_collision,
    "d1_wholebody_gripper": export_d1_wholebody_gripper,
    "d1_arm": export_d1_arm,
}


def export(flavour: str, dest: str, mesh_prefix: str = "",
           with_assets: bool = False) -> dict:
    files: dict = {}
    absent_opt, absent_ext = FLAVOURS[flavour](dest, mesh_prefix, files,
                                               with_assets)
    manifest = {
        "source_repo": "Omakase-Robotics-Org/manipulation-kit",
        "source_path": "src/manipulation_kit/description/",
        "source_commit": git("rev-parse", "HEAD") or "unknown (no checkout)",
        "source_dirty": bool(git("status", "--porcelain", ".")),
        "exported": date.today().isoformat(),
        "tool": "manipulation_kit.description.tools.export_description",
        "flavour": flavour,
        "mesh_prefix": mesh_prefix,
        "with_assets": with_assets,
        # Rendering-only meshes this kit deliberately does not carry; the URDF
        # still references them. See the module docstring.
        "absent_optional": absent_opt,
        # CAD geometry this repository does not carry AT ALL, because its
        # redistribution rights are unresolved (LICENSE-STATUS.md). The URDF
        # references it verbatim; `mkit-urdf fetch-assets` or an export made
        # with --with-assets completes it. Named, not silently dropped: a
        # consumer must be able to tell "withheld" from "lost".
        "absent_external": absent_ext,
        "assets_repo": ASSETS_REPO if absent_ext else None,
        "files": files,
    }
    with open(os.path.join(dest, "PROVENANCE.json"), "w") as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
        f.write("\n")
    return manifest


def check(flavour: str, dest: str, mesh_prefix: str = "",
          with_assets: bool = False) -> int:
    """Re-derive into a temp dir and diff content files against dest."""
    with tempfile.TemporaryDirectory() as tmp:
        fresh = export(flavour, tmp, mesh_prefix, with_assets)
        bad = []
        stale = os.path.join(dest, "PROVENANCE.json")
        if os.path.exists(stale):
            with open(stale) as stream:
                committed = json.load(stream)
            for key, label in (("absent_optional", "optional visual layer"),
                               ("absent_external", "withheld CAD")):
                if sorted(committed.get(key, [])) != sorted(fresh[key]):
                    bad.append(
                        f"ABSENT-SET  {label} differs: committed "
                        f"{sorted(committed.get(key, []))} vs "
                        f"{sorted(fresh[key])}")
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
    notes = []
    if fresh["absent_optional"]:
        notes.append(f"{len(fresh['absent_optional'])} optional visual "
                     "mesh(es) absent by design")
    if fresh["absent_external"]:
        notes.append(f"{len(fresh['absent_external'])} withheld CAD mesh(es) "
                     f"declared (see {ASSETS_REPO})")
    note = (", " + ", ".join(notes)) if notes else ""
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
    ap.add_argument("--with-assets", action="store_true",
                    help="also collect the withheld CAD, from a fetched "
                         "checkout or $MKIT_ASSETS_DIR (internal use)")
    args = ap.parse_args()
    dest = os.path.abspath(args.dest)
    if args.check:
        return check(args.flavour, dest, args.mesh_prefix, args.with_assets)
    manifest = export(args.flavour, dest, args.mesh_prefix, args.with_assets)
    print(f"exported {args.flavour}: {len(manifest['files'])} files -> {dest}")
    print(f"source commit {manifest['source_commit']}"
          + (" (DIRTY description/)" if manifest["source_dirty"] else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
