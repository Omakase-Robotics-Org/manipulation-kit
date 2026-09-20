"""``mkit-urdf`` — build, export and complete the D1 description.

    mkit-urdf build [--only d1.urdf]          regenerate the URDFs in place
    mkit-urdf build --hardware-revision rev2  ... for the next head part
    mkit-urdf export <variant> --dest DIR     vendor a variant, with provenance
    mkit-urdf export <variant> --dest DIR --check   fail on drift
    mkit-urdf variants                        list what can be built
    mkit-urdf fetch-assets --from SRC         fetch the withheld CAD
    mkit-urdf fetch-visuals --from-d1-sdk DIR fetch the optional visual layer

``build`` runs ``description/d1/tools/generate_d1_urdf.py`` (and, with
``--yubi``, ``assemble_d1_yubi.py``). The URDFs are GENERATED: edit the
generator, never the ``.urdf``. A test regenerates and diffs, so a hand-edit
fails CI rather than shipping.
"""
from __future__ import annotations

import argparse
import runpy
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from .. import assets
from . import ROOT
from .variants import VARIANTS, VariantNotBuildable, resolve

GENERATOR = ROOT / "d1" / "tools" / "generate_d1_urdf.py"
YUBI_GENERATOR = ROOT / "d1_yubi_description_v2" / "tools" / "assemble_d1_yubi.py"

#: Where the optional visual layer lands, and what it is called upstream.
VISUAL_DIR = ROOT / "d1" / "meshes" / "body_hifi"
VISUAL_SOURCE = Path("description") / "d1" / "meshes" / "body_hifi"


def _run(script: Path, argv: list) -> int:
    """Run a generator script as ``__main__`` in-process.

    In-process rather than by subprocess so the CLI works from a wheel, a
    zip-safe-free install or a checkout without caring where ``python3`` is or
    what ``sys.path`` looks like in a child.
    """
    saved = sys.argv
    sys.argv = [str(script), *argv]
    try:
        runpy.run_path(str(script), run_name="__main__")
    except SystemExit as exc:
        return int(exc.code or 0)
    finally:
        sys.argv = saved
    return 0


def cmd_build(args) -> int:
    argv = []
    for name in args.only or []:
        argv += ["--only", name]
    if args.hardware_revision:
        argv += ["--hardware-revision", args.hardware_revision]
    rc = _run(GENERATOR, argv)
    if rc == 0 and args.yubi:
        rc = _run(YUBI_GENERATOR, [])
    return rc


def cmd_export(args) -> int:
    from .tools import export_description as ex  # noqa: PLC0415

    try:
        variant = resolve(args.variant)
    except VariantNotBuildable as exc:
        print(f"mkit-urdf: {exc}", file=sys.stderr)
        return 2
    dest = str(Path(args.dest).absolute())
    if args.check:
        return ex.check(variant.flavour, dest, args.mesh_prefix,
                        args.with_assets)
    manifest = ex.export(variant.flavour, dest, args.mesh_prefix,
                         args.with_assets)
    print(f"exported {variant.name}: {len(manifest['files'])} files -> {dest}")
    print(f"source commit {manifest['source_commit']}"
          + (" (DIRTY)" if manifest["source_dirty"] else ""))
    if manifest["absent_optional"]:
        print(f"{len(manifest['absent_optional'])} optional visual mesh(es) "
              f"absent by design — `mkit-urdf fetch-visuals` to complete them")
    if manifest["absent_external"]:
        print(f"{len(manifest['absent_external'])} CAD mesh(es) withheld and "
              f"declared in PROVENANCE.json (LICENSE-STATUS.md) — they live in "
              f"{manifest['assets_repo']}; `mkit-urdf fetch-assets --from` it, "
              f"or re-export with --with-assets, to include them")
    return 0


def cmd_variants(_args) -> int:
    width = max(len(n) for n in VARIANTS)
    for name in sorted(VARIANTS):
        v = VARIANTS[name]
        mark = " " if v.buildable else "!"
        print(f"{mark} {name:<{width}}  {v.summary}")
    if any(not v.buildable for v in VARIANTS.values()):
        print("\n! = registered but not buildable; "
              "`mkit-urdf export <name>` prints why")
    return 0


def _asset_source(spec: str):
    """Yield a local directory for ``spec``, cloning it first if it is a URL.

    A git URL is cloned ``--depth 1`` into a temp dir that is removed on the
    way out: the CAD ends up in the package (where ``.gitignore`` covers it)
    and nowhere else, so a fetch cannot leave a second unlicensed copy on the
    machine for somebody to find later and publish.
    """
    path = Path(spec).expanduser()
    if path.is_dir():
        return None, path
    if not any(spec.startswith(p) or "@" in spec.split("/")[0]
               for p in ("http://", "https://", "git@", "ssh://", "git://")):
        raise SystemExit(
            f"mkit-urdf: {spec!r} is neither a directory nor a git URL")
    tmp = tempfile.TemporaryDirectory(prefix="mkit-assets-")
    print(f"cloning {spec} ...")
    try:
        subprocess.run(["git", "clone", "--depth", "1", spec, tmp.name + "/a"],
                       check=True)
    except (subprocess.CalledProcessError, OSError) as exc:
        tmp.cleanup()
        raise SystemExit(f"mkit-urdf: could not clone {spec}: {exc}") from exc
    return tmp, Path(tmp.name) / "a"


def cmd_fetch_assets(args) -> int:
    """Copy the withheld CAD in from a manipulation-kit-assets checkout.

    This repository ships the URDFs and not the geometry they reference (see
    LICENSE-STATUS.md). The assets repository mirrors this package's layout
    exactly, so completing a checkout is a copy and never a rewrite: no URDF
    is touched, no path is patched, and `mkit-urdf build` reproduces the same
    bytes before and after.

    The destinations are all in ``.gitignore``. Fetching cannot put the CAD
    back into this repository's history, which is the point.
    """
    tmp, src = _asset_source(args.source)
    try:
        root = src / "manipulation_kit"
        if not root.is_dir():
            root = src
        found = {}
        for rel in assets.external_dirs():
            d = root / rel
            if not d.is_dir():
                continue
            names = sorted(p for p in d.iterdir()
                           if p.is_file() and p.suffix in assets.ASSET_SUFFIXES)
            if names:
                found[rel] = names
        if not found:
            print(f"mkit-urdf: no CAD under {src}\n"
                  f"  expected a manipulation-kit-assets checkout with "
                  f"{assets.EXTERNAL_ASSET_DIRS[0]}/ in it", file=sys.stderr)
            return 2
        total = 0
        for rel, names in sorted(found.items()):
            out = assets.PACKAGE_ROOT / rel
            out.mkdir(parents=True, exist_ok=True)
            for name in names:
                shutil.copy2(name, out / name.name)
                total += 1
            print(f"  {len(names):3d} -> manipulation_kit/{rel}/")
        print(f"fetched {total} file(s) from {src}")
        missing = [r for r in assets.EXTERNAL_ASSET_DIRS if r not in found]
        if missing:
            print("still incomplete: " + ", ".join(missing), file=sys.stderr)
            return 1
    finally:
        if tmp is not None:
            tmp.cleanup()
    return 0


def cmd_fetch_visuals(args) -> int:
    """Copy the ~50 MB decorative body visuals in from a d1-sdk checkout.

    manipulation-kit does not carry them: they are rendering-only, the guard
    never sees them, and their redistribution rights are unresolved (see
    LICENSE-STATUS.md). The URDFs reference them verbatim, so dropping the
    files in is all it takes — no regeneration.
    """
    src = Path(args.from_d1_sdk).expanduser() / VISUAL_SOURCE
    if not src.is_dir():
        print(f"mkit-urdf: no visual meshes at {src}\n"
              f"  pass --from-d1-sdk pointing at a d1-sdk checkout "
              f"(the directory containing description/)", file=sys.stderr)
        return 2
    VISUAL_DIR.mkdir(parents=True, exist_ok=True)
    n = 0
    for obj in sorted(src.glob("*.obj")):
        shutil.copy2(obj, VISUAL_DIR / obj.name)
        n += 1
    print(f"fetched {n} visual mesh(es) -> {VISUAL_DIR}")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="mkit-urdf",
                                 description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)

    b = sub.add_parser("build", help="regenerate the URDFs from the generator")
    b.add_argument("--hardware-revision", default=None,
                   help="which D1 head part to build for (rev1 = d1-1..d1-3 "
                        "at 15 deg, rev2 = the next units at 20 deg). Only "
                        "the head-camera mount tilt depends on it. Default: "
                        "the revision the committed URDFs describe.")
    b.add_argument("--only", action="append",
                   help="regenerate one file (e.g. d1.urdf); repeatable")
    b.add_argument("--yubi", action="store_true",
                   help="also reassemble d1_yubi.urdf")
    b.set_defaults(func=cmd_build)

    e = sub.add_parser("export", help="vendor a variant into a consumer repo")
    e.add_argument("variant", choices=sorted(VARIANTS))
    e.add_argument("--dest", required=True)
    e.add_argument("--mesh-prefix", default="")
    e.add_argument("--check", action="store_true",
                   help="verify an existing export instead of writing it")
    e.add_argument("--with-assets", action="store_true",
                   help="also collect the withheld CAD, from a fetched "
                        "checkout or $MKIT_ASSETS_DIR (internal use)")
    e.set_defaults(func=cmd_export)

    v = sub.add_parser("variants", help="list the variants")
    v.set_defaults(func=cmd_variants)

    fa = sub.add_parser("fetch-assets",
                        help="fetch the withheld CAD geometry")
    fa.add_argument("--from", dest="source", required=True,
                    metavar="PATH-OR-GIT-URL",
                    help="a manipulation-kit-assets checkout, or a git URL "
                         "to clone")
    fa.set_defaults(func=cmd_fetch_assets)

    fv = sub.add_parser("fetch-visuals",
                        help="fetch the optional decorative body meshes")
    fv.add_argument("--from-d1-sdk", required=True, metavar="DIR")
    fv.set_defaults(func=cmd_fetch_visuals)

    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
