"""``mkit-urdf`` — build, export and complete the D1 description.

    mkit-urdf build [--only d1.urdf]          regenerate the URDFs in place
    mkit-urdf export <variant> --dest DIR     vendor a variant, with provenance
    mkit-urdf export <variant> --dest DIR --check   fail on drift
    mkit-urdf variants                        list what can be built
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
import sys
from pathlib import Path

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
        return ex.check(variant.flavour, dest, args.mesh_prefix)
    manifest = ex.export(variant.flavour, dest, args.mesh_prefix)
    print(f"exported {variant.name}: {len(manifest['files'])} files -> {dest}")
    print(f"source commit {manifest['source_commit']}"
          + (" (DIRTY)" if manifest["source_dirty"] else ""))
    if manifest["absent_optional"]:
        print(f"{len(manifest['absent_optional'])} optional visual mesh(es) "
              f"absent by design — `mkit-urdf fetch-visuals` to complete them")
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
    e.set_defaults(func=cmd_export)

    v = sub.add_parser("variants", help="list the variants")
    v.set_defaults(func=cmd_variants)

    fv = sub.add_parser("fetch-visuals",
                        help="fetch the optional decorative body meshes")
    fv.add_argument("--from-d1-sdk", required=True, metavar="DIR")
    fv.set_defaults(func=cmd_fetch_visuals)

    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
