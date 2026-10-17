"""``mkit-retarget`` — a hand's retarget map as data, and its golden vectors.

    mkit-retarget list
    mkit-retarget table <maker>/<model> [out.json|-]     # default map's table
    mkit-retarget golden <maker>/<model> [out.json|-]    # its golden vectors
    mkit-retarget build                                  # rewrite the committed files
    mkit-retarget check                                  # exit 1 if any differs

The committed files are ``hands/<maker>/<model>/retarget_table.json`` and
``retarget_golden.json`` (:mod:`manipulation_kit.hands.mapping`). **The code is
the source of truth**: regenerate them with ``build``, never hand-edit one.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .mapping import golden_dumps, retarget_table

#: Hands with a version-2 retarget map, i.e. what this command accepts.
MODELS = ("leadshine/dh116s", "robotera/xhand1", "robotera/xhand1_lite", "wuji/wujihand2")

TABLE_FILE = "retarget_table.json"
GOLDEN_FILE = "retarget_golden.json"


def model_dir(model: str) -> Path:
    """The package directory of ``model`` (``hands/<maker>/<model>``)."""
    maker, _, name = model.partition("/")
    return Path(__file__).resolve().parent / maker / name


def expected_files(model: str):
    """``(path, text)`` for each committed file of ``model``."""
    d = model_dir(model)
    return ((d / TABLE_FILE, retarget_table(model).dumps()),
            (d / GOLDEN_FILE, golden_dumps(model)))


def _write(text: str, out: str) -> None:
    if out == "-":
        sys.stdout.write(text)
    else:
        Path(out).write_text(text, encoding="utf-8")


def cmd_list(_args) -> int:
    for model in MODELS:
        t = retarget_table(model)
        print(f"{model:<24} {len(t.rows):>2} joints  {sum(t.held())} held  "
              f"{len(t.channels()):>2} channels  {t.digest()[:16]}")
    return 0


def cmd_table(args) -> int:
    _write(retarget_table(args.model).dumps(), args.out)
    return 0


def cmd_golden(args) -> int:
    _write(golden_dumps(args.model), args.out)
    return 0


def cmd_build(_args) -> int:
    for model in MODELS:
        for path, text in expected_files(model):
            path.write_text(text, encoding="utf-8")
            print(f"wrote {path}", file=sys.stderr)
    return 0


def cmd_check(_args) -> int:
    stale = [str(path) for model in MODELS for path, text in expected_files(model)
             if not path.exists() or path.read_text(encoding="utf-8") != text]
    for path in stale:
        print(f"stale: {path} (run mkit-retarget build)", file=sys.stderr)
    return 1 if stale else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="mkit-retarget", description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list", help="the hands with a table").set_defaults(func=cmd_list)
    for name, func, what in (("table", cmd_table, "the default map's table"),
                             ("golden", cmd_golden, "the default map's golden vectors")):
        p = sub.add_parser(name, help=f"write {what} as JSON")
        p.add_argument("model", choices=MODELS, metavar="<maker>/<model>")
        p.add_argument("out", nargs="?", default="-", help="output path, or '-' (default)")
        p.set_defaults(func=func)
    sub.add_parser("build", help="rewrite the committed files").set_defaults(func=cmd_build)
    sub.add_parser("check", help="exit 1 if a committed file is stale").set_defaults(func=cmd_check)
    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
