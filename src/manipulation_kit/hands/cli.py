"""``mkit-toolconfig`` — export a hand's physical registration data as JSON.

    mkit-toolconfig export <maker>/<model> out.json
    mkit-toolconfig export leadshine/dh116s -        # stdout
    mkit-toolconfig list

The written document is the schema an arm controller's set-tool call consumes:
a ``kinematics`` array of 6 (TCP x,y,z mm + rx,ry,rz deg) and a ``dynamics``
array of 10 (mass kg, COM x,y,z mm, inertia Ixx,Ixy,Ixz,Iyy,Iyz,Izz kg*m^2),
plus provenance fields for humans.

The JSON files committed under ``manipulation_kit/config/tool_configs/`` are
exports of exactly this. **The exporter is the source of truth** — regenerate
them, never hand-edit one, or the numbers on the robot stop being the numbers
anybody can find the derivation of.
"""
from __future__ import annotations

import argparse
import sys

from . import get_tool_config

#: Hands with tool data, i.e. what ``export`` accepts. Keeping it explicit
#: (rather than walking the package) means ``list`` cannot quietly start
#: advertising a half-finished hand.
MODELS = ("d1/parallel_gripper", "leadshine/dh116s", "linkerbot/o30")


def cmd_export(args) -> int:
    text = get_tool_config(args.model).to_json()
    if args.out == "-":
        sys.stdout.write(text)
    else:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(text)
        print(f"wrote {args.model} tool config -> {args.out}", file=sys.stderr)
    return 0


def cmd_list(_args) -> int:
    for model in MODELS:
        tc = get_tool_config(model)
        print(f"{model:<24} mass {tc.mass_kg:.3f} kg   "
              f"TCP {tuple(round(v, 1) for v in tc.kinematics()[:3])} mm")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="mkit-toolconfig",
                                 description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("export", help="write one hand's tool config as JSON")
    e.add_argument("model", choices=MODELS, metavar="<maker>/<model>")
    e.add_argument("out", help="output path, or '-' for stdout")
    e.set_defaults(func=cmd_export)
    ls = sub.add_parser("list", help="list the hands with tool data")
    ls.set_defaults(func=cmd_list)
    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
