"""Export a hand's tool physical config as the JSON d1-sdk consumes.

Usage::

    python -m manipulation_kit.hands.export_tool_config <maker>/<model> <out.json>
    python -m manipulation_kit.hands.export_tool_config leadshine/dh116s -   # stdout

The written document is the schema ``omakase_arm::ToolConfig::fromJsonFile``
(d1-sdk) reads: a ``kinematics`` array of 6 (TCP x,y,z mm + rx,ry,rz deg)
and a ``dynamics`` array of 10 (mass kg, COM x,y,z mm, inertia
Ixx,Ixy,Ixz,Iyy,Iyz,Izz kg*m^2), plus provenance fields for humans.
"""

from __future__ import annotations

import sys

from . import get_tool_config


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else list(argv)
    if len(argv) != 2:
        print(__doc__.strip(), file=sys.stderr)
        return 2
    model, out_path = argv
    text = get_tool_config(model).to_json()
    if out_path == "-":
        sys.stdout.write(text)
    else:
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(text)
        print(f"wrote {model} tool config -> {out_path}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
