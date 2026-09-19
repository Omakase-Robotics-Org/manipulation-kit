"""Render the offer as a Jev-style typed-choice request. Runnable, no network.

Jev answers by CHOOSING, not by writing. It never sees an image and never emits
a number, so everything it needs has to be in two strings: the world as text and
a numbered list of already-bound, already-planned actions. This script prints
exactly what would be sent.

    python examples/agent/jev_menu.py
    python examples/agent/jev_menu.py --json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Optional, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parent))

from scene import demo_scene            # noqa: E402
from schema import choice_menu          # noqa: E402


def render(menu) -> str:
    lines = [menu["state"], "", menu["question"]]
    if not menu["choices"]:
        lines += ["", menu["nothing_offered"]]
        return "\n".join(lines)
    lines += [f"  {c['index']:2d}. {c['label']}" for c in menu["choices"]]
    if menu["refused"]:
        lines += ["", f"({len(menu['refused'])} other actions were considered and "
                      f"refused by the motion guard or the IK; they are in the "
                      f"trace, not in this menu — an unreachable option never "
                      f"becomes a word in the prompt.)"]
    return "\n".join(lines)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--cap", type=int, default=20)
    parser.add_argument("--json", action="store_true", help="emit the raw request")
    args = parser.parse_args(argv)

    world, kin = demo_scene()
    menu = choice_menu(world, kin, cap=args.cap)
    print(json.dumps(menu, indent=2, default=str) if args.json else render(menu))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
