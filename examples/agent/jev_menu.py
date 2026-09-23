"""Print the typed-choice request a Jev-class model would be sent. No network.

Jev answers by CHOOSING, not by writing. It never emits a number, so everything
this menu needs has to be in two strings: the world as text and a numbered list
of already-bound, already-planned actions — plus the TASK, which the old version
of this script never sent. (Jev-Omni also takes one image; the example that
uses that is ``jev_servo.py``, where it judges a wrist photo.)

    python examples/agent/jev_menu.py
    python examples/agent/jev_menu.py --task "put the red block in the box" --json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Optional, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parent))

from menu import choice_menu, render      # noqa: E402
from scene import demo_scene              # noqa: E402

DEFAULT_TASK = "put the red block in the box"


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--task", default=DEFAULT_TASK,
                        help="what the model is being asked to achieve")
    parser.add_argument("--cap", type=int, default=20)
    parser.add_argument("--json", action="store_true", help="emit the raw request")
    args = parser.parse_args(argv)

    world, kin = demo_scene()
    menu = choice_menu(world, kin, task=args.task, cap=args.cap)
    print(json.dumps(menu, indent=2, default=str) if args.json else render(menu))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
