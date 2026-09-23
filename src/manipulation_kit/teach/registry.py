"""The omakaseos ``gesture.yaml`` entry for a taught gesture.

Registration lived in omakase-core ``status_server/d1/teach.py::
_register_in_gesture_yaml``: after Save, ``{name, csv, sentiment, usage,
source: teach}`` was added to (or replaced in) ``robot_stack/robots/omakase/
d1/gesture.yaml`` and the CSV moved to ``csv/<name>_motion.csv``. The file is
omakaseos's; this module only produces that entry — as text, since the kit
does not depend on PyYAML — and can splice it into a checkout's file whose
layout is the one omakase-core writes (``gestures:`` list of ``- name:``
blocks, two-space indent). The gesture NAME omakaseos uses carries a ``d1_``
prefix (``d1_right_arm_1`` -> ``right_arm_1_motion.csv``).
"""
from __future__ import annotations

import re
from pathlib import Path
from typing import List, Sequence

#: the old panel: "Gesture name (a-z, 0-9, _, -)"
NAME_RE = re.compile(r"^[a-z0-9][a-z0-9_-]*$")
#: the old /d1_teach panel's choices (omakase-core static/d1_teach.html)
SENTIMENTS = ("neutral", "positive", "thoughtful")
USAGES = ("filler", "opening", "ending")


def check_name(name: str) -> str:
    if not NAME_RE.match(name or ""):
        raise ValueError(f"gesture name {name!r}: use a-z, 0-9, '_' and '-' "
                         f"(no dots, not starting with '_' or '-')")
    return name


def csv_filename(name: str) -> str:
    """``right_arm_1`` -> ``right_arm_1_motion.csv`` (the library's naming)."""
    return f"{check_name(name)}_motion.csv"


def entry_yaml(name: str, *, sentiment: str = "neutral",
               usage: Sequence[str] = ("filler",)) -> str:
    """The ``gesture.yaml`` block omakase-core's Save wrote, verbatim layout."""
    check_name(name)
    if sentiment not in SENTIMENTS:
        raise ValueError(f"sentiment must be one of {SENTIMENTS}")
    bad = [u for u in usage if u not in USAGES]
    if bad or not usage:
        raise ValueError(f"usage must be one or more of {USAGES}, got {list(usage)}")
    lines = [f"- name: d1_{name}", f"  csv: {csv_filename(name)}",
             f"  sentiment: {sentiment}", "  usage:"]
    lines += [f"  - {u}" for u in usage]
    lines.append("  source: teach")
    return "\n".join(lines) + "\n"


def register(yaml_path: Path, name: str, *, sentiment: str = "neutral",
             usage: Sequence[str] = ("filler",)) -> str:
    """Add or replace ``d1_<name>`` in an omakaseos ``gesture.yaml``. Returns
    ``"added"`` or ``"replaced"``. Refuses a file whose layout it does not
    recognise rather than guessing at YAML it cannot parse."""
    path = Path(yaml_path)
    text = path.read_text(encoding="utf-8")
    lines = text.splitlines()
    if "gestures:" not in [ln.rstrip() for ln in lines]:
        raise ValueError(f"{path}: no top-level 'gestures:' list")
    block = entry_yaml(name, sentiment=sentiment, usage=usage).splitlines()
    start = next((i for i, ln in enumerate(lines)
                  if ln.rstrip() == f"- name: d1_{name}"), None)
    if start is None:
        body: List[str] = lines + block
        how = "added"
    else:
        end = start + 1
        while end < len(lines) and lines[end].startswith("  "):
            end += 1
        body = lines[:start] + block + lines[end:]
        how = "replaced"
    path.write_text("\n".join(body) + "\n", encoding="utf-8")
    return how
