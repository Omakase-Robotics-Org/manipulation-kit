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
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

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


def csv_dir(yaml_path: Path) -> Path:
    """Where the yaml's CSVs live: its ``csv_base_dir`` (relative to the yaml;
    omakase-core writes ``./csv``), ``csv/`` beside it when absent."""
    path = Path(yaml_path)
    for line in path.read_text(encoding="utf-8").splitlines():
        match = re.match(r"^csv_base_dir:\s*(\S+)\s*$", line)
        if match:
            base = Path(match.group(1).strip("'\""))
            return base if base.is_absolute() else (path.parent / base)
    return path.parent / "csv"


class UnsafeCsv(ValueError):
    """A CSV force-saved UNSAFE is not installed into omakaseos."""


def install(csv_path: Path, yaml_path: Path, *, sentiment: Optional[str] = None,
            usage: Optional[Sequence[str]] = None) -> Tuple[str, str, Path]:
    """Put an exported gesture CSV into an omakaseos checkout: copy it to the
    yaml's CSV directory as ``<name>_motion.csv`` and add / replace its entry,
    with the name, sentiment and usage its ``# mkit-teach:`` header carries
    (``sentiment`` / ``usage`` override). Refuses a CSV marked UNSAFE.
    Returns ``(how, name, destination)``."""
    from .gesture_csv import load_csv  # noqa: PLC0415
    source = Path(csv_path).expanduser().resolve()
    gesture = load_csv(source)
    if gesture.unsafe:
        raise UnsafeCsv(f"{source} was force-saved UNSAFE ("
                        + "; ".join(gesture.unsafe) + "); re-teach it before "
                        "installing it into omakaseos")
    stem = source.stem
    name = check_name(gesture.meta.get("name")
                      or (stem[:-len("_motion")] if stem.endswith("_motion") else stem))
    sentiment = sentiment or gesture.meta.get("sentiment") or "neutral"
    usage = list(usage or gesture.meta.get("usage", "filler").split() or ["filler"])
    yaml_path = Path(yaml_path).expanduser().resolve()
    target = csv_dir(yaml_path) / csv_filename(name)
    target.parent.mkdir(parents=True, exist_ok=True)
    how = register(yaml_path, name, sentiment=sentiment, usage=usage)
    if target != source:
        shutil.copyfile(source, target)
    return how, name, target


@dataclass(frozen=True)
class Entry:
    name: str
    csv: str
    source: str
    sentiment: str
    present: bool


def entries(yaml_path: Path) -> List[Entry]:
    """The ``gestures:`` entries of an omakaseos ``gesture.yaml`` (the layout
    omakase-core writes), each with whether its CSV exists."""
    path = Path(yaml_path).expanduser().resolve()
    base = csv_dir(path)
    out: List[Entry] = []
    current: Optional[dict] = None
    for line in path.read_text(encoding="utf-8").splitlines() + ["- name: _end"]:
        match = re.match(r"^- name:\s*(\S+)", line)
        if match:
            if current is not None:
                csv = current.get("csv", "")
                out.append(Entry(current["name"], csv, current.get("source", "-"),
                                 current.get("sentiment", "-"),
                                 bool(csv) and (base / csv).is_file()))
            current = {"name": match.group(1)}
            continue
        field = re.match(r"^  (csv|source|sentiment):\s*(\S+)", line)
        if field and current is not None:
            current[field.group(1)] = field.group(2)
    return out
