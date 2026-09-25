"""The harness boundary: ``src/manipulation_kit`` is the robot harness and
knows no agent. Models, prompts, question wording and judge formulations
live under ``examples/``; the seam the harness offers them is model-neutral
(``manipulation_kit.agent.judge``, ``manipulation_kit.agent.servo``)."""

from __future__ import annotations

import ast
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src" / "manipulation_kit"
EXAMPLES = ROOT / "examples"

#: agent-specific names that must not appear anywhere in the harness
#: (``astral`` — the uv installer's domain — is not one): the judge and
#: planner models, and the segmentation models behind the outline seam
AGENT_NAMES = re.compile(r"jev|astra(?!l)|openai|gpt|\bsam[ _-]?\d|"
                         r"grounded.?sam|grounding.?dino|segment.anything",
                         re.IGNORECASE)

#: files allowed to mention one anyway. It must stay empty: a new entry is a
#: boundary decision, not a test fix
ALLOWLIST: frozenset = frozenset()


def _harness_files():
    for path in sorted(SRC.rglob("*")):
        if not path.is_file() or "__pycache__" in path.parts:
            continue
        try:
            yield path, path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            continue                    # meshes, images


def test_the_allowlist_is_empty():
    assert ALLOWLIST == frozenset()


def test_the_harness_names_no_agent():
    hits = []
    for path, text in _harness_files():
        rel = str(path.relative_to(ROOT))
        if rel in ALLOWLIST:
            continue
        for number, line in enumerate(text.splitlines(), start=1):
            match = AGENT_NAMES.search(line)
            if match:
                hits.append(f"{rel}:{number}: {match.group(0)!r} in {line.strip()[:80]}")
    assert not hits, "agent-specific names in the harness:\n" + "\n".join(hits)


def _example_modules():
    return {p.stem for p in EXAMPLES.rglob("*.py")} | {"examples"}


def test_the_harness_never_imports_an_example():
    examples = _example_modules()
    hits = []
    for path in sorted(SRC.rglob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom) and not node.level:
                names = [node.module or ""]
            else:
                continue
            for name in names:
                if name.split(".")[0] in examples:
                    hits.append(f"{path.relative_to(ROOT)}:{node.lineno}: {name}")
        if "sys.path" in path.read_text(encoding="utf-8") and \
                "examples" in path.read_text(encoding="utf-8"):
            hits.append(f"{path.relative_to(ROOT)}: puts examples on sys.path")
    assert not hits, "the harness imports from examples:\n" + "\n".join(hits)


def test_the_pattern_catches_what_it_is_for():
    for word in ("Jev-Omni", "jev_judge", "AstraDetector", "astra_loop",
                 "OpenAI", "gpt-6", "SAM 3", "facebook/sam3", "sam2.1",
                 "Grounded-SAM-2", "GroundingDINO", "Segment Anything"):
        assert AGENT_NAMES.search(word), word
    for word in ("https://astral.sh/uv/install.sh", "the same shape",
                 "sample", "sampled"):
        assert not AGENT_NAMES.search(word), word
