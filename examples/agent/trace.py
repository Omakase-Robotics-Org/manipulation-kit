"""One JSONL record per decision: what was offered, what was chosen, what happened.

The field that matters is the last one. ``claimed`` is what the model said;
``verdict`` is what the kit MEASURED. Writing them side by side is the whole
point — a run where those two columns disagree is a run you can learn from, and
a log that only keeps the model's own account is a log that will tell you the
robot succeeded every time.

Append-only JSONL so a run can be tailed live and replayed afterwards.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Any, Dict, List, Optional


@dataclass
class DecisionRecord:
    """One turn of a loop."""

    iteration: int
    world: Dict[str, Any]
    offered: List[Dict[str, Any]] = field(default_factory=list)
    refused: List[Dict[str, Any]] = field(default_factory=list)
    #: what the model asked for, verbatim, before the kit touched it
    choice: Optional[Dict[str, Any]] = None
    #: per-choice probability, when the model exposes one (Jev does)
    distribution: Optional[Dict[str, float]] = None
    plan: Optional[Dict[str, Any]] = None
    run: Optional[Dict[str, Any]] = None
    #: the model's own claim of completion. Recorded, never believed.
    claimed: Optional[str] = None
    #: the measured verdict for THIS primitive, from a verifier over a LATER world
    verdict: Optional[Dict[str, Any]] = None
    #: the measured verdict for the TASK, which is the one a claim of "done"
    #: is actually about. A primitive can succeed on a turn that leaves the
    #: task unfinished, and a model saying "done" then is wrong about the task,
    #: not about the primitive.
    goal_verdict: Optional[Dict[str, Any]] = None
    stamp: float = field(default_factory=time.time)

    def to_json(self) -> Dict[str, Any]:
        return asdict(self)


class DecisionTrace:
    """Append-only JSONL writer. Also usable in memory, for a test."""

    def __init__(self, path: Optional[Path] = None):
        self.path = Path(path) if path else None
        self.records: List[DecisionRecord] = []
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, record: DecisionRecord) -> DecisionRecord:
        self.records.append(record)
        if self.path is not None:
            with self.path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(record.to_json(), default=str) + "\n")
        return record

    def disagreements(self) -> List[DecisionRecord]:
        """Turns where the model claimed success and the measurement did not.

        The first report worth reading off a run.
        """
        out = []
        for record in self.records:
            claimed = (record.claimed or "").strip().lower() in ("done", "true", "yes")
            measured = (record.goal_verdict or record.verdict or {}).get("verdict")
            if claimed and measured != "true":
                out.append(record)
        return out

    def summary(self) -> Dict[str, Any]:
        verdicts: Dict[str, int] = {}
        for record in self.records:
            key = (record.verdict or {}).get("verdict", "none")
            verdicts[key] = verdicts.get(key, 0) + 1
        return {"turns": len(self.records), "verdicts": verdicts,
                "refusals": sum(len(r.refused) for r in self.records),
                "claimed_but_unmeasured": len(self.disagreements())}
