"""One JSONL record per decision: what was offered, what was chosen, what happened.

The field that matters is the last one. ``claimed`` is what the model said;
``verdict`` is what the kit MEASURED. Writing them side by side is the whole
point — a run where those two columns disagree is a run you can learn from, and
a log that only keeps the model's own account is a log that will tell you the
robot succeeded every time.

Append-only JSONL so a run can be tailed live and replayed afterwards.

Moved from ``examples/agent/trace.py`` into the wheel (design C.8) with two
additions a design review asked for (item 15): ``effective`` is the call AS
IT RAN — after the operator policy's cap — beside ``choice``, which stays the
model's request verbatim; and ``error`` records a turn that failed outside the
transport. :meth:`DecisionTrace.save_messages` writes the model's chat history
atomically, images replaced by their labels.
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Any, Dict, List, Optional


@dataclass
class DecisionRecord:
    """One turn of a loop."""

    iteration: int
    world: Dict[str, Any]
    #: the TASK this turn was about. A trace of choices with no goal in it
    #: cannot be argued with later.
    task: str = ""
    offered: List[Dict[str, Any]] = field(default_factory=list)
    refused: List[Dict[str, Any]] = field(default_factory=list)
    #: what the model asked for, verbatim, before the kit touched it
    choice: Optional[Dict[str, Any]] = None
    #: the call as it RAN — after the operator policy's cap — when that
    #: differs from nothing at all; ``choice`` is never rewritten
    effective: Optional[Dict[str, Any]] = None
    #: a wrist look taken on this turn (where the object should appear)
    look: Optional[Dict[str, Any]] = None
    #: per-choice probability, when the judge or model exposes one
    distribution: Optional[Dict[str, float]] = None
    #: the System 1 alignment taken on this turn in place of the wrist-look
    #: text (:class:`~manipulation_kit.agent.servo.ServoReport`): every
    #: judgement with its distribution, every correction with its run
    servo: Optional[Dict[str, Any]] = None
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
    #: the observation taken AFTER the action, so a record is replayable
    observation_after: Optional[Dict[str, Any]] = None
    #: why the loop ended, on the turn it ended
    stop: Optional[str] = None
    #: an exception that ended the turn outside the transport (repr)
    error: Optional[str] = None
    #: the contacts a probe / press MEASURED (``RunReport.contacts``), each a
    #: ``ContactReport.to_json()`` — where the hand met something, and why it
    #: stopped
    contacts: List[Dict[str, Any]] = field(default_factory=list)
    #: measurements that REPLACED a declaration this turn — a grasp that
    #: measured the object's width along the jaws (``width_correction``)
    corrections: List[Dict[str, Any]] = field(default_factory=list)
    stamp: float = field(default_factory=time.time)

    def to_json(self) -> Dict[str, Any]:
        return asdict(self)


class DecisionTrace:
    """Append-only JSONL writer. Also usable in memory, for a test."""

    def __init__(self, path: Optional[Path] = None):
        self.path = Path(path) if path else None
        self.records: List[DecisionRecord] = []
        self.task: str = ""
        self.stop: str = ""
        self.stop_detail: str = ""
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, record: DecisionRecord) -> DecisionRecord:
        self.records.append(record)
        if self.path is not None:
            with self.path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(record.to_json(), default=str) + "\n")
        return record

    def save_messages(self, messages: List[Dict[str, Any]]) -> Optional[Path]:
        """The model's whole chat history next to the trace, ATOMICALLY
        (write a sibling, then rename), so a crash mid-turn leaves the last
        complete copy rather than half of one. Image payloads are replaced by
        their ``_file`` label — the trace is not a second copy of the photos."""
        if self.path is None:
            return None
        path = self.path.with_suffix(".messages.json")

        def slim(message):
            content = message.get("content")
            if not isinstance(content, list):
                return message
            return dict(message, content=[
                dict(p, image_url=f"<{p.get('_file', 'image')}>")
                if p.get("type") == "input_image" else p for p in content])

        tmp = path.with_name(path.name + ".tmp")
        tmp.write_text(json.dumps([slim(m) for m in messages], indent=1,
                                  default=str), encoding="utf-8")
        os.replace(tmp, path)
        return path

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
        goals = [r.goal_verdict for r in self.records if r.goal_verdict]
        return {"task": self.task,
                "turns": len(self.records), "verdicts": verdicts,
                "refusals": sum(len(r.refused) for r in self.records),
                "claimed_but_unmeasured": len(self.disagreements()),
                # FOUR SEPARATE RESULTS, never one. Transport completion,
                # per-primitive success, task success and why the loop ended
                # are different claims and the old summary flattened them.
                "transport_completed": sum(
                    1 for r in self.records
                    if (r.run or {}).get("completed")),
                "goal_verdict": (goals[-1] or {}).get("verdict") if goals else None,
                "stop": self.stop, "stop_detail": self.stop_detail}
