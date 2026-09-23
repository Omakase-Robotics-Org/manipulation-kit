"""Check-then-write: the old panel's Save, minus the omakaseos file moves.

omakase-core ``teach.py::save_teach_session`` ran the Python safety validator
on the staged CSV and refused an unsafe one unless ``force`` ("Force-save even
if unsafe (will need --no-safety to play)"). Same rule here; a force-saved
file carries ``# mkit-teach: UNSAFE=<violation>`` lines, which
:func:`manipulation_kit.teach.play.play` refuses without ``no_safety``.
"""
from __future__ import annotations

import hashlib
import json
from typing import Optional, Sequence, Tuple

from .check import CheckReport, check_gesture
from .gesture_csv import Gesture, Keyframe
from .registry import USAGES, check_name


class UnsafeGesture(RuntimeError):
    def __init__(self, report: CheckReport):
        self.report = report
        super().__init__("refusing to write an unsafe gesture (re-teach, or "
                         "--force to write it flagged UNSAFE):\n" + report.summary())


def home_digest(home: Sequence[float]) -> str:
    """A short fingerprint of the HOME a CSV pins (GESTURES.md: a CSV pins the
    HOME it was recorded against; changing home_pose.json means re-fitting)."""
    raw = json.dumps([round(float(v), 4) for v in home]).encode()
    return hashlib.sha256(raw).hexdigest()[:12]


def export(gesture: Gesture, home: Sequence[float], *, name: Optional[str] = None,
           sentiment: str = "neutral", usage: Sequence[str] = ("filler",),
           force: bool = False, check_kwargs=None, extra_meta=None
           ) -> Tuple[Gesture, CheckReport]:
    """Validate and stamp ``gesture``. Raises :class:`UnsafeGesture` when the
    check fails and ``force`` is not set."""
    report = check_gesture(gesture, home, **(check_kwargs or {}))
    if not report.ok and not force:
        raise UnsafeGesture(report)
    meta = {}
    if name:
        meta["name"] = check_name(name)
    meta.update({"sentiment": sentiment,
                 "usage": " ".join(u for u in usage if u in USAGES) or "filler",
                 "source": "teach", "home_sha": home_digest(home)})
    meta.update(extra_meta or {})
    out = Gesture([Keyframe(k.duration, k.positions) for k in gesture.keyframes],
                  meta=meta,
                  unsafe=([] if report.ok else list(report.violations[:5])))
    return out, report
