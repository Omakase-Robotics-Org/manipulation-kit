"""The camera-grab contract: run a command, get FRESH, LABELLED frames — or stop.

Astra review 14: the old hook ignored the command's exit status, threw its
diagnostics away and attached whichever expected files existed — so a failed
grab showed the model no picture, or last turn's. ``Snapshotter`` checks the
return code, kills the whole process group on timeout, requires every frame to
be written AFTER the command started, and labels each camera in the text the
model reads. Any failure raises the loop's ``ObservationError``: no blind turn.

The command is a typed argument with two placeholders, not an env var::

    --snapshot-cmd "grab_frames.sh --turn {turn} --out {out_dir}"

and it must write ``turn{turn}_<camera>.jpg`` into ``{out_dir}`` for every
camera in ``cameras`` (default ``base_0_rgb`` = head, then both wrists).
"""

from __future__ import annotations

import base64
import os
import shlex
import signal
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from manipulation_kit.agent import ObservationError

#: file stem -> what the model is told the photo is
CAMERAS: Tuple[Tuple[str, str], ...] = (
    ("base_0_rgb", "HEAD camera"), ("right_wrist_0_rgb", "RIGHT wrist camera"),
    ("left_wrist_0_rgb", "LEFT wrist camera"))


class Snapshotter:
    def __init__(self, command: str, out_dir: Path, *,
                 cameras: Sequence[Tuple[str, str]] = CAMERAS,
                 timeout_s: float = 40.0):
        self.command, self.out_dir = str(command), Path(out_dir)
        self.cameras, self.timeout_s = tuple(cameras), float(timeout_s)

    def capture(self, turn: int) -> List[Tuple[str, Path]]:
        """``[(label, path)]`` for this turn, every one written just now."""
        self.out_dir.mkdir(parents=True, exist_ok=True)
        argv = [part.format(turn=turn, out_dir=self.out_dir)
                for part in shlex.split(self.command)]
        started = time.time()
        proc = subprocess.Popen(argv, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, start_new_session=True)
        try:
            out, _ = proc.communicate(timeout=self.timeout_s)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.communicate()
            raise ObservationError(f"snapshot command timed out after "
                                   f"{self.timeout_s:.0f}s: {argv}") from None
        if proc.returncode != 0:
            raise ObservationError(
                f"snapshot command exited {proc.returncode}: "
                f"{(out or b'').decode(errors='replace')[-400:]}")
        frames = []
        for stem, label in self.cameras:
            path = self.out_dir / f"turn{turn}_{stem}.jpg"
            if not path.exists() or path.stat().st_mtime < started - 1.0:
                raise ObservationError(f"no FRESH {label} frame at {path}: "
                                       f"the command did not write it this turn")
            frames.append((f"{label}, turn {turn}, captured "
                           f"{time.strftime('%H:%M:%S', time.localtime(path.stat().st_mtime))}",
                           path))
        return frames

    def observe(self, turn: int, world: Any = None) -> List[Dict[str, Any]]:
        """The loop's ``observe``: each photo preceded by its camera's name."""
        parts: List[Dict[str, Any]] = []
        for label, path in self.capture(turn):
            data = base64.b64encode(path.read_bytes()).decode("ascii")
            parts.append({"type": "input_text", "text": f"[{label}]"})
            parts.append({"type": "input_image", "detail": "high",
                          "image_url": f"data:image/jpeg;base64,{data}",
                          "_file": path.name})
        return parts


def wrist_frames(snapshotter):
    """``Servo``'s frame seam over the example's camera-grab contract: one
    fresh grab per look, that hand's wrist file out of it."""
    counter = {"n": 1000}

    def frame(side: str) -> Optional[Path]:
        counter["n"] += 1
        for _label, path in snapshotter.capture(counter["n"]):
            if path.name.endswith(f"_{side}_wrist_0_rgb.jpg"):
                return path
        return None
    return frame
