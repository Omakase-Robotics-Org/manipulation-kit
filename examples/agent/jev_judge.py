"""Jev-Omni as the wrist-look judge: a local and a remote judge, and an
offline re-judge of a recorded run's wrist photos.

The kit owns the choice ids, the mark, the direction and the step
(``manipulation_kit.agent.servo``) and the model-neutral seam — typed
questions, mirrored views (``manipulation_kit.agent.judge``). What Jev is
asked is ``jev_questions.py``; what is here is where the classifier runs.
Both judges take ``formulation=`` (a name in ``jev_questions.NAMES``) and
``views=`` (upright, flip_v, flip_h, rot180).

    JevJudge()                 the 12B classifier in THIS process (a GPU with
                               ~47 GB free, measured; loaded on first call)
    RemoteJudge(url)           the same behind ``jev_judge_server.py``, for a
                               robot whose Jetson cannot hold it (160-194 ms
                               warm on an RTX PRO 6000, + the network)
    rejudge(run_dir, judge)    a live run's saved wrist photos, marked where
                               the trace's objects project, judged again
"""

from __future__ import annotations

import base64
import json
import sys
import time
import urllib.request
from pathlib import Path
from typing import Any, Callable, Dict, Mapping, Optional

import numpy as np

from manipulation_kit.agent import Servo, ServoLook
from manipulation_kit.agent.judge import DEFAULT_VIEWS, AskingJudge
from jev_questions import FORMULATIONS, LABELS  # noqa: F401


def _asking(formulation: str, views, debug: bool) -> Dict[str, Any]:
    """``AskingJudge`` keyword arguments from the examples' flags."""
    if formulation not in FORMULATIONS:
        raise ValueError(f"formulation must be one of {tuple(FORMULATIONS)}")
    return {"formulation": FORMULATIONS[formulation], "views": tuple(views),
            "log": ((lambda line: print(f"[jev_judge] {line}",
                                        file=sys.stderr)) if debug else None)}

class JevJudge(AskingJudge):
    """Jev-Omni in this process. Loaded on first use."""

    def __init__(self, debug: bool = False, *, formulation: str = "choice",
                 views=DEFAULT_VIEWS):
        super().__init__(**_asking(formulation, views, debug))
        self.classifier = None

    def load(self):
        if self.classifier is None:
            from huggingface_hub import snapshot_download  # noqa: PLC0415
            sys.path.insert(0, snapshot_download("akhilaaa3/Jev-Omni"))
            from jev_omni import load_jev_omni  # noqa: PLC0415
            self.classifier = load_jev_omni()
        return self.classifier

    def _ask(self, image, state, items):
        started, out = time.time(), []
        for q in items:
            result = self.load().predict(state=state, question=q.question,
                                         options=list(q.options),
                                         media=str(image), modality="image")
            out.append([float(result["probabilities"][o]) for o in q.options])
        return out, (time.time() - started) * 1000.0


class RemoteJudge(AskingJudge):
    """The ``judge(look)`` seam over HTTP: POST the marked photo and the
    words to ``jev_judge_server.py``, get a probability per option back.
    Every question about one view goes in ONE ``/judge_batch`` request when
    the server's ``/health`` says ``"batch": true``; an older server is
    asked one ``/judge`` per question."""

    def __init__(self, url: str, *, timeout_s: float = 10.0,
                 debug: bool = False, formulation: str = "choice",
                 views=DEFAULT_VIEWS):
        super().__init__(**_asking(formulation, views, debug))
        self.base = url.rstrip("/")
        self.url = self.base + "/judge"
        self.timeout_s = float(timeout_s)
        self.batch: Optional[bool] = None       # unknown until first asked

    def _post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        request = urllib.request.Request(
            self.base + path, data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(request, timeout=self.timeout_s) as reply:
            return json.loads(reply.read())

    def _ask(self, image, state, items):
        photo = base64.b64encode(Path(image).read_bytes()).decode("ascii")
        started = time.time()
        if self.batch is None:      # a server says whether it takes batches
            with urllib.request.urlopen(self.base + "/health",
                                        timeout=self.timeout_s) as reply:
                self.batch = bool(json.loads(reply.read()).get("batch"))
        if self.batch:
            answer = self._post("/judge_batch", {
                "state": state, "image_jpeg_b64": photo,
                "items": [{"question": q.question, "options": list(q.options)}
                          for q in items]})["answers"]
        else:
            answer = [self._post("/judge", {
                "state": state, "question": q.question,
                "options": list(q.options), "image_jpeg_b64": photo})
                for q in items]
        return ([[float(a["probabilities"][o]) for o in q.options]
                 for q, a in zip(items, answer)],
                (time.time() - started) * 1000.0)


def _objects(world: Mapping[str, Any]):
    """The trace's base-frame objects as ObjectViews (surfaces skipped)."""
    from scipy.spatial.transform import Rotation as R  # noqa: PLC0415
    from manipulation_kit.world import ObjectView  # noqa: PLC0415
    for o in world.get("objects", []):
        if o.get("kind") == "surface" or o.get("frame_id", "base") != "base":
            continue
        yield ObjectView(o["name"], p=o["p"], size=o["size"],
                         r=R.from_quat(o.get("quat_xyzw", [0, 0, 0, 1])),
                         kind=o.get("kind", "object"))


def _line(where: str, look: ServoLook, dist: Dict[str, float]) -> str:
    top = sorted(dist.items(), key=lambda kv: -kv[1])[:3]
    return (f"{where}: mark ({look.u:.0f}, {look.v:.0f}) -> "
            f"{top[0][0]} {top[0][1]:.2f}   "
            + "  ".join(f"{c}={p:.2f}" for c, p in top))


def rejudge(run_dir: Path, judge: Callable, *, kin, wrist: Mapping[str, Any],
            only: Optional[str] = None, say=print) -> int:
    """Judge a recorded run's wrist photos again, nothing moved. Returns the
    number of judgements. ``wrist`` = ``{side: WristCamera.from_flange
    kwargs}``; ``only`` restricts to one object name."""
    from manipulation_kit.perception import WristCamera  # noqa: PLC0415
    from manipulation_kit.world import FrameGraph  # noqa: PLC0415
    run_dir = Path(run_dir)
    servo, frames, n = Servo(lambda side: None, judge,
                             out_dir=run_dir / "rejudge"), FrameGraph(), 0
    for line in (run_dir / "trace.jsonl").read_text().splitlines():
        record = json.loads(line)
        turn, world = record["iteration"], record.get("world") or {}
        gaps = {g["side"]: (g["jaw_gap_m"], "measured") for g in
                world.get("grippers", []) if g.get("jaw_gap_m") is not None}
        for arm in world.get("arms", []):
            side = arm["side"]
            photo = run_dir / f"turn{turn}_{side}_wrist_0_rgb.jpg"
            if not photo.exists() or side not in wrist:
                continue
            saved = np.array(kin.joints(side), dtype=float)
            try:
                kin.set_joints(side, np.radians(arm["joints_deg"]))
                camera = WristCamera.from_kin(kin, side, **wrist[side])
            finally:
                kin.set_joints(side, saved)
            for item in _objects(world):
                if only and item.name != only:
                    continue
                look = servo.look_at(camera, item, frames, photo, side=side,
                                     gap=gaps.get(side))
                if look is None:
                    say(f"turn {turn} {side}_wrist {item.name}: not in frame")
                    continue
                dist, _choice, _conf = servo.ask(look)
                n += 1
                say(_line(f"turn {turn} {side}_wrist {item.name}", look, dist))
        for i, step in enumerate((record.get("servo") or {}).get("steps", [])):
            n += _rejudge_step(servo, run_dir, turn, i, step, say)
    return n


def _rejudge_step(servo: Servo, run_dir: Path, turn: int, i: int,
                  step: Mapping[str, Any], say) -> int:
    """A servo judgement recorded in the trace: its photo, re-marked with the
    RECORDED mark and box, asked again beside the recorded answer."""
    from manipulation_kit.agent.servo import mark  # noqa: PLC0415
    look = step["look"]
    photo = look.get("photo")
    if photo and not Path(photo).exists():
        photo = run_dir / Path(photo).name      # recorded relative to the cwd
    if not photo or not Path(photo).exists():
        return 0
    image = mark(Path(photo), look["u"], look["v"],
                 run_dir / "rejudge" / f"turn{turn}_servo{i}_marked.jpg",
                 box_px=look.get("box_px"), radius_px=look.get("radius_px", 0))
    again = ServoLook(side=look["side"], object=look["object"], camera=None,
                      u=look["u"], v=look["v"], depth_m=look["depth_m"],
                      image=image, photo=Path(photo))
    dist, _choice, _conf = servo.ask(again)
    say(_line(f"turn {turn} servo step {i} (was {step['choice']} "
              f"{step['confidence']:.2f})", again, dist))
    return 1
