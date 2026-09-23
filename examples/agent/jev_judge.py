"""Jev-Omni as the wrist-look judge: the words, a local and a remote judge,
and an offline re-judge of a recorded run's wrist photos.

The kit owns the choice ids, the mark, the direction and the step
(``manipulation_kit.agent.servo``). What is here is the part that knows a
model exists: how the question is phrased, where the classifier runs.

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
from typing import Any, Callable, Dict, Mapping, Optional, Tuple

import numpy as np

from manipulation_kit.agent import Servo, ServoLook

#: what each kit choice id is called in the question. The ids are the kit's.
LABELS: Dict[str, str] = {
    "on": "the {object} is entirely inside the green box",
    "left": "the {object} sticks out to the LEFT of the green box",
    "right": "the {object} sticks out to the RIGHT of the green box",
    "above": "the {object} sticks out ABOVE the green box",
    "below": "the {object} sticks out BELOW the green box",
    "not_visible": "the {object} is not in the photo"}

STATE = ("Photo from the camera on a robot hand, looking along the hand past "
         "its two gripper fingers. A green box has been drawn on the photo "
         "where the robot BELIEVES the {object} is, slightly larger than the "
         "{object} should appear; a green cross marks the box's centre.")
QUESTION = "How does the {object} sit relative to the green box?"


def words(look: ServoLook) -> Tuple[str, str, Dict[str, str]]:
    """(state, question, {choice id: option label}) for one look."""
    name = look.object.replace("_", " ")
    return (STATE.format(object=name), QUESTION.format(object=name),
            {c: LABELS[c].format(object=name) for c in look.choices})


def _by_choice(labels: Dict[str, str], probabilities: Mapping[str, float]):
    by_label = {label: c for c, label in labels.items()}
    return {by_label[k]: float(p) for k, p in probabilities.items()}


class JevJudge:
    """Jev-Omni in this process. Loaded on first use."""

    def __init__(self, debug: bool = False):
        self.classifier, self.debug = None, debug

    def load(self):
        if self.classifier is None:
            from huggingface_hub import snapshot_download  # noqa: PLC0415
            sys.path.insert(0, snapshot_download("akhilaaa3/Jev-Omni"))
            from jev_omni import load_jev_omni  # noqa: PLC0415
            self.classifier = load_jev_omni()
        return self.classifier

    def __call__(self, look: ServoLook) -> Dict[str, float]:
        if look.image is None:
            raise RuntimeError("Jev judges a photo; this robot gave none "
                               "(--snapshot-cmd)")
        state, question, labels = words(look)
        started = time.time()
        result = self.load().predict(
            state=state, question=question, options=list(labels.values()),
            media=str(look.image), modality="image")
        out = _by_choice(labels, result["probabilities"])
        if self.debug:
            print(f"[jev_judge] {(time.time() - started) * 1000:.0f} ms "
                  f"{look.image.name}: {out}", file=sys.stderr)
        return out


class RemoteJudge:
    """The ``judge(look)`` seam over HTTP: POST the marked photo and the
    words to ``jev_judge_server.py``, get a probability per option back."""

    def __init__(self, url: str, *, timeout_s: float = 10.0,
                 debug: bool = False):
        self.url = url.rstrip("/") + "/judge"
        self.timeout_s, self.debug = float(timeout_s), debug

    def __call__(self, look: ServoLook) -> Dict[str, float]:
        if look.image is None:
            raise RuntimeError("the remote judge judges a photo; this robot "
                               "gave none (--snapshot-cmd)")
        state, question, labels = words(look)
        body = json.dumps({
            "state": state, "question": question,
            "options": list(labels.values()),
            "image_jpeg_b64": base64.b64encode(
                Path(look.image).read_bytes()).decode("ascii")}).encode()
        started = time.time()
        request = urllib.request.Request(
            self.url, data=body, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(request, timeout=self.timeout_s) as reply:
            answer = json.loads(reply.read())
        out = _by_choice(labels, answer["probabilities"])
        if self.debug:
            print(f"[jev_judge] remote {(time.time() - started) * 1000:.0f} ms "
                  f"(server {answer.get('ms', '?')} ms) {look.image.name}: "
                  f"{out}", file=sys.stderr)
        return out


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
                look = servo.look_at(camera, item, frames, photo, side=side)
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
