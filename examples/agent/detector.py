"""``AstraDetector``: a model as the box detector — the model glue, and only it.

Everything a detection turns into — footprint, height, width, the scene file
— is :mod:`manipulation_kit.perception`'s. This file is the part that talks to
a model: the prompt, the Responses-API call, and a strict parse of the reply
into :class:`~manipulation_kit.perception.Detection` s (pixels only; the
detector's job ends at the image).

It is a :class:`~manipulation_kit.perception.Perceiver` by subclassing the
kit's :class:`~manipulation_kit.perception.ScenePerceiver`, so ``locate`` and
``declare`` are the kit's rules, not a second copy of them.

    from detector import AstraDetector
    found = AstraDetector(model="gpt-6-astra").detect(image_rgb, ["cup"])
"""

from __future__ import annotations

import json
import os
import re
from typing import Any, Dict, List, Sequence

import numpy as np

from manipulation_kit.perception import Detection, ScenePerceiver

ASTRA_PROMPT = """You are given ONE photograph from a robot's head camera,
looking down and forward at a table.

For each object named below, report where it is IN PIXELS. Reply with STRICT
JSON and nothing else - no prose, no markdown fence:

{"objects": [{"name": "<the requested name, verbatim>",
              "bbox": [x0, y0, x1, y1],
              "base_px": [u, v],
              "upright": true,
              "shape": "box" | "cylinder" | "other",
              "kind": "object" | "container",
              "confidence": 0.0-1.0}]}

  bbox     the object's tight pixel bounding box, x right, y down, origin at
           the top-left of the image.
  base_px  the pixel where the object's FOOTPRINT touches the table: the
           bottom-centre of its contact patch, NOT the bottom of the bounding
           box if the object overhangs, and NOT the centre of the object.
           This single pixel decides where the robot reaches, so look at it.
  upright  false if the object has tipped over or is lying on its side.
  kind     "container" if the robot could put something INSIDE it.

Omit an object you cannot see; do not invent one. The image is %(width)d x
%(height)d pixels.

The objects: %(names)s"""


def parse_reply(text: str, requested: Sequence[Dict[str, str]]
                 ) -> List[Detection]:
    """Strict JSON, leniently extracted, strictly validated.

    Models fence their JSON, prefix it with "Here is", and occasionally return
    the list without the wrapper object. All three are recoverable and none of
    them is a reason to lose a robot run; a bbox that is not four numbers is
    NOT recoverable and raises.
    """
    payload = text.strip()
    fence = re.search(r"```(?:json)?\s*(.*?)```", payload, re.S)
    if fence:
        payload = fence.group(1).strip()
    start = min((i for i in (payload.find("{"), payload.find("[")) if i >= 0),
                default=-1)
    if start < 0:
        raise ValueError(f"no JSON in the reply: {text[:200]!r}")
    decoded, _ = json.JSONDecoder().raw_decode(payload[start:])
    items = decoded["objects"] if isinstance(decoded, dict) else decoded
    if not isinstance(items, list):
        raise ValueError(f"expected a list of objects, got {type(items)}")
    wanted = {item["name"]: item for item in requested}
    out: List[Detection] = []
    for raw in items:
        name = str(raw.get("name", ""))
        if name not in wanted:
            continue
        bbox = [float(v) for v in raw["bbox"]]
        if len(bbox) != 4:
            raise ValueError(f"{name}: bbox must be 4 numbers, got {raw['bbox']}")
        x0, y0, x1, y1 = bbox
        base = raw.get("base_px") or [(x0 + x1) / 2.0, y1]
        notes = []
        if not raw.get("base_px"):
            notes.append("no base_px in the reply; the bbox's bottom-centre "
                         "was used and an overhang would bias it")
        out.append(Detection(
            name=name, kind=str(raw.get("kind") or wanted[name]["kind"]),
            bbox=(min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)),
            base_px=(float(base[0]), float(base[1])),
            upright=bool(raw.get("upright", True)),
            shape=str(raw.get("shape", "other")),
            colour=wanted[name].get("colour"),
            confidence=float(raw.get("confidence", 0.6)),
            source="astra", notes=notes))
    missing = sorted(set(wanted) - {d.name for d in out})
    if missing:
        raise ValueError(f"the reply does not contain {missing}")
    return out


def detect_objects(image, names: Sequence[Any], *, model: str = "gpt-6-astra",
                   client=None, order: str = "rgb",
                   retries: int = 1) -> List[Detection]:
    """Name the objects in one frame with one Responses-API call.

    ``names`` may be plain strings or the ``{"name", "kind", "colour"}`` dicts
    the CLI builds. ``order`` says whether ``image`` is RGB (this module's
    convention, and Pillow's) or BGR (OpenCV's) — getting it wrong is silent
    and turns a brown cup blue, so it is a parameter rather than a guess.

    One retry on a reply that is not parseable, with the parse error fed back;
    a second failure raises, because a detector that keeps inventing scenes
    until one parses is worse than one that stops.
    """
    requested = [{"name": n, "kind": "object", "colour": None}
                 if isinstance(n, str) else dict(n) for n in names]
    array = np.asarray(image)
    if order.lower() == "bgr":
        array = array[:, :, ::-1]
    height, width = array.shape[:2]
    prompt = ASTRA_PROMPT % {"width": width, "height": height,
                             "names": ", ".join(i["name"] for i in requested)}
    if client is None:                          # pragma: no cover - network
        from openai import OpenAI  # noqa: PLC0415
        client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))
    content: List[Dict[str, Any]] = [
        {"type": "input_text", "text": prompt},
        {"type": "input_image", "detail": "high",
         "image_url": "data:image/jpeg;base64," + _jpeg_base64(array)}]
    messages = [{"role": "user", "content": content}]
    last = None
    for attempt in range(retries + 1):
        response = client.responses.create(model=model, input=messages)
        text = getattr(response, "output_text", "") or ""
        try:
            return parse_reply(text, requested)
        except (ValueError, KeyError, TypeError, IndexError) as exc:
            last = exc
            if attempt >= retries:
                break
            messages = messages + [
                {"role": "assistant", "content": text},
                {"role": "user", "content":
                    f"That was not usable: {exc}. Reply with the JSON object "
                    f"described above and nothing else."}]
    raise ValueError(f"the detector never returned usable JSON: {last}")


def _jpeg_base64(rgb: np.ndarray) -> str:
    import base64  # noqa: PLC0415
    import io  # noqa: PLC0415

    from PIL import Image  # noqa: PLC0415
    buffer = io.BytesIO()
    Image.fromarray(np.asarray(rgb, dtype=np.uint8)).save(
        buffer, format="JPEG", quality=92)
    return base64.b64encode(buffer.getvalue()).decode("ascii")


class AstraDetector(ScenePerceiver):
    """The Astra box detector as a :class:`Perceiver`.

    ``cameras`` / ``world`` are the kit's :class:`ScenePerceiver` state (so
    ``locate`` and ``declare`` work); :meth:`detect` is the one thing this
    class adds.
    """

    def __init__(self, *, model: str = "gpt-6-astra", client=None,
                 cameras=None, world=None, retries: int = 1):
        super().__init__(cameras=cameras, world=world)
        self.model = model
        self.client = client
        self.retries = retries

    def detect(self, image, names: Sequence[Any], *,
               order: str = "rgb") -> List[Detection]:
        return detect_objects(image, names, model=self.model,
                              client=self.client, order=order,
                              retries=self.retries)


__all__ = ["ASTRA_PROMPT", "AstraDetector", "detect_objects", "parse_reply"]
