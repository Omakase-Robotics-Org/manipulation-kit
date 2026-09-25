"""The servo's object outline from a text-prompted segmenter (phase 2):
``segment_server.py`` (SAM 3, or Grounded-SAM-2) behind HTTP, asked from the
robot with the standard library only — no torch in the wheel's environment.

    RemoteSegmenter(url)        a ``manipulation_kit.agent.jaws.ObjectOutline``:
                                ``(photo, look) -> Outline | None``
    Servo(..., object_outline=RemoteSegmenter("http://100.x.y.z:8767"))

A name the segmenter does not find (or a request that fails) is ``None``,
and the servo draws the declared shape instead and records that it did.
``prompts`` maps a scene name to the words sent (``{"tape": "roll of
tape"}``); by default the name with its underscores as spaces.
"""

from __future__ import annotations

import base64
import json
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

from manipulation_kit.agent.jaws import Outline, outline_from_points


class RemoteSegmenter:
    """``POST /segment`` per photo: the object's polygon as an
    :class:`~manipulation_kit.agent.jaws.Outline` (source ``segmented:<model>``,
    the segmenter's score), or ``None``."""

    def __init__(self, url: str, *, timeout_s: float = 10.0,
                 threshold: float = 0.3,
                 prompts: Optional[Mapping[str, str]] = None,
                 debug: bool = False):
        self.base = url.rstrip("/")
        self.timeout_s, self.threshold = float(timeout_s), float(threshold)
        self.prompts = dict(prompts or {})
        self.debug = bool(debug)
        #: the last answer, for the trace and the lab
        self.last: Dict[str, Any] = {}

    def health(self) -> Dict[str, Any]:
        with urllib.request.urlopen(self.base + "/health",
                                    timeout=self.timeout_s) as reply:
            return json.loads(reply.read())

    def segment(self, photo: Path, names) -> Dict[str, Any]:
        """The server's answer for ``names`` in ``photo``."""
        body = {"image_jpeg_b64": base64.b64encode(
            Path(photo).read_bytes()).decode("ascii"),
            "names": list(names), "threshold": self.threshold}
        request = urllib.request.Request(
            self.base + "/segment", data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(request, timeout=self.timeout_s) as reply:
            return json.loads(reply.read())

    def __call__(self, photo: Optional[Path], look) -> Optional[Outline]:
        if photo is None:
            return None
        prompt = self.prompts.get(look.object, look.object.replace("_", " "))
        try:
            answer = self.segment(photo, [prompt])
        except (OSError, urllib.error.URLError, ValueError) as exc:
            self.last = {"error": str(exc)}
            if self.debug:
                print(f"[segmenter] {exc}", file=sys.stderr)
            return None
        found = answer.get("objects", {}).get(prompt) or {}
        self.last = {"model": answer.get("model"), "ms": answer.get("ms"),
                     "found": bool(found.get("found")),
                     "score": found.get("score")}
        if not found.get("found") or len(found.get("polygon") or []) < 3:
            return None
        return outline_from_points(
            [tuple(p) for p in found["polygon"]],
            f"segmented:{answer.get('model', '?')}", score=found.get("score"),
            extra={"bbox": found.get("bbox"), "area_px": found.get("area_px"),
                   "prompt": prompt, "ms": answer.get("ms")}, hull=False)


def add_outline_flags(parser) -> None:
    """``--segment-url`` (phase 2) and ``--object-shape NAME=SHAPE``."""
    parser.add_argument("--segment-url", default=None, help="the object "
                        "outline from segment_server.py (default: the "
                        "declared shape)")
    parser.add_argument("--object-shape", action="append", default=[],
                        metavar="NAME=SHAPE", help="the declared shape of an "
                        "object (box, cylinder, other); also read from the "
                        "scene file's \"shape\" keys")


def outline_options(args) -> Dict[str, Any]:
    """``Servo`` keyword arguments from those flags: ``shapes`` (the scene
    file's ``"shape"`` keys, then the flags) and ``object_outline``."""
    shapes: Dict[str, str] = {}
    scene = getattr(args, "scene", None)
    if scene is not None and Path(scene).is_file():
        for item in json.loads(Path(scene).read_text()).get("objects", []):
            if item.get("shape"):
                shapes[item["name"]] = item["shape"]
    for pair in args.object_shape:
        name, _eq, shape = pair.partition("=")
        shapes[name] = shape
    out: Dict[str, Any] = {"shapes": shapes}
    if args.segment_url:
        out["object_outline"] = RemoteSegmenter(args.segment_url)
    return out
