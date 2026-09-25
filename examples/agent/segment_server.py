"""Text-prompted segmentation behind HTTP, for the wrist servo's object
outline (phase 2): the photo and the object names in, a polygon per name out.
Run it on a GPU workstation next to ``jev_judge_server.py`` and point
``jev_servo.py --segment-url`` at it::

    python examples/agent/segment_server.py --host 127.0.0.1 --port 8767
    # on the Tailscale address, so a robot can reach it:
    python examples/agent/segment_server.py --host 100.x.y.z --port 8767

``POST /segment`` takes ``{"image_jpeg_b64", "names": [...]}`` (optional
``"threshold"``, default 0.3) and answers ``{"objects": {name: {"found",
"polygon": [[u, v], ...], "score", "bbox": [u0, v0, u1, v1], "rle": {"size":
[h, w], "counts": [...]}, "area_px"}}, "ms", "model"}`` — per name the BEST
instance (highest score); ``found: false`` and no polygon when none clears
the threshold. The RLE is COCO's uncompressed form, column-major, starting
with a run of zeros. ``GET /health`` answers ``{"ok", "loaded", "model",
"device"}``.

Models, in order of preference (``--model auto``):

    sam3     Meta SAM 3 (``facebook/sam3``, gated on the Hub: log in or put a
             token in ``~/.cache/huggingface/token``), concept prompts —
             "tape" finds the tape — through ``transformers``
             (``Sam3Model`` / ``Sam3Processor``)
    gsam2    Grounded-SAM-2: GroundingDINO (``IDEA-Research/grounding-dino-
             base``) boxes from the text, SAM 2.1 (``facebook/sam2.1-hiera-
             large``) masks inside them

Needs torch, transformers, pillow, numpy and opencv-python-headless (for the
contour), not this kit. Requests are serialised: one GPU, one model. No
authentication: bind to loopback or to a private address, never to a public
interface.
"""

from __future__ import annotations

import argparse
import base64
import io
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional, Sequence

MAX_BODY_BYTES = 8 * 1024 * 1024
DEFAULT_THRESHOLD = 0.3
#: a contour simplified to this fraction of its perimeter keeps the shape and
#: stays a few dozen points
POLYGON_EPSILON = 0.004


def rle_encode(mask) -> Dict[str, Any]:
    """COCO uncompressed RLE of a boolean HxW mask (column-major, zeros
    first)."""
    import numpy as np  # noqa: PLC0415
    flat = np.asarray(mask, dtype=bool).flatten(order="F")
    changes = np.flatnonzero(np.diff(flat.astype(np.int8))) + 1
    bounds = np.concatenate([[0], changes, [flat.size]])
    counts = np.diff(bounds).tolist()
    if flat.size and flat[0]:
        counts = [0] + counts
    return {"size": [int(mask.shape[0]), int(mask.shape[1])], "counts": counts}


def rle_decode(rle: Dict[str, Any]):
    """The inverse of :func:`rle_encode`."""
    import numpy as np  # noqa: PLC0415
    h, w = rle["size"]
    flat = np.zeros(h * w, dtype=bool)
    at, value = 0, False
    for n in rle["counts"]:
        flat[at:at + n] = value
        at += n
        value = not value
    return flat.reshape((h, w), order="F")


def polygon_of(mask) -> List[List[float]]:
    """The outer contour of the largest connected region of ``mask``,
    simplified."""
    import cv2  # noqa: PLC0415
    import numpy as np  # noqa: PLC0415
    m = np.asarray(mask, dtype=np.uint8)
    contours, _h = cv2.findContours(m, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return []
    c = max(contours, key=cv2.contourArea)
    c = cv2.approxPolyDP(c, POLYGON_EPSILON * cv2.arcLength(c, True), True)
    return [[float(p[0][0]), float(p[0][1])] for p in c]


def describe(mask, score: float) -> Dict[str, Any]:
    import numpy as np  # noqa: PLC0415
    m = np.asarray(mask, dtype=bool)
    ys, xs = np.nonzero(m)
    if xs.size == 0:
        return {"found": False}
    return {"found": True, "score": round(float(score), 4),
            "polygon": polygon_of(m),
            "bbox": [int(xs.min()), int(ys.min()), int(xs.max()) + 1,
                     int(ys.max()) + 1],
            "area_px": int(xs.size), "rle": rle_encode(m)}


class Sam3:
    """SAM 3 through ``transformers``: one text prompt per name."""

    name = "facebook/sam3"

    def __init__(self, device: str = "cuda"):
        import torch  # noqa: PLC0415
        from transformers import Sam3Model, Sam3Processor  # noqa: PLC0415
        self.torch, self.device = torch, device
        self.processor = Sam3Processor.from_pretrained(self.name)
        self.model = Sam3Model.from_pretrained(
            self.name, torch_dtype=torch.bfloat16).to(device).eval()

    def segment(self, image, names: Sequence[str], threshold: float
                ) -> Dict[str, Dict[str, Any]]:
        out = {}
        for name in names:
            inputs = self.processor(images=image, text=name.replace("_", " "),
                                    return_tensors="pt").to(self.device)
            for key, value in inputs.items():
                if hasattr(value, "is_floating_point") and value.is_floating_point():
                    inputs[key] = value.to(self.torch.bfloat16)
            with self.torch.inference_mode():
                outputs = self.model(**inputs)
            result = self.processor.post_process_instance_segmentation(
                outputs, threshold=threshold, mask_threshold=0.5,
                target_sizes=inputs.get("original_sizes").tolist())[0]
            scores = result["scores"].float().cpu().numpy()
            if scores.size == 0:
                out[name] = {"found": False}
                continue
            best = int(scores.argmax())
            mask = result["masks"][best].cpu().numpy() > 0.5
            out[name] = describe(mask, float(scores[best]))
        return out


class GroundedSam2:
    """GroundingDINO boxes from the text, SAM 2.1 masks inside them."""

    name = "IDEA-Research/grounding-dino-base+facebook/sam2.1-hiera-large"

    def __init__(self, device: str = "cuda"):
        import torch  # noqa: PLC0415
        from transformers import (AutoModelForZeroShotObjectDetection,  # noqa: PLC0415
                                  AutoProcessor, Sam2Model, Sam2Processor)
        self.torch, self.device = torch, device
        dino = "IDEA-Research/grounding-dino-base"
        self.dino_processor = AutoProcessor.from_pretrained(dino)
        self.dino = AutoModelForZeroShotObjectDetection.from_pretrained(
            dino).to(device).eval()
        sam = "facebook/sam2.1-hiera-large"
        self.sam_processor = Sam2Processor.from_pretrained(sam)
        self.sam = Sam2Model.from_pretrained(sam).to(device).eval()

    def segment(self, image, names, threshold):
        out = {}
        for name in names:
            text = name.replace("_", " ") + "."
            inputs = self.dino_processor(images=image, text=text,
                                         return_tensors="pt").to(self.device)
            with self.torch.inference_mode():
                det = self.dino(**inputs)
            boxes = self.dino_processor.post_process_grounded_object_detection(
                det, inputs.input_ids, threshold=threshold,
                text_threshold=threshold,
                target_sizes=[image.size[::-1]])[0]
            if len(boxes["scores"]) == 0:
                out[name] = {"found": False}
                continue
            best = int(boxes["scores"].argmax())
            box = boxes["boxes"][best].tolist()
            sam_in = self.sam_processor(images=image, input_boxes=[[box]],
                                        return_tensors="pt").to(self.device)
            with self.torch.inference_mode():
                pred = self.sam(**sam_in, multimask_output=False)
            masks = self.sam_processor.post_process_masks(
                pred.pred_masks.cpu(), sam_in["original_sizes"])[0]
            mask = masks[0, 0].numpy() > 0.5
            out[name] = describe(mask, float(boxes["scores"][best]))
        return out


def load(kind: str, device: str):
    if kind in ("sam3", "auto"):
        try:
            return Sam3(device)
        except Exception as exc:                 # gated, missing, too old
            if kind == "sam3":
                raise
            print(f"[segment_server] SAM 3 unavailable ({exc}); falling back "
                  f"to Grounded-SAM-2", flush=True)
    return GroundedSam2(device)


def make_server(segmenter: Any, host: str = "127.0.0.1", port: int = 8767
                ) -> ThreadingHTTPServer:
    """``/segment`` over ``segmenter.segment(PIL image, names, threshold)``
    (a model above, or a fake with the same method — the tests)."""
    lock = threading.Lock()

    class Handler(BaseHTTPRequestHandler):
        def _send(self, code: int, body: Any) -> None:
            data = json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self) -> None:  # noqa: N802
            if self.path != "/health":
                return self._send(404, {"error": "GET /health or POST /segment"})
            self._send(200, {"ok": True, "loaded": True,
                             "model": getattr(segmenter, "name", "?"),
                             "device": getattr(segmenter, "device", "?")})

        def do_POST(self) -> None:  # noqa: N802
            if self.path != "/segment":
                return self._send(404, {"error": "POST /segment"})
            size = int(self.headers.get("Content-Length") or 0)
            if not 0 < size <= MAX_BODY_BYTES:
                return self._send(413, {"error": f"body of {size} bytes"})
            try:
                ask = json.loads(self.rfile.read(size))
                from PIL import Image  # noqa: PLC0415
                image = Image.open(io.BytesIO(
                    base64.b64decode(ask["image_jpeg_b64"]))).convert("RGB")
                names = [str(n) for n in ask["names"]]
                if not 1 <= len(names) <= 16:
                    raise ValueError("1-16 names")
                threshold = float(ask.get("threshold", DEFAULT_THRESHOLD))
            except (KeyError, TypeError, ValueError, OSError) as exc:
                return self._send(400, {"error": f"bad request: {exc}"})
            started = time.time()
            with lock:
                objects = segmenter.segment(image, names, threshold)
            self._send(200, {"objects": objects,
                             "model": getattr(segmenter, "name", "?"),
                             "ms": round((time.time() - started) * 1000.0)})

        def log_message(self, fmt: str, *args: Any) -> None:
            print(f"[segment_server] {self.address_string()} {fmt % args}",
                  flush=True)

    return ThreadingHTTPServer((host, int(port)), Handler)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1",
                        help="loopback (default) or a private address; "
                             "there is no authentication")
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--model", choices=("auto", "sam3", "gsam2"),
                        default="auto")
    parser.add_argument("--device", default="cuda")
    args = parser.parse_args(argv)
    segmenter = load(args.model, args.device)
    server = make_server(segmenter, args.host, args.port)
    print(f"[segment_server] {segmenter.name} listening on "
          f"http://{args.host}:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
