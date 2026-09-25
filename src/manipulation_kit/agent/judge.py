"""The seam between the servo and whatever answers its questions: typed
questions, typed answers, mirrored views, and marks drawn for a judge.

Nothing here knows a model. A judge implementation (the examples ship one)
supplies a :class:`Formulation` — which questions, how the photo is
described, how the answers become the servo's six-way distribution — and a
transport (:meth:`AskingJudge._ask`) that returns a probability per option.
This module owns the parts that are geometry, not wording:

    Question / read()     the typed answer shapes: ``choice`` (a
                          distribution over ids), ``noul`` (yes/no: option 0
                          is "yes", the answer is its probability), ``score``
                          (ordered options with values: the expected value)
    VIEWS / unview()      the photo shown mirrored, and each answer mapped
                          back to the upright photo by how its question
                          depends on the image axes (``Question.mirror``)
    draw_letters()        a letter on a disc outside each side of the servo's
                          box (:data:`LETTERS`), for an answer that names a
                          letter rather than a direction word
    AskingJudge           ``judge(look) -> {choice: p}`` over a formulation,
                          views and a transport

The views and the letters need pillow (the ``perception`` extra), loaded
when used.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import (Any, Callable, Dict, List, Mapping, Optional, Sequence,
                    Tuple, Union)

try:                                     # Python 3.8+: typing.Protocol
    from typing import Protocol
except ImportError:                      # pragma: no cover
    Protocol = object                    # type: ignore

from .servo import Reading, ServoLook

#: how a question's answer depends on the image axes — what a mirrored view
#: does to it (:func:`unview`):
#:   ``choice``   the ids are kit choice ids; left/right swap on a
#:                left-right mirror, above/below on a top-bottom one
#:   ``u`` / ``v`` an ordinal along that image axis: reversed when that axis
#:                is mirrored
#:   ``handed``   a turning sense: reversed by a single mirror, not by a
#:                half turn (two mirrors)
#:   ``painted``  the options name marks drawn INTO the photo (letters):
#:                they travel with it, nothing to map
#:   ``fixed``    no dependence on the image axes (a yes/no about overlap)
MIRRORS = ("choice", "u", "v", "handed", "painted", "fixed")


@dataclass(frozen=True)
class Question:
    """One question to a judge: ``kind`` is the answer's shape (``choice``,
    ``noul``, ``score``), ``ids`` the kit id (choice) or value (score) of each
    option, ``mirror`` how a mirrored view changes it (:data:`MIRRORS`)."""

    id: str
    kind: str
    question: str
    options: Tuple[str, ...]
    ids: Tuple[Any, ...] = ()
    mirror: str = "fixed"

    def __post_init__(self) -> None:
        if self.kind not in ("choice", "noul", "score"):
            raise ValueError(f"kind must be choice, noul or score, got "
                             f"{self.kind!r}")
        if self.mirror not in MIRRORS:
            raise ValueError(f"mirror must be one of {MIRRORS}")
        if self.kind == "noul" and len(self.options) != 2:
            raise ValueError("a noul question has two options, yes first")
        if self.kind != "noul" and len(self.ids) != len(self.options):
            raise ValueError("one id per option")


class Formulation(Protocol):
    """What a judge asks about one look (a judge implementation supplies
    it)."""

    name: str

    def questions(self, obj: str) -> List[Question]: ...

    def state(self, obj: str) -> str: ...

    def to_choices(self, answers: Mapping[str, Mapping[str, Any]]
                   ) -> Dict[str, float]: ...

    def decorate(self, look: ServoLook, image: Path) -> Path: ...

    # optional: ``to_reading(answers) -> Reading`` — the typed answers
    # (depth, occlusion, turn, fit) beside the direction; a formulation
    # without it answers the direction alone (``to_choices``)


#: view -> (PIL transpose name or None, image axes it mirrors). Measured on
#: d1-2 wrist frames (the examples' question lab: 4 photos x 25 known box
#: offsets, one photo per decision): a judge asked about the UPRIGHT photo —
#: gripper at the bottom — read left/right but not up/down, and the servo
#: would have stepped the wrong way on 19 of 100; averaged over the four
#: views, 3 of 100. A bias that does not follow the image cancels over
#: mirrored views while the signal adds
VIEWS: Dict[str, Tuple[Optional[str], Tuple[str, ...]]] = {
    "upright": (None, ()),
    "flip_v": ("FLIP_TOP_BOTTOM", ("v",)),
    "flip_h": ("FLIP_LEFT_RIGHT", ("u",)),
    "rot180": ("ROTATE_180", ("u", "v"))}
#: all four mirrors of the photo — the default: the upright wrist photo
#: alone steps the wrong way on d1-2 (see :data:`VIEWS`)
ALL_VIEWS: Tuple[str, ...] = tuple(VIEWS)
DEFAULT_VIEWS = ALL_VIEWS

_MIRROR_CHOICE = {"u": {"left": "right", "right": "left"},
                  "v": {"above": "below", "below": "above"}}


def view_image(photo: Path, view: str, out: Path) -> Path:
    """``photo`` as ``view`` shows it (``upright``: the file itself)."""
    op, _axes = VIEWS[view]
    if op is None:
        return Path(photo)
    from PIL import Image  # noqa: PLC0415
    with Image.open(photo) as image:
        image.transpose(getattr(Image, op)).save(out)
    return out


def unview(question: Question, probs: Sequence[float], view: str) -> List[float]:
    """The probability of each of ``question``'s options in the UPRIGHT
    photo, from the answer asked about ``view`` of it."""
    axes = VIEWS[view][1]
    probs = [float(p) for p in probs]
    if question.mirror in ("fixed", "painted"):
        return probs
    if question.mirror == "choice":
        index = {c: i for i, c in enumerate(question.ids)}
        out = list(probs)
        for c, i in index.items():
            mapped = c
            for axis in axes:
                mapped = _MIRROR_CHOICE[axis].get(mapped, mapped)
            out[index[mapped]] = probs[i]
        return out
    mirrored = ((question.mirror == "u" and "u" in axes)
                or (question.mirror == "v" and "v" in axes)
                # a mirror image turns the other way; a half turn does not
                or (question.mirror == "handed" and len(axes) == 1))
    return probs[::-1] if mirrored else probs


def read(question: Question, probs: Sequence[float]) -> Dict[str, Any]:
    """An answer in its shape: ``choice`` -> ``{"dist"}``, ``noul`` ->
    ``{"p"}`` (P(yes)), ``score`` -> ``{"value", "confidence", "probs"}``."""
    probs = [float(p) for p in probs]
    total = sum(probs) or 1.0
    probs = [p / total for p in probs]
    if question.kind == "choice":
        return {"dist": dict(zip(question.ids, probs))}
    if question.kind == "noul":
        return {"p": probs[0]}
    return {"value": float(sum(v * p for v, p in zip(question.ids, probs))),
            "confidence": max(probs), "probs": probs}


#: the letters drawn beside the box (:func:`draw_letters`): letter -> the kit
#: choice id of the side it is drawn on, in the image AS DRAWN — painted into
#: the photo, a mirrored view carries them along
LETTERS: Tuple[Tuple[str, str], ...] = (("A", "above"), ("B", "right"),
                                        ("C", "below"), ("D", "left"))


def box_of(look: ServoLook) -> Tuple[float, float, float, float]:
    """The drawn box of a look (u0, v0, u1, v1); the tolerance circle's
    square when the object's outline was not known."""
    if look.box_px is not None:
        return tuple(float(x) for x in look.box_px)          # type: ignore
    r = max(float(look.radius_px), 10.0)
    return (look.u - r, look.v - r, look.u + r, look.v + r)


#: the letter discs: radius, the gap between the box and the arrow, the
#: arrow's length, and the frame margin a disc is kept inside
LETTER_RADIUS_PX = 14
LETTER_GAP_PX = 4
LETTER_ARROW_PX = 14
LETTER_MARGIN_PX = 2


def letter_centres(box: Sequence[float], width: int, height: int,
                   letters: Sequence[Tuple[str, str]] = LETTERS
                   ) -> Dict[str, Tuple[float, float]]:
    """Where each letter's disc goes (letter -> centre): centred on its side
    of ``box``, just outside it (gap + arrow + radius), then clamped into the
    frame. A disc stays outside the box unless the box itself reaches the
    frame's edge."""
    u0, v0, u1, v1 = (float(x) for x in box)
    cu, cv = (u0 + u1) / 2.0, (v0 + v1) / 2.0
    off = LETTER_GAP_PX + LETTER_ARROW_PX + LETTER_RADIUS_PX
    side = {"above": (cu, v0 - off), "right": (u1 + off, cv),
            "below": (cu, v1 + off), "left": (u0 - off, cv)}
    raw = {letter: side[c] for letter, c in letters}
    lo = LETTER_RADIUS_PX + LETTER_MARGIN_PX
    return {k: (min(max(u, lo), width - lo), min(max(v, lo), height - lo))
            for k, (u, v) in raw.items()}


def _font(size: int):
    from PIL import ImageFont  # noqa: PLC0415
    for name in ("DejaVuSans-Bold.ttf", "Arial Bold.ttf", "arialbd.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    try:
        return ImageFont.load_default(size=size)
    except TypeError:                    # pillow < 10.1
        return ImageFont.load_default()


def draw_letters(photo: Path, box: Sequence[float], out: Path, *,
                 letters: Sequence[Tuple[str, str]] = LETTERS
                 ) -> Tuple[Path, Dict[str, Tuple[float, float]]]:
    """``photo`` (already marked with the box) with a short green arrow out
    of each side of ``box`` and the letter of that side (:data:`LETTERS`) in
    white on a black disc with a white rim beyond it. Returns the file and
    each letter's disc centre."""
    from PIL import Image, ImageDraw  # noqa: PLC0415
    with Image.open(photo) as source:
        image = source.convert("RGB")
    draw = ImageDraw.Draw(image)
    u0, v0, u1, v1 = (float(x) for x in box)
    cu, cv = (u0 + u1) / 2.0, (v0 + v1) / 2.0
    g, a = LETTER_GAP_PX, LETTER_ARROW_PX
    green = (0, 230, 0)
    arrows = {"above": ((cu, v0 - g), (cu, v0 - g - a)),
              "right": ((u1 + g, cv), (u1 + g + a, cv)),
              "below": ((cu, v1 + g), (cu, v1 + g + a)),
              "left": ((u0 - g, cv), (u0 - g - a, cv))}
    for (start, end) in arrows.values():
        draw.line([start, end], fill=green, width=4)
        du, dv = end[0] - start[0], end[1] - start[1]
        n = max((du * du + dv * dv) ** 0.5, 1e-9)
        du, dv = du / n, dv / n
        head = [end, (end[0] - 7 * du - 5 * dv, end[1] - 7 * dv + 5 * du),
                (end[0] - 7 * du + 5 * dv, end[1] - 7 * dv - 5 * du)]
        draw.polygon(head, fill=green)
    where = letter_centres(box, image.width, image.height, letters)
    font = _font(2 * LETTER_RADIUS_PX - 6)
    r = LETTER_RADIUS_PX
    for letter, (u, v) in where.items():
        draw.ellipse([u - r, v - r, u + r, v + r], fill=(0, 0, 0),
                     outline=(255, 255, 255), width=2)
        draw.text((u, v), letter, fill=(255, 255, 255), font=font, anchor="mm")
    out.parent.mkdir(parents=True, exist_ok=True)
    image.save(out)
    return out, where


class AskingJudge:
    """``judge(look) -> {choice: p}`` over a transport: a
    :class:`Formulation` says what is asked, ``views`` which mirrors of the
    photo, and the answers are read back into the kit's distribution.
    Subclasses implement :meth:`_ask`. ``last`` keeps the last look's
    per-question answers and timing; ``log`` (optional) gets one line per
    look."""

    def __init__(self, formulation: "Formulation", *,
                 views: Sequence[str] = DEFAULT_VIEWS,
                 log: Optional[Callable[[str], None]] = None):
        unknown = [v for v in views if v not in VIEWS]
        if not views or unknown:
            raise ValueError(f"views must be among {tuple(VIEWS)}, got "
                             f"{list(views)!r}")
        self.formulation, self.views = formulation, tuple(views)
        self.log = log
        self.last: Dict[str, Any] = {}

    def _ask(self, image: Path, state: str,
             items: Sequence[Question]) -> Tuple[List[List[float]], float]:
        """``([probabilities per option, per question], milliseconds)`` for
        one image."""
        raise NotImplementedError

    def answers(self, image: Path, obj: str) -> Dict[str, Any]:
        """Every question of the formulation over every view, mapped back to
        the upright photo and averaged over the views."""
        items = self.formulation.questions(obj)
        state = self.formulation.state(obj)
        summed = {q.id: [0.0] * len(q.options) for q in items}
        ms = 0.0
        for view in self.views:
            shown = view_image(image, view,
                               image.with_name(f"{image.stem}_{view}{image.suffix}"))
            probs, took = self._ask(shown, state, items)
            ms += took
            for q, p in zip(items, probs):
                for i, x in enumerate(unview(q, p, view)):
                    summed[q.id][i] += x / len(self.views)
        out = {q.id: read(q, summed[q.id]) for q in items}
        out["_ms"] = ms
        return out

    def __call__(self, look: ServoLook) -> Union[Reading, Dict[str, float]]:
        if look.image is None:
            raise RuntimeError(f"{type(self).__name__} judges a photo; this "
                               f"robot gave none (--snapshot-cmd)")
        started = time.time()
        image = self.formulation.decorate(look, Path(look.image))
        answers = self.answers(image, look.object)
        typed = getattr(self.formulation, "to_reading", None)
        dist = (typed(answers) if typed is not None
                else self.formulation.to_choices(answers))
        name = getattr(self.formulation, "name", type(self.formulation).__name__)
        self.last = {"answers": answers, "views": list(self.views),
                     "formulation": name,
                     "ms": round((time.time() - started) * 1000.0)}
        if self.log is not None:
            said = dist.to_json() if isinstance(dist, Reading) else dist
            self.log(f"{self.last['ms']} ms {Path(look.image).name} {name} "
                     f"x{len(self.views)}: {said}")
        return dist


__all__ = ["ALL_VIEWS", "AskingJudge", "DEFAULT_VIEWS", "Formulation",
           "LETTERS", "MIRRORS", "Question", "VIEWS", "box_of",
           "draw_letters", "letter_centres", "read", "unview", "view_image"]
