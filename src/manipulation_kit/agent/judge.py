"""What the servo's judge is ASKED, in which shape, about which view of the
photo — independent of the model that answers.

A decision classifier (Jev-Omni: one head, a probability over 2-256 options)
answers any question whose answer is one of a few options. How the question
is SHAPED decides what it can say:

    ``choice``   one question, one option per kit choice id (:data:`CHOICES`)
                 — the six-way form the servo started with
    ``score``    each image axis as an ORDINAL score (-2 .. +2: far left ..
                 far right, far above .. far below), plus one yes/no
                 question — "is it inside the box" — decided by
                 per-question thresholds (:func:`score_verdict`). A yes/no
                 "is it in the photo" was measured and dropped: it said
                 "no" at up to 0.78 about a roll plainly in view
    ``grasp``    ``score`` plus two grasp-geometry questions (would closing
                 at the box grasp it; how far would the jaws turn)
    ``letters``  a letter on a disc outside each side of the box (A above,
                 B right, C below, D left) and one choice question: toward
                 which letter, inside the box, or not in the photo — no
                 direction words at all (:func:`draw_letters`)

and :data:`VIEWS` shows the same marked photo mirrored, mapping each answer
back to the upright photo before averaging. :class:`AskingJudge` is the
``judge(look)`` seam of :class:`~manipulation_kit.agent.servo.Servo` over a
transport ``ask(image, state, questions) -> [[p per option], ...]`` — the
model, local or remote, lives with the caller (``examples/agent/jev_judge.py``).

Nothing here opens a socket or imports a model; the views need pillow (the
``perception`` extra), loaded when a view other than ``upright`` is asked.
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from .servo import CHOICES, ServoLook

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


# --------------------------------------------------------------------------- #
# question formulations: what is asked, in which shape
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Question:
    """One question to the classifier. ``kind`` is the typed-decision shape:
    ``choice`` (a distribution over labels), ``noul`` (yes/no: the answer is
    P(yes)) or ``score`` (ordered options; the answer is the expected
    ``values`` and the top probability). Jev-Omni has ONE head — a
    probability over 2-256 options — so ``noul`` and ``score`` are that head
    over two, or over ordered, options; the shape is in how the answer is
    read."""

    id: str
    kind: str
    question: str
    options: Tuple[str, ...]
    #: ``choice``: the kit's choice id of each option; ``score``: the value
    ids: Tuple[Any, ...] = ()


YES_NO = ("yes", "no")

#: ordinal answers, far to one side .. far to the other, as a score -2..+2.
#: "half the box" is the unit the mark gives the judge: the box is the
#: object's outline grown by the tolerance
HORIZONTAL = ("far to the LEFT of the box centre (half a box or more)",
              "a little LEFT of the box centre",
              "level with the box centre, left to right",
              "a little RIGHT of the box centre",
              "far to the RIGHT of the box centre (half a box or more)")
VERTICAL = ("far ABOVE the box centre in the photo (half a box or more)",
            "a little ABOVE the box centre in the photo",
            "level with the box centre, top to bottom",
            "a little BELOW the box centre in the photo",
            "far BELOW the box centre in the photo (half a box or more)")
#: how far the jaws would turn about the approach axis, in degrees
JAW_TURN = (("a quarter turn anticlockwise (about 90 degrees)", -90.0),
            ("an eighth of a turn anticlockwise (about 45 degrees)", -45.0),
            ("no turn: the jaws already close across its narrowest width", 0.0),
            ("an eighth of a turn clockwise (about 45 degrees)", 45.0),
            ("a quarter turn clockwise (about 90 degrees)", 90.0))

#: formulation name -> what it asks. ``choice`` is the one-question six-way
#: form the servo has used; ``score`` asks each image axis as an ordinal
#: score plus two yes/no questions (the escape as its own question, not an
#: option); ``grasp`` adds two grasp-geometry questions to ``score``.
FORMULATIONS = ("choice", "score", "grasp", "letters")

#: the ``letters`` formulation: a letter on a disc just outside each side of
#: the box, the answer a letter rather than a direction word. Letter ->
#: the kit choice id of that side of the box, in the image AS DRAWN — the
#: letters are painted into the photo, so a mirrored view carries them along
#: and the answer needs no mapping back
LETTERS: Tuple[Tuple[str, str], ...] = (("A", "above"), ("B", "right"),
                                        ("C", "below"), ("D", "left"))

LETTER_STATE = ("Photo from the camera on a robot hand, looking along the hand "
                "past its two gripper fingers. A green box has been drawn on "
                "the photo where the robot BELIEVES the {object} is, slightly "
                "larger than the {object} should appear; a green cross marks "
                "the box's centre. Just outside the box, one on each of its "
                "four sides, are the letters A, B, C and D, each on a black "
                "disc with a short green arrow pointing out from the box.")


def state_for(obj: str, formulation: str) -> str:
    """The ``state`` text the photo is described with, for ``formulation``."""
    name = obj.replace("_", " ")
    return (LETTER_STATE if formulation == "letters" else STATE).format(
        object=name)


def questions(obj: str, formulation: str = "choice") -> List[Question]:
    """The questions of ``formulation`` about ``obj`` (a scene name)."""
    name = obj.replace("_", " ")
    if formulation not in FORMULATIONS:
        raise ValueError(f"formulation must be one of {FORMULATIONS}, got "
                         f"{formulation!r}")
    if formulation == "choice":
        return [Question("where", "choice", QUESTION.format(object=name),
                         tuple(LABELS[c].format(object=name) for c in CHOICES),
                         CHOICES)]
    if formulation == "letters":
        return [Question(
            "letters", "choice",
            f"Toward which letter does the centre of the {name} lie, seen "
            f"from the green cross?",
            tuple(f"toward the letter {letter}" for letter, _c in LETTERS)
            + ("it is inside the green box",
               f"the {name} is not in the photo"),
            tuple(c for _l, c in LETTERS) + ("on", "not_visible"))]
    out = [
        Question("inside", "noul", f"Is the {name} entirely inside the green "
                 f"box?", YES_NO),
        Question("horizontal", "score", f"Left to right in the photo, where "
                 f"is the centre of the {name} compared with the centre of "
                 f"the green box?", HORIZONTAL, (-2, -1, 0, 1, 2)),
        Question("vertical", "score", f"Top to bottom in the photo, where is "
                 f"the centre of the {name} compared with the centre of the "
                 f"green box?", VERTICAL, (-2, -1, 0, 1, 2))]
    if formulation == "grasp":
        out += [
            Question("grasp_here", "noul", f"If the gripper closed its jaws "
                     f"at the green box, would it grasp the {name}?", YES_NO),
            Question("jaw_turn", "score", f"How far would the jaws have to "
                     f"turn to close across the {name}'s narrowest width?",
                     tuple(t for t, _v in JAW_TURN),
                     tuple(v for _t, v in JAW_TURN))]
    return out


# --------------------------------------------------------------------------- #
# views: the same marked photo shown flipped, and the answer mapped back
# --------------------------------------------------------------------------- #

#: view -> (PIL transpose name or None, image axes it mirrors). Measured on
#: d1-2 wrist frames (``tools/jev_questions_lab.py``: 4 photos x 25 known box
#: offsets, one photo per decision): asked about the UPRIGHT photo — gripper
#: at the bottom — the six-way choice reads left/right but not up/down (axis
#: sign 96 of 160, a "below" bias: 0 of 6 "above" boxes in the first sweep)
#: and the servo would step the wrong way on 19 of 100; averaged over the
#: four views, 3 of 100. On the three photos of the d1-2 live run the upright
#: answers, accumulated, step "below" — away from the roll, which is above
#: the box — and the four-view answers do not step
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
    if question.id == "letters":
        return probs            # the letters moved with the image
    if question.kind == "choice":
        index = {c: i for i, c in enumerate(question.ids)}
        out = list(probs)
        for c, i in index.items():
            mapped = c
            for axis in axes:
                mapped = _MIRROR_CHOICE[axis].get(mapped, mapped)
            out[index[mapped]] = probs[i]
        return out
    mirrored = ((question.id == "horizontal" and "u" in axes)
                or (question.id == "vertical" and "v" in axes)
                # a mirror image turns the other way; a half turn does not
                or (question.id == "jaw_turn" and len(axes) == 1))
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


#: per-question thresholds of the ``score`` formulation (lesson: one common
#: threshold across questions discards the answers). Fitted on the d1-2
#: wrist-photo lab (``tools/jev_questions_lab.py``, 4 photos x 25 known box
#: offsets, score@rot180): stepping when an axis' expected score reaches
#: 0.7 and stopping "on" when P(inside) reaches 0.6 gave 90 right / 0 wrong
#: / 6 premature "on" / 4 no-answer of 100, the same thresholds chosen with
#: each photo held out in turn. One scene, one object: a starting point to
#: re-measure, not a constant of the model.
SCORE_STEP = 0.7
SCORE_ON = 0.6

#: the vote a ``score`` look casts in the servo's six-way distribution: the
#: decided choice at this probability, the rest shared — the thresholds
#: above decide, the servo's accumulation counts the votes over photos
SCORE_VOTE = 0.6


def score_verdict(answers: Mapping[str, Mapping[str, Any]], *,
                  step: float = SCORE_STEP, on: float = SCORE_ON) -> str:
    """A ``score`` look's verdict: the direction of the axis whose expected
    score is furthest from 0, when it reaches ``step``; else "on" when
    P(inside) reaches ``on``; else "" (no answer)."""
    h = float(answers["horizontal"]["value"])
    v = float(answers["vertical"]["value"])
    if max(abs(h), abs(v)) >= step:
        if abs(h) >= abs(v):
            return "right" if h > 0 else "left"
        return "below" if v > 0 else "above"
    return "on" if float(answers["inside"]["p"]) >= on else ""


def to_choices(answers: Mapping[str, Mapping[str, Any]], *,
               step: float = SCORE_STEP, on: float = SCORE_ON
               ) -> Dict[str, float]:
    """The kit's six-way distribution (:data:`CHOICES`) from a
    formulation's answers: ``choice`` as it is; ``score`` as a VOTE for
    :func:`score_verdict` (:data:`SCORE_VOTE` on the verdict, the rest
    shared; uniform when there is no verdict). Squeezing the ordinal answers
    into six probabilities and one common margin instead measured 12-49 of
    100 right on the same lab set."""
    for chosen in ("where", "letters"):
        if chosen in answers:
            return {c: float(answers[chosen]["dist"].get(c, 0.0))
                    for c in CHOICES}
    verdict = score_verdict(answers, step=step, on=on)
    if not verdict:
        return {c: 1.0 / len(CHOICES) for c in CHOICES}
    rest = (1.0 - SCORE_VOTE) / (len(CHOICES) - 1)
    return {c: SCORE_VOTE if c == verdict else rest for c in CHOICES}


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
    """``judge(look) -> {choice: p}`` over a transport: which questions
    (``formulation``), which views, and the answers read back into the kit's
    distribution. Subclasses implement :meth:`_ask`. ``last`` keeps the last
    look's per-question answers and timing (the servo's log reads it)."""

    def __init__(self, *, formulation: str = "choice",
                 views: Sequence[str] = DEFAULT_VIEWS, debug: bool = False):
        if formulation not in FORMULATIONS:
            raise ValueError(f"formulation must be one of {FORMULATIONS}")
        unknown = [v for v in views if v not in VIEWS]
        if not views or unknown:
            raise ValueError(f"views must be among {tuple(VIEWS)}, got "
                             f"{list(views)!r}")
        self.formulation, self.views = formulation, tuple(views)
        self.debug = debug
        self.last: Dict[str, Any] = {}

    def _ask(self, image: Path, state: str,
             items: Sequence[Question]) -> Tuple[List[List[float]], float]:
        """``([probabilities per option, per question], milliseconds)`` for
        one image."""
        raise NotImplementedError

    def answers(self, image: Path, obj: str) -> Dict[str, Any]:
        """Every question of the formulation over every view, mapped back to
        the upright photo and averaged over the views."""
        items = questions(obj, self.formulation)
        state = state_for(obj, self.formulation)
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

    def __call__(self, look: ServoLook) -> Dict[str, float]:
        if look.image is None:
            raise RuntimeError(f"{type(self).__name__} judges a photo; this "
                               f"robot gave none (--snapshot-cmd)")
        started = time.time()
        image = Path(look.image)
        if self.formulation == "letters":
            image, _where = draw_letters(image, box_of(look),
                                         image.with_name(f"{image.stem}_letters.png"))
        answers = self.answers(image, look.object)
        dist = to_choices(answers)
        self.last = {"answers": answers, "views": list(self.views),
                     "formulation": self.formulation,
                     "ms": round((time.time() - started) * 1000.0)}
        if self.debug:
            print(f"[jev_judge] {self.last['ms']} ms {Path(look.image).name} "
                  f"{self.formulation} x{len(self.views)}: {dist}",
                  file=sys.stderr)
        return dist


__all__ = ["ALL_VIEWS", "AskingJudge", "DEFAULT_VIEWS", "LETTERS", "LETTER_STATE",
           "box_of", "draw_letters", "letter_centres", "state_for", "FORMULATIONS", "LABELS", "QUESTION",
           "Question", "SCORE_ON", "SCORE_STEP", "SCORE_VOTE", "STATE",
           "VIEWS", "YES_NO", "questions", "read", "score_verdict",
           "to_choices", "unview", "view_image", "words"]
