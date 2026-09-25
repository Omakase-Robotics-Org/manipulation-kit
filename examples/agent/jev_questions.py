"""What Jev-Omni is asked about the servo's marked wrist photo, in which
shape: the wording, the option labels and the formulations, as
:class:`manipulation_kit.agent.judge.Formulation` s.

    choice    one six-way question (the servo's first form)
    score     each image axis as an ordinal score (-2 .. +2), plus a yes/no
              "inside the box", decided by per-question thresholds
    grasp     score plus two grasp-geometry questions
    letters   a letter beside each side of the box (``draw_letters``) and
              "toward which letter" — no direction words
    jaws      the servo's v2 drawing (the jaw opening at the object's depth
              and the object's outline, ``manipulation_kit.agent.jaws``):
              letters around the OPENING, and in the same batch how far,
              the depth against the real fingers, occlusion, the jaw turn
              and the fit — a typed ``Reading``
    jaws_words  the same batch with the direction asked in words (left of the
              gap, above it, ...) instead of letters

Jev-Omni has ONE head (a probability over 2-256 options, one question per
forward pass): ``noul`` and ``score`` are that head over two, or over
ordered, options; the shape is in how the answer is read.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Mapping

from manipulation_kit.agent.jaws import axial_mean
from manipulation_kit.agent.judge import (LETTERS, Question, box_of,
                                          draw_letters)
from manipulation_kit.agent.servo import CHOICES, DEPTHS, Reading, ServoLook

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


LETTER_STATE = ("Photo from the camera on a robot hand, looking along the hand "
                "past its two gripper fingers. A green box has been drawn on "
                "the photo where the robot BELIEVES the {object} is, slightly "
                "larger than the {object} should appear; a green cross marks "
                "the box's centre. Just outside the box, one on each of its "
                "four sides, are the letters A, B, C and D, each on a black "
                "disc with a short green arrow pointing out from the box.")

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

#: the v2 drawing: the jaw opening (two green pad bars at the object's
#: depth, the gap outlined between them) and the object's outline
JAWS_STATE = ("Photo from the camera on a robot hand, looking along the hand "
              "past its two gripper fingers, which are at the bottom of the "
              "photo. Two short green bars have been drawn where the fingers' "
              "pads will be when the hand has come down to the {object}: the "
              "jaws close in the thin green gap between the bars, and a small "
              "green cross marks the middle of that gap. A magenta line "
              "outlines the {object} as the robot expects its shape.")
JAWS_LETTERS_STATE = JAWS_STATE + (
    " Just outside the green marks, one on each of four sides, are the "
    "letters A, B, C and D, each on a black disc with a short green arrow "
    "pointing out from the gap.")

#: where the object lies from the gap, in words (the kit's choice ids)
JAW_WORDS: Dict[str, str] = {
    "on": "the {object} is in the gap between the green bars",
    "left": "the {object} is to the LEFT of the gap",
    "right": "the {object} is to the RIGHT of the gap",
    "above": "the {object} is ABOVE the gap in the photo",
    "below": "the {object} is BELOW the gap in the photo",
    "not_visible": "the {object} is not in the photo"}

#: how far from the gap, as a score (0 in it .. 3 far)
JAW_DISTANCE = (("it is in the gap", 0.0),
                ("less than one gap-width away from it", 1.0),
                ("one to two gap-widths away", 2.0),
                ("more than two gap-widths away", 3.0))

#: the depth against the REAL fingers (the kit's :data:`DEPTHS` ids)
JAW_DEPTH = (("ahead", "still ahead of the real fingers: farther from the "
                       "camera than their tips"),
             ("between", "between the real fingers, level with their pads"),
             ("behind", "behind or beside the real fingers, not in front of "
                        "them"))

#: the formulation names (see :data:`FORMULATIONS` for the objects)
NAMES = ("choice", "score", "grasp", "letters", "jaws", "jaws_words")


def jaw_questions(obj: str, formulation: str) -> List[Question]:
    """The v2 batch: the direction (letters or words) about the drawn jaw
    opening, then distance, depth, occlusion, turn and fit."""
    name = obj.replace("_", " ")
    if formulation == "jaws":
        where = Question(
            "letters", "choice",
            f"Seen from the green cross, toward which letter does the centre "
            f"of the {name} lie?",
            tuple(f"toward the letter {letter}" for letter, _c in LETTERS)
            + (f"the {name} is in the gap between the green bars",
               f"the {name} is not in the photo"),
            tuple(c for _l, c in LETTERS) + ("on", "not_visible"),
            mirror="painted")
    else:
        where = Question(
            "where", "choice",
            f"Where is the {name} compared with the gap between the green "
            f"bars?", tuple(JAW_WORDS[c].format(object=name) for c in CHOICES),
            CHOICES, mirror="choice")
    return [
        where,
        Question("distance", "score",
                 f"How far is the centre of the {name} from the middle of the "
                 f"gap between the green bars?",
                 tuple(t for t, _v in JAW_DISTANCE),
                 tuple(v for _t, v in JAW_DISTANCE)),
        Question("depth", "choice",
                 f"Along the view, where is the {name} compared with the real "
                 f"gripper fingers?",
                 tuple(f"the {name} is {text}" for _d, text in JAW_DEPTH),
                 tuple(d for d, _t in JAW_DEPTH)),
        Question("occluded", "noul",
                 f"Are the real fingers or the hand hiding part of the "
                 f"{name}?", YES_NO),
        Question("jaw_turn", "score",
                 f"How far would the green bars have to turn about the green "
                 f"cross so that the jaws close across the {name}'s narrowest "
                 f"width?",
                 tuple(t for t, _v in JAW_TURN),
                 tuple(v for _t, v in JAW_TURN), mirror="handed"),
        Question("fit", "noul",
                 f"Would the {name} fit in the gap between the green bars?",
                 YES_NO)]


def to_reading(answers: Mapping[str, Mapping[str, Any]]) -> Reading:
    """A v2 batch as the servo's typed :class:`Reading`: the ordered answers
    read as scores, no confidence gate; the turn as an AXIS mean (a quarter
    turn either way is the same turn)."""
    chosen = answers.get("letters") or answers.get("where")
    turn = answers.get("jaw_turn")
    return Reading(
        direction={c: float(chosen["dist"].get(c, 0.0)) for c in CHOICES},
        distance=(None if "distance" not in answers
                  else float(answers["distance"]["value"])),
        depth=(None if "depth" not in answers
               else {d: float(answers["depth"]["dist"].get(d, 0.0))
                     for d in DEPTHS}),
        occluded=(None if "occluded" not in answers
                  else float(answers["occluded"]["p"])),
        turn_deg=(None if turn is None else axial_mean(
            [v for _t, v in JAW_TURN], turn["probs"])),
        fit=None if "fit" not in answers else float(answers["fit"]["p"]))


def state_for(obj: str, formulation: str) -> str:
    """The ``state`` text the photo is described with, for ``formulation``."""
    name = obj.replace("_", " ")
    text = {"letters": LETTER_STATE, "jaws": JAWS_LETTERS_STATE,
            "jaws_words": JAWS_STATE}.get(formulation, STATE)
    return text.format(object=name)


def questions(obj: str, formulation: str = "choice") -> List[Question]:
    """The questions of ``formulation`` about ``obj`` (a scene name)."""
    name = obj.replace("_", " ")
    if formulation not in NAMES:
        raise ValueError(f"formulation must be one of {NAMES}, got "
                         f"{formulation!r}")
    if formulation in ("jaws", "jaws_words"):
        return jaw_questions(obj, formulation)
    if formulation == "choice":
        return [Question("where", "choice", QUESTION.format(object=name),
                         tuple(LABELS[c].format(object=name) for c in CHOICES),
                         CHOICES, mirror="choice")]
    if formulation == "letters":
        return [Question(
            "letters", "choice",
            f"Toward which letter does the centre of the {name} lie, seen "
            f"from the green cross?",
            tuple(f"toward the letter {letter}" for letter, _c in LETTERS)
            + ("it is inside the green box",
               f"the {name} is not in the photo"),
            tuple(c for _l, c in LETTERS) + ("on", "not_visible"),
            mirror="painted")]
    out = [
        Question("inside", "noul",
                 f"Is the {name} entirely inside the green box?", YES_NO),
        Question("horizontal", "score",
                 f"Left to right in the photo, where is the centre of the "
                 f"{name} compared with the centre of the green box?",
                 HORIZONTAL, (-2, -1, 0, 1, 2), mirror="u"),
        Question("vertical", "score",
                 f"Top to bottom in the photo, where is the centre of the "
                 f"{name} compared with the centre of the green box?",
                 VERTICAL, (-2, -1, 0, 1, 2), mirror="v")]
    if formulation == "grasp":
        out += [
            Question("grasp_here", "noul",
                     f"If the gripper closed its jaws at the green box, "
                     f"would it grasp the {name}?", YES_NO),
            Question("jaw_turn", "score",
                     f"How far would the jaws have to turn to close across "
                     f"the {name}'s narrowest width?",
                     tuple(t for t, _v in JAW_TURN),
                     tuple(v for _t, v in JAW_TURN), mirror="handed")]
    return out


#: per-question thresholds of the ``score`` formulation (lesson: one common
#: threshold across questions discards the answers). Fitted on the d1-2
#: wrist-photo lab (``jev_questions_lab.py``, 4 photos x 25 known box
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


@dataclass(frozen=True)
class Asked:
    """One formulation as a :class:`~manipulation_kit.agent.judge.Formulation`."""

    name: str

    def questions(self, obj: str) -> List[Question]:
        return questions(obj, self.name)

    def state(self, obj: str) -> str:
        return state_for(obj, self.name)

    def to_choices(self, answers: Mapping[str, Mapping[str, Any]]
                   ) -> Dict[str, float]:
        return to_choices(answers)

    def decorate(self, look: ServoLook, image: Path) -> Path:
        """``letters`` / ``jaws``: the letters drawn beside the look's box
        (for ``jaws`` the jaw opening's)."""
        if self.name not in ("letters", "jaws"):
            return image
        out, _where = draw_letters(image, box_of(look),
                                   image.with_name(f"{image.stem}_letters.png"))
        return out


@dataclass(frozen=True)
class AskedJaws(Asked):
    """A v2 formulation: the batch answers as a typed ``Reading``."""

    def to_reading(self, answers: Mapping[str, Mapping[str, Any]]) -> Reading:
        return to_reading(answers)


#: name -> formulation
FORMULATIONS: Dict[str, Asked] = {
    name: (AskedJaws(name) if name.startswith("jaws") else Asked(name))
    for name in NAMES}

