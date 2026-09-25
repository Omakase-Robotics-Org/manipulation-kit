"""What Jev-Omni is asked about the servo's marked wrist photo, in which
shape: the wording, the option labels and the formulations, as
:class:`manipulation_kit.agent.judge.Formulation` s.

    choice    one six-way question (the servo's first form)
    score     each image axis as an ordinal score (-2 .. +2), plus a yes/no
              "inside the box", decided by per-question thresholds
    grasp     score plus two grasp-geometry questions
    letters   a letter beside each side of the box (``draw_letters``) and
              "toward which letter" — no direction words

Jev-Omni has ONE head (a probability over 2-256 options, one question per
forward pass): ``noul`` and ``score`` are that head over two, or over
ordered, options; the shape is in how the answer is read.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Mapping

from manipulation_kit.agent.judge import (LETTERS, Question, box_of,
                                          draw_letters)
from manipulation_kit.agent.servo import CHOICES, ServoLook

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

#: the formulation names (see :data:`FORMULATIONS` for the objects)
NAMES = ("choice", "score", "grasp", "letters")


def state_for(obj: str, formulation: str) -> str:
    """The ``state`` text the photo is described with, for ``formulation``."""
    name = obj.replace("_", " ")
    return (LETTER_STATE if formulation == "letters" else STATE).format(
        object=name)


def questions(obj: str, formulation: str = "choice") -> List[Question]:
    """The questions of ``formulation`` about ``obj`` (a scene name)."""
    name = obj.replace("_", " ")
    if formulation not in NAMES:
        raise ValueError(f"formulation must be one of {NAMES}, got "
                         f"{formulation!r}")
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
        """``letters``: the letters drawn beside the look's box."""
        if self.name != "letters":
            return image
        out, _where = draw_letters(image, box_of(look),
                                   image.with_name(f"{image.stem}_letters.png"))
        return out


#: name -> formulation
FORMULATIONS: Dict[str, Asked] = {name: Asked(name) for name in NAMES}

