"""The System 1 step: align the hand on the object before a stroke, by
JUDGEMENT rather than by measurement.

The look before a stroke (:mod:`.loop`, ``look_before_stroke``) asks the model
whether the photo agrees with the declaration, and if not WHERE the object is
— a pixel it must write. A decision classifier cannot write a pixel; it can
only choose. This module asks it what it can answer: the kit DRAWS, on the
fresh wrist photo, WHERE THE JAWS WILL CLOSE — the two pads at the current jaw
gap, carried along the approach axis to the object's depth
(:func:`~manipulation_kit.agent.jaws.jaw_opening`) — and the object's outline
with its real shape (:mod:`.jaws`: the declared shape, or a segmenter's
polygon), and the judge answers typed questions about the object RELATIVE TO
THE FINGERS (:class:`Reading`): which side of the opening it lies on and how
far, whether it is ahead of the fingers, between them or behind, whether the
hand hides it, how far the jaws must turn to close across it, whether it
would fit. The kit does the rest — the image direction becomes a base-frame
step through the wrist camera's known orientation, a turn becomes a yaw about
the approach axis, "ahead" a step along it — and moves the hand with its own
:class:`~manipulation_kit.primitives.Nudge`, photographs again, until the
object is between the jaws or a budget is spent.

Why the jaw opening and not a box around the belief: a box drawn where the
kit believes the object is asks "does the belief agree with the photo", and
once the declaration has moved with the hand (every correction re-declares
it) the box follows the hand and "on" stops meaning "under the jaws". The
opening is the hand's own geometry; "between the fingers" is the stroke's own
precondition.

WHEN IT RUNS: before EVERY stroke of :data:`SERVO_VERBS` (``grasp`` on its
``object``, ``press`` on its ``target``), whether or not the model looked or
``locate``-d from the wrist first. d1-2, 2026-09-24 01:47Z (``--judge-only``):
a servo that ran only on an unmet look was never asked — twelve records,
``servo: null`` on every one, and the tape several cm off the jaws when the
stroke closed. ``probe`` has no named target to align on, ``handover`` closes
at a posture reached inside its own plan, and ``approach`` is the move that
brings the hand to the look — none is judged.

THE LOOP. :class:`Servo` runs photo -> draw -> judge -> act or stop inside the
turn, at the photo/judge rate, averaging the judge's readings over the photos
since the last move (:func:`accumulate`), and acting on the accumulated lead
by a FIXED PRIORITY (:func:`choose`):

    1. retreat      the fingers or the hand hide the object: step back along
                    the approach axis
    2. turn         the object is elongated and the jaws do not close across
                    its narrowest width: a yaw step about the approach axis
    3. xy           the object is off the opening: a table-plane step on the
                    nudge grid (coarse, then fine once an axis overshoots)
    4. approach     centred, and still ahead of the fingers: a step along the
                    approach axis, bounded so the fingertips stay above the
                    object and its support surface (the stroke's own descent
                    and fingertip contact search do the rest); "behind"
                    backs off instead

— until the opening is ``on`` the object (and, when asked, turned and close
enough), a cap, or evidence that stays flat. Every step is logged with its
reason, one line per photo.

Two seams, and the wheel owns everything between them::

    frame(side) -> Path | None      a FRESH photo from that hand's wrist camera
                                    (None: no camera here — the judge must
                                    answer from geometry, as a test stub does)
    judge(look) -> Reading | {choice: prob}
                                    the typed answers (a bare distribution over
                                    :data:`CHOICES` is the direction alone)

and one optional one, the object's outline
(:data:`~manipulation_kit.agent.jaws.ObjectOutline`, default the declared
shape). A judge READS THE PHOTO unless it says otherwise (:func:`photoless`,
which :func:`geometry_judge` is): when ``frame(side)`` gives no photo, a photo
judge is not asked at all and the report says ``skipped: "no wrist frame"``.

``observe_only=True``  photograph, draw, ask, record the readings in
                       ``DecisionRecord.servo``, never move or re-declare; the
                       loop then goes on with the model's own look rule.
``refine=True``        after ``on``, the same photo is drawn again with the jaw
                       opening shifted by ``refine_m`` on a 3 x 3 grid along
                       the jaw axis and across it; the declaration moves to
                       the jaw point of the shift the judge rates ``on``
                       highest, only when that beats the unshifted opening. The
                       hand does not move and a worse answer is never taken.
"""

from __future__ import annotations

import dataclasses
import logging
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import (Any, Callable, Dict, List, Mapping, Optional, Sequence,
                    Tuple, Union)

import numpy as np
from scipy.spatial.transform import Rotation as R

from ..primitives import Nudge
from ..primitives.offer import check, label_for
from ..primitives.types import NUDGE_GRID_M, NUDGE_MAX_YAW_RAD
from . import jaws as J
from .policy import OperatorPolicy, PolicyState

#: the verbs the servo judges before, -> the argument naming what the stroke
#: acts on. Explicit, not derived: a stroke is judged only where the hand
#: already stands at the posture it acts from and a named thing is there to
#: be marked (see the module docstring for the verbs left out, and why).
SERVO_VERBS: Dict[str, str] = {"grasp": "object", "press": "target"}

#: where the object is relative to the drawn jaw opening, IN THE IMAGE:
#: ``on`` (between the jaws) is the answer that lets the stroke run
CHOICES: Tuple[str, ...] = ("on", "left", "right", "above", "below",
                            "not_visible")

#: where the object is along the approach axis, relative to the fingers:
#: still ahead of the fingertips (approach), between the pads (hold), or
#: behind / beside them (retreat)
DEPTHS: Tuple[str, ...] = ("ahead", "between", "behind")

#: image direction of each displacement choice, in the optical frame (ROS
#: convention: +x image right, +y image down)
_IMAGE_AXIS: Dict[str, Tuple[float, float]] = {
    "left": (-1.0, 0.0), "right": (1.0, 0.0),
    "above": (0.0, -1.0), "below": (0.0, 1.0)}

#: an image axis whose horizontal (table-plane) component is smaller than
#: this is not a direction the hand can be moved along on the table
MIN_HORIZONTAL = 0.3

#: default correction steps: coarse first, fine after the judge's answer
#: changes sign on an axis. Both on the nudge grid.
DEFAULT_STEPS_M: Tuple[float, ...] = (NUDGE_GRID_M[1], NUDGE_GRID_M[0])

#: along the approach axis: the step toward the object (fine), and the
#: back-off when the hand hides it (coarse)
DEFAULT_APPROACH_STEP_M = NUDGE_GRID_M[0]
DEFAULT_RETREAT_STEP_M = NUDGE_GRID_M[1]
#: how far the servo may bring the hand along the approach axis in one
#: alignment, and how far above the object's near face (and its support
#: surface) the fingertips must stay: the stroke's own descent and fingertip
#: contact search close the rest
DEFAULT_MAX_APPROACH_M = 0.03
DEFAULT_APPROACH_CLEARANCE_M = 0.02

#: the yaw step (the nudge's own cap) and the most the servo turns the hand
#: in one alignment; a turn smaller than half a step is not taken
DEFAULT_YAW_STEP_RAD = NUDGE_MAX_YAW_RAD
DEFAULT_MAX_TURN_RAD = math.radians(90.0)

#: how many consecutive judgements (photos at the same hand pose) are
#: averaged before the servo acts: a shrug on one photo is not the end of
#: the loop, a shrug that stays flat over this many is
DEFAULT_WINDOW = 3

#: the accumulated answer must lead by this much to be acted on: a direction
#: over "on" to step, "on" over the runner-up to stop, "yes" over "no".
#: Evidence, not a confidence gate on one photo (d1-2 2026-09-24: "below" at
#: 0.31-0.36 on every live photo, and a 0.50 gate stopped the loop on the
#: first one)
DEFAULT_MARGIN = 0.10

#: the occlusion answer must lead by this much (P(hidden) - P(not hidden))
#: before the hand backs off: a yes/no a judge answers near 0.5 whatever it
#: sees is not evidence. Measured (the examples' jaws lab, d1-2 wrist
#: photos, 100 looks x 4 views with nothing hiding the object): P(hidden)
#: 0.30-0.72, median 0.53 — a 0.10 lead would have backed off on half of
#: them; a 0.5 lead (P >= 0.75) on none
DEFAULT_OCCLUSION_MARGIN = 0.5

#: the loop's own caps, besides the operator policy's nudge budget
DEFAULT_BUDGET_S = 6.0
DEFAULT_MAX_ITERATIONS = 12

#: after a correction, wait this long past the move's end before the next
#: photo, and refuse a photo written before the move ended
DEFAULT_SETTLE_S = 0.15

#: how close is close enough: one fine nudge step — inside it no correction
#: the grid can make is smaller than the error
DEFAULT_TOLERANCE_M = NUDGE_GRID_M[0]

#: the refinement step: the declaration may move this far (per axis)
#: without the hand moving
DEFAULT_REFINE_M = 0.5 * NUDGE_GRID_M[0]

#: how the servo ended
ALIGNED = "aligned"
#: observe_only: judged, recorded, nothing moved
OBSERVED = "judged_only"
NOT_VISIBLE = "not_visible"
#: a photo judge, and ``frame(side)`` gave no photo: not asked
NO_FRAME = "no_frame"
UNSURE = "unsure"
BUDGET = "nudge_budget"
#: the loop's wall-clock or iteration cap
TIMEOUT = "servo_budget"
#: the frame source kept giving photos written before the last move ended
STALE = "stale_frame"
UNMAPPABLE = "unmappable_direction"
REFUSED = "nudge_refused"
NOT_MOVED = "nudge_not_completed"

#: the action kinds, in priority order (:func:`choose`)
ACTIONS: Tuple[str, ...] = ("retreat", "turn", "xy", "approach")


@dataclass(frozen=True)
class Reading:
    """A judge's typed answers about one look. ``direction`` is always
    there (a distribution over :data:`CHOICES`); the rest only when the
    judge was asked:

    ``distance``  how far the object is from the opening's centre, as an
                  expected score (0 between the jaws .. 2 far off)
    ``depth``     a distribution over :data:`DEPTHS`
    ``occluded``  P(the fingers or the hand hide the object)
    ``turn_deg``  how far the jaws must turn, clockwise IN THE PHOTO, to close
                  across the object's narrowest width (expected value)
    ``fit``       P(the object fits between the fingers at this opening) —
                  recorded, not acted on
    """

    direction: Dict[str, float]
    distance: Optional[float] = None
    depth: Optional[Dict[str, float]] = None
    occluded: Optional[float] = None
    turn_deg: Optional[float] = None
    fit: Optional[float] = None

    def to_json(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {"direction": {k: round(float(v), 4) for k, v
                                             in self.direction.items()}}
        for name in ("distance", "occluded", "turn_deg", "fit"):
            value = getattr(self, name)
            if value is not None:
                out[name] = round(float(value), 4)
        if self.depth is not None:
            out["depth"] = {k: round(float(v), 4) for k, v in self.depth.items()}
        return out


def as_reading(answer: Union[Reading, Mapping[str, float]],
               choices: Sequence[str] = CHOICES) -> Reading:
    """A judge's answer as a :class:`Reading`, the direction normalised over
    ``choices``; an answer naming a choice the kit did not offer is an error."""
    reading = answer if isinstance(answer, Reading) else Reading(
        direction=dict(answer))
    unknown = set(reading.direction) - set(choices)
    if unknown:
        raise ValueError(f"the judge answered with choices the kit did "
                         f"not offer: {sorted(unknown)}")
    dist = {c: float(reading.direction.get(c, 0.0)) for c in choices}
    total = sum(dist.values())
    if total > 0:
        dist = {c: p / total for c, p in dist.items()}
    depth = reading.depth
    if depth is not None:
        extra = set(depth) - set(DEPTHS)
        if extra:
            raise ValueError(f"depth answers outside {DEPTHS}: {sorted(extra)}")
        depth = {d: float(depth.get(d, 0.0)) for d in DEPTHS}
        total = sum(depth.values())
        if total > 0:
            depth = {d: p / total for d, p in depth.items()}
    return dataclasses.replace(reading, direction=dist, depth=depth)


@dataclass(frozen=True)
class ServoLook:
    """What the judge is shown: the photo with the jaw opening and the
    object's outline drawn, and the geometry behind it so a stub can answer
    without a photo."""

    side: str
    object: str
    camera: Any                      # the WristCamera model at this posture
    #: the REFERENCE the judge is asked about: the jaw opening's centre (the
    #: approach axis at the object's depth); the declared centre's projection
    #: when the camera carries no flange pose to draw the jaws from
    u: float
    v: float
    depth_m: float
    image: Optional[Path]            # the photo WITH the drawing, or None
    photo: Optional[Path]            # the photo as taken, or None
    #: the tolerance circle at this depth, in pixels (a pinhole estimate)
    radius_px: float = 0.0
    #: the box the letters are drawn around: the jaw opening's bounding box
    box_px: Optional[Tuple[float, float, float, float]] = None
    choices: Tuple[str, ...] = CHOICES
    #: the declared centre (base, m) and the tolerance "on" means
    declared_p: Optional[Tuple[float, float, float]] = None
    tolerance_m: float = DEFAULT_TOLERANCE_M
    #: the jaw opening drawn (None: no flange pose, the declared centre marked)
    jaws: Optional[J.JawOpening] = None
    #: the outline drawn, and the declared shape's outline beside it
    outline: Optional[J.Outline] = None
    declared_outline: Optional[J.Outline] = None
    shape: str = "box"
    #: whether the jaws' orientation matters for this object
    elongated: bool = False
    #: the jaw point (base, m) — where the object is when the judge says "on"
    reference_p: Optional[Tuple[float, float, float]] = None

    def to_json(self) -> Dict[str, Any]:
        return {"side": self.side, "object": self.object, "u": self.u,
                "v": self.v, "depth_m": self.depth_m,
                #: which wrist mount the drawing was projected through
                "mount": getattr(self.camera, "mount", None),
                "radius_px": self.radius_px, "tolerance_m": self.tolerance_m,
                "declared_p": (None if self.declared_p is None
                               else [round(float(x), 5) for x in self.declared_p]),
                "reference_p": (None if self.reference_p is None
                                else [round(float(x), 5) for x in self.reference_p]),
                "box_px": None if self.box_px is None else list(self.box_px),
                "jaws": None if self.jaws is None else self.jaws.to_json(),
                "outline": None if self.outline is None else self.outline.to_json(),
                "shape": self.shape, "elongated": self.elongated,
                "image": None if self.image is None else str(self.image),
                "photo": None if self.photo is None else str(self.photo)}


Frame = Callable[[str], Optional[Path]]
Judge = Callable[[ServoLook], Union[Reading, Mapping[str, float]]]


@dataclass
class ServoStep:
    look: Dict[str, Any]
    distribution: Dict[str, float]
    choice: str
    confidence: float
    #: the base-frame displacement of the hand's tool point (dx, dy) when the
    #: step moved the hand (0, 0 for a pure turn); None when nothing moved
    step_m: Optional[Tuple[float, float]] = None
    #: "look" (a judgement that may move the hand) or "refine" (the same
    #: photo drawn with the jaw opening shifted by ``shift_m``)
    kind: str = "look"
    shift_m: Optional[Tuple[float, float]] = None
    plan: Optional[Dict[str, Any]] = None
    run: Optional[Dict[str, Any]] = None
    verdict: Optional[Dict[str, Any]] = None
    #: the inner loop: which iteration, when (s since the loop began), the
    #: reading averaged over the photos since the last move and how many they
    #: were, what was decided, and where the time went
    iteration: int = 0
    t_s: float = 0.0
    accumulated: Optional[Dict[str, float]] = None
    frames: int = 1
    decision: str = ""
    frame_age_s: Optional[float] = None
    timing_s: Optional[Dict[str, float]] = None
    #: the typed answers of this photo and their mean over the window
    reading: Optional[Dict[str, Any]] = None
    accumulated_reading: Optional[Dict[str, Any]] = None
    #: what the hand did and why: {"kind": one of :data:`ACTIONS`,
    #: "reason", "nudge": {dx, dy, dz, dyaw, frame}}
    action: Optional[Dict[str, Any]] = None

    def to_json(self) -> Dict[str, Any]:
        return dataclasses.asdict(self)


@dataclass
class ServoReport:
    side: str
    object: str
    outcome: str
    detail: str = ""
    steps: List[ServoStep] = field(default_factory=list)
    #: the loop's wall-clock time and number of judged photos
    elapsed_s: float = 0.0
    iterations: int = 0

    @property
    def aligned(self) -> bool:
        return self.outcome == ALIGNED

    @property
    def skipped(self) -> bool:
        """No judgement was made (no wrist photo for a photo judge)."""
        return self.outcome == NO_FRAME

    def to_json(self) -> Dict[str, Any]:
        out = {"side": self.side, "object": self.object,
               "outcome": self.outcome, "detail": self.detail,
               "elapsed_s": round(self.elapsed_s, 3),
               "iterations": self.iterations,
               #: the moves made, by kind (an action refused or cut short
               #: by a budget is in its step, not counted here)
               "actions": {k: sum(1 for s in self.steps if s.action
                                  and s.step_m is not None
                                  and s.action["kind"] == k) for k in ACTIONS},
               "steps": [s.to_json() for s in self.steps]}
        if self.skipped:
            out["skipped"] = "no wrist frame"
        return out

    def to_text(self) -> str:
        n = sum(1 for s in self.steps if s.step_m is not None)
        if self.skipped:
            return (f"the wrist look's judge was not asked about "
                    f"{self.object!r}: {self.detail}")
        if self.outcome == OBSERVED:
            return (f"the wrist look's judge said {self.detail} for "
                    f"{self.object!r} (recorded only; nothing moved)")
        if self.aligned:
            return (f"the {self.side} hand was aligned on {self.object!r} by "
                    f"the wrist look ({n} correction{'s' if n != 1 else ''})")
        return (f"the wrist look could not align the {self.side} hand on "
                f"{self.object!r}: {self.outcome} — {self.detail} "
                f"({n} correction{'s' if n != 1 else ''} made)")


def servo_line(servo: Optional[Mapping[str, Any]]) -> str:
    """One line per judged stroke, for an operator watching a run: what the
    judge said and what the kit did with it (from ``record.servo``)::

        servo judged: on 0.81 (left 0.12, right 0.04, ...) — recorded only
        servo aligned: on 0.97 after 2 step(s)
        servo skipped: no wrist frame
    """
    if not servo:
        return "servo: not run"
    if servo.get("skipped"):
        return f"servo skipped: {servo['skipped']}"
    looks = [s for s in servo.get("steps", []) if s.get("kind", "look") == "look"]
    said = ""
    if looks:
        dist = looks[-1].get("accumulated") or looks[-1]["distribution"]
        ranked = sorted(dist.items(), key=lambda kv: -kv[1])
        said = (f"{ranked[0][0]} {ranked[0][1]:.2f} ("
                + ", ".join(f"{c} {p:.2f}" for c, p in ranked[1:]) + ")")
    moved = sum(1 for s in servo.get("steps", []) if s.get("step_m"))
    kinds = {k: n for k, n in (servo.get("actions") or {}).items() if n}
    how = ("; " + ", ".join(f"{n} {k}" for k, n in kinds.items())
           if kinds and set(kinds) != {"xy"} else "")
    outcome = servo.get("outcome")
    if outcome == OBSERVED:
        return f"servo judged: {said} — recorded only"
    if outcome == ALIGNED:
        return f"servo aligned: {said} after {moved} step(s){how}"
    return (f"servo {outcome}: {servo.get('detail', '')}"
            + (f"; last judged {said}" if said else "")
            + f" ({moved} step(s){how})")


def footprint_px(camera: Any, obj: Any, frames: Any, *,
                 margin_m: float = 0.0) -> Optional[Tuple[float, float, float, float]]:
    """The image-space bounding box (u0, v0, u1, v1) of the declared object's
    size box grown by ``margin_m`` on every side, every corner projected.
    None when a corner is behind the lens. (The reference drawn when the
    camera carries no flange pose to draw the jaws from.)"""
    from ..perception import NotOnThePlane  # noqa: PLC0415
    p, r = obj.pose_in_base(frames)
    half = np.asarray(obj.size, dtype=float).reshape(3) / 2.0 + float(margin_m)
    us, vs = [], []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                corner = np.asarray(p, dtype=float) + r.apply(half * [sx, sy, sz])
                try:
                    u, v = camera.project(corner)
                except NotOnThePlane:
                    return None
                us.append(float(u))
                vs.append(float(v))
    return (min(us), min(vs), max(us), max(vs))


def mark(photo: Path, u: float, v: float, out: Path, *,
         box_px: Optional[Tuple[float, float, float, float]] = None,
         radius_px: float = 0.0, arm_px: int = 25, width: int = 5) -> Path:
    """A recorded look drawn again from its numbers: a green cross at
    ``(u, v)`` and the box (or the tolerance circle). The live servo draws
    the jaw opening and the outline instead (:func:`.jaws.draw`)."""
    try:
        from PIL import Image, ImageDraw  # noqa: PLC0415
    except ImportError as exc:  # pragma: no cover - environment
        raise RuntimeError("drawing the wrist mark needs pillow (the "
                           "`perception` extra)") from exc
    image = Image.open(photo).convert("RGB")
    draw = ImageDraw.Draw(image)
    u, v = float(u), float(v)
    green = (0, 230, 0)
    draw.line([(u - arm_px, v), (u + arm_px, v)], fill=green, width=width)
    draw.line([(u, v - arm_px), (u, v + arm_px)], fill=green, width=width)
    if box_px is not None:
        draw.rectangle([box_px[0], box_px[1], box_px[2], box_px[3]],
                       outline=green, width=width)
    elif radius_px > 0.0:
        r = float(radius_px)
        draw.ellipse([u - r, v - r, u + r, v + r], outline=green, width=width)
    out.parent.mkdir(parents=True, exist_ok=True)
    image.save(out)
    return out


def image_direction_in_base(camera: Any, choice: str) -> Optional[np.ndarray]:
    """The table-plane unit vector (base frame, z = 0) the object lies along
    when the judge says it appears ``choice`` of the opening. ``None`` when
    that image axis is nearly vertical in base, so a horizontal step cannot
    follow it (a camera looking along the table sees "above" as "farther
    up")."""
    axis = _IMAGE_AXIS.get(choice)
    if axis is None:
        return None
    optical = np.array([axis[0], axis[1], 0.0], dtype=float)
    in_base = np.asarray(camera.r.apply(optical), dtype=float)
    horizontal = np.array([in_base[0], in_base[1], 0.0])
    norm = float(np.linalg.norm(horizontal))
    if norm < MIN_HORIZONTAL:
        return None
    return horizontal / norm


def turn_sign(camera: Any) -> float:
    """+1 when a positive yaw of the hand about its approach axis turns the
    jaws CLOCKWISE in the photo (image y down), -1 when anticlockwise: the
    sign that maps a judged turn in the image to a ``Nudge.dyaw``."""
    z_tool = (camera.flange_r.as_matrix()[:, 2]
              if getattr(camera, "flange_r", None) is not None else None)
    z_cam = camera.r.as_matrix()[:, 2]
    if z_tool is None:
        return 1.0
    return 1.0 if float(np.dot(z_tool, z_cam)) >= 0.0 else -1.0


#: the displacement choices, and the one each contradicts
_OPPOSITE = {"left": "right", "right": "left", "above": "below",
             "below": "above"}

#: the servo's log: one line per judged photo, and one when the loop ends
LOG = logging.getLogger(__name__)


def accumulate(window: Sequence[Union[Reading, Mapping[str, float]]]
               ) -> Union[Reading, Dict[str, float]]:
    """The mean of the answers in ``window`` (photos at one pose). Bare
    distributions average to a distribution; readings to a reading, each
    typed answer over the photos that carried it."""
    if not window:
        raise ValueError("an empty window")
    if not any(isinstance(w, Reading) for w in window):
        out = {c: 0.0 for c in CHOICES}
        for dist in window:
            for c in CHOICES:
                out[c] += float(dist.get(c, 0.0)) / len(window)
        return out
    readings = [as_reading(w) for w in window]
    direction = {c: sum(r.direction.get(c, 0.0) for r in readings) / len(readings)
                 for c in CHOICES}

    def mean(name):
        values = [getattr(r, name) for r in readings if getattr(r, name) is not None]
        return None if not values else sum(values) / len(values)
    depths = [r.depth for r in readings if r.depth is not None]
    depth = None if not depths else {
        d: sum(x[d] for x in depths) / len(depths) for d in DEPTHS}
    return Reading(direction=direction, distance=mean("distance"), depth=depth,
                   occluded=mean("occluded"), turn_deg=mean("turn_deg"),
                   fit=mean("fit"))


def decide(acc: Mapping[str, float], frames: int, *, window: int,
           margin: float) -> Tuple[str, str]:
    """What the accumulated DIRECTION ``acc`` over ``frames`` photos says:

    ``("on", why)``          "on" leads the runner-up by ``margin``
    ``(direction, why)``     a direction leads "on" AND its opposite by
                             ``margin``: step that way
    ``("not_visible", why)`` "not in the photo" leads the runner-up by it
    ``("more", why)``        not decisive yet, and fewer than ``window``
                             photos: take another
    ``("flat", why)``        not decisive after ``window`` photos: no blind
                             step

    Confidence routes, it does not gate: a 0.31 answer that holds its lead
    over three photos is evidence, a 0.60 answer contradicted by its
    opposite is not."""
    ranked = sorted(acc.items(), key=lambda kv: -kv[1])
    top, p_top = ranked[0]
    runner = ranked[1][1]
    if top in ("on", "not_visible") and p_top - runner >= margin:
        return top, f"{top} leads by {p_top - runner:.2f}"
    if top in _OPPOSITE:
        lead_on = p_top - acc["on"]
        lead_back = p_top - acc[_OPPOSITE[top]]
        if lead_on >= margin and lead_back >= margin:
            return top, (f"{top} leads on by {lead_on:.2f}, "
                         f"{_OPPOSITE[top]} by {lead_back:.2f}")
    why = (f"{top} {p_top:.2f} leads by {p_top - runner:.2f} < {margin:.2f}"
           f" over {frames} photo{'s' if frames != 1 else ''}")
    return ("flat" if frames >= window else "more"), why


@dataclass(frozen=True)
class Choice:
    """What :func:`choose` decided: ``kind`` is an action of
    :data:`ACTIONS`, or ``on`` / ``not_visible`` / ``more`` / ``flat``;
    ``direction`` the image direction of an ``xy`` step, ``sign`` +1 / -1
    of a ``turn`` (clockwise in the photo) or of an ``approach`` (+1
    toward the object, -1 back)."""

    kind: str
    why: str
    direction: str = ""
    sign: float = 0.0


def choose(acc: Reading, frames: int, *, look: ServoLook, window: int,
           margin: float, occlusion_margin: float = DEFAULT_OCCLUSION_MARGIN,
           turned_rad: float = 0.0,
           max_turn_rad: float = DEFAULT_MAX_TURN_RAD,
           yaw_step_rad: float = DEFAULT_YAW_STEP_RAD,
           approach_room_m: float = 0.0) -> Choice:
    """The servo's fixed priority over an accumulated :class:`Reading`:

    1. ``retreat``   P(occluded) leads P(not occluded) by
                     ``occlusion_margin``
    2. ``turn``      the object is elongated, the judged turn is at least
                     half a yaw step and the servo has not turned
                     ``max_turn_rad`` yet
    3. ``xy``        the direction says a side (:func:`decide`)
    4. ``approach``  the direction says ``on`` and the depth answer puts the
                     object ahead of the fingers (and there is
                     ``approach_room_m`` left), or behind them (-1)

    and ``on`` when the direction says on and nothing above asks to move.
    ``more`` / ``flat`` / ``not_visible`` as :func:`decide`."""
    if acc.occluded is not None and \
            (2.0 * acc.occluded - 1.0) >= occlusion_margin:
        return Choice("retreat", f"occluded {acc.occluded:.2f}", sign=-1.0)
    verdict, why = decide(acc.direction, frames, window=window, margin=margin)
    if verdict in ("more", "flat", "not_visible"):
        return Choice(verdict, why)
    if (look.elongated and acc.turn_deg is not None
            and abs(math.radians(acc.turn_deg)) >= 0.5 * yaw_step_rad
            and abs(turned_rad) + yaw_step_rad <= max_turn_rad + 1e-9):
        return Choice("turn", f"elongated, turn {acc.turn_deg:+.0f} deg in "
                              f"the photo ({why})",
                      sign=1.0 if acc.turn_deg > 0 else -1.0)
    if verdict != "on":
        return Choice("xy", why, direction=verdict)
    if acc.depth is not None:
        ranked = sorted(acc.depth.items(), key=lambda kv: -kv[1])
        (top, p_top), p_second = ranked[0], ranked[1][1]
        if p_top - p_second >= margin:
            if top == "ahead" and approach_room_m > 1e-6:
                return Choice("approach", f"centred ({why}); ahead of the "
                              f"fingers {p_top:.2f}", sign=1.0)
            if top == "behind":
                return Choice("approach", f"centred ({why}); behind the "
                              f"fingers {p_top:.2f}", sign=-1.0)
    return Choice("on", why)


def _ranked(dist: Mapping[str, float]) -> str:
    ranked = sorted(dist.items(), key=lambda kv: -kv[1])
    return (f"{ranked[0][0]} {ranked[0][1]:.2f} ("
            + ", ".join(f"{c} {p:.2f}" for c, p in ranked[1:]) + ")")


def iteration_line(step: ServoStep, *, side: str) -> str:
    """The live log line of one judged photo::

        t=+0.83s iter 3 side=right judge below 0.41 (on 0.30, ...) acc[2]
        below 0.38 -> xy +10mm along image-below -> base (dx,dy)=(+0.000,
        -0.010) m (reason) [frame 0.21s judge 0.17s step 0.44s]
    """
    acc = step.accumulated or step.distribution
    head = (f"t=+{step.t_s:.2f}s iter {step.iteration} side={side} judge "
            f"{_ranked(step.distribution)}")
    extra = step.reading or {}
    typed = [f"{k} {extra[k]:+.2f}" if k == "turn_deg" else f"{k} {extra[k]:.2f}"
             for k in ("distance", "occluded", "turn_deg", "fit") if k in extra]
    if extra.get("depth"):
        typed.append("depth " + _ranked(extra["depth"]).split(" (")[0])
    if typed:
        head += " {" + ", ".join(typed) + "}"
    if step.frames > 1:
        top = max(acc, key=acc.get)
        head += f" acc[{step.frames}] {top} {acc[top]:.2f}"
    action = step.action or {}
    if step.step_m is not None and action.get("kind", "xy") == "xy":
        size = math.hypot(*step.step_m) * 1000.0
        head += (f" -> step {size:+.0f}mm along image-{step.choice} -> base "
                 f"(dx,dy)=({step.step_m[0]:+.3f},{step.step_m[1]:+.3f}) m")
    elif step.step_m is not None:
        nudge = action.get("nudge", {})
        if action["kind"] == "turn":
            head += f" -> turn {math.degrees(nudge.get('dyaw', 0.0)):+.0f} deg"
        else:
            head += (f" -> {action['kind']} {nudge.get('dz', 0.0) * 1000:+.0f}mm "
                     f"along the approach axis")
    if step.step_m is not None and action.get("reason"):
        head += f" ({action['reason']})"
    if step.step_m is None and step.decision:
        head += f" -> {step.decision}"
    if step.frame_age_s is not None:
        head += f" (photo {step.frame_age_s:.2f}s after the move)"
    if step.timing_s:
        head += " [" + " ".join(f"{k} {v:.2f}s" for k, v in
                                step.timing_s.items()) + "]"
    return head


def exit_line(report: ServoReport) -> str:
    """The loop's last log line: outcome, why, and the totals."""
    moved = sum(1 for s in report.steps if s.step_m is not None)
    per = report.elapsed_s / report.iterations if report.iterations else 0.0
    return (f"t=+{report.elapsed_s:.2f}s servo {report.side} "
            f"{report.object!r}: {report.outcome} — {report.detail} "
            f"({report.iterations} photo{'s' if report.iterations != 1 else ''}"
            f", {moved} step{'s' if moved != 1 else ''}, {per:.2f} s/iteration)")


class Servo:
    """Align one hand on one object: a closed loop at the photo/judge rate.
    Built once per run; :meth:`align` is called by the loop before every
    stroke of :data:`SERVO_VERBS`.

    Each iteration: a FRESH photo (written after the last move ended, and
    ``settle_s`` after it), drawn (the jaw opening and the object's outline);
    the judge's reading; the mean over the photos since the last move
    (``window`` of them at most); then :func:`choose` — retreat, turn, step,
    approach, stop ``on``, or take another photo. The loop ends on ``on``,
    "not visible", evidence that stays flat over ``window`` photos, the
    operator policy's nudge budget, ``max_iterations`` photos or
    ``budget_s`` seconds. One log line per photo (``log``, default the
    ``manipulation_kit.agent.servo`` logger at INFO) and one at the end.

    ``shapes`` names the declared shape of an object (:data:`.jaws.SHAPES`,
    default ``box``) — an operator statement, not a model's: the
    declaration's size box is all the scene tools carry. ``object_outline``
    is the outline seam (:data:`.jaws.ObjectOutline`, default the declared
    shape); when it finds nothing the declared shape is drawn and the look
    says so (``outline.extra["fallback"]``)."""

    def __init__(self, frame: Frame, judge: Judge, *,
                 steps_m: Sequence[float] = DEFAULT_STEPS_M,
                 window: int = DEFAULT_WINDOW,
                 margin: float = DEFAULT_MARGIN,
                 budget_s: float = DEFAULT_BUDGET_S,
                 max_iterations: int = DEFAULT_MAX_ITERATIONS,
                 settle_s: float = DEFAULT_SETTLE_S,
                 tolerance_m: float = DEFAULT_TOLERANCE_M,
                 out_dir: Optional[Path] = None,
                 observe_only: bool = False, refine: bool = False,
                 refine_m: float = DEFAULT_REFINE_M,
                 shapes: Optional[Mapping[str, str]] = None,
                 object_outline: J.ObjectOutline = J.declared,
                 approach_step_m: float = DEFAULT_APPROACH_STEP_M,
                 retreat_step_m: float = DEFAULT_RETREAT_STEP_M,
                 max_approach_m: float = DEFAULT_MAX_APPROACH_M,
                 approach_clearance_m: float = DEFAULT_APPROACH_CLEARANCE_M,
                 yaw_step_rad: float = DEFAULT_YAW_STEP_RAD,
                 max_turn_rad: float = DEFAULT_MAX_TURN_RAD,
                 occlusion_margin: float = DEFAULT_OCCLUSION_MARGIN,
                 log: Optional[Callable[[str], None]] = None,
                 clock: Callable[[], float] = time.time,
                 sleep: Callable[[float], None] = time.sleep):
        grid = max(NUDGE_GRID_M)
        if not steps_m or any(not 0.0 < float(s) <= grid for s in steps_m):
            raise ValueError(f"steps_m must be nudge-grid steps, got "
                             f"{list(steps_m)!r}")
        for name, value in (("approach_step_m", approach_step_m),
                            ("retreat_step_m", retreat_step_m)):
            if not 0.0 < float(value) <= grid:
                raise ValueError(f"{name} must be a nudge-grid step")
        if not 0.0 < float(yaw_step_rad) <= NUDGE_MAX_YAW_RAD + 1e-12:
            raise ValueError("yaw_step_rad is positive, at most the nudge's "
                             "yaw cap")
        if float(max_turn_rad) < 0.0 or float(max_approach_m) < 0.0 or \
                float(approach_clearance_m) < 0.0:
            raise ValueError("max_turn_rad, max_approach_m and "
                             "approach_clearance_m are not negative")
        if int(window) < 1 or int(max_iterations) < 1:
            raise ValueError("window and max_iterations are at least 1")
        if not 0.0 <= float(margin) <= 1.0 or \
                not 0.0 <= float(occlusion_margin) <= 1.0:
            raise ValueError("margin and occlusion_margin are differences "
                             "of probabilities")
        if not float(budget_s) > 0.0 or float(settle_s) < 0.0:
            raise ValueError("budget_s is positive, settle_s not negative")
        if not 0.0 < float(tolerance_m) <= grid:
            raise ValueError("tolerance_m is a small positive distance")
        if not 0.0 < float(refine_m) <= float(tolerance_m):
            raise ValueError("refine_m is a positive distance no larger than "
                             "tolerance_m")
        shapes = dict(shapes or {})
        bad = {k: v for k, v in shapes.items() if v not in J.SHAPES}
        if bad:
            raise ValueError(f"shapes must be among {J.SHAPES}, got {bad}")
        self.frame, self.judge = frame, judge
        #: whether the judge needs the photo (:func:`photoless` says no)
        self.reads_photo = bool(getattr(judge, "reads_photo", True))
        self.steps_m = tuple(float(s) for s in steps_m)
        self.window, self.margin = int(window), float(margin)
        self.budget_s, self.max_iterations = float(budget_s), int(max_iterations)
        self.settle_s = float(settle_s)
        self.tolerance_m = float(tolerance_m)
        self.out_dir = Path(out_dir) if out_dir is not None else None
        self.observe_only, self.refine = bool(observe_only), bool(refine)
        self.refine_m = float(refine_m)
        self.shapes = shapes
        self.object_outline = object_outline
        self.approach_step_m = float(approach_step_m)
        self.retreat_step_m = float(retreat_step_m)
        self.max_approach_m = float(max_approach_m)
        self.approach_clearance_m = float(approach_clearance_m)
        self.yaw_step_rad, self.max_turn_rad = float(yaw_step_rad), float(max_turn_rad)
        self.occlusion_margin = float(occlusion_margin)
        self.log = log if log is not None else LOG.info
        self.clock, self.sleep = clock, sleep
        self._n = 0
        #: the turn and directory of the current :meth:`align` (the drawn
        #: photos are named ``turn{N}_servo_{side}[ _k].png`` beside the trace)
        self._turn: Optional[int] = None
        self._where: Optional[Path] = None
        self._in_turn = 0

    # -- one look ------------------------------------------------------------ #
    def look(self, robot, world, side: str, name: str) -> Tuple[Optional[ServoLook], str]:
        """Project, photograph (``frame(side)``) and draw: the look the judge
        is asked about, or (None, why)."""
        camera = robot.cameras(world).get(f"{side}_wrist")
        item = world.find(name)
        if camera is None or item is None:
            return None, f"no {side}_wrist camera model, or {name!r} is not in the scene"
        seen = camera.project_object(item, world.frames)
        if not seen.visible:
            return None, (f"{name!r} is not in the {side}_wrist frame from "
                          f"this posture ({seen.reason})")
        photo = self.frame(side)
        if photo is None and self.reads_photo:
            return None, NO_FRAME
        return self.look_at(camera, item, world.frames, photo, side=side,
                            gap=J.jaw_gap(world.gripper(side))), ""

    def look_at(self, camera, item, frames, photo: Optional[Path], *,
                side: str, tolerance_m: Optional[float] = None,
                tag: str = "", gap: Optional[Tuple[float, str]] = None,
                shift_m: Tuple[float, float] = (0.0, 0.0),
                outline: Optional[J.Outline] = None) -> Optional[ServoLook]:
        """Draw on ``photo`` (may be None) the jaw opening at ``item``'s depth
        (at the jaw gap ``gap``: (metres, source), default the hand's nominal
        opening; shifted by ``shift_m`` along the jaw axis / across it) and
        ``item``'s outline (``outline``, else the :attr:`object_outline`
        seam, else the declared shape): one look, no motion. Public so a
        recorded photo can be judged again offline. None when the item does
        not project into the image."""
        tolerance = self.tolerance_m if tolerance_m is None else float(tolerance_m)
        seen = camera.project_object(item, frames)
        if not seen.visible:
            return None
        p = np.asarray(item.pose_in_base(frames)[0], dtype=float)
        gap_m, gap_source = gap if gap is not None else J.jaw_gap(None)
        depth = J.approach_depth(camera, p)
        opening = (None if depth is None else J.jaw_opening(
            camera, gap_m=gap_m, depth_m=depth, gap_source=gap_source,
            shift_m=shift_m))
        shape = self.shapes.get(item.name, "box")
        declared_outline = J.declared_outline(camera, item, frames, shape)
        # the tolerance at this depth, in pixels (a pinhole estimate; the
        # circle is a drawing, not a measurement)
        radius = float(camera.fx) * tolerance / max(float(seen.depth_m), 0.02)
        if opening is not None:
            u, v = opening.centre
            box = opening.bbox()
            reference = opening.point
        else:
            u, v = float(seen.u), float(seen.v)
            box = footprint_px(camera, item, frames, margin_m=tolerance)
            reference = tuple(float(x) for x in p)
        look = ServoLook(side=side, object=item.name, camera=camera,
                         u=float(u), v=float(v), depth_m=float(seen.depth_m),
                         image=None,
                         photo=None if photo is None else Path(photo),
                         radius_px=radius, box_px=box,
                         declared_p=tuple(float(x) for x in p),
                         tolerance_m=tolerance, jaws=opening,
                         declared_outline=declared_outline, shape=shape,
                         reference_p=tuple(float(x) for x in reference))
        if outline is None:
            outline = self.object_outline(
                None if photo is None else Path(photo), look)
            if outline is None and declared_outline is not None:
                outline = dataclasses.replace(
                    declared_outline, extra={**declared_outline.extra,
                                             "fallback": "not found"})
        if outline is None or outline.source.startswith("declared"):
            # the declaration's own footprint, not its perspective silhouette
            a, b = sorted(float(x) for x in np.asarray(item.size)[:2])
            elongated = shape != "cylinder" and b >= J.ELONGATED_RATIO * a
        else:
            elongated = bool(outline.elongated)
        look = dataclasses.replace(look, outline=outline, elongated=elongated)
        if photo is not None:
            out = self._marked_path(Path(photo), side, tag)
            if opening is not None:
                image = J.draw(Path(photo), out, jaws=opening, outline=outline)
            else:
                image = mark(Path(photo), u, v, out, box_px=box,
                             radius_px=radius)
            look = dataclasses.replace(look, image=image)
        return look

    def _marked_path(self, photo: Path, side: str, tag: str) -> Path:
        """``turn{N}_servo_{side}.png`` for a turn's first drawn photo (the
        next ``_2``, ``_3``, ...), beside the trace; outside a turn
        ``servo{n:03d}_{side}{tag}.png`` next to ``out_dir`` or the photo."""
        self._n += 1
        out_dir = self.out_dir or self._where or photo.parent
        if self._turn is None:
            return out_dir / f"servo{self._n:03d}_{side}{tag}.png"
        self._in_turn += 1
        suffix = "" if self._in_turn == 1 else f"_{self._in_turn}"
        return out_dir / f"turn{self._turn}_servo_{side}{tag}{suffix}.png"

    def read(self, look: ServoLook) -> Reading:
        """The judge's answer about ``look`` as a normalised :class:`Reading`."""
        return as_reading(self.judge(look), look.choices)

    def ask(self, look: ServoLook) -> Tuple[Dict[str, float], str, float]:
        """The direction alone: (distribution, top choice, its probability)."""
        dist = self.read(look).direction
        choice = max(dist, key=dist.get)
        return dist, choice, dist[choice]

    # -- the loop ------------------------------------------------------------ #
    def align(self, *, robot, policy: OperatorPolicy, state: PolicyState,
              side: str, name: str, settings: Mapping[str, Any],
              run_plan: Callable[..., Any], turn: Optional[int] = None,
              out_dir: Optional[Path] = None) -> ServoReport:
        """Judge (and, unless ``observe_only``, align) ``side`` on ``name``.
        ``turn`` / ``out_dir`` name the drawn photos (``out_dir`` is used
        when the servo was built without one — the loop passes the trace's
        directory)."""
        self._turn, self._in_turn = turn, 0
        self._where = None if out_dir is None else Path(out_dir)
        started = self.clock()
        report = ServoReport(side=side, object=name, outcome=UNSURE)
        try:
            self._align(report, started, robot=robot, policy=policy,
                        state=state, side=side, name=name, settings=settings,
                        run_plan=run_plan)
        finally:
            self._turn, self._where = None, None
            report.elapsed_s = self.clock() - started
            report.iterations = sum(1 for s in report.steps if s.kind == "look")
        if not report.skipped:
            self.log(exit_line(report))
        return report

    def _grab(self, robot, world, side, name, moved_at):
        """One look whose photo was written after ``moved_at`` (the end of
        the last move): up to three grabs. ``(look, why, age_s, grab_s)``."""
        started, age, look, why = self.clock(), None, None, ""
        for _attempt in range(3):
            look, why = self.look(robot, world, side, name)
            if look is None or look.photo is None or moved_at is None:
                break
            try:
                age = Path(look.photo).stat().st_mtime - moved_at
            except OSError:
                age = None
                break
            if age >= 0.0:
                break
            look, why = None, STALE
        return look, why, age, self.clock() - started

    def approach_room(self, world, look: ServoLook, approached_m: float) -> float:
        """How much farther (m) the hand may come along the approach axis:
        the servo's own ``max_approach_m`` less what it has come, and the
        room left before the fingertips are ``approach_clearance_m`` from
        the object's near face or from the top of any support surface ahead
        of them. 0 when the camera carries no flange pose."""
        camera = look.camera
        if getattr(camera, "flange_p", None) is None:
            return 0.0
        from ..hands.d1.parallel_gripper.description import (  # noqa: PLC0415
            PAD_TIP_Z_M)
        z = camera.flange_r.as_matrix()[:, 2]
        room = self.max_approach_m - approached_m
        faces = []
        item = world.find(look.object)
        things = ([item] if item is not None else []) + list(world.surfaces())
        for thing in things:
            p = np.asarray(thing.pose_in_base(world.frames)[0], dtype=float)
            near = (float(np.dot(p - camera.flange_p, z))
                    - thing.extent_along(z, world.frames) / 2.0)
            faces.append(near)
        for near in faces:
            room = min(room, near - self.approach_clearance_m - PAD_TIP_Z_M)
        return max(0.0, room)

    def _align(self, report: ServoReport, started: float, *, robot, policy,
               state, side, name, settings, run_plan) -> None:
        signs: Dict[str, float] = {}       # axis -> sign of the last step
        fine = False
        window: List[Reading] = []
        moved_at: Optional[float] = None
        iteration = 0
        turned, approached = 0.0, 0.0
        while True:
            if iteration >= self.max_iterations or (
                    iteration and self.clock() - started >= self.budget_s):
                report.outcome = TIMEOUT
                report.detail = (f"{iteration} photos in "
                                 f"{self.clock() - started:.1f} s without an "
                                 f"answer (caps: {self.max_iterations} photos,"
                                 f" {self.budget_s:.1f} s)")
                return
            world = robot.world()
            look, why, age, grab_s = self._grab(robot, world, side, name,
                                                moved_at)
            if look is None and why == NO_FRAME:
                report.outcome = NO_FRAME
                report.detail = (f"no {side}_wrist photo from the frame source "
                                 f"(no snapshotter, or the grab gave none), and "
                                 f"this judge reads the photo")
                return
            if look is None and why == STALE:
                report.outcome = STALE
                report.detail = (f"three {side}_wrist photos in a row were "
                                 f"written before the last move ended")
                return
            if look is None:
                report.outcome, report.detail = NOT_VISIBLE, why
                return
            iteration += 1
            judged = self.clock()
            reading = self.read(look)
            judge_s = self.clock() - judged
            window = (window + [reading])[-self.window:]
            acc = accumulate(window)
            dist = reading.direction
            top = max(dist, key=dist.get)
            step = ServoStep(look=look.to_json(), distribution=dist,
                             choice=top, confidence=dist[top],
                             iteration=iteration,
                             t_s=round(judged - started, 3),
                             accumulated=acc.direction, frames=len(window),
                             frame_age_s=None if age is None else round(age, 3),
                             timing_s={"frame": round(grab_s, 3),
                                       "judge": round(judge_s, 3)},
                             reading=reading.to_json(),
                             accumulated_reading=acc.to_json())
            report.steps.append(step)
            room = self.approach_room(world, look, approached)
            chosen = choose(acc, len(window), look=look, window=self.window,
                            margin=self.margin,
                            occlusion_margin=self.occlusion_margin,
                            turned_rad=turned,
                            max_turn_rad=self.max_turn_rad,
                            yaw_step_rad=self.yaw_step_rad,
                            approach_room_m=room)
            if self.observe_only and chosen.kind != "more":
                step.decision = f"recorded only (would {chosen.kind}: {chosen.why})"
                self.log(iteration_line(step, side=side))
                report.outcome = OBSERVED
                best = max(acc.direction, key=acc.direction.get)
                report.detail = (f"{best!r} at {acc.direction[best]:.2f} over "
                                 f"{len(window)} photo"
                                 f"{'s' if len(window) != 1 else ''}")
                return
            if chosen.kind in ("more", "flat"):
                step.decision = ("another photo" if chosen.kind == "more"
                                 else "no blind step")
                self.log(iteration_line(step, side=side) + f" ({chosen.why})")
                if chosen.kind == "flat":
                    report.outcome = UNSURE
                    report.detail = (f"the evidence stayed flat over "
                                     f"{len(window)} photos ({chosen.why}); no "
                                     f"blind step")
                    return
                continue
            if chosen.kind == "on":
                step.decision = "between the jaws"
                self.log(iteration_line(step, side=side))
                state.aimed(side, name)
                state.look(side, name, world.arm(side).joints)
                self._declare_at_jaws(robot, world, look, acc.direction["on"])
                report.outcome = ALIGNED
                report.detail = (f"between the jaws at {acc.direction['on']:.2f}"
                                 f" over {len(window)} photo"
                                 f"{'s' if len(window) != 1 else ''}")
                if self.refine:
                    report.detail += self._refine(robot, robot.world(), look,
                                                  name, report)
                return
            if chosen.kind == "not_visible":
                step.decision = "not visible"
                self.log(iteration_line(step, side=side))
                report.outcome = NOT_VISIBLE
                report.detail = (f"the judge does not see {name!r} in the "
                                 f"{side}_wrist photo "
                                 f"({acc.direction['not_visible']:.2f} over "
                                 f"{len(window)} photos)")
                return
            moving = self.clock()
            if chosen.kind == "xy":
                direction = image_direction_in_base(look.camera, chosen.direction)
                if direction is None:
                    step.decision = "unmappable"
                    self.log(iteration_line(step, side=side))
                    report.outcome = UNMAPPABLE
                    report.detail = (f"image {chosen.direction!r} is nearly "
                                     f"vertical in base from this posture; no "
                                     f"table-plane step follows it")
                    return
                axis = "u" if chosen.direction in ("left", "right") else "v"
                sign = -1.0 if chosen.direction in ("left", "above") else 1.0
                if axis in signs and signs[axis] != sign:
                    fine = True         # overshot: switch to the fine step
                signs[axis] = sign
                size = self.steps_m[-1] if fine else self.steps_m[0]
                step.choice = chosen.direction  # what the step follows
                delta = direction * size
                nudge = Nudge(side=side, dx=float(delta[0]), dy=float(delta[1]),
                              frame="base")
            elif chosen.kind == "turn":
                dyaw = chosen.sign * self.yaw_step_rad * turn_sign(look.camera)
                nudge = Nudge(side=side, dyaw=float(dyaw), frame="tool")
            elif chosen.kind == "retreat":
                nudge = Nudge(side=side, dz=-self.retreat_step_m, frame="tool")
            else:
                size = self.approach_step_m
                if chosen.sign > 0:     # on the grid, inside the room left
                    size = max([g for g in NUDGE_GRID_M
                                if g <= min(size, room) + 1e-9] or [0.0])
                if size <= 0.0:
                    chosen = Choice("on", chosen.why + "; no approach room")
                    step.decision = "between the jaws (no approach room)"
                    self.log(iteration_line(step, side=side))
                    state.aimed(side, name)
                    state.look(side, name, world.arm(side).joints)
                    self._declare_at_jaws(robot, world, look,
                                          acc.direction["on"])
                    report.outcome = ALIGNED
                    report.detail = (f"between the jaws at "
                                     f"{acc.direction['on']:.2f}; no approach "
                                     f"room left")
                    return
                nudge = Nudge(side=side, dz=float(chosen.sign * size),
                              frame="tool")
            step.action = {"kind": chosen.kind, "reason": chosen.why,
                           "nudge": {"dx": nudge.dx, "dy": nudge.dy,
                                     "dz": nudge.dz, "dyaw": nudge.dyaw,
                                     "frame": nudge.frame}}
            done = self._correct(robot, world, policy, state, side, name,
                                 nudge, settings, run_plan, step, look)
            step.timing_s["step"] = round(self.clock() - moving, 3)
            step.decision = (f"{chosen.kind} {chosen.direction}".strip()
                             if step.step_m is not None else "no step")
            self.log(iteration_line(step, side=side))
            if done is not None:
                report.outcome, report.detail = done
                return
            if chosen.kind == "turn":
                turned += nudge.dyaw
            elif chosen.kind == "approach":
                approached += nudge.dz
            window = []                 # a new pose: the old photos are gone
            moved_at = self.clock()
            if self.reads_photo and self.settle_s > 0.0:
                self.sleep(self.settle_s)

    def _declare_at_jaws(self, robot, world, look: ServoLook,
                         confidence: float) -> None:
        """"On": the object is where the jaws will close — re-declared at the
        jaw point (a JUDGEMENT, ``provenance="judged"``). Nothing to do when
        no jaw opening was drawn (the declared centre was the reference)."""
        if look.jaws is None or look.reference_p is None:
            return
        item = world.find(look.object)
        if item is None:
            return
        p_old = np.asarray(item.pose_in_base(world.frames)[0], dtype=float)
        p_new = np.asarray(look.reference_p, dtype=float)
        if np.allclose(p_old, p_new, atol=1e-6):
            return
        r = item.pose_in_base(world.frames)[1]
        robot.declare([dataclasses.replace(
            item, p=p_new, r=r, frame_id="base", provenance="judged",
            confidence=float(confidence), stamp=float(world.stamp))])

    def _refine(self, robot, world, look: ServoLook, name: str,
                report: ServoReport) -> str:
        """After ``on``: the same photo, drawn with the jaw opening shifted
        on a 3 x 3 grid of ``refine_m`` along the jaw axis and across it. The
        declaration moves to the jaw point of the best-rated shift only when
        it beats the unshifted opening; the hand never moves."""
        item = world.find(name)
        if item is None or look.photo is None and self.reads_photo:
            return "; no refinement (no photo)"
        if look.jaws is None:
            return "; no refinement (no jaw opening)"
        s = self.refine_m
        offsets = [(0.0, 0.0)] + [(a, b) for a in (0.0, s, -s)
                                  for b in (0.0, s, -s) if (a, b) != (0.0, 0.0)]
        best, unshifted = None, 0.0
        gap = (look.jaws.gap_m, look.jaws.gap_source)
        for a, b in offsets:
            seen = self.look_at(look.camera, item, world.frames, look.photo,
                                side=look.side, tolerance_m=s, tag="_refine",
                                gap=gap, shift_m=(a, b), outline=look.outline)
            if seen is None or seen.reference_p is None:
                continue
            dist, choice, confidence = self.ask(seen)
            shift = np.asarray(seen.reference_p) - np.asarray(look.reference_p)
            report.steps.append(ServoStep(
                look=seen.to_json(), distribution=dist, choice=choice,
                confidence=confidence, kind="refine",
                shift_m=(float(shift[0]), float(shift[1]))))
            if a == b == 0.0:
                unshifted = dist["on"]
            if best is None or dist["on"] > best[0]:
                best = (dist["on"], seen.reference_p, shift, (a, b))
        if best is None or best[0] < self.margin:
            return f"; refinement found no opening within {s * 1000:.0f} mm"
        if best[3] == (0.0, 0.0) or best[0] - unshifted < self.margin:
            return (f"; refined: the unshifted opening is best (on "
                    f"{unshifted:.2f}, best shift {best[0]:.2f})")
        robot.declare([dataclasses.replace(
            item, p=np.asarray(best[1], dtype=float), frame_id="base",
            r=item.pose_in_base(world.frames)[1], provenance="judged",
            confidence=float(best[0]), stamp=float(world.stamp))])
        return (f"; refined: declaration moved {best[2][0] * 1000:+.0f} / "
                f"{best[2][1] * 1000:+.0f} mm (on {best[0]:.2f} within "
                f"{s * 1000:.0f} mm), hand not moved")

    def _correct(self, robot, world, policy, state, side, name, nudge: Nudge,
                 settings, run_plan, step: ServoStep, look: ServoLook
                 ) -> Optional[Tuple[str, str]]:
        """Move the hand by ``nudge`` and carry the declaration with the
        judgement: a table-plane step moves the object by the same step, a
        turn turns it about the approach axis (so the stroke, which plans its
        jaw roll from the object's footprint, closes the way the judge
        asked). Returns an (outcome, detail) that ENDS the servo, else None."""
        item = world.find(name)
        _nudge, unmet = policy.clamp(nudge, state, side=side)
        if unmet:
            return BUDGET, "; ".join(str(u) for u in unmet)
        plan = check(nudge, world, robot.kin)
        if not getattr(plan, "ok", False):
            return REFUSED, f"{label_for(nudge)} was refused: {plan}"
        arm0 = world.arm(side)
        report = run_plan(plan, robot.executor, kin=robot.kin, **settings)
        step.plan = plan.to_json()
        step.run = report.to_json()
        state.nudged(side)
        state.moved(side)
        if not report.completed:
            return NOT_MOVED, f"{label_for(nudge)}: {report.stop_reason} — {report.error}"
        after = robot.world()
        arm1 = after.arm(side)
        if arm0 is not None and arm1 is not None and arm0.tool_p is not None \
                and arm1.tool_p is not None:
            moved = np.asarray(arm1.tool_p) - np.asarray(arm0.tool_p)
        else:
            moved = np.zeros(3)
        if nudge.frame == "base":
            step.step_m = (float(nudge.dx), float(nudge.dy))
        elif nudge.dyaw:
            step.step_m = (0.0, 0.0)
        else:
            step.step_m = (float(moved[0]), float(moved[1]))
        p, r = item.pose_in_base(world.frames)
        p_new = np.asarray(p, dtype=float)
        r_new = r
        if nudge.frame == "base":
            p_new = p_new + np.array([nudge.dx, nudge.dy, 0.0])
        if nudge.dyaw:
            axis = (arm0.tool_r.as_matrix()[:, 2] if arm0 is not None and
                    arm0.tool_r is not None else np.array([0.0, 0.0, -1.0]))
            r_new = R.from_rotvec(axis * float(nudge.dyaw)) * r
        if nudge.frame == "base" or nudge.dyaw:
            # a JUDGEMENT, not a sighting: the classifier chose and the kit
            # stepped; nobody measured where the object is
            robot.declare([dataclasses.replace(
                item, p=p_new, r=r_new, frame_id="base", provenance="judged",
                confidence=float(step.confidence), stamp=float(world.stamp))])
            after = robot.world()
        step.verdict = nudge.verifier(world)(after).to_json()
        return None


#: the geometry judge's "on" is a distance in metres compared with the look's
#: tolerance; a 30 mm nudge from 40 mm off leaves 10.0000002 mm, which is
#: floating point, not a residual
GEOMETRY_SLACK_M = 1e-4


def photoless(judge: Judge) -> Judge:
    """Mark ``judge`` as answering WITHOUT the photo (from geometry, a
    recorded answer, a test stub): the servo asks it even when the frame
    source gives none. Every other judge reads the photo, and is not asked
    without one."""
    try:
        judge.reads_photo = False            # type: ignore[attr-defined]
    except AttributeError:                   # a builtin or a bound method
        inner = judge

        def judge(look):                     # noqa: F811
            return inner(look)
        judge.reads_photo = False            # type: ignore[attr-defined]
    return judge


def geometry_judge(truth: Mapping[str, Sequence[float]], *,
                   tol_m: Optional[float] = None,
                   yaw: Optional[Mapping[str, float]] = None,
                   depth: bool = False,
                   occluded: Optional[Callable[[ServoLook], bool]] = None
                   ) -> Judge:
    """A judge that answers from geometry instead of a photo: "on" when the
    TRUE object (``truth[name]``, base metres) is within the look's tolerance
    (``look.tolerance_m``, or ``tol_m``) of the jaw point across the approach
    axis (of the declared centre when no jaws were drawn); otherwise the side
    of the opening it projects to. The stand-in for tests and dry runs — it
    needs no photo and no model, and it exercises every line of the servo
    except the classifier.

    ``yaw`` (name -> the TRUE yaw about base z, rad) adds the turn answer: the
    image turn that puts the jaw travel across the true outline's long axis.
    ``depth=True`` adds the depth answer (ahead of the fingertips, between
    the pads, behind them); ``occluded(look)`` the occlusion answer."""
    def judge(look: ServoLook) -> Reading:
        p = truth.get(look.object)
        if p is None:
            return Reading({"not_visible": 1.0})
        p = np.asarray(p, dtype=float)
        seen = look.camera.project_point(p)
        if not seen.visible:
            return Reading({"not_visible": 1.0})
        du, dv = float(seen.u) - look.u, float(seen.v) - look.v
        inside = look.tolerance_m if tol_m is None else float(tol_m)
        reference = look.reference_p if look.reference_p is not None \
            else look.declared_p
        axis = (look.camera.flange_r.as_matrix()[:, 2]
                if getattr(look.camera, "flange_r", None) is not None
                and look.jaws is not None else np.array([0.0, 0.0, 1.0]))
        if reference is not None:
            off = np.asarray(p) - np.asarray(reference, dtype=float)
            off = off - float(np.dot(off, axis)) * axis
            if float(np.linalg.norm(off)) <= inside + GEOMETRY_SLACK_M:
                direction = {"on": 1.0}
            elif abs(du) >= abs(dv):
                direction = {"left" if du < 0 else "right": 1.0}
            else:
                direction = {"above" if dv < 0 else "below": 1.0}
        elif math.hypot(du, dv) <= look.radius_px:
            direction = {"on": 1.0}
        elif abs(du) >= abs(dv):
            direction = {"left" if du < 0 else "right": 1.0}
        else:
            direction = {"above" if dv < 0 else "below": 1.0}
        turn = None
        if yaw is not None and look.object in yaw and look.jaws is not None:
            long_axis = np.array([math.cos(float(yaw[look.object])),
                                  math.sin(float(yaw[look.object])), 0.0])
            a = look.camera.project_point(p - 0.02 * long_axis)
            b = look.camera.project_point(p + 0.02 * long_axis)
            if a.visible and b.visible:
                angle = math.degrees(math.atan2(b.v - a.v, b.u - a.u))
                turn = J.wrap_turn(angle + 90.0 - look.jaws.jaw_angle_deg)
        where = None
        if depth and getattr(look.camera, "flange_p", None) is not None:
            from ..hands.d1.parallel_gripper.description import (  # noqa: PLC0415
                PAD_ROOT_Z_M, PAD_TIP_Z_M)
            d = J.approach_depth(look.camera, p)
            where = ({"ahead": 1.0} if d > PAD_TIP_Z_M else
                     {"between": 1.0} if d >= PAD_ROOT_Z_M else {"behind": 1.0})
        hidden = None if occluded is None else (1.0 if occluded(look) else 0.0)
        return Reading(direction=direction, depth=where, occluded=hidden,
                       turn_deg=turn)
    return photoless(judge)


__all__ = ["ACTIONS", "ALIGNED", "BUDGET", "CHOICES", "Choice", "DEPTHS",
           "DEFAULT_BUDGET_S", "DEFAULT_MARGIN", "DEFAULT_MAX_ITERATIONS",
           "DEFAULT_OCCLUSION_MARGIN",
           "DEFAULT_REFINE_M", "DEFAULT_SETTLE_S", "DEFAULT_STEPS_M",
           "DEFAULT_TOLERANCE_M", "DEFAULT_WINDOW", "STALE", "TIMEOUT",
           "Reading", "accumulate", "as_reading", "choose", "decide",
           "exit_line", "iteration_line",
           "Frame", "Judge", "NOT_VISIBLE", "NO_FRAME", "OBSERVED",
           "SERVO_VERBS", "Servo", "ServoLook", "ServoReport", "ServoStep",
           "UNSURE", "footprint_px", "geometry_judge",
           "image_direction_in_base", "mark", "photoless", "servo_line",
           "turn_sign"]
