"""The System 1 step: align the hand on the object before a stroke, by
JUDGEMENT rather than by measurement.

The look before a stroke (:mod:`.loop`, ``look_before_stroke``) projects the
declared object into the wrist camera and asks the model whether the photo
agrees, and if not, WHERE the object is — a pixel it must write. A decision
classifier cannot write a pixel; it can only choose. This module asks it the
one thing it can answer well: the kit DRAWS the projected pixel on the fresh
wrist photo, and the judge says whether the object is ON that mark or LEFT /
RIGHT / ABOVE / BELOW it in the image. The kit does the rest — turns the image
direction into a base-frame step through the wrist camera's known orientation,
re-declares the object that step away (``provenance="judged"``: a classifier
chose it, nobody measured it — see :data:`~manipulation_kit.world.views.STATED`),
moves the hand by the same step with its own
:class:`~manipulation_kit.primitives.Nudge`, photographs again — until the
judge says ON or the nudge budget of the operator policy is spent.

Why a drawn mark and a relative question (measured on this branch, report
``jev-servo-loop``): on rendered wrist frames, the same 12B classifier answered
"which way must the hand move" with the same option on every image (a prior,
not a perception) and put a cup that was outside the jaws "between" them at
0.86; with the kit's projected pixel drawn on the frame, it placed the cup
relative to the mark correctly on 8 of 8 frames, the displaced six at 0.93 or
better. The yes/no form of the same question missed the two ON cases, so the
question is five-way, never yes/no.

WHEN IT RUNS: before EVERY stroke of :data:`SERVO_VERBS` (``grasp`` on its
``object``, ``press`` on its ``target``), whether or not the model looked or
``locate``-d from the wrist first. d1-2, 2026-09-24 01:47Z (``--judge-only``):
the model called ``locate`` on the wrist camera before each grasp, that
satisfied the operator policy's look, and a servo that ran only on an unmet
look was never asked — twelve records, ``servo: null`` on every one, and the
tape several cm off the jaws when the stroke closed. The servo is the System 1
alignment before the stroke; the model's own look does not replace it.
``probe`` has no named target to align on (it measures where the surface
is), ``handover`` closes at a posture reached inside its own plan, and
``approach`` is the move that brings the hand to the look — none is judged.

Two seams, and the wheel owns everything between them::

    frame(side) -> Path | None      a FRESH photo from that hand's wrist camera
                                    (None: no camera here — the judge must
                                    answer from geometry, as a test stub does)
    judge(look) -> {choice: prob}   a probability per :data:`CHOICES` id

A judge READS THE PHOTO unless it says otherwise (:func:`photoless`, which
:func:`geometry_judge` is): when ``frame(side)`` gives no photo, a photo
judge is not asked at all and the report says ``skipped: "no wrist frame"``
— recorded, never a silent ``null``.

Two optional behaviours, both off by default until real wrist photos have
characterised the judge:

``observe_only=True``  the servo photographs, marks and asks, records the
                       distribution in ``DecisionRecord.servo``, and never
                       steps or re-declares; the loop then goes on with the
                       model's own look rule (the ordinary wrist-look
                       question when the policy's look is unmet, else the
                       stroke). The judge rides along on a live run without
                       moving anything.
``refine=True``        after ``on``, bring the residual (up to the tolerance,
                       10-21 mm in the mirror runs) under ``refine_m``
                       (5 mm) WITHOUT moving the hand: the same photo is
                       re-marked with the declaration shifted by ``refine_m``
                       along each mappable image axis (a 3 x 3 grid, the
                       unshifted mark first) and a box grown by ``refine_m``
                       only; the declaration moves to the shift the judge
                       rates ``on`` highest, and only when that beats the
                       unshifted mark. The grasp plans to the declaration, so
                       this is the correction; the hand does not move and a
                       worse answer is never taken. With a perfect judge the
                       grid (spacing ``refine_m``, radius ``refine_m``) covers
                       every residual inside the tolerance, so the result is
                       within ``refine_m``.

Roll is not asked for: it is planner-only on this branch (``roll_rad``,
``roll_candidates``), and the same classifier could not tell a rotation's
sense on a synthetic bar (clockwise for both tilts).
"""

from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import (Any, Callable, Dict, List, Mapping, Optional, Sequence,
                    Tuple)

import numpy as np

from ..primitives import Nudge
from ..primitives.offer import check, label_for
from ..primitives.types import NUDGE_GRID_M
from .policy import OperatorPolicy, PolicyState

#: the verbs the servo judges before, -> the argument naming what the stroke
#: acts on. Explicit, not derived: a stroke is judged only where the hand
#: already stands at the posture it acts from and a named thing is there to
#: be marked (see the module docstring for the verbs left out, and why).
SERVO_VERBS: Dict[str, str] = {"grasp": "object", "press": "target"}

#: what the judge chooses among. The object, relative to the drawn mark, IN
#: THE IMAGE: ``on`` is the answer that lets the stroke run.
CHOICES: Tuple[str, ...] = ("on", "left", "right", "above", "below",
                            "not_visible")

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

#: below this the judge is not believed: no blind nudge on a shrug
DEFAULT_MIN_CONFIDENCE = 0.5

#: how close is close enough: the radius of the circle drawn around the mark,
#: one fine nudge step — inside it no correction the grid can make is smaller
#: than the error
DEFAULT_TOLERANCE_M = NUDGE_GRID_M[0]

#: the refinement step: the declaration may move this far (per image axis)
#: without the hand moving, and "on" then means inside a box grown by it
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
UNMAPPABLE = "unmappable_direction"
REFUSED = "nudge_refused"
NOT_MOVED = "nudge_not_completed"


@dataclass(frozen=True)
class ServoLook:
    """What the judge is shown: the marked photo, and the geometry behind it
    so a stub can answer without a photo."""

    side: str
    object: str
    camera: Any                      # the WristCamera model at this posture
    u: float                         # where the declared centre projects
    v: float
    depth_m: float
    image: Optional[Path]            # the photo WITH the mark drawn, or None
    photo: Optional[Path]            # the photo as taken, or None
    #: the circle drawn around the mark: ``tolerance_m`` at this depth, in
    #: pixels — "on" means the object's centre is inside it
    radius_px: float = 0.0
    #: the declared object's projected outline (u0, v0, u1, v1), drawn as a
    #: box: "on" means the object fills it rather than sticking out
    box_px: Optional[Tuple[float, float, float, float]] = None
    choices: Tuple[str, ...] = CHOICES
    #: the declared centre (base, m) the mark was drawn for, and the margin
    #: the box was grown by — what "on" means for this look
    declared_p: Optional[Tuple[float, float, float]] = None
    tolerance_m: float = DEFAULT_TOLERANCE_M

    def to_json(self) -> Dict[str, Any]:
        return {"side": self.side, "object": self.object, "u": self.u,
                "v": self.v, "depth_m": self.depth_m,
                #: which wrist mount the mark was projected through
                "mount": getattr(self.camera, "mount", None),
                "radius_px": self.radius_px, "tolerance_m": self.tolerance_m,
                "declared_p": (None if self.declared_p is None
                               else [round(float(x), 5) for x in self.declared_p]),
                "box_px": None if self.box_px is None else list(self.box_px),
                "image": None if self.image is None else str(self.image),
                "photo": None if self.photo is None else str(self.photo)}


Frame = Callable[[str], Optional[Path]]
Judge = Callable[[ServoLook], Mapping[str, float]]


@dataclass
class ServoStep:
    look: Dict[str, Any]
    distribution: Dict[str, float]
    choice: str
    confidence: float
    step_m: Optional[Tuple[float, float]] = None   # base dx, dy applied
    #: "look" (a judgement that may step the hand) or "refine" (the same
    #: photo re-marked with the declaration shifted by ``shift_m``)
    kind: str = "look"
    shift_m: Optional[Tuple[float, float]] = None
    plan: Optional[Dict[str, Any]] = None
    run: Optional[Dict[str, Any]] = None
    verdict: Optional[Dict[str, Any]] = None

    def to_json(self) -> Dict[str, Any]:
        return dataclasses.asdict(self)


@dataclass
class ServoReport:
    side: str
    object: str
    outcome: str
    detail: str = ""
    steps: List[ServoStep] = field(default_factory=list)

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
        dist = looks[-1]["distribution"]
        ranked = sorted(dist.items(), key=lambda kv: -kv[1])
        said = (f"{ranked[0][0]} {ranked[0][1]:.2f} ("
                + ", ".join(f"{c} {p:.2f}" for c, p in ranked[1:]) + ")")
    moved = sum(1 for s in servo.get("steps", []) if s.get("step_m"))
    outcome = servo.get("outcome")
    if outcome == OBSERVED:
        return f"servo judged: {said} — recorded only"
    if outcome == ALIGNED:
        return f"servo aligned: {said} after {moved} step(s)"
    return (f"servo {outcome}: {servo.get('detail', '')}"
            + (f"; last judged {said}" if said else "")
            + f" ({moved} step(s))")


def footprint_px(camera: Any, obj: Any, frames: Any, *,
                 margin_m: float = 0.0) -> Optional[Tuple[float, float, float, float]]:
    """The image-space bounding box (u0, v0, u1, v1) of the declared object's
    OUTLINE grown by ``margin_m`` on every side — its size box at its pose,
    every corner projected. The margin is the alignment tolerance: an object
    displaced by less than it still sits inside the box. None when a corner
    is behind the lens."""
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
    """Draw the kit's belief on the photo: a green cross at the projected
    centre and, when the object's outline is known, a green box — the outline
    grown by the alignment tolerance. The judge is asked about the object
    RELATIVE to the box: inside it, or sticking out to one side.

    Why a box and not a cross (kinematic-mirror runs with Jev-Omni, report
    ``jev-servo-loop``): with the cross alone a block 10 mm off — inside any
    grasp tolerance — was judged "on" at 0.24 and a block 30 mm off got no
    answer above 0.36; the cross was hidden under the object and the judge
    reads overlap, not centres. Against the outline box every direction came
    back at 0.93-1.00 down to a 10 mm residual. Without an outline the
    tolerance is drawn as a circle instead."""
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
        # the object's expected OUTLINE grown by the tolerance: a judge that
        # sees overlap rather than centres is asked whether the object is
        # inside the box or sticks out of it — a 20 mm offset on a 40 mm
        # object with a 10 mm margin sticks out plainly, a 5 mm one does not
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
    when the judge says it appears ``choice`` of the mark. ``None`` when that
    image axis is nearly vertical in base, so a horizontal step cannot follow
    it (a camera looking along the table sees "above" as "farther up")."""
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


class Servo:
    """Align one hand on one object, by repeated judgement. Built once per
    run; :meth:`align` is called by the loop in place of the wrist-look text
    whenever a stroke needs a look."""

    def __init__(self, frame: Frame, judge: Judge, *,
                 steps_m: Sequence[float] = DEFAULT_STEPS_M,
                 min_confidence: float = DEFAULT_MIN_CONFIDENCE,
                 tolerance_m: float = DEFAULT_TOLERANCE_M,
                 out_dir: Optional[Path] = None,
                 observe_only: bool = False, refine: bool = False,
                 refine_m: float = DEFAULT_REFINE_M):
        if not steps_m or any(not 0.0 < float(s) <= max(NUDGE_GRID_M)
                              for s in steps_m):
            raise ValueError(f"steps_m must be nudge-grid steps, got "
                             f"{list(steps_m)!r}")
        if not 0.0 <= float(min_confidence) <= 1.0:
            raise ValueError("min_confidence is a probability")
        if not 0.0 < float(tolerance_m) <= max(NUDGE_GRID_M):
            raise ValueError("tolerance_m is a small positive distance")
        if not 0.0 < float(refine_m) <= float(tolerance_m):
            raise ValueError("refine_m is a positive distance no larger than "
                             "tolerance_m")
        self.frame, self.judge = frame, judge
        #: whether the judge needs the photo (:func:`photoless` says no)
        self.reads_photo = bool(getattr(judge, "reads_photo", True))
        self.steps_m = tuple(float(s) for s in steps_m)
        self.min_confidence = float(min_confidence)
        self.tolerance_m = float(tolerance_m)
        self.out_dir = Path(out_dir) if out_dir is not None else None
        self.observe_only, self.refine = bool(observe_only), bool(refine)
        self.refine_m = float(refine_m)
        self._n = 0
        #: the turn and directory of the current :meth:`align` (the marked
        #: photos are named ``turn{N}_servo_{side}[ _k].png`` beside the trace)
        self._turn: Optional[int] = None
        self._where: Optional[Path] = None
        self._in_turn = 0

    # -- one look ------------------------------------------------------------ #
    def look(self, robot, world, side: str, name: str) -> Tuple[Optional[ServoLook], str]:
        """Project, photograph (``frame(side)``) and mark: the look the judge
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
        return self.look_at(camera, item, world.frames, photo, side=side), ""

    def look_at(self, camera, item, frames, photo: Optional[Path], *,
                side: str, tolerance_m: Optional[float] = None,
                tag: str = "") -> Optional[ServoLook]:
        """Mark ``photo`` (may be None) with where ``item`` projects in
        ``camera`` and the box grown by ``tolerance_m``: one look, no motion.
        Public so a recorded photo can be judged again offline
        (``examples/agent/jev_servo.py --rejudge``). None when the item does
        not project into the image."""
        tolerance = self.tolerance_m if tolerance_m is None else float(tolerance_m)
        seen = camera.project_object(item, frames)
        if not seen.visible:
            return None
        # the tolerance at this depth, in pixels (a pinhole estimate; the
        # circle is a drawing, not a measurement)
        radius = float(camera.fx) * tolerance / max(float(seen.depth_m), 0.02)
        box = footprint_px(camera, item, frames, margin_m=tolerance)
        image = None
        if photo is not None:
            image = mark(Path(photo), seen.u, seen.v,
                         self._marked_path(Path(photo), side, tag),
                         box_px=box, radius_px=radius)
        p = np.asarray(item.pose_in_base(frames)[0], dtype=float)
        return ServoLook(side=side, object=item.name, camera=camera,
                         u=float(seen.u), v=float(seen.v),
                         depth_m=float(seen.depth_m), image=image,
                         photo=None if photo is None else Path(photo),
                         radius_px=radius, box_px=box,
                         declared_p=tuple(float(x) for x in p),
                         tolerance_m=tolerance)

    def _marked_path(self, photo: Path, side: str, tag: str) -> Path:
        """``turn{N}_servo_{side}.png`` for a turn's first marked photo (the
        next ``_2``, ``_3``, ...), beside the trace; outside a turn
        ``servo{n:03d}_{side}{tag}.png`` next to ``out_dir`` or the photo."""
        self._n += 1
        out_dir = self.out_dir or self._where or photo.parent
        if self._turn is None:
            return out_dir / f"servo{self._n:03d}_{side}{tag}.png"
        self._in_turn += 1
        suffix = "" if self._in_turn == 1 else f"_{self._in_turn}"
        return out_dir / f"turn{self._turn}_servo_{side}{tag}{suffix}.png"

    def ask(self, look: ServoLook) -> Tuple[Dict[str, float], str, float]:
        raw = dict(self.judge(look))
        unknown = set(raw) - set(look.choices)
        if unknown:
            raise ValueError(f"the judge answered with choices the kit did "
                             f"not offer: {sorted(unknown)}")
        dist = {c: float(raw.get(c, 0.0)) for c in look.choices}
        total = sum(dist.values())
        if total > 0:
            dist = {c: p / total for c, p in dist.items()}
        choice = max(dist, key=dist.get)
        return dist, choice, dist[choice]

    # -- the loop ------------------------------------------------------------ #
    def align(self, *, robot, policy: OperatorPolicy, state: PolicyState,
              side: str, name: str, settings: Mapping[str, Any],
              run_plan: Callable[..., Any], turn: Optional[int] = None,
              out_dir: Optional[Path] = None) -> ServoReport:
        """Judge (and, unless ``observe_only``, align) ``side`` on ``name``.
        ``turn`` / ``out_dir`` name the marked photos (``out_dir`` is used
        when the servo was built without one — the loop passes the trace's
        directory)."""
        self._turn, self._in_turn = turn, 0
        self._where = None if out_dir is None else Path(out_dir)
        try:
            return self._align(robot=robot, policy=policy, state=state,
                               side=side, name=name, settings=settings,
                               run_plan=run_plan)
        finally:
            self._turn, self._where = None, None

    def _align(self, *, robot, policy, state, side, name, settings,
               run_plan) -> ServoReport:
        report = ServoReport(side=side, object=name, outcome=UNSURE)
        signs: Dict[str, float] = {}       # axis -> sign of the last step
        fine = False
        while True:
            world = robot.world()
            look, why = self.look(robot, world, side, name)
            if look is None and why == NO_FRAME:
                report.outcome = NO_FRAME
                report.detail = (f"no {side}_wrist photo from the frame source "
                                 f"(no snapshotter, or the grab gave none), and "
                                 f"this judge reads the photo")
                return report
            if look is None:
                report.outcome, report.detail = NOT_VISIBLE, why
                return report
            dist, choice, confidence = self.ask(look)
            step = ServoStep(look=look.to_json(), distribution=dist,
                             choice=choice, confidence=confidence)
            report.steps.append(step)
            if self.observe_only:
                report.outcome = OBSERVED
                report.detail = f"{choice!r} at {confidence:.2f}"
                return report
            if confidence < self.min_confidence:
                report.outcome = UNSURE
                report.detail = (f"the judge's best answer {choice!r} at "
                                 f"{confidence:.2f} is below "
                                 f"{self.min_confidence:.2f}; no blind step")
                return report
            if choice == "on":
                state.aimed(side, name)
                state.look(side, name, world.arm(side).joints)
                report.outcome = ALIGNED
                report.detail = f"on the mark at {confidence:.2f}"
                if self.refine:
                    report.detail += self._refine(robot, world, look, name, report)
                return report
            if choice == "not_visible":
                report.outcome = NOT_VISIBLE
                report.detail = (f"the judge does not see {name!r} in the "
                                 f"{side}_wrist photo ({confidence:.2f})")
                return report
            direction = image_direction_in_base(look.camera, choice)
            if direction is None:
                report.outcome = UNMAPPABLE
                report.detail = (f"image {choice!r} is nearly vertical in "
                                 f"base from this posture; no table-plane "
                                 f"step follows it")
                return report
            axis = "u" if choice in ("left", "right") else "v"
            sign = -1.0 if choice in ("left", "above") else 1.0
            if axis in signs and signs[axis] != sign:
                fine = True             # overshot: switch to the fine step
            signs[axis] = sign
            size = self.steps_m[-1] if fine else self.steps_m[0]
            delta = direction * size
            done = self._correct(robot, world, policy, state, side, name,
                                 delta, settings, run_plan, step)
            if done is not None:
                report.outcome, report.detail = done
                return report

    def _refine(self, robot, world, look: ServoLook, name: str,
                report: ServoReport) -> str:
        """After ``on``: the same photo, re-marked with the declaration
        shifted on a 3 x 3 grid of ``refine_m`` along the image axes, a box
        grown by ``refine_m``. The declaration moves to the best-rated shift
        only when it beats the unshifted mark; the hand never moves."""
        item = world.find(name)
        axes = [d for d in (image_direction_in_base(look.camera, "right"),
                            image_direction_in_base(look.camera, "below"))
                if d is not None]
        if item is None or not axes:
            return "; no refinement (no mappable image axis)"
        s = self.refine_m
        offsets = [(0.0, 0.0)] + [(a, b) for a in (0.0, s, -s)
                                  for b in (0.0, s, -s) if (a, b) != (0.0, 0.0)]
        best = None
        for a, b in offsets:
            if b and len(axes) < 2:
                continue
            shift = a * axes[0] + (b * axes[1] if len(axes) > 1 else 0.0)
            moved = dataclasses.replace(
                item, p=np.asarray(item.pose_in_base(world.frames)[0], dtype=float)
                + np.array([shift[0], shift[1], 0.0]), frame_id="base")
            seen = self.look_at(look.camera, moved, world.frames, look.photo,
                                side=look.side, tolerance_m=s, tag="_refine")
            if seen is None:
                continue
            dist, choice, confidence = self.ask(seen)
            report.steps.append(ServoStep(
                look=seen.to_json(), distribution=dist, choice=choice,
                confidence=confidence, kind="refine",
                shift_m=(float(shift[0]), float(shift[1]))))
            if best is None or dist["on"] > best[0]:
                best = (dist["on"], moved, shift)
        if best is None or best[0] < self.min_confidence:
            return f"; refinement found no mark within {s * 1000:.0f} mm"
        if not np.any(best[2]):
            return f"; refined: the unshifted mark is best (on {best[0]:.2f})"
        robot.declare([dataclasses.replace(
            best[1], provenance="judged", confidence=float(best[0]),
            stamp=float(world.stamp))])
        return (f"; refined: declaration moved {best[2][0] * 1000:+.0f} / "
                f"{best[2][1] * 1000:+.0f} mm (on {best[0]:.2f} within "
                f"{s * 1000:.0f} mm), hand not moved")

    def _correct(self, robot, world, policy, state, side, name, delta,
                 settings, run_plan, step: ServoStep) -> Optional[Tuple[str, str]]:
        """Re-declare the object ``delta`` away and move the hand by the same
        step. Returns an (outcome, detail) that ENDS the servo, else None."""
        item = world.find(name)
        p_old = np.asarray(item.pose_in_base(world.frames)[0], dtype=float)
        p_new = p_old + np.array([delta[0], delta[1], 0.0])
        nudge = Nudge(side=side, dx=float(delta[0]), dy=float(delta[1]),
                      frame="base")
        _nudge, unmet = policy.clamp(nudge, state, side=side)
        if unmet:
            return BUDGET, "; ".join(str(u) for u in unmet)
        plan = check(nudge, world, robot.kin)
        if not getattr(plan, "ok", False):
            return REFUSED, f"{label_for(nudge)} was refused: {plan}"
        step.step_m = (float(delta[0]), float(delta[1]))
        step.plan = plan.to_json()
        report = run_plan(plan, robot.executor, kin=robot.kin, **settings)
        step.run = report.to_json()
        state.nudged(side)
        state.moved(side)
        if not report.completed:
            return NOT_MOVED, f"{label_for(nudge)}: {report.stop_reason} — {report.error}"
        # the correction is a JUDGEMENT, not a sighting: the classifier chose
        # a side and the kit stepped; nobody measured where the object is
        seen = dataclasses.replace(item, p=p_new, frame_id="base",
                                   provenance="judged",
                                   confidence=float(step.confidence),
                                   stamp=float(world.stamp))
        robot.declare([seen])
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
                   tol_m: Optional[float] = None) -> Judge:
    """A judge that answers from geometry instead of a photo: "on" when the
    TRUE object (``truth[name]``, base metres) is within the look's tolerance
    (``look.tolerance_m``, or ``tol_m``) of the declared centre in the table
    plane; otherwise the side of the mark it projects to. The stand-in for
    tests and dry runs — it needs no photo and no model, and it exercises
    every line of the servo except the classifier."""
    def judge(look: ServoLook) -> Dict[str, float]:
        p = truth.get(look.object)
        if p is None:
            return {"not_visible": 1.0}
        p = np.asarray(p, dtype=float)
        seen = look.camera.project_point(p)
        if not seen.visible:
            return {"not_visible": 1.0}
        du, dv = float(seen.u) - look.u, float(seen.v) - look.v
        inside = look.tolerance_m if tol_m is None else float(tol_m)
        if look.declared_p is not None:
            off = math.hypot(p[0] - look.declared_p[0], p[1] - look.declared_p[1])
            if off <= inside + GEOMETRY_SLACK_M:
                return {"on": 1.0}
        elif math.hypot(du, dv) <= look.radius_px:
            return {"on": 1.0}
        if abs(du) >= abs(dv):
            return {"left" if du < 0 else "right": 1.0}
        return {"above" if dv < 0 else "below": 1.0}
    return photoless(judge)


__all__ = ["ALIGNED", "BUDGET", "CHOICES", "DEFAULT_MIN_CONFIDENCE",
           "DEFAULT_REFINE_M", "DEFAULT_STEPS_M", "DEFAULT_TOLERANCE_M",
           "Frame", "Judge", "NOT_VISIBLE", "NO_FRAME", "OBSERVED",
           "SERVO_VERBS", "Servo", "ServoLook", "ServoReport", "ServoStep",
           "UNSURE", "geometry_judge", "image_direction_in_base", "mark",
           "photoless", "servo_line"]
