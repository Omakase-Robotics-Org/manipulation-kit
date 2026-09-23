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
re-declares the object that step away (``provenance="observed"``, exactly as
the ``locate`` correction does), moves the hand by the same step with its own
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

Two seams, and the wheel owns everything between them::

    frame(side) -> Path | None      a FRESH photo from that hand's wrist camera
                                    (None: no camera here — the judge must
                                    answer from geometry, as a test stub does)
    judge(look) -> {choice: prob}   a probability per :data:`CHOICES` id

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

#: how the servo ended
ALIGNED = "aligned"
NOT_VISIBLE = "not_visible"
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

    def to_json(self) -> Dict[str, Any]:
        return {"side": self.side, "object": self.object, "u": self.u,
                "v": self.v, "depth_m": self.depth_m,
                "radius_px": self.radius_px,
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

    def to_json(self) -> Dict[str, Any]:
        return {"side": self.side, "object": self.object,
                "outcome": self.outcome, "detail": self.detail,
                "steps": [s.to_json() for s in self.steps]}

    def to_text(self) -> str:
        n = sum(1 for s in self.steps if s.step_m is not None)
        if self.aligned:
            return (f"the {self.side} hand was aligned on {self.object!r} by "
                    f"the wrist look ({n} correction{'s' if n != 1 else ''})")
        return (f"the wrist look could not align the {self.side} hand on "
                f"{self.object!r}: {self.outcome} — {self.detail} "
                f"({n} correction{'s' if n != 1 else ''} made)")


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
                 out_dir: Optional[Path] = None):
        if not steps_m or any(not 0.0 < float(s) <= max(NUDGE_GRID_M)
                              for s in steps_m):
            raise ValueError(f"steps_m must be nudge-grid steps, got "
                             f"{list(steps_m)!r}")
        if not 0.0 <= float(min_confidence) <= 1.0:
            raise ValueError("min_confidence is a probability")
        if not 0.0 < float(tolerance_m) <= max(NUDGE_GRID_M):
            raise ValueError("tolerance_m is a small positive distance")
        self.frame, self.judge = frame, judge
        self.steps_m = tuple(float(s) for s in steps_m)
        self.min_confidence = float(min_confidence)
        self.tolerance_m = float(tolerance_m)
        self.out_dir = Path(out_dir) if out_dir is not None else None
        self._n = 0

    # -- one look ------------------------------------------------------------ #
    def look(self, robot, world, side: str, name: str) -> Tuple[Optional[ServoLook], str]:
        camera = robot.cameras(world).get(f"{side}_wrist")
        item = world.find(name)
        if camera is None or item is None:
            return None, f"no {side}_wrist camera model, or {name!r} is not in the scene"
        seen = camera.project_object(item, world.frames)
        if not seen.visible:
            return None, (f"{name!r} is not in the {side}_wrist frame from "
                          f"this posture ({seen.reason})")
        # the tolerance at this depth, in pixels (a pinhole estimate; the
        # circle is a drawing, not a measurement)
        radius = float(camera.fx) * self.tolerance_m / max(float(seen.depth_m), 0.02)
        box = footprint_px(camera, item, world.frames, margin_m=self.tolerance_m)
        photo = self.frame(side)
        image = None
        if photo is not None:
            self._n += 1
            out_dir = self.out_dir or Path(photo).parent
            image = mark(Path(photo), seen.u, seen.v,
                         out_dir / f"servo{self._n:03d}_{side}_wrist_marked.jpg",
                         box_px=box, radius_px=radius)
        return ServoLook(side=side, object=name, camera=camera, u=float(seen.u),
                         v=float(seen.v), depth_m=float(seen.depth_m),
                         image=image, photo=None if photo is None else Path(photo),
                         radius_px=radius, box_px=box), ""

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
              run_plan: Callable[..., Any]) -> ServoReport:
        report = ServoReport(side=side, object=name, outcome=UNSURE)
        signs: Dict[str, float] = {}       # axis -> sign of the last step
        fine = False
        while True:
            world = robot.world()
            look, why = self.look(robot, world, side, name)
            if look is None:
                report.outcome, report.detail = NOT_VISIBLE, why
                return report
            dist, choice, confidence = self.ask(look)
            step = ServoStep(look=look.to_json(), distribution=dist,
                             choice=choice, confidence=confidence)
            report.steps.append(step)
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
        # the correction is a sighting of the object, as a wrist locate is
        seen = dataclasses.replace(item, p=p_new, frame_id="base",
                                   provenance="observed",
                                   stamp=float(world.stamp))
        robot.declare([seen])
        after = robot.world()
        step.verdict = nudge.verifier(world)(after).to_json()
        return None


def geometry_judge(truth: Mapping[str, Sequence[float]], *,
                   tol_px: Optional[float] = None) -> Judge:
    """A judge that answers from geometry instead of a photo: where the TRUE
    object (``truth[name]``, base metres) projects, relative to the mark —
    "on" inside the drawn circle (``look.radius_px``, or ``tol_px``). The
    stand-in for tests and dry runs — it needs no photo and no model, and it
    exercises every line of the servo except the classifier."""
    def judge(look: ServoLook) -> Dict[str, float]:
        p = truth.get(look.object)
        if p is None:
            return {"not_visible": 1.0}
        seen = look.camera.project_point(np.asarray(p, dtype=float))
        if not seen.visible:
            return {"not_visible": 1.0}
        du, dv = float(seen.u) - look.u, float(seen.v) - look.v
        inside = look.radius_px if tol_px is None else tol_px
        if math.hypot(du, dv) <= inside:
            return {"on": 1.0}
        if abs(du) >= abs(dv):
            return {"left" if du < 0 else "right": 1.0}
        return {"above" if dv < 0 else "below": 1.0}
    return judge


__all__ = ["ALIGNED", "BUDGET", "CHOICES", "DEFAULT_MIN_CONFIDENCE",
           "DEFAULT_STEPS_M", "DEFAULT_TOLERANCE_M", "Frame", "Judge",
           "NOT_VISIBLE", "Servo",
           "ServoLook", "ServoReport", "ServoStep", "UNSURE",
           "geometry_judge", "image_direction_in_base", "mark"]
