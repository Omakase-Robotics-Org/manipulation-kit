"""WHERE and HOW the hand meets an object — the one module that decides it.

:mod:`.orientation` turns a direction into a wrist. This module decides
everything else about a grasp, on a resolved
:class:`~manipulation_kit.world.Direction`:

* **which part of the hand makes contact** — :class:`GraspReference`,
  :data:`PAD` (the pad centre, the default) or :data:`TIP` (the fingertips),
  built from the hand description's measured numbers rather than module
  constants. The model picks one with ``Grasp(contact="pad"|"tip")``.
* **the grasp point and its descent floor** — :func:`grasp_pose`. The floor
  is the MEASURED SURFACE the object stands on when one is known
  (:func:`support_of`) and the object's own declared underside only when none
  is, and the plan's notes say which (L5: a declaration 20 mm below a known
  table used to put the finger tips 20 mm into it).
* **the standoff** — :func:`standoff_point`. ``standoff_m`` is the gap between
  the LEADING tool geometry (the finger tips) and the object's silhouette
  along the direction, not a distance from the grasp point.
* **the one roll sweep** — :func:`roll_candidates`. Square to the footprint
  first, then the quarter turns whose presented width still fits. The verbs
  try them in that order; nothing else in the package generates a roll.
* **the fit test** — :func:`fits`: the width across the jaws against the
  reference's clearance, and the flat test from the reference's lead, so a
  6 mm card is graspable at the tips and refused at the pads.

Every waypoint the planner sees is still the pose of the pad centre
(:data:`.orientation.TOOL_Z_M`): a fingertip grasp is converted onto it here,
by :func:`tool_point`, and nowhere else.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from ..hands.d1.parallel_gripper.description import (
    DRIVEN_OPEN_GAP_M, HAND_CLOSEDNESS, HAND_POSES, PAD_CENTRE_Z_M,
    PAD_CLEARANCE_PER_SIDE_M, PAD_TIP_Z_M, TIP_CLEARANCE_PER_SIDE_M)
from ..world import (UPRIGHT_TOL_RAD, BASE, FrameError, FrameGraph,
                     ObjectView, SurfaceView)
from ..world.direction import Direction, object_frame
from . import orientation as _o
from .types import OBJECT_TOO_FLAT, OBJECT_TOO_WIDE, SurfaceBackoff, Unmet

__all__ = [
    "GraspReference", "PAD", "TIP", "REFERENCES", "CONTACTS", "GraspSpec",
    "DEFAULT_STANDOFF_M", "SUPPORT_CATCH_M", "HAND_POSES", "hand_closedness",
    "support_of", "descent_floor", "grasp_pose", "tool_point",
    "standoff_point", "roll_candidates", "graspable_width_m", "fits",
    "fit_problems",
    "achieved_clearance", "tilted", "own_face_direction",
    "TIP_CONTACT_THIN_M", "TIP_SEARCH_START_M", "CONTACT_OVERTRAVEL_M",
    "TIP_CONTACT_NM", "descends_by_contact",
    "CONTACT_BACKOFF_M", "CONTACT_BACKOFF_MIN_M", "SURFACE_CONTACT_BAND_M",
    "surface_backoff",
]


# --------------------------------------------------------------------------- #
# where on the hand
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class GraspReference:
    """WHERE on the hand the contact is — geometry, not a module constant.

    ``offset_z_m``  the contact point along TCP +z from the flange [m];
    ``lead_m``      how far the leading geometry (the finger tips) reaches past
                    it along the approach [m] — what the descent floor and the
                    flat test are computed from;
    ``clearance_per_side_m``  what a grasped object must leave free on each
                    side inside the jaw opening [m].
    """

    name: str
    offset_z_m: float
    lead_m: float
    clearance_per_side_m: float


#: The pad CENTRE: the 58 mm pad face closes on the object, the tips reach
#: 29 mm further. The default, and the only reference before 0.16.0.
PAD = GraspReference("pad", PAD_CENTRE_Z_M, PAD_TIP_Z_M - PAD_CENTRE_Z_M,
                     PAD_CLEARANCE_PER_SIDE_M)
#: The finger TIPS: nothing leads them, so a descent may bring them to the
#: support's clearance — which is how a 6 mm card is taken off a table.
TIP = GraspReference("tip", PAD_TIP_Z_M, 0.0, TIP_CLEARANCE_PER_SIDE_M)

REFERENCES = {PAD.name: PAD, TIP.name: TIP}
#: the ``contact`` argument's closed set, in the order a model is shown it
CONTACTS: Tuple[str, ...] = (PAD.name, TIP.name)

assert PAD.offset_z_m == _o.TOOL_Z_M, "the waypoint tool point IS the pad centre"


def hand_closedness(shape: str) -> float:
    """The closedness a non-grasping verb sends the hand to for ``shape``
    (``open`` / ``closed`` / ``pinched``) — the hand description's number."""
    if shape not in HAND_CLOSEDNESS:
        raise ValueError(f"hand shape must be one of {list(HAND_POSES)}, "
                         f"got {shape!r}")
    return float(HAND_CLOSEDNESS[shape])


#: default gap between the finger tips and the object's silhouette at the
#: standoff [m] — far enough that the travel onto the object is a straight line
#: the guard can clear, short enough to stay in reach
DEFAULT_STANDOFF_M = 0.08

#: How far ABOVE a measured surface an object's declared underside may sit and
#: the surface still be taken as what it stands on [m] (the same 30 mm a
#: Release accepts as "something under it will catch it"). An underside
#: BELOW the top is always the surface's: it is a declaration error, and the
#: measured table wins (L5).
SUPPORT_CATCH_M = 0.030


@dataclass(frozen=True)
class GraspSpec:
    """Everything that decides WHERE and HOW the hand meets the object.

    ``direction`` is how the hand travels onto it; the functions below take it
    in the BASE frame or in the grasped object's own frame
    (:meth:`resolved` turns any resolvable direction into the former).
    ``roll_rad`` is the turn about the direction after the jaws are squared to
    the object — a planner value from :func:`roll_candidates`, never a model's.
    """

    direction: Direction
    reference: GraspReference = PAD
    roll_rad: float = 0.0
    standoff_m: float = DEFAULT_STANDOFF_M

    def resolved(self, world, *, side: Optional[str] = None) -> "GraspSpec":
        """The same spec with its direction in BASE (raises ``FrameError``)."""
        d = self.direction.resolve(world, side=side)
        return GraspSpec(Direction(tuple(float(c) for c in d), BASE,
                                   via=self.direction.label()),
                         self.reference, self.roll_rad, self.standoff_m)

    def with_roll(self, roll_rad: float) -> "GraspSpec":
        return GraspSpec(self.direction, self.reference, float(roll_rad),
                         self.standoff_m)


def _d(obj: ObjectView, frames: FrameGraph, spec: GraspSpec) -> np.ndarray:
    """``spec.direction`` in base: a base vector, or ``obj``'s own axes."""
    frame = spec.direction.frame
    v = spec.direction.vector()
    if frame == BASE:
        return v
    if frame == object_frame(obj.name):
        return np.asarray(obj.pose_in_base(frames)[1].apply(v), dtype=float)
    raise ValueError(f"grasp geometry takes a base-frame or "
                     f"{object_frame(obj.name)!r} direction; resolve "
                     f"{spec.direction.label()!r} first (GraspSpec.resolved)")


# --------------------------------------------------------------------------- #
# the support, the tilt, and the object's own frame
# --------------------------------------------------------------------------- #

def support_of(world, name: str) -> Optional[SurfaceView]:
    """The MEASURED surface ``name`` stands on, or ``None`` when none is known.

    A level :class:`~manipulation_kit.world.SurfaceView` whose footprint is
    under the object's centre, whose top is below the object's top face, and
    under whose top the object's declared underside sits no more than
    :data:`SUPPORT_CATCH_M` above. The highest such surface wins (a tray on a
    table). An underside declared BELOW the top still belongs to it — the
    declaration is wrong and the table is not.
    """
    obj = world.find(name)
    if obj is None or isinstance(obj, SurfaceView):
        return None
    try:
        centre = obj.pose_in_base(world.frames)[0]
        under = obj.bottom_z(world.frames)
        top_face = obj.top_face_z(world.frames)
    except FrameError:
        return None
    best, best_top = None, -math.inf
    for item in world.surfaces():
        if item.name == name:
            continue
        try:
            if not item.level(world.frames) or not item.over(centre, world.frames):
                continue
            top = item.top_z(world.frames)
        except FrameError:
            continue
        if top < top_face and under - top <= SUPPORT_CATCH_M and top > best_top:
            best, best_top = item, top
    return best


def descent_floor(obj: ObjectView, frames: FrameGraph,
                  support: Optional[SurfaceView]) -> Tuple[float, str]:
    """``(z, note)``: what the finger tips must stay above, and where it came
    from. The support's measured top when there is one; the object's own
    underside ONLY when there is not (L5)."""
    if support is not None:
        z = support.top_z(frames)
        return z, (f"descent floor: {support.name}'s top at z={z:.3f} m "
                   f"(the surface {obj.name} stands on)")
    z = obj.bottom_z(frames)
    return z, (f"descent floor: {obj.name}'s own declared underside at "
               f"z={z:.3f} m (no measured surface under it)")


def tilted(obj: ObjectView, frames: FrameGraph) -> bool:
    """Tilted past :data:`~manipulation_kit.world.UPRIGHT_TOL_RAD` — the
    threshold above which a descent is taken in the object's own frame."""
    return obj.tilt_rad(frames) > UPRIGHT_TOL_RAD


def own_face_direction(obj: ObjectView, frames: FrameGraph) -> Direction:
    """A descent onto ``obj``'s own TOP face: minus its most-vertical body
    axis, expressed in ``object:<name>`` (req 4)."""
    columns = obj.pose_in_base(frames)[1].as_matrix()
    i = int(np.argmax([abs(float(columns[2, k])) for k in range(3)]))
    v = [0.0, 0.0, 0.0]
    v[i] = -1.0 if float(columns[2, i]) > 0.0 else 1.0
    return Direction(tuple(v), object_frame(obj.name))


# --------------------------------------------------------------------------- #
# the grasp point and the standoff
# --------------------------------------------------------------------------- #

def tool_point(p_contact, d_base, reference: GraspReference) -> np.ndarray:
    """The pad-centre TOOL POINT that puts ``reference`` at ``p_contact`` with
    the approach axis along ``d_base``. The identity for :data:`PAD`."""
    d = _o._unit(d_base)
    return (np.asarray(p_contact, dtype=float)
            - d * (float(reference.offset_z_m) - _o.TOOL_Z_M))


def _contact(obj: ObjectView, frames: FrameGraph, spec: GraspSpec,
             support: Optional[SurfaceView], droop_margin_m: float = 0.0,
             clearance_m: Optional[float] = None):
    """``(p_contact, raised, floor_z, floor_note)`` for ``spec``.

    The leading tips keep ``clearance_m`` over the floor when it is given
    (a descent that then SEARCHES for the floor by contact), else the rigid
    :data:`~.orientation.SUPPORT_CLEARANCE_M` plus ``droop_margin_m``."""
    d = _unit_d(obj, frames, spec)
    p = np.asarray(obj.pose_in_base(frames)[0], dtype=float).reshape(3).copy()
    floor, note = descent_floor(obj, frames, support)
    raised = False
    if float(d[2]) < -1e-3:
        # The tips lead the contact point by ``lead_m`` along the travel; they
        # must stop SUPPORT_CLEARANCE above the floor. Back the contact point
        # out ALONG THE TRAVEL until they do — straight up for ``down``.
        tips_z = float(p[2] + d[2] * spec.reference.lead_m)
        least = floor + (_o.SUPPORT_CLEARANCE_M + float(droop_margin_m)
                         if clearance_m is None else float(clearance_m))
        if tips_z < least - 1e-9:        # not for a float's last bit
            p = p - d * ((least - tips_z) / -float(d[2]))
            raised = True
    return p, raised, floor, note


def _unit_d(obj, frames, spec) -> np.ndarray:
    return _o._unit(_d(obj, frames, spec))


def grasp_pose(obj: ObjectView, frames: FrameGraph, spec: GraspSpec, *,
               side: str, support: Optional[SurfaceView],
               droop_margin_m: float = 0.0,
               clearance_m: Optional[float] = None
               ) -> Tuple[np.ndarray, R, List[str]]:
    """``(p_tool, r_tcp, notes)``: the tool pose at contact.

    The contact point is the object's resolved centre, backed out along the
    travel just far enough that the finger tips keep
    :data:`~.orientation.SUPPORT_CLEARANCE_M` over the descent floor
    (:func:`descent_floor`: the support when known, the object's underside
    only when not — the notes say which). ``p_tool`` is the PAD-CENTRE point
    that puts ``spec.reference`` there; ``r_tcp`` is
    :func:`~.orientation.grasp_orientation` squared to the object and rolled
    by ``spec.roll_rad``.

    ``droop_margin_m`` is how far the real arm sags below the commanded pose
    (``clearance.ClearancePolicy.droop_margin_m``, 0 for the rigid model); it
    is added to that fingertip clearance. ``clearance_m`` replaces both: the
    tips' height over the floor for a descent that finishes BY CONTACT
    (:func:`descends_by_contact`), where no sag has to be guessed.
    """
    d = _unit_d(obj, frames, spec)
    r_tcp = _o.grasp_orientation(side, d, obj, frames, roll_rad=spec.roll_rad)
    p_contact, raised, _floor, floor_note = _contact(obj, frames, spec, support,
                                                     droop_margin_m, clearance_m)
    notes = [floor_note] if float(d[2]) < -1e-3 else []
    if raised:
        centre = np.asarray(obj.pose_in_base(frames)[0], dtype=float)
        notes.append(f"grasping {(p_contact[2] - float(centre[2])) * 1000:.0f} "
                     f"mm above the object's centre so the pad tips clear "
                     f"what it is standing on")
    if spec.reference is not PAD:
        notes.append(f"contact at the {spec.reference.name}s "
                     f"({spec.reference.offset_z_m * 1000:.0f} mm along the "
                     f"tool axis)")
    return tool_point(p_contact, d, spec.reference), r_tcp, notes


def standoff_point(obj: ObjectView, frames: FrameGraph, spec: GraspSpec,
                   p_grasp, r_tcp: Optional[R] = None) -> np.ndarray:
    """Where the tool point waits before travelling onto the object.

    ``standoff_m`` is the gap between the LEADING TOOL GEOMETRY — the finger
    tips, :data:`~...description.PAD_TIP_Z_M` along the tool axis whatever the
    reference — and the object's SILHOUETTE along the direction (its near face,
    ``extent_along(d) / 2`` before its centre). It used to be measured from the
    grasp point: with the tips 29 mm past a tool point that is itself only
    ~32 mm above the table, an 80 mm standoff left the tips 51 mm up, under a
    110 mm cup's rim (C.4). ``r_tcp`` is accepted for symmetry with
    :func:`grasp_pose`; the approach axis IS the direction.
    """
    d = _unit_d(obj, frames, spec)
    p_grasp = np.asarray(p_grasp, dtype=float).reshape(3)
    lead = PAD_TIP_Z_M - _o.TOOL_Z_M       # tips past the tool point
    centre = np.asarray(obj.pose_in_base(frames)[0], dtype=float)
    near = float(np.dot(centre, d)) - obj.extent_along(d, frames) / 2.0
    back = float(np.dot(p_grasp, d)) + lead - near + float(spec.standoff_m)
    return p_grasp - d * max(back, 0.0)


# --------------------------------------------------------------------------- #
# the fit, and the one roll sweep
# --------------------------------------------------------------------------- #

def graspable_width_m(reference: GraspReference = PAD,
                      open_gap_m: Optional[float] = None) -> float:
    """The widest object ``reference`` can close on, for a hand opening to
    ``open_gap_m`` — the executor's measured ``HandState.open_gap_m``
    (``GripperView.open_gap_m``) when the world carries one, the hand
    description's nominal driven opening when it does not. Never an
    environment variable."""
    opening = DRIVEN_OPEN_GAP_M if open_gap_m is None else float(open_gap_m)
    return opening - 2.0 * float(reference.clearance_per_side_m)


def fit_problems(obj: ObjectView, frames: FrameGraph, spec: GraspSpec,
                 r_tcp: R, *, open_gap_m: Optional[float] = None,
                 support: Optional[SurfaceView] = None) -> List[Unmet]:
    """Everything that stops ``spec`` closing on ``obj`` with the jaws at
    ``r_tcp`` (empty when it can). :func:`fits` is the first of these.

    FLAT first: when the contact point had to be backed out past the object's
    near face to keep the tips off the floor, the jaws would close on air.
    That is computed from ``reference.lead_m``, so it fires for a 6 mm card at
    the pads (29 mm lead) and not at the tips (none). Then WIDE: the object's
    extent ALONG THE JAW AXIS (not its smallest side, R9) against
    :func:`graspable_width_m` for this reference and this hand.
    """
    d = _unit_d(obj, frames, spec)
    ref = spec.reference
    problems: List[Unmet] = []
    p_contact, _raised, _floor, floor_note = _contact(obj, frames, spec, support)
    centre = np.asarray(obj.pose_in_base(frames)[0], dtype=float)
    depth = float(np.dot(p_contact - centre, d))
    if depth < -obj.extent_along(d, frames) / 2.0:
        tall = obj.vertical_extent(frames)
        hint = ("grasp it at the fingertips (contact='tip'), come in from the "
                "side, or use a different tool" if ref is PAD else
                "it is thinner than the tips' clearance over the surface; "
                "slide it to an edge or use a different tool")
        problems.append(Unmet(
            OBJECT_TOO_FLAT,
            f"{obj.name} stands {tall * 1000:.0f} mm tall and a {ref.name} "
            f"grasp's finger tips reach {ref.lead_m * 1000:.0f} mm past its "
            f"contact point, so it would close above the object with the tips "
            f"still {_o.SUPPORT_CLEARANCE_M * 1000:.0f} mm over the floor "
            f"({floor_note})",
            hint, {"height_m": round(tall, 4), "contact": ref.name,
                   "lead_m": round(ref.lead_m, 4)}))
    width = _o.grasp_width(obj, frames, r_tcp)
    graspable = graspable_width_m(ref, open_gap_m)
    if width > graspable:
        opening = DRIVEN_OPEN_GAP_M if open_gap_m is None else float(open_gap_m)
        problems.append(Unmet(
            OBJECT_TOO_WIDE,
            f"{obj.name} presents {width * 1000:.0f} mm across the jaw axis of "
            f"this {spec.direction.label()} {ref.name} grasp, and the driven "
            f"jaws take {graspable * 1000:.0f} mm (opening "
            f"{opening * 1000:.0f} mm, "
            f"{ref.clearance_per_side_m * 1000:.0f} mm clearance per side)",
            "approach it across a narrower face, or use a different tool",
            {"presented_width_m": round(width, 4),
             "graspable_width_m": round(graspable, 4)}))
    return problems


def fits(obj: ObjectView, frames: FrameGraph, spec: GraspSpec, r_tcp: R, *,
         open_gap_m: Optional[float] = None,
         support: Optional[SurfaceView] = None) -> Optional[Unmet]:
    """Why ``spec`` cannot close on ``obj`` with the jaws at ``r_tcp``, or
    ``None`` when it can — the first of :func:`fit_problems` (flat before
    wide)."""
    problems = fit_problems(obj, frames, spec, r_tcp, open_gap_m=open_gap_m,
                            support=support)
    return problems[0] if problems else None


#: the quarter turns tried after the squared posture, in order. A parallel
#: gripper is symmetric under a half turn, so these two close across the SAME
#: side of the object from two different wrist postures (one may be inside a
#: joint limit when the other is not).
QUARTER_TURNS_RAD: Tuple[float, ...] = (-math.pi / 2, math.pi / 2)


def roll_candidates(obj: ObjectView, frames: FrameGraph, spec: GraspSpec, *,
                    open_gap_m: Optional[float] = None) -> Tuple[float, ...]:
    """The rolls worth trying, best first. THE ONE SWEEP.

    ``0.0`` — the jaws squared across the object's footprint axis — first,
    then each quarter turn whose presented width still fits this reference's
    jaws. A roll whose width does not fit is dropped rather than tried: the
    planner would only discover the same refusal three times. When nothing
    fits, ``(0.0,)``, so the refusal a caller sees is the squared grasp's.

    The width is the same for both hands (the mirrored seed is a half turn
    about the approach axis, which maps the jaw axis onto its negative), so
    no side is needed. ``Approach``, ``Grasp`` — and through them
    ``reach.plan_chain`` / ``reach.choose_side`` and any loop that decodes a
    model's call — take their roll from here and nowhere else.
    """
    d = _unit_d(obj, frames, spec)
    out = []
    for roll in (0.0,) + QUARTER_TURNS_RAD:
        r = _o.grasp_orientation("left", d, obj, frames, roll_rad=roll)
        if _o.grasp_width(obj, frames, r) <= graspable_width_m(
                spec.reference, open_gap_m):
            out.append(roll)
    return tuple(out) if out else (0.0,)


def achieved_clearance(p_tool, r_tcp: R, floor_z: float) -> float:
    """How far the finger TIPS end up above the descent floor for a SOLVED
    tool point. Negative means the fingers are in the surface."""
    tips = (np.asarray(p_tool, dtype=float)
            + r_tcp.apply([0.0, 0.0, PAD_TIP_Z_M - _o.TOOL_Z_M]))
    return float(tips[2]) - float(floor_z)



# --------------------------------------------------------------------------- #
# the fingertip descent that finishes by contact
# --------------------------------------------------------------------------- #

#: An object thinner than this is taken at the tips BY CONTACT even with no
#: measured surface under it [m]: twice the 29 mm the tips lead the pad
#: centre. Thinner than that and a fixed fingertip height decides whether the
#: jaws close on the object or on the air above it.
TIP_CONTACT_THIN_M = 2.0 * PAD.lead_m

#: Where a contact-finished descent stops BEFORE it searches: the finger tips
#: this far over the descent floor [m]. Not a sag guess — the search below it
#: measures the floor — just far enough that a sagging arm (run 7: ~10 mm at
#: x 0.48) does not meet the table on the plain descent.
TIP_SEARCH_START_M = 0.015

#: How far PAST the modelled floor the search may travel before it gives up
#: [m] — the 5 mm a tape-measured / probed table height is published to
#: (``contact.PROBE_HEIGHT_UNCERTAINTY_M``). A search that reaches it met
#: nothing: the floor is lower than the scene says, and the run says so.
CONTACT_OVERTRAVEL_M = 0.005

#: The joint-torque rise that ends the search [Nm]: under the probe's 4 Nm
#: (the tips should touch, not push), over the < 2 Nm a free-air leg is
#: expected to show (docs/probe-hardware-trial.md G4). UNMEASURED on
#: hardware; the tip trial reports the torque it stopped at.
TIP_CONTACT_NM = 3.0


def descends_by_contact(obj: ObjectView, frames: FrameGraph, spec: GraspSpec,
                        support: Optional[SurfaceView]) -> bool:
    """Does this grasp finish its descent BY CONTACT (a ``ContactStep`` that
    stops when the tips touch the support), rather than at a fixed height?

    For a fingertip grasp travelling DOWN onto an object that stands on a
    known surface, or that is thinner than :data:`TIP_CONTACT_THIN_M`. The
    d1-2 tip trial (2026-09-23) is why: the fixed height put the tips 14.9 mm
    over the wagon (3 mm clearance + 12 mm droop margin), the arm did not sag
    in that near posture, and the jaws closed 7 mm above an 8 mm slab —
    three of three. A pad grasp keeps the fixed height (and the droop margin):
    its tips are 29 mm past the pads and must not touch anything.
    """
    if spec.reference is not TIP:
        return False
    d = _unit_d(obj, frames, spec)
    if not _o.is_descent(d):
        return False
    return support is not None or obj.vertical_extent(frames) < TIP_CONTACT_THIN_M


# --------------------------------------------------------------------------- #
# after the tips touched: back off the support before closing
# --------------------------------------------------------------------------- #

#: How far a fingertip grasp retreats along minus its travel once the search
#: stopped ON THE SUPPORT, before the jaws close [m] — the default of
#: :attr:`.clearance.ClearancePolicy.contact_backoff_m`.
#:
#: WHY. Jaws closed with the tips pressed on the table drag across it: on
#: d1-2 (2026-09-24, ``servo-judgeonly-2`` turn 6) a tape grasp whose declared
#: centre was 55 mm off stopped its search on the table, the close stroke
#: dragged the tips across the surface and the friction stalled the jaws at
#: 48 mm — inside the 46-54 mm window of the 50 mm declaration — and the
#: daemon reported a hold. At closure nothing tells that drag from an object.
#: Lifted clear of the surface, jaws with nothing between them close to the
#: empty gap and say so.
#:
#: WHY 1 mm. Enough that the tips no longer touch, small enough that a thin
#: thing is still between them (an 8 mm slab keeps 7 mm of it). It is above
#: what the executor resolves (:data:`CONTACT_BACKOFF_MIN_M`) by twice.
CONTACT_BACKOFF_M = 0.001

#: The smallest back-off the executor carries out reliably [m]. Built from
#: the numbers the executor owns, not from the ones it verifies with:
#:
#: * an ``exact`` contact-leg knot is solved to 0.2 mm / 1 mrad at Link7,
#:   <= 0.3 mm at the tool (``planning`` / ``docs/probe-hardware-trial.md``),
#:   and the daemon's joint interpolation between 5 mm knots bows < 0.01 mm
#:   (``planning.CONTACT_KNOT_M``);
#: * the stop itself is placed to one state poll at the leg speed:
#:   10 mm/s x 20 ms = 0.2 mm (``contact.PROBE_SPEED_M_S``), and ten d1-2
#:   table contacts repeated to sigma 0.09 mm.
#:
#: 0.3 + 0.2 = 0.5 mm. NOT the model's correction grid
#: (:data:`~.types.NUDGE_GRID_M`, 10 mm at its finest — a menu of corrections
#: a model may ask for, not the arm's resolution), and NOT the arrival
#: barrier (3 deg / 5 mm across / 10 mm along — deadlines against a jam, far
#: too coarse to see a millimetre): the back-off is therefore MEASURED after
#: it ran (``ContactReport.tip_z_after_m``) rather than trusted to a barrier.
CONTACT_BACKOFF_MIN_M = 0.0005

#: A search stop at most this far short of the modelled support (or anywhere
#: past it) is a stop ON the support [m] — the 5 mm a tape-measured table
#: height is published to (:data:`CONTACT_OVERTRAVEL_M`), unless the support
#: states a tighter ``height_uncertainty_m``. Capped at half the object's
#: extent along the travel, so a stop on a thin object's top is not taken for
#: the table (an 8 mm slab: 4 mm).
SURFACE_CONTACT_BAND_M = CONTACT_OVERTRAVEL_M


def surface_backoff(obj: ObjectView, frames: FrameGraph, d, *,
                    support: Optional[SurfaceView], floor_z: float,
                    clearance_m: float, distance_m: float):
    """``(SurfaceBackoff | None, note)`` for a fingertip search along ``d``.

    ``clearance_m`` is how far the SOLVED leg start leaves the tips above
    ``floor_z`` (:func:`achieved_clearance`), so ``surface_at_m`` — where the
    tips meet the modelled support along the leg — is that height over the
    travel's descent rate. ``distance_m`` is the policy's back-off, raised to
    :data:`CONTACT_BACKOFF_MIN_M` when it is below it. ``None`` (the stop is
    only reported, nothing moves) when the retreat would lift the tips past
    half the object's own extent along the travel: so thin a thing is taken
    with the tips on the surface or not at all.
    """
    d = np.asarray(d, dtype=float).reshape(3)
    d = d / max(float(np.linalg.norm(d)), 1e-12)
    descent = max(-float(d[2]), 1e-6)
    surface_at = float(clearance_m) / descent
    extent = float(obj.extent_along(d, frames))
    uncertainty = (None if support is None
                   else getattr(support, "height_uncertainty_m", None))
    band = min(SURFACE_CONTACT_BAND_M if uncertainty is None
               else float(uncertainty), extent / 2.0)
    name = "" if support is None else support.name
    where = (f"{name}'s top" if name else
             f"{obj.name}'s own declared underside")
    distance = float(distance_m)
    raised = ""
    if 0.0 < distance < CONTACT_BACKOFF_MIN_M:
        raised = (f" (raised from {distance * 1000:.2f} mm to the "
                  f"{CONTACT_BACKOFF_MIN_M * 1000:.1f} mm the executor "
                  f"resolves)")
        distance = CONTACT_BACKOFF_MIN_M
    if distance > extent / 2.0:
        return None, (
            f"no back-off after the tips touch: {obj.name} is "
            f"{extent * 1000:.1f} mm along the travel, and a "
            f"{distance * 1000:.1f} mm retreat would lift the tips past half "
            f"of it; the jaws close where the tips stopped")
    lead = PAD_TIP_Z_M - _o.TOOL_Z_M
    backoff = SurfaceBackoff(distance, surface_at, band, name, lead)
    if distance <= 0.0:
        return backoff, (
            f"contact policy back_off with a 0 mm back-off: a stop on "
            f"{where} is recorded and nothing is retracted")
    return backoff, (
        f"contact policy back_off: when the tips stop on {where} (at most "
        f"{band * 1000:.1f} mm short of it, {surface_at * 1000:.1f} mm into "
        f"the search) the hand retreats {distance * 1000:.1f} mm along minus "
        f"the travel before the jaws close{raised}; a stop higher up, on "
        f"{obj.name}, closes where it stopped")
