"""The two contact verbs: ``probe`` and ``press``. Neither names a direction.

Probing the height of a table and pressing an elevator button are ONE
operation in two directions — travel along a :class:`~manipulation_kit.world.Direction`
until something resists — so they are two verbs over one executor capability
(``move_until``, :class:`~.types.ContactStep`), and the direction is an
argument: ``probe(direction=down)`` measures a table, ``probe(direction=forward)``
a wall, ``press(target="button", direction=forward)`` a button. ``touch_down``
does not exist and must not.

POSITION MODE ONLY (Shu, 2026-09-22). The contact leg is a position-commanded
straight line watched for a torque rise; no compliant or torque arm mode is
set, and nothing here is a commanded force. ``contact_nm`` / ``force_nm`` are
MEASURED thresholds (:class:`~.types.ContactCriterion`).

Both plan as: stand off, re-oriented so the tool's approach axis IS the
direction (:func:`.orientation.align_tool`), then the contact leg along
``+direction``. The standoff and the leg are ordinary
:class:`~.types.Waypoint`\\ s, solved by the same IK, clamp and guard as every
other verb — the leg with ``allow_via=False``, because its shape is the
promise.

THE RESULT FEEDS THE SCENE. A run's :class:`~manipulation_kit.executor.ContactReport`
is folded into the next world by :func:`record_contacts`, as a
:class:`~manipulation_kit.world.ContactView` (the evidence ``ContactMade``
reads) and — with ``declare_as`` — as a :class:`~manipulation_kit.world.SurfaceView`
whose face IS the measured plane: one probe gives a plane through the contact
with the probe's normal, three give a least-squares plane with a real normal
(:func:`fit_plane`). The table the guard needs is then the table the probe
measured, not a number in a scene file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from ..hands.d1.parallel_gripper.description import HAND_CLOSEDNESS
from ..world import ContactView, SurfaceView, WorldView
from ..world.direction import ALIASES, BASE, Direction
from . import grasp_geometry as gg
from . import orientation as ap
from . import verifiers as V
from .arguments import ROLE_ANY, check_arguments
from .clearance import SceneGate
from .planning import (DUPLICATE_KNOT_RAD, IncompleteObservation, Kin,  # noqa: F401
                       coupled_limit_notes, CONTACT_KNOT_M, leg_knots,
                       solve_path)
from .types import (AUTO, BAD_ARGUMENT, BAD_SIDE, ARM_UNKNOWN, JOINT_LIMIT,
                    ContactCriterion, ContactStep, GripStep, JointStep, Plan,
                    PlanBinding, PlanError, Primitive, SIDES, SettleStep,
                    Unmet, Verifier, Waypoint)
from .verbs import (SETTLE_S, _coerce_direction, _incomplete, _locate,
                    _must_be_free, _resolve, _resolved_side, _unchecked_note)

# --------------------------------------------------------------------------- #
# numbers
# --------------------------------------------------------------------------- #

#: How fast the tool travels along a contact leg [m/s]. What a leg overshoots
#: after first contact is this speed times the transport's detection latency
#: (one state poll: 20 ms at 50 Hz, i.e. 0.2 mm here), and the command is
#: frozen while the rise is confirmed, so confirming costs no travel. Slow on
#: purpose; the d1-2 trial (``docs/probe-hardware-trial.md``) is what may
#: raise it.
PROBE_SPEED_M_S = 0.01
PRESS_SPEED_M_S = 0.01
#: How far off a press target's near face the hand stands before pressing [m].
PRESS_STANDOFF_M = 0.05
#: The height uncertainty a contact-measured surface is published with, when
#: its own fit residual is smaller [m]. PROVISIONAL: it is the hardware
#: gate's accuracy target (table z within +-3 mm of the tape, decision 2)
#: widened to 5 mm until that gate has measured the real number.
PROBE_HEIGHT_UNCERTAINTY_M = 0.005
#: A surface a probe publishes with no existing footprint to keep extends this
#: far around the contacts in its own plane [m] — what was touched, plus a
#: margin, and not a metre of table nobody measured.
CONTACT_FOOTPRINT_MARGIN_M = 0.05
#: The thickness of the slab a measured plane is published as [m].
CONTACT_SLAB_M = 0.02
#: Three contacts closer to a LINE than this [m] fit no plane: the normal
#: about that line is unobserved, and the probes' own normal is used.
COLLINEAR_TOL_M = 0.005
#: A probe's postures keep at least this far from every joint's BOX limit
#: [deg], or the roll is not taken. A probe is repeated (a trial is 3 air +
#: 10 table probes with a lift between each), so a roll that parks a joint
#: next to its stop is the posture the next cycle has to start from; on d1-2
#: (2026-09-23) the probe trial ended with J5 at 171.5 of 173 deg and the
#: next leg's solver pinned there. Refused with ``joint_limit`` when no roll
#: keeps it, instead of drifting further.
PROBE_BOX_MARGIN_DEG = 10.0


def _unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(3)
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else v


# --------------------------------------------------------------------------- #
# planes from contacts
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class ContactPlane:
    """A plane fitted to contact points: through ``point``, along ``normal``.

    ``fitted`` says whether the normal is MEASURED (three or more contacts
    spanning an area) or the probes' own hint (one or two, or a line).
    ``rms_m`` is the fit residual.
    """

    point: np.ndarray
    normal: np.ndarray
    rms_m: float
    count: int
    fitted: bool


def fit_plane(points: Sequence[Sequence[float]],
              hints: Sequence[Sequence[float]]) -> ContactPlane:
    """Least-squares plane through contact points, oriented by the hints.

    Implemented here as a 3-point SVD fit rather than borrowed from
    :mod:`manipulation_kit.perception.plane`, whose planes are fitted to
    IMAGE edges (a camera's view of a table top), not to points in space.

    * one point: the plane through it, normal = the probe's hint;
    * two, or several on a line: through their mean, normal = the hint made
      perpendicular to the line (the only direction they constrain);
    * three or more spanning an area: the SVD normal — a REAL normal, which
      is how a tilted table shows up — flipped to agree with the hints.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    if pts.shape[0] == 0:
        raise ValueError("fit_plane needs at least one point")
    hint = _unit(np.mean(np.asarray(hints, dtype=float).reshape(-1, 3), axis=0))
    centre = pts.mean(axis=0)
    if pts.shape[0] == 1:
        return ContactPlane(pts[0], hint, 0.0, 1, False)
    _u, sv, vt = np.linalg.svd(pts - centre, full_matrices=True)
    spread = sv / math.sqrt(pts.shape[0])
    if pts.shape[0] < 3 or len(spread) < 2 or float(spread[1]) < COLLINEAR_TOL_M:
        line = vt[0]
        normal = _unit(hint - line * float(np.dot(hint, line)))
        if float(np.linalg.norm(normal)) < 1e-9:
            normal = hint
        rms = float(np.sqrt(np.mean(np.dot(pts - centre, normal) ** 2)))
        return ContactPlane(centre, normal, rms, int(pts.shape[0]), False)
    normal = _unit(vt[2])
    if float(np.dot(normal, hint)) < 0.0:
        normal = -normal
    rms = float(np.sqrt(np.mean(np.dot(pts - centre, normal) ** 2)))
    return ContactPlane(centre, normal, rms, int(pts.shape[0]), True)


def surface_from_contacts(name: str, contacts: Sequence[ContactView], *,
                          like: Optional[SurfaceView] = None, frames=None,
                          stamp: float = 0.0) -> SurfaceView:
    """The :class:`SurfaceView` a set of contacts measures.

    ``like`` is the surface of that name the world already had, if any: its
    footprint and in-plane orientation are KEPT and only its plane — height
    and tilt — is replaced by the measured one. With nothing to keep, the
    footprint is what was touched plus :data:`CONTACT_FOOTPRINT_MARGIN_M`.
    """
    plane = fit_plane([c.p for c in contacts], [c.normal for c in contacts])
    n = plane.normal
    uncertainty = max(plane.rms_m, PROBE_HEIGHT_UNCERTAINTY_M)
    if like is not None and frames is not None:
        p, r = like.pose_in_base(frames)
        axes = r.as_matrix()
        # the old footprint's centre, dropped onto the measured plane
        k = int(np.argmax([abs(float(np.dot(axes[:, i], n))) for i in range(3)]))
        others = [i for i in range(3) if i != k]
        face = np.asarray(p, dtype=float)
        centre = face - n * float(np.dot(face - plane.point, n))
        footprint = (float(like.size[others[0]]), float(like.size[others[1]]))
        return SurfaceView.from_plane(
            name, centre, n, footprint_m=footprint,
            thickness_m=float(like.size[k]), yaw_axis=axes[:, others[0]],
            plane_source="contact", height_uncertainty_m=uncertainty,
            stamp=stamp, colour=like.colour)
    pts = np.asarray([c.p for c in contacts], dtype=float)
    centre = plane.point - n * float(np.dot(plane.point - pts.mean(axis=0), n))
    seed = np.array([1.0, 0.0, 0.0]) - n * float(n[0])
    if float(np.linalg.norm(seed)) < 1e-6:
        seed = np.array([0.0, 1.0, 0.0]) - n * float(n[1])
    x = _unit(seed)
    y = np.cross(n, x)
    rel = pts - centre
    span_x = float(np.ptp(rel @ x)) if len(pts) > 1 else 0.0
    span_y = float(np.ptp(rel @ y)) if len(pts) > 1 else 0.0
    footprint = (span_x + 2 * CONTACT_FOOTPRINT_MARGIN_M,
                 span_y + 2 * CONTACT_FOOTPRINT_MARGIN_M)
    return SurfaceView.from_plane(
        name, centre, n, footprint_m=footprint, thickness_m=CONTACT_SLAB_M,
        yaw_axis=x, plane_source="contact", height_uncertainty_m=uncertainty,
        stamp=stamp)


def record_contacts(world: WorldView, reports: Any,
                    verb: Optional[Primitive] = None, *,
                    stamp: Optional[float] = None) -> WorldView:
    """Fold a run's MEASURED contacts into the world the verifier will read.

    ``reports`` is a :class:`~manipulation_kit.executor.RunReport` (its
    ``contacts``) or a sequence of
    :class:`~manipulation_kit.executor.ContactReport`. Each becomes a
    :class:`ContactView` whose ``p`` is where the SURFACE was met — the
    leading fingertip, :attr:`.grasp_geometry.PAD` ``lead_m`` past the
    measured tool point along the travel (the leg travels along the tool's
    own approach axis, by construction) — and, when ``verb`` asks to
    ``declare_as`` a surface, every contact under that name so far is fitted
    and the surface is published (or re-measured) with ``plane_source
    ="contact"``. The world is otherwise untouched.
    """
    items = getattr(reports, "contacts", reports)
    name = str(getattr(verb, "declare_as", "") or "")
    verb_name = "" if verb is None else verb.name()
    when = float(world.stamp if stamp is None else stamp)
    new: List[ContactView] = []
    for report in items:
        travel = -np.asarray(report.normal_hint, dtype=float)
        tip = (np.asarray(report.p_tool, dtype=float)
               + _unit(travel) * gg.PAD.lead_m)
        new.append(ContactView(
            report.side, bool(report.made), tip, report.p_tool,
            report.normal_hint, report.stopped_by, float(report.travel_m),
            float(report.torque_nm), name if report.made else "", verb_name,
            when))
    if not new:
        return world
    contacts = tuple(world.contacts) + tuple(new)
    objects = tuple(world.objects)
    measured = [c for c in contacts if c.made and name and c.surface == name]
    if name and measured and any(c.made for c in new):
        like = world.find(name)
        surface = surface_from_contacts(
            name, measured,
            like=like if isinstance(like, SurfaceView) else None,
            frames=world.frames, stamp=when)
        if world.find(name) is None:
            objects = objects + (surface,)
        else:
            objects = tuple(surface if o.name == name else o for o in objects)
    return world.with_(contacts=contacts, objects=objects)


# --------------------------------------------------------------------------- #
# shared planning
# --------------------------------------------------------------------------- #

def _closedness(hand: str) -> float:
    return float(HAND_CLOSEDNESS[hand])


def _upward(direction: Direction, d: np.ndarray) -> List[Unmet]:
    """Refuse a leg that travels UP: no hand here leads with its fingertips
    upward (``align_tool`` cannot face the tool that way)."""
    if float(d[2]) > ap.VERTICAL_COS:
        return [Unmet(BAD_ARGUMENT,
                      f"direction {direction.label()!r} travels UP, and no "
                      f"hand here can lead with its fingertips upward",
                      "use down or a horizontal direction",
                      {"argument": "direction",
                       "axis_base": [round(float(c), 3) for c in d]})]
    return []


#: the rolls a Probe tries about its direction, in order: the wrist's own,
#: then the quarter turns, then the half turn
PROBE_ROLLS_RAD: Tuple[float, ...] = ((0.0,) + gg.QUARTER_TURNS_RAD
                                      + (math.pi,))


def _contact_plan(verb: Primitive, world: WorldView, kin, side: str, *,
                  d: np.ndarray, p_standoff, standoff_label: str,
                  standoff_via: bool, standoff_arrive: bool,
                  travel_m: float, criterion: ContactCriterion,
                  speed_m_s: float, hold_s: float, retract: bool,
                  notes: Sequence[str], roll_to=None,
                  roll_rad: float = 0.0, keep=None) -> Any:
    """Standoff, then the contact leg — solved, split and wrapped.

    The standoff's joint steps stay ordinary :class:`JointStep`\\ s. The
    leg's go INSIDE the :class:`ContactStep` together with each knot's
    distance along ``d``, measured by forward kinematics on the solved
    posture (the leg's own geometry, not its ideal).
    """
    # The jaw-gap axis is kept as close to where it is as the direction
    # allows: a probe is about the fingertips, not the roll, and turning the
    # wrist half a revolution in place is how a posture next to the body
    # becomes a guard refusal.
    r_tool = ap.align_tool(side, d, roll_to=roll_to, roll_rad=roll_rad,
                           keep=keep)
    p_standoff = np.asarray(p_standoff, dtype=float)
    waypoints = [
        Waypoint(standoff_label, p_standoff, r_tool, allow_via=standoff_via,
                 arrive=standoff_arrive),
        Waypoint("contact_limit", p_standoff + d * float(travel_m), r_tool,
                 allow_via=False, knot_m=CONTACT_KNOT_M)]
    # The scene gates the standoff transit and the leg — except the surface
    # the leg is MEANT to reach: the first obstacle its ray meets (and a
    # press's named target), which a probe ends past by construction.
    target = str(getattr(verb, "target", "") or "")
    scene = SceneGate.for_contact(world, kin, p_standoff, d, travel_m,
                                  exclude=(target,) if target else ())
    try:
        with Kin(kin, world, scene=scene) as borrowed:
            q_now = borrowed.joints(side)
            steps, error, detours = solve_path(borrowed, side, waypoints,
                                               primitive=verb.name())
            if error is not None:
                return error
            standoff = [s for s in steps if s.waypoint == 0]
            leg = [s for s in steps if s.waypoint == 1]
            q_start = standoff[-1].q if standoff else q_now
            path, dist = leg_knots(borrowed, side, q_start, leg, d)
    except IncompleteObservation as exc:
        return _incomplete(verb, side, exc)
    if len(path) < 2 or dist[-1] <= 1e-4:
        return PlanError(BAD_ARGUMENT, f"the {side} contact leg has no length",
                         primitive=verb.name(), side=side)
    contact = ContactStep(side, Direction(tuple(d), BASE), float(travel_m),
                          criterion, waypoint=1, path=tuple(path),
                          s=tuple(dist), speed_m_s=float(speed_m_s),
                          hold_s=float(hold_s), retract=bool(retract))
    all_steps = ((GripStep(side, _closedness(verb.hand), "soft", 0),)
                 + tuple(standoff) + (contact, SettleStep(SETTLE_S)))
    measured = ((f"the {scene.contact_target!r} it is aimed at is left out "
                 f"of the scene check (the leg is meant to reach it)",)
                if scene.contact_target else ())
    return Plan(verb.name(), side, tuple(waypoints), all_steps,
                tuple(notes) + tuple(detours) + measured
                + tuple(_unchecked_note(scene))
                + coupled_limit_notes(kin, all_steps),
                binding=PlanBinding.of(world, kin))


def _box_margin(kin, side: str, steps) -> Tuple[float, str]:
    """The smallest distance [deg] any posture of ``steps`` keeps from a box
    limit, and which joint at which angle that is."""
    lo, hi = (np.degrees(np.asarray(v, dtype=float)) for v in kin.limits(side))
    worst, where = float("inf"), ""
    for step in steps:
        if isinstance(step, JointStep):
            postures = (step.q,) if step.side == side else ()
        elif isinstance(step, ContactStep):
            postures = step.path if step.side == side else ()
        else:
            continue
        for q in postures:
            deg = np.degrees(np.asarray(q, dtype=float))
            gaps = np.minimum(deg - lo, hi - deg)
            j = int(np.argmin(gaps))
            if float(gaps[j]) < worst:
                worst = float(gaps[j])
                where = f"J{j + 1}={deg[j]:+.1f} deg"
    return worst, where


# --------------------------------------------------------------------------- #
# the verbs
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Probe(Primitive):
    """Move the hand in one direction until something resists, and report WHERE.

    From where the hand is now: it is turned so its fingertips lead along
    ``direction``, then travels at most ``max_travel_m`` and stops at the
    first rise of ``contact_nm`` on any joint. Where it stopped is MEASURED.
    ``declare_as`` publishes the touched surface under that name (three
    probes under one name fit a plane).
    """

    VERB = "probe"
    DIRECTION_ARRIVES = True
    side: str = AUTO
    direction: Direction = ALIASES["down"]
    max_travel_m: float = 0.15
    contact_nm: float = 4.0
    hand: str = "closed"
    declare_as: str = ""

    def __post_init__(self) -> None:
        _coerce_direction(self)

    def _side(self, world: WorldView) -> Optional[str]:
        if self.side in SIDES:
            return self.side
        for side in SIDES:
            if world.arm(side) is not None and not _must_be_free(world, side):
                return side
        return None

    def preconditions(self, world: WorldView) -> List[Unmet]:
        unmet = check_arguments(self)
        if unmet:
            return unmet
        side = self._side(world)
        if side is None:
            return [Unmet(BAD_SIDE, "no free hand with a measured arm to probe "
                                    "with", "name the side, or free a hand")]
        if world.arm(side) is None:
            return [Unmet(ARM_UNKNOWN, f"the {side} arm is not in this "
                                       f"observation")]
        unmet += _must_be_free(world, side)
        unmet += _declared_name_ok(world, self.declare_as)
        if self.direction.frame != "tool":
            d, bad = _resolve(self.direction, world, side)
            unmet += bad
            if d is not None:
                unmet += _upward(self.direction, d)
        return unmet

    def plan(self, world: WorldView, kin) -> Any:
        unmet = self.preconditions(world)
        if unmet:
            return self._unmet_error(unmet, self._side(world) or "")
        side = self._side(world)
        try:
            with Kin(kin, world) as borrowed:
                p_tool, r_tool = borrowed.tool_pose(side)
        except IncompleteObservation as exc:
            return _incomplete(self, side, exc)
        d, bad = _resolve(self.direction, world, side, tool_r=r_tool)
        if bad:
            return self._unmet_error(bad, side)
        d = _unit(d)
        bad = _upward(self.direction, d)
        if bad:
            return self._unmet_error(bad, side)
        notes = [] if self.side != AUTO else [f"side chosen automatically: {side}"]
        notes.append(
            f"probes {self.direction.label()} from where the hand is, at most "
            f"{self.max_travel_m * 1000:.0f} mm at "
            f"{PROBE_SPEED_M_S * 1000:.0f} mm/s, stopping at a "
            f"{self.contact_nm:.1f} Nm joint-torque rise (position control, "
            f"measured, never commanded)")
        # THE ROLL IS THE PLANNER'S, tried in order — the same sweep Grasp
        # and Approach make (grasp_geometry.QUARTER_TURNS_RAD). The wrist's
        # current jaw axis comes first (a probe is about the fingertips, and
        # the smallest turn is the likeliest to stay clear of the body), but
        # it is not the only answer: turning a TILTED hand to fingertips-down
        # while keeping its roll can need a J7 past its limit. On d1-2
        # (2026-09-22) every Probe from a real posture was refused that way
        # (ik_fail, 0.25-1.2 rad residual) while a quarter turn plans.
        first = None
        near = None
        for roll in PROBE_ROLLS_RAD:
            plan = _contact_plan(
                self, world, kin, side, d=d, p_standoff=p_tool,
                standoff_label="probe_start", standoff_via=False,
                standoff_arrive=False, travel_m=self.max_travel_m,
                criterion=ContactCriterion(joint_torque_nm=self.contact_nm),
                speed_m_s=PROBE_SPEED_M_S, hold_s=0.0, retract=False,
                notes=notes + ([] if roll == 0.0 else [
                    f"jaws rolled {math.degrees(roll):+.0f} deg about the "
                    f"probe direction (planner choice: the wrist's own roll "
                    f"did not plan)"]),
                keep=r_tool, roll_rad=roll)
            if getattr(plan, "ok", False):
                margin, where = _box_margin(kin, side, plan.steps)
                if margin >= PROBE_BOX_MARGIN_DEG:
                    return plan
                if near is None or margin > near[0]:
                    near = (margin, where, roll)
                continue
            if first is None:
                first = plan
        if near is not None:
            margin, where, roll = near
            return PlanError(
                JOINT_LIMIT,
                f"the {side} arm can probe {self.direction.label()} only in a "
                f"posture next to a joint stop: at best {where} is "
                f"{margin:.1f} deg from its box limit (jaws rolled "
                f"{math.degrees(roll):+.0f} deg), inside the "
                f"{PROBE_BOX_MARGIN_DEG:.0f} deg a repeated probe keeps. "
                f"Move the hand to a posture further from its limits first",
                primitive=self.name(), side=side)
        return first

    def verifier(self, world0: WorldView) -> Verifier:
        side = self._side(world0)
        if side is None:
            return V.Never(self.name(), world0, "no side to verify")
        parts: List[Verifier] = [V.ContactMade(
            self.name(), world0, side, max_travel_m=self.max_travel_m)]
        if self.declare_as:
            parts.append(V.SurfaceMeasured(self.name(), world0, side,
                                           self.declare_as))
        return parts[0] if len(parts) == 1 else V.All(self.name(), world0, parts)


@dataclass(frozen=True)
class Press(Primitive):
    """Push the hand against a target's near face, hold, and back out.

    Stands off ``target`` along ``-direction``, travels onto its near face and
    up to ``depth_m`` past it, stops when a joint's torque has risen
    ``force_nm`` (pressed hard enough), holds ``hold_s``, and returns to the
    standoff along the same line.
    """

    VERB = "press"
    DIRECTION_ARRIVES = True
    side: str = AUTO
    target: str = ""
    direction: Direction = ALIASES["forward"]
    depth_m: float = 0.005
    force_nm: float = 6.0
    hold_s: float = 0.5
    hand: str = "closed"

    def __post_init__(self) -> None:
        _coerce_direction(self)

    @classmethod
    def arg_roles(cls):
        # a button, a panel, a wall: anything the world names can be pressed
        return {"target": ROLE_ANY}

    def resolve_side(self, world: WorldView) -> Optional[str]:
        _item, p, _r, unmet = _locate(world, self.target, "target")
        if unmet or p is None:
            return None if self.side == AUTO else self.side
        return _resolved_side(self.side, world, p)

    def preconditions(self, world: WorldView) -> List[Unmet]:
        unmet = check_arguments(self)
        if unmet:
            return unmet
        _item, p, _r, found = _locate(world, self.target, "target")
        unmet += found
        if p is None or unmet:
            return unmet
        side = self.resolve_side(world)
        unmet += _must_be_free(world, side)
        d, bad = _resolve(self.direction, world, side)
        unmet += bad
        if d is not None:
            unmet += _upward(self.direction, d)
        return unmet

    def _geometry(self, world: WorldView):
        """``(side, d, p_standoff, p_face_tool)`` or the refusal."""
        item, p, _r, unmet = _locate(world, self.target, "target")
        if unmet:
            return None, None, None, None, unmet
        side = _resolved_side(self.side, world, p)
        d, unmet = _resolve(self.direction, world, side)
        if unmet:
            return None, None, None, None, unmet
        d = _unit(d)
        half = item.extent_along(d, world.frames) / 2.0
        face = np.asarray(p, dtype=float) - d * half
        # the fingertips lead, so the TOOL point is PAD.lead_m behind
        at_face = face - d * gg.PAD.lead_m
        return side, d, at_face - d * PRESS_STANDOFF_M, at_face, []

    def plan(self, world: WorldView, kin) -> Any:
        unmet = self.preconditions(world)
        if unmet:
            return self._unmet_error(unmet, self.resolve_side(world) or "")
        side, d, p_standoff, _at_face, _ = self._geometry(world)
        notes = [] if self.side != AUTO else [f"side chosen automatically: {side}"]
        notes.append(
            f"presses {self.target} {self.direction.label()}: from "
            f"{PRESS_STANDOFF_M * 1000:.0f} mm off its near face to at most "
            f"{self.depth_m * 1000:.0f} mm past it at "
            f"{PRESS_SPEED_M_S * 1000:.0f} mm/s, stopping at a "
            f"{self.force_nm:.1f} Nm joint-torque rise, holding "
            f"{self.hold_s:.1f} s, then back to the standoff")
        return _contact_plan(
            self, world, kin, side, d=d, p_standoff=p_standoff,
            standoff_label="press_standoff", standoff_via=True,
            standoff_arrive=True,
            travel_m=PRESS_STANDOFF_M + self.depth_m,
            criterion=ContactCriterion(joint_torque_nm=self.force_nm),
            speed_m_s=PRESS_SPEED_M_S, hold_s=self.hold_s, retract=True,
            notes=notes)

    def verifier(self, world0: WorldView) -> Verifier:
        side, _d, p_standoff, _f, unmet = self._geometry(world0)
        if unmet:
            return V.Never(self.name(), world0,
                           f"press cannot be verified: {unmet[0]}")
        return V.All(self.name(), world0, [
            V.ContactMade(self.name(), world0, side,
                          max_travel_m=PRESS_STANDOFF_M + self.depth_m),
            V.ToolAt(self.name(), world0, side, p_standoff)])


def _declared_name_ok(world: WorldView, name: str) -> List[Unmet]:
    """``declare_as`` may name a new thing or an existing SURFACE, nothing else."""
    if not name:
        return []
    existing = world.find(name)
    if existing is None or isinstance(existing, SurfaceView):
        return []
    return [Unmet(BAD_ARGUMENT,
                  f"declare_as={name!r} names {existing.kind} {name!r}, which "
                  f"is not a surface; a probe publishes a surface",
                  "use a new name, or the name of a surface",
                  {"argument": "declare_as"})]


CONTACT_VERBS: Tuple[type, ...] = (Probe, Press)

__all__ = ["Probe", "Press", "CONTACT_VERBS", "ContactPlane", "fit_plane",
           "record_contacts", "surface_from_contacts", "PROBE_SPEED_M_S",
           "PRESS_SPEED_M_S", "PRESS_STANDOFF_M", "PROBE_HEIGHT_UNCERTAINTY_M"]
