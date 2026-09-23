"""Which hand can do the WHOLE task? Planned before anything moves.

**Why this is in the wheel.** It is task feasibility, not provider behaviour:
a script, a teleop assist and a learned pipeline all need to avoid picking
something up with the hand that cannot deliver it, and none of them is a
model. F10 measured what skipping it costs.

The gate in :mod:`.offer` asks "can this one verb be done right now?". That is
the right question for a turn and the wrong one for a task: a block on the
robot's right is nearest the right hand, the right hand grasps it, and three
verbs later the bin it has to go in turns out to be on the far left and out of
that arm's reach. Measured on ``blocks-eval`` 2026-09-19: three trials in a row
grasped and lifted cleanly and then refused ``Carry`` with ``ik_fail`` at
``over_destination`` — two of them because the block had been picked up by the
arm that could not deliver it.

So the task planner plans the chain — Approach, Grasp, Lift, Carry, Place —
for BOTH arms, with the kit's own pure ``plan()``, and picks the arm whose
whole chain plans. Nothing moves; ``plan()`` is a value and the kinematic
mirror is restored after every call.

**The hypothetical world is the load-bearing part, and it is stated, not
hidden.** ``Carry`` cannot be planned against the world as it is now, because
right now nothing is held: its preconditions say so, correctly. So each link
of the chain is planned against a world rolled FORWARD by the previous link —
the arm posed at the last joint step the plan produced, the gripper marked
holding, the object carried along with the tool. Every rolled-forward world is
stamped and revisioned so it can never be mistaken for an observation, and
``ChainPlan.hypothetical`` says so out loud. That is a prediction, not a
measurement, and the difference matters:

* what it gets right is REACH, which is the question being asked — the joint
  vectors are the kit's own solved ones, against the real URDF and the real
  guard;
* what it cannot know is whether the grasp will actually hold. That is
  measured, one verb at a time, by the verifiers, and a chain that plans is
  never evidence that a trial succeeded.

It answers one question — *which hand?* — and answers it before the first move.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np

from ..world import ArmView, GripperView, WorldView
from ..world.attach import GraspTransform, grasp_transform, with_attached
from ..world.direction import ALIASES, Direction
from .orientation import tool_from_link7
from .types import JointStep, Plan, PlanError, Primitive
from .verbs import Approach, Carry, Grasp, Lift, Place

SIDES: Tuple[str, ...] = ("left", "right")


# --------------------------------------------------------------------------- #
# the result
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class ChainLink:
    """One verb of the chain, and what the kit said about it."""

    primitive: Primitive
    result: Union[Plan, PlanError]

    @property
    def verb(self) -> str:
        return self.primitive.name()

    @property
    def ok(self) -> bool:
        return bool(getattr(self.result, "ok", False))

    def to_json(self) -> Dict[str, Any]:
        return {"verb": self.verb, "ok": self.ok,
                **({"plan": self.result.to_json()} if self.ok
                   else {"refusal": self.result.to_json()})}


@dataclass(frozen=True)
class ChainPlan:
    """Every link that was planned for one arm, in order, with its verdict."""

    side: str
    links: Tuple[ChainLink, ...]
    #: every link after the first was planned against a world this module
    #: PREDICTED, not one anybody observed. Published so a caller cannot
    #: mistake a planned chain for a measured outcome.
    hypothetical: bool = True

    @property
    def ok(self) -> bool:
        return bool(self.links) and all(link.ok for link in self.links)

    @property
    def planned(self) -> int:
        """How many links planned before the first refusal."""
        for index, link in enumerate(self.links):
            if not link.ok:
                return index
        return len(self.links)

    @property
    def broke_at(self) -> Optional[ChainLink]:
        for link in self.links:
            if not link.ok:
                return link
        return None

    def sentence(self) -> str:
        broke = self.broke_at
        if broke is None:
            return f"the {self.side} arm can plan the whole chain"
        return (f"the {self.side} arm plans {self.planned} of "
                f"{len(self.links)} verbs and then {broke.result}")

    def to_json(self) -> Dict[str, Any]:
        return {"side": self.side, "ok": self.ok, "planned": self.planned,
                "hypothetical": bool(self.hypothetical),
                "note": ("reach only: every link after the first was planned "
                         "against a predicted world, and a chain that plans "
                         "is not evidence that a trial succeeded"),
                "links": [link.to_json() for link in self.links]}


@dataclass(frozen=True)
class SideChoice:
    """Which hand, why, and what was tried for the other one."""

    side: str
    reason: str
    reachable: bool
    chains: Dict[str, ChainPlan]

    def to_json(self) -> Dict[str, Any]:
        return {"side": self.side, "reason": self.reason,
                "reachable": self.reachable,
                "chains": {s: c.to_json() for s, c in self.chains.items()}}


# --------------------------------------------------------------------------- #
# rolling the world forward
# --------------------------------------------------------------------------- #

def _tool_of(kin, side: str, q) -> Tuple[np.ndarray, Any]:
    """The tool pose for a joint vector, with the mirror put back afterwards."""
    saved = np.array(kin.joints(side), dtype=float)
    try:
        kin.set_joints(side, np.asarray(q, dtype=float))
        return tool_from_link7(*kin.ee_pose(side))
    finally:
        kin.set_joints(side, saved)


def _last_q(plan, side: str, fallback) -> np.ndarray:
    steps = [s for s in plan.steps
             if isinstance(s, JointStep) and s.side == side]
    return np.array(steps[-1].q if steps else fallback, dtype=float)


def _posed(world: WorldView, kin, side: str, q) -> WorldView:
    """The same world with one arm moved to ``q`` — measured tool pose and all."""
    p_tool, r_tool = _tool_of(kin, side, q)
    arms = dict(world.arms)
    was = arms.get(side)
    arms[side] = ArmView(side, joints=np.asarray(q, dtype=float),
                         tool_p=p_tool, tool_r=r_tool,
                         mode=was.mode if was is not None else "unknown",
                         stationary=True)
    return world.with_(arms=arms, revision=int(world.revision) + 1)


def _grasped(world: WorldView, side: str, name: str
             ) -> Tuple[WorldView, Optional[GraspTransform]]:
    """The same world with ``side`` holding ``name``, and the grasp recorded.

    The closedness is the COMMAND (1.0), not a prediction of where the jaws
    will stop — nothing in a plan reads it, and inventing a stall position
    would be a measurement this function is not entitled to make.

    The object then RIDES THE TOOL through
    :func:`manipulation_kit.world.attach.with_attached` — the same rule a live
    producer uses — so the predicted world and the observed one cannot disagree
    about where a held object goes. ``None`` for the grasp when the object or
    the tool pose is missing (the object is then left where it was).
    """
    item = world.find(name)
    grippers = dict(world.grippers)
    was = grippers.get(side)
    grippers[side] = GripperView(
        side, 1.0, holding=True, held_object=name,
        jaw_gap_m=None if item is None else item.min_horizontal_extent(),
        grip=was.grip if was is not None else "firm", jaw_stalled=True)
    world = world.with_(grippers=grippers, revision=int(world.revision) + 1)
    try:
        grasp = grasp_transform(world, side=side, name=name)
    except LookupError:
        return world, None
    return with_attached(world, side=side, name=name, grasp=grasp), grasp


def _moved(world: WorldView, grasp: GraspTransform) -> WorldView:
    """The same world with the held object carried to where the tool now is.

    A rigid grasp, in a value: the tool's new pose composed with the transform
    recorded at the stroke (:func:`~manipulation_kit.world.attach.with_attached`),
    rotation included. The old rule added the tool's TRANSLATION to the
    object's position and ignored the wrist's turn — a second grasp rule,
    disagreeing with the first one about every carry that rotates the hand.
    """
    try:
        return with_attached(world, side=grasp.side, name=grasp.name,
                             grasp=grasp)
    except LookupError:
        return world


# --------------------------------------------------------------------------- #
# the chain
# --------------------------------------------------------------------------- #

def plan_chain(world: WorldView, kin, *, obj: str, destination: str, side: str,
               direction: Direction = ALIASES["down"], lift_m: float = 0.12,
               contact: str = "pad",
               grip: str = "firm") -> ChainPlan:
    """Approach -> Grasp -> Lift -> Carry -> Place, for ONE arm. Nothing moves.

    ``direction`` is how the hand travels onto the object (any
    :class:`~manipulation_kit.world.Direction` or alias) and ``contact`` where
    on the hand it is taken (``"pad"`` / ``"tip"``). There is no roll
    argument: ``Approach`` and ``Grasp`` each try
    :func:`.grasp_geometry.roll_candidates` — THE one sweep — and ``Grasp``
    continues with the roll its ``Approach`` stood at.

    Stops at the first refusal: a chain whose ``Grasp`` is refused has no
    posture to plan a ``Carry`` from, and planning one against a world that
    never happened would be the fiction this module exists to avoid.
    """
    links: List[ChainLink] = []
    state = world
    grasp: Optional[GraspTransform] = None
    for primitive in (Approach(object=obj, side=side, direction=direction,
                               contact=contact),
                      Grasp(object=obj, side=side, direction=direction,
                            grip=grip, contact=contact),
                      Lift(object=obj, side=side, height_m=lift_m),
                      Carry(object=obj, to=destination, side=side),
                      Place(object=obj, to=destination, side=side)):
        arm = state.arm(side)
        q_before = (np.array(arm.joints, dtype=float) if arm is not None
                    else np.array(kin.joints(side), dtype=float))
        result = primitive.plan(state, kin)
        links.append(ChainLink(primitive, result))
        if not getattr(result, "ok", False):
            break
        q_after = _last_q(result, side, q_before)
        state = _posed(state, kin, side, q_after)
        if grasp is not None:
            state = _moved(state, grasp)
        if primitive.name() == "grasp":
            state, grasp = _grasped(state, side, obj)
    return ChainPlan(side, tuple(links))


def _near_hand(world: WorldView, obj: str) -> str:
    """The hand the block is nearest, by the sign of its y. +y is robot-left.

    This was the WHOLE rule until 2026-09-19. It is kept as the tie-break,
    because when both arms can deliver, the near one is the shorter reach.
    """
    item = world.find(obj)
    if item is None:
        return "right"
    try:
        p = item.pose_in_base(world.frames)[0]
    except LookupError:
        p = np.asarray(item.p, dtype=float)
    return "right" if float(p[1]) < 0.0 else "left"


def choose_side(world: WorldView, kin, *, obj: str, destination: str,
                direction: Direction = ALIASES["down"], lift_m: float = 0.12,
                contact: str = "pad",
                sides: Sequence[str] = SIDES) -> SideChoice:
    """The hand that can plan the WHOLE task, not the hand nearest the block.

    Order of preference, and it is fixed so the choice is a function of the
    world alone:

    1. an arm whose whole chain plans — the near hand first when both do;
    2. otherwise the arm that plans the most links, near hand on a tie;
    3. and the caller is told which case it got (``reachable``), because
       "no arm can deliver this" is a finding, not a default.
    """
    chains = {side: plan_chain(world, kin, obj=obj, destination=destination,
                               side=side, direction=direction, lift_m=lift_m,
                               contact=contact)
              for side in sides}
    near = _near_hand(world, obj)
    ordered = sorted(chains.values(),
                     key=lambda c: (not c.ok, -c.planned, c.side != near))
    best = ordered[0]
    other = chains.get("left" if best.side == "right" else "right")
    if best.ok:
        if other is not None and other.ok:
            reason = (f"both arms plan the whole chain; took the near hand "
                      f"({near}) by the block's own y")
        else:
            reason = (f"only the {best.side} arm plans the whole chain — "
                      + (other.sentence() if other is not None else "no other arm"))
        return SideChoice(best.side, reason, True, chains)
    reason = ("no arm plans the whole chain; took the one that gets furthest — "
              + "; ".join(chains[s].sentence() for s in sorted(chains)))
    return SideChoice(best.side, reason, False, chains)


# --------------------------------------------------------------------------- #
# handover: the same chain, over two sides
# --------------------------------------------------------------------------- #

#: Where a handover may MEET, tried in order, largest margin first: the HELD
#: OBJECT's centre in the base frame, with ``y`` stated for a LEFT-hand giver
#: (mirrored for a right-hand one — the meeting leans to the giver's side of
#: the midline, where the giving arm has its reach and the receiving arm
#: still has its own). The same idiom as ``verbs.CARRY_CLEARANCE_LADDER_M``: a
#: fixed ordered list, the first rung whose WHOLE two-arm chain plans wins, so
#: the choice is a function of the world alone. The rungs are where both
#: arms' reachable sets overlap on this URDF. RE-SURVEYED 2026-09-22 under
#: the coupled wrist-roll limit (``arms.coupled_limits``; the first survey,
#: x 0.35-0.45 / y 0-0.10 / z 0.20-0.35, was box-only and every one of its
#: rungs needed a J7 the d1-2 wrist does not have): a 40 mm cube held top-down
#: by the left hand after Approach/Grasp/Lift at (0.40, 0.15), the right hand
#: travelling ``left`` onto it, grid x 0.30-0.50, y -0.10..0.15, z 0.20-0.40
#: in 5 cm steps — these five of 150 plan, all near the chest (x 0.30-0.35)
#: and low (z 0.20-0.25). A point is a candidate, never a promise; a world no
#: rung reaches is refused as ``unreachable_handover``.
HANDOVER_MEETING_POINTS_M: Tuple[Tuple[float, float, float], ...] = (
    (0.30, 0.05, 0.20), (0.30, 0.10, 0.20), (0.30, 0.00, 0.25),
    (0.35, 0.15, 0.20), (0.30, 0.15, 0.25))
#: the held object's underside must be this far above anything under the
#: meeting point [m] — two hands meet in free air, not over a table edge
HANDOVER_FLOOR_M = 0.10


@dataclass(frozen=True)
class HandoverChain:
    """One meeting point, and every link planned for it, in order."""

    meeting: Tuple[float, float, float]
    links: Tuple[ChainLink, ...]
    #: the link each plan belongs to: ``meet`` (giver), ``approach``,
    #: ``grasp`` (receiver), ``release``, ``retreat`` (giver)
    labels: Tuple[str, ...]
    #: the rung was not tried because something is under it (a sentence)
    skipped: str = ""

    @property
    def ok(self) -> bool:
        return (not self.skipped and len(self.links) == 5
                and all(link.ok for link in self.links))

    @property
    def broke_at(self) -> Optional[str]:
        for label, link in zip(self.labels, self.links):
            if not link.ok:
                return label
        return None

    def sentence(self) -> str:
        where = "({:.2f}, {:+.2f}, {:.2f}) m".format(*self.meeting)
        if self.skipped:
            return f"{where}: {self.skipped}"
        broke = self.broke_at
        if broke is None:
            return f"{where}: plans"
        link = self.links[self.labels.index(broke)]
        return f"{where}: {broke} refused ({link.result.reason})"


def _meeting_for(point: Tuple[float, float, float], giver: str) -> np.ndarray:
    x, y, z = (float(c) for c in point)
    return np.array([x, y if giver == "left" else -y, z])


def _top_under(world: WorldView, p, exclude: str) -> Optional[float]:
    """The highest top of anything whose footprint is under ``p`` (base)."""
    best = None
    for item in world.objects:
        if item.name == exclude:
            continue
        try:
            c, r = item.pose_in_base(world.frames)
            local = r.inv().apply(np.asarray(p, dtype=float) - c)
            half = np.asarray(item.size, dtype=float).reshape(3) / 2.0
            if abs(local[0]) > half[0] or abs(local[1]) > half[1]:
                continue
            top = item.top_face_z(world.frames)
        except LookupError:
            continue
        best = top if best is None else max(best, top)
    return best


def _opened(world: WorldView, side: str) -> WorldView:
    """The same world with ``side``'s jaws open and empty — the COMMAND, like
    :func:`_grasped`'s closedness, not a measurement."""
    grippers = dict(world.grippers)
    was = grippers.get(side)
    grippers[side] = GripperView(
        side, 0.0, holding=False, held_object=None, jaw_gap_m=None,
        grip=was.grip if was is not None else "soft", jaw_stalled=False,
        open_gap_m=getattr(was, "open_gap_m", None))
    return world.with_(grippers=grippers, revision=int(world.revision) + 1)


def handover_chain(primitive: Primitive, world: WorldView, kin, *, obj: str,
                   giver: str, receiver: str, direction: Direction,
                   retreat_m: float, grip: str,
                   meeting: Tuple[float, float, float]) -> HandoverChain:
    """Meet -> receiver Approach -> receiver Grasp -> giver Release -> giver
    Retreat, at ONE meeting point. Nothing moves.

    ``plan_chain`` over two sides: every link is the kit's own verb planned
    against the world rolled forward by the previous one (the giver posed at
    the meeting with the object riding its tool, then the receiver posed and
    holding with the object re-attached to IT, then the giver open). The
    first link is the giver's transit, planned with ``primitive`` (the
    ``Handover``) so its scene gate leaves out the object the hands share.
    """
    from .verbs import Approach, Grasp, Release, Retreat, _plan_for  # noqa: PLC0415
    from .types import Waypoint  # noqa: PLC0415
    item = world.find(obj)
    arm = world.arm(giver)
    p_obj = item.pose_in_base(world.frames)[0]
    target = _meeting_for(meeting, giver)
    under = _top_under(world, target, obj)
    if under is not None:
        underside = float(target[2]) - item.vertical_extent(world.frames) / 2.0
        if underside - under < HANDOVER_FLOOR_M - 1e-9:
            return HandoverChain(tuple(float(c) for c in target), (), (),
                                 skipped=(f"only {(underside - under) * 1000:.0f}"
                                          f" mm above what is under it"))
    offset = np.asarray(arm.tool_p, dtype=float) - np.asarray(p_obj)
    links: List[ChainLink] = []
    labels: List[str] = []
    state = world
    grasp: Optional[GraspTransform]
    try:
        grasp = grasp_transform(world, side=giver, name=obj)
    except LookupError:
        grasp = None

    def record(label: str, prim: Primitive, result) -> bool:
        links.append(ChainLink(prim, result))
        labels.append(label)
        return bool(getattr(result, "ok", False))

    meet = _plan_for(primitive, state, kin, giver,
                     [Waypoint("meeting", target + offset, arm.tool_r,
                               allow_via=True, arrive=True)])
    if record("meet", primitive, meet):
        state = _posed(state, kin, giver,
                       _last_q(meet, giver, arm.joints))
        if grasp is not None:
            state = _moved(state, grasp)
        for label, prim in (
                ("approach", Approach(object=obj, side=receiver,
                                      direction=direction)),
                ("grasp", Grasp(object=obj, side=receiver,
                                direction=direction, grip=grip))):
            result = prim.plan(state, kin)
            if not record(label, prim, result):
                break
            q0 = state.arm(receiver).joints
            state = _posed(state, kin, receiver,
                           _last_q(result, receiver, q0))
            if label == "grasp":
                # the object now rides the RECEIVER: its transform is taken
                # at this instant, with the giver still closed on it
                state, grasp = _grasped(state, receiver, obj)
        else:
            release = Release(side=giver)
            if record("release", release, release.plan(state, kin)):
                state = _opened(state, giver)
                retreat = Retreat(side=giver, distance_m=retreat_m)
                record("retreat", retreat, retreat.plan(state, kin))
    return HandoverChain(tuple(float(c) for c in target), tuple(links),
                         tuple(labels))


def plan_handover(primitive: Primitive, world: WorldView, kin, *, obj: str,
                  giver: str, receiver: str, direction: Direction,
                  retreat_m: float, grip: str,
                  meetings: Sequence[Tuple[float, float, float]] = ()
                  ) -> Tuple[Optional[HandoverChain], List[HandoverChain]]:
    """The first meeting point of the ladder whose whole chain plans, and
    every chain that was tried (for the refusal). ``meetings`` defaults to
    :data:`HANDOVER_MEETING_POINTS_M`, read at call time."""
    tried: List[HandoverChain] = []
    for meeting in (tuple(meetings) or HANDOVER_MEETING_POINTS_M):
        chain = handover_chain(primitive, world, kin, obj=obj, giver=giver,
                               receiver=receiver, direction=direction,
                               retreat_m=retreat_m, grip=grip, meeting=meeting)
        tried.append(chain)
        if chain.ok:
            return chain, tried
    return None, tried
