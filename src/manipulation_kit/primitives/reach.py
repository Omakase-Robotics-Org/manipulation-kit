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
from .approach import tool_from_link7
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


def _grasped(world: WorldView, side: str, name: str) -> WorldView:
    """The same world with ``side`` holding ``name``.

    The closedness is the COMMAND (1.0), not a prediction of where the jaws
    will stop — nothing in a plan reads it, and inventing a stall position
    would be a measurement this function is not entitled to make.
    """
    item = world.find(name)
    grippers = dict(world.grippers)
    was = grippers.get(side)
    grippers[side] = GripperView(
        side, 1.0, holding=True, held_object=name,
        jaw_gap_m=None if item is None else item.min_horizontal_extent(),
        grip=was.grip if was is not None else "firm", jaw_stalled=True)
    return world.with_(grippers=grippers, revision=int(world.revision) + 1)


def _moved(world: WorldView, name: str, delta) -> WorldView:
    """The same world with one object translated — a rigid grasp, in a value.

    ``delta`` is a BASE-frame displacement and ``o.p`` is in the object's OWN
    frame, so it is rotated into that frame before it is added. Adding one to
    the other directly is the same mistake as R1, one module along: on a
    wagon frame yawed 25 degrees it moved the predicted object sideways.
    An object whose frame will not resolve is left alone rather than moved by
    a number that means nothing.
    """
    import dataclasses
    delta = np.asarray(delta, dtype=float)
    objects = []
    for item in world.objects:
        if item.name != name:
            objects.append(item)
            continue
        try:
            _p, r = item.pose_in_base(world.frames)
            local = r.inv().apply(delta) if item.frame_id != "base" else delta
        except LookupError:
            objects.append(item)
            continue
        objects.append(dataclasses.replace(
            item, p=np.asarray(item.p, dtype=float) + local))
    return world.with_(objects=tuple(objects))


# --------------------------------------------------------------------------- #
# the chain
# --------------------------------------------------------------------------- #

def plan_chain(world: WorldView, kin, *, obj: str, destination: str, side: str,
               approach: str = "top_down", lift_m: float = 0.12,
               grip: str = "firm") -> ChainPlan:
    """Approach -> Grasp -> Lift -> Carry -> Place, for ONE arm. Nothing moves.

    Stops at the first refusal: a chain whose ``Grasp`` is refused has no
    posture to plan a ``Carry`` from, and planning one against a world that
    never happened would be the fiction this module exists to avoid.
    """
    links: List[ChainLink] = []
    state = world
    for primitive in (Approach(object=obj, side=side, approach=approach),
                      Grasp(object=obj, side=side, approach=approach, grip=grip),
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
        held = state.gripper(side)
        carrying = bool(held is not None and held.holding
                        and held.held_object == obj)
        if carrying:
            before = _tool_of(kin, side, q_before)[0]
            after = _tool_of(kin, side, q_after)[0]
            state = _moved(state, obj, after - before)
        state = _posed(state, kin, side, q_after)
        if primitive.name() == "grasp":
            state = _grasped(state, side, obj)
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
                approach: str = "top_down", lift_m: float = 0.12,
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
                               side=side, approach=approach, lift_m=lift_m)
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
