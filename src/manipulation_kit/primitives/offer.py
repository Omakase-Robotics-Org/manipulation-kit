"""The gate: an unreachable candidate never becomes a word in a prompt.

**Why this is in the wheel.** Shu's rule is "primitives are kit capabilities,
not agent internals" (design note 7.1), and *what can this robot do right now*
is a capability question, not a model question. A script, a teleop assist, a
collection macro and a learned pipeline all want it, and none of them wants a
JSON tool schema. The bit that IS model-specific — ranking, menu capping,
provider envelopes, prompts — stays in ``examples/agent/``.

The rule itself is the one that separates a robot agent that works from one
that spends thirty turns asking for a move the guard already refused
(dx-inspect-robots PR #17, where the MotionGuard stopped a -y move after 19 mm
and the model was told "executing move_by over 4 steps", unchanged, thirty
times). Every candidate is planned — same IK, same joint clamp, same collision
guard the teleop stack runs — BEFORE it is offered.

The refusals are returned, not discarded. They are what the caller renders when
NOTHING is offerable: "the left hand cannot reach the red block: guard_reject
at the pregrasp, 19 mm short" is a sentence a model can act on, and an empty
menu with no explanation is not.

NO CAP HERE. ``offer()`` plans what it is given and returns all of it. Capping
is a rendering decision and it belongs where the rendering is: the old
``cap=20`` silently truncated the OFFERED list after planning every candidate,
so the cap bounded neither the computation nor what the caller could see, and
a right-hand recovery action could vanish because the left arm went first.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Sequence, Tuple

from ..world import ContainerView, SurfaceView, WorldView
from .types import Plan, PlanError, Primitive
from .verbs import (Approach, Carry, GoHome, Grasp, Lift, Nudge, NUDGE_GRID_M,
                    Place, Release, Retreat)


@dataclass(frozen=True)
class Offered:
    """A candidate that PLANS. The plan is kept so accepting it costs nothing."""

    primitive: Primitive
    plan: Plan
    label: str

    @property
    def id(self) -> str:
        """A stable identity for this bound action, independent of menu order.

        An index into a list changes meaning the moment the list is rebuilt
        from a newer observation, which is exactly when a model is answering
        with one.
        """
        args = ",".join(f"{k}={v!r}" for k, v in sorted(arguments(
            self.primitive).items()))
        return f"{self.primitive.name()}({args})"

    def to_json(self) -> Dict[str, Any]:
        return {"id": self.id, "verb": self.primitive.name(),
                "label": self.label, "arguments": arguments(self.primitive),
                "plan": self.plan.to_json()}


@dataclass(frozen=True)
class Refused:
    """A candidate that does not, and the typed reason why."""

    primitive: Primitive
    error: PlanError
    label: str

    @property
    def id(self) -> str:
        return Offered(self.primitive, None, self.label).id  # type: ignore[arg-type]

    def to_json(self) -> Dict[str, Any]:
        return {"id": self.id, "verb": self.primitive.name(),
                "label": self.label, "arguments": arguments(self.primitive),
                "refusal": self.error.to_json()}

    def sentence(self) -> str:
        return f"{self.label}: {self.error}"


def arguments(primitive: Primitive) -> Dict[str, Any]:
    return {name: getattr(primitive, name) for name in primitive.arguments()}


def label_for(primitive: Primitive) -> str:
    """A short imperative a person (and a typed-choice model) can read."""
    verb = primitive.name()
    args = arguments(primitive)
    if verb in ("approach", "grasp"):
        return (f"{verb} {args['object']} with the {args['side']} hand, "
                f"{args['approach'].replace('_', ' ')}")
    if verb == "lift":
        return f"lift {args['object']} by {args['height_m'] * 100:.0f} cm"
    if verb in ("carry", "place"):
        return f"{verb} {args['object']} to {args['to']}"
    if verb == "release":
        return f"open the {args['side']} hand"
    if verb == "nudge":
        parts = [f"{v * 1000:+.0f}mm {axis}" for axis, v in
                 (("x", args["dx"]), ("y", args["dy"]), ("z", args["dz"])) if v]
        if args["dyaw"]:
            parts.append(f"{args['dyaw']:+.2f}rad yaw")
        return f"nudge the {args['side']} hand {', '.join(parts)} ({args['frame']} frame)"
    if verb == "retreat":
        return f"back the {args['side']} hand out {args['distance_m'] * 100:.0f} cm"
    if verb == "go_home":
        return "return both arms to HOME"
    if verb == "pour":
        return f"pour {args['source']} into {args['target']}"
    return verb


def offer(candidates: Iterable[Primitive], world: WorldView, kin
          ) -> Tuple[List[Offered], List[Refused]]:
    """Plan every candidate; return what survives and what did not, with reasons."""
    offered: List[Offered] = []
    refused: List[Refused] = []
    for candidate in candidates:
        label = label_for(candidate)
        plan = candidate.plan(world, kin)
        if getattr(plan, "ok", False):
            offered.append(Offered(candidate, plan, label))
        else:
            refused.append(Refused(candidate, plan, label))
    return offered, refused


def check(call: Primitive, world: WorldView, kin) -> Any:
    """Plan ONE bound call. The gate applied to what a model actually said.

    A model may ask for anything; it does not get to skip the guard. This is
    the step dx-inspect-robots PR #17 was missing.
    """
    return call.plan(world, kin)


def why_nothing(refused: Sequence[Refused], limit: int = 6) -> str:
    """The line to render when the offered list is empty. Never say nothing."""
    if not refused:
        return ("Nothing is on offer and nothing was even tried — the world has "
                "no objects the robot can name.")
    lines = ["Nothing is possible right now. What was tried, and why it failed:"]
    lines += [f"  - {r.sentence()}" for r in refused[:limit]]
    if len(refused) > limit:
        lines.append(f"  ... and {len(refused) - limit} more")
    return "\n".join(lines)


# --------------------------------------------------------------------------- #
# candidate generation
# --------------------------------------------------------------------------- #

def candidates_for(world: WorldView, *,
                   approaches: Sequence[str] = ("top_down", "front"),
                   nudge_frame: str = "base",
                   corrections: bool = True) -> List[Primitive]:
    """Everything worth TRYING in this world, before any of it is checked.

    Role-aware: containers and surfaces are destinations, loose objects are
    grasp candidates. It is generous within those roles on purpose —
    generating a candidate is free and :func:`offer` is what decides — and the
    expensive mistake is the other one, not generating the option that would
    have worked.

    A hand whose gripper reports NOTHING gets no candidates at all: the kit
    cannot tell whether it is free, and offering both "grasp" and "release"
    for it would be offering to guess.

    ``nudge_frame`` defaults to ``"base"``. A tool-frame correction is the
    more useful one once a model has the hand's orientation in front of it,
    and it is the harder one to get right from a text description — "the
    hand's own x" means nothing without the axes, which is why they are now in
    ``WorldView.to_text()``. Base frame is what the first example should show
    (R, section 3).
    """
    task: List[Primitive] = []
    fine: List[Primitive] = []
    destinations = [o.name for o in world.objects
                    if isinstance(o, (ContainerView, SurfaceView))]
    graspable = [o.name for o in world.objects
                 if not isinstance(o, (ContainerView, SurfaceView))]
    for side in ("left", "right"):
        gripper = world.gripper(side)
        if gripper is None:
            continue
        held = gripper.held_object if gripper.holding else None
        if not gripper.holding:
            for name in graspable:
                for how in approaches:
                    task.append(Approach(object=name, side=side, approach=how))
                    task.append(Grasp(object=name, side=side, approach=how))
        elif held:
            task.append(Lift(object=held, side=side))
            for name in destinations:
                task.append(Carry(object=held, to=name, side=side))
                task.append(Place(object=held, to=name, side=side))
            task.append(Release(side=side))
        if not corrections:
            continue
        for axis in ("dx", "dy", "dz"):
            for step in NUDGE_GRID_M:
                for sign in (1.0, -1.0):
                    fine.append(Nudge(side=side, frame=nudge_frame,
                                      **{axis: sign * step}))
        # the yaw corrections the menu never had (R, section 3)
        for dyaw in (0.15, -0.15):
            fine.append(Nudge(side=side, frame=nudge_frame, dyaw=dyaw))
        fine.append(Retreat(side=side))
    if corrections:
        fine.append(GoHome())
    # Task verbs first, corrections after: a caller that trims the tail has
    # trimmed the fine adjustments, not the grasp.
    return task + fine
