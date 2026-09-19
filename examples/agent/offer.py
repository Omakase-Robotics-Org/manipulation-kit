"""The gate: an unreachable candidate never becomes a word in the prompt.

This is the one rule that separates a robot agent that works from one that
spends thirty turns asking for a move the guard already refused
(dx-inspect-robots PR #17, where the MotionGuard stopped a -y move after 19 mm
and the model was told "executing move_by over 4 steps", unchanged, thirty
times). Every candidate is planned — same IK, same joint clamp, same collision
guard the teleop stack runs — BEFORE it is offered.

The refusals are returned, not discarded. They are what the caller renders when
NOTHING is offerable: "the left hand cannot reach the red block: guard_reject
at the pregrasp, 19 mm short" is a sentence a model can act on, and an empty
menu with no explanation is not.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from manipulation_kit.primitives import (Approach, Carry, GoHome, Grasp, Lift,
                                         Nudge, Place, Primitive, Release,
                                         Retreat)
from manipulation_kit.world import ContainerView, SurfaceView, WorldView

#: A readable menu is about twenty labels. Jev itself allows up to 255 choices,
#: but a model choosing among 200 near-identical options is not choosing.
DEFAULT_CAP = 20


@dataclass(frozen=True)
class Offered:
    """A candidate that PLANS. The plan is kept so accepting it costs nothing."""

    primitive: Primitive
    plan: Any
    label: str

    def to_json(self) -> Dict[str, Any]:
        return {"verb": self.primitive.name(), "label": self.label,
                "arguments": _arguments(self.primitive),
                "plan": self.plan.to_json()}


@dataclass(frozen=True)
class Refused:
    """A candidate that does not, and the typed reason why."""

    primitive: Primitive
    error: Any
    label: str

    def to_json(self) -> Dict[str, Any]:
        return {"verb": self.primitive.name(), "label": self.label,
                "arguments": _arguments(self.primitive),
                "refusal": self.error.to_json()}

    def sentence(self) -> str:
        return f"{self.label}: {self.error}"


def _arguments(primitive: Primitive) -> Dict[str, Any]:
    return {name: getattr(primitive, name) for name in primitive.arguments()}


def label_for(primitive: Primitive) -> str:
    """A short imperative a person (and a typed-choice model) can read."""
    verb = primitive.name()
    args = _arguments(primitive)
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


def offer(candidates: Iterable[Primitive], world: WorldView, kin, *,
          cap: int = DEFAULT_CAP) -> Tuple[List[Offered], List[Refused]]:
    """Plan every candidate; return what survives and what did not, with reasons.

    ``cap`` bounds the OFFERED list only. Every refusal is kept, because the
    caller needs all of them the moment the offered list comes back empty.
    """
    offered: List[Offered] = []
    refused: List[Refused] = []
    for candidate in candidates:
        label = label_for(candidate)
        plan = candidate.plan(world, kin)
        if getattr(plan, "ok", False):
            if len(offered) < cap:
                offered.append(Offered(candidate, plan, label))
        else:
            refused.append(Refused(candidate, plan, label))
    return offered, refused


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
                   nudge_frame: str = "tool") -> List[Primitive]:
    """Everything worth TRYING in this world, before any of it is checked.

    Generous on purpose: generating a candidate is free and
    :func:`offer` is what decides. The expensive mistake is the other one —
    not generating the option that would have worked.
    """
    task: List[Primitive] = []
    corrections: List[Primitive] = []
    destinations = [o.name for o in world.objects
                    if isinstance(o, (ContainerView, SurfaceView))]
    holders = {side: g.held_object for side, g in world.grippers.items()
               if g.holding}
    graspable = [o.name for o in world.objects
                 if not isinstance(o, (ContainerView, SurfaceView))]
    for side in ("left", "right"):
        held = holders.get(side)
        if held is None:
            for name in graspable:
                for how in approaches:
                    task.append(Approach(object=name, side=side, approach=how))
                    task.append(Grasp(object=name, side=side, approach=how))
        else:
            task.append(Lift(object=held, side=side))
            for name in destinations:
                task.append(Carry(object=held, to=name, side=side))
                task.append(Place(object=held, to=name, side=side))
            task.append(Release(side=side))
        for axis in ("dx", "dy", "dz"):
            for step in (0.010, 0.030, 0.050):
                for sign in (1.0, -1.0):
                    corrections.append(Nudge(side=side, frame=nudge_frame,
                                             **{axis: sign * step}))
        corrections.append(Retreat(side=side))
    corrections.append(GoHome())
    # Task verbs first, corrections after: the cap trims the TAIL, and a menu
    # that spent its twenty slots on nudges has hidden the grasp.
    return task + corrections


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Print the offer for a small hard-coded scene. ``python offer.py``."""
    from scene import demo_scene  # noqa: PLC0415 - sibling example module

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--cap", type=int, default=DEFAULT_CAP)
    args = parser.parse_args(argv)

    world, kin = demo_scene()
    offered, refused = offer(candidates_for(world), world, kin, cap=args.cap)
    print(world.to_text())
    print(f"\n{len(offered)} offered of {len(offered) + len(refused)} tried:")
    for item in offered:
        print(f"  - {item.label}  [{len(item.plan.joint_steps())} joint steps]")
    print("\nrefused (first 8):")
    for item in refused[:8]:
        print(f"  - {item.sentence()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
