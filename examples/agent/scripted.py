"""A scripted stand-in for a function-calling model: no key, no network.

``astra_loop.py --dry-run`` (or no ``--model``) plays this, so the loop, the
operator policy, the gate and the verifiers are runnable offline. It plays a
correct pick-and-place — approach, grasp, lift, carry, place — and answers a
wrist look by asking again (it cannot see). NOT A RESULT: a stub that always
does the right thing proves the plumbing, never the model.

ONE DELIBERATE LIE. It claims "done" at the carry, one verb early, so the
trace's claimed-vs-measured column has something in it: the run prints
``CLAIMED DONE, NOT MEASURED, at turn 4`` and carries on to the place. That
line is the demonstration that the loop ends on a measurement, not on what
the model says.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional


class ScriptedModel:
    """``model(messages, tools) -> {name, arguments, claimed, call_id}``."""

    def __init__(self, obj: str = "red_block", to: str = "box",
                 declare: Optional[List[Dict[str, Any]]] = None):
        #: SCRIPTED FICTION: with an empty scene it declares the two things it
        #: was handed (:func:`two_things_on`). A real model reads the photo.
        self.declare = declare
        side = {"side": "left"}
        self.script: List[Dict[str, Any]] = [
            {"name": "approach", "arguments": dict(object=obj, direction="down",
                                                   **side)},
            {"name": "grasp", "arguments": dict(object=obj, direction="down",
                                                **side)},
            # a shorter hop from a perceived 0.17 m table: 0.10 m of lift from
            # there leaves the arm's envelope — a reach fact, not the loop's
            {"name": "lift", "arguments": dict(object=obj, height_m=0.05
                                               if declare else 0.1, **side)},
            {"name": "carry", "arguments": dict(object=obj, to=to, **side)},
            {"name": "place", "arguments": dict(object=obj, to=to, **side)},
        ]
        self.turn = 0

    def use_side(self, side: str) -> None:
        """Play the script with the hand the task planner chose."""
        for call in self.script:
            call["arguments"]["side"] = side

    def __call__(self, messages, tools) -> Dict[str, Any]:
        if self.declare is not None and "declare_scene" in {t["name"] for t in tools}:
            payload, self.declare = self.declare, None
            return {"name": "declare_scene", "claimed": "",
                    "call_id": "scripted-declare",
                    "arguments": {"objects": payload}}
        last = next((str(m["content"]) for m in reversed(messages)
                     if str(m.get("content", "")).startswith("[result of")), "")
        if "WRIST LOOK" in last and "call grasp again" in last:
            self.turn -= 1          # the photo "agrees": ask again
        if self.turn >= len(self.script):
            return {"name": None, "arguments": {}, "claimed": "done"}
        call = self.script[self.turn]
        self.turn += 1
        # The deliberate over-claim, at the CARRY.
        claimed = "done" if self.turn == len(self.script) - 1 else ""
        return dict(call, claimed=claimed, call_id=f"scripted-{self.turn}")


def two_things_on(world, *, obj: str, destination: str
                  ) -> Optional[List[Dict[str, Any]]]:
    """Two objects on the widest surface in ``world``, for the STUB only: a
    perceived table with no things, and a stand-in that cannot see. Not
    measurements of anything; the trace says ``scripted-declare``."""
    from manipulation_kit.world import SurfaceView  # noqa: PLC0415
    from scene import BLOCK_P, BOX_P  # noqa: PLC0415
    surfaces = [o for o in world.objects if isinstance(o, SurfaceView)]
    if not surfaces or any(o.name in (obj, destination) for o in world.objects):
        return None
    table = max(surfaces, key=lambda s: float(s.size[0]) * float(s.size[1]))
    top = float(table.p[2]) + float(table.size[2]) / 2.0
    return [{"name": obj, "kind": "object",
             "p": [BLOCK_P[0], BLOCK_P[1], top + 0.025],
             "size": [0.045, 0.02, 0.05], "confidence": 0.3},
            {"name": destination, "kind": "container",
             "p": [BOX_P[0], BOX_P[1], top + 0.03], "size": [0.12, 0.12, 0.06],
             "interior": [0.10, 0.10, 0.05], "confidence": 0.3}]
