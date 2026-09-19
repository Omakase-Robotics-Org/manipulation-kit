"""ONE definition set, two exports. This is the most valuable thing to share.

Astra-class models bind arguments themselves and want JSON Schema function
definitions. Jev-class models cannot emit free arguments at all and want a
numbered menu of already-bound choices. Those are two RENDERINGS of the same
vocabulary, and the moment each consumer grows its own, the two drift: a verb
renamed for the menu, an argument range widened for the tool schema, and the
same robot now has two different action spaces depending on which model is
driving.

So both come out of the primitive dataclasses, and
``tests/agent/test_schema_drift.py`` asserts they enumerate the same verbs and
the same argument domains. The drift gate is the same pattern the kit already
uses to keep its generated URDFs honest.
"""

from __future__ import annotations

import json
from dataclasses import fields
from typing import Any, Dict, List, Optional, Sequence

from manipulation_kit.primitives import (APPROACHES, GRIPS, NUDGE_FRAMES,
                                         NUDGE_GRID_M, NUDGE_MAX_YAW_RAD,
                                         PRIMITIVES, SIDE_CHOICES)
from manipulation_kit.primitives.approach import APPROACH_DOC
from manipulation_kit.world import WorldView

#: argument name -> the closed set of values it may take. An argument NOT in
#: here is a number, and the only verb with numbers a model chooses freely is
#: ``nudge`` (see the design note's section 2).
ENUMS: Dict[str, Sequence[str]] = {
    "side": SIDE_CHOICES,
    "approach": APPROACHES,
    "grip": GRIPS,
    "frame": NUDGE_FRAMES,
}

#: argument name -> (minimum, maximum) for the numeric ones
RANGES: Dict[str, Sequence[float]] = {
    "standoff_m": (0.02, 0.30),
    "height_m": (0.01, 0.40),
    "clearance_m": (0.0, 0.40),
    "distance_m": (0.01, 0.40),
    "tilt_deg": (15.0, 120.0),
    "dx": (-max(NUDGE_GRID_M), max(NUDGE_GRID_M)),
    "dy": (-max(NUDGE_GRID_M), max(NUDGE_GRID_M)),
    "dz": (-max(NUDGE_GRID_M), max(NUDGE_GRID_M)),
    "dyaw": (-NUDGE_MAX_YAW_RAD, NUDGE_MAX_YAW_RAD),
}

#: arguments that name something in the world
OBJECT_ARGS = ("object", "to", "source", "target")

_ARG_DOC = {
    "object": "the name of a thing in the world, exactly as the world lists it",
    "to": "the container or surface to move it to, by name",
    "source": "the vessel to pour FROM, by name",
    "target": "the vessel to pour INTO, by name",
    "side": "which hand; 'auto' lets the robot pick the near one",
    "approach": "; ".join(f"{k}: {v}" for k, v in APPROACH_DOC.items()),
    "grip": "how hard to hold: the preset owns the stop torque",
    "standoff_m": "how far off the object to wait before closing on it",
    "height_m": "how far up",
    "clearance_m": "how far above the destination",
    "distance_m": "how far back",
    "tilt_deg": "how far to tip the source",
    "dx": f"correction along the frame's x, snapped to "
          f"{[int(g * 1000) for g in NUDGE_GRID_M]} mm",
    "dy": "correction along the frame's y, same grid",
    "dz": "correction along the frame's z, same grid",
    "dyaw": "turn about the approach axis, clamped to +-15 degrees",
    "frame": "'tool' = relative to the hand, 'base' = relative to the robot",
    "policy": "which learned checkpoint runs this verb",
}


def argument_domain(name: str, verb: Optional[str] = None) -> Dict[str, Any]:
    """The single description of one argument, shared by both exports.

    ``verb`` lets a primitive narrow or widen an argument for itself through
    ``Primitive.arg_enums`` — ``go_home`` is the case: it is the only verb for
    which ``side="both"`` means anything, and the drift test is what noticed
    that the exports did not know.
    """
    from manipulation_kit.primitives import BY_VERB  # noqa: PLC0415
    overrides = BY_VERB[verb].arg_enums() if verb in BY_VERB else {}
    if name in overrides:
        return {"kind": "enum", "values": list(overrides[name])}
    if name in ENUMS:
        return {"kind": "enum", "values": list(ENUMS[name])}
    if name in OBJECT_ARGS:
        return {"kind": "name"}
    if name in RANGES:
        low, high = RANGES[name]
        return {"kind": "number", "minimum": float(low), "maximum": float(high)}
    return {"kind": "string"}


def verbs() -> List[type]:
    return list(PRIMITIVES)


def tool_schemas(world: Optional[WorldView] = None) -> List[Dict[str, Any]]:
    """JSON Schema function definitions — Astra / Claude function calling.

    With a ``world``, every object-naming argument is narrowed to an ``enum``
    of what is actually there. That is not a convenience: it is the same rule
    as the offer gate, applied one level up. A model cannot ask for the green
    block if the word is not in its schema.
    """
    names = list(world.names()) if world is not None else None
    out: List[Dict[str, Any]] = []
    for cls in verbs():
        properties: Dict[str, Any] = {}
        required: List[str] = []
        for field in fields(cls):
            if field.name == "verb":
                continue
            domain = argument_domain(field.name, cls.name())
            schema: Dict[str, Any] = {"description": _ARG_DOC.get(field.name, "")}
            if domain["kind"] == "enum":
                schema.update(type="string", enum=domain["values"])
            elif domain["kind"] == "name":
                schema["type"] = "string"
                if names is not None:
                    schema["enum"] = names
                required.append(field.name)
            elif domain["kind"] == "number":
                schema.update(type="number", minimum=domain["minimum"],
                              maximum=domain["maximum"])
            else:
                schema["type"] = "string"
            if field.default is not None and domain["kind"] != "name":
                schema["default"] = field.default
            properties[field.name] = schema
        out.append({
            "name": cls.name(),
            "description": (cls.__doc__ or "").strip().splitlines()[0],
            "parameters": {"type": "object", "properties": properties,
                           "required": required, "additionalProperties": False},
        })
    return out


def choice_menu(world: WorldView, kin, *, cap: int = 20) -> Dict[str, Any]:
    """A Jev-style typed-choice request: already-bound options, each planned.

    The arguments are gone by the time the model sees this — every choice is a
    concrete primitive that has already passed IK and the guard. That is the
    whole reason a typed-choice model can drive a robot at all.
    """
    from offer import candidates_for, offer, why_nothing  # noqa: PLC0415

    offered, refused = offer(candidates_for(world), world, kin, cap=cap)
    return {
        "state": world.to_text(),
        "question": "Which single action moves the task forward?",
        "choices": [{"index": i, "label": item.label,
                     "verb": item.primitive.name(),
                     "arguments": {n: getattr(item.primitive, n)
                                   for n in item.primitive.arguments()}}
                    for i, item in enumerate(offered)],
        "nothing_offered": why_nothing(refused) if not offered else "",
        "refused": [r.to_json() for r in refused],
    }


def verbs_in(schemas: Sequence[Dict[str, Any]]) -> List[str]:
    return sorted(s["name"] for s in schemas)


def domains_in(schemas: Sequence[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    """verb -> argument -> domain, as the drift test compares them."""
    return {s["name"]: {name: argument_domain(name, s["name"])
                        for name in s["parameters"]["properties"]}
            for s in schemas}


def main(argv: Optional[Sequence[str]] = None) -> int:
    print(json.dumps(tool_schemas(), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
