"""ONE definition set, exported for whoever is asking. Dependency-free.

**Why this is in the wheel.** The counter-argument in the old README —
"putting the schema in the package would make every consumer of the kinematics
carry it" — does not survive contact with this file: it imports ``json`` and
``dataclasses`` and nothing else. No provider SDK, no HTTP client, no new
dependency in ``pyproject.toml``. What it exports is a DESCRIPTION of the
kit's own action space, which is the kit's to describe; the drift the old
argument worried about is prevented by the vocabulary living here, not by it
living somewhere a second copy can grow beside it.

What stays in ``examples/agent/``: provider SDKs, authentication, prompts, the
scripted policy, Jev request envelopes, ranking and menu capping. Those are
adapters. This is the thing they adapt.

Two renderings come out of the same table
(:mod:`manipulation_kit.primitives.arguments`):

``tool_schemas(world)``   JSON Schema function definitions, for a model that
                          binds arguments itself
``decode(call, world)``   the inverse — a name plus a dict of arguments back
                          into a checked primitive, or a typed refusal

A menu of already-bound choices is :func:`~manipulation_kit.primitives.offer.offer`
plus a renderer, and the renderer is an example.

ROLE-NARROWED, which the old export was not: it applied EVERY world name to
``object``, ``to``, ``source`` and ``target``, so a table was a grasp
candidate and a loose block was a placement destination. A model that cannot
say "grasp the table" will not.
"""

from __future__ import annotations

import json
from dataclasses import fields
from typing import Any, Dict, List, Optional, Sequence, Union

from ..world import WorldView
from .arguments import ARGUMENTS, ROLE_ANY, argument, check_arguments, names_for
from .types import BAD_ARGUMENT, PlanError, Primitive, Unmet
from .verbs import BY_VERB, PRIMITIVES


def verbs() -> List[type]:
    return list(PRIMITIVES)


def argument_domain(name: str, verb: Optional[str] = None) -> Dict[str, Any]:
    """The single description of one argument, shared by every export."""
    overrides = BY_VERB[verb].arg_enums() if verb in BY_VERB else {}
    return argument(name, verb, overrides).domain()


def _json_schema(spec, names: Optional[Sequence[str]]) -> Dict[str, Any]:
    domain = spec.domain()
    schema: Dict[str, Any] = {"description": spec.doc}
    if domain["kind"] == "enum":
        schema.update(type="string", enum=list(domain["values"]))
    elif domain["kind"] == "name":
        schema["type"] = "string"
        if names is not None:
            schema["enum"] = list(names)
    elif domain["kind"] == "number":
        schema.update(type="number", minimum=domain["minimum"],
                      maximum=domain["maximum"])
        if spec.unit:
            schema["description"] = f"{spec.doc} [{spec.unit}]"
    elif domain["kind"] == "boolean":
        schema["type"] = "boolean"
    else:
        schema["type"] = "string"
    return schema


def tool_schemas(world: Optional[WorldView] = None) -> List[Dict[str, Any]]:
    """JSON Schema function definitions, narrowed by ROLE and by holding state.

    With a ``world``, every object-naming argument is narrowed to an ``enum``
    of the names that can legally fill that role — graspable things for
    ``object``, containers and surfaces for ``to``, vessels for pour. That is
    not a convenience: it is the offer gate applied one level up. A model
    cannot ask for the green block if the word is not in its schema, and it
    cannot ask to put the bin inside the block.

    It is still a LIST OF DESCRIPTIONS, not a checked offer: an unreachable
    argument remains expressible and must be refused after binding, by
    :func:`decode` and then by the plan. A name enum is not IK feasibility.
    """
    out: List[Dict[str, Any]] = []
    for cls in verbs():
        overrides = cls.arg_enums()
        properties: Dict[str, Any] = {}
        required: List[str] = []
        for field in fields(cls):
            spec = argument(field.name, cls.name(), overrides)
            names = (None if world is None or spec.domain()["kind"] != "name"
                     else names_for(world, spec.role))
            properties[field.name] = _json_schema(spec, names)
            if spec.domain()["kind"] == "name":
                required.append(field.name)
            elif field.default is not None:
                properties[field.name]["default"] = field.default
        out.append({
            "name": cls.name(),
            "description": (cls.__doc__ or "").strip().splitlines()[0],
            "parameters": {"type": "object", "properties": properties,
                           "required": required, "additionalProperties": False},
        })
    return out


def decode(name: str, arguments: Dict[str, Any],
           world: Optional[WorldView] = None) -> Union[Primitive, PlanError]:
    """A model's function call back into a primitive, or a typed refusal.

    Every way a call can be malformed ends here as a ``PlanError`` with
    ``reason="bad_argument"`` — an unknown verb, an unknown field, a wrong
    type, a nonfinite number, a value out of range, a name that is not in the
    world or cannot play that role. The old loop caught ``TypeError`` and
    ``ValueError`` around the CONSTRUCTOR only, and planning happened outside
    that handler (R3/R14).
    """
    cls = BY_VERB.get(name)
    if cls is None:
        return PlanError(BAD_ARGUMENT,
                         f"no verb called {name!r}; the verbs are "
                         f"{sorted(BY_VERB)}", primitive=str(name))
    known = {f.name for f in fields(cls)}
    extra = sorted(set(arguments) - known)
    if extra:
        return PlanError(BAD_ARGUMENT,
                         f"{name} takes {sorted(known)}, not {extra}",
                         primitive=name,
                         unmet=tuple(Unmet(BAD_ARGUMENT, f"unknown argument {e!r}")
                                     for e in extra))
    try:
        call = cls(**arguments)
    except (TypeError, ValueError) as exc:
        return PlanError(BAD_ARGUMENT, f"{name}: {exc}", primitive=name)
    unmet = tuple(check_arguments(call))
    if world is not None:
        unmet += tuple(_role_errors(call, world))
    if unmet:
        return PlanError(BAD_ARGUMENT,
                         "; ".join(str(u) for u in unmet),
                         primitive=name, unmet=unmet)
    return call


def _role_errors(call: Primitive, world: WorldView) -> List[Unmet]:
    out: List[Unmet] = []
    for field in fields(call):
        spec = ARGUMENTS.get(field.name)
        if spec is None or spec.kind != "name" or spec.role == ROLE_ANY:
            continue
        value = getattr(call, field.name)
        if not value:
            continue
        allowed = names_for(world, spec.role)
        if value not in allowed:
            out.append(Unmet(BAD_ARGUMENT,
                             f"{field.name}={value!r} cannot play the "
                             f"{spec.role} role here",
                             f"one of: {', '.join(allowed) or 'nothing'}"))
    return out


def domains() -> Dict[str, Dict[str, Any]]:
    """verb -> argument -> domain, straight from the kit's own table.

    The drift gate compares an EXPORT against this, so a renderer that
    invented its own range has something to fail against. The old test
    compared the export with the same function that generated it.
    """
    return {cls.name(): {f.name: argument_domain(f.name, cls.name())
                         for f in fields(cls)}
            for cls in verbs()}


def verbs_in(schemas: Sequence[Dict[str, Any]]) -> List[str]:
    return sorted(s["name"] for s in schemas)


def domains_in(schemas: Sequence[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    """verb -> argument -> domain, read back OUT of a rendered schema.

    Independently of :func:`domains`, on purpose: the two are compared, and a
    comparison whose halves come from the same call proves nothing.
    """
    out: Dict[str, Dict[str, Any]] = {}
    for schema in schemas:
        rendered: Dict[str, Any] = {}
        for name, prop in schema["parameters"]["properties"].items():
            if "enum" in prop and name in ("object", "to", "source", "target"):
                rendered[name] = {"kind": "name",
                                  "role": ARGUMENTS[name].role}
            elif "enum" in prop:
                rendered[name] = {"kind": "enum", "values": list(prop["enum"])}
            elif prop.get("type") == "number":
                rendered[name] = {"kind": "number",
                                  "minimum": prop["minimum"],
                                  "maximum": prop["maximum"],
                                  "unit": ARGUMENTS[name].unit}
            elif prop.get("type") == "boolean":
                rendered[name] = {"kind": "boolean"}
            elif name in ARGUMENTS and ARGUMENTS[name].kind == "name":
                rendered[name] = {"kind": "name", "role": ARGUMENTS[name].role}
            else:
                rendered[name] = {"kind": "string"}
        out[schema["name"]] = rendered
    return out


def main(argv: Optional[Sequence[str]] = None) -> int:
    print(json.dumps(tool_schemas(), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
