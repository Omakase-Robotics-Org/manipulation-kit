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
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

from ..world import WorldView
from ..world.direction import (ALIAS_MEANING, ALIASES, FRAMES, OBJECT_PREFIX,
                               Direction, parse_direction)
from .arguments import ARGUMENTS, ROLE_ANY, argument, check_arguments, names_for
from .types import BAD_ARGUMENT, PlanError, Primitive, Unmet
from .verbs import BY_VERB, PRIMITIVES


#: Arguments a MODEL is not offered — and that :func:`decode` REFUSES from one
#: — because they are deployment configuration or planner choices rather than
#: a choice about the task. ``policy`` names a learned checkpoint: which one is
#: served is a property of the robot in front of you, and a free string
#: invites a model to invent one (R, section 3). It keeps its default and is
#: still bindable from Python. THIS IS THE ONE ALLOWLIST: a consumer does not
#: strip arguments on the way out or pop them on the way in. (The planner's
#: roll about the approach axis is not here because it is not a field at all
#: since 0.16.0 step 3: ``grasp_geometry.roll_candidates`` is the one sweep
#: and the plan's notes say which roll was used. Shown to a model as
#: ``jaw_turn_deg``, d1-2 run8, it picked the IK-infeasible turn on its own.)
NOT_MODEL_BINDABLE: Tuple[str, ...] = ("policy",)

#: What a verb cannot promise, added to its description so the limitation
#: reaches capability discovery rather than only a docstring.
CAVEATS: Dict[str, str] = {
    "pour": ("Runnable ONLY where a learned-policy executor is registered; "
             "without one it refuses with learned_policy_required. Its "
             "verifier confirms the TILT and then returns UNKNOWN: nothing on "
             "this robot weighs the source or reads the target's level, and "
             "target-relative alignment is not measured."),
}


def verbs() -> List[type]:
    return list(PRIMITIVES)


def argument_domain(name: str, verb: Optional[str] = None) -> Dict[str, Any]:
    """The single description of one argument, shared by every export."""
    cls = BY_VERB.get(verb)
    overrides = cls.arg_enums() if cls is not None else {}
    roles = cls.arg_roles() if cls is not None else {}
    return argument(name, verb, overrides, roles).domain()


def _json_schema(spec, names: Optional[Sequence[str]]) -> Dict[str, Any]:
    domain = spec.domain()
    schema: Dict[str, Any] = {"description": spec.doc}
    if domain["kind"] == "enum":
        schema.update(type="string", enum=list(domain["values"]))
    elif domain["kind"] == "name":
        schema["type"] = "string"
        if names is not None:
            schema["enum"] = list(names)
            if not schema["enum"]:
                # An EMPTY enum is an unsatisfiable grammar: the Responses API
                # returns status=incomplete with zero output tokens and no error
                # (d1-2 2026-09-22, a scene with nothing declared yet). Leave the
                # name free and say why; decode() still rejects an unknown name.
                del schema["enum"]
                schema["description"] = (schema.get("description", "")
                                         + " (nothing of this kind is known yet; declare_scene first)")
    elif domain["kind"] == "number":
        schema.update(type="number", minimum=domain["minimum"],
                      maximum=domain["maximum"])
        if spec.unit:
            schema["description"] = f"{spec.doc} [{spec.unit}]"
    elif domain["kind"] == "boolean":
        schema["type"] = "boolean"
    elif domain["kind"] == "direction":
        schema["oneOf"] = [
            {"type": "string", "enum": list(domain["aliases"]),
             "description": "a named direction"},
            {"type": "object",
             "description": "a free direction: a vector in a named frame",
             "properties": {
                 "axis": {"type": "array", "items": {"type": "number"},
                          "minItems": 3, "maxItems": 3},
                 "frame": _frame_schema(names)},
             "required": ["axis", "frame"], "additionalProperties": False}]
    else:
        schema["type"] = "string"
    return schema


def _frame_schema(object_names: Optional[Sequence[str]]) -> Dict[str, Any]:
    """The frames a free direction may name: base, tool, or an object's own."""
    if object_names is None:
        return {"type": "string",
                "description": "base, tool, or object:<name>",
                "pattern": f"^(base|tool|{OBJECT_PREFIX}.+)$"}
    return {"type": "string",
            "enum": list(FRAMES) + [f"{OBJECT_PREFIX}{n}" for n in object_names]}


def _default(value: Any) -> Any:
    """A field default as JSON: a direction is written the way a model writes it."""
    return value.as_argument() if isinstance(value, Direction) else value


def direction_doc() -> str:
    """The prompt text for directions, GENERATED from the aliases.

    One line per alias from :meth:`Direction.label` and its vector, so the
    text a model reads cannot drift from what ``decode`` accepts (L11: the old
    ``APPROACH_DOC`` existed to be this and was never used).
    """
    lines = ["A direction is which way the HAND TRAVELS. Name one of:"]
    for name, d in ALIASES.items():
        axis = ", ".join(f"{c:+.0f}" for c in d.v)
        lines.append(f"  {d.label()}: ({axis}) in {d.frame} — "
                     f"{ALIAS_MEANING.get(name, '')}".rstrip(" —"))
    lines.append("  or give {axis: [x, y, z], frame: base|tool|object:<name>} "
                 "for any other; base is +x forward, +y the robot's left, +z up.")
    return "\n".join(lines)


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
        roles = cls.arg_roles()
        properties: Dict[str, Any] = {}
        required: List[str] = []
        for field in fields(cls):
            if field.name in NOT_MODEL_BINDABLE:
                continue
            spec = argument(field.name, cls.name(), overrides, roles)
            kind = spec.domain()["kind"]
            if world is None or kind not in ("name", "direction"):
                names = None
            elif kind == "direction":
                names = world.names()
            else:
                names = names_for(world, spec.role)
            properties[field.name] = _json_schema(spec, names)
            if kind == "name":
                required.append(field.name)
            elif field.default is not None:
                properties[field.name]["default"] = _default(field.default)
        description = (cls.__doc__ or "").strip().splitlines()[0]
        if cls.name() in CAVEATS:
            description = f"{description} {CAVEATS[cls.name()]}"
        out.append({
            "name": cls.name(),
            "description": description,
            "parameters": {"type": "object", "properties": properties,
                           "required": required, "additionalProperties": False},
        })
    return out


def decode(name: str, arguments: Dict[str, Any],
           world: Optional[WorldView] = None, *,
           model_bindable_only: bool = True) -> Union[Primitive, PlanError]:
    """A model's function call back into a primitive, or a typed refusal.

    Every way a call can be malformed ends here as a ``PlanError`` with
    ``reason="bad_argument"`` — an unknown verb, an unknown field, a wrong
    type, a nonfinite number, a value out of range, a name that is not in the
    world or cannot play that role. The old loop caught ``TypeError`` and
    ``ValueError`` around the CONSTRUCTOR only, and planning happened outside
    that handler (R3/R14).

    ``model_bindable_only`` (the default — this is the MODEL's door) refuses
    every field in :data:`NOT_MODEL_BINDABLE`: a model that sends ``policy``
    gets a typed refusal, not a silently-honoured knob (and ``roll_rad``,
    which no verb has, is an unknown argument)
    (Astra review 5). A direction arrives as an alias or ``{axis, frame}``
    (or a bare ``[x, y, z]``, base frame) and leaves as a ``Direction``.
    """
    cls = BY_VERB.get(name)
    if cls is None:
        return PlanError(BAD_ARGUMENT,
                         f"no verb called {name!r}; the verbs are "
                         f"{sorted(BY_VERB)}", primitive=str(name))
    known = {f.name for f in fields(cls)}
    if model_bindable_only:
        hidden = sorted(set(arguments) & set(NOT_MODEL_BINDABLE) & known)
        if hidden:
            return PlanError(
                BAD_ARGUMENT,
                f"{name}: {hidden} not model-bindable — the robot decides "
                f"{'them' if len(hidden) > 1 else 'it'}; leave "
                f"{'them' if len(hidden) > 1 else 'it'} out",
                primitive=name,
                unmet=tuple(Unmet(BAD_ARGUMENT,
                                  f"{h!r} is not a model argument",
                                  "leave it out", {"argument": h})
                            for h in hidden))
        known -= set(NOT_MODEL_BINDABLE)
    extra = sorted(set(arguments) - known)
    if extra:
        return PlanError(BAD_ARGUMENT,
                         f"{name} takes {sorted(known)}, not {extra}",
                         primitive=name,
                         unmet=tuple(Unmet(BAD_ARGUMENT, f"unknown argument {e!r}")
                                     for e in extra))
    arguments = dict(arguments)
    for key, value in list(arguments.items()):
        spec = ARGUMENTS.get(key)
        if spec is not None and spec.kind == "direction":
            try:
                arguments[key] = parse_direction(value)
            except (TypeError, ValueError) as exc:
                return PlanError(BAD_ARGUMENT, f"{name}: {key}: {exc}",
                                 primitive=name,
                                 unmet=(Unmet(BAD_ARGUMENT, str(exc),
                                              "name a direction or give "
                                              "{axis, frame}",
                                              {"argument": key}),))
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
    roles = call.arg_roles()
    for field in fields(call):
        spec = (argument(field.name, call.name(), None, roles)
                if field.name in ARGUMENTS else None)
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


def domains(*, model_bindable_only: bool = True) -> Dict[str, Dict[str, Any]]:
    """verb -> argument -> domain, straight from the kit's own table.

    The drift gate compares an EXPORT against this, so a renderer that
    invented its own range has something to fail against. The old test
    compared the export with the same function that generated it.
    """
    return {cls.name(): {f.name: argument_domain(f.name, cls.name())
                         for f in fields(cls)
                         if not (model_bindable_only
                                 and f.name in NOT_MODEL_BINDABLE)}
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
        cls = BY_VERB.get(schema["name"])
        roles = cls.arg_roles() if cls is not None else {}
        for name, prop in schema["parameters"]["properties"].items():
            if "enum" in prop and name in ("object", "to", "source", "target"):
                rendered[name] = {"kind": "name",
                                  "role": roles.get(name, ARGUMENTS[name].role)}
            elif "enum" in prop:
                rendered[name] = {"kind": "enum", "values": list(prop["enum"])}
            elif prop.get("type") == "number":
                rendered[name] = {"kind": "number",
                                  "minimum": prop["minimum"],
                                  "maximum": prop["maximum"],
                                  "unit": ARGUMENTS[name].unit}
            elif prop.get("type") == "boolean":
                rendered[name] = {"kind": "boolean"}
            elif "oneOf" in prop:
                named = [b for b in prop["oneOf"] if b.get("type") == "string"]
                free = [b for b in prop["oneOf"] if b.get("type") == "object"]
                rendered[name] = {"kind": "direction",
                                  "aliases": list(named[0]["enum"]) if named else [],
                                  "free": bool(free)}
            elif name in ARGUMENTS and ARGUMENTS[name].kind == "name":
                rendered[name] = {"kind": "name",
                                  "role": roles.get(name, ARGUMENTS[name].role)}
            else:
                rendered[name] = {"kind": "string"}
        out[schema["name"]] = rendered
    return out


def main(argv: Optional[Sequence[str]] = None) -> int:
    print(json.dumps(tool_schemas(), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
