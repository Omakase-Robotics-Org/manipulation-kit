"""ONE definition of every argument a caller (or a model) may bind.

The kit owns this, not a schema exporter and not a renderer, because there is
exactly one thing a primitive's arguments can legally be and it must be the
same thing whichever door they came in through:

* a **Python caller** writing ``Place(object="x", to="bin", clearance_m=0.01)``,
* a **function-calling model** whose JSON arrives as ``{"clearance_m": "1cm"}``,
* a **typed-choice model** that picked an already-bound candidate,
* and the **JSON Schema export**, which must describe exactly what the runtime
  will accept.

Before this module the three disagreed. ``Place.preconditions`` validated a
freshly constructed ``Carry`` with ``clearance_m=0.0`` and never looked at its
own clearance, so a negative or nonfinite one reached ``_drop_pose``;
``Nudge(dx=NaN)`` snapped to +10 mm and ``Nudge(dyaw=NaN)`` to +15 degrees
because ``min``/``copysign`` are happy to compare with a NaN; and a wrong JSON
type raised a ``TypeError`` from inside a comparison, past the loop's ``except
(TypeError, ValueError)`` boundary in some paths (R14). A numeric range in a
schema is a description of intent, not a check.

So: one table, three consumers. Units are in the NAME (``_m``, ``_deg``,
``dyaw`` is radians and says so), every number must be finite, and a value
outside its domain produces a typed :class:`~.types.Unmet` with
``code="bad_argument"`` rather than an exception.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields
from typing import Any, Dict, List, Optional, Sequence, Tuple

from .approach import APPROACH_DOC
from .types import (APPROACHES, BAD_ARGUMENT, GRIPS, NUDGE_FRAMES,
                    NUDGE_GRID_M, NUDGE_MAX_YAW_RAD, SIDE_CHOICES, Unmet)

#: What an argument NAMES in the world. Roles narrow both the schema and the
#: runtime check: a table is not a grasp candidate and a loose block is not a
#: placement destination, and offering all world names for every slot is how
#: ``tool_schemas`` invited both (R, section 3).
ROLE_ANY = "any"
ROLE_GRASPABLE = "graspable"     # a movable thing: not a container, not a surface
ROLE_DESTINATION = "destination"  # a container or a surface
ROLE_VESSEL = "vessel"           # something with an inside, for pour


@dataclass(frozen=True)
class Argument:
    """One argument: its type, its domain, its units and what it means."""

    name: str
    kind: str                       # enum | name | number | string
    doc: str = ""
    values: Tuple[str, ...] = ()    # kind == enum
    minimum: float = float("-inf")  # kind == number
    maximum: float = float("inf")
    unit: str = ""
    role: str = ROLE_ANY            # kind == name

    def domain(self) -> Dict[str, Any]:
        """The shared description both exports render from."""
        if self.kind == "enum":
            return {"kind": "enum", "values": list(self.values)}
        if self.kind == "name":
            return {"kind": "name", "role": self.role}
        if self.kind == "number":
            return {"kind": "number", "minimum": float(self.minimum),
                    "maximum": float(self.maximum), "unit": self.unit}
        if self.kind == "bool":
            return {"kind": "boolean"}
        return {"kind": "string"}

    def check(self, value: Any) -> List[Unmet]:
        """Everything wrong with ``value``, as typed conditions."""
        if self.kind == "enum":
            if value not in self.values:
                return [_bad(self.name, f"must be one of "
                                        f"{list(self.values)}, got {value!r}")]
            return []
        if self.kind in ("name", "string"):
            if not isinstance(value, str):
                return [_bad(self.name, f"must be a name (a string), got "
                                        f"{type(value).__name__}")]
            return []
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return [_bad(self.name, f"must be a number{_units(self.unit)}, got "
                                    f"{type(value).__name__} {value!r}")]
        number = float(value)
        if not math.isfinite(number):
            # BEFORE any snapping or clamping: min()/copysign() will happily
            # take a NaN and hand back a plausible 10 mm.
            return [_bad(self.name, f"must be finite{_units(self.unit)}, got "
                                    f"{value!r}")]
        if not (self.minimum - 1e-12 <= number <= self.maximum + 1e-12):
            return [_bad(self.name,
                         f"must be between {self.minimum} and {self.maximum}"
                         f"{_units(self.unit)}, got {number}")]
        return []


def _units(unit: str) -> str:
    return f" [{unit}]" if unit else ""


def _bad(name: str, detail: str) -> Unmet:
    return Unmet(BAD_ARGUMENT, f"{name} {detail}",
                 "bind it inside its published domain",
                 {"argument": name})


def _enum(name: str, values: Sequence[str], doc: str) -> Argument:
    return Argument(name, "enum", doc, values=tuple(values))


def _number(name: str, low: float, high: float, unit: str, doc: str) -> Argument:
    return Argument(name, "number", doc, minimum=float(low),
                    maximum=float(high), unit=unit)


#: THE table. Every field of every primitive resolves through this.
ARGUMENTS: Dict[str, Argument] = {a.name: a for a in (
    Argument("object", "name",
             "the movable thing to act on, named exactly as the world lists it",
             role=ROLE_GRASPABLE),
    Argument("to", "name",
             "the container or surface to move it to, by name",
             role=ROLE_DESTINATION),
    Argument("source", "name", "the vessel to pour FROM, by name",
             role=ROLE_VESSEL),
    Argument("target", "name", "the vessel to pour INTO, by name",
             role=ROLE_VESSEL),
    _enum("side", SIDE_CHOICES,
          "which hand; 'auto' lets the robot pick, and the plan says which"),
    _enum("approach", APPROACHES,
          "; ".join(f"{k}: {v}" for k, v in APPROACH_DOC.items())),
    _enum("grip", GRIPS,
          "how hard to hold: the preset owns the stop torque, so there is no "
          "number here"),
    _enum("frame", NUDGE_FRAMES,
          "'tool' = along the hand's own axes, 'base' = along the robot's"),
    _number("standoff_m", 0.02, 0.30, "m",
            "how far off the object to wait before closing on it"),
    _number("height_m", 0.01, 0.40, "m", "how far straight up"),
    _number("clearance_m", 0.0, 0.40, "m",
            "how far above the destination (carry: transit height above the "
            "rim; place: how far above its floor the object is let go)"),
    _number("distance_m", 0.01, 0.40, "m", "how far straight back"),
    _number("tilt_deg", 15.0, 120.0, "deg", "how far to tip the source"),
    _number("dx", -max(NUDGE_GRID_M), max(NUDGE_GRID_M), "m",
            f"correction along the frame's x, snapped to "
            f"{[int(g * 1000) for g in NUDGE_GRID_M]} mm"),
    _number("dy", -max(NUDGE_GRID_M), max(NUDGE_GRID_M), "m",
            "correction along the frame's y, same grid"),
    _number("dz", -max(NUDGE_GRID_M), max(NUDGE_GRID_M), "m",
            "correction along the frame's z, same grid"),
    _number("dyaw", -NUDGE_MAX_YAW_RAD, NUDGE_MAX_YAW_RAD, "rad",
            "turn about the hand's approach axis, clamped to +-15 degrees"),
    Argument("policy", "string",
             "deployment configuration, not a model choice: which learned "
             "checkpoint runs this verb"),
    Argument("allow_drop", "bool",
             "may the object be RELEASED above its destination when the arm "
             "cannot reach down to set it down?"),
)}


def argument(name: str, verb: Optional[str] = None,
             arg_enums: Optional[Dict[str, Sequence[str]]] = None) -> Argument:
    """The canonical description of ``name``, narrowed for ``verb``.

    ``Primitive.arg_enums`` is the per-verb widening/narrowing — ``go_home`` is
    the only verb for which ``side="both"`` means anything — and it lives on
    the verb so the two exports cannot disagree about it.
    """
    base = ARGUMENTS.get(name)
    if base is None:
        return Argument(name, "string", "")
    if arg_enums and name in arg_enums:
        return Argument(base.name, "enum", base.doc,
                        values=tuple(arg_enums[name]))
    return base


def check_arguments(primitive: Any) -> List[Unmet]:
    """Every bound field of ``primitive``, against the table. Cheap, no world.

    Called first by every ``preconditions``, so a malformed call is a typed
    refusal instead of an exception from inside a comparison — and so a
    nonfinite number never reaches a snap.
    """
    overrides = primitive.arg_enums()
    unmet: List[Unmet] = []
    for field in fields(primitive):
        spec = argument(field.name, primitive.name(), overrides)
        if spec.kind == "bool":
            value = getattr(primitive, field.name)
            if not isinstance(value, bool):
                unmet.append(_bad(field.name, "must be true or false"))
            continue
        unmet += spec.check(getattr(primitive, field.name))
    return unmet


def names_for(world, role: str) -> Tuple[str, ...]:
    """The world's names that can legally fill a ``role``.

    The semantic narrowing the schema export needs: a surface is a placement
    destination and not a grasp candidate, a loose block is the other way
    round, and a model that cannot say "grasp the table" will not.
    """
    from ..world import ContainerView, SurfaceView  # noqa: PLC0415
    if role == ROLE_GRASPABLE:
        return tuple(o.name for o in world.objects
                     if not isinstance(o, (ContainerView, SurfaceView)))
    if role == ROLE_DESTINATION:
        return tuple(o.name for o in world.objects
                     if isinstance(o, (ContainerView, SurfaceView)))
    if role == ROLE_VESSEL:
        return tuple(o.name for o in world.objects
                     if isinstance(o, ContainerView))
    return world.names()
