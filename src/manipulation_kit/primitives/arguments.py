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

from ..hands.d1.parallel_gripper.description import HAND_POSES
from ..world.direction import ALIASES, Direction
from .types import (BAD_ARGUMENT, GRIPS, NUDGE_FRAMES,
                    NUDGE_GRID_M, NUDGE_MAX_YAW_RAD, SIDE_CHOICES, Unmet)

#: THE MODEL-FACING BOUNDS OF THE CONTACT VERBS. A contact threshold is a
#: number a model may pick — "touch lightly", "press firmly" — but only inside
#: a window that is safe on the arm it drives, and the operator policy
#: (``agent.policy``, redesign step 7) may clamp it further.
#:
#: ``contact_nm`` 1.5-6 Nm. Below 1.5 Nm the rise is inside what an arm
#: moving at contact speed shows WITHOUT touching anything (gravity torque
#: changing with posture over a 15 cm leg, the controller's own tracking
#: effort); above 6 Nm a probe leans on what it touches instead of feeling
#: it — the default 4 Nm is the design's number (C.3), pending the d1-2 gate.
#: ``force_nm`` 2-8 Nm: a press must be able to exceed a probe's default and
#: must stay under 2x the probe ceiling, where the watch stops at once
#: (``executor.CONTACT_ABORT_FACTOR``). Neither is measured on hardware yet —
#: ``docs/probe-hardware-trial.md`` is the trial that measures them.
CONTACT_NM_RANGE: Tuple[float, float] = (1.5, 6.0)
FORCE_NM_RANGE: Tuple[float, float] = (2.0, 8.0)
#: the longest contact leg a model may ask for [m]
MAX_CONTACT_TRAVEL_M = 0.30
#: the deepest a press may push past the declared face [m]
MAX_PRESS_DEPTH_M = 0.03
#: the longest a press may hold [s]
MAX_HOLD_S = 5.0

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
    kind: str                       # enum | name | number | string | bool | direction
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
        if self.kind == "direction":
            # a named alias OR a free vector in a named frame
            return {"kind": "direction", "aliases": list(ALIASES),
                    "free": True}
        return {"kind": "string"}

    def check(self, value: Any) -> List[Unmet]:
        """Everything wrong with ``value``, as typed conditions."""
        if self.kind == "enum":
            if value not in self.values:
                return [_bad(self.name, f"must be one of "
                                        f"{list(self.values)}, got {value!r}")]
            return []
        if self.kind == "direction":
            if not isinstance(value, Direction):
                return [_bad(self.name, f"must be a Direction (an alias of "
                                        f"{sorted(ALIASES)} or "
                                        f"{{axis, frame}}), got {value!r}")]
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
    Argument("target", "name",
             "the thing to act ON, by name: the vessel to pour into (pour), "
             "the thing whose near face to press (press)",
             role=ROLE_VESSEL),
    _enum("side", SIDE_CHOICES,
          "which hand; 'auto' lets the robot pick, and the plan says which"),
    _enum("from_side", SIDE_CHOICES,
          "handover: the hand that holds the object now; 'auto' = whichever "
          "one does"),
    _enum("to_side", SIDE_CHOICES,
          "handover: the hand that takes it; 'auto' = the other one"),
    Argument("direction", "direction",
             "which way the hand TRAVELS: a named direction ("
             + ", ".join(ALIASES) + ") or {axis: [x, y, z], frame: "
             "base|tool|object:<name>}. The wrist orientation is derived from "
             "it; you never give one"),
    _enum("grip", GRIPS,
          "how hard to hold: the preset owns the stop torque, so there is no "
          "number here"),
    _enum("frame", NUDGE_FRAMES,
          "'tool' = along the hand's own axes, 'base' = along the robot's"),
    _number("standoff_m", 0.02, 0.30, "m",
            "how far off the object to wait before closing on it"),
    # "pad" / "tip": the literal is the closed set of
    # ``grasp_geometry.CONTACTS`` (checked by the suite; importing it here
    # would be a cycle)
    _enum("contact", ("pad", "tip"),
          "WHERE on the hand the object is taken: 'pad' = between the pad "
          "centres (the default; the fingers reach past it), 'tip' = between "
          "the finger tips (for something flat lying on a surface, like a "
          "card)"),
    _number("height_m", 0.01, 0.40, "m", "how far to lift, along its direction"),
    _number("clearance_m", 0.0, 0.40, "m",
            "how far above the destination (carry: transit height above the "
            "rim; place: how far above its floor the object is let go); "
            "handover: how far the giving hand backs out after letting go"),
    _number("distance_m", 0.01, 0.40, "m",
            "how far to back out, along its direction"),
    _number("tilt_deg", 15.0, 120.0, "deg", "how far to tip the source"),
    _number("dx", -max(NUDGE_GRID_M), max(NUDGE_GRID_M), "m",
            f"correction along the frame's x, snapped to "
            f"{[int(g * 1000) for g in NUDGE_GRID_M]} mm. THE BOUND IS PER "
            f"AXIS: three components at {int(max(NUDGE_GRID_M) * 1000)} mm "
            f"move the hand {max(NUDGE_GRID_M) * 1000 * 3 ** 0.5:.0f} mm"),
    _number("dy", -max(NUDGE_GRID_M), max(NUDGE_GRID_M), "m",
            "correction along the frame's y, same grid, same per-axis bound"),
    _number("dz", -max(NUDGE_GRID_M), max(NUDGE_GRID_M), "m",
            "correction along the frame's z, same grid, same per-axis bound"),
    _number("dyaw", -NUDGE_MAX_YAW_RAD, NUDGE_MAX_YAW_RAD, "rad",
            "turn about the HAND'S OWN approach axis (not base yaw), clamped "
            "to +-15 degrees. Radians"),
    Argument("policy", "string",
             "DEPLOYMENT CONFIGURATION, not a model choice: which learned "
             "checkpoint runs this verb. Not offered to a model — see "
             "manipulation_kit.primitives.schema.NOT_MODEL_BINDABLE"),
    Argument("allow_drop", "bool",
             "may the object be RELEASED above its destination when the arm "
             "cannot reach down to set it down?"),
    # -- the contact verbs (probe / press) --------------------------------- #
    _number("max_travel_m", 0.01, MAX_CONTACT_TRAVEL_M, "m",
            "how far the hand may travel along its direction looking for "
            "something to touch; it stops at the first resistance"),
    _number("contact_nm", CONTACT_NM_RANGE[0], CONTACT_NM_RANGE[1], "Nm",
            "how much a joint's torque must RISE over its value before the "
            "motion for the resistance to count as contact (measured, never "
            "commanded: the arm stays in position control)"),
    _number("depth_m", 0.0, MAX_PRESS_DEPTH_M, "m",
            "how far past the target's near face the press may push"),
    _number("force_nm", FORCE_NM_RANGE[0], FORCE_NM_RANGE[1], "Nm",
            "the joint-torque rise at which the press has pressed hard "
            "enough and stops (measured, never commanded)"),
    _number("hold_s", 0.0, MAX_HOLD_S, "s",
            "how long to keep pressing once it has pressed"),
    Argument("declare_as", "string",
             "publish the measured contact as a SURFACE with this name (a new "
             "name, or an existing surface to re-measure); '' publishes "
             "nothing. Three probes under one name fit a plane"),
    _enum("hand", HAND_POSES,
          "what the hand is while it touches: 'closed' = pads shut, one "
          "blunt fingertip; 'open' = both tips lead; 'pinched' = nearly shut"),
)}


def argument(name: str, verb: Optional[str] = None,
             arg_enums: Optional[Dict[str, Sequence[str]]] = None,
             arg_roles: Optional[Dict[str, str]] = None) -> Argument:
    """The canonical description of ``name``, narrowed for ``verb``.

    ``Primitive.arg_enums`` is the per-verb widening/narrowing — ``go_home`` is
    the only verb for which ``side="both"`` means anything — and it lives on
    the verb so the two exports cannot disagree about it.
    ``Primitive.arg_roles`` does the same for what a NAME argument may name:
    ``target`` is a vessel for ``pour`` and anything in the world for
    ``press``.
    """
    base = ARGUMENTS.get(name)
    if base is None:
        return Argument(name, "string", "")
    if arg_enums and name in arg_enums:
        return Argument(base.name, "enum", base.doc,
                        values=tuple(arg_enums[name]))
    if arg_roles and name in arg_roles and base.kind == "name":
        return Argument(base.name, "name", base.doc, role=arg_roles[name])
    return base


def check_arguments(primitive: Any) -> List[Unmet]:
    """Every bound field of ``primitive``, against the table. Cheap, no world.

    Called first by every ``preconditions``, so a malformed call is a typed
    refusal instead of an exception from inside a comparison — and so a
    nonfinite number never reaches a snap.
    """
    overrides = primitive.arg_enums()
    roles = primitive.arg_roles()
    unmet: List[Unmet] = []
    for field in fields(primitive):
        spec = argument(field.name, primitive.name(), overrides, roles)
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
