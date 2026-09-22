"""What this robot may do tonight — one object instead of eight env vars.

Design C.8, L10, L13, B12. The d1-2 nights of 2026-09-22 grew operator policy
as environment variables read inside the example (``ASTRA_GRIP_CAP``,
``ASTRA_APPROACH_ALLOW``, ``ASTRA_VEL_RATIO``, ``ASTRA_ARRIVE_TIMEOUT_S``): a
wheel customer could not get them, a test could not see them, and the
refusals they produced were hand-rolled dicts that did not look like any other
refusal the model learns from. They are fields here:

    policy = OperatorPolicy(max_grip="soft", allowed_directions=("down",))
    call, unmet = policy.clamp(Grasp(object="cube", grip="strong"))
    call.grip   # "soft" — a cap, not a refusal
    unmet       # [] — or kit Unmets, which the loop sends back as a refusal

:meth:`OperatorPolicy.clamp` is the ONE home for the grip cap, the direction
restriction, the look-before-stroke rule and the nudge budget, and everything
it refuses with is a kit :class:`~manipulation_kit.primitives.types.Unmet`.
The timeouts are resolved ONCE (:meth:`OperatorPolicy.settings`) and the same
numbers reach the executor's constructor and every ``run()`` (L13).

``to_json`` / ``from_json`` make it a file an operator can keep beside a run;
``add_arguments`` / ``from_args`` make it CLI flags, generated from the fields,
so a new field cannot exist without a flag.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Tuple

import numpy as np

from ..primitives.types import GRIPS, Primitive, Unmet
from ..world.direction import ALIASES, Direction

#: the policy refused a direction it does not allow
DIRECTION_NOT_ALLOWED = "direction_not_allowed"
#: a stroke needs one wrist look first, and none has been taken at this posture
LOOK_REQUIRED = "look_required"
#: a look is required and this robot has no wrist camera model to take it with
LOOK_UNAVAILABLE = "look_unavailable"
#: the correction budget for this target is spent
NUDGE_LIMIT = "nudge_limit"
#: a look is required and the verb closes a hand at a posture no look can
#: precede (``handover``: the receiver reaches its closing posture inside the
#: same plan)
LOOK_NOT_POSSIBLE = "look_not_possible"

POLICY_CODES: Tuple[str, ...] = (DIRECTION_NOT_ALLOWED, LOOK_REQUIRED,
                                 LOOK_UNAVAILABLE, NUDGE_LIMIT,
                                 LOOK_NOT_POSSIBLE)

def directed_verbs() -> Tuple[str, ...]:
    """The registered verbs whose ``direction`` is how the hand ARRIVES — the
    ones ``allowed_directions`` restricts. Derived from the verb registry
    (``Primitive.DIRECTION_ARRIVES``), so a new contact verb cannot slip past
    the operator's restriction; a Lift's "up" or a Retreat is not an arrival.
    """
    from ..primitives.verbs import BY_VERB  # noqa: PLC0415
    return tuple(name for name, cls in BY_VERB.items()
                 if "direction" in cls.arguments()
                 and getattr(cls, "DIRECTION_ARRIVES", False))


#: :func:`directed_verbs`, at import (approach, grasp, probe, press, handover)
DIRECTED_VERBS: Tuple[str, ...] = directed_verbs()
#: the verbs that close the jaws on something
STROKE_VERBS: Tuple[str, ...] = ("grasp",)
#: the verbs that close the jaws at a posture they reach INSIDE their own
#: plan, so no look can be taken from it first
UNLOOKABLE_STROKE_VERBS: Tuple[str, ...] = ("handover",)


@dataclass
class PolicyState:
    """What the policy has to remember between turns: which object each hand
    is working on, how many corrections it has spent on it, and where the
    last wrist look was taken. Kept by the loop, read by :meth:`clamp`."""

    target: Dict[str, Optional[str]] = field(default_factory=dict)
    nudges: Dict[Tuple[str, str], int] = field(default_factory=dict)
    #: side -> (object, the arm's joints when the look was taken)
    looked: Dict[str, Tuple[str, np.ndarray]] = field(default_factory=dict)

    def aimed(self, side: str, name: Optional[str]) -> None:
        if name:
            self.target[side] = name

    def nudged(self, side: str) -> int:
        name = self.target.get(side) or ""
        key = (side, name)
        self.nudges[key] = self.nudges.get(key, 0) + 1
        return self.nudges[key]

    def look(self, side: str, name: str, joints) -> None:
        self.looked[side] = (name, np.array(joints, dtype=float))

    def moved(self, side: str) -> None:
        """The arm went somewhere a look did not see: the look is spent."""
        self.looked.pop(side, None)

    def has_looked(self, side: str, name: str, joints,
                   tol_rad: float = math.radians(1.0)) -> bool:
        seen = self.looked.get(side)
        if seen is None or seen[0] != name or joints is None:
            return False
        return bool(np.max(np.abs(np.asarray(joints, dtype=float) - seen[1]))
                    <= tol_rad)


def _positive(name: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"OperatorPolicy.{name} must be a positive finite "
                         f"number, got {value!r}")
    return value


@dataclass(frozen=True)
class OperatorPolicy:
    """What this robot may do tonight. One object — documented, serialisable,
    testable — instead of eight environment variables."""

    #: the firmest grip preset a call may use. A CAP, not a choice: a firmer
    #: request is lowered to it (d1-2 2026-09-22: a ``firm`` preload on a
    #: rigid body wound the hold up to -4.2 Nm and faulted the motor).
    max_grip: str = "strong"
    #: the directions (aliases) a hand may ARRIVE along — every verb in
    #: :data:`DIRECTED_VERBS` (approach, grasp, probe, press); ``None`` =
    #: whatever plans. Anything else — another alias or a free
    #: ``{axis, frame}`` — is refused with :data:`DIRECTION_NOT_ALLOWED`.
    allowed_directions: Optional[Tuple[str, ...]] = None
    #: the daemon's velocity (and acceleration) ratio, (0, 1]
    vel_ratio: float = 0.15
    #: how long a posture may take to be reached, at every arrival barrier
    arrive_timeout_s: float = 4.0
    #: how long a gripper stroke may take. ``None`` = the executor's own bound
    #: (on firmware, the daemon document's ``x-timeout-seconds`` for the
    #: stroke — a policy number must not silently override the document).
    stroke_timeout_s: Optional[float] = None
    #: a Grasp stroke needs one wrist look (and, if the photo disagrees, one
    #: correction) at the posture it closes from
    look_before_stroke: bool = True
    #: corrections (``nudge``, and look-corrections) per hand per target
    max_nudges_per_target: int = 3
    #: the loop's hard cap
    max_turns: int = 12
    #: the firmest contact threshold a ``probe`` may stop at [Nm of joint
    #: torque rise] — a CAP like ``max_grip`` (the schema allows 1.5..6)
    max_contact_nm: float = 4.0
    #: the firmest ``press`` [Nm of joint torque rise] (the schema allows 2..8)
    max_force_nm: float = 6.0
    #: how far THIS arm sags below the commanded pose at a long reach [m]
    #: (F16). It raises the fingertip floor of every descent — the typed
    #: replacement of ``MKIT_SUPPORT_CLEARANCE_M``, applied to the kinematics
    #: as :class:`~manipulation_kit.primitives.clearance.ClearancePolicy`
    #: ``droop_margin_m`` when a loop starts. 0 = the rigid model (the
    #: default, and right for the kinematic mirror); **d1-2 measured 0.012**
    #: (2026-09-22 run 7: 3 mm left the pad tips on the wagon and the
    #: controller raised error 15; 15 mm cleared it).
    droop_margin_m: float = 0.0

    def __post_init__(self) -> None:
        if self.max_grip not in GRIPS:
            raise ValueError(f"OperatorPolicy.max_grip must be one of {GRIPS}, "
                             f"got {self.max_grip!r}")
        if self.allowed_directions is not None:
            allowed = tuple(str(d) for d in self.allowed_directions)
            unknown = sorted(set(allowed) - set(ALIASES))
            if unknown or not allowed:
                raise ValueError(f"OperatorPolicy.allowed_directions must be "
                                 f"named directions from {sorted(ALIASES)}, "
                                 f"got {list(self.allowed_directions)!r}")
            object.__setattr__(self, "allowed_directions", allowed)
        ratio = float(self.vel_ratio)
        if not (math.isfinite(ratio) and 0.0 < ratio <= 1.0):
            raise ValueError(f"OperatorPolicy.vel_ratio is a FRACTION in "
                             f"(0, 1], got {self.vel_ratio!r}")
        object.__setattr__(self, "vel_ratio", ratio)
        object.__setattr__(self, "arrive_timeout_s",
                           _positive("arrive_timeout_s", self.arrive_timeout_s))
        if self.stroke_timeout_s is not None:
            object.__setattr__(self, "stroke_timeout_s",
                               _positive("stroke_timeout_s",
                                         self.stroke_timeout_s))
        for name in ("max_nudges_per_target", "max_turns"):
            value = getattr(self, name)
            if isinstance(value, bool) or int(value) != value or value < (
                    1 if name == "max_turns" else 0):
                raise ValueError(f"OperatorPolicy.{name} must be a "
                                 f"{'positive' if name == 'max_turns' else 'non-negative'}"
                                 f" integer, got {value!r}")
            object.__setattr__(self, name, int(value))
        object.__setattr__(self, "look_before_stroke",
                           bool(self.look_before_stroke))
        for name in ("max_contact_nm", "max_force_nm"):
            object.__setattr__(self, name, _positive(name, getattr(self, name)))
        droop = float(self.droop_margin_m)
        if not math.isfinite(droop) or droop < 0.0:
            raise ValueError(f"OperatorPolicy.droop_margin_m must be a finite "
                             f"non-negative length, got {self.droop_margin_m!r}")
        object.__setattr__(self, "droop_margin_m", droop)

    # -- the one gate --------------------------------------------------------- #
    def clamp(self, call: Primitive, state: Optional[PolicyState] = None, *,
              side: Optional[str] = None, joints=None
              ) -> Tuple[Primitive, List[Unmet]]:
        """``call`` as this policy lets it run, and what it refuses.

        The grip cap LOWERS the grip (it is a cap, not a refusal; :meth:`notes`
        says what changed). The direction restriction, the look rule and the
        nudge budget REFUSE, with kit ``Unmet`` s — the loop hands them back as
        a ``PlanError`` so a policy refusal reads like every other refusal.

        ``state`` / ``side`` / ``joints`` (the resolved hand and its measured
        joints) are what the two stateful rules need; without ``state`` only
        the stateless ones apply.
        """
        unmet: List[Unmet] = []
        verb = call.name()
        grip = getattr(call, "grip", None)
        if isinstance(grip, str) and grip in GRIPS and \
                GRIPS.index(grip) > GRIPS.index(self.max_grip):
            call = dataclasses.replace(call, grip=self.max_grip)
        # the contact verbs' torque thresholds (``Probe.contact_nm``,
        # ``Press.force_nm``, step 4) are capped the same way: lowered, not
        # refused — a softer stop is always the safer answer
        for attr, cap in (("contact_nm", self.max_contact_nm),
                          ("force_nm", self.max_force_nm)):
            value = getattr(call, attr, None)
            if isinstance(value, (int, float)) and not isinstance(value, bool) \
                    and float(value) > cap:
                call = dataclasses.replace(call, **{attr: cap})
        direction = getattr(call, "direction", None)
        if (self.allowed_directions is not None and verb in DIRECTED_VERBS
                and isinstance(direction, Direction)):
            name = direction.alias()
            if name not in self.allowed_directions:
                unmet.append(Unmet(
                    DIRECTION_NOT_ALLOWED,
                    f"{verb} {direction.label()!r} is not allowed on this "
                    f"robot tonight",
                    f"use one of: {', '.join(self.allowed_directions)}",
                    {"asked": direction.as_argument(),
                     "allowed": list(self.allowed_directions)}))
        if self.look_before_stroke and verb in UNLOOKABLE_STROKE_VERBS:
            unmet.append(Unmet(
                LOOK_NOT_POSSIBLE,
                f"a {verb} closes the receiving hand at a posture it only "
                f"reaches inside its own plan, so no wrist look can precede "
                f"that stroke, and this policy requires one",
                "compose it: approach the held object with the receiving "
                "hand, grasp it (the look happens there), release the giving "
                "hand, retreat it — or run with --no-look-before-stroke"))
        if state is None or side is None:
            return call, unmet
        name = getattr(call, "object", "") or state.target.get(side) or ""
        if verb == "nudge" and self.max_nudges_per_target >= 0:
            spent = state.nudges.get((side, state.target.get(side) or ""), 0)
            if spent >= self.max_nudges_per_target:
                unmet.append(Unmet(
                    NUDGE_LIMIT,
                    f"{spent} corrections of the {side} hand on "
                    f"{state.target.get(side) or 'this target'} already; the "
                    f"budget is {self.max_nudges_per_target}",
                    "re-declare the object from what you see, or approach "
                    "it again", {"spent": spent,
                                 "budget": self.max_nudges_per_target}))
        if (self.look_before_stroke and verb in STROKE_VERBS and name
                and not state.has_looked(side, name, joints)):
            unmet.append(Unmet(
                LOOK_REQUIRED,
                f"a {verb} stroke on {name!r} needs one wrist look at the "
                f"posture it closes from, and the {side} hand has not looked "
                f"from here",
                "the robot takes the look now; answer it, then ask again"))
        return call, unmet

    def notes(self, asked: Primitive, clamped: Primitive) -> List[str]:
        """What :meth:`clamp` changed, in words for the model."""
        out = []
        a, b = getattr(asked, "grip", None), getattr(clamped, "grip", None)
        if a != b:
            out.append(f"grip {a!r} is capped to {b!r} on this robot tonight")
        for attr in ("contact_nm", "force_nm"):
            a, b = getattr(asked, attr, None), getattr(clamped, attr, None)
            if a != b:
                out.append(f"{attr} {a:.1f} is capped to {b:.1f} Nm on this "
                           f"robot tonight")
        return out

    def describe(self) -> str:
        """The policy as the model is told it, generated from the fields."""
        lines = [f"OPERATOR POLICY (this robot, tonight): grip at most "
                 f"{self.max_grip!r}; probe contact at most "
                 f"{self.max_contact_nm:.1f} Nm, press at most "
                 f"{self.max_force_nm:.1f} Nm"]
        if self.allowed_directions is not None:
            lines.append(f"{'/'.join(DIRECTED_VERBS)} directions allowed: "
                         + ", ".join(self.allowed_directions))
        if self.look_before_stroke:
            lines.append("every grasp stroke is preceded by one wrist look "
                         "and, if the photo disagrees, one correction")
        lines.append(f"at most {self.max_nudges_per_target} corrections per "
                     f"hand per object; at most {self.max_turns} turns")
        return "; ".join(lines) + "."

    def apply_to(self, kin: Any) -> None:
        """Attach this policy's planning half to the arm model: the droop
        margin becomes the kinematics' ``ClearancePolicy`` (every verb's
        ``plan(world, kin)`` reads it there), other clearance fields kept."""
        from ..primitives.clearance import policy_of, set_policy  # noqa: PLC0415
        set_policy(kin, dataclasses.replace(
            policy_of(kin), droop_margin_m=self.droop_margin_m))

    # -- L13: resolved once ------------------------------------------------- #
    def settings(self, executor: Any = None) -> Dict[str, float]:
        """The timeouts every ``run()`` of this loop gets — resolved ONCE.

        ``stroke_timeout_s=None`` resolves to the executor's own configured
        bound (``executor.stroke_timeout_s``, which the firmware executor takes
        from the daemon's document) and only then to the kit default.
        """
        from ..executor import STROKE_TIMEOUT_S  # noqa: PLC0415
        stroke = self.stroke_timeout_s
        if stroke is None:
            stroke = getattr(executor, "stroke_timeout_s", None)
        if stroke is None:
            stroke = STROKE_TIMEOUT_S
        return {"arrive_timeout_s": float(self.arrive_timeout_s),
                "stroke_timeout_s": float(stroke)}

    # -- serialisation ------------------------------------------------------ #
    def to_json(self) -> Dict[str, Any]:
        out = dataclasses.asdict(self)
        if out["allowed_directions"] is not None:
            out["allowed_directions"] = list(out["allowed_directions"])
        return out

    @classmethod
    def from_json(cls, data: Mapping[str, Any]) -> "OperatorPolicy":
        known = {f.name for f in fields(cls)}
        extra = sorted(set(data) - known)
        if extra:
            raise ValueError(f"unknown OperatorPolicy fields {extra}; the "
                             f"fields are {sorted(known)}")
        data = dict(data)
        if data.get("allowed_directions") is not None:
            data["allowed_directions"] = tuple(data["allowed_directions"])
        return cls(**data)

    @classmethod
    def from_file(cls, path) -> "OperatorPolicy":
        return cls.from_json(json.loads(Path(path).read_text(encoding="utf-8")))

    # -- CLI, generated from the fields ------------------------------------- #
    @staticmethod
    def flag(name: str) -> str:
        return "--" + name.replace("_", "-")

    @classmethod
    def add_arguments(cls, parser: argparse.ArgumentParser) -> None:
        """One flag per field (``--max-grip``, ``--allowed-directions down,
        forward``, ``--no-look-before-stroke``...), plus ``--policy FILE``.
        Every flag defaults to "not given", so a file's value survives."""
        group = parser.add_argument_group(
            "operator policy", "manipulation_kit.agent.OperatorPolicy; flags "
            "override --policy FILE, which overrides the defaults")
        group.add_argument("--policy", type=Path, default=None,
                           metavar="FILE", help="an OperatorPolicy JSON file")
        for f in fields(cls):
            if f.type in ("bool", bool):
                group.add_argument(cls.flag(f.name), dest=f"policy_{f.name}",
                                   action=argparse.BooleanOptionalAction
                                   if hasattr(argparse, "BooleanOptionalAction")
                                   else "store_true", default=None)
            else:
                group.add_argument(cls.flag(f.name), dest=f"policy_{f.name}",
                                   default=None, metavar=f.name.upper())

    @classmethod
    def from_args(cls, args: argparse.Namespace) -> "OperatorPolicy":
        base = (cls.from_file(args.policy) if getattr(args, "policy", None)
                else cls())
        changes: Dict[str, Any] = {}
        for f in fields(cls):
            value = getattr(args, f"policy_{f.name}", None)
            if value is None:
                continue
            if f.name == "allowed_directions":
                value = (None if value in ("", "any") else
                         tuple(v.strip() for v in str(value).split(",")
                               if v.strip()))
            elif f.name in ("vel_ratio", "arrive_timeout_s",
                            "stroke_timeout_s", "max_contact_nm",
                            "max_force_nm", "droop_margin_m"):
                value = float(value)
            elif f.name in ("max_nudges_per_target", "max_turns"):
                value = int(value)
            changes[f.name] = value
        return dataclasses.replace(base, **changes)


__all__ = ["DIRECTED_VERBS", "directed_verbs", "DIRECTION_NOT_ALLOWED", "LOOK_REQUIRED",
           "LOOK_NOT_POSSIBLE", "UNLOOKABLE_STROKE_VERBS",
           "LOOK_UNAVAILABLE", "NUDGE_LIMIT", "OperatorPolicy",
           "POLICY_CODES", "PolicyState", "STROKE_VERBS"]
