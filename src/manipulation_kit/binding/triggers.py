"""``omakase.binding.v2`` — triggers: which operator input runs which action.

Version 2 of a binding file keeps everything version 1 has (``[[source]]``,
``[[bind]]`` for hands, arms, grippers and accessories) and adds the part
that turns an operator's switches into the teleoperation server's actions::

    schema = "omakase.binding.v2"

    [[source]]
    device    = "udcap-buttons"
    family    = "teleop_controls.event_stream.v1"
    companion = "udcap-sticks"          # a controllers-family source of the same glove
    sides     = { left = [0, 1, 2, 3], right = [4, 5, 6, 7] }
    endpoint  = "127.0.0.1:5582"        # optional "host:port" to subscribe to

    [[input]]                           # a threshold on an analog value
    name      = "ker-left-grip"
    from      = "ker-0/left_gripper"
    threshold = { on = 0.8, off = 0.6 }

    [[trigger]]
    from      = "pedal/#0"              # an input address or an [[input]] name
    with      = ["ker-left-grip"]       # optional modifiers
    modifiers = "during"                # "before" | "during"
    on        = "hold"                  # press | release | tap | hold | while
    hold_s    = 2.0                     # required iff on = "hold"
    do        = "engage.start"          # an action of :data:`CATALOG`
    args      = { arms = "right" }

This module only PARSES and VALIDATES. It has no run-time engine and no
robot-specific number: how a press is timed, which edge is real, and what a
robot's speed limit is belong to the consumer (the teleoperation server). The
checks here are the ones the file alone decides; the rest (a device's
declaration, its kind, the daemon's limits) are the consumer's, made when the
device arrives.

The catalog
-----------
Every ``do`` names an entry of :data:`CATALOG`. An entry has a TYPE (``oneshot``:
one gesture runs it once; ``held``: it lasts while a ``while`` gesture lasts)
and a safety CLASS, and the class fixes which gestures may run it and whether
modifiers may be used:

=================== =========================== =========
class               gestures                    modifiers
=================== =========================== =========
``stop``            ``press``                   no
``start_motion``    ``tap`` ``hold``            yes
``large_motion``    ``hold`` (>= 0.8 s)         yes
``continuous_motion`` ``while``                 no
``no_motion``       any                         yes
=================== =========================== =========

``engage.toggle`` is ``tap`` only: its press side is a stop that the consumer
performs by itself when the press begins while an arm is following.

``large_motion`` ``hold_s`` has a floor of :data:`LARGE_MOTION_MIN_HOLD_S`
here. The consumer applies a longer floor per kind of input once a device
declares its kind (a foot pedal needs longer than a hand button); this module
cannot know the kind.

Refusals
--------
Like version 1, every problem is listed at once in one
:class:`~manipulation_kit.binding.BindingError`, and an unknown key is refused.
"""
from __future__ import annotations

import re
import sys
from dataclasses import dataclass, field, replace
from typing import (Any, Dict, FrozenSet, List, Mapping, Optional,
                    Sequence, Tuple)

#: The ``schema`` string of a version 2 binding file.
BINDING_SCHEMA_V2 = "omakase.binding.v2"

#: The stick/switch family of a VR or game controller: the home of the axes
#: ``base.drive`` reads.
CONTROLLERS_FAMILY = "teleop_controllers.input_stream.v1"

#: The gesture vocabulary of a ``[[trigger]]``. ``while`` lasts as long as the
#: control is down.
TRIGGER_GESTURES = ("press", "release", "tap", "hold", "while")

#: How a key's modifiers are read: ``before`` = the modifiers held when the key
#: went down, ``during`` = held while the key is down (may be added late).
MODIFIER_READS = ("before", "during")

#: What an ``engage.*`` action's ``arms`` may be.
ENGAGE_ARMS = ("both", "left", "right")

#: Smallest ``hold_s`` of a ``large_motion`` action this module accepts. A
#: per-kind floor (longer for a pedal) is the consumer's.
LARGE_MOTION_MIN_HOLD_S = 0.8

#: The ``hold_s`` a v1 ``one_arm`` gesture becomes (the server's value).
ONE_ARM_HOLD_S = 2.0

#: Default ``threshold`` of the derived input a v1 ``one_arm`` becomes.
ONE_ARM_THRESHOLD = (0.8, 0.6)

#: ``base.drive`` defaults, applied by the parser so a parsed trigger carries
#: every value the consumer needs.
BASE_DRIVE_DEFAULTS = {"invert_linear": False, "invert_angular": False,
                       "deadband": 0.15, "max_linear_m_s": 0.25,
                       "max_angular_rad_s": 0.5}

_TRIGGER_KEYS = {"from", "with", "modifiers", "on", "hold_s", "do", "args"}
_INPUT_KEYS = {"name", "from", "threshold"}

_SIDES = ("left", "right")
_ONE_ARM_CHANNEL_RE = re.compile(r"^(left|right)_gripper$")
_ENDPOINT_RE = re.compile(r"^(?P<host>\[[0-9A-Fa-f:.]+\]|[^\s:/\[\]]+):(?P<port>\d+)$")

STOP = "stop"
START_MOTION = "start_motion"
LARGE_MOTION = "large_motion"
CONTINUOUS_MOTION = "continuous_motion"
NO_MOTION = "no_motion"

_CLASS_GESTURES = {
    STOP: ("press",),
    START_MOTION: ("tap", "hold"),
    LARGE_MOTION: ("hold",),
    CONTINUOUS_MOTION: ("while",),
    NO_MOTION: TRIGGER_GESTURES,
}
_CLASS_MODIFIERS = {STOP: False, START_MOTION: True, LARGE_MOTION: True,
                    CONTINUOUS_MOTION: False, NO_MOTION: True}


@dataclass(frozen=True)
class ActionSpec:
    """One row of :data:`CATALOG`."""

    name: str
    #: ``oneshot`` or ``held``.
    type: str
    #: ``stop``, ``start_motion``, ``large_motion``, ``continuous_motion`` or
    #: ``no_motion``.
    safety_class: str
    #: The gestures that may run it.
    gestures: Tuple[str, ...]
    #: Whether a trigger may carry ``with``/``modifiers``.
    modifiers_allowed: bool
    #: Every argument name it accepts.
    args: Tuple[str, ...] = ()
    #: The arguments with no default.
    required_args: Tuple[str, ...] = ()
    #: ``True`` for ``engage.toggle``: a press that begins while an arm follows
    #: is also a stop, performed by the consumer without a trigger of its own.
    implicit_press_stop: bool = False


def _spec(name: str, type_: str, cls: str, *, gestures: Optional[Tuple[str, ...]] = None,
          args: Tuple[str, ...] = (), required: Tuple[str, ...] = (),
          implicit_press_stop: bool = False) -> ActionSpec:
    return ActionSpec(name, type_, cls, gestures or _CLASS_GESTURES[cls],
                      _CLASS_MODIFIERS[cls], args, required, implicit_press_stop)


#: The actions a ``[[trigger]]`` may run, by name.
CATALOG: Dict[str, ActionSpec] = {s.name: s for s in (
    _spec("engage.start", "oneshot", START_MOTION, args=("arms",), required=("arms",)),
    _spec("engage.stop", "oneshot", STOP, args=("arms",), required=("arms",)),
    _spec("engage.toggle", "oneshot", START_MOTION, gestures=("tap",),
          args=("arms",), required=("arms",), implicit_press_stop=True),
    _spec("home.run", "oneshot", LARGE_MOTION),
    _spec("recording.start", "oneshot", NO_MOTION),
    _spec("recording.stop", "oneshot", NO_MOTION),
    _spec("recording.toggle", "oneshot", NO_MOTION),
    _spec("recording.abort", "oneshot", NO_MOTION),
    _spec("base.drive", "held", CONTINUOUS_MOTION,
          args=("linear", "angular", "invert_linear", "invert_angular", "deadband",
                "max_linear_m_s", "max_angular_rad_s"),
          required=("linear", "angular")),
    _spec("base.stop", "oneshot", STOP),
)}


def split_address(text: str) -> Tuple[str, Optional[str]]:
    """``"<device>/<channel>"`` to ``(device, channel)``; ``"<device>"`` to
    ``(device, None)``. Does not validate."""
    device, _, channel = text.partition("/")
    return device, (channel or None)


@dataclass(frozen=True)
class DerivedInput:
    """An ``[[input]]``: a button made from an analog value with hysteresis."""

    name: str
    #: ``"<device>/<channel>"`` of the analog value, as written.
    from_: str
    #: The value at or above which the button goes down.
    on: float
    #: The value at or below which it comes up again.
    off: float

    @property
    def device(self) -> str:
        return split_address(self.from_)[0]

    @property
    def channel(self) -> Optional[str]:
        return split_address(self.from_)[1]


@dataclass(frozen=True)
class Trigger:
    """One ``[[trigger]]``: a gesture on an input runs an action."""

    #: The key: an input address (``pedal/#0``) or an ``[[input]]`` name.
    from_: str
    #: The ``do`` action name, an entry of :data:`CATALOG`.
    do: str
    #: The gesture: one of :data:`TRIGGER_GESTURES`.
    on: str
    #: Modifiers: ``[[input]]`` names or button addresses. Empty = unmodified.
    with_: Tuple[str, ...] = ()
    #: ``"before"`` or ``"during"`` for a modified trigger, else ``None``.
    modifiers: Optional[str] = None
    #: Seconds, for ``on = "hold"`` only.
    hold_s: Optional[float] = None
    #: The action's arguments, with the catalog's defaults applied.
    args: Dict[str, Any] = field(default_factory=dict)

    @property
    def spec(self) -> Optional[ActionSpec]:
        return CATALOG.get(self.do)


def _pkg():
    # This module is imported by the package's __init__; reach back into it
    # lazily for the version 1 constants instead of importing in a cycle.
    return sys.modules[__package__]


def _num(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


# ---------------------------------------------------------------------------
# [[source]] additions
# ---------------------------------------------------------------------------

def parse_source_v2(raw: Mapping[str, Any], where: str, problems: List[str]
                    ) -> Tuple[Optional[str], Optional[str],
                               Optional[Dict[str, Tuple[int, ...]]]]:
    """The version 2 keys of one ``[[source]]``: ``(endpoint, companion,
    sides)``. A bad value is reported and comes back ``None``."""
    endpoint = companion = sides = None
    if "endpoint" in raw:
        value = raw["endpoint"]
        m = _ENDPOINT_RE.match(value) if isinstance(value, str) else None
        if m is None or not 1 <= int(m.group("port")) <= 65535:
            problems.append(f"{where}: endpoint must be 'host:port' with a port "
                            f"from 1 to 65535, got {value!r}")
        else:
            endpoint = value
    if "companion" in raw:
        value = raw["companion"]
        if not isinstance(value, str) or not _pkg()._DEVICE_RE.match(value):
            problems.append(f"{where}: companion must be a device id, got {value!r}")
        else:
            companion = value
    if "sides" in raw:
        value = raw["sides"]
        if not isinstance(value, Mapping) or not value:
            problems.append(f"{where}: sides must be a table of "
                            "left/right -> list of control indices")
        else:
            parsed: Dict[str, Tuple[int, ...]] = {}
            ok = True
            for side, idx in value.items():
                if side not in _SIDES:
                    problems.append(f"{where}: sides key {side!r} is not one of {_SIDES}")
                    ok = False
                elif (not isinstance(idx, list) or not idx
                      or any(not isinstance(i, int) or isinstance(i, bool) or i < 0
                             for i in idx)):
                    problems.append(f"{where}: sides.{side} must be a non-empty list of "
                                    f"non-negative control indices, got {idx!r}")
                    ok = False
                elif len(set(idx)) != len(idx):
                    problems.append(f"{where}: sides.{side} lists an index twice")
                    ok = False
                else:
                    parsed[side] = tuple(idx)
            if ok:
                left, right = set(parsed.get("left", ())), set(parsed.get("right", ()))
                if left & right:
                    problems.append(f"{where}: sides left and right share control "
                                    f"index(es) {sorted(left & right)}")
                else:
                    sides = parsed
    return endpoint, companion, sides


def check_sources_v2(sources: Sequence[Any], problems: List[str]) -> None:
    """Cross-source rules of the version 2 ``[[source]]`` keys."""
    pkg = _pkg()
    by_device = {s.device: s for s in sources}
    for s in sources:
        where = f"[[source]] {s.device}"
        if s.companion is not None:
            if s.sides is None:
                problems.append(f"{where}: companion needs sides (which control belongs "
                                "to which hand)")
            target = by_device.get(s.companion)
            if target is None:
                problems.append(f"{where}: companion {s.companion!r} is not a [[source]]")
            elif target.family != CONTROLLERS_FAMILY:
                problems.append(f"{where}: companion {s.companion!r} must be a "
                                f"{CONTROLLERS_FAMILY} source, it is {target.family}")
        if s.sides is not None and s.family != pkg.CONTROLS_FAMILY:
            problems.append(f"{where}: sides applies to a {pkg.CONTROLS_FAMILY} source, "
                            f"this is {s.family}")


# ---------------------------------------------------------------------------
# address helpers
# ---------------------------------------------------------------------------

def _address(text: Any, by_device: Mapping[str, Any]):
    """``(device, channel, source)`` of ``"<device>/<channel>"``, else
    ``None``."""
    if not isinstance(text, str):
        return None
    m = _pkg()._FROM_RE.match(text)
    if m is None or m.group("channel") is None:
        return None
    source = by_device.get(m.group("device"))
    if source is None:
        return None
    return m.group("device"), m.group("channel"), source


def _is_button_channel(channel: str) -> bool:
    return channel.startswith("#")


# ---------------------------------------------------------------------------
# [[input]]
# ---------------------------------------------------------------------------

def _parse_inputs(data: Mapping[str, Any], problems: List[str]) -> List[DerivedInput]:
    raws = data.get("input", [])
    if not isinstance(raws, list):
        problems.append("input must be an array of tables ([[input]])")
        return []
    out: List[DerivedInput] = []
    for i, raw in enumerate(raws):
        where = f"[[input]] #{i + 1}"
        if not isinstance(raw, Mapping):
            problems.append(f"{where}: not a table")
            continue
        extra = set(raw) - _INPUT_KEYS
        if extra:
            problems.append(f"{where}: unknown key(s) {sorted(extra)}")
        name, frm, thr = raw.get("name"), raw.get("from"), raw.get("threshold")
        ok = True
        if not isinstance(name, str) or not _pkg()._DEVICE_RE.match(name):
            problems.append(f"{where}: name must be a non-empty id with no '/', '#' "
                            f"or whitespace, got {name!r}")
            ok = False
        if not isinstance(frm, str):
            problems.append(f"{where}: from must be '<device>/<channel>', got {frm!r}")
            ok = False
        if (not isinstance(thr, Mapping) or set(thr) != {"on", "off"}
                or not _num(thr.get("on")) or not _num(thr.get("off"))):
            problems.append(f"{where}: threshold must be {{ on = <number>, off = <number> }}")
            ok = False
        if ok:
            out.append(DerivedInput(name, frm, float(thr["on"]), float(thr["off"])))
    return out


def check_inputs(inputs: Sequence[DerivedInput], by_device: Mapping[str, Any],
                 problems: List[str]) -> None:
    """Semantic checks of the derived inputs (shared with the v1 converter)."""
    pkg = _pkg()
    seen: set = set()
    for i, inp in enumerate(inputs):
        where = f"[[input]] #{i + 1} ({inp.name})"
        if inp.name in seen:
            problems.append(f"{where}: input name {inp.name!r} is used twice")
        seen.add(inp.name)
        if inp.name in by_device:
            problems.append(f"{where}: name collides with the device id of a [[source]]")
        addr = _address(inp.from_, by_device)
        if addr is None:
            problems.append(f"{where}: from must be '<device>/<channel>' of a [[source]], "
                            f"got {inp.from_!r}")
        elif addr[2].family not in (pkg.LEADERS_FAMILY, pkg.CONTROLS_FAMILY):
            problems.append(f"{where}: from must be a leaders or controls source, "
                            f"{addr[0]!r} is {addr[2].family}")
        if not 0 <= inp.off < inp.on <= 1:
            problems.append(f"{where}: threshold needs 0 <= off < on <= 1, "
                            f"got on={inp.on} off={inp.off}")


# ---------------------------------------------------------------------------
# [[trigger]]
# ---------------------------------------------------------------------------

def _parse_triggers(data: Mapping[str, Any], problems: List[str]) -> List[Trigger]:
    raws = data.get("trigger", [])
    if not isinstance(raws, list):
        problems.append("trigger must be an array of tables ([[trigger]])")
        return []
    out: List[Trigger] = []
    for i, raw in enumerate(raws):
        where = f"[[trigger]] #{i + 1}"
        if not isinstance(raw, Mapping):
            problems.append(f"{where}: not a table")
            continue
        extra = set(raw) - _TRIGGER_KEYS
        if extra:
            problems.append(f"{where}: unknown key(s) {sorted(extra)}")
        ok = True
        for key in ("from", "on", "do"):
            if not isinstance(raw.get(key), str):
                problems.append(f"{where}: {key} is required and must be a string")
                ok = False
        with_: Tuple[str, ...] = ()
        if "with" in raw:
            w = raw["with"]
            if not isinstance(w, list) or not w or any(not isinstance(x, str) for x in w):
                problems.append(f"{where}: with must be a non-empty list of input names "
                                "or button addresses")
                ok = False
            else:
                with_ = tuple(w)
        if "modifiers" in raw and not isinstance(raw["modifiers"], str):
            problems.append(f"{where}: modifiers must be a string")
            ok = False
        args = raw.get("args", {})
        if not isinstance(args, Mapping):
            problems.append(f"{where}: args must be a table")
            ok = False
        hold_s = raw.get("hold_s")
        if "hold_s" in raw and not _num(hold_s):
            problems.append(f"{where}: hold_s must be a number of seconds, got {hold_s!r}")
            ok = False
        if ok:
            out.append(Trigger(from_=raw["from"], do=raw["do"], on=raw["on"],
                               with_=with_, modifiers=raw.get("modifiers"),
                               hold_s=float(hold_s) if hold_s is not None else None,
                               args=dict(args)))
    return out


def _check_args(t: Trigger, spec: ActionSpec, where: str, by_device: Mapping[str, Any],
                inputs: Mapping[str, DerivedInput], problems: List[str]
                ) -> Optional[Dict[str, Any]]:
    """The arguments of ``t`` with defaults applied, or ``None`` if refused."""
    n_before = len(problems)
    extra = set(t.args) - set(spec.args)
    if extra:
        problems.append(f"{where}: {t.do} takes {list(spec.args) or 'no arguments'}, "
                        f"not {sorted(extra)}")
    for name in spec.required_args:
        if name not in t.args:
            problems.append(f"{where}: {t.do} needs the argument {name!r}")
    if spec.name.startswith("engage.") and "arms" in t.args:
        if t.args["arms"] not in ENGAGE_ARMS:
            problems.append(f"{where}: arms must be one of {ENGAGE_ARMS}, "
                            f"got {t.args['arms']!r}")
    out = {k: v for k, v in t.args.items() if k in spec.args}
    if spec.name == "base.drive":
        for k in ("invert_linear", "invert_angular"):
            if k in t.args and not isinstance(t.args[k], bool):
                problems.append(f"{where}: {k} must be true or false")
        if "deadband" in t.args:
            d = t.args["deadband"]
            if not _num(d) or not 0 <= d < 1:
                problems.append(f"{where}: deadband must be a number with 0 <= deadband < 1")
        for k in ("max_linear_m_s", "max_angular_rad_s"):
            if k in t.args and (not _num(t.args[k]) or not t.args[k] > 0):
                problems.append(f"{where}: {k} must be a positive number")
        for k, v in BASE_DRIVE_DEFAULTS.items():
            out.setdefault(k, v)
        # The axes: on the deadman's companion when it has one, else on any
        # controllers-family source.
        key = inputs[t.from_].from_ if t.from_ in inputs else t.from_
        key_dev = split_address(key)[0]
        companion = getattr(by_device.get(key_dev), "companion", None)
        for axis in ("linear", "angular"):
            if axis not in t.args:
                continue
            addr = _address(t.args[axis], by_device)
            if addr is None or _is_button_channel(addr[1]):
                problems.append(f"{where}: {axis} must be an axis address "
                                f"'<device>/<channel>' of a [[source]], got {t.args[axis]!r}")
            elif addr[2].family != CONTROLLERS_FAMILY:
                problems.append(f"{where}: {axis} axis {t.args[axis]!r} must be on a "
                                f"{CONTROLLERS_FAMILY} source")
            elif companion is not None and addr[0] != companion:
                problems.append(f"{where}: the deadman {key!r} has companion "
                                f"{companion!r}, so {axis} must be on it, not on "
                                f"{addr[0]!r}")
    return out if len(problems) == n_before else None


def validate_triggers(triggers: Sequence[Trigger], inputs: Sequence[DerivedInput],
                      by_device: Mapping[str, Any], problems: List[str]) -> List[Trigger]:
    """Check ``triggers`` against the sources and inputs; every problem is
    appended to ``problems``. Returns the triggers with the catalog's argument
    defaults applied, in the same order."""
    pkg = _pkg()
    by_input = {inp.name: inp for inp in inputs}
    key_families = (pkg.CONTROLS_FAMILY, CONTROLLERS_FAMILY, pkg.LEADERS_FAMILY)

    def underlying(text: str) -> str:
        return by_input[text].from_ if text in by_input else text

    result: List[Trigger] = []
    known: List[Tuple[str, Trigger, ActionSpec, str]] = []  # (canon key, t, spec, where)
    for i, t in enumerate(triggers):
        where = f"[[trigger]] #{i + 1} ({t.from_} {t.on} -> {t.do})"
        n_before = len(problems)
        spec = CATALOG.get(t.do)
        if spec is None:
            problems.append(f"{where}: unknown action {t.do!r}; the catalog has "
                            f"{sorted(CATALOG)}")

        # the key
        if t.from_ in by_input:
            pass
        elif "/" not in t.from_:
            problems.append(f"{where}: from {t.from_!r} is neither an [[input]] name nor "
                            "'<device>/<channel>'")
        else:
            addr = _address(t.from_, by_device)
            if addr is None:
                problems.append(f"{where}: from must be '<device>/<channel>' of a "
                                f"[[source]], got {t.from_!r}")
            elif addr[2].family not in key_families:
                problems.append(f"{where}: from must be on a controls, controllers or "
                                f"leaders source; {addr[0]!r} is {addr[2].family}")

        # the gesture
        if t.on not in TRIGGER_GESTURES:
            problems.append(f"{where}: on must be one of {TRIGGER_GESTURES}, got {t.on!r}")
        elif spec is not None and t.on not in spec.gestures:
            problems.append(f"{where}: {t.do} is {spec.safety_class} and allows only "
                            f"{list(spec.gestures)}, not {t.on!r}")
        if t.on == "hold":
            if t.hold_s is None:
                problems.append(f"{where}: on = 'hold' needs hold_s")
            elif not t.hold_s > 0:
                problems.append(f"{where}: hold_s must be a positive number of seconds")
            elif (spec is not None and spec.safety_class == LARGE_MOTION
                    and t.hold_s < LARGE_MOTION_MIN_HOLD_S):
                problems.append(f"{where}: {t.do} is large_motion; hold_s must be at "
                                f"least {LARGE_MOTION_MIN_HOLD_S} s, got {t.hold_s}")
        elif t.hold_s is not None:
            problems.append(f"{where}: hold_s applies to on = 'hold' only")

        # the modifiers
        if t.with_:
            if t.modifiers is None:
                problems.append(f"{where}: a trigger with 'with' must say modifiers = "
                                f"{' | '.join(repr(m) for m in MODIFIER_READS)}")
            elif t.modifiers not in MODIFIER_READS:
                problems.append(f"{where}: modifiers must be one of {MODIFIER_READS}, "
                                f"got {t.modifiers!r}")
            if spec is not None and not spec.modifiers_allowed:
                problems.append(f"{where}: {t.do} is {spec.safety_class}; it takes no "
                                "modifiers")
            if len(set(t.with_)) != len(t.with_):
                problems.append(f"{where}: with lists a modifier twice")
            key_canon = underlying(t.from_)
            for m in t.with_:
                if underlying(m) == key_canon:
                    problems.append(f"{where}: {m!r} is the key itself and cannot be "
                                    "its own modifier")
                if m in by_input:
                    continue
                addr = _address(m, by_device)
                if addr is None:
                    problems.append(f"{where}: modifier {m!r} is neither an [[input]] "
                                    "name nor a '<device>/<channel>' of a [[source]]")
                elif (addr[2].family != pkg.CONTROLS_FAMILY
                      or not _is_button_channel(addr[1])):
                    problems.append(f"{where}: modifier {m!r} is an analog or axis "
                                    "channel; wrap it in an [[input]] with a threshold")
        elif t.modifiers is not None:
            problems.append(f"{where}: modifiers is written but the trigger has no "
                            "'with'")

        args = t.args
        if spec is not None:
            checked = _check_args(t, spec, where, by_device, by_input, problems)
            if checked is not None:
                args = checked
        result.append(replace(t, args=args))
        if spec is not None:
            known.append((underlying(t.from_), t, spec, where))

    # ---- rules over several triggers ----
    groups: Dict[Tuple[str, FrozenSet[str]], List[Tuple[Trigger, ActionSpec]]] = {}
    by_key: Dict[str, List[Tuple[Trigger, ActionSpec]]] = {}
    for key, t, spec, _ in known:
        groups.setdefault((key, frozenset(underlying(m) for m in t.with_)), []
                          ).append((t, spec))
        by_key.setdefault(key, []).append((t, spec))

    for (key, mods), items in groups.items():
        label = f"{key}" + (f" with {sorted(mods)}" if mods else "")
        seen: Dict[Tuple[str, Optional[float]], str] = {}
        for t, _ in items:
            ident = (t.on, t.hold_s)
            if ident in seen:
                what = (f"hold_s {t.hold_s}" if t.on == "hold" else repr(t.on))
                problems.append(f"{label}: two triggers on the same gesture ({what}): "
                                f"{seen[ident]} and {t.do}")
            seen.setdefault(ident, t.do)
        if mods and sum(1 for t, _ in items if t.on == "hold") > 1:
            problems.append(f"{label}: a modified key allows one hold only "
                            "(it fires at its threshold, with no stages)")
        if any(t.on == "while" for t, _ in items) and any(t.on != "while" for t, _ in items):
            problems.append(f"{label}: a 'while' trigger cannot share its key and "
                            "modifiers with another gesture")

    for key, items in by_key.items():
        reads = {t.modifiers for t, _ in items if t.with_ and t.modifiers in MODIFIER_READS}
        if len(reads) > 1:
            problems.append(f"{key}: modifiers mixes 'before' and 'during'; a key has "
                            "one reading")
        stops = [t for t, s in items if s.safety_class == STOP and not t.with_]
        starts = [(t, s) for t, s in items if s.safety_class in (START_MOTION, LARGE_MOTION)
                  and not s.implicit_press_stop]  # engage.toggle is exempt
        if stops and starts:
            problems.append(f"{key}: has an unmodified stop trigger ({stops[0].do}) and "
                            f"also {starts[0][0].do} ({starts[0][1].safety_class}); a key "
                            "that stops must not also start motion")

    hold_motion_keys = {key for key, t, s, _ in known
                        if t.on == "hold" and s.safety_class in (START_MOTION, LARGE_MOTION)}
    reported: set = set()
    for _, t, _, where in known:
        for m in t.with_:
            canon = underlying(m)
            if canon in hold_motion_keys and canon not in reported:
                reported.add(canon)
                problems.append(f"{where}: {m!r} is a modifier but is also the key of a "
                                "hold trigger that starts motion")
    return result


def parse_v2_sections(data: Mapping[str, Any], by_device: Mapping[str, Any],
                      problems: List[str]
                      ) -> Tuple[Tuple[DerivedInput, ...], Tuple[Trigger, ...]]:
    """The ``[[input]]`` and ``[[trigger]]`` tables of a version 2 document.
    Problems are appended to ``problems``."""
    inputs = _parse_inputs(data, problems)
    check_inputs(inputs, by_device, problems)
    triggers = _parse_triggers(data, problems)
    triggers = validate_triggers(triggers, inputs, by_device, problems)
    return tuple(inputs), tuple(triggers)


# ---------------------------------------------------------------------------
# v1 -> v2
# ---------------------------------------------------------------------------

def v1_action_triggers(binding: Any) -> Tuple[Tuple[DerivedInput, ...], Tuple[Trigger, ...]]:
    """The ``action/<name>`` binds of a version 1 :class:`Binding` as version 2
    ``([[input]], [[trigger]])`` objects.

    * ``action/engage`` ``start`` / ``stop`` / ``toggle`` become
      ``engage.start`` / ``.stop`` / ``.toggle`` with ``arms = "both"``;
    * ``action/home`` ``run`` becomes ``home.run`` (``hold_s`` of the bind);
    * ``action/recording`` verbs map one to one;
    * ``action/engage`` ``one_arm`` on a leader channel ``<device>/<side>_gripper``
      becomes a derived input on that channel (threshold on 0.8, off 0.6) and,
      on the control key of the engage switch (the bind whose map has
      ``tap = "toggle"``), a ``during`` modifier trigger ``hold`` 2.0 s
      ``engage.start``. The server's meaning is kept: the side whose gripper is
      held stays still and the OTHER arm engages, so the gripper
      ``left_gripper`` gives ``arms = "right"``.

    The result is validated like a parsed version 2 file, so a v1 binding that
    has no version 2 form (a ``hold_s`` under the large-motion floor, a
    ``one_arm`` with no engage switch) raises :class:`BindingError` listing
    every reason."""
    pkg = _pkg()
    by_device = {s.device: s for s in binding.sources}
    problems: List[str] = []
    action_binds = [b for b in binding.binds if b.kind == "action"]
    switches = [b for b in action_binds
                if b.to == "action/engage" and b.gestures and b.map.get("tap") == "toggle"
                and getattr(by_device.get(b.device), "family", None) == pkg.CONTROLS_FAMILY]
    inputs: Dict[str, DerivedInput] = {}
    triggers: List[Trigger] = []
    for b in action_binds:
        name = b.to[len("action/"):]
        where = f"{b.from_} -> {b.to}"
        if not b.gestures or not b.map:
            problems.append(f"{where}: only a gesture map converts to triggers")
            continue
        for gesture, verb in b.map.items():
            hold_s = b.hold_s if gesture == "hold" else None
            if name == "engage" and verb == "one_arm":
                m = _ONE_ARM_CHANNEL_RE.match(b.channel or "")
                family = getattr(by_device.get(b.device), "family", None)
                if m is None or family != pkg.LEADERS_FAMILY or gesture != "hold":
                    problems.append(f"{where}: one_arm converts only from a hold on a "
                                    "leader channel '<device>/<left|right>_gripper'")
                    continue
                if not switches:
                    problems.append(f"{where}: one_arm needs an engage switch (a "
                                    "controls bind to action/engage with tap = 'toggle')")
                    continue
                side = m.group(1)
                inp = DerivedInput(f"{b.device}-{side}-grip", b.from_, *ONE_ARM_THRESHOLD)
                inputs.setdefault(inp.name, inp)
                other = "right" if side == "left" else "left"
                for sw in switches:
                    triggers.append(Trigger(
                        from_=sw.from_, do="engage.start", on="hold", with_=(inp.name,),
                        modifiers="during", hold_s=ONE_ARM_HOLD_S, args={"arms": other}))
                continue
            if name == "engage":
                do, args = f"engage.{verb}", {"arms": "both"}
            elif name == "home":
                do, args = "home.run", {}
            elif name == "recording":
                do, args = f"recording.{verb}", {}
            else:  # pragma: no cover - v1 parsing refuses other action names
                problems.append(f"{where}: no version 2 action for {name!r}")
                continue
            triggers.append(Trigger(from_=b.from_, do=do, on=gesture, hold_s=hold_s,
                                    args=args))
    input_list = list(inputs.values())
    check_inputs(input_list, by_device, problems)
    result = validate_triggers(triggers, input_list, by_device, problems)
    if problems:
        raise pkg.BindingError(problems, "v1 to v2 migration")
    return tuple(input_list), tuple(result)


__all__ = ["BASE_DRIVE_DEFAULTS", "BINDING_SCHEMA_V2", "CATALOG", "CONTROLLERS_FAMILY",
           "ENGAGE_ARMS", "LARGE_MOTION_MIN_HOLD_S", "MODIFIER_READS", "ONE_ARM_HOLD_S",
           "TRIGGER_GESTURES", "ActionSpec", "DerivedInput", "Trigger",
           "split_address", "v1_action_triggers"]
