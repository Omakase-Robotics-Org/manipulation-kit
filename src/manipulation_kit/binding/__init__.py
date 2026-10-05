"""``omakase.binding.v1`` — which input device drives which robot output.

A binding file is TOML, kept on the robot, and read once when a teleoperation
server starts. It names the input devices (``[[source]]``) and, for each
robot output, the one thing that drives it (``[[bind]]``)::

    schema = "omakase.binding.v1"

    [[source]]
    device = "udcap-right"                    # the publisher's device id
    family = "teleop_gloves.pose_stream.v1"

    [[source]]
    device = "pedal"
    family = "teleop_controls.event_stream.v1"
    instance = 0

    # A whole hand bound to one glove. The hand's retarget map decides which
    # glove channels drive which joint; this file only says which glove.
    [[bind]]
    from        = "udcap-right"
    to          = "hand/right"
    retarget    = "robotera/xhand1"           # optional: else the fitted hand's model
    calibration = "operators/alice-udcap.json" # optional: relative to this file

    # Optional: override fields of that hand map's RetargetConfig (needs
    # `retarget`). Checked against the dataclass at load; see
    # manipulation_kit.binding.retarget_config for the value shapes.
    [bind.retarget_config]
    ema_alpha = 0.2
    [bind.retarget_config.joints.thumb_rota1]
    sources = { thumb_mp_pitch = 1.0 }
    lo = 0.12

    # A pedal's gestures to an action of the teleoperation server itself.
    [[bind]]
    from   = "pedal/#1"
    to     = "action/recording"
    map    = { tap = "toggle", hold = "abort" }
    hold_s = 0.8                              # optional; default 0.5

    # A simple output bound 1:1 to one input value.
    [[bind]]
    from  = "pedal/#0"
    to    = "gripper/left/openness"
    range = { lo = 0.0, hi = 1.0, invert = true }

Two kinds of bind
-----------------
**Whole hand.** ``to = "hand/<left|right>"`` and ``from = "<device_id>"`` —
no channel. A dexterous hand is driven as a unit, by one glove, through the
hand's own retarget map (:mod:`manipulation_kit.hands`), so N gloves and M
hands cost N + M maps instead of N x M per-axis lines. ``calibration`` names
the operator's glove calibration file
(:mod:`manipulation_kit.gloves.calibration`); it is parsed here, at startup.

**One to one.** ``from = "<device_id>/<channel>"`` (``<device_id>/#<index>``
for a family that numbers its controls instead of naming them) and ``to`` an
output path:

* ``<kind>/<left|right>[/<axis>]`` — a device on the robot (``gripper``,
  ``hand``, …), existence checked against ``available_outputs``;
* ``accessory/<name>/<channel>`` — an external device registered with
  ``d1-firmwared`` under ``<name>``, likewise checked;
* ``action/<name>`` — not a device: an action the teleoperation server
  performs itself (:data:`ACTIONS`; today ``action/recording``). It is not
  checked against ``available_outputs``, because the robot does not have it;
  it is checked against :data:`ACTIONS` instead, and its ``map`` values must be
  that action's verbs. It is a kind of its own rather than
  ``accessory/recorder/...`` because an accessory is a device the daemon
  registers and reports (``GET /v1/accessories``): a recorder there would be
  looked up on the robot, refused as absent, and could collide with a real
  accessory of that name.

``range`` maps a continuous value to [0, 1] with the same
``lo``/``hi``/``invert`` semantics as a glove channel range. ``map`` maps to
a verb or a value, keyed EITHER by discrete input values (integers, e.g. a
pedal's level ``{ 0 = 0, 2 = 1 }``) OR — for a ``teleop_controls`` source
only — by gestures (:data:`GESTURES`: ``press``, ``release``, ``tap``,
``hold``), never a mix. A gesture map that uses ``tap`` or ``hold`` splits
them at ``hold_s`` seconds (the bind's, else :data:`DEFAULT_HOLD_S`); the
split is in the file, not left to a consumer, so two consumers of one
binding split a press the same way. These 1:1 binds remain for simple
outputs — a pedal to a gripper's openness or to recording — not for the
joints of a hand.

Startup refusals
----------------
:func:`load_binding` / :func:`parse_binding` raise :class:`BindingError`
listing EVERY problem at once (so one restart fixes them all):

* two binds to the same output. ``hand/right`` and ``hand/right/<axis>``
  count as the same output, and so does any pair where one path is a prefix
  of the other: a hand bound whole cannot also have one finger bound
  elsewhere. The message names both ``from`` values;
* a ``from`` whose device is not a ``[[source]]``;
* a ``range`` with ``lo == hi``;
* an output the live daemon does not have — checked only when the caller
  passes ``available_outputs`` (this package opens no socket; the caller asks
  ``d1-firmwared`` what is fitted and passes the paths it found);
* a whole-hand bind whose source is not a glove pose stream, or whose
  ``retarget`` names no hand map in this package;
* a ``retarget_config`` without a resolvable ``retarget``, or with a field
  the hand's ``RetargetConfig`` does not have, a value of the wrong type or
  length, an unknown joint or channel name, or values the config's own
  checks refuse (an XHAND1 interval outside a joint's limits);
* a ``map`` key that is neither an integer nor a gesture, a map mixing the
  two, gestures from a non-controls source, a ``hold_s`` that is not a
  positive number or has no ``tap``/``hold`` to split, an ``action/<name>``
  that is not in :data:`ACTIONS`, or an action ``map`` value that is not one
  of that action's verbs;
* a malformed path, an unknown key (a typo is otherwise a setting that
  silently never applies), a wrong ``schema``.

What this module does not do is run anything: it returns a validated
:class:`Binding` and the consumer arbitrates at run time. A channel that
disappears from a device's declaration while running stops only its own bind,
and is never filled with 0.0.
"""
from __future__ import annotations

import importlib
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import (Any, Dict, Iterable, List, Mapping, Optional, Tuple, Union)

from ..gloves.calibration import ChannelRange, load_ranges
from .retarget_config import build_retarget_config

#: The ``schema`` string a binding file carries. Refused if different.
BINDING_SCHEMA = "omakase.binding.v1"

#: The glove pose stream family; the only source a whole hand binds to.
GLOVE_FAMILY = "teleop_gloves.pose_stream.v1"

SIDES = ("left", "right")

#: The switch/dial family (pedals, MIDI pads, button boxes): the only source
#: a GESTURE map may bind, because only it reports a control going down and up.
CONTROLS_FAMILY = "teleop_controls.event_stream.v1"

#: The gesture vocabulary of a controls-family ``map``. One press of a
#: control produces, in order: ``press`` (it went down), then exactly ONE of
#: ``tap`` (it came up before ``hold_s`` elapsed) or ``hold`` (it was still
#: down when ``hold_s`` elapsed — fired then, not at release), then
#: ``release`` (it came up, after either). ``tap`` and ``hold`` therefore never
#: both fire for one press, which is what lets one pedal carry two verbs.
GESTURES = ("press", "release", "tap", "hold")

#: Seconds a control must stay down to be a ``hold`` rather than a ``tap``,
#: when the bind gives no ``hold_s``. A file-level fact, not a consumer
#: choice: two consumers reading the same binding must split the same press
#: the same way.
DEFAULT_HOLD_S = 0.5

#: Outputs that are not devices on the robot but actions the teleoperation
#: server performs itself, addressed ``action/<name>``, with the verbs each
#: accepts as ``map`` values. ``recording``: ``start`` an episode, ``stop``
#: it and keep it, ``toggle`` (start when idle, stop when recording),
#: ``abort`` (stop it and discard it).
ACTIONS = {
    "recording": ("start", "stop", "toggle", "abort"),
}

#: Human-facing side names to ``d1-firmwared``'s arm sides: ``a`` is the
#: robot's physical left arm, ``b`` its right.
DAEMON_SIDE = {"left": "a", "right": "b"}

_ID = r"[A-Za-z0-9][A-Za-z0-9_.\-]*"
# A device id is whatever the driver publishes as its device_id: the stream
# libraries impose no character set (the UDCAP glove driver emits
# ``omakase@5555``, role@udp_port). Only the binding's own separators are
# refused -- ``/`` (device/channel) and ``#`` (``<device>/#<index>``, and the
# session suffix of a stream ``source``) -- plus whitespace and control
# characters, which no driver emits and a typo often does.
_DEVICE = r"[^/#\s\x00-\x1f\x7f]+"
_DEVICE_RE = re.compile(rf"^{_DEVICE}$")
_FROM_RE = re.compile(rf"^(?P<device>{_DEVICE})(?:/(?P<channel>#\d+|{_ID}))?$")
_OUTPUT_RE = re.compile(rf"^(?P<kind>{_ID})/(?P<rest>{_ID}(?:/{_ID})*)$")
_MODEL_RE = re.compile(rf"^{_ID}/{_ID}$")

_SOURCE_KEYS = {"device", "family", "instance"}
_BIND_KEYS = {"from", "to", "range", "map", "retarget", "calibration",
              "retarget_config", "hold_s"}


class BindingError(ValueError):
    """A binding that must not start. ``problems`` lists every reason."""

    def __init__(self, problems: List[str], origin: str = "binding"):
        self.problems = list(problems)
        super().__init__(f"{origin}: {len(problems)} problem(s):\n  - "
                         + "\n  - ".join(problems))


@dataclass(frozen=True)
class Source:
    """One input device."""

    device: str
    family: str
    instance: Optional[int] = None


@dataclass(frozen=True)
class Bind:
    """One output and what drives it."""

    #: The ``from`` string as written.
    from_: str
    #: The output path, e.g. ``hand/right`` or ``gripper/left/openness``.
    to: str
    #: The source device id (the part of ``from`` before ``/``).
    device: str
    #: The channel (``index_mp_pitch``, ``#0``), or ``None`` for a whole hand.
    channel: Optional[str]
    range: Optional[ChannelRange] = None
    #: Discrete input value -> verb or value.
    map: Optional[Dict[Any, Any]] = None
    #: ``"<maker>/<model>"`` of the hand map, for a whole-hand bind.
    retarget: Optional[str] = None
    #: Absolute path of the operator's glove calibration file.
    calibration: Optional[Path] = None
    #: Its parsed overrides (channel -> range), loaded at startup.
    calibration_ranges: Dict[str, ChannelRange] = field(default_factory=dict)
    #: Whether ``map`` is keyed by gestures (:data:`GESTURES`) rather than by
    #: discrete input values.
    gestures: bool = False
    #: The tap/hold split in seconds for a gesture map that uses either, else
    #: ``None``: the bind's ``hold_s`` or :data:`DEFAULT_HOLD_S`.
    hold_s: Optional[float] = None
    #: The ``retarget_config`` table as written (field -> value).
    retarget_overrides: Dict[str, Any] = field(default_factory=dict)
    #: The hand map's ``RetargetConfig`` built from those overrides, or
    #: ``None`` when the bind gives none (the hand's defaults apply).
    retarget_config: Any = None

    def build_retarget(self, model: Optional[str] = None):
        """The hand map for this bind: ``get_retarget(model, config)``.

        ``model`` defaults to the bind's ``retarget``; pass the fitted hand's
        model (the daemon descriptor's ``model``) when the bind names none.
        A ``retarget_config`` was validated against ``retarget``'s config,
        so it is refused for any other model."""
        from ..hands import get_retarget  # noqa: PLC0415
        model = model or self.retarget
        if model is None:
            raise ValueError(f"{self.from_} -> {self.to}: no retarget model named")
        if self.retarget_config is not None and model != self.retarget:
            raise ValueError(f"{self.from_} -> {self.to}: retarget_config was built "
                             f"for {self.retarget}, not {model}")
        return get_retarget(model, config=self.retarget_config)

    @property
    def whole_hand(self) -> bool:
        return self.channel is None

    @property
    def side(self) -> Optional[str]:
        """``left``/``right`` for a ``<kind>/<side>…`` output, else ``None``."""
        parts = self.to.split("/")
        return parts[1] if len(parts) > 1 and parts[1] in SIDES else None


@dataclass(frozen=True)
class Binding:
    schema: str
    sources: Tuple[Source, ...]
    binds: Tuple[Bind, ...]

    def source(self, device: str) -> Optional[Source]:
        return next((s for s in self.sources if s.device == device), None)

    def hand_binds(self) -> Tuple[Bind, ...]:
        return tuple(b for b in self.binds if b.whole_hand)


def _overlaps(a: str, b: str) -> bool:
    return a == b or a.startswith(b + "/") or b.startswith(a + "/")


def _parse_map(raw: Mapping[str, Any]) -> Dict[Any, Any]:
    # TOML table keys are always strings; a pedal level is an integer.
    out: Dict[Any, Any] = {}
    for key, value in raw.items():
        out[int(key) if re.fullmatch(r"-?\d+", key) else key] = value
    return out


def _resolves_retarget(model: str) -> Optional[str]:
    maker, _, name = model.partition("/")
    try:
        mod = importlib.import_module(f"manipulation_kit.hands.{maker}.{name}.retarget")
    except ImportError:
        return f"retarget {model!r} names no hand map in manipulation_kit.hands"
    if not hasattr(mod, "build_retarget"):
        return f"retarget {model!r} has no build_retarget()"
    return None


def parse_binding(data: Mapping[str, Any], *,
                  base_dir: Union[str, Path, None] = None,
                  available_outputs: Optional[Iterable[str]] = None,
                  origin: str = "binding") -> Binding:
    """Validate an already-parsed binding document.

    ``base_dir`` resolves relative ``calibration`` paths (default: the current
    directory). ``available_outputs``, when given, is every output path the
    live daemon has — for example ``{"hand/right", "gripper/left",
    "gripper/left/openness"}`` — and a bind to anything else is refused. Pass
    the whole output AND each axis path it exposes; matching is exact."""
    problems: List[str] = []
    base = Path(base_dir) if base_dir is not None else Path.cwd()

    schema = data.get("schema")
    if schema != BINDING_SCHEMA:
        problems.append(f"schema must be {BINDING_SCHEMA!r}, got {schema!r}")
    unknown_top = set(data) - {"schema", "source", "bind"}
    if unknown_top:
        problems.append(f"unknown top-level key(s) {sorted(unknown_top)}")

    sources: List[Source] = []
    for i, raw in enumerate(data.get("source", []) or []):
        where = f"[[source]] #{i + 1}"
        if not isinstance(raw, Mapping):
            problems.append(f"{where}: not a table")
            continue
        extra = set(raw) - _SOURCE_KEYS
        if extra:
            problems.append(f"{where}: unknown key(s) {sorted(extra)}")
        device, family, instance = raw.get("device"), raw.get("family"), raw.get("instance")
        if not isinstance(device, str) or not _DEVICE_RE.match(device):
            problems.append(f"{where}: device must be a non-empty id with no '/', '#' "
                            f"or whitespace, got {device!r}")
            continue
        if not isinstance(family, str) or not family:
            problems.append(f"{where} {device}: family must be a string")
            continue
        if instance is not None and (isinstance(instance, bool)
                                     or not isinstance(instance, int)):
            problems.append(f"{where} {device}: instance must be an integer")
            continue
        if any(s.device == device for s in sources):
            problems.append(f"{where}: device {device!r} is declared twice")
            continue
        sources.append(Source(device, family, instance))
    by_device = {s.device: s for s in sources}

    available = None if available_outputs is None else set(available_outputs)
    binds: List[Bind] = []
    for i, raw in enumerate(data.get("bind", []) or []):
        where = f"[[bind]] #{i + 1}"
        if not isinstance(raw, Mapping):
            problems.append(f"{where}: not a table")
            continue
        extra = set(raw) - _BIND_KEYS
        if extra:
            problems.append(f"{where}: unknown key(s) {sorted(extra)}")
        frm, to = raw.get("from"), raw.get("to")
        fm = _FROM_RE.match(frm) if isinstance(frm, str) else None
        om = _OUTPUT_RE.match(to) if isinstance(to, str) else None
        if fm is None:
            problems.append(f"{where}: from must be '<device>' or "
                            f"'<device>/<channel>', got {frm!r}")
        if om is None:
            problems.append(f"{where}: to must be an output path such as "
                            f"'hand/right', got {to!r}")
        if fm is None or om is None:
            continue
        where = f"{where} ({frm} -> {to})"
        device, channel = fm.group("device"), fm.group("channel")
        kind, rest = om.group("kind"), om.group("rest").split("/")
        if kind == "accessory":
            if len(rest) != 2:
                problems.append(f"{where}: an accessory output is "
                                "'accessory/<name>/<channel>'")
        elif kind == "action":
            if len(rest) != 1 or rest[0] not in ACTIONS:
                problems.append(f"{where}: an action output is 'action/<name>' with "
                                f"<name> one of {sorted(ACTIONS)}")
        elif rest[0] not in SIDES:
            problems.append(f"{where}: side must be one of {SIDES}, got {rest[0]!r}")

        source = by_device.get(device)
        if source is None:
            problems.append(f"{where}: device {device!r} is not a [[source]]")

        whole_hand = channel is None
        if whole_hand and not (kind == "hand" and len(rest) == 1):
            problems.append(f"{where}: a 'from' with no channel binds a whole hand, "
                            "so 'to' must be 'hand/<left|right>'")
        if kind == "hand" and len(rest) == 1 and not whole_hand:
            problems.append(f"{where}: a whole hand is bound to a device, not to "
                            "one channel; drop the channel or bind an axis")
        if whole_hand and source is not None and source.family != GLOVE_FAMILY:
            problems.append(f"{where}: a whole hand binds to a {GLOVE_FAMILY} "
                            f"source; {device!r} is {source.family}")

        rng = None
        if "range" in raw:
            try:
                rng = ChannelRange.from_json(raw["range"])
            except ValueError as exc:
                problems.append(f"{where}: range: {exc}")
        mapping = None
        if "map" in raw:
            if not isinstance(raw["map"], Mapping) or not raw["map"]:
                problems.append(f"{where}: map must be a non-empty table")
            else:
                mapping = _parse_map(raw["map"])
        if "range" in raw and "map" in raw:
            problems.append(f"{where}: give range OR map, not both")
        if whole_hand and ("range" in raw or "map" in raw):
            problems.append(f"{where}: a whole-hand bind takes no range or map; "
                            "per-channel ranges go in its calibration file")

        gestures, hold_s = False, None
        if mapping is not None:
            keys = list(mapping)
            named = [k for k in keys if isinstance(k, str)]
            unknown = [k for k in named if k not in GESTURES]
            if unknown:
                problems.append(f"{where}: map key(s) {unknown} are neither a "
                                f"discrete value (an integer) nor a gesture {GESTURES}")
            elif named and len(named) != len(keys):
                problems.append(f"{where}: a map is either all gestures or all "
                                "discrete values, not both")
            elif named:
                gestures = True
                if source is not None and source.family != CONTROLS_FAMILY:
                    problems.append(f"{where}: gestures need a {CONTROLS_FAMILY} "
                                    f"source; {device!r} is {source.family}")
        if "hold_s" in raw:
            value = raw["hold_s"]
            if not gestures or not ({"tap", "hold"} & set(mapping or {})):
                problems.append(f"{where}: hold_s applies to a gesture map with "
                                "tap or hold")
            elif isinstance(value, bool) or not isinstance(value, (int, float)) \
                    or not value > 0:
                problems.append(f"{where}: hold_s must be a number of seconds > 0, "
                                f"got {value!r}")
            else:
                hold_s = float(value)
        elif gestures and {"tap", "hold"} & set(mapping):
            hold_s = DEFAULT_HOLD_S

        if kind == "action" and len(rest) == 1 and rest[0] in ACTIONS:
            verbs = ACTIONS[rest[0]]
            if mapping is None:
                problems.append(f"{where}: an action is driven by a map to its verbs "
                                f"{verbs}")
            else:
                bad = [v for v in mapping.values() if v not in verbs]
                if bad:
                    problems.append(f"{where}: {bad} are not verbs of {to} {verbs}")

        retarget = raw.get("retarget")
        if retarget is not None:
            if not whole_hand:
                problems.append(f"{where}: retarget applies to a whole-hand bind only")
            elif not isinstance(retarget, str) or not _MODEL_RE.match(retarget):
                problems.append(f"{where}: retarget must be '<maker>/<model>', "
                                f"got {retarget!r}")
            else:
                why = _resolves_retarget(retarget)
                if why:
                    problems.append(f"{where}: {why}")

        overrides, built = raw.get("retarget_config"), None
        if overrides is not None:
            if not whole_hand:
                problems.append(f"{where}: retarget_config applies to a whole-hand "
                                "bind only")
            elif not isinstance(overrides, Mapping):
                problems.append(f"{where}: retarget_config must be a table")
            elif not isinstance(retarget, str) or not _MODEL_RE.match(retarget) \
                    or _resolves_retarget(retarget):
                problems.append(f"{where}: retarget_config needs a resolvable "
                                "'retarget' to be checked against")
            else:
                built, why = build_retarget_config(
                    retarget, overrides, f"{where}: retarget_config")
                problems.extend(why)

        calibration, cal_ranges = None, {}
        if raw.get("calibration") is not None:
            if not whole_hand:
                problems.append(f"{where}: calibration applies to a whole-hand bind only")
            elif not isinstance(raw["calibration"], str):
                problems.append(f"{where}: calibration must be a path string")
            else:
                calibration = (base / raw["calibration"]).resolve()
                try:
                    cal_ranges = load_ranges(calibration)
                except (OSError, ValueError) as exc:
                    problems.append(f"{where}: calibration: {exc}")

        if available is not None and kind != "action" and to not in available:
            problems.append(f"{where}: the robot has no output {to!r}")

        binds.append(Bind(from_=frm, to=to, device=device, channel=channel,
                          range=rng, map=mapping,
                          retarget=retarget if isinstance(retarget, str) else None,
                          calibration=calibration, calibration_ranges=cal_ranges,
                          retarget_overrides=dict(overrides)
                          if isinstance(overrides, Mapping) else {},
                          retarget_config=built, gestures=gestures,
                          hold_s=hold_s))

    for i, a in enumerate(binds):
        for b in binds[i + 1:]:
            if _overlaps(a.to, b.to):
                problems.append(f"two binds drive the same output: {a.from_} -> "
                                f"{a.to} and {b.from_} -> {b.to}")

    if problems:
        raise BindingError(problems, origin)
    return Binding(schema=BINDING_SCHEMA, sources=tuple(sources), binds=tuple(binds))


def _toml_loads(text: str) -> Dict[str, Any]:
    try:
        import tomllib  # noqa: PLC0415  (3.11+)
    except ImportError:  # pragma: no cover - exercised on 3.9/3.10
        try:
            import tomli as tomllib  # noqa: PLC0415
        except ImportError as exc:
            raise ImportError(
                "reading a binding file on Python < 3.11 needs 'tomli': "
                "pip install 'manipulation-kit[binding]'") from exc
    return tomllib.loads(text)


def load_binding(path: Union[str, Path], *,
                 available_outputs: Optional[Iterable[str]] = None) -> Binding:
    """Read, parse and validate a binding file. Relative ``calibration``
    paths resolve against the file's directory."""
    path = Path(path)
    try:
        data = _toml_loads(path.read_text(encoding="utf-8"))
    except ValueError as exc:  # tomllib.TOMLDecodeError is a ValueError
        raise BindingError([f"not valid TOML: {exc}"], str(path)) from exc
    return parse_binding(data, base_dir=path.parent,
                         available_outputs=available_outputs, origin=str(path))


__all__ = ["ACTIONS", "BINDING_SCHEMA", "Bind", "Binding", "BindingError",
           "CONTROLS_FAMILY", "DAEMON_SIDE", "DEFAULT_HOLD_S", "GESTURES",
           "GLOVE_FAMILY", "Source", "build_retarget_config", "load_binding",
           "parse_binding"]
