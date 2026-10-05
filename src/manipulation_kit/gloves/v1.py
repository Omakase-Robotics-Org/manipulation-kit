"""Decode ``teleop_gloves.pose_stream.v1`` into per-hand radians.

What arrives on the wire
------------------------
A glove driver sends JSON objects (one per line over TCP, or one per message
over ZeroMQ), each with a ``type`` and a ``schema``:

``hello``                who is publishing. Informational.
``channel_declaration``  sent once at connect time: which anatomical channels
                         this device reports, where each comes from
                         (``provenance``), and a default radians range.
``pose_frames``          per tick: ``frames.<side>.finger_angles_rad.<finger>.
                         <field>`` in radians, five fingers by up to six
                         generic fields (``mcp_flex pip_flex dip_flex swing
                         thumb_yaw total_flex``).

A pose frame names no channel. Which anatomical channel a ``<finger>.<field>``
slot carries is fixed by the vocabulary for every slot but one —
``thumb.swing`` is ``thumb_cm_roll`` on one glove and ``thumb_cm_swing`` on
another — and whether a number in a slot is a MEASUREMENT at all is said only
by the declaration. So :class:`PoseStreamDecoder` decodes pose frames AGAINST
the declaration, and by default decodes nothing for a side until that side's
declaration has arrived.

Two dialects, one decoder
-------------------------
* device-wide (the glove stream encoder the UDCAP glove driver publishes
  through): one declaration for the whole device, no ``side``;
  ``derived_from`` lists CHANNEL names.
* the LitchiBot glove driver's own encoder: one declaration PER HAND carrying
  ``side``; each channel carries a ``v1_field`` object
  (``{"finger": "index", "field": "mcp_flex"}``) naming its slot, an optional
  ``caveat``; ``derived_from`` lists sensor NODES
  (``[{"node": "<sensor>"}]``); and the thumb abduction is the
  extension channel ``thumb_cm_swing``.

Both are accepted. A declaration's own ``v1_field`` must agree with the
vocabulary's slot for that name (a disagreement is refused — it would put a
number into the wrong finger), and a declaration claiming one slot with two
channels is refused (``thumb_cm_roll`` and ``thumb_cm_swing`` together).

What decoding does NOT do
-------------------------
* It never writes 0.0 for a missing number. A declared channel with no
  finite value this tick decodes to ``None``. v1's own reference decoder read
  an absent field as 0.0 — a fully extended finger, which opens a robot hand —
  and that is the behaviour this module exists to not have.
* It never reads a channel the device declared ``uncalibrated``: there is a
  number on the wire and no angle behind it. Such a channel is always
  ``None``.
* It ignores ``thumb_yaw`` on the four fingers: v1 defines it as 0.0 there.
* It does not enforce ``schema`` (the shared envelope rule: report it, do not
  gate on it) and it drops an unknown ``type`` silently (that is how a family
  grows a new packet without breaking a consumer).
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Iterable, List, Mapping, Optional, Tuple, Union

from .calibration import ChannelCalibration, ChannelRange
from .channels import (
    ALL_CHANNELS,
    FINGERS,
    ChannelKind,
    channels_for_slot,
    spec_of,
)
from .pose import HandAngles

#: The family identifier. Reported, never enforced.
SCHEMA = "teleop_gloves.pose_stream.v1"
HELLO_PACKET_TYPE = "hello"
DECLARATION_PACKET_TYPE = "channel_declaration"
POSE_PACKET_TYPE = "pose_frames"

#: v1's per-finger angle field names.
ANGLE_FIELDS: Tuple[str, ...] = (
    "mcp_flex", "pip_flex", "dip_flex", "swing", "thumb_yaw", "total_flex",
)

SIDES: Tuple[str, ...] = ("left", "right")


class DeclarationError(ValueError):
    """A ``channel_declaration`` that cannot be decoded against safely."""


class Provenance(Enum):
    """Where a channel's numbers come from ON THIS DEVICE. Static."""

    #: A sensor placed at that joint measures that quantity.
    MEASURED = "measured"
    #: The device's own solver produced it from its own observations.
    DERIVED = "derived"
    #: The driver computed it from other channels; adds no information.
    SYNTHESIZED = "synthesized"
    #: A sensor reports, but no calibration makes it an angle. Never drive
    #: from one.
    UNCALIBRATED = "uncalibrated"


@dataclass(frozen=True)
class DeclaredChannel:
    """One channel as a device declared it."""

    name: str
    kind: ChannelKind
    provenance: Provenance
    #: ``(finger, field)`` of the v1 slot it is published in, or ``None``.
    slot: Optional[Tuple[str, str]]
    #: CHANNEL names it was computed from (the device-wide dialect).
    derived_from: Tuple[str, ...] = ()
    #: Sensor NODE names it was computed from (the LitchiBot dialect).
    derived_from_nodes: Tuple[str, ...] = ()
    default_range: Optional[ChannelRange] = None
    note: str = ""
    caveat: str = ""


@dataclass(frozen=True)
class Declaration:
    """Every channel one device declares for one hand (or for every hand,
    when ``side`` is ``None``), in declared order."""

    device_model: str
    side: Optional[str]
    channels: Dict[str, DeclaredChannel] = field(default_factory=dict)

    def declares(self, channel: str) -> bool:
        return channel in self.channels

    def default_ranges(self) -> Dict[str, ChannelRange]:
        """Channel -> declared default range, for the channels that have one.

        Feed this to :class:`~manipulation_kit.gloves.calibration.ChannelCalibration`
        as its ``defaults``. An uncalibrated channel has none, so a calibration
        built on these refuses an override for it."""
        return {name: c.default_range for name, c in self.channels.items()
                if c.default_range is not None
                and c.provenance is not Provenance.UNCALIBRATED}

    def calibration(self, ranges: Optional[Mapping[str, ChannelRange]] = None
                    ) -> ChannelCalibration:
        """A :class:`ChannelCalibration` for this device: its declared default
        ranges, ``ranges`` (one operator's overrides, e.g. from
        :func:`~manipulation_kit.gloves.calibration.load_ranges`) over them,
        and overrides accepted for every declared channel that is not
        UNCALIBRATED."""
        known = [name for name, c in self.channels.items()
                 if c.provenance is not Provenance.UNCALIBRATED]
        return ChannelCalibration(self.default_ranges(), ranges, known)

    def refuse_as_drive_source(self, channel: str) -> Optional[str]:
        """Why ``channel`` must not drive a robot, or ``None`` if it may.

        The Rust ``DeviceDeclaration::refuse_as_drive_source`` rule: a channel
        the device does not declare is refused; ``uncalibrated`` is refused;
        ``synthesized`` is refused while EVERY channel it was synthesized from
        is itself declared measured or derived (reading the sum would discard
        those measurements), and allowed when any is missing (then the
        synthesis is the only thing there is)."""
        model = self.device_model
        declared = self.channels.get(channel)
        if declared is None:
            return f"{model} does not report {channel}"
        if declared.provenance is Provenance.UNCALIBRATED:
            return (f"{channel} on {model} is UNCALIBRATED: {declared.note} "
                    "There is a number on the wire and no angle behind it.")
        if declared.provenance is Provenance.SYNTHESIZED:
            sources = [s for s in declared.derived_from
                       if s in self.channels and self.channels[s].provenance
                       in (Provenance.MEASURED, Provenance.DERIVED)]
            if not declared.derived_from or len(sources) != len(declared.derived_from):
                return None
            return (f"{channel} on {model} is SYNTHESIZED from "
                    f"{', '.join(declared.derived_from)}, which this device reports "
                    "directly. Drive from those instead.")
        return None

    def check_drive_sources(self, channels: Iterable[str]) -> None:
        """Raise :class:`DeclarationError` listing every refusal among
        ``channels``. Call it ONCE, at bind time, with a hand map's
        ``required_channels()``: a check that runs at 50 Hz is a check that
        gets removed."""
        refusals = [r for r in (self.refuse_as_drive_source(c) for c in channels)
                    if r is not None]
        if refusals:
            raise DeclarationError("\n".join(refusals))


def _parse_slot(name: str, raw: object) -> Optional[Tuple[str, str]]:
    spec = spec_of(name)
    assert spec is not None
    if raw is None:
        return spec.v1_slot
    if not (isinstance(raw, Mapping) and isinstance(raw.get("finger"), str)
            and isinstance(raw.get("field"), str)):
        raise DeclarationError(f"{name}: v1_field must be "
                               f"{{finger, field}}, got {raw!r}")
    slot = (raw["finger"], raw["field"])
    if slot != spec.v1_slot:
        raise DeclarationError(
            f"{name}: declared in v1 slot {slot[0]}.{slot[1]}, but the vocabulary "
            f"publishes it in {spec.v1_slot}. Decoding it would read another "
            "joint's number as this one")
    return slot


def parse_declaration(payload: Mapping[str, object]) -> Declaration:
    """One ``channel_declaration`` payload, in either dialect, validated."""
    device_model = payload.get("device_model")
    if not isinstance(device_model, str):
        raise DeclarationError("channel_declaration has no device_model string")
    side = payload.get("side")
    if side is not None and side not in SIDES:
        raise DeclarationError(f"channel_declaration side must be one of "
                               f"{SIDES}, got {side!r}")
    raw_channels = payload.get("channels")
    if not isinstance(raw_channels, Mapping):
        raise DeclarationError("channel_declaration has no channels object")

    channels: Dict[str, DeclaredChannel] = {}
    slots: Dict[Tuple[str, str], str] = {}
    for name, raw in raw_channels.items():
        spec = spec_of(name)
        if spec is None:
            raise DeclarationError(f"{name!r} is not in the glove channel vocabulary")
        if not isinstance(raw, Mapping):
            raise DeclarationError(f"{name}: a declared channel is an object")
        try:
            kind = ChannelKind(raw.get("kind"))
            provenance = Provenance(raw.get("provenance"))
        except ValueError as exc:
            raise DeclarationError(f"{name}: {exc}") from exc
        if kind is not spec.kind:
            raise DeclarationError(f"{name}: declared kind {kind.value!r}, the "
                                   f"vocabulary says {spec.kind.value!r}")
        derived, nodes = [], []
        for item in raw.get("derived_from") or ():
            if isinstance(item, str):
                derived.append(item)
            elif isinstance(item, Mapping) and isinstance(item.get("node"), str):
                nodes.append(item["node"])
            else:
                raise DeclarationError(f"{name}: derived_from entry {item!r} is "
                                       "neither a channel name nor {node}")
        default_range = None
        if raw.get("default_range") is not None:
            try:
                default_range = ChannelRange.from_json(raw["default_range"])
            except ValueError as exc:
                raise DeclarationError(f"{name}: default_range: {exc}") from exc
        slot = _parse_slot(name, raw.get("v1_field"))
        if slot is not None:
            other = slots.setdefault(slot, name)
            if other != name:
                raise DeclarationError(
                    f"{other} and {name} both claim v1 slot {slot[0]}.{slot[1]}; "
                    "a pose frame could not say which one a number is")
        channels[name] = DeclaredChannel(
            name=name, kind=kind, provenance=provenance, slot=slot,
            derived_from=tuple(derived), derived_from_nodes=tuple(nodes),
            default_range=default_range,
            note=str(raw.get("note") or ""), caveat=str(raw.get("caveat") or ""))
    return Declaration(device_model=device_model, side=side, channels=channels)


def vocabulary_declaration(device_model: str = "unknown/undeclared") -> Declaration:
    """A stand-in declaration for a publisher that sends none.

    Every slot is read as the vocabulary's own channel (``thumb.swing`` as
    ``thumb_cm_roll``), every channel as ``measured``, with no default range.
    Only for :class:`PoseStreamDecoder` with ``require_declaration=False``;
    a calibration built on it has no defaults, so every range must come from
    the operator's file."""
    channels = {}
    for spec in ALL_CHANNELS:
        if spec.v1_slot is None or spec.name == "thumb_cm_swing":
            continue
        channels[spec.name] = DeclaredChannel(
            name=spec.name, kind=spec.kind, provenance=Provenance.MEASURED,
            slot=spec.v1_slot)
    return Declaration(device_model=device_model, side=None, channels=channels)


def _finite(value: object) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def decode_hand(frame: Mapping[str, object], declaration: Declaration,
                side: str) -> HandAngles:
    """One ``frames.<side>`` body to :class:`HandAngles` over the declared
    channels. Every declared channel is a key; the value is ``None`` unless
    its slot holds a finite number and the channel is not uncalibrated."""
    angles = frame.get("finger_angles_rad")
    angles = angles if isinstance(angles, Mapping) else {}
    radians: Dict[str, Optional[float]] = {}
    for name, declared in declaration.channels.items():
        value = None
        if declared.slot is not None and declared.provenance is not Provenance.UNCALIBRATED:
            finger, fld = declared.slot
            fields = angles.get(finger)
            if isinstance(fields, Mapping):
                value = _finite(fields.get(fld))
        radians[name] = value
    return HandAngles(side=side, radians=radians,
                      timestamp_s=_finite(frame.get("timestamp_s")))


class PoseStreamDecoder:
    """Stateful decoder for one connection to one glove driver.

    Feed it every received packet (a ``dict`` or one JSON line) in order;
    :meth:`feed` returns the hands a ``pose_frames`` packet carried, decoded
    against the latest declaration for each side, and ``[]`` for anything
    else. A new declaration replaces the old one for its side (or for every
    side, in the device-wide dialect).

    ``require_declaration`` (default ``True``): a pose frame for a side with
    no declaration yet decodes to nothing and is counted in
    :attr:`undeclared_frames`. Set ``False`` only for a publisher that never
    sends one; it then decodes against :func:`vocabulary_declaration`."""

    def __init__(self, *, require_declaration: bool = True):
        self.require_declaration = require_declaration
        self._declarations: Dict[Optional[str], Declaration] = {}
        self.hello: Optional[Dict[str, object]] = None
        self.last_sequence: Optional[int] = None
        self.last_source: Optional[str] = None
        self.last_schema: Optional[str] = None
        self.undeclared_frames = 0

    def declaration_for(self, side: str) -> Optional[Declaration]:
        """The declaration a ``side`` frame decodes against, if any."""
        found = self._declarations.get(side) or self._declarations.get(None)
        if found is None and not self.require_declaration:
            return vocabulary_declaration()
        return found

    @property
    def declarations(self) -> Dict[Optional[str], Declaration]:
        return dict(self._declarations)

    def feed(self, packet: Union[str, bytes, Mapping[str, object]]) -> List[HandAngles]:
        if isinstance(packet, (str, bytes)):
            packet = json.loads(packet)
        if not isinstance(packet, Mapping):
            return []
        kind = packet.get("type")
        schema = packet.get("schema")
        if isinstance(schema, str):
            self.last_schema = schema
        if kind == HELLO_PACKET_TYPE:
            self.hello = dict(packet)
            return []
        if kind == DECLARATION_PACKET_TYPE:
            declaration = parse_declaration(packet)
            if declaration.side is None:
                self._declarations = {None: declaration}
            else:
                self._declarations.pop(None, None)
                self._declarations[declaration.side] = declaration
            return []
        if kind != POSE_PACKET_TYPE:
            return []
        seq = packet.get("sequence")
        self.last_sequence = seq if isinstance(seq, int) and not isinstance(seq, bool) else None
        src = packet.get("source")
        self.last_source = src if isinstance(src, str) else None
        frames = packet.get("frames")
        if not isinstance(frames, Mapping):
            return []
        out = []
        for side in SIDES:
            frame = frames.get(side)
            if not isinstance(frame, Mapping):
                continue
            declaration = self.declaration_for(side)
            if declaration is None:
                self.undeclared_frames += 1
                continue
            out.append(decode_hand(frame, declaration, side))
        return out


__all__ = [
    "ANGLE_FIELDS", "DECLARATION_PACKET_TYPE", "Declaration", "DeclarationError",
    "DeclaredChannel", "FINGERS", "HELLO_PACKET_TYPE", "POSE_PACKET_TYPE",
    "PoseStreamDecoder", "Provenance", "SCHEMA", "SIDES", "channels_for_slot",
    "decode_hand", "parse_declaration", "vocabulary_declaration",
]
