"""Radians to flexion in [0, 1], per channel, per operator.

This is the middle of the three layers between a glove and a robot hand:

1. the glove driver publishes anatomical channels in RADIANS
   (:mod:`manipulation_kit.gloves.v1` decodes them);
2. **this module** normalises each channel to flexion in [0, 1] against a
   range that belongs to one operator wearing one glove;
3. a hand's retarget map turns flexion into that hand's joint targets
   (:mod:`manipulation_kit.hands`).

The split is what makes N gloves and M hands cost N + M instead of N x M: a
hand map never sees radians or a glove's quirks, and a glove's calibration
never knows which hand it will drive.

The semantics are those of the glove stream encoder's own calibration (a
Rust library), deliberately identical so a range measured on either side
reads the same: ``lo`` reads 0.0, ``hi`` reads 1.0, values between scale linearly and
values outside clamp, ``invert`` flips the result afterwards. A bipolar channel
(a finger's lateral spread) needs no special case: its range is a signed pair
such as ``(-0.349, 0.349)`` and the linear map lands a neutral finger at 0.5.

``None`` in, ``None`` out
-------------------------
Nothing here can turn "no reading" into 0.0. On every flexion channel 0.0 is a
fully extended finger, so a fabricated 0.0 OPENS a robot hand. A missing
reading normalises to ``None``, and a hand map turns that into "hold this
joint" rather than a position.

The file format
---------------
A calibration file holds one operator's OVERRIDES on top of the defaults the
device declared (its ``channel_declaration``'s ``default_range``). Overrides
only, so a bench session that measured one finger writes that one finger and
inherits the rest::

    {
      "schema": "manipulation_kit.glove_calibration.v1",
      "device_model": "udexreal/udcap",
      "ranges": {
        "index_mp_pitch": {"lo": 0.05, "hi": 1.31, "invert": false}
      }
    }

``device_model`` is informational (a log line, a refusal message); it does
not select behaviour.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Mapping, Optional, Union

from .pose import HandAngles, HandPose

#: The ``schema`` string a calibration file carries. Refused if different.
CALIBRATION_SCHEMA = "manipulation_kit.glove_calibration.v1"


class UnknownChannels(KeyError):
    """A calibration was asked about, or given, a channel it has no range for."""

    def __init__(self, channels):
        self.channels = sorted(channels)
        super().__init__(f"no range for channel(s) {self.channels}")


@dataclass(frozen=True)
class ChannelRange:
    """One channel's linear normalisation from radians to [0, 1].

    Refuses ``lo == hi`` (and a non-finite bound) at construction: a division
    by zero inside a 50 Hz loop is not where anyone wants to find that out.
    ``lo > hi`` is allowed — it is a channel whose sign runs the other way —
    and is not the same as ``invert``, which flips the clamped result."""

    lo: float
    hi: float
    invert: bool = False

    def __post_init__(self):
        if not (math.isfinite(self.lo) and math.isfinite(self.hi)):
            raise ValueError(f"ChannelRange bounds must be finite, got "
                             f"lo={self.lo!r} hi={self.hi!r}")
        if self.lo == self.hi:
            raise ValueError(f"ChannelRange needs a non-zero span, got "
                             f"lo == hi == {self.lo!r}")

    @property
    def span(self) -> float:
        return self.hi - self.lo

    def inverted(self) -> "ChannelRange":
        return ChannelRange(self.lo, self.hi, not self.invert)

    def normalize(self, radians: float) -> float:
        """One reading in radians to flexion in [0, 1], clamped."""
        value = min(1.0, max(0.0, (radians - self.lo) / (self.hi - self.lo)))
        return 1.0 - value if self.invert else value

    def to_json(self) -> Dict[str, object]:
        return {"lo": self.lo, "hi": self.hi, "invert": self.invert}

    @classmethod
    def from_json(cls, data: object) -> "ChannelRange":
        """``{"lo", "hi", "invert"?}`` or a ``[lo, hi]`` pair (the declaration's
        ``default_range`` shape). Anything else is refused."""
        if isinstance(data, (list, tuple)) and len(data) == 2:
            return cls(_number(data[0], "lo"), _number(data[1], "hi"))
        if isinstance(data, Mapping):
            unknown = set(data) - {"lo", "hi", "invert"}
            if unknown or "lo" not in data or "hi" not in data:
                raise ValueError(f"a range is {{lo, hi, invert?}}, got {dict(data)!r}")
            invert = data.get("invert", False)
            if not isinstance(invert, bool):
                raise ValueError(f"invert must be true or false, got {invert!r}")
            return cls(_number(data["lo"], "lo"), _number(data["hi"], "hi"), invert)
        raise ValueError(f"a range is {{lo, hi, invert?}} or [lo, hi], got {data!r}")


def _number(value: object, what: str) -> float:
    # bool is an int in Python; a range bound of `true` is a typo, not 1.0.
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"range {what} must be a number, got {value!r}")
    return float(value)


class ChannelCalibration:
    """Per-channel overrides on top of a set of defaults.

    ``defaults`` are normally the device's declared ``default_range`` values;
    ``ranges`` are one operator's overrides. ``known`` names the channels an
    override may be given for, and defaults to the channels that have a
    default. An override for any other channel is refused: a typo would
    otherwise be a range that silently never applies.

    ``known`` exists because a device may declare a drivable channel with NO
    default range — the LitchiBot glove declares none for its finger flexions
    — and that is exactly the channel an operator's file has to supply.
    :meth:`manipulation_kit.gloves.v1.Declaration.calibration` builds one with
    ``known`` = every channel the device declared and did not mark
    UNCALIBRATED (a channel that is, is not something a robot may be driven
    from, whatever range a file gives it)."""

    def __init__(self, defaults: Mapping[str, ChannelRange],
                 ranges: Optional[Mapping[str, ChannelRange]] = None,
                 known: Optional[Iterable[str]] = None):
        ranges = dict(ranges or {})
        self._known = frozenset(defaults) if known is None else frozenset(known)
        unknown = set(ranges) - self._known
        if unknown:
            raise UnknownChannels(unknown)
        self._defaults: Dict[str, ChannelRange] = dict(defaults)
        self._ranges: Dict[str, ChannelRange] = ranges

    @property
    def defaults(self) -> Dict[str, ChannelRange]:
        return dict(self._defaults)

    @property
    def overrides(self) -> Dict[str, ChannelRange]:
        return dict(self._ranges)

    def range_for(self, channel: str) -> Optional[ChannelRange]:
        """``channel``'s range: the override if there is one, else the default."""
        return self._ranges.get(channel, self._defaults.get(channel))

    def overriding(self, ranges: Mapping[str, ChannelRange]) -> "ChannelCalibration":
        """A copy with ``ranges`` merged over the current overrides."""
        merged = dict(self._ranges)
        merged.update(ranges)
        return ChannelCalibration(self._defaults, merged, self._known)

    def normalize(self, channel: str, radians: Optional[float]) -> Optional[float]:
        """Flexion in [0, 1], or ``None`` when there was no reading.

        A channel with no range raises :class:`UnknownChannels`: an error, not
        a guess."""
        rng = self.range_for(channel)
        if rng is None:
            raise UnknownChannels([channel])
        if radians is None or not math.isfinite(radians):
            return None
        return rng.normalize(radians)

    def normalize_hand(self, angles: HandAngles) -> HandPose:
        """One decoded hand (radians) to one canonical :class:`HandPose`.

        Channels with no range in this calibration — the ones the device
        declared UNCALIBRATED — come out ``None``, never 0.0: a hand map that
        asks for one holds that joint."""
        flex: Dict[str, Optional[float]] = {}
        for channel, radians in angles.radians.items():
            rng = self.range_for(channel)
            if rng is None or radians is None or not math.isfinite(radians):
                flex[channel] = None
            else:
                flex[channel] = rng.normalize(radians)
        return HandPose(side=angles.side, flex=flex,
                        timestamp_s=angles.timestamp_s)

    # ------------------------------------------------------------- files --
    def save(self, path: Union[str, Path], *,
             device_model: Optional[str] = None) -> None:
        """Write the OVERRIDES (not the defaults) as a calibration file."""
        save_ranges(path, self._ranges, device_model=device_model)

    @classmethod
    def load(cls, path: Union[str, Path],
             defaults: Mapping[str, ChannelRange],
             known: Optional[Iterable[str]] = None) -> "ChannelCalibration":
        """Overrides from a calibration file, on top of ``defaults``."""
        return cls(defaults, load_ranges(path), known)


def load_ranges(path: Union[str, Path]) -> Dict[str, ChannelRange]:
    """The ranges in a calibration file, validated.

    Refuses a wrong ``schema``, an unknown top-level key, a channel name
    outside the vocabulary and a malformed or zero-span range — each with the
    file and channel named — so a typo stops a teleop session at startup and
    not at the first grasp."""
    from .channels import spec_of  # noqa: PLC0415

    path = Path(path)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"{path}: not JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"{path}: a calibration file is a JSON object")
    if data.get("schema") != CALIBRATION_SCHEMA:
        raise ValueError(f"{path}: schema must be {CALIBRATION_SCHEMA!r}, "
                         f"got {data.get('schema')!r}")
    unknown = set(data) - {"schema", "device_model", "ranges", "note"}
    if unknown:
        raise ValueError(f"{path}: unknown key(s) {sorted(unknown)}")
    raw = data.get("ranges", {})
    if not isinstance(raw, dict):
        raise ValueError(f"{path}: 'ranges' is an object of channel -> range")
    out: Dict[str, ChannelRange] = {}
    for channel, value in raw.items():
        if spec_of(channel) is None:
            raise ValueError(f"{path}: {channel!r} is not a glove channel name")
        try:
            out[channel] = ChannelRange.from_json(value)
        except ValueError as exc:
            raise ValueError(f"{path}: {channel}: {exc}") from exc
    return out


def save_ranges(path: Union[str, Path], ranges: Mapping[str, ChannelRange], *,
                device_model: Optional[str] = None) -> None:
    """Write ``ranges`` as a calibration file (sorted, so diffs stay small)."""
    data: Dict[str, object] = {"schema": CALIBRATION_SCHEMA}
    if device_model is not None:
        data["device_model"] = device_model
    data["ranges"] = {name: ranges[name].to_json() for name in sorted(ranges)}
    Path(path).write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
