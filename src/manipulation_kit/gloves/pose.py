"""The two per-hand values that cross the glove/hand seam.

:class:`HandAngles` is what a decoder produces: one hand, one instant, each
declared channel in RADIANS or ``None``. :class:`HandPose` is the CANONICAL
hand pose a hand retarget map consumes: the same channels as flexion in
[0, 1] (after :class:`~manipulation_kit.gloves.calibration.ChannelCalibration`)
or ``None``.

``None`` is the only way to say "no reading". A missing key and a ``None``
value mean the same thing, and :meth:`HandPose.get` answers ``None`` for both;
neither ever becomes 0.0, which on a flexion channel is a fully extended
finger.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, Optional


@dataclass(frozen=True)
class HandAngles:
    """One hand as decoded from the wire: channel -> radians, or ``None``."""

    #: ``"left"`` or ``"right"``.
    side: str
    #: Every channel the device DECLARED for this hand. ``None`` = declared,
    #: and no usable value this tick (absent, non-finite or uncalibrated).
    radians: Dict[str, Optional[float]] = field(default_factory=dict)
    #: The frame's ``timestamp_s`` (the publisher's monotonic clock), if any.
    timestamp_s: Optional[float] = None

    def get(self, channel: str) -> Optional[float]:
        return self.radians.get(channel)


@dataclass(frozen=True)
class HandPose:
    """The canonical hand pose: channel -> flexion in [0, 1], or ``None``.

    0 = extended, 1 = fully flexed towards the palm. A bipolar (swing) channel
    sits at 0.5 when neutral."""

    side: str
    flex: Dict[str, Optional[float]] = field(default_factory=dict)
    timestamp_s: Optional[float] = None

    def get(self, channel: str) -> Optional[float]:
        """``channel``'s flexion, or ``None``. Never a fabricated zero."""
        return self.flex.get(channel)

    def missing(self, channels) -> list:
        """The names among ``channels`` with no value in this pose."""
        return [c for c in channels if self.flex.get(c) is None]

    def as_flex_callable(self) -> Callable[[str], float]:
        """A legacy ``flex(channel) -> float`` closure over this pose.

        For the version-1 retarget call (``mapper(flex)``), whose maps take a
        float for every channel. A missing channel RAISES ``KeyError`` instead
        of answering 0.0; prefer a hand map's ``joint_targets(pose)``, which
        holds the affected joints instead."""
        def flex(channel: str) -> float:
            value = self.flex.get(channel)
            if value is None:
                raise KeyError(f"no {channel} reading in this {self.side} hand pose")
            return value
        return flex
