"""DH116S axis vocabulary — lifted out of the retired CANFD driver.

``driver.py`` did not come to manipulation-kit (the wire is ``d1-firmwared``'s
job now), but its axis NAMES, count, wire scale and manual ROM are not driver
facts — they are what the hand is, and :mod:`.retarget` and every consumer of
``AXIS_NAMES`` still need them. Values copied verbatim from dx-manipulator
``hands/leadshine/dh116s/driver.py`` @ 2ce2c64; the daemon uses the same order.
"""
from __future__ import annotations

from typing import Tuple

#: Number of independently commanded axes.
NUM_AXES = 6

#: Total travel coefficient — a wire position is a fraction of full stroke
#: scaled to this. NOT degrees.
POS_MAX = 10000

#: Axis names in wire order.
AXIS_NAMES: Tuple[str, ...] = (
    "thumb_swing",
    "thumb_flex",
    "index_flex",
    "middle_flex",
    "ring_flex",
    "pinky_flex",
)

#: Active-joint ROM in degrees, from the DH116S user manual (axis order above).
ACTIVE_ROM_DEG: Tuple[float, ...] = (91.0, 59.0, 72.0, 72.0, 72.0, 74.0)
