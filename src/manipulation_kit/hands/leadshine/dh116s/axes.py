"""DH116S axis vocabulary — lifted out of the retired CANFD driver.

``driver.py`` did not come to manipulation-kit (the wire is ``d1-firmwared``'s
job now), but its axis NAMES, count, wire scale and manual ROM are not driver
facts — they are what the hand is, and :mod:`.retarget` and every consumer of
``AXIS_NAMES`` still need them. Values copied verbatim from dx-manipulator
``hands/leadshine/dh116s/driver.py`` @ 2ce2c64; the daemon uses the same order.
"""
from __future__ import annotations

from typing import Tuple

from ...joints import UNIT_FRACTION, JointSpec

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

#: The joints as ``d1-firmwared``'s end-effector descriptor publishes them
#: (``GET /v1/end_effectors/{side}``, driver ``leadshine-dh116s``): the axes
#: above in the same order, each a ``fraction`` 0.0 .. 1.0 of its wire range
#: (wire = fraction x :data:`POS_MAX`). 0.0 is the open preset on every axis;
#: 1.0 is full stroke (a flexion axis fully closed; the thumb swing at the
#: far end of its 0-91 degree travel).
JOINTS: Tuple[JointSpec, ...] = tuple(
    JointSpec(name, UNIT_FRACTION, 0.0, 1.0) for name in AXIS_NAMES)
