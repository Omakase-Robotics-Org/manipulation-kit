"""XHAND1 joint vocabulary — names, order, units and limits.

What ``d1-firmwared`` publishes for this hand (model ``robotera/xhand1``), so
a retarget map written here produces targets the daemon accepts without
translation. The daemon takes its XHAND1 descriptor from its hand driver
library, which defines it as follows:

* names: snake-case forms of the joint names IIT's ``yarp-device-xhand``
  uses, in the hand's wire order (block ``k`` of the real-time exchange is
  joint ``k``);
* limits: RobotEra's "X-Hand 1 Product Manual V1.0" (2024-10-15), section
  3.2, in DEGREES, converted exactly. The manual's own radian column is
  rounded (1.57, 1.92); the degrees are what it specifies.

The XHAND1's own wire unit is a ``float32`` angle in radians per joint, so
unlike the DH116S (0..10000) and the O30 (0..255) there is no integer scale to
divide by: :data:`JOINTS` are ``rad``.

``tests/hands/test_xhand_descriptor_drift.py`` parses the hand driver's
descriptor source and compares it with this table when
``MKIT_HAND_DESCRIPTOR_DIR`` holds a copy of it (opt-in: the driver is not
part of this repository).
"""
from __future__ import annotations

import math
from typing import Tuple

from ...joints import UNIT_RAD, JointSpec

#: Number of active joints; the daemon commands all twelve in one exchange.
NUM_AXES = 12

#: Joint names in wire order (knuckle id 0-11).
AXIS_NAMES: Tuple[str, ...] = (
    "thumb_bend",
    "thumb_rota1",
    "thumb_rota2",
    "index_bend",
    "index_j1",
    "index_j2",
    "mid_j1",
    "mid_j2",
    "ring_j1",
    "ring_j2",
    "pinky_j1",
    "pinky_j2",
)

#: Each joint's range of motion in degrees, ``(min, max)``, wire order —
#: the manual's numbers.
JOINT_LIMITS_DEG: Tuple[Tuple[float, float], ...] = (
    (0.0, 90.0),
    (-60.0, 90.0),
    (0.0, 90.0),
    (-5.0, 17.0),
    (0.0, 110.0),
    (0.0, 110.0),
    (0.0, 110.0),
    (0.0, 110.0),
    (0.0, 110.0),
    (0.0, 110.0),
    (0.0, 110.0),
    (0.0, 110.0),
)

#: The same in radians, converted exactly.
JOINT_LIMITS_RAD: Tuple[Tuple[float, float], ...] = tuple(
    (math.radians(lo), math.radians(hi)) for lo, hi in JOINT_LIMITS_DEG)

#: The joints as the end-effector descriptor lists them: name, ``rad``, limits.
JOINTS: Tuple[JointSpec, ...] = tuple(
    JointSpec(name, UNIT_RAD, lo, hi)
    for name, (lo, hi) in zip(AXIS_NAMES, JOINT_LIMITS_RAD))

assert len(AXIS_NAMES) == len(JOINT_LIMITS_RAD) == NUM_AXES
