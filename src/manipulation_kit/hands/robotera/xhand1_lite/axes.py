"""XHAND1 Lite joint vocabulary — names, order, units and limits.

The Lite is the six-joint variant of RobotEra's XHAND1: one flexion joint per
finger and two thumb joints. It speaks the XHAND1's RS-485 real-time exchange
with all twelve slots, but only slots 0-5 are joints. ``d1-firmwared``
publishes it as model ``robotera/xhand1_lite`` (from its hand driver
library's descriptor), and :data:`JOINTS` is that descriptor: these names,
in this order, ``rad``, these limits.

Provenance
----------
* Order: RobotEra's xbot SDK, ``robot_controller/robot_controller.py``
  (``control_xhand_lite_grasp``) at commit ``f542ed2``. The vendor's names
  there are ``thumb_bend_joint, thumb_rota_joint1, index_joint1, mid_joint1,
  ring_joint1, pinky_joint1``; the driver names the same joints with its
  XHAND1 names (:mod:`manipulation_kit.hands.robotera.xhand1`).
* Limits: PROVISIONAL, a tested range. RobotEra publishes no range for the
  Lite; each maximum is an angle the joint has been driven to under position
  control, rounded down, and is not a mechanical end stop. The floor,
  :data:`MIN_RAD` = -0.05 rad on every joint, is the open hand (0 rad) less a
  small margin so that a hand at rest just below 0 is inside its range; the
  retarget map's open end stays at 0 rad.

``tests/hands/test_xhand_descriptor_drift.py`` parses the hand driver's
descriptor source and compares it with this table when
``MKIT_HAND_DESCRIPTOR_DIR`` holds a copy of it (opt-in: the driver is not
part of this repository).
"""
from __future__ import annotations

from typing import Tuple

from ...joints import UNIT_RAD, JointSpec

#: Number of active joints.
NUM_AXES = 6

#: Joint names in wire order (slots 0-5), the driver's names.
AXIS_NAMES: Tuple[str, ...] = (
    "thumb_bend",
    "thumb_rota1",
    "index_j1",
    "mid_j1",
    "ring_j1",
    "pinky_j1",
)

#: Every joint's lower limit, rad: 0 rad (open) less a resting margin.
MIN_RAD = -0.05

#: Commanded range per joint in radians, ``(min, max)``, wire order.
JOINT_LIMITS_RAD: Tuple[Tuple[float, float], ...] = (
    (MIN_RAD, 1.95),
    (MIN_RAD, 1.29),
    (MIN_RAD, 1.51),
    (MIN_RAD, 1.51),
    (MIN_RAD, 1.53),
    (MIN_RAD, 1.46),
)

#: The joints as the end-effector descriptor lists them.
JOINTS: Tuple[JointSpec, ...] = tuple(
    JointSpec(name, UNIT_RAD, lo, hi)
    for name, (lo, hi) in zip(AXIS_NAMES, JOINT_LIMITS_RAD))

assert len(AXIS_NAMES) == len(JOINT_LIMITS_RAD) == NUM_AXES
