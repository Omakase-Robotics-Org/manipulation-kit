"""Wuji Hand 2 joint vocabulary — names, order, units and limits.

What an ``end_effector.shm/1`` provider for this hand declares (model
``wuji/wujihand2``), so a retarget map written here produces targets
``d1-firmwared`` accepts without translation. A provider builds its
descriptor FROM :data:`JOINTS`, so the two cannot drift apart.

* names: the vendor SDK's joint labels (``wuji_sdk``: ``hand.joint(i).label``),
  in the SDK's joint index order — finger major, thumb first, ``S1..S4`` per
  finger. The index of a name here is the index the SDK's joint command takes.
* limits: the vendor URDFs (``wuji_v2/urdf/right.urdf`` and ``left.urdf`` of
  the vendor's retarget application), in radians as written there. The two
  sides declare the SAME limits joint for joint, and the same anatomical
  direction: the left URDF mirrors each flexion axis (``-1 0 0`` against
  ``1 0 0``), so positive is flexion on both hands, and positive ``S2`` moves
  index, ring and pinky towards the little-finger side on both hands
  (forward kinematics of both URDFs). One table therefore serves both sides.

What each joint is (vendor URDF joint names in brackets):

=========  ======================================  ======================
joint      index, middle, ring, pinky              thumb
=========  ======================================  ======================
``S1``     knuckle flexion (``mcp_flex``)          base flexion (``cmc_flex``)
``S2``     lateral spread (``mcp_abd``)            base abduction (``cmc_abd``)
``S3``     middle joint flexion (``pip``)          second joint (``mcp``)
``S4``     last joint flexion (``dip``)            last joint (``ip``)
=========  ======================================  ======================

``tests/hands/test_wujihand2.py`` parses both vendor URDFs and compares them
with this table when ``MKIT_WUJI_URDF_DIR`` holds them (opt-in: the URDFs
are the vendor's and not part of this repository).
"""
from __future__ import annotations

from typing import Tuple

from ...joints import UNIT_RAD, JointSpec

#: Number of joints; the SDK commands all twenty in one message.
NUM_AXES = 20

#: The five digits, in SDK order.
FINGERS: Tuple[str, ...] = ("thumb", "index", "middle", "ring", "pinky")

#: Joint names in SDK order (joint index 0-19).
AXIS_NAMES: Tuple[str, ...] = tuple(f"{f}_S{i}" for f in FINGERS for i in range(1, 5))

_THUMB = ((-1.187, 1.291), (-1.484, 0.698), (-1.047, 1.570), (-1.047, 1.570))
_FINGER = ((-1.047, 1.570), (-0.698, 0.698), (-1.047, 2.094), (-1.047, 1.570))

#: Each joint's range in radians, ``(min, max)``, SDK order — the vendor
#: URDFs' numbers, identical on both sides.
JOINT_LIMITS_RAD: Tuple[Tuple[float, float], ...] = _THUMB + _FINGER * 4

#: The joints as the end-effector descriptor lists them: name, ``rad``, limits.
JOINTS: Tuple[JointSpec, ...] = tuple(
    JointSpec(name, UNIT_RAD, lo, hi)
    for name, (lo, hi) in zip(AXIS_NAMES, JOINT_LIMITS_RAD))

assert len(AXIS_NAMES) == len(JOINT_LIMITS_RAD) == NUM_AXES
