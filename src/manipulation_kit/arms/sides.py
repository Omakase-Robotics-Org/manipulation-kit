"""The crossed side convention — the single nastiest trap on this robot.

On D1 the naming crosses over, and it has bitten every consumer at least once:

    logical "left"  = URDF suffix ``_R`` = vendor SDK ArmSide **A** = the arm on
                      the operator's/robot's PHYSICAL LEFT
    logical "right" = URDF suffix ``_L`` = vendor SDK ArmSide **B** = physical RIGHT

"logical" is the operator-facing name — logical left is the arm the operator's
LEFT hand drives. The URDF suffixes are inverted relative to it because the
vendor authored the description from the robot's own point of view and then the
mount was mirrored; the SDK letters are the vendor's own ordering.

The wire vector is always 14 values, side A first then side B (i.e. logical
left then logical right), 7 joints each — the two arms are commanded as a pair.

Keeping this in one module means a consumer never has to re-derive it from a
comment in someone else's file. Every mapping below is data, not behaviour, so
this module imports nothing.
"""

from __future__ import annotations

from typing import Dict, Tuple

#: the logical sides, in wire order (A then B)
SIDES: Tuple[str, str] = ("left", "right")

#: logical side -> URDF link/joint name suffix. NOTE the inversion.
URDF_SUFFIX: Dict[str, str] = {"left": "R", "right": "L"}

#: logical side -> vendor SDK ArmSide letter
SDK_SIDE: Dict[str, str] = {"left": "A", "right": "B"}

#: logical side -> the URDF body that is the end-effector frame.
#: MuJoCo fuses the fixed ``TCP_Link`` into ``Link7`` (fusestatic), so Link7 is
#: the EE frame there — a CONSTANT flange offset, which is irrelevant for
#: RELATIVE (clutched) teleop because it cancels in the anchor delta. A
#: consumer doing ABSOLUTE Cartesian goals must add the tool offset itself
#: (``manipulation_kit.hands.get_tool_config``).
EE_BODY: Dict[str, str] = {"left": "Link7_R", "right": "Link7_L"}

#: logical side -> its 7 URDF joint names, proximal to distal
ARM_JOINTS: Dict[str, Tuple[str, ...]] = {
    side: tuple(f"Joint{i}_{suf}" for i in range(1, 8))
    for side, suf in URDF_SUFFIX.items()
}

#: shoulder / elbow bodies, used by the READY-seed clearance scoring
SHOULDER_BODY: Dict[str, str] = {
    side: f"Link2_{suf}" for side, suf in URDF_SUFFIX.items()
}
ELBOW_BODY: Dict[str, str] = {
    side: f"Link4_{suf}" for side, suf in URDF_SUFFIX.items()
}

#: sign of the arm's lateral mount offset in the base frame (+y is robot-left)
MOUNT_Y_SIGN: Dict[str, float] = {"left": 1.0, "right": -1.0}

#: joints per arm
JOINTS_PER_ARM = 7


def other(side: str) -> str:
    """The opposite logical side."""
    check(side)
    return "right" if side == "left" else "left"


def check(side: str) -> str:
    """Validate a logical side name, returning it (raises ``ValueError``)."""
    if side not in EE_BODY:
        raise ValueError(f"side must be one of {SIDES}, got {side!r}")
    return side
