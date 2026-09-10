"""manipulation_kit.guard — pure-python D1 range-of-motion + self-collision guard.

Was ``d1-sdk devices/omakase_arm/pyguard``. Same code, same numbers; the only
change is that the URDF is resolved inside this package instead of from a
d1-sdk checkout, and ``GuardedRobot`` now wraps whatever object a caller hands
it (on D1 that is a d1-firmwared REST client, not a vendor SDK handle).

Usage (opt-in; nothing changes for existing code):

    from manipulation_kit.guard import MotionGuard, GuardedRobot

    guard = MotionGuard()                 # loads description/d1/d1.urdf
    rep = guard.check(jointsA, jointsB)   # degrees, SDK order J1..J7
    if not rep.ok:
        print(rep)

    robot = GuardedRobot(ArmRobot(), guard)   # drop-in wrapper
    robot.set_joint_cmd_pose('A', jointsA)        # blocked if unsafe

See guard.py for the model/frames and configuration knobs.
"""
from .guard import (GuardReport, GuardViolation, GuardedRobot, MotionGuard,
                    ARM_SIDES, JOINTS_PER_ARM, EE_LINK_PREFIXES, is_ee_body)
from .urdf_model import DEFAULT_URDF, UrdfModel

__all__ = [
    "MotionGuard", "GuardedRobot", "GuardReport", "GuardViolation",
    "UrdfModel", "DEFAULT_URDF", "ARM_SIDES", "JOINTS_PER_ARM",
    "EE_LINK_PREFIXES", "is_ee_body",
]
