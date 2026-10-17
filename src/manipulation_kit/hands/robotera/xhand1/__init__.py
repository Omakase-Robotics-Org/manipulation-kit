"""RobotEra XHAND1 dexterous hand — joint vocabulary and glove retarget map.

Only what a planner or a teleop stack needs without a robot: the twelve
joints exactly as ``d1-firmwared`` publishes them (:mod:`axes`) and the
anatomical-flexion → joint map (:mod:`retarget`). The RS-485 driver is the
daemon's. There is no ``toolconfig`` and no
``description`` yet: the manual gives a mass (1.1 kg) but no tool point,
centre of mass or inertia, and no CAD or URDF ships with this package — see
``README.md``.
"""

from .axes import AXIS_NAMES, JOINT_LIMITS_DEG, JOINT_LIMITS_RAD, JOINTS, NUM_AXES  # noqa: F401
