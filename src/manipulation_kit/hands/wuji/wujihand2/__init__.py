"""Wuji Hand 2 dexterous hand — joint vocabulary and glove retarget map.

Only what a planner or a teleop stack needs without a robot: the twenty
joints exactly as an ``end_effector.shm/1`` provider for this hand declares
them to ``d1-firmwared`` (:mod:`axes`) and the anatomical-flexion → joint map
(:mod:`retarget`). The hand itself is driven over Ethernet by the vendor's
SDK, in the provider, not here. There is no ``toolconfig`` and no
``description`` yet — see ``README.md``.
"""

from .axes import AXIS_NAMES, JOINT_LIMITS_RAD, JOINTS, NUM_AXES  # noqa: F401
