"""RobotEra XHAND1 Lite — the six-joint XHAND1: joint table and glove map.

:mod:`axes` holds the six joints exactly as ``d1-firmwared`` publishes them
(model ``robotera/xhand1_lite``) and :mod:`retarget` the anatomical-flexion →
joint map. No ``toolconfig`` and no ``description``: no mass, tool point,
centre of mass, inertia or CAD for the Lite is documented in the sources this
package was written from — see ``README.md``.
"""

from .axes import AXIS_NAMES, JOINT_LIMITS_RAD, JOINTS, MIN_RAD, NUM_AXES  # noqa: F401
