"""LinkerBot LinkerHand O30 dexterous hand — geometry-free tool data (stub tier).

Only the parts a planner needs are here: the physical :mod:`toolconfig` and the
axis vocabulary in :mod:`axes`. The HOP v0.0.3 CANFD driver and transport moved
to ``d1-firmwared``; the :mod:`retarget` map is still unwritten (it needs
per-joint direction/range calibration on hardware) and raises a clear
``NotImplementedError``.

There is no O30 CAD or URDF yet, so this hand has NO ``descriptions/`` and the
``d1-wholebody-o30`` URDF variant cannot be built — see
:mod:`manipulation_kit.description.variants`.
"""

from .axes import ACTIVE_JOINTS, AXIS_NAMES, JOINT_SI, NUM_AXES, POS_MAX  # noqa: F401
