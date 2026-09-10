"""Anatomical flexion channels → LinkerHand O30 20-joint retarget map — STUB.

This module exists so the O30 answers the same seam as every other hand
(``manipulation_kit.hands.get_retarget`` imports ``<maker>/<model>.retarget`` and calls
``build_retarget(config)``), and so the failure is a clear "not written yet"
instead of an import error that reads like a missing package.

The mapping itself is a separate task. What it will take:

* **20 axes, not 6.** The DH116S map
  (:mod:`manipulation_kit.hands.leadshine.dh116s.retarget`) collapses a glove's
  per-joint flexions onto one flexion linkage per finger plus a thumb swing.
  The O30 drives root1, root2 and tip per finger independently, plus yaw on
  all five fingers and roll on the thumb — the glove channels that were
  averaged away (``*_pip_pitch``, ``*_dip_pitch``) now have their own axes,
  and abduction/adduction has axes for the first time.
* **Direction and range per joint are uncalibrated.** On root1/tip, 0 is
  roughly straight and ~200 is bent (the resting readback sits around 25-37).
  For roll, yaw and root2 neither end has been identified on hardware. A
  retarget map written before that calibration would be guesswork with a
  plausible shape, which is worse than nothing.
* **Wire units are 0-255**, not the DH116S's 0-10000 stroke fraction
  (:data:`~.axes.POS_MAX`), so the output scaling is not a copy of the
  DH116S map.

Until then the axis order and names are already fixed and importable from
:mod:`~.axes` (:data:`AXIS_NAMES`), so a mapper written elsewhere can be
dropped in here without renaming anything.
"""

from __future__ import annotations

from dataclasses import dataclass

from .axes import AXIS_NAMES, NUM_AXES, POS_MAX  # noqa: F401  (re-exported)

#: Anatomical input channels a future map is expected to consume — the same
#: glove-side vocabulary the DH116S map uses, kept identical on purpose so the
#: teleop stack's ``flex(channel) -> [0, 1]`` closure needs no changes.
_FINGERS = ("index", "middle", "ring", "pinky")
CHANNELS = (
    "thumb_cm_yaw", "thumb_cm_roll", "thumb_cm_pitch", "thumb_mp_pitch",
) + tuple(f"{f}_{j}_pitch" for f in _FINGERS for j in ("mp", "pip", "dip"))


@dataclass
class RetargetConfig:
    """Placeholder config so callers can already pass one (no knobs yet)."""


def build_retarget(config: "RetargetConfig | None" = None):
    """Standard cross-hand retarget constructor — NOT IMPLEMENTED for the O30.

    Raises :class:`NotImplementedError` with what is missing; see the module
    docstring. Deliberately not a pass-through of the DH116S 6-axis map: the
    O30 has 20 independent joints whose directions are not calibrated, and
    silently driving them from a 6-axis map would move fingers in unverified
    directions on real hardware.
    """
    raise NotImplementedError(
        "linkerbot/o30 has no retarget map yet: the glove→20-joint mapping "
        "needs per-joint direction and range calibration on hardware first "
        "(roll/yaw/root2 ends are unidentified). Axis order and names are "
        "fixed — see manipulation_kit.hands.linkerbot.o30.axes.AXIS_NAMES."
    )
