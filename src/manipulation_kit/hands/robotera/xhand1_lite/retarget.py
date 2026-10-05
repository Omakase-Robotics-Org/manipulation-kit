"""Anatomical flexion channels → XHAND1 Lite 6-joint retarget map.

The Lite has one flexion joint per finger and two thumb joints, where the
full XHAND1 (:mod:`manipulation_kit.hands.robotera.xhand1`) has two per finger,
three on the thumb and an index spread. The map has the same shape — every
joint a weighted mean of channels scaled onto a radians interval, clamped to
the joint's limits, ``None`` when an input has no reading — and reuses that
module's :class:`JointMap`, :class:`RetargetConfig` checks and
:class:`Retargeter` arithmetic with this hand's :data:`~.axes.JOINTS`.

========================  ============================================  ==========
joint                     default source(s)                             interval
========================  ============================================  ==========
``thumb_bend``      ``thumb_cm_yaw``                              open .. max
``thumb_rota1``     ``thumb_cm_pitch`` 0.5 + ``thumb_mp_pitch`` 0.5  open .. max
``<finger>_j1``            ``<finger>_mp_pitch`` 0.5 + ``_pip_pitch`` 0.5   open .. max
========================  ============================================  ==========

(``<finger>`` is index, middle, ring, pinky; the hand calls the middle finger
``mid``. "max" is the joint's published maximum (:data:`~.axes.JOINTS`);
"open" is 0 rad, the daemon's ``open`` preset, or the joint's minimum when 0
rad is outside its range.)

The fingers
-----------
One flexion joint per finger, so — exactly as the DH116S map does for its one
flexion linkage per finger — it reads the weighted mean of the knuckle (MP)
and middle-joint (PIP) flexions, with the last joint (DIP) at weight 0
because it is anatomically coupled to the PIP and adds noise, not
information. 0 rad is open (the daemon's ``open`` preset sends 0 rad).

The thumb is PROVISIONAL and needs on-hand tuning
-------------------------------------------------
The joint names are the only description the vendor gives, and the mapping
is reasoned from them, not watched on a hand:

* ``bend`` is the vendor's word for a SIDEWAYS joint: on the full XHAND1,
  ``index_bend`` is the index finger's spread (range -5 .. 17 degrees
  rad, far too small for a flexion). So ``thumb_bend`` is read as the
  thumb's swing across the palm and driven from ``thumb_cm_yaw`` (rotation of
  the thumb towards the palm).
* ``rota`` names the thumb's curling chain: the full XHAND1 has
  ``thumb_rota1`` and ``thumb_rota2`` (base and tip) where the
  Lite has only the first. So ``thumb_rota1`` carries the whole thumb
  flexion and reads the DH116S map's thumb-flexion mean of
  ``thumb_cm_pitch`` and ``thumb_mp_pitch``.

If watching the hand shows the two swapped, swap the sources in
:class:`RetargetConfig`; no code changes. Two glove facts bear on it: the
LitchiBot glove declares ``thumb_cm_pitch`` UNCALIBRATED, so with that glove
``thumb_rota1`` is held until it is re-sourced (for example
``thumb_mp_pitch`` alone), and that glove's ``thumb_mp_pitch`` slot carries the
thumb's last joint.

If a hand's joint rests open at a reading above 0 rad, raise that joint's
``lo`` in the config (per binding: ``[bind.retarget_config.joints.<name>]``).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar, Dict, Tuple

from ...joints import JointSpec
from ..xhand1.retarget import JointMap
from ..xhand1.retarget import RetargetConfig as _XhandConfig
from ..xhand1.retarget import Retargeter as _XhandRetargeter
from .axes import AXIS_NAMES, JOINTS, NUM_AXES  # noqa: F401

_FINGERS = (("index", "index"), ("mid", "middle"), ("ring", "ring"),
            ("pinky", "pinky"))


def default_joint_maps() -> Dict[str, JointMap]:
    """The defaults tabled in the module docstring, keyed by joint name."""
    # open end: 0 rad (the daemon's open preset) where the range contains it
    lim = {j.name: (min(max(0.0, j.min), j.max), j.max) for j in JOINTS}
    maps = {
        "thumb_bend": JointMap((("thumb_cm_yaw", 1.0),),
                                     *lim["thumb_bend"]),
        "thumb_rota1": JointMap(
            (("thumb_cm_pitch", 0.5), ("thumb_mp_pitch", 0.5)),
            *lim["thumb_rota1"]),
    }
    for joint, finger in _FINGERS:
        name = f"{joint}_j1"
        maps[name] = JointMap(
            ((f"{finger}_mp_pitch", 0.5), (f"{finger}_pip_pitch", 0.5),
             (f"{finger}_dip_pitch", 0.0)), *lim[name])
    return maps


@dataclass
class RetargetConfig(_XhandConfig):
    """The XHAND1 Lite's per-joint maps (same checks as the XHAND1's)."""

    JOINTS: ClassVar[Tuple[JointSpec, ...]] = JOINTS

    joints: Dict[str, JointMap] = field(default_factory=default_joint_maps)


class Retargeter(_XhandRetargeter):
    """Canonical hand pose → 6 XHAND1 Lite joint targets in radians.

    Retarget contract version 2 (:data:`JOINTS`, :meth:`required_channels`,
    :meth:`joint_targets`); also callable with a ``flex`` callable for the
    version-1 seam, returning radians (the hand's own wire unit)."""

    JOINTS = JOINTS
    CONFIG = RetargetConfig


def build_retarget(config: "RetargetConfig | None" = None) -> Retargeter:
    """Standard cross-hand retarget constructor (see ``manipulation_kit.hands.get_retarget``)."""
    return Retargeter(config=config)
