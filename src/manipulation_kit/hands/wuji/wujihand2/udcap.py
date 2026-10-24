"""The UDCAP data glove driving a Wuji Hand 2: 1:1 fingers, a pinch-aware thumb.

What the UDCAP glove stream publishes, per hand, in the shared glove channel
vocabulary and in radians (0 = extended, positive towards the palm; zero is
the vendor open pose, offsets already removed by the driver):

* the three joints of each finger, ``<finger>_mp_pitch`` / ``_pip_pitch`` /
  ``_dip_pitch``, measured;
* the spreads ``index_mp_swing`` (negative as the finger moves away from the
  middle finger), ``ring_mp_swing`` and ``pinky_mp_swing`` (positive as they
  do), measured, zero at the vendor 3-pose calibration's "fingers together"
  pose; the glove's solver clamps them to index 25, ring 15 and pinky 35
  degrees;
* ``middle_mp_swing``, which the vendor computes from the index and ring
  spreads (``0.24 * index - 0.32 * ring``): declared synthesized, so a bind
  that reads it is refused (``Declaration.check_drive_sources``);
* the thumb: ``thumb_cm_pitch`` (flexion at the base joint), ``thumb_mp_pitch``
  and ``thumb_ip_pitch``, from the glove's thumb sensors, and
  ``thumb_cm_yaw`` (the base joint's second axis; the driver removes the
  vendor's +20 degree rest offset) and ``thumb_cm_roll``, which are NOT
  measured although the driver declares them so: the vendor's solver computes
  them from ``thumb_cm_pitch``. Its row for the yaw has a zero weight on every
  sensor, so the yaw is ``min(2.5 * thumb_cm_pitch, 0.611)`` rad plus a term
  from the little finger's spread, and the roll is ``0.6 * thumb_cm_pitch``
  less 5 degrees. No sensor reports how far the thumb is opposed.

:func:`udcap_config` is the :class:`RetargetConfig` that follows them:

=========================  ===============================  ==============
joint                      source (radians, 1:1)            note
=========================  ===============================  ==============
``<finger>_S1/S3/S4``      ``<finger>_mp/pip/dip_pitch``    as the default
``index_S2``               ``index_mp_swing``               sign as the glove
``ring_S2``, ``pinky_S2``  ``<finger>_mp_swing``            sign as the glove
``middle_S2``              HELD at 0 rad                    see below
``thumb_S1``               ``thumb_cm_pitch``               ``cmc_flex``
``thumb_S2``               ``thumb_cm_yaw``                 ``cmc_abd``
``thumb_S3``               ``thumb_mp_pitch``               ``mcp``
``thumb_S4``               ``thumb_ip_pitch``               ``ip``
=========================  ===============================  ==============

and on top of the thumb's four, the pinch-aware thumb
(:attr:`RetargetConfig.thumb_pinch`, :mod:`.pinch`): as ``thumb_cm_pitch``
rises from 0.15 to 0.30 rad the thumb moves from the 1:1 thumb to the thumb
that meets the Wuji's index tip.

* **Signs.** The glove's spread sign and the Wuji's ``S2`` agree (negative
  when the index moves away from the middle finger, positive for ring and
  pinky), and positive flexion is towards the palm on both, so every joint is
  ``+1``. A hand whose spread reads the other way sets ``invert`` on that
  joint of its bind's ``retarget_config``.
* **The middle finger's spread is held at 0.** The glove's spreads are
  measured against the middle finger, so the middle finger is the reference;
  its own number is a computed one that the declaration refuses as a drive
  source.
* **The thumb is driven too**, from the glove's own thumb angles, which the
  LitchiBot map could not do. ``thumb_S3`` and ``thumb_S4`` follow the thumb's
  second and last joint (the joints are named alike on both). ``thumb_S1``
  follows ``thumb_cm_pitch`` (flexion at the base: positive towards the palm
  on both). ``thumb_S2`` follows ``thumb_cm_yaw`` with sign ``+1``: that is the
  one pairing that is a judgement and not a stated fact (the glove calls the
  axis yaw, the vendor URDF calls the joint ``cmc_abd``, positive towards the
  index finger), and has not been seen on a hand: invert it on the bind if the
  thumb swings the wrong way. ``thumb_cm_roll`` drives nothing. Each thumb
  joint is clamped to the Wuji's limits, narrower than what the glove reads
  (``thumb_cm_yaw`` reads up to 0.96 rad, ``thumb_S2`` stops at 0.698).
* **The thumb pinches.** The 1:1 thumb alone cannot: over an operator's
  recorded pinches (index tip and thumb tip touching on the glove) forward
  kinematics of the vendor's right-hand URDF put the Wuji's thumb tip 81 mm
  (median) from its index tip, and a positive ``thumb_S2`` (the computed yaw
  never goes negative) pushes the thumb into the side of the index. So
  ``thumb_pinch`` is on: opposition is read from ``thumb_cm_pitch`` (the
  glove's one measured opposition signal; the recorded open hand reads 0.07
  rad median, the pinches 0.42), ``pinch_open_rad = 0.15`` and
  ``pinch_closed_rad = 0.30`` bound the blend, and each thumb joint moves at
  most ``pinch_max_step_rad = 0.1`` rad per call. Over the same recording the
  pinches come to 13 mm (median, tip to tip); an open hand moves 7 mm
  (median 78 to 71 mm) because some open-hand frames read a ``thumb_cm_pitch``
  above 0.15. ``thumb_pinch = False`` restores the plain 1:1 thumb.
  ``pinch_open_rad`` and ``pinch_closed_rad`` are per operator: set them in
  the binding.
* **Which finger, per operator.** For some operators ``thumb_cm_pitch``
  reads the same with an open hand and a pinch (0.31 and 0.30 rad in one
  calibration session), and the glove has no sensor for which finger the
  thumb opposes. A binding that carries a fitted ``thumb_model``
  (:class:`.pinch.ThumbModel`) reads that from the whole hand instead and
  sends the thumb to whichever of the four fingers it names;
  ``udcap_config()`` carries none, since the model is the operator's.
* **No channel range is read.** The glove's declared ranges are its catalogue
  ranges (a finger's full flexion), not the operator's, which is why the
  absolute mapping does not use them.
"""
from __future__ import annotations

from dataclasses import replace
from typing import Dict

from ...robotera.xhand1.retarget import JointMap
from .axes import AXIS_NAMES
from .retarget import RetargetConfig, default_joint_maps

#: The UDCAP thumb: Wuji thumb joint -> the glove's angle channel it follows.
THUMB_SOURCES: Dict[str, str] = {
    "thumb_S1": "thumb_cm_pitch",
    "thumb_S2": "thumb_cm_yaw",
    "thumb_S3": "thumb_mp_pitch",
    "thumb_S4": "thumb_ip_pitch",
}

#: Held at 0 rad: the middle finger is the reference of the glove's spreads.
HELD = ("middle_S2",)


def udcap_config(**overrides) -> RetargetConfig:
    """The map for a UDCAP glove (see the module docstring).

    ``overrides`` are fields of :class:`RetargetConfig` applied on top
    (``zero_rad``, ``ema_alpha``, a joint's ``invert`` through ``joints``,
    an operator's ``pinch_open_rad`` / ``pinch_closed_rad``, or
    ``thumb_pinch=False`` for the plain 1:1 thumb)."""
    joints = default_joint_maps()
    for joint, channel in THUMB_SOURCES.items():
        joints[joint] = JointMap(((channel, 1.0),), joints[joint].lo, joints[joint].hi)
    hold = tuple(name in HELD for name in AXIS_NAMES)
    cfg = RetargetConfig(joints=joints, hold=hold, mapping="absolute", thumb_absolute=True,
                         thumb_pinch=True, pinch_channel="thumb_cm_pitch",
                         pinch_open_rad=0.15, pinch_closed_rad=0.30, pinch_max_step_rad=0.1)
    return replace(cfg, **overrides) if overrides else cfg


__all__ = ["HELD", "THUMB_SOURCES", "udcap_config"]
