"""The UDCAP data glove driving a Wuji Hand 2: the absolute (1:1) map.

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
* the thumb: ``thumb_cm_pitch`` (flexion at the base joint), ``thumb_mp_pitch``,
  ``thumb_ip_pitch``, ``thumb_cm_yaw`` (the base joint's second axis; the
  driver removes the vendor's +20 degree rest offset) and ``thumb_cm_roll``,
  all measured.

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
    """The absolute map for a UDCAP glove (see the module docstring).

    ``overrides`` are fields of :class:`RetargetConfig` applied on top
    (``zero_rad``, ``ema_alpha``, a joint's ``invert`` through ``joints``)."""
    joints = default_joint_maps()
    for joint, channel in THUMB_SOURCES.items():
        joints[joint] = JointMap(((channel, 1.0),), joints[joint].lo, joints[joint].hi)
    hold = tuple(name in HELD for name in AXIS_NAMES)
    cfg = RetargetConfig(joints=joints, hold=hold, mapping="absolute", thumb_absolute=True)
    return replace(cfg, **overrides) if overrides else cfg


__all__ = ["HELD", "THUMB_SOURCES", "udcap_config"]
