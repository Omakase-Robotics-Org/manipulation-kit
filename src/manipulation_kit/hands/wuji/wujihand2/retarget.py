"""Anatomical flexion channels → Wuji Hand 2 20-joint retarget map.

The map has the XHAND1's shape and reuses its arithmetic
(:mod:`manipulation_kit.hands.robotera.xhand1.retarget`): every joint a
weighted mean of channels scaled onto a radians interval, clamped to the
joint's limits, ``None`` when an input has no reading. It adds one thing the
XHAND1 map has no use for: a joint can be HELD at a fixed angle
(:attr:`RetargetConfig.hold`, :attr:`RetargetConfig.hold_rad`), reading no
channel at all. A held joint is not ``None`` — the consumer never has to
invent a value for it — and its channels are not required.

The defaults
------------
They are the table that drove a right Wuji Hand 2 from the LitchiBot glove
on 2026-09-25, split along this package's layers: the glove side of each row
(which angle reads as open and which as full) is the operator's
calibration, the hand side (which joint, which interval) is here.

=========================  ===============================  =================
joint                      default source                   interval (rad)
=========================  ===============================  =================
``<finger>_S1``            ``<finger>_mp_pitch``            0 .. 1.570
``<finger>_S2``            ``<finger>_mp_swing``            -0.648 .. 0.648
``<finger>_S3``            ``<finger>_pip_pitch``           0 .. 2.094
``<finger>_S4``            ``<finger>_dip_pitch``           0 .. 1.570
``thumb_S1`` .. ``S4``     HELD at 0 rad                    (see below)
=========================  ===============================  =================

(``<finger>`` is index, middle, ring, pinky.)

* **Flexion** runs from 0 rad (open; the vendor's flat hand) to the joint's
  maximum, one channel per joint: the Wuji has a motor at every finger
  joint, so unlike the DH116S and XHAND1 maps nothing is averaged.
* **Spread.** ``S2`` reads the bipolar swing channel, whose neutral is 0.5,
  so a neutral finger lands on 0 rad. ±0.648 rad is the URDF's ±0.698 less
  a 0.05 rad margin. The sign was checked on the vendor's model of the
  right hand and on the right glove: the glove's swing and the Wuji's
  ``S2`` are both negative when the index moves away from the middle finger
  and positive when ring or pinky do. The table that ran
  doubled the glove's swing (``S2 = 2 x swing``, saturating at ±18.564
  degrees of swing); that gain is the calibration range of the swing channel
  (±0.324 rad), not a number here. **Both gloves this package decodes declare
  their finger swing UNCALIBRATED today**, so a bind that follows ``S2``
  is refused at bind time (``Declaration.check_drive_sources``); hold the
  four ``S2`` (``hold``) to bind with such a glove.
* **The thumb is held at 0 rad**, as on the hand: the LitchiBot glove's
  thumb was not usable (its fitter leaves the thumb uncalibrated, and the
  glove declares ``thumb_cm_pitch`` uncalibrated). The thumb's maps below are
  PROVISIONAL — reasoned from the vendor's joint names and the URDF's
  kinematics, never watched on a hand — and are read only once ``hold`` is
  cleared for that joint: ``S1`` (base flexion; positive moves the tip
  towards the palm) from ``thumb_cm_pitch``, ``S2`` (base abduction;
  positive moves the tip towards the index finger) from ``thumb_cm_yaw``
  over 0 .. 0.698, ``S3`` from ``thumb_mp_pitch``, ``S4`` from
  ``thumb_ip_pitch``. Note that the LitchiBot glove's ``thumb_mp_pitch``
  slot carries the thumb's LAST joint.

Sides: the vendor's left and right URDFs share limits and anatomical
directions (:mod:`.axes`), so this one map serves both hands. Whether the
LitchiBot glove's LEFT hand reports swing with the same anatomical sign as
its right has not been checked; if a left hand spreads the wrong way, set
``invert`` on that side's ``S2`` maps in the left bind's
``retarget_config``.

Everything above is :class:`RetargetConfig` data, so bring-up changes no
code (per binding: ``[bind.retarget_config]``, with ``hold = [...]`` and
``hold_rad = [...]`` as arrays of twenty).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import ClassVar, Dict, List, Optional, Tuple

from ...joints import JointSpec
from ...robotera.xhand1.retarget import JointMap
from ...robotera.xhand1.retarget import RetargetConfig as _XhandConfig
from ...robotera.xhand1.retarget import Retargeter as _XhandRetargeter
from .axes import AXIS_NAMES, JOINT_LIMITS_RAD, JOINTS, NUM_AXES  # noqa: F401

#: The spread interval: the URDF's ±0.698 rad less a 0.05 rad margin.
SPREAD_RAD = 0.648

_HAND_FINGERS = ("index", "middle", "ring", "pinky")


def default_joint_maps() -> Dict[str, JointMap]:
    """The defaults tabled in the module docstring, keyed by joint name."""
    hi = {j.name: j.max for j in JOINTS}
    maps = {
        "thumb_S1": JointMap((("thumb_cm_pitch", 1.0),), 0.0, hi["thumb_S1"]),
        "thumb_S2": JointMap((("thumb_cm_yaw", 1.0),), 0.0, hi["thumb_S2"]),
        "thumb_S3": JointMap((("thumb_mp_pitch", 1.0),), 0.0, hi["thumb_S3"]),
        "thumb_S4": JointMap((("thumb_ip_pitch", 1.0),), 0.0, hi["thumb_S4"]),
    }
    for f in _HAND_FINGERS:
        maps[f"{f}_S1"] = JointMap(((f"{f}_mp_pitch", 1.0),), 0.0, hi[f"{f}_S1"])
        maps[f"{f}_S2"] = JointMap(((f"{f}_mp_swing", 1.0),), -SPREAD_RAD, SPREAD_RAD)
        maps[f"{f}_S3"] = JointMap(((f"{f}_pip_pitch", 1.0),), 0.0, hi[f"{f}_S3"])
        maps[f"{f}_S4"] = JointMap(((f"{f}_dip_pitch", 1.0),), 0.0, hi[f"{f}_S4"])
    return maps


#: The thumb's four joints are held; every other joint follows its channel.
DEFAULT_HOLD: Tuple[bool, ...] = tuple(name.startswith("thumb_") for name in AXIS_NAMES)


@dataclass
class RetargetConfig(_XhandConfig):
    """The Wuji Hand 2's per-joint maps, plus which joints are held.

    Same checks as the XHAND1's, and: ``hold`` and ``hold_rad`` have one
    entry per joint, and every ``hold_rad`` lies inside its joint's limits."""

    JOINTS: ClassVar[Tuple[JointSpec, ...]] = JOINTS

    joints: Dict[str, JointMap] = field(default_factory=default_joint_maps)
    #: Per joint, SDK order: ``True`` = command ``hold_rad`` and read nothing.
    hold: Tuple[bool, ...] = DEFAULT_HOLD
    #: Per joint, SDK order: the angle a held joint is commanded to.
    hold_rad: Tuple[float, ...] = (0.0,) * NUM_AXES

    def __post_init__(self):
        super().__post_init__()
        for what, values in (("hold", self.hold), ("hold_rad", self.hold_rad)):
            if len(values) != len(self.JOINTS):
                raise ValueError(f"{what} needs {len(self.JOINTS)} entries, got {len(values)}")
        for spec, value in zip(self.JOINTS, self.hold_rad):
            if not (math.isfinite(value) and spec.min <= value <= spec.max):
                raise ValueError(f"{spec.name}: hold_rad {value} leaves the joint's "
                                 f"limits {spec.min}..{spec.max}")


class _Only:
    """``pose`` restricted to ``channels``: any other channel reads ``None``."""

    def __init__(self, pose, channels) -> None:
        self._pose = pose
        self._channels = frozenset(channels)

    def get(self, channel, default=None):
        return self._pose.get(channel) if channel in self._channels else default


class Retargeter(_XhandRetargeter):
    """Canonical hand pose → 20 Wuji Hand 2 joint targets in radians.

    Retarget contract version 2 (:data:`JOINTS`, :meth:`required_channels`,
    :meth:`joint_targets`); also callable with a ``flex`` callable for the
    version-1 seam, returning radians (the hand's own command unit)."""

    JOINTS = JOINTS
    CONFIG = RetargetConfig

    def required_channels(self) -> Tuple[str, ...]:
        out: List[str] = []
        for spec, held in zip(self.JOINTS, self.cfg.hold):
            if held:
                continue
            for ch, w in self.cfg.joints[spec.name].sources:
                if w > 0 and ch not in out:
                    out.append(ch)
        return tuple(out)

    def joint_targets(self, pose) -> List[Optional[float]]:
        """One canonical hand pose → twenty ``rad`` targets or ``None``."""
        # A held joint reads nothing: its channels are never asked of the
        # pose, so a calibration with no range for them (a glove that
        # declares finger swing uncalibrated, with the spreads held) is not
        # refused for a channel nothing drives.
        out = super().joint_targets(_Only(pose, self.required_channels()))
        for i, held in enumerate(self.cfg.hold):
            if held:
                out[i] = float(self.cfg.hold_rad[i])
        return out


def build_retarget(config: "RetargetConfig | None" = None) -> Retargeter:
    """Standard cross-hand retarget constructor (see ``manipulation_kit.hands.get_retarget``)."""
    return Retargeter(config=config)
