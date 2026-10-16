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
  (±0.324 rad), not a number here. A glove that declares its finger swing
  UNCALIBRATED (the LitchiBot glove) has no angle behind it, so a bind that
  follows ``S2`` is refused at bind time
  (``Declaration.check_drive_sources``); hold the four ``S2`` (``hold``) to
  bind with such a glove. A glove that measures the index, ring and pinky
  spreads but declares the middle one SYNTHESIZED from the index and ring
  (the UDCAP glove, newer driver) is refused for ``middle_mp_swing`` alone
  ("drive from those instead"); :func:`config_for` rewrites such a source
  into the channels it was computed from, so ``middle_S2`` follows the mean
  of the index and ring spreads (provisional: the middle finger stays
  between them), and :func:`config_for` leaves every refusal that is not
  of that kind for ``check_drive_sources`` to report.
* **The thumb is held at 0 rad** by default, as on the hand: the
  LitchiBot glove's thumb used to be unusable (its fitter left the thumb
  uncalibrated). The thumb's maps below are PROVISIONAL, reasoned from the
  vendor's joint names and the URDF's kinematics, never watched on a hand,
  and are read only once ``hold`` is cleared for that joint: ``S1`` (base
  flexion; positive moves the tip towards the palm) from ``thumb_cm_pitch``,
  ``S2`` (base abduction; positive moves the tip towards the index finger)
  from ``thumb_cm_yaw`` over 0 .. 0.698, ``S3`` from ``thumb_mp_pitch``,
  ``S4`` from ``thumb_ip_pitch``. Note that the LitchiBot glove's
  ``thumb_mp_pitch`` slot carries the thumb's LAST joint.
* **A glove that solves the thumb on the wearer** (the LitchiBot driver
  now: ``thumb_cm_yaw``, ``thumb_cm_swing`` and ``thumb_mp_pitch`` DERIVED,
  ``thumb_cm_pitch`` UNCALIBRATED, ``thumb_ip_pitch`` not declared) drives
  the thumb through :func:`config_for`:

  - ``S1`` reads ``thumb_cm_swing`` over -0.436 .. 0.436 rad (the swing
    channel's ±25 degrees; bipolar, so a neutral thumb lands on 0 rad, well
    inside S1's -1.187 .. 1.291). It replaces ``thumb_cm_pitch`` only when
    the declaration refuses ``thumb_cm_pitch`` and accepts
    ``thumb_cm_swing``; a glove that measures ``thumb_cm_pitch`` keeps it.
    The sign is an assumption: positive swing is taken to drive positive
    ``S1``. It was not watched on a hand; if the thumb swings the wrong way,
    set ``invert`` on that ``thumb_S1`` map in the bind's
    ``retarget_config``.
  - ``S2`` reads ``thumb_cm_yaw`` (0 at open, 1 at the wearer's maximum
    opposition) onto 0 .. 0.698, and ``S3`` reads ``thumb_mp_pitch`` onto
    0 .. 1.570 (its channel range is 0 .. 1.571).
  - ``S4`` stays held at 0 rad: the glove has no sensor for the thumb's
    last-but-one joint, and the kit has no coupling that derives a joint
    from another one.
  - With ``config`` left at its default, :func:`config_for` also clears the
    hold on ``S1`` .. ``S3``, but only when the declaration can drive all
    three; ``S4`` is cleared too when it can drive ``thumb_ip_pitch``. A
    thumb whose channels are UNCALIBRATED or undeclared keeps the default
    hold. A config the caller passes keeps its own ``hold``, so a bind that
    also holds the finger spreads clears ``S1`` .. ``S3`` itself (see
    :func:`config_for`).

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
from dataclasses import dataclass, field, replace
from typing import ClassVar, Dict, List, Optional, Tuple

from ....gloves.v1 import Provenance
from ...joints import JointSpec
from ...robotera.xhand1.retarget import JointMap
from ...robotera.xhand1.retarget import RetargetConfig as _XhandConfig
from ...robotera.xhand1.retarget import Retargeter as _XhandRetargeter
from .axes import AXIS_NAMES, JOINT_LIMITS_RAD, JOINTS, NUM_AXES  # noqa: F401

#: The spread interval: the URDF's ±0.698 rad less a 0.05 rad margin.
SPREAD_RAD = 0.648

#: The thumb swing interval: the LitchiBot glove's ±25 degrees of wearer swing.
THUMB_SWING_RAD = math.radians(25.0)

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


def config_for(declaration, config: "RetargetConfig | None" = None) -> RetargetConfig:
    """``config`` (default: the defaults) adapted to what ``declaration`` says.

    A channel the map reads that the device declares SYNTHESIZED from channels
    it also reports as measured is refused as a drive source (reading the
    sum would discard those measurements). Here such a source is replaced by
    the channels it was computed from, sharing its weight equally; a joint
    held by ``config`` is left alone. Nothing else is loosened: an
    UNCALIBRATED channel, an undeclared one, or a synthesized one with a
    missing input stays as it is, so ``declaration.check_drive_sources(
    Retargeter(result).required_channels())`` still refuses it, with the
    reason.

    The thumb: see the module docstring. ``thumb_S1`` is re-sourced to
    ``thumb_cm_swing`` when the glove refuses ``thumb_cm_pitch`` and accepts
    ``thumb_cm_swing`` (whatever the hold), and with the default ``config`` the
    hold on the thumb is cleared as the docstring says."""
    cfg = config if config is not None else RetargetConfig()
    joints = dict(cfg.joints)
    # A thumb base flexion the glove does not measure, but whose abduction
    # it does: drive S1 from the abduction (see the module docstring).
    s1 = joints["thumb_S1"]
    if (s1.sources == default_joint_maps()["thumb_S1"].sources
            and declaration.refuse_as_drive_source("thumb_cm_pitch") is not None
            and declaration.refuse_as_drive_source("thumb_cm_swing") is None):
        joints["thumb_S1"] = JointMap((("thumb_cm_swing", 1.0),),
                                      -THUMB_SWING_RAD, THUMB_SWING_RAD)
    if config is None:
        hold = list(cfg.hold)
        names = ("thumb_S1", "thumb_S2", "thumb_S3", "thumb_S4")
        drivable = [all(declaration.refuse_as_drive_source(ch) is None
                        for ch, w in joints[n].sources if w > 0) for n in names]
        if all(drivable[:3]):
            for n, ok in zip(names, drivable):
                if ok:
                    hold[AXIS_NAMES.index(n)] = False
        cfg = replace(cfg, hold=tuple(hold))
    for spec, held in zip(JOINTS, cfg.hold):
        if held:
            continue
        sources: List[Tuple[str, float]] = []
        changed = False
        for ch, w in joints[spec.name].sources:
            declared = declaration.channels.get(ch)
            inputs = declared.derived_from if declared is not None else ()
            if (w > 0 and declared is not None
                    and declared.provenance is Provenance.SYNTHESIZED
                    and declaration.refuse_as_drive_source(ch) is not None
                    and inputs
                    and all(declaration.refuse_as_drive_source(i) is None for i in inputs)):
                sources.extend((i, w / len(inputs)) for i in inputs)
                changed = True
            else:
                sources.append((ch, w))
        if changed:
            joints[spec.name] = replace(joints[spec.name], sources=tuple(sources))
    return replace(cfg, joints=joints)


def build_retarget(config: "RetargetConfig | None" = None) -> Retargeter:
    """Standard cross-hand retarget constructor (see ``manipulation_kit.hands.get_retarget``)."""
    return Retargeter(config=config)
