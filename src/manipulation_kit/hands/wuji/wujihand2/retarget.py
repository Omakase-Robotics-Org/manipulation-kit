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
``pinky_S2``               ``pinky_mp_swing``               -0.45 .. 0.45
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
  and positive when ring or pinky do. ``pinky_S2`` stops at ±0.45 rad, the
  spread the little finger was seen to reach and hold on the hand. The
  wearer's spread is scaled onto the interval ONCE, by the swing channel's
  range: a glove that measures the wearer's own spread declares it as the
  channel's ``default_range``, and an operator calibration file then carries
  no swing range, because a second, wider range on top of the declared one
  caps the joint short of its interval (a ±0.182 rad declared pinky read
  through a ±0.324 rad file reaches 0.364 of 0.648 rad). A glove that declares its finger swing
  UNCALIBRATED (the LitchiBot glove) has no angle behind it, so a bind that
  follows ``S2`` is refused at bind time
  (``Declaration.check_drive_sources``); hold the four ``S2`` (``hold``) to
  bind with such a glove.
* **The thumb is held at 0 rad** by default, as on the hand: a glove whose
  thumb is uncalibrated has nothing to drive it from. Once the thumb is
  released (``hold`` cleared, or :func:`config_for` with a glove that can
  drive it), it follows two channels:

  =================  ====================  =============
  joint              source                interval (rad)
  =================  ====================  =============
  ``thumb_S1``       ``thumb_cm_yaw``      0 .. 0.8
  ``thumb_S2``       ``thumb_cm_yaw``      0 .. 0.6
  ``thumb_S3``       ``thumb_mp_pitch``    0 .. 1.0
  ``thumb_S4``       ``thumb_mp_pitch``    0 .. 0.6
  =================  ====================  =============

  ``thumb_cm_yaw`` is opposition (0 at an open thumb, 1 at the wearer's
  maximum opposition). On the vendor's right-hand URDF the joint that
  carries the thumb tip across the palm is ``S1`` (``cmc_flex``; positive
  moves the tip towards the palm), and positive ``S2`` (``cmc_abd``) moves
  it towards the index finger; opposition needs both at once, so both read
  the same channel and cannot be moved independently. The tip's two joints
  bend together from the glove's thumb flexion (``thumb_mp_pitch``; on the
  LitchiBot glove that slot carries the thumb's last joint); ``S4`` reads
  the same channel over a shorter interval because the glove has no sensor
  of its own for that joint.
  Every interval lies inside the joint's limits. Forward kinematics of the
  vendor's right-hand URDF puts the thumb tip of a fully opposed, fully
  flexed thumb (all four at their interval's top) 27 mm from the base of
  the little finger; with ``S1`` left at 0 rad the same thumb stops 56 mm
  away, and an open thumb is 147 mm away.
  The thumb's lateral swing (``thumb_cm_swing``) drives nothing.

Absolute and range mapping
--------------------------
``RetargetConfig.mapping`` picks how a finger joint reads its channel:

* ``"absolute"`` (the default): the glove's angle IS the joint angle, 1:1 in
  radians. ``<finger>_S1/S3/S4`` read the finger's ``mp_pitch`` / ``pip_pitch``
  / ``dip_pitch`` and ``<finger>_S2`` its ``mp_swing``; the output is the
  angle less the joint's entry of :attr:`RetargetConfig.zero_rad` (default 0:
  the glove driver's own re-zero already makes an open hand read 0), negated
  where the joint map says ``invert``, clamped to the joint's limits. No
  channel range is read and none is needed, so an operator calibration may
  carry no range for the channels. A joint map's ``lo`` and ``hi`` are not
  used. The angles are RADIANS, so this mode takes them as
  :meth:`Retargeter.joint_targets`'s ``angles`` argument (a
  :class:`~manipulation_kit.gloves.pose.HandAngles`); the flexion ``pose``
  is then read only by the thumb.
* ``"range"``: the table above, exactly: the glove's calibrated range is
  stretched onto the joint's interval. A glove whose measured range is
  narrower than the interval (a spread of 5 to 10 degrees onto ±0.648 rad)
  multiplies its own error by the ratio, 2.4 to 7.3 times on the LitchiBot
  glove's right hand; use it only for a glove whose range is the operator's
  own full range of motion.

The thumb follows the table above in both modes, from the flexion pose:
its sources are opposition (``thumb_cm_yaw``, a fraction of the wearer's
maximum), not an angle, so there is no radian to pass 1:1 — unless the
glove measures the thumb's joints as angles: with
:attr:`RetargetConfig.thumb_absolute` the thumb's joints read their
``sources`` as radians like the fingers (see :mod:`.udcap`). On top of that,
:attr:`RetargetConfig.thumb_pinch` (off by default) blends that 1:1 thumb
towards the thumb that meets the index tip as an opposition angle rises
(:mod:`.pinch`); the four fingers stay 1:1.

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

from ....gloves.pose import HandPose
from ...joints import JointSpec
from ...robotera.xhand1.retarget import JointMap
from ...robotera.xhand1.retarget import RetargetConfig as _XhandConfig
from ...robotera.xhand1.retarget import Retargeter as _XhandRetargeter
from .axes import AXIS_NAMES, JOINT_LIMITS_RAD, JOINTS, NUM_AXES  # noqa: F401
from .pinch import BLEND_LEAD, INDEX_AXES, THUMB_AXES, opposition, pinch_table

#: SDK indices of the thumb's joints and of the index joints the pinch table reads.
_THUMB = tuple(AXIS_NAMES.index(n) for n in THUMB_AXES)
_INDEX = tuple(AXIS_NAMES.index(n) for n in INDEX_AXES)

#: The spread interval: the URDF's ±0.698 rad less a 0.05 rad margin.
SPREAD_RAD = 0.648

#: The little finger's spread interval (see the module docstring).
PINKY_SPREAD_RAD = 0.45

#: The values of :attr:`RetargetConfig.mapping`.
MAPPINGS = ("absolute", "range")

#: The thumb's maps (see the module docstring): joint -> (channel, interval max).
THUMB_MAPS: Dict[str, Tuple[str, float]] = {
    "thumb_S1": ("thumb_cm_yaw", 0.8),
    "thumb_S2": ("thumb_cm_yaw", 0.6),
    "thumb_S3": ("thumb_mp_pitch", 1.0),
    "thumb_S4": ("thumb_mp_pitch", 0.6),
}

_HAND_FINGERS = ("index", "middle", "ring", "pinky")


def default_joint_maps() -> Dict[str, JointMap]:
    """The defaults tabled in the module docstring, keyed by joint name."""
    hi = {j.name: j.max for j in JOINTS}
    maps = {name: JointMap(((channel, 1.0),), 0.0, top)
            for name, (channel, top) in THUMB_MAPS.items()}
    for f in _HAND_FINGERS:
        maps[f"{f}_S1"] = JointMap(((f"{f}_mp_pitch", 1.0),), 0.0, hi[f"{f}_S1"])
        spread = PINKY_SPREAD_RAD if f == "pinky" else SPREAD_RAD
        maps[f"{f}_S2"] = JointMap(((f"{f}_mp_swing", 1.0),), -spread, spread)
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
    #: ``"absolute"`` (glove radians 1:1) or ``"range"`` (calibrated range
    #: stretched onto the joint's interval); see the module docstring.
    mapping: str = "absolute"
    #: Per joint, SDK order: radians subtracted from the glove's angle before
    #: it is applied, in absolute mode only.
    zero_rad: Tuple[float, ...] = (0.0,) * NUM_AXES
    #: ``absolute`` mapping only: whether the thumb's joints read the glove's
    #: angles 1:1 like the fingers' (their ``sources`` then name angle
    #: channels, as :func:`manipulation_kit.hands.wuji.wujihand2.udcap.udcap_config`
    #: sets them) instead of the opposition fractions of the table above.
    thumb_absolute: bool = False
    #: ``absolute`` mapping with ``thumb_absolute`` only: blend the 1:1 thumb
    #: towards the thumb that meets the index (:mod:`.pinch`) as the
    #: ``pinch_channel`` angle rises from ``pinch_open_rad`` to
    #: ``pinch_closed_rad``. The four fingers are not affected.
    thumb_pinch: bool = False
    #: The glove angle that says how far the thumb is opposed, rad.
    pinch_channel: str = "thumb_cm_pitch"
    #: ``pinch_channel`` at which the thumb is still the 1:1 thumb (``o = 0``).
    pinch_open_rad: float = 0.15
    #: ``pinch_channel`` at which the thumb is the pinch thumb (``o = 1``).
    pinch_closed_rad: float = 0.30
    #: The most a thumb joint's target moves per call while ``thumb_pinch``
    #: is on, rad (0 = no bound). It bounds the step a fast change of
    #: opposition or of the index's curl makes; the consumer's own velocity
    #: limit still applies on top.
    pinch_max_step_rad: float = 0.1

    def __post_init__(self):
        super().__post_init__()
        if self.thumb_pinch and not (self.mapping == "absolute" and self.thumb_absolute):
            raise ValueError("thumb_pinch needs mapping = \"absolute\" and thumb_absolute")
        if not (math.isfinite(self.pinch_open_rad) and math.isfinite(self.pinch_closed_rad)
                and self.pinch_open_rad < self.pinch_closed_rad):
            raise ValueError(f"pinch_open_rad ({self.pinch_open_rad}) must be below "
                             f"pinch_closed_rad ({self.pinch_closed_rad})")
        if not (math.isfinite(self.pinch_max_step_rad) and self.pinch_max_step_rad >= 0):
            raise ValueError(f"pinch_max_step_rad must be >= 0, got {self.pinch_max_step_rad}")
        if self.mapping not in MAPPINGS:
            raise ValueError(f"mapping must be one of {MAPPINGS}, got {self.mapping!r}")
        if len(self.zero_rad) != len(self.JOINTS):
            raise ValueError(f"zero_rad needs {len(self.JOINTS)} entries, "
                             f"got {len(self.zero_rad)}")
        if not all(math.isfinite(v) for v in self.zero_rad):
            raise ValueError("zero_rad must be finite")
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
        if self._pinching() and self.cfg.pinch_channel not in out:
            out.append(self.cfg.pinch_channel)
        return tuple(out)

    def _pinching(self) -> bool:
        """Whether the pinch-aware thumb is on and some thumb joint is driven."""
        return self.cfg.thumb_pinch and not all(self.cfg.hold[i] for i in _THUMB)

    _last_thumb: Optional[List[Optional[float]]] = None

    def reset(self) -> None:
        super().reset()
        self._last_thumb = None

    def _pinch(self, out: List[Optional[float]], angles) -> None:
        """Blend the 1:1 thumb in ``out`` towards the pinch thumb, in place
        (see :mod:`.pinch`), then bound each thumb joint's step."""
        cfg = self.cfg
        o = opposition(angles.get(cfg.pinch_channel), cfg.pinch_open_rad, cfg.pinch_closed_rad)
        index = [out[i] for i in _INDEX]
        pinch = None
        if o is not None and o > 0.0 and None not in index:
            pinch = pinch_table().thumb_for(*index)
        last = self._last_thumb or [None] * len(_THUMB)
        step = cfg.pinch_max_step_rad
        for n, i in enumerate(_THUMB):
            if cfg.hold[i]:
                continue
            on = None if o is None else min(1.0, BLEND_LEAD[n] * o)
            if on is None or (on > 0.0 and pinch is None) or (on < 1.0 and out[i] is None):
                out[i] = None  # nothing to aim with: the consumer holds the joint
                continue
            if on > 0.0:
                one = out[i] if on < 1.0 else 0.0
                out[i] = self.JOINTS[i].clamp((1.0 - on) * one + on * pinch[n])
            if step > 0.0 and last[n] is not None:
                out[i] = min(last[n] + step, max(last[n] - step, out[i]))
            last[n] = out[i]
        self._last_thumb = last

    _plain = False

    def _smooth(self, targets):
        return targets if self._plain else super()._smooth(targets)

    def flex_map(self) -> "Retargeter":
        """This map in ``range`` mode (``self`` when it already is): the
        flexion-space arithmetic :meth:`table_rows` and the golden vectors
        describe."""
        if self.cfg.mapping == "range":
            return self
        return Retargeter(replace(self.cfg, mapping="range", thumb_pinch=False))

    def __call__(self, flex):
        """Version-1 seam: flexion in, radians out, always the range map."""
        return self.flex_map()._call_range(flex)

    def _call_range(self, flex):
        return super().__call__(flex)

    def joint_targets(self, pose, angles=None) -> List[Optional[float]]:
        """One canonical hand pose → twenty ``rad`` targets or ``None``.

        ``range`` mapping reads ``pose`` (flexion in [0, 1]) only. ``absolute``
        mapping reads the thumb from ``pose`` and every other joint from
        ``angles``, the glove's radians (anything with ``get(channel)`` in
        radians, such as :class:`~manipulation_kit.gloves.pose.HandAngles`);
        calling it without ``angles`` is an error, not a hand held open."""
        if self.cfg.mapping == "range":
            return self._range_targets(pose)
        if angles is None:
            raise ValueError("absolute mapping reads the glove's angles in radians: "
                             "pass angles=HandAngles (the flexion pose has no radians)")
        if isinstance(angles, HandPose):
            raise TypeError("angles must be radians (HandAngles); got a flexion HandPose")
        self._plain = True  # the thumb is read unsmoothed; the whole row is smoothed below
        try:
            thumb = {ch for spec, held in zip(self.JOINTS, self.cfg.hold)
                     if not held and spec.name.startswith("thumb_")
                     and not self.cfg.thumb_absolute
                     for ch, w in self.cfg.joints[spec.name].sources if w > 0}
            flex = self._range_targets(pose if pose is not None else {}, thumb)
        finally:
            self._plain = False
        out: List[Optional[float]] = []
        for i, (spec, held) in enumerate(zip(self.JOINTS, self.cfg.hold)):
            if held or (spec.name.startswith("thumb_") and not self.cfg.thumb_absolute):
                out.append(flex[i])
                continue
            m = self.cfg.joints[spec.name]
            total = acc = 0.0
            for ch, w in m.sources:
                if w <= 0:
                    continue
                v = angles.get(ch)
                if v is None or not math.isfinite(v):
                    acc = None
                    break
                acc += w * v
                total += w
            if acc is None:
                out.append(None)
                continue
            rad = acc / total - self.cfg.zero_rad[i]
            out.append(spec.clamp(-rad if m.invert else rad))
        if self._pinching():
            self._pinch(out, angles)
        return self._smooth(out)

    def _range_targets(self, pose, channels=None) -> List[Optional[float]]:
        # A held joint reads nothing: its channels are never asked of the
        # pose, so a calibration with no range for them (a glove that
        # declares finger swing uncalibrated, with the spreads held) is not
        # refused for a channel nothing drives.
        out = super().joint_targets(
            _Only(pose, self.required_channels() if channels is None else channels))
        for i, held in enumerate(self.cfg.hold):
            if held:
                out[i] = float(self.cfg.hold_rad[i])
        return out


    def table_rows(self):
        """As the XHAND1's, with each held joint written as held."""
        rows = super().table_rows()
        return tuple(replace(r, hold=float(v)) if held else r
                     for r, held, v in zip(rows, self.cfg.hold, self.cfg.hold_rad))


def config_for(declaration, config: "RetargetConfig | None" = None) -> RetargetConfig:
    """``config`` (default: the defaults) adapted to what ``declaration`` says.

    With the default ``config``, the hold on all four thumb joints is cleared
    when the declaration can drive every channel their maps read
    (``thumb_cm_yaw`` and ``thumb_mp_pitch``); otherwise the thumb stays held.
    A config the caller passes is returned unchanged. Nothing is loosened: a
    channel the declaration refuses still makes
    ``declaration.check_drive_sources(Retargeter(result).required_channels())``
    refuse the bind, with the reason."""
    cfg = config if config is not None else RetargetConfig()
    if config is None:
        hold = list(cfg.hold)
        names = ("thumb_S1", "thumb_S2", "thumb_S3", "thumb_S4")
        if all(declaration.refuse_as_drive_source(ch) is None
               for n in names for ch, w in cfg.joints[n].sources if w > 0):
            for n in names:
                hold[AXIS_NAMES.index(n)] = False
        cfg = replace(cfg, hold=tuple(hold))
    return cfg


def build_retarget(config: "RetargetConfig | None" = None) -> Retargeter:
    """Standard cross-hand retarget constructor (see ``manipulation_kit.hands.get_retarget``)."""
    return Retargeter(config=config)
