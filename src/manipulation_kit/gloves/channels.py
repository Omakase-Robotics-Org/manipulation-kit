"""The anatomical channel vocabulary a hand retarget map is written against.

A glove driver publishes RADIANS per anatomical channel; a hand retarget map
asks for FLEXION per anatomical channel. This table is the shared name space
in between, and it answers three questions about a channel name without
knowing which glove is answering it:

1. what joint and what motion it names (:class:`ChannelSpec`);
2. which ``teleop_gloves.pose_stream.v1`` slot it is published in
   (``finger_angles_rad.<finger>.<field>``), or that v1 has none;
3. whether the older twenty-four-name vocabulary (the one the DH116S retarget
   map was first written against) has a name for it.

Source of truth
---------------
:data:`CHANNELS` mirrors the channel table of the glove stream encoder, a
Rust library the glove drivers publish through. It is copied, not imported,
because that library is Rust and this package has no dependency beyond numpy
and scipy. ``tests/gloves/test_channels.py`` parses that Rust table and
compares it with this one entry by entry when ``MKIT_GLOVE_REFERENCE_DIR``
holds a copy of it (``channels.rs``), so a change on one side and not the
other is a test failure, not a comment that drifted. Update both together.

The vocabulary is a SUPERSET, on purpose
----------------------------------------
It carries names no single glove reports. The older twenty-four names were
derived from one glove, which has no sensor at the thumb's last joint; another
glove does, and under the older names that reading would be published and
then dropped because no consumer named it. So the vocabulary is anatomical:
what a hand HAS, not what one glove reads. A channel a device does not declare
is simply absent — never a stand-in zero.

Directions and units
--------------------
Every channel is radians on the wire. Every FLEXION channel runs ``0.0 =
extended`` and positive towards the palm (the v1 convention), so a glove
driver does all the sign work and nothing downstream has to. SWING channels
are signed about a neutral hand and have no "open" end: normalised against a
signed range they land a neutral finger at 0.5, not 0.0.

One extension the Rust table does not carry
-------------------------------------------
:data:`EXTENSION_CHANNELS` holds ``thumb_cm_swing``, the thumb ABDUCTION the
LitchiBot glove driver declares in v1's ``thumb.swing`` slot. The encoder's
table fills that slot with ``thumb_cm_roll`` (an axial roll, which is what the
UDCAP glove measures there) and refuses any name outside the table. The two are different physical quantities sharing one wire
slot; nothing in a pose packet says which arrived, and only the device's
``channel_declaration`` does. Keeping the extension separate is what lets the
drift test compare :data:`CHANNELS` with the encoder's table exactly, while the
decoder (:mod:`manipulation_kit.gloves.v1`) still accepts what LitchiBot
actually sends. Fold it into :data:`CHANNELS` when the encoder's table grows it.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Iterator, Optional, Tuple

#: The five digits, thumb first, as v1 names them.
FINGERS: Tuple[str, ...] = ("thumb", "index", "middle", "ring", "pinky")


class ChannelKind(Enum):
    """What sort of quantity a channel is. The value is the declaration string."""

    #: One joint's flexion. Unipolar: 0 = extended, up = towards the palm.
    FLEX = "flex"
    #: A whole finger's curl. Unipolar, and NOT one joint.
    TOTAL = "total"
    #: Rotation of the thumb towards the palm. Its extended end is not 0.
    YAW = "yaw"
    #: Axial rotation. Unipolar.
    ROLL = "roll"
    #: Lateral spread. BIPOLAR: signed about a neutral hand, so its
    #: normalised neutral is 0.5 and not 0.0.
    SWING = "swing"

    @property
    def bipolar(self) -> bool:
        """Whether its neutral sits in the middle of its range."""
        return self is ChannelKind.SWING


@dataclass(frozen=True)
class ChannelSpec:
    """One anatomical channel: what it is, and where it can be published."""

    #: The channel's name, e.g. ``index_mp_pitch``.
    name: str
    #: The v1 finger it belongs to.
    finger: str
    #: What sort of quantity it is.
    kind: ChannelKind
    #: The v1 ``finger_angles_rad.<finger>.<field>`` slot, or ``None`` when v1
    #: has no field for this quantity at all.
    v1_field: Optional[str]
    #: The older twenty-four-name vocabulary's name for it, or ``None`` when
    #: that vocabulary has none.
    omakase_channel: Optional[str]
    #: What it measures.
    description: str
    #: Set when this name means something different on the consumer side than
    #: it does here. A shared name with an unshared meaning is worse than two
    #: names, because nothing in the type system objects to it.
    caveat: str = ""

    @property
    def v1_slot(self) -> Optional[Tuple[str, str]]:
        """``(finger, field)`` in a v1 pose frame, or ``None``."""
        return (self.finger, self.v1_field) if self.v1_field else None


#: Channels whose name is shared with the older vocabulary and whose MEANING
#: is not. Every one carries a :attr:`ChannelSpec.caveat`.
SAME_NAME_DIFFERENT_THING: Tuple[str, ...] = ("thumb_cm_roll",)

THUMB_CM_ROLL_CAVEAT = (
    "SAME NAME, DIFFERENT QUANTITY. Here it is an actual axial roll of the "
    "thumb's base joint. In the older twenty-four-name vocabulary the same name "
    "is a SLOT filled by the LitchiBot glove's thumb ABDUCTION, because that "
    "glove's packet has no axial-roll field and a hand map wanted a second "
    "lateral thumb channel to blend against the yaw ('a role match, not a name "
    "match'). The LitchiBot driver now declares that abduction as "
    "thumb_cm_swing (EXTENSION_CHANNELS), so under this vocabulary "
    "thumb_cm_roll is a roll from the glove that measures one, and nothing "
    "else. A DH116S thumb_swing_roll_blend tuned on one glove does not carry "
    "over to the other."
)


def _spec(name: str, finger: str, kind: ChannelKind, v1_field: Optional[str],
          omakase_channel: Optional[str], description: str,
          caveat: str = "") -> ChannelSpec:
    return ChannelSpec(name, finger, kind, v1_field, omakase_channel,
                       description, caveat)


def _finger(f: str) -> Tuple[ChannelSpec, ...]:
    return (
        _spec(f"{f}_mp_pitch", f, ChannelKind.FLEX, "mcp_flex", f"{f}_mp_pitch",
              "knuckle (metacarpophalangeal) flexion"),
        _spec(f"{f}_pip_pitch", f, ChannelKind.FLEX, "pip_flex", f"{f}_pip_pitch",
              "middle (proximal interphalangeal) joint flexion"),
        _spec(f"{f}_dip_pitch", f, ChannelKind.FLEX, "dip_flex", f"{f}_dip_pitch",
              "last (distal interphalangeal) joint flexion"),
        _spec(f"{f}_total_pitch", f, ChannelKind.TOTAL, "total_flex",
              f"{f}_total_pitch",
              "the whole finger's curl. Which of this and the three joint "
              "channels is the MEASUREMENT depends on the glove"),
        _spec(f"{f}_mp_swing", f, ChannelKind.SWING, "swing", f"{f}_mp_swing",
              "lateral spread of the knuckle. Bipolar: neutral is the middle "
              "of the range, not its bottom"),
    )


_THUMB: Tuple[ChannelSpec, ...] = (
    _spec("thumb_cm_pitch", "thumb", ChannelKind.FLEX, "mcp_flex",
          "thumb_cm_pitch",
          "carpometacarpal flexion: the thumb's base joint, at the wrist"),
    _spec("thumb_mp_pitch", "thumb", ChannelKind.FLEX, "pip_flex",
          "thumb_mp_pitch",
          "metacarpophalangeal flexion: the thumb's second joint"),
    # The one channel in this table with a v1 slot and no older name.
    _spec("thumb_ip_pitch", "thumb", ChannelKind.FLEX, "dip_flex", None,
          "interphalangeal flexion: the thumb's last joint"),
    _spec("thumb_cm_yaw", "thumb", ChannelKind.YAW, "thumb_yaw", "thumb_cm_yaw",
          "rotation of the thumb towards the palm. Its extended end is NOT "
          "zero on any glove this vocabulary has met"),
    _spec("thumb_cm_roll", "thumb", ChannelKind.ROLL, "swing", "thumb_cm_roll",
          "axial roll of the thumb's base joint, positive towards the palm",
          THUMB_CM_ROLL_CAVEAT),
    _spec("thumb_total_pitch", "thumb", ChannelKind.TOTAL, "total_flex", None,
          "the whole thumb's curl"),
)

_ROLLS: Tuple[ChannelSpec, ...] = (
    _spec("index_mp_roll", "index", ChannelKind.ROLL, None, None,
          "axial roll of the index knuckle. v1 has no field for it"),
    _spec("pinky_mp_roll", "pinky", ChannelKind.ROLL, None, None,
          "axial roll of the pinky knuckle. v1 has no field for it"),
)

#: Every channel of the shared vocabulary, in the Rust table's declaration
#: order (thumb, index, middle, ring, pinky, then the two finger rolls).
CHANNELS: Tuple[ChannelSpec, ...] = (
    _THUMB + _finger("index") + _finger("middle") + _finger("ring")
    + _finger("pinky") + _ROLLS
)

#: Channels a glove driver really declares that the Rust table does not carry
#: yet. See the module docstring.
EXTENSION_CHANNELS: Tuple[ChannelSpec, ...] = (
    _spec("thumb_cm_swing", "thumb", ChannelKind.SWING, "swing", None,
          "thumb abduction (lateral swing away from the palm plane), as the "
          "LitchiBot glove declares it. Shares v1's thumb.swing slot with "
          "thumb_cm_roll; a device declares one or the other, never both"),
)

#: Every name the decoder accepts.
ALL_CHANNELS: Tuple[ChannelSpec, ...] = CHANNELS + EXTENSION_CHANNELS

_BY_NAME = {spec.name: spec for spec in ALL_CHANNELS}
assert len(_BY_NAME) == len(ALL_CHANNELS), "a channel name is declared twice"

#: Every channel name, in :data:`ALL_CHANNELS` order.
CHANNEL_NAMES: Tuple[str, ...] = tuple(spec.name for spec in ALL_CHANNELS)


def spec_of(name: str) -> Optional[ChannelSpec]:
    """One channel's spec, or ``None`` for a name outside the vocabulary."""
    return _BY_NAME.get(name)


def channels_for_slot(finger: str, field: str) -> Iterator[ChannelSpec]:
    """Every channel that can fill one v1 slot.

    Usually one. ``thumb.swing`` has two (``thumb_cm_roll`` and the extension
    ``thumb_cm_swing``), which is exactly why a pose frame cannot be decoded
    without the device's declaration. ``thumb_yaw`` off the thumb has none:
    v1 defines that field's value itself (always 0.0)."""
    return (spec for spec in ALL_CHANNELS
            if spec.finger == finger and spec.v1_field == field)
