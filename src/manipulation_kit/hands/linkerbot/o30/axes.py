"""LinkerHand O30 axis vocabulary — lifted out of the retired CANFD driver.

``driver.py`` did not come to manipulation-kit (the wire is ``d1-firmwared``'s
job now), but the joint table is what the hand IS, and :mod:`.retarget` — the
stub that will one day carry the glove→20-joint map — is written against these
names. Values copied verbatim from dx-manipulator
``hands/linkerbot/o30/driver.py`` @ 2ce2c64.

Sub-indices are kept even though nothing here addresses a wire: they are how the
daemon and the vendor spec name the same joints, and dropping them would make
the two vocabularies impossible to line up.
"""
from __future__ import annotations

from typing import Dict, Tuple

#: Joint type groups: name -> (first sub-index, slot count). Five slots each,
#: finger order thumb -> index -> middle -> ring -> little.
JOINT_TYPE_GROUPS: Dict[str, Tuple[int, int]] = {
    "roll": (0x00, 5),
    "yaw": (0x05, 5),
    "root1": (0x0A, 5),
    "root2": (0x0F, 5),
    "tip": (0x14, 5),
}
JOINT_TYPE_ORDER = ("roll", "yaw", "root1", "root2", "tip")
FINGER_ORDER = ("thumb", "index", "middle", "ring", "little")

#: The 20 motors the O30 actually has, as ``(name, sub-index)`` in wire order
#: (type-major, finger-minor). Holes: roll exists on the thumb only; root2 has
#: no thumb.
ACTIVE_JOINTS: Tuple[Tuple[str, int], ...] = (
    ("thumb_roll", 0x00),
    ("thumb_yaw", 0x05), ("index_yaw", 0x06), ("middle_yaw", 0x07),
    ("ring_yaw", 0x08), ("little_yaw", 0x09),
    ("thumb_root1", 0x0A), ("index_root1", 0x0B), ("middle_root1", 0x0C),
    ("ring_root1", 0x0D), ("little_root1", 0x0E),
    ("index_root2", 0x10), ("middle_root2", 0x11), ("ring_root2", 0x12),
    ("little_root2", 0x13),
    ("thumb_tip", 0x14), ("index_tip", 0x15), ("middle_tip", 0x16),
    ("ring_tip", 0x17), ("little_tip", 0x18),
)

#: Axis names in wire order — the cross-hand equivalent of the DH116S's.
AXIS_NAMES: Tuple[str, ...] = tuple(name for name, _ in ACTIVE_JOINTS)
JOINT_NAMES = AXIS_NAMES
JOINT_SI: Dict[str, int] = {name: si for name, si in ACTIVE_JOINTS}

NUM_AXES = len(ACTIVE_JOINTS)   # 20

#: Single-byte position channel (MI 0x01), 0-255. NOT the DH116S's 0-10000.
POS_MAX = 255
