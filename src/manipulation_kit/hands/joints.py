"""Retarget contract version 2: canonical hand pose -> descriptor-order joints.

Version 1 of the retarget seam (``mapper(flex)``, still served by
:func:`manipulation_kit.hands.get_retarget`) takes a ``flex(channel) -> float``
callable and returns the hand's WIRE units (the DH116S's 0..10000). Two things
about it do not survive more than one glove and more than one hand:

* it has no way to say "no reading". ``flex`` must answer a float for every
  channel, so a glove without a sensor answers 0.0 — a fully extended finger —
  and the robot hand opens;
* wire units are the hand driver's business, and that moved into
  ``d1-firmwared``. Its ``POST /v1/end_effectors/{side}/joints`` takes one
  target per DECLARED joint, in the order and unit of the descriptor that
  ``GET /v1/end_effectors/{side}`` publishes (``joints[]``: ``name``,
  ``unit`` ``rad`` or ``fraction``, ``min``, ``max``), and refuses a count
  mismatch, a non-finite target or one outside ``min..max`` with 422.

Version 2 is what every hand's retarget object exposes beside version 1:

``JOINTS``                    a tuple of :class:`JointSpec`, the hand's joints
                              exactly as the daemon's descriptor lists them.
``required_channels()``       the anatomical channels this map reads WITH ITS
                              CURRENT CONFIG (a channel weighted 0 is not
                              required). A consumer checks them once, at bind
                              time, against the glove's declaration
                              (:meth:`manipulation_kit.gloves.v1.Declaration.
                              check_drive_sources`).
``joint_targets(pose)``       a :class:`~manipulation_kit.gloves.HandPose` in,
                              one ``float | None`` per joint out, in
                              :data:`JOINTS` order and units. ``None`` = an
                              input of that joint has no reading: the consumer
                              HOLDS that joint (re-sends its last commanded
                              target) rather than moving it.

The daemon's ``joints`` request has no per-joint "hold", so filling a ``None``
is the consumer's job (:func:`fill_held`), and so is refusing to engage
before every joint has had a value once.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import (Iterable, List, Mapping, Optional, Sequence, Tuple)

try:  # 3.8+: typing.Protocol; kept import-light like the rest of hands/
    from typing import Protocol
except ImportError:  # pragma: no cover
    Protocol = object  # type: ignore[assignment,misc]

#: The descriptor's two units.
UNIT_RAD = "rad"
UNIT_FRACTION = "fraction"


@dataclass(frozen=True)
class JointSpec:
    """One joint as ``d1-firmwared``'s end-effector descriptor publishes it."""

    name: str
    #: ``"rad"`` or ``"fraction"`` (0..1 of the axis's wire range).
    unit: str
    #: The smallest target the daemon accepts, in ``unit``.
    min: float
    #: The largest target the daemon accepts, in ``unit``.
    max: float

    def clamp(self, value: float) -> float:
        return min(self.max, max(self.min, value))


class HandRetarget(Protocol):
    """What a version-2 hand retarget object provides (see module docstring)."""

    JOINTS: Tuple[JointSpec, ...]

    def required_channels(self) -> Tuple[str, ...]: ...

    def joint_targets(self, pose) -> List[Optional[float]]: ...

    def reset(self) -> None: ...


def check_descriptor(joints: Sequence[JointSpec],
                     descriptor_joints: Iterable[Mapping[str, object]]) -> None:
    """Refuse a live descriptor that does not match the kit's joint table.

    ``descriptor_joints`` is the ``descriptor.joints`` list from
    ``GET /v1/end_effectors/{side}``. Names, order and unit must match
    exactly, and so must ``min``/``max``: for a ``fraction`` joint the range
    decides which physical angle a target means, so a daemon whose range moved
    would put the same number somewhere else, and for a ``rad`` joint a kit
    target outside the daemon's range is a 422. Raises ``ValueError``
    naming every difference. Call it once, at bind time."""
    live = list(descriptor_joints)
    problems = []
    if len(live) != len(joints):
        problems.append(f"the daemon declares {len(live)} joints, the kit "
                        f"{len(joints)}")
    for i, (kit, got) in enumerate(zip(joints, live)):
        name, unit = got.get("name"), got.get("unit")
        if name != kit.name:
            problems.append(f"joint {i}: daemon {name!r}, kit {kit.name!r}")
        if unit != kit.unit:
            problems.append(f"joint {i} {kit.name}: daemon unit {unit!r}, kit {kit.unit!r}")
        lo, hi = got.get("min"), got.get("max")
        if not (isinstance(lo, (int, float)) and isinstance(hi, (int, float))):
            problems.append(f"joint {i} {kit.name}: no numeric min/max")
            continue
        # 1e-6 rad: the driver computes its limits in float32 (90 deg is
        # 1.5707964 there, 1.5707963267948966 here); anything larger is a
        # different range, not rounding.
        if not (math.isclose(lo, kit.min, abs_tol=1e-6)
                and math.isclose(hi, kit.max, abs_tol=1e-6)):
            problems.append(f"joint {i} {kit.name}: daemon range {lo}..{hi}, kit "
                            f"{kit.min}..{kit.max}")
    if problems:
        raise ValueError("end-effector descriptor does not match the kit: "
                         + "; ".join(problems))


def fill_held(targets: Sequence[Optional[float]],
              held: Sequence[Optional[float]]) -> Optional[List[float]]:
    """Replace each ``None`` target by the held (last commanded) value.

    Returns ``None`` if a joint has neither — nothing has ever been commanded
    on it, so there is no pose to hold and the request must not be sent."""
    out = []
    for target, last in zip(targets, held):
        value = target if target is not None else last
        if value is None:
            return None
        out.append(float(value))
    return out
