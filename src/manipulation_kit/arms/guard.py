"""The collision-guard seam — injected, never imported at module scope.

Migrated from dx-vr-teleop ``server/backends.py`` (``_load_guard``,
``_guard_ok``) at post-#41 ``master``; omakase-core #94 restates the same
degrees conversion and fail-closed rule in
``robot_stack/teleop/vr_arm/guard.py``.

The guard itself is :class:`manipulation_kit.guard.MotionGuard`, now a sibling
subpackage rather than a d1-sdk checkout addressed through ``$D1_SDK_DIR``.
The seam is kept anyway, and deliberately:

- the guard is a PROTOCOL here, and :class:`GuardGate` accepts any object that
  satisfies it — a consumer that already built one (omakase-core builds it via
  its ``_sdk_facts`` module, and needs the same instance for other purposes)
  INJECTS it rather than having a second one made behind its back. That is also
  what lets a caller substitute the daemon's own guard report, or a stub, with
  no change here;
- :func:`load_motion_guard` is offered as a convenience for consumers that
  don't already have one. It imports the guard LAZILY, so this module stays
  importable for the pose math alone (no URDF parsed, no file touched).

FAIL-CLOSED is the one non-negotiable: any exception out of the guard blocks the
motion. A guard whose API drifted must never read as "no collision".
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Optional, Protocol, Tuple, runtime_checkable

import numpy as np


@runtime_checkable
class CollisionGuard(Protocol):
    """What :class:`GuardGate` needs of a collision guard.

    ``check`` takes the two arms' joint vectors in **DEGREES** — that is the
    :class:`manipulation_kit.guard.MotionGuard` convention and the source of a real P1 bug: passing
    radians silently evaluated a near-zero-degree posture and made the guard
    useless (elbow-through-torso passed). The radians/degrees seam is therefore
    in exactly ONE place, :meth:`GuardGate.report`.
    """

    def check(self, joints_a_deg, joints_b_deg) -> Any:
        """Return a report with ``.ok`` and (optionally) ``.violations`` and
        ``.min_body_clearance``."""
        ...


class GuardGate:
    """Radians-in, fail-closed wrapper over a :class:`CollisionGuard`.

    ``guard=None`` means NO guard is installed. That is permitted (a pure
    kinematics consumer, a unit test) and then :meth:`ok` returns True — but the
    caller is expected to have decided that deliberately, which is why it is an
    explicit ``None`` rather than a silent default.
    """

    def __init__(self, guard: Optional[CollisionGuard] = None):
        self.guard = guard
        #: ``(reason, monotonic_timestamp)`` of the most recent rejection, for
        #: surfacing on an operator HUD so a frozen arm is EXPLAINED (e.g.
        #: "gripper finger inside the chassis keep-out").
        self.last_reject: Optional[Tuple[str, float]] = None

    @property
    def installed(self) -> bool:
        return self.guard is not None

    def report(self, qa_rad, qb_rad) -> Any:
        """The raw guard report for a full commanded posture (both arms).

        The guard checks BOTH arms because arm-arm collision is one of the
        things it is for; a per-arm check would be meaningless.
        """
        if self.guard is None:
            return None
        return self.guard.check(np.degrees(np.asarray(qa_rad, dtype=float)),
                                np.degrees(np.asarray(qb_rad, dtype=float)))

    def ok(self, qa_rad, qb_rad) -> bool:
        """True iff the posture is allowed. Any exception blocks (fail SAFE)."""
        if self.guard is None:
            return True
        try:
            res = self.report(qa_rad, qb_rad)
            if not res.ok:
                import time as _t
                v = getattr(res, "violations", None) or []
                self.last_reject = (str(v[0]) if v else "guard reject",
                                    _t.monotonic())
            return bool(res.ok)
        except Exception:  # noqa: BLE001 — guard API drift -> fail SAFE (block)
            return False

    def clearance(self, qa_rad, qb_rad) -> Optional[float]:
        """Minimum body clearance [m], or ``None`` if unavailable/rejected."""
        try:
            res = self.report(qa_rad, qb_rad)
        except Exception:  # noqa: BLE001
            return None
        if res is None or not getattr(res, "ok", False):
            return None
        return getattr(res, "min_body_clearance", None)


#: MotionGuard constructor parameters this loader will forward from a config
#: file, and how to coerce them. Keys ABSENT from the file fall through to
#: ``MotionGuard``'s own default rather than to a copy of it kept here —
#: :mod:`manipulation_kit.guard` stays the source of truth for the collision
#: model and the config file records only the deliberate site overrides.
_COERCE = {
    "body_margin_m": float,
    "arm_arm_margin_m": float,
    "self_margin_m": float,
    "disabled_body_boxes": tuple,
    "box_pad_m": dict,
    "chest_keepout": lambda v: v,   # dict of per-side AABBs; None disables
}


def load_motion_guard(config: Optional[Path] = None, *,
                      quiet: bool = False) -> Optional[CollisionGuard]:
    """Build :class:`manipulation_kit.guard.MotionGuard`, or ``None``.

    ``config`` is an optional JSON file of overrides (keys as in ``_COERCE``;
    ``_``-prefixed keys are treated as comments). With no file the guard is
    exactly the default guard.

    A ``ValueError`` — an unknown box name in ``box_pad_m``, a non-numeric
    margin, malformed JSON — is a CONFIG ERROR and propagates. Running on with
    no collision guard because a key was mistyped is the one outcome that must
    never happen silently. Anything else (a build without the description
    assets, an import failure) is an unavailable guard: warn and return
    ``None``, letting the caller decide.

    The old ``sdk_dir=`` parameter is gone: the guard and its URDF now ship in
    this package, so there is no checkout to point at.
    """
    try:
        import json  # noqa: PLC0415
        from ..guard.guard import MotionGuard  # noqa: PLC0415

        cfg = {}
        if config is not None and Path(config).exists():
            cfg = {k: v for k, v in json.loads(Path(config).read_text()).items()
                   if not k.startswith("_")}
        kw = {k: fn(cfg[k]) for k, fn in _COERCE.items() if k in cfg}
        guard = MotionGuard(**kw)
        if not quiet:
            src = Path(config).name if cfg else "package defaults"
            print(f"[manipulation_kit.arms] guard ({src}): body_margin "
                  f"{guard.body_margin_m * 1000:.0f}mm, arm-arm "
                  f"{guard.arm_arm_margin_m * 1000:.0f}mm, self "
                  f"{guard.self_margin_m * 1000:.0f}mm, overridden "
                  f"{sorted(kw) or 'nothing'}")
        return guard
    except ValueError:
        raise
    except Exception as exc:  # noqa: BLE001
        if not quiet:
            print(f"[manipulation_kit.arms] WARNING: motion guard unavailable "
                  f"({exc}); running WITHOUT the collision guard")
        return None
