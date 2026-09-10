"""manipulation_kit.arms — cross-robot ARM-side kinematics, drivers and transports.

Sibling of :mod:`manipulation_kit.hands` in the same repo (dx-manipulator), same
layout: ``arms/<maker>/<model>/`` installing as
``manipulation_kit.arms.<maker>.<model>``. Vendor name first, so a second arm does not
have to be told apart by model alone.

Why arms live beside hands rather than in a robot repo
------------------------------------------------------
Two independent reasons, and they point the same way.

1. On D1 the hand is mounted on the arm flange and reachable ONLY through the
   arm's own CANFD end-module passthrough. The code that bridges them belongs
   to neither side alone, and duplicating it into each consumer would mean two
   copies of the code arbitrating one shared, process-exclusive SDK handle.

2. The kinematics that turn an end-effector goal into a safe joint vector are
   needed by EVERY consumer that moves the arm, not just by teleop: VR teleop
   (dx-vr-teleop), the runtime that starts teleop mid-conversation
   (omakase-core), and episode replay (d1-inference) all need the same IK, the
   same collision gate and the same clamps. Kept in one robot stack, the
   others had to copy it — and a second copy of a SAFETY limiter is strictly
   worse than no second copy, because the two drift and nobody can tell which
   one the robot is actually running.

Arm-agnostic seam
-----------------
Consumers should NOT import a specific model. They take a ``"<maker>/<model>"``
string and call :func:`get_arm_kinematics`, driving whatever it returns through
the :class:`~manipulation_kit.arms.kinematics.ArmKinematics` protocol — the same shape as
``manipulation_kit.hands``' :func:`~manipulation_kit.hands.get_hand` / ``get_retarget`` /
``get_tool_config`` resolvers. Adding a second arm = drop an
``arms/<maker>/<model>/`` package exposing ``build_kinematics(...)``; no
consumer change.

Where this package STOPS
------------------------
**The library ends where the wire begins.** Everything here is
transport-agnostic: it computes on a kinematic model and returns numbers. It
opens no connection, holds no robot lock, owns no socket and streams nothing.
The vendor SDK is never imported at module scope — where SDK-derived data is
genuinely needed (the collision guard, the URDF, the HOME pose) it is either
INJECTED by the caller or loaded lazily from ``$D1_SDK_DIR`` behind a
try/except, so a consumer that only wants the pose math never pays for it.

Consequently the following deliberately do NOT live here (yet):

- arm MODE sequencing — impedance/compliance entry and exit, gravity
  compensation, servo enable, tool registration on the controller. Those are
  sequences of vendor SDK calls against a live connection.
- the streaming path — ``set_joint_cmd_pose`` and the per-tick "warp clamp"
  guarding it, whose reference value is *what was last put on the wire*, i.e.
  transport state by definition.
- session semantics — who currently owns the arm, grip-button decoding,
  WebSocket/HUD plumbing, episode recording.

See ``arms/README.md`` for the migration status table.
"""

from __future__ import annotations

import importlib
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:  # annotation only — keep import-time surface minimal
    from .kinematics import ArmKinematics


def _split(model: str) -> tuple[str, str]:
    maker, sep, name = model.partition("/")
    if not sep or not maker or not name:
        raise ValueError(f"arm model must be '<maker>/<model>', got {model!r}")
    return maker, name


def get_arm_kinematics(model: str, **kwargs: Any) -> "ArmKinematics":
    """Resolve a ``"<maker>/<model>"`` id to that arm's kinematics.

    Dynamically imports ``manipulation_kit.arms.<maker>.<model>.kinematics`` and calls
    its ``build_kinematics(**kwargs)``, returning something satisfying the
    :class:`~manipulation_kit.arms.kinematics.ArmKinematics` protocol — forward
    kinematics, guarded IK, joint limits and the model's HOME pose.

    Keyword arguments are passed through to the model's factory; every model
    accepts ``guard=`` (an injected collision guard, see
    :mod:`manipulation_kit.arms.guard`) and most accept ``urdf=`` to override the
    description path. ``execute`` does not appear anywhere in this seam: these
    objects command no hardware, they only compute.

    Raises ``ValueError`` on a malformed / unknown id and
    ``NotImplementedError`` if the model package lacks the factory.
    """
    maker, name = _split(model)
    try:
        mod = importlib.import_module(f"manipulation_kit.arms.{maker}.{name}.kinematics")
    except ImportError as exc:
        raise ValueError(f"no kinematics for arm model {model!r}: {exc}") from exc
    build = getattr(mod, "build_kinematics", None)
    if build is None:
        raise NotImplementedError(
            f"manipulation_kit.arms.{maker}.{name}.kinematics has no build_kinematics() "
            f"factory (add build_kinematics(**kwargs) -> ArmKinematics)"
        )
    return build(**kwargs)


def get_clutch_tuning(model: str) -> Any:
    """Resolve a ``"<maker>/<model>"`` id to that arm's clutch/clamp tuning.

    Dynamically imports ``manipulation_kit.arms.<maker>.<model>.kinematics`` and calls
    its ``clutch_tuning()``, returning a
    :class:`~manipulation_kit.arms.targeting.ClutchTuning` — the reachable workspace box
    plus the per-tick step caps and parking deadband that
    :class:`~manipulation_kit.arms.targeting.ArmClutch` enforces.

    This is a separate resolver rather than a method on the kinematics object
    on purpose: the numbers are needed to CONFIGURE target shaping before any
    model is loaded (and by consumers that shape targets for an arm they are
    not simulating), and building a MuJoCo model just to read a box would be
    absurd.
    """
    maker, name = _split(model)
    try:
        mod = importlib.import_module(f"manipulation_kit.arms.{maker}.{name}.kinematics")
    except ImportError as exc:
        raise ValueError(f"unknown arm model {model!r}: {exc}") from exc
    fn = getattr(mod, "clutch_tuning", None)
    if fn is None:
        raise NotImplementedError(
            f"manipulation_kit.arms.{maker}.{name}.kinematics has no clutch_tuning() "
            f"factory (add clutch_tuning() -> ClutchTuning)"
        )
    return fn()
