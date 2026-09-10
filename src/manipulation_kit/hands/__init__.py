"""manipulation_kit.hands — cross-robot end-effector geometry, tool data and retargeting.

Layout is ``hands/<maker>/<model>/``, importable as
``manipulation_kit.hands.<maker>.<model>``. Compositions with specific robots
(adapter plates, wrist transforms, composed sim models) live in
:mod:`manipulation_kit.description`, not here.

NO DRIVERS LIVE HERE — and that is the point of this repo
---------------------------------------------------------
dx-manipulator's ``hands/`` carried the CANFD drivers, transports and the D1
arm passthrough alongside this data. All of that moved to the Rust daemon
``exp--d1-firmware`` (``d1-firmwared``, REST :4750), which is now the ONLY thing
that touches a wire. What stayed is everything a planner, an IK loop, a
simulator or a data pipeline needs WITHOUT a robot: the hand's shape, its
physical registration data, and the glove→hand retarget maps.

So the old ``get_hand(...)`` resolver is deliberately gone. Ask the daemon for a
hand; ask this package what the hand IS. Removed with it: ``canbus``,
``<model>/driver.py``, ``<model>/transport.py``, ``d1/parallel_gripper/
arm_passthrough.py`` and the smoke/wiggle scripts.

The ``"<maker>/<model>"`` ID CONVENTION IS UNCHANGED and load-bearing: it is
stored as a string by data-infra and dx-data-collection, and read back by
d1-firmwared. Adding a hand is still "drop a ``hands/<maker>/<model>/`` package";
it just exposes ``toolconfig`` / ``retarget`` / ``description`` instead of
``build``.

The seam covers retargeting: :func:`get_retarget` resolves the model's
``retarget`` submodule and returns its mapper (anatomical flexion channels →
that hand's wire-unit axis commands). Teleop stacks own the GLOVE side
(transport, per-user calibration) and hand it a ``flex(channel) -> [0, 1]``
callable; which channels drive which axis is hand knowledge and lives here.
Likewise a model may ship a ``description`` submodule for its geometry —
``load_mjspec`` (MJCF, ``leadshine/dh116s``) or ``load_urdf`` (URDF,
``d1/parallel_gripper``), both resolved off the model's
``description_path()``. That is the single source of truth for the hand's
SHAPE; robot-specific composition (wrist mounts, adapter plates, which links
the sim articulates) stays in the robot repos.

And the seam covers the tool's PHYSICAL registration data:
:func:`get_tool_config` resolves the model's ``toolconfig`` submodule and
returns its :class:`~manipulation_kit.hands.toolconfig.ToolConfig` (TCP offset, mass,
COM, inertia — what an arm controller's set-tool/gravity-compensation call
needs). Which numbers describe which hand is hand knowledge and lives here;
consumers load the exported JSON (``mkit-toolconfig export <maker>/<model>
out.json``) instead of hardcoding one gripper's values. The JSON files under
``manipulation_kit/config/tool_configs/`` are exports of exactly this, and the
exporter — not the JSON — is the source of truth; regenerate, never hand-edit.
"""

from __future__ import annotations

import importlib
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:  # annotation only — keep import-time surface minimal
    from .toolconfig import ToolConfig


def _installed_version() -> str:
    try:
        from importlib.metadata import version
        return version("manipulation-kit")
    except Exception:  # noqa: BLE001 — a bare checkout is not an error
        return "0+unknown"


#: Distribution version, for logs and bug reports. It is NOT the thing to gate
#: a feature on: this package is often installed EDITABLE, so the code that runs
#: is whatever branch the checkout is on while the version string stays put.
__version__ = _installed_version()


def get_retarget(model: str, config: Any = None) -> Any:
    """Resolve a ``"<maker>/<model>"`` id to that hand's retarget mapper.

    Dynamically imports ``manipulation_kit.hands.<maker>.<model>.retarget`` and calls
    its ``build_retarget(config)``. The returned mapper is callable per
    sample with a ``flex(channel_name) -> float`` callable (flexion in
    [0, 1], 0 = open) and yields the hand's wire-unit axis command — see the
    model's ``retarget`` module for its channel names and axis order.
    ``config=None`` = the model's defaults. Raises ``ValueError`` on a
    malformed / unknown id or a model without a retarget map, and
    ``NotImplementedError`` if the module lacks a ``build_retarget`` factory."""
    maker, sep, name = model.partition("/")
    if not sep or not maker or not name:
        raise ValueError(f"hand model must be '<maker>/<model>', got {model!r}")
    try:
        mod = importlib.import_module(f"manipulation_kit.hands.{maker}.{name}.retarget")
    except ImportError as exc:
        raise ValueError(
            f"no retarget map for hand model {model!r}: {exc}") from exc
    build = getattr(mod, "build_retarget", None)
    if build is None:
        raise NotImplementedError(
            f"manipulation_kit.hands.{maker}.{name}.retarget has no build_retarget() "
            f"factory (add build_retarget(config=None))"
        )
    return build(config=config)


def get_tool_config(model: str) -> "ToolConfig":
    """Resolve a ``"<maker>/<model>"`` id to that hand's physical tool config.

    Dynamically imports ``manipulation_kit.hands.<maker>.<model>.toolconfig`` and
    calls its ``tool_config()``, returning a
    :class:`~manipulation_kit.hands.toolconfig.ToolConfig` (TCP offset, mass, COM,
    inertia in the arm controller's set-tool units — mm/deg/kg/kg*m^2).
    Raises ``ValueError`` on a malformed / unknown id or a model without
    tool data, and ``NotImplementedError`` if the module lacks a
    ``tool_config()`` factory."""
    maker, sep, name = model.partition("/")
    if not sep or not maker or not name:
        raise ValueError(f"hand model must be '<maker>/<model>', got {model!r}")
    try:
        mod = importlib.import_module(f"manipulation_kit.hands.{maker}.{name}.toolconfig")
    except ImportError as exc:
        raise ValueError(
            f"no tool config for hand model {model!r}: {exc}") from exc
    build = getattr(mod, "tool_config", None)
    if build is None:
        raise NotImplementedError(
            f"manipulation_kit.hands.{maker}.{name}.toolconfig has no tool_config() "
            f"factory (add tool_config() -> ToolConfig)"
        )
    return build()
