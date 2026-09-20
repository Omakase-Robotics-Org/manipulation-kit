"""Run a planned primitive on a real D1, through the d1-firmwared daemon.

The one place in this package with a wire, behind the ``[firmware]`` extra.
Importing this module opens no socket and needs no daemon: the client is built
when an executor is constructed, not here.

Four modules, and the split is the point:

:mod:`~manipulation_kit.executors.firmware.executor`
    :class:`FirmwareExecutor` — the lease, position mode, the rate clamp, the
    trajectory upload and the barriers. All the judgement.
:mod:`~manipulation_kit.executors.firmware.ensure`
    Which generated client matches the daemon in front of us, and regenerating
    one on the spot when the bundled snapshot is older than the firmware.
:mod:`~manipulation_kit.executors.firmware.client`
    The four verbs the executor drives, over that generated client.
:mod:`~manipulation_kit.executors.firmware.errors`
    What can go wrong, all of it under :class:`FirmwareUnavailable`.

The usual entry is still one name::

    from manipulation_kit.executors.firmware import FirmwareExecutor

    with FirmwareExecutor(base_url="http://d1-2:4750") as robot:
        run(plan, robot)
"""
from __future__ import annotations

from .client import ArmState, FirmwareClient, GripperState
from .ensure import (ClientTree, Resolution, bundled_spec_sha256, ensure_client,
                     resolve_client, snapshot)
from .errors import (ClientUnavailable, DeviceUnavailable, FirmwareError,
                     FirmwareUnavailable, LeasePreempted, ProtocolError,
                     RateRefused)
from .executor import (ANCHOR_GAP_DEG, DEFAULT_ACC_RATIO, DEFAULT_TTL_S,
                       DEFAULT_VEL_RATIO, INTERPOLATION_S, LEASE_CLASS,
                       MAX_COMMAND_STEP_DEG, MAX_JOINT_RATE_DEG_S, STREAM_HZ,
                       FirmwareExecutor, Lease, default_holder)

__all__ = [
    "ANCHOR_GAP_DEG", "ArmState", "ClientTree", "ClientUnavailable",
    "DEFAULT_ACC_RATIO", "DEFAULT_TTL_S", "DEFAULT_VEL_RATIO",
    "DeviceUnavailable", "FirmwareClient", "FirmwareError", "FirmwareExecutor",
    "FirmwareUnavailable", "GripperState", "INTERPOLATION_S", "LEASE_CLASS",
    "Lease", "LeasePreempted", "MAX_COMMAND_STEP_DEG", "MAX_JOINT_RATE_DEG_S",
    "ProtocolError", "RateRefused", "Resolution", "STREAM_HZ",
    "bundled_spec_sha256", "default_holder", "ensure_client", "resolve_client",
    "snapshot",
]
