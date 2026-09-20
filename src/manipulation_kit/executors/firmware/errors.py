"""The failures this executor can hand back, in one place.

They live here rather than in :mod:`~manipulation_kit.executors.firmware.executor`
because :mod:`~manipulation_kit.executors.firmware.ensure` raises one of them
before an executor exists at all, and a module that only defines exceptions can
be imported from both without a cycle.

Everything is a :class:`FirmwareUnavailable`, so a consumer that catches the one
name still catches the lot; the subclasses exist for the cases where carrying on
would be actively wrong rather than merely impossible.
"""
from __future__ import annotations


class FirmwareUnavailable(RuntimeError):
    """The firmware client could not be produced, or the daemon refused a step."""


class ClientUnavailable(FirmwareUnavailable):
    """No usable d1-firmwared client for this machine and this daemon.

    Raised by :func:`~manipulation_kit.executors.firmware.ensure.ensure_client`
    when the daemon serves an OpenAPI document the bundled snapshot was not
    generated from AND the client cannot be regenerated here — with
    ``policy="strict"``, which is the setting that says "a client I cannot
    prove matches this daemon is not a client I will drive arms with". The
    default policy warns and uses the bundled tree instead.

    Also raised on an interpreter older than 3.10, which the generated tree
    does not import on.
    """


class LeasePreempted(FirmwareUnavailable):
    """Somebody senior took the arms while we were using them.

    A new lease EPOCH means we lost possession and got it back. Everything we
    believed in between — the mode, the ratios, the last commanded pose, the
    trajectory that was playing — is stale, and continuing is continuing from
    a posture nobody checked. So it raises, and the caller re-observes and
    replans. The old code cleared a cache and carried on.
    """


class RateRefused(FirmwareUnavailable):
    """A knot is too large for one command, and clipping it is not the fix.

    The stream used to clip each component against the last transmitted
    vector, which does three separate wrong things: it changes the checked
    geometry, it can change the PATH (componentwise clipping is not a
    rescaling), and it silently stops short of the endpoint — a real
    ``Grasp(red_block, left)`` ended 6.5283 degrees from its final target
    while ``RunReport.completed`` was ``True``, and the close stroke followed
    the truncated travel (R4).
    """


class ProtocolError(FirmwareUnavailable):
    """The daemon answered, but not in the shape its own spec advertises."""


class FirmwareError(FirmwareUnavailable):
    """The daemon answered with a refusal, carrying its own explanation."""

    def __init__(self, method: str, path: str, status: int, message: str):
        self.method, self.path, self.status = method, path, status
        super().__init__(f"{method} {path} (HTTP {status}): {message}")


class DeviceUnavailable(FirmwareError):
    """The daemon answered, but could not read that device.

    A state slot never fails the whole response: a device the daemon could not
    reach degrades to ``{"error": "..."}`` in its own slot while the request
    still succeeds. That is a device failure reported correctly, not a broken
    contract, so it arrives as a :class:`FirmwareError` carrying the daemon's
    own words rather than as a :class:`ProtocolError` about a missing field.
    """
