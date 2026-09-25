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


class TrajectoryInvalid(FirmwareUnavailable):
    """A trajectory the kit built breaks the document's own ``Waypoint``
    contract ("seconds, starting at zero, strictly increasing"; finite
    joints) and was NOT uploaded.

    Checked before the upload, so the refusal names the offending knots
    instead of the daemon's one-line HTTP 400 (d1-2 2026-09-23: "trajectory
    values must be finite with increasing times <=120 seconds", which says
    neither which knot nor which rule). It is a bug in whatever produced the
    points, never a reason to retry.
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


class OperationUnavailable(FirmwareUnavailable):
    """The daemon's OpenAPI document (as this client has it) lacks an operation.

    Raised instead of reaching around the generated client with a hand-built
    request: a route the document does not publish is a route the kit does not
    call. The message names the route and how to get a client that has it.
    """


class ModeUnconfirmed(FirmwareUnavailable):
    """A mode the kit asked for was not what the arm REPORTED in time.

    ``POST /v1/arm/{side}/mode`` answers as soon as the daemon has accepted
    the request, not when the controller has made the transition (d1-2
    2026-09-23: the answer came back in 0 ms and the controller reported
    ``idle`` 11 ms later). Anything gated on the transition — the brake
    release accepts only an arm whose LIVE mode is ``idle`` or ``error`` —
    has to wait for the report; this is what that wait raises when the
    report never comes, or when the arm reports a fault instead. The
    message names the last mode observed.
    """


class ClientTimeout(FirmwareUnavailable):
    """The KIT stopped waiting for an answer — not a refusal by the daemon.

    Closing the connection is what the daemon sees, and it cancels the
    handler: on d1-2 (2026-09-23 20:46Z) a recover was cancelled at exactly
    the client's old blanket 2 s while the daemon's confirm loop (up to
    8 x 500 ms) was still running, although the document gives that route
    ``x-timeout-seconds: 65``. Every request now waits at least the
    document's bound for its route; this is raised when even that ran out.
    Whether the operation took effect is unknown: re-read the state rather
    than re-issue it.
    """

    def __init__(self, method: str, path: str, timeout_s: float):
        self.method, self.path, self.timeout_s = method, path, float(timeout_s)
        super().__init__(
            f"{method} {path}: the kit gave up waiting for the daemon's answer "
            f"after {self.timeout_s:g} s (client timeout, not a refusal); the "
            f"daemon cancels a request whose connection closes, so whether it "
            f"took effect is unknown")


class RecoverFailed(FirmwareUnavailable):
    """``FirmwareExecutor.recover_arm`` did not get the arm into a confirmed
    position hold. ``failures`` is one ``(reason, message)`` per attempt,
    ``reason`` one of :data:`RECOVER_REASONS`; :attr:`reason` is the last."""

    def __init__(self, wire: str, failures):
        self.wire = wire
        self.failures = list(failures)
        super().__init__(f"arm {wire}: recover failed after "
                         f"{len(self.failures)} attempt(s): "
                         + "; ".join(f"[{r}] {m}" for r, m in self.failures))

    @property
    def reason(self) -> str:
        return self.failures[-1][0] if self.failures else "error"


#: Why one recover attempt failed: the daemon answered with a refusal; the kit
#: gave up waiting for its answer; it answered but the arm never reported
#: ``position``; the arm never came to rest before the recover could be sent;
#: anything else (a malformed answer).
RECOVER_REASONS = ("refused", "client_timeout", "unconfirmed", "not_steady", "error")
