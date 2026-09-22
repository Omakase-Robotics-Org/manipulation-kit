"""The verb surface :class:`FirmwareExecutor` drives, over the generated client.

:mod:`~manipulation_kit.executors.firmware.ensure` produces a *generated* client
— the whole daemon surface, one typed operation per route, models for every
schema. That is the right artifact to regenerate from a spec, and the wrong
shape to write a lease loop against: the executor needs four things
(``request``, ``arm_state``, ``gripper_state``, ``gripper_set``) and needs them
in terms of plain numbers, not attrs models whose ``mode`` is a union of six
generated enums.

So :class:`FirmwareClient` is a thin adapter, not a second client:

* **transport is the generated ``Client``** — its ``httpx`` session, its
  ``base_url``, its timeouts. Nothing here opens its own socket;
* **the envelope is checked once**, here, because every ``/v1`` response is
  ``{"status": …, "data": …, "message": …}`` and a caller that forgets is a
  caller that treats an error body as a reading;
* **states are parsed into frozen dataclasses** with validated fields, so a
  degraded slot or a missing key fails at the boundary rather than as a NaN
  three layers into the IK.

Everything the adapter does *not* cover is still right there — every operation
the daemon publishes, typed. Reach it through
:meth:`FirmwareClient.api_module`, never through a literal ``import d1fw_api``:
the tree in use may be the bundled one or a regenerated one under
``~/.cache``, and only the client knows which::

    neck = robot.client.api_module("neck.neck_state")
    neck.sync(client=robot.client.api_client)
"""
from __future__ import annotations

import importlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Tuple
from urllib.parse import urlsplit

from .ensure import ClientTree, ensure_client
from .errors import DeviceUnavailable, FirmwareError, ProtocolError

#: The wire spelling of a side, as the daemon names them.
SIDES: Tuple[str, str] = ("a", "b")
GRIPS: Tuple[str, str, str] = ("soft", "firm", "strong")

#: The daemon's ``StrokeKind`` — the OUTCOME of the last gripper stroke, and
#: the only thing that distinguishes "the jaws stopped because they are
#: holding something" from "the jaws stopped because the motor is disabled".
#: Taken from the generated model (``models/stroke_kind.py``) plus the two
#: d1-firmware PR #91 adds; an unknown value is carried through verbatim
#: rather than rejected, because a newer daemon inventing an outcome must not
#: break a client that would otherwise read the state fine.
STROKE_KINDS: Tuple[str, ...] = ("blind", "contact", "empty", "fault", "grasp",
                                 "lost", "open", "overload", "timeout")

#: Outcomes that mean the GRIPPER IS FAULTED: the motor has latched an error
#: (Damiao ``is_fault``) or refused the stroke, and every stroke after it ends
#: the same way until something clears it. d1-2, 2026-09-22: a firm hold on a
#: rigid charger wound its own torque to -4.17 Nm with nothing commanded, the
#: motor raised its fault flag, and from then on the jaws sat perfectly
#: stationary — which the settle barrier read as "terminal state reached" and
#: passed. A disabled gripper is not a settled one (Astra review, finding 13).
FAULT_KINDS: Tuple[str, ...] = ("fault", "overload")

#: Outcomes that are terminal and SUCCESSFUL — the stroke finished and the
#: report describes what it found. ``empty`` is in here: closing on nothing is
#: a true answer about the world, and it is the verifier's job, not the
#: barrier's, to decide the task failed.
SETTLED_KINDS: Tuple[str, ...] = ("grasp", "open", "empty", "contact", "lost")

#: Outcomes that are neither: the stroke ran out of time. Terminal on the
#: wire, but it establishes nothing about where the jaws are, so it is a
#: failed barrier rather than a fault.
UNFINISHED_KINDS: Tuple[str, ...] = ("timeout",)
#: Routes this adapter spells out by hand. A test asserts every one of them is
#: in the bundled OpenAPI document, so a spec that moves a route breaks the
#: build rather than the robot.
ROUTES = ("/v1/arm/{side}/state", "/v1/gripper/{side}/state",
          "/v1/gripper/{side}/set")


def side_name(side: str) -> str:
    """``a``/``b`` — the daemon's own spelling, validated."""
    value = str(side).strip().lower()
    if value not in SIDES:
        raise ValueError(f"side must be 'a' or 'b', got {side!r}")
    return value


def joints7(values: Sequence[float]) -> Tuple[float, ...]:
    result = tuple(float(v) for v in values)
    if len(result) != 7 or not all(math.isfinite(v) for v in result):
        raise ValueError("values must contain exactly seven finite numbers")
    return result


def _reject_degraded_slot(value: Any, endpoint: str) -> None:
    """A slot the daemon could not read degrades to ``{"error": "..."}``."""
    if (isinstance(value, dict) and set(value) == {"error"}
            and isinstance(value["error"], str)):
        raise DeviceUnavailable("GET", endpoint, 200, value["error"])


@dataclass(frozen=True)
class ArmState:
    """One sample of an arm: what it is doing and what it was told to do."""

    mode: Any
    error_code: int
    feedback_joints: Tuple[float, ...]
    command_joints: Tuple[float, ...]
    feedback_velocity: Tuple[float, ...]
    feedback_torque: Tuple[float, ...]
    feedback_temperature: Tuple[float, ...]
    frame_serial: int
    stationary: bool

    endpoint = "/v1/arm/{side}/state"

    @classmethod
    def parse(cls, value: Any) -> "ArmState":
        _reject_degraded_slot(value, cls.endpoint)
        try:
            arrays = {key: joints7(value[key]) for key in (
                "feedback_joints", "command_joints", "feedback_velocity",
                "feedback_torque", "feedback_temperature")}
            if not isinstance(value["stationary"], bool):
                raise ValueError("stationary must be boolean")
            mode = value["mode"]
            if not (mode in ("idle", "position", "pvt", "torque", "release",
                             "error")
                    or (isinstance(mode, dict) and set(mode) == {"unknown"}
                        and isinstance(mode["unknown"], str))):
                raise ValueError(f"invalid arm mode {mode!r}")
            if type(value["error_code"]) is not int or type(value["frame_serial"]) is not int:
                raise ValueError("error_code and frame_serial must be integers")
            return cls(mode=mode, error_code=value["error_code"],
                       frame_serial=int(value["frame_serial"]),
                       stationary=value["stationary"], **arrays)
        except (KeyError, TypeError, ValueError) as exc:
            raise ProtocolError(f"invalid arm state: {exc}") from exc


@dataclass(frozen=True)
class GripperState:
    """One sample of the parallel gripper.

    ``kind`` is the last stroke's OUTCOME (:data:`STROKE_KINDS`) and it is the
    field that says whether the jaws are stationary because they hold
    something or because the motor is disabled. It was parsed and then thrown
    away by every consumer; :attr:`faulted` is what reads it.
    """

    kind: str
    jaw_rad: float
    torque_nm: float
    holding: bool
    grip_preload_rad: float
    live: bool
    open_rad: Optional[float] = None
    coil_c: Optional[int] = None
    #: the daemon's own fault code, when it publishes one (d1-firmware #91
    #: adds it; older daemons do not carry the key at all). ``None`` means
    #: "not published", which is NOT the same as zero.
    fault_code: Optional[int] = None

    endpoint = "/v1/gripper/{side}/state"

    @property
    def faulted(self) -> bool:
        """Is this gripper in a state no further stroke will leave?

        Either the outcome is one of :data:`FAULT_KINDS`, or the daemon
        published a non-zero ``fault_code``. Both, because they arrived in
        different firmware versions and the older one only has the kind.
        """
        return (self.kind in FAULT_KINDS
                or bool(self.fault_code))

    @classmethod
    def parse(cls, value: Any) -> "GripperState":
        _reject_degraded_slot(value, cls.endpoint)
        try:
            fields = {key: float(value[key]) for key in (
                "jaw_rad", "torque_nm", "grip_preload_rad")}
            if not all(math.isfinite(v) for v in fields.values()):
                raise ValueError("non-finite gripper reading")
            if not isinstance(value["kind"], str):
                raise ValueError("kind must be a string")
            if type(value["holding"]) is not bool or type(value.get("live", False)) is not bool:
                raise ValueError("holding and live must be boolean")
            for key in ("open_rad", "coil_c"):
                if value.get(key) is not None and not math.isfinite(value[key]):
                    raise ValueError(f"{key} must be finite when available")
            fault_code = value.get("fault_code")
            if fault_code is not None and type(fault_code) is not int:
                raise ValueError("fault_code must be an integer when present")
            return cls(kind=value["kind"], holding=value["holding"],
                       live=value.get("live", False),
                       open_rad=value.get("open_rad"), coil_c=value.get("coil_c"),
                       fault_code=fault_code, **fields)
        except (KeyError, TypeError, ValueError) as exc:
            raise ProtocolError(f"invalid gripper state: {exc}") from exc


class FirmwareClient:
    """A d1-firmwared connection whose client was checked against the daemon.

    Constructing one runs
    :func:`~manipulation_kit.executors.firmware.ensure.ensure_client`, so by the
    time any request goes out the generated tree in use is either the one this
    release ships (spec hashes match) or one regenerated from *this* daemon's
    document. ``tree=`` skips that when the caller has already resolved it.
    """

    def __init__(self, base_url: str = "http://127.0.0.1:4750", *,
                 timeout: float = 2.0,
                 policy: str = "auto",
                 cache_dir: Optional[Path] = None,
                 tree: Optional[ClientTree] = None):
        url = urlsplit(base_url)
        if (url.scheme not in ("http", "https") or not url.hostname
                or url.username or url.password or url.query or url.fragment
                or url.path not in ("", "/")):
            raise ValueError("base_url must be an http(s) origin without credentials")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be positive and finite")
        self.base_url = base_url.rstrip("/")
        self.tree = tree if tree is not None else ensure_client(
            self.base_url, policy=policy, cache_dir=cache_dir)
        #: The generated package — every operation the daemon publishes.
        self.api = self.tree.module
        import httpx  # noqa: PLC0415 - a dependency of the [firmware] extra only

        #: The generated client. Hand it to any generated operation.
        self.api_client = self.api.Client(
            base_url=self.base_url, raise_on_unexpected_status=False,
            timeout=httpx.Timeout(timeout))
        self._http = self.api_client.get_httpx_client()

    # -- provenance -------------------------------------------------------- #
    @property
    def spec_sha256(self) -> str:
        """sha256 of the OpenAPI document this client was generated from."""
        return self.tree.spec_sha256

    def daemon_spec_version(self) -> str:
        """``info.version`` of the document THIS daemon serves.

        ``GET /openapi.json`` is not enveloped — it is the document — so it does
        not go through :meth:`request`.
        """
        response = self._http.request("GET", "/openapi.json")
        try:
            return str(json.loads(response.content)["info"]["version"])
        except (ValueError, KeyError, TypeError) as exc:
            raise ProtocolError(
                f"GET /openapi.json: HTTP {response.status_code}, "
                f"not an OpenAPI document") from exc

    def api_module(self, dotted: str):
        """Import one generated operation module, e.g. ``"arm.arm_state"``.

        The generated tree is imported under a name that depends on where it
        came from — the bundled snapshot is a subpackage of this wheel, a
        regenerated one is ``_mkit_d1fw_api_<sha>`` out of the cache — so a
        literal ``from d1fw_api.api.arm import arm_state`` would either fail or,
        worse, pick up a DIFFERENT project's ``d1fw_api``. This resolves against
        the tree this client actually resolved.
        """
        return importlib.import_module(f"{self.api.__name__}.api.{dotted}")

    # -- the wire ---------------------------------------------------------- #
    def request(self, method: str, path: str, body: Any = None) -> Any:
        """One enveloped ``/v1`` call; returns its ``data``.

        Raises :class:`~manipulation_kit.executors.firmware.errors.FirmwareError`
        on a refusal (carrying the daemon's own message) and
        :class:`~manipulation_kit.executors.firmware.errors.ProtocolError` when
        the answer is not the shape the spec advertises.
        """
        kwargs: Dict[str, Any] = {}
        if body is not None:
            kwargs["content"] = json.dumps(body, allow_nan=False)
            kwargs["headers"] = {"Content-Type": "application/json"}
        response = self._http.request(method, path, **kwargs)
        try:
            envelope = json.loads(response.content)
        except ValueError as exc:
            raise ProtocolError(
                f"{method} {path}: HTTP {response.status_code}, invalid JSON") from exc
        if not isinstance(envelope, dict) or envelope.get("status") not in ("ok", "error"):
            raise ProtocolError(f"{method} {path}: invalid envelope")
        if not response.is_success or envelope["status"] != "ok":
            raise FirmwareError(method, path, response.status_code,
                                str(envelope.get("message")))
        if "data" not in envelope:
            raise ProtocolError(f"{method} {path}: missing data")
        return envelope["data"]

    # -- the four verbs the executor needs --------------------------------- #
    def arm_state(self, side: str) -> ArmState:
        return ArmState.parse(self.request("GET", f"/v1/arm/{side_name(side)}/state"))

    def gripper_state(self, side: str) -> GripperState:
        return GripperState.parse(
            self.request("GET", f"/v1/gripper/{side_name(side)}/state"))

    def gripper_set(self, side: str, closedness: float, *,
                    grip: Optional[str] = None) -> None:
        if not math.isfinite(closedness) or not 0 <= closedness <= 1:
            raise ValueError("closedness must be finite in [0, 1]")
        if grip is not None and grip not in GRIPS:
            raise ValueError(f"grip must be one of {GRIPS}, got {grip!r}")
        body: Dict[str, Any] = {"closedness": float(closedness)}
        if grip is not None:
            body["grip"] = grip
        self.request("POST", f"/v1/gripper/{side_name(side)}/set", body)

    # -- lifecycle --------------------------------------------------------- #
    def close(self) -> None:
        self._http.close()

    def __enter__(self) -> "FirmwareClient":
        return self

    def __exit__(self, *_exc: Any) -> None:
        self.close()
