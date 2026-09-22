"""The verb surface :class:`FirmwareExecutor` drives, over the generated client.

:mod:`~manipulation_kit.executors.firmware.ensure` produces a *generated* client
— the whole daemon surface, one typed operation per route, a model for every
schema — from the daemon's own OpenAPI document. **Everything this package
reads from d1-firmwared comes through that client** (Shu's firmware access
principle, 2026-09-22): the request is built by the generated operation, the
answer is decoded by the generated model, and a field the document does not
carry is ``None`` here — never a key read by hand, never a kit constant
standing in for a number the daemon publishes.

So :class:`FirmwareClient` is a thin adapter, not a second client:

* **operations are the generated ones** — ``arm.arm_state``,
  ``gripper.gripper_state``, ``gripper.gripper_set``, ``neck.neck_state``,
  ``slider.slider_state``: their ``_get_kwargs`` builds the request, their
  models (``ArmState``, ``GripperReport``, ``NeckState``, ``SliderState``,
  ``SlotError``) decode the ``data``;
* **the envelope is checked once**, here, because every ``/v1`` response is
  ``{"status": …, "data": …, "message": …}`` and a caller that forgets is a
  caller that treats an error body as a reading;
* **the conversion to the kit's state is field for field**
  (:func:`joint_state`, :func:`hand_state`, :func:`neck_state`,
  :func:`lift_state`), with the units made the kit's (degrees -> radians) and
  the one piece of GEOMETRY the kit owns — motor radians -> jaw gap, from the
  hand description — applied to the daemon's own values.

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
from pathlib import Path
from typing import Any, Callable, Dict, Optional, Sequence, Tuple
from urllib.parse import urlsplit

import numpy as np

from ...executor import HandState, JointState, LiftState, NeckState
from ...hands.d1.parallel_gripper.description import gap_from_motor_rad
from .ensure import ClientTree, document_bytes, ensure_client
from .errors import DeviceUnavailable, FirmwareError, ProtocolError

#: The wire spelling of a side, as the daemon names them.
SIDES: Tuple[str, str] = ("a", "b")
#: Routes this adapter drives through generated operations. A test asserts
#: every one of them is in the bundled OpenAPI document, so a spec that moves
#: a route breaks the build rather than the robot.
ROUTES = ("/v1/arm/{side}/state", "/v1/gripper/{side}/state",
          "/v1/gripper/{side}/set", "/v1/neck/state", "/v1/slider/state")

#: The daemon's ``StrokeKind`` values that mean the GRIPPER IS FAULTED: the
#: motor latched or refused the stroke, and every stroke after it ends the
#: same way until it is cleared. A faulted gripper's jaws are perfectly
#: stationary, which is how a fault used to pass the stroke barrier as a
#: finished stroke (Astra review 13; d1-2 2026-09-22, the -4.17 Nm hold).
FAULT_KINDS: Tuple[str, ...] = ("fault", "overload")
#: ...that mean the stroke stopped ON something before its target.
STALLED_KINDS: Tuple[str, ...] = ("grasp", "contact")
#: ...that mean the stroke ran to its target with nothing in the way.
FREE_KINDS: Tuple[str, ...] = ("open", "empty")
#: ...that are terminal and establish the jaws' state (a successful stroke,
#: including "closed on nothing", which is a true answer about the world).
SETTLED_KINDS: Tuple[str, ...] = STALLED_KINDS + FREE_KINDS + ("lost",)
#: ...that ran out of time inside the daemon: terminal, and it says nothing.
UNFINISHED_KINDS: Tuple[str, ...] = ("timeout",)


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


# --------------------------------------------------------------------------- #
# generated model -> kit state, field for field
# --------------------------------------------------------------------------- #

def _given(value: Any) -> Any:
    """``None`` for a field the document marks optional and the daemon left
    out (the generated ``Unset``), else the value."""
    if value is None or type(value).__name__ == "Unset":
        return None
    return value


def _word(value: Any) -> Optional[str]:
    """A generated enum (or the ``{"unknown": …}`` arm-mode object) -> its word."""
    value = _given(value)
    if value is None:
        return None
    if hasattr(value, "value"):
        return str(value.value)
    if hasattr(value, "unknown"):
        return "unknown"
    return str(value)


def _finite(value: Any, name: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def joint_state(arm: Any) -> JointState:
    """``ArmState`` (generated) -> :class:`~manipulation_kit.executor.JointState`.

    ``feedback_joints`` / ``feedback_velocity`` are degrees on the wire and
    radians in the kit; ``feedback_torque`` is newton-metres on both.
    """
    try:
        q = np.radians(joints7(arm.feedback_joints))
        qd = np.radians(joints7(arm.feedback_velocity))
        torque = np.asarray(joints7(arm.feedback_torque), dtype=float)
        if type(arm.error_code) is not int:
            raise ValueError("error_code must be an integer")
        if not isinstance(arm.stationary, bool):
            raise ValueError("stationary must be boolean")
        return JointState(q=q, qd=qd, torque_nm=torque,
                          mode=_word(arm.mode) or "unknown",
                          error_code=int(arm.error_code),
                          stationary=bool(arm.stationary))
    except (AttributeError, TypeError, ValueError) as exc:
        raise ProtocolError(f"invalid arm state: {exc}") from exc


def hand_state(report: Any, *, commanded: Optional[float] = None,
               gap_of: Callable[[float], float] = gap_from_motor_rad
               ) -> HandState:
    """``GripperReport`` (generated) -> :class:`~manipulation_kit.executor.HandState`.

    * ``closedness`` = ``1 - jaw_rad / open_rad`` — ``None`` when the daemon
      does not publish ``open_rad``, because a closedness against a ceiling
      the kit made up is a number about the kit, not the jaws;
    * ``jaw_gap_m`` / ``open_gap_m`` are the daemon's ``jaw_rad`` /
      ``open_rad`` through the hand description's kinematic map ``gap_of``;
    * ``commanded`` is the daemon's ``target_closedness`` when it publishes
      one (the streamed target lane), else what THIS executor last commanded
      (``commanded``), else ``None``;
    * ``stalled`` and ``fault`` are read from the stroke outcome ``kind`` and
      ``fault_code``, never from jaw motion.
    """
    try:
        jaw = _finite(report.jaw_rad, "jaw_rad")
        torque = _finite(report.torque_nm, "torque_nm")
        if type(report.holding) is not bool:
            raise ValueError("holding must be boolean")
        open_rad = _given(getattr(report, "open_rad", None))
        open_rad = None if open_rad is None else _finite(open_rad, "open_rad")
        target = _given(getattr(report, "target_closedness", None))
        target = None if target is None else _finite(target, "target_closedness")
        kind = _word(report.kind) or ""
        code = _given(getattr(report, "fault_code", None))
    except (AttributeError, TypeError, ValueError) as exc:
        raise ProtocolError(f"invalid gripper state: {exc}") from exc
    closedness = (None if open_rad is None or open_rad <= 0.0
                  else max(0.0, min(1.0, 1.0 - jaw / open_rad)))
    if code is not None:
        fault: Optional[str] = str(code)
    elif kind in FAULT_KINDS:
        fault = kind
    else:
        fault = None
    stalled = (True if kind in STALLED_KINDS else
               False if kind in FREE_KINDS else None)
    return HandState(
        closedness=closedness,
        commanded=target if target is not None else commanded,
        holding=bool(report.holding),
        jaw_gap_m=gap_of(jaw),
        torque_nm=torque,
        stalled=stalled,
        fault=fault,
        open_gap_m=(None if open_rad is None or open_rad <= 0.0
                    else gap_of(open_rad)))


def neck_state(model: Any) -> NeckState:
    """``NeckState`` (generated) -> the kit's. Pitch sign is the daemon's
    LOGICAL one and is NOT flipped here (``head_camera`` owns that)."""
    try:
        return NeckState(pitch_rad=_finite(model.pitch, "pitch"),
                         yaw_rad=_finite(model.yaw, "yaw"),
                         enabled=bool(model.enabled), moving=bool(model.moving))
    except (AttributeError, TypeError, ValueError) as exc:
        raise ProtocolError(f"invalid neck state: {exc}") from exc


def lift_state(model: Any) -> LiftState:
    """``SliderState`` (generated) -> :class:`~manipulation_kit.executor.LiftState`.

    ``alarm`` is the drive's own ``alarm_text`` when ``alarm`` is set (its
    ``alarm_code`` when the text is empty), ``None`` otherwise.
    """
    try:
        alarm = None
        if bool(model.alarm):
            alarm = (str(model.alarm_text or "").strip()
                     or f"alarm_code {int(model.alarm_code)}")
        return LiftState(height_m=_finite(model.height_m, "height_m"),
                         moving=bool(model.moving), alarm=alarm)
    except (AttributeError, TypeError, ValueError) as exc:
        raise ProtocolError(f"invalid slider state: {exc}") from exc


class FirmwareClient:
    """A d1-firmwared connection whose client was checked against the daemon.

    Constructing one runs
    :func:`~manipulation_kit.executors.firmware.ensure.ensure_client`, so by the
    time any request goes out the generated tree in use is either the one this
    release ships (spec hashes match) or one regenerated from *this* daemon's
    document. ``tree=`` skips that when the caller has already resolved it.

    ``timeout`` is the per-request default for the non-blocking reads. An
    operation the document marks with ``x-timeout-seconds`` (the blocking
    gripper stroke: 40 s on 0.3.0) gets that instead
    (:meth:`operation_timeout_s`), so a real close is not cut off at two
    seconds — the reason the example used to build this client with a blanket
    20 s.
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

        self._httpx = httpx
        #: The generated client. Hand it to any generated operation.
        self.api_client = self.api.Client(
            base_url=self.base_url, raise_on_unexpected_status=False,
            timeout=httpx.Timeout(timeout))
        self._http = self.api_client.get_httpx_client()
        self._document: Optional[Dict[str, Any]] = None

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

    def operation_timeout_s(self, method: str, route: str) -> Optional[float]:
        """The document's ``x-timeout-seconds`` for one operation, or ``None``.

        Read from the document the client in use was GENERATED from (the
        generator does not carry vendor extensions into the code), so it is
        the same contract, not a kit constant.
        """
        if self._document is None:
            try:
                self._document = json.loads(document_bytes(self.tree.resolution))
            except (OSError, ValueError):
                self._document = {}
        operation = (self._document.get("paths", {}).get(route, {})
                     .get(method.lower(), {}))
        value = operation.get("x-timeout-seconds")
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        return value if math.isfinite(value) and value > 0 else None

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

    def model(self, name: str):
        """One generated model class, e.g. ``"ArmState"``, from the tree in use."""
        return getattr(importlib.import_module(f"{self.api.__name__}.models"), name)

    # -- the wire ---------------------------------------------------------- #
    def request(self, method: str, path: str, body: Any = None) -> Any:
        """One enveloped ``/v1`` call; returns its ``data``.

        Raises :class:`~manipulation_kit.executors.firmware.errors.FirmwareError`
        on a refusal (carrying the daemon's own message) and
        :class:`~manipulation_kit.executors.firmware.errors.ProtocolError` when
        the answer is not the shape the spec advertises.
        """
        kwargs: Dict[str, Any] = {"method": method, "url": path}
        if body is not None:
            kwargs["content"] = json.dumps(body, allow_nan=False)
            kwargs["headers"] = {"Content-Type": "application/json"}
        return self._send(kwargs)

    def _send(self, kwargs: Dict[str, Any], *,
              timeout_s: Optional[float] = None) -> Any:
        """Send what a generated ``_get_kwargs`` built; check the envelope."""
        method = str(kwargs.get("method", "GET")).upper()
        path = str(kwargs.get("url", ""))
        send = {k: v for k, v in kwargs.items() if k not in ("method", "url")}
        if timeout_s is not None:
            send["timeout"] = self._httpx.Timeout(float(timeout_s))
        response = self._http.request(method, path, **send)
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

    def _read(self, operation: str, model: str, endpoint: str, **params) -> Any:
        """GET through a generated operation, decoded by a generated model.

        A device the daemon could not read degrades to the document's
        ``SlotError`` in its slot; that is a device failure reported correctly
        (:class:`DeviceUnavailable`, the daemon's own words), not a broken
        contract.
        """
        data = self._send(self.api_module(operation)._get_kwargs(**params))
        if isinstance(data, dict) and set(data) == {"error"}:
            slot = self.model("SlotError").from_dict(data)
            raise DeviceUnavailable("GET", endpoint, 200, str(slot.error))
        try:
            return self.model(model).from_dict(data)
        except (AttributeError, KeyError, TypeError, ValueError) as exc:
            raise ProtocolError(f"GET {endpoint}: not a {model}: {exc!r}") from exc

    # -- the verbs the executor needs -------------------------------------- #
    def arm_state(self, side: str):
        """The generated ``ArmState`` for one arm."""
        wire = side_name(side)
        return self._read("arm.arm_state", "ArmState", f"/v1/arm/{wire}/state",
                          side=self.model("ArmSide")(wire))

    def gripper_state(self, side: str):
        """The generated ``GripperReport`` for one gripper."""
        wire = side_name(side)
        return self._read("gripper.gripper_state", "GripperReport",
                          f"/v1/gripper/{wire}/state",
                          side=self.model("ArmSide")(wire))

    def neck_state(self):
        """The generated ``NeckState``."""
        return self._read("neck.neck_state", "NeckState", "/v1/neck/state")

    def slider_state(self):
        """The generated ``SliderState`` (the torso lift)."""
        return self._read("slider.slider_state", "SliderState", "/v1/slider/state")

    def grips(self) -> Tuple[str, ...]:
        """The force presets the document publishes (``GripPreset``)."""
        return tuple(p.value for p in self.model("GripPreset"))

    def gripper_set(self, side: str, closedness: float, *,
                    grip: Optional[str] = None,
                    timeout_s: Optional[float] = None) -> None:
        """One blocking stroke (``POST /v1/gripper/{side}/set``).

        The document says this operation "blocks for the stroke and returns
        nothing": its success ``data`` is ``null``, and the OUTCOME is the
        next ``GripperReport`` (``kind``, ``fault_code``), which is where the
        executor reads it. ``timeout_s`` bounds the HTTP call; the default is
        the document's own ``x-timeout-seconds`` for this operation.
        """
        wire = side_name(side)
        if not math.isfinite(closedness) or not 0 <= closedness <= 1:
            raise ValueError("closedness must be finite in [0, 1]")
        presets = self.grips()
        if grip is not None and grip not in presets:
            raise ValueError(f"grip must be one of {presets}, got {grip!r}")
        fields: Dict[str, Any] = {"closedness": float(closedness)}
        if grip is not None:
            fields["grip"] = self.model("GripPreset")(grip)
        body = self.model("GripperTarget")(**fields)
        kwargs = self.api_module("gripper.gripper_set")._get_kwargs(
            side=self.model("ArmSide")(wire), body=body)
        if timeout_s is None:
            timeout_s = self.operation_timeout_s("POST", "/v1/gripper/{side}/set")
        data = self._send(kwargs, timeout_s=timeout_s)
        if data is not None:
            raise ProtocolError(f"POST /v1/gripper/{wire}/set: the document "
                                f"says this returns null, got {data!r}")

    # -- lifecycle --------------------------------------------------------- #
    def close(self) -> None:
        self._http.close()

    def __enter__(self) -> "FirmwareClient":
        return self

    def __exit__(self, *_exc: Any) -> None:
        self.close()
