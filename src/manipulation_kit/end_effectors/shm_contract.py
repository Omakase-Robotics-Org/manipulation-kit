"""The ``end_effector.shm/1`` messages, as ctypes structures.

``end_effector.shm/1`` is the contract by which an end effector driven by a
process of its own — a *provider* — is served by ``d1-firmwared`` through its
``/v1/end_effectors`` resource, over iceoryx2 0.9.3 shared memory. The
contract's text and its byte layout live in the d1-firmware repository
(``spec/end_effector.shm/1.md`` and ``spec/end_effector.shm/1.layout.json``);
this module mirrors version 1 for a provider written in Python.

``end_effector.shm.1.layout.json`` beside this file is a verbatim copy of that
layout. Version 1 is frozen (a change is version 2, in its own file), so the
copy's SHA-256 is pinned in :data:`LAYOUT_SHA256` — the same value the
contract's Rust crate pins — and the tests check every structure below
against it field by field: a layout that drifts on either side fails a test
on that side.

This module computes only: it imports no iceoryx2 and opens nothing.
"""

from __future__ import annotations

import ctypes
import json
import math
import os
import time
from dataclasses import dataclass, field
from importlib import resources
from typing import Any, Dict, List, Optional, Tuple

#: The contract's name and major version.
CONTRACT = "end_effector.shm/1"
#: ``schema_version`` in every message.
SCHEMA_VERSION = 1
#: The iceoryx2 release every participant must be built against.
ICEORYX2_VERSION = "0.9.3"
#: SHA-256 of ``end_effector.shm.1.layout.json`` (frozen with version 1).
LAYOUT_SHA256 = "e66b7890142f92fe2429d48847a906e313e0836b3558fc4806a7b3eb0c527a49"
#: The vendored layout's file name, beside this module.
LAYOUT_FILE = "end_effector.shm.1.layout.json"

#: The shared-memory root and prefix on a robot.
DEFAULT_ROOT_PATH = "/run/d1-firmwared/iceoryx2/"
DEFAULT_PREFIX = "omakase_ee_"

MAX_JOINTS = 32
NAME_LEN = 64
KEY_LEN = 32
MAX_SET_EXTRA = 8
FAULT_LEN = 128
MESSAGE_LEN = 256
DETAIL_LEN = 8192
EXTRA_LEN = 1024
IDS_LEN = 1024
BODY_LEN = 16384
MAX_BASE_LEN = 128
DEFAULT_SET_TIMEOUT_MS = 10_000
MAX_SET_TIMEOUT_MS = 60_000
MAX_COMMAND_HZ = 10_000

LATCHED_REPUBLISH_MS = 500
MAX_HEARTBEAT_PERIOD_MS = 100
DEFAULT_STALE_AFTER_MS = 500
CONTROL_PERIOD_MS = 100
START_WINDOW_MS = 250
JOINTS_MAX_AGE_MS = 200

JOINT_UNIT_RAD = 0
JOINT_UNIT_FRACTION = 1

CAP_OPENNESS = 1 << 0
CAP_JOINTS = 1 << 1
CAP_STROKE_COMPLETION = 1 << 2
CAP_GRIP_TORQUE = 1 << 3
CAP_CONTACT_SENSING = 1 << 4
CAP_CLEAR_FAULT = 1 << 5
CAP_REGISTERS = 1 << 6
CAP_TICKS = 1 << 7

GRIP_SOFT = 1 << 0
GRIP_FIRM = 1 << 1
GRIP_STRONG = 1 << 2

FORCE_SENSING_NONE = 0
FORCE_SENSING_ESTIMATED = 1
FORCE_SENSING_MEASURED = 2

STATE_LIVE = 1 << 0
STATE_TRACKING = 1 << 1
STATE_HOLDING_KNOWN = 1 << 2
STATE_HOLDING = 1 << 3
STATE_CONTACT_KNOWN = 1 << 4
STATE_CONTACT = 1 << 5
STATE_CAP_REACHED_KNOWN = 1 << 6
STATE_CAP_REACHED = 1 << 7
STATE_DETAIL_DROPPED = 1 << 8

PHASE_IDLE = 0
PHASE_CLOSING = 1
PHASE_OPENING = 2
PHASE_HOLD = 3

TICK_COMMANDED = 1 << 0
TICK_EFFORT = 1 << 1

OP_SET = 1
OP_STOP = 2
OP_CLEAR_FAULT = 1
OP_REGISTERS = 2

SET_WAIT = 1 << 0
SET_GRIP = 1 << 1
SET_TORQUE = 1 << 2

GRIP_INDEX_SOFT = 0
GRIP_INDEX_FIRM = 1
GRIP_INDEX_STRONG = 2

STATUS_OK = 0
STATUS_REFUSED = 1
STATUS_UNPROCESSABLE = 2
STATUS_UNAVAILABLE = 3
STATUS_DEVICE = 4
STATUS_TIMEOUT = 5
STATUS_STALE_TARGET = 6
STATUS_INVALID = 7

#: Service suffixes under a side's base name.
DESCRIPTOR = "descriptor"
HEARTBEAT = "heartbeat"
STATE = "state"
TICK = "tick"
JOINTS = "joints"
CONTROL = "control"
COMMAND = "command"
FAULT = "fault"

u8, u32, u64, i32, f64 = (ctypes.c_uint8, ctypes.c_uint32, ctypes.c_uint64,
                          ctypes.c_int32, ctypes.c_double)


def _named(name: str):
    """Give a structure the iceoryx2 type name the contract registers."""
    def wrap(cls):
        cls.type_name = staticmethod(lambda: name)
        return cls
    return wrap


@_named("omakase_ee_v1_Header")
class Header(ctypes.Structure):
    _fields_ = [("schema_version", u32), ("reserved", u32), ("instance_id", u64),
                ("seq", u64), ("mono_ns", u64)]


@_named("omakase_ee_v1_Joint")
class Joint(ctypes.Structure):
    _fields_ = [("name", u8 * KEY_LEN), ("unit", u32), ("reserved", u32),
                ("min", f64), ("max", f64)]


@_named("omakase_ee_v1_ExtraKey")
class ExtraKey(ctypes.Structure):
    _fields_ = [("name", u8 * KEY_LEN)]


@_named("omakase_ee_v1_Descriptor")
class Descriptor(ctypes.Structure):
    _fields_ = [("header", Header), ("generation", u64), ("driver", u8 * NAME_LEN),
                ("family", u8 * NAME_LEN), ("model", u8 * NAME_LEN), ("node_id", i32),
                ("joint_count", u32), ("joints", Joint * MAX_JOINTS), ("capabilities", u32),
                ("grip_presets", u32), ("force_sensing", u32), ("max_command_hz", u32),
                ("set_timeout_ms", u32), ("set_extra_count", u32),
                ("set_extra", ExtraKey * MAX_SET_EXTRA)]


@_named("omakase_ee_v1_Heartbeat")
class Heartbeat(ctypes.Structure):
    _fields_ = [("header", Header), ("descriptor_generation", u64),
                ("state_period_ms", u32), ("reserved", u32)]


@_named("omakase_ee_v1_State")
class State(ctypes.Structure):
    _fields_ = [("header", Header), ("descriptor_generation", u64), ("openness", f64),
                ("torque_nm", f64), ("flags", u32), ("joint_count", u32),
                ("joints", f64 * MAX_JOINTS), ("fault", u8 * FAULT_LEN), ("detail_len", u32),
                ("reserved", u32), ("detail", u8 * DETAIL_LEN)]


@_named("omakase_ee_v1_Tick")
class Tick(ctypes.Structure):
    _fields_ = [("header", Header), ("descriptor_generation", u64), ("tick_seq", u64),
                ("timestamp_us", u64), ("phase", u32), ("joint_count", u32), ("flags", u32),
                ("reserved", u32), ("measured", f64 * MAX_JOINTS),
                ("commanded", f64 * MAX_JOINTS), ("effort", f64 * MAX_JOINTS),
                ("fault", u8 * FAULT_LEN)]


@_named("omakase_ee_v1_JointsCommand")
class JointsCommand(ctypes.Structure):
    _fields_ = [("header", Header), ("target_instance_id", u64),
                ("descriptor_generation", u64), ("timestamp_us", u64), ("joint_count", u32),
                ("reserved", u32), ("targets", f64 * MAX_JOINTS)]


@_named("omakase_ee_v1_Control")
class Control(ctypes.Structure):
    _fields_ = [("header", Header), ("stop_generation", u64), ("period_ms", u32),
                ("reserved", u32)]


@_named("omakase_ee_v1_CommandRequest")
class CommandRequest(ctypes.Structure):
    _fields_ = [("header", Header), ("target_instance_id", u64),
                ("descriptor_generation", u64), ("start_by_mono_ns", u64), ("op", u32),
                ("flags", u32), ("grip", u32), ("extra_len", u32), ("openness", f64),
                ("torque_nm", f64), ("extra", u8 * EXTRA_LEN)]


@_named("omakase_ee_v1_CommandResponse")
class CommandResponse(ctypes.Structure):
    _fields_ = [("header", Header), ("status", u32), ("reserved", u32),
                ("message", u8 * MESSAGE_LEN)]


@_named("omakase_ee_v1_FaultRequest")
class FaultRequest(ctypes.Structure):
    _fields_ = [("header", Header), ("target_instance_id", u64), ("op", u32),
                ("ids_len", u32), ("ids", u8 * IDS_LEN)]


@_named("omakase_ee_v1_FaultResponse")
class FaultResponse(ctypes.Structure):
    _fields_ = [("header", Header), ("status", u32), ("body_len", u32),
                ("message", u8 * MESSAGE_LEN), ("body", u8 * BODY_LEN)]


#: Every message, in the order the contract lists them.
MESSAGES = (Header, Joint, ExtraKey, Descriptor, Heartbeat, State, Tick, JointsCommand,
            Control, CommandRequest, CommandResponse, FaultRequest, FaultResponse)


def load_layout() -> Dict[str, Any]:
    """The vendored layout document."""
    text = resources.files(__package__).joinpath(LAYOUT_FILE).read_text(encoding="utf-8")
    return json.loads(text)


# --------------------------------------------------------------------------
# Field codecs
# --------------------------------------------------------------------------


class ContractError(ValueError):
    """A message, or a declaration, the contract refuses."""


def monotonic_ns() -> int:
    """``CLOCK_MONOTONIC`` nanoseconds: the clock of every ``mono_ns``."""
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


def wall_clock_us() -> int:
    """Wall-clock microseconds since the Unix epoch."""
    return time.time_ns() // 1000


def new_instance_id() -> int:
    """A random non-zero 64-bit instance id, drawn once per process start."""
    while True:
        value = int.from_bytes(os.urandom(8), "little")
        if value:
            return value


def header(instance_id: int, seq: int) -> Header:
    """A header for a message sent now as the ``seq``-th on its service."""
    return Header(schema_version=SCHEMA_VERSION, reserved=0, instance_id=instance_id,
                  seq=seq, mono_ns=monotonic_ns())


def check_header(value: Header) -> None:
    """Refuse a header of another schema version or with no sender."""
    if value.schema_version != SCHEMA_VERSION:
        raise ContractError(f"schema_version {value.schema_version} is not the contract's "
                            f"{SCHEMA_VERSION}")
    if value.instance_id == 0:
        raise ContractError("instance_id 0 names no sender")


def put_str(field_: Any, text: str, what: str) -> None:
    """Write ``text`` NUL-padded into a byte array field."""
    data = text.encode("utf-8")
    if len(data) > len(field_):
        raise ContractError(f"{what} {text!r} is {len(data)} bytes; the field holds "
                            f"{len(field_)}")
    if b"\0" in data:
        raise ContractError(f"{what} {text!r} contains a NUL byte")
    ctypes.memset(ctypes.addressof(field_), 0, len(field_))
    ctypes.memmove(ctypes.addressof(field_), data, len(data))


def put_text(field_: Any, text: str) -> None:
    """Write human-readable text, cut at a character boundary when too long."""
    data = text.replace("\0", " ").encode("utf-8")[:len(field_)]
    data = data.decode("utf-8", errors="ignore").encode("utf-8")
    ctypes.memset(ctypes.addressof(field_), 0, len(field_))
    ctypes.memmove(ctypes.addressof(field_), data, len(data))


def get_str(field_: Any, what: str) -> str:
    """Read a NUL-padded UTF-8 field, refusing bytes after the first NUL."""
    data = bytes(field_)
    end = data.find(b"\0")
    if end < 0:
        end = len(data)
    if any(data[end:]):
        raise ContractError(f"{what} has bytes after its terminating NUL")
    try:
        return data[:end].decode("utf-8")
    except UnicodeDecodeError as error:
        raise ContractError(f"{what} is not UTF-8") from error


def put_json(field_: Any, value: Any, what: str) -> int:
    """Write ``value`` as JSON into a byte array; return its length."""
    data = json.dumps(value, separators=(",", ":")).encode("utf-8")
    if len(data) > len(field_):
        raise ContractError(f"{what} is {len(data)} bytes of JSON; the field holds "
                            f"{len(field_)}")
    ctypes.memset(ctypes.addressof(field_), 0, len(field_))
    ctypes.memmove(ctypes.addressof(field_), data, len(data))
    return len(data)


def get_json(field_: Any, length: int, what: str) -> Any:
    """Read ``length`` bytes of JSON from a byte array, ``None`` when empty."""
    if length > len(field_):
        raise ContractError(f"{what} length {length} exceeds its field's {len(field_)}")
    if length == 0:
        return None
    try:
        return json.loads(bytes(field_)[:length].decode("utf-8"))
    except (UnicodeDecodeError, ValueError) as error:
        raise ContractError(f"{what} is not JSON: {error}") from error


def _count(value: int, capacity: int, what: str) -> int:
    if value > capacity:
        raise ContractError(f"{what} {value} exceeds the contract's {capacity}")
    return value


def _no_unknown_bits(value: int, known: int, what: str) -> None:
    if value & ~known:
        raise ContractError(f"{what} sets bits {value & ~known:#x} that schema "
                            f"{SCHEMA_VERSION} does not define")


def _finite(values: List[float], what: str) -> None:
    if not all(math.isfinite(value) for value in values):
        raise ContractError(f"{what} holds a non-finite value")


# --------------------------------------------------------------------------
# The declaration
# --------------------------------------------------------------------------

GRIP_PRESETS = (("soft", GRIP_SOFT, GRIP_INDEX_SOFT), ("firm", GRIP_FIRM, GRIP_INDEX_FIRM),
                ("strong", GRIP_STRONG, GRIP_INDEX_STRONG))
FORCE_SENSING = {"none": FORCE_SENSING_NONE, "estimated": FORCE_SENSING_ESTIMATED,
                 "measured": FORCE_SENSING_MEASURED}
JOINT_UNITS = {"rad": JOINT_UNIT_RAD, "fraction": JOINT_UNIT_FRACTION}


@dataclass
class JointSpec:
    """One joint of the ``joints`` command mode."""

    name: str
    unit: str  # "rad" or "fraction"
    min: float
    max: float


@dataclass
class Capabilities:
    """What an end effector can do. Every flag is a promise the provider keeps."""

    openness: bool = False
    joints: bool = False
    stroke_completion: bool = False
    grip_presets: List[str] = field(default_factory=list)
    grip_torque: bool = False
    set_extra: List[str] = field(default_factory=list)
    force_sensing: str = "none"
    contact_sensing: bool = False
    clear_fault: bool = False
    registers: bool = False
    ticks: bool = False
    max_command_hz: Optional[int] = None


@dataclass
class DescriptorSpec:
    """What a provider drives."""

    driver: str
    family: str
    model: str
    joints: List[JointSpec]
    capabilities: Capabilities
    node_id: Optional[int] = None
    set_timeout_ms: Optional[int] = None

    def validate(self) -> None:
        """Refuse a declaration the daemon would refuse."""
        for what, value in (("driver", self.driver), ("family", self.family),
                            ("model", self.model)):
            if not value or len(value.encode()) > NAME_LEN or "\0" in value:
                raise ContractError(f"descriptor {what} {value!r} must be 1 to {NAME_LEN} "
                                    "bytes without NUL")
        if len(self.joints) > MAX_JOINTS:
            raise ContractError(f"{len(self.joints)} joints; the contract holds {MAX_JOINTS}")
        names = set()
        for joint in self.joints:
            if not joint.name or len(joint.name.encode()) > KEY_LEN or "\0" in joint.name:
                raise ContractError(f"joint name {joint.name!r} must be 1 to {KEY_LEN} bytes")
            if joint.name in names:
                raise ContractError(f"joint name {joint.name!r} is declared twice")
            names.add(joint.name)
            if joint.unit not in JOINT_UNITS:
                raise ContractError(f"joint {joint.name} unit {joint.unit!r} is not rad or "
                                    "fraction")
            if not (math.isfinite(joint.min) and math.isfinite(joint.max)) \
                    or joint.min >= joint.max:
                raise ContractError(f"joint {joint.name} limits must be finite with min "
                                    "below max")
        caps = self.capabilities
        if caps.joints and not self.joints:
            raise ContractError("the joints capability needs at least one joint")
        if any(preset not in dict((n, b) for n, b, _ in GRIP_PRESETS)
               for preset in caps.grip_presets):
            raise ContractError(f"grip presets {caps.grip_presets} name an unknown preset")
        if caps.force_sensing not in FORCE_SENSING:
            raise ContractError(f"force_sensing {caps.force_sensing!r} is not a sensing kind")
        if len(caps.set_extra) > MAX_SET_EXTRA or len(set(caps.set_extra)) != \
                len(caps.set_extra):
            raise ContractError("set_extra holds at most 8 distinct keys")
        for key in caps.set_extra:
            if not key or len(key.encode()) > KEY_LEN or "\0" in key:
                raise ContractError(f"set_extra key {key!r} must be 1 to {KEY_LEN} bytes")
        if caps.max_command_hz is not None and not 1 <= caps.max_command_hz <= MAX_COMMAND_HZ:
            raise ContractError(f"max_command_hz must be 1 to {MAX_COMMAND_HZ}")
        if self.set_timeout_ms is not None and \
                not 1 <= self.set_timeout_ms <= MAX_SET_TIMEOUT_MS:
            raise ContractError(f"set_timeout_ms must be 1 to {MAX_SET_TIMEOUT_MS}")
        if self.node_id is not None and not 0 <= self.node_id <= 0xFFFF:
            raise ContractError("node_id must be 0 to 65535")

    def to_wire(self, generation: int, value_header: Header) -> Descriptor:
        """The wire form, as generation ``generation``."""
        self.validate()
        out = Descriptor()
        out.header = value_header
        out.generation = generation
        put_str(out.driver, self.driver, "driver")
        put_str(out.family, self.family, "family")
        put_str(out.model, self.model, "model")
        out.node_id = -1 if self.node_id is None else self.node_id
        out.joint_count = len(self.joints)
        for slot, joint in zip(out.joints, self.joints):
            put_str(slot.name, joint.name, "joint name")
            slot.unit = JOINT_UNITS[joint.unit]
            slot.min = joint.min
            slot.max = joint.max
        caps = self.capabilities
        bits = 0
        for flag, bit in ((caps.openness, CAP_OPENNESS), (caps.joints, CAP_JOINTS),
                          (caps.stroke_completion, CAP_STROKE_COMPLETION),
                          (caps.grip_torque, CAP_GRIP_TORQUE),
                          (caps.contact_sensing, CAP_CONTACT_SENSING),
                          (caps.clear_fault, CAP_CLEAR_FAULT), (caps.registers, CAP_REGISTERS),
                          (caps.ticks, CAP_TICKS)):
            if flag:
                bits |= bit
        out.capabilities = bits
        out.grip_presets = sum(bit for name, bit, _ in GRIP_PRESETS
                               if name in caps.grip_presets)
        out.force_sensing = FORCE_SENSING[caps.force_sensing]
        out.max_command_hz = caps.max_command_hz or 0
        out.set_timeout_ms = self.set_timeout_ms or 0
        out.set_extra_count = len(caps.set_extra)
        for slot, key in zip(out.set_extra, caps.set_extra):
            put_str(slot.name, key, "set_extra key")
        return out


# --------------------------------------------------------------------------
# What a provider publishes
# --------------------------------------------------------------------------


@dataclass
class StateSample:
    """One state sample, in descriptor order and units."""

    openness: Optional[float] = None
    joints: List[float] = field(default_factory=list)
    live: bool = False
    tracking: bool = False
    holding: Optional[bool] = None
    contact: Optional[bool] = None
    cap_reached: Optional[bool] = None
    torque_nm: Optional[float] = None
    fault: Optional[str] = None
    #: ``{"family", "schema_version", "data"}`` or ``None``
    detail: Optional[Dict[str, Any]] = None

    def to_wire(self, generation: int, value_header: Header) -> State:
        """The wire form; a detail too large is left out and flagged."""
        out = State()
        out.header = value_header
        out.descriptor_generation = generation
        if self.openness is not None and not 0.0 <= self.openness <= 1.0:
            raise ContractError(f"state openness {self.openness} is outside [0, 1]")
        out.openness = math.nan if self.openness is None else self.openness
        out.torque_nm = math.nan if self.torque_nm is None else self.torque_nm
        if len(self.joints) > MAX_JOINTS:
            raise ContractError("state joints exceed the contract's 32")
        _finite(self.joints, "state joints")
        out.joint_count = len(self.joints)
        for index, value in enumerate(self.joints):
            out.joints[index] = value
        put_text(out.fault, self.fault or "")
        flags = STATE_LIVE if self.live else 0
        flags |= STATE_TRACKING if self.tracking else 0
        for value, known, bit in ((self.holding, STATE_HOLDING_KNOWN, STATE_HOLDING),
                                  (self.contact, STATE_CONTACT_KNOWN, STATE_CONTACT),
                                  (self.cap_reached, STATE_CAP_REACHED_KNOWN,
                                   STATE_CAP_REACHED)):
            if value is not None:
                flags |= known | (bit if value else 0)
        if self.detail is not None:
            try:
                out.detail_len = put_json(out.detail, self.detail, "detail")
            except ContractError:
                out.detail_len = 0
                flags |= STATE_DETAIL_DROPPED
        out.flags = flags
        return out


@dataclass
class TickSample:
    """One control-loop tick."""

    seq: int
    timestamp_us: int
    measured: List[float]
    commanded: Optional[List[float]] = None
    effort: Optional[List[float]] = None
    fault: Optional[str] = None
    phase: int = PHASE_IDLE

    def to_wire(self, generation: int, value_header: Header) -> Tick:
        """The wire form."""
        out = Tick()
        out.header = value_header
        out.descriptor_generation = generation
        out.tick_seq = self.seq
        out.timestamp_us = self.timestamp_us
        out.phase = self.phase
        if len(self.measured) > MAX_JOINTS:
            raise ContractError("tick measured exceeds the contract's 32")
        _finite(self.measured, "tick measured")
        out.joint_count = len(self.measured)
        for index, value in enumerate(self.measured):
            out.measured[index] = value
        for values, array, bit in ((self.commanded, out.commanded, TICK_COMMANDED),
                                   (self.effort, out.effort, TICK_EFFORT)):
            if values is not None:
                if len(values) != len(self.measured):
                    raise ContractError("tick arrays must be as long as measured")
                _finite(values, "tick values")
                for index, value in enumerate(values):
                    array[index] = value
                out.flags |= bit
        put_text(out.fault, self.fault or "")
        return out


def heartbeat(generation: int, state_period_ms: int, value_header: Header) -> Heartbeat:
    """A heartbeat naming descriptor generation ``generation``."""
    return Heartbeat(header=value_header, descriptor_generation=generation,
                     state_period_ms=state_period_ms, reserved=0)


# --------------------------------------------------------------------------
# What the daemon sends
# --------------------------------------------------------------------------


@dataclass
class SetCommand:
    """A ``set``, as a provider receives it."""

    openness: float
    wait: bool = True
    grip: Optional[str] = None
    torque_nm: Optional[float] = None
    extra: Optional[Dict[str, Any]] = None


@dataclass
class CommandEnvelope:
    """A ``command`` request: its ``SetCommand``, or ``None`` for a stop."""

    target_instance_id: int
    descriptor_generation: int
    start_by_mono_ns: int
    command: Optional[SetCommand]


def read_command(message: CommandRequest) -> CommandEnvelope:
    """Read a ``command`` request."""
    check_header(message.header)
    if message.op == OP_STOP:
        command = None
    elif message.op == OP_SET:
        _no_unknown_bits(message.flags, SET_WAIT | SET_GRIP | SET_TORQUE, "set flags")
        if not 0.0 <= message.openness <= 1.0:
            raise ContractError(f"openness {message.openness} is outside [0, 1]")
        grip = None
        if message.flags & SET_GRIP:
            names = [name for name, _, index in GRIP_PRESETS if index == message.grip]
            if not names:
                raise ContractError(f"grip index {message.grip} names no preset")
            grip = names[0]
        torque = message.torque_nm if message.flags & SET_TORQUE else None
        if torque is not None and not math.isfinite(torque):
            raise ContractError("torque_nm is not finite")
        extra = get_json(message.extra, message.extra_len, "set extra")
        if extra is not None and not isinstance(extra, dict):
            raise ContractError("set extra is not a JSON object")
        command = SetCommand(openness=message.openness, wait=bool(message.flags & SET_WAIT),
                             grip=grip, torque_nm=torque, extra=extra)
    else:
        raise ContractError(f"command op {message.op} is not an op")
    return CommandEnvelope(target_instance_id=message.target_instance_id,
                           descriptor_generation=message.descriptor_generation,
                           start_by_mono_ns=message.start_by_mono_ns, command=command)


@dataclass
class JointTargets:
    """Streamed joint targets, as a provider receives them."""

    target_instance_id: int
    descriptor_generation: int
    issued_mono_ns: int
    timestamp_us: int
    targets: List[float]


def read_joints(message: JointsCommand) -> JointTargets:
    """Read a ``joints`` sample."""
    check_header(message.header)
    count = _count(message.joint_count, MAX_JOINTS, "joints joint_count")
    targets = list(message.targets[:count])
    _finite(targets, "joint targets")
    return JointTargets(target_instance_id=message.target_instance_id,
                        descriptor_generation=message.descriptor_generation,
                        issued_mono_ns=message.header.mono_ns,
                        timestamp_us=message.timestamp_us, targets=targets)


def read_control(message: Control) -> Tuple[int, int]:
    """Read a ``control`` word: (daemon instance id, stop generation)."""
    check_header(message.header)
    return message.header.instance_id, message.stop_generation


def read_fault_request(message: FaultRequest) -> Tuple[int, int, Optional[List[str]]]:
    """Read a ``fault`` request: (target instance, op, register ids)."""
    check_header(message.header)
    if message.op == OP_CLEAR_FAULT:
        return message.target_instance_id, OP_CLEAR_FAULT, None
    if message.op == OP_REGISTERS:
        ids = get_json(message.ids, message.ids_len, "register ids")
        if ids is not None and (not isinstance(ids, list)
                                or not all(isinstance(i, str) for i in ids)):
            raise ContractError("register ids are not a JSON array of strings")
        return message.target_instance_id, OP_REGISTERS, ids
    raise ContractError(f"fault op {message.op} is not an op")


def command_response(status: int, message: str, value_header: Header) -> CommandResponse:
    """A ``command`` response."""
    out = CommandResponse()
    out.header = value_header
    out.status = status
    put_text(out.message, message if status != STATUS_OK else "")
    return out


def fault_response(status: int, message: str, body: Any,
                   value_header: Header) -> FaultResponse:
    """A ``fault`` response; a body too large is answered ``device``."""
    out = FaultResponse()
    out.header = value_header
    out.status = status
    if status == STATUS_OK:
        try:
            out.body_len = put_json(out.body, body, "fault response body")
        except ContractError as error:
            out.status = STATUS_DEVICE
            out.body_len = 0
            put_text(out.message, str(error))
    else:
        put_text(out.message, message)
    return out
