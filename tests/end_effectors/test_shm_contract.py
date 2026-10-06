"""The ctypes messages of ``end_effector.shm/1`` against the contract's
layout, and their codecs.

The vendored layout is version 1's, which is frozen: its SHA-256 is pinned
here and in the contract's Rust crate (``LAYOUT_SHA256``), so a change on
either side fails a test on that side. Every structure is checked field by
field against it — name, offset, size and type — and every constant
against its code table. With ``D1_FIRMWARE_DIR`` naming a d1-firmware
checkout, the vendored copy is also compared byte for byte with that
checkout's ``spec/end_effector.shm/1.layout.json``.
"""

from __future__ import annotations

import ctypes
import hashlib
import json
import math
import os
from importlib import resources
from pathlib import Path

import pytest

from manipulation_kit.end_effectors import shm_contract as c

LAYOUT = c.load_layout()
BY_NAME = {cls.__name__: cls for cls in c.MESSAGES}
PRIMITIVES = {"u8": ctypes.c_uint8, "u32": ctypes.c_uint32, "u64": ctypes.c_uint64,
              "i32": ctypes.c_int32, "f64": ctypes.c_double}


def _vendored_bytes() -> bytes:
    return resources.files("manipulation_kit.end_effectors") \
        .joinpath(c.LAYOUT_FILE).read_bytes()


def test_the_vendored_layout_is_the_frozen_version_1():
    assert hashlib.sha256(_vendored_bytes()).hexdigest() == c.LAYOUT_SHA256
    assert LAYOUT["contract"] == c.CONTRACT
    assert LAYOUT["schema_version"] == c.SCHEMA_VERSION
    assert LAYOUT["iceoryx2"] == c.ICEORYX2_VERSION


@pytest.mark.skipif(not os.environ.get("D1_FIRMWARE_DIR"),
                    reason="D1_FIRMWARE_DIR names no d1-firmware checkout")
def test_the_vendored_layout_is_the_daemons():
    spec = Path(os.environ["D1_FIRMWARE_DIR"]) / "spec" / "end_effector.shm" / "1.layout.json"
    assert spec.read_bytes() == _vendored_bytes()


def _ctype_of(spec: str):
    """The ctypes type a layout type string names (``u8[64]``, ``Joint[32]``)."""
    if "[" in spec:
        base, count = spec[:-1].split("[")
        return _ctype_of(base) * int(count)
    return PRIMITIVES.get(spec) or BY_NAME[spec]


@pytest.mark.parametrize("message", LAYOUT["messages"], ids=lambda m: m["name"])
def test_every_structure_has_the_contract_layout(message):
    cls = BY_NAME[message["name"]]
    assert cls.type_name() == message["type_name"]
    assert ctypes.sizeof(cls) == message["size"]
    assert ctypes.alignment(cls) == message["align"]
    assert [name for name, _ in cls._fields_] == [f["name"] for f in message["fields"]]
    for (name, ctype), spec in zip(cls._fields_, message["fields"]):
        assert getattr(cls, name).offset == spec["offset"], name
        assert getattr(cls, name).size == spec["size"], name
        expected = _ctype_of(spec["type"])
        assert ctypes.sizeof(ctype) == ctypes.sizeof(expected), name
        assert getattr(ctype, "_type_", None) == getattr(expected, "_type_", None), name
        assert getattr(ctype, "_length_", None) == getattr(expected, "_length_", None), name


def test_every_constant_is_the_contracts():
    for name, value in LAYOUT["codes"].items():
        assert getattr(c, name) == value, name
    for name, value in LAYOUT["limits"].items():
        assert getattr(c, name) == value, name
    for name, value in LAYOUT["timing_ms"].items():
        assert getattr(c, name) == value, name
    assert LAYOUT["root"]["root_path"] == c.DEFAULT_ROOT_PATH
    assert LAYOUT["root"]["prefix"] == c.DEFAULT_PREFIX
    suffixes = [service["suffix"] for service in LAYOUT["services"]]
    assert suffixes == [c.DESCRIPTOR, c.HEARTBEAT, c.STATE, c.TICK, c.JOINTS, c.CONTROL,
                        c.COMMAND, c.FAULT]


def _gripper() -> c.DescriptorSpec:
    return c.DescriptorSpec(
        driver="acme-gripper", family="parallel_gripper", model="acme/g2", node_id=3,
        joints=[c.JointSpec("jaw", "rad", 0.0, 0.08)],
        capabilities=c.Capabilities(openness=True, joints=True, grip_presets=["soft", "firm"],
                                    set_extra=["speed"], force_sensing="estimated",
                                    ticks=True, max_command_hz=100),
        set_timeout_ms=3000)


def test_a_declaration_encodes_as_the_contract_spells_it():
    message = _gripper().to_wire(1, c.header(7, 1))
    assert message.header.schema_version == 1
    assert c.get_str(message.model, "model") == "acme/g2"
    assert message.node_id == 3
    assert message.joint_count == 1
    assert c.get_str(message.joints[0].name, "joint") == "jaw"
    assert message.capabilities == c.CAP_OPENNESS | c.CAP_JOINTS | c.CAP_TICKS
    assert message.grip_presets == c.GRIP_SOFT | c.GRIP_FIRM
    assert message.force_sensing == c.FORCE_SENSING_ESTIMATED
    assert message.max_command_hz == 100
    assert message.set_extra_count == 1


@pytest.mark.parametrize("change", [
    lambda d: setattr(d, "model", "m" * 65),
    lambda d: d.joints.append(c.JointSpec("jaw", "rad", 0.0, 1.0)),
    lambda d: setattr(d.joints[0], "max", -1.0),
    lambda d: setattr(d.joints[0], "unit", "deg"),
    lambda d: setattr(d.capabilities, "max_command_hz", 0),
    lambda d: setattr(d.capabilities, "grip_presets", ["hard"]),
])
def test_a_declaration_the_daemon_refuses_does_not_encode(change):
    descriptor = _gripper()
    change(descriptor)
    with pytest.raises(c.ContractError):
        descriptor.to_wire(1, c.header(7, 1))


def test_a_state_carries_its_tristates_and_drops_an_oversize_detail():
    sample = c.StateSample(openness=0.25, joints=[0.02], live=True, holding=True,
                           contact=False, fault="hot",
                           detail={"family": "parallel_gripper", "schema_version": 1,
                                   "data": {"kind": "grasp"}})
    message = sample.to_wire(1, c.header(7, 2))
    assert message.flags == (c.STATE_LIVE | c.STATE_HOLDING_KNOWN | c.STATE_HOLDING
                             | c.STATE_CONTACT_KNOWN)
    assert math.isnan(message.torque_nm)
    assert c.get_str(message.fault, "fault") == "hot"
    assert c.get_json(message.detail, message.detail_len, "detail")["data"]["kind"] == "grasp"
    big = c.StateSample(detail={"data": "x" * c.DETAIL_LEN}).to_wire(1, c.header(7, 3))
    assert big.detail_len == 0
    assert big.flags & c.STATE_DETAIL_DROPPED


def test_commands_decode_and_malformed_ones_are_refused():
    request = c.CommandRequest()
    request.header = c.header(9, 1)
    request.target_instance_id = 7
    request.descriptor_generation = 1
    request.start_by_mono_ns = 123
    request.op = c.OP_SET
    request.flags = c.SET_WAIT | c.SET_GRIP
    request.grip = c.GRIP_INDEX_STRONG
    request.openness = 0.5
    request.extra_len = c.put_json(request.extra, {"speed": 0.2}, "extra")
    envelope = c.read_command(request)
    assert envelope.command == c.SetCommand(openness=0.5, wait=True, grip="strong",
                                            extra={"speed": 0.2})
    request.op = c.OP_STOP
    assert c.read_command(request).command is None
    for field_, value in (("op", 9), ("flags", 1 << 5)):
        bad = c.CommandRequest.from_buffer_copy(request)
        bad.op = c.OP_SET
        setattr(bad, field_, value)
        with pytest.raises(c.ContractError):
            c.read_command(bad)
    bad = c.CommandRequest.from_buffer_copy(request)
    bad.header.schema_version = 2
    with pytest.raises(c.ContractError, match="schema_version 2"):
        c.read_command(bad)


def test_text_after_the_nul_is_refused_and_long_text_is_cut_at_a_character():
    field_ = (ctypes.c_uint8 * 4)()
    c.put_text(field_, "aé日本")
    assert c.get_str(field_, "t") == "aé"
    padded = (ctypes.c_uint8 * 6)()
    c.put_str(padded, "ab", "t")
    padded[4] = 1
    with pytest.raises(c.ContractError, match="after its terminating NUL"):
        c.get_str(padded, "t")


def test_responses_carry_status_and_body():
    response = c.fault_response(c.STATUS_OK, "", {"ok": True, "fault": None, "details": {}},
                                c.header(7, 1))
    assert json.loads(bytes(response.body[:response.body_len]))["ok"] is True
    oversize = c.fault_response(c.STATUS_OK, "", "x" * c.BODY_LEN, c.header(7, 2))
    assert oversize.status == c.STATUS_DEVICE
    refused = c.command_response(c.STATUS_REFUSED, "no", c.header(7, 3))
    assert c.get_str(refused.message, "message") == "no"
