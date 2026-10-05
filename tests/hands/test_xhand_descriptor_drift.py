"""The XHAND1 / XHAND1 Lite joint tables against their source of truth.

``d1-firmwared`` takes both descriptors from its hand driver library, whose
Rust descriptor source is not part of this repository. Put a copy of it at
``$MKIT_HAND_DESCRIPTOR_DIR/descriptor.rs`` and this test parses
``JOINT_NAMES``, ``JOINT_LIMITS_DEG``, ``LITE_JOINT_NAMES`` and
``LITE_JOINT_LIMITS_RAD`` from it and compares them with the kit's tables;
without it the test SKIPS and says so.
"""
import math
import os
import re
from pathlib import Path

import pytest

from manipulation_kit.hands.robotera import xhand1, xhand1_lite


def _const(src: str, name: str) -> str:
    m = re.search(rf"pub const {name}: [^=]+= \[(.*?)\];", src, re.S)
    assert m, f"{name} not found in descriptor.rs"
    return m.group(1)


def _names(body: str):
    return tuple(re.findall(r'"(\w+)"', body))


def _pairs(body: str, src: str = ""):
    """``(a, b)`` pairs; a bound may be a number or a ``pub const`` name."""
    consts = {n: float(v) for n, v in re.findall(
        r"pub const (\w+): f(?:32|64) = (-?\d+(?:\.\d+)?);", src)}
    tok = r"(-?\d+(?:\.\d+)?|[A-Z_][A-Z0-9_]*)"

    def val(t):
        return consts[t] if t in consts else float(t)
    return tuple((val(a), val(b))
                 for a, b in re.findall(rf"\(\s*{tok}\s*,\s*{tok}\s*\)", body))


def test_xhand_tables_match_the_hand_driver():
    root = os.environ.get("MKIT_HAND_DESCRIPTOR_DIR")
    path = Path(root) / "descriptor.rs" if root else None
    if path is None or not path.is_file():
        pytest.skip("no $MKIT_HAND_DESCRIPTOR_DIR/descriptor.rs: the hand "
                    "driver's descriptor is not part of this repository")
    src = path.read_text()
    assert _names(_const(src, "JOINT_NAMES")) == xhand1.AXIS_NAMES
    assert _pairs(_const(src, "JOINT_LIMITS_DEG")) == xhand1.axes.JOINT_LIMITS_DEG
    for (lo, hi), (rlo, rhi) in zip(xhand1.axes.JOINT_LIMITS_DEG, xhand1.JOINT_LIMITS_RAD):
        assert (math.radians(lo), math.radians(hi)) == (rlo, rhi)
    assert _names(_const(src, "LITE_JOINT_NAMES")) == xhand1_lite.AXIS_NAMES
    assert _pairs(_const(src, "LITE_JOINT_LIMITS_RAD"), src) == xhand1_lite.JOINT_LIMITS_RAD
