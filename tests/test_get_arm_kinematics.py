"""The arm-agnostic seam — same shape as ``get_hand`` / ``get_tool_config``."""

from __future__ import annotations

import sys
import types

import pytest

from manipulation_kit.arms import get_arm_kinematics, get_clutch_tuning


@pytest.mark.parametrize("bad", ["d1_arm", "", "/d1_arm", "d1/", "d1"])
def test_malformed_id_rejected(bad):
    with pytest.raises(ValueError, match="<maker>/<model>"):
        get_arm_kinematics(bad)
    with pytest.raises(ValueError, match="<maker>/<model>"):
        get_clutch_tuning(bad)


def test_unknown_model_is_a_value_error():
    with pytest.raises(ValueError, match="no kinematics for arm model"):
        get_arm_kinematics("acme/nonesuch")
    with pytest.raises(ValueError, match="unknown arm model"):
        get_clutch_tuning("acme/nonesuch")


def _install_stub(monkeypatch, name, **attrs):
    """Register a fake ``manipulation_kit.arms.<maker>.<model>.kinematics`` module."""
    for part, mod in (("manipulation_kit.arms.stubmaker", types.ModuleType("x")),
                      ("manipulation_kit.arms.stubmaker." + name, types.ModuleType("y"))):
        monkeypatch.setitem(sys.modules, part, mod)
    km = types.ModuleType("kinematics")
    for k, v in attrs.items():
        setattr(km, k, v)
    monkeypatch.setitem(
        sys.modules, f"manipulation_kit.arms.stubmaker.{name}.kinematics", km)
    return km


def test_model_without_factory_raises_not_implemented(monkeypatch):
    _install_stub(monkeypatch, "noflow")
    with pytest.raises(NotImplementedError, match="build_kinematics"):
        get_arm_kinematics("stubmaker/noflow")
    with pytest.raises(NotImplementedError, match="clutch_tuning"):
        get_clutch_tuning("stubmaker/noflow")


def test_factory_receives_keywords(monkeypatch):
    seen = {}

    def build_kinematics(**kw):
        seen.update(kw)
        return "built"

    _install_stub(monkeypatch, "ok", build_kinematics=build_kinematics,
                  clutch_tuning=lambda: "tuned")
    assert get_arm_kinematics("stubmaker/ok", guard=None, quiet=True) == "built"
    assert seen == {"guard": None, "quiet": True}
    assert get_clutch_tuning("stubmaker/ok") == "tuned"


def test_d1_arm_is_registered_without_importing_mujoco():
    """Resolving the tuning must NOT drag in the MuJoCo substrate.

    A consumer that only shapes targets (or only wants the workspace box) has to
    be able to do it on a machine with no MuJoCo installed — omakase-core is
    exactly that consumer.
    """
    pytest.importorskip("scipy")
    had_mujoco = "mujoco" in sys.modules
    tun = get_clutch_tuning("d1/arm")
    assert tun.workspace["x"] == (0.15, 0.55)
    assert tun.pos_scale == 1.5
    if not had_mujoco:
        assert "mujoco" not in sys.modules, (
            "get_clutch_tuning imported mujoco — the pose-math path must stay "
            "substrate-free")
