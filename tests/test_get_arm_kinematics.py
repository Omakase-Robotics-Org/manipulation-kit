"""The arm-agnostic seam — same shape as ``get_hand`` / ``get_tool_config``."""

from __future__ import annotations

import os
import subprocess
import sys
import textwrap
import types

import pytest

from manipulation_kit.arms import get_arm_kinematics, get_clutch_tuning


#: run from ``/`` so no stray relative path can help it: build the arm, run a
#: guarded solve, and fail if ``mujoco`` ever entered ``sys.modules``.
_NO_MUJOCO_PROBE = textwrap.dedent("""
    import sys
    from manipulation_kit.arms import get_arm_kinematics
    arm = get_arm_kinematics("d1/arm", guard=None, find_ready=False, quiet=True)
    p, r = arm.ee_pose("left")
    assert arm.solve_ee("left", p, r).ok, "guarded solve failed"
    assert "mujoco" not in sys.modules, "the default IK path imported mujoco"
    print("ok")
""")


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


def test_solving_ik_does_not_import_mujoco():
    """THE regression this file exists for, after 2026-09-11.

    ``get_arm_kinematics("d1/arm")`` used to build a MuJoCo model — so inverse
    kinematics, the thing every consumer that moves an arm needs, dragged a
    physics engine into the install. It now walks the URDF in numpy by default.
    This asserts the whole path, not just the import: build the arm, run a
    guarded solve, and require that ``mujoco`` never entered ``sys.modules``.

    In a SUBPROCESS on purpose: by the time this file runs, another test in
    the session has usually imported ``mujoco`` already, and an in-process
    ``sys.modules`` check would then skip itself and prove nothing.
    """
    pytest.importorskip("scipy")
    proc = subprocess.run(
        [sys.executable, "-c", _NO_MUJOCO_PROBE],
        capture_output=True, text=True, cwd=os.sep)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert proc.stdout.strip().endswith("ok"), proc.stdout


def test_the_mujoco_substrate_is_still_selectable():
    """Optional, not removed: a consumer that already has a MuJoCo mirror (and
    the dx-vr-teleop parity gate) must still be able to ask for it."""
    pytest.importorskip("scipy")
    pytest.importorskip("mujoco")
    arm = get_arm_kinematics("d1/arm", chain="mujoco", guard=None,
                             find_ready=False, quiet=True)
    assert arm.substrate == "mujoco"
    assert arm.model.nq > 0          # a real MjModel was built


def test_an_unknown_substrate_is_a_value_error():
    pytest.importorskip("scipy")
    with pytest.raises(ValueError, match="chain must be one of"):
        get_arm_kinematics("d1/arm", chain="bullet", guard=None, quiet=True)
