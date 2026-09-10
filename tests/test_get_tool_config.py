"""The hand-agnostic tool-config seam: get_tool_config resolves
<maker>/<model> to that hand's physical registration data (TCP / mass /
COM / inertia) without the consumer importing it, and the export CLI emits
the JSON an arm controller's set-tool call consumes."""
import json

import pytest

import manipulation_kit.hands as oh
from manipulation_kit.hands.export_tool_config import main as export_main
from manipulation_kit.hands.toolconfig import SCHEMA, ToolConfig


def test_dh116s_tool_config_values():
    """The REGISTERED set is the arm engineer's 2026-08-08 one: TCP 100 mm,
    COM z 40 mm, 0.4 kg — supplied to fix compliance-mode misbehaviour on d1-1
    and superseding the measured 2026-07 set (TCP 210 mm / COM z 120 mm /
    0.380 kg weighed).

    This test asserted the OLD numbers and had been red on dx-manipulator
    ``main`` since ``43fef52`` changed the registration without it. Fixed
    during the move rather than carried over red — but the open question the
    module docstring raises travels with it: the TCP/COM halving looks like a
    change of reference point (fingertips -> palm face), and that is an
    inference, not a record. See :mod:`...dh116s.toolconfig`.
    """
    tc = oh.get_tool_config("leadshine/dh116s")
    assert isinstance(tc, ToolConfig)
    assert tc.kinematics() == [0.0, 0.0, 100.0, 0.0, 0.0, 0.0]
    dyn = tc.dynamics()
    assert len(dyn) == 10
    assert dyn[0] == 0.4                        # mass kg
    assert dyn[1:4] == [0.0, 0.0, 40.0]         # COM mm
    # inertia is an estimate: small, positive-definite diagonal, vendor
    # upper-triangular order (xx, xy, xz, yy, yz, zz)
    ixx, ixy, ixz, iyy, iyz, izz = dyn[4:]
    assert ixx > 0 and iyy > 0 and izz > 0
    assert ixy == ixz == iyz == 0.0
    assert ixx < 0.01 and iyy < 0.01


def test_parallel_gripper_matches_dsdk_default():
    # The stock D1 gripper is data too — must equal the values that were
    # hardcoded as d1-sdk ToolConfig::defaultGripper(). The 1.5 kg was
    # confirmed on a scale 2026-07-29; do not "correct" it towards the
    # shell-only vendor CAD's 0.328 kg.
    tc = oh.get_tool_config("d1/parallel_gripper")
    assert tc.kinematics() == [0.0, 0.0, 136.0, 0.0, 0.0, 0.0]
    assert tc.dynamics() == [1.5, 0.0, 0.0, 68.0,
                             0.003, 0.0, 0.0, 0.003, 0.0, 0.001]


def test_every_hand_records_the_mass_that_was_weighed():
    """Both hands were weighed at the robot on 2026-07-29, and every hand must
    still be able to say what it weighed:

        d1/parallel_gripper   registered 1.5 kg    weighed 1.5   kg
        leadshine/dh116s      registered 0.4 kg    weighed 0.380 kg

    The gripper's invariant is the strict one — what is registered IS what was
    on the scale; do not "correct" 1.5 kg towards the shell-only vendor CAD's
    0.328 kg.

    The DH116S is the documented exception, and it is worth stating rather than
    hiding: since 2026-08-08 it registers the ARM ENGINEER's 0.4 kg instead of
    the 0.380 kg weighed, as part of a set supplied to fix compliance-mode
    misbehaviour on d1-1. 20 g of that is fine as a margin; what is NOT
    documented is why the TCP and COM halved in the same change. So the
    invariant kept here is the one that still holds for both: MEASURED_MASS_KG
    records the scale reading, the registered mass is never BELOW it, and the
    provenance says which is which. A hand that cannot say what it weighs has
    no business being registered from a datasheet."""
    from manipulation_kit.hands.d1.parallel_gripper import toolconfig as grip
    from manipulation_kit.hands.leadshine.dh116s import toolconfig as hand

    for module in (grip, hand):
        tc = module.tool_config()
        assert tc.mass_kg >= module.MEASURED_MASS_KG, (
            f"{tc.model} registers LESS than the {module.MEASURED_MASS_KG} kg "
            f"on the scale — gravity compensation would under-hold it")
        assert str(module.MEASURED_MASS_KG) in tc.provenance, (
            f"{tc.model} does not record the weighed mass in its provenance")
    assert grip.MEASURED_MASS_KG == 1.5
    assert grip.tool_config().mass_kg == 1.5      # gripper: registered == weighed
    assert hand.MEASURED_MASS_KG == 0.380
    assert hand.tool_config().mass_kg == 0.4      # engineer's set, 2026-08-08


@pytest.mark.parametrize("model", ["leadshine/dh116s", "d1/parallel_gripper"])
def test_json_export_schema(model):
    doc = oh.get_tool_config(model).to_json_dict()
    assert doc["schema"] == SCHEMA
    assert doc["model"] == model
    assert len(doc["kinematics"]) == 6
    assert len(doc["dynamics"]) == 10
    assert all(isinstance(v, float) for v in doc["kinematics"])
    assert all(isinstance(v, float) for v in doc["dynamics"])
    assert doc["_comment"]  # provenance must not be empty


@pytest.mark.parametrize("bad", ["", "leadshine", "no-slash", "un/known"])
def test_get_tool_config_rejects_malformed_and_unknown(bad):
    with pytest.raises((ValueError, NotImplementedError)):
        oh.get_tool_config(bad)


def test_export_cli_writes_json(tmp_path):
    out = tmp_path / "dh116s.json"
    assert export_main(["leadshine/dh116s", str(out)]) == 0
    doc = json.loads(out.read_text())
    assert doc["kinematics"][2] == 100.0
    assert doc["dynamics"][0] == 0.4


def test_export_cli_usage_error():
    assert export_main([]) == 2
