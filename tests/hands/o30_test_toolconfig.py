"""The O30's physical registration data — and the provenance rules it must obey.

Kept in the package (rather than in ``tests/test_get_tool_config.py``) because
every assertion here is about THIS hand's numbers; the cross-hand seam tests
stay where they are.
"""

import pytest

from manipulation_kit.hands.linkerbot.o30 import toolconfig as o30
from manipulation_kit.hands.toolconfig import SCHEMA, ToolConfig


def test_registers_the_mass_that_was_weighed():
    """Repo invariant: what is registered is what was on the scale.

    750 g measured at the robot 2026-08-10 beats the vendor's 730 g for the
    bare hand — gravity compensation holds up the assembly (hand + mounting
    hardware), not the catalogue part. Same lesson as the DH116S's 359 g spec
    vs 380 g weighed.
    """
    tc = o30.tool_config()
    assert isinstance(tc, ToolConfig)
    assert tc.mass_kg == o30.MEASURED_MASS_KG == 0.75
    assert o30.SPEC_MASS_KG == 0.730
    assert tc.mass_kg > o30.SPEC_MASS_KG      # mounting hardware is included


def test_tcp_is_not_invented():
    # Undetermined = the flange origin, so nothing pretends to be a measured
    # grasp center. Choosing one is a design decision, not a measurement.
    assert o30.tool_config().kinematics() == [0.0] * 6


def test_com_is_the_provisional_urdf_composition():
    dyn = o30.tool_config().dynamics()
    assert dyn[0] == 0.75
    assert dyn[1:4] == [6.5, 12.2, 80.5]
    # z dominates: the hand is long along the flange axis
    assert dyn[3] > dyn[1] and dyn[3] > dyn[2]


def test_inertia_is_left_unfilled_rather_than_estimated():
    assert o30.tool_config().dynamics()[4:] == [0.0] * 6


def test_provenance_records_every_open_question():
    text = o30.tool_config().provenance.lower()
    for needle in ("provisional", "0.75", "urdf", "tcp", "inertia"):
        assert needle in text, needle


def test_json_export_shape_matches_the_d1_sdk_schema():
    doc = o30.tool_config().to_json_dict()
    assert doc["schema"] == SCHEMA
    assert doc["model"] == "linkerbot/o30"
    assert len(doc["kinematics"]) == 6
    assert len(doc["dynamics"]) == 10
    assert all(isinstance(v, float) for v in doc["kinematics"] + doc["dynamics"])
    assert doc["_comment"]


def test_resolves_through_the_cross_hand_seam():
    import manipulation_kit.hands as oh

    assert oh.get_tool_config("linkerbot/o30").model == "linkerbot/o30"


def test_retarget_is_an_explicit_not_implemented():
    import manipulation_kit.hands as oh

    with pytest.raises(NotImplementedError, match="calibration"):
        oh.get_retarget("linkerbot/o30")


def test_get_hand_dry_run_conforms_to_the_hand_protocol():
    import manipulation_kit.hands as oh

    h = oh.get_hand("linkerbot/o30", execute=False)
    try:
        assert isinstance(h, oh.Hand)
        assert h.num_axes == 20
        assert h.identify()["model"] == "O30"
        h.enable()
        h.home()
        h.set_positions([10] * 20)
        assert h.latest_feedback is None or h.latest_feedback.pos
    finally:
        h.close()


def test_get_hand_maps_node_id_onto_the_frame_id():
    import manipulation_kit.hands as oh

    left = oh.get_hand("linkerbot/o30", node_id=2, execute=False)
    try:
        assert left.frame_id == 2
        assert left.identify()["reply_id"] == 0x402
    finally:
        left.close()


def test_the_registration_announces_that_it_is_provisional():
    """A docstring cannot warn anyone at the moment the numbers are registered.

    The consumer (an arm stack) feeds these into the controller's dynamics
    model. Mounting this hand moves the registered tool point from the DH116S's
    100 mm to the flange origin, so "TCP is not chosen yet" has to be readable
    at runtime, not only in this module's prose.
    """
    tc = o30.tool_config()
    assert tc.provisional is True
    assert tc.unmeasured == ("tcp", "com", "inertia")
    assert "PROVISIONAL" in tc.caveat()
    assert "tcp" in tc.caveat()
    assert tc.to_json_dict()["provisional"] is True
    assert tc.to_json_dict()["unmeasured"] == ["tcp", "com", "inertia"]


def test_a_measured_tool_says_nothing():
    tc = ToolConfig(model="acme/measured", tcp_xyz_mm=(0.0, 0.0, 100.0),
                    mass_kg=0.4, com_mm=(0.0, 0.0, 50.0))
    assert tc.provisional is False and tc.caveat() == ""


def test_a_placeholder_that_does_not_announce_itself_is_refused():
    with pytest.raises(ValueError, match="provisional=False"):
        ToolConfig(model="acme/quiet", tcp_xyz_mm=(0.0, 0.0, 0.0), mass_kg=0.4,
                   com_mm=(0.0, 0.0, 0.0), unmeasured=("tcp",))
