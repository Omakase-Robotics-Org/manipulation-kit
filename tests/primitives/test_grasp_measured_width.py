"""A stalled grasp MEASURES the object's width, and the measurement outranks
the declaration.

The live case (d1-2, 2026-09-23 03:46Z, trace ``astra-20260923-1146``
record 6): the model declared ``tape`` 50x50x18 mm; a right-hand tip grasp
stalled at a 57.1 mm pad gap with the daemon reporting holding=true,
jaw_stalled=true, closedness 0.056 — the hand WAS holding the roll (~57 mm
across). The old +-4 mm window around the declared width said FALSE ("the
jaws never reached it"), the model believed it, let go and ran out of turns.
"""

from __future__ import annotations

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from manipulation_kit.primitives import grasp_geometry as gg
from manipulation_kit.primitives.orientation import jaw_axis
from manipulation_kit.primitives.types import Verdict
from manipulation_kit.primitives.verifiers import (FIT_BLOCKED, FIT_EMPTY,
                                                   FIT_HELD, Holding, grip_fit)
from manipulation_kit.world import (ArmView, ContainerView, GripperView,
                                    ObjectView, SurfaceView, WorldView,
                                    with_measured_width)

#: record 6, as the trace has it
TAPE = ObjectView("tape", p=(0.359, -0.188, 0.175), size=(0.05, 0.05, 0.018),
                  confidence=0.7)
TOOL_Q = (0.7071, 0.7071, 0.0002, 0.0001)      # right tool, after the stroke
OPEN_GAP_M = 0.0605                              # d1-2 open_rad 1.35


def _world(*, holding=False, gap=OPEN_GAP_M, stalled=False, closedness=0.0,
           held=None):
    return WorldView.of(
        [SurfaceView("table", p=(0.506, 0.0, 0.156), size=(0.4, 0.6, 0.02)),
         TAPE,
         ContainerView("cup", p=(0.382, -0.031, 0.226), size=(0.09, 0.09, 0.12),
                       interior=(0.065, 0.065, 0.11))],
        arms=[ArmView("right", joints=np.zeros(7),
                      tool_p=np.array([0.359, -0.188, 0.197]),
                      tool_r=R.from_quat(TOOL_Q), mode="position")],
        grippers=[GripperView("right", closedness, holding=holding,
                              jaw_gap_m=gap, jaw_stalled=stalled,
                              open_gap_m=OPEN_GAP_M, held_object=held)])


def _verifier(reference=gg.TIP):
    return Holding("grasp", _world(), "right", TAPE,
                   jaw_axis=jaw_axis(R.from_quat(TOOL_Q)), reference=reference)


def test_record_6_replayed_is_a_hold_with_the_width_corrected():
    report = _verifier()(_world(holding=True, gap=0.0571, stalled=True,
                                closedness=0.056, held="tape"))
    assert report.verdict == Verdict.TRUE, report.reason
    assert "holding 'tape' at a 57.1 mm gap" in report.reason
    assert "declared 50.0 mm, width corrected" in report.reason
    assert "MEASURED 57.1 mm" in report.reason
    m = report.measured
    assert m["fit"] == FIT_HELD and m["matches_declaration"] is False
    # the old window survives as a NOTE on the declaration, not the verdict
    assert m["width_window_m"] == [0.046, 0.054]
    assert m["width_provenance"] == "measured"
    fix = m["width_correction"]
    assert fix["object"] == "tape"
    assert (fix["declared_m"], fix["measured_m"]) == (0.05, 0.0571)
    assert np.allclose(np.abs(fix["jaw_axis"]), [0.0, 1.0, 0.0], atol=1e-3)


def test_a_gap_near_zero_is_nothing_held():
    report = _verifier()(_world(holding=True, gap=0.003, stalled=True,
                                closedness=0.95, held="tape"))
    assert report.verdict == Verdict.FALSE
    assert report.measured["fit"] == FIT_EMPTY
    assert "nothing is between the pads" in report.reason


def test_a_gap_near_the_open_gap_is_blocked():
    report = _verifier()(_world(holding=True, gap=0.0600, stalled=True,
                                closedness=0.01, held="tape"))
    assert report.verdict == Verdict.FALSE
    assert report.measured["fit"] == FIT_BLOCKED
    assert "never reached it" in report.reason


def test_a_plausible_gap_without_holding_is_still_false():
    report = _verifier()(_world(holding=False, gap=0.0571, stalled=True,
                                closedness=0.056))
    assert report.verdict == Verdict.FALSE
    assert "nothing between its pads" in report.reason


def test_a_declaration_that_was_right_carries_no_correction():
    report = _verifier()(_world(holding=True, gap=0.0512, stalled=True,
                                closedness=0.15, held="tape"))
    assert report.verdict == Verdict.TRUE
    assert report.measured["matches_declaration"] is True
    assert "width_correction" not in report.measured
    assert "corrected" not in report.reason


def test_the_contact_reference_sets_the_blocked_margin():
    """The band's upper edge is the open gap less the per-side clearance the
    grasp's reference plans with: 2 mm for the tips, 4 mm for the pads. The
    same 57.1 mm is a hold for a tip grasp and pads that barely moved for a
    pad grasp on a 60.5 mm hand."""
    assert grip_fit(0.0571, 0.05, open_gap_m=OPEN_GAP_M,
                    reference=gg.TIP)["fit"] == FIT_HELD
    assert grip_fit(0.0571, 0.05, open_gap_m=OPEN_GAP_M,
                    reference=gg.PAD)["fit"] == FIT_BLOCKED
    # a thin card is not "empty" at 5 mm: the empty floor is half its width
    assert grip_fit(0.005, 0.006, open_gap_m=OPEN_GAP_M,
                    reference=gg.TIP)["fit"] == FIT_HELD


def test_the_measured_width_replaces_the_declared_extent_along_the_jaws():
    world = _world()
    axis = jaw_axis(R.from_quat(TOOL_Q))
    measured = with_measured_width(TAPE, world.frames, axis, 0.0571)
    assert measured.extent_along(axis, world.frames) == pytest.approx(0.0571)
    assert measured.size_provenance == "measured"
    assert measured.to_json()["size_provenance"] == "measured"
    assert "size MEASURED" in measured.to_text(world.frames)
    # only the measured side moved
    assert measured.size[0] == pytest.approx(0.05)
    assert measured.size[2] == pytest.approx(0.018)
    with pytest.raises(ValueError):
        ObjectView("x", p=(0, 0, 0), size=(1, 1, 1), size_provenance="guessed")
