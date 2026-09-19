"""A frame that cannot be trusted must REFUSE, not return a plausible number.

The bug being pinned is ``d1-inference``'s table homography: a fit valid for
exactly one neck pose still evaluates after the neck moves, returns a plane
that looks right, and is wrong by centimetres. Every test here is a variation
on "the graph said no instead".
"""

from __future__ import annotations

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from manipulation_kit.world import (BASE, FRAME_STALE, UNKNOWN_FRAME, Frame,
                                    FrameError, FrameGraph, ObjectView)


def test_a_pose_in_the_base_frame_needs_no_graph_at_all():
    graph = FrameGraph()
    assert np.allclose(graph.to_base([1.0, 2.0, 3.0]), [1.0, 2.0, 3.0])
    assert graph.known(BASE)


def test_a_registered_frame_composes_translation_and_rotation():
    graph = FrameGraph.of([Frame("table", BASE, p=[0.5, 0.0, 0.1],
                                 r=R.from_euler("z", 90, degrees=True))])
    p = graph.to_base([0.1, 0.0, 0.0], frame_id="table")
    # the table's +x points along the base's +y after a 90 deg yaw
    assert np.allclose(p, [0.5, 0.1, 0.1], atol=1e-9)


def test_frames_chain_through_their_parents():
    graph = FrameGraph.of([
        Frame("wagon", BASE, p=[0.6, 0.0, 0.0], r=R.identity()),
        Frame("tray", "wagon", p=[0.0, 0.1, 0.3], r=R.identity()),
    ])
    assert np.allclose(graph.to_base([0.0, 0.0, 0.0], frame_id="tray"),
                       [0.6, 0.1, 0.3])


def test_an_unregistered_frame_refuses_with_unknown_frame():
    graph = FrameGraph()
    with pytest.raises(FrameError) as caught:
        graph.pose_in_base("wagon_aruco:7")
    assert caught.value.reason == UNKNOWN_FRAME


def test_a_frame_whose_parent_is_missing_refuses_too():
    graph = FrameGraph.of([Frame("tray", "wagon", p=[0, 0, 0], r=R.identity())])
    with pytest.raises(FrameError) as caught:
        graph.pose_in_base("tray")
    assert caught.value.reason == UNKNOWN_FRAME
    assert "wagon" in caught.value.detail


def test_a_frame_past_its_max_age_refuses_with_frame_stale():
    graph = FrameGraph.of([Frame("table", BASE, p=[0.5, 0, 0], r=R.identity(),
                                 stamp=100.0, max_age_s=2.0)], now=100.5)
    assert np.allclose(graph.to_base([0, 0, 0], frame_id="table"), [0.5, 0, 0])
    graph.now = 143.0
    with pytest.raises(FrameError) as caught:
        graph.to_base([0, 0, 0], frame_id="table")
    assert caught.value.reason == FRAME_STALE
    assert "43.0s old" in caught.value.detail


def test_an_explicitly_invalidated_fit_refuses_regardless_of_age():
    """``table_frame`` knows its homography died when the neck moved. That is
    not an age, it is a fact, and it must refuse anyway."""
    graph = FrameGraph.of([Frame("table", BASE, p=[0.5, 0, 0], r=R.identity(),
                                 valid=False)])
    with pytest.raises(FrameError) as caught:
        graph.pose_in_base("table")
    assert caught.value.reason == FRAME_STALE
    assert "invalid" in caught.value.detail


def test_staleness_anywhere_in_the_chain_refuses_the_whole_chain():
    graph = FrameGraph.of([
        Frame("wagon", BASE, p=[0.6, 0, 0], r=R.identity(), stamp=0.0,
              max_age_s=1.0),
        Frame("tray", "wagon", p=[0, 0, 0.3], r=R.identity()),
    ], now=9.0)
    with pytest.raises(FrameError) as caught:
        graph.pose_in_base("tray")
    assert caught.value.reason == FRAME_STALE


def test_a_cycle_is_reported_rather_than_hung():
    graph = FrameGraph.of([
        Frame("a", "b", p=[0, 0, 0], r=R.identity()),
        Frame("b", "a", p=[0, 0, 0], r=R.identity()),
    ])
    with pytest.raises(FrameError) as caught:
        graph.pose_in_base("a")
    assert caught.value.reason == UNKNOWN_FRAME
    assert "cycle" in caught.value.detail


def test_an_object_carries_its_frame_and_refuses_through_it():
    graph = FrameGraph.of([Frame("table", BASE, p=[0.5, 0, 0], r=R.identity(),
                                 stamp=0.0, max_age_s=1.0)], now=30.0)
    block = ObjectView("block", p=[0.1, 0, 0], size=(0.05, 0.05, 0.05),
                       frame_id="table")
    with pytest.raises(FrameError):
        block.pose_in_base(graph)


def test_the_root_may_not_be_registered():
    with pytest.raises(ValueError):
        Frame(BASE, "", p=[0, 0, 0], r=R.identity())
