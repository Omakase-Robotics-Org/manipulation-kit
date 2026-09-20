"""FrameGraph.to_base must copy its input: producers freeze their arrays with
``setflags(write=False)`` and scipy >= 1.17 refuses read-only input to
``Rotation.apply`` (reproduced in the d1-isaaclab harness venv, 2026-09-19)."""
import numpy as np
from scipy.spatial.transform import Rotation as R

from manipulation_kit.world.frames import BASE, Frame, FrameGraph


def _graph():
    return FrameGraph().add(
        Frame(frame_id="table", parent=BASE, p=np.array([0.30, 0.20, 0.0]), r=R.from_euler("z", 0.3))
    )


def test_to_base_accepts_a_frozen_read_only_point():
    p = np.array([0.08, 0.05, 0.05])
    p.setflags(write=False)
    out = _graph().to_base(p, frame_id="table")
    assert out.flags.writeable
    expected = np.array([0.30, 0.20, 0.0]) + R.from_euler("z", 0.3).apply([0.08, 0.05, 0.05])
    assert np.allclose(out, expected)


def test_to_base_never_returns_the_callers_buffer():
    p = np.array([0.1, 0.2, 0.3])
    out = _graph().to_base(p, frame_id=BASE)
    assert out is not p
    out[0] = 9.0
    assert p[0] == 0.1
