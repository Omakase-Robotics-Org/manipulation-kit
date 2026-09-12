"""Run the arm-on-D1 tests on BOTH kinematic substrates.

These files used to ``importorskip("mujoco")`` at module scope, because the
only :class:`~manipulation_kit.arms.ik.KinematicChain` there was the MuJoCo one
— so on a machine without a physics engine the entire inverse-kinematics suite
skipped, and a green run proved nothing about the code the robot runs.

The default substrate is now
:class:`~manipulation_kit.arms.urdf_chain.UrdfChain` (numpy), so the same tests
run everywhere; ``"mujoco"`` is an extra pass that skips when the optional
extra is not installed. Anything genuinely MuJoCo-specific belongs in
``test_urdf_chain_parity.py``, not here.
"""

from __future__ import annotations

import pytest


@pytest.fixture(scope="module", params=["urdf", "mujoco"])
def substrate(request):
    """``"urdf"`` (always) and ``"mujoco"`` (when installed)."""
    if request.param == "mujoco":
        pytest.importorskip("mujoco")
    return request.param
