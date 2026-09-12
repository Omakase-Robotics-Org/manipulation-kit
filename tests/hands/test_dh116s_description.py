"""Description loader tests — right (vendor) model and the mirrored left.

mujoco is NOT a dx-manipulator dependency, so everything here skips cleanly
when it is absent (pip install 'mujoco>=3.10' to run)."""

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from manipulation_kit.hands.leadshine.dh116s.description import load_mjspec  # noqa: E402


def test_right_loads_and_compiles():
    spec = load_mjspec(mujoco, side="right")
    model = spec.compile()
    assert model.nbody > 1
    # the vendor model's 6 active + 5 coupled passive joints
    assert model.njnt == 11
    assert model.neq == 5  # the passive-coupling equality constraints


def test_lazy_mujoco_import():
    # mujoco=None → imported lazily inside (the no-dependency seam)
    spec = load_mjspec(side="right")
    assert spec.compile().nbody > 1


def test_left_compiles_same_topology():
    right = load_mjspec(mujoco, side="right").compile()
    left = load_mjspec(mujoco, side="left").compile()
    assert left.nbody == right.nbody
    assert left.njnt == right.njnt
    assert left.neq == right.neq


def test_left_mirrors_body_positions():
    """Sagittal mirror: every body's LOCAL pos.x is negated; at least one
    base_link child must actually sit off the x=0 plane for this to bite."""
    right = load_mjspec(mujoco, side="right").compile()
    left = load_mjspec(mujoco, side="left").compile()
    flipped = 0
    for i in range(right.nbody):
        rb, lb = right.body(i), left.body(i)
        assert lb.name == rb.name
        assert np.allclose(lb.pos[0], -rb.pos[0], atol=1e-9)
        assert np.allclose(lb.pos[1:], rb.pos[1:], atol=1e-9)
        if abs(rb.pos[0]) > 1e-6:
            flipped += 1
            assert np.sign(lb.pos[0]) == -np.sign(rb.pos[0])
    assert flipped > 0, "no body off the sagittal plane — mirror untested"


def test_left_mirrors_joint_axes():
    right = load_mjspec(mujoco, side="right").compile()
    left = load_mjspec(mujoco, side="left").compile()
    for i in range(right.njnt):
        ra, la = right.jnt_axis[i], left.jnt_axis[i]
        assert np.allclose(la, [ra[0], -ra[1], -ra[2]], atol=1e-9)


def test_left_mesh_scale_negative_x():
    spec = load_mjspec(mujoco, side="left")
    assert len(spec.meshes) > 0
    for m in spec.meshes:
        assert m.scale[0] < 0, f"mesh {m.name}: scale.x not mirrored"
        assert m.scale[1] > 0 and m.scale[2] > 0
    # and the right hand is untouched
    right = load_mjspec(mujoco, side="right")  # NB hold the spec — MjsMesh
    for m in right.meshes:                     # views into a freed spec read
        assert m.scale[0] > 0                  # garbage memory


def test_mesh_paths_absolute():
    from pathlib import Path
    for side in ("right", "left"):
        spec = load_mjspec(mujoco, side=side)
        for m in spec.meshes:
            assert Path(m.file).is_absolute()


def test_bad_side_rejected():
    with pytest.raises(ValueError):
        load_mjspec(mujoco, side="both")
