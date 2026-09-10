import math

import manipulation_kit.guard.geometry as g


def test_seg_seg_parallel():
    assert g.seg_seg_distance((0, 0, 0), (1, 0, 0),
                              (0, 1, 0), (1, 1, 0)) == 1.0


def test_seg_seg_crossing():
    # skew segments crossing at right angles, closest at midpoints, gap 2
    d = g.seg_seg_distance((-1, 0, 0), (1, 0, 0), (0, -1, 2), (0, 1, 2))
    assert abs(d - 2.0) < 1e-12


def test_seg_seg_degenerate_points():
    assert abs(g.seg_seg_distance((0, 0, 0), (0, 0, 0),
                                  (3, 4, 0), (3, 4, 0)) - 5.0) < 1e-12


def test_point_aabb():
    lo, hi = (-1, -1, -1), (1, 1, 1)
    assert g.point_aabb_distance((0, 0, 0), lo, hi) == 0.0
    assert abs(g.point_aabb_distance((3, 0, 0), lo, hi) - 2.0) < 1e-12
    assert abs(g.point_aabb_distance((2, 2, 1), lo, hi) - math.sqrt(2)) < 1e-12


def test_seg_aabb():
    lo, hi = (-1, -1, -1), (1, 1, 1)
    # segment passing straight over the box at z = 3
    assert abs(g.seg_aabb_distance((-5, 0, 3), (5, 0, 3), lo, hi) - 2.0) < 1e-9
    # segment ending inside the box
    assert g.seg_aabb_distance((0, 0, 0), (5, 0, 0), lo, hi) == 0.0


def test_rpy_composition_matches_urdf_convention():
    # URDF fixed-axis rpy: R = Rz(y) * Ry(p) * Rx(r); check a known case
    tf = g.from_rpy_xyz(math.pi / 2, 0, 0, 0, 0, 0)   # roll +90
    assert all(abs(a - b) < 1e-12
               for a, b in zip(tf.apply((0, 0, 1)), (0, -1, 0)))
    tf = g.from_rpy_xyz(0, math.pi / 2, 0, 0, 0, 0)   # pitch +90
    assert all(abs(a - b) < 1e-12
               for a, b in zip(tf.apply((0, 0, 1)), (1, 0, 0)))


def test_rot_axis_negative_z():
    tf = g.rot_axis((0, 0, -1), math.pi / 2)
    # -z rotation by +90 == +z rotation by -90: x -> -y
    assert all(abs(a - b) < 1e-12
               for a, b in zip(tf.apply((1, 0, 0)), (0, -1, 0)))
