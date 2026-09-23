"""Minimal 3D math + primitive-distance kernels for the D1 motion guard.

Pure standard library (no numpy) so it runs on the robot PC unchanged.
The segment-segment and segment-AABB kernels mirror
include/omakase_arm/collision_model.h byte-for-byte in behaviour so the
Python guard and the C++ validator agree (cross-checked in
tests/test_cpp_crosscheck.py).
"""
from __future__ import annotations

import math

Vec3 = tuple  # (x, y, z)

IDENTITY_R = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


class Tf:
    """Rigid transform: 3x3 rotation (row tuples) + translation."""

    __slots__ = ("R", "t")

    def __init__(self, R=IDENTITY_R, t=(0.0, 0.0, 0.0)):
        self.R = R
        self.t = t

    def mul(self, o: "Tf") -> "Tf":
        a, b = self.R, o.R
        R = tuple(
            tuple(a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
                  for j in range(3))
            for i in range(3))
        t = self.apply(o.t)
        return Tf(R, t)

    def apply(self, p) -> Vec3:
        R, t = self.R, self.t
        return (R[0][0] * p[0] + R[0][1] * p[1] + R[0][2] * p[2] + t[0],
                R[1][0] * p[0] + R[1][1] * p[1] + R[1][2] * p[2] + t[1],
                R[2][0] * p[0] + R[2][1] * p[1] + R[2][2] * p[2] + t[2])


def from_rpy_xyz(r, p, y, tx, ty, tz) -> Tf:
    """URDF fixed-axis rpy (R = Rz(y)*Ry(p)*Rx(r)) + translation."""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    R = ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
         (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
         (-sp, cp * sr, cp * cr))
    return Tf(R, (tx, ty, tz))


def rot_axis(axis, a) -> Tf:
    """Rotation of angle a (rad) about a unit axis (Rodrigues)."""
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return Tf()
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(a), math.sin(a)
    C = 1.0 - c
    R = ((c + x * x * C, x * y * C - z * s, x * z * C + y * s),
         (y * x * C + z * s, c + y * y * C, y * z * C - x * s),
         (z * x * C - y * s, z * y * C + x * s, c + z * z * C))
    return Tf(R)


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def norm(a):
    return math.sqrt(dot(a, a))


def _clamp01(v):
    return 0.0 if v < 0.0 else (1.0 if v > 1.0 else v)


def seg_seg_distance(p1, q1, p2, q2) -> float:
    """Min distance between segments [p1,q1] and [p2,q2]
    (same algorithm as collision_model.h::segSegDistance)."""
    d1, d2, r = sub(q1, p1), sub(q2, p2), sub(p1, p2)
    a, e, f = dot(d1, d1), dot(d2, d2), dot(d2, r)
    eps = 1e-12
    if a <= eps and e <= eps:
        return norm(sub(p1, p2))
    if a <= eps:
        s, t = 0.0, _clamp01(f / e)
    else:
        c = dot(d1, r)
        if e <= eps:
            t, s = 0.0, _clamp01(-c / a)
        else:
            b = dot(d1, d2)
            den = a * e - b * b
            s = _clamp01((b * f - c * e) / den) if den > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, _clamp01(-c / a)
            elif t > 1.0:
                t, s = 1.0, _clamp01((b - c) / a)
    return norm(sub(add(p1, scale(d1, s)), add(p2, scale(d2, t))))


def point_aabb_distance(p, lo, hi) -> float:
    dx = max(lo[0] - p[0], 0.0, p[0] - hi[0])
    dy = max(lo[1] - p[1], 0.0, p[1] - hi[1])
    dz = max(lo[2] - p[2], 0.0, p[2] - hi[2])
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def seg_aabb_distance(a, b, lo, hi) -> float:
    """Exact minimum distance from segment [a,b] to an AABB [lo,hi].

    The squared distance from a point moving along the segment to a box is a
    convex, piecewise-quadratic function of the segment parameter s, with its
    pieces separated by the at most six values of s where one coordinate
    crosses a face plane. On each piece every axis is either inside its slab
    (contributing nothing) or outside on a known side (contributing a linear
    term squared), so the minimum on the piece is a clamped closed form. The
    result is the global minimum up to floating-point rounding.

    (This replaced 25-point sampling, which could read up to ~2.4 mm too far
    on a 0.26 m link, i.e. on the unsafe side.)
    """
    d = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    cuts = [0.0, 1.0]
    for k in range(3):
        if d[k] != 0.0:
            for w in (lo[k], hi[k]):
                s = (w - a[k]) / d[k]
                if 0.0 < s < 1.0:
                    cuts.append(s)
    cuts.sort()

    def sq(s):
        total = 0.0
        for k in range(3):
            p = a[k] + s * d[k]
            e = lo[k] - p if p < lo[k] else (p - hi[k] if p > hi[k] else 0.0)
            total += e * e
        return total

    best = sq(0.0)
    for s0, s1 in zip(cuts, cuts[1:]):
        if s1 <= s0:
            continue
        mid = 0.5 * (s0 + s1)
        # f(s) = sum_k (c_k + s d_k)^2 over the axes outside their slab.
        num = 0.0
        den = 0.0
        for k in range(3):
            p = a[k] + mid * d[k]
            if p < lo[k]:
                c, dk = a[k] - lo[k], d[k]      # e = lo - p = -(c + s dk)
            elif p > hi[k]:
                c, dk = a[k] - hi[k], d[k]      # e = p - hi = c + s dk
            else:
                continue
            num += c * dk
            den += dk * dk
        s = s0 if den == 0.0 else min(max(-num / den, s0), s1)
        best = min(best, sq(s), sq(s1))
    return math.sqrt(best)
