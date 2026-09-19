"""Orientation is DERIVED here, and nowhere else. The model never emits one.

The argument, short: rotation is the dimension where every emitter measurably
fails. pi05 end-effector policies fit rotation ~3.3x worse than translation on
a clean held-out split (0.120 vs 0.037 rad/m); an LLM steering roll/pitch/yaw
deltas has to maintain an integrator across turns that it cannot see; and the
per-arm convention is mirrored, so the same words mean opposite turns on the
two hands (using the LEFT hand's top-down quaternion on the RIGHT wrist put
the wrist camera against the torso, twin 2026-09-08). Translation has none of
those problems, which is why :class:`~manipulation_kit.primitives.motion.Nudge`
still carries dx/dy/dz.

So the vocabulary is four named approaches, and the one remaining degree of
freedom — the roll about the approach axis, i.e. which way the jaws close — is
taken from the object's own principal axis. A model names ``top_down`` and a
block; the kit works out that the jaws must close across the 40 mm side.

Frames, once. ``Link7`` is the IK's end-effector body. The URDF's fixed
``JointTCP_*`` puts the TCP frame on it (translation ``(0, -0.087, 0)`` in
Link7, rpy ``(1.5708, -1.5708, 0)``), the gripper's ``base_link`` IS that TCP
frame, its +z is the approach axis and its x is the jaw-gap axis. The tool
POINT is the pad centre, 100 mm along +z — MEASURED on d1-3 2026-09-16, which
is also why it is 100 mm here and 108.5 mm in older CAD-derived code.
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from ..hands.d1.parallel_gripper.description import (DRIVEN_OPEN_GAP_M,
                                                     PAD_CENTRE_Z_M)
from ..world import FrameGraph, ObjectView
from .types import APPROACHES, FRONT, SIDE_LEFT, SIDE_RIGHT, TOP_DOWN

#: Link7 -> TCP, read off ``description/d1/d1.urdf``'s ``JointTCP_{R,L}``.
#: Identical on both arms; the per-arm mirroring is in the POSE, not here.
TCP_P = np.array([0.0, -0.087, 0.0])
TCP_R_EULER_XYZ: Tuple[float, float, float] = (1.5708, -1.5708, 0.0)
TCP_R = R.from_euler("xyz", TCP_R_EULER_XYZ)

#: Jaw pocket along the TCP frame's +z from the flange [m]. The grasp happens
#: HERE, not at the pad tips (129 mm, the registered TCP) and not at the flange.
TOOL_Z_M = PAD_CENTRE_Z_M

#: Largest gap the DRIVEN gripper reaches (51.96 mm measured on d1-3, against
#: the 64 mm the mechanism could reach at a higher OPEN_RAD). Planning uses the
#: driven number: an object sized against the mechanism's travel does not fit
#: the gripper that is actually on the robot.
JAW_OPEN_M = DRIVEN_OPEN_GAP_M
#: clearance per side that a graspable object must leave inside that opening
JAW_CLEARANCE_M = 0.004

#: Top-down grasp orientation of the TCP frame, (w, x, y, z), per LOGICAL side.
#: LEFT: z_tcp -> world -Z (pads down), x_tcp -> world -Y, i.e. the jaw gap
#: runs along world y and the wrist camera points away from the torso. RIGHT is
#: the mirror image, yawed 180 deg about world Z. Verified on the twin; using
#: the left quaternion on both arms is the 2026-09-08 wrist-camera bug.
PADS_DOWN_WXYZ_BY_SIDE: Dict[str, Tuple[float, float, float, float]] = {
    "left": (0.0, -0.7071068, 0.7071068, 0.0),
    "right": (0.0, -0.7071068, -0.7071068, 0.0),
}

#: Unit vector the TOOL TRAVELS ALONG as it closes on the object, base frame.
#: ``top_down`` descends (-z); ``front`` comes in from the robot's side of the
#: object and pushes forward (+x); ``side_left`` comes in from the robot's LEFT
#: and travels toward -y. The tool's approach axis (TCP +z) is aligned with it,
#: so the standoff pose sits at ``grasp_point - direction * standoff``.
APPROACH_DIRECTION: Dict[str, np.ndarray] = {
    TOP_DOWN: np.array([0.0, 0.0, -1.0]),
    FRONT: np.array([1.0, 0.0, 0.0]),
    SIDE_LEFT: np.array([0.0, -1.0, 0.0]),
    SIDE_RIGHT: np.array([0.0, 1.0, 0.0]),
}

APPROACH_DOC: Dict[str, str] = {
    TOP_DOWN: "descend onto the object from above; jaws close horizontally",
    FRONT: "come in horizontally from the robot's side and close facing forward",
    SIDE_LEFT: "come in horizontally from the robot's LEFT",
    SIDE_RIGHT: "come in horizontally from the robot's RIGHT",
}


def check_approach(approach: str) -> str:
    if approach not in APPROACHES:
        raise ValueError(f"approach must be one of {APPROACHES}, got {approach!r}")
    return approach


def direction(approach: str) -> np.ndarray:
    return APPROACH_DIRECTION[check_approach(approach)].copy()


def fits_jaws(obj: ObjectView) -> bool:
    """Can the driven gripper close on this at all, with clearance per side?"""
    return obj.min_horizontal_extent() <= JAW_OPEN_M - 2 * JAW_CLEARANCE_M


def tcp_from_tool(p_tool, r_tcp: R) -> np.ndarray:
    """The TCP-frame ORIGIN that puts the tool point at ``p_tool``."""
    return np.asarray(p_tool, dtype=float) - r_tcp.apply([0.0, 0.0, TOOL_Z_M])


def link7_from_tool(p_tool, r_tcp: R) -> Tuple[np.ndarray, R]:
    """``(p, r)`` of Link7 that puts the TOOL POINT at ``p_tool`` with TCP ``r_tcp``.

    The inverse of the chain the URDF states, done once so no consumer has to
    re-derive it (and get the sign of the 87 mm wrong).
    """
    r7 = r_tcp * TCP_R.inv()
    p7 = tcp_from_tool(p_tool, r_tcp) - r7.apply(TCP_P)
    return p7, r7


def tool_from_link7(p7, r7: R) -> Tuple[np.ndarray, R]:
    """The forward direction: Link7 pose in, tool point + TCP orientation out."""
    r_tcp = r7 * TCP_R
    p_tcp = np.asarray(p7, dtype=float) + r7.apply(TCP_P)
    return p_tcp + r_tcp.apply([0.0, 0.0, TOOL_Z_M]), r_tcp


def _seed(side: str, d: np.ndarray) -> R:
    """A TCP orientation whose +z is ``d``, before the principal-axis roll.

    Vertical approaches start from the measured per-side PADS_DOWN quaternion,
    so the mirror convention enters exactly once. Horizontal approaches build
    a frame with the jaw gap horizontal, which is the grasp a person makes when
    reaching for something on a table.
    """
    if abs(float(d[2])) > 0.9:
        w, x, y, z = PADS_DOWN_WXYZ_BY_SIDE[side]
        r = R.from_quat([x, y, z, w])
        if float(d[2]) > 0:           # "pads up" is not in the approach set
            raise ValueError("no upward approach is defined")
        return r
    up = np.array([0.0, 0.0, 1.0])
    x_axis = np.cross(up, d)
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = np.cross(d, x_axis)
    return R.from_matrix(np.column_stack([x_axis, y_axis, d]))


def _roll_to(seed: R, d: np.ndarray, gap_axis: np.ndarray) -> R:
    """Roll ``seed`` about ``d`` until its x (the jaw gap) lies along ``gap_axis``.

    A parallel gripper is symmetric under a half turn, so the representative
    nearest the seed is chosen — the wrist travels at most 90 deg, never 180.
    """
    x_now = seed.as_matrix()[:, 0]
    y_now = seed.as_matrix()[:, 1]
    angle = math.atan2(float(np.dot(gap_axis, y_now)), float(np.dot(gap_axis, x_now)))
    while angle > math.pi / 2:
        angle -= math.pi
    while angle < -math.pi / 2:
        angle += math.pi
    return R.from_rotvec(d * angle) * seed


def grasp_orientation(side: str, approach: str, obj: Optional[ObjectView] = None,
                      frames: Optional[FrameGraph] = None, *,
                      dyaw_rad: float = 0.0) -> R:
    """The TCP orientation for ``approach`` on ``obj``, in the base frame.

    This is the entire orientation surface of this package. ``dyaw_rad`` is the
    bounded correction ``Nudge`` may carry — a roll about the approach axis,
    the one rotation with an obvious visual meaning ("turn the hand a little").

    With no object the approach set's own default jaw orientation is kept.
    With one, the jaws are squared to the object's own faces — its long axis
    where it has one, its widest horizontal axis where it does not
    (``ObjectView.footprint_axis``). A square footprint has no PREFERRED grasp
    and it still has a wrong one: a 40 mm cube yawed 11.7 deg presents 47.3 mm
    across base-aligned jaws, more than the 43.96 mm the driven gripper can
    take, so the pads meet two corners and hold nothing (measured over five
    blocks-eval trials, 2026-09-19).
    """
    d = direction(approach)
    r = _seed(side, d)
    axis = None if obj is None or frames is None else obj.footprint_axis(frames)
    if axis is not None:
        # The jaws must close ACROSS the object's long axis: the gap direction
        # is perpendicular to both the approach and that axis.
        perp = axis - d * float(np.dot(axis, d))
        if np.linalg.norm(perp) > 1e-6:
            gap = np.cross(d, perp / np.linalg.norm(perp))
            if np.linalg.norm(gap) > 1e-6:
                r = _roll_to(r, d, gap / np.linalg.norm(gap))
    if dyaw_rad:
        r = R.from_rotvec(d * float(dyaw_rad)) * r
    return r


def standoff_pose(grasp_p, approach: str, standoff_m: float) -> np.ndarray:
    """Where the tool point waits before travelling along the approach axis."""
    return np.asarray(grasp_p, dtype=float) - direction(approach) * float(standoff_m)


def choose_side(obj_p_base, *, available=("left", "right")) -> str:
    """The near hand: +y is the robot's left. Ties go to the right.

    Deliberately trivial. The offer filter re-runs IK and the guard on whatever
    this picks, so a wrong guess costs one refusal and a fallback, not a
    collision.
    """
    y = float(np.asarray(obj_p_base, dtype=float).reshape(3)[1])
    preferred = "left" if y > 0.0 else "right"
    if preferred in available:
        return preferred
    return available[0]
