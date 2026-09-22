"""Orientation is DERIVED here, and nowhere else. The model never emits one.

The argument, short: rotation is the dimension where every emitter measurably
fails. pi05 end-effector policies fit rotation ~3.3x worse than translation on
a clean held-out split (0.120 vs 0.037 rad/m); an LLM steering roll/pitch/yaw
deltas has to maintain an integrator across turns that it cannot see; and the
per-arm convention is mirrored, so the same words mean opposite turns on the
two hands (using the LEFT hand's top-down quaternion on the RIGHT wrist put
the wrist camera against the torso, twin 2026-09-08). Translation has none of
those problems, which is why :class:`~manipulation_kit.primitives.verbs.Nudge`
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
import os
from typing import Dict, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from ..hands.d1.parallel_gripper.description import (DRIVEN_OPEN_GAP_M,
                                                     PAD_CENTRE_Z_M,
                                                     PAD_TIP_Z_M)
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

#: How far the pad TIP reaches past the tool point, 29 mm. The tool point is
#: the pad CENTRE; the fingers keep going.
TIP_BELOW_TOOL_M = PAD_TIP_Z_M - PAD_CENTRE_Z_M
#: ...and how much daylight the tips must keep over whatever the object is
#: standing on. MEASURED, 2026-09-19: a top-down grasp that put the tool point
#: on a 40 mm cube's CENTRE asked for the pad tips 9 mm BELOW the wagon top.
#: The fingers jammed on the table, the arm stopped 17 mm high and 19 mm off
#: to the side (still 2.5 deg from the commanded posture after two seconds of
#: holding it, while the same arm tracks a free-air posture to 0.00 deg in
#: 0.7 s), and the jaws closed beside the block. Ten attempts, ten failures.
SUPPORT_CLEARANCE_M = float(os.environ.get("MKIT_SUPPORT_CLEARANCE_M", "0.003"))
# ^ Operator override. 3 mm is right for a rigid arm; the real D1 arm sags
# ~1 cm at a long reach (F16 droop, no along-axis compensation in ToolGate
# yet), so on d1-2 (2026-09-22 run7, x 0.48) the pad tips met the table and
# the controller raised error 15 during the descent. Set e.g. 0.015 on the
# robot until the gate compensates droop.

#: ...and the least the SOLVED descent may actually keep, as opposed to what
#: the waypoint asked for. The IK converges to about 2 mm and the path window
#: is 12 mm, so a plan whose ideal grasp point clears by 3 mm can land
#: anywhere in that band; this is the number the achieved pose is checked
#: against (``Grasp.plan``). 1 mm rather than 3: at 3 the check would fail on
#: the solver's own convergence noise — measured, a blocks-eval cube solves
#: 0.2 mm below its ideal grasp point — and at 0 it would allow the tips into
#: the surface.
MIN_ACHIEVED_CLEARANCE_M = 0.001

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


def tool_revision() -> str:
    """A fingerprint of the TOOL GEOMETRY every waypoint is expressed against.

    The tool point is the pad centre and the jaw gap is the driven opening;
    swap the gripper (or re-measure it) and every waypoint in every plan means
    a different place. A plan records this and the executor refuses to run one
    whose tool no longer matches (R8) — cheap, and the only alternative is
    trusting that nobody changed the hand between planning and moving.

    A pure function of the hand DESCRIPTION: no environment variable feeds it
    (the ``MKIT_DRIVEN_OPEN_GAP_M`` knob that did is gone). What a particular
    robot's hand opens to is a measurement published by its executor
    (``HandState.open_gap_m``), and which firmware produced that measurement
    is bound separately (``PlanBinding.firmware_spec``).
    """
    return (f"pad_centre={PAD_CENTRE_Z_M:.4f};pad_tip={PAD_TIP_Z_M:.4f};"
            f"driven_open={DRIVEN_OPEN_GAP_M:.5f}")


def check_approach(approach: str) -> str:
    if approach not in APPROACHES:
        raise ValueError(f"approach must be one of {APPROACHES}, got {approach!r}")
    return approach


def direction(approach: str) -> np.ndarray:
    return APPROACH_DIRECTION[check_approach(approach)].copy()


#: The widest object the driven jaws can take, clearance included.
GRASPABLE_WIDTH_M = JAW_OPEN_M - 2 * JAW_CLEARANCE_M


def graspable_width_m(open_gap_m: Optional[float] = None) -> float:
    """What the jaws can close on, for a hand that opens to ``open_gap_m``.

    ``None`` is the hand description's nominal driven opening
    (:data:`GRASPABLE_WIDTH_M`); a measured gap — the executor's
    ``HandState.open_gap_m``, carried on ``GripperView.open_gap_m`` — replaces
    it. The clearance per side is the same either way.
    """
    if open_gap_m is None:
        return GRASPABLE_WIDTH_M
    return float(open_gap_m) - 2 * JAW_CLEARANCE_M


def jaw_axis(r_tcp: R) -> np.ndarray:
    """The direction the pads travel along, base frame: the TCP frame's x.

    The jaw GAP is measured along this axis, so it is the axis every fit
    question has to be asked about — not the object's smallest extent, and
    not a base axis.
    """
    return np.asarray(r_tcp.as_matrix()[:, 0], dtype=float)


def grasp_width(obj: ObjectView, frames: FrameGraph, r_tcp: R) -> float:
    """What the object PRESENTS to these jaws: its extent along the jaw axis.

    ``sum(|jaw . body_axis_i| * size_i)`` over the RESOLVED orientation. A
    100x80x40 mm box passed the old ``min(size)`` test on its 40 mm extent
    while the derived top-down grasp closed across 80 mm of it (R9); this is
    the number that says so.
    """
    return obj.extent_along(jaw_axis(r_tcp), frames)


def fits_jaws(obj: ObjectView, frames: FrameGraph, r_tcp: R) -> bool:
    """Can the driven gripper close on this, ALONG THE JAW AXIS it will use?"""
    return grasp_width(obj, frames, r_tcp) <= GRASPABLE_WIDTH_M


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


def lowest_top_down_tool_z(obj: ObjectView, frames: FrameGraph) -> float:
    """The lowest tool-point z a top-down grasp of ``obj`` may command.

    A parallel gripper cannot put its finger tips through the table. The
    object's own underside IS the table here — it is standing on it — so the
    constraint needs no surface lookup: tips at ``bottom + SUPPORT_CLEARANCE``,
    tool point ``TIP_BELOW_TOOL_M`` above that.

    RESOLVED, both halves. The underside comes from the base-frame pose and
    the vertical extent from the base-frame orientation: an object measured in
    a table frame that sits 30 mm above the base has its underside 30 mm
    higher, and a yawed box is not taller (R1/R9).
    """
    return obj.bottom_z(frames) + TIP_BELOW_TOOL_M + SUPPORT_CLEARANCE_M


def grasp_point(obj: ObjectView, approach: str, frames: FrameGraph
                ) -> Tuple[np.ndarray, bool]:
    """Where the TOOL POINT goes to grasp ``obj``, and whether it was raised.

    The object's RESOLVED base-frame centre, except for ``top_down``, where
    the finger tips would otherwise be driven into whatever the object is
    standing on: there the point is lifted to :func:`lowest_top_down_tool_z`.
    The pads are 58 mm deep, so a 40 mm cube grasped 12 mm above its centre
    still has 37 mm of pad against its side — the grasp does not get worse, it
    gets possible.

    ``frames`` is not optional: reading ``obj.p`` here produced a base-frame
    command from a table-frame coordinate — a probe with a 0.30/0.20 m table
    offset asked the IK for (0.08, 0.05, 0.057) for an object at
    (0.38, 0.25, 0.05) (R1).
    """
    p = np.asarray(obj.pose_in_base(frames)[0], dtype=float).reshape(3).copy()
    if check_approach(approach) != TOP_DOWN:
        return p, False
    floor = lowest_top_down_tool_z(obj, frames)
    if floor <= p[2]:
        return p, False
    p[2] = floor
    return p, True


def achieved_clearance(obj: ObjectView, frames: FrameGraph, p_tool) -> float:
    """How far the pad TIPS end up above what ``obj`` stands on, for a solved
    top-down tool point. Negative means the fingers are in the surface."""
    tips = float(np.asarray(p_tool, dtype=float)[2]) - TIP_BELOW_TOOL_M
    return tips - obj.bottom_z(frames)


def grasps_above_its_top(obj: ObjectView, frames: FrameGraph) -> bool:
    """Is this object too FLAT for the fingers to reach beside it at all?

    When the lowest legal tool point is above the object's top face, the pads
    would close over thin air with the tips still on the table. A refusal is
    the honest answer; a grasp that cannot touch the object is not.
    """
    return lowest_top_down_tool_z(obj, frames) > obj.top_face_z(frames)


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
