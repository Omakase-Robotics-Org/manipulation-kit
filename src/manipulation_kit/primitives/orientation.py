"""Orientation is DERIVED here, and nowhere else. The model never emits one.

THE ONLY PLACE A QUATERNION IS PRODUCED: :func:`align_tool`. Every verb hands
it a base-frame direction (a resolved :class:`~manipulation_kit.world.Direction`)
and, where it matters, the axis the jaws should close across; it returns the TCP
orientation. (This module was ``primitives/approach.py`` before 0.16.0, when
the direction was a four-word enum whose geometry lived in a table here.)

The argument, short: rotation is the dimension where every emitter measurably
fails. pi05 end-effector policies fit rotation ~3.3x worse than translation on
a clean held-out split (0.120 vs 0.037 rad/m); an LLM steering roll/pitch/yaw
deltas has to maintain an integrator across turns that it cannot see; and the
per-arm convention is mirrored, so the same words mean opposite turns on the
two hands (using the LEFT hand's top-down quaternion on the RIGHT wrist put
the wrist camera against the torso, twin 2026-09-08). Translation has none of
those problems, which is why :class:`~manipulation_kit.primitives.verbs.Nudge`
still carries dx/dy/dz.

So the vocabulary is a DIRECTION (``down``, ``forward``, or any vector in a
named frame), and the one remaining degree of freedom — the roll about the
approach axis, i.e. which way the jaws close — is taken from the object's own
principal axis. A model names ``down`` and a block; the kit works out that the
jaws must close across the 40 mm side. A further roll (``roll_rad``) is a
PLANNER choice and is never offered to a model.

Frames, once. ``Link7`` is the IK's end-effector body. The URDF's fixed
``JointTCP_*`` puts the TCP frame on it (translation ``(0, -0.087, 0)`` in
Link7, rpy ``(1.5708, -1.5708, 0)``), the gripper's ``base_link`` IS that TCP
frame, its +z is the approach axis and its x is the jaw-gap axis. The tool
POINT is the pad centre, 100 mm along +z — MEASURED on d1-3 2026-09-16, which
is also why it is 100 mm here and 108.5 mm in older CAD-derived code.

WHERE on the hand a grasp makes contact (the pad centre or the fingertips),
how far the fingers reach past that point, and everything that follows from
it — the grasp point, the descent floor, the standoff, the fit test and the
one roll sweep — is :mod:`.grasp_geometry`. Every waypoint is still expressed
as the pose of the pad centre (:data:`TOOL_Z_M`); a fingertip grasp is
converted onto it there, once.
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from ..hands.d1.parallel_gripper.description import (DRIVEN_OPEN_GAP_M,
                                                     PAD_CENTRE_Z_M,
                                                     PAD_CLEARANCE_PER_SIDE_M,
                                                     PAD_TIP_Z_M)
from ..world import FrameGraph, ObjectView

#: Link7 -> TCP, read off ``description/d1/d1.urdf``'s ``JointTCP_{R,L}``.
#: Identical on both arms; the per-arm mirroring is in the POSE, not here.
TCP_P = np.array([0.0, -0.087, 0.0])
TCP_R_EULER_XYZ: Tuple[float, float, float] = (1.5708, -1.5708, 0.0)
TCP_R = R.from_euler("xyz", TCP_R_EULER_XYZ)

#: The TOOL POINT every waypoint is expressed against, along the TCP frame's
#: +z from the flange [m]: the pad centre. It is ``grasp_geometry.PAD``'s
#: ``offset_z_m``; a fingertip grasp (``grasp_geometry.TIP``, 129 mm) is
#: converted onto this point rather than moving it, so the planner, the
#: tool-arrival gate and the executors keep ONE tool point.
TOOL_Z_M = PAD_CENTRE_Z_M

#: clearance per side a grasped object leaves inside the opening at the PADS
#: (``grasp_geometry.PAD.clearance_per_side_m``); also the verifiers' grip
#: width tolerance
JAW_CLEARANCE_M = PAD_CLEARANCE_PER_SIDE_M

#: How much daylight the finger TIPS must keep over whatever the object is
#: standing on. MEASURED, 2026-09-19: a top-down grasp that put the tool point
#: on a 40 mm cube's CENTRE asked for the pad tips 9 mm BELOW the wagon top.
#: The fingers jammed on the table, the arm stopped 17 mm high and 19 mm off
#: to the side (still 2.5 deg from the commanded posture after two seconds of
#: holding it, while the same arm tracks a free-air posture to 0.00 deg in
#: 0.7 s), and the jaws closed beside the block. Ten attempts, ten failures.
SUPPORT_CLEARANCE_M = 0.003
# ^ For a RIGID arm. The real D1 arm sags ~1 cm at a long reach (F16 droop):
# on d1-2 (2026-09-22 run 7, x 0.48) the pad tips met the table at this floor
# and the controller raised error 15 during the descent. That used to be
# patched with an environment variable (``MKIT_SUPPORT_CLEARANCE_M``, deleted
# in 0.16.0); it is now the typed ``droop_margin_m`` of
# :class:`~manipulation_kit.primitives.clearance.ClearancePolicy`, which the
# operator policy sets and which reaches this floor as the ``droop_margin_m``
# argument of :func:`~manipulation_kit.primitives.grasp_geometry.grasp_pose`.

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

#: ``|d . z|`` above which a direction counts as VERTICAL: the tool is seeded
#: from the measured per-side PADS_DOWN quaternion rather than built from a
#: horizontal jaw gap, and a downward one is a DESCENT onto what the object
#: stands on (the support floor applies).
VERTICAL_COS = 0.9


def tool_revision(reference: Optional[str] = None) -> str:
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

    ``reference`` names WHERE on the hand the plan makes contact
    (``grasp_geometry.GraspReference.name``: ``"pad"`` or ``"tip"``). A grasp
    plan records it, so a fingertip plan — whose waypoints put the tips, not
    the pad centres, on the object — cannot be replayed where a pad plan is
    expected: :meth:`~.types.PlanBinding.drift` compares every field BOTH
    revisions state. An executor states only the hand (no reference), so it
    runs either kind of plan for the hand it drives.
    """
    base = (f"pad_centre={PAD_CENTRE_Z_M:.4f};pad_tip={PAD_TIP_Z_M:.4f};"
            f"driven_open={DRIVEN_OPEN_GAP_M:.5f}")
    return base if reference is None else f"{base};reference={reference}"


def _unit(d) -> np.ndarray:
    d = np.asarray(d, dtype=float).reshape(3)
    n = float(np.linalg.norm(d))
    if n < 1e-9:
        raise ValueError("a tool direction cannot be the zero vector")
    return d / n


def is_descent(d_base) -> bool:
    """Does the tool travel DOWN onto the object (so its tips meet the support)?"""
    return float(_unit(d_base)[2]) < -VERTICAL_COS


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

    Vertical directions start from the measured per-side PADS_DOWN quaternion,
    so the mirror convention enters exactly once; a direction a few degrees off
    vertical tilts that seed by the smallest rotation that puts its +z on
    ``d`` (none at all for exactly ``down``). Horizontal ones build a frame with
    the jaw gap horizontal, which is the grasp a person makes when reaching for
    something on a table.
    """
    if abs(float(d[2])) > VERTICAL_COS:
        if float(d[2]) > 0:           # pads UP: nothing on a D1 reaches that way
            raise ValueError("no upward tool direction is defined: the hand "
                             "cannot approach an object from underneath")
        w, x, y, z = PADS_DOWN_WXYZ_BY_SIDE[side]
        r = R.from_quat([x, y, z, w])
        z_now = r.as_matrix()[:, 2]
        axis = np.cross(z_now, d)
        s = float(np.linalg.norm(axis))
        if s > 1e-12:
            angle = math.atan2(s, float(np.dot(z_now, d)))
            r = R.from_rotvec(axis / s * angle) * r
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


def roll_about(r: R, axis, roll_rad: float) -> R:
    """``r`` turned by ``roll_rad`` about ``axis`` (base frame). The one roll."""
    if not roll_rad:
        return r
    return R.from_rotvec(np.asarray(axis, dtype=float) * float(roll_rad)) * r


def roll_tool(r_tcp: R, roll_rad: float) -> R:
    """Turn the hand about ITS OWN approach axis (TCP +z) — what a Nudge's
    ``dyaw`` asks for."""
    return roll_about(r_tcp, r_tcp.as_matrix()[:, 2], roll_rad)


def align_tool(side: str, d_base, *, roll_to=None, roll_rad: float = 0.0) -> R:
    """The TCP orientation whose +z is ``d_base``, in the base frame.

    Rolled about ``d_base`` so the jaw-gap axis (TCP +x) lies along ``roll_to``
    (its component perpendicular to ``d_base``; the nearest half-turn
    representative, so the wrist turns at most 90 deg), then by ``roll_rad``.
    The per-side mirrored PADS_DOWN seed is used when ``|d.z| > 0.9``,
    otherwise the horizontal frame. THE ONLY PLACE A QUATERNION IS PRODUCED.

    Raises ``ValueError`` for an upward direction; verbs refuse that before
    they get here.
    """
    d = _unit(d_base)
    r = _seed(side, d)
    if roll_to is not None:
        gap = np.asarray(roll_to, dtype=float).reshape(3)
        gap = gap - d * float(np.dot(gap, d))
        n = float(np.linalg.norm(gap))
        if n > 1e-6:
            r = _roll_to(r, d, gap / n)
    return roll_about(r, d, roll_rad)


def grasp_orientation(side: str, d_base, obj: Optional[ObjectView] = None,
                      frames: Optional[FrameGraph] = None, *,
                      roll_rad: float = 0.0) -> R:
    """The TCP orientation for a grasp travelling along ``d_base`` onto ``obj``.

    ``d_base`` is a RESOLVED base-frame vector (``Direction.resolve``); the
    rotation itself is :func:`align_tool`'s. ``roll_rad`` is the planner's
    extra turn about the approach axis (never a model argument).

    With no object the direction's own default jaw orientation is kept.
    With one, the jaws are squared to the object's own faces — its long axis
    where it has one, its widest horizontal axis where it does not
    (``ObjectView.footprint_axis``). A square footprint has no PREFERRED grasp
    and it still has a wrong one: a 40 mm cube yawed 11.7 deg presents 47.3 mm
    across base-aligned jaws, more than the 43.96 mm the driven gripper can
    take, so the pads meet two corners and hold nothing (measured over five
    blocks-eval trials, 2026-09-19).
    """
    d = _unit(d_base)
    axis = None if obj is None or frames is None else obj.footprint_axis(frames)
    gap = None
    if axis is not None:
        # The jaws must close ACROSS the object's long axis: the gap direction
        # is perpendicular to both the approach and that axis.
        perp = axis - d * float(np.dot(axis, d))
        if np.linalg.norm(perp) > 1e-6:
            cross = np.cross(d, perp / np.linalg.norm(perp))
            if np.linalg.norm(cross) > 1e-6:
                gap = cross / np.linalg.norm(cross)
    return align_tool(side, d, roll_to=gap, roll_rad=roll_rad)


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
