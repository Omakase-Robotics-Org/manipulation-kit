"""A URDF-walking :class:`~manipulation_kit.arms.ik.KinematicChain` — numpy only.

THE DEFAULT SUBSTRATE. The solver in :mod:`manipulation_kit.arms.ik` needs one
thing from a robot: pose it, then ask for the end-effector pose, the Jacobian
and the joint limits. That is Rodrigues FK plus a textbook geometric Jacobian
over the description this package already ships and already parses for the
collision guard — so it is done here, in numpy, and a physics engine is not a
dependency of inverse kinematics.

``mujoco`` remains available as an alternative substrate
(:class:`~manipulation_kit.arms.d1.arm.mujoco_chain.MujocoChain`) for consumers
that already carry a MuJoCo mirror of the robot — dx-vr-teleop has a viewer, a
sim backend and a composed hand model, so FK is free there and its parity gate
compares against it. The two agree to ~1e-15 on this URDF
(``tests/arms/test_urdf_chain_parity.py``), which is float noise, not a
different model: MuJoCo's URDF importer gives every link body the URDF link
frame, and ``mj_jacBody`` reports exactly the columns built below.

FRAMES AND CONVENTIONS (these must match MujocoChain exactly, or the solver
silently means something different depending on which substrate it got):

* poses are in the ROOT link frame of the URDF (D1: ``dual_base``), which is
  also MuJoCo's world frame for a URDF with a fixed base;
* :meth:`ee_pose` reports the frame of the ``ee_body`` LINK, with any
  ``tool_offset`` applied after it. MuJoCo fuses the fixed ``TCP_Link`` into
  ``Link7`` (``fusestatic``) without moving ``Link7``'s frame, so the default
  ``tool_offset=None`` is what MujocoChain reports and the constant flange
  offset is the consumer's to add (:mod:`manipulation_kit.hands`), exactly as
  :mod:`manipulation_kit.arms.sides` documents;
* :meth:`jacobian` is the 6 x ndof GEOMETRIC Jacobian of that same point, in
  root-frame coordinates, LINEAR ROWS ON TOP — ``mj_jacBody``'s ``jacp`` over
  ``jacr``. Column i of a revolute joint is ``(a_i x (p_ee - o_i), a_i)`` and
  of a prismatic joint ``(a_i, 0)``, with ``a_i``/``o_i`` the joint's axis and
  origin in the root frame at the current pose.

Not thread-safe, by the same rule as every chain: it is STATEFUL (``set_joints``
then query), and its owner serialises access —
:class:`~manipulation_kit.arms.kinematics.GuardedArm` holds one re-entrant lock.
Unlike MuJoCo's, two instances here share nothing, so the two arms of one robot
are genuinely independent objects.
"""

from __future__ import annotations

import os
from typing import Dict, List, Optional, Sequence, Tuple, Union

import numpy as np
from scipy.spatial.transform import Rotation as R

# ``..guard`` is manipulation_kit.guard (the stdlib-only URDF parser and
# collision guard), NOT manipulation_kit.arms.guard (the gate seam next door).
from ..guard.urdf_model import UrdfModel

#: URDF joint types this chain can actuate. Everything else on the path
#: (``fixed``) contributes a constant transform; anything else is refused
#: loudly rather than silently treated as rigid.
ACTUATED = ("revolute", "continuous", "prismatic")


def _parse(model: Union[UrdfModel, str, "os.PathLike"]) -> UrdfModel:
    """Accept an already-parsed model, or a path to parse once."""
    return model if isinstance(model, UrdfModel) else UrdfModel(os.fspath(model))


def _tf(rot: np.ndarray, t: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = rot
    out[:3, 3] = t
    return out


def _joint_origin(joint) -> np.ndarray:
    """The joint's fixed parent->child offset as a 4x4, from the parsed URDF."""
    return _tf(np.asarray(joint.origin.R, dtype=float),
               np.asarray(joint.origin.t, dtype=float))


class _Segment:
    """One joint on the root -> end-effector path."""

    __slots__ = ("name", "child", "kind", "origin", "axis", "skew", "skew2",
                 "qi")

    def __init__(self, joint, qi: Optional[int]):
        self.name = joint.name
        self.child = joint.child
        self.kind = joint.type
        self.origin = _joint_origin(joint)
        axis = np.asarray(joint.axis, dtype=float)
        n = float(np.linalg.norm(axis))
        self.axis = axis / n if n > 0 else axis
        # Rodrigues, precomputed: Rot(a, q) = I + sin(q) K + (1 - cos(q)) K@K
        x, y, z = self.axis if self.axis.size == 3 else (0.0, 0.0, 1.0)
        self.skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
        self.skew2 = self.skew @ self.skew
        self.qi = qi


class UrdfChain:
    """One serial arm read out of a URDF, with numpy FK and a geometric Jacobian.

    ``model``: a :class:`~manipulation_kit.guard.urdf_model.UrdfModel` (or a path
    to a URDF, parsed here). Passing a model lets the two arms of one robot
    share a single parse — and the SAME parse the collision guard uses, so the
    solver and the guard cannot disagree about the geometry.

    ``ee_body``: the link whose frame is the end-effector frame. Named
    ``ee_body`` rather than ``ee_link`` to match
    :class:`~manipulation_kit.arms.d1.arm.mujoco_chain.MujocoChain`'s keyword, so
    the two substrates are interchangeable at the call site.

    ``joint_names``: the actuated joints, proximal to distal. They must be
    exactly the actuated joints on the root -> ``ee_body`` path, in that order;
    a mismatch is a ``ValueError`` at construction rather than a wrong Jacobian
    at runtime.

    ``tool_offset``: optional 4x4 applied AFTER the ``ee_body`` frame. Defaults
    to identity, which is the MuJoCo substrate's behaviour (see the module
    docstring).
    """

    def __init__(self, model: Union[UrdfModel, str, "os.PathLike"], *,
                 ee_body: str, joint_names: Sequence[str],
                 tool_offset: Optional[np.ndarray] = None):
        self.model = _parse(model)
        self.ee_body = ee_body
        self._tool = (np.eye(4) if tool_offset is None
                      else np.asarray(tool_offset, dtype=float).reshape(4, 4))
        self._names: Tuple[str, ...] = tuple(joint_names)

        path = self._path_to(ee_body)
        actuated = [j for j in path if j.type in ACTUATED]
        rigid = [j for j in path if j.type not in ACTUATED and j.type != "fixed"]
        if rigid:
            raise ValueError(
                f"{ee_body}: unsupported joint type(s) on the chain: "
                + ", ".join(f"{j.name} ({j.type})" for j in rigid))
        got = tuple(j.name for j in actuated)
        if got != self._names:
            raise ValueError(
                f"{ee_body}: the actuated joints on the path are {got}, "
                f"not {self._names} — the URDF and the side convention disagree")

        order = {n: i for i, n in enumerate(self._names)}
        self._segments: List[_Segment] = [
            _Segment(j, order.get(j.name) if j.type in ACTUATED else None)
            for j in path]
        self._kinds = [j.type for j in actuated]

        lo, hi = [], []
        for j in actuated:
            if j.lower is None or j.upper is None:   # continuous joint
                lo.append(-np.inf)
                hi.append(np.inf)
            else:
                lo.append(float(j.lower))
                hi.append(float(j.upper))
        self._lo = np.asarray(lo, dtype=float)
        self._hi = np.asarray(hi, dtype=float)

        self._q = np.zeros(len(self._names))
        self._refresh()

    # -- construction helpers --------------------------------------------- #
    def _path_to(self, link: str) -> List:
        """The joints from the URDF root down to ``link``, proximal first."""
        if link not in self.model.links:
            raise ValueError(f"no link named {link!r} in {self.model.path}")
        chain, cur = [], link
        while cur in self.model.parent_link:
            chain.append(self.model.joints[cur])
            cur = self.model.parent_link[cur]
        chain.reverse()
        return chain

    # -- KinematicChain ---------------------------------------------------- #
    @property
    def ndof(self) -> int:
        return len(self._names)

    @property
    def joint_names(self) -> Tuple[str, ...]:
        return self._names

    def joints(self) -> np.ndarray:
        return self._q.copy()

    def set_joints(self, q) -> None:
        q = np.asarray(q, dtype=float)
        if q.shape != (self.ndof,):
            raise ValueError(f"expected {self.ndof} joints, got {q.shape}")
        self._q = q.copy()
        self._refresh()

    def ee_pose(self) -> Tuple[np.ndarray, R]:
        return self._p_ee.copy(), R.from_matrix(self._rot_ee.copy())

    def jacobian(self) -> np.ndarray:
        """6 x ndof, linear rows on top — see the module docstring."""
        J = np.zeros((6, self.ndof))
        for i in range(self.ndof):
            a = self._axes[i]
            if self._kinds[i] == "prismatic":
                J[:3, i] = a
            else:
                J[:3, i] = np.cross(a, self._p_ee - self._origins[i])
                J[3:, i] = a
        return J

    def limits(self) -> Tuple[np.ndarray, np.ndarray]:
        return self._lo.copy(), self._hi.copy()

    # -- extras the D1 binding uses (MujocoChain has both) ----------------- #
    def body_xpos(self, name: str) -> np.ndarray:
        """Root-frame position of an arbitrary link at the current joints.

        Used by the READY-seed probe to score elbow geometry. Links off this
        chain resolve too (joints this chain does not drive sit at 0), which is
        what MuJoCo's shared ``mjData`` gives for free.
        """
        if name in self._link_tf:
            return self._link_tf[name][:3, 3].copy()
        T = np.eye(4)
        for j in self._path_to(name):
            T = T @ _joint_origin(j)
            if j.type in ACTUATED:
                q = float(self._q[self._names.index(j.name)]) \
                    if j.name in self._names else 0.0
                T = T @ self._joint_tf(j.type, np.asarray(j.axis, float), q)
        return T[:3, 3].copy()

    def forward(self) -> None:
        """Refresh derived quantities without changing the joints."""
        self._refresh()

    # -- FK ---------------------------------------------------------------- #
    @staticmethod
    def _joint_tf(kind: str, axis: np.ndarray, q: float) -> np.ndarray:
        n = float(np.linalg.norm(axis))
        a = axis / n if n > 0 else axis
        if kind == "prismatic":
            return _tf(np.eye(3), a * q)
        x, y, z = a
        K = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
        rot = np.eye(3) + np.sin(q) * K + (1.0 - np.cos(q)) * (K @ K)
        return _tf(rot, np.zeros(3))

    def _refresh(self) -> None:
        """Walk the chain once: EE pose, per-joint world axis and origin.

        Everything the solver asks for afterwards is a read of these, so a
        ``set_joints`` + ``ee_pose`` + ``jacobian`` triple (what one DLS
        iteration does) walks the chain exactly once, like ``mj_forward``.
        """
        T = np.eye(4)
        axes = np.zeros((self.ndof, 3))
        origins = np.zeros((self.ndof, 3))
        link_tf: Dict[str, np.ndarray] = {}
        for seg in self._segments:
            T = T @ seg.origin
            if seg.qi is not None:
                rot = T[:3, :3]
                axes[seg.qi] = rot @ seg.axis
                origins[seg.qi] = T[:3, 3]
                q = self._q[seg.qi]
                if seg.kind == "prismatic":
                    T = T.copy()
                    T[:3, 3] = T[:3, 3] + rot @ (seg.axis * q)
                else:
                    # Rodrigues on the precomputed skew — the rotation is about
                    # the joint axis expressed in the frame we just reached.
                    step = (np.eye(3) + np.sin(q) * seg.skew
                            + (1.0 - np.cos(q)) * seg.skew2)
                    T = _tf(rot @ step, T[:3, 3])
            link_tf[seg.child] = T
        self._axes = axes
        self._origins = origins
        self._link_tf = link_tf
        T_ee = T @ self._tool
        self._p_ee = T_ee[:3, 3]
        self._rot_ee = T_ee[:3, :3]


def build_chains(model: Union[UrdfModel, str, "os.PathLike"],
                 spec: Dict[str, Sequence[str]],
                 ee_bodies: Dict[str, str]) -> Dict[str, "UrdfChain"]:
    """One :class:`UrdfChain` per logical side, sharing one parsed URDF."""
    parsed = _parse(model)
    return {side: UrdfChain(parsed, ee_body=ee_bodies[side],
                            joint_names=spec[side])
            for side in spec}
