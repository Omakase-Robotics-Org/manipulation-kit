"""A MuJoCo-backed :class:`~manipulation_kit.arms.ik.KinematicChain`.

Migrated from dx-vr-teleop ``server/backends.py`` (``SimBackend``'s joint/body
index bookkeeping, ``ee_pose``, ``_set_joints``, the ``mj_jacBody`` call inside
``_ik``) at post-#41 ``master``.

This is the OPTIONAL substrate. It exists because dx-vr-teleop already carries a
full MuJoCo mirror of the robot (viewer, sim backend, composed hand model), so
FK and the Jacobian are free there and the solver can be pointed at the model
that repository is already stepping. Everyone else gets the same quantities from
:class:`~manipulation_kit.arms.urdf_chain.UrdfChain`, which walks the same URDF
in numpy and is the DEFAULT — ``mujoco`` is not a dependency of inverse
kinematics. The two agree to ~1e-15 on D1
(``tests/arms/test_urdf_chain_parity.py``), and the solver in
:mod:`manipulation_kit.arms.ik` never learns which one it is talking to.

``mujoco`` is imported by the caller and injected, not imported at module scope:
``manipulation_kit.arms`` must stay importable for the pose math alone.
"""

from __future__ import annotations

from typing import Any, List, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R


class MujocoChain:
    """One arm inside a shared ``MjModel``/``MjData``.

    NOT thread-safe, and deliberately so: the two chains of one robot share the
    same ``mjData``, and ``mjData`` is not thread-safe at all — two threads
    inside ``mj_forward`` on the same data corrupt memory. Serialisation is the
    owner's job (:class:`~manipulation_kit.arms.kinematics.GuardedArm` holds one
    re-entrant lock for both chains).
    """

    def __init__(self, mujoco: Any, model: Any, data: Any, *,
                 ee_body: str, joint_names: Sequence[str]):
        self._mj = mujoco
        self.model = model
        self.data = data
        self._bid = model.body(ee_body).id
        self._joint_ids: List[int] = [model.joint(n).id for n in joint_names]
        self._qadr: List[int] = [model.jnt_qposadr[j] for j in self._joint_ids]
        self._dof: List[int] = [model.jnt_dofadr[j] for j in self._joint_ids]

    @property
    def ndof(self) -> int:
        return len(self._joint_ids)

    @property
    def body_id(self) -> int:
        return self._bid

    def joints(self) -> np.ndarray:
        return np.array([self.data.qpos[a] for a in self._qadr])

    def set_joints(self, q) -> None:
        for a, v in zip(self._qadr, np.asarray(q, dtype=float)):
            self.data.qpos[a] = v
        self._mj.mj_forward(self.model, self.data)

    def ee_pose(self) -> Tuple[np.ndarray, R]:
        p = self.data.xpos[self._bid].copy()
        r = R.from_matrix(self.data.xmat[self._bid].reshape(3, 3).copy())
        return p, r

    def jacobian(self) -> np.ndarray:
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        self._mj.mj_jacBody(self.model, self.data, jacp, jacr, self._bid)
        return np.vstack([jacp[:, self._dof], jacr[:, self._dof]])

    def limits(self) -> Tuple[np.ndarray, np.ndarray]:
        lo = self.model.jnt_range[self._joint_ids, 0]
        hi = self.model.jnt_range[self._joint_ids, 1]
        return np.asarray(lo, dtype=float), np.asarray(hi, dtype=float)

    def body_xpos(self, name: str) -> np.ndarray:
        """World position of an arbitrary body — used by the READY-seed probe
        to score elbow geometry."""
        return self.data.xpos[self.model.body(name).id].copy()

    def forward(self) -> None:
        """Refresh derived quantities without changing the joints."""
        self._mj.mj_forward(self.model, self.data)
