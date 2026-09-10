"""the D1 arm kinematics as mounted on D1 — the vendor binding.

This is the ``<maker>/<model>`` half of the seam: everything here is knowledge
about THIS arm on THIS robot — which URDF describes it, what its links are
called, which side is which (see :mod:`manipulation_kit.arms.sides`), where HOME is, and
what its reachable envelope measures. The solver, the clamps, the guard rule and
the filters are shared and live in :mod:`manipulation_kit.arms`.

Migrated from dx-vr-teleop ``server/backends.py`` (``SimBackend``) at post-#41
``master`` (``de8d781``). Arithmetic unchanged; verified numerically identical to
that implementation by ``tests/test_parity_dx_vr_teleop.py``.

Assets: NONE outside this package. The description (``d1.urdf``) and the HOME
pose (``home_pose.json``) are DATA shipped by
:mod:`manipulation_kit.description` / :mod:`manipulation_kit.config` and read at
build time; both can still be overridden by argument, and the collision guard is
injected or lazily loaded. There is no ``$D1_SDK_DIR`` and no sibling checkout to
find. No vendor transport, no SDK handle, nothing that talks to a robot — see
:mod:`manipulation_kit.arms`' "the library ends where the wire begins". On D1 the
wire is d1-firmwared (REST :4750); this module hands it joint vectors and never
opens the socket itself.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from ... import safety, sides
from ...guard import GuardGate, load_motion_guard
from ...ik import IkTuning, find_ready_seed
from ...kinematics import GuardedArm
from ...targeting import ClutchTuning
from .mujoco_chain import MujocoChain


#: The package root, used to resolve bundled assets. Package-relative, not
#: ``$D1_SDK_DIR``: an installed wheel has no checkout beside it.
_KIT = Path(__file__).resolve().parents[3]


def default_urdf() -> Path:
    """The bundled D1 description. Primitives-only (it IS the collision model),
    so it loads anywhere without meshes and matches what the guard checks.

    Same file as :data:`manipulation_kit.guard.DEFAULT_URDF` — one copy, not two.
    """
    return _KIT / "description" / "d1" / "d1.urdf"


def default_home_pose_json() -> Path:
    """The hardware HOME pose, degrees, executable order A1..A7,B1..B7.

    NOT the arms-hanging ("daran") pose: ``home_pose.json``'s own ``_comment``
    records that the wrist-forward "ready" pose (TCP ~300,300,300 mm, gripper
    rotated 90 deg about Y, facing forward) REPLACED the daran pose so gestures
    rest wrist-up.
    """
    return _KIT / "config" / "home_pose.json"


def load_home(path: Optional[Path] = None, *, quiet: bool = False
              ) -> Dict[str, np.ndarray]:
    """Per-logical-side 7-joint HOME [rad]; URDF zero (T-pose) as fallback.

    Wire order is side A then side B, i.e. logical left then logical right —
    see :mod:`manipulation_kit.arms.sides` for why those cross over.
    """
    p = path if path is not None else default_home_pose_json()
    try:
        deg = json.loads(Path(p).read_text())["home_pose"]
        n = sides.JOINTS_PER_ARM
        return {"left": np.deg2rad(deg[:n]), "right": np.deg2rad(deg[n:2 * n])}
    except Exception as exc:  # noqa: BLE001
        if not quiet:
            print(f"[manipulation_kit.arms] WARNING: home_pose.json unavailable ({exc}); "
                  f"falling back to URDF zero (T-pose)")
        n = sides.JOINTS_PER_ARM
        return {"left": np.zeros(n), "right": np.zeros(n)}


def clutch_tuning() -> ClutchTuning:
    """This arm's reachable envelope and per-tick clamps.

    Resolved by :func:`manipulation_kit.arms.get_clutch_tuning`. The box is a measured
    property of the D1 arm as mounted on D1 — see
    :data:`manipulation_kit.arms.safety.WORKSPACE` for the measurements and for why the
    x-floor is 0.15 rather than 0.
    """
    return ClutchTuning(workspace=safety.WORKSPACE)


class D1ArmKinematics(GuardedArm):
    """Both D1 arms in one MuJoCo model, collision-gated.

    ``ready()`` returns the READY posture the IK null space biases toward; it is
    searched once at construction (deterministic, 200 restarts per side) unless
    ``find_ready=False``, in which case HOME is used.
    """

    def __init__(self, *, urdf: Optional[Path] = None,
                 home: Optional[Dict[str, np.ndarray]] = None,
                 guard: Optional[GuardGate] = None,
                 tuning: IkTuning = IkTuning(),
                 find_ready: bool = True,
                 quiet: bool = False):
        import mujoco   # local: manipulation_kit.arms must import without MuJoCo

        urdf_path = Path(urdf) if urdf else default_urdf()
        self.urdf_path = urdf_path
        self._mj = mujoco
        self.model = mujoco.MjModel.from_xml_string(urdf_path.read_text())
        self.data = mujoco.MjData(self.model)
        chains = {
            side: MujocoChain(mujoco, self.model, self.data,
                              ee_body=sides.EE_BODY[side],
                              joint_names=sides.ARM_JOINTS[side])
            for side in sides.SIDES
        }
        mujoco.mj_forward(self.model, self.data)
        super().__init__(chains, home=home if home is not None else load_home(quiet=quiet),
                         guard=guard, tuning=tuning)
        # Start at the real D1 HOME (the wrist-forward pose home_pose.json
        # holds), not URDF zero = T-pose — the same posture the physical robot
        # wakes up in, and where following engages.
        for side in sides.SIDES:
            self.set_joints(side, self._home[side])
        if find_ready:
            # NOTE ordering: _ready must stay EMPTY until both sides are done,
            # so each side's search is unbiased (solve_ik falls back to its own
            # seed as q_ref). dx-vr-teleop relies on the same property.
            found = {s: self._search_ready(s, quiet=quiet) for s in sides.SIDES}
            self._ready = found

    # -- READY-seed search ------------------------------------------------ #
    def _probe(self, side: str):
        """``(clearance, elbow_forward_of_shoulder)`` for a candidate posture,
        or ``None`` when the collision guard refuses it."""
        chain = self._chains[side]
        shoulder = sides.SHOULDER_BODY[side]
        elbow = sides.ELBOW_BODY[side]

        def probe(q: np.ndarray):
            if not self._gate.installed:
                return None
            qa = q if side == "left" else self.joints("left")
            qb = q if side == "right" else self.joints("right")
            try:
                rep = self._gate.report(qa, qb)
            except Exception:  # noqa: BLE001
                return None
            if rep is None or not rep.ok:
                return None
            # elbow geometry of THIS branch (pose the model so FK is current)
            chain.set_joints(q)
            sh = chain.body_xpos(shoulder)
            el = chain.body_xpos(elbow)
            return float(rep.min_body_clearance), float(el[0] - sh[0])

        return probe

    def _search_ready(self, side: str, *, quiet: bool = False) -> np.ndarray:
        target = np.array([safety.READY_SEED_TARGET_X,
                           sides.MOUNT_Y_SIGN[side] * safety.READY_SEED_TARGET_Y,
                           safety.READY_SEED_TARGET_Z])
        _, r0 = self.ee_pose(side)
        with self._lock:
            q, clearance = find_ready_seed(
                self._chains[side], q_home=self._home[side],
                probe=self._probe(side), target=target, r_target=r0,
                tuning=self._tuning)
        if not quiet:
            print(f"[manipulation_kit.arms] ready seed {side}: "
                  f"body clearance {clearance:.3f} m")
        return q


def build_kinematics(*, urdf: Optional[Path] = None,
                     home: Optional[Dict[str, np.ndarray]] = None,
                     guard: Any = "auto",
                     guard_config: Optional[Path] = None,
                     tuning: IkTuning = IkTuning(),
                     find_ready: bool = True,
                     quiet: bool = False) -> D1ArmKinematics:
    """Factory resolved by :func:`manipulation_kit.arms.get_arm_kinematics`.

    ``guard``:
      - ``"auto"`` (default) — lazily load
        :class:`manipulation_kit.guard.MotionGuard`, applying ``guard_config``
        if given. Unavailable => no guard, with a warning.
      - a :class:`~manipulation_kit.arms.guard.GuardGate` — used as-is.
      - any object with ``check(joints_a_deg, joints_b_deg)`` — wrapped. Pass the
        guard you ALREADY built (omakase-core has one for other purposes) rather
        than letting a second one be created behind your back.
      - ``None`` — explicitly no collision guard.
    """
    if isinstance(guard, GuardGate):
        gate: Optional[GuardGate] = guard
    elif isinstance(guard, str) and guard == "auto":
        cfg = guard_config
        if cfg is None:
            env = os.environ.get("OMAKASE_ARM_GUARD_CONFIG") or os.environ.get(
                "D1_TELEOP_GUARD_CONFIG")
            cfg = Path(env) if env else None
        gate = GuardGate(load_motion_guard(cfg, quiet=quiet))
    elif guard is None:
        gate = GuardGate(None)
    else:
        gate = GuardGate(guard)
    return D1ArmKinematics(urdf=urdf, home=home, guard=gate, tuning=tuning,
                            find_ready=find_ready, quiet=quiet)
