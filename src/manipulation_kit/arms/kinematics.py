"""The arm-agnostic kinematics contract, and the guarded-solve composition.

:class:`ArmKinematics` is what a consumer programs against after calling
:func:`manipulation_kit.arms.get_arm_kinematics`. :class:`GuardedArm` implements the part
that is the same for every arm — pose a candidate, run IK, bound the joint step,
collision-check the FULL two-arm posture, commit or restore — leaving each
``<maker>/<model>`` package to supply only its own chains, HOME pose and READY
seeds.

Migrated from dx-vr-teleop ``server/backends.py`` (``SimBackend.try_move_ee`` /
``_try_move_ee_locked`` / ``_limit_margin_deg``) at post-#41 ``master``.

One deliberate interface change, with no behavioural change: the original
returns a bare ``bool`` and leaves the caller to infer WHY a tick died from a
separate diagnostics counter. :meth:`GuardedArm.solve_ee` returns an
:class:`IkResult` carrying the joint vector and the reason. The accept/reject
decision, the joint vector committed, and the clamping are identical — this is
verified numerically against dx-vr-teleop, see
``tests/test_parity_dx_vr_teleop.py``.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import (Dict, Optional, Protocol, Sequence, Tuple,
                    runtime_checkable)

import numpy as np
from scipy.spatial.transform import Rotation as R

from . import safety, sides
from .coupled_limits import CoupledJointLimit, first_violation
from .guard import GuardGate
from .ik import IkTuning, KinematicChain, clamp_joint_step, solve_ik

#: why a solve ended — ``"ok"`` committed, or the stage that refused it
Reason = str
OK: Reason = "ok"
IK_FAIL: Reason = "ik_fail"
GUARD_REJECT: Reason = "guard_reject"
#: the solver claimed convergence but its joints do not put the end-effector on
#: the requested pose — see :meth:`GuardedArm.solve_ee`. Distinct from
#: ``IK_FAIL`` on purpose: ``IK_FAIL`` means "no solution found", this one means
#: "a solution was reported and it is not one", which is a BUG somewhere below
#: and should be loud in a diagnostics counter rather than blend into the
#: unreachable-target noise.
INFEASIBLE: Reason = "infeasible"
#: no solution inside the arm's COUPLED joint limits (e.g. the D1 wrist roll
#: J7 narrowing with J6, :mod:`manipulation_kit.arms.coupled_limits`), while a
#: box-only solve reaches the pose — the target needs more of a joint than the
#: hardware has in that posture. ``IkResult.detail`` names the limit.
JOINT_LIMIT: Reason = "joint_limit"


@dataclass(frozen=True)
class IkResult:
    """Outcome of one guarded end-effector solve."""

    ok: bool
    reason: Reason
    #: the committed joint vector [rad] when ``ok``, else ``None``
    q: Optional[np.ndarray] = None
    #: True if the solution was scaled down to respect the per-tick joint cap
    step_clamped: bool = False
    #: for ``joint_limit``: which limit, at which posture
    detail: str = ""

    def __bool__(self) -> bool:      # so `if arm.solve_ee(...):` reads naturally
        return self.ok


@runtime_checkable
class ArmKinematics(Protocol):
    """The cross-arm kinematics contract every ``<maker>/<model>`` implements.

    Poses are in the ROBOT BASE frame, joints in radians, and ``side`` is a
    LOGICAL side name (``"left"``/``"right"`` — see :mod:`manipulation_kit.arms.sides`,
    which is where the URDF/SDK crossover is written down once).

    Nothing here commands hardware. ``solve_ee`` moves the internal model only;
    putting the resulting joint vector on the wire is the consumer's job.
    """

    @property
    def sides(self) -> Tuple[str, ...]: ...
    def joints(self, side: str) -> np.ndarray: ...
    def set_joints(self, side: str, q) -> None: ...
    def ee_pose(self, side: str) -> Tuple[np.ndarray, R]: ...
    def home(self, side: str) -> np.ndarray: ...
    def ready(self, side: str) -> np.ndarray: ...
    def solve_ee(self, side: str, p, r: R, **kw) -> IkResult: ...
    def limit_margin_deg(self, side: str) -> float: ...


class GuardedArm:
    """Two chains sharing one model, gated by one collision guard.

    Subclasses supply ``_chains``, ``_home`` and ``_ready`` and may override
    ``_probe``. The mirror lock is held across a whole solve because the IK
    iterates by POSING the model: a concurrent reader would otherwise observe
    intermediate postures, and a shared MuJoCo ``mjData`` would corrupt outright.
    """

    #: per-tick joint-change cap [rad]
    max_joint_step = safety.MAX_JOINT_STEP_RAD

    def __init__(self, chains: Dict[str, KinematicChain], *,
                 home: Dict[str, np.ndarray],
                 guard: Optional[GuardGate] = None,
                 tuning: IkTuning = IkTuning(),
                 coupled: Optional[Dict[str, Sequence[CoupledJointLimit]]] = None):
        self._chains = chains
        #: side -> the joint limits that depend on another joint, enforced
        #: with the box inside the IK and by :meth:`posture_violation`
        self._coupled: Dict[str, Tuple[CoupledJointLimit, ...]] = {
            side: tuple((coupled or {}).get(side, ())) for side in chains}
        self._home = {s: np.asarray(q, dtype=float) for s, q in home.items()}
        self._ready: Dict[str, np.ndarray] = {}
        self._gate = guard if guard is not None else GuardGate(None)
        self._tuning = tuning
        # Re-entrant: a lock-holding op (solve_ee -> solve_ik -> set_joints)
        # nests safely, and the two chains share one model.
        self._lock = threading.RLock()

    # -- introspection ---------------------------------------------------- #
    @property
    def sides(self) -> Tuple[str, ...]:
        return tuple(self._chains)

    @property
    def lock(self) -> "threading.RLock":
        """The mirror lock. A consumer doing its own multi-step read/modify of
        the model (e.g. read joints, filter, write back) must hold it, exactly
        as dx-vr-teleop's ``_mirror_guard()`` does."""
        return self._lock

    @property
    def tuning(self) -> IkTuning:
        """The IK tuning :meth:`solve_ee` uses when it is given none."""
        return self._tuning

    @property
    def gate(self) -> GuardGate:
        return self._gate

    def _chain(self, side: str) -> KinematicChain:
        sides.check(side)
        return self._chains[side]

    # -- state ------------------------------------------------------------ #
    def joints(self, side: str) -> np.ndarray:
        with self._lock:
            return np.array(self._chain(side).joints(), dtype=float)

    def set_joints(self, side: str, q) -> None:
        with self._lock:
            self._chain(side).set_joints(np.asarray(q, dtype=float))

    def ee_pose(self, side: str) -> Tuple[np.ndarray, R]:
        with self._lock:
            p, r = self._chain(side).ee_pose()
            return np.array(p, dtype=float), r

    def home(self, side: str) -> np.ndarray:
        return np.array(self._home[sides.check(side)], dtype=float)

    def ready(self, side: str) -> np.ndarray:
        sides.check(side)
        return np.array(self._ready.get(side, self._home[side]), dtype=float)

    def limits(self, side: str) -> Tuple[np.ndarray, np.ndarray]:
        with self._lock:
            return self._chain(side).limits()

    def coupled_limits(self, side: str) -> Tuple[CoupledJointLimit, ...]:
        """The joint limits of ``side`` that depend on another joint."""
        return self._coupled[sides.check(side)]

    def posture_violation(self, side: str, q) -> Optional[str]:
        """Why the arm cannot hold ``q`` [rad], or ``None`` when it can.

        THE posture check: the per-joint box and the coupled limits, in one
        place. ``solve_ee`` applies it to every solution it returns; a joint
        vector that did not come out of the solver (a recorded plan, a
        joint-space goal) is checked here too.
        """
        q = np.asarray(q, dtype=float)
        with self._lock:
            lo, hi = self._chain(side).limits()
        for i, (v, a, b) in enumerate(zip(q, lo, hi)):
            if v < a - 1e-9 or v > b + 1e-9:
                return (f"J{i + 1}={np.degrees(v):+.1f} deg is outside the "
                        f"box [{np.degrees(a):.0f}, {np.degrees(b):.0f}] deg")
        return first_violation(self._coupled[side], q)

    def limit_margin_deg(self, side: str) -> float:
        """Smallest distance [deg] of any joint to its limit.

        A joint pinned at a limit makes DLS IK grind and the vendor controller
        brake, so this is the number to watch when an arm feels "stuck".
        """
        with self._lock:
            q = self.joints(side)
            lo, hi = self._chain(side).limits()
            return float(np.degrees(np.min(np.minimum(q - lo, hi - q))))

    # -- the guarded solve ------------------------------------------------ #
    def guard_ok(self, side: str, q) -> bool:
        """Collision-check a CANDIDATE posture for ``side``.

        The other arm contributes its CURRENT committed joints, because the
        guard evaluates the full two-arm posture.
        """
        with self._lock:
            q = np.asarray(q, dtype=float)
            qa = q if side == "left" else self.joints("left")
            qb = q if side == "right" else self.joints("right")
            return self._gate.ok(qa, qb)

    @staticmethod
    def _reaches(chain: KinematicChain, q, p, r: R, tuning: IkTuning) -> bool:
        """Does ``q`` actually put the end-effector on ``(p, r)``, and can the
        arm hold it? Poses the chain at ``q`` and asks FK.

        This re-asks a question :func:`~manipulation_kit.arms.ik.solve_ik` already
        answered, on purpose. ``solve_ik`` used to answer it about a joint
        vector it then MODIFIED on the way out (clipping into limits), so
        ``ok=True`` could come back with the end-effector 16 cm and 81 deg from
        where it was asked for — observed on d1-3, 2026-08-25. That hole is
        closed in the solver itself; this is the second wall, so that the next
        code path which returns an unverified ``q`` — a different solver, an
        analytic seed, a cache — cannot reach a motion primitive as ``ok``.
        Cost is one FK per accepted solve against 60 inside the loop.
        """
        chain.set_joints(np.asarray(q, dtype=float))
        p_fk, r_fk = chain.ee_pose()
        e_p = np.asarray(p, dtype=float) - np.asarray(p_fk, dtype=float)
        e_r = (r * r_fk.inv()).as_rotvec()
        lo, hi = chain.limits()
        return bool(np.linalg.norm(e_p) < tuning.pos_tol
                    and np.linalg.norm(e_r) < tuning.rot_tol
                    and np.all(q >= lo) and np.all(q <= hi))

    def solve_ee(self, side: str, p, r: R, *, q0=None,
                 tuning: Optional[IkTuning] = None) -> IkResult:
        """IK to an end-effector pose, bounded and collision-gated.

        On success the model is left AT the accepted joints; on any refusal it
        is left exactly where it was. That is the behaviour the real robot needs:
        a refused target means the arm holds its last safe pose and stops at the
        body boundary rather than passing through it.

        ``ok`` MEANS THE SOLUTION REACHES THE TARGET. It does not mean the arm is
        there yet — a solution further than :attr:`max_joint_step` is committed
        PARTIALLY and flagged ``step_clamped`` (see
        :func:`~manipulation_kit.arms.ik.clamp_joint_step`); the consumer loops and
        re-asks, and converges over a few ticks. Those two are different claims
        and both are load-bearing: a motion primitive that trusts ``ok`` is
        trusting that the pose it asked for is achievable, and it reads
        ``step_clamped`` to know it must keep asking.
        """
        tun = tuning or self._tuning
        with self._lock:
            chain = self._chain(side)
            q_start = np.array(self.joints(side) if q0 is None else q0, dtype=float)
            coupled = self._coupled[side]
            q = solve_ik(chain, p, r, q_start, q_ref=self.ready(side),
                         tuning=tun, coupled=coupled)
            if q is None:
                return self._why_not(side, chain, p, r, q_start, tun)
            if not self._reaches(chain, q, p, r, tun):
                chain.set_joints(q_start)   # leave the arm exactly where it was
                return IkResult(False, INFEASIBLE)
            why = first_violation(coupled, q)
            if why is not None:
                # the second wall, as for the box in ``_reaches``: a solver
                # that returned a posture the wrist cannot hold
                chain.set_joints(q_start)
                return IkResult(False, JOINT_LIMIT, detail=why)
            q, clamped = clamp_joint_step(q_start, q, self.max_joint_step)
            if self._gate.installed and not self.guard_ok(side, q):
                chain.set_joints(q_start)   # leave the arm exactly where it was
                return IkResult(False, GUARD_REJECT, step_clamped=clamped)
            chain.set_joints(q)
            return IkResult(True, OK, q=q, step_clamped=clamped)

    def _why_not(self, side: str, chain: KinematicChain, p, r: R, q_start,
                 tun: IkTuning) -> IkResult:
        """A failed solve, told apart: was it the COUPLED limit?

        Re-solve inside the box alone from the same seed. When that reaches the
        pose at a posture the coupled limit forbids, the refusal is
        ``joint_limit`` naming the limit — the caller (the planner's roll and
        seed candidates) needs to know that another roll can fix it and the
        operator needs to know which joint. Otherwise ``ik_fail`` as before.
        Only runs on a failure, and leaves the chain at ``q_start``.
        """
        coupled = self._coupled[side]
        if coupled:
            q_box = solve_ik(chain, p, r, q_start, q_ref=self.ready(side),
                             tuning=tun)
            chain.set_joints(q_start)
            if q_box is not None:
                why = first_violation(coupled, q_box)
                if why is not None:
                    return IkResult(False, JOINT_LIMIT,
                                    detail=f"the box-only solution has {why}")
        return IkResult(False, IK_FAIL)
