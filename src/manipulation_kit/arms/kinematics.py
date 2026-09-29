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
import math
from dataclasses import dataclass
from typing import (Dict, List, Mapping, Optional, Protocol, Sequence,
                    Tuple, runtime_checkable)

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


#: The guard's clearance stages, in the daemon's order (``CLEARANCE_STAGES``
#: in ``d1fw-core``, plus the kit's hand-against-other-arm stage), each with the :class:`~manipulation_kit.guard.GuardReport`
#: field that holds its minimum. The same-arm stage has its margin already
#: subtracted in both implementations.
CLEARANCE_STAGES: Tuple[Tuple[str, str], ...] = (
    ("body", "min_body_clearance"),
    ("chest keep-out", "min_chest_clearance"),
    ("arm-arm", "min_arm_arm"),
    ("self", "min_self_clearance"),
    ("hand-arm", "min_hand_arm"),
)


def swept_sample_count(q_from: Mapping[str, Sequence[float]],
                       q_to: Mapping[str, Sequence[float]], *,
                       step_deg: float = safety.SWEPT_PATH_STEP_DEG) -> int:
    """How many samples the straight line ``q_from -> q_to`` is checked at.

    ``max(1, ceil(largest joint delta / step_deg))`` over both arms, the target
    included and the start not — d1-firmwared's ``sample_count``. Joints are
    radians per logical side; the delta is taken in DEGREES, as the daemon
    takes it, so both count the same samples for the same wire command.
    """
    if not (math.isfinite(step_deg) and step_deg > 0):
        raise ValueError(f"step_deg must be finite and positive, got {step_deg!r}")
    largest = max(float(np.max(np.abs(np.degrees(np.asarray(q_to[s], dtype=float))
                                      - np.degrees(np.asarray(q_from[s], dtype=float)))))
                  for s in sides.SIDES)
    return max(1, int(math.ceil(largest / step_deg)))


def swept_path_samples(q_from: Mapping[str, Sequence[float]],
                       q_to: Mapping[str, Sequence[float]], *,
                       step_deg: float = safety.SWEPT_PATH_STEP_DEG
                       ) -> List[Dict[str, np.ndarray]]:
    """The postures the swept-path check visits, in order: samples ``1..n`` of
    the straight joint-space line, the target last and the start not.

    Pure arithmetic, no guard: this is what a batched mirror (the sim's torch
    guard) feeds its own verdict to reproduce
    :meth:`GuardedArm.swept_path_violation` sample for sample.
    """
    n = swept_sample_count(q_from, q_to, step_deg=step_deg)
    q0 = {s: np.asarray(q_from[s], dtype=float) for s in sides.SIDES}
    q1 = {s: np.asarray(q_to[s], dtype=float) for s in sides.SIDES}
    return [{s: q0[s] + (q1[s] - q0[s]) * (k / n) for s in sides.SIDES}
            for k in range(1, n + 1)]


def _moving_joints(q_from, q_to, q_at) -> str:
    """``arm A J3=+42.0`` for every joint the move changes, at this sample —
    the daemon's wording, in degrees and SDK arm names."""
    out = []
    for side in sides.SIDES:
        for j in range(sides.JOINTS_PER_ARM):
            if float(q_from[side][j]) != float(q_to[side][j]):
                out.append(f"arm {sides.SDK_SIDE[side]} J{j + 1}="
                           f"{math.degrees(float(q_at[side][j])):+.1f}")
    return ", ".join(out)


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

    def swept_path_violation(self, q_from: Mapping[str, Sequence[float]],
                             q_to: Mapping[str, Sequence[float]], *,
                             step_deg: float = safety.SWEPT_PATH_STEP_DEG
                             ) -> Optional[str]:
        """Why the straight joint move ``q_from -> q_to`` would be refused, or
        ``None`` when every sample on it passes.

        ``q_from`` / ``q_to`` hold BOTH logical sides [rad], because the guard
        judges the two-arm posture and a two-arm move is checked as one line.
        This is d1-firmwared's single-shot joint-move guard
        (``move_joints_both``), applied with the kit's verdict:

        * the target of every MOVING arm must pass :meth:`posture_violation`
          (the joint box and the coupled limits; the daemon checks the box);
        * the line is sampled at ``step_deg`` per joint
          (:func:`swept_path_samples`), and each sample must pass
          :meth:`posture_violation` for the moving arms and the collision
          guard on the two-arm posture;
        * the ESCAPE RULE: a line whose START already violates a guard margin
          (an arm left there by hand guiding, a gesture, feedback noise on the
          margin) is not refused for that alone — refusing it would leave the
          arm unable to move out. Its samples pass while no clearance stage
          (:data:`CLEARANCE_STAGES`) reads closer than at the start, within
          :data:`~manipulation_kit.arms.safety.SWEPT_PATH_DEEPER_TOL_M`.

        The endpoint verdict (:meth:`posture_violation` + :meth:`guard_ok`) is
        this with one sample and no escape. The reason names the sample, the
        moving joints there and the guard's violations, in the daemon's words.
        """
        q0 = {s: np.asarray(q_from[s], dtype=float).reshape(sides.JOINTS_PER_ARM)
              for s in sides.SIDES}
        q1 = {s: np.asarray(q_to[s], dtype=float).reshape(sides.JOINTS_PER_ARM)
              for s in sides.SIDES}
        moving = [s for s in sides.SIDES if not np.array_equal(q0[s], q1[s])]
        for side in moving:
            why = self.posture_violation(side, q1[side])
            if why is not None:
                return (f"target outside joint limits: arm "
                        f"{sides.SDK_SIDE[side]} {why}")
        samples = swept_path_samples(q0, q1, step_deg=step_deg)
        n = len(samples)
        start = None
        for k, q in enumerate(samples, start=1):
            for side in moving:
                why = self.posture_violation(side, q[side])
                if why is not None:
                    return (f"path at sample {k}/{n} "
                            f"({_moving_joints(q0, q1, q)}): arm "
                            f"{sides.SDK_SIDE[side]} {why}")
            if not self._gate.installed:
                continue
            rep = self._gate.report(q["left"], q["right"])
            if rep.ok:
                continue
            if start is None:
                start = self._gate.report(q0["left"], q0["right"])
            inside = not start.ok
            deeper = next((name for name, field in CLEARANCE_STAGES
                           if getattr(rep, field) < getattr(start, field)
                           - safety.SWEPT_PATH_DEEPER_TOL_M), None)
            if inside and deeper is None:
                continue
            at = _moving_joints(q0, q1, q)
            if inside:
                field = dict(CLEARANCE_STAGES)[deeper]
                return (f"path at sample {k}/{n} ({at}): the arm starts inside "
                        f"the guard's margins ({list(start.violations)}) and "
                        f"this moves the {deeper} clearance closer, "
                        f"{max(getattr(start, field), 0.0):.3f} m -> "
                        f"{max(getattr(rep, field), 0.0):.3f} m: "
                        f"{list(rep.violations)}")
            return f"path at sample {k}/{n} ({at}): {list(rep.violations)}"
        return None

    def swept_path_ok(self, q_from: Mapping[str, Sequence[float]],
                      q_to: Mapping[str, Sequence[float]], *,
                      step_deg: float = safety.SWEPT_PATH_STEP_DEG) -> bool:
        """``True`` iff :meth:`swept_path_violation` finds nothing."""
        return self.swept_path_violation(q_from, q_to, step_deg=step_deg) is None

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
