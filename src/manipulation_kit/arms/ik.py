"""Damped-least-squares IK with a null-space posture bias — substrate-agnostic.

Migrated from dx-vr-teleop ``server/backends.py`` (``SimBackend._ik``,
``_find_ready_seed``, the joint-step clamp inside ``_try_move_ee_locked``) at
post-#41 ``master``. The arithmetic is unchanged.

WHY THIS IS SUBSTRATE-AGNOSTIC
------------------------------
Consumers do forward kinematics with different machinery, on purpose:

- the DEFAULT, :class:`~manipulation_kit.arms.urdf_chain.UrdfChain`, walks the
  URDF this package already parses for the collision guard, with Rodrigues FK
  and a geometric Jacobian in numpy. No physics engine — inverse kinematics
  does not need one, and most consumers (omakase-core, d1-inference,
  poc-dx-inspect-robots) have no reason to install one;
- dx-vr-teleop already keeps a full MuJoCo mirror of the robot (it has a
  viewer, a sim backend and a composed hand model), so there FK and the
  Jacobian are free from ``mj_jacBody`` —
  :class:`~manipulation_kit.arms.d1.arm.mujoco_chain.MujocoChain`, reached with
  ``get_arm_kinematics("d1/arm", chain="mujoco")``.

If the shared solver were welded to either one, the other could not adopt it,
and we would keep two copies of the SOLVER — which is the part that carries the
tuning and the safety behaviour. So the solver talks to a
:class:`KinematicChain`: something that can be posed, and then asked for its
end-effector pose, its Jacobian and its joint limits. Both substrates satisfy
that in a few lines (and agree to ~1e-15 on D1 — see
``tests/arms/test_urdf_chain_parity.py``), and the solver, its constants and
its failure semantics are then shared.

The protocol is deliberately STATEFUL (``set_joints`` then query) because that
is what a shared MuJoCo ``mjData`` requires; a purely functional chain
(``fk(q)``, ``jacobian(q)``) adapts by storing ``q`` and forwarding.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import (TYPE_CHECKING, Callable, Optional, Protocol, Sequence,
                    Tuple, runtime_checkable)

import numpy as np
from scipy.spatial.transform import Rotation as R

from . import safety

if TYPE_CHECKING:  # pragma: no cover
    from .coupled_limits import CoupledJointLimit


@runtime_checkable
class KinematicChain(Protocol):
    """One serial arm, posable, differentiable at the current pose.

    Implementations are NOT required to be thread-safe: a chain backed by a
    shared model (MuJoCo ``mjData``) must be serialised by its owner, which is
    why :class:`~manipulation_kit.arms.kinematics.GuardedArm` holds a re-entrant lock
    around every access.
    """

    @property
    def ndof(self) -> int:
        """Number of actuated joints in this chain."""
        ...

    def joints(self) -> np.ndarray:
        """Current joint vector [rad], length ``ndof``."""
        ...

    def set_joints(self, q: np.ndarray) -> None:
        """Pose the chain at ``q`` [rad] and refresh derived quantities."""
        ...

    def ee_pose(self) -> Tuple[np.ndarray, R]:
        """End-effector position [m] and orientation at the current joints."""
        ...

    def jacobian(self) -> np.ndarray:
        """6 x ndof geometric Jacobian at the current joints (linear on top)."""
        ...

    def limits(self) -> Tuple[np.ndarray, np.ndarray]:
        """Lower and upper joint limits [rad], each length ``ndof``."""
        ...


@dataclass(frozen=True)
class IkTuning:
    """Solver knobs; defaults are the hardware-driven values in ``safety``."""

    iters: int = safety.IK_ITERS
    pos_tol: float = safety.IK_POS_TOL
    rot_tol: float = safety.IK_ROT_TOL
    damping: float = safety.IK_DAMPING
    posture_gain: float = safety.IK_POSTURE_GAIN
    pull_clip: float = safety.IK_PULL_CLIP
    dq_clip: float = safety.IK_DQ_CLIP


DEFAULT_TUNING = IkTuning()


def solve_ik(chain: KinematicChain, p_target, r_target: R, q0,
             *, q_ref=None, tuning: IkTuning = DEFAULT_TUNING,
             coupled: Sequence["CoupledJointLimit"] = ()
             ) -> Optional[np.ndarray]:
    """Damped least squares on the chain's joints, seeded at ``q0``.

    A 7-DoF arm has a free elbow-swivel null space; without a bias the solver
    happily routes the elbow THROUGH the torso (the collision guard then
    rejects, and following freezes). We project a gentle pull toward ``q_ref``
    — the READY posture, see :func:`find_ready_seed` — into the Jacobian null
    space: while the operator moves, the elbow CONTINUOUSLY drifts toward the
    clearance-maximising branch, with no discrete branch jumps. Joint-space
    continuity is a hard safety requirement; an elbow that snaps to another IK
    branch mid-motion is dangerous on real hardware. Task-space accuracy is
    untouched.

    Returns the joint vector on convergence, or ``None``. The returned vector is
    always WITHIN the joint limits AND at the pose it converged to — see below.
    On failure the chain is restored to ``q0`` — a failed solve must leave
    nothing moved.

    THE LIMITS ARE ENFORCED INSIDE THE ITERATION (projected gradient), NOT AFTER
    ---------------------------------------------------------------------------
    This loop used to iterate an UNBOUNDED ``q``, test convergence on it, and
    only then clip the winner into the joint limits on the way out. That clip
    moves joints, and nothing ever re-checked where the clipped posture actually
    puts the end-effector — so a solve that "converged" at an unreachable
    posture returned a DIFFERENT, silently wrong one, with ``ok`` still set.
    Measured on d1-3 (2026-08-25) driving a ~90 deg wrist reorientation for
    press_hand: ``ok=True`` with the returned joints FK-ing 16 cm and 81 deg off
    target, J6 sitting exactly on its -1.0472 limit. A random sweep over this
    chain found the same lie on 5570 of 7733 "successful" solves.

    Clipping each UPDATE instead makes the loop a projected-gradient descent on
    the feasible box: every ``q`` the convergence test ever sees is one the arm
    can actually hold, so a pass means the SOLUTION reaches the target, and a
    target that needs a joint past its limit now runs out of iterations and
    returns ``None`` — the honest answer. Where the limits are not active the
    projection is the identity and the arithmetic is bit-for-bit what it was;
    teleop following, which moves a few hundredths of a rad per tick well inside
    the box, is untouched.

    COUPLED LIMITS ARE PART OF THE FEASIBLE SET (``coupled``). The box is not
    the whole envelope: on the D1 the wrist roll J7 reaches less far the more
    J6 is pitched (:mod:`manipulation_kit.arms.coupled_limits`). Each update is
    projected onto those too, after the box, so the same argument holds — a
    target that needs more roll than the wrist has returns ``None`` and the
    caller tries another roll or seed. Measured 2026-09-22 on d1-2: the box-only
    solver returned J7 = -90 deg at J6 = 55 deg, the wrist stopped at -39.5.
    """
    q = np.array(q0, dtype=float)
    q_ref = np.asarray(q0 if q_ref is None else q_ref, dtype=float)
    n = chain.ndof
    eye_n = np.eye(n)
    eye6 = np.eye(6)
    lo, hi = chain.limits()
    # A seed handed in out of limits is not a posture the arm can hold either;
    # start feasible so the very first convergence test is honest too.
    q = _feasible(np.clip(q, lo, hi), coupled)
    for _ in range(tuning.iters):
        chain.set_joints(q)
        p, r = chain.ee_pose()
        e_p = np.asarray(p_target, dtype=float) - np.asarray(p, dtype=float)
        e_r = (r_target * r.inv()).as_rotvec()
        err = np.concatenate([e_p, e_r])
        if (np.linalg.norm(e_p) < tuning.pos_tol
                and np.linalg.norm(e_r) < tuning.rot_tol):
            # q is already inside the limits (it is clipped on every update
            # below), and the errors just tested are the ones FK reports AT
            # this q. No post-hoc clip, because a post-hoc clip is exactly the
            # bug: it would move the joints after the test that blessed them.
            return q
        J = chain.jacobian()
        dq = J.T @ np.linalg.solve(J @ J.T + tuning.damping * eye6, err)
        # gentle, BOUNDED posture pull: cap the per-iteration null-space
        # displacement so the elbow migrates toward q_ref over many ticks
        # (smooth, always under the caller's joint-step clamp) instead of
        # re-configuring inside a single solve (= a visible jump).
        pull = np.clip(q_ref - q, -tuning.pull_clip, tuning.pull_clip)
        dq = dq + tuning.posture_gain * (eye_n - np.linalg.pinv(J) @ J) @ pull
        q = _feasible(np.clip(q + np.clip(dq, -tuning.dq_clip, tuning.dq_clip),
                              lo, hi), coupled)
    # restore the seed pose — IK failed, nothing should have moved
    chain.set_joints(np.array(q0, dtype=float))
    return None


def _feasible(q: np.ndarray, coupled) -> np.ndarray:
    """``q`` projected onto the coupled limits (identity when there are none)."""
    for lim in coupled:
        q = lim.project(q)
    return q


def clamp_joint_step(q0, q, limit: float = safety.MAX_JOINT_STEP_RAD
                     ) -> Tuple[np.ndarray, bool]:
    """Bound the per-tick joint change, executing an over-solve PARTIALLY.

    Never JUMP, but never STALL either: a solution further than ``limit`` is
    executed partially (scaled along ``q0 -> q``), so fast wrist flicks track at
    bounded joint speed instead of freezing. Reject-stalls read as "huge
    latency" to the operator, so scaling is strictly better than refusing. The
    scaled posture is still collision-checked like any other.

    Returns ``(q_out, was_clamped)``.
    """
    q0 = np.asarray(q0, dtype=float)
    q = np.asarray(q, dtype=float)
    dq = q - q0
    worst = float(np.max(np.abs(dq))) if dq.size else 0.0
    if limit > 0.0 and worst > limit:
        return q0 + dq * (limit / worst), True
    return q, False


#: ``probe(q) -> (min_body_clearance_m, elbow_forward_of_shoulder_m)`` or None
#: when the candidate posture is rejected outright (collision guard).
ReadyProbe = Callable[[np.ndarray], Optional[Tuple[float, float]]]


def find_ready_seed(chain: KinematicChain, *, q_home, probe: ReadyProbe,
                    target, r_target: R,
                    samples: int = safety.READY_SEED_SAMPLES,
                    rng_seed: int = safety.READY_SEED_RNG,
                    iters: int = safety.READY_SEED_IK_ITERS,
                    fwd_weight: float = safety.READY_SEED_FWD_W,
                    fwd_free: float = safety.READY_SEED_FWD_FREE,
                    tuning: IkTuning = DEFAULT_TUNING,
                    coupled: Sequence["CoupledJointLimit"] = ()
                    ) -> Tuple[np.ndarray, float]:
    """One-time search for the rest branch the IK null space biases toward.

    Objective = MAX body clearance (this is what keeps the guard from rejecting
    belly-front reaches mid-motion — a low-clearance branch means holds and
    jerks) MINUS a light forward-jut penalty. Pure max-clearance picked an
    elbow-up-and-FORWARD branch for the right arm (+0.26 m fwd) that looked
    wrong; the penalty steers to an elbow-down-and-out branch of equal or
    better clearance without jutting. An outboard/armpit penalty was tried and
    reverted: tucking the elbow in halved clearance and destabilised following.

    Deterministic: the RNG is seeded, because the posture found here is the bias
    every later solve drifts toward and it must be reproducible.

    Restores the chain to its entry pose. Returns ``(q_ready, clearance)`` and
    falls back to ``q_home`` (clearance ``-1.0``) if nothing scored.
    """
    lo, hi = chain.limits()
    cur = np.array(chain.joints(), dtype=float).copy()
    rng = np.random.default_rng(rng_seed)
    best = np.asarray(q_home, dtype=float)
    best_score, best_clear = -1e9, -1.0
    probe_tuning = IkTuning(iters=iters, pos_tol=tuning.pos_tol,
                            rot_tol=tuning.rot_tol, damping=tuning.damping,
                            posture_gain=tuning.posture_gain,
                            pull_clip=tuning.pull_clip, dq_clip=tuning.dq_clip)
    for _ in range(samples):
        seed = rng.uniform(lo, hi)
        q = solve_ik(chain, target, r_target, seed, tuning=probe_tuning)
        # a READY posture the wrist cannot hold is no bias to drift toward.
        # Filtered, not projected: the samples the search draws, and so the
        # posture it picks when the winner is feasible, stay what they were.
        if q is None or any(lim.violation(q) for lim in coupled):
            continue
        got = probe(q)
        if got is None:
            continue
        clearance, fwd = got
        score = clearance - fwd_weight * max(0.0, fwd - fwd_free)
        if score > best_score:
            best, best_score, best_clear = q, score, clearance
    chain.set_joints(cur)          # restore
    return best, best_clear
