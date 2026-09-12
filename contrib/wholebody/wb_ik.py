"""Whole-body differential IK for D1 (arms + lift + differential-drive base).

Loads description/d1/d1_wholebody.urdf into MuJoCo and solves velocity-level
IK in a REDUCED velocity space that enforces the base's nonholonomic
constraint by construction:

    u = [ v, w, dlift, qdot_R(7), qdot_L(7) ]          (16 dof)

    qdot_base = [xdot, ydot, yawdot] = [v*cos(yaw), v*sin(yaw), w]

so the QP/DLS never even sees a lateral base velocity — the differential
2-wheel chassis constraint (no sideways slide) cannot be violated.

Redundancy is resolved with weighted damped least squares: each reduced DoF
carries a motion cost (arms cheap, lift mid, base expensive), which makes the
solver prefer arm motion and recruit the lift/base only when the task demands
it — the "everything flows naturally" behaviour, emergent from the weights.

The EE task frame is the vendor TCP (JointTCP_*): MuJoCo fuses fixed-joint
children, so the TCP frame is reconstructed rigidly from Link7 using the
generator's TCP_XYZ/TCP_RPY.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field

import mujoco
import numpy as np

#: Resolved through the import: this package lives outside the wheel now
#: (contrib/), while the description still ships inside manipulation_kit.
URDF = os.path.join(
    os.path.dirname(os.path.abspath(__import__("manipulation_kit").__file__)),
    "description", "d1", "d1_wholebody.urdf")

# Link7 -> TCP fixed transform (generator TCP_XYZ / TCP_RPY, rpy=XYZ-fixed).
_TCP_XYZ = np.array([0.0, -0.087, 0.0])
_TCP_RPY = (1.5708, -1.5708, 0.0)


def _rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


R_L7_TCP = _rpy_to_mat(*_TCP_RPY)


def _mat_to_rotvec(R):
    """SO(3) log map (rotation vector)."""
    tr = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    th = np.arccos(tr)
    if th < 1e-9:
        return np.zeros(3)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return th / (2.0 * np.sin(th)) * w


@dataclass
class Costs:
    """Per-DoF motion costs (diagonal D of the DLS regularizer). Bigger =
    more reluctant to move. Tuned so arms lead, lift assists, base follows."""
    arm: float = 1.0
    lift: float = 8.0
    base_v: float = 30.0
    base_w: float = 60.0
    damping: float = 1e-3          # global Tikhonov floor
    posture: float = 1.0           # null-space pull of arms toward home


class WholeBodyIK:
    MODES = ("arms", "arms+lift", "full")

    def __init__(self, mode: str = "full", costs: Costs | None = None):
        assert mode in self.MODES, mode
        self.mode = mode
        self.costs = costs or Costs()
        self.model = mujoco.MjModel.from_xml_path(URDF)
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.j = {m.joint(i).name: i for i in range(m.njnt)}
        self.qadr = {n: m.joint(i).qposadr[0] for n, i in self.j.items()}
        self.vadr = {n: m.joint(i).dofadr[0] for n, i in self.j.items()}
        self.arm_joints = {s: [f"Joint{k}_{s}" for k in range(1, 8)] for s in ("R", "L")}
        self.link7 = {s: m.body(f"Link7_{s}").id for s in ("R", "L")}
        # reduced-space layout: [v, w, dlift, R arm(7), L arm(7)]
        self.nu = 17
        self._u_arm0 = {"R": 3, "L": 10}
        # joint limits for clamping (arms + lift)
        self.limits = {}
        for s in ("R", "L"):
            for jn in self.arm_joints[s]:
                jid = self.j[jn]
                self.limits[jn] = tuple(m.jnt_range[jid])
        self.limits["lift"] = tuple(m.jnt_range[self.j["lift"]])
        self.home_arm = {s: np.zeros(7) for s in ("R", "L")}

    # -- state ---------------------------------------------------------------
    def set_home(self, arm_q: dict[str, np.ndarray] | None = None,
                 lift: float = 0.0, base=(0.0, 0.0, 0.0)):
        d, q = self.data, self.data.qpos
        q[:] = 0.0
        if arm_q:
            for s, qs in arm_q.items():
                self.home_arm[s] = np.asarray(qs, float).copy()
                for jn, v in zip(self.arm_joints[s], qs):
                    q[self.qadr[jn]] = v
        q[self.qadr["lift"]] = lift
        q[self.qadr["base_x"]], q[self.qadr["base_y"]], q[self.qadr["base_yaw"]] = base
        mujoco.mj_kinematics(self.model, d)

    def base_pose(self):
        q = self.data.qpos
        return np.array([q[self.qadr["base_x"]], q[self.qadr["base_y"]],
                         q[self.qadr["base_yaw"]]])

    def lift_q(self) -> float:
        return float(self.data.qpos[self.qadr["lift"]])

    def arm_q(self, side: str) -> np.ndarray:
        return np.array([self.data.qpos[self.qadr[jn]] for jn in self.arm_joints[side]])

    def tcp_pose(self, side: str):
        """World (position, rotation matrix) of the vendor TCP frame."""
        bid = self.link7[side]
        R7 = self.data.xmat[bid].reshape(3, 3)
        p7 = self.data.xpos[bid]
        return p7 + R7 @ _TCP_XYZ, R7 @ R_L7_TCP

    # -- core ----------------------------------------------------------------
    def _selector(self):
        """S(q): nv x nu map from reduced velocities to joint velocities."""
        m, q = self.model, self.data.qpos
        yaw = q[self.qadr["base_yaw"]]
        S = np.zeros((m.nv, self.nu))
        if self.mode == "full":
            S[self.vadr["base_x"], 0] = np.cos(yaw)
            S[self.vadr["base_y"], 0] = np.sin(yaw)
            S[self.vadr["base_yaw"], 1] = 1.0
        if self.mode in ("arms+lift", "full"):
            S[self.vadr["lift"], 2] = 1.0
        for s in ("R", "L"):
            for k, jn in enumerate(self.arm_joints[s]):
                S[self.vadr[jn], self._u_arm0[s] + k] = 1.0
        return S

    def _cost_diag(self):
        c = self.costs
        D = np.empty(self.nu)
        D[0], D[1], D[2] = c.base_v, c.base_w, c.lift
        D[3:10] = c.arm
        D[10:17] = c.arm
        return D

    def step(self, targets: dict[str, tuple[np.ndarray, np.ndarray]],
             dt: float = 0.02, gain: float = 4.0,
             pos_w: float = 1.0, ori_w: float = 0.5,
             ff: dict | None = None) -> dict:
        """One velocity-IK step toward per-side (pos, R) targets.

        ff: optional per-side 3-vector reference VELOCITY feedforward — a
        pure proportional tracker lags a moving reference by v_ref/gain_eff.
        """
        m, d = self.model, self.data
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        S = self._selector()
        rows_J, rows_e, rows_w = [], [], []
        for s, (p_t, R_t) in targets.items():
            bid = self.link7[s]
            p_cur, R_cur = self.tcp_pose(s)
            jacp = np.zeros((3, m.nv)); jacr = np.zeros((3, m.nv))
            mujoco.mj_jac(m, d, jacp, jacr, p_cur, bid)
            e_p = p_t - p_cur
            e_r = _mat_to_rotvec(R_t @ R_cur.T)
            v_ff = (ff or {}).get(s, np.zeros(3))
            rows_J += [jacp, jacr]
            rows_e += [gain * e_p + v_ff, gain * e_r]
            rows_w += [np.full(3, pos_w), np.full(3, ori_w)]
        J = np.vstack(rows_J) @ S
        e = np.concatenate(rows_e)
        w = np.concatenate(rows_w)
        c = self.costs
        # Weighted LEAST-NORM (not ridge): realize the task velocity exactly
        # (up to a small regularizer) and let the motion costs D only decide
        # HOW the velocity is distributed across DoF. Ridge (JᵀWJ+D)⁻¹JᵀWe
        # attenuates the realized velocity by σ²/(σ²+D) — a tracker built on
        # it lags a moving reference by v_ref on that factor.
        sw = np.sqrt(w)
        Jw = J * sw[:, None]
        ew = e * sw
        Dinv = 1.0 / self._cost_diag()
        A = Jw @ (Dinv[:, None] * Jw.T) + c.damping * np.eye(Jw.shape[0])
        u = Dinv * (Jw.T @ np.linalg.solve(A, ew))
        max_ep = max(np.linalg.norm(p_t - self.tcp_pose(s)[0])
                     for s, (p_t, _R) in targets.items())
        # posture: pull arms toward home IN THE TASK NULL SPACE only, so it
        # regularizes redundancy without biasing the EE solution (a weighted
        # sum would settle at a cm-level standoff from the target). The
        # projector must come from a near-exact pseudoinverse of J — building
        # it from the damped normal matrix A leaks task-space motion.
        # Scale the posture flow with the task error: the projection only
        # cancels EE motion to FIRST order, and near convergence the
        # second-order (curvature) drift of a large null-space flow swamps
        # the tiny remaining task velocity — the solver floors mm short.
        # floor: never fully kill the posture flow — the small residual pull
        # keeps re-centering the lift/arms without measurable EE drift.
        post_scale = float(np.clip(max_ep / 0.10, 0.15, 1.0))
        u_post = np.zeros(self.nu)
        for s in ("R", "L"):
            a0 = self._u_arm0[s]
            u_post[a0:a0 + 7] = np.clip(
                c.posture * (self.home_arm[s] - self.arm_q(s)),
                -0.5, 0.5) * post_scale
        if self.mode in ("arms+lift", "full"):
            # the lift needs a home-pull too: without one it races to a rail
            # end during the approach and STAYS there (velocity costs never
            # pay it back), leaving the arm in a worse basin than arms-only.
            u_post[2] = np.clip(c.posture * (0.0 - self.lift_q()), -0.2, 0.2) * post_scale
        Jp = J.T @ np.linalg.solve(J @ J.T + 1e-8 * np.eye(J.shape[0]), J)
        u = u + (np.eye(self.nu) - Jp) @ u_post
        # integrate reduced velocities
        q = d.qpos
        yaw = q[self.qadr["base_yaw"]]
        if self.mode == "full":
            q[self.qadr["base_x"]] += np.cos(yaw) * u[0] * dt
            q[self.qadr["base_y"]] += np.sin(yaw) * u[0] * dt
            q[self.qadr["base_yaw"]] += u[1] * dt
        if self.mode in ("arms+lift", "full"):
            lo, hi = self.limits["lift"]
            q[self.qadr["lift"]] = np.clip(q[self.qadr["lift"]] + u[2] * dt, lo, hi)
        for s in ("R", "L"):
            for k, jn in enumerate(self.arm_joints[s]):
                lo, hi = self.limits[jn]
                q[self.qadr[jn]] = np.clip(
                    q[self.qadr[jn]] + u[self._u_arm0[s] + k] * dt, lo, hi)
        mujoco.mj_kinematics(m, d)
        errs = {}
        for s, (p_t, R_t) in targets.items():
            p_cur, R_cur = self.tcp_pose(s)
            errs[s] = (float(np.linalg.norm(p_t - p_cur)),
                       float(np.degrees(np.linalg.norm(_mat_to_rotvec(R_t @ R_cur.T)))))
        return {"u": u, "err": errs}

    # -- discrete solve (Levenberg–Marquardt) ---------------------------------
    def _task_matrices(self, targets, pos_w, ori_w):
        m, d = self.model, self.data
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        S = self._selector()
        rows_J, rows_e, rows_w = [], [], []
        for s, (p_t, R_t) in targets.items():
            p_cur, R_cur = self.tcp_pose(s)
            jacp = np.zeros((3, m.nv)); jacr = np.zeros((3, m.nv))
            mujoco.mj_jac(m, d, jacp, jacr, p_cur, self.link7[s])
            rows_J += [jacp, jacr]
            rows_e += [p_t - p_cur, _mat_to_rotvec(R_t @ R_cur.T)]
            rows_w += [np.full(3, pos_w), np.full(3, ori_w)]
        return (np.vstack(rows_J) @ S, np.concatenate(rows_e),
                np.concatenate(rows_w))

    def _get_q(self):
        return self.data.qpos.copy()

    def _apply_du(self, q0, du):
        """q = q0 ⊕ du in reduced coordinates (with limit clamping)."""
        q = self.data.qpos
        q[:] = q0
        yaw = q[self.qadr["base_yaw"]]
        if self.mode == "full":
            q[self.qadr["base_x"]] += np.cos(yaw) * du[0]
            q[self.qadr["base_y"]] += np.sin(yaw) * du[0]
            q[self.qadr["base_yaw"]] += du[1]
        if self.mode in ("arms+lift", "full"):
            lo, hi = self.limits["lift"]
            q[self.qadr["lift"]] = np.clip(q[self.qadr["lift"]] + du[2], lo, hi)
        for s in ("R", "L"):
            for k, jn in enumerate(self.arm_joints[s]):
                lo, hi = self.limits[jn]
                q[self.qadr[jn]] = np.clip(
                    q[self.qadr[jn]] + du[self._u_arm0[s] + k], lo, hi)
        mujoco.mj_kinematics(self.model, self.data)

    def _weighted_err(self, targets, pos_w, ori_w):
        tot = 0.0
        for s, (p_t, R_t) in targets.items():
            p_cur, R_cur = self.tcp_pose(s)
            tot += pos_w * float(np.sum((p_t - p_cur) ** 2))
            tot += ori_w * float(np.sum(_mat_to_rotvec(R_t @ R_cur.T) ** 2))
        return tot

    def solve(self, targets, iters: int = 100,
              pos_tol: float = 5e-3, ori_tol_deg: float = 2.0,
              pos_w: float = 1.0, ori_w: float = 0.5,
              du_max: float = 0.25, **_ignored):
        """Levenberg–Marquardt solve to per-target tolerance.

        Full Gauss-Newton steps with adaptive damping + backtracking — unlike
        the streaming step(), convergence does not slow down near singular
        directions (the fixed-rate integrator crawls at σ²/(σ²+D) per tick).
        The motion-cost diagonal still shapes WHICH DoF absorb the motion.
        """
        lam = 1e-3
        D = self._cost_diag()
        for it in range(iters):
            J, e, w = self._task_matrices(targets, pos_w, ori_w)
            errs = {}
            for s, (p_t, R_t) in targets.items():
                p_cur, R_cur = self.tcp_pose(s)
                errs[s] = (float(np.linalg.norm(p_t - p_cur)),
                           float(np.degrees(np.linalg.norm(
                               _mat_to_rotvec(R_t @ R_cur.T)))))
            if all(ep < pos_tol and eo < ori_tol_deg for ep, eo in errs.values()):
                return {"success": True, "iters": it, "err": errs}
            W = np.diag(w)
            f0 = self._weighted_err(targets, pos_w, ori_w)
            q0 = self._get_q()
            improved = False
            for _try in range(6):
                du = np.linalg.solve(J.T @ W @ J + lam * np.diag(D), J.T @ W @ e)
                n = np.max(np.abs(du))
                if n > du_max:
                    du *= du_max / n
                self._apply_du(q0, du)
                if self._weighted_err(targets, pos_w, ori_w) < f0:
                    lam = max(lam * 0.5, 1e-5)
                    improved = True
                    break
                lam *= 6.0
            if not improved:
                self._apply_du(q0, np.zeros(self.nu))     # restore
                break
            # gentle null-space posture re-centering between LM steps —
            # skipped near convergence (the projection only cancels EE motion
            # to first order; close to the target its curvature drift shows
            # up as a mm-scale limit cycle)
            max_ep = max(np.linalg.norm(p_t - self.tcp_pose(s)[0])
                         for s, (p_t, _R) in targets.items())
            if max_ep < 0.03:
                continue
            J2, _e2, _w2 = self._task_matrices(targets, pos_w, ori_w)
            u_post = np.zeros(self.nu)
            for s in ("R", "L"):
                a0 = self._u_arm0[s]
                u_post[a0:a0 + 7] = np.clip(
                    0.1 * (self.home_arm[s] - self.arm_q(s)), -0.05, 0.05)
            if self.mode in ("arms+lift", "full"):
                u_post[2] = np.clip(-0.1 * self.lift_q(), -0.02, 0.02)
            Jp = J2.T @ np.linalg.solve(
                J2 @ J2.T + 1e-8 * np.eye(J2.shape[0]), J2)
            self._apply_du(self._get_q(),
                           (np.eye(self.nu) - Jp) @ u_post)
        errs = {}
        for s, (p_t, R_t) in targets.items():
            p_cur, R_cur = self.tcp_pose(s)
            errs[s] = (float(np.linalg.norm(p_t - p_cur)),
                       float(np.degrees(np.linalg.norm(
                           _mat_to_rotvec(R_t @ R_cur.T)))))
        return {"success": all(ep < pos_tol and eo < ori_tol_deg
                               for ep, eo in errs.values()),
                "iters": iters, "err": errs}

    def solve_multistart(self, targets, iters: int = 400, **kw):
        """solve() with goal-seeded restarts (standard for discrete reach:
        greedy velocity IK is path-dependent, so a lift/base seeded toward
        the target rescues targets the plain flow strands mid-basin).

        Restarts re-seed the LIFT toward the target height and, in full
        mode, the BASE toward the target x — arms always restart from home.
        """
        arm_home = {s: self.home_arm[s].copy() for s in ("R", "L")}
        z_t = float(np.mean([t[0][2] for t in targets.values()]))
        x_t = float(np.mean([t[0][0] for t in targets.values()]))
        lo, hi = self.limits["lift"]
        seeds = [dict(lift=0.0, base=(0.0, 0.0, 0.0))]
        if self.mode in ("arms+lift", "full"):
            seeds.append(dict(lift=float(np.clip(z_t - 0.95, lo, hi)),
                              base=(0.0, 0.0, 0.0)))
        if self.mode == "full":
            seeds.append(dict(lift=float(np.clip(z_t - 0.95, lo, hi)),
                              base=(max(0.0, x_t - 0.55), 0.0, 0.0)))
        best, best_sd = None, None
        for sd in seeds:
            self.set_home(arm_home, lift=sd["lift"], base=sd["base"])
            res = self.solve(targets, iters=iters, **kw)
            if res["success"]:
                return res
            worst = max(ep for ep, _ in res["err"].values())
            if best is None or worst < best[0]:
                best, best_sd = (worst, res), sd
        # restore the best seed's final state (self.data currently holds the
        # LAST seed's result, which callers would otherwise read)
        self.set_home(arm_home, lift=best_sd["lift"], base=best_sd["base"])
        return self.solve(targets, iters=iters, **kw)
