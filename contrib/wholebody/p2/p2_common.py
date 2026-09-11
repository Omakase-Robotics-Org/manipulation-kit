"""Shared helpers for the P2 ΔEE-action pipeline.

Frame / naming conventions (READ THIS — the classic D1 trap lives here):

- Dataset ``OmakaseAI/d1_teleop_joint_reviewed`` axes are named by PHYSICAL
  side: dims 0..7 = ``d1_left_arm_joint_1..7 + left_gripper``, dims 8..15 =
  right arm + right gripper.
- The d1-sdk URDF tree naming is flipped: SDK ArmSide A = URDF ``_R`` tree =
  PHYSICAL LEFT arm. So dataset LEFT block drives ``Joint1_R..Joint7_R`` and
  dataset RIGHT block drives ``Joint1_L..Joint7_L``.
- Gripper keeps the DATASET closedness convention (1.0 = closed) end to end.

ΔEE action layout (14-dim, same physical-side ordering as the source):

    [ L: Δx Δy Δz Δrx Δry Δrz grip | R: Δx Δy Δz Δrx Δry Δrz grip ]

with Δ = log( T_tcp(t)^-1 · T_tcp(t+1) ) split into translation (m, expressed
in the CURRENT gripper/TCP frame) and rotation vector (rad) — UMI-style
per-step relative motion, invariant to base motion. TCP poses come from FK of
``observation.state`` joints through the whole-body MuJoCo model with
lift = 0 and base = 0.
"""
from __future__ import annotations

import os
import sys

import numpy as np

_WB_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _WB_DIR)

from wb_ik import WholeBodyIK, _mat_to_rotvec  # noqa: E402

# dataset physical side -> URDF tree suffix (ArmSide A = URDF _R = phys LEFT)
SIDE_TO_URDF = {"left": "R", "right": "L"}
# dataset 16-dim layout
ARM_SLICE = {"left": slice(0, 7), "right": slice(8, 15)}
GRIP_IDX = {"left": 7, "right": 15}
# ΔEE 14-dim layout
DEE_SLICE = {"left": slice(0, 6), "right": slice(7, 13)}
DEE_GRIP_IDX = {"left": 6, "right": 13}

DEE_AXES = [f"{side}_{ax}" for side in ("left", "right")
            for ax in ("dx", "dy", "dz", "drx", "dry", "drz", "gripper")]


def rotvec_to_mat(v: np.ndarray) -> np.ndarray:
    """SO(3) exp map (Rodrigues)."""
    th = np.linalg.norm(v)
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1.0 - np.cos(th)) * (K @ K)


mat_to_rotvec = _mat_to_rotvec


class FKModel:
    """Batch FK: 16-dim dataset joint vectors -> per-side TCP poses.

    Uses the whole-body MuJoCo model with lift = 0, base = (0,0,0), so TCP
    poses are expressed in the robot base (base_footprint) frame.
    """

    def __init__(self):
        self.ik = WholeBodyIK(mode="arms")   # arms-only selector; FK is full
        self.ik.set_home(None, lift=0.0, base=(0.0, 0.0, 0.0))

    def tcp_from_state(self, q16: np.ndarray):
        """q16: (16,) dataset joint vector -> {"left": (p, R), "right": (p, R)}."""
        import mujoco
        ik = self.ik
        q = ik.data.qpos
        for side, urdf in SIDE_TO_URDF.items():
            for jn, v in zip(ik.arm_joints[urdf], q16[ARM_SLICE[side]]):
                q[ik.qadr[jn]] = float(v)
        mujoco.mj_kinematics(ik.model, ik.data)
        return {side: ik.tcp_pose(urdf) for side, urdf in SIDE_TO_URDF.items()}

    def tcp_traj(self, Q: np.ndarray):
        """Q: (T,16) -> dict side -> (pos (T,3), rot (T,3,3))."""
        T = Q.shape[0]
        out = {s: (np.empty((T, 3)), np.empty((T, 3, 3))) for s in SIDE_TO_URDF}
        for t in range(T):
            poses = self.tcp_from_state(Q[t])
            for s, (p, R) in poses.items():
                out[s][0][t] = p
                out[s][1][t] = R
        return out


def dee_from_traj(pos: np.ndarray, rot: np.ndarray) -> np.ndarray:
    """Per-step ΔEE from a TCP trajectory: (T,3),(T,3,3) -> (T,6).

    Row t = [Δxyz, Δrotvec] of T(t)^-1 T(t+1) in frame t; last row = 0.
    """
    T = pos.shape[0]
    out = np.zeros((T, 6))
    for t in range(T - 1):
        Rt = rot[t]
        out[t, :3] = Rt.T @ (pos[t + 1] - pos[t])
        out[t, 3:] = mat_to_rotvec(Rt.T @ rot[t + 1])
    return out


def integrate_dee(p0: np.ndarray, R0: np.ndarray, dee: np.ndarray):
    """Integrate (H,6) per-step ΔEE from an initial pose -> (H,3), (H,3,3).

    Returned poses are T(1..H): pose AFTER applying each successive delta.
    """
    H = dee.shape[0]
    pos = np.empty((H, 3))
    rot = np.empty((H, 3, 3))
    p, R = p0.copy(), R0.copy()
    for k in range(H):
        p = p + R @ dee[k, :3]
        R = R @ rotvec_to_mat(dee[k, 3:])
        pos[k] = p
        rot[k] = R
    return pos, rot


def rot_geodesic_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    return float(np.degrees(np.linalg.norm(mat_to_rotvec(Ra.T @ Rb))))
