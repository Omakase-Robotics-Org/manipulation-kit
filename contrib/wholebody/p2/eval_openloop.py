#!/usr/bin/env python3
"""Open-loop verifier for P2: joint-ACT baseline vs ΔEE-ACT.

On the held-out episodes (12,25,38,51,64,77,90,103 — never seen in training),
for chunk starts every `chunk_size` frames:

  1. feed the ground-truth observation (3 cams + 16-dim joint state) at t,
  2. predict one full action chunk,
  3. score it
     - NATIVE space: per-step MSE vs the GT action chunk (each policy in its
       own action space — not comparable across policies, reported for
       completeness),
     - COMMON space (EE pose, comparable): the predicted chunk is mapped to a
       TCP pose trajectory — ΔEE chunks are integrated from the GT TCP pose
       at t (UMI-style), joint chunks are mapped through FK — and compared to
       the GT TCP trajectory. Position error in cm, rotation geodesic in deg.

GT TCP reference: FK of observation.state (the executed trajectory). The
ΔEE supervision was derived from exactly this trajectory; the joint policy's
supervision (commanded joints) leads it by the teleop tracking lag, so for
the joint policy we ALSO report the error vs FK(GT commanded joints) — its
own supervision — to keep the comparison honest in both directions.

Only the LEFT arm moves in this dataset (all 108 episodes); right-arm errors
are ~0 by construction and are excluded from the headline numbers.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
import pandas as pd
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from p2_common import (  # noqa: E402
    ARM_SLICE, GRIP_IDX, DEE_GRIP_IDX, FKModel, integrate_dee, rot_geodesic_deg,
)

HELD_OUT = [12, 25, 38, 51, 64, 77, 90, 103]
OBS_KEYS = ["observation.images.base_0_rgb", "observation.images.left_wrist_0_rgb",
            "observation.images.right_wrist_0_rgb", "observation.state"]


def load_policy(ckpt: str):
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors
    from lerobot.configs.policies import PreTrainedConfig
    policy = ACTPolicy.from_pretrained(ckpt).eval().cuda()
    cfg = PreTrainedConfig.from_pretrained(ckpt)
    pre, post = make_pre_post_processors(cfg, pretrained_path=ckpt)
    return policy, pre, post


@torch.no_grad()
def predict_chunk(policy, pre, post, item) -> np.ndarray:
    batch = {k: item[k].unsqueeze(0) for k in OBS_KEYS}
    if "task" in item:
        batch["task"] = [item["task"]]
    batch = pre(batch)
    chunk = policy.predict_action_chunk(batch)          # (1, H, D) normalized
    chunk = post(chunk)                                  # unnormalized
    return chunk[0].float().cpu().numpy()


def episode_frames(root: str, ep: int) -> pd.DataFrame:
    files = sorted(__import__("pathlib").Path(root).glob("data/chunk-*/file-*.parquet"))
    for f in files:
        df = pd.read_parquet(f)
        if ep in df.episode_index.values:
            return df[df.episode_index == ep].reset_index(drop=True)
    raise KeyError(ep)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kind", choices=["joint", "dee"], required=True)
    ap.add_argument("--ckpt", required=True)
    ap.add_argument("--root", required=True)
    ap.add_argument("--out", default=None)
    ap.add_argument("--episodes", type=int, nargs="*", default=HELD_OUT)
    args = ap.parse_args()

    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    repo = "OmakaseAI/d1_teleop_dee_v0" if args.kind == "dee" else \
        "OmakaseAI/d1_teleop_joint_reviewed"
    policy, pre, post = load_policy(args.ckpt)
    H = policy.config.chunk_size
    fk = FKModel()

    native_se, grip_ae = [], []
    pos_err, rot_err = [], []          # vs FK(state) reference, left arm
    pos_err_cmd, rot_err_cmd = [], []  # joint only: vs FK(commanded joints)
    for ep in args.episodes:
        ds = LeRobotDataset(repo, root=args.root, episodes=[ep])
        df = episode_frames(args.root, ep)
        Q = np.stack(df["observation.state"].to_numpy()).astype(np.float64)
        A = np.stack(df["action"].to_numpy()).astype(np.float64)
        T = len(df)
        gt = fk.tcp_traj(Q)["left"]                       # (pos, rot) from state
        if args.kind == "joint":
            gt_cmd = fk.tcp_traj(A)["left"]               # FK of commanded joints
        for t in range(0, T - H, H):
            pred = predict_chunk(policy, pre, post, ds[t])[:H]
            gt_chunk = A[t:t + H]
            native_se.append(((pred - gt_chunk) ** 2).mean(axis=0))
            if args.kind == "dee":
                grip_ae.append(np.abs(pred[:, DEE_GRIP_IDX["left"]]
                                      - gt_chunk[:, DEE_GRIP_IDX["left"]]).mean())
                p_int, r_int = integrate_dee(gt[0][t], gt[1][t], pred[:, 0:6])
                ref_p, ref_r = gt[0][t + 1:t + 1 + H], gt[1][t + 1:t + 1 + H]
                pos_err += list(np.linalg.norm(p_int - ref_p, axis=1))
                rot_err += [rot_geodesic_deg(a, b) for a, b in zip(r_int, ref_r)]
            else:
                grip_ae.append(np.abs(pred[:, GRIP_IDX["left"]]
                                      - gt_chunk[:, GRIP_IDX["left"]]).mean())
                traj_pred = fk.tcp_traj(pred)["left"]
                # vs executed (state) trajectory, same t+1 alignment as ΔEE
                ref_p, ref_r = gt[0][t + 1:t + 1 + H], gt[1][t + 1:t + 1 + H]
                n = min(len(ref_p), H)
                pos_err += list(np.linalg.norm(traj_pred[0][:n] - ref_p, axis=1))
                rot_err += [rot_geodesic_deg(a, b)
                            for a, b in zip(traj_pred[1][:n], ref_r)]
                # vs its own supervision (commanded joints)
                refc_p, refc_r = gt_cmd[0][t:t + H], gt_cmd[1][t:t + H]
                pos_err_cmd += list(np.linalg.norm(traj_pred[0] - refc_p, axis=1))
                rot_err_cmd += [rot_geodesic_deg(a, b)
                                for a, b in zip(traj_pred[1], refc_r)]
        print(f"ep {ep}: {T} frames, {len(range(0, T - H, H))} chunks done")

    pe, re_ = np.array(pos_err) * 100.0, np.array(rot_err)
    res = {
        "kind": args.kind, "ckpt": args.ckpt, "episodes": args.episodes,
        "chunk_size": H, "n_steps_scored": len(pe),
        "native_action_mse_mean": float(np.mean(np.stack(native_se))),
        "native_action_mse_per_dim": np.stack(native_se).mean(axis=0).tolist(),
        "gripper_mae_left": float(np.mean(grip_ae)),
        "ee_pos_err_cm": {"mean": float(pe.mean()),
                          "p50": float(np.percentile(pe, 50)),
                          "p95": float(np.percentile(pe, 95)),
                          "max": float(pe.max())},
        "ee_rot_err_deg": {"mean": float(re_.mean()),
                           "p50": float(np.percentile(re_, 50)),
                           "p95": float(np.percentile(re_, 95)),
                           "max": float(re_.max())},
    }
    if pos_err_cmd:
        pc, rc = np.array(pos_err_cmd) * 100.0, np.array(rot_err_cmd)
        res["ee_pos_err_cm_vs_cmd"] = {"mean": float(pc.mean()),
                                       "p95": float(np.percentile(pc, 95))}
        res["ee_rot_err_deg_vs_cmd"] = {"mean": float(rc.mean()),
                                        "p95": float(np.percentile(rc, 95))}
    print(json.dumps(res, indent=2))
    if args.out:
        json.dump(res, open(args.out, "w"), indent=2)


if __name__ == "__main__":
    main()
