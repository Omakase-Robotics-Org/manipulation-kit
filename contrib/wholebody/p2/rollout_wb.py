#!/usr/bin/env python3
"""P2 closed-loop-ish sanity: execute ΔEE-ACT chunks through WholeBodyIK.

For each held-out episode:

- ``dee`` kind: initialize the whole-body model ("full" mode: arms + lift +
  base) at the episode's first joint state. At every chunk boundary, feed the
  GROUND-TRUTH observation (dataset images + joint state at frame t) to the
  policy, then execute the predicted ΔEE chunk sequentially: the running
  target pose is advanced by each predicted delta FROM THE CURRENT ROLLOUT
  TARGET (as on a real robot — deltas compose on the executed pose, so
  prediction errors compound within and across chunks), and the streaming
  whole-body controller ``step()`` tracks it at 30 Hz. Metric: realized TCP
  vs the dataset's FK(state) TCP trajectory.

- ``joint`` kind (comparison): the predicted joint chunk is applied directly
  to the arm joints (the baseline does not need an IK layer) and scored with
  the same metric.

LIMITATION (stated up front): this barebones MuJoCo model has no cameras and
no objects, so the policy cannot be rolled out on its own visual feedback —
observations come from the dataset (open-loop chunk execution). What this
DOES verify is the P2 execution contract: ΔEE chunks --compose--> pose
targets --WholeBodyIK.step()--> whole-body motion that stays on the
demonstrated trajectory. True closed-loop needs the real robot
(d1-inference) — see README.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from p2_common import (  # noqa: E402
    ARM_SLICE, SIDE_TO_URDF, FKModel, rotvec_to_mat, rot_geodesic_deg,
)
from eval_openloop import HELD_OUT, episode_frames, load_policy, predict_chunk  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from wb_ik import WholeBodyIK  # noqa: E402

FPS = 30.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kind", choices=["joint", "dee"], required=True)
    ap.add_argument("--ckpt", default="(oracle)")
    ap.add_argument("--root", required=True)
    ap.add_argument("--out", default=None)
    ap.add_argument("--episodes", type=int, nargs="*", default=HELD_OUT)
    ap.add_argument("--substeps", type=int, default=2,
                    help="controller step() calls per 30Hz frame")
    ap.add_argument("--oracle", action="store_true",
                    help="execute GROUND-TRUTH action chunks instead of policy "
                         "predictions (controller tracking floor, no policy)")
    ap.add_argument("--anchor_chunks", action="store_true",
                    help="re-anchor the running pose target to the GT pose at "
                         "every chunk boundary — models a deployment whose "
                         "feedback (visual servoing / re-observation) corrects "
                         "accumulated drift between chunks; isolates "
                         "WITHIN-chunk error from cross-chunk compounding")
    args = ap.parse_args()

    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    repo = "OmakaseAI/d1_teleop_dee_v0" if args.kind == "dee" else \
        "OmakaseAI/d1_teleop_joint_reviewed"
    if args.oracle:
        policy, pre, post, H = None, None, None, 50
    else:
        policy, pre, post = load_policy(args.ckpt)
        H = policy.config.chunk_size
    fk = FKModel()

    pos_err, rot_err = [], []          # realized TCP vs FK(state), left arm
    per_ep = {}
    for ep in args.episodes:
        ds = None if args.oracle else LeRobotDataset(repo, root=args.root, episodes=[ep])
        df = episode_frames(args.root, ep)
        Q = np.stack(df["observation.state"].to_numpy()).astype(np.float64)
        A = np.stack(df["action"].to_numpy()).astype(np.float64)
        T = len(df)
        gt = fk.tcp_traj(Q)                              # per-side FK(state)

        ep_pos, ep_rot = [], []
        if args.kind == "dee":
            ik = WholeBodyIK(mode="full")
            ik.set_home({SIDE_TO_URDF[s]: Q[0][ARM_SLICE[s]] for s in SIDE_TO_URDF},
                        lift=0.0, base=(0.0, 0.0, 0.0))
            # running targets start at the model's own TCP poses
            tgt = {s: [p.copy() for p in ik.tcp_pose(SIDE_TO_URDF[s])]
                   for s in SIDE_TO_URDF}
            for t0 in range(0, T - H, H):
                pred = A[t0:t0 + H] if args.oracle else \
                    predict_chunk(policy, pre, post, ds[t0])[:H]
                if args.anchor_chunks:
                    tgt = {s: [gt[s][0][t0].copy(), gt[s][1][t0].copy()]
                           for s in SIDE_TO_URDF}
                for k in range(H):
                    for i, s in enumerate(("left", "right")):
                        d = pred[k, 7 * i:7 * i + 6]
                        p, R = tgt[s]
                        tgt[s] = [p + R @ d[:3], R @ rotvec_to_mat(d[3:])]
                    v_ff = {SIDE_TO_URDF[s]: (tgt[s][0] - ik.tcp_pose(SIDE_TO_URDF[s])[0]) * FPS
                            for s in SIDE_TO_URDF}
                    for _ in range(args.substeps):
                        ik.step({SIDE_TO_URDF[s]: tuple(tgt[s]) for s in SIDE_TO_URDF},
                                dt=1.0 / (FPS * args.substeps), gain=8.0, ff=v_ff)
                    p_real, R_real = ik.tcp_pose(SIDE_TO_URDF["left"])
                    ref_p, ref_R = gt["left"][0][t0 + k + 1], gt["left"][1][t0 + k + 1]
                    ep_pos.append(np.linalg.norm(p_real - ref_p))
                    ep_rot.append(rot_geodesic_deg(R_real, ref_R))
            base_final = ik.base_pose()
            lift_final = ik.lift_q()
        else:
            for t0 in range(0, T - H, H):
                pred = A[t0:t0 + H] if args.oracle else \
                    predict_chunk(policy, pre, post, ds[t0])[:H]
                traj = fk.tcp_traj(pred)["left"]
                for k in range(min(H, T - 1 - t0)):
                    ep_pos.append(np.linalg.norm(traj[0][k] - gt["left"][0][t0 + k + 1]))
                    ep_rot.append(rot_geodesic_deg(traj[1][k], gt["left"][1][t0 + k + 1]))
            base_final, lift_final = None, None
        pos_err += ep_pos
        rot_err += ep_rot
        per_ep[ep] = {
            "pos_cm_mean": float(np.mean(ep_pos) * 100),
            "pos_cm_p95": float(np.percentile(ep_pos, 95) * 100),
            "rot_deg_mean": float(np.mean(ep_rot)),
            **({"base_final_xy_yaw": [round(float(x), 4) for x in base_final],
                "lift_final_m": round(float(lift_final), 4)} if base_final is not None else {}),
        }
        print(f"ep {ep}: pos mean {per_ep[ep]['pos_cm_mean']:.2f} cm, "
              f"p95 {per_ep[ep]['pos_cm_p95']:.2f} cm, "
              f"rot mean {per_ep[ep]['rot_deg_mean']:.2f} deg")

    pe, re_ = np.array(pos_err) * 100.0, np.array(rot_err)
    res = {
        "kind": args.kind, "ckpt": args.ckpt, "episodes": args.episodes,
        "controller": "WholeBodyIK full (arms+lift+base)" if args.kind == "dee"
        else "direct joint playback (no IK)",
        "n_steps": len(pe),
        "ee_pos_err_cm": {"mean": float(pe.mean()),
                          "p50": float(np.percentile(pe, 50)),
                          "p95": float(np.percentile(pe, 95)),
                          "max": float(pe.max())},
        "ee_rot_err_deg": {"mean": float(re_.mean()),
                           "p95": float(np.percentile(re_, 95))},
        "per_episode": per_ep,
    }
    print(json.dumps({k: v for k, v in res.items() if k != "per_episode"}, indent=2))
    if args.out:
        json.dump(res, open(args.out, "w"), indent=2)


if __name__ == "__main__":
    main()
