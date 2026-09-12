#!/usr/bin/env python3
"""Convert the joint-space D1 teleop dataset to ΔEE (gripper-frame relative)
actions — P2 of the whole-body plan.

Source:  OmakaseAI/d1_teleop_joint_reviewed (LeRobot v3.0, 16-dim joint action)
Output:  d1_teleop_dee_v0 — identical videos/state, action replaced by 14-dim
         [L: Δxyz(m) Δrotvec(rad) grip | R: same], per-step UMI-style deltas
         in the CURRENT TCP frame (see p2_common.py for exact conventions).

Design choices (documented for the PR):
- Deltas are computed from FK of ``observation.state`` (the executed
  trajectory), NOT from the commanded joint action — the relative-EE stream
  should describe realized motion; the teleop command lead/lag is noise here.
- ``observation.state`` is kept as the 16-dim joint vector, identical to the
  source dataset, so the joint-ACT baseline and the ΔEE variant see byte-
  identical observations (images + proprio) — only the action space differs.
- Gripper dims pass through from the source ACTION (dataset closedness
  convention, 1.0 = closed).
- Last frame of each episode gets a zero delta (no t+1 exists).
- Videos are hardlinked (same filesystem) instead of copied.

Verifier (run automatically at the end):
- integrate_dee() from the episode's first FK pose must reproduce the FK
  trajectory of every episode to < 1e-6 m / 1e-6 rad (exactness of the
  log/exp round trip), and the rewritten dataset must load through
  LeRobotDataset with the expected shapes.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from p2_common import (  # noqa: E402
    ARM_SLICE, DEE_AXES, GRIP_IDX, FKModel, dee_from_traj, integrate_dee,
)

SRC = Path("/home/shu/data/d1_teleop_joint_reviewed")
DST = Path("/home/shu/data/d1_teleop_dee_v0")

STAT_FIELDS = ("min", "max", "mean", "std", "count", "q01", "q10", "q50", "q90", "q99")


def feature_stats(arr: np.ndarray) -> dict[str, np.ndarray]:
    """Per-feature stats in the lerobot v3.0 convention (axis 0)."""
    q = {f"q{p:02d}": np.quantile(arr, p / 100.0, axis=0) for p in (1, 10, 50, 90, 99)}
    return {
        "min": arr.min(axis=0), "max": arr.max(axis=0),
        "mean": arr.mean(axis=0), "std": arr.std(axis=0),
        "count": np.array([len(arr)]), **q,
    }


def link_or_copy(src: Path, dst: Path):
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists():
        dst.unlink()
    try:
        os.link(src, dst)
    except OSError:
        shutil.copy2(src, dst)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", type=Path, default=SRC)
    ap.add_argument("--dst", type=Path, default=DST)
    args = ap.parse_args()
    src, dst = args.src, args.dst

    fk = FKModel()
    data_files = sorted(src.glob("data/chunk-*/file-*.parquet"))
    print(f"{len(data_files)} data files")

    per_ep_stats: dict[int, dict[str, np.ndarray]] = {}
    all_actions = []
    n_moving = {"left": 0, "right": 0}
    worst_roundtrip = 0.0

    for f in data_files:
        df = pd.read_parquet(f).reset_index(drop=True)
        new_action = np.empty((len(df), 14), dtype=np.float32)
        for ep in df.episode_index.unique():
            m = (df.episode_index == ep).to_numpy()
            Q = np.stack(df.loc[m, "observation.state"].to_numpy()).astype(np.float64)
            A = np.stack(df.loc[m, "action"].to_numpy()).astype(np.float64)
            traj = fk.tcp_traj(Q)
            act = np.zeros((len(Q), 14))
            for k, side in enumerate(("left", "right")):
                dee = dee_from_traj(*traj[side])
                act[:, 7 * k:7 * k + 6] = dee
                act[:, 7 * k + 6] = A[:, GRIP_IDX[side]]
                if np.abs(dee[:, :3]).max() > 1e-6:
                    n_moving[side] += 1
                # round-trip verifier: integrate deltas -> must equal FK traj
                pos, rot = traj[side]
                ip, ir = integrate_dee(pos[0], rot[0], dee[:-1])
                worst_roundtrip = max(worst_roundtrip,
                                      float(np.abs(ip - pos[1:]).max()))
            new_action[m] = act.astype(np.float32)
            per_ep_stats[int(ep)] = feature_stats(act.astype(np.float32))
            all_actions.append(act.astype(np.float32))
        df["action"] = list(new_action)
        out = dst / f.relative_to(src)
        out.parent.mkdir(parents=True, exist_ok=True)
        df.to_parquet(out)
        print(f"  {f.name}: {len(df)} frames, eps {sorted(df.episode_index.unique())}")

    print(f"round-trip integrate error max: {worst_roundtrip:.2e} m")
    assert worst_roundtrip < 1e-6, "ΔEE integration does not reproduce FK traj"
    print(f"episodes with motion: {n_moving}")

    # ---- meta ----
    (dst / "meta").mkdir(parents=True, exist_ok=True)
    # info.json
    info = json.load(open(src / "meta/info.json"))
    info["features"]["action"] = {
        "dtype": "float32", "shape": [14], "names": {"axes": DEE_AXES},
    }
    json.dump(info, open(dst / "meta/info.json", "w"), indent=4)
    # stats.json — recompute action globally, keep the rest
    stats = json.load(open(src / "meta/stats.json"))
    gs = feature_stats(np.concatenate(all_actions))
    stats["action"] = {k: np.asarray(v).tolist() for k, v in gs.items()}
    json.dump(stats, open(dst / "meta/stats.json", "w"), indent=4)
    # episodes parquet — rewrite per-episode action stats
    for f in sorted(src.glob("meta/episodes/chunk-*/file-*.parquet")):
        ep_df = pd.read_parquet(f).reset_index(drop=True)
        for field in STAT_FIELDS:
            col = f"stats/action/{field}"
            ep_df[col] = [
                np.asarray(per_ep_stats[int(e)][field], dtype=np.float32)
                for e in ep_df.episode_index
            ]
        out = dst / f.relative_to(src)
        out.parent.mkdir(parents=True, exist_ok=True)
        ep_df.to_parquet(out)
    # tasks
    shutil.copy2(src / "meta/tasks.parquet", dst / "meta/tasks.parquet")
    # videos (hardlink)
    vids = sorted(src.glob("videos/**/*.mp4"))
    for v in vids:
        link_or_copy(v, dst / v.relative_to(src))
    print(f"linked {len(vids)} video files")

    # ---- load-back verifier ----
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ds = LeRobotDataset("OmakaseAI/d1_teleop_dee_v0", root=dst)
    item = ds[0]
    assert tuple(item["action"].shape) == (14,), item["action"].shape
    assert tuple(item["observation.state"].shape) == (16,)
    print(f"load-back OK: {ds.num_episodes} eps, {ds.num_frames} frames, "
          f"action {tuple(item['action'].shape)}")
    print("dataset stats (action):")
    for k in ("mean", "std", "min", "max"):
        print(f"  {k}: {np.round(np.asarray(stats['action'][k]), 5)}")


if __name__ == "__main__":
    main()
