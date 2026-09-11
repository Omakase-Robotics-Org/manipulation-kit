# P2 — ΔEE-action ACT vs joint-action ACT (whole-body plan)

P2 of the whole-body plan (P0/P1 = `wholebody/wb_ik.py`, PR #35): train ACT
on **ΔEE actions** — per-step relative TCP motion expressed in the current
gripper frame (UMI-style, base-motion invariant) — and prove it against the
standard joint-space ACT baseline on the same data, same budget. ΔEE is the
action space that lets a policy drive the P0/P1 whole-body controller
(arms + lift + base) instead of being welded to arm joints.

All-local PoC on an RTX 5080 (16 GB).

## Action space

Per arm `[Δx Δy Δz Δrx Δry Δrz grip]`, both arms → 14-dim:

- `Δ = log( T_tcp(t)^-1 · T_tcp(t+1) )` — translation (m) and rotation
  vector (rad) in the CURRENT vendor-TCP frame; poses from FK of
  `observation.state` through the whole-body MuJoCo model (lift 0, base 0).
- Gripper passes through from the source action, dataset closedness
  convention (1.0 = closed).
- Observations (3 cams + 16-dim joint state) are byte-identical to the
  baseline — only the action space differs.
- Naming trap honored throughout: dataset "left" (physical left) drives the
  URDF `_R` tree (SDK ArmSide A = URDF `_R` = physical LEFT).

## Data

`OmakaseAI/d1_teleop_joint_reviewed` (LeRobot v3.0): 108 episodes, 61,883
frames @ 30 fps, 3× 480×640 cams, 16-dim joint state/action. All 108
episodes move ONLY the left arm (right-arm ΔEE dims are identically zero;
right gripper carries VR-toggle noise and is passed through as-is).
Converted dataset: `OmakaseAI/d1_teleop_dee_v0` (videos hardlinked/copied,
action + stats rewritten). Conversion verifier: integrating the ΔEE stream
reproduces the FK trajectory to 1.3e-15 m.

Per-step left-arm deltas: |Δxyz| mean ≈ 2.7 mm (p95 ≈ 10 mm), |Δrot|
mean ≈ 0.3°.

## Training (identical budgets)

`train_both.sh`: LeRobot 0.6.0 ACT, chunk 50, batch 16, 20k steps, seed
1000, 100 train episodes / 8 held out (12, 25, 38, 51, 64, 77, 90, 103),
~83 min per run.

| run | wandb |
|---|---|
| ACT-joint baseline | https://wandb.ai/omakase-robotics/d1-wholebody-p2/runs/p962cp03 |
| ACT-ΔEE | https://wandb.ai/omakase-robotics/d1-wholebody-p2/runs/y117wul5 |

(The baseline run was killed by the environment at step 14.3k and resumed
from the 10k checkpoint into the same wandb run — final budget is the same
20k steps; final training losses: joint 0.051, ΔEE 0.19 — not comparable
across action spaces, both normalized L1+KL.)

## Verifier results (defined before training)

### Open-loop, held-out episodes (`eval_openloop.py`)

GT observation at every 50-frame chunk boundary → predict one chunk → map
to a common EE space (ΔEE chunks integrated from the GT pose at the chunk
start; joint chunks through FK) → compare against the FK(state) TCP
trajectory. Left arm; N = 4050 steps.

| metric | ACT-joint | ACT-ΔEE |
|---|---|---|
| EE pos err mean (cm) | 5.07 | **4.84** |
| EE pos err p95 (cm) | 12.64 | 13.28 |
| EE rot err mean (deg) | 7.67 | **6.49** |
| EE rot err p95 (deg) | 18.26 | 19.44 |
| gripper MAE (left) | 0.067 | 0.068 |
| native-space action MSE | 1.11e-2 rad² | 1.49e-2 (mixed m/rad/grip) |

Notes: native MSE is not comparable across action spaces (reported for
completeness). The joint baseline is supervised on COMMANDED joints, which
lead the executed state by the teleop lag — against its own supervision
(FK of GT commanded joints) it scores 5.03 cm mean / 12.55 cm p95.

### Whole-body execution (`rollout_wb.py`)

Predicted ΔEE chunks composed into a running pose target and tracked by
`WholeBodyIK.step()` (full mode: arms + lift + base) at 30 Hz; realized TCP
vs the demonstrated trajectory. Observations still come from the dataset
(see limitations). Same held-out episodes.

| rollout | pos mean (cm) | pos p95 (cm) | rot mean (deg) |
|---|---|---|---|
| GT ΔEE through controller (oracle floor) | 0.07 | 0.24 | 1.4 |
| ACT-ΔEE, chunk-anchored (`--anchor_chunks`) | 4.87 | 13.30 | 7.1 |
| ACT-ΔEE, free-running (no re-anchoring) | 18.36 | 59.80 | 18.1 |
| ACT-joint direct playback (no IK) | 5.07 | 12.64 | 7.7 |

Reading the table:

- **Oracle** is the P2 execution contract in isolation: ΔEE stream →
  compose → whole-body controller stays within a millimeter of the
  demonstrated EE trajectory while quietly recruiting lift/base (final base
  offsets of a few cm — redundancy resolution, not error).
- **Chunk-anchored** re-anchors the running target to the measured GT pose
  at each 50-step chunk boundary (what any deployment with feedback-driven
  drift correction effectively does). It lands at 4.87 cm — the open-loop
  number plus a negligible controller cost, i.e. ΔEE execution through the
  whole-body controller costs almost nothing over the policy's own error.
- **Free-running** composes every predicted delta onto the previous target
  with no correction ever. Relative actions accumulate drift by
  construction, and because our sim feeds ground-truth images, the policy
  cannot see (much less correct) its own drift — an artifact of the eval,
  the known failure mode of relative action spaces without closed-loop
  feedback, and the strongest argument that the real-robot rollout must be
  closed-loop (UMI-style ΔEE is deployed exactly that way).
- The joint baseline predicts absolute targets, so its playback error
  cannot accumulate: its row equals its open-loop error by construction.

## Reproduce

```sh
.venv-p2/bin/python wholebody/p2/make_dee_dataset.py
wholebody/p2/train_both.sh
wholebody/p2/run_all_evals.sh
```

Venv: `.venv-p2` = lerobot 0.6.0 + torch 2.11 cu128 + mujoco 3.10
(RTX 5080 / sm_120 needs the cu128 wheels).

## Limitations / next steps

- **No visual closed loop.** The barebones whole-body MuJoCo model has no
  cameras or objects, so rollouts feed ground-truth dataset observations at
  chunk boundaries (open-loop chunk execution). Prediction errors compound
  through the composed pose target exactly as they would on the robot, but
  the policy never sees the consequences of its own actions. Real rollout
  needs d1-inference integration (ΔEE → `WholeBodyIK.step()` → joint
  streaming) on the robot.
- Single task, single arm moving, 108 episodes — this is a representation
  comparison, not a capability claim.
- ΔEE supervision here is derived from executed states; a real deployment
  should decide whether to supervise on commanded or executed motion
  (teleop lag ≈ the gap between the two joint-baseline rows above).
- Right-arm action dims are constant zero in this dataset; their normalized
  targets are degenerate (harmless for ACT's mean/std scheme, but a
  bimanual dataset is needed before any claim about the right arm).
- The whole-body controller's velocity limits are not enforced yet
  (P0/P1 known limit) — required before hardware execution.
