#!/usr/bin/env bash
# P2: train ACT-joint (baseline) and ACT-ΔEE on identical budgets.
# Both runs: 100 train episodes (8 held out: 12,25,38,51,64,77,90,103),
# chunk 50, batch 16, 20k steps, seed 1000, wandb project d1-wholebody-p2.
set -euo pipefail
cd "$(dirname "$0")/../.."   # d1-sdk root
source /home/shu/credentials
export HF_TOKEN=$HUGGINGFACE_API_KEY
export WANDB_API_KEY

HELD_OUT="12,25,38,51,64,77,90,103"
TRAIN_EPS=$(python3 -c "print([i for i in range(108) if i not in ($HELD_OUT)])")
RUNS_DIR=${RUNS_DIR:-/home/shu/data/p2_runs}
STEPS=${STEPS:-20000}

common=(
  --dataset.episodes="$TRAIN_EPS"
  --policy.type=act --policy.chunk_size=50 --policy.n_action_steps=50
  --policy.push_to_hub=false --policy.device=cuda
  --batch_size=16 --num_workers=8 --steps="$STEPS" --seed=1000
  --log_freq=200 --save_freq=10000 --save_checkpoint=true
  --wandb.enable=true --wandb.project=d1-wholebody-p2 --wandb.entity=omakase-robotics
)

echo "=== [1/2] ACT-joint baseline ==="
.venv-p2/bin/lerobot-train \
  --dataset.repo_id=OmakaseAI/d1_teleop_joint_reviewed \
  --dataset.root=/home/shu/data/d1_teleop_joint_reviewed \
  --job_name=act_joint_baseline \
  --output_dir="$RUNS_DIR/act_joint_baseline" \
  "${common[@]}"

echo "=== [2/2] ACT-ΔEE ==="
.venv-p2/bin/lerobot-train \
  --dataset.repo_id=OmakaseAI/d1_teleop_dee_v0 \
  --dataset.root=/home/shu/data/d1_teleop_dee_v0 \
  --job_name=act_dee_v0 \
  --output_dir="$RUNS_DIR/act_dee_v0" \
  "${common[@]}"

echo "=== both runs done ==="
