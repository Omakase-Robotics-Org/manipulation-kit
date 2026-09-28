#!/usr/bin/env bash
# P2: run the full verifier suite against the final checkpoints.
# bash-floor v1: bash >= 4.4, or re-exec under the tree's resolved bash, or refuse (reports/deploy-scripts-bash32-silent-exit/).
case "${BASH_VERSION:-}" in [0-3].*|4.[0-3].*)
  _bash_floor="$(cd "$(dirname "$0")" 2>/dev/null && pwd -P)"
  while [ -n "$_bash_floor" ] && [ ! -x "$_bash_floor/.bash-floor/bin/bash" ]; do _bash_floor="${_bash_floor%/*}"; done
  if [ -n "$_bash_floor" ] && [ "${BASH_SOURCE[0]:-}" = "$0" ] && [ -f "$0" ] && [ "${WS_BASH_FLOOR_REEXEC:-}" != "$$" ]; then
    WS_BASH_FLOOR_REEXEC="$$" PATH="$_bash_floor/.bash-floor/bin:$PATH" exec "$_bash_floor/.bash-floor/bin/bash" "$0" "$@"
  fi
  echo "bash-floor: refused: ${BASH_SOURCE[0]:-$0} needs bash >= 4.4 and runs under bash ${BASH_VERSION} (${BASH:-?}); declare WS_BASH=<absolute path of a bash >= 4.4> (macOS: brew install bash)" >&2
  # shellcheck disable=SC2317 # exit is reached when this file is executed, return when it is sourced
  return 1 2>/dev/null || exit 1 ;;
esac

set -euo pipefail
cd "$(dirname "$0")/../.."
source /home/shu/credentials
export HF_TOKEN=$HUGGINGFACE_API_KEY MUJOCO_GL=egl
PY=.venv-p2/bin/python
RUNS=${RUNS_DIR:-/home/shu/data/p2_runs}
STEP=${STEP:-020000}
OUT=${OUT:-/home/shu/data/p2_eval}
mkdir -p "$OUT"

JOINT_CKPT="$RUNS/act_joint_baseline/checkpoints/$STEP/pretrained_model"
DEE_CKPT="$RUNS/act_dee_v0/checkpoints/$STEP/pretrained_model"
JOINT_ROOT=/home/shu/data/d1_teleop_joint_reviewed
DEE_ROOT=/home/shu/data/d1_teleop_dee_v0

echo "=== open-loop: joint baseline ==="
$PY wholebody/p2/eval_openloop.py --kind joint --ckpt "$JOINT_CKPT" \
    --root $JOINT_ROOT --out "$OUT/openloop_joint.json"
echo "=== open-loop: dee ==="
$PY wholebody/p2/eval_openloop.py --kind dee --ckpt "$DEE_CKPT" \
    --root $DEE_ROOT --out "$OUT/openloop_dee.json"
echo "=== wb rollout: oracle floor ==="
$PY wholebody/p2/rollout_wb.py --kind dee --oracle \
    --root $DEE_ROOT --out "$OUT/rollout_oracle.json"
echo "=== wb rollout: dee policy (free-running) ==="
$PY wholebody/p2/rollout_wb.py --kind dee --ckpt "$DEE_CKPT" \
    --root $DEE_ROOT --out "$OUT/rollout_dee.json"
echo "=== wb rollout: dee policy (chunk-anchored) ==="
$PY wholebody/p2/rollout_wb.py --kind dee --anchor_chunks --ckpt "$DEE_CKPT" \
    --root $DEE_ROOT --out "$OUT/rollout_dee_anchored.json"
echo "=== joint playback (comparison) ==="
$PY wholebody/p2/rollout_wb.py --kind joint --ckpt "$JOINT_CKPT" \
    --root $JOINT_ROOT --out "$OUT/rollout_joint.json"
echo "all evals done -> $OUT"
