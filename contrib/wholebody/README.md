# D1 whole-body IK (arms + lift + differential-drive base)

Sunday-Robotics-style "everything flows" coordination for D1: the policy/
teleop commands an EE pose, and a whole-body controller decides how much
arm / lift / base to use. Phases (Shu, 2026-07-17):

- **P0** — lift (0.30 m prismatic) in the IK chain → `bench_p0.py`
- **P1** — differential-drive base in the chain (nonholonomic) → `demo_p1.py`
- **P2** — ΔEE-action ACT trained on teleop data, executed through this
  controller (separate work, not in this dir yet)

## Model

`description/d1/d1_wholebody.urdf` (or `d1_wholebody_gripper.urdf` for the
same robot wearing the D1 stock parallel gripper instead of the YUBI hand —
identical everything above the tool flange):

- `world → base_x → base_y → base_yaw → base_footprint` (planar base at
  ground level; chassis geometry fixed to it)
- `base_footprint → lift (prismatic z, 0…0.300) → dual_base →
  torso/head/arms` — straight from the vendor body URDF `urdf2026072302`:
  `q_lift = 0` is the retracted rail with `dual_base` 0.52678 m above the
  floor, effort 80 N, 0.03 m/s. The 0.300 stroke matches the spec sheet
  (total height 1293–1593 mm).
- `torso_column → neck_pan (±1.57, axis −z) → neck_tilt (−0.18…+0.65) →
  head_link` — the vendor PTU. **Axis signs are unverified on hardware**:
  the vendor CSV contradicts its own URDF on the tilt (and on the lift);
  the URDF is taken as authoritative. `d1.urdf` keeps a single static
  `head_link` instead, because the guard's body boxes must stay
  world-axis-aligned.

## Solver (`wb_ik.py`)

Reduced velocity space `u = [v, w, dlift, qdot_R(7), qdot_L(7)]` with
`[xdot, ydot, yawdot] = [v cosθ, v sinθ, w]` — the diff-drive no-side-slip
constraint holds **by construction** (verified 1e-15 in demo_p1).

- **Streaming `step()`** (teleop/tracking, 50 Hz): weighted **least-norm**
  velocity distribution — realize the commanded EE velocity, let per-DoF
  motion costs (arm 1 / lift ~4 / base ~6–30) decide who moves. Supports
  reference-velocity feedforward. NOT ridge-damped: ridge attenuates the
  realized velocity by σ²/(σ²+D) and the tracker lags a moving reference.
- **Discrete `solve()`** (reach targets): Levenberg–Marquardt with
  backtracking — converges in 2–20 iterations where a fixed-rate velocity
  integrator crawls near low-σ directions. `solve_multistart()` adds
  goal-seeded lift/base restarts (greedy flows are path-dependent).
- Posture (arms+lift home-pull) acts in the task **null space** only, scaled
  away near convergence (first-order projection leaves curvature drift).

## P0 result (bench_p0.py, 14 targets × 3 chains)

Reach success (pos <5 mm, ori <2°, height-realistic tool pitch):

| chain | reached |
|---|---|
| arms only | 5/14 |
| **arms + lift** | **7/14** (superset of arms) |
| **+ base (full)** | **12/14** |

Only the near-floor rows (z=0.35) stay unreachable — lift bottomed, wrist
limits bind: a genuine hardware envelope, not a solver failure.

## P1 result (demo_p1.py)

2.4 m forward traverse + 0.30 m height sweep + lateral weave @ 0.25 m/s,
one continuous stream — base, lift and arm move simultaneously:

- steady-state tracking mean **7.9 mm**, p95 28.5 mm (the p95 comes from the
  down-reach phase z≈0.80 where the streaming envelope binds; static reach
  there is fine — reconfiguring while cruising is the hard part)
- lateral base slip ≤ 2e-15 m/s (exact, by construction)
- video: `out/p1_traverse_fullbody.mp4`

## Known limits / next

- Down-forward reach (z<0.7 while moving) binds on wrist limits (J6 ±60°) +
  lift bottom — relevant to low-table tasks; worth checking against the
  real lift's low end.
- Velocity limits (lift 0.1 m/s, base accel) are not yet enforced in
  `step()` — add before driving hardware.
- P2: export ΔEE (gripper-frame relative) actions from d1_teleop data,
  train ACT, execute through this controller.

Run: `MUJOCO_GL=egl .venv-wb/bin/python wholebody/bench_p0.py` (or demo_p1).
Deps: `mujoco numpy imageio[ffmpeg]`.
