# `manipulation_kit.arms` — arm-side kinematics, drivers and transports

Sibling of `manipulation_kit.hands` in this repo. Layout mirrors it exactly:

```
arms/<maker>/<model>/            # importable as manipulation_kit.arms.<maker>.<model>
    kinematics.py                # build_kinematics() + clutch_tuning() factories
    tests/
arms/*.py                        # the arm-AGNOSTIC layer (solver, clamps, conventions)
```

| path | arm | status |
|---|---|---|
| `arms/d1/arm/` | the D1 arm (as mounted on D1) | kinematics migrated from `dx-vr-teleop`; driver still in `d1-sdk` |

## Why arms live here

Two reasons, pointing the same way:

1. **The flange is shared.** A hand on the D1 arm flange has no bus of its own —
   it is reachable only through the arm's CANFD end-module passthrough. That
   bridge belongs to neither side alone.
2. **Every consumer that moves the arm needs the same kinematics.** VR teleop
   (`dx-vr-teleop`), the runtime that starts teleop mid-conversation
   (`omakase-core`) and episode replay (`d1-inference`) all need the same IK, the
   same collision gate, the same clamps. Kept inside one robot stack, the others
   had to copy it — and **a duplicated safety limiter is strictly worse than one
   shared limiter**, because when the copies disagree nobody can say which one
   the robot is actually running.

That is not hypothetical. Before this package existed, the per-tick joint cap was
`0.25 rad` in `dx-vr-teleop` and `0.05 rad` in `omakase-core` — a 5x difference
in a safety clamp, arrived at purely because the value was copied instead of
shared. See `safety.MAX_JOINT_STEP_RAD`.

## The seam

Consumers resolve by `"<maker>/<model>"`, exactly like `get_hand` /
`get_tool_config`, and never import a model directly:

```python
from manipulation_kit.arms import get_arm_kinematics, get_clutch_tuning
from manipulation_kit.arms.targeting import ArmClutch

arm = get_arm_kinematics("d1/arm")        # guard auto-loaded from $D1_SDK_DIR
clutch = ArmClutch(tuning=get_clutch_tuning("d1/arm"))

# on engage (grip toggle, conversation intent, whatever the consumer decides)
clutch.engage(ctrl_p, ctrl_q, *arm.ee_pose("left"))

# per tick
target = clutch.target(ctrl_p, ctrl_q)          # None => parked, hold
if target is not None:
    res = arm.solve_ee("left", *target)         # IK + step clamp + collision gate
    if res.ok:
        clutch.accept(*target)
        send_to_robot(res.q)                    # <- consumer's transport
    else:
        clutch.reject()                         # hold the last safe pose
```

Already have a `MotionGuard`? Inject it — don't let a second one be built behind
your back: `get_arm_kinematics("d1/arm", guard=my_guard)`.

No MuJoCo? `get_clutch_tuning` and everything in `targeting`/`filters`/`frames`
work without it, and the solver takes any `ik.KinematicChain` — see
"Substrates" below.

## What is here, and what is deliberately not

**The library ends where the wire begins.** Everything here computes on a model
and returns numbers. It opens no connection, holds no robot lock, owns no socket
and streams nothing.

| module | contents |
|---|---|
| `safety.py` | every clamp / IK / workspace constant, with provenance — one source of truth |
| `sides.py` | the logical / URDF / SDK side crossover (`left` = `_R` = A = physical LEFT) |
| `ik.py` | `KinematicChain` protocol, DLS IK with null-space posture bias, joint-step clamp, READY-seed search |
| `kinematics.py` | `ArmKinematics` protocol, `IkResult`, `GuardedArm` (solve → clamp → collision-gate → commit) |
| `guard.py` | `CollisionGuard` protocol, fail-closed rad→deg gate, lazy `pyguard` loader |
| `targeting.py` | `ArmClutch` — relative anchoring, workspace box, step clamp, parking deadband |
| `filters.py` | One-Euro pose/joint filters |
| `frames.py` | WebXR ↔ robot base frame |

**Not here (yet), on purpose:**

- **Arm mode sequencing** — impedance/compliance entry and exit, gravity
  compensation, servo enable, tool registration on the controller. These are
  sequences of vendor SDK calls against a *live connection*, they were repaired
  in `dx-vr-teleop` PR #41 only on 2026-07-29, and they cannot be verified off
  the robot. They should stabilise in the field before being enshrined as the
  shared implementation. This is the natural phase 1b.
- **The streaming path** — `set_joint_cmd_pose` and the per-tick "warp clamp"
  guarding it, whose reference is *what was last put on the wire*: transport
  state by definition.
- **Session semantics** — who owns the arm, grip-button decoding, WebSocket/HUD
  plumbing, episode recording. `omakase-core`'s arbiter/ownership layer is the
  right home for the first of those and is not duplicated anywhere.

## Substrates

`ik.solve_ik` talks to a `KinematicChain` — something posable, that can then be
asked for its EE pose, its Jacobian and its joint limits. This is not
gratuitous abstraction; the substrates genuinely differ:

| `chain=` | substrate | why |
|---|---|---|
| `"urdf"` (**default**) | `arms/urdf_chain.py` — the URDF the guard already parses, Rodrigues FK + geometric Jacobian, numpy only | inverse kinematics must not require a physics engine. Every consumer that only computes (`omakase-core`, `d1-inference`, `poc-dx-inspect-robots`) gets IK with `pip install manipulation-kit` and nothing else |
| `"mujoco"` | `arms/d1/arm/mujoco_chain.py` — `mj_jacBody` on a shared `MjModel`/`MjData` | `dx-vr-teleop` already carries a full MuJoCo mirror (viewer, sim backend, composed hand model); pointing the solver at **that** model avoids running a second one beside it |

Welding the solver to either one would leave the other unable to adopt it — and
the solver is the part that carries the tuning and the safety behaviour.
`tests/test_substrate_independence.py` implements a chain in plain numpy and
drives the shared solver with it, so "a non-MuJoCo consumer can adopt this" is a
test rather than a promise.

The two are not merely interchangeable in principle — they are measured against
each other. `tests/arms/test_urdf_chain_parity.py` sweeps 500 random in-limit
postures per side and compares FK, the 6x7 Jacobian, the joint limits, the
probe link positions, the 200-restart READY seed and `solve_ee` over a full
trajectory. Worst observed difference on `d1.urdf` is ~5e-16 m / ~1.4e-15 rad /
~3e-15 on the Jacobian: double-precision noise, not a different model.

## Named poses: `home`, `home_clear`, `stow`

The arm poses the kit ships live in `config/` as one file each, in
`home_pose.json`'s format (14 degrees, A1..A7 then B1..B7, mirror-symmetric),
and are read by name through
`manipulation_kit.arms.d1.arm.kinematics.load_named_pose(name)`
(`NAMED_POSES`). Nobody copies the numbers.

| name | file | what it is |
|---|---|---|
| `home` | `home_pose.json` | THE HOME: the measured wrist-forward rest pose. `D1ArmKinematics` starts here, `go_home` ramps here, gestures pin it as their first and last keyframe |
| `home_clear` | `home_clear_pose.json` | HOME with both shoulders' J2 10 deg lower (87.38 -> 77.38). Opt-in |
| `stow` | `stow_pose.json` | the compact tuck for driving (an endpoint, not a rest) |

### Why `home_clear`, and why it is not HOME

Measured with the kit's own verdict (`GuardedArm.posture_violation` + the
motion guard; `tests/arms/test_home_clear_pose.py`):

| pose | body clearance right / left [mm] | binding link | +/-0.1 rad band kit-clean, right / left |
|---|---|---|---|
| `home` | 36.3 / 30.3 | `Link2` (upper arm) vs `torso_belly` | 75.7 % / 48.7 % |
| `home_clear` | 43.1 / 38.1 | `Link1` (shoulder, fixed at the mount) vs `torso_belly` | 100 % / 100 % |

The guard's body margin is 30 mm, so HOME's left arm has 0.3 mm of slack and a
+J2 step of 0.062 rad from HOME already commands a refused posture. At
`home_clear` the binding capsule is the shoulder sphere, which does not move
with the joints: 43.1 / 38.1 mm is the best any posture of these arms reaches
against this collision model (the left arm is 6 mm closer because the belly box
is 6 mm off-centre). The straight joint move HOME -> `home_clear` is
guard-clean at 1 deg samples; the tool point moves 10 mm and the tool frame
tilts 10 deg about the J2 axis.

It is opt-in because HOME is embedded downstream. A consumer that switches to
it must handle its own row:

| consumer | embeds HOME how | what switching needs |
|---|---|---|
| recorded datasets (e.g. 0824: frame-0 median == HOME) and the ACT / pi policies trained on them | the initial-state distribution | nothing; those policies start at their dataset's frame 0 (`d1-inference run_act --start-pose checkpoint`, the default), never at `home_clear` |
| d1-inference `tools/run_act.py`, `tools/go_home.py` | reads the kit HOME live | to park at `home_clear`, load it by name for the HOME move; the policy preflight still walks to the checkpoint start pose |
| d1-inference `policy/preflight.py` `EPISODE_START_STATE` | frozen radians == HOME | nothing (it is the 0824 checkpoint's start) |
| d1-isaaclab RL skills (`home_q` from `default_joint_pos`, `kit_home_rad()`) | reset pose, reward target and the `arm_q - home_q` observation | opt in with `load_named_pose("home_clear")` and retrain; existing checkpoints stay on HOME |
| d1-isaaclab ACT eval (`DATASET_HOME_RAD`) | frozen dataset HOME | nothing |
| gesture CSVs (`mkit-teach`, omakaseos `d1_gesture_gen.py`) | first and last keyframe == HOME, fingerprinted | nothing unless the gesture set moves; then regenerate and re-fit every CSV |
| d1-sdk `devices/omakase_arm/config/home_pose.json` (read by omakaseos, dx-vr-teleop, d1-vr-teleop) | a second copy of `home_pose.json` | nothing; if HOME itself ever changes, both files change together |
| dx-teleop-server `devices/arms/home.py`, `executor/arm_model.py` | reads the kit HOME live | pass the named pose to use it as the park/ramp target |
| d1-firmware (`assets/home_pose.json`, guard test vectors) | vendored snapshot of the kit file | nothing (the daemon has no arm HOME endpoint) |
| `perception.camera.provisional_table_z` | HOME tool z (0.201 m) | nothing (`home_clear` tool z is 0.204 m; the value is provisional by contract) |

## Phase 1 of 2: the copy in `dx-vr-teleop` is still live

This package was migrated from `dx-vr-teleop` `master` @ `de8d781` (i.e. PR #41
merged; tree byte-identical to its branch tip `043d521`). **Nothing was deleted
from `dx-vr-teleop`** — by design. Both copies run until the migration is
verified.

That is only safe because "have they diverged?" has an answer:

```sh
D1_SDK_DIR=~/d1-sdk \
D1_TELEOP_GUARD_CONFIG=~/d1-vr-teleop/guard_config.json \
DX_VR_TELEOP_DIR=~/d1-vr-teleop \
pytest tests/test_parity_dx_vr_teleop.py -v
```

It loads **both** implementations, on the same URDF and the same guard, and
asserts they agree **exactly** (`atol=0`) on:

- the collision guard being installed at all (else nothing else means anything)
- the HOME pose, and the 200-restart deterministic READY-seed search
- `ee_pose` and the joint-limit margin
- every accept/reject decision *and* the resulting joint vector over an 80-tick
  trajectory per arm that deliberately covers accepted ticks, IK failures and
  guard rejections
- `OneEuro`, `PoseFilter`, `JointFilter` over 200 samples each
- the WebXR→robot frame conversions
- every clamp constant, including `WORKSPACE` and `MAX_JOINT_STEP`
- 300 ticks of `ArmClutch` targets — box clamp, anchor-union latch, step clamp,
  parking deadband, accept/reject bookkeeping — handheld and strapped

It **skips silently** when `DX_VR_TELEOP_DIR` is unset, since `dx-vr-teleop` is a
separate private repo and not a dependency (the same pattern `dx-vr-teleop`
already uses in the other direction with `importorskip("manipulation_kit.hands")`).

### Phase 2 completion criterion

Deleting `dx-vr-teleop`'s copy is permitted when **all** of these hold:

1. The parity gate above passes **on the robot host** (`d1-1`), with the real
   `guard_config.json` and the real `d1-sdk` — not just in CI.
2. `dx-vr-teleop`'s own suite passes with its `server/` arm modules re-pointed at
   `manipulation_kit.arms` (that re-pointing is the phase-2 change itself).
3. One VR teleop session on real hardware driven through `manipulation_kit.arms`, with
   the operator confirming the four behaviours #41 fixed: no sag on compliance
   entry/exit, soft-follow tracking, HOME latency, and body-boundary stop.
4. `omakase-core` imports `manipulation_kit.arms` rather than its own
   `robot_stack/teleop/vr_arm/{kinematics,clutch,filters,safety,sides}.py`.

Until (1)–(4), the duplication stays and the gate is the thing that keeps it
honest. If the gate ever fails, the first question is which side moved: it
reports the baseline ref (`de8d781`) in its failure messages.

### Who wins a disagreement

**`dx-vr-teleop` `master` is the reference implementation.** Where this library
and another consumer disagree about a value or a behaviour, master wins — it is
the code that actually runs on the robot. A consumer's differing number is a
divergence to be corrected, not an alternative to be reconciled; omakase-core's
`0.05` joint-step clamp against master's `0.25` was the first instance, and it
was simply wrong. If the gate fails because *master* moved, follow master.
(Ruling by Shu, 2026-07-29.)
