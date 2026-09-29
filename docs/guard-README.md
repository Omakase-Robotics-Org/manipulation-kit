# pyguard — D1 software range-of-motion / self-collision guard

Pure-python (standard library only, no numpy/ROS) guard that checks
commanded **joint vectors** for the D1's two D1 arms **before
they reach the hardware**:

1. **per-joint limit clamp** — limits parsed from the full-body URDF
   (`description/d1/d1.urdf`);
2. **torso / head keep-out** — every arm capsule vs the measured body
   shapes (FK + exact capsule-vs-AABB distance, capsule-vs-vertical-cylinder
   by a convex search, configurable margin, default 0.005 m of real air),
   plus a
   per-arm **upper-chest keep-out** that closes the shoulder-band notch
   left by the exempt CAD shoulder shell;
3. **arm–arm distance** — the two arms' capsules must stay
   ≥ `arm_arm_margin_m` apart (default 0.045 m);
4. **same-arm self collision** — non-adjacent capsule pairs with the
   "bridged by a short link" skip rule of `collision_model.h`.

Angles are **degrees** in SDK order `J1..J7`; arm `'A'` = `_R` link tree =
physical **left** (+y), `'B'` = `_L` = physical **right** (−y) — the same
conventions as `safety_zones.h` / `gesture_csv.h`.

### Collision policy — what is and isn't protected

The guard protects the **arm structure** (shoulder → upper arm → elbow →
forearm → wrist) from the torso and from the other arm. The
**end-effector bodies are excluded**: every body **distal of the tool
mounting flange** (`JointTCP`: `Link7 → TCP_Link`) — the `TCP_Link` and the
whole YUBI hand the guard model carries for its frames — is dropped from the
body, arm–arm and self checks. `Link7` and everything proximal is arm
structure and stays checked. See `EE_LINK_PREFIXES` / `is_ee_body()`.

**The hand envelope** (`hand_envelope_<R|L>`, on the flange; `HAND_ENVELOPE`
in `generate_d1_urdf.py`) is the tool the robots wear — the parallel gripper,
its jaws over their whole travel, the wrist-camera plate and housing — as six
capsules covering every collision box of its description to within 0.74 mm.
It is checked

| against | checked? | margin |
|---|---|---|
| body shapes, chest keep-out | yes | `body_margin_m` |
| the other arm's structure | yes | `body_margin_m` (real air, as against the body) |
| the other hand's envelope | no — bimanual contact and hand-offs are intended | — |
| its own arm | no — the hand is where its wrist is | — |
| scene obstacles (`primitives.clearance.SceneGate`) | no — the hand is what touches the scene | — |

so a jaw tip or the camera plate can no longer be driven into the torso while
the arm links report clearance. `GuardReport.min_hand_arm` is the
hand-against-other-arm minimum.

### Body shapes: boxes and vertical cylinders

Static body geometry is axis-aligned boxes and **vertical cylinders**. A body
the URDF cannot give as one primitive is a union of parts named `<body>`,
`<body>__<part>`; the guard reports the body (`body_name()`), never the part.
The belly band `torso_belly` is the measured torso box with its vertical edges
rounded (front 80 mm, back 40 mm) — three boxes and four cylinders — because
the shell curves inward at the corners and a square box stopped the arm ~40 mm
short of it there. `description/d1/tools/fit_torso_belly.py` measures the
shape against the CAD shell:

| shape | shell outside it (max) | air covered, front corners | air covered, back corners |
|---|---|---|---|
| square box (0.19) | 0.0 mm | 39.8 mm | 33.8 mm |
| rounded band (0.20) | 3.4 mm | 7.6 mm | 21.6 mm |

(against the CAD outline moved onto the measured faces; the CAD itself is
7-13 mm wider per side than the built robot, see the script). A full dual-arm
check costs ~1.4 ms in CPython on a laptop CPU (0.5 ms before the hand
envelope and the cylinders).

Compared to the C++ `collision_model.h` (used inside the numeric IK), this
model checks against the **measured** torso, head and chassis boxes from
the D1 CAD (not just the legacy keep-out column) and adds the upper-chest
keep-out; the TCP flange / YUBI hand are modelled for FK but excluded from
the guard checks by the policy above.

## Where the verdict is applied: three layers

One verdict, applied at three different points on the way to the arms. The
verdict is `GuardedArm.posture_violation` (the joint box and the coupled limits
such as J7(J6)) followed by this guard on the full two-arm posture. The layers
differ in which postures they apply it to.

| layer | where | what it checks | what it cannot see |
|---|---|---|---|
| 1. planner knots | `primitives.planning` (`solve_path` via `GuardedArm.solve_ee`, `joint_ramp`) | every knot of a plan, which are at most `MAX_JOINT_STEP_RAD` (0.25 rad, 14.3 deg) apart | the straight line between two knots |
| 2. per-send endpoint | `GuardedArm.posture_violation` + `GuardedArm.guard_ok` on each commanded target (the executor's tool-gate corrections; any consumer that filters targets one at a time) | the posture a command asks for | the line the arm travels to reach it |
| 3. swept path | d1-firmwared's single-shot joint verbs (`move_joint`, `move_joints`, `move_joints_both`); in the kit, `GuardedArm.swept_path_violation` / `swept_path_ok`, which `FirmwareExecutor.send_joints` runs before every streamed `move_joints_both` | every sample of the straight joint-space line from the arms' pose to the target, at `safety.SWEPT_PATH_STEP_DEG` (1 deg) per joint, target included; a line that starts inside a margin passes while no clearance stage reads closer than at the start | nothing between samples finer than 1 deg |

The daemon's trajectory lane (`POST /v1/arm/trajectory/start`, the executor's
default transport) re-checks every pose it samples on its 1 ms ticker. That is
layer 3 at a finer spacing, so an uploaded plan is swept by the daemon itself.

Layer 3 exists because layer 2 is not enough. Two postures can each pass while
the line between them dips into a margin: `tests/arms/test_swept_path.py` pins
a right-arm pair 18.2 / 43.2 mm from `torso_belly` whose line brings the hand
envelope within 2 mm of it (margin 5 mm). Without the local check the kit would send that command and learn
from the daemon's HTTP 409 that it was refused. With it, the command is
refused before it is sent (`GuardRefused`), naming the sample, the moving
joints and the pair under its margin, in the daemon's wording.

The kit's line starts at the last command `send_joints` sent, which in position
mode is where the arm is heading. It starts at measured feedback when nothing
has been sent yet. The daemon's line starts at feedback, so while an arm lags
its command the two checks start from different postures. The daemon's check
still runs behind the kit's.

### What the sim mirrors

d1-isaaclab's `skills/guard_terms.py` (`BatchedGuard`, `kit_verdict`) mirrors
**layer 2**: the endpoint verdict on every commanded joint target, batched in
torch and held equal to the kit on random postures. It does not yet mirror
layer 3, so the sim can accept a target step that the robot's stream refuses.
To mirror layer 3, feed `manipulation_kit.arms.kinematics.swept_path_samples(q_from, q_to)`
(pure arithmetic: samples `1..n`, `n = swept_sample_count(...)`) to the batched
verdict, apply the same escape rule when the start is inside a margin, and use `GuardedArm.swept_path_violation` as the reference in its
parity tests. At the RL action scale of 0.1 rad per step that is at most 6
samples per command.

## Usage

```python
from manipulation_kit.guard import MotionGuard, GuardedRobot

guard = MotionGuard()                       # loads description/d1/d1.urdf
rep = guard.check(jointsA, jointsB)         # either may be None
if not rep.ok:
    print(rep)                              # human-readable reasons

# drop-in wrapper around vendor_arm_sdk.robot.ArmRobot:
robot = GuardedRobot(ArmRobot(), guard) # blocks unsafe commands
robot.set_joint_cmd_pose('A', jointsA)      # returns 2 + prints if blocked
```

**Opt-in wiring in `sdk/TJ.py`** (teleop): run with `OMAKASE_ARM_GUARD=1`
to enable; with the variable unset the behavior is byte-identical to
before (covered by a test).

Useful knobs (see `guard.py` docstrings): `body_margin_m` (default 0.005),
`arm_arm_margin_m` (0.045), `clamp_limits`, `check_body/check_self/
check_arm_arm`, `disabled_body_boxes` (chassis boxes are disabled by
default because their position depends on the lift extension),
`chest_keepout` (per-arm upper-chest keep-out boxes; `None` disables it),
`on_reject="block"|"raise"`.

A full dual-arm check costs ~1.4 ms on a laptop CPU — fine for teleop-rate
command streams.

## Tests

```sh
cd devices/omakase_arm
PYTHONPATH= AMENT_PREFIX_PATH= python3 -m pytest pyguard/tests -q
```

Coverage: FK vs known poses (T-pose, hardware HOME), URDF limits vs
`safety_zones.json`, known-colliding poses rejected (torso hit, structural
self-collision, arm-arm crossing, elbow-into-chest via the chest keep-out),
the **EE collision policy** (a bimanual hands-touching pose passes; EE
bodies are excluded from every check), the shipped `test_gesture_motion.csv`
keyframes all pass, limit clamping, guard-disabled passthrough is
call-for-call identical, and a **number-for-number cross-check against the
C++ `collision_model.h`** (compiles `tests/cpp_dump.cpp` with the host
`g++`, requires FK and clearance agreement < 1e-6 m; skipped when g++ is
absent).
