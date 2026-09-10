# pyguard — D1 software range-of-motion / self-collision guard

Pure-python (standard library only, no numpy/ROS) guard that checks
commanded **joint vectors** for the D1's two D1 arm arms **before
they reach the hardware**:

1. **per-joint limit clamp** — limits parsed from the full-body URDF
   (`description/d1/d1.urdf`);
2. **torso / head keep-out** — every arm capsule vs the measured body
   boxes (FK + capsule-vs-AABB distance, configurable margin), plus a
   per-arm **upper-chest keep-out** that closes the shoulder-band notch
   left by the exempt CAD shoulder shell;
3. **arm–arm distance** — the two arms' capsules must stay
   ≥ `arm_arm_margin_m` apart (default 0.06 m = `safety_zones.json`);
4. **same-arm self collision** — non-adjacent capsule pairs with the
   "bridged by a short link" skip rule of `collision_model.h`.

Angles are **degrees** in SDK order `J1..J7`; arm `'A'` = `_R` link tree =
physical **left** (+y), `'B'` = `_L` = physical **right** (−y) — the same
conventions as `safety_zones.h` / `gesture_csv.h`.

### Collision policy — what is and isn't protected

The guard protects the **arm structure** (shoulder → upper arm → elbow →
forearm → wrist) from the torso and from the other arm. The
**end-effectors are excluded from every check**: every body **distal of the
tool mounting flange** (`JointTCP`: `Link7 → TCP_Link`) — the `TCP_Link`
and the whole YUBI hand — is dropped from the body, arm–arm and self
checks. The hands are the working surfaces and must be free to contact the
world and each other (bimanual manipulation / hand-offs). `Link7` and
everything proximal is arm structure and stays checked. See
`EE_LINK_PREFIXES` / `is_ee_body()` in `guard.py`.

Compared to the C++ `collision_model.h` (used inside the numeric IK), this
model checks against the **measured** torso, head and chassis boxes from
the D1 CAD (not just the legacy keep-out column) and adds the upper-chest
keep-out; the TCP flange / YUBI hand are modelled for FK but excluded from
the guard checks by the policy above.

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

Useful knobs (see `guard.py` docstrings): `body_margin_m` (default 0.03),
`arm_arm_margin_m` (0.06), `clamp_limits`, `check_body/check_self/
check_arm_arm`, `disabled_body_boxes` (chassis boxes are disabled by
default because their position depends on the lift extension),
`chest_keepout` (per-arm upper-chest keep-out boxes; `None` disables it),
`on_reject="block"|"raise"`.

A full dual-arm check costs ~2–3 ms on a laptop CPU — fine for teleop-rate
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
