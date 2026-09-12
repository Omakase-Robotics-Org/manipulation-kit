# D1 dual-arm conversational gestures — teach & smooth playback

Two C++ tools turn drag-teaching into smooth, repeatable D1 (D1 arm dual-arm)
gestures, plus the cross-repo CSV/home-pose/CLI contract shared with **omakaseos**.

- `example/gesture_record.cpp` — drag-teach → keyframe-reduced gesture CSV (with safety-on-recording)
- `example/gesture_play.cpp` — smooth (non-jerky) playback of one or a sequence of CSVs, with a runtime collision-reactive guard
- `include/omakase_arm/gesture_csv.h` — header-only CSV/interpolation/home-pose logic (host-buildable, unit-tested)
- `include/omakase_arm/safety_zones.h` — header-only self-collision / torso keep-out validator (host-tested)
- `include/omakase_arm/collision_monitor.h` — header-only collision-detection core (host-tested)
- `include/omakase_arm/collision_monitor_arm.h` — SDK adapter: drives the detector from `Arm::info()` and does the stop→compliance reaction
- `config/home_pose.json` — **the single source of truth for the HOME pose**, 14 angles (currently the wrist-forward "ready" pose). Every consumer in this repo, `omakaseos` and `d1-vr-teleop` reads this file; nothing copies the numbers — see [§2](#2-home-pose-config--confighome_posejson)
- `config/stow_pose.json` — the STOW (compact tuck) pose: arms folded inside the AMR chassis footprint for driving; moved to/from with `example/stow_goto.cpp` (an ENDPOINT move — parks there, no return to HOME)
- `config/collision_thresholds.json` — per-joint collision thresholds (conservative; tune on the real robot)
- `test/gesture_csv_test.cpp`, `test/safety_zones_test.cpp`, `test/collision_monitor_test.cpp`, `test/stow_pose_test.cpp` — host unit tests (no robot/.so needed)

> Runtime needs the robot: the tools link the vendor arm SDK `.so`/`libKine.so`
> (aarch64). On an x86 dev box they **compile** but cannot link the vendor .so;
> build & run them on the robot. The CSV/maths in `gesture_csv.h` are fully
> unit-tested host-side.

---

## Build

```sh
cd devices/omakase_arm
./build.sh gesture_record      # -> example/gesture_record   (needs robot to link)
./build.sh gesture_play        # -> example/gesture_play
# or with CMake (also builds + registers the host unit test):
cmake -S . -B build && cmake --build build
ctest --test-dir build --output-on-failure     # runs gesture_csv_test
# unit test alone, no CMake / no .so:
g++ -std=c++14 -Iinclude test/gesture_csv_test.cpp -o /tmp/t && /tmp/t
```

## Record a gesture

```sh
# from devices/omakase_arm (so config/home_pose.json resolves)
./example/gesture_record mygesture_motion.csv --rate-hz 50 --stationary-s 1.5
```

Enters **pure compliance** (`ForceComplianceConfig::xAxisCompliance` — force 0,
the J7-safe path). The tool prints `=== COMPLIANCE ENGAGED -- move the arms by
hand ===` and a live sample count. Drag both arms by hand; samples are polled at
`--rate-hz` until you press **Enter**, hit `--duration-s`, or stay still for
`--stationary-s`. The first & last keyframes are snapped to HOME, and collinear
samples are dropped (`--epsilon-deg`) so the CSV is compact but smooth.

**Safety on recording:** a teach can capture an UNSAFE pose (you can physically
drag an arm into a self-collision or the torso). After reduction the recorded
trajectory is run through the **same** self-collision / torso keep-out validator
(`safety_zones.h`) that `gesture_play` uses as its pre-flight gate. By default an
unsafe teach is **REFUSED** (no CSV written); the tool prints
`recorded N keyframes, M reduced, safety: OK/VIOLATIONS, saved to X`. Pass
`--allow-unsafe` to write it anyway (it stays flagged, and `gesture_play` will
still refuse to play it without `--no-safety`). After recording the arms return
safely via `lockCurrentPositionMode` (hold current pose), not limp.

**Smoothing on recording:** compliance hand-teaching carries encoder jitter and
occasional spikes. Left raw, that noise survives keyframe reduction and — worse —
inflates the per-joint velocity the playability pass measures, so the gesture is
over-stretched (a 7 s teach balloons to ~16 s) and playback wiggles. The recorder
runs a per-joint **median (spike-reject) + mean (low-pass)** pre-filter on the raw
samples before snap-home/reduction. Tune with `--smooth-window N` (odd sample
count, default 5 ≈ 0.1 s at 50 Hz; even is rounded up); `--no-smooth` (window=1)
disables it.

Options: `--rate-hz` (50) · `--duration-s` (0=until Enter) · `--stationary-s`
(0=off) · `--epsilon-deg` (1.5) · `--smooth-window` (5) · `--no-smooth` ·
`--home PATH` · `--no-snap-home` · `--adj-limit-mm` (2.0) · `--no-safety-check` ·
`--allow-unsafe`.

### Externally controllable (dashboard / headless)

`gesture_record` can be driven by a parent process (the omakaseos status server
teach panel) with **no TTY**:

- **Stop**: **SIGINT *or* SIGTERM** is handled gracefully → finalize (snap home,
  reduce, safety-check, save) and exit 0. A **`--stop-file <path>`** it polls is
  the robust signal-less fallback — drop the file and it stops the same way.
- **Live status**: **`--status-file <path>`** is rewritten at the capture rate
  with a JSON snapshot the dashboard polls:
  `{state: starting|compliance_engaged|recording|finalizing|done|error,
  samples, duration_s, safety: null|"ok"|"violations", csv_path, message}`
  (written atomically via temp + rename). stdout logging is unchanged.

```sh
# headless: server spawns this, polls status.json, drops stop to finalize
./example/gesture_record /staging/x_motion.csv \
    --status-file /staging/status.json --stop-file /staging/stop
```

The status/stop plumbing lives in `include/omakase_arm/teach_control.h` and is
host-unit-tested (`test/teach_control_test.cpp`, no robot required).

## Play a gesture (smooth)

```sh
./example/gesture_play mygesture_motion.csv
# sequence, continuous with a 250 ms gap between gestures:
./example/gesture_play a_motion.csv b_motion.csv --gap-ms 250
```

Enters **force-compliance mode by default** (see below), eases into HOME, then
streams `moveJointsBoth` targets at `--rate-hz` (default 100) with
**Catmull-Rom + smootherstep** interpolation so the motion is fluid, not
stepped ("gacha-gacha"). Always `home → gesture → home`. Gestures play
continuously; the inter-gesture gap is `--gap-ms` (default 0 = continuous).

Options: `--gap-ms` (0) · `--rate-hz` (100) · `--vel-ratio` (10) ·
`--acc-ratio` (10) · `--home PATH` · `--home-settle-s` (1.5) · `--no-safety` ·
`--no-collision-guard` · `--collision-config PATH` · `--collision-threshold-scale N` ·
`--no-compliance` · `--compliance-adj-mm` (2.0).

### Compliance-mode playback (default)

Gestures play in the controller's **force/compliance mode** (`ARM_STATE_TORQ` +
`ImpType=3`, target force **0 N** = pure compliance), not stiff position mode,
so the arm **yields softly to contact while it is moving**. Entry sequence
(mirrors the vendor `kinefixed` demo and `Arm::setForceComplianceMode()`):

1. `setTool(ToolConfig::defaultGripper())` — register the gripper's dynamics
   **before** the torque-based mode. Skipping this fails TORQ entry with
   `ARM_ERR_RequestSensorMode(6)` and drifts J7 once inside.
2. `setForceComplianceMode(xAxisCompliance(--compliance-adj-mm), --vel-ratio,
   --acc-ratio)` — force command stays 0 N (a positive force actively pushes in
   free space → continuous J7 drift; see `ForceComplianceConfig` docs).
3. Joint targets stream exactly as before — the torque controller tracks
   `m_Joint_CMD_Pos` compliantly, so no CSV/streaming change.

`--no-compliance` restores the old stiff position-mode playback. NOTE: under
compliance the tracking error during normal playback is larger than in
position mode — if the collision guard's `tracking_error_threshold_deg` was
calibrated for position mode and now false-trips, re-run the calibration below
(or temporarily raise `--collision-threshold-scale`).

## Collision-reactive playback (person bumps in → stop → go soft)

While streaming, `gesture_play` runs a **CollisionMonitor** every control cycle.
It reads, per joint, the `feedback_torque` deviation from a (pose-dependent,
auto-learned) baseline **and** the position tracking error `|command −
feedback|`. If a person bumps/hits an arm, either signal spikes; once it stays
over its per-joint threshold for `trigger_cycles` consecutive cycles the monitor
**immediately halts the trajectory and switches the arm to compliance**
(`setForceComplianceMode` — the arm goes soft and yields) instead of rigidly
pushing through the obstacle (the person). The arm then holds compliant in a safe
state; the operator re-homes / re-enables to recover. `gesture_play` exits with
code 2 on a collision-triggered stop.

- **Reusable component**: `CollisionMonitor` (`include/omakase_arm/
  collision_monitor_arm.h`) wraps **any** position-mode control loop, not just
  `gesture_play`:

  ```cpp
  CollisionMonitor mon(arm, collision::loadConfig("config/collision_thresholds.json"));
  for (each control tick) {
      arm.moveJointsBoth(a, b);
      if (mon.check(a, b).detected) { mon.reactGoCompliant(); break; }
  }
  ```

  The pure detection core (`include/omakase_arm/collision_monitor.h`, std-only)
  is host-unit-tested (`test/collision_monitor_test.cpp`); the SDK adapter only
  pulls feedback out of `Arm::info()`.
- **Config**: `config/collision_thresholds.json` — per-joint `torque_threshold_nm`,
  `tracking_error_threshold_deg`, `trigger_cycles`, `baseline_warmup_cycles`.
  Conservative defaults: per Shu, a false-trigger-stop beats missing a real
  collision. Flags: `--collision-threshold-scale N` (>1 less sensitive, <1 more),
  `--no-collision-guard`, `--collision-config PATH`.
- **Calibration (REAL ROBOT ONLY — cannot be tuned without hardware):**
  1. Play a few normal safe gestures with the guard effectively off (e.g.
     `--collision-threshold-scale 5`) while logging `feedback_torque` per joint
     (`status` / `compliance_status` dump it). That is the **normal envelope**.
  2. Set each `torque_threshold_nm[Jn]` **above** the observed envelope but
     **below** a real human-bump torque (push the arm by hand, watch the
     readings). Leave `torque_baseline_nm` all-zero to auto-learn the holding/
     gravity baseline per playback (recommended — holding torque is pose-dependent).
  3. Set `tracking_error_threshold_deg[Jn]` above the normal smooth-playback
     tracking error (a few degrees) but below a bump-induced lag.
  4. Verify: a gentle hand-push mid-gesture stops the motion and the arm goes
     soft; normal playback never triggers.

## Safety gate (self-collision / torso keep-out)

Before touching the robot, `gesture_play` validates each gesture's **smooth
path** (`HOME → keyframes`, the exact Catmull-Rom path it will stream) against
`config/safety_zones.json` and **REFUSES** to play a CSV that would:

- bring the two arms within `min_arm_arm_distance_m` (default 0.06 m) — the
  "hands collide in front of the belly" case;
- drive any actuated link into the `torso_keepout_box` (margin
  `min_body_clearance_m`, default 0.03 m);
- exceed a D1 arm joint limit.

The validator (`include/omakase_arm/safety_zones.h`, header-only) models each
arm as a chain of **capsules** using the **same FK chain as `d1_dual.urdf`**
and the **measured shoulder geometry** (bases 74 mm apart = ±0.037 m, from the
STEP assembly `d1-face/d1_face.step`). It is **mirrored byte-for-byte** in the
omakaseos authoring viewer (`static/d1_safety_validator.js`) so a motion the
editor calls "safe" is exactly what this gate accepts. Host-tested by
`test/safety_zones_test.cpp`. Override (at your own risk) with `--no-safety`.

`config/safety_zones.json` is the **derived export** of the guard header:
`include/omakase_arm/safety_zones.h` is the single authored source of the
numbers (it is what `gesture_play` compiles and runs), and the JSON carries the
same joint limits, capsule radii, kinematic chain, arm mounts and keep-out
zones so external consumers (the omakaseos Python and JavaScript validators)
can read one artifact instead of authoring their own copy. The derivation is
enforced in-repo by `pyguard/tests/test_safety_zones_export.py`, which
text-parses the header and asserts the JSON equals it field by field (no
compiler needed); if the header changes, that test fails until the JSON is
regenerated to match. Do not hand-edit numbers into the JSON that the header
does not author — the header wins, and the test guards it.

## Generated safe sample gestures

A library of conservative, **safe-by-default** sample gestures is generated (and
re-validated against `safety_zones.h`) by the omakaseos generator
`robot_stack/robots/omakase/d1_gesture_gen.py` and shipped in the omakaseos
gesture lib (`robot_stack/robots/omakase/d1/csv/` + `gesture.yaml`, marked
`generated: true`). They are `HOME → small excursion → HOME`, within D1 arm
limits: a one-arm wave, a two-arm "present/welcome" (opens **outward**, not toward
the belly), a gentle nod (symmetric shoulder-pitch bow), a "thinking" idle sway, a
one-arm point-forward, and a small beckon. They are placeholders, not
teach-quality choreography — replace with `gesture_record` captures for production.
Every generated CSV passes **this** repo's `gesture_play` safety gate (the Python
generator's validator is a byte-equivalent mirror of `safety_zones.h`,
cross-verified numerically).

---

## CROSS-REPO CONTRACT (keep identical to `omakaseos` `robot_stack/robots/omakase/d1/GESTURES.md`)

This is the integration seam between **d1-sdk** (C++ tools) and **omakaseos**
(status server / conversation). Changing any of these three things requires
updating BOTH repos' GESTURES docs in the same change.

### 1. CSV format

`<name>_motion.csv` — the omakaseos keyframe motion format with **14 joint
columns** (dual-arm, 7 per arm), **angles in DEGREES**:

```
# free-form comment line(s)
duration,R1,R2,R3,R4,R5,R6,R7,L1,L2,L3,L4,L5,L6,L7
0.50,0,-90,0,0,0,0,0,0,-90,0,0,0,0,0
0.40,12,-70,5,0,0,0,0,-12,-70,-5,0,0,0,0
...
```

- **Column 1 `duration`** = seconds to move FROM the previous keyframe TO this
  one (same semantics as the Unitree G1 CSVs / `MotionData`). The first
  keyframe's duration is the time from the playback's current pose to it.
- **Joint order**: `R1..R7` = `ArmSide::A`, `L1..L7` = `ArmSide::B`. On the real
  robot (per `config/home_pose.json`, measured on hardware) **ArmSide::A is the
  physical LEFT arm and ArmSide::B is the physical RIGHT arm** — the `R`/`L`
  column tags are historical and follow the vendored `_R`/`_L` mesh trees, NOT
  the physical side. Each joint is the URDF `JointN_{R,L}` angle in degrees
  (DH/SDK native unit; the robot's `moveJoints*` API takes degrees).
- Optional `kp,…` / `kd,…` rows and `#` comments are tolerated and ignored by
  the D1 tools (position mode, no per-joint gains). This keeps the file
  loadable by `omakaseos` `csv_motion_loader.load_motion_csv(path, num_joints=14)`.
- The recorder snaps keyframe[0] and keyframe[-1] to HOME (whatever
  `config/home_pose.json` holds) so every gesture starts and ends at rest.

### 2. Home-pose config — `config/home_pose.json`

```json
{ "units": "degrees",
  "joint_order": ["A1",...,"A7","B1",...,"B7"],
  "home_pose": [ ...14 floats... ] }
```

14 floats, degrees, in `A1..A7,B1..B7` order (== the CSV's `R1..R7,L1..L7`
columns). Currently the **wrist-forward "ready" pose**; the file's own
`_comment` records that it replaced the earlier arms-hanging "だらん" (daran)
pose (measured on hardware 2026-06-13), which itself replaced the original
`J2=-90` best-guess. The two arms are a mirrored pair: B's J1/J3/J5/J7 are the
sign-flip of A's.

**This file is the single source of truth for HOME. Never copy the numbers.**
The numbers are deliberately not reproduced in any doc, header, page or
constant — a literal copy is how the description package and the omakaseos
viewers each silently drifted a full pose behind the robot. Every consumer
reads *this* file:

| Consumer | How it resolves the file |
| --- | --- |
| `gesture_play` / `gesture_record` / `stow_goto` / `ik_click_move` | `--home PATH`, default `config/home_pose.json` relative to CWD |
| `pyguard` tests | path relative to the repo |
| `description/d1_yubi_description_v2` (RViz, `home_move_publisher`) | its `config/home_pose.json` is a **symlink** to this file |
| omakaseos gesture generator + `/api/d1/home_pose` (teach / preview / authoring pages) | `$OMAKASE_D1_SDK_DIR/config/home_pose.json` |
| `d1-vr-teleop` (`server/backends.py`) | `$D1_SDK_DIR/devices/omakase_arm/config/home_pose.json`, read once at backend init |

Consequences of editing it: the C++ tools and the viewers pick it up
immediately; **`d1-vr-teleop` needs a process restart**; and every recorded /
generated gesture CSV must be re-recorded or regenerated, because a CSV pins
HOME as its first and last keyframe (omakaseos'
`test_d1_gesture_home_sync.py` is the gate for that).

### 3. `gesture_play` CLI signature (how omakaseos subprocesses it)

```
gesture_play <csv> [<csv2> ...] [--gap-ms N] [--rate-hz N] \
             [--vel-ratio N] [--acc-ratio N] [--home PATH] [--home-settle-s N] \
             [--no-safety] [--no-collision-guard] [--collision-config PATH] \
             [--collision-threshold-scale N] [--no-compliance] \
             [--compliance-adj-mm N]
gesture_record <out.csv> [--rate-hz N] [--duration-s N] [--stationary-s N] \
               [--epsilon-deg N] [--smooth-window N] [--no-smooth] \
               [--home PATH] [--no-snap-home] [--adj-limit-mm N] \
               [--no-safety-check] [--allow-unsafe] \
               [--status-file PATH] [--stop-file PATH]
```

The status server's teach panel spawns `gesture_record` with `--status-file` +
`--stop-file` (and stops it via SIGTERM / the stop-file). See "Externally
controllable" above and `teach_control.h` for the status JSON schema.

`omakaseos` spawns `gesture_play <csv> [--gap-ms N]` (and `gesture_record
<out.csv>`) as a subprocess — exactly like the G1 `play_action` pattern. The
binary path is configured in omakaseos via env/constant; omakaseos degrades
gracefully (returns "hardware not available") when the binary or robot is absent.

### 4. `gesture_play --server` — persistent arm session (brake off per session)

To avoid paying the ~2 s connect + servo-enable (brake-release) cost on **every**
gesture, omakaseos owns ONE persistent `gesture_play --server` for the whole
conversation: the brake is released once at session start and re-engaged once at
session end, and individual gestures are served over stdin so they start
instantly (the servo/brake stays ON — the arm holds, not limp — for the session).

Line protocol (one command/event per line; stdout is protocol-only, human logs
go to stderr):

```
stdin  (omakaseos -> server):
  PLAY <id> <csv_path>   play one gesture CSV; <id> correlates the reply
  CANCEL                 abort the gesture now playing (arm stays servo-held)
  STATUS <id>            report live per-joint feedback; answered immediately,
                         even mid-gesture (see TEMP below)
  QUIT                   release the brake and exit
  STREAM ON              open a live joint stream (see STREAM below)
  STREAM <a1..a7> <b1..b7>  14 joint targets in DEGREES, ArmSide::A then
                         ArmSide::B -- the same order as a gesture CSV row
  STREAM OFF             close the stream
stdout (server -> omakaseos):
  READY                  connected + control mode set (compliance by default,
                         brake off); ready
  START <id>             began playing <id>
  DONE  <id> <code>      finished <id>: 0=ok, 2=collision, 3=cancelled
  ERR   <id> <message>   <id> failed validation/load (no motion happened)
  TEMP  <id> <A J1..J7> <B J1..J7>   reply to STATUS <id>: 14 feedback joint
                         temperatures in degrees C, arm A (7 floats) then arm
                         B (7 floats), space-separated
  STREAMING              the stream is open and the arm is being commanded
  SREJ  <reason>         a streamed target was refused; it was NOT commanded
  STOPPED                left stream mode; the arm is in a position hold
  BYE                    released the brake and exiting
```

STREAM is for interactive teleoperation rather than authored gestures: the
server commands the arm at `--stream-rate-hz` (default 100) whether or not a
fresh target arrived, holding the last **accepted** pose, and ends the stream
after `--stream-watchdog-ms` (default 500) of STREAM silence. Each target goes
through the same safety validator a CSV does, and the commanded pose moves
toward it by at most `--stream-max-joint-vel-deg-s` (default 350) per second
per joint. Streaming and PLAY share one arm: a queued gesture stops the stream
first. Full description and the on-robot test procedure are in
`devices/omakase_arm/README.md`.

On a collision the arm is switched to compliance and the server exits
(`DONE <id> 2` then `BYE`); the owner re-homes/re-enables to recover. omakaseos
drives this from `D1ArmGestureBridge.start_session()` (waits `READY`) /
`execute_arm_action()` (sends `PLAY`, blocks on `DONE`) / `end_session()` (sends
`QUIT`), and **falls back to per-gesture `gesture_play <csv>`** when the binary or
robot is absent or the session can't start. While a session is active omakaseos
publishes a cross-process lock so the status-server dashboard `/play` + teach
refuse to spawn a competing arm subprocess. The dashboard's D1 hardware tab
polls arm servo temperature by sending `STATUS <id>` to the same persistent
session (via `D1ArmGestureBridge`) and reading back `TEMP <id> ...` — it never
opens a second `gesture_play` process.

### Smoothness approach (mirrored in the browser preview)

Playback builds a control-point path = `[HOME, kf0, kf1, …]`, then samples it at
the control rate with a **Catmull-Rom** spline through the points (C1-continuous,
passes through every keyframe) using **linear time within each segment**, so the
arm keeps moving through interior keyframes. A per-segment smootherstep time-warp
was intentionally removed: it forced zero velocity/acceleration at *every*
keyframe, which made dense (teach-recorded) gestures stop-and-go / stutter. The
gentle ease out of / into HOME at the two ends now comes from the position-mode
servo's own acceleration limit (`--vel-ratio`/`--acc-ratio`), not a time warp.
Keeping the mapping per-segment (peak velocity ∝ 1/duration) is also what lets
the playability limiter stretch each segment independently. The omakaseos
`d1_motion_preview.html` viewer uses the *same* maths so the on-screen motion
matches the robot (the browser has no servo, so its HOME ends look a touch
sharper than hardware; the interior matches). Tune via `--rate-hz` and the
position-mode `--vel-ratio`/`--acc-ratio`.
