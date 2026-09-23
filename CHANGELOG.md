# Changelog

Consumers pin this repository by commit and pip decides whether to reinstall by
*version*, so every change that moves what a consumer imports carries a version
bump (`tools/check_version_bump.py`). This file says what the bump was for, and
in particular what it **breaks** — the repository's rule is a clean break with a
loud reason, not a legacy path kept alive beside the new one.


## 0.16.0 — unreleased

The root-cause redesign of PR #21 (design `DESIGN.md`, steps 1-9): the
vocabulary a model and a Python caller use is rebuilt around a `Direction`,
the executor state is typed from the daemon's own OpenAPI document, contact
and scene clearance become kit concepts, perception and the agent loop move
into the wheel, `handover` plans both arms, and every per-robot number lives
in one typed robot profile. **It is a clean break**: nothing below is kept
alive beside its replacement except the one transitional `RawState`
accessor set, removed in 0.17.

### Teach: hand-taught omakaseos gestures over d1-firmwared (`mkit-teach`)

Shu, 2026-09-23: the teaching tool that produced the omakaseos gesture CSVs
(d1-sdk `gesture_record`, driven by omakase-core's `/d1_teach` panel) stopped
working at the firmwared migration — playback was ported to daemon
trajectories, teach was not, and `gesture_record` cannot reach the arm while
the daemon owns it. It comes back here, over the generated client.

- **New `manipulation_kit.teach`** (`docs/teach.md`): `record` (hand-guide in
  `force_compliance` — gesture_record's own parameters — or with the holding
  brakes released for a timed window, or idle; sample both arms at 20 Hz with
  timestamps and gripper closedness; always engage + `recover` on exit),
  `process` (gesture_record's smoothing / collinear reduction / HOME snap /
  `limitJointDynamics`, plus Douglas–Peucker, min spacing and the panel's idle
  trim), `check` (the DAEMON's Catmull-Rom through `MotionGuard` with limits
  unclamped, the coupled wrist-roll limit, and per-segment velocity /
  acceleration caps; FK flange sweep; ASCII joint strips), `export`
  (check-then-write; `--force` stamps `# mkit-teach: UNSAFE=`), `play`
  (through `FirmwareExecutor`: lease, approach to HOME, measured first knot,
  arrival barrier, settle; refuses UNSAFE without `--no-safety`), `registry`
  (the `gesture.yaml` entry, `source: teach`).
- **New CLI `mkit-teach`** — `record | keyframes | export | check | play | register`.
- **New `FirmwareExecutor.play_waypoints(points)`**: upload an already-timed
  dual-arm trajectory with the plan path's contract checks, polling and
  cancel-on-exit (`_play` now shares `_run_job` with it).
- **New `FirmwareClient` verbs** over generated operations: `arm_mode`
  (`ArmModeCommand`), `arm_recover`, `arm_tool_state`, `brake_release` /
  `brake_engage` / `brake_state` (d1-firmware PR #92), and `operation()`,
  which raises the new **`OperationUnavailable`** when the client's document
  lacks a route — the bundled 0.3.0 document has no brake routes.
- The CSV contract is pinned from the firmware reader: 15 columns, degrees,
  R = physical LEFT, row 0 / last row replaced by HOME and row 0's duration
  ignored. Row 0 is therefore HOME itself (a deviation from gesture_record,
  whose first row the new player would have dropped).

Follow-up, Shu 2026-09-23 16:52Z / 16:56Z ("teaching is easier with the
brakes released; compliance was hard to move", "for teaching, drop the
guard") — **breaking for `mkit-teach` callers**:

- **Recording default = brake release (hand guiding).** `record()` /
  `mkit-teach record` default to `guide="brake"`: idle, then the generated
  `brake_release` with `RELEASE_BRAKE` and a timed window the daemon closes
  itself, renewed every third of the window. The contract is printed and ONE
  typed `HOLDING` covers the session and all taught arms. `--guide` and
  `--hand-guide` are gone: `--compliance` selects gesture_record's
  `force_compliance`, and `--no-brake` gives servos-off-only.
- **Operator flow**: the take starts at HOME. Without `--no-home-start` the
  arms are driven there first. The start is refused when a taught joint is
  more than 2 deg off HOME. Then a 3-2-1 countdown (`--countdown`) runs, and
  `t = 0` of the take is the instant the release is acknowledged. Stop engages
  the brakes FIRST, before `recover` or anything else. The renewal thread
  stops under a lock, so no release can land after the engage.
- **Export HOME rules** (`process`): the brake-release sag is cut. That is the
  last sample within 0.5 s of the start whose joint speed exceeds 8 deg/s;
  `--sag-max-s`, `--sag-vel`, `--no-sag-trim`. The motion starts at HOME and
  blends into the stream. The last recorded pose is KEPT, and a return-to-HOME
  row is appended at a constant joint speed (`max|dq| / 20 deg/s`,
  `--home-speed`), so long and short returns move alike. Ends within epsilon
  of HOME snap as before. The golden wave take is geometrically unchanged.
- **Guard is advisory for teaching.** In `check`, MotionGuard clearance
  (body / chest / arm-arm / self) is now a `GuardFinding` in
  `CheckReport.guard_findings`, printed as a WARNING with the closest
  distance, the frames and the time. It is no longer a violation. Joint
  limits (incl. the coupled wrist limit), velocity / acceleration and
  timing are still hard; a timing check (finite, 0-based, strictly
  increasing) is new. `export` never stamps UNSAFE for a guard-only finding
  and records `# mkit-teach: min_clearance=…` (and `guard_advisory=…`).
  `play` announces the guard warning before moving and still refuses UNSAFE
  without `--no-safety`. **d1-firmwared still refuses such an upload.** On
  main d090ac4 `arm_trajectory.rs::validate` guards every 1 ms sample with
  the same model and margins. It also does NOT refuse out-of-limit joints:
  its guard clamps for the check and the raw pose is commanded. See
  d1-firmware issue #101 (teach-mode relaxation question + that gap).
- **The daemon's clearance guard is always on** (Shu 2026-09-23 21:14Z,
  reversing 17:50Z). `mkit-teach play` sends no `guard` field; there is no
  `--guard`, no `play(guard=)`, no `FirmwareExecutor.play_waypoints(guard=)`
  and no `trajectory_guards()`. The daemon checks every sample with its
  realistic geometry and 20 mm margin; the kit's `check` clearance findings
  stay advisory WARNINGs, printed before anything moves together with the
  fact that the daemon will refuse the same violations. (The per-job
  relaxation d1-firmware PR #102 added is being removed from the daemon API
  in PR #106's follow-up.)
- **Bundled client snapshot → d1-firmware PR #102** (`1ec29096…`, branch
  `feat/trajectory-guard-speed-only` @ 39537a4), superseding the
  `a9c8b0d2…` snapshot below. It still carries the optional trajectory
  `guard` field, which the kit no longer sends; PR #106's head (9ac6677)
  still has it too, so the snapshot is refreshed once a document without it
  exists. d1-2 serves `a9c8b0d2…` until
  its daemon is redeployed with PR #102, so connecting to it regenerates the
  client again, as designed. `guard` is optional in the status schema, so a
  bundled client still decodes an older daemon's status.
- **Confirmed mode transitions + a teardown the controller accepts** (first
  live `mkit-teach record` on d1-2, 2026-09-23 20:16Z, d1-firmware main
  5e7e23a). The idle request answered in 0 ms and the brake release 5 ms
  later was refused (the controller reported idle only 11 ms after the
  request); the recover 2 ms after the engage was refused (`RESET1` code 8,
  mode flapping idle/error); an arm left idle by hand 40.2 deg from its
  command stopped the session in `FirmwareExecutor.__enter__`. Now:
  new `FirmwareExecutor.wait_for_mode()` (poll the generated `ArmState`
  until the REPORTED mode matches; `ModeUnconfirmed`, new, on timeout or an
  unexpected `error`), used by `position_mode()` and by `record` after every
  mode request; order **idle → confirmed → 3-2-1 → release = t 0**; teardown
  = brakes engaged → steady mode + stationary for 0.3 s → `recover_arm()`
  (confirmed `position`, 3 attempts 1 s apart) → else the arm is left idle
  with the brakes engaged and `exit_problems` / a WARNING line say how to
  recover (console Arms → Recover, `POST /v1/arm/{side}/recover`);
  `FirmwareExecutor(recover_on_entry=True, announce=)` (`recover_idle_arms()`)
  recovers every non-position arm at its measured pose before position mode,
  announced — set by `mkit-teach record` and `play` only; the agent loop's
  executor still refuses. `record.HOLD_RATIO` is gone (`RECOVER_RATIO` in the
  executor).
- **Every firmware request waits the document's `x-timeout-seconds`**
  (second live run, dc51dbe, 20:46Z: the take recorded fine, then the
  teardown recover was cancelled by the daemon at exactly 2.0 s — the
  client's blanket timeout hung up on a route declared at 65 s).
  `FirmwareClient.timeout_for(method, path)` maps a concrete path to its
  route's documented bound, floored at the client's `timeout`; `_send` uses
  it for every generated operation and `request()`. New `ClientTimeout`
  ("the kit gave up", not a refusal) and `RecoverFailed` (per-attempt
  reason); `recover_arm` never re-issues after a client timeout, and
  `exit_problems` says whether the daemon refused or the kit gave up.
- **Teach operator flow after d1-2 take2** (Shu 2026-09-23 21:09Z / 21:12Z)
  — **breaking for `mkit-teach` callers**:
  - **Speed: one `process.SpeedPolicy`**, used by export (stretch), check and
    play (judge); `MAX_JOINT_VEL_DEG_S` / `MAX_JOINT_ACC_DEG_S2` and
    `check_gesture(max_vel_deg_s=, max_acc_deg_s2=)` are gone
    (`speed=SpeedPolicy(...)`; `KeyframeOptions.speed`). **Default ceiling
    150 deg/s, 600 deg/s^2** (was gesture_record's 25 / 120, now
    `LEGACY_SPEED`): take2's ~130 deg/s J1 swing had been stretched 3.83 s ->
    5.52 s. The caps travel in the CSV (`# mkit-teach: max_joint_vel=…
    max_joint_acc=…`, plus `speed_stretch=…` when it stretched), and
    `check` / `play` hold a CSV to its own caps unless `--max-joint-vel` /
    `--max-joint-acc` override. Export prints the stretch only when it
    stretched. This ceiling is the only gesture speed policy (the daemon
    caps 350 deg/s per step, the omakaseos player none). HOME connect /
    return stays a constant 20 deg/s.
  - **Named takes**: `mkit-teach record` asks the gesture name first
    (before the HOLDING contract) or takes `--name`; the take is
    `<teach dir>/<name>.json` (`$MKIT_TEACH_DIR`, default `~/teach`), an
    existing one is overwritten only on request (`--yes` in scripts); a
    positional path still works. The name is stored in the recording.
  - **`export` writes beside the take** when `out` is omitted:
    `<take dir>/<name>_motion.csv`, the name from the recording or `--name`,
    printed as an absolute path. Sentiment / usage are asked for on a
    terminal when omitted, else neutral / filler.
  - `[saved]` and `check` print the same duration (the daemon spline's end;
    new `Gesture.played_s` replaces `total_duration_s`, which counted row
    0's ignored duration).
  - **Every subcommand ends with `Next:`** — the recommended next command
    with absolute paths (`next_steps()`), or `To fix:` with the recover /
    re-run command on a failure.
  - `record` also asks **which arm(s)** after the name (`both/left/right`,
    b/l/r; `--arms` skips it) and names only those in the contract.
    **Ctrl-C while recording is a Stop** that keeps the take (new
    `RecordAborted` when fewer than two samples); `To fix:` appears only
    when an arm was left without a position hold or the run failed, and
    **no hint ever suggests `--yes`** (it skips the HOLDING confirmation;
    the retry is `mkit-teach record --name <name> --arms <arms>`).
  - **The wrist is kept as taught** (Shu 21:51Z: task4's L7 35.7 deg was
    exported as 0.0). `KeyframeOptions.lock_wrist` / `--free-wrist` are
    replaced by `pin_wrist` / `--pin-wrist` (off by default) and
    `wrist_noise_deg` (2 deg): a wrist joint that moved less than that is
    pinned to HOME, and export says so.
  - **HOME legs at the take's own speed**: `home_speed_deg_s=None` (default)
    = the take's peak joint speed after smoothing, clamped to 20..90 deg/s;
    `--home-speed` overrides; keyframe mode keeps 20.
  - **Export prints where the time went and what each joint kept**: new
    `process.Reduction` (`reduce_samples` / `reduce_poses`) with a
    `timing: recorded … -> body … (speed cap stretched N knot(s), +x s) +
    HOME connect … + return … = … s` line and per-joint ranges recorded ->
    exported, flagging pinned and LOST joints.
- **Bundled client snapshot refreshed** (consumer-sweep item from the probe
  entry below, done here): `_client/` is regenerated from spec `a9c8b0d2…`
  (0.3.0 with d1-firmware PR #92's brake routes). That document is what d1-2
  serves at `GET /openapi.json` (fetched 2026-09-23), byte-identical to
  d1-firmware main d090ac4 `openapi/d1-firmwared.v1.json`. It adds 10
  generated files and 3 operations. The brake verbs are now real generated
  operations out of the box, and connecting to d1-2 no longer regenerates.
  Nothing else on the executor path changed.

### Grasp: the measured width outranks the declared one

First live Astra run on d1-2 (2026-09-23 03:46Z, trace
`astra-20260923-1146`, record 6): the model declared `tape` 50x50x18 mm, a
right tip grasp stalled with the daemon reporting holding=true,
jaw_stalled=true and a 57.1 mm pad gap — the hand WAS holding the roll — and
`Holding` said FALSE ("wider than tape's 50.0 mm — the jaws never reached
it") because the gap fell outside a +-4 mm window around the DECLARED width.
The model believed it, let go, and spent its remaining turns.

- **BREAKING (verdicts): `Holding` judges a stalled gap by `grip_fit`**
  (`primitives/verifiers.py`). Holding + stalled is TRUE when the gap is a
  plausible width for the named object: inside
  `[declared / WIDTH_PLAUSIBLE_FACTOR, min(open_gap - clearance, declared x 2)]`
  and above `EMPTY_GAP_M` (6 mm, or half the declared width for something
  thinner). FALSE only when the jaws closed to (near) zero (`fit: "empty"`),
  stalled within one per-side clearance of the hand's open gap
  (`fit: "blocked"`; the clearance is the grasp reference's — 4 mm pad, 2 mm
  tip — so `Holding(reference=)` is new and `Grasp` passes its own), or the
  producer reports nothing held. More than a factor of two off the
  declaration is UNKNOWN (was FALSE). The +-4 mm window survives only as
  `measured.matches_declaration` / `width_window_m`, a note on the
  declaration; when it is off, the verdict says so ("declared 50.0 mm, width
  corrected to the MEASURED 57.1 mm") and carries `width_correction`.
- **NEW `ObjectView.size_provenance`** (`None` | `"measured"`) and
  **`world.with_measured_width(item, frames, axis, width_m)`**: the object's
  extent along the jaw axis becomes the measured gap.
  `SceneSource.measured_width` / `LiveRobot.measured_width` apply it to the
  world source, and the agent loop does so after a TRUE grasp that corrected
  the width — so Carry, Place and the scene gate plan with the real size. The
  trace records it in the new `DecisionRecord.corrections`, and the model is
  told in the verb's answer.
- `examples/agent/astra_loop.py`: without `--task`, the task is derived from
  `--object` / `--destination` (`default_task`: "put the {object} into the
  {destination}"); the run above was told "put the red block in the box".
  `DEFAULT_TASK` is now `default_task("red_block", "box")`.
- `examples/agent/astra_loop.py`: a scene-tool turn (`declare_scene`,
  `locate`) prints its own answer, never a verdict — a wrist `locate` line
  showed its correction nudge's verdict in the verb's column.

### Calibration schema: the kit ships the reader, the robot holds the values

#### BREAKING: the kit owns the calibration schema and reader; the robot holds the values

Decision (Shu, 2026-09-23): no per-robot number ships in this wheel. Until now
d1-2's measured profile was committed as `description/profiles/d1-2.json` and
installed as package data; every robot would have needed a kit release to
change its own camera. It now lives ON THE ROBOT, in one file in the new
format **`omakase.camera_calibration/2`**, default
`~/.config/omakase/camera_calibration.json`, written by seiryu-calib.

(0.16.x is not released; the bump is a patch only so that 0.17 stays the
release that removes the transitional `RawState` accessors. It breaks every
caller of the API below, and pip needs a version that moves.)

- **NEW `manipulation_kit.description.camera_calibration`** — the typed reader
  of `omakase.camera_calibration/2`: `load(path, *, allow_failed_gate=False)`,
  `parse(doc)`, `validate(doc)` (hand-written, no `jsonschema` dependency:
  schema string, required and unknown keys, coefficient count per distortion
  model, finite numbers, unit quaternions, principal point inside the stream,
  a gate needs at least one check), dataclasses `CameraCalibration` /
  `Camera` / `Intrinsics` / `Distortion` / `Mount` / `Pose` / `Gate` /
  `Provenance` / `Hand`, `DEFAULT_PATH` and `installed_path()` (the default
  file when this machine has one) for entry points. No environment variable:
  the agent examples read none, by rule. The JSON Schema ships as package data,
  `description/schemas/omakase.camera_calibration-2.schema.json`.
- **Gates are enforced by the reader.** Every layer (`intrinsics`, `mount`)
  carries `gate.verdict PASS|WARN|FAIL`. A FAIL layer raises
  `FailedCalibrationGate` naming the camera, the layer and the reasons,
  unless the file's gate records an `override` with a reason or the caller
  passes `allow_failed_gate=True`. WARN is accepted with a `UserWarning`.
- **`RobotProfile` is built from that file**:
  `RobotProfile.from_camera_calibration(path|doc|CameraCalibration,
  allow_failed_gate=)`, `RobotProfile.load(path)`. The head mount's ABSOLUTE
  `head_link -> optical` pose becomes the `HeadMountDelta` on the nominal the
  file records (`delta = nominal^-1 * measured`), so `HeadCamera` behaves
  exactly as before (round trip to 1e-9 in the tests). `left_wrist` /
  `right_wrist` become `wrist_cameras["left"|"right"]`: `kannala_brandt` is
  `fisheye`, `none` (or an all-zero Brown-Conrady) is `pinhole`, a non-zero
  Brown-Conrady wrist is refused. A measured WRIST mount is refused too: the
  kit's wrist camera is still the nominal plate geometry and would silently
  ignore it. `hand.open_gap_m` becomes `HandMeasurement`.
- **Removed**: `description/profiles/` and its package-data line,
  `PROFILES`, `RobotProfile.named`, `RobotProfile.from_files`,
  `RobotProfile.from_json` / `to_json` (seiryu-calib is the writer),
  `RobotProfile.notes`, `HeadMountDelta.from_json` / `from_head_calibration`,
  `WristIntrinsics.from_json`. An old `manipulation_kit.robot_profile/1` file
  is refused with the migration.
- **`RobotProfile.resolve(ref)` takes a PATH** (tried as given, then relative
  to `relative_to`) or a profile; `None` stays `None`; a bare name like
  `"d1-2"` raises `NotACalibrationFile` saying where the values live now.
  `load_scene(..., allow_failed_gate=)`, `scene_robot_block` / `with_profile`
  accept a path as `profile=` too; `LiveRobot.from_flag` / `.firmware` take
  `profile=PATH` and `allow_failed_gate=`.
- **CLI flags**: `astra_loop.py --robot-profile PATH` and `perceive.py
  --robot-profile PATH` default to
  `~/.config/omakase/camera_calibration.json` when it exists (on the robot),
  else — for `astra_loop.py --scene` — the file the scene names, else none.
  NEW `--allow-failed-calibration` on both.
- **Scenes**: `"robot": {"profile": "PATH"}` is a path relative to the scene
  file. `examples/agent/scenes/d1-2_tape_cup.json` no longer names a profile.
- d1-2's file (the numbers of the old `profiles/d1-2.json`, verbatim) is the
  test fixture `tests/data/d1-2.camera_calibration.json`; the old v1 file is
  kept as `tests/data/robot_profile/d1-2.robot_profile-v1.json`, the
  reference the v2 file must reproduce.

#### Migration

| old | new |
|---|---|
| `RobotProfile.named("d1-2")` | `RobotProfile.load("~/.config/omakase/camera_calibration.json")` (on d1-2), or the path of a copy |
| `--robot-profile d1-2` | nothing on the robot (the default file); `--robot-profile PATH` elsewhere (offline: `tests/data/d1-2.camera_calibration.json`) |
| `"robot": {"profile": "d1-2"}` in a scene | drop it (the robot's file is the default), or `"profile": "relative/path.json"` |
| `RobotProfile.from_files(name, head_calibration=, wrist=)` | `seiryu-calib migrate` those d1-inference files into the robot's v2 file, then `RobotProfile.load` |
| a `manipulation_kit.robot_profile/1` JSON | the same numbers as an `omakase.camera_calibration/2` file (head mount ABSOLUTE with its nominal) |
| a FAIL calibration layer used silently | refused; `gate.override` with a reason in the file, or `--allow-failed-calibration` |

### BREAKING — read this first

1. **`approach: "top_down"|"front"|"side_left"|"side_right"` → `direction`**
   on every verb (a `world.Direction`: an alias or `{axis, frame}`), and
   **`jaw_turn_deg` is removed** (the roll is the kit's one sweep). Map old
   names BY VECTOR — `side_left` is `right`, `side_right` is `left`.
2. **`RawState` is `arms: {side: JointState}` + `hands: {side: HandState}`**;
   the flat `joints/grippers/holding/commanded_grippers/stationary`
   accessors and keywords survive ONE release (0.17 removes them).
3. **Environment variables are gone**: `MKIT_DRIVEN_OPEN_GAP_M` (read the
   daemon's `open_rad`; a robot profile for a transport without one),
   `MKIT_SUPPORT_CLEARANCE_M` (`OperatorPolicy.droop_margin_m`), and all eight
   `ASTRA_*` (`OperatorPolicy` fields and `astra_loop.py` flags).
4. **`primitives/approach.py` is `primitives/orientation.py`**; the grasp
   point, standoff and fit moved to `primitives/grasp_geometry.py`.
5. **`examples/agent/{camera,live,mirror,trace}.py` are gone** — import
   `manipulation_kit.perception` and `manipulation_kit.agent`. The
   `perceive` extra is renamed **`perception`**.
6. The firmware executor's hand-written `ArmState`/`GripperState` are gone;
   state comes from the generated client's models.

#### Migration, for the consumer sweep (d1-inference, dx-inspect-robots, d1-isaaclab)

| old (0.15) | new (0.16) | known user (the sweep checks all three) |
|---|---|---|
| `Grasp(object=o, approach="top_down")` | `Grasp(object=o, direction="down")` | — |
| `approach="front"` / `"side_left"` / `"side_right"` | `direction="forward"` / **`"right"`** / **`"left"`** | — |
| `jaw_turn_deg=90` | *(delete it; the plan's notes say which roll was used)* | — |
| `from manipulation_kit.primitives.approach import tool_from_link7, TCP_P, TCP_R, ...` | `from manipulation_kit.primitives.orientation import ...` | **d1-isaaclab `scripts/eval/agent_eval/world.py`** |
| `approach.GRASPABLE_WIDTH_M`, `JAW_OPEN_M`, `grasp_point`, `standoff_pose`, `TIP_BELOW_TOOL_M` | `grasp_geometry.graspable_width_m(PAD, open_gap_m)`, `hands.d1.parallel_gripper.description.DRIVEN_OPEN_GAP_M`, `grasp_geometry.grasp_pose` / `standoff_point`, `PAD.lead_m` | — |
| `RawState(joints=..., grippers=..., holding=...)` | `RawState(arms={s: JointState(q=...)}, hands={s: HandState(closedness=..., holding=...)})` (old keywords still accepted until 0.17) | d1-isaaclab `agent_eval` executor |
| `MKIT_DRIVEN_OPEN_GAP_M=0.0605` | `HandState.open_gap_m` from the executor; `KinematicExecutor(open_gap_m=)`; `RobotProfile.hand` | d1-2 dry runs |
| `MKIT_SUPPORT_CLEARANCE_M=0.015` | `OperatorPolicy(droop_margin_m=0.012)` / `--droop-margin-m 0.012` | d1-2 operators |
| `ASTRA_GRIP_CAP` / `ASTRA_APPROACH_ALLOW` / `ASTRA_VEL_RATIO` / `ASTRA_ARRIVE_TIMEOUT_S` | `--max-grip` / `--allowed-directions` / `--vel-ratio` / `--arrive-timeout-s` (or `--policy FILE`) | operators |
| `ASTRA_SNAPSHOT_CMD` / `ASTRA_MAX_OUTPUT_TOKENS` / `ASTRA_REASONING` / `ASTRA_DEBUG` | `--snapshot-cmd` / `--max-output-tokens` / `--reasoning` / `--debug` | operators |
| `examples/agent/live.py` `LiveRobot`, `mirror.py`, `trace.py` | `manipulation_kit.agent.LiveRobot` / `KinematicMirror` / `DecisionTrace` | — |
| `examples/agent/camera.py` `PinholeCamera`/`HeadCamera` | `manipulation_kit.perception` | — |
| a scene's `robot.wrist_camera` / `robot.hand` numbers | `"robot": {"profile": "d1-2"}` (or `--robot-profile`); scene keys still override | everyone with a d1-2 scene |
| `pip install '.[perceive]'` | `pip install '.[perception]'` | CI |
| `candidates_for(approaches=...)` | `candidates_for(directions=...)` | — |
| `reach.plan_chain(..., approach=, jaw_turn_deg=)` | `reach.plan_chain(..., direction=, contact=)` | — |

The consumers are updated in a following sweep, not in this repository.

### Fingertip grasps onto a surface finish by contact (d1-2 tip trial, 2026-09-23)

The d1-2 tip trial (8 mm slab of business cards, 91 x 55 mm, left hand,
three cycles, zero controller errors) closed the jaws on nothing three of
three. Two findings:

* **Height — fixed here.** The plan stopped the finger tips 14.9 mm over the
  wagon: 3 mm `SUPPORT_CLEARANCE_M` + the operator's 12 mm
  `ClearancePolicy.droop_margin_m`. That margin is the ~1 cm sag of a LONG
  reach (run 7, x 0.48); at x 0.40 the arm did not sag, so the tips stopped
  ~7 mm above the slab. A constant sag guess cannot be right for a slab
  thinner than the guess. **`Grasp(contact="tip")` descending onto a known
  surface (or onto anything thinner than `TIP_CONTACT_THIN_M` = 58 mm) now
  finishes BY CONTACT** (`grasp_geometry.descends_by_contact`): the plain
  descent stops with the tips `TIP_SEARCH_START_M` (15 mm) over the modelled
  surface, then a `ContactStep` searches along the travel for at most that
  plus `CONTACT_OVERTRAVEL_M` (5 mm past the modelled top), ending at a
  `TIP_CONTACT_NM` (3.0 Nm, unmeasured) joint-torque rise; the runner
  re-commands the measured posture and the jaws close there. The droop
  margin is not applied on that path; pad grasps (which must not touch)
  keep the fixed height and the droop margin. A `KinematicExecutor` travels
  the whole search and reports `made=False, stopped_by="max_travel"` —
  plans and dry-runs still verify, and say no contact was measured. The
  plan's waypoints gain a third, `contact_limit`; `plan.contact_steps`
  appears on those plans (golden cases with a fingertip descent change
  accordingly). `planning.leg_knots` is the one knot/distance builder for
  every contact leg (probe, press, fingertip grasp).
* **Jaw axis — no kit bug; the slab lay 90 deg from its declaration.** Shu's
  photos show the open jaws spanning the slab's 91 mm side. The recorded
  plan's grasp quaternion `[0, 1, 0, 0]` puts TCP x (the jaw axis) along base
  -x; the kit's FK of the executed joints, and an independent walk of the
  gripper description's prismatic finger joints, both give the finger
  travel as base (-1, 0.001, 0). The slab was declared at yaw 1.5708 (91 mm
  along base y), so it presented 55.1 mm along that axis and `fits()` —
  which measures the same `orientation.jaw_axis` — correctly passed it. The
  photos are taken from the robot's RIGHT side, where "left-right" is the
  robot's forward axis: the slab lay long side along base x (yaw 0), where
  the executed posture presents 91 mm and `fits()` refuses it
  (`object_too_wide`). Declared as it lay, the left wrist cannot turn the
  jaws across base y at (0.403, 0.10) (coupled J7 limit) and the plan is
  refused. `tests/primitives/test_tip_grasp_by_contact.py` pins all of it.
  **Open, not fixed here:** a quarter-turn slab sits exactly on
  `align_tool`'s half-turn fold: yaw 1.5708 plans J5 +2.8 deg, yaw
  `math.pi / 2` plans the other half turn at J5 +173 deg (1 deg from the box
  that flipped the probe trial). The roll sweep never tries the squared
  roll's half turn, which is also why yaw 0 is refused rather than planned
  at the other wrist (it plans, J5 +115.5 deg, when a pi candidate is added).

### Straight legs stay on their line: no xy drift over repeated probes (d1-2, 2026-09-23)

The passing probe gate (02:31Z) showed the ten table contacts walking 88 mm
in y and 18 mm in x although every leg was vertical. The IK converges on
Link7 within 2 mm / 0.05 rad; the tool point is 100 mm further out, so a
"converged" posture left the tool 7-9 mm to the side, re-solving from it was
a no-op, and `_straight` walked on inside the 12 mm transit window — always to
the same side, because the READY pull biases where the solver stops.

- `Waypoint.exact` (new, `allow_via=False` legs only): the leg's LINE is what
  it reports. Set on `Nudge` and on the contact leg of `Probe`, `Press` and a
  fingertip `Grasp`. Such a leg is solved with `planning.straight_tuning(arm)`:
  the arm's tuning tightened to `STRAIGHT_IK_POS_TOL_M` 0.2 mm /
  `STRAIGHT_IK_ROT_TOL_RAD` 1 mrad at Link7 (never loosened) with the
  null-space READY pull OFF, each knot seeded from the previous solution; a
  knot that tuning cannot solve falls back to the ordinary solve, and the
  gates below judge the result either way.
- **Tighter gate:** an exact leg's knots and end are judged against
  `STRAIGHT_PATH_TOL_M` (= `ARRIVE_TOL_M`, 3 mm) instead of `PATH_TOL_M`
  (12 mm); one that cannot hold its line within 3 mm is refused
  (`knot_exhausted` / `off_line` / `arrival`). Other legs are unchanged — a
  long straight transit (Carry across the wagon, Place into a shelf bin)
  NEEDS the READY pull to keep the elbow off the body and was refused
  `guard_reject` / `joint_limit` without it, which is why this is per leg.
- `Waypoint.knot_m` (new, optional, only finer than `safety.MAX_STEP_M`): every
  contact leg is knotted every `planning.CONTACT_KNOT_M` = 5 mm, so the
  joint-space interpolation the daemon plays between knots stays on the line
  wherever the contact stops it.
- Golden plans regenerated: only the 20 `nudge` cases change (same step count,
  final joints within 0.05 rad), because a nudge is now solved exact.
- `GuardedArm.tuning` (new read-only property): the tuning `solve_ee` uses.

Replay (`tests/executors/test_probe_session_replay.py`): ten probe/lift
cycles, largest xy offset from the first contact 74.9 mm -> 0.4 mm; a
+50 mm Nudge ends 8.7 mm -> <= 1 mm from its commanded xy.

### Repeated probes: no wrist flip, no zero-duration knots (d1-2, 2026-09-23)

The d1-2 probe trial (`docs/probe-hardware-trial.md`, fcc2087) stopped at
table probe 4 of 10 with `HTTP 400: trajectory values must be finite with
increasing times`, the left wrist at J5 = 171.5 deg (box 173) and J7 turned
from -55 to +42.5 deg. Replayed on a fake daemon from the standoff posture
alone (`tests/executors/test_probe_session_replay.py`), both faults
reproduce, and both are fixed at the producer:

- **The wrist flip.** `Probe` kept "the roll the hand has" by passing its jaw
  axis to `align_tool(roll_to=...)`, which folds a jaw axis to the half-turn
  representative nearest the PADS_DOWN *seed*. A hand whose jaw axis pointed
  the other way (the d1-2 standoff) was turned 180 deg in place — a
  105-sample re-aim that took J5 from -5 to +169 deg — and back on a later
  probe, and over again, until J5 sat against its stop.
  `align_tool(..., keep=r_now)` now tilts the CURRENT orientation by the
  smallest rotation onto the direction; `Probe` uses it (its roll sweep of
  0 / +-90 / 180 deg is now relative to the hand, as documented).
- **Posture continuity.** `Probe` takes a roll only when every posture of
  its plan keeps `PROBE_BOX_MARGIN_DEG` (10 deg) from every box limit; when
  only nearer postures plan it refuses **`joint_limit`** naming the joint,
  instead of planning a leg the solver pins at the stop. From the live end
  posture it now refuses. (The IK's existing null-space pull toward READY
  still moves the elbow a few degrees per probe/lift cycle; it converges —
  replay: J1 -20 -> -4.5 deg over 13 cycles, every uploaded posture
  keeps >= 24.9 deg from its box, J7 the nearest.)
- **Zero-duration knots.** With J5 pinned the leg's solver re-solved the
  same posture (35 of 41 knots identical); each got the same distance along
  the leg, hence the same time. `_contact_plan` drops a knot identical to
  the one before it, `ContactStep` refuses a distance that decreases or a
  non-finite knot, and `ContactStep.timed_path()` is strictly increasing
  (a knot that moves joints without advancing gets
  `CONTACT_KNOT_MIN_DT_S` = 20 ms); `duration_s()` is its last time.
- **Checked before upload.** Every firmware upload (`_play` and the contact
  leg's `trajectory_start`) goes through
  `executors.firmware.client.check_waypoints`: the document's `Waypoint.t`
  contract ("seconds, starting at zero, strictly increasing") plus finite
  values. A violation raises the new **`TrajectoryInvalid`** (a
  `FirmwareUnavailable`) naming the offending knots and times, and nothing
  is sent. The daemon's 120 s / 10 000-point ceilings are not in the
  document and are not guessed (d1-firmware issue: publish them as
  `maximum` / `maxItems`).
- **Bundled client snapshot was behind.** d1-2 now serves spec `a9c8b0d2…`
  (d1-firmware PR #92 added the brake routes); `ensure.py` regenerated the
  client from it at connect time, as designed. The bundled `_client/`
  snapshot (`388bcd08…`, 0.3.0 before #92) was refreshed to `a9c8b0d2…` in
  the teach follow-up above.

### Coupled wrist-roll limit (d1-2 hardware, 2026-09-22)

The per-joint box (J6 +/-60, J7 +/-90 deg) is not the D1 wrist's envelope: the
hand, wrist camera plate and cables catch on the J6 link, so |J7| stops at
65 deg with J6 = 30 and at 39 deg with J6 = 55 (measured by hand on d1-2,
2026-09-22, both signs equal). The live Approach of 22:42Z solved J7 = -90 at
J6 = 55.2 and the wrist stopped at -39.5.

- `config/coupled_joint_limits.json` holds the table per arm revision
  (`d1-lite-7dof-wrist-camera-plate-v2`), with provenance; read by
  `manipulation_kit.arms.coupled_limits` (`CoupledJointLimit`,
  `wrist_roll_limit_deg(j6_deg)`). Linear between the points, linear beyond
  them, capped at the box, 5 deg margin. |J6| < 30 is extrapolated and says so.
- `solve_ik(..., coupled=)` projects every update onto the coupled limits as
  it does onto the box, so no solution past them is returned; `GuardedArm`
  (and `D1ArmKinematics`, `build_kinematics(coupled=)`) carries them per side
  and passes them in. The READY-seed search drops seeds that violate them.
- `GuardedArm.posture_violation(side, q)`: the box and the coupled limits in
  one check; `solve_ee` applies it to every solution and `joint_ramp` to its
  goal. A failure the coupled limit caused is refused `joint_limit` (new in
  `PLAN_REASONS`, and a via reason) with the limit named in the detail, not
  `ik_fail`; Approach / Grasp / Probe then try their other rolls.
- A plan whose postures come within 10 deg of a coupled limit says so in
  `notes` ("near the coupled wrist_roll limit: ...").
- A leg whose shape is the promise (`allow_via=False`: descents, lifts,
  nudges) is refused (`ik_fail`, stage `off_line`) when a solver step leaves
  its straight line by more than `PATH_TOL_M` — with the wrist held inside the
  limit the solver can change posture branch and the clamped steps toward it
  carried the tool 110 mm off a 17 cm leg.
- `reach.HANDOVER_MEETING_POINTS_M` re-surveyed under the limit: the old five
  (x 0.35-0.45, z 0.20-0.35) all needed a wrist roll the hardware does not
  have; the new five of a 150-point grid are near the chest, x 0.30-0.35,
  z 0.20-0.25.
- **Breaking — capability lost.** Side approaches that needed
  |J7| > limit(J6) are now refused with `joint_limit` (or `guard_reject` on
  the branch that is left): every side-on approach/grasp of the golden
  `side_shelf` bottle (cases 139-144) and the d1-2 cube's side grasps (72,
  76) — recorded as `COUPLED_REFUSED` in `tests/primitives/test_golden_plans.py`.
  A redesign of the side-approach roll or a hand without the camera plate
  would restore them. Side approaches still plan close to the chest (golden
  scene `side_chest`, bottle at x 0.30). Top-down picks on the centre line
  far out (blocks-eval `block_blue` at (0.451, 0.005)) are refused by both
  arms; 2 cm nearer / to the left they plan. Plans that used |J7| past the
  limit otherwise change roll or route (golden: 27 cases replanned).
- **d1-3 is unmeasured.** The table was measured on d1-2 only; d1-3 carries
  the same arm and plate and gets the same table until measured.

### Executor state

### The last leg is measured (d1-2 Approach miss, 2026-09-22)

- **A run no longer says `completed` without measuring where the arm ended
  up.** Both runners (`run_steps` and `FirmwareExecutor.run_plan`) now run the
  joint-space arrival barrier before every `SettleStep` and at the end of the
  plan whenever the last joint leg was not already gated; the report carries
  it as an arrival labelled `end of motion`, with the tool-point miss when a
  kinematic model is at hand, and a miss stops the run with
  `barrier_failed` and a typed refusal. `SettleStep` alone only ever said the
  arms were STATIONARY. On d1-2 an `Approach` ended with J7 at -39.5 deg
  against a -90 deg command (tool 167 mm / 53 deg off) and reported
  `completed: true, arrivals: []`. **Breaking for callers** of executors
  without `wait_arrived` (or a `RecordingExecutor` without
  `pretend_arrived`): a plan that moves the arm now fails its final barrier on
  them, as a plan with a stroke already did.
- **The trajectory runner gates an `arrive` waypoint before a settle, a
  contact leg and at the end of the plan**, not only before another joint leg
  or a stroke. A gated last waypoint used to go unmeasured on the default
  firmware transport while the streamed one gated it.
- **`Approach` marks its standoff `arrive`**, the same tool gate `Grasp`
  puts on the same pose. It had none.
- **`Probe` tries the planner's rolls** (`contact.PROBE_ROLLS_RAD`: the
  wrist's own, then the quarter turns, then the half turn) instead of only
  the wrist's current roll; a roll other than the first is stated in the
  plan notes.
- `docs/probe-hardware-trial.md`: the trial script stops at the first
  refused probe instead of lifting after it, and refuses to start from a hand
  more than 20 deg from fingertips-down.

### Executor state (redesign step 1)

- **The bundled d1-firmwared client is regenerated from the document the
  daemon on d1-2 actually serves** (d1-firmwared 0.3.0, `GET /openapi.json`,
  sha256 `388bcd08…`, 116 paths). The previous snapshot was spec `a66b5a65…`
  (0.1.0), so every connect to d1-2 either regenerated into `~/.cache` or, on
  a machine without the generator, fell back to a client that provably did
  not match. `tests/executors/test_firmware_client_snapshot.py` now pins the
  d1-2 hash.
- **`RawState` is `arms: {side: JointState}` + `hands: {side: HandState}`**
  (+ `stamp`, and `extra` for genuinely foreign data only). `JointState` is
  `q, qd, torque_nm, mode, error_code, stationary`; `HandState` is
  `closedness, commanded, holding, jaw_gap_m, torque_nm, stalled, fault,
  open_gap_m`. A field a producer does not measure is `None`, never a
  default number. **Transitional, removed in 0.17:** the flat
  `joints`/`grippers`/`holding`/`commanded_grippers`/`stationary` read-only
  accessors, and the same keywords at construction (`RawState(joints=...)`),
  so `wire()` and d1-isaaclab's `agent_eval` executor keep working for one
  release. Migrate to `arms=`/`hands=`.
- **The firmware executor builds that state from the generated client's
  models, field for field.** `client.py` no longer parses JSON keys: its reads
  go through the generated operations (`arm.arm_state`, `gripper.gripper_state`,
  `neck.neck_state`, `slider.slider_state`) and are decoded by the generated
  models, then converted by `joint_state()`/`hand_state()`/`neck_state()`/
  `lift_state()`. **Breaking:** the hand-written `ArmState`/`GripperState`
  dataclasses are gone from `manipulation_kit.executors.firmware`;
  `FirmwareClient.arm_state()`/`gripper_state()` return the generated
  `ArmState`/`GripperReport` models. Velocity, torque, mode and error code are
  now typed fields; the `{side}_mode`/`{side}_error_code` keys in `extra`
  (74168de) are gone.
- **Faults stop the run in the kit.** A latched arm controller
  (`mode == "error"` or a non-zero `error_code`) stops `run`/`run_steps`/the
  firmware `run_plan` before the first byte and between steps with the new
  stop reason **`controller_fault`** (also a `RunRefusal` reason). A faulted
  gripper (`fault_code`, or a `fault`/`overload` stroke outcome) is no longer
  a settled one: `wait_gripper_settled` reads the daemon's `StrokeKind`
  instead of accepting two identical jaw readings, and the run stops with
  refusal reason **`gripper_fault`**. `StrokeReport` gains `kind` and `fault`.
- **The firmware executor publishes `HandState.commanded`** — what it last
  commanded and the daemon accepted (or the daemon's `target_closedness`). The
  runner's refusal to move a holding hand with no known command
  (`run_steps`, F9) no longer fires on hardware after every successful grasp.
  `jaw_gap_m`/`stalled` reach `GripperView` through the typed state, so the
  width-band half of the grasp verifier is live on hardware.
- **`MKIT_DRIVEN_OPEN_GAP_M` is deleted.** `description.DRIVEN_OPEN_GAP_M` is
  the nominal 51.96 mm again, for kinematics and sim. A robot's own opening is
  a measurement: the firmware executor derives `HandState.open_gap_m` from the
  daemon's `open_rad` through the description's new kinematic map
  (`gap_from_motor_rad`, 44.8 mm/rad), it travels on the new
  `GripperView.open_gap_m`, and `Grasp` judges fit against it
  (`approach.graspable_width_m`). `tool_revision()` no longer depends on any
  environment variable.
- **Plans are bound to the firmware they were checked on.** `PlanBinding`
  gains `firmware_spec` (from the new `WorldView.firmware_spec`), and
  `check_binding` refuses a plan whose recorded spec differs from the
  executor's `firmware_spec` (the sha256 of the OpenAPI document its client
  was generated from; `"kinematic"` for `KinematicExecutor`).
- **`run()` exposes `tool_tol_along_m` and `settle_timeout_s`**, and forwards
  `hz` to a transport's `run_plan` (it was silently dropped, L13).
- **The firmware executor sizes its own timing.** The blocking gripper stroke
  is bounded by `stroke_timeout_s`, which defaults to the document's own
  `x-timeout-seconds` for `POST /v1/gripper/{side}/set` (40 s on 0.3.0), else
  `DEFAULT_STROKE_TIMEOUT_S` (20 s); the example no longer builds the client
  with a blanket 20 s timeout (0b1a145). The trajectory schedule rate defaults
  to `schedule_rate_deg_s(vel_ratio)` = 140 deg/s x the ratio the executor
  installs (21ad024 moved into the executor); pass `max_joint_rate_deg_s` to
  override.
- **`neck_state()` / `lift_state()`** on the firmware executor, thin calls on
  the generated `GET /v1/neck/state` / `GET /v1/slider/state`, returning the
  kit's `NeckState(pitch_rad, yaw_rad, enabled, moving)` (daemon's logical
  pitch sign, unflipped) and `LiftState(height_m, moving, alarm)`. An optional
  capability (`read_neck`/`read_lift`); `KinematicExecutor` returns `None`.
- Examples: `astra_loop`'s arm-fault gate is deleted (the kit stops the run;
  the loop ends on a `controller_fault` report). The d1-2 hand (60.5 mm) now
  comes from the d1-2 robot profile (below), so `--dry-run --executor
  kinematic --scene d1-2_tape_cup.json` reaches `goal_verified` with no
  environment variable.

### BREAKING: a direction is a value, not one of four words

`approach: str` (`"top_down"`, `"front"`, `"side_left"`, `"side_right"`) and
`jaw_turn_deg` are **deleted** from every verb. There is no compatibility
shim: a call that still says `approach=` is a `TypeError` from Python and a
`bad_argument` refusal from `decode`.

* **`manipulation_kit.world.Direction(v, frame)`** — a unit vector and the
  frame it is expressed in: `"base"`, `"tool"` (the hand's own TCP frame) or
  `"object:<name>"` (that object's axes). `resolve(world, side=)` returns the
  base-frame vector and raises `FrameError` for a frame that does not resolve;
  it never falls back to base. `ALIASES` names `down`, `up`, `forward`,
  `backward`, `left`, `right`, `along_tool`; `parse_direction()` takes an
  alias, `[x, y, z]` or `{"axis": [...], "frame": ...}`; `toward()` builds
  "toward the cup".
* **Verbs:** `Approach`/`Grasp` take `direction` (default `down`);
  `Retreat` takes `direction` (default `-along_tool`, i.e. tool -z — the axis
  that was hardcoded); `Lift` takes `direction` (default `up`; it must rise).
  `Nudge` keeps `dx/dy/dz/dyaw`; its frames are the shared names
  (`world.direction.TOOL`/`BASE`) and its yaw is applied by the same roll
  function.
* **Migrating an old approach name — map BY VECTOR.** A `Direction` is the
  way the hand TRAVELS; the old side names said where it came FROM:

  | old `approach=` | new `direction=` | vector (base) |
  |---|---|---|
  | `top_down` | `down` | (0, 0, -1) |
  | `front` | `forward` | (+1, 0, 0) |
  | `side_left` | **`right`** | (0, -1, 0) |
  | `side_right` | **`left`** | (0, +1, 0) |

  `side_left` came in from the robot's left and travelled toward -y, so it is
  `right`. Mapping by name (`side_left -> left`) reverses the approach.
* **`jaw_turn_deg` is gone.** (Step 2 briefly kept it as a planner-only
  `roll_rad` field; step 3 below deletes that too — the roll is the kit's one
  sweep.) `reach.plan_chain`/`reach.choose_side`'s `approach=` is now
  `direction=`. The plan's notes say which roll was used.
* **`primitives/approach.py` is now `primitives/orientation.py`.**
  `align_tool(side, d_base, *, roll_to, roll_rad)` is the only place a
  quaternion is produced; `grasp_orientation(side, d_base, obj, frames,
  roll_rad=)` (was `approach, ..., dyaw_rad=`) takes a resolved base-frame
  vector (`grasp_point`/`standoff_pose` moved to `grasp_geometry` in step 3). Deleted: `APPROACHES`, `TOP_DOWN`, `FRONT`, `SIDE_LEFT`,
  `SIDE_RIGHT`, `APPROACH_DIRECTION`, `APPROACH_DOC`, `check_approach`,
  `direction()`.
* **One model allowlist.** `schema.NOT_MODEL_BINDABLE` is
  `("policy",)` (with `roll_rad` until step 3 deleted the field), and `decode(..., model_bindable_only=True)` (the
  default) now REFUSES those fields from a model instead of honouring them
  (`policy` used to be hidden from the schema but still accepted — Astra review
  5). Pass `model_bindable_only=False` from trusted Python. The example's
  `PLANNER_ONLY_ARGS`/`_hide_planner_args` and its inbound `pop()` are gone.
* **Schema:** `direction` is exported as
  `oneOf[enum of aliases, {axis: [x,y,z], frame}]`; its domain is
  `{"kind": "direction", "aliases": [...], "free": true}`.
  `schema.direction_doc()` is the prompt text, generated from the aliases.
* **One search order:** `types.GRASP_DIRECTIONS = ("down", "forward", "left",
  "right")` replaces the four hand-written orderings (prompt, example chain
  chooser, `ASTRA_APPROACH_ALLOW` default, `offer.candidates_for`).
  `candidates_for(approaches=)` is now `directions=` and defaults to all four
  (it was `("top_down", "front")`).
* **The vocabulary change alone moved nothing.** Every plan the example
  scenes produced — 392 cases, each verb, each old approach and jaw turn, both
  arms, the whole Approach->Place chain — was identical to the last float with
  the alias of the same vector before the grasp geometry below changed any of
  them on purpose (`tests/primitives/test_golden_plans.py`).

### Grasp geometry — where and how the hand meets an object

**BREAKING.** `primitives/grasp_geometry.py` is new and is the one place that
decides the grasp point, its descent floor, the standoff, the roll and the
fit. What it replaces is deleted, not deprecated.

* **Where on the hand: `GraspReference`.** `PAD` (the pad centre, 100 mm,
  the tips leading it by 29 mm, 4 mm clearance a side) and `TIP` (the finger
  tips, 129 mm, nothing leading, 2 mm a side), built from the hand
  description's measured `PAD_CENTRE_Z_M`/`PAD_TIP_Z_M` and its new
  `PAD_CLEARANCE_PER_SIDE_M`/`TIP_CLEARANCE_PER_SIDE_M`. The model picks one:
  **`Grasp(contact="pad"|"tip")`** (also on `Approach`, so it stands off for
  the same geometry; one enum in `arguments.py`). A 6 mm card on a table is
  grasped at the tips and refused (`object_too_flat`) at the pads. Every
  waypoint is still the pad-centre tool point (`orientation.TOOL_Z_M`); a tip
  grasp is converted onto it once (`grasp_geometry.tool_point`).
* **`tool_revision(reference=None)`** — a grasp plan records its reference
  (`...;reference=tip`), and `PlanBinding.drift` now compares every field
  BOTH revisions state: an executor states the hand only, so a tip plan runs on
  the hand it was made for and is refused wherever a pad plan is expected.
  `PlanBinding.of(..., reference=)`.
* **The descent floor is the measured support (L5, Astra review 4).**
  `support_of(world, name)` finds the level `SurfaceView` under the object
  (an underside declared below its top, or at most `SUPPORT_CATCH_M` = 30 mm
  above it); the tips keep `SUPPORT_CLEARANCE_M` over THAT, and over the
  object's own underside only when no surface is known. The plan's notes say
  which (`descent floor: table's top at z=...` / `...own declared
  underside...`), and the achieved-clearance check uses the same floor. The
  floor applies to any descending direction, backing the contact point out
  along the travel (straight up for `down`).
* **The standoff is measured from the silhouette (C.4).** `standoff_m` is the
  gap between the finger tips and the object's near face along the
  direction — not a distance from the grasp point, which left the tips under
  a 110 mm cup's rim. `Approach` stands exactly where `Grasp` descends from
  (it used to stand off the object's centre). `DEFAULT_STANDOFF_M` stays
  80 mm, so a top-down standoff over a 50 mm block is ~47-54 mm higher and a
  `forward` one 54 mm further back than before.
* **One roll sweep.** `roll_candidates(obj, frames, spec)` — the squared
  posture, then the quarter turns whose presented width still fits this
  reference and this hand — is the only roll generator. `Approach`/`Grasp`
  try it in order (a `Grasp` first tries the roll its `Approach` stands at)
  and the plan's notes say which roll was used; `roll_rad` is **deleted** from
  both verbs (it had been a hidden field for one step) and from
  `NOT_MODEL_BINDABLE`/`arguments.py`, and `reach.plan_chain`/`choose_side`
  lose their `roll_rad=` (they gain `contact=`). A model that sends
  `roll_rad` gets `bad_argument` (unknown argument). The example's chooser
  sweep (`astra_loop.plan_the_hand`) and live jaw-turn fallback are deleted.
  The verifiers of a multi-roll `Approach`/`Grasp` check the candidate the
  measured wrist ended at.
* **`fits(obj, frames, spec, r_tcp, *, open_gap_m, support)`** /
  `fit_problems(...)`: flat from `reference.lead_m`, width along the jaw axis
  against `graspable_width_m(reference, open_gap_m)` — the world's measured
  `GripperView.open_gap_m` when it carries one, the description's nominal
  otherwise. Deleted from `orientation`: `GRASPABLE_WIDTH_M`,
  `graspable_width_m`, `JAW_OPEN_M`, `TIP_BELOW_TOOL_M` (use
  `reference.lead_m`), `fits_jaws`, `lowest_top_down_tool_z`, `grasp_point`,
  `achieved_clearance`, `grasps_above_its_top`, `standoff_pose` (the
  `grasp_geometry` functions replace them). `primitives` exports
  `GraspReference`, `GraspSpec`, `PAD`, `TIP`, `grasp_pose`,
  `standoff_point`, `roll_candidates`, `fits`, `graspable_width_m`,
  `support_of` instead of `GRASPABLE_WIDTH_M`/`JAW_OPEN_M`.
* **A tilted object is grasped along its own face (req 4).**
  `_upright_geometry` is `support_geometry_known(world, name)`: a tilted
  object is refused (`object_tilted`) only when no measured surface is under
  it. Above `UPRIGHT_TOL_RAD` (10 deg, now the threshold for the NOTE, not a
  gate) a `down` grasp descends along the object's own top-face normal,
  `Direction(axis, frame="object:<name>")`, with the jaws squared to its
  projected footprint, and the notes say so. A tilted DESTINATION (Carry/
  Place) is still refused.
* **Hand shapes for non-grasping verbs** read the hand description's
  `HAND_CLOSEDNESS` (`open`/`pinched`/`closed`, shared with the contact verbs
  of step 4, which own the `hand` argument) through
  `grasp_geometry.hand_closedness`.
* **Golden plans regenerated on purpose** (`tests/data/golden_plans/plans.json`,
  was `pre_direction_e1dce97.json`): the per-`jaw_turn` cases collapse into
  one each (163 cases, 91 plans), the committed d1-2 scene's measured hand
  opening is applied, and new cases pin a side approach that PLANS in both
  directions, a tip grasp, a tilted object, an object declared into its table
  and a tall cup. A `forward` chain in the demo/tabletop scenes now stops at
  `Lift` (`guard_reject`): the 54 mm deeper standoff reaches the same grasp
  pose on a different elbow branch.

### `manipulation_kit.perception` — perception is a kit interface

**BREAKING for anyone importing the example modules.** `examples/agent/camera.py`
is gone (moved, history kept, to `manipulation_kit.perception.camera`), and the
library half of `examples/agent/perceive.py` — plane fit, height policy,
measurement, scene file, the colour-mask fallback — is now
`manipulation_kit.perception.{plane,measure}`. `perceive.py` is a thin CLI
(decode, flags, pick a detector) and the Astra box detector is
`examples/agent/detector.py` (`AstraDetector`, a `Perceiver`). The extra is
renamed `perceive` → **`perception`** (`pip install -e '.[perception]'`); it
still only carries `pillow`, and nothing under `manipulation_kit` decodes an
image.

* **`Perceiver` protocol** (`perception/protocol.py`): `cameras()`,
  `locate(camera, u, v, plane=)`, `declare(objects, support=)`,
  `attached(side)`. `ScenePerceiver` is the reference implementation; a
  customer with another camera stack implements four methods instead of forking
  the example.
* **`Located.kind`** — `"contact"` (a surface point: for the bottom of a
  silhouette, the object's NEAR edge) or `"centre"`. `contact_to_centre()`
  performs the footprint-centre + half-height conversion that a prompt
  sentence used to ask the model to do (Astra review 7); the loop's `locate`
  tool takes an optional `size`/`yaw_rad` and returns the centre.
* **Uncertainty in independent parts** (Astra review 8). `locate` reports
  `mount_uncertainty_m` (lens position and aim as INDEPENDENT unknowns — every
  combination, not six samples that moved both along one axis; values are
  larger than 0.15.0's by design) and `height_uncertainty_m` (the plane moved
  by `plane_uncertainty_m`), and `uncertainty_m` is their `hypot`.
* **`SurfaceView.plane_source` / `height_uncertainty_m`** — a perceived
  table's height provenance now reaches the world (scene files carry them as
  top-level fields; `live.objects_from` passes them through) and a pixel
  located on it inherits them.
* **One interior fraction:** `world.views.INTERIOR_FRACTION = 0.9`.
  Perception used 0.85 while the view (and the operator's preflight text) said
  90 %; perceived interiors are now 0.9 × the measured outside.
* **One neck sign flip:** `description.head_camera.neck_joints_from_state()`
  (accepts a `NeckState`-like object or a `/v1/neck/state` body). The two other
  copies (`camera.py`, `astra_loop.py`) are gone.
* **`HeadCameraConfig`** (fx, fy, cx, cy, width, height, neck, lift) and
  `HeadCamera.from_config()`, which FAILS CLOSED on a missing or moving neck;
  `read_head_state(executor)` uses the executor's `neck_state()` /
  `lift_state()` (step 1). `astra_loop.py`'s `robot_camera_opts` — a raw
  `urllib` read of `/v1/neck/state` + `/v1/slider/state`, a hand-negated pitch
  and a whitespace re-parse of a flag string (Astra review 9) — is deleted.
  A live neck state plus a `--neck-pitch` flag is refused as two answers.
* **Declared objects are lifted onto their support by a kit rule**
  (`lift_onto_support`, via `SurfaceView.top_z(frames)`), replacing
  `astra_loop.py`'s open-coded `p[2] + size[2]/2`, which ignored the surface's
  frame and rotation (L5/L15). Only surfaces the object is over are
  candidates; an object over none is left as declared and the note says so.
* **Refusals:** non-finite pixels/planes are `ValueError`; a plane ABOVE the
  lens (intersection behind the camera) is `NotOnThePlane`, as a ray above
  the horizon already was (Astra review 9).
* **`WristCamera`** (`perception/wrist.py`) — the plate mount the kit already
  had (`CAMERA_MOUNT_XYZ_M`, `CAMERA_TILT_RAD`, `CAMERA_ARM_YAW_RAD`) composed
  with the arm's FK; `project_object()` says whether an object is in the wrist
  frame and where (what step 7's look-before-the-stroke needs). Intrinsics are
  not defaulted.
* The mask detector's refusals and `requested_objects()` raise `ValueError`
  (a library does not `SystemExit`); the CLI converts them.
* `examples/preflight.py` prints the pad-centre and driven-opening numbers
  and the interior fraction from the kit instead of prose.

Tests: `tests/agent/test_perceive.py` moved to `tests/perception/test_perceive.py`
(kit side, plus PR #21's Validated table pinned row by row); the CLI, Astra and
scene-reader tests are `tests/agent/test_perceive_cli.py`; new
`tests/perception/test_perception_interface.py`.

### Contact verbs: `probe` and `press` on `move_until`

Contact becomes a concept (design C.3, L8/B8). Additive — nothing is removed —
and **not yet validated on hardware**: `docs/probe-hardware-trial.md` is the
d1-2 gate (zero controller errors; table z within +-3 mm of the tape over 10
probes) and it has not been run.

- **Position mode only** (Shu, decision 3). A contact leg is a position-
  commanded straight line WATCHED for a joint-torque rise; no arm mode is
  set, no torque or force is ever commanded.
- **`primitives.types.ContactCriterion`** (measured thresholds:
  `joint_torque_nm` rise over the pre-motion baseline, `tool_force_n`,
  `stall_velocity_rad_s`, `settle_s`) and **`ContactStep`** (`side,
  direction, max_travel_m, criterion, waypoint`, plus the leg's own knots
  `path`/`s`, `speed_m_s`, `hold_s`, `retract`) — `ContactStep` joins the
  closed `Step` union. The leg's knots live INSIDE the step, so a runner that
  does not understand it can only refuse it, never play the leg blind.
- **`executor.ContactReport`** (`made, p_tool, travel_m, normal_hint,
  torque_nm, stopped_by` ∈ `contact | max_travel | fault | guard |
  unmeasured`, plus `q_stop`, `joint`, `leg_t_s`), measured from the state —
  never from the command. `RunReport.contacts` carries every leg's report.
- **`move_until(path, *, side, criterion, hz, kin, direction)`** — an optional
  executor capability. `ContactWatch` is the one criterion implementation
  (freeze the command while a rise is confirmed; contact after `settle_s`
  stalled, or at once at 2x the threshold). `stream_move_until` is the
  default for a streaming transport that reports torque (opt in with
  `StreamingContact`). `FirmwareExecutor.move_until` uploads the leg as a
  guarded trajectory and polls `ArmState` + `TrajectoryStatus`, cancelling
  on the criterion — all through **generated** operations
  (`FirmwareClient.trajectory_start/_status/_cancel`, new). `KinematicExecutor`
  travels the whole leg and reports `made=False, stopped_by="max_travel"`.
  An executor without `move_until` fails a contact step with
  `transport_error`.
- **`primitives.contact`: `Probe(side, direction=down, max_travel_m=0.15,
  contact_nm=4.0, hand="closed", declare_as="")` and `Press(side, target,
  direction=forward, depth_m=0.005, force_nm=6.0, hold_s=0.5,
  hand="closed")`**, both on `Direction`, neither naming one (`touch_down`
  does not exist). Verifiers `ContactMade`, `SurfaceMeasured` (and `ToolAt`
  the standoff, for a press).
- **Contacts feed the scene.** `WorldView.contacts` (`world.ContactView`) is
  the evidence; `primitives.record_contacts(world, run_report, verb)` folds a
  run in and, with `declare_as`, publishes a `SurfaceView` whose face is the
  measured plane (`plane_source="contact"`): one probe = a plane through the
  contact with the probe's normal, three = a least-squares plane with a real
  normal (`fit_plane`). `SurfaceView.from_plane`, `.top_normal`,
  `.plane_offset` are new.
- **Model surface.** `arguments.py` gains `max_travel_m` (0.01-0.30 m),
  `contact_nm` (**1.5-6 Nm**), `depth_m` (0-0.03 m), `force_nm` (**2-8 Nm**),
  `hold_s` (0-5 s), `declare_as`, `hand` (`open|pinched|closed`, mapped to a
  closedness by `hands/d1/parallel_gripper/description.HAND_CLOSEDNESS`).
  `Primitive.arg_roles()` lets a verb narrow what a name argument may name
  (`press.target` is anything in the world; `pour.target` stays a vessel).

### The scene is an obstacle set

- **New `manipulation_kit.primitives.clearance`: `Obstacle`, `obstacles_of`,
  `SceneGate`, `ClearanceReport`, `ClearancePolicy`.** Every `SurfaceView`,
  `ContainerView` and `ObjectView` in the world — minus the verb's own
  target/destination (`object`, `to`, `source`, `target`) and whatever a
  gripper holds — is an oriented box the arm's LINKS (`Base`..`Link7`, the
  body guard's own capsules via `guard.urdf_model.prim_to_world`) must keep
  clear of. `guard/` is unchanged and still stdlib-only; the scene half lives
  in `primitives/`, where numpy and `WorldView` already are.
- **Every accepted joint step is swept against the scene** (`planning.Kin(...,
  scene=)`; `_straight` and `joint_ramp` call `SceneGate.swept_ok` per
  interval). A hit is the existing **`guard_reject`** refusal with the
  obstacle and link NAMED in `detail`, `attempted=("obstacle:<name>",
  "link:<link>")`, `stage="scene"`, and **`residual_m` = how far the link is
  inside the clearance that obstacle requires** (Shu decision 4: refuse, with
  the number). A dead end whose straight line died on the body guard while
  the routes around it died on the scene says so too. An arm that STARTS
  inside an envelope may move out of it, never deeper.
- **Margins are per obstacle** (`ClearancePolicy`): a probed surface
  (`plane_source` `probed`/`contact`) 5 mm, a declared surface 10 mm, a
  declared object/container 10 mm or its own new **`ObjectView.uncertainty_m`**
  when stated; a surface's `height_uncertainty_m` grows its box vertically
  only. **Sampling is stated**: links every 4 mm, postures every 6 mm of link
  travel, and the 5 mm this can miss by (`SceneGate.sampling_allowance_m`) is
  ADDED to every requirement — a declared obstacle needs 15 mm.
- **Up-and-over is the default shape of a free transit** (`allow_via=True`
  legs): when the hand would pass low over something (its top + required
  clearance + how far the hand hangs below the tool point, per orientation),
  the leg rises — straight up, or through the measured `VIA_OFFSETS_M`
  clearance points, now raised to at least that height — traverses and
  descends, and the plan note says what it cleared. `VIA_OFFSETS_M` is the
  fallback, no longer the recovery list. `allow_via=False` legs (grasp
  descent, lift, nudge, retreat, carry/place legs) are never re-shaped.
  When no route over plans, the old straight/via transit is still tried with
  the links checked, and the note says the hand's clearance was not built.
- **BREAKING: `MKIT_SUPPORT_CLEARANCE_M` is deleted.** `SUPPORT_CLEARANCE_M` is
  the rigid-arm 3 mm constant again; the F16 arm droop the variable
  compensated is **`ClearancePolicy.droop_margin_m`** (default **0.0** =
  rigid model; d1-2 measured **0.012**, the old 15 mm floor), attached to the
  kinematics with `clearance.set_policy(kin, ClearancePolicy(...))` and
  read by every plan (`policy_of(kin)`). It raises the top-down fingertip
  floor (`grasp_geometry.grasp_pose(..., droop_margin_m=)`, read by
  `Grasp.plan`) and every obstacle clearance. The operator policy sets it on
  a live robot (`OperatorPolicy.droop_margin_m`, see "Phase B integration").
- What changes for existing plans (golden file, capture stack; re-derived
  on the phase-B `plans.json`): 19 of 163 cases, all side-on approaches
  (`forward`, and `left`/`right` at the shelf), become refusals naming the
  table or shelf their forearm/wrist passes 12-15 mm from, or whose standoff
  posture the body guard refuses with every route around it refused by the
  table (listed by number in
  `tests/primitives/test_golden_plans.py::SCENE_REFUSED`); no top-down case
  changes. The d1-2 top-down pick-and-place still plans end to end, with and
  without the 12 mm droop. Planning cost on the d1-2 scene: +60-75 ms per
  plan (213 -> 274 ms for a top-down grasp, 264 -> 338 ms for the 5-verb
  chain).
- **Contact legs** (step 4's `Probe`/`Press`, driven up to `max_travel_m`
  past the surface they measure): build their gate with
  `SceneGate.for_contact(world, kin, p_start, direction, travel_m)`, which
  leaves out the first obstacle the leg's ray meets (`contact_target`) and
  keeps the rest. Obstacles keep their full pose rotation, and a surface's
  `height_uncertainty_m` grows it along its OWN normal (a probed wall's is
  horizontal).
- Not yet: the hand (TCP flange, palm, fingers) is not in the scene gate —
  same collision policy as the body guard; its transit clearance is by
  construction only.

### `manipulation_kit.agent` — the loop is a kit module

- **NEW `manipulation_kit.agent`**: `OperatorPolicy` (`policy.py`), the
  provider-independent loop `run()` (`loop.py`), `LiveRobot` /
  `KinematicMirror` / `SceneSource` and the `--executor` registry
  (`robot.py`), the `declare_scene` / `locate` observation tools (`tools.py`)
  and `DecisionRecord` / `DecisionTrace` (`trace.py`). `examples/agent/live.py`,
  `mirror.py` and `trace.py` are **gone** (moved, not copied); import from
  `manipulation_kit.agent`.
- **BREAKING: all eight `ASTRA_*` environment variables are deleted.**
  `ASTRA_GRIP_CAP` → `OperatorPolicy.max_grip`, `ASTRA_APPROACH_ALLOW` →
  `allowed_directions`, `ASTRA_VEL_RATIO` → `vel_ratio`,
  `ASTRA_ARRIVE_TIMEOUT_S` → `arrive_timeout_s`; `ASTRA_SNAPSHOT_CMD` →
  `astra_loop.py --snapshot-cmd "CMD {turn} {out_dir}"` (`examples/agent/
  snapshot.py`: exit code checked, frames must be fresh, each camera
  labelled, failure stops the loop); `ASTRA_MAX_OUTPUT_TOKENS` /
  `ASTRA_REASONING` / `ASTRA_DEBUG` → `--max-output-tokens` / `--reasoning` /
  `--debug`. The real model is `--model NAME` (the OpenAI SDK reads
  `OPENAI_API_KEY` itself); without it the scripted stub runs.
- **`OperatorPolicy`**: `max_grip`, `allowed_directions`, `vel_ratio`,
  `arrive_timeout_s`, `stroke_timeout_s` (`None` = the executor's own bound —
  on firmware the daemon document's), `look_before_stroke`,
  `max_nudges_per_target`, `max_turns`, and for step 4's contact verbs
  `max_contact_nm` (4.0) / `max_force_nm` (6.0). `clamp(call)` is the ONE home
  of the caps (lowered, reported by `notes()`) and the refusals (direction,
  look, nudge budget) — kit `Unmet`s, sent back as a kit `PlanError`, not the
  hand-rolled `approach_disabled` dict. `to_json`/`from_json`, and CLI flags
  generated from the fields (`--max-grip`, `--no-look-before-stroke`, ...,
  `--policy FILE`). The timeouts are resolved once and the same numbers reach
  the executor's constructor and every `run()` (L13, Astra review 12).
- **Look before the stroke** (`look_before_stroke=True`, the default): a
  `grasp` the hand has not looked at from its current posture is answered
  with a wrist look (`WristCamera.project_object`) instead of run; `locate` on
  a wrist camera re-measures the object there and nudges the hand by the
  difference. A robot without wrist intrinsics (`scene["robot"]
  ["wrist_camera"]`) **refuses to start** (`look_unavailable`) unless the
  operator turns the rule off.
- **Held objects ride the tool** (`world/attach.py`: `grasp_transform`,
  `with_attached`, `attached`, `released`; review 11). `ObjectView.provenance`
  = `observed | declared | attached | predicted` (scene-file objects are
  `declared`). `SceneSource` associates the object between the pads when the
  gripper starts holding, publishes its attached pose, and leaves it
  `predicted` where the hand let go. `robot.expect()` is gone. `ObjectRose`
  therefore measures a real rise; `ObjectOver` / `ObjectClears` / `ObjectIn`
  say "inferred, not sighted" and turn a negative verdict on an inferred
  pose into UNKNOWN (`ObjectIn` is never TRUE on one). `reach.plan_chain`
  uses the same rule (`_grasped` returns `(world, grasp)`; `_moved(world,
  grasp)`), which moves the five chain cases' `Place` waypoints by up to
  0.36 mm — the golden file is regenerated for exactly those five.
- **`--executor NAME`** is resolved through a registry: `firmware`,
  `kinematic` built in; `register_executor(name, factory)`; the
  `manipulation_kit.executors` entry-point group (how d1-isaaclab provides
  `isaac` without the kit importing it); `--executor-class module:factory`.
  An unknown name fails with the available names and what to install.
  `LiveRobot` is a context manager (the example's `ExitStack` is gone).
- The example's operator gates, its two candidate sweeps (the arm choice is
  `reach.choose_side`, whose chain sweeps `roll_candidates()`), the jaw-turn
  fallback, the dead `isinstance` and the shadowed `turn` are deleted; the
  `controller_fault` stop is the loop's. The prompt quotes no kit number —
  `agent.robot_facts()` generates the jaw capacity, nudge grid and
  directions. `DecisionRecord` gains `effective` (the call as it ran, beside
  the verbatim `choice`), `look`, `error` and `contacts`; every turn is
  recorded however it ends; the message history is written atomically and
  earlier photos are not resent.

### The pieces together

- **Contact legs are scene-gated.** `Probe`/`Press` plan through
  `SceneGate.for_contact(world, kin, p_standoff, direction, travel_m,
  exclude=(target,))`: every declared thing gates the standoff transit and
  the leg except the surface the leg is aimed at (the first obstacle its ray
  meets) and a press's named target. The plan says which surface was left
  out. Known limitation: the ray is widened by the hand and the clearance, so
  a press on a small object standing on a table leaves the table out too.
- **`OperatorPolicy.droop_margin_m`** (default 0.0; **d1-2 measured 0.012**)
  — `--droop-margin-m`, and in a policy file. `agent.run()` applies it to the
  robot's kinematics as `ClearancePolicy(droop_margin_m=...)`
  (`OperatorPolicy.apply_to(kin)`), so a live d1-2 grasp keeps the pad tips
  ~15 mm over the wagon, as `MKIT_SUPPORT_CLEARANCE_M=0.015` did. It reaches
  the floor through `grasp_geometry.grasp_pose(..., droop_margin_m=)`.
- **`allowed_directions` covers every verb that ARRIVES along a direction**:
  `agent.policy.DIRECTED_VERBS` (was `APPROACH_VERBS = ("approach",
  "grasp")`) is derived from the verb registry via the new class attribute
  `Primitive.DIRECTION_ARRIVES` — approach, grasp, probe, press. A Lift's
  "up" and a Retreat are not arrivals and stay unrestricted.
- **Measured contacts persist across turns.** `SceneSource.remember_contacts`
  / `forget_contacts` (and the same on `LiveRobot`): the loop hands every
  folded contact to the world source, later observations carry them in
  `WorldView.contacts`, and a `declare_scene` clears them. A surface they
  fitted stays a `SurfaceView` with `plane_source="contact"`.
- **A wrist block marked `"measured": false` is a placeholder**: the
  kinematic mirror may dry-run the look policy through it, the firmware robot
  ignores it (`wrist_camera_from_scene(..., measured_only=True)`) and stops
  with `look_unavailable`. (The d1-2 scene's own placeholder is gone: its
  lenses are measured now, see "Robot profile".)
- The chain chooser in `agent/loop.py` has no roll logic: every
  Approach/Grasp in the chain sweeps `grasp_geometry`'s candidate rolls.
  When every roll fails, the refusal also lists the obstacles that refused
  the OTHER rolls in `attempted` (and keeps the gate's own
  `obstacle:<name>`).
- `manipulation_kit.primitives` re-exports `SceneGate`, `ClearancePolicy`,
  `ClearanceReport`, `Obstacle`, `obstacles_of`, `policy_of`, `set_policy`
  and the `clearance`, `contact`, `grasp_geometry` modules.
- **Golden (`tests/data/golden_plans/plans.json`) regenerated once**, gate
  off, on the merged tree: 5 of 163 cases change, all top-down chains whose
  Place link moved (<= 1.1 mm waypoints) because a held object now turns with
  the wrist (step 7's `world.attach`). `SCENE_REFUSED` re-derived (above).
- `Primitive.DIRECTION_ARRIVES` / `agent.policy.DIRECTED_VERBS` — see
  above; `handover` is directed too (its receiver arrives).

### Robot profile, and the wrist fisheye

- **NEW `manipulation_kit.description.robot_profile`**: `RobotProfile(name,
  hand: HandMeasurement(open_gap_m), head_mount_delta: HeadMountDelta | None,
  wrist_cameras: {side: WristIntrinsics(fx, fy, cx, cy, width, height,
  model, k, valid_radius_px)})` — one typed file per robot, `RobotProfile.
  load(path)`, `.named("d1-2")` (committed under `description/profiles/`,
  shipped in the wheel), `.resolve(name_or_path)`, and `.from_files(...)`,
  which reads the d1-inference artefacts VERBATIM: `head_aruco`'s
  `cameras_<robot>.head*.json` and `d1-calibrate-wrist`'s
  `wrist_<side>_intrinsics.json` (PR #77).
- **`description/profiles/d1-2.json`** — measured: hand 60.5 mm (daemon
  `open_rad` 1.35); **both wrist fisheyes** (2026-09-22, 48 ChArUco views
  each, gate PASS: left fx 238.54 fy 239.67 cx 314.00 cy 224.76, RMS 0.218 px,
  valid radius 304 px; right fx 237.64 fy 238.95 cx 328.37 cy 225.58, RMS
  0.392 px, valid radius 326 px); the head mount (interior fit 2026-09-10,
  [-23.9, -43.0, +24.6] mm / [+5.60, +2.27, +1.34] deg, RMS 3.89 px).
- **The head mount is applied ABSOLUTELY.** `HeadMountDelta` keeps the
  nominal it was fitted against (the 17.25 deg head tilt of 2026-09-10) and
  rebuilds `head_link -> optical` from it; the kit's own nominal has moved
  since (15 deg design tilt), and pasting the delta onto it would put the
  camera 2.25 deg off. `HeadCamera.from_robot/from_config(...,
  mount_delta=)` (new `description.head_camera.measured_head_camera_pose`)
  and **only then** `calibrated: true`. On the d1-2 cube's pixel at neck
  0.62 rad the measured mount moves the wagon-top point ~8 cm outward
  relative to the nominal.
- **`WristCamera` consumes the fisheye model** (`model="fisheye"`, `k`):
  project and unproject through OpenCV's `cv2.fisheye` equidistant model
  (`theta_d = theta (1 + k1 theta^2 + k2 theta^4 + k3 theta^6 + k4 theta^8)`)
  in numpy, fixed-point inverse — no OpenCV dependency. A pinhole model of
  that lens is ~10 % off at 30 deg off-axis, where the look-before-stroke
  looks. Pixels outside `valid_radius_px` are "outside the calibrated
  radius", not trusted.
- **Scenes name a profile**: `"robot": {"profile": "d1-2"}` (a committed name
  or a path relative to the scene); the scene's own `robot` keys override the
  profile's. `agent.load_scene(path, profile=)`, `LiveRobot.from_flag(...,
  profile=)` / `LiveRobot.firmware(..., profile=)`, `astra_loop.py
  --robot-profile NAME|FILE`, `perceive.py --robot-profile` (head mount).
  Wrist intrinsics are per side now (`{side: kwargs}`; a flat block still
  means "both hands"), and a MEASURED profile's lenses reach the firmware
  robot, so the look policy works live on d1-2.
- **Before the first live look**: the wrist EXTRINSIC (lens on the plate) is
  still the kit's nominal plate geometry; `--no-look-before-stroke` is the
  documented first-live-run setting (`docs/agent.md`).

### `handover` — the one verb that plans both arms

- **`Handover(object, from_side=auto, to_side=auto, direction="left",
  clearance_m=0.10)`** (design C.10): the giving hand takes the object to a
  meeting point, the receiving hand approaches and grasps it travelling
  `direction`, the giving hand opens and backs out `clearance_m`. Built on
  the chain planner over two sides (`reach.handover_chain`,
  `reach.plan_handover`); the meeting point is the first rung of
  `reach.HANDOVER_MEETING_POINTS_M` whose whole two-arm chain plans. None →
  the new plan reason **`unreachable_handover`** (every rung in `attempted`).
  A direction that takes the receiver AWAY from the giver is `bad_argument`.
  Verifier `Holding(receiver) ∧ NotHolding(giver) ∧ ToolClearOf(giver)` (new
  `verifiers.ToolClearOf`).
- **The give waits for a measured take**: `GripStep.expect_hold` — the
  receiver's close must end with `StrokeReport.holding is True` or the run
  stops with the new run-refusal reason **`hold_not_confirmed`** before the
  giver opens (generic runner and firmware executor).
- `Release` counts the other hand holding the same named object as support
  (so the model can also compose the handover verb by verb);
  `SceneSource` re-attaches an object that passed hand to hand within one
  observation at the receiver's pad centre.
- `Primitive.applicable(world)`: `tool_schemas(world)` describes `handover`
  only while exactly one hand holds a named object and the other is free, and
  `candidates_for` offers it then. Under `look_before_stroke` the operator
  policy refuses a `handover` with **`look_not_possible`** (its receiving
  close has no posture a look can precede) and says how to compose it.
  New arguments `from_side`, `to_side`.

### Examples and docs

- `examples/agent/astra_loop.py` is **199 lines**: the prompt, the OpenAI
  client, `loop()` and `main()` over `manipulation_kit.agent`. The scripted
  stand-in moved to `examples/agent/scripted.py`; `--perceive`'s helpers to
  `examples/agent/run_scene.py` (`perceived_scene`, `robot_head_state`,
  `scene_for_run`). The
  prompt quotes no kit number: jaw capacity, nudge grid, directions
  (`direction_doc()`), `contact` (with `"tip"` marked EXPERIMENTAL — not yet
  measured on hardware) and `tool_revision()` reach the model as the
  generated `agent.robot_facts()`.
- The three C.12 example tests pass (`test_the_example_is_short` is no
  longer an expected failure).
- `docs/probe-hardware-trial.md` gains the **tip grasp trial** (6 mm card,
  `Grasp(contact="tip", grip="soft")` x 10; until it passes the default stays
  `contact="pad"`). `README.md`, `docs/agent.md`, `examples/README.md` and
  `examples/agent/DATAFLOW.md` describe the 0.16 flow.

### Known, not fixed here

- The demo/tabletop `forward` chains stop at `Lift` with `guard_reject`
  (golden cases 17, 48, gate off): the deeper standoff reaches the grasp on
  another elbow branch and the straight lift from there is refused; the
  up-and-over applies to `allow_via` transits only, not to a Lift.
- A `press` on a small object standing on a table leaves the TABLE out of
  the scene check too (the contact ray, widened by the hand and the
  clearance, meets the table top first).
- `handover` plans on the kinematic mirror only; nothing about a hand-to-hand
  transfer is measured on hardware, and the hands' own geometry (fingers,
  palm) is not in the scene gate.

## 0.15.0 — 2026-09-22

**A head frame is now an observation, and NO PER-SCENE CALIBRATION GOES INTO
IT.** The first cut of this work took the table's width and the x of its far
edge as inputs. Shu's answer on reading it was
「中途半端にこっちでシーンごとの calib をするのは消したい」, and he is right:
those are measurements of the furniture, they are stale the moment the wagon is
nudged, and a pipeline that needs them has moved the tape measure rather than
put it away.

The rule now, and it is the whole change:

* **ROBOT-specific calibration is allowed** — the head camera's intrinsics, the
  neck joints, the lift, and the pose of the lens in `base` that the kit reads
  out of its own URDF. Those belong to the machine.
* **SCENE-specific numbers are not inputs.** No table width, no far-edge x, no
  table height, no marker, no tape on anything. `--table-width` survives as an
  OPTIONAL refinement and is the only one left.

### What one camera can and cannot do

A pixel is a RAY in `base`, exactly, from robot facts alone; a ray plus a
horizontal plane at a known height is a point. **The height is the one thing a
single camera cannot measure** — twice as far and twice as big is the same
picture. So the scale comes from outside the geometry, from exactly three
places, and every number that depends on it says which:

`declared` — the MODEL says so, from the photograph, with the robot's own
hands at known base-frame positions in it for scale. `known-length` —
`--table-width`, optional, solved in closed form. `provisional` — nobody has
said; the plane goes at the z of the arms' HOME tool points (a robot fact, not
a measurement of anything in front of the camera) and every object measured
against it drops to `confidence` 0.2.

Everything else in the fit is scale-free and always available: masking the
top, fitting the far and side edges, and projecting those lines onto the plane
gives the table's rectangle — where it is, how big, which way it is turned —
correct in proportion for whatever the height turns out to be. Which is why
one declared number from the model fixes the whole scene at once.

### Added

* **`manipulation_kit.description.head_camera`** — the one part inside the
  wheel. The head camera has had a FRAME here since the whole-body URDF gained
  cameras (`head_camera_link` on the neck-tilt link, plus its ROS optical
  child); what it did not have is a way to ASK for it.
  `head_camera_pose(neck_pitch, neck_yaw)` reads the committed URDF through
  this package's own parser, so it cannot drift from the asset. Also
  `pose_from_neck_state()` — the daemon reports LOGICAL pitch, the negative of
  the URDF joint, and that flip now happens in one place — and
  `floor_to_base_m(lift)`.

  NOMINAL, not calibrated: vendor geometry plus the head part's design tilt,
  the lens at the housing's front face, the d1-3 ArUco fit ~2.5 deg off it.
  Everything built on it is stamped `calibrated: false`.

  **The lift does not move the camera in `base`** — it sits BELOW `dual_base`
  and raises the base and the camera together. A consumer "correcting" for it
  is wrong by up to 300 mm.

* **`examples/agent/camera.py`** — the head camera as a model: pixel to ray to
  base-frame point, and the inverse. Its `locate()` PROPAGATES the mount's two
  documented unknowns (±20 mm of lens position, ±3 deg of aim) into a
  per-pixel uncertainty rather than quoting a flat number, so a pixel near the
  far edge reports the 70 mm it is worth and a grazing one says `GRAZING`.

* **`examples/agent/perceive.py`** — the plane fit, the height policy above,
  two optional detectors, a scene writer and a debug PNG. The default
  (`--detector model`) detects NOTHING: it writes the camera, the table and no
  things, because the loop's own model is the detector.
  `level_correction_deg` falls out for free and is the diagnostic worth
  reading — scale-free, and large when the neck angle you passed is wrong or
  its sign is.

* **`astra_loop.py`: two tools that are about the OBSERVATION, not about
  moving.**

  `declare_scene(objects=[{name, kind, p, size, yaw_rad, confidence,
  interior}])` — the model says where things are, in base metres, from the
  photographs. It goes through the SAME reader a hand-written scene file does
  (`live.objects_from`), so a model cannot declare something a person could
  not have written. `interior` is there because `Place` refuses to drop into
  an interior nobody stated, and a model that can see into a cup has to be
  able to say what it sees.

  `locate(u, v, camera='head')` — the deterministic half. A pixel the model
  picked, turned into a base-frame point on the current table plane, with its
  uncertainty. The model should not be doing projective geometry in its head
  when a function can.

  Neither moves anything, so neither goes through the motion gate; both are
  answered inside the loop and reported back correlated with the call. The
  prompt now tells the model it IS the detector, hands it the camera block,
  and points at both arms' tool points — already in the world text, in the
  same base metres, and usually visible in the head photo — as its scale
  reference.

* **`astra_loop.py --perceive <frame|snapshot>`** — measure the scene before
  turn 0 instead of reading one. `snapshot` reuses the existing
  `$ASTRA_SNAPSHOT_CMD` hook; the result is written beside the trace as
  `scene_perceived.json`, and the CAMERA travels in that file rather than as a
  second set of loop flags (a neck that has moved since the frame was taken is
  a different camera). Mutually exclusive with `--scene`. **Once, before turn
  zero** — re-perceiving every turn is deliberately out of scope: the object
  moves while the loop holds it, so a fresh scene would have to be reconciled
  with the gripper's `held_object` rather than replacing the old one.

### Changed

* **The loop no longer refuses at turn zero when the scene is empty.** The arm
  choice used to be made before anything moved and `unreachable_task` returned
  if the object was not in the scene — which is now every run that has not
  measured the furniture first. It is made the first turn both names exist,
  and a model that stops before declaring anything gets `model_stopped` with
  the reason, not a measured success.

* **`ObjectView.to_text` prints a `confidence` below 1.** The field has been on
  `WorldView` since the beginning and never reached the text, so a producer
  that said "0.3, I am guessing" had the one honest thing about its number
  erased on the way to the only consumer that could act on it. Nothing in the
  kit GATES on confidence — checked — so it is information, not a permission,
  and a 0.3 declaration still plans an Approach and a Grasp. A confidence of 1
  prints nothing, so no existing text changes.

* **The scripted stub declares, when there is nothing to act on.**
  `--perceive --dry-run` hands the loop a real table and no things, and the
  stand-in cannot see; rather than stop at turn zero for want of a detector it
  declares a plausible pair (at the demo scene's own reachable x/y, at the
  PERCEIVED table's height) so the loop itself stays runnable with no key.
  Fiction, labelled `scripted-declare` in the trace.

* **`examples/agent/live.py`**: `objects_from` now carries `confidence`
  through from the file, and reads `interior_measured` — `ContainerView` only
  clears that flag for the interior it invents itself, so a file that GAVE an
  interior was believed unconditionally, measured or not.
  `LiveRobot.declare()` / `SceneMirrorRobot.declare()` are what
  `declare_scene` writes into, and `SceneMirrorRobot` no longer refuses to be
  built when the tracked object is not in the scene yet.

### Validated

Five real frames of the d1-2 JP wagon, in `tests/data/perceive/`. Far corners
within 1.3 px of what `cv2.fitLine(DIST_HUBER)` produced on the night; the
near edge 8 mm off a 400 mm tape on both frames that show it, and refused (not
invented from the image border) on the two that clip it; `--table-width 0.60`
solves the height to 0.150-0.153 m on all five against a tape's 0.166 — a
consistent ~14 mm low, which is the lens position inside a 90 mm housing and
is a BIAS, not noise. With the height declared, the two objects on the run2
frame land within 20 mm of the positions Shu used for that run.

### What this does NOT do

* **It does not measure the plane's height, ever**, and nothing in it pretends
  to. See above.
* **The extent along the unseen horizontal axis.** One view gives one
  silhouette, so the width is written on BOTH horizontal axes and yaw is 0 (a
  square footprint is rotation-invariant, which makes that zero harmless
  rather than invented). On the run2 frame the charger measures 50 mm across
  its footprint and the driven jaws take 44 mm, so the chain Shu's hand-made
  file planned does not plan off the measurement — his file declared the
  charger 20 mm across y, which is a tape measurement, not a picture. Both
  halves are pinned in `tests/agent/test_perceive.py`, and `--size` /
  `declare_scene` are how you put it back.
* **A container's interior**, as above.

Neither of the last two is a bug to be fixed by better fitting. They want a
second viewpoint, a depth camera, or a tape.

## 0.14.1 — 2026-09-21

**The jaw meshes now open 64 mm, like the joints always said.** The vendor CAD
was cut for a 70 mm opening: `tcp_r_Link.STL` spans local z 35 … 75 mm and
`tcp_l_Link.STL` −75 … −35 mm, so each jaw's inner pad FACE sits **35 mm**
from its link origin. 0.14.0 moved the joint origins and limits onto the
callipers (32 mm per jaw, a 64 mm gap) but left the meshes where the CAD put
them, because a mesh is only replaced by a CAD drop. That was fine for anyone
planning against a TCP or a collision *primitive* — and wrong for everyone
who renders or **collides the mesh**. Isaac builds its convex hulls from it,
so `d1-isaaclab`'s D1 grasps with jaws that are 70 mm open and 6 mm "closed"
while this package's gate, its `DRIVEN_OPEN_GAP_M` and the real robot all say
64 / 0 / **51.96**. Measured consequence on `blocks-eval` (seed 7): the
tool-space arrival gate added in 0.14.0 is calibrated to the 52 mm opening
and refuses lateral misses those 70 mm sim jaws would still catch, and
descents jam 17–25 mm short along the approach axis on the over-long collider.

### Changed

* **`hands/d1/parallel_gripper/descriptions/gripper.urdf` and
  `gripper_with_camera.urdf`**: each jaw's `<visual>` *and* `<collision>`
  mesh `<origin>` moves from `0 0 0` by `JAW_STROKE_M − CAD_JAW_STROKE_M` =
  **−3 mm** along the jaw travel axis, toward the centre. That axis is the
  link's own +z (both joints carry `axis="0 0 -1"` in that frame), and the
  sign follows which side the face is on, so `tcp_r_Link` gets `0 0 -0.003`
  and `tcp_l_Link` gets `0 0 0.003`. The mesh is MOVED, not re-cut: it keeps
  its 40 mm thickness, and nothing changes along the approach axis — the jaw
  meshes still overshoot the measured 129 mm pad tip by 6 mm, unchanged and
  still recorded.

  Pad face to pad face the description now reads **64 mm at `q = 0`, 0 mm at
  `|q| = JAW_STROKE_M`**, and `DRIVEN_OPEN_Q` puts the driver's stop at
  **51.96 mm** — the kit's own three numbers, for the first time true of the
  geometry as well as of the joints.
* **`description/d1/d1_wholebody_gripper.urdf`** (and `dist/`) regenerated:
  `generate_d1_urdf.py` gains `GRIPPER_JAW_MESH_ORIGIN`, derived the same way
  from the same two constants, and emits it on the jaw visuals. Its collision
  boxes were already on the measured 32 mm and are untouched.
* **`tools/vendoring/vendor_gripper_description.py`** gains the shift as local
  change 7, so a fresh vendor drop is re-vendored with it instead of being
  hand-patched. `descriptions/README.md` records it as local change 8.

### Added

* `description.CAD_JAW_STROKE_M` (0.035 — where the CAD put the face),
  `JAW_MESH_FACE_SIGN` and `JAW_MESH_ORIGIN_Z_M`, so a consumer that has to
  place a pad face reads it from here rather than re-measuring an STL.
  `d1-isaaclab`'s scene geometry and jaw colliders import these.
* Tests: the pad face is at `±JAW_STROKE_M` at `q = 0`, asserted from the
  authored `<origin>` alone on a CAD-free checkout and again from the real
  mesh bounds when `$MKIT_ASSETS_DIR` is set; visual and collision must carry
  the same offset; and the whole-body file's jaw mesh origins must equal the
  hand description's.

### Unchanged

* No tolerance in the 0.14.0 arrival gate moves. The gate was right about the
  robot; the sim geometry was wrong about the gripper.

## 0.14.0 — 2026-09-21

**The arrival barrier learns to ask about the jaws.** `wait_arrived` compares
seven angles, and its 3° tolerance exists because the real arm droops about a
degree under gravity (F16, J1 ~0.9°). MEASURED on `blocks-eval` (seed 7,
2026-09-21): a `grasp` straight from HOME succeeded 1 of 3, and every failure
passed that barrier at 2.2–2.8° and then closed the jaws beside the block
("stalled at 13.5 mm inside `block_red`'s 43.8 mm", 66 mm "wider than", 33 mm).
At a half-metre reach 2.8° is centimetres at the tool point, so the barrier the
grasp needed was never being asked for. `Approach` alone had a tool-space check
— in its *verifier*, after the fact — and `Grasp` / `Place` had none at all.

### Added

* `Waypoint.arrive` (default `False`). A plan can now say "the TOOL has to be
  measurably on this one". `Grasp.plan` sets it on `standoff` and `grasp`,
  `Place.plan` on `over_destination` and the release waypoint. Everything else,
  `Approach` included, is unchanged — its verifier already measures the tool
  point, and its stroke happens before the arm moves.
* `manipulation_kit.executor.ToolGate`: the barrier itself. At a gated
  waypoint it runs the joint-space arrival, then computes the tool point of
  the commanded and of the measured posture with this package's own FK
  (`primitives.approach.tool_from_link7`) and judges the miss **decomposed
  about the approach axis** (`ToolMiss`): `ARRIVE_TOL_M` (**5 mm**) ACROSS it,
  where the jaws close, `ARRIVE_TOL_ALONG_M` (**10 mm**) along it, where a
  descent is stopped by contact on purpose, and `ARRIVE_TOL_ROT_RAD`
  (**5°**). It asks the transport for nothing new: both postures are ones
  every `Executor` already reports.

  The three numbers are bounded by numbers this package already owns. Across:
  the driven jaws take 43.96 mm against a 40 mm block, so 4 mm is spare per
  side. Along: the tool point is the CENTRE of a 58 mm pad, and
  `approach.SUPPORT_CLEARANCE_M` deliberately parks the fingertips 3 mm above
  the surface, so a position-controlled arm stops a few mm high when they
  touch — measured on `blocks-eval`, grasps whose jaws closed correctly sat
  3.8–4.4 mm short along the axis and 2–4 mm across, while the ones that
  jammed were 13–17 mm short AND 10–12 mm across. Rotation: below 2.9 deg is
  inside the IK's own convergence (`safety.IK_ROT_TOL`) and above ~11.7 deg
  the presented width outgrows the jaws, so 5° sits clear of both.
* **The arm is stopped before the tool point is read** (`ARRIVE_SETTLE_S`,
  **2 s**), through the `settle` every executor already implements. A reading
  taken while the arm is still converging is where it was passing, not where it
  is going to be: measured on `blocks-eval`, reading at the instant the 3° gate
  passed made every correction round chase the same settle — 23.8 → 14.9 → 9.1
  → 7.2 mm, converging on nothing. An arm that will not stop is the typed
  refusal `not_settled`, and nothing is fed forward from a blur. The settle's
  FLAG is read with its NUMBER (`ARM_STATIONARY_DEG_S`, 3 deg/s): a settle
  that waits for the jaws too — the Isaac one does, deliberately — must not
  refuse a barrier about the arm, and did: "still moving at 0.0 deg/s ... the
  right jaws are still moving".
* **The joint gate's deadline is not the arm's last chance.** When
  `wait_arrived` times out but the settle that follows says the arm has
  stopped, the barrier re-asks where the arm is NOW and accepts it if it is
  within `tol_rad`. A long travel that was still converging used to run
  straight on into the next leg (there was no barrier there at all); turning
  that into a refusal because a 2 s tick budget expired would have been a new
  failure of its own — measured with Astra on blocks-eval, a HOME → standoff
  travel sat 75 mm out at the moment the clock stopped and was fine a moment
  later.
* **In-place correction**, on by default (`run(..., correct_arrival=True)`).
  A tool miss on a settled arm is a steady-state offset, so it is fed forward:
  the same commanded tool pose shifted by −Δp (and, when the rotation is itself
  out of tolerance, pre-rotated by the inverse error), re-solved from the
  commanded joints through `solve_ee`, collision-checked against the same guard
  the plan used, sent, re-measured. At most `MAX_ARRIVAL_CORRECTIONS` (**2**)
  rounds. A correction the guard or the IK refuses is **not sent**, and a plan
  that was guarded may not be corrected by a model with no guard installed.

  Two things the correction deliberately does not do. It does not push a
  descent deeper than `ARRIVE_TOL_ALONG_M` past its waypoint — what stops a
  descent short is usually contact, and shoving is F5 — and when ONLY the
  depth is wrong (the jaws lined up, the arm parked high) it refuses at once
  instead of spending rounds. And it re-solves up to `MAX_CORRECTION_SOLVES`
  (**8**) times per round, exactly as `planning._straight` walks a knot,
  because the IK's target is Link7 and its tolerances are Link7's: one solve
  the solver calls converged can leave the TOOL, 100 mm further out, 8 mm off —
  measured, one solve took a 17.0 mm miss to 7.8 mm and the next rounds
  returned the same joints.
* `RunReport.refusal` — a typed `RunRefusal` in the `manipulation_kit.refusal/2`
  shape a plan refusal already uses (same keys, same units: `residual_m` is the
  tool error, `residual_rad` the rotation error, `attempted` the correction
  rounds). New reasons only: `arrived_off_by`, `arrival_unknown`,
  `not_settled`, `stroke_unfinished` (`RUN_REASONS`). The schema version does
  not move — nothing that could read a refusal/2 object reads this one any less
  well. So the agent gets "the right tool point is 27 mm from the grasp pose
  after 2 corrections", not a jaw stall three steps later — and, with it, the
  MOVE that answers it, which is not the same move for the two misses: a
  lateral miss is a nudge, a miss along the approach axis is contact under the
  fingers and asking for the same descent again will not move it.
* `ArrivalReport` carries `tool_error_m`, `tool_across_m`, `tool_along_m`,
  `tool_rot_error_rad`, `settled`, `corrections` and `waypoint_label` beside
  the unchanged `worst_error_deg`, so every `run.arrivals[*]` in a trace can be
  argued with afterwards.

### Changed

* `run()` / `run_steps()` take `kin=`, `correct_arrival=`, `tool_tol_m=`,
  `tool_rot_tol_rad=` and `max_corrections=`. `kin` is the model the barrier
  computes with: the caller's, else `executor.kin`, else the kit's own guarded
  `d1/arm`, built once per process. **Pass the model the plan was built
  against** — the corrections are guarded by *that* model's guard.
* The barrier now runs at the END of a gated waypoint (once per waypoint, not
  per interpolation knot), so a standoff miss is caught BEFORE the descent —
  the one leg a plan may not re-route. The pre-stroke barrier is unchanged for
  everything else and is not paid for twice when the waypoint gate just ran.
* `FirmwareExecutor.run_plan` gates the same waypoints: a gated waypoint flushes
  the trajectory batch and measures before the next leg is uploaded, so a grasp
  now uploads two jobs (travel, descent) instead of one. It accepts the gate
  through a new keyword; an out-of-tree `run_plan` that does not take it keeps
  the joint-space barrier and is not broken.
* `RecordingExecutor.pretend_arrived` now pretends the POSTURE as well —
  it reports the last commanded joints. A double that claims an arrival while
  reporting joints a radian away is not pretending, it is lying, and the tool
  gate reads the measurement. The honest default (nothing moves, so nothing
  arrives) is untouched: that is the executor a verifier must fail against.

### Downstream

* **d1-isaaclab** (`scripts/eval/agent_eval`): `IsaacExecutor` already
  satisfies the barrier — it publishes measured joints and takes
  `send_joints` — so nothing is required. One line is worth adding:
  `run_plan()` should pass `kin=` the `d1/arm` model `run_trials` already
  builds, so corrections are guarded by the same model the plans were, instead
  of by a second one built inside the wheel.
* Anything reading `RunReport.to_json()` gets two new keys (`refusal`, and
  `tool_error_m` / `tool_rot_error_deg` / `corrections` inside `arrivals[*]`).
  Additive only.

## 0.13.1 — 2026-09-20

**The torso keep-out starts at the moving sleeve's lip.** `torso_core` ran the
full 0 .. 0.49 m of `torso_column`, i.e. from the lift origin up. The bottom
79 mm of that is not torso: with the lift retracted the sleeve's lower lip sits
at world z = 0.592 m against a lift origin of 0.513 m (measured on d1-3,
d1-isaaclab ustea physical-mount check), so that band is the fixed column — and
it lies entirely inside the chassis-side `lift_pole` box (X ±70, Y −67..73,
world z 0.389 .. 0.889), which encloses `torso_core`'s X ±45 / Y ±55 footprint.

### Changed

* `torso_core` now spans z 0.079 .. 0.49 m (`TORSO_SLEEVE_LIP_M`), size
  0.09 × 0.11 × 0.411. The guard loses no keep-out at q_lift = 0; at full
  0.30 m extension the top 3 mm of the removed band (world 0.889 .. 0.892)
  are no longer covered by any box. Every simulator that loaded the old box
  saw it collide with the AMR cover on every episode (d1-isaaclab carried a
  local carve for exactly this, now retired). Approved by Shu 2026-09-20.
* `dist/d1-collision` and `dist/d1-wholebody-gripper` re-exported.

### Downstream

* d1-firmware's Rust guard (`d1fw-core/src/guard`) mirrors the keep-out from
  this URDF and pins it with `tests/golden/guard_vectors.json`; bumping its kit
  pin needs `tools/gen_guard_vectors.py` re-run and the vectors reviewed.
* d1-isaaclab `robot/build_urdf.py` should drop `carve_torso_core` once it
  pins this version.

## 0.13.0 — 2026-09-20

**Two robot facts that consumers were reinstating downstream come home.** Both
were found while d1-isaaclab was being refactored to consume this package as
its single source of truth for the D1's shape: anything the sim had to patch
back in after loading a description was, by definition, a fact this package
was failing to state.

### Changed

* **Every arm link carries the vendor CAD inertial.** `Base_*`, `Link1_*` …
  `Link7_*` used to get the family's 0.5 kg / 1e-3 diagonal PLACEHOLDER while
  the vendor D1 arm URDFs — committed in this repository at
  `description/d1_arm/{left,right}` — carried the real masses, COMs and
  tensors all along. The generator now reads them from those files and emits
  them verbatim (`vendor_arm_inertials`). One side of the arm chain goes from
  4.05 kg of placeholder to the vendor's 8.08 kg, so gravity compensation,
  contact forces and the lift's duty are right in any sim that loads these
  files without editing them first. `TCP_Link_*` is massless in the vendor
  files and is now massless here too (it was 0.05 kg), because it is a pure
  frame at the end of the chain, not a part.

  This is a **dynamics change** for anything that loads `d1_wholebody*.urdf`
  as a physics asset. It is not a change for the guard (`d1.urdf` is a keep-out
  model; inertials are not read), for IK, or for any planner.

* **The head-camera mount tilt is a named HARDWARE REVISION, not a literal.**
  15° is a property of the head PART: d1-1, d1-2 and d1-3 wear it by design,
  and the units built next are 20° (Shu, 2026-09-20). So
  `manipulation_kit.description.HEAD_CAMERA_TILT_DEG` maps `"rev1" -> 15.0`
  and `"rev2" -> 20.0`, `head_camera_tilt_deg(revision)` resolves one (an
  unknown revision raises rather than defaulting), the generated URDFs name
  the revision they were built for in their header, and
  `mkit-urdf build --hardware-revision rev2` builds the other one. The
  committed URDFs are `rev1` and their `head_camera_mount` yaw is unchanged at
  0.261799 rad.

  **A per-robot deviation is still not this number.** d1-3's ArUco fit reads
  ~2.5° off nominal; that belongs in that robot's `cameras_d1-3.json` as an
  absolute `head_link` -> camera extrinsic, and folding it into a shared
  description — as d1-isaaclab's composed asset did, at 17.25° — makes every
  other robot wrong.

### Added

* `manipulation_kit.description.HEAD_CAMERA_TILT_DEG`,
  `DEFAULT_HARDWARE_REVISION` and `head_camera_tilt_deg()`.
* `mkit-urdf build --hardware-revision` (and `--hardware-revision` on the
  generator script).
* `tests/test_arm_inertials_and_head_revision.py`.

### Not changed, and worth saying so

The **gripper** jaw origin (0.100 m, the measured pad centre), the ±0.032 m
stroke and the 1.5 kg gripper mass are already the 2026-09-16 calliper
measurements and stay exactly as they are. d1-isaaclab's composed asset
carried the superseded vendor-CAD 0.10847 / ±0.035 / rescaled-to-1.5 versions
of all three; the fix for that is downstream, in the consumer, not here. The
camera plate (0.1053 kg) and wrist camera (0.03 kg) are hardware on top of the
1.5 kg gripper, not part of it, so the whole-body gripper links summing to
1.635 kg is correct and the tool config's 1.5 kg is correct.

## 0.12.0 — 2026-09-19

**The `[firmware]` extra no longer depends on an unpublished package.** It used
to name `d1fw-client`, which lives in a repository nobody outside the org can
`pip install` by name, so the README carried an interim "install this git URL
first" step and CI could not test the extra at all. Shu's decision on
2026-09-19: do what `d1-inference` does — ship a generated client, and check it
against the daemon's live OpenAPI document at connect time, regenerating on the
spot when they differ.

### Breaking

* **`manipulation_kit.executors.firmware` is a PACKAGE, not a module.** Every
  name it exported is re-exported from it unchanged
  (`from manipulation_kit.executors.firmware import FirmwareExecutor` still
  works), but `manipulation_kit.executors.firmware.py` is now
  `…/firmware/executor.py` and the private `_client_class()` is gone —
  replaced by `ensure_client()`. A consumer that imported the submodule path or
  that helper has to move.
* **`firmware = ["d1fw-client"]` → `firmware = ["httpx", "attrs",
  "typing_extensions"]`.** `d1fw-client` is no longer installed, used or
  mentioned. A venv that has it keeps it; nothing here imports it.
* **Execution needs Python 3.10.** The generated client is emitted with PEP 604
  unions at module scope. Planning, the guard, IK and the whole rest of the
  package still run on 3.9, and the suite says so by skipping rather than by
  passing quietly.

### Added

* **`manipulation_kit.executors.firmware.ensure`** — `ensure_client(base_url,
  *, cache_dir, policy)`. Fetches `GET <base_url>/openapi.json` (2 s timeout),
  sha256s it, and: identical to the bundled snapshot → use the bundled client;
  different → regenerate from *that* document into
  `~/.cache/manipulation-kit/d1fw/<sha>/` and import from there; cannot
  regenerate → `WARNING` + bundled (`policy="auto"`) or
  `ClientUnavailable` (`policy="strict"`); daemon unreachable → bundled with
  the reason in the note. `policy="bundled"` never asks. One INFO line per
  connect names the spec hash, the source and the path.
  `FirmwareExecutor(client_policy=…)` passes the policy through.
* **`executors/firmware/_client/`** — the committed snapshot: the generated
  `d1fw_api` (302 files, `openapi-python-client==0.29.1`, lowered to Python
  3.10), the OpenAPI document it came from, and `SNAPSHOT.json` recording its
  sha256, `info.version` and provenance. The document is
  `Omakase-Robotics-Org/d1-firmware@1937f575` `openapi/d1-firmwared.v1.json`,
  obtained through the public `d1-firmware-client-py@bfd6a678`, which vendors
  it; the firmware repository itself is private.
* **`executors/firmware/client.py`** — `FirmwareClient`, the four verbs the
  executor drives (`request`, `arm_state`, `gripper_state`, `gripper_set`) over
  the generated client's `httpx` session, with the daemon's envelope checked
  once and states parsed into validated frozen dataclasses. `api_module()`
  reaches every other generated operation without guessing the tree's name.
* **`mkit-firmware-client`** — maintainer tool. `refresh --url http://d1-2:4750`
  or `refresh --spec <file>` rewrites the document, the generated tree and
  `SNAPSHOT.json` together; `check [--url …]` verifies the committed snapshot
  is self-consistent and, optionally, matches a daemon.
* Tests: the snapshot matches its recorded hash and imports; the three policies
  against a **real loopback OpenAPI server** (match → bundled, drift +
  generator → cache, drift without a generator → warning or refusal); and the
  adapter itself against a daemon-shaped server — the first bytes this suite
  has ever put on a socket.

### Notes

* **`uv` is an optional runtime tool, never a pip dependency.** It is only used
  to run `openapi-python-client` (which needs Python 3.11, and the robots run
  3.10) when a regeneration is actually required. Without it a drifted daemon
  gets a warning and the bundled client.
* CI now installs `[firmware]` in every job, including `fresh-install`, which
  imports the executor from the built wheel in a clean venv with no daemon and
  no network. That was impossible while the extra named an unpublished package.

## 0.11.0 — 2026-09-19

Fixes for the review of PR #16 (findings R1-R14). Several are behaviour
changes a consumer will notice.

### Breaking

* **`Plan` carries a `binding`, and `manipulation_kit.executor.run` refuses to
  run a plan whose binding does not match what the executor measures.** The
  binding holds the measured joints of both arms, the observation's identity,
  the frame stamps, the tool revision and whether the collision guard was
  installed. A hand-built `Plan` now needs `run(..., allow_unbound=True)`; a
  plan made against a guard-disabled model needs `allow_unguarded=True`.
* **The `Executor` protocol grew two barriers**, `wait_arrived` and
  `wait_gripper_settled`. An executor that implements neither can still send
  joints, but `run` will not let it close or release a gripper: the stroke's
  meaning is "the tool is on the object now", and transport completion does not
  establish that. Implement them, or return an explicit `ArrivalReport(True,
  ...)` if your transport genuinely blocks.
* **`RunReport` gained `stop_reason`** from a closed set (`not_bound`,
  `unguarded_plan`, `stale_binding`, `refused_plan`, `barrier_failed`,
  `transport_error`) plus `stopped_at`, `arrivals` and `strokes`. Transport
  faults that used to raise `FirmwareUnavailable` out of `run_plan` now come
  back as a report with `stop_reason="transport_error"`.
* **A failed settle, a failed arrival or an unfinished stroke stops the run.**
  Both runners used to carry on and report `completed=True`.
* **The firmware stream no longer clips a command.** A knot larger than
  `MAX_COMMAND_STEP_DEG` raises `RateRefused` *before* anything is sent, and
  the 140 deg/s ceiling is met by stretching the schedule
  (`FirmwareExecutor.stream_schedule`) rather than by shrinking the motion.
* **`FirmwareExecutor`'s default holder is unique per session**
  (`manipulation-kit/<pid>-<token>`). A consumer that looked for the literal
  `"manipulation-kit"` in the daemon's lease view should look for the prefix.
  A changed lease epoch during a run now raises `LeasePreempted`.
* **`Grasp`/`Approach` resolve `side="auto"` before checking occupancy**, and
  an unreadable gripper is `gripper_unknown` rather than "empty". Automatic
  `Grasp` on a robot whose hands are both full is now a refusal.
* **`Lift`/`Carry`/`Place`/`Pour` on an empty hand return a `PlanError`**
  instead of raising `ValueError` from inside `plan()`.
* **`Place` no longer releases above the rim unless asked.** Pass
  `allow_drop=True`; without it a set-down the arm cannot reach is
  `unreachable_destination` and says so. `Release` over nothing is refused the
  same way.
* **`Approach` emits an opening stroke** and both it and `Lift`, `Carry`,
  `Nudge`, `Retreat` append a `SettleStep`. Step counts changed.
* **Geometry is measured along the axis it happens on.** `fits_jaws(obj)`
  became `fits_jaws(obj, frames, r_tcp)`; `grasp_point`,
  `lowest_top_down_tool_z` and `grasps_above_its_top` take a `FrameGraph`. A
  tilted object is refused with `object_tilted`.
* **`offer()` no longer takes `cap`.** It plans what it is given and returns
  all of it; capping is a rendering decision and moved to the renderer.
* **A held hand with no retained gripper command is a refusal.** The
  documented fallback — send the measurement when nobody retained the command
  — is honest for an *empty* hand and unsafe for a full one: re-commanding a
  stalled aperture tells a force-limited gripper to stop squeezing. Publish
  `RawState.commanded_grippers`.
* **`pour`'s `policy` is no longer in the tool schema.** Which checkpoint is
  served is deployment configuration, not a model choice. It is still bindable
  from Python. `pour`'s schema description now names both its limitations: it
  needs a registered policy executor, and its verifier returns UNKNOWN after
  confirming the tilt.
* **`candidates_for` generates BASE-frame nudges by default** (was `tool`).
* `Verifier.unchanged()` and `Carry._goal()` are gone (neither was used).

### New refusal reasons

`incomplete_observation`, `stale_plan`, `unsupported_geometry`,
`bad_argument` join the `PLAN_REASONS` vocabulary. `Unmet` now serialises as
`{code, detail, remedy, measured}` rather than as a sentence, and
`PlanError` carries `residual_rad`, `stage` and `attempted`. `joint_ramp` puts
radians in `residual_rad`, not in `residual_m`.

### New, non-breaking

* `manipulation_kit.primitives.offer` — the IK+guard gate and its result types.
* `manipulation_kit.primitives.schema` / `.arguments` — the canonical argument
  table, a dependency-free JSON Schema export, and `decode()` back. Names are
  narrowed by ROLE.
* `manipulation_kit.primitives.reach` — which hand can do the whole task.
* `ObjectView.extent_along/vertical_extent/tilt_rad/bottom_z/top_face_z`,
  `ContainerView.floor_z/contains_object/fits_inside/interior_measured`,
  `SurfaceView.normal/level/over/supports_object`, `WorldView.revision` and
  `observation_id()`, `FrameGraph.copy/revision`.
* `examples/preflight.py`, `examples/agent/scenes/tabletop.json`,
  `examples/agent/live.py`.

## 0.10.0 and earlier

See `docs/HISTORY.md`.
