# Changelog

Consumers pin this repository by commit and pip decides whether to reinstall by
*version*, so every change that moves what a consumer imports carries a version
bump (`tools/check_version_bump.py`). This file says what the bump was for, and
in particular what it **breaks** — the repository's rule is a clean break with a
loud reason, not a legacy path kept alive beside the new one.

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
  (`primitives.approach.tool_from_link7`) and requires `ARRIVE_TOL_M` (**5 mm**)
  and `ARRIVE_TOL_ROT_RAD` (**2°**). It asks the transport for nothing new:
  both postures are ones every `Executor` already reports.
* **The arm is stopped before the tool point is read** (`ARRIVE_SETTLE_S`,
  **2 s**), through the `settle` every executor already implements. A reading
  taken while the arm is still converging is where it was passing, not where it
  is going to be: measured on `blocks-eval`, reading at the instant the 3° gate
  passed made every correction round chase the same settle — 23.8 → 14.9 → 9.1
  → 7.2 mm, converging on nothing. An arm that will not stop is the typed
  refusal `not_settled`, and nothing is fed forward from a blur.
* **In-place correction**, on by default (`run(..., correct_arrival=True)`).
  A tool miss on a settled arm is a steady-state offset, so it is fed forward:
  the same commanded tool pose shifted by −Δp (and, when the rotation is itself
  out of tolerance, pre-rotated by the inverse error), re-solved from the
  commanded joints through `solve_ee`, collision-checked against the same guard
  the plan used, sent, re-measured. At most `MAX_ARRIVAL_CORRECTIONS` (**2**)
  rounds. A correction the guard or the IK refuses is **not sent**, and a plan
  that was guarded may not be corrected by a model with no guard installed.
* `RunReport.refusal` — a typed `RunRefusal` in the `manipulation_kit.refusal/2`
  shape a plan refusal already uses (same keys, same units: `residual_m` is the
  tool error, `residual_rad` the rotation error, `attempted` the correction
  rounds). New reasons only: `arrived_off_by`, `arrival_unknown`,
  `not_settled`, `stroke_unfinished` (`RUN_REASONS`). The schema version does
  not move — nothing that could read a refusal/2 object reads this one any less
  well. So the agent gets "the right tool point is 27 mm from the grasp pose
  after 2 corrections", not a jaw stall three steps later.
* `ArrivalReport` carries `tool_error_m`, `tool_rot_error_rad`, `corrections`
  and `waypoint_label` beside the unchanged `worst_error_deg`, so every
  `run.arrivals[*]` in a trace can be argued with afterwards.

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
