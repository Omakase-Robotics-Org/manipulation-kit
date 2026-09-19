# Changelog

Consumers pin this repository by commit and pip decides whether to reinstall by
*version*, so every change that moves what a consumer imports carries a version
bump (`tools/check_version_bump.py`). This file says what the bump was for, and
in particular what it **breaks** — the repository's rule is a clean break with a
loud reason, not a legacy path kept alive beside the new one.

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
