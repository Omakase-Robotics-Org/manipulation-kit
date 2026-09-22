# `astra_loop.py` data flow (live robot, `--executor firmware`)

One turn = **observe → model → gate → execute → verify → record**. Nothing the
model says moves the robot until the kit's own gate has re-derived it.

```
 d1-firmwared (REST :4750)                 scene file (--scene) or perceive
   GET /v1/arm/{a,b}/state  ─┐              (objects: name, kind, p, size, …)
   GET /v1/gripper/{a,b}/state│                          │
                              ▼                          ▼
                   FirmwareExecutor.state()  ──►  LiveRobot.world()  ──►  WorldView
                   (joints, closedness, holding)  + FK tool point per arm     │
                                                  + held_object bookkeeping  │
                                                    (robot.expect(side, obj)) │
   $ASTRA_SNAPSHOT_CMD ──► turnN_{base_0,right_wrist_0,left_wrist_0}_rgb.jpg ┤
                                                                              ▼
                             messages += user: [WorldView.to_text(), images…]
                                                                              │
                                              tools = tool_schemas(world)     │
                                              (verbs × objects present)       ▼
                                          OpenAIModel  ── Responses API ──► gpt-6-astra
                                          (exactly ONE function call per turn, or a stop)
                                                                              │
                                                                              ▼
                        decode(name, args, world)  → primitive (Grasp/Lift/Carry/Place/Release/Nudge…)
                        check(primitive, world, kin) → Plan | Refusal
                          · preconditions (object_too_wide/flat, gripper_unknown, unsupported_release…)
                          · IK for every waypoint, motion guard (body / arm-arm / self)
                          · Waypoint.arrive flags → tool-space arrival gate at standoff & grasp
                                                                              │ Plan
                                                                              ▼
                        run(plan, executor)  → FirmwareExecutor
                          · lease (policy class, TTL heartbeat)  POST /v1/arm/lease
                          · POST /v1/arm/trajectory/start  (absolute-time dual-arm waypoints,
                            first point = measured pose, segments ≥ span/140 deg/s)
                          · GET  /v1/arm/trajectory/{job}/status until done
                          · ToolGate: measured FK vs commanded tool pose, ≤2 corrections
                          · POST /v1/gripper/{side}/set {closedness, grip}   (blocks until stroke ends)
                                                                              │ RunReport
                                                                              ▼
                        after = robot.world()
                        verdict = primitive.verifier(world)(after)      (per-verb measurement)
                        goal    = goal.verifier(world)(after)           (task-level measurement)
                                                                              │
                                                                              ▼
                        messages += user: "[result of <call_id>] <verb>: transport ok|<stop_reason>; measured <verdict> — <reason>"
                        trace.jsonl   += DecisionRecord (world, offered, refused, choice, plan, run, verdict, goal)
                        trace.messages.json = full chat history (images as <file names>)
```

## What the model sees
- **Text**: `WorldView.to_text()` — every object with kind, base-frame position (m), size, yaw,
  container interior; both arms (joints, tool point, stationary); both grippers (closedness,
  holding, held_object); the frames block. Base frame = torso platform, +x forward, +y robot-left.
- **Images** (when `ASTRA_SNAPSHOT_CMD` is set): the head camera and both wrist cameras of the
  same turn, as JPEG `input_image` parts (~390 input tokens each). The system prompt tells the
  model the text positions are ±1–2 cm and to prefer the photo when they disagree.
- **Tool results**: the previous call's transport outcome and the kit's measured verdict, tagged
  with the call id. Refusals arrive verbatim (`manipulation_kit.refusal/2` JSON).

## What the model never controls
- Joint targets, speeds (`vel_ratio` 0.15), the lease, the guard, the arrival gate, the grip
  torque presets (soft/firm/strong are names; the daemon's `[gripper.*]` table holds the Nm).
- Which hand: `choose_side()` plans the whole chain for both arms before turn 0 and the loop
  pins the hand; the model's `side` argument is accepted but the plan is re-checked.
- Success: only the kit's verifiers (gripper `holding` + `held_object`, FK tool point, object
  pose bookkeeping) can turn a turn "true". A model "done" without a measured goal is recorded
  as `claimed_but_unmeasured`.

## Files written per run (`--trace DIR/trace.jsonl`)
| file | content |
|---|---|
| `trace.jsonl` | one `DecisionRecord` per turn, written BEFORE any exception propagates |
| `trace.messages.json` | the exact message list sent to the model (image bytes replaced by `<file>`) |
| `turnN_base_0_rgb.jpg`, `turnN_right_wrist_0_rgb.jpg`, `turnN_left_wrist_0_rgb.jpg` | camera frames at observe time |
| `scene.json` / `scene_perceived.json` | the scene the run used |

## Known gaps (2026-09-22, d1-2)
- Object poses come from a measured scene file; perception (`perceive.py`) is in progress.
- `LiveRobot` cannot see WHAT the jaws hold — `held_object` is the loop's bookkeeping via `expect()`.
- The daemon refused one descent with "runtime speed exceeds 350 deg/s" although the kit times
  segments at ≤140 deg/s; cause open (settling arm at trajectory start suspected).
- Gripper hold on a rigid object wound up to −4.2 Nm and faulted (d1-firmwared hold controller).
