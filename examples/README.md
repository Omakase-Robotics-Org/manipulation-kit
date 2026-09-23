# Examples

Runnable scripts built on the installed package. They are **outside the wheel**
— nothing in `manipulation_kit` imports them — so they are free to want a
display, a video encoder or a vendor binary without putting any of that in the
dependency list.

Install the kit first (`pip install -e .` from the repository root); each script
resolves the URDF and the HOME pose through that import, so it works from a
checkout and from an installed wheel alike.

| script | what it does | extra needs |
|---|---|---|
| `make_joint_test_gesture.py` | generate the joint-wiggle assembly-test gesture CSV, every keyframe validated by `MotionGuard` | — |
| `preview_gesture.py` | render a gesture CSV to MP4 so you can watch it before a robot does | `mujoco`, `imageio[ffmpeg]` |
| `ik_click_move.py` | click a point in a MuJoCo view, solve IK to it, export the session as a gesture; `--targets FILE` runs the same thing headless | `mujoco`, a display (except `--targets`) |
| `make_cartesian_test_gesture.py` | the Cartesian half of the assembly test | **does not run here**: it shells out to the d1-sdk C++ `fk_batch`/`ik_batch`, which are not in this repository. Kept for the waypoint layout and the seeding strategy; see its header |

## `agent/` — a model driving the kit

Outside the wheel on purpose (Shu, 2026-09-19: 「agent 的なのは examples フォルダ
に切り離す」). Everything a customer has to TRUST — the gate, the schema, the
task planner, the loop, the operator policy, the robot, the trace — is in the
wheel (`manipulation_kit.primitives.{offer,schema,reach}`,
`manipulation_kit.agent`); these files are the part that knows a model exists.

| script | what it does | extra needs |
|---|---|---|
| `astra_loop.py` | the prompt, the OpenAI client, `loop()` and `main()` over `manipulation_kit.agent` (< 200 lines). Runs the scripted stand-in unless `--model` is given | `openai` only for `--model` |
| `scripted.py` | `ScriptedModel`: a stand-in that plays a correct pick-and-place with no key and no network — and claims "done" one verb early **on purpose** (see below) | — |
| `jev_menu.py`, `menu.py` | the same offer rendered as a typed-choice (Jev) request: ranking, the cap, wait/rescan/stop | — |
| `jev_servo.py` | `astra_loop` with a `Servo`: Astra decides the verbs (System 2), Jev-Omni judges the wrist look (System 1) — the kit draws its projected pixel on the wrist photo, Jev says on / left / right / above / below it, the kit steps the hand. `--dry-run --misplace-mm 40` runs the geometry stand-in with no model | `--judge jev`: `huggingface_hub`, `torch`, `torchvision`, `transformers`, a CUDA GPU (~26 GiB), `--snapshot-cmd` |
| `perceive.py` | **one head frame → a scene file**, a thin CLI over `manipulation_kit.perception`. The only calibration it takes is the ROBOT's (intrinsics, neck joints, `--robot-profile` for the measured head mount); no table width, no far-edge x, no table height | `pillow` (or OpenCV); `openai` only for `--detector astra` |
| `run_scene.py` | where a run's scene comes from before turn 0: `--perceive` (one frame, the live neck on firmware) or `--scene` (resolved against the robot profile) | as `perceive.py` |
| `detector.py` | `AstraDetector`: a model as the box detector for `perceive.py --detector astra` — the prompt, the call and a strict parse | `openai` |
| `snapshot.py` | the camera-grab contract: run `--snapshot-cmd`, check its exit code, require fresh frames, label each camera — or stop the loop | — |
| `scene.py`, `scenes/` | the demo scene, and two MEASURED scene files (`tabletop.json`, `d1-2_tape_cup.json`; a robot's measured numbers are NOT in them — they come from the robot's calibration file) | — |

The gate (`offer.py`), the schema export (`schema.py`) and the chain planner
(`reach.py`) that older versions of this table listed here are in the wheel,
under `manipulation_kit/primitives/`.

### `--executor`: where the plan runs

| flag | what it builds |
|---|---|
| `--executor kinematic` (default) | the kinematic mirror: a `KinematicExecutor` and the scene it moves. Plans are well formed; nothing about whether a grasp holds |
| `--executor firmware --robot http://d1-2:4750` | a real D1 through d1-firmwared, configured by the operator policy (`--vel-ratio`, timeouts) |
| `--executor isaac --isaac-url tcp://HOST:8977` | d1-isaaclab's simulator, when that package registers itself (entry point `manipulation_kit.executors`), or with `--executor-class agent_eval.kit_executor:isaac`; see `docs/agent.md` |

A robot's MEASURED hand gap, head mount and wrist lenses come from its
`omakase.camera_calibration/2` file: `~/.config/omakase/camera_calibration.json`
on the robot (read by default), or `--robot-profile PATH` (offline, d1-2's is
`tests/data/d1-2.camera_calibration.json`). A layer whose calibration gate
FAILED is refused unless `--allow-failed-calibration`. The operator policy is flags (`--max-grip`,
`--allowed-directions`, `--no-look-before-stroke`, `--droop-margin-m`, ...)
or `--policy FILE`.

```sh
python examples/agent/astra_loop.py --dry-run
python examples/agent/astra_loop.py --dry-run --executor kinematic \
    --scene examples/agent/scenes/d1-2_tape_cup.json --object cube --destination cup \
    --robot-profile tests/data/d1-2.camera_calibration.json
python examples/agent/jev_menu.py

# a scene from one frame, with NO scene number at all
pip install -e '.[perception]'
python examples/agent/perceive.py --image head.jpg \
    --neck-pitch 0.52 --neck-yaw 0.0 --lift 0.205 --robot-profile tests/data/d1-2.camera_calibration.json \
    --out examples/agent/scenes/live.json --debug /tmp/fit.png

# ...and the loop doing it for itself, then declaring the things
python examples/agent/astra_loop.py --perceive head.jpg --object charger \
    --destination cup --trace /tmp/run/trace.jsonl \
    --perceive-opts "--neck-pitch 0.52 --lift 0.205"
```

**`CLAIMED DONE, NOT MEASURED, at turn 4` is intentional.** The scripted
stand-in says "done" at the carry, one verb before the place; the task
verifier says the cube is still in the hand, the loop carries on, and the run
ends `goal_verified` on the measurement. The line is the demonstration that a
model's claim never ends a run.

### The model is the detector

`perceive.py`'s default (`--detector model`) finds **nothing**. It writes the
camera, the table and no things, and the loop's own model — looking at the same
frame — declares them with two tools `astra_loop.py` adds beside the motion
verbs:

| tool | what it does |
|---|---|
| `declare_scene(objects=[…])` | the model says where things are, in base metres. Same reader a hand-written scene file goes through, so it cannot declare something a person could not have written |
| `locate(u, v)` | a pixel it picked → a base-frame point on the current table plane, with its uncertainty. Exact geometry, so the model is not doing projective maths in its head |

### What one camera cannot do

**Measure the height of the plane it is looking at.** Twice as far and twice as
big is the same picture. So the height is `declared` (by the model, or
`--table-z`), `known-length` (`--table-width`, the one optional scene number,
solved in closed form), or `provisional` — and in the last case every object
measured against it carries `confidence` 0.2 and says so. Everything else in
the fit is scale-free, so one declared number fixes the whole scene at once.

Also unmeasurable from one silhouette: an object's extent along the axis you
cannot see (the width is written on both horizontal axes and yaw is 0), and a
container's interior (90 % of the outside, flagged `interior_measured: false`,
which `Place` then refuses to drop into — by design). `--size` and `--interior`
declare them when you have a tape; the file keeps what the frame said beside
what you declared. See the 0.15.0 entry in [`../CHANGELOG.md`](../CHANGELOG.md).
