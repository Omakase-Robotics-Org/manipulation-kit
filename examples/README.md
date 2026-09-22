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

## `agent/` — rendering the primitives to a model

Outside the wheel on purpose (Shu, 2026-09-19: 「agent 的なのは examples フォルダ
に切り離す」). These files import `manipulation_kit.primitives` and add nothing
to it, which is what keeps one vocabulary instead of one per consumer.

| script | what it does | extra needs |
|---|---|---|
| `offer.py` | the gate: plan every candidate through the real IK and the real guard, return what survives **and every refusal with its reason**. An unreachable option never becomes a word in the prompt | — |
| `chain.py` | the task planner's one question: plan the WHOLE pick-and-place (Approach → Grasp → Lift → Carry → Place) for **both** arms before anything moves, and grasp with the arm that can deliver. The near hand is only the tie-break | — |
| `schema.py` | one definition set, two exports: `tool_schemas()` (JSON Schema, for Astra-style function calling) and `choice_menu()` (already-bound options, for Jev-style typed answers). A test asserts they do not drift | — |
| `trace.py` | one JSONL record per decision: offers, refusals, the model's claim, and the **measured** verdict beside it | — |
| `astra_loop.py` | observe → offer → tool call → execute → verify, with two stop conditions. Runs a scripted stub when `OPENAI_API_KEY` is unset | `openai` only for a real run |
| `jev_menu.py` | the same offer rendered as a typed-choice request | — |
| `perceive.py` | **one head frame → a scene file.** The only calibration it takes is the ROBOT's: head-camera intrinsics and the camera pose from the neck joints. No table width, no far-edge x, no table height | `pillow` (or OpenCV); `openai` only for `--detector astra` |
| `camera.py` | the head camera as a model: pixel ↔ base-frame point, with the uncertainty the nominal mount actually carries | — |
| `live.py` | a real D1 as the loop's robot: firmware transport plus a scene | `manipulation-kit[firmware]` |

```sh
python examples/agent/astra_loop.py --dry-run
python examples/agent/jev_menu.py
python examples/agent/offer.py

# a scene from one frame, with NO scene number at all
pip install -e '.[perceive]'
python examples/agent/perceive.py --image head.jpg \
    --neck-pitch 0.52 --neck-yaw 0.0 --lift 0.205 \
    --out examples/agent/scenes/live.json --debug /tmp/fit.png

# ...and the loop doing it for itself, then declaring the things
python examples/agent/astra_loop.py --perceive head.jpg --object charger \
    --destination cup --trace /tmp/run/trace.jsonl \
    --perceive-opts "--neck-pitch 0.52 --lift 0.205"
```

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
container's interior (85 % of the outside, flagged `interior_measured: false`,
which `Place` then refuses to drop into — by design). `--size` and `--interior`
declare them when you have a tape; the file keeps what the frame said beside
what you declared. See the 0.15.0 entry in [`../CHANGELOG.md`](../CHANGELOG.md).
