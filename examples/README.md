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
| `perceive.py` | **one head frame → a measured scene file.** Priors: the table top's width and the camera's `fx`. No ArUco, no tape on the objects, and with `--anchor camera` no table height either — that is measured | `pillow` (or OpenCV); `openai` only for `--detector astra` |
| `live.py` | a real D1 as the loop's robot: firmware transport plus a scene file | `manipulation-kit[firmware]` |

```sh
python examples/agent/astra_loop.py --dry-run
python examples/agent/jev_menu.py
python examples/agent/offer.py

# measure a scene from one frame instead of writing one by hand
pip install -e '.[perceive]'
python examples/agent/perceive.py --image head.jpg --table-width 0.60 \
    --anchor camera --neck-pitch 0.52 --lift 0.205 \
    --objects charger:object,cup:container --detector mask \
    --out examples/agent/scenes/live.json --debug /tmp/fit.png

# ...or let the loop do it before turn 0
python examples/agent/astra_loop.py --perceive head.jpg --object charger \
    --destination cup --trace /tmp/run/trace.jsonl \
    --perceive-opts "--table-width 0.60 --anchor camera --neck-pitch 0.52"
```

### What `perceive.py` measures, and what it cannot

The plane, the table's height and depth, each object's footprint and height —
all from the fit, all carrying a `confidence` below 1 and a `measurement`
block saying which parts were seen. What it cannot get from one silhouette is
an object's extent along the UNSEEN horizontal axis (so the measured width is
written on both, and yaw is 0) and a container's interior (85 % of the
outside, flagged `interior_measured: false`, which `Place` then refuses to
drop into — by design). Both want a second viewpoint or a tape. When you have the tape, `--size
NAME=LX,LY,LZ` and `--interior NAME=LX,LY,LZ` declare them, and the file keeps
what the frame said beside what you declared. See the 0.15.0 entry in
[`../CHANGELOG.md`](../CHANGELOG.md).

The contract these render is [`../docs/PRIMITIVE_CONTRACT.md`](../docs/PRIMITIVE_CONTRACT.md).

The gesture CSV format and the record → preview → play workflow are documented
in [`../docs/GESTURES.md`](../docs/GESTURES.md).
