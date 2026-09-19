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
| `schema.py` | one definition set, two exports: `tool_schemas()` (JSON Schema, for Astra-style function calling) and `choice_menu()` (already-bound options, for Jev-style typed answers). A test asserts they do not drift | — |
| `trace.py` | one JSONL record per decision: offers, refusals, the model's claim, and the **measured** verdict beside it | — |
| `astra_loop.py` | observe → offer → tool call → execute → verify, with two stop conditions. Runs a scripted stub when `OPENAI_API_KEY` is unset | `openai` only for a real run |
| `jev_menu.py` | the same offer rendered as a typed-choice request | — |

```sh
python examples/agent/astra_loop.py --dry-run
python examples/agent/jev_menu.py
python examples/agent/offer.py
```

The contract these render is [`../docs/PRIMITIVE_CONTRACT.md`](../docs/PRIMITIVE_CONTRACT.md).

The gesture CSV format and the record → preview → play workflow are documented
in [`../docs/GESTURES.md`](../docs/GESTURES.md).
