# How this repository came to be

The migration log. `manipulation-kit` was not written from scratch: it was
assembled out of three internal repositories in September 2026, and this file
is the record of what came from where, what deliberately did not come, and
which consumers still have to be repointed. It was the top half of the root
`README.md` until that README was rewritten for an external reader.

Nothing here is needed to *use* the kit. Read it when you are wondering why
something is shaped the way it is, or when you are the one finishing the
migration.

## What moved here, from where

Imported from `Omakase-Robotics-Org/d1-sdk` @ `47a6337` (2026-09-09) and
`Omakase-Robotics-Org/dx-manipulator` @ `2ce2c64` (2026-09-03), plus
dx-manipulator **PR #31** `fix/ik-ok-semantics` @ `b0a5338`.

| here | from | path there |
|---|---|---|
| `manipulation_kit/arms/` | dx-manipulator `2ce2c64` | `arms/` (`ik.py`, `kinematics.py`, `safety.py`, `targeting.py`, `filters.py`, `frames.py`, `sides.py`, `guard.py`, `d1/arm/{kinematics,mujoco_chain}.py`) |
| `manipulation_kit/hands/` | dx-manipulator `2ce2c64` | `hands/` — registry, `toolconfig.py`, per-model `toolconfig.py`, `d1/parallel_gripper/{description.py,descriptions/,tools/}`, `leadshine/dh116s/{coupling.py,data/,description.py,descriptions/,retarget.py}`, `linkerbot/o30/retarget.py` |
| `manipulation_kit/guard/` | d1-sdk `47a6337` | `devices/omakase_arm/pyguard/` |
| `manipulation_kit/description/` | d1-sdk `47a6337` | `description/` (generator, exporter, assets) |
| `manipulation_kit/config/` | d1-sdk `47a6337` | `devices/omakase_arm/config/*.json` + `config/tool_configs/*.json` |
| `contrib/wholebody/` | d1-sdk `47a6337` | `wholebody/` |
| `examples/` | d1-sdk `47a6337` | `devices/omakase_arm/scripts/` (the pure-computation four) |
| `docs/{description-README,d1-description-README,d1-arm-notes,GESTURES,guard-README,arms-README}.md` | both | verbatim — institutional knowledge, not rewritten |
| `docs/reference/safety_zones.h` | d1-sdk `47a6337` | `devices/omakase_arm/include/omakase_arm/safety_zones.h`, **frozen** (see [`reference/README.md`](reference/README.md)) |

`wholebody/` and `scripts/` landed under `src/` in the first import and moved
out of the wheel later: neither is imported by the package, both carry
dependencies the kit itself refuses (mujoco, torch, a display), and a consumer
installing IK should not be installing a research benchmark.

## PR #31 (`fix/ik-ok-semantics`) is applied

It is a correctness fix, not a refactor: the DLS loop used to iterate an
unbounded joint vector, test convergence on it, then clip into the joint limits
**on the way out** — moving joints after the test that blessed them. On `d1-3`
(2026-08-25) that returned `ok=True` with the end-effector **16 cm and 81° off
target**, J6 pinned to its limit; a sweep found the same lie on 5570 of 7733
"successful" solves. The limits are now enforced inside the iteration
(projected gradient), and `GuardedArm.solve_ee` FK-verifies whatever it is
handed before committing (`INFEASIBLE`). Two of the PR's own tests were red as
filed — their fixture took the target from an out-of-limits posture — and are
fixed here **in the fixture**. `tests/arms/test_ik_feasibility.py` keeps the
pre-fix algorithm inlined and scores it beside the current one on every run.

## Why the version has to move with every change

**Consumers pin a commit** — `manipulation-kit @
git+ssh://git@github.com/Omakase-Robotics-Org/manipulation-kit.git@<sha>` in
`requirements.txt` / `pyproject.toml` (dx-vr-teleop, omakase-core,
d1-inference, poc-dx-inspect-robots). pip compares **versions**, never commits:
against an installed `0.1.0` a new `0.1.0` from a different sha is "Requirement
already satisfied", so `git pull && pip install -e .` prints success and leaves
the OLD kit in the venv.

That is not a hypothetical. On 2026-09-09 the history rewrite that renamed the
modules (`6655b2d` → `ce802ad`) reached three repositories on `d1-2` as a green
install followed by `ModuleNotFoundError: manipulation_kit.arms.d1`, because pip
had reinstalled nothing. `tools/check_version_bump.py` runs in CI and fails a
pull request that changes `src/` or the dependency lists without moving
`project.version`; consumers run the mirror-image check at start-up and refuse
to launch against a kit that is not the commit they pin.

## What is deliberately NOT here

| | why |
|---|---|
| **Hand and gripper drivers** — `driver.py`, `transport.py`, `canbus.py`, `arm_passthrough.py`, `scripts/{smoke,wiggle}.py` | The wire is `d1-firmwared`'s. The force-limited grasp, supervised preload hold and thermal self-protection grown against the robots moved with it. `get_hand()` is gone from the registry; ask the daemon for a hand, ask this package what a hand *is*. |
| **`arms/d1/arm/channel_bus.py`** | ctypes over the vendor arm SDK `.so` — the arm's CAN channel passthrough. Hardware. |
| **The vendor SDK** — `sdk/`, the vendor arm SDK `.so`, `libKine.so` | Vendor source and binaries; `DISTRIBUTION.md §7.2` requires they stay internal. Destined for a `vendor-arm-package`. |
| **The C++ wrapper, `example/`, `numeric_ik`** | FK there is vendor `libKine`. Retired into the daemon. `test_cpp_crosscheck.py` came off with it. |
| **`pyarmstate`** | Arm modes, recovery and state sequencing are the daemon's job by definition. |
| **Every other device** — eyes, car, neck, slider, camera, audio, microphone | Not manipulation. |
| **`d1/meshes/body_hifi/*.obj`** (~96 MB, 50 MB of it referenced) | Decorative body visuals. Rendering only — the collision model is primitives everywhere and the guard never opens one. The URDFs keep their references, exports declare them absent in `PROVENANCE.json`, and `mkit-urdf fetch-visuals` completes them. |
| **`d1_yubi_description_v2/d1_arm_yubi_description/meshes/`** (~27 MB) | A byte-identical *second copy* of the arm STLs, kept in d1-sdk because ROS resolves `package://` per package. The exporter now aliases those URIs onto the single `d1_arm/` tree — verified byte-identical output — so the copy is deleted rather than policed. |

## The two guards, and the one that must be repointed

`manipulation_kit.guard.MotionGuard` is in-process, in Python, stdlib-only; the
daemon's is a Rust port in `exp--d1-firmware` and is the last word on the wire.
They agree because the Rust port was **validated against 426 golden vectors
generated from this pyguard**. That generator,
`exp--d1-firmware/tools/gen_guard_vectors.py`, still imports the guard from a
d1-sdk checkout — **it must be repointed at `manipulation_kit.guard`**, or the
two guards drift the moment this one changes. Tracked as follow-up work.

`config/safety_zones.json` stays honest the same way: it is a *derived export*
of numbers authored in `docs/reference/safety_zones.h`, and
`tests/guard/test_safety_zones_export.py` asserts field by field that it still
is. When the daemon publishes its own authored source, repoint that test and
delete the frozen header.

## Consumers to repoint

These still import `omakase_arms` / `omakase_hands` / `pyguard`, or resolve
`$D1_SDK_DIR`. Each needs a PR of its own:

| repo | files |
|---|---|
| **dx-vr-teleop** | `server/sdk/paths.py`, `arm/home.py` |
| **omakase-core** | `_sdk_facts.py`, `constants.py` |
| **poc-dx-inspect-robots** | `config.py`, `guard.py` |
| **d1-inference** | `preflight.py` |
| **exp--d1-firmware** | `tools/gen_guard_vectors.py` (see above) |

## Phase 1 of 2: the copy in dx-vr-teleop is still live

Nothing was deleted from `dx-vr-teleop` when `arms/` was migrated — by design,
both copies run until the migration is verified. That is only safe because
"have they diverged?" has an answer: `tests/test_parity_dx_vr_teleop.py` drives
both implementations against each other on the same description, the same
guard and the same input sequences, and asserts they agree exactly. It runs
against the MuJoCo substrate, because that is the one dx-vr-teleop uses. See
[`arms-README.md`](arms-README.md).
