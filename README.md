# manipulation-kit

**Everything the D1 humanoid needs to *think* about manipulation, with no
hardware in it.** Inverse kinematics, the robot description and its generator,
the in-process motion guard, end-effector tool configs and the retargeting math.
Nothing in this repository opens a socket, holds a robot lock, or moves a joint.

Hardware is `exp--d1-firmware` — the Rust daemon `d1-firmwared`, REST on
`:4750`. That split is the point of this repo: a customer doing **second
development** receives a D1 with the firmware pre-flashed, and this is the layer
they read, extend and plan against.

> **Licensing:** there is deliberately **no `LICENSE` file yet** — what we
> grant on our own code is still an open decision. What is no longer open is
> the CAD: **this repository ships the URDFs and not the geometry they
> reference.** Read **[LICENSE-STATUS.md](LICENSE-STATUS.md)** before
> publishing or sharing anything from here.

## Install

```sh
uv pip install -e .            # or: pip install -e .
uv pip install -e '.[mujoco]'  # + the MuJoCo IK substrate
uv pip install -e '.[dev]'     # + pytest
```

Assets (`d1.urdf`, `home_pose.json`, `safety_zones.json`, the URDFs) ship
**inside the package** and resolve package-relative. There is no `$D1_SDK_DIR`,
no sibling checkout, and nothing to set up:

```python
from manipulation_kit.guard import MotionGuard
from manipulation_kit.arms import get_arm_kinematics

guard = MotionGuard()                       # loads the bundled d1.urdf
arm = get_arm_kinematics("d1/arm")   # IK, clamps, HOME, guard wired in
```

## CLI

```sh
mkit-urdf variants                                    # what can be built
mkit-urdf build [--only d1.urdf] [--yubi]             # regenerate the URDFs
mkit-urdf export d1-wholebody-gripper --dest DIR      # vendor it, with provenance
mkit-urdf export d1-wholebody-gripper --dest DIR --check   # CI drift gate
mkit-urdf fetch-assets --from ~/manipulation-kit-assets     # the CAD (below)
mkit-urdf fetch-visuals --from-d1-sdk ~/d1-sdk        # optional visual layer

mkit-toolconfig list
mkit-toolconfig export d1/parallel_gripper out.json
```

| variant | what it is |
|---|---|
| `d1-wholebody-gripper` | The **authoritative** whole body (base + lift + neck + both arms) wearing the stock parallel gripper. Prebuilt (mesh-free) in [`dist/d1-wholebody-gripper/`](dist/d1-wholebody-gripper). |
| `d1-collision` | The mesh-free guard model `d1.urdf` alone — one file, zero assets, loads anywhere. Prebuilt in [`dist/d1-collision/`](dist/d1-collision). |
| `d1-yubi` | Mesh-bearing dual-arm D1 + YUBI hands (RViz, Genesis, MuJoCo). |
| `d1-arm` | The per-arm packages, left and right. |
| `d1-wholebody-o30` | **Registered, not buildable.** No O30 CAD and no measured TCP exist yet; `export` prints the four things needed, in order, rather than "unknown variant". |

**The URDFs are generated. Edit the generator, never the `.urdf`** — a test
regenerates and byte-diffs, so a hand-edit fails CI instead of shipping. That
holds with no CAD on the machine: `mkit-urdf build` reproduces all three D1
URDFs byte for byte from a bare checkout.

## The CAD is not here

**The URDFs are complete; the STL geometry they reference is not in this
repository.** Its redistribution rights are unresolved and this repository is
meant to be publishable without waiting for that answer, so on 2026-09-10 the
Omakase and vendor CAD was removed from the work tree *and from the whole git
history* and moved to the private
[`manipulation-kit-assets`](https://github.com/Omakase-Robotics-Org/manipulation-kit-assets).
`tests/test_no_vendor_cad.py` fails if any of it comes back.

Third-party geometry that already carries a licence **stays**: the YUBI hand
and the DH116S hand are both Apache-2.0 and ship here, with their upstream
`package.xml` / README intact. [`LICENSE-STATUS.md`](LICENSE-STATUS.md) has
the file-by-file table.

Nothing about the *shape* of this package changed. Every `<mesh>` reference is
verbatim, at the path it always had; the two ways to complete a checkout both
put files exactly where the URDFs already look:

```sh
export MKIT_ASSETS_DIR=~/manipulation-kit-assets            # resolve in place
mkit-urdf fetch-assets --from ~/manipulation-kit-assets     # or copy them in
mkit-urdf fetch-assets --from git@github.com:Omakase-Robotics-Org/manipulation-kit-assets.git
```

Both destinations are `.gitignore`d — a completed work tree cannot commit the
CAD back.

**Exports declare what they withhold.** `mkit-urdf export
d1-wholebody-gripper` produces the URDF and a `PROVENANCE.json` naming all 22
withheld meshes under `absent_external` (and the 13 optional visuals under
`absent_optional`), so a consumer can always tell *withheld* from *lost*. Add
`--with-assets` for the full mesh-bearing bundle; the assets repository also
carries a prebuilt one under `dist/d1-wholebody-gripper/`.

`--check` is honest in both directions: a mesh-free `dist/` verifies on a
machine that has the CAD fetched, because the export depends on the flag it
was given and never on what happens to be on disk.

## Layout

```
src/manipulation_kit/
  arms/          DLS IK + null-space, joint clamps and safety limits (single
                 source of truth), ArmClutch target shaping, filters, frames,
                 side conventions, the guard seam; d1/arm/ binding
  guard/         MotionGuard — stdlib-only joint limits + torso keep-out +
                 self-collision over the primitives-only whole-body URDF
  hands/         "<maker>/<model>" identity: tool configs, vendor CAD
                 descriptions, glove->hand retarget maps
  description/   the D1 URDF family, its generator, the exporter, the assets
  wholebody/     whole-body IK research (base + lift + neck + both arms)
  config/        the exported JSON a controller consumes
  scripts/       gesture generation and preview (pure computation)
dist/            prebuilt, provenance-tracked exports
docs/            the institutional notes, verbatim
```

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
| `manipulation_kit/wholebody/` | d1-sdk `47a6337` | `wholebody/` |
| `manipulation_kit/scripts/` | d1-sdk `47a6337` | `devices/omakase_arm/scripts/` (the pure-computation four) |
| `docs/{description-README,d1-description-README,d1-arm-notes,GESTURES,guard-README,arms-README}.md` | both | verbatim — institutional knowledge, not rewritten |
| `docs/reference/safety_zones.h` | d1-sdk `47a6337` | `devices/omakase_arm/include/omakase_arm/safety_zones.h`, **frozen** (see [`docs/reference/README.md`](docs/reference/README.md)) |

**PR #31 (`fix/ik-ok-semantics`) is applied.** It is a correctness fix, not a
refactor: the DLS loop used to iterate an unbounded joint vector, test
convergence on it, then clip into the joint limits **on the way out** — moving
joints after the test that blessed them. On d1-3 (2026-08-25) that returned
`ok=True` with the end-effector **16 cm and 81° off target**, J6 pinned to its
limit; a sweep found the same lie on 5570 of 7733 "successful" solves. The
limits are now enforced inside the iteration (projected gradient), and
`GuardedArm.solve_ee` FK-verifies whatever it is handed before committing
(`INFEASIBLE`). Two of the PR's own tests were red as filed — their fixture took
the target from an out-of-limits posture — and are fixed here **in the fixture**.

## What is deliberately NOT here

| | why |
|---|---|
| **Hand and gripper drivers** — `driver.py`, `transport.py`, `canbus.py`, `arm_passthrough.py`, `scripts/{smoke,wiggle}.py` | The wire is `d1-firmwared`'s. The force-limited grasp, supervised preload hold and thermal self-protection grown against d1-2 moved with it. `get_hand()` is gone from the registry; ask the daemon for a hand, ask this package what a hand *is*. |
| **`arms/d1/arm/channel_bus.py`** | ctypes over the vendor arm SDK `.so` — the arm's CAN channel passthrough. Hardware. |
| **The vendor SDK** — `sdk/`, the vendor arm SDK `.so`, `libKine.so` | Vendor source and binaries; `DISTRIBUTION.md §7.2` requires they stay internal. Destined for a `vendor-arm-package`. |
| **The C++ wrapper, `example/`, `numeric_ik`** | FK there is vendor `libKine`. Retired into the daemon. `test_cpp_crosscheck.py` came off with it. |
| **`pyarmstate`** | Arm modes, recovery and state sequencing are the daemon's job by definition. |
| **Every other device** — eyes, car, neck, slider, camera, audio, microphone | Not manipulation. |
| **`d1/meshes/body_hifi/*.obj`** (~96 MB, 50 MB of it referenced) | Decorative body visuals. Rendering only — the collision model is primitives everywhere and the guard never opens one. The URDFs keep their references, exports declare them absent in `PROVENANCE.json`, and `mkit-urdf fetch-visuals` completes them. |
| **`d1_yubi_description_v2/d1_arm_yubi_description/meshes/`** (~27 MB) | A byte-identical *second copy* of the arm STLs, kept in d1-sdk because ROS resolves `package://` per package. The exporter now aliases those URIs onto the single `d1_arm/` tree — verified byte-identical output — so the copy is deleted rather than policed. |

## The guard story

There are two motion guards, and they are not redundant.

**This one** (`manipulation_kit.guard.MotionGuard`) is **in-process, in Python,
stdlib-only.** It exists for planners and IK loops: `arms.GuardedArm.solve_ee`
calls it on every candidate solution, before anything is committed, thousands of
times a second, with no daemon in the path. Asking a REST endpoint per IK
iteration is not a design, and a planner that cannot evaluate a posture without
a robot is not a planner.

**The daemon's** is a Rust port in `exp--d1-firmware`, and it is the last word
on the wire: it re-checks what actually arrives, including commands from
processes that never touched this library.

They agree because the Rust port was **validated against 426 golden vectors
generated from this pyguard**. That generator, `exp--d1-firmware/tools/
gen_guard_vectors.py`, still imports the guard from a d1-sdk checkout —
**it must be repointed at `manipulation_kit.guard`**, or the two guards drift the
moment this one changes. Tracked as follow-up work.

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

## Tests

```sh
pytest
```

Everything is a test of a claim somebody once got wrong on a robot; the
docstrings say which. The suite is `pytest`-only and runs on every push
([`.github/workflows/ci.yml`](.github/workflows/ci.yml)), including
`mkit-urdf build` byte-reproduction and `--check` against the committed
`dist/`.

Tests that OPEN a mesh skip themselves, by name and with the reason, when the
CAD is absent — 10 of them. Set `MKIT_ASSETS_DIR` (or fetch) and they run.
Tests that check the URDFs still NAME the right meshes run unconditionally;
those are the ones that catch a generator which quietly stopped referencing
half the robot.
