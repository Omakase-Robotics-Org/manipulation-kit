# manipulation-kit

**Everything a D1 humanoid needs to *think* about manipulation, plus one way to
send it.** Inverse kinematics, the robot description and its generator, the
in-process motion guard, end-effector tool configs, the retargeting math, and a
set of **verbs** — `Approach Grasp Lift Carry Place Release Nudge Retreat
GoHome` — each with cheap preconditions, a pure `plan()` and a **measured**
verifier.

**Planning is pure: it opens no socket, holds no lock and moves no joint.** One
module is the deliberate exception — `manipulation_kit.executors.firmware`,
behind the optional `[firmware]` extra — because otherwise every consumer
re-derives the same lease handling, the same mode entry and the same rate
clamp, and that is how safety code ends up in four copies that disagree.

Hardware is a separate layer — the `d1-firmwared` daemon, REST on `:4750`. That
split is the point of this repo: a customer doing **second development**
receives a D1 with the firmware pre-flashed, and this is the layer they read,
extend and plan against.

> **Licensing:** Apache-2.0 (see [`LICENSE`](LICENSE) and [`NOTICE`](NOTICE)).
> One thing to know up front: **this repository ships the URDFs and not the
> vendor CAD they reference.** See [`LICENSE-STATUS.md`](LICENSE-STATUS.md) for
> the provenance and the third-party (YUBI, DH116S) attributions.

---

## Quickstart: a measured pickup of a named block

This is the whole path, in order, and it is meant to take under an hour on a
prepared D1. It assumes **no camera and no detector**: you measure the block
once with a tape, and the robot does the rest. Automatic perception is a later
problem and does not block this.

**What you need:** a D1 running `d1-firmwared` that you can reach over HTTP, the
stock parallel gripper, a table, a block that fits the jaws (≤ 44 mm across the
face the jaws close on), and a tape measure.

### 1. Install

```sh
pip install -e .                 # numpy + scipy. Planning only.
pip install -e '.[firmware]'     # + d1fw-client, for the one module with a wire
```

`[firmware]` names `d1fw-client` as a plain distribution. **Until that package
is published, install it explicitly first** — this step is interim, and Shu
decides publication separately:

```sh
pip install "git+ssh://git@github.com/Omakase-Robotics-Org/d1-firmware-client-py.git@bfd6a678"
pip install -e '.[firmware]'
```

The pin is a commit on purpose. Without SSH access to that organisation this
step does not work yet, and the rest of this Quickstart does not either: the
planning half below runs on the base install, the execution half does not.
Everything else — URDFs, `home_pose.json`, `safety_zones.json`, the tool
configs — ships **inside the package** and resolves package-relative. There is
nothing to download and no environment variable to set.

### 2. Connect and preflight

```sh
python examples/preflight.py --robot http://d1-2:4750 \
    --scene examples/agent/scenes/tabletop.json
```

It reads the lease, both arms' joints, both grippers and the arm mode; it
prints the tool revision the kit plans with and the **forward-kinematic tool
point for the arm's measured joints**. Put a tape measure on that point. If it
disagrees with the robot by more than a few millimetres, stop — a 10 mm FK
error is a 10 mm grasp error and no amount of careful scene measurement
recovers it. Exit code 0 means every check passed.

Two things it will refuse on, and both are the point: a gripper it cannot read
(that is **UNKNOWN**, not "open") and an arm whose commanded pose is far from
its measurement (engaging position control there snaps the arm to a stale
command — on d1-2, 2026-09-10, by 48 degrees).

### 3. Describe the block you measured

[`examples/agent/scenes/tabletop.json`](examples/agent/scenes/tabletop.json) is
a worked example with the conventions written into it. The short version:

| field | what it means |
|---|---|
| units | **metres and radians**, everywhere. There are no millimetres in the file. |
| `base` | +x forward, +y the robot's **left**, +z up, origin on the torso platform. It is where the IK reports, so it is the root of every transform. |
| `p` | the object's **centre** — not its bottom, not a corner. A 50 mm cube on a table whose top is at z = 0.010 has `p[2] = 0.035`. |
| `size` | the full extent along the object's **own** axes, `(length, width, height)`. A yawed box has the same size and a different `yaw_rad`. |
| `interior` | a container's usable **inner** extent. Leave it out and the kit marks it *estimated* and refuses to place into it — the walls are then not where anybody said they are. |
| `frame_id` | the frame the pose was measured in. A frame with `max_age_s` expires, and everything measured in it then **refuses** rather than returning a plausible number. |

The robot half of the observation — both arms, their tool poses, both grippers
— is filled in from live state by
[`examples/agent/live.py`](examples/agent/live.py); you supply only the things.

### 4. Offer: what can it actually do right now?

```python
from manipulation_kit.arms import get_arm_kinematics
from manipulation_kit.primitives.offer import candidates_for, offer, why_nothing

kin = get_arm_kinematics("d1/arm", quiet=True)
offered, refused = offer(candidates_for(world), world, kin)
print("\n".join(o.label for o in offered) or why_nothing(refused))
```

Every candidate is **planned** — same IK, same joint clamp, same collision
guard the teleop stack runs — before it is offered. An unreachable action never
becomes a word in a prompt, and every refusal keeps its reason, its waypoint
and its residual.

### 5. Choose

Three ways, same checked actions underneath:

```sh
python examples/agent/astra_loop.py --dry-run          # scripted: no key, no network
OPENAI_API_KEY=... python examples/agent/astra_loop.py # a function-calling model
python examples/agent/jev_menu.py --task "put the red block in the box"
```

`pip install openai` first for the second one — it is not a dependency of this
repository.

### 6. Execute, re-observe, verify

```python
from manipulation_kit.executor import run
from manipulation_kit.executors.firmware import FirmwareExecutor

with FirmwareExecutor(base_url="http://d1-2:4750") as robot:
    verb = Grasp(object="red_block", side="left")
    plan = verb.plan(world, kin)          # pure: nothing has moved
    report = run(plan, robot)             # lease, mode, barriers, transport
    after = source.world()                # RE-OBSERVE. Always.
    print(report.completed, verb.verifier(world)(after).to_json())
```

`report.completed` is about the **executor**: every step was sent and every
barrier was met. Whether the block is in the hand is the **verifier's** answer,
from a later observation, and the two are reported separately on purpose.

To verify the pickup rather than the stroke, ask for the thing you actually
want — the block went up, and it is the block:

```python
from manipulation_kit.primitives import Lift
lift = Lift(object="red_block", side="left", height_m=0.10)
run(lift.plan(after, kin), robot)
print(lift.verifier(after)(source.world()).to_json())
```

### FALSE versus UNKNOWN, and what to do about each

Three verdicts, and the third one is the point.

| verdict | what it means | what to do |
|---|---|---|
| **TRUE** | the evidence is there and it says the thing happened | go on |
| **FALSE** | the evidence is there and it says otherwise — the object did not rise, the jaws stalled at the wrong width, the gripper reports something else | re-observe and replan. This is a real failure and a retry of the same action will fail the same way |
| **UNKNOWN** | the evidence **does not exist on this robot**: no gripper report, the object is not in the later observation, the frame it was measured in has expired, or nothing ties the thing between the pads to the name you asked for | **fix the observation, not the robot.** Publish `GripperView(held_object=...)`, re-measure the frame, keep the object in view. Treating UNKNOWN as success is how a task gets marked done because nobody was looking |

The pickup predicate is deliberately strict: a torque stall proves *something*
is between the pads, never that it is *the named block*. It becomes TRUE when
the producer names what it holds (`held_object`) **or** the named object is
measured at the tool point; with neither, it is UNKNOWN and the verdict says
what would settle it.

**If something goes wrong.** A refused plan sent nothing — read
`PlanError.reason` and the residual, and change the ask. A `RunReport` with
`completed=False` carries a `stop_reason`: `stale_binding` (the world moved
between planning and running — re-observe and replan), `barrier_failed` (the
arm did not arrive, the stroke did not finish, or the arms did not settle —
nothing after that step ran), `transport_error` (the daemon refused or the
lease was lost). With an object still held, `Release(side=..., allow_drop=True)`
is the explicit way to let go; without `allow_drop` the kit refuses to open a
hand over nothing.

---

## Install

```sh
pip install -e .            # everything below: numpy + scipy, nothing else
pip install -e '.[dev]'     # + pytest, for the test suite
pip install -e '.[mujoco]'  # + the OPTIONAL alternative IK substrate
pip install -e '.[firmware]'  # + d1fw-client, for executors/firmware.py ONLY
```

**MuJoCo is not required.** Inverse kinematics runs on numpy over the URDF this
package ships. The extra exists for consumers that already keep a MuJoCo mirror
of the robot, and for the demos under `contrib/` and `examples/` — see
[Substrates](#substrates).

Assets (`d1.urdf`, `home_pose.json`, `safety_zones.json`, the tool configs) ship
**inside the package** and resolve package-relative. There is no environment
variable to set, no sibling checkout to find, and nothing to download:

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

from manipulation_kit.arms import get_arm_kinematics
from manipulation_kit.guard import MotionGuard

arm = get_arm_kinematics("d1/arm")      # IK, clamps, HOME, guard wired in
guard = MotionGuard()                   # or build your own and inject it

p, r = arm.ee_pose("left")              # where the left hand is now, base frame
res = arm.solve_ee("left", p + np.array([0.05, 0.0, 0.0]),
                   R.from_rotvec([0, 0, 0.1]) * r)
if res.ok:
    print(np.degrees(res.q))            # 7 joint angles, safe to command
else:
    print(res.reason)                   # ik_fail | infeasible | guard_reject
```

`res.ok` means *the solution reaches the pose asked for*, not *the arm is there
yet*: a solution further than the per-tick joint cap is committed partially and
flagged `step_clamped`, so a controller loops and re-asks. Both claims are
load-bearing; `manipulation_kit.arms.kinematics` documents why.

### Substrates

`solve_ik` talks to a `KinematicChain` — anything posable that can then report
its end-effector pose, its Jacobian and its joint limits. Two ship:

| `chain=` | what it is | when |
|---|---|---|
| `"urdf"` (default) | `arms/urdf_chain.py` — Rodrigues FK and a geometric Jacobian over `d1.urdf`, in numpy | always, unless you have a reason not to |
| `"mujoco"` | `mj_jacBody` on a `MjModel`/`MjData` of the same URDF | you already run MuJoCo and want the solver driving *that* model rather than a second one |

They are measured against each other, not merely assumed interchangeable:
`tests/arms/test_urdf_chain_parity.py` sweeps 500 random in-limit postures per
arm and compares FK, the 6×7 Jacobian, the joint limits, the READY-seed search
and full guarded-solve trajectories. Worst observed difference is ~5e-16 m /
~1.4e-15 rad — double-precision noise.

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
meant to be publishable without waiting for that answer, so the Omakase and
vendor CAD was removed from the work tree *and from the whole git history* and
moved to the private
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
src/manipulation_kit/     the installed package — this, and only this, is the wheel
  arms/          DLS IK + null-space, the numpy URDF substrate, joint clamps
                 and safety limits (single source of truth), ArmClutch target
                 shaping, filters, frames, side conventions; d1/arm/ binding
  world/         ObjectView / ContainerView / SurfaceView / ArmView /
                 GripperView / WorldView and the FrameGraph — the perception
                 RESULTS the kit reads. It never imports a camera
  primitives/    Approach Grasp Lift Carry Place Release Nudge Retreat GoHome
                 (+ the Pour contract): preconditions, a pure plan(), and a
                 measured verifier(). See docs/PRIMITIVE_CONTRACT.md.
                 Beside them, the model-INDEPENDENT action boundary:
                 offer.py (the IK+guard gate), schema.py + arguments.py (the
                 canonical argument table and a dependency-free JSON Schema
                 export), reach.py (which hand can do the whole task)
  executor.py    the Executor protocol and two pure test doubles
  executors/     the ONE place with a wire: firmware.py, behind [firmware]
  guard/         MotionGuard — stdlib-only joint limits + torso keep-out +
                 self-collision over the primitives-only whole-body URDF
  hands/         "<maker>/<model>" identity: tool configs, CAD descriptions,
                 glove->hand retarget maps
  description/   the D1 URDF family, its generator, the exporter, the assets
  config/        the exported JSON a controller consumes
contrib/         research, not installed (whole-body IK: base + lift + neck)
examples/        runnable scripts: preflight.py (the live-robot check),
                 gesture generation, preview, click-to-move IK, and agent/ —
                 the part that knows a model exists
tools/vendoring/ CAD re-import; needs the private assets repo, not installed
dist/            prebuilt, provenance-tracked exports
docs/            the institutional notes, verbatim
```

## Primitives and the agent examples

Above the IK there is a small set of **verbs**: `Approach Grasp Lift Carry
Place Release Nudge Retreat GoHome`, plus the `Pour` contract whose body is a
learned policy. Each is a frozen dataclass with the same three parts —
`preconditions(world)`, a pure `plan(world, kin)`, and a `verifier(world0)`
that returns a **measured** verdict from a later observation. The full contract
is [`docs/PRIMITIVE_CONTRACT.md`](docs/PRIMITIVE_CONTRACT.md).

```python
from manipulation_kit.arms import get_arm_kinematics
from manipulation_kit.executor import KinematicExecutor, run
from manipulation_kit.primitives import Grasp
from manipulation_kit.world import ArmView, GripperView, ObjectView, WorldView

kin = get_arm_kinematics("d1/arm", quiet=True)
world = WorldView.of(
    [ObjectView("red_block", p=(0.38, 0.25, 0.05), size=(0.05, 0.04, 0.05))],
    arms=[ArmView(s, joints=kin.joints(s)) for s in ("left", "right")],
    grippers=[GripperView(s, 0.0) for s in ("left", "right")])

verb = Grasp(object="red_block", side="left", approach="top_down")
plan = verb.plan(world, kin)                 # pure — nothing has moved
print(plan if not plan.ok else run(plan, KinematicExecutor(kin)))
print(verb.verifier(world)(world).verdict)   # 'false': nothing was measured yet
```

A `Plan` is **bound** to what it was checked against — the measured joints of
both arms, the observation's revision, the frame stamps and the tool geometry.
`run()` compares that with what the executor measures and refuses to send a
byte if the world moved in between; the answer to drift is to replan, never to
clamp.

Three things are load-bearing, and each is a bug somebody shipped:

- **`ObjectView.size` is mandatory and `frame_id` travels with the pose.**
  Every clearance is computed from the extent, and a frame that has gone stale
  refuses (`frame_stale`) rather than returning a plausible number — the
  `table_frame` homography lesson, in the type system.
- **`plan()` is checked end to end before the first joint moves.** Every
  waypoint is split into per-tick knots and each one passes the same
  `solve_ee` the teleop stack runs, guard included. A refusal is a typed
  `PlanError` with the waypoint and the residual, never a silent no-op.
- **A guard-rejected knot is routed around, not reported.** A top-down grasp
  over a wagon can have a clean standoff, a clean grasp point, and a straight
  line between them that puts the elbow through the torso. The rejected
  waypoint is retried through a short measured list of clearance points
  (`planning.VIA_OFFSETS_M`) and then a `ready()` re-seed; only a waypoint
  nothing reaches is refused, and the refusal is still the straight line's.
- **Orientation is derived, not emitted.** A caller names one of four
  approaches; the kit computes the wrist from the approach axis and the
  object's principal axis. `Nudge` is the only verb whose numbers are *snapped*
  — ±10/30/50 mm and ±15° of yaw about the approach axis — but it is not the
  only one that takes numbers: `standoff_m`, `height_m`, `clearance_m`,
  `distance_m` and `tilt_deg` are used as written, inside the published range.
  Every one of them is validated against the kit's own argument table before
  anything is planned, so a nonfinite or out-of-range value is a typed
  refusal rather than a plan.
- **A geometry question is asked about the axis it happens on.** Whether an
  object fits the jaws is its extent along *the jaw axis of that grasp*, not
  its smallest side; how tall it stands is its extent along base +z from its
  *resolved* orientation. A tilted box is refused rather than approximated.

### Running a plan

`manipulation_kit.executor` holds the `Executor` protocol
(`state` / `send_joints` / `set_gripper` / `settle`, plus the two measured
barriers `wait_arrived` / `wait_gripper_settled`) and two pure doubles:
`RecordingExecutor`, which accepts everything and moves nothing, and
`KinematicExecutor`, which mirrors the plan onto the model.

`manipulation_kit.executors.firmware` is **the one module in this repository
that opens a socket** — a deliberate exception to the promise at the top of
this file, decided by Shu on 2026-09-19, so that lease handling, mode entry and
the rate clamp exist once instead of in every consumer. It is behind the
optional `[firmware]` extra (`d1fw-client`), nothing else in the package
imports it, and its tests run against a fake client. Its default transport is
`POST /v1/arm/trajectory/start`: a plan is already fully checked, so uploading
it once puts the timing on the component with a real-time loop and adds a
second, independent guard pass over the whole path. Streaming
`move_joints_both` at 50 Hz stays available for the case that genuinely is a
stream.

### Where the line between the kit and a model runs

Shu, 2026-09-19: 「approach とか少し高次のスキルも manip kit に実装するわけで、
それは agent の中ではなくて、普通に primitive の中に入れる」 and 「agent 的なのは
examples フォルダに切り離す」. So the split runs between *capability* and *one way
of driving it* — and after the 2026-09-19 review the line moved, because three
things had been filed on the wrong side of it:

| in the wheel — capability | in `examples/agent/` — the model |
|---|---|
| `world/` — the perception-result types | `astra_loop.py` — the prompt, the provider client, the scripted stand-in, the message bookkeeping |
| `primitives/` — the verbs, their plans and their measured verifiers | `menu.py` / `jev_menu.py` — the Jev renderer: ranking, the cap, wait/rescan/stop, the question itself |
| `primitives/offer.py` — the IK+guard gate: what can this robot do right now | `mirror.py`, `live.py` — the demo robot and the real one |
| `primitives/schema.py`, `arguments.py` — the canonical argument table, a JSON Schema export, and `decode()` back | `scene.py`, `trace.py` — the shared demo scene and the JSONL decision record |
| `primitives/reach.py` — which hand can do the WHOLE task | |
| `executor.py`, `executors/` — how a plan reaches a robot | |

"Can this arm reach that?", "what may `clearance_m` be?" and "which hand can
deliver?" are questions a script, a teleop assist or a collection macro asks
with no model anywhere near it. The old argument for keeping the schema out —
that it would make every kinematics consumer carry a model dependency — did not
survive being measured: `primitives/schema.py` imports `json` and
`dataclasses`, and `tests/primitives/test_offer_and_schema.py` asserts in a
subprocess that importing the whole boundary pulls in no provider SDK and no
HTTP client. The package still installs as numpy + scipy.

What a model still cannot do is widen the vocabulary: the argument table lives
in the kit, both renderings come out of it, and a drift test compares the kit's
own domains with what is read back out of a generated schema.

```sh
python examples/agent/astra_loop.py --dry-run   # scripted; no API key needed
python examples/agent/jev_menu.py --task "put the red block in the box"
```

### What the simulation runs do and do not show

The design note's Isaac harness (d1-isaaclab #54) is the only end-to-end
evidence this API has, and it is worth being exact about what it is:

* **4 successes out of 5 attempted**, 6 draws total, one unreachable draw
  excluded — and of those, **two `Place` executions measured TRUE**. Two other
  task successes had `Carry` FALSE and `Place` refused.
* The policy was the **scripted** stand-in, not a frontier model: it already
  knew the answer.
* The observations were the simulator's **ground truth**, not a camera.
* The grip force was a single 45 N per-jaw setting rather than the
  soft/firm/strong presets, with observed penetration — an acknowledged
  mismatch with the robot.
* No trial ran on a real D1.

That is a small scripted-policy experiment with a known force mismatch. It is
not a model result, not the planned 20-trial comparison, and not real-robot
validation. Attempted-task success, draw feasibility, complete verified-chain
success and physical-contact incidents are tracked as separate numbers on
purpose, and "AI reliably performs pick-and-place" is not one of them.

## The two guards

**This one** (`manipulation_kit.guard.MotionGuard`) is **in-process, in Python,
stdlib-only.** It exists for planners and IK loops: `arms.GuardedArm.solve_ee`
calls it on every candidate solution, before anything is committed, thousands of
times a second, with no daemon in the path. Asking a REST endpoint per IK
iteration is not a design, and a planner that cannot evaluate a posture without
a robot is not a planner.

**The daemon's** is a Rust port, and it is the last word on the wire: it
re-checks what actually arrives, including commands from processes that never
touched this library. The two agree because the Rust port was validated against
426 golden vectors generated from this one — and keeping them agreeing is
tracked in [`docs/HISTORY.md`](docs/HISTORY.md).

## Tests

```sh
pip install -e '.[dev]' && pytest
```

Everything is a test of a claim somebody once got wrong on a robot; the
docstrings say which. The suite is `pytest`-only and runs on every push
([`.github/workflows/ci.yml`](.github/workflows/ci.yml)), with and without the
`mujoco` extra, including `mkit-urdf build` byte-reproduction and `--check`
against the committed `dist/`.

Tests that OPEN a mesh skip themselves, by name and with the reason, when the
CAD is absent. Set `MKIT_ASSETS_DIR` (or fetch) and they run. Tests that check
the URDFs still NAME the right meshes run unconditionally; those are the ones
that catch a generator which quietly stopped referencing half the robot.

## Versioning

**Bump `project.version` with every change consumers must pick up.** They pin
this repository by commit, and pip decides whether to reinstall by *version* —
a new commit under an unchanged version installs as "Requirement already
satisfied" and changes nothing. `tools/check_version_bump.py` enforces it in
CI. The incident that produced the rule is in
[`docs/HISTORY.md`](docs/HISTORY.md).

## Further reading

[`docs/`](docs) carries the institutional notes verbatim — the description
invariants, the left/right asymmetry that will burn you, the guard's model, the
gesture workflow. [`docs/HISTORY.md`](docs/HISTORY.md) is the migration log:
what this repository was assembled out of, what deliberately did not come, and
which consumers still have to be repointed.
