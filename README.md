# manipulation-kit

**Everything a D1 humanoid needs to *think* about manipulation, with no
hardware in it.** Inverse kinematics, the robot description and its generator,
the in-process motion guard, end-effector tool configs and the retargeting math.
Nothing in this repository opens a socket, holds a robot lock, or moves a joint.

Hardware is a separate layer — the `d1-firmwared` daemon, REST on `:4750`. That
split is the point of this repo: a customer doing **second development**
receives a D1 with the firmware pre-flashed, and this is the layer they read,
extend and plan against.

> **Licensing:** there is deliberately **no `LICENSE` file yet** — what we
> grant on our own code is still an open decision. What is no longer open is
> the CAD: **this repository ships the URDFs and not the geometry they
> reference.** Read **[LICENSE-STATUS.md](LICENSE-STATUS.md)** before
> publishing or sharing anything from here.

## Install

```sh
pip install -e .            # everything below: numpy + scipy, nothing else
pip install -e '.[dev]'     # + pytest, for the test suite
pip install -e '.[mujoco]'  # + the OPTIONAL alternative IK substrate
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
  guard/         MotionGuard — stdlib-only joint limits + torso keep-out +
                 self-collision over the primitives-only whole-body URDF
  hands/         "<maker>/<model>" identity: tool configs, CAD descriptions,
                 glove->hand retarget maps
  description/   the D1 URDF family, its generator, the exporter, the assets
  config/        the exported JSON a controller consumes
contrib/         research, not installed (whole-body IK: base + lift + neck)
examples/        runnable scripts: gesture generation, preview, click-to-move IK
tools/vendoring/ CAD re-import; needs the private assets repo, not installed
dist/            prebuilt, provenance-tracked exports
docs/            the institutional notes, verbatim
```

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
