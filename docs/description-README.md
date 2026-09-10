# D1 robot description — single source of truth

This directory is the **canonical home of the D1 robot's geometry**. Every
URDF of the D1 or its D1 arm arms used anywhere in the Omakase
stack either lives here or is a **vendored, provenance-tracked export** of
what lives here. Do not hand-copy a URDF out of this directory, and do not
hand-edit a copy in a consumer repository.

## Inventory

| package | content | consumers |
|---|---|---|
| `d1/` | **generated** primitives-only collision models: `d1.urdf` (static head), `d1_wholebody.urdf` (mobile base + lift + neck PTU as joints) and `d1_wholebody_gripper.urdf` (same, wearing the D1 stock parallel gripper instead of the YUBI hand — **the authoritative whole-body asset other repos consume**). The two whole-body files carry the real vendor CAD as `<visual>` (body in `d1/meshes/body/`, gripper in `d1/meshes/gripper/`, arms referenced in place from `d1_arm/`) while keeping primitive `<collision>`; only `d1.urdf` is mesh-free. Regenerate them all with `python3 d1/tools/generate_d1_urdf.py`; **edit the generator, never the .urdf**. | `devices/omakase_arm/pyguard` (motion guard), planners, d1-isaaclab |
| `d1_yubi_description_v2/` | **generated** mesh-bearing dual-arm robot (torso + both arms + YUBI hands, STL visuals). Regenerate with `python3 d1_yubi_description_v2/tools/assemble_d1_yubi.py`; **edit the generator, never the .urdf**. | RViz, d1-manip-sim (Genesis / MuJoCo) |
| `d1_arm/` | **the** vendor per-arm arm package (URDF + STL meshes, left/right) + `yubi_description` hand macro + `*_with_yubi` assembly xacros. Every other model in this directory derives its arm chain from here. | d1-isaaclab (visual/inertial injection), single-arm tools |

`d1_yubi_description_v2/d1_arm_yubi_description/meshes/` is a second copy of the
`d1_arm` arm STLs, kept because ROS resolves `package://` URIs per package.
The copies are byte-identical and the consistency test asserts it, so they cannot fork.

## How consumers reference this directory

There are exactly **two** sanctioned mechanisms, and which one a consumer
should use follows from one question: *does it need the files at RUNTIME, or
only when it builds something it then commits?*

### 1. Resolve d1-sdk by path (`$D1_SDK_DIR`) — for build-time-only consumers

Read the files in place from a d1-sdk checkout, located by `$D1_SDK_DIR` with a
sibling-directory fallback (`dx-simulator-workspace` puts every repo side by
side under `source/`). Nothing is copied, so nothing can go stale.

The cost is real and worth stating: a path-resolved consumer is **not
self-contained**. That is acceptable exactly when only its *generators* need
d1-sdk and the generated artefact is committed — then a bare clone still runs.

| consumer | what it reads | why path resolution is right here |
|---|---|---|
| d1-isaaclab (Isaac Lab RL) | `d1/d1_wholebody.urdf` + `d1/meshes/body/` + `d1_arm/{left,right}/` via `$D1_SDK_DIR` | only `assets/d1/build_d1_urdf.py` and `scripts/convert_usd.py` need them; the built `usd/*.usd` are committed and self-contained (they embed all geometry), so training and evaluation from a bare clone keep working |

### 2. Vendor a provenance-tracked export — for runtime consumers

Simulator repositories that load the URDF and its meshes **every run** cannot
depend on a sibling checkout being present, so they vendor with
`tools/export_description.py`:

```sh
python3 description/tools/export_description.py d1_yubi --dest <consumer>/assets
python3 description/tools/export_description.py d1_yubi --dest <consumer>/assets --check   # CI drift check
```

The export rewrites `package://` mesh URIs to URDF-relative paths (never
absolute machine paths), collects every mesh the URDF references so the copy
actually loads, and writes `PROVENANCE.json` (d1-sdk commit + sha256 per file)
next to the files. The consumer commits both; its tests verify the files still
match the manifest, and `--check` against a d1-sdk checkout detects drift from
this directory. Re-running the export is the only sanctioned way to update a
vendored copy. Here the duplicated bytes are not laziness — they are what makes
the consumer runnable — and the drift check is what stops them forking.

| consumer | what it vendors | how |
|---|---|---|
| d1-manip-sim (Genesis) | `assets/{d1_yubi.urdf,d1_arm_yubi_description/meshes,yubi_description/meshes}` + `assets/PROVENANCE.json` | `export_description.py d1_yubi`; `tests/test_description_provenance.py` fails on a hand-edit and, with `D1_SDK_DIR` set, on drift from this directory. Genesis loads the URDF directly at sim time, so the copy IS the runtime input — there is no generated artefact that could stand in for it. |

Flavours: `d1_yubi`, `d1_collision` (the mesh-free guard model `d1.urdf`
alone), `d1_wholebody_gripper` (the authoritative whole body **with** all 31
meshes collected next to it), `d1_arm`. A test asserts every flavour's
mesh references resolve inside its own export — `d1_collision` used to ship
`d1_wholebody.urdf` too, and once the whole-body files gained real CAD that copy
arrived with all 32 mesh references dangling.

What is **not** sanctioned: a consumer-side copy script with no drift check.
d1-isaaclab had one (`assets/sync_assets.py`, plus an undeclared copy of
`meshes/body/`) and it silently pinned a d1-sdk commit from weeks earlier.

## Invariants

The two arms are the **same physical arm** (not mirror hardware); the wrist
frames land 180° apart under mirror joint values and the YUBI hand mount on
the `_L` tree compensates with a 180° rotation. Read
[`d1-arm-notes.md`](d1-arm-notes.md) before touching anything that
looks like a left/right asymmetry. These invariants — plus "generated files
match the generator" and "every model file agrees on the arm chain and hand
mounts" — are pinned by
`devices/omakase_arm/pyguard/tests/test_description_consistency.py`.

Naming trap: the `_R`-suffix tree is SDK `ArmSide::A` = the robot's
**physical LEFT** arm (+y); `_L` is `ArmSide::B` = physical RIGHT (−y).
