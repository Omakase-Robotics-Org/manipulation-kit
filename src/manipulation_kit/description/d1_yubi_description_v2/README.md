# d1_yubi_description_v2

Visual (mesh) URDF of the **D1 dual-arm + YUBI hands**, v2 — vendored from Shu's
2026-07 "D1 arm M6-S with yubi" v2 zip. A `dual_base` → `torso_column` (CAD-measured
primitives) with two mirror-mounted **D1 arm M6-S 7-DoF arms** (`Base_{L,R}` →
`Link1..7_{L,R}` → `TCP_Link_{L,R}`, STL meshes) and a **YUBI** parallel-jaw hand on
each TCP flange (`yubi_{L,R}_hand_root` / `hand_cam_link` / `{left,right}_finger_link`).

28 links, 27 joints. All geometry is `package://d1_yubi_description_v2/…` mesh/primitive
visual — this is the **rendering** description.

## vs `description/d1/d1.urdf`
Complementary, not a replacement:
- `d1/d1.urdf` — primitives-only **collision** model (whole robot, zero mesh assets)
  behind the pyguard motion guard.
- `d1_yubi_description_v2` — mesh **visual** model of the dual-arm + hands for RViz /
  `robot_state_publisher` / sim rendering.

## Use
```sh
ros2 launch d1_yubi_description_v2 display_gui.launch.py

ros2 run d1_yubi_description_v2 home_move_publisher --ros-args \
  -p hold_at_zero:=2.0 -p move_duration:=5.0
```

## Provenance — `urdf/d1_yubi.urdf` is GENERATED

```sh
python3 description/d1_yubi_description_v2/tools/assemble_d1_yubi.py
```

**Edit the generator, never `urdf/d1_yubi.urdf`.** It composes the model from
material that already lives in this repository:

- arm chains, STL visuals and CAD inertials read from the single vendor copy,
  `description/d1_arm/{right,left}/d1_arm_{right,left}.urdf`;
- torso column + shoulder mounts (±37 mm, z = 0.50 m) measured from the D1 STEP
  assembly;
- the AIRoA `yubi_description` `yubi_hand` macro reproduced inline, with inertials
  added and the `<mimic>` left finger emitted as an independent joint (Genesis does
  not implement `<mimic>`).

`d1_arm_yubi_description/meshes/d1_arm_{r,l}/` holds this package's copies of the
arm STLs; they are byte-identical to `description/d1_arm/{right,left}/meshes/`
and `devices/omakase_arm/pyguard/tests/test_description_consistency.py` asserts that,
so the two trees cannot silently fork.

The `_L` hand mount is 180° rotated relative to `_R` on purpose — both D1 arms are the
same physical arm. Read `description/d1-arm-notes.md` before changing it.
