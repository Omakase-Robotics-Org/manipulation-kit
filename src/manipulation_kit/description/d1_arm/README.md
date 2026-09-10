# D1 arm arm descriptions

Authoritative URDF descriptions for the **D1 arm** 7-DoF arm used on the D1
robot. These are the **vendor real-robot descriptions** — they match the physical
hardware and are the source of truth for sim/real parity.

- **Source:** vendor "D1 arm D1 arm" URDF export (SolidWorks → URDF Exporter).
- **Authoritative:** use these descriptions for sim and real; they are kinematically
  consistent with the deployed robot.

## Packages

colcon-discoverable `ament_cmake` packages:

| Package | Path | Description |
|---|---|---|
| `d1_arm_right` | `right/` | Right arm URDF `d1_arm_right.urdf` + `*_with_yubi.urdf.xacro` |
| `d1_arm_left`  | `left/`  | Left arm URDF `d1_arm_left.urdf` + `*_with_yubi.urdf.xacro`  |
| `yubi_description`       | `yubi_description/` | Vendored YUBI parallel-jaw hand description (Toyota, Apache-2.0) |

Mesh references use the ROS package URI scheme, e.g.
`package://d1_arm_right/meshes/Link1_R.STL`. Each package installs its URDF
and `meshes/` directory under `share/<package>/` so the `package://` paths resolve
after a `colcon build` + source of the install space.

### Arm + YUBI hand (`*_with_yubi.urdf.xacro`)

Each arm package also ships a `d1_arm_<side>_with_yubi.urdf.xacro` that
attaches the YUBI parallel-jaw hand to the arm's TCP flange (`TCP_Link_<S>`). It
`xacro:include`s the in-package arm URDF and the `yubi_description` hand macro, then
fixes the hand to the flange with the **vendor straight-mount** transform
`xyz = 0 -0.019 0.055`, `rpy = (90°, -90°, 0)` (from the 2026-06 vendor
"D1 arm M6-S with yubi" update). Expand with:

```sh
xacro right/d1_arm_right_with_yubi.urdf.xacro > /tmp/right_with_yubi.urdf
```

Naming note: the upstream vendor package was `d1_arm_yubi_description` with meshes
under `meshes/d1_arm_{r,l}/`. We do **not** re-add that package — the bare-arm URDFs
and meshes already in `d1_arm_{right,left}` are **byte-identical** to the
vendor update (verified), so we only add the new yubi-attachment xacro to each
existing package and vendor `yubi_description` for the hand geometry. This keeps the
clean `d1_arm_*` package names already established in this repo and avoids
duplicating the arm geometry.

## Left vs Right

The left and right arms are **kinematically identical** — the descriptions differ only
in naming (link/joint suffix `_L` vs `_R`, robot `name`, package name, and mesh
filenames). Joint origins, axes, limits, and inertials are the same.

## Kinematics notes

- 7 revolute joints (`Joint1..Joint7`) from `Base` → `Link1` → ... → `Link7`.
- TCP frame: `JointTCP` is a **fixed** joint `Link7 → TCP_Link` with
  `xyz = 0 -0.087 0`, `rpy = 1.5708 -1.5708 0`.

## Build

```sh
# from a colcon workspace whose src/ contains these packages
colcon build --packages-select d1_arm_left d1_arm_right
```
