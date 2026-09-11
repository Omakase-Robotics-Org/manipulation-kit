# D1 stock parallel gripper — vendor CAD description

Single source of truth for this gripper's SHAPE. Consumers (d1-sdk,
d1-isaaclab, d1-manip-sim, …) resolve it from the installed package
(`manipulation_kit.hands.d1.parallel_gripper.description_path()` /
`description.load_urdf()`); they compose it onto a wrist, they do **not**
vendor copies of the meshes.

## Source

Vendor ROS package `gripper` v1.0.0 (SolidWorks `sw_urdf_exporter`), received
2026-07-29, staged at `_incoming/gripper-urdf-20260729/`. Its `package.xml`
describes it as *"URDF Description package for gripper (extracted from
x4_26042301)"* — the vendor cut it out of a 6-axis arm + gripper assembly,
which is why the body mesh shipped named after an arm link (see local changes).

SHA256 of the drop as received:

```
0b1c287c6d71e5891c92709eeb409527ea4a37c07fae5733271448ace0630d47  CMakeLists.txt
53070f3cee632fdd679a1d0a2657a2c1e298ec3a691eaf1a3095f9e84fa072a5  config/joint_names_gripper.yaml
ec9c2298a90f6621534a0238158bd474124f63ef085c5db50e5cfa2165d0b0fe  launch/display.launch
17d8d61f5b3cfb8dcd20ef9e7182e74010aedef48e1d240f81ff5a9f360be2c1  launch/gazebo.launch
5b896a665aeef91617531d13c50b7141282f26da2b389f7a18248d031565d9f7  meshes/j6_Link.STL
e71e9c0152fb5b210235965982217fff2f97cccdb4be4c85926210f7dc3a99a0  meshes/tcp_l_Link.STL
6bb88eaa350ead2b40a04dfeba026b246477d09879ecc3dfbc7a5b0d2cdc14de  meshes/tcp_r_Link.STL
125cd059a827db2c39634d82fb6a1f17e8e0ded93ca43b34dedfad7ecc4425d3  package.xml
cb57aec5b16228e2b5d830b2a402f1f3046f6230f5d601fd7527a5c1e9eed0a3  urdf/gripper.urdf
```

The catkin scaffolding (`CMakeLists.txt`, `package.xml`, `launch/`,
`config/joint_names_gripper.yaml`) is **not** vendored: this package is not a
ROS package and the two joint names it lists are already in the URDF.

## What the model says

| | |
|---|---|
| Links | `base_link` (body + rails), `tcp_r_Link`, `tcp_l_Link` |
| Joints | `tcp_r_joint` prismatic 0 … 0.035 m; `tcp_l_joint` prismatic −0.035 … 0, `mimic` of `tcp_r_joint` with multiplier −1 |
| Jaw travel | 35 mm per jaw ⇒ **70 mm** total gap range |
| Frame | flange at `base_link` origin; fingers extend along **+Z**; jaws travel along **±X** |
| Jaw tips | Z = 143.5 mm; registered TCP is Z = 136 mm |
| Envelope | body 57 × 58 × 64 mm (Z 16.5 → 80 mm), rail bar 160 mm across X in the top 18.5 mm |
| Mass (as committed) | **1.5 kg** — measured, see below. The CAD claimed 0.327917 kg |

**Jaw polarity trap.** `q = 0` is the fully **OPEN** 70 mm gap and
`|q| = 0.035` is fully **CLOSED** — the opposite of the CAN 2.0 wire command,
where 0.0 rad is closed and ~1.16 rad is open. Anything wiring a commanded
gripper state to this URDF has to invert.

## Local changes vs the vendor drop

All applied by `tools/vendoring/vendor_gripper_description.py` (repository root); re-run it against a
fresh drop rather than hand-editing the committed files.

1. **Mesh paths** `package://gripper/meshes/…` → `meshes/…`. `package://`
   only resolves inside a ROS workspace; this ships as Python package data.
2. **`j6_Link.STL` → `base_link.STL`.** The body mesh kept the name of the
   *arm* link it was extracted off. A mesh named after a joint this file does
   not contain is a trap, and the link it belongs to is `base_link`.
3. **`velocity="0"` → `velocity="0.05"` on both finger joints.** A zero
   velocity limit is a CAD-export artefact; MoveIt, Drake, Isaac and PyBullet
   all read it as *this joint cannot move* and will refuse to plan or actuate
   the jaws. 0.05 m/s is **not a measured spec** — it is a permissive
   placeholder of the right order (35 mm stroke in well under a second),
   chosen so the limit never binds. Replace it with the real datasheet figure
   when we have one.
4. **Header comment replaced.** The vendor header named `j6_Link.STL` and
   warned *"j6_Link.STL may contain camera geometry that needs manual removal
   via 3D editing tool"*. It does not: the mesh splits into 13 connected
   components — the actuator body (87.3 cm³), the mount plate, the rail bar
   and jaw carriage, and six M3 cap screws — with nothing camera-shaped in it.
   Recorded here so the warning is not lost, and dismissed.
5. **Meshes decimated**, component-aware. `base_link.STL` 112 954 → 23 988
   triangles (5.6 MB → 1.2 MB): the six cap screws (0.035 cm³ each, but
   ~60 000 triangles between them) are dropped, the remaining shells are
   simplified with a per-component budget, and every vertex is clamped back
   inside the original CAD bounding box so the decimated shell can never
   protrude past the real part. Enclosed volume error **0.60 %**, bounding-box
   growth **0.000 mm**. Jaw meshes (2 696 triangles each) are untouched.
   Kinematics and joint limits are untouched by all of this. Same policy as
   `leadshine/dh116s/descriptions`.
6. **`base_link`'s inertial replaced** — the CAD's masses are wrong by 4.6×.
   See below.

## Mass: the CAD said 0.328 kg, the scale says 1.5 kg

Shu weighed the gripper at the robot on 2026-07-29: **1.5 kg**, confirming the
value d1-sdk has registered all along (`defaultGripper()`). The CAD export sums
to 0.327917 kg — light by **1.1721 kg**.

The missing mass has only one place it can be, and this is solved rather than
assumed:

| put 1.1721 kg on… | implied density | verdict |
|---|---|---|
| the two jaws | 15 065 kg/m³ | denser than lead — no |
| `base_link`'s **modelled** 143.2 cm³ | 9 894 kg/m³ | denser than steel — so the *mesh* is incomplete too |
| `base_link`'s bounding envelope, 585.6 cm³ (the mesh fills 24 % of it) | 2 420 kg/m³ | ordinary for a housing holding a motor, gearbox, leadscrew and PCB |

So this is a **shell-only export**: the outside of the gripper, without its
contents. The implied CAD densities say the same thing independently —
1 709 kg/m³ for the body, 998 kg/m³ for the jaws. Neither is a metal, and 998
is water. Those are default/unassigned material values, not a weighed assembly.

What is committed, therefore, is **not** the CAD inertial:

- `base_link` mass = 1.5 − 2 × 0.041574 = **1.416851 kg** (measured minus the
  jaws).
- `base_link` COM z = **66.155 mm**, *solved* so the ASSEMBLY COM lands on the
  registered **68 mm** — the one COM figure with hardware behind it (the wrist
  stopped sagging when the lever moved from 0 to 68 mm). Keeping the CAD's
  34.6 mm would have put the assembly COM at 38 mm, contradicting that by
  30 mm.
- `base_link` inertia tensor = the CAD tensor × 5.7885 (the mass ratio). A
  first-order estimate; where exactly the un-modelled internals sit is unknown.
- The **jaw** inertials are the CAD's, verbatim. At 41.6 g each they are 5.5 %
  of the total, so their default-material density barely moves the assembly.

The `<inertial>` element in the file is labelled as an estimate and quotes the
CAD originals beside it, and `../toolconfig.py` keeps `CAD_MASS_KG` /
`CAD_COM_MM` as a record of the known-wrong figures so a future CAD drop can be
compared against them.

**Distrust the next CAD drop too.** The DH116S is registered at 0.359 kg and
weighs ~380 g on the same scale: +6 %, which is what mounting hardware looks
like. +358 % is not mounting hardware. CAD exports in this pipeline under-report
mass; weigh the thing.

## Camera plate + wrist camera (`gripper_with_camera.urdf`)

The gripper bolts to the flange through a two-plate stack that the vendor
gripper CAD leaves as an EMPTY 16.5 mm gap (the body mesh starts at
z = 16.5 mm — that is where the plates live, and why the registered 136 mm
TCP needs **no change** when they are modelled). `gripper_with_camera.urdf`
is `gripper.urdf` plus the arm-end plate and the wrist camera, generated by
`tools/vendoring/vendor_camera_plate.py` (repository root); re-run it rather than hand-editing.

Source: `夹爪连接板（手臂端）V2.0.stl` (arm-end connection plate V2.0,
SolidWorks binary STL in mm, received 2026-08-21), SHA256
`3d2ba4b71da48057b21ed86cf8cee6574ce8aa4a8d979ee7aaaf36ac00caea63`,
filed on the Drive under `D1/`. Converted to metres verbatim (7 470
triangles, 365 KB — no decimation needed).

| | |
|---|---|
| `camera_plate` | the plate mesh at identity: its CAD origin IS the flange centre, disc (Ø72 × 8 mm, 12-hole Ø24.9 bolt circle) on the flange plane, camera arm along **+Y** rising to z = 18.6 mm at y ≈ 100 mm |
| `wrist_camera` | camera MOUNT face: centre of the 4-hole pattern at `(0, 79.24, 14.54) mm`, rpy `(+15°, 0, 0)` — local +Z is the face normal, pitched 15° toward the fingers; +Y up the arm |
| `wrist_camera_optical` | ROS optical (+Z forward, +X image right, +Y image down) = mount rotated π about Z. Fixed by the real wrist streams: the jaws sit at the BOTTOM edge of the image |
| mass | plate **78 g** (mesh volume 28.95 cm³ at aluminium 2 700 kg/m³ — assumed density, weigh it); camera **30 g placeholder** |

**Per-arm clocking is not encoded here.** The plate makes the assembly
chiral; which way +Y points on the robot is robot⊕hand composition. Measured
2026-08-21 from the d1 teleop dataset (FK at a grasp frame cross-checked
against the head-camera view): the camera sits on TOP of the wrist on both
arms — the physical LEFT arm (SDK "_R" tree) mounts this file with yaw = π
about the flange Z, the physical RIGHT ("_L") with yaw = 0. Exported as
`description.CAMERA_ARM_YAW_RAD`.

Still placeholder / not modelled in the camera variant:

- The **camera module box (30 × 30 × 16 mm) and its 30 g** — the real unit is
  the wide-angle UVC camera already on d1-2 (no nameplate); measure it and
  update `vendor_camera_plate.py`. The lens offset from the mount face is
  unknown, so the optical origin sits ON the face.
- The **gripper-end plate (夹爪端)** filling the remaining 8.5 mm of the
  flange gap — separate hardware, no CAD drop yet.
- The registered tool config (1.5 kg, COM 68 mm) does NOT include the plate
  and camera (~108 g at ~25 mm). Registering the combined tool is a
  controller-facing decision left to a separate change.

## Not modelled

- No wrist camera, adapter plate, pigtail or cable loom in `gripper.urdf` —
  the vendor drop is the gripper alone (the camera variant above models the
  arm-end plate + camera). Whatever else hangs off the flange on the real
  robot is part of the mounted-tool mass but not of this shape.
- No internal components. The 1.17 kg of motor / gearbox / leadscrew / PCB
  that the mass correction accounts for has no geometry in this file; it is
  inertia only, inside the housing envelope.
- No `<transmission>`, no gazebo tags, no collision simplification: the
  collision geometry is the same (decimated) visual mesh the vendor shipped.
  A convex decomposition can be added if a sim needs one.
