# D1 full-body collision description

`d1.urdf` is a single URDF of the **whole D1 robot** — chassis + lift +
torso + head + both D1 arm 7-DoF arms + an end effector —
whose collision geometry is **simple primitives only** (boxes, cylinders,
spheres; no meshes).

## The three files

| file | base + lift | end effector | visual geometry |
|---|---|---|---|
| `d1.urdf` | fixed at `dual_base` | `omakase/yubi` | **none** — primitives only |
| `d1_wholebody.urdf` | planar base + lift + neck as JOINTS | `omakase/yubi` | real CAD |
| `d1_wholebody_gripper.urdf` | same as above | `d1/parallel_gripper` | real CAD |

**Visual and collision are deliberately different answers.** Collision is
primitives in all three: it is what the motion guard checks, what the C++
validators mirror, and what a simulator wants for fast, stable contact. Visual
is the real vendor CAD in the two whole-body files, because "what does the robot
look like" is a different question from "what should it collide with". A test
enforces that no `<collision>` in this family is ever a mesh.

`d1.urdf` is the one file with **no meshes at all**, and that is on purpose, not
a leftover: pyguard parses it directly, it has to load on the robot and in bare
planners with zero assets, and `test_guard_model_stays_mesh_free` keeps it that
way. It is the lightweight variant — it is not the "primitive-looking" model
that got retired.

`d1.urdf` is the model behind the software motion guard, so its end effector
is **deliberately** left as the YUBI hand: pyguard, the C++ validators and the
trained policies were all built against those primitives, and swapping them is
a safety-behaviour change, not a regeneration.

The end effector is addressed by dx-manipulator's `"<maker>/<model>"` id —
the same id `manipulation_kit.hands.get_tool_config` and d1-isaaclab's
`get_end_effector` take. **dx-manipulator owns the hand** (identity, physical
data, vendor CAD under `hands/<maker>/<model>/descriptions/`); this repo owns
the **composition** — the flange transform, the collision approximation, and
the robot⊕hand assembly other repositories consume.

## Consuming `d1_wholebody_gripper.urdf` from another repository

This file is the **authoritative whole-body D1 wearing the stock gripper**.
d1-isaaclab and anything else should reference it rather than composing their
own. What a consumer needs to know:

| | |
|---|---|
| Path | `description/d1/d1_wholebody_gripper.urdf` |
| Generated? | Yes — by `description/d1/tools/generate_d1_urdf.py`. Never hand-edit it; a test fails if it drifts from the generator |
| Root link | `world`, **at floor level**. Spawn at z = 0. At `lift = 0`, `dual_base` is 0.513 m above the floor — see **The AMR cover offset** below |
| Actuated | `base_x`, `base_y`, `base_yaw` (planar; the real chassis is differential-drive, so command it as (v, ω) — see `wholebody/`), `lift` (0…0.300 m), `neck_pan`, `neck_tilt`, `Joint1..7_{R,L}`, `gripper_{R,L}_tcp_{r,l}_joint` |
| Mesh paths | **relative to the URDF file** — `meshes/body/base_link.STL`, `meshes/gripper/base_link.STL`, `../d1_arm/right/meshes/Link3_R.STL`. Never `package://`, never absolute, never cwd-relative, so it resolves outside ROS from any working directory |
| Meshes present | **the whole robot**, as `<visual>`: 34 refs — body CAD (10), both arms (18), gripper (6). Arm and YUBI meshes are referenced IN PLACE from their own packages via `../`, so nothing is duplicated; the body CAD lives in `meshes/body/` |
| Cameras | `head_camera_link` + `head_camera_optical_frame`; the YUBI variant also has `yubi_{R,L}_hand_cam_optical_frame`, the gripper variant `gripper_{R,L}_wrist_cam_optical_frame` on the arm-end camera plate. See **Camera frames** below — they are nominal, not calibrated |
| To copy it out | `python3 description/tools/export_description.py d1_wholebody_gripper --dest <consumer>` — it collects all 31 meshes and rewrites the paths. Hand-copying only `description/d1/` leaves the `../d1_arm/...` arm refs dangling, and hand-copying is not a sanctioned mechanism anyway (see `description/README.md`) |
| Collision | Primitives everywhere, including the gripper — so a collision-only consumer needs no mesh assets at all |
| Mass | 1.5 kg per gripper, **measured**; equals `ToolConfig::defaultGripper()`, so the URDF and the arm controller describe one object |
| Handedness | `_R` = SDK `ArmSide::A` = the **physical LEFT** arm. Zero pose is a T-pose |
| End-effector variants | `omakase/yubi` and `d1/parallel_gripper`; add one in the generator's `END_EFFECTORS` registry |

The gripper STLs under `meshes/gripper/` are **byte-identical vendored copies**
of dx-manipulator `hands/d1/parallel_gripper/descriptions/meshes/`, which is the
single source of truth — the same vendored-copy-plus-byte-identity-test pattern
the arm meshes already use between `d1_arm` and
`d1_yubi_description_v2`. A test enforces it when a dx-manipulator checkout sits
beside this one (or `$DX_MANIPULATOR_DIR` points at one). Do not edit the copies.

Its purpose is **collision checking**, not rendering:

- it is the model behind the software motion guard
  `devices/omakase_arm/pyguard` (joint-limit clamp + torso keep-out +
  self-collision check of commanded joint vectors);
- it loads anywhere (sim, planner, bare host) with zero mesh assets —
  verified to load in Genesis with FK agreement < 1e-6 m vs the pyguard FK.

Only `d1.urdf` is mesh-free (test-enforced). The two whole-body files carry the
real vendor CAD as `<visual>` while keeping primitive `<collision>`.

## Regenerating

The URDFs are generated — **edit the generator, not the files**:

```sh
python3 description/d1/tools/generate_d1_urdf.py            # all three
python3 description/d1/tools/generate_d1_urdf.py --only d1.urdf
```

The generator parses its own output before writing it: a URDF with a `--`
inside an XML comment is unreadable to every parser, and that has shipped from
a generator in this codebase before.

## Frame

Root link `dual_base`: **z up, +x forward, +y robot's left**, shoulders at
`y = ±0.037 m, z = 0.50 m` — identical to the `mount_R` / `mount_L` joints of
`d1_yubi_description_v2/urdf/d1_yubi.urdf` (which is what d1-manip-sim loads),
omakaseos `d1_dual_description`, `config/safety_zones.json`, and the C++
validators `safety_zones.h` / `collision_model.h`. Keep them in sync.
(d1-manip-sim's `assets/d1_dual/d1_dual.urdf` used to be named here; it
was deleted when that repo stopped keeping a private copy of the geometry.)

### The AMR cover offset

`dual_base` sits **0.513 m** above the floor at `lift = 0`, not the 0.484 m
the CAD chain was calibrated to. The built robot's AMR cover is **29 mm**
taller than any CAD in this repository models it, so everything the lift
carries — column, torso, both arms, neck, head — starts that much higher.

The correction is applied **once, at the lift joint origin**
(`base_footprint → dual_base`). Nothing inside the `dual_base` frame moves:
the shoulders stay at `z = 0.50`, the neck at `0.621`, the torso cameras and
keep-out boxes at their CAD heights, and the lift's 0…0.300 m travel is the
actuator's own. Every consumer that works shoulder-relative — the motion
guard, `config/safety_zones.json`, the frozen `safety_zones.h`, all IK — is
bit-for-bit unaffected; only the robot's height above the floor changed.

The one thing that did move with the torso is decorative: the moving lift
sleeve is a visual cover authored in the torso frame, so it rose 29 mm off the
chassis mast boot and was extended 29 mm downward to stay over it — height
116 → 145 mm, torso-local Z 50…195 mm, lower lip back at world Z 563 mm with
11.7 mm of clearance over the boot; no collision or inertial geometry involved.
A longer, full-stroke variant (Z −275…195 mm, 470 mm) that kept the lip
permanently inside the AMR shell was tried on 2026-09-16 and rejected by Shu
the next day as a visual regression — it hides the telescoping motion — so do
not lengthen the sleeve again. See `docs/ROBOT_BACK_VISUALS.md`.

**Measured** by Shu with a tape on d1-3, 2026-09-16, at `lift = 0`,
floor-referenced, ±2 mm. `132.2 cm` where the spec says `129.3 cm`, and the
3 cm appears at the top of the AMR:

| frame (floor-referenced) | URDF before | measured | URDF now | residual |
|---|---|---|---|---|
| torso body bottom | 660 mm | 700 mm | 689 mm | −11 mm |
| torso top | 1080.6 mm | 1120 mm | 1109.6 mm | −10 mm |
| head RealSense (`head_camera_link`) | 1233.7 mm | 1266 mm | 1262.7 mm | −3 mm |
| head top | 1293 mm | 1322 mm | 1318.9 mm | −3 mm |

Flat residuals across a 660 mm span are what a single offset at the base looks
like. The rejected alternative — a +53.83 mm extension of the moving column,
fitted to one gripper-clearance reading on 2026-09-13 — gives 714 / 1134 /
1287 / 1347 mm, i.e. +14…+27 mm and growing with height. That earlier reading
and this body measurement disagree by ~25 mm; four frames beat one, so the
body measurement is what the description is fitted to.
`tests/test_lift_origin_amr_cover_offset.py` pins all of it.

The chassis boxes are unchanged: they are authored floor-relative and model
the CAD cover, whose keep-out top (0.4987 m) still sits above the 0.460 m deck
the tape found — conservative, which is all a keep-out has to be.

The published **1.293–1.593 m height range is the CAD figure and is stale**:
with the offset the head top sweeps 1.322–1.622 m, which is what the tape
reads.

## Where the numbers come from

| What | Source |
|---|---|
| Arm joint origins / axes / limits | vendor URDFs `description/d1_arm/{right,left}` (D1 arm D1 arm) |
| Arm mounts (±0.037 m, z 0.50, roll ∓90°) | measured from the D1 STEP assembly (d1-face/extract_arm_mounts.py); carried by `d1_yubi_description_v2/urdf/d1_yubi.urdf` |
| Link capsule radii | `config/safety_zones.json` (conservative mesh bounds; same as `collision_model.h`) |
| YUBI mounts + palm/camera geometry | d1-manip-sim `assets/d1_yubi.urdf` (merged main) / `yubi_description` xacro |
| YUBI finger boxes | bounding boxes of the `yubi_description` finger collision STLs |
| Torso / chassis boxes | **measured 2026-07-01 from the D1 CAD STEP** "omakase D1 assy PKG 260607" (`~/Downloads/cad/omakase_D1_assy_PKG_260607.stp`, byte-identical to `d1-face/d1_face.step`): subtree extraction + OCP/XCAF located bounding boxes + z-band slicing of the body shell |
| Floor height, lift travel, neck PTU, head box | **vendor body URDF `urdf2026072302`** (SolidWorks export 2026-07-23; Drive `D1/URDF/urdf2026072302.zip`) |
| Lift zero (`dual_base` 0.513 m above the floor) | vendor chain **plus a measured +29 mm** — the built robot's AMR cover is taller than the CAD. Tape on d1-3, Shu 2026-09-16, ±2 mm; see **The AMR cover offset** |
| Gripper mount, jaw joints, boxes, masses | **vendor CAD** in dx-manipulator `hands/d1/parallel_gripper/descriptions/gripper.urdf` (received 2026-07-29) — see the GRIPPER block in the generator |

### The 16.5 mm void at the flange — half real now, half still to ask for

The void is a two-plate stack. The **arm-end half is now real CAD**: the
camera plate V2.0 (夹爪連接板 手臂端, received 2026-08-21; Drive `D1/URDF/`)
is an 8 mm aluminium disc filling z = 0 … 8 mm, vendored through
dx-manipulator and drawn/collided on its own `gripper_<S>_camera_plate` link
— it also carries the wrist camera (see below).

**Still ask the vendor for: the gripper-end plate (夹爪端) spanning the
remaining z = 8 … 16.5 mm — as STL or STEP, with its mass.** Also worth asking
whether `j6_Link.STL` was exported as the *arm's* J6 link of the `x4_26042301`
assembly, in which case that band is the x4 arm's flange boss and we need to
know what replaces it on the D1 arm.

Why: the vendor mesh has **zero vertices below z = 16.5 mm**. It begins at the
gripper's own mount plate (a 57 × 57 mm plate spanning 16.5 → 23 mm) and models
nothing reaching back to the arm. `Link7`'s mesh ends exactly at the flange face
(z = 0.00 mm), so 16.5 mm of the assembly is simply absent from the export —
which shows up as a visible gap between the arm and the gripper in any render.

The frames are **not** wrong. `gripper_<S>_base_link`'s origin sits at 0.00 mm
from the flange frame on both arms, and the 16.5 mm is real hardware:

**MEASURED, 2026-09-16** (Shu, d1-3, callipers), along the tool axis outward
from the Marvin arm flange face — this supersedes every CAD figure below:

| band | measured |
|---|---|
| camera mounting plate | 0 … 2 mm |
| spacer block | 2 … 9 mm |
| gripper body | 9 … 51 mm |
| finger base plate | 51 … 71 mm |
| pads (graspable depth 58 mm) | 71 … 129 mm |
| pad centre — where the jaw joints hang | 100.0 mm |
| pad tip — the registered TCP | 129.0 mm |
| maximum opening, pad face to pad face | 64 mm |

| superseded | was | measured | error |
|---|---|---|---|
| registered TCP | 136.0 mm | 129.0 mm | +7.0 mm |
| jaw joint origin (pad centre) | 108.47 mm | 100.0 mm | +8.5 mm |
| CAD jaw tips | 143.5 mm | 129.0 mm | +14.5 mm |
| jaw opening | 70 mm | 64 mm | +6 mm |
| camera plate thickness | 8 mm | 2 mm | +6 mm |

The 136 mm was carried over from `d1-sdk`'s hardcoded `defaultGripper()` and
"validated" against the CAD's own 143.5 mm jaw tips and the YUBI jaw tips at
148.4 mm — three numbers, none of them a calliper on this gripper. With the
pads measured, the gripper tip is **19.4 mm shorter** than the YUBI tips: the
two end effectors are NOT interchangeable at one tool config, which the CAD
numbers made them look like.

The measured stack also puts only **9 mm** of hardware in front of the flange
where the CAD mesh leaves a 16.5 mm void, so the CAD void is 7.5 mm too deep —
the same direction and order as its 8.5 mm-long pad centre. The band is now
filled by the 2 mm camera plate plus `gripper_<S>_gripper_spacer`, a
57 × 57 × 7 mm box whose thickness is measured and whose footprint still
**bounds** the candidate shapes (a round collar up to 57 mm, or a square
plate). It carries no mass — the missing 1.17 kg is already on the link's
inertial. Left as a void this band was a real hole: a volume the motion guard
could not see (the arm capsules are only 30 mm in radius, while the
mount-plate corners reach 40.3 mm). Camera plate + spacer together tile the
axis from the flange face outward with no gap, pinned by
`test_no_unmodelled_void_between_the_flange_and_the_gripper`.

Two residuals are NOT resolved by the sketch and are deliberately visible:
the jaw MESHES still reach 6 mm past the measured pad tip, and
`camera_plate.STL` still models an 8 mm disc. Meshes are vendor CAD; only a
new CAD drop replaces one. Constants, joint origins and collision primitives
follow the calliper, so planning is right and rendering is ~6 mm generous.

### The gripper, and what is still unknown about it

`TCP_Link_<S>` is the arm's tool flange (the vendor D1 arm URDF gives it zero
mass and an empty mesh — a pure frame), and the gripper's `base_link` origin
IS its own mounting flange, so the mount transform is a **pure rotation, zero
translation**. The registered tool config puts the TCP on the MEASURED pad tip,
129 mm along flange +z.

Handedness: `TCP_Link_R` has +y DOWN and `TCP_Link_L` has +y UP, so mounting
the same part identically on both flanges would put it upside down on one arm.
`_R` therefore carries a half turn about the approach axis and `_L` does not,
which lands the gripper's +y up and the jaws travelling fore/aft on both.
(`_R` is the vendor unit on the PHYSICAL LEFT arm.) Tested, not assumed.

Jaw polarity: `q = 0` is the fully **OPEN** 64 mm gap (measured) and
`|q| = 0.032` is **CLOSED** — the opposite of the CAN 2.0 wire command, where
0.0 is closed.

Not modelled, and it matters if you use this file as a keep-out volume:

- ~~no wrist camera~~ **carried since 2026-08-21**: the arm-end camera plate
  V2.0 holds the wide-angle UVC module 79 mm up the gripper's +y ("up" on both
  arms), its mount face pitched 15° toward the fingers.
  `gripper_<S>_wrist_cam_link` follows the family lens-along-+x convention and
  gets the standard `gripper_<S>_wrist_cam_optical_frame`. The frames were
  cross-checked against the robot: FK of the d1 teleop dataset at a grasp
  frame puts the camera exactly where the head camera sees the real bracket,
  and the real wrist streams show the jaws at the bottom edge of the image.
  Still placeholder: the module housing (16 × 30 × 30 mm box) and the lens
  offset within it — the unit on d1-2 has no nameplate; the optical origin
  sits ON the plate's mount face. The plate is 78 g at aluminium book density
  (tensor exact from the mesh, part not yet weighed), the module a 30 g box;
  neither is in the weighed 1.5 kg nor the registered tool config —
  registering the extra ~108 g is a controller-facing decision not taken here.
- **gripper-end plate still missing.** The camera plate covers z = 0 … 8 mm of
  the CAD's 16.5 mm clearance; the rest is the assumed box above.
- No internal components. Shu weighed the gripper at **1.5 kg** (2026-07-29),
  confirming `ToolConfig::defaultGripper()`; the vendor CAD claimed 0.3279 kg
  because it is a shell-only export. The URDF carries the measured 1.5 kg, with
  the missing 1.1721 kg of motor / gearbox / leadscrew / PCB on `base_link` as
  inertia with no geometry, and its COM solved so the assembly COM lands on the
  validated 68 mm. The **total is measured; the per-link split is an
  estimate.** Derivation: dx-manipulator
  `hands/d1/parallel_gripper/{toolconfig.py,descriptions/README.md}`.
  Treat CAD inertials in this pipeline as suspect: the DH116S spec'd 0.359 kg
  and weighed 380 g, so it is now registered at the weighed figure, and that
  +6 % is what mounting hardware looks like — +358 % is an export that left
  the housing contents out.

Arm collision capsules are emitted as cylinder + two end-sphere elements
(a true capsule union); collision element names ending in `_exempt` mark
volumes the arms legitimately pass through (the shoulder-cover shell band) —
the guard skips them.

## Camera frames

The two whole-body variants carry camera frames. `d1.urdf` deliberately does
**not**: it is the guard model, pyguard enumerates its links to check keep-out
volumes, and a camera frame is not a volume.

| frame | in | parent |
|---|---|---|
| `head_camera_link` | both whole-body files | `head_link` (the neck tilt link) |
| `head_camera_optical_frame` | both | `head_camera_link` |
| `yubi_{R,L}_hand_cam_optical_frame` | `d1_wholebody.urdf` only | `yubi_{R,L}_hand_cam_link` |
| `gripper_{R,L}_wrist_cam_optical_frame` | `d1_wholebody_gripper.urdf` only | `gripper_{R,L}_wrist_cam_link` (on the arm-end camera plate) |

**Why the optical frames exist.** A camera link says where the camera *body*
sits; it does not say which axis looks out of the lens, and guessing is silent.
d1-isaaclab mounted renderers straight on `yubi_<S>_hand_cam_link` with the ROS
convention (`optical axis = +z`) and got two pictures of the ceiling and one of
the inside of the head shell — the policy scored a believable 0/4 and nothing
raised. Both D1 camera families point their lens along their link's **+x**, so
each `*_optical_frame` is that link rotated by `rpy = (0, π/2, 0)`, which lands
the standard ROS optical convention: **+z out of the lens, +x image right, +y
image down**. Mount a renderer on the optical frame with `convention="ros"` and
no rotation is needed anywhere downstream.

**Where the numbers come from — measured, not chosen.**

- *Head.* The vendor `headcamera_Link` is not a lens frame: `headcamerjoint` is
  identity, so it is just the mesh sharing `uphead_Link`'s frame. The direction
  is read off the **housing**: in `head_link` coordinates it is a
  27.4 × 25.7 × 89.7 mm bar — 90 mm wide *across* the robot (`head_link` +z is
  `dual_base` +y) and only 27 mm deep — and a bar like that faces along its
  shallow axis. `head_camera_link` sits at the centre of that +x face, **solved
  from the mesh at generation time**, so a vendor mesh revision moves the frame
  with it. That puts it 1.2627 m above the floor with the lift retracted —
  3 mm under the 1.266 m the tape read on d1-3 (2026-09-16). The *tilt* cannot
  come from that mesh — an axis-aligned bar's AABB carries no rotation — so it
  is the **design value from the head-part CAD section**: the D435 mounting
  face is 15° below horizontal, aiming the field of view down at the
  workspace (Shu, 2026-09-17). That supersedes the 17.25° previously read off
  the D435 slab normal (0.955, 0.005, −0.297) in the 2026-08-23 full-robot
  CAD. **It is a property of the head PART, not of the D1**, so it is a named
  hardware revision rather than a literal — `rev1` (15°) is what d1-1, d1-2
  and d1-3 wear and what the committed URDFs describe; `rev2` (20°) is the
  part the next units are built with (Shu, 2026-09-20). See
  `manipulation_kit.description.HEAD_CAMERA_TILT_DEG` and
  `mkit-urdf build --hardware-revision`.
- *Wrists (YUBI).* The YUBI camera housing is a 35 × 32 × 42 mm box centred at
  (−0.0175, 0, 0) — it extends *backwards* along −x, so the link origin plane
  already is the lens face — and the fingers reach +x (tips at x = +0.109). The
  lens looks down the approach axis, which is what a wrist camera is for.
- *Wrists (gripper).* The lens direction is the arm-end camera plate's own
  mount face: a machined plane pitched exactly 15° from the approach axis
  toward the fingers, read off the plate CAD (dx-manipulator
  `vendor_camera_plate.py` re-derives it from the mesh on every run). The
  180° roll IS resolved for this one: the real wrist streams show the jaws at
  the bottom edge of the image, which fixes image-down toward the fingers —
  cross-checked 2026-08-21 by FK of the d1 teleop dataset at a grasp frame
  against the head camera's view of the physical bracket.

**What is still uncalibrated.** These are *nominal* frames from vendor
geometry. Three things are genuinely unknown and a consumer must not read them as
a calibrated extrinsic:

1. the **lens position within the housing**. The head housing is a 90 mm
   multi-sensor bar (an Intel D435 is 90 × 25 × 25 mm) so the colour lens is a
   centimetre or two off the housing centre, and the CAD does not say which
   sensor is which. Right for "is the camera looking at the workspace", not for
   pixel-accurate work.
2. the **180° roll about the lens axis** — which way is up in the image. The
   axis *assignment* is measured (optical x along the housing's wide transverse
   axis, optical y along the narrow one); the *sign* is not, and the D1 stack is
   already known to need a 180° rotation on the real head-camera stream
   somewhere. Do not trust image-space left/right from these frames until they
   are checked against a real frame.
3. the **pitch on a particular robot**. 15° is what the `rev1` part is
   designed to;
   the ArUco calibration on d1-3 (`d1-inference`
   `calibrate_head_aruco.py`, session `d1-3-tokyo-20260916`, neck sweep) reads
   the optical axis 12.2–12.8° below `head_link` forward with −1.3…−1.9° of
   lateral yaw. The ~2.5° gap is the neck-tilt zero and gravity sag, not the
   mount. A consumer that needs the real extrinsic reads the per-robot
   `cameras_<robot>.json` (an absolute `head_link` → camera transform); this
   nominal exists to seed those calibration sweeps and to render an
   uncalibrated robot.

**A consequence worth knowing before you use the head camera.** At the parked
neck pose it looks straight ahead, pitched **15° below horizontal** — and that
is not enough to see the near workspace. A tabletop 0.36 m in front of the
robot at 0.885 m sits ~48° below horizontal, so it is still ~33° below the
optical axis, i.e. outside the frame. That is a fact about the robot: seeing a
close work surface is what `neck_tilt` is for. A sim that teleports the camera
downwards instead of tilting the neck is modelling a robot that does not exist.

Four tests pin all of this (`test_description_consistency.py`):
`test_camera_frames_exist_only_where_they_should`,
`test_optical_frames_put_the_lens_on_plus_z`,
`test_head_camera_is_the_front_face_of_the_vendor_housing` (which re-reads the
STL and re-derives the frame independently of the generator), and
`test_camera_frames_are_massless`.

**Note for URDF importers:** these are massless, geometry-less frames. An
importer that merges fixed joints — Isaac Lab's does by default — can collapse
them away, so a consumer that needs the prim must keep fixed joints or read the
transform from the URDF.

`d1_yubi_description_v2/urdf/d1_yubi.urdf` also has `yubi_<S>_hand_cam_link` and
has **not** been given optical frames; add them in its generator if a consumer
needs them.

## Frame heights (vendor `urdf2026072302`)

The floor is at `dual_base` z = −0.513 with the lift retracted — see
**The AMR cover offset** for where that number comes from. The vendor chain
alone puts it at −0.52678: axle at base_link −0.18517, mesh radius 0.0853
⇒ base_link 0.27047 above the floor, plus the 0.28831 sliderjoint origin
and the 32 mm below, cross-checked against the `base_link` mesh, which bottoms
out at ground z = +0.0002. The two are 14 mm apart; the tape decides.

`dual_base` itself is 32 mm below the vendor `slider_Link` frame: the vendor
arm plates sit at slider (−0.0016171, ±0.037, 0.468) with roll ∓90°, and we
put the same arm bases at (0, ±0.037, 0.50) with the same roll signs — the
±0.037 half-width and both roll signs agree with the 2026-07-01 CAD
measurement, so equating the two frames fixes the offset.

**Vendor side naming is the opposite of ours**: vendor `rarmbase` is at
y = −0.037 = the robot's physical RIGHT, which is our `_L` tree.

## Known gaps (TODO: measure)

- **Neck axis signs**: the vendor CSV contradicts the vendor URDF on
  `upheadjoint` and `sliderjoint`. The URDF is taken as authoritative
  (lift: +z = up). Confirm both on hardware.
- **Head keep-out is the parked pose**, not the swept PTU volume. The swept
  box `(-0.1049, -0.1415, 0.5925)…(0.1431, 0.1415, 0.8105)` rejects arm
  poses the guard accepts today, so adopting it is a separate safety review.
- **Vendor inertias are unusable** (`lidar_Link`: 0.4 kg with Ixx = 12.7 and
  its CoM 0.55 m off-link; both arm plates carry the same copy-pasted
  tensor) and `rarmbase` / `larmbase` / `lidar` ship as 80-byte STLs with
  zero triangles. Only kinematics and the head/neck meshes were taken.
- **Head height disagrees with the June CAD** by 70 mm (vendor top 0.8034
  vs CAD 0.873 above `dual_base`); x/y agree within 13 mm. Consistent with
  the head revision implied by the export name ("去双目" = stereo pair
  removed), but unconfirmed.
- A stray `D435相机` instance in the CAD chest module has an implausible
  located bbox (±0.22 m span) and was ignored; re-check its placement when
  the CAD is next revised.
