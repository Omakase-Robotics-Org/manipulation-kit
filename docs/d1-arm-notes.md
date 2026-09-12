# The D1 arm — why the two arms are NOT mirror images

**Read this before "fixing" anything about the D1's left/right arm symmetry.**
The facts below were measured from the model files in this repository,
cross-checked against the vendor's own files, and confirmed with the arm
vendor directly. They explain an asymmetry that looks like a bug and is not
one. Machine-checked by
`devices/omakase_arm/pyguard/tests/test_description_consistency.py`.

## 1. Both arms are the SAME physical arm

The D1 carries two identical D1 arm units — not a left-handed and a
right-handed variant. The vendor confirmed this directly, and the vendor's own
per-arm URDFs agree: in
`description/d1_arm/left/d1_arm_left.urdf` and
`right/d1_arm_right.urdf`, **every joint origin, axis and limit of
`Joint1..7` and `JointTCP` is identical** once the `_L`/`_R` name suffixes and
the mesh file names are normalised away. For example `Joint3` is
`xyz="0 0.264 0" rpy="-1.5708 0 0"` on **both** sides — not `±0.264`.

Three cosmetic differences remain in the vendor's two files, none of them
kinematic, and they are the vendor's, not ours:

| link | difference |
|---|---|
| `Link5` | inertial COM x,y and the products `ixz`, `iyz` are sign-flipped between the files (mass and diagonal inertia identical) |
| `Link6` | inertial COM x,z and the products `ixy`, `iyz` are sign-flipped (mass and diagonal inertia identical) |
| `TCP_Link` | the `_R` file carries a zero-mass `<inertial>` block; the `_L` file has none |

They look like a SolidWorks mirrored-assembly export artifact. They do not move
any frame; `test_arm_chain_identical_across_all_models_and_sides` compares the
joints and passes.

(`_R`-suffix tree = SDK `ArmSide::A` = physical LEFT arm;
`_L` = `ArmSide::B` = physical RIGHT. That naming trap is documented in
`description/d1/tools/generate_d1_urdf.py` and is not the subject here.)

## 2. Therefore the wrist frames land 180° apart — necessarily

Mount two copies of the *same* chain with mirrored mounts (roll ∓90° at
y = ±0.037) and command mirror-symmetric joint values
(`q_L = (−q1, q2, −q3, q4, −q5, q6, −q7)` of `q_R`): every link **position**
mirrors in y, but the wrist link **orientation** cannot — a proper (same-
handed) mechanism cannot produce its own mirror image. Measured with the
pyguard FK on `description/d1/d1.urdf` over random in-range configurations:

| frame | vs. perfect mirror of the other side |
|---|---|
| `Link7` (wrist) | rotated **180.0°** about the local flange axis (z) |
| `TCP_Link` | **exact** mirror (deviation < 0.01°) |
| `yubi_*_hand_root` | **exact** mirror (deviation < 0.01°) |

This is geometry, not a modeling choice. It would take a physical 180°
wrist correction to remove it, and **J7's range is ±90°** (±1.5708 rad, in
every model file and in the vendor's own files), so no joint-space
correction is reachable.

## 3. The hand mount compensates — by design

Because the wrists are 180° apart, the YUBI hand is physically mounted
**180° rotated on the `_L` tree** so that the *hands* end up mirror-
symmetric:

```
_R tree:  xyz="0 -0.019 0.055"  rpy="1.5708 -1.5708 0"
_L tree:  xyz="0  0.019 0.055"  rpy="1.5708 -1.5708 3.1416"   (yaw + π, y flipped)
```

Consequences you can see on the real robot:

- the two hands **look** symmetric;
- the wrist **cables do not** — one arm's cable exits on the other side.

The cable asymmetry is therefore EXPECTED, not an assembly error. Every
integrator using the same arms shows the same thing (e.g. Genesis.ai's
published robot footage).

## 4. Which model files encode this correctly

Arm chain identical on both sides + hand mount 180° apart with the y offset
sign flipped — all consistent:

- `description/d1/d1.urdf` and `d1_wholebody.urdf` (generated — see
  `description/d1/tools/generate_d1_urdf.py`, `YUBI_FLANGE`)
- `description/d1_yubi_description_v2/urdf/d1_yubi.urdf` (generated — see
  `description/d1_yubi_description_v2/tools/assemble_d1_yubi.py`,
  `YUBI_MOUNT_RPY`)
- d1-manip-sim `assets/d1_yubi.urdf` — a vendored export of the file above,
  not an independent model
- d1-isaaclab `assets/d1/d1.urdf` (vendored copy of `description/d1/d1.urdf`)
  and `assets/d1/d1_bimanual.urdf` (built from it by that repo's
  `assets/d1/build_d1_urdf.py`)

The 180° mount is not only a modelling convention: in the Genesis simulation
(d1-manip-sim `scripts/diag_gripper_dir.py`, RTX 5080, home pose) it is what
puts the fingers away from the flange (approach·flange→hand = +0.945) and the
wrist camera up (+1.000) on **both** arms. With the vendor's identical-both-
sides mount the left camera comes out at −0.78, i.e. upside down.

## 5. Known-wrong files

- `description/d1_arm/left/d1_arm_left_with_yubi.urdf.xacro`
  — the vendor's drop mounted the hand with the **same** transform on both
  sides (a verbatim copy of the right file), i.e. a left hand rotated 180°
  from the real robot. **Fixed 2026-07-25** to the `_L` transform above;
  the consistency test now pins it. The vendor's v2 "D1 arm M6-S with yubi"
  drop shipped the same defect in its own `d1_arm_{r,l}_with_yubi.urdf.xacro`;
  those two files were never used to build a model and are not kept here.
- The vendor's merged body+arms URDF (Drive:
  `D1/URDF/urdf20260725.zip`, `merged_robot/urdf/merged_robot.urdf`) builds
  its left arm as a **geometric mirror** of the right: joint origins
  y-negated / rpy negated (e.g. `Joint3_L xyz="0 -0.264 0"` vs vendor
  right-arm `xyz="0 0.264 0"`), and all nine `_L` links reuse the `_R`
  STLs with `scale="1 -1 1"`. That contradicts the vendor's own per-arm
  URDFs and their own statement that both arms are identical hardware.
  **Do not adopt that file's arm chains.** (Its FK happens to mirror
  correctly, but it models a left-handed arm that does not exist, its
  joint-value sign conventions differ from the real `_L` arm, and mirrored
  `scale` meshes break most collision/inertia pipelines.) Reported as the
  vendor's problem; the body-only drop `urdf2026072302.zip` is the one we
  adopted (see `description/d1/README.md`).

## 6. If you think you found a symmetry bug

Run `python3 -m pytest devices/omakase_arm/pyguard/tests/test_description_consistency.py -q`.
It pins: chains identical across every model file and both sides, hand
mounts 180° apart, wrist-180°/TCP-mirror/hand-mirror FK invariants, and J7
= ±90°. If those pass, the asymmetry you are looking at is the one
described here.
