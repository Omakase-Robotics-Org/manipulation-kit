# DH116S vendor model (right hand, R000-A1)

Single source of truth for the DH116S sim model — consumers (d1-vr-teleop,
d1-manip-sim, …) resolve it from the installed package
(`manipulation_kit.hands.leadshine.dh116s.description_path()`), they do not vendor
copies.

Source: [sorrowfeng/leadtron_hand_descriptions](https://github.com/sorrowfeng/leadtron_hand_descriptions)
@ `bd9c573` (`DH116S/DH116S_URDF_MJCF_Files/DH116S-R000-A1/`), Apache-2.0.
(Originally vendored in d1-vr-teleop `assets/dh116s/`; moved here when the
hand driver was restructured out of d1-sdk into dx-manipulator.)

What the vendor model gives us (and why we use the MJCF, not the URDF):

- **6 active joints** with position actuators, in exactly the driver's
  `AXIS_NAMES` axis order: `finger11` (thumb swing), `finger12`
  (thumb flex), `finger21/31/41/51` (index/middle/ring/pinky MCP).
- **5 passive joints** (`finger13/22/32/42/52` — thumb IP + finger PIPs)
  coupled to their active joint by `<equality … polycoef>` quadratics, the
  vendor's fit of the same worm-gear linkage that this package's
  `data/coupling_*.csv` tabulates. URDF cannot express this —
  that's why the MJCF is vendored. (The CSV tables remain the
  hardware source of truth; the polycoefs are sim-grade.)
- 5 rubber fingertip pads as fixed bodies.

Local changes vs upstream:

- `meshdir` patched `../meshes` → `meshes` (flat vendored layout).
- STLs decimated with `fast-simplification` (base_link 121k → 12k faces,
  finger links → ≤4k; 15 MB → 3.6 MB). Visual-quality only — inertials and
  kinematics are untouched. Re-fetch upstream for full-resolution meshes.

The LEFT-hand variant (`DH116S-L000-A1`) is NOT vendored (strapped/glove
default side is the logical right = URDF `_L` = physical RIGHT arm). Pull it
from upstream the same way if a left hand ever mounts.
