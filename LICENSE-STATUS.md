# Licensing status — READ BEFORE MAKING THIS REPOSITORY PUBLIC

**There is no `LICENSE` file in this repository, and that is deliberate.**
manipulation-kit is intended to become OSS for "second development" customers
who receive a D1 with `d1-firmwared` pre-flashed.

Two questions used to block that. **One of them is now answered.**

## 1. Vendor CAD — ANSWERED: it is not here any more

**This repository contains no Omakase or vendor CAD geometry.** On 2026-09-10
every unlicensed mesh was removed from the work tree **and from the whole git
history**, and moved to the private
[`Omakase-Robotics-Org/manipulation-kit-assets`](https://github.com/Omakase-Robotics-Org/manipulation-kit-assets).
Decision by Shu (CTO): *publish the URDFs, not the CAD.*

What stayed is exactly what a description needs to be useful:

* the URDFs, complete and unedited — a URDF that **names** a mesh is not a
  redistribution of it;
* the generator that produces them, and `mkit-urdf build` still reproduces
  every committed URDF byte for byte with no CAD present;
* the mesh-free `d1.urdf` guard model, which never had geometry in it;
* third-party geometry that already carries a licence (below).

`tests/test_no_vendor_cad.py` is what keeps it that way: it fails if a mesh
with no recorded licence is ever committed here again, and it checks that the
`.gitignore` rules covering the fetch destinations are intact.

### Getting the geometry back

Neither of these puts anything into this repository's history — the fetch
destinations are all in `.gitignore`.

```sh
export MKIT_ASSETS_DIR=~/manipulation-kit-assets    # resolve, copying nothing
mkit-urdf fetch-assets --from ~/manipulation-kit-assets   # or copy it in
mkit-urdf export d1-wholebody-gripper --dest DIR --with-assets   # full bundle
```

An export made without `--with-assets` is mesh-free and **says so**: every
withheld reference is listed in `PROVENANCE.json` under `absent_external`,
with `assets_repo` naming where the rest of the robot is. A reader can always
tell *withheld* from *lost*.

## 2. Our own licence — still open, still Shu's call

What do we grant on Omakase-authored code in this repository — Apache-2.0,
BSD-3, something with an explicit patent grant, something source-available?

**Suggested: Apache-2.0** — it is what the third-party descriptions we already
carry use, so one `NOTICE` covers everything, and its patent grant is the
thing a second-development customer's lawyer actually looks for.

Nothing has been relicensed by being moved. Until a `LICENSE` lands, the code
carries the notice it arrived with:

| origin | its licence today |
|---|---|
| `dx-manipulator` (`arms/`, `hands/`) | **Proprietary, all rights reserved.** *"Copyright (c) 2026 Omakase Robotics. All rights reserved. Proprietary and confidential… no license is granted for use, copying, modification, or distribution outside the organization without prior written permission."* |
| `d1-sdk` (`description/`, `pyguard`, `wholebody/`, `scripts/`) | No `LICENSE` file in d1-sdk either; unlicensed org-internal code. |

## Every mesh in this repository, and why it is allowed to be

Class **A** is Omakase/vendor CAD with no redistribution grant — **not here**,
private assets repo. Class **B** is third-party geometry under a licence that
permits redistribution — **here**, with its upstream metadata intact.

| class | path | asset | licence | where it is |
|---|---|---|---|---|
| A | `description/d1_arm/{left,right}/meshes/*.STL` (18) | D1 arm CAD, vendor SolidWorks→URDF export | none granted in-repo | assets repo |
| A | `description/d1/meshes/body/*.STL` (10) | D1 body CAD (vendor export `urdf20260725`) | none granted in-repo | assets repo |
| A | `description/d1/meshes/gripper/*.STL` (5) | D1 stock parallel gripper + camera plate | none granted in-repo | assets repo |
| A | `hands/d1/parallel_gripper/descriptions/meshes/*.STL` (5) | the same gripper CAD, at its owning package | none granted in-repo | assets repo |
| A | `dist/d1-wholebody-gripper/**/*.STL` (22) | the prebuilt full bundle | none granted in-repo | assets repo (`dist/`) |
| B | `description/d1_arm/yubi_description/meshes/*.STL` (8) | **YUBI** parallel-jaw hand (Toyota / AIRoA) | **Apache-2.0** — `package.xml` says so (maintainer Jumpei Arima, Toyota) | ✅ committed here |
| B | `description/d1_yubi_description_v2/yubi_description/meshes/*.STL` (8) | the same YUBI hand, at the v2 package | **Apache-2.0** | ✅ committed here |
| B | `hands/leadshine/dh116s/descriptions/meshes/*.STL` (17) + `DH116S-R000-A1.xml` | **DH116S** hand meshes + MJCF | **Apache-2.0** — [`sorrowfeng/leadtron_hand_descriptions`](https://github.com/sorrowfeng/leadtron_hand_descriptions) @ `bd9c573` | ✅ committed here |

Non-mesh files that are third-party or vendor-shaped and stay:

| path | what | status |
|---|---|---|
| `description/d1/meshes/body/merged_robot.urdf` | the vendor's own body URDF | a description, not geometry — stays |
| `hands/d1/parallel_gripper/descriptions/gripper*.urdf` | vendor gripper URDF + our camera-plate composition | descriptions — stay |
| `docs/reference/safety_zones.h` | our own authored C++ header, frozen copy | ours to relicense |

The ~96 MB `d1/meshes/body_hifi/*.obj` decorative visuals were never imported.
That was a size decision (rendering only; the guard never opens one) and it is
also one fewer directory of unlicensed CAD. `mkit-urdf fetch-visuals` gets them
from a d1-sdk checkout.

The vendor `.so` files, the vendor `sdk/` source tree and the C++ examples are
**not here** — they stayed in d1-sdk and are being retired into `d1-firmwared`.
`DISTRIBUTION.md §7.2` requires the vendor source stay internal, and this repo
honours that.

## Still to do before flipping the switch

- [ ] **Our licence.** Apache-2.0? Shu / legal. (Question 2 above.)
- [ ] **`NOTICE` file** carrying the Apache-2.0 attributions for
      `yubi_description` and the DH116S descriptions, added with the `LICENSE`.
- [ ] **Re-check the remote before going public.** The history rewrite removed
      the CAD from *our* objects, but GitHub can keep unreachable objects
      server-side for some time and they stay reachable by SHA. Re-clone this
      repository fresh and confirm no `.STL` is reachable, and/or ask GitHub
      Support to garbage-collect it, **before** changing visibility.
- [ ] **One vendor name still in three URDFs.** `d1.urdf`,
      `d1_wholebody.urdf` and `d1_wholebody_gripper.urdf` each carry two XML
      comments naming the arm vendor's model. They were left alone on purpose:
      `d1.urdf` is embedded byte-for-byte in `exp--d1-firmware`
      (`crates/d1fw-core/assets/d1.urdf`, gated by `make check-assets`) and 426
      golden guard vectors are attributed to it. Changing them is one line in
      `generate_d1_urdf.py` plus re-copying the file into the firmware; the
      guard vectors' NUMBERS are unaffected. Do it in a coordinated pair of
      PRs, not as a drive-by.

Until the first two boxes are ticked, this repository stays **private**.
