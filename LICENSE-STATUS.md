# Provenance and licensing

manipulation-kit is Open Source under the **Apache License, Version 2.0**
(see [`LICENSE`](LICENSE)). Attributions for the third-party robot
descriptions it redistributes are in [`NOTICE`](NOTICE).

This note explains one thing that surprises readers: **the D1 CAD geometry is
not in this repository, on purpose.** We publish the URDFs, not the CAD.

## What is here, and what is not

The vendor/Omakase CAD (arm, body, gripper meshes) is **not** distributed here.
On 2026-09-10 every unlicensed mesh was removed from the work tree and from the
whole git history and moved to the private
[`Omakase-Robotics-Org/manipulation-kit-assets`](https://github.com/Omakase-Robotics-Org/manipulation-kit-assets).

What stayed is exactly what a description needs to be useful:

* the URDFs, complete and unedited — a URDF that **names** a mesh is not a
  redistribution of it;
* the generator that produces them; `mkit-urdf build` reproduces every
  committed URDF byte for byte with no CAD present;
* the mesh-free `d1.urdf` guard model, which never had geometry in it;
* third-party geometry that already carries a redistribution licence (below).

`tests/test_no_vendor_cad.py` keeps it that way: it fails if a mesh with no
recorded licence is ever committed here again.

An export made without `--with-assets` is mesh-free and says so: every withheld
reference is listed in `PROVENANCE.json` under `absent_external`, with
`assets_repo` naming where the rest of the robot is. A reader can always tell
*withheld* from *lost*.

### Getting the geometry back

Neither of these puts anything into this repository's history — the fetch
destinations are all in `.gitignore`.

```sh
export MKIT_ASSETS_DIR=~/manipulation-kit-assets            # resolve, copying nothing
mkit-urdf fetch-assets --from ~/manipulation-kit-assets     # or copy it in
mkit-urdf export d1-wholebody-gripper --dest DIR --with-assets   # full bundle
```

## Every mesh in this repository, and why it is allowed to be

Class **A** is Omakase/vendor CAD with no redistribution grant — **not here**,
private assets repo. Class **B** is third-party geometry under a licence that
permits redistribution — **here**, with its upstream metadata intact.

| class | path | asset | licence | where it is |
|---|---|---|---|---|
| A | `description/d1_arm/{left,right}/meshes/*.STL` (18) | D1 arm CAD | none granted in-repo | assets repo |
| A | `description/d1/meshes/body/*.STL` (10) | D1 body CAD | none granted in-repo | assets repo |
| A | `description/d1/meshes/gripper/*.STL` (5) | D1 stock gripper + camera plate | none granted in-repo | assets repo |
| A | `hands/d1/parallel_gripper/descriptions/meshes/*.STL` (5) | the same gripper CAD | none granted in-repo | assets repo |
| A | `dist/d1-wholebody-gripper/**/*.STL` (22) | the prebuilt full bundle | none granted in-repo | assets repo (`dist/`) |
| B | `description/d1_arm/yubi_description/meshes/*.STL` (8) | **YUBI** parallel-jaw hand (Toyota / AIRoA) | **Apache-2.0** — `package.xml` (maintainer Jumpei Arima) | committed here |
| B | `description/d1_yubi_description_v2/yubi_description/meshes/*.STL` (8) | the same YUBI hand, v2 package | **Apache-2.0** | committed here |
| B | `hands/leadshine/dh116s/descriptions/meshes/*.STL` (17) + `DH116S-R000-A1.xml` | **DH116S** hand meshes + MJCF | **Apache-2.0** — [`sorrowfeng/leadtron_hand_descriptions`](https://github.com/sorrowfeng/leadtron_hand_descriptions) @ `bd9c573` | committed here |

Non-mesh files that are vendor-shaped and stay, because a description is not
geometry: `description/d1/meshes/body/merged_robot.urdf` (the vendor's body
URDF), `hands/d1/parallel_gripper/descriptions/gripper*.urdf` (vendor gripper
URDF + our camera-plate composition), and `docs/reference/safety_zones.h` (our
own authored header).

The ~96 MB `d1/meshes/body_hifi/*.obj` decorative visuals were never imported —
a size decision (rendering only; the guard never opens one). `mkit-urdf
fetch-visuals` gets them from a d1-sdk checkout.
