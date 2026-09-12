# Vendoring tools — you almost certainly do not need these

These scripts **re-import CAD from a vendor drop** into the shapes this package
ships: they decimate meshes, derive mount geometry from them, and rewrite the
bundled descriptions. They are how the committed `.urdf` and `.STL` files got
their geometry, kept here so that provenance is reproducible rather than folklore.

They are **not part of the kit**. They are not installed by the wheel, no
runtime code imports them, and nothing you do with `manipulation_kit` — IK, the
motion guard, tool configs, `mkit-urdf build` — calls them. Every one of them
needs source CAD that **is not in this repository**: the geometry has no
redistribution grant on file (see [`LICENSE-STATUS.md`](../../LICENSE-STATUS.md))
and lives in the private `manipulation-kit-assets` repository, plus, for the
mesh work, `pip install 'manipulation-kit[meshes]'` (trimesh, fast-simplification).
Without those inputs the scripts have nothing to read, which is the expected
state of a public checkout.

| script | takes | writes into the package |
|---|---|---|
| `vendor_visual_meshes.py` | vendor body STLs | `description/d1/meshes/body/` |
| `prepare_visual_meshes.py` | hi-fi body OBJs | `description/d1/meshes/body_hifi/` |
| `vendor_gripper_description.py` | gripper CAD + vendor URDF | `hands/d1/parallel_gripper/descriptions/` |
| `vendor_camera_plate.py` | wrist-camera plate mesh | `hands/d1/parallel_gripper/descriptions/` |

Run them from a checkout with the kit installed (`pip install -e .`);
`_kit_paths.py` resolves the destination through the import, so they write into
the package whether that is a checkout or a wheel.
