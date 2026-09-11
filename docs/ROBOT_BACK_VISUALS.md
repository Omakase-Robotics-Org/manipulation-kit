# Rear shell and telescoping lift visuals

The September 10, 2026 reference set `Robot back and middle part / IMG_1384–1391`
shows a white rear shell, a rounded blue control panel extending below the red
emergency stop, a blue lower waist, and two overlapping white lift covers.

`tools/vendoring/refine_back_and_lift.py` repairs the two broad rear shell
surfaces and replaces disconnected lift covers with overlapping covers. The
static cover spans world Z .30–.72 m. The moving cover spans torso-local Z
−.15–.195 m, overlapping it through the full 0–.30 m lift travel. Widths and
edge radii are visual estimates fitted to the existing CAD envelope and photos,
not new mechanical measurements. Joints, limits, collisions and inertias stay
unchanged. Smooth normals retain sharp sheet-metal edges.

The private `manipulation-kit-assets` repository carries recovered source
surfaces and generated OBJs under `manipulation_kit/description/d1/meshes/body_hifi`.
The sources were recovered from the committed Isaac visual meshes because the
original optional hi-fi OBJ files were absent from the available checkouts.
Their provenance is recorded with the private assets.

Rebuild, with all three repositories as siblings:

```sh
BODY=../manipulation-kit-assets/manipulation_kit/description/d1/meshes/body_hifi
blender --background --python tools/vendoring/refine_back_and_lift.py -- "$BODY"
python src/manipulation_kit/description/d1/tools/build_visual_colors.py --body-dir "$BODY" --refined
cp "$BODY/color_regions.json" src/manipulation_kit/description/d1/meshes/body_hifi/
mkit-urdf build
mkit-urdf export d1-wholebody-gripper --dest dist/d1-wholebody-gripper
```

In `d1-isaaclab`, after any URDF-to-USD conversion, run
`python scripts/sync_robot_body_visuals.py` with a Python containing `usd-core`.
It consumes this repository's generated URDF and the private OBJ assets,
updates the torso, chassis and head visual hosts in the simulator URDF/USD, and preserves the
robot's physics layers and arm geometry. The runtime USD is self-contained.

## Controls, head and chassis correction

The supplied closeups and https://omakaserobotics.ai/en/ show white chassis
sides and wheel covers, a blue top, a complete red stop cap, and a silver
power button on the sloping rear control panel. The controls are now separate
beveled meshes with explicit per-object paint, rather than coordinate-clipped
paint on a single CAD assembly. The silver button is aligned to the panel.

The two main head shells use their convex outer envelopes to remove inward
CAD dents; smooth normals replace the damaged imported normals. The eyes and
nose retain their CAD geometry. Camera trim/lenses, a microphone cap and rear
vent detail are restored as separate visual geometry. All optical frames,
collisions, inertias and joint locations are unchanged. These small decorative
parts are photo-fitted representations, not new mechanical specifications.

The color generator recognizes Blender objects named `Paint_<color>_*` and
keeps their full surface in that material, preventing blue stripes on buttons.

White paint revealed folded triangles in the outer chassis sidewalls. The
upper and lower visual panels are rebuilt with beveled edges at the CAD
bounds, retaining the blue roof and the separation between chassis levels.
Small front sensor bezels reproduce the reference appearance; these are
visuals only and do not relocate sensor frames.
