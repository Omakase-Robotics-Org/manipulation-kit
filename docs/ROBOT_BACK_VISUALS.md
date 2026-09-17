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

## Pricing-gallery lower chassis and forehead window

All seven pricing-gallery views at https://omakaserobotics.ai/en/ were
compared, including the direct rear view and both side views. The lower
apron now follows a rounded rectangular perimeter (52 mm corner radius),
with a curved rear lower cutout rising from 36 to 103 mm above the floor.
A recessed silver deck, broad curved front sensor insert, and round caster
hubs/treads reproduce the visible construction. These profile/radius values
are fitted from the photos within the CAD envelope; they are not claimed as
factory measurements. The original joint and collision geometry is preserved.

The forehead optical window is a thin capsule, 79 × 22 mm, with a fine trim
and three small optical elements behind dark glass. Its rounded outline is
constructed independently of its 2 mm depth; a generic cube bevel previously
clamped to the thickness and left square corners. Optical frames stay fixed.

The apron rebuild removes lower decorative CAD fragments so they cannot
occlude the wheel cutout. Six round visual wheels replace the decimated
surfaces: four caster wheels and two drive wheels with white hubs and dark
treads. Their visual radii/locations follow the original CAD bounds; the
physical collision and joint definitions remain unchanged.

The final front-only correction follows the direct front gallery image:
a 444 mm wide gray inset with a flat central lower edge and rounded ends,
a 90 × 24 mm light-bordered capsule sensor window, and 14 mm of shallow
front lower-edge relief exposing the front caster bottoms. Rear cutout,
forehead optics and robot/table placement are unchanged by this correction.

## Chassis close-up refinement

The supplied `chassis/IMG_1396 2.HEIC` and `IMG_1401 2.HEIC` show a
raked front insert, with its upper edge recessed and its lower edge flowing
outward into the apron. A continuous 32 mm photo-fitted rake now deforms the
front apron, gray insert and sensor trim together. The upper white housing
uses a 60 mm plan-view corner radius independently of its 3 mm rolled edge.
The rear lower apron and wheel opening are retained. These remain visual
estimates within the original envelope, not measured mechanical dimensions.

The blue roof now shares the rounded white housing perimeter, with a 1.5 mm
covering lip and a smoothly rolled top. The old recovered roof and attached
upper-wall fragments are removed while retaining the raised rear mast boot.
This eliminates the exposed white corner ledges and overlapping old sidewalls.


## Physical lift zero calibration (supersedes earlier cover dimensions)

The official height range is 1293–1593 mm, with a 300 mm stroke. The neutral
refined head is 809 mm above dual_base, so its fully lowered mounting height
is corrected to 484 mm above the floor (previously 526.78 mm). This changes
upper-body mounting height, not arm lengths, relative joint axes, or stroke.
The previous Isaac command offset of +20 mm must not be applied.

The annotated September 10 photo places the moving sleeve lower edge 95 mm
above the chassis roof at zero, and 295 mm above it at +200 mm. Its static
counterpart spans world Z 480–883 mm and remains overlapped throughout the
stroke. Neutral-head height and sleeve zero are measured from the resulting
geometry in Isaac tests. The preserved blue rear mast boot is about 84 mm
above the chassis roof.


## Sleeve extension for the 29 mm AMR cover offset (2026-09-16)

The moving sleeve is authored in the torso frame, so moving the lift origin
moves it too. When the measured AMR cover height moved `base_footprint ->
dual_base` from 0.484 m to 0.513 m (see **The AMR cover offset** in
`docs/d1-description-README.md`), the sleeve's 79 mm bottom rose with the
torso and floated 40.7 mm above the chassis mast boot at lift 0, with a 9 mm
see-through gap over the static column at lift 300.

The sleeve is therefore 29 mm longer at the BOTTOM: height 116 -> 145 mm,
bottom torso-local Z 79 -> 50 mm, top unchanged at 195 mm. It now spans
torso-local Z 50–195 mm, which puts the lower lip back at world Z 563 mm —
the 11 mm boot clearance the sleeve had before the origin moved — and makes
lift 300 overlap the static column by 20 mm.

| | value |
|---|---|
| torso-local Z span | **50 … 195 mm** (height 145 mm) |
| lower lip, lift 0 | world Z **563 mm** — 11.7 mm above the 551.3 mm boot top |
| lower lip, lift 300 | world Z **863 mm** |
| static-column overlap | 145 mm at lift 0, 20 mm at lift 300 (column Z 400–883 mm) |

This is a visual cover only: no joint, limit, inertial or collision primitive
changes, and the guard is unaffected. The authoring constant is the
`Moving lift sleeve` box in `tools/vendoring/refine_back_and_lift.py`; the
shipped OBJ carries the same edit in `manipulation-kit-assets` PR #5, restored
by PR #6.

### Rejected: full-stroke insertion (2026-09-17)

A longer sleeve was tried and **rejected — do not retry it.** The variant put
the bottom cap at torso-local Z **−275 mm** (470 mm tall,
`box('Moving lift sleeve',(0.010,0.0016,-.040),(0.122,0.162,.470),.003)`), so
the lower lip stayed buried in the opaque AMR shell at every lift value: world
Z 238 mm at lift 0, still 13.3 mm below the boot top at lift 300. That is what
a real telescoping outer tube does, and it looks worse. Shu, 2026-09-17:

> これは劣化してる。隙間があるままの方がまし
> ("this is a regression; leaving the gap is better")

The long cover reads as a solid slab rather than a telescoping one, and the
visible gap that shows the lift moving disappears. The 11 mm clearance over the
boot at lift 0 is wanted, not a defect to design out. It shipped briefly as
`manipulation-kit` `3d77521` and `manipulation-kit-assets` `c26707b`, both
reverted.
