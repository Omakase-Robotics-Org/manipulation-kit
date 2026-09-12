# D1 reference colors

The whole-body URDF visuals use the supplied Desktop `d1.avif` reference:
white body/arms, navy head cap/chest/lower torso/base trim, dark camera windows,
silver neck/front fascia, and small cyan/red face accents. Region boundaries
approximate the photograph on the existing CAD. Gripper geometry is unchanged.

`tools/build_visual_colors.py` splits the original `meshes/body_hifi/*_hifi.obj`
surface into material regions. Triangles crossing a boundary are clipped at
exact planes, with normals and UVs interpolated at the new vertices. This
preserves the CAD surface and avoids stair-step borders from whole-face paint.
The eyes and nose are colored by their complete connected CAD components,
so paint follows the molded parts instead of an approximate projected mask.
`color_regions.json` records the palette, source hashes, face counts and before/after surface areas. Every input face
is checked for surface-area conservation during generation.
Original OBJ files are included so regeneration also works in a fresh clone.

Rebuild from the SDK root:

```sh
python3 tools/vendoring/prepare_visual_meshes.py
python3 description/d1/tools/build_visual_colors.py
python3 description/d1/tools/generate_d1_urdf.py
```

Joints, collision shapes, inertias and guard geometry are unchanged. Isaac Sim
5.1's OBJ importer drops these URDF colors; d1-isaaclab's
`scripts/restore_obj_materials.py` reapplies the URDF colors during conversion.

`prepare_visual_meshes.py` requires numpy, scipy and trimesh. It retains the
original CAD files, identifies both eye pieces and the nose, and replaces the
two front torso panels with one continuous smooth envelope, removing their
horizontal seam. Speaker inserts and
small decorative details are removed; the chest camera opening is retained.
Only visual geometry changes: collision shapes, cameras and joints are kept.

The navy collar is a continuous wrap around the neck with a rounded U-shaped
front edge and a shallower upper-back extension. Its boundary is an inclined
elliptical cylinder, clipped with 64 arc segments to avoid coarse triangular
paint edges. The front camera window and lower blue waist band are unchanged.
