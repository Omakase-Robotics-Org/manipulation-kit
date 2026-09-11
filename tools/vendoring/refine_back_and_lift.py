"""Refine visual shells from September 2026 rear/lift reference photographs.

Run with Blender --background --python this_file -- BODY_DIRECTORY.
Inputs are recovered CAD surfaces (*_recovered.obj), kept in the private assets
repository. No joints, inertials or collision shapes are changed. The moving
sleeve is an approximate visual cover within the existing lift envelope.
"""
import sys
from pathlib import Path
import bpy
import bmesh
from math import radians

BODY = Path(sys.argv[sys.argv.index('--') + 1])
bpy.ops.object.select_all(action='SELECT'); bpy.ops.object.delete(use_global=False)

def finish(obj):
    bpy.context.view_layer.objects.active = obj
    # Sharp edges keep sheet-metal faces planar; curved shell normals are smooth.
    for p in obj.data.polygons: p.use_smooth = True
    bm=bmesh.new(); bm.from_mesh(obj.data)
    for e in bm.edges:
        e.smooth = len(e.link_faces)==2 and e.calc_face_angle(0) < radians(40)
    bm.to_mesh(obj.data); bm.free()
    mod=obj.modifiers.new('Area weighted shell normals','WEIGHTED_NORMAL');mod.keep_sharp=True;mod.weight=40
    bpy.ops.object.modifier_apply(modifier=mod.name)

def box(name, center, size, bevel):
    bpy.ops.mesh.primitive_cube_add(size=1,location=center)
    o=bpy.context.object;o.name=name;o.dimensions=size
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    m=o.modifiers.new('Folded cover edge','BEVEL');m.width=bevel;m.segments=4
    bpy.ops.object.modifier_apply(modifier=m.name);finish(o)
    return o

for host in ('torso_column','chassis_link'):
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.wm.obj_import(filepath=str(BODY/(host+'_recovered.obj')),forward_axis='Y',up_axis='Z')
    obj=bpy.context.object
    if host=='torso_column':
        # Repair the two broad rear panels independently, retaining their seam.
        bpy.ops.object.mode_set(mode='EDIT'); bpy.ops.mesh.separate(type='LOOSE'); bpy.ops.object.mode_set(mode='OBJECT')
        for part in list(bpy.context.selected_objects):
            pts=[v.co for v in part.data.vertices]
            lo=[min(v[i] for v in pts) for i in range(3)];hi=[max(v[i] for v in pts) for i in range(3)]
            if lo[0]<-.11 and hi[0]<.016 and hi[1]-lo[1]>.23:
                bpy.ops.object.select_all(action='DESELECT');part.select_set(True);bpy.context.view_layer.objects.active=part
                bpy.ops.object.mode_set(mode='EDIT');bpy.ops.mesh.select_all(action='SELECT');bpy.ops.mesh.convex_hull();bpy.ops.object.mode_set(mode='OBJECT')
        # Sliding inner cover follows the torso, overlapping the static column
        # across the full 0–300 mm travel (world z=.527+q+local z).
        box('Moving lift sleeve',(0.010,0.0016,.0225),(0.122,0.162,.345),.003)
    else:
        # Remove disconnected CAD rail covers and their floating top cap.
        bpy.ops.object.mode_set(mode='EDIT'); bpy.ops.mesh.separate(type='LOOSE'); bpy.ops.object.mode_set(mode='OBJECT')
        for part in list(bpy.context.selected_objects):
            pts=[v.co for v in part.data.vertices]
            lo=[min(v[i] for v in pts) for i in range(3)];hi=[max(v[i] for v in pts) for i in range(3)]
            if hi[2]>.59 and lo[0]>-.1 and hi[0]<.11:
                bpy.data.objects.remove(part,do_unlink=True)
        box('Static lift cover',(.010,.0016,.51),(.106,.143,.42),.002)
    for part in list(bpy.context.scene.objects):
        if part.type=='MESH':finish(part)
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.wm.obj_export(filepath=str(BODY/(host+'_refined.obj')),export_selected_objects=True,export_materials=False,forward_axis='Y',up_axis='Z')
    bpy.ops.object.delete(use_global=False)
