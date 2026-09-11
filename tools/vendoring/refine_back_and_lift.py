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
    if any(label in obj.name for label in ('FaceShell','Crown')):
        obj.data.normals_split_custom_set([(0,0,0)]*len(obj.data.loops))
        return
    mod=obj.modifiers.new('Area weighted shell normals','WEIGHTED_NORMAL');mod.keep_sharp=True;mod.weight=40
    bpy.ops.object.modifier_apply(modifier=mod.name)

def box(name, center, size, bevel):
    bpy.ops.mesh.primitive_cube_add(size=1,location=center)
    o=bpy.context.object;o.name=name;o.dimensions=size
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    m=o.modifiers.new('Folded cover edge','BEVEL');m.width=bevel;m.segments=4
    bpy.ops.object.modifier_apply(modifier=m.name);finish(o)
    return o

def cylinder(name, center, radius, depth):
    bpy.ops.mesh.primitive_cylinder_add(vertices=96, radius=radius, depth=depth, location=center, rotation=(0,radians(90),0))
    o=bpy.context.object;o.name=name
    bpy.ops.object.transform_apply(location=False,rotation=True,scale=True)
    m=o.modifiers.new('Machined rim','BEVEL');m.width=.0007;m.segments=3
    bpy.ops.object.modifier_apply(modifier=m.name);finish(o)
    return o

for host in ('torso_column','chassis_link','head_link'):
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
        # Replace the rear control hardware, instead of clipping paint through
        # a decimated CAD button. Component bounds are in the torso frame.
        for part in list(bpy.context.scene.objects):
            if part.type!='MESH':continue
            pts=[v.co for v in part.data.vertices]
            lo=[min(v[i] for v in pts) for i in range(3)];hi=[max(v[i] for v in pts) for i in range(3)]
            if hi[0]<-.05 and lo[2]>.530 and hi[2]<.570:
                bpy.data.objects.remove(part,do_unlink=True)
        cylinder('Paint_dark_StopGasket',(-.102,.0016,.550),.0198,.012)
        cylinder('Paint_red_EmergencyStop',(-.111,.0016,.550),.018,.012)
        from mathutils import Vector
        for name,center,radius,depth in (
            ('Paint_silver_PowerBezel',(-.075,.0016,.577),.0095,.005),
            ('Paint_silver_PowerFace',(-.077,.0016,.579),.0078,.0015),
        ):
            o=cylinder(name,center,radius,depth)
            # Cylinder helper bakes its X axis; rotate it onto the sloping bib.
            o.rotation_mode='QUATERNION'
            o.rotation_quaternion=Vector((1,0,0)).rotation_difference(Vector((-.707,0,.707)))
            bpy.context.view_layer.objects.active=o
            bpy.ops.object.transform_apply(location=False,rotation=True,scale=False)
        # Sliding inner cover follows the torso, overlapping the static column
        # across the full 0–300 mm travel (world z=.527+q+local z).
        box('Moving lift sleeve',(0.010,0.0016,.0225),(0.122,0.162,.345),.003)
    elif host=='chassis_link':
        # Remove disconnected CAD rail covers and their floating top cap.
        bpy.ops.object.mode_set(mode='EDIT'); bpy.ops.mesh.separate(type='LOOSE'); bpy.ops.object.mode_set(mode='OBJECT')
        for part in list(bpy.context.selected_objects):
            pts=[v.co for v in part.data.vertices]
            lo=[min(v[i] for v in pts) for i in range(3)];hi=[max(v[i] for v in pts) for i in range(3)]
            rail = hi[2]>.59 and lo[0]>-.1 and hi[0]<.11
            upper_shell = .273<lo[2]<.275 and .44<hi[2]<.444 and hi[1]-lo[1]>.47
            lower_wall = .035<lo[2]<.038 and .237<hi[2]<.24 and (hi[1]-lo[1]>.45 or hi[0]-lo[0]>.35)
            if rail or upper_shell or lower_wall:
                bpy.data.objects.remove(part,do_unlink=True)
        box('Static lift cover',(.010,.0016,.51),(.106,.143,.42),.002)
        # Clean outer sheet-metal covers at the existing CAD envelope. Old
        # triangulated walls contained inward folds visible under white paint.
        box('Paint_white_UpperChassisCover',(-.0077,.0016,.356),(.539,.480,.164),.014)
        box('Paint_white_LowerChassisCover',(.010,.0016,.137),(.578,.480,.202),.014)
        box('Paint_dark_UpperSensorGlass',(.263,.0016,.348),(.003,.035,.020),.008)
        for y in (-.0074,.0106):
            cylinder('Paint_silver_UltrasoundRing',(.265,y,.348),.007,.001)
            cylinder('Paint_dark_Ultrasound',(.266,y,.348),.0055,.001)
        box('Paint_silver_BaseCameraPanel',(.300,.0016,.207),(.003,.145,.048),.009)
        box('Paint_dark_BaseCameraGlass',(.302,.0016,.207),(.003,.063,.016),.006)
        for y in (-.018,.0016,.021):
            cylinder('Paint_silver_BaseLens',(.304,y,.207),.004,.001)
    else:
        bpy.ops.object.mode_set(mode='EDIT');bpy.ops.mesh.separate(type='LOOSE');bpy.ops.object.mode_set(mode='OBJECT')
        for part in list(bpy.context.selected_objects):
            pts=[v.co for v in part.data.vertices]
            lo=[min(v[i] for v in pts) for i in range(3)];hi=[max(v[i] for v in pts) for i in range(3)]
            shell=lo[0]<-.07 and hi[0]>.11 and hi[2]-lo[2]>.18
            eye=lo[0]>.103 and -.043<lo[1]<-.040 and hi[1]>-.002
            nose=.066<lo[0]<.069 and hi[0]>.122 and .020<hi[1]<.023
            if shell:
                bpy.ops.object.select_all(action='DESELECT');part.select_set(True);bpy.context.view_layer.objects.active=part
                bpy.ops.object.mode_set(mode='EDIT');bpy.ops.mesh.select_all(action='SELECT');bpy.ops.mesh.convex_hull();bpy.ops.object.mode_set(mode='OBJECT')
                part.name='Paint_navy_Crown' if hi[1]<0 else 'Paint_white_FaceShell'
            elif eye:part.name='Paint_cyan_Eye'
            elif nose:part.name='Paint_red_Nose'
            else:bpy.data.objects.remove(part,do_unlink=True)
        # Restore the rear ventilation detail over the repaired surface.
        from mathutils.bvhtree import BVHTree
        from mathutils import Vector
        shell=next(o for o in bpy.context.scene.objects if 'FaceShell' in o.name)
        bm=bmesh.new();bm.from_mesh(shell.data);bvh=BVHTree.FromBMesh(bm);bm.free()
        for i in range(11):
            z=-.027+(i-5)*.005
            points=[]
            for j in range(9):
                y=.020+j*.002
                hit,normal,_,_=bvh.ray_cast(Vector((-.2,y,z)),Vector((1,0,0)))
                if hit is not None:points.append(hit+Vector((-.0008,0,0)))
            if len(points)>1:
                curve=bpy.data.curves.new('RearVent','CURVE');curve.dimensions='3D';curve.bevel_depth=.0009;curve.bevel_resolution=2
                poly=curve.splines.new('POLY');poly.points.add(len(points)-1)
                for v,co in zip(poly.points,points):v.co=(*co,1)
                obj=bpy.data.objects.new('Paint_dark_RearVent',curve);bpy.context.collection.objects.link(obj)
                bpy.ops.object.select_all(action='DESELECT');obj.select_set(True);bpy.context.view_layer.objects.active=obj;bpy.ops.object.convert(target='MESH')
        o=cylinder('Paint_navy_MicrophoneCap',(.016,-.131,-.027),.034,.003)
        o.rotation_euler[2]=radians(90)
        bpy.context.view_layer.objects.active=o;bpy.ops.object.transform_apply(location=False,rotation=True,scale=False)
        # Rounded camera bezel and individual optical windows sit on the
        # repaired forehead. Optical/kinematic frames are unchanged.
        box('Paint_silver_HeadCameraRim',(.114,-.075,-.027),(.006,.024,.085),.005)
        box('Paint_dark_HeadCameraGlass',(.1175,-.075,-.027),(.002,.020,.080),.004)
        for z in (-.054,-.027,0):
            cylinder('Paint_silver_LensRing',(.119,-.075,z),.0048,.0015)
            cylinder('Paint_dark_Lens',(.120,-.075,z),.0039,.001)
    for part in list(bpy.context.scene.objects):
        if part.type=='MESH':finish(part)
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.wm.obj_export(filepath=str(BODY/(host+'_refined.obj')),export_selected_objects=True,export_materials=False,forward_axis='Y',up_axis='Z')
    bpy.ops.object.delete(use_global=False)
