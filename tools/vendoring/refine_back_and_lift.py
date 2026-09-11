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

def camera_window(name, x, cy, cz, width, height, depth):
    """Thin capsule in the YZ plane; corner radius is independent of depth."""
    from math import cos,sin,pi
    radius=height/2;outline=[]
    for side,start in ((1,-pi/2),(-1,pi/2)):
        center=cz+side*(width/2-radius)
        for i in range(33):
            a=start+i*pi/32
            outline.append((cy+radius*sin(a),center+radius*cos(a)))
    n=len(outline);verts=[(xx,y,z) for xx in (x-depth/2,x+depth/2) for y,z in outline]
    faces=[tuple(reversed(range(n))),tuple(range(n,2*n))]
    faces += [(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
    mesh=bpy.data.meshes.new(name);mesh.from_pydata(verts,[],faces);mesh.update()
    obj=bpy.data.objects.new(name,mesh);bpy.context.collection.objects.link(obj)
    bm=bmesh.new();bm.from_mesh(mesh);bmesh.ops.recalc_face_normals(bm,faces=bm.faces);bm.to_mesh(mesh);bm.free()
    return obj

def upper_chassis_cover():
    """Broad plan-view corner radii with a small rolled vertical edge."""
    from math import sin, cos, pi
    hx,hy,r=.539/2,.480/2,.060
    verts=[];faces=[];count=4*33
    for z,inset in ((.274,.003),(.277,0),(.435,0),(.438,.003)):
        for cx,cy,start in ((hx-r,hy-r,0),(-hx+r,hy-r,90),
                            (-hx+r,-hy+r,180),(hx-r,-hy+r,270)):
            for i in range(33):
                a=(start+i*90/32)*pi/180
                verts.append((-.0077+cx+(r-inset)*cos(a),.0016+cy+(r-inset)*sin(a),z))
    for level in range(3):
        for i in range(count):
            j=(i+1)%count
            faces.append((level*count+i,level*count+j,(level+1)*count+j,(level+1)*count+i))
    faces += [tuple(reversed(range(count))),tuple(range(3*count,4*count))]
    mesh=bpy.data.meshes.new('Rounded upper chassis');mesh.from_pydata(verts,[],faces);mesh.update()
    obj=bpy.data.objects.new('Paint_white_UpperChassisCover',mesh);bpy.context.collection.objects.link(obj)

def slope_front_panel():
    """Photo-fitted continuous rake; sensor trim follows the same surface."""
    for obj in bpy.context.scene.objects:
        if not any(n in obj.name for n in ('LowerChassisCover','BaseCamera','BaseLens','RecessedDeck')):continue
        # Bake locations so the deformation is shared by panels and hardware.
        bpy.context.view_layer.objects.active=obj
        bpy.ops.object.select_all(action='DESELECT');obj.select_set(True)
        bpy.ops.object.transform_apply(location=True,rotation=True,scale=True)
        for v in obj.data.vertices:
            x,y,z=v.co
            front=max(0,min(1,(x-.220)/.065))
            t=max(0,min(1,(z-.160)/.079))
            v.co.x += front*(.0055-.032*t*t*(3-2*t))
        obj.data.update()

def lower_chassis_profile():
    """Gallery-fitted wraparound apron; retains the original CAD extrema."""
    from math import sin, cos, pi
    hx, hy, radius = .289, .240, .052
    perimeter=[]
    for cx,cy,start in ((hx-radius,hy-radius,0),(-hx+radius,hy-radius,90),
                        (-hx+radius,-hy+radius,180),(hx-radius,-hy+radius,270)):
        for i in range(25):
            a=(start+i*90/24)*pi/180
            perimeter.append((.010+cx+radius*cos(a),.0016+cy+radius*sin(a)))
    # Sample the straight rear edge too: corner-only samples would bridge
    # across and flatten the wheel opening between the corner arcs.
    from math import ceil,hypot
    dense=[]
    for i,a in enumerate(perimeter):
        b=perimeter[(i+1)%len(perimeter)]
        steps=max(1,ceil(hypot(b[0]-a[0],b[1]-a[1])/.010))
        for j in range(steps):
            t=j/steps;dense.append((a[0]*(1-t)+b[0]*t,a[1]*(1-t)+b[1]*t))
    perimeter=dense
    count=len(perimeter);verts=[];faces=[]
    for level in range(21):
        for x,y in perimeter:
            rear=max(0,min(1,(-x-.210)/.060))
            arch=max(0,cos(min(1,max(0,(abs(y-.0016)-.130)/.098))*pi/2))
            front=max(0,min(1,(x-.210)/.060))
            front_relief=max(0,min(1,(.228-abs(y-.0016))/.068))
            bottom=.036+.067*rear*arch+.014*front*front_relief
            if level == 0:inset,z=.003,bottom
            elif level == 1:inset,z=0,bottom+.007
            else:
                z=.150+(level-2)*(.239-.150)/18
                inset=max(0,min(.004,(z-.219)*.004/.016)) if z<=.235 else .004+(z-.235)*2
            verts.append((.010+(x-.010)*(1-inset/hx),.0016+(y-.0016)*(1-inset/hy),z))
    for level in range(20):
        for i in range(count):
            j=(i+1)%count;faces.append((level*count+i,level*count+j,(level+1)*count+j,(level+1)*count+i))
    mesh=bpy.data.meshes.new('Wraparound apron');mesh.from_pydata(verts,[],faces);mesh.update()
    obj=bpy.data.objects.new('Paint_white_LowerChassisCover',mesh);bpy.context.collection.objects.link(obj)
    # Real top deck is recessed inside the white rim, revealing the supports.
    box('Paint_silver_RecessedDeck',(.010,.0016,.233),(.549,.448,.004),.0015)
    # Wide front insert with a curved lower edge, rather than a small plaque.
    from math import sqrt
    verts=[];faces=[]
    for i in range(65):
        y=.0016-.222+i*.444/64
        t=max(0,min(1,(abs(y-.0016)-.140)/.082))
        low=.166+.068*(1-sqrt(max(0,1-t*t)))
        for j in range(9):
            z=low+(.234-low)*j/8
            inset=max(0,min(.004,(z-.219)*.004/.016))
            local_y=(y-.0016)/(1-inset/.240)
            dy=max(0,abs(local_y)-.188)
            outer_x=.247+sqrt(max(0,.052**2-dy**2))
            x=.010+(outer_x-.010)*(1-inset/.289)+.0006
            verts.append((x,y,z))
    for i in range(64):
        for j in range(8):
            n=i*9+j;faces.append((n,n+9,n+10,n+1))
    mesh=bpy.data.meshes.new('Front insert');mesh.from_pydata(verts,[],faces);mesh.update()
    obj=bpy.data.objects.new('Paint_gray_BaseCameraPanel',mesh);bpy.context.collection.objects.link(obj)
    # Rear service slots and connector visible in the direct rear gallery view.
    for z in (.157,.172):
        box('Paint_dark_RearServiceSlot',(-.280,.036,z),(.001,.032,.003),.0004)
    cylinder('Paint_dark_RearConnector',(-.280,-.068,.166),.007,.0015)
    # Wheel geometry is rebuilt with the apron so old decimated wheel caps
    # cannot fill the rear cutout or protrude through the new round hubs.
    for y in (-.185,.185):
        for name,radius,depth in (('Paint_dark_DriveTread',.086,.048),('Paint_white_DriveHub',.064,.050)):
            o=cylinder(name,(.010,y,.087),radius,depth);o.rotation_euler[2]=radians(90)
            bpy.context.view_layer.objects.active=o;bpy.ops.object.transform_apply(location=False,rotation=True,scale=False)
    for x in (-.20,.20):
        for y in (-.17,.17):
            o=cylinder('Paint_silver_DeckSupport',(x,y,.255),.006,.040)
            o.rotation_euler[1]=radians(90)
            bpy.context.view_layer.objects.active=o;bpy.ops.object.transform_apply(location=False,rotation=True,scale=False)
    for x in (-.248,.235):
        for y in (-.133,.133):
            for name,radius,depth in (('Paint_dark_CasterTread',.041,.040),('Paint_white_CasterHub',.032,.042)):
                o=cylinder(name,(x,y,.070),radius,depth);o.rotation_euler[2]=radians(90)
                bpy.context.view_layer.objects.active=o;bpy.ops.object.transform_apply(location=False,rotation=True,scale=False)

hosts = ('chassis_link',) if '--chassis-only' in sys.argv else ('torso_column','chassis_link','head_link')
for host in hosts:
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
            caster = hi[2]<.122 and lo[2]>.025 and (lo[0]>.17 or hi[0]<-.17)
            if rail or upper_shell or lower_wall or caster or hi[2]<.274:
                bpy.data.objects.remove(part,do_unlink=True)
        box('Static lift cover',(.010,.0016,.51),(.106,.143,.42),.002)
        # Clean outer sheet-metal covers at the existing CAD envelope. Old
        # triangulated walls contained inward folds visible under white paint.
        upper_chassis_cover()
        lower_chassis_profile()
        box('Paint_dark_UpperSensorGlass',(.263,.0016,.348),(.003,.035,.020),.008)
        for y in (-.0074,.0106):
            cylinder('Paint_silver_UltrasoundRing',(.265,y,.348),.007,.001)
            cylinder('Paint_dark_Ultrasound',(.266,y,.348),.0055,.001)
        for name,x,width,height,depth in (
            ('Paint_white_BaseCameraRim',.301,.090,.024,.0015),
            ('Paint_dark_BaseCameraGlass',.302,.085,.019,.0015),
        ):
            o=camera_window(name,x,0,0,width,height,depth)
            o.rotation_euler[0]=radians(90);o.location=(0,.0016,.203)
            bpy.context.view_layer.objects.active=o
            bpy.ops.object.transform_apply(location=False,rotation=True,scale=False)
        for y in (-.025,.0016,.028):
            cylinder('Paint_silver_BaseLens',(.304,y,.203),.004,.001)
        slope_front_panel()
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
        camera_window('Paint_silver_HeadCameraRim',.1135,-.075,-.027,.079,.022,.002)
        camera_window('Paint_dark_HeadCameraGlass',.1148,-.075,-.027,.0775,.0205,.001)
        for z in (-.053,-.027,-.001):
            cylinder('Paint_optic_Lens',(.1155,-.075,z),.0034,.0004)
    for part in list(bpy.context.scene.objects):
        if part.type=='MESH':finish(part)
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.wm.obj_export(filepath=str(BODY/(host+'_refined.obj')),export_selected_objects=True,export_materials=False,forward_axis='Y',up_axis='Z')
    bpy.ops.object.delete(use_global=False)
