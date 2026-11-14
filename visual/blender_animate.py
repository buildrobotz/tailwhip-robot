"""Render the Tailwhip demo act in Blender from cad/out/groups/*.glb and visual/out/motion.json.

  /Applications/Blender.app/Contents/MacOS/Blender -b -P visual/blender_animate.py -- OUT_DIR [--preview] [--still FRAME]

Writes OUT_DIR/frames/####.png and OUT_DIR/tailwhip_demo.mp4 (H.264 via Blender's FFmpeg output) and saves the
scene as OUT_DIR/tailwhip_demo.blend so it can be opened and tweaked by hand. Keep the video out of git.
--preview renders at half resolution with fewer samples; --still N renders one frame to OUT_DIR/still.png.
"""
import os, sys, json, math
import bpy
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
OUT = os.path.expanduser(argv[0]) if argv else os.path.join(HERE, 'out', 'render')
PREVIEW = '--preview' in argv
STILL = int(argv[argv.index('--still') + 1]) if '--still' in argv else None
os.makedirs(OUT, exist_ok=True)

motion = json.load(open(os.path.join(HERE, 'out', 'motion.json')))
FR = motion['frames']

# ------------------------------------------------------------------------------------------------- scene
bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
sc.render.fps = motion['fps']
sc.frame_start, sc.frame_end = 1, len(FR)
for eng in ('BLENDER_EEVEE_NEXT', 'BLENDER_EEVEE'):
    try:
        sc.render.engine = eng; break
    except TypeError:
        pass
sc.render.resolution_x, sc.render.resolution_y = (960, 540) if PREVIEW else (1280, 720)
sc.render.resolution_percentage = 100
ee = sc.eevee
for attr, val in (('taa_render_samples', 16 if PREVIEW else 32), ('use_shadows', True), ('use_raytracing', False),
                  ('use_gtao', True), ('shadow_ray_count', 2), ('shadow_step_count', 6)):
    if hasattr(ee, attr):
        try:
            setattr(ee, attr, val)
        except Exception:
            pass
sc.view_settings.view_transform = 'Standard'
sc.view_settings.look = 'None'
sc.view_settings.exposure = -0.35

# world: soft studio grey-blue
world = bpy.data.worlds.new('World'); sc.world = world; world.use_nodes = True
bg = world.node_tree.nodes['Background']
bg.inputs[0].default_value = (0.62, 0.68, 0.76, 1); bg.inputs[1].default_value = 0.35

# ------------------------------------------------------------------------------------------------- robot
groups = {}
for g in ('body', 'wheel_l', 'wheel_r', 'tail', 'head'):
    before = set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=os.path.join(ROOT, 'cad', 'out', 'groups', f'{g}.glb'))
    new = [o for o in bpy.data.objects if o not in before]
    root = bpy.data.objects.new(f'grp_{g}', None); sc.collection.objects.link(root)
    for o in new:
        if o.parent is None:
            o.parent = root
        if o.type == 'MESH':
            for poly in o.data.polygons:
                poly.use_smooth = False
            for m in o.data.materials:
                if m and m.use_nodes:
                    b = m.node_tree.nodes.get('Principled BSDF')
                    if b:
                        b.inputs['Roughness'].default_value = 0.55
                        if 'Specular IOR Level' in b.inputs:
                            b.inputs['Specular IOR Level'].default_value = 0.35
    groups[g] = root

# ------------------------------------------------------------------------------------------------- floor
bpy.ops.mesh.primitive_plane_add(size=12, location=(1.0, 0, 0))
floor = bpy.context.active_object; floor.name = 'floor'
fm = bpy.data.materials.new('floor'); fm.use_nodes = True
nt = fm.node_tree; bsdf = nt.nodes['Principled BSDF']
chk = nt.nodes.new('ShaderNodeTexChecker'); chk.inputs['Scale'].default_value = 60.0
chk.inputs['Color1'].default_value = (0.46, 0.44, 0.41, 1); chk.inputs['Color2'].default_value = (0.40, 0.385, 0.36, 1)
tc = nt.nodes.new('ShaderNodeTexCoord'); nt.links.new(tc.outputs['Object'], chk.inputs['Vector'])
nt.links.new(chk.outputs['Color'], bsdf.inputs['Base Color'])
bsdf.inputs['Roughness'].default_value = 0.92
if 'Specular IOR Level' in bsdf.inputs:
    bsdf.inputs['Specular IOR Level'].default_value = 0.08
floor.data.materials.append(fm)

# ------------------------------------------------------------------------------------------------- lights
def light(name, kind, loc, rot, energy, size=1.0, color=(1, 1, 1)):
    ld = bpy.data.lights.new(name, kind); ld.energy = energy; ld.color = color
    if kind == 'AREA':
        ld.size = size
    if kind == 'SUN':
        ld.angle = math.radians(6)
    ob = bpy.data.objects.new(name, ld); sc.collection.objects.link(ob)
    ob.location = loc; ob.rotation_euler = [math.radians(a) for a in rot]
    return ob
light('key', 'SUN', (0, 0, 3), (48, 8, -35), 1.7, color=(1.0, 0.96, 0.9))
light('fill', 'AREA', (0.4, 1.4, 1.2), (60, 0, 160), 45, size=2.0, color=(0.85, 0.9, 1.0))
light('rim', 'AREA', (-1.2, -0.3, 0.9), (65, 0, -100), 50, size=1.0)

# ------------------------------------------------------------------------------------------------- ball
bpy.ops.mesh.primitive_uv_sphere_add(radius=0.035, segments=48, ring_count=24, location=(5, 5, -5))
ball = bpy.context.active_object; ball.name = 'ball'
bpy.ops.object.shade_smooth()
bm = bpy.data.materials.new('ball'); bm.use_nodes = True
bm.node_tree.nodes['Principled BSDF'].inputs['Base Color'].default_value = (0.85, 0.12, 0.10, 1)
bm.node_tree.nodes['Principled BSDF'].inputs['Roughness'].default_value = 0.45
ball.data.materials.append(bm)

# ------------------------------------------------------------------------------------------------- camera + captions
cam_d = bpy.data.cameras.new('cam'); cam_d.lens = 50; cam_d.sensor_width = 36
cam_d.dof.use_dof = False; cam_d.dof.aperture_fstop = 5.6
cam = bpy.data.objects.new('cam', cam_d); sc.collection.objects.link(cam); sc.camera = cam
tgt = bpy.data.objects.new('cam_target', None); sc.collection.objects.link(tgt)
tr = cam.constraints.new('TRACK_TO'); tr.target = tgt; tr.track_axis = 'TRACK_NEGATIVE_Z'; tr.up_axis = 'UP_Y'
cam_d.dof.focus_object = tgt

caps = sorted({f['caption'] for f in FR if f['caption']}, key=lambda c: [f['caption'] for f in FR].index(c))
cap_mat = bpy.data.materials.new('caption'); cap_mat.use_nodes = True
em = cap_mat.node_tree.nodes.new('ShaderNodeEmission'); em.inputs['Color'].default_value = (0.08, 0.09, 0.11, 1)
em.inputs['Strength'].default_value = 1.0
cap_mat.node_tree.links.new(em.outputs[0], cap_mat.node_tree.nodes['Material Output'].inputs[0])
cap_objs = {}
for c in caps:
    cu = bpy.data.curves.new(f'cap_{len(cap_objs)}', 'FONT'); cu.body = c; cu.size = 0.0115; cu.align_x = 'CENTER'
    ob = bpy.data.objects.new(cu.name, cu); sc.collection.objects.link(ob)
    ob.data.materials.append(cap_mat)
    ob.parent = cam; ob.location = (0, -0.108, -0.6)
    cap_objs[c] = ob
title_cu = bpy.data.curves.new('title', 'FONT'); title_cu.body = 'TAILWHIP  ·  design animation, not a simulation'
title_cu.size = 0.0072; title_cu.align_x = 'LEFT'
title = bpy.data.objects.new('title', title_cu); sc.collection.objects.link(title)
title.data.materials.append(cap_mat); title.parent = cam; title.location = (-0.205, 0.110, -0.6)

# ------------------------------------------------------------------------------------------------- keyframes
def key_matrix(ob, M, frame):
    ob.matrix_world = Matrix(M)
    ob.keyframe_insert('location', frame=frame); ob.keyframe_insert('rotation_euler', frame=frame)

prev_cap = None
for k, f in enumerate(FR):
    fr = k + 1
    for g, ob in groups.items():
        key_matrix(ob, f['groups'][g], fr)
    if f['ball']:
        ball.location = f['ball']
    else:
        ball.location = (5, 5, -5)
    ball.keyframe_insert('location', frame=fr)
    cam.location = f['cam']['pos']; cam.keyframe_insert('location', frame=fr)
    tgt.location = f['cam']['target']; tgt.keyframe_insert('location', frame=fr)
    if f['caption'] != prev_cap:
        for c, ob in cap_objs.items():
            vis = (c == f['caption'])
            ob.hide_render = not vis; ob.hide_viewport = not vis
            ob.keyframe_insert('hide_render', frame=fr); ob.keyframe_insert('hide_viewport', frame=fr)
        prev_cap = f['caption']

# euler keys can flip between frames; make every rotation curve continuous and linear
for g, ob in groups.items():
    if ob.animation_data and ob.animation_data.action:
        act = ob.animation_data.action
        fcs = act.fcurves if hasattr(act, 'fcurves') else [fc for l in act.layers for s in l.strips for cb in s.channelbags for fc in cb.fcurves]
        for fc in fcs:
            if fc.data_path == 'rotation_euler':
                pts = fc.keyframe_points
                for i in range(1, len(pts)):
                    d = pts[i].co[1] - pts[i - 1].co[1]
                    if abs(d) > math.pi:
                        pts[i].co[1] -= round(d / (2 * math.pi)) * 2 * math.pi
            for p in fc.keyframe_points:
                p.interpolation = 'LINEAR'

bpy.ops.wm.save_as_mainfile(filepath=os.path.join(OUT, 'tailwhip_demo.blend'))

# ------------------------------------------------------------------------------------------------- render
if STILL is not None:
    sc.frame_set(STILL)
    sc.render.image_settings.file_format = 'PNG'
    sc.render.filepath = os.path.join(OUT, f'still_{STILL:04d}.png')
    bpy.ops.render.render(write_still=True)
else:
    sc.render.image_settings.file_format = 'PNG'
    sc.render.filepath = os.path.join(OUT, 'frames', '')
    bpy.ops.render.render(animation=True)
print('DONE', OUT)
