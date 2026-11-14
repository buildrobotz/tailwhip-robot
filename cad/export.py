"""Print files, assembly models and renders, all into cad/out/. Run: python export.py

  print/<part>.stl            each printed part in its print orientation, sitting on z = 0
  assembled_upright.glb/.stl  every part, balancing pose; per-part colours in the GLB
  assembled_rest.glb/.stl     resting on its tail
  groups/<group>.glb          body, wheel_l, wheel_r, tail, head in their own frames (for Blender / the simulator)
  render_*.png                MuJoCo offscreen renders (work headless on macOS)
"""
import os, shutil, tempfile
import numpy as np
import trimesh
import mujoco
from PIL import Image, ImageDraw, ImageFont
import tailwhip as W

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'out')
SWAP = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]], float) @ np.diag([1e-3, 1e-3, 1e-3, 1])
REST_PITCH = -25.0
REST_TAIL = W.tail_angle_for_foot_on_floor(REST_PITCH)
UP_TAIL = W.tail_ready()  # held at the rest angle while balancing: the tail is the back bumper


# ------------------------------------------------------------------------------------------------- print files
PRINT_POSE = {   # rotation (deg about x, y, z) that puts the part on its print face
    'core_left': [90, 0, 0],      # cut face (y = 0) down
    'core_right': [-90, 0, 0],
    'torso': [0, 0, 0],
    'tail': [-90, 0, 0],          # horn-arm face (y max) down
    'head': [0, 90, 0],           # open back (x min) down
    'encoder_ring': [90, 0, 0],
}

def write_print_files():
    d = os.path.join(OUT, 'print'); os.makedirs(d, exist_ok=True)
    for name, (fn, mat, n, note) in W.PRINTED.items():
        m = fn().rotate(PRINT_POSE[name])
        bb = m.bounding_box()
        m = m.translate([-(bb[0] + bb[3]) / 2, -(bb[1] + bb[4]) / 2, -bb[2]])
        W.to_stl(m, os.path.join(d, f'{name}.stl'))
    print('wrote', d)


# ------------------------------------------------------------------------------------------------- meshes
def tm(man, colour):
    v, t = W.mesh_np(man)
    mesh = trimesh.Trimesh(v.copy(), t.copy(), process=False)
    mat = trimesh.visual.material.PBRMaterial(baseColorFactor=list(colour) + [1.0], metallicFactor=0.0, roughnessFactor=0.6)
    mesh.visual = trimesh.visual.TextureVisuals(material=mat)
    return mesh

def posed(its, X, pitch, tail, pan):
    tf = W.group_tfs(X, pitch, tail, pan)
    return [(i, W.apply(i.solid, tf[i.group])) for i in its if i.solid is not None]

def write_glb(name, pairs):
    scene = trimesh.Scene(); allm = []
    for k, (i, m) in enumerate(pairs):
        mesh = tm(m, i.colour); allm.append(mesh.copy())
        mesh.apply_transform(SWAP)
        scene.add_geometry(mesh, node_name=f'{i.name}', geom_name=f'{i.name}_{k}')
    scene.export(os.path.join(OUT, f'{name}.glb'))
    trimesh.util.concatenate(allm).export(os.path.join(OUT, f'{name}.stl'))

def write_groups(its):
    d = os.path.join(OUT, 'groups'); os.makedirs(d, exist_ok=True)
    for g in ('body', 'wheel_l', 'wheel_r', 'tail', 'head'):
        scene = trimesh.Scene()
        for k, i in enumerate([i for i in its if i.group == g and i.solid is not None]):
            mesh = tm(i.solid, i.colour); mesh.apply_transform(SWAP)
            scene.add_geometry(mesh, node_name=i.name, geom_name=f'{i.name}_{k}')
        scene.export(os.path.join(d, f'{g}.glb'))
    print('wrote', d)


# ------------------------------------------------------------------------------------------------- renders
def render(pairs, cams, size=(1400, 1000), ortho=False):
    d = tempfile.mkdtemp()
    assets, geoms = [], []
    for n, (i, m) in enumerate(pairs):
        f = os.path.join(d, f'm{n}.stl'); W.to_stl(m, f)
        assets.append(f'<mesh name="m{n}" file="{f}" scale="0.001 0.001 0.001" inertia="shell"/>')
        r, g, b = i.colour
        geoms.append(f'<geom type="mesh" mesh="m{n}" rgba="{r} {g} {b} 1" contype="0" conaffinity="0"/>')
    cam_xml = []
    for cn, (pos, xy, fovy) in cams.items():
        orth = ' projection="orthographic"' if ortho else ''
        cam_xml.append(f'<camera name="{cn}" pos="{" ".join(f"{v / 1000:.5f}" for v in pos)}" '
                       f'xyaxes="{" ".join(f"{v:.5f}" for v in xy)}" fovy="{fovy / 1000 if ortho else fovy:.5f}"{orth}/>')
    xml = f"""<mujoco><compiler angle="radian"/>
      <visual><global offwidth="{size[0]}" offheight="{size[1]}"/><quality shadowsize="4096" offsamples="8"/>
        <headlight ambient="0.40 0.40 0.40" diffuse="0.55 0.55 0.55" specular="0.1 0.1 0.1"/>
        <map znear="0.001" zfar="5"/></visual>
      <asset>{''.join(assets)}
        <texture name="grid" type="2d" builtin="checker" rgb1="0.94 0.93 0.90" rgb2="0.89 0.88 0.84" width="512" height="512"/>
        <material name="desk" texture="grid" texrepeat="16 16" reflectance="0.03"/>
        <texture type="skybox" builtin="flat" rgb1="0.86 0.89 0.93" rgb2="0.86 0.89 0.93" width="16" height="16"/></asset>
      <worldbody>
        <light pos="0.3 -0.5 0.9" dir="-0.3 0.5 -0.9" diffuse="0.45 0.45 0.45" castshadow="true"/>
        <light pos="-0.4 0.3 0.6" dir="0.4 -0.3 -0.6" diffuse="0.25 0.25 0.25" castshadow="false"/>
        <geom type="plane" size="2 2 0.01" material="desk" contype="0" conaffinity="0"/>
        {''.join(geoms)}{''.join(cam_xml)}
      </worldbody></mujoco>"""
    model = mujoco.MjModel.from_xml_string(xml); data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    out = {}
    with mujoco.Renderer(model, size[1], size[0]) as r:
        for cn in cams:
            r.update_scene(data, camera=cn); out[cn] = r.render().copy()
    shutil.rmtree(d, ignore_errors=True)
    return out

def look(pos, target, up=(0, 0, 1)):
    f = np.array(target, float) - np.array(pos, float); f /= np.linalg.norm(f)
    x = np.cross(f, up); x /= np.linalg.norm(x); y = np.cross(x, f)
    return list(x) + list(y)

def font(sz):
    for p in ('/System/Library/Fonts/Supplemental/Arial.ttf', '/System/Library/Fonts/Helvetica.ttc'):
        if os.path.exists(p):
            return ImageFont.truetype(p, sz)
    return ImageFont.load_default()

def save(img, name, title=None):
    im = Image.fromarray(img) if not isinstance(img, Image.Image) else img
    if title:
        band = Image.new('RGB', (im.width, im.height + 60), (255, 255, 255)); band.paste(im, (0, 60)); im = band
        ImageDraw.Draw(im).text((24, 16), title, fill=(40, 40, 40), font=font(26))
    im.save(os.path.join(OUT, name))

def side_view(its, pitch, tail, name, title, dims):
    """Orthographic view from the right (camera on -y looking +y), with dimension annotations.
    dims: list of (kind, args). World mm -> pixels: u = W/2 + (x - cx) * s, v = H/2 - (z - cz) * s."""
    Wd, Hd, ext = 1400, 1000, 340.0
    cx, cz = -10.0, 135.0
    pairs = posed(its, 0, pitch, tail, 0)
    img = render(pairs, {'side': ((cx, -800, cz), look((cx, -800, cz), (cx, 0, cz)), ext)}, size=(Wd, Hd), ortho=True)['side']
    s = Hd / ext
    P = lambda x, z: (Wd / 2 + (x - cx) * s, Hd / 2 - (z - cz) * s)
    im = Image.fromarray(img); d = ImageDraw.Draw(im); f = font(22); col = (200, 30, 30)
    for kind, a in dims:
        if kind == 'h':      # horizontal dimension at height z between x0 and x1
            x0, x1, z, txt = a
            p0, p1 = P(x0, z), P(x1, z); d.line([p0, p1], fill=col, width=2)
            for p in (p0, p1):
                d.line([(p[0], p[1] - 8), (p[0], p[1] + 8)], fill=col, width=2)
            d.text(((p0[0] + p1[0]) / 2, p0[1] - 16), txt, fill=col, font=f, anchor='mm')
        elif kind == 'v':    # vertical dimension at x between z0 and z1
            x, z0, z1, txt = a
            p0, p1 = P(x, z0), P(x, z1); d.line([p0, p1], fill=col, width=2)
            for p in (p0, p1):
                d.line([(p[0] - 8, p[1]), (p[0] + 8, p[1])], fill=col, width=2)
            d.text((p0[0] + 12, (p0[1] + p1[1]) / 2), txt, fill=col, font=f, anchor='lm')
        elif kind == 'pt':   # labelled point
            x, z, txt = a
            p = P(x, z); d.ellipse([p[0] - 5, p[1] - 5, p[0] + 5, p[1] + 5], outline=col, width=3)
            d.text((p[0] + 12, p[1] - 4), txt, fill=col, font=f, anchor='lm')
        elif kind == 'ang':  # angle from vertical at the wheel contact
            x0, z0, deg, ln, txt = a
            p0 = P(x0, z0)
            d.line([p0, P(x0, z0 + ln)], fill=(120, 120, 120), width=2)
            d.line([p0, P(x0 + ln * np.sin(np.radians(deg)), z0 + ln * np.cos(np.radians(deg)))], fill=col, width=2)
            d.text(P(x0 + 0.5 * ln * np.sin(np.radians(deg / 2)) - 4, z0 + 0.62 * ln), txt, fill=col, font=f, anchor='mm')
    save(im, name, title)


def main():
    os.makedirs(OUT, exist_ok=True)
    its = W.items()
    write_print_files()
    write_groups(its)
    up = posed(its, 0, 0, UP_TAIL, 0)
    rest = posed(its, 0, REST_PITCH, REST_TAIL, 0)
    write_glb('assembled_upright', up)
    write_glb('assembled_rest', rest)
    print('wrote assembled_upright / assembled_rest')

    hero = (420, -520, 330)
    imgs = render(up, {'hero': (hero, look(hero, (0, 0, 120)), 32),
                       'back': ((-460, 380, 300), look((-460, 380, 300), (0, 0, 115)), 34),
                       'front': ((520, 0, 150), look((520, 0, 150), (0, 0, 135)), 32)})
    save(imgs['hero'], 'render_upright.png', 'Tailwhip, balancing (tail held ready at its rest angle)')
    save(imgs['back'], 'render_back.png', 'From behind: ESP32 on the spine, tail servo in the hip block')
    save(imgs['front'], 'render_front.png', 'From the front: power boards on the spine, IMU on the motor web')
    imgs = render(rest, {'hero': (hero, look(hero, (-20, 0, 100)), 32)})
    save(imgs['hero'], 'render_rest.png', f'Tailwhip at rest on its tail ({-REST_PITCH:.0f} deg back)')

    H = W.PAN_TOP + W.HEAD['z1'] + W.R_WHEEL
    side_view(its, 0, 24.3, 'side_upright.png', 'Side view, upright, tail foot on the floor (dimensions in mm)',
              [('v', (95, 0, H, f'{H:.0f}')), ('v', (60, W.R_WHEEL, W.R_WHEEL + 76, 'CoM 76')),
               ('pt', (0, W.R_WHEEL + 76, 'CoM')), ('pt', (W.TAIL_PIVOT[0], W.R_WHEEL + W.TAIL_PIVOT[1], 'tail pivot')),
               ('h', (W.TAIL_PIVOT[0], 0, -12, '30')), ('v', (-110, 0, W.R_WHEEL, 'axle 32.5'))])
    side_view(its, REST_PITCH, REST_TAIL, 'side_rest.png', 'Side view, resting on the tail',
              [('ang', (0, W.R_WHEEL, REST_PITCH, 230, f'{-REST_PITCH:.0f} deg'))])
    side_view(its, -25.0, REST_TAIL, 'side_heel.png', 'Fallen back: the tail catches it at 25 deg',
              [('ang', (0, W.R_WHEEL, -25.0, 230, '25 deg'))])
    side_view(its, 18.0, REST_TAIL, 'side_chin.png', 'Fallen forward: the feet stop it at 18 deg',
              [('ang', (0, W.R_WHEEL, 18.0, 230, '18 deg'))])
    # parts laid out
    parts = []
    xoff = -260
    for name, (fn, mat, n, note) in W.PRINTED.items():
        m = fn(); bb = m.bounding_box()
        m = m.translate([xoff - bb[0], -(bb[1] + bb[4]) / 2, -bb[2]])
        parts.append((W.Item(name, m, W.COL['tail'] if mat == 'ABS' else W.COL['body'], 'body'), m))
        xoff += (bb[3] - bb[0]) + 25
    cam = (60, -620, 420)
    save(render(parts, {'p': (cam, look(cam, (60, 0, 30)), 36)}, size=(1600, 900))['p'], 'render_parts.png',
         'Printed parts: core halves, torso, tail (ABS), head, encoder rings')
    print('wrote renders')


if __name__ == '__main__':
    main()
