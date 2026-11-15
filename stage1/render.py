"""Offscreen MuJoCo renders for the stage 1 CAD (same look as cad/export.py)."""
import os, sys, shutil, tempfile
from dataclasses import dataclass, field
import numpy as np
from PIL import Image, ImageDraw
import mujoco

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'cad'))
import tailwhip as W

MATT = dict(spec=0.15, shin=0.2)

@dataclass
class It:
    name: str
    solid: object
    colour: tuple
    group: str = ''
    mat: dict = field(default_factory=lambda: MATT)

# ====================================================================================== rendering
def render(pairs, cams, size=(1400, 1000), ortho=False, dark=False):
    d = tempfile.mkdtemp(); assets, geoms = [], []
    for n, (i, m) in enumerate(pairs):
        f = os.path.join(d, f'm{n}.stl'); W.to_stl(m, f)
        r, g, b = i.colour; mt = i.mat
        assets.append(f'<mesh name="m{n}" file="{f}" scale="0.001 0.001 0.001" inertia="shell"/>'
                      f'<material name="t{n}" rgba="{r} {g} {b} 1" specular="{mt.get("spec", 0.2)}" shininess="{mt.get("shin", 0.3)}" '
                      f'reflectance="{mt.get("refl", 0)}" emission="{mt.get("emis", 0)}"/>')
        geoms.append(f'<geom type="mesh" mesh="m{n}" material="t{n}" contype="0" conaffinity="0"/>')
    cam_xml = []
    for cn, (pos, xy, fovy) in cams.items():
        orth = ' projection="orthographic"' if ortho else ''
        cam_xml.append(f'<camera name="{cn}" pos="{" ".join(f"{v / 1000:.5f}" for v in pos)}" '
                       f'xyaxes="{" ".join(f"{v:.5f}" for v in xy)}" fovy="{fovy / 1000 if ortho else fovy:.5f}"{orth}/>')
    if dark:
        look_ = """<headlight ambient="0.26 0.26 0.28" diffuse="0.34 0.34 0.36" specular="0.2 0.2 0.2"/>"""
        tex = """<texture type="skybox" builtin="gradient" rgb1="0.20 0.22 0.26" rgb2="0.03 0.03 0.04" width="64" height="512"/>
        <material name="desk" rgba="0.13 0.13 0.15 1" reflectance="0.10" specular="0.3" shininess="0.6"/>"""
        lights = """<light pos="0.5 -0.6 0.9" dir="-0.5 0.6 -0.9" diffuse="0.85 0.83 0.78" specular="0.5 0.5 0.5" castshadow="true"/>
        <light pos="-0.6 0.5 0.5" dir="0.6 -0.5 -0.5" diffuse="0.35 0.45 0.60" specular="0.4 0.4 0.5" castshadow="false"/>
        <light pos="0.2 0.7 0.3" dir="-0.2 -0.7 -0.3" diffuse="0.30 0.22 0.15" castshadow="false"/>"""
    else:
        look_ = """<headlight ambient="0.40 0.40 0.40" diffuse="0.55 0.55 0.55" specular="0.1 0.1 0.1"/>"""
        tex = """<texture name="grid" type="2d" builtin="checker" rgb1="0.94 0.93 0.90" rgb2="0.89 0.88 0.84" width="512" height="512"/>
        <material name="desk" texture="grid" texrepeat="16 16" reflectance="0.03"/>
        <texture type="skybox" builtin="flat" rgb1="0.86 0.89 0.93" rgb2="0.86 0.89 0.93" width="16" height="16"/>"""
        lights = """<light pos="0.3 -0.5 0.9" dir="-0.3 0.5 -0.9" diffuse="0.45 0.45 0.45" castshadow="true"/>
        <light pos="-0.4 0.3 0.6" dir="0.4 -0.3 -0.6" diffuse="0.25 0.25 0.25" castshadow="false"/>"""
    xml = f"""<mujoco><compiler angle="radian"/>
      <visual><global offwidth="{size[0]}" offheight="{size[1]}"/><quality shadowsize="4096" offsamples="8"/>
        {look_}<map znear="0.001" zfar="5"/></visual>
      <asset>{''.join(assets)}{tex}</asset>
      <worldbody>{lights}
        <geom type="plane" size="2 2 0.01" material="desk" contype="0" conaffinity="0"/>
        {''.join(geoms)}{''.join(cam_xml)}
      </worldbody></mujoco>"""
    model = mujoco.MjModel.from_xml_string(xml); data = mujoco.MjData(model); mujoco.mj_forward(model, data)
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
    from PIL import ImageFont
    for p in ('/System/Library/Fonts/Supplemental/Arial.ttf', '/System/Library/Fonts/Helvetica.ttc'):
        if os.path.exists(p):
            return ImageFont.truetype(p, sz)
    return ImageFont.load_default()

def label(img, text, dark=False, sz=28):
    im = Image.fromarray(img) if not isinstance(img, Image.Image) else img
    ImageDraw.Draw(im).text((24, 18), text, fill=(235, 235, 235) if dark else (40, 40, 40), font=font(sz))
    return im
