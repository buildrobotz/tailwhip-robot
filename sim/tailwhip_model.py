"""Build the Tailwhip MuJoCo model (MJCF) from the CAD in cad/tailwhip.py.

    python sim/tailwhip_model.py          # writes sim/tailwhip.xml and sim/meshes/*.stl, prints mass properties

Bodies (world frame: x forward, z up; body frame = the CAD body frame, origin on the axle):
  chassis   free joint. Mass, centre of mass and inertia of every part that rides on the body (motors, boards,
            battery, head, tail servo) computed from the CAD meshes. Collision: heel and chin capsules (the
            skid's rounded corners), a torso box and a head sphere.
  gear_l/r  motor output shaft: hinge about y, driven by the DC-motor model in tailwhip_env.py. Tiny mass plus
            the motor's reflected rotor inertia as armature.
  wheel_l/r hinge relative to gear_l/r limited to +-backlash/2: that is the gearbox slop. Cylinder collision.
  tail      hinge at the tail pivot, position servo (rate- and torque-limited in tailwhip_env.py).
  foot      hinge 26 mm down the tail with the leaf spring's stiffness: the printed C-leaf.

Every number comes from cad/tailwhip.py (tagged there) or is tagged here. Nothing is measured yet: the bench
script host/bench.py writes host/measurements.json and tailwhip_env.py uses it when present.
"""
import os, sys
import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'cad'))
import tailwhip as W

MESH_DIR = os.path.join(HERE, 'meshes')
WHEEL_Y = W.WHEEL_IN + W.W_WHEEL / 2           # mm, centre of each tyre
LEAF_AT = 26.0                                 # mm down the tail where the leaf hinge sits [C]
ROTOR_INERTIA = 1.2e-7                          # kg m2, 130-size motor rotor [E]
GEAR = 48.0                                     # [D]
ARMATURE = ROTOR_INERTIA * GEAR ** 2            # reflected at the wheel: 2.8e-4 kg m2
LEAF_K = 0.66                                   # N m/rad, from cad/analysis.py [E]
LEAF_DAMP = 0.004                               # N m s/rad [E] (ABS is lossy; ~10 % of critical)


def mass_props(items, M_to_frame):
    """Total mass (kg), CoM (m) and inertia about the CoM (kg m2) of items, in a common frame."""
    pts = []
    for i in items:
        m = W.item_mass(i) / 1000.0
        if i.solid is None:
            c = np.array(i.point, float) / 1000.0
            pts.append((m, c, np.zeros((3, 3))))
            continue
        v, t = W.mesh_np(W.apply(i.solid, M_to_frame(i)))
        tm = trimesh.Trimesh(v / 1000.0, t, process=False)
        if tm.volume <= 0:
            tm.invert()
        dens = m / tm.volume
        I = tm.moment_inertia * dens            # about the mesh's own centre of mass
        pts.append((m, tm.center_mass, I))
    M = sum(p[0] for p in pts)
    c = sum(p[0] * p[1] for p in pts) / M
    I = np.zeros((3, 3))
    for m, ci, Ii in pts:
        d = ci - c
        I += Ii + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return M, c, I

def full_inertia(I):
    return f'{I[0,0]:.6e} {I[1,1]:.6e} {I[2,2]:.6e} {I[0,1]:.6e} {I[0,2]:.6e} {I[1,2]:.6e}'

def write_mesh(items, M_to_frame, name):
    ms = [W.apply(i.solid, M_to_frame(i)) for i in items if i.solid is not None]
    W.to_stl(W.union(ms), os.path.join(MESH_DIR, name + '.stl'))


def build():
    os.makedirs(MESH_DIR, exist_ok=True)
    its = W.items()
    body_its = [i for i in its if i.group in ('body', 'head')]
    tail_its = [i for i in its if i.group == 'tail']
    wl_its = [i for i in its if i.group == 'wheel_l']
    wr_its = [i for i in its if i.group == 'wheel_r']
    to_body = lambda i: W.head_tf(0) if i.group == 'head' else np.eye(4)
    ident = lambda i: np.eye(4)
    Mb, cb, Ib = mass_props(body_its, to_body)
    Mt, ct, It = mass_props(tail_its, ident)
    Mw, cw, Iw = mass_props(wl_its, ident)
    write_mesh([i for i in body_its if i.group == 'body'], ident, 'body')
    write_mesh([i for i in body_its if i.group == 'head'], to_body, 'head')
    write_mesh(tail_its, ident, 'tail')
    write_mesh(wl_its, ident, 'wheel_l')
    write_mesh(wr_its, ident, 'wheel_r')

    mm = lambda *v: ' '.join(f'{x / 1000:.5f}' for x in v)
    R = W.R_WHEEL
    hx, hz, hr = W.HEEL; tx, tz, tr = W.TOE
    px, pz = W.TAIL_PIVOT
    xs0, xs1 = W.HOLDER_X - W.HOLDER[0] / 2 - 2.3, W.HOLDER_X + W.HOLDER[0] / 2 + 2.3
    torso_c = ((xs0 + xs1) / 2, 0, (W.FLANGE_Z[1] + W.TRAY_TOP) / 2)
    torso_h = ((xs1 - xs0) / 2, W.HOLDER[1] / 2 + 1.5, (W.TRAY_TOP - W.FLANGE_Z[1]) / 2)
    head_c = (1.0, 0, W.PAN_TOP + 24.0)
    # tail split at the leaf hinge: hub keeps the fork, foot carries the rest. Mass split 60/40 [E].
    tail_hub_m, tail_foot_m = 0.6 * Mt, 0.4 * Mt

    xml = f'''<mujoco model="tailwhip">
  <compiler angle="radian" meshdir="meshes" autolimits="true"/>
  <option timestep="0.002" integrator="implicitfast" cone="elliptic"/>
  <size memory="8M"/>
  <visual><global offwidth="1280" offheight="720"/><quality shadowsize="4096"/>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.5 0.5 0.5"/></visual>
  <default>
    <geom contype="1" conaffinity="1" condim="3" friction="0.8 0.02 0.001" solref="0.004 1" solimp="0.9 0.95 0.001"/>
    <default class="vis"><geom contype="0" conaffinity="0" group="2" mass="0" density="0"/></default>
  </default>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.86 0.84 0.80" rgb2="0.80 0.78 0.74" width="512" height="512"/>
    <material name="floor" texture="grid" texrepeat="40 40" reflectance="0.02"/>
    <texture type="skybox" builtin="gradient" rgb1="0.80 0.85 0.92" rgb2="0.95 0.95 0.95" width="64" height="64"/>
    <mesh name="body" file="body.stl" scale="0.001 0.001 0.001"/>
    <mesh name="head" file="head.stl" scale="0.001 0.001 0.001"/>
    <mesh name="tail" file="tail.stl" scale="0.001 0.001 0.001"/>
    <mesh name="wheel_l" file="wheel_l.stl" scale="0.001 0.001 0.001"/>
    <mesh name="wheel_r" file="wheel_r.stl" scale="0.001 0.001 0.001"/>
  </asset>
  <worldbody>
    <light pos="0.5 -0.6 1.2" dir="-0.4 0.5 -1" diffuse="0.6 0.6 0.6" castshadow="true"/>
    <geom name="floor" type="plane" size="20 20 0.1" material="floor"/>
    <body name="chassis" pos="0 0 {R / 1000:.5f}">
      <freejoint name="root"/>
      <inertial pos="{cb[0]:.5f} {cb[1]:.5f} {cb[2]:.5f}" mass="{Mb:.5f}" fullinertia="{full_inertia(Ib)}"/>
      <geom class="vis" type="mesh" mesh="body" rgba="0.96 0.62 0.22 1"/>
      <geom class="vis" type="mesh" mesh="head" rgba="0.95 0.94 0.90 1"/>
      <geom name="heel" type="capsule" fromto="{mm(hx, -W.SKID_HW + hr, hz, hx, W.SKID_HW - hr, hz)}" size="{hr / 1000:.5f}" mass="0" rgba="1 0 0 0"/>
      <geom name="chin" type="capsule" fromto="{mm(tx, -W.SKID_HW + tr, tz, tx, W.SKID_HW - tr, tz)}" size="{tr / 1000:.5f}" mass="0" rgba="1 0 0 0"/>
      <geom name="feet" type="capsule" fromto="{mm(W.JAW[0], -W.JAW[3] + W.JAW[2], W.JAW[1], W.JAW[0], W.JAW[3] - W.JAW[2], W.JAW[1])}" size="{W.JAW[2] / 1000:.5f}" mass="0" rgba="1 0 0 0"/>
      <geom name="torso" type="box" pos="{mm(*torso_c)}" size="{mm(*torso_h)}" mass="0" rgba="0 0 1 0"/>
      <geom name="headc" type="sphere" pos="{mm(*head_c)}" size="0.024" mass="0" rgba="0 0 1 0"/>
      <site name="imu" pos="{mm(19, 0, 27)}" size="0.004"/>
      <site name="headtop" pos="{mm(0, 0, W.PAN_TOP + W.HEAD['z1'])}" size="0.004"/>
      <site name="push" pos="{mm(0, 0, 150)}" size="0.004"/>
      <body name="gear_l" pos="0 0 0">
        <joint name="motor_l" type="hinge" axis="0 1 0" armature="{ARMATURE:.3e}" damping="0.0004" frictionloss="0.002"/>
        <inertial pos="0 {WHEEL_Y / 1000:.5f} 0" mass="0.002" diaginertia="1e-7 1e-7 1e-7"/>
        <body name="wheel_l" pos="0 0 0">
          <joint name="slop_l" type="hinge" axis="0 1 0" range="-0.04 0.04" damping="0.0002"/>
          <inertial pos="{cw[0]:.5f} {cw[1]:.5f} {cw[2]:.5f}" mass="{Mw:.5f}" fullinertia="{full_inertia(Iw)}"/>
          <geom name="tyre_l" type="cylinder" pos="0 {WHEEL_Y / 1000:.5f} 0" euler="1.5708 0 0" size="{R / 1000:.5f} {W.W_WHEEL / 2000:.5f}" mass="0" rgba="0 0 0 0"/>
          <geom class="vis" type="mesh" mesh="wheel_l" rgba="0.15 0.15 0.16 1"/>
        </body>
      </body>
      <body name="gear_r" pos="0 0 0">
        <joint name="motor_r" type="hinge" axis="0 1 0" armature="{ARMATURE:.3e}" damping="0.0004" frictionloss="0.002"/>
        <inertial pos="0 {-WHEEL_Y / 1000:.5f} 0" mass="0.002" diaginertia="1e-7 1e-7 1e-7"/>
        <body name="wheel_r" pos="0 0 0">
          <joint name="slop_r" type="hinge" axis="0 1 0" range="-0.04 0.04" damping="0.0002"/>
          <inertial pos="{cw[0]:.5f} {-cw[1]:.5f} {cw[2]:.5f}" mass="{Mw:.5f}" fullinertia="{full_inertia(Iw)}"/>
          <geom name="tyre_r" type="cylinder" pos="0 {-WHEEL_Y / 1000:.5f} 0" euler="1.5708 0 0" size="{R / 1000:.5f} {W.W_WHEEL / 2000:.5f}" mass="0" rgba="0 0 0 0"/>
          <geom class="vis" type="mesh" mesh="wheel_r" rgba="0.15 0.15 0.16 1"/>
        </body>
      </body>
      <body name="tail" pos="{mm(px, 0, pz)}">
        <joint name="tail" type="hinge" axis="0 1 0" range="0.30 2.70" damping="0.004" armature="2e-5"/>
        <inertial pos="0 0 -0.012" mass="{tail_hub_m:.5f}" diaginertia="2e-6 2e-6 1e-6"/>
        <geom class="vis" type="mesh" mesh="tail" rgba="0.20 0.62 0.70 1"/>
        <body name="foot" pos="0 0 {-LEAF_AT / 1000:.5f}">
          <joint name="leaf" type="hinge" axis="0 1 0" stiffness="{LEAF_K}" damping="{LEAF_DAMP}" range="-0.6 0.6"/>
          <inertial pos="0 0 -0.024" mass="{tail_foot_m:.5f}" diaginertia="2e-6 2e-6 1e-6"/>
          <geom name="foot" type="capsule" fromto="{mm(0, -W.FOOT_HW + W.FOOT_R, -(W.TAIL_L - LEAF_AT), 0, W.FOOT_HW - W.FOOT_R, -(W.TAIL_L - LEAF_AT))}" size="{W.FOOT_R / 1000:.5f}" mass="0" rgba="0.2 0.6 0.7 0.0"/>
        </body>
      </body>
    </body>
  </worldbody>
  <contact>
    <exclude body1="chassis" body2="wheel_l"/><exclude body1="chassis" body2="wheel_r"/>
    <exclude body1="chassis" body2="tail"/><exclude body1="chassis" body2="foot"/>
    <exclude body1="tail" body2="foot"/>
  </contact>
  <actuator>
    <motor name="m_l" joint="motor_l" ctrlrange="-1 1" gear="1"/>
    <motor name="m_r" joint="motor_r" ctrlrange="-1 1" gear="1"/>
    <position name="tail" joint="tail" kp="1.5" ctrlrange="0.30 2.70" forcerange="-0.13 0.13"/>
  </actuator>
  <sensor>
    <gyro name="gyro" site="imu"/>
    <accelerometer name="acc" site="imu"/>
  </sensor>
</mujoco>
'''
    path = os.path.join(HERE, 'tailwhip.xml')
    with open(path, 'w') as f:
        f.write(xml)
    return dict(mass_body=Mb, com_body=cb, I_body=Ib, mass_tail=Mt, mass_wheel=Mw, path=path)


if __name__ == '__main__':
    p = build()
    print(f"chassis {p['mass_body'] * 1000:.0f} g, CoM {np.round(p['com_body'] * 1000, 1)} mm, "
          f"tail {p['mass_tail'] * 1000:.1f} g, wheel {p['mass_wheel'] * 1000:.1f} g each")
    print('inertia about CoM (kg m2):\n', np.round(p['I_body'], 7))
    import mujoco
    m = mujoco.MjModel.from_xml_path(p['path'])
    d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    print('total mass', round(float(m.body_subtreemass[1]) * 1000, 1), 'g; wrote', p['path'])
