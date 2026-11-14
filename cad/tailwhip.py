"""Tailwhip geometry: every dimension, every printed part, every bought part, and how they move.

One module so the CAD, the checks, the renders, the Blender animation and (later) the MuJoCo model all read
the same numbers. `pip install manifold3d numpy trimesh`.

FRAMES (mm, degrees)
  body   origin on the wheel axle, midway between the wheels. x forward, y left, z up along the body.
  tail   origin on the tail servo shaft. At tail angle 0 the tail points straight down the body (-z);
         a positive tail angle swings it backwards (towards -x).
  head   origin on the pan axis at the top of the pan-servo horn. Positive pan turns the head to the left.
  world  x forward along the floor, z up, floor at z = 0. Body pitch is positive leaning FORWARD;
         the tail-down rest is a negative pitch (leaning back).

SOURCES. Every number is tagged:  [D] datasheet or seller drawing,  [E] estimate (typical listing, to be
measured),  [C] chosen by this design. Bench measurements will replace the [E] numbers; nothing here has been
measured yet.
"""
from dataclasses import dataclass, field
import numpy as np
from manifold3d import Manifold, CrossSection

# =============================================================================================== parameters
CLR = 0.3                          # [C] print clearance around bought parts

# --- wheels: stock yellow TT wheel with rubber tyre
R_WHEEL, W_WHEEL = 32.5, 26.0      # [D] 65 mm wheel, ~26 mm wide
R_RIM = 25.0                       # [E] rim radius under the tyre
# --- TT gearmotor, 1:48, mounted VERTICALLY: shaft on the axle, motor can pointing up the body
TT_T = 18.8                        # [E] gearbox thickness along the shaft (y)
TT_W = 22.5                        # [E] gearbox width (x)
TT_GB_LEN = 37.0                   # [E] gearbox length (z)
TT_SHAFT_END = 11.0                # [E] shaft centre to the near end of the gearbox
TT_CAN_LEN, TT_CAN_D, TT_CAN_FLAT = 33.0, 20.0, 15.0    # [E] 130-size can: length, diameter, across flats
TT_STUB = 9.1                      # [E] shaft stub beyond each gearbox face (37 mm tip to tip)
TT_SHAFT_D = 5.4                   # [D] double-D shaft
TT_HOLES = [(-8.0, 18.0), (8.0, 18.0)]   # [E] the two M3 through-holes (x, z), parallel to the shaft. MEASURE.
# --- y stack (from the middle out)
WEB_HALF = 3.0                     # [C] each printed half carries 3 mm of the central web
GB_IN = 13.5                       # [C] gearbox inner face |y|: leaves 1.4 mm between the inner stub and the web
GB_OUT = GB_IN + TT_T              # 32.3
WHEEL_IN = 35.5                    # [C] wheel inner face |y| (hub on 5.9 mm of the 9.1 mm stub, plus its screw)
RING_T = 1.2                       # [C] printed encoder ring glued to the inside face of each wheel
RING_R0, RING_R1, RING_SLOTS = 17.5, 25.0, 16   # [C] ring radii and window count (32 edges per turn)
TRACK = 2 * (WHEEL_IN + W_WHEEL / 2)
# --- SG-90 servo [D] TowerPro drawing (B 22.8, D 12.2, E 32, F 16), clones vary +-0.5 mm
S_LEN, S_THK, S_H = 22.8, 12.2, 22.7       # case length, thickness, height (case bottom to case top)
S_SHAFT = 5.9                              # shaft centre to the near end of the case
S_TAB_Y, S_TAB_T, S_TAB_L = 15.9, 2.5, 4.85
S_SPLINE_TOP, S_HUMP_R = 32.0, 5.9
HORN_HUB_R, HORN_PLATE_T, HORN_LEN = 3.6, 1.6, 15.5     # single-arm horn
# --- tail servo, in the hip block behind the motor cans; shaft along +y
TAIL_PIVOT = (-30.0, 30.0)         # [C] (x, z) in the body frame. Far back = less servo torque (see analysis.py)
TS_Y0 = -20.0                      # [C] tail servo case bottom (y); spline top at y = +12
HORN_IN = TS_Y0 + S_SPLINE_TOP + 0.5   # 12.5: horn plate underside
# --- tail
TAIL_L = 62.0                      # [C] pivot to the centre of the foot pad
FOOT_R, FOOT_HW = 6.0, 20.0        # [C] foot pad radius and half width
LEAF_T, LEAF_W = 2.4, 14.0         # [C] leaf spring thickness and width (ABS)
LEAF_Z0, LEAF_Z1, LEAF_BULGE = -24.0, -48.0, 6.0   # [C] where the C-leaf starts/ends along the tail, and its bulge
FORK_HORN = (HORN_IN, HORN_IN + 4.2)     # horn arm of the fork (y)
FORK_PIVOT = (-28.4, -24.4)              # pivot arm of the fork (y), M3 shoulder screw into the hip block
# --- bumpers (lowest corners of the skid under the motors)
HEEL = (-29.0, -10.0, 7.0)         # [C] (x, z, radius) of the rounded heel (kept out of the tail's swing)
TOE = (29.0, -11.0, 6.5)           # [C] (x, z, radius) of the rounded toe / chin bumper
SKID_HW = 22.0                     # [C] skid half width
# Rev B (found in the MuJoCo model): the heel and chin touch first but sit under the centre of mass, and a real
# fall carries the body straight over any bumper close to it. What stops a fall is a contact FAR beyond the centre
# of mass: long low feet in front, and the tail (held at a 'ready' angle while balancing) behind.
JAW = (86.0, 0.0, 6.0, 22.0)       # [C] front feet bar: x, z, radius, half width (on the core, prints with it)
JAW_HW = 8.0                       # [C] half thickness of the plate carrying the bar
READY_PITCH = -25.0                # [C] the tail's ready angle puts the foot on the floor at this pitch
# --- torso: electronics spine, battery sleeve ON TOP of it (raises the centre of mass), pan-servo block
FLANGE_Z = (60.0, 64.0)
SPINE_T = 3.0                      # [C] spine plate thickness (x)
SPINE_HW = 28.0
TRAY_Z0 = 124.0                    # [C] battery sleeve floor (bottom face)
HOLDER = (41.0, 77.5, 20.5)        # [E] 2 x 18650 series holder: x, y, z
HOLDER_X = 4.0                     # [C] holder shifted forward to put the centre of mass over the axle
TRAY_WALL = 2.0
TRAY_TOP = TRAY_Z0 + 2 * TRAY_WALL + HOLDER[2] + 0.6
W_SLEEVE_FRONT = HOLDER_X + HOLDER[0] / 2 + CLR + TRAY_WALL   # front face of the battery sleeve (x)
PAN_Y0 = TRAY_TOP + 1.0            # [C] pan servo case bottom (z), shaft vertical at x = 0
PAN_TOP = PAN_Y0 + S_SPLINE_TOP + 0.5 + HORN_PLATE_T   # top of the pan horn = head frame origin
# --- head (ESP32-CAM)
CAM_PCB = (27.0, 40.0, 1.6)        # [D] ESP32-CAM board y, z, thickness
HEAD = dict(x0=-15.0, x1=17.0, hw=19.5, z0=1.0, z1=47.0, wall=1.8)   # [C] head shell in the head frame

PLA, ABS = 1.24, 1.04              # [D] g/cm3
FILL = 0.55                        # [E] printed mass / solid mass at 3 walls + 20 % gyroid for parts this thin
BED = (220, 220, 250)              # [D] Ender 3

COL = dict(body=(0.96, 0.55, 0.16), body2=(0.98, 0.78, 0.30), tail=(0.20, 0.62, 0.70), head=(0.97, 0.95, 0.90),
           ring=(0.97, 0.97, 0.95), tt=(0.98, 0.83, 0.10), can=(0.72, 0.72, 0.75), tyre=(0.12, 0.12, 0.13),
           rim=(0.98, 0.83, 0.10), servo=(0.15, 0.30, 0.75), horn=(0.95, 0.95, 0.93), pcb=(0.08, 0.40, 0.28),
           pcb_blue=(0.10, 0.25, 0.65), chip=(0.12, 0.12, 0.14), cell=(0.25, 0.55, 0.30), holder=(0.10, 0.10, 0.11),
           cap=(0.15, 0.20, 0.55), steel=(0.65, 0.66, 0.70), lens=(0.05, 0.05, 0.06), eye=(0.10, 0.10, 0.12))


# =============================================================================================== primitives
def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])

def cbox(cx, cy, cz, sx, sy, sz):
    return box(cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2, cz - sz / 2, cz + sz / 2)

def ycyl(x, z, r, y0, y1, seg=56):
    return Manifold.cylinder(y1 - y0, r, r, seg).rotate([-90, 0, 0]).translate([x, y0, z])

def zcyl(x, y, r, z0, z1, seg=48):
    return Manifold.cylinder(z1 - z0, r, r, seg).translate([x, y, z0])

def xcyl(y, z, r, x0, x1, seg=40):
    return Manifold.cylinder(x1 - x0, r, r, seg).rotate([0, 90, 0]).translate([x0, y, z])

def xz_round(x0, x1, z0, z1, r, y0, y1):
    """Box with its four edges along y rounded to r."""
    return Manifold.batch_hull([ycyl(x, z, r, y0, y1) for x in (x0 + r, x1 - r) for z in (z0 + r, z1 - r)])

def xz_prism(cs, y0, y1):
    """Extrude a 2D CrossSection drawn in (x, z) along y from y0 to y1."""
    return cs.extrude(y1 - y0).rotate([90, 0, 0]).translate([0, y1, 0])

def hull(*ms):
    return Manifold.batch_hull(list(ms))

def union(ms):
    out = Manifold()
    for m in ms:
        out = out + m
    return out

def mesh_np(m):
    me = m.to_mesh()
    return np.asarray(me.vert_properties)[:, :3].astype(float), np.asarray(me.tri_verts).astype(int)

def centroid(m):
    v, t = mesh_np(m)
    a, b, c = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
    vol6 = np.einsum('ij,ij->i', a, np.cross(b, c))
    return (vol6[:, None] * (a + b + c)).sum(0) / (4 * vol6.sum())

def to_stl(man, path):
    v, t = mesh_np(man)
    tri = v[t]; n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    rec = np.zeros(len(t), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    rec['n'], rec['v'] = n, tri
    with open(path, 'wb') as f:
        f.write(b'Tailwhip'.ljust(80, b' ')); f.write(np.uint32(len(t)).tobytes()); f.write(rec.tobytes())


# =============================================================================================== bought parts
def tt_motor(side):
    """TT gearmotor in the body frame. side = +1 (left, +y) or -1 (right)."""
    y0, y1 = sorted((side * GB_IN, side * GB_OUT))
    gb = box(-TT_W / 2, TT_W / 2, y0, y1, -TT_SHAFT_END, TT_GB_LEN - TT_SHAFT_END)
    yc = side * (GB_IN + TT_T / 2)
    can = zcyl(0, yc, TT_CAN_D / 2, TT_GB_LEN - TT_SHAFT_END, TT_GB_LEN - TT_SHAFT_END + TT_CAN_LEN) ^ \
        cbox(0, yc, 60, TT_CAN_D + 1, TT_CAN_FLAT, 200)
    holes = union([ycyl(x, z, 1.6, y0 - 1, y1 + 1) for x, z in TT_HOLES])
    stubs = ycyl(0, 0, TT_SHAFT_D / 2, min(side * (GB_IN - TT_STUB), side * (GB_OUT + TT_STUB)),
                 max(side * (GB_IN - TT_STUB), side * (GB_OUT + TT_STUB)))
    return (gb - holes), can, stubs

def wheel(side):
    y0, y1 = sorted((side * WHEEL_IN, side * (WHEEL_IN + W_WHEEL)))
    tyre = ycyl(0, 0, R_WHEEL, y0 + 1, y1 - 1, 96) - ycyl(0, 0, R_RIM, y0, y1, 96)
    tyre = tyre + (ycyl(0, 0, R_WHEEL - 1.0, y0, y1, 96) - ycyl(0, 0, R_RIM, y0 - 1, y1 + 1, 96))
    rim = ycyl(0, 0, R_RIM, y0, y1, 72) - ycyl(0, 0, R_RIM - 2, y0 + 3, y1 - 3, 72)
    # six spokes and a hub, with the two holes a cheap TT wheel has
    hub = ycyl(0, 0, 6.5, y0, y1)
    spokes = union([ycyl(0, 0, R_RIM - 1, (y0 + y1) / 2 - 1.5, (y0 + y1) / 2 + 1.5) -
                    union([ycyl(r * np.cos(a), r * np.sin(a), 5.2, y0 - 1, y1 + 1)
                           for a in np.linspace(0, 2 * np.pi, 6, endpoint=False) for r in (15.5,)])])
    return tyre, rim + hub + spokes

def sg90(shaft_axis):
    """SG-90 in its own frame: case bottom at h = 0, shaft at the origin of the (l, t) plane, pointing +h.
    Returned in (x, y, z) = (l, t, h) and rotated by the caller."""
    case = box(-S_SHAFT, S_LEN - S_SHAFT, -S_THK / 2, S_THK / 2, 0, S_H)
    tabs = box(-S_SHAFT - S_TAB_L, S_LEN - S_SHAFT + S_TAB_L, -S_THK / 2, S_THK / 2, S_TAB_Y, S_TAB_Y + S_TAB_T)
    tabs = tabs - union([zcyl(x, 0, 1.0, 0, 40) for x in (-S_SHAFT - 2.4, S_LEN - S_SHAFT + 2.4)])
    hump = zcyl(0, 0, S_HUMP_R, S_H, S_H + 4.0) + zcyl(0, 0, 2.4, S_H, S_SPLINE_TOP)
    horn = hull(zcyl(0, 0, HORN_HUB_R, S_SPLINE_TOP + 0.5 - 4.0, S_SPLINE_TOP + 0.5 + HORN_PLATE_T),
                zcyl(-HORN_LEN, 0, 2.0, S_SPLINE_TOP + 0.5, S_SPLINE_TOP + 0.5 + HORN_PLATE_T))
    return case + tabs + hump, horn

def place_tail_servo(m):
    """sg90 frame -> body: shaft along +y at TAIL_PIVOT, case runs FORWARD from the shaft (+l = +x)."""
    return m.rotate([-90, 0, 0]).translate([TAIL_PIVOT[0], TS_Y0, TAIL_PIVOT[1]])

def place_pan_servo(m):
    """sg90 frame -> body: shaft along +z at x = 0, case runs BACKWARD from the shaft."""
    return m.rotate([0, 0, 180]).translate([0, 0, PAN_Y0])

def pcb(x0, x1, y0, y1, z0, z1, col='pcb'):
    return box(x0, x1, y0, y1, z0, z1)


@dataclass
class Item:
    name: str
    solid: Manifold          # in its group's local frame
    colour: tuple
    group: str               # body, wheel_l, wheel_r, tail, head
    mass: float = 0.0        # g; 0 = computed from volume for printed parts
    material: str = ''       # 'PLA'/'ABS' for printed parts
    point: tuple = None      # mass-only items (no solid): centre of mass in the group frame
    source: str = ''


# =============================================================================================== printed parts
def motor_envelope(c=CLR):
    env = Manifold()
    for s in (1, -1):
        y0, y1 = sorted((s * (GB_IN - c), s * (GB_OUT + c)))
        env += box(-TT_W / 2 - c, TT_W / 2 + c, y0, y1, -TT_SHAFT_END - c, TT_GB_LEN - TT_SHAFT_END + TT_CAN_LEN + c)
        env += ycyl(0, 0, TT_SHAFT_D / 2 + 1.2, *sorted((s * (GB_IN - TT_STUB - 0.6), s * GB_IN)))
    return env

def tail_servo_pocket():
    case, horn = sg90(0)
    c = CLR
    pocket = box(-S_SHAFT - c, S_LEN - S_SHAFT + c, -S_THK / 2 - c, S_THK / 2 + c, -c, S_H + c)
    tabs = box(-S_SHAFT - S_TAB_L - c, S_LEN - S_SHAFT + S_TAB_L + c, -S_THK / 2 - c, S_THK / 2 + c,
               S_TAB_Y - c, S_TAB_Y + S_TAB_T + c)
    top = zcyl(0, 0, S_HUMP_R + 1.0, S_H - 1, 60) + zcyl(-8.5, 0, 3.6, S_H - 1, 60)  # hump + idler, horn side open
    screws = union([zcyl(x, 0, 0.8, S_TAB_Y - 8, S_TAB_Y + 3) for x in (-S_SHAFT - 2.4, S_LEN - S_SHAFT + 2.4)])
    wire = box(S_LEN - S_SHAFT - 2, S_LEN - S_SHAFT + 9, -3, 3, 1.0, 5.0)        # lead leaves the front end
    return place_tail_servo(pocket + tabs + top + screws + wire)

def core():
    """The motor web, hip block (tail servo), skid with heel and toe bumpers, IMU pad, encoder-sensor posts and
    the flange the torso bolts to. Printed as two halves split at y = 0 (each flat on its cut face)."""
    web = box(-15, 15, -WEB_HALF, WEB_HALF, -8, FLANGE_Z[1])
    bosses = union([ycyl(x, z, 4.5, -GB_IN, GB_IN) for x, z in TT_HOLES])
    hx, hz, hr = HEEL; tx, tz, tr = TOE
    skid = hull(ycyl(hx, hz, hr, -SKID_HW, SKID_HW), ycyl(tx, tz, tr, -SKID_HW, SKID_HW),
                ycyl(hx + 4, hz + 3, hr, -SKID_HW, SKID_HW), box(tx - 8, tx - 4, -SKID_HW, SKID_HW, tz - 1, tz + 4))
    # hip block: wraps the tail servo, boss for the pivot screw on the -y side
    px, pz = TAIL_PIVOT
    hip = xz_round(px - 12.5, -9.5, pz - 9, pz + 9, 3, TS_Y0 - 2.0, 5.0)
    hip += ycyl(px, pz, 5.5, FORK_PIVOT[1] + 0.4, TS_Y0 - 1.0)
    ribs = union([hull(box(-24, -10, y0, y1, pz + 6, pz + 9), box(-23.5, -12, y0, y1, FLANGE_Z[0], FLANGE_Z[1]))
                  for y0, y1 in ((-14, -11), (2, 5))])
    flange = box(-23.5, 23.5, -14, 14, FLANGE_Z[0], FLANGE_Z[1])
    imu_pad = box(14, 17.5, -12, 12, 18, 40)
    posts = union([box(15, 29, min(s * 20.0, s * 24.3), max(s * 20.0, s * 24.3), -12, 4) for s in (1, -1)])
    jx, jz, jr, jhw = JAW
    jaw = hull(ycyl(tx - 2, tz + 1, 5.5, -JAW_HW, JAW_HW), ycyl(tx + 6, tz + 6, 4.5, -JAW_HW, JAW_HW),
               ycyl(jx, jz, jr, -JAW_HW, JAW_HW)) + \
        hull(ycyl(jx, jz, jr, -jhw, jhw), ycyl(jx - 10, jz - 2, 4, -jhw + 4, jhw - 4))
    body = web + bosses + skid + hip + ribs + flange + imu_pad + posts + jaw
    # holes and pockets
    body -= motor_envelope()
    body -= tail_servo_pocket()
    body -= union([ycyl(x, z, 1.7, -40, 40) for x, z in TT_HOLES])                 # M3 x 70 motor bolts
    body -= ycyl(px, pz, 1.25, FORK_PIVOT[1] - 1, TS_Y0 - 0.5)                      # M3 shoulder screw, tapped
    body -= union([zcyl(x, y, 1.7, FLANGE_Z[0] - 8, FLANGE_Z[1] + 1) for x in (-18, 18) for y in (-9, 9)])
    body -= union([cbox(x, y, FLANGE_Z[0] + 1.0, 5.8, 5.8, 2.6) for x in (-18, 18) for y in (-9, 9)])  # nut traps
    body -= union([ycyl(22, -7, 1.6, -30, 30)])                                     # IR module M3 hole
    body -= box(-10, 10, -10, 10, 27, 43) + box(-10, 10, -10, 10, 52, 58)           # web windows (lightening)
    body -= union([box(x0, x1, s0, s1, -30, -4) for x0, x1 in ((-30, -14), (-6, 12))
                   for s0, s1 in ((5, 19), (-19, -5))])                             # skid windows (lightening)
    body -= box(-16, 16, -WEB_HALF - 1, WEB_HALF + 1, 48, 51.2) - box(-11, 11, -10, 10, -100, 100)   # can-strap slots
    body -= union([box(16, 17.6, y - 1.6, y + 1.6, 16, 42) for y in (-6, 6)])       # IMU zip-tie slots
    return body

def core_halves():
    c = core()
    left = c.trim_by_plane([0, 1, 0], 0.0)
    right = c.trim_by_plane([0, -1, 0], 0.0)
    return left, right

def torso():
    """Electronics spine (ESP32 behind, power boards in front), battery sleeve on top (holder slides in from
    the side), pan-servo block on the sleeve roof. Prints upright with supports under the sleeve floor."""
    hx, hy, hz = HOLDER
    w = TRAY_WALL
    base = box(-23.5, 23.5, -14, 14, FLANGE_Z[1], FLANGE_Z[1] + 3.0)              # sits on the core flange
    base -= union([zcyl(x, y, 1.7, FLANGE_Z[1] - 1, FLANGE_Z[1] + 5) for x in (-18, 18) for y in (-9, 9)])
    spine = box(-SPINE_T / 2, SPINE_T / 2, -SPINE_HW, SPINE_HW, FLANGE_Z[1], TRAY_Z0 + 1)
    edge = union([box(-5, 5, s * SPINE_HW - 2.5 if s > 0 else -SPINE_HW, s * SPINE_HW if s > 0 else -SPINE_HW + 2.5,
                      FLANGE_Z[1], TRAY_Z0 + 1) for s in (1, -1)])
    feet = union([hull(box(-SPINE_T / 2, SPINE_T / 2, y - 1.5, y + 1.5, FLANGE_Z[1], FLANGE_Z[1] + 18),
                       box(-22, 22, y - 1.5, y + 1.5, FLANGE_Z[1], FLANGE_Z[1] + 3)) for y in (-SPINE_HW + 1.5, SPINE_HW - 1.5)])
    xs0, xs1 = HOLDER_X - hx / 2 - CLR - w, HOLDER_X + hx / 2 + CLR + w
    sleeve = xz_round(xs0, xs1, TRAY_Z0, TRAY_TOP, 4, -hy / 2 - 1.5, hy / 2 + 1.5)
    sleeve -= box(xs0 + w, xs1 - w, -60, 60, TRAY_Z0 + w, TRAY_TOP - w)
    sleeve -= union([box(HOLDER_X - 12, HOLDER_X + 12, y - 6, y + 6, TRAY_Z0 - 1, TRAY_Z0 + w + 1) for y in (-24, 24)])
    under = union([hull(box(-SPINE_T / 2, SPINE_T / 2, y - 1.5, y + 1.5, TRAY_Z0 - 14, TRAY_Z0),
                        box(xs0 + 2, xs1 - 2, y - 1.5, y + 1.5, TRAY_Z0 - 0.5, TRAY_Z0)) for y in (-SPINE_HW + 1.5, SPINE_HW - 1.5)])
    # pan servo block on the roof: case pocket, tabs rest on its top face
    tab_z = PAN_Y0 + S_TAB_Y
    blk = xz_round(-S_LEN + S_SHAFT - S_TAB_L - 1.5, S_SHAFT + S_TAB_L + 1.5, TRAY_TOP - 1, tab_z, 3, -9.0, 9.0)
    pocket = box(-S_LEN + S_SHAFT - CLR, S_SHAFT + CLR, -S_THK / 2 - CLR, S_THK / 2 + CLR, PAN_Y0 - CLR, tab_z + 10)
    screws = union([zcyl(x, 0, 0.8, tab_z - 8, tab_z + 1) for x in (S_SHAFT + 2.4, -S_LEN + S_SHAFT - 2.4)])
    wire = box(-S_LEN + S_SHAFT - 6, -S_LEN + S_SHAFT + 2, -3, 3, PAN_Y0 - 1, PAN_Y0 + 6)
    t = base + spine + edge + feet + sleeve + under + blk
    t -= pocket + screws + wire
    # zip-tie slot pairs for the boards (3.2 x 1.6 through the spine)
    for zc, half in ((75, 14), (95, 14), (115, 14), (80, 22), (105, 22)):
        for y in (-half - 2.4, half + 2.4):
            t -= box(-5, 5, y - 0.8, y + 0.8, zc - 1.6, zc + 1.6)
    return t

def tail():
    """Tail in the tail frame: fork round the servo, bridge, C-shaped ABS leaf spring, foot pad."""
    hz0 = -17.0
    horn_arm = hull(ycyl(0, 0, 8.0, *FORK_HORN), box(-5, 5, *FORK_HORN, hz0 - 6, hz0))
    piv_arm = hull(ycyl(0, 0, 7.0, *FORK_PIVOT), box(-5, 5, *FORK_PIVOT, hz0 - 6, hz0))
    bridge = xz_round(-5, 5, LEAF_Z0 - 1, hz0, 2, FORK_PIVOT[0], FORK_HORN[1])
    # C-leaf: an arc through (0, LEAF_Z0) and (0, LEAF_Z1) bulging to x = +LEAF_BULGE
    half = (LEAF_Z0 - LEAF_Z1) / 2
    cxx = (LEAF_BULGE ** 2 - half ** 2) / (2 * LEAF_BULGE)     # centre x (negative)
    R = LEAF_BULGE - cxx
    zc = (LEAF_Z0 + LEAF_Z1) / 2
    ring = CrossSection.circle(R + LEAF_T / 2, 96) - CrossSection.circle(R - LEAF_T / 2, 96)
    ring = ring.translate([cxx, zc]) ^ CrossSection.square([40, 60]).translate([-0.8, zc - 30])
    leaf = xz_prism(ring, -LEAF_W / 2, LEAF_W / 2)
    foot = hull(box(-4, 4, -LEAF_W / 2, LEAF_W / 2, LEAF_Z1 - 4, LEAF_Z1 + 1),
                ycyl(0, -TAIL_L, FOOT_R, -LEAF_W / 2, LEAF_W / 2))
    pad = hull(ycyl(0, -TAIL_L, FOOT_R, -FOOT_HW, FOOT_HW), ycyl(1.5, -TAIL_L + 3.5, FOOT_R - 2, -FOOT_HW + 2, FOOT_HW - 2))
    t = horn_arm + piv_arm + bridge + leaf + foot + pad
    # SG-90 single-arm horn pocket on the inside of the horn arm, pointing down the tail
    hp = hull(ycyl(0, 0, HORN_HUB_R + 0.3, HORN_IN - 1, HORN_IN + HORN_PLATE_T + 0.2),
              ycyl(0, -HORN_LEN, 2.3, HORN_IN - 1, HORN_IN + HORN_PLATE_T + 0.2))
    t -= hp
    t -= ycyl(0, 0, 2.4, HORN_IN, 40)                                       # horn screw access
    t -= union([ycyl(0, -z, 0.9, HORN_IN, 40) for z in (8.5, 11.5, 14.0)])  # horn screws (2 used)
    t -= ycyl(0, 0, 1.7, FORK_PIVOT[0] - 1, FORK_PIVOT[1] + 1)               # M3 shoulder
    return t

def head():
    """Head shell in the head frame (pan horn top at the origin): base plate, open back, camera window."""
    h = HEAD
    shell = xz_round(h['x0'], h['x1'], h['z0'], h['z1'], 8, -h['hw'], h['hw'])
    inner = xz_round(h['x0'] - 5, h['x1'] - h['wall'], h['z0'] + 2.4, h['z1'] - h['wall'], 6,
                     -h['hw'] + h['wall'], h['hw'] - h['wall'])
    s = shell - inner
    s -= xcyl(0, 31.0, 5.2, h['x1'] - 4, h['x1'] + 2)                        # camera lens window
    s -= xz_round(h['x0'] - 1, h['x0'] + 2.5, h['z0'] + 6, h['z1'] - 6, 3, -h['hw'] + 5, h['hw'] - 5)  # open back
    # board slots: ESP32-CAM slides down between two ridges, lens forward
    for y in (-CAM_PCB[0] / 2 - 1.2, CAM_PCB[0] / 2 + 1.2):
        s += box(5.2, 6.2, y - 1.2, y + 1.2, h['z0'] + 2.4, h['z1'] - 4) + box(8.4, 9.4, y - 1.2, y + 1.2, h['z0'] + 2.4, h['z1'] - 4)
    # base: horn pocket and screw holes
    s += box(-8, 8, -9, 9, h['z0'], h['z0'] + 3.5)
    s -= hull(zcyl(0, 0, HORN_HUB_R + 0.3, -1, h['z0'] + 1.0), zcyl(HORN_LEN, 0, 2.3, -1, h['z0'] + 1.0))
    s -= zcyl(0, 0, 2.4, -1, 10)
    # eye ring and brow ridges, to read as a face
    s += xcyl(0, 31.0, 8.5, h['x1'] - 1.0, h['x1'] + 1.0) - xcyl(0, 31.0, 5.2, h['x1'] - 2, h['x1'] + 3)
    for y in (-11.0, 11.0):
        s += hull(xcyl(y, 43.5, 1.6, h['x1'] - 3, h['x1'] + 0.8), xcyl(y * 0.55, 45.0, 1.6, h['x1'] - 3, h['x1'] + 0.8))
    return s

def encoder_ring():
    r = ycyl(0, 0, RING_R1, 0, RING_T, 96) - ycyl(0, 0, RING_R0 - 4, -1, RING_T + 1, 64)
    for k in range(RING_SLOTS):
        a = 2 * np.pi * (k + 0.5) / RING_SLOTS
        w = Manifold.cube([RING_R1 - RING_R0, RING_T + 2, 2 * np.pi * 21.5 / RING_SLOTS / 2], center=True)
        r -= w.translate([(RING_R0 + RING_R1) / 2, RING_T / 2, 0]).rotate([0, -np.degrees(a), 0])
    return r

PRINTED = {   # name: (builder, material, count, print note)
    'core_left':    (lambda: core_halves()[0], 'PLA', 1, 'flat on the cut face (y = 0 down); no supports'),
    'core_right':   (lambda: core_halves()[1], 'PLA', 1, 'flat on the cut face (y = 0 down); no supports'),
    'torso':        (torso, 'PLA', 1, 'upright (base down); tree supports under the battery-sleeve floor'),
    'tail':         (tail, 'ABS', 1, 'on the horn-arm face (y max down) so the leaf layers run along the bend; tree supports under the pivot arm'),
    'head':         (head, 'PLA', 1, 'on its open back (x min down); no supports'),
    'encoder_ring': (encoder_ring, 'PLA', 2, 'flat; print in white or light PLA'),
}


# =============================================================================================== assembly
def tail_tf(alpha):
    """tail frame -> body frame."""
    return T(TAIL_PIVOT[0], 0, TAIL_PIVOT[1]) @ Ry(alpha)

def head_tf(pan):
    return T(0, 0, PAN_TOP) @ Rz(pan)

def T(x, y, z):
    m = np.eye(4); m[:3, 3] = (x, y, z); return m

def Ry(deg):
    a = np.radians(deg); c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]], float)

def Rz(deg):
    a = np.radians(deg); c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], float)

def apply(m, M):
    return m.transform(M[:3, :4].tolist())

def items():
    """Every printed and bought part in its group's local frame, with mass (g)."""
    it = []
    L, R = core_halves()
    it += [Item('core_left', L, COL['body'], 'body', material='PLA'),
           Item('core_right', R, COL['body'], 'body', material='PLA'),
           Item('torso', torso(), COL['body2'], 'body', material='PLA'),
           Item('tail', tail(), COL['tail'], 'tail', material='ABS'),
           Item('head', head(), COL['head'], 'head', material='PLA')]
    for s, g in ((1, 'wheel_l'), (-1, 'wheel_r')):
        gb, can, stub = tt_motor(s)
        it += [Item(f'tt_gearbox_{g[-1]}', gb, COL['tt'], 'body', 22.0, source='[E] TT motor ~29 g total'),
               Item(f'tt_can_{g[-1]}', can, COL['can'], 'body', 7.0, source='[E]'),
               Item(f'tt_shaft_{g[-1]}', stub, COL['horn'], g, 0.5, source='[E]')]
        ty, rim = wheel(s)
        it += [Item(f'tyre_{g[-1]}', ty, COL['tyre'], g, 10.0, source='[E] 65 mm TT wheel ~22 g'),
               Item(f'rim_{g[-1]}', rim, COL['rim'], g, 12.0, source='[E]')]
        ring = encoder_ring()
        ring = ring.translate([0, WHEEL_IN - RING_T, 0]) if s > 0 else ring.mirror([0, 1, 0]).translate([0, -WHEEL_IN + RING_T, 0])
        it += [Item(f'encoder_ring_{g[-1]}', ring, COL['ring'], g, material='PLA')]
        # IR reflective module: PCB in the x-z plane, TCRT5000 pointing out at the ring at (x 22, z 0)
        yp = s * 24.3
        it += [Item(f'ir_pcb_{g[-1]}', box(15, 29, min(yp, yp + s * 1.6), max(yp, yp + s * 1.6), -10, 22),
                    COL['pcb_blue'], 'body', 2.5, source='[E] TCRT5000 module 32 x 14 mm, 3 g'),
               Item(f'ir_tcrt_{g[-1]}', box(19, 25, min(yp + s * 1.6, yp + s * 8.6), max(yp + s * 1.6, yp + s * 8.6), -3, 3),
                    COL['chip'], 'body', 0.5, source='[E]')]
    # tail servo + horn (horn moves with the tail)
    case, horn = sg90(0)
    it += [Item('tail_servo', place_tail_servo(case), COL['servo'], 'body', 9.0, source='[D] SG-90 9 g')]
    hornT = apply(place_tail_servo(horn), np.linalg.inv(tail_tf(0)))
    hornT = apply(hornT, Ry(-90))      # sg90 horn points -l (backwards); rotate so it lies down the tail
    it += [Item('tail_horn', hornT, COL['horn'], 'tail', 0.6)]
    # pan servo + horn (horn moves with the head)
    case, horn = sg90(0)
    it += [Item('pan_servo', place_pan_servo(case), COL['servo'], 'body', 9.0, source='[D] SG-90 9 g')]
    it += [Item('pan_horn', apply(place_pan_servo(horn), np.linalg.inv(head_tf(0))), COL['horn'], 'head', 0.6)]
    # battery holder with two cells, in the sleeve on top of the spine
    hx, hy, hz = HOLDER
    zb = TRAY_Z0 + TRAY_WALL + 0.3
    it += [Item('holder', box(HOLDER_X - hx / 2, HOLDER_X + hx / 2, -hy / 2, hy / 2, zb, zb + hz) -
                box(HOLDER_X - hx / 2 + 1.2, HOLDER_X + hx / 2 - 1.2, -hy / 2 + 4, hy / 2 - 4, zb + 1.2, zb + hz + 1),
                COL['holder'], 'body', 14.0, source='[E] 2 x 18650 holder'),
           Item('cells', union([ycyl(HOLDER_X + x, zb + 10.2, 9.2, -32.5, 32.5) for x in (-10.0, 10.0)]),
                COL['cell'], 'body', 92.0, source='[E] 18650 ~46 g each')]
    # electronics on the spine: front face (+x) power, back face (-x) ESP32 (module towards the spine)
    f = SPINE_T / 2
    it += [Item('esp32', box(-f - 5.6, -f - 4.0, -14, 14, 67, 122) + box(-f - 4.0, -f - 0.8, -9, 9, 99, 117)
                + box(-f - 14.0, -f - 5.6, -13.5, -11, 69, 120) + box(-f - 14.0, -f - 5.6, 11, 13.5, 69, 120),
                COL['chip'], 'body', 10.0, source='[E] 38-pin DevKit 55 x 28 mm, 10 g'),
           Item('mx1508', box(f + 0.8, f + 2.4, -12, 12, 102.5, 123.5) + box(f + 2.4, f + 4.4, -4, 4, 109, 116),
                COL['pcb'], 'body', 3.0, source='[E] MX1508 board 24 x 21 mm'),
           Item('mini360_logic', box(f + 0.8, f + 5.0, -25.3, -8.1, 90, 101.2), COL['pcb_blue'], 'body', 2.0, source='[D] 17 x 11 mm'),
           Item('mini360_servo', box(f + 0.8, f + 5.0, 8.1, 25.3, 90, 101.2), COL['pcb_blue'], 'body', 2.0, source='[D]'),
           Item('bms_2s', box(f + 0.8, f + 4.3, -24.8, 24.8, 67.4, 89.4), COL['pcb'], 'body', 4.0,
                source='[D] HX-2S-JH20 balancing 2S BMS, about 50 x 22 x 3.5 mm'),
           Item('cap_470u', box(f + 0.8, f + 2.4, 12.6, 24.8, 103, 123) + xcyl(18.7, 113, 4.0, f + 2.4, f + 14.4),
                COL['cap'], 'body', 3.0, source='[E] junction board (5 x 8 holes) with the 470 uF servo-rail capacitor'),
           Item('switch', box(W_SLEEVE_FRONT, W_SLEEVE_FRONT + 1.6, -7.6, 7.6, 128.5, 143.7)
                + box(W_SLEEVE_FRONT + 1.6, W_SLEEVE_FRONT + 4.0, -2.5, 2.5, 133.6, 138.6), COL['pcb_blue'], 'body', 1.5,
                source='[D] Pololu 2810 mini MOSFET slide switch, 15.2 x 15.2 mm, on the front of the battery sleeve'),
           Item('imu', box(17.6, 19.2, -10.5, 10.5, 19, 35) + box(19.2, 20.2, -2, 2, 25, 29), COL['pcb_blue'], 'body', 2.5,
                source='[E] GY-521 21 x 16 mm, 2.5 g')]
    # head: ESP32-CAM in its slots, camera forward
    h = HEAD
    it += [Item('esp32cam', box(6.4, 8.0, -CAM_PCB[0] / 2, CAM_PCB[0] / 2, h['z0'] + 3.6, h['z0'] + 3.6 + CAM_PCB[1])
                + box(8.0, 12.0, -4.5, 4.5, 26.5, 35.5) + xcyl(0, 31.0, 4.0, 12.0, 15.6), COL['chip'], 'head', 10.0,
                source='[E] ESP32-CAM with OV2640, ~10 g')]
    # mass-only items
    it += [Item('wiring_and_connectors', None, None, 'body', 15.0, point=(0, 0, 95), source='[E]'),
           Item('screws_nuts_ties', None, None, 'body', 9.0, point=(-5, 0, 30), source='[E] 2 x M3x70, 4 x M3x10, M3 shoulder, M2 tab screws, ties'),
           Item('motor_leads', None, None, 'body', 3.0, point=(0, 0, 50), source='[E]')]
    return it

def item_mass(i):
    if i.mass:
        return i.mass
    rho = PLA if i.material == 'PLA' else ABS
    return i.solid.volume() / 1000.0 * rho * FILL

def item_com(i):
    return np.array(i.point, float) if i.solid is None else centroid(i.solid)


# =============================================================================================== kinematics
def group_tfs(X, pitch, tail_angle, pan, wheel_spin=None):
    """World transforms of each group. X = axle position along the floor (mm), pitch (deg, + forward)."""
    Tb = T(X, 0, R_WHEEL) @ Ry(pitch)
    psi = np.degrees(X / R_WHEEL) if wheel_spin is None else wheel_spin
    Tw = T(X, 0, R_WHEEL) @ Ry(psi)
    return dict(body=Tb, wheel_l=Tw, wheel_r=Tw, tail=Tb @ tail_tf(tail_angle), head=Tb @ head_tf(pan))

def foot_centre_body(alpha):
    return (tail_tf(alpha) @ np.array([0, 0, -TAIL_L, 1.0]))[:3]

def tail_angle_for_foot_on_floor(pitch, branch=+1):
    """Tail angle that puts the foot pad on the floor at this body pitch (pad centre at height FOOT_R)."""
    px, pz = TAIL_PIVOT
    P = (Ry(pitch) @ np.array([px, 0, pz, 1.0]))[:3]
    h = R_WHEEL + P[2] - FOOT_R          # pivot height above the pad-centre plane
    if h > TAIL_L:
        return None
    phi = np.degrees(np.arccos(h / TAIL_L))  # angle of the tail from straight down (world), + backwards
    return branch * phi - pitch               # body tail angle (world angle minus body pitch... sign below)

def tail_ready():
    """Tail angle held while balancing: the foot lands on the floor if the body falls back to READY_PITCH."""
    return tail_angle_for_foot_on_floor(READY_PITCH)

def foot_world(X, pitch, alpha):
    return (T(X, 0, R_WHEEL) @ Ry(pitch) @ np.append(foot_centre_body(alpha), 1.0))[:3]
