"""Stage 1: the bare-minimum balancer. One dual-shaft TT motor, a wheel on each end of its shaft, two 18650s on
top, an ESP32, an MPU-6050 and an MX1508. Its only job: prove a TT motor is fast enough to keep a body upright
with a PID loop.

This file is the geometry, the mass table and the design check. Bought-part dimensions come from the published
Tailwhip CAD (tailwhip.py). Run:  python stage1.py   (--no-render skips the pictures)
Writes out/design.md, out/print/*.stl (each in its print orientation, on the bed at z = 0), out/render_*.png,
and out/model.json, out/body.stl, out/wheels.stl (read by sim.py).

FRAME (mm): origin on the axle midway between the wheels, x forward, y left, z up the body.
SOURCES: every number is tagged [D] datasheet, [E] estimate (measure it), [C] chosen by this design.
Nothing here has been printed, built or measured."""
import os, sys, json
from dataclasses import dataclass
import numpy as np
from manifold3d import Manifold

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'cad'))
import tailwhip as W

OUT = os.path.join(HERE, 'out'); os.makedirs(os.path.join(OUT, 'print'), exist_ok=True)
C, CLR, R = W.COL, W.CLR, W.R_WHEEL

# =============================================================================================== dimensions, mm
# --- motor, wheels and the cheeks between them
GB_HW = W.TT_T / 2                 # 9.4   [E] gearbox half thickness along the shaft
CLR_Y = 0.1                        # [C] gearbox pocket clearance along the shaft (the cheeks locate it in y)
POCKET_HW = GB_HW + CLR_Y          # 9.5
CHEEK = 2.3                        # [C] cheek plate between gearbox and wheel (was 2.1)
CH_HW = POCKET_HW + CHEEK          # 11.8  half width of everything between the wheels
WHEEL_GAP = 1.2                    # [C] cheek face to wheel inner face (was 0.8)
WHEEL_IN = CH_HW + WHEEL_GAP       # 13.0  wheel inner face |y|
HUB_ON = GB_HW + W.TT_STUB - WHEEL_IN   # 5.5 mm of the 9.1 mm stub inside the wheel hub
TRACK = 2 * (WHEEL_IN + W.W_WHEEL / 2)
GB_Z0, GB_Z1 = -W.TT_SHAFT_END, W.TT_GB_LEN - W.TT_SHAFT_END     # -11, 26
CAN_Z1 = GB_Z1 + W.TT_CAN_LEN      # 59   top of the motor can
TAB_ROOM = 5.0                     # [E] room above the can for the solder tabs, the leads and C3 (100 nF)
SHAFT_HOLE_R = 4.0                 # [C] shaft hole in the cheeks: clears the 5.4 shaft and a bearing boss up to 7.4 [E]
PIN_R, PIN_L = 1.5, 22.0           # [C] 3 mm steel pin through the gearbox's two M3 holes, ends 0.8 below the cheek faces
# --- core block and crash bars
CORE_X, CORE_Z0, CORE_Z1, CORE_RND = 23.0, -15.0, 72.0, 4.0   # [C] half length, bottom, top, edge radius
SKID_X, SKID_Z, SKID_R, SKID_HW = 80.0, 6.5, 5.0, 30.0        # [C] crash bars front and back (the sim uses these)
# --- fasteners (ISO metric; [D] nominal sizes, [C] holes)
BOLTS = [(-17.0, -5.0), (17.0, -5.0), (-17.0, 40.0), (17.0, 40.0)]   # [C] 4 x M3 x 20 clamp bolts (x, z), along y
BOLT_L, HEAD_D, HEAD_H, NUT_AF, NUT_H = 20.0, 5.7, 1.65, 5.5, 2.4   # [D] ISO 7380 button head, ISO 4032 nut (socket heads fit milestone A)
HEAD_REC_R, NUT_REC_AF, REC_DEPTH = 3.1, 5.8, 3.2                  # [C] recesses: head 6.2 dia, nut 5.8 across flats
M3_CLEAR_R, M3_PILOT_R, M25_PILOT_R = 1.7, 1.25, 1.05              # [C] 3.4 clearance, 2.5 / 2.1 self-tap pilots in PLA
TUN_W, TUN_H, TUN_BAR = 1.8, 3.0, 1.8                              # [C] zip-tie tunnels for 2.5 mm ties, 1.8 mm bar
# --- mast (one print: foot, plate, flanges, gusset, board mounts, tray bosses)
FOOT_Z0, FOOT_Z1 = CORE_Z1, CORE_Z1 + 4.0
MAST_SCREWS = [(x, y) for x in (-16.0, 16.0) for y in (-7.0, 7.0)]  # [C] 4 x M3 x 12 self-tap into the core top
PLATE_T, MAST_HW, MAST_TOP = 4.0, 31.0, 132.0                       # [C] plate thickness (was 3), half width, top
PF = PLATE_T / 2
FL_T, FL_D = 2.4, 8.0              # [C] edge flanges: thickness, depth off the plate
GUSSET_T, GUSSET_TOP = 3.0, 97.0   # [C] back root gusset at y = 0: thickness, top (45 deg)
STRAP_W = 10.0                     # [C] hook-and-loop battery strap width
# --- battery tray (separate print) and holder
HOLDER = W.HOLDER                  # [E] 2 x 18650 series holder: 41 x 77.5 x 20.5
HOLDER_FLOOR = 1.2                 # [E] holder floor thickness
TRAY_Z0, FLOOR_T, TRAY_WALL = MAST_TOP, 3.0, 2.0
TRAY_Z1 = TRAY_Z0 + 14.0
TX = HOLDER[0] / 2 + CLR + TRAY_WALL        # 22.8
TY = HOLDER[1] / 2 + CLR + TRAY_WALL        # 41.05
HOLDER_SCREWS = [(0.0, -18.0), (0.0, 18.0)]  # [E] holder floor holes on its centre line. MEASURE (drill if they differ)
# --- boards (all [E] unless tagged: typical listings, measure yours). Positions follow docs/electronics.md.
ESP = (55.0, 28.0, 1.6)            # [E] ESP32 DevKit (38-pin, WROOM-32) L x W x PCB. 30-pin boards are ~51 x 28
ESP_GAP = 6.0                      # [C] PCB to plate (module side towards the plate, header pins outwards)
ESP_Y0, ESP_Z0 = -ESP[0] / 2, 100.0          # back of the mast, across, USB end at y +27.5 (robot left)
ESP_ZC = ESP_Z0 + ESP[1] / 2
ESP_TIES = (ESP_ZC - 9.5, ESP_ZC + 9.5)      # [C] two ties between the header rows, clear of the USB plug
BACK_TIE_Z = 86.3                  # [C] MX1508 and Mini360 ties: both pass through the gusset root at this height
IMU = (21.0, 16.0, 1.6)            # [E] GY-521; 2 holes ~3 mm, 15 mm apart, 2.5 mm in from the edge opposite the header
IMU_Z0, IMU_POST = 16.0, 2.0       # drawn landscape (21 wide x 16 tall); the posts also take it portrait (see docs)
IMU_HOLES = [(-7.5, IMU_Z0 + 2.5), (7.5, IMU_Z0 + 2.5)]                    # landscape: header along the top edge
IMU_PILOTS = IMU_HOLES + [(-7.5, IMU_Z0 + 17.5), (7.5, IMU_Z0 + 17.5)]     # + portrait: holes up one side, 15 apart
MX = (24.7, 21.0)                  # [E] MX1508 board y x z (no holes), back of the mast, robot right
MX_Y0, MX_Z0 = -27.0, 76.5
MINI = (17.2, 11.2)                # [E] Mini360 buck (no holes), back of the mast, robot left
MINI_Y0, MINI_Z0 = 4.0, 80.0
BMS = (50.0, 22.0)                 # [E] 2S BMS, HX-2S-JH20 class, ~50 x 22 x 3.5 (no holes), front top
BMS_Z0 = 101.0
SW = (15.2, 15.2)                  # [E] Pololu 2810 switch board, front, robot left, slider forward
SW_Y0, SW_Z0 = 6.0, 77.0
JB = (21.0, 13.0)                  # [E] junction perfboard 8 x 5 holes at 2.54 (cut to ~21 x 13), front, robot right
JB_Y0, JB_Z0 = -25.5, 79.0
XH = (7.5, 5.8, 18.0)              # [E] JST-XH 2-pin plug and socket mated: width, depth, length with wire strain relief
J1 = (5.5, 14.0)                   # [E] DC 5.5 x 2.1 pigtail jack body: radius, length
J1_POS = (PF + FL_D + 5.8, MAST_HW - 3.0, 100.0)   # [C] socket mouth (pointing down), tied to the front edge of the robot-left flange
# --- milestone B: one wheel sensor. A TCRT5000 reads a printed 16-window ring glued to the inside face of the LEFT
# wheel. The only part of that face the core does not hide is below the core, so the bare TCRT5000 sits in a clip
# under the core (moved off its module on a 4-wire pigtail); the module board goes on the back of the core.
RING_T = 0.6                       # [C] ring thickness: 3 layers at 0.2 (the published robot's 1.2 mm does not fit)
RING_R0, RING_R1, RING_N = 17.5, 25.0, 16   # [C] as the published robot; windows are sectors, so 50 % at every radius
RING_Y0 = WHEEL_IN - RING_T        # 12.4  ring face (towards the core)
RING_WIN = (RING_R0 + 1.2, RING_R1 - 1.2)   # [C] windows run from r 18.7 to 23.8
RECESS, RECESS_R0, RECESS_R1 = 0.6, 16.0, 26.3   # [C] shallow annular recess in the left cheek face under the ring
TCRT = (10.2, 5.8, 7.0)            # [D] Vishay TCRT5000 body: length, width, height (lenses to lead end)
TCRT_GAP = 2.5                     # [D] Vishay: peak operating distance 2.5 mm (operating range 0.2-15 mm)
TCRT_R = 20.5                      # [C] radius read on the ring (track 17.5-25): sensor centre at (0, -TCRT_R)
TCRT_FACE = RING_Y0 - TCRT_GAP     # 9.9   lens face
TCRT_BACK = TCRT_FACE - TCRT[2]    # 2.9   lead end
MOD = (14.0, 32.0, 1.6)            # [E] TCRT5000 line-follower module PCB: width, length, thickness
MOD_Z0 = 14.0                      # [C] on the back of the core, y -7..7, z 14..46, components facing back
STEEL = 7.85e-3                    # g/mm3
E_PLA, S_ALONG, S_ACROSS = 2300.0, 40.0, 20.0   # [E] printed PLA: modulus, strength along / across layers (MPa)

@dataclass
class It:
    name: str
    solid: object
    colour: tuple
    group: str            # body | wheels
    mass: float = 0.0     # g; 0 = printed, from volume
    point: tuple = None
    source: str = ''
    material: str = ''
    kind: str = ''        # printed | bought | board | fastener | wire | mass

# =============================================================================================== helpers
def fb(s, xf, d0, d1, y0, y1, z0, z1):
    """Box on the face x = s*xf: from d0 to d1 outwards (s = +1 front face, -1 back face)."""
    a, b = s * (xf + d0), s * (xf + d1)
    return W.box(min(a, b), max(a, b), y0, y1, z0, z1)

def hexy(x, z, af, y0, y1):
    r = af / np.sqrt(3)
    return Manifold.cylinder(y1 - y0, r, r, 6).rotate([0, 0, 30]).rotate([-90, 0, 0]).translate([x, y0, z])

def anchor(s, xf, yc, zc, axis='y', half=1.3):
    """Zip-tie anchor on a vertical face: a block 3.6 proud with a 1.8 x 3.0 tunnel at the face, along y or z.
    Returns (block, tunnel). In an upright print the tunnel roof spans only 1.8 mm."""
    if axis == 'y':
        blk = fb(s, xf, -0.5, TUN_W + TUN_BAR, yc - half, yc + half, zc - 3.5, zc + 3.5)
        tun = fb(s, xf, 0.0, TUN_W, yc - 5, yc + 5, zc - TUN_H / 2, zc + TUN_H / 2)
    else:
        blk = fb(s, xf, -0.5, TUN_W + TUN_BAR, yc - 3.5, yc + 3.5, zc - half, zc + half)
        tun = fb(s, xf, 0.0, TUN_W, yc - TUN_H / 2, yc + TUN_H / 2, zc - 5, zc + 5)
    return blk, tun

def tube(pts, r=1.3):
    pts = [np.array(p, float) for p in pts]
    sph = [Manifold.sphere(r, 16).translate(p.tolist()) for p in pts]
    return W.union([W.hull(a, b) for a, b in zip(sph[:-1], sph[1:])])

def plen(pts):
    return float(sum(np.linalg.norm(np.subtract(b, a)) for a, b in zip(pts[:-1], pts[1:])))

# =============================================================================================== bought parts
def motor():
    gb = W.box(-W.TT_W / 2, W.TT_W / 2, -GB_HW, GB_HW, GB_Z0, GB_Z1)
    gb -= W.union([W.ycyl(x, z, 1.6, -GB_HW - 1, GB_HW + 1) for x, z in W.TT_HOLES])
    can = W.zcyl(0, 0, W.TT_CAN_D / 2, GB_Z1, CAN_Z1) ^ W.cbox(0, 0, 60, W.TT_CAN_D + 1, W.TT_CAN_FLAT, 200)
    shaft = W.ycyl(0, 0, W.TT_SHAFT_D / 2, -GB_HW - W.TT_STUB, GB_HW + W.TT_STUB)
    return gb, can, shaft

def wheels():
    tl, rl = W.wheel(1); tr, rr = W.wheel(-1)
    d = WHEEL_IN - W.WHEEL_IN
    return (tl.translate([0, d, 0]), rl.translate([0, d, 0])), (tr.translate([0, -d, 0]), rr.translate([0, -d, 0]))

def wheel_envelope():
    """Everything a wheel (and the encoder ring on the left one) sweeps in a full turn."""
    env = W.union([W.ycyl(0, 0, R, *sorted((s * WHEEL_IN, s * (WHEEL_IN + W.W_WHEEL))), 96) for s in (1, -1)])
    return env + (W.ycyl(0, 0, RING_R1, RING_Y0, WHEEL_IN + 0.1, 96) - W.ycyl(0, 0, RING_R0, RING_Y0 - 1, WHEEL_IN + 1, 96))

# =============================================================================================== printed parts
def core():
    """Clamshell round the gearbox and can, split at y = 0 and printed flat on the cut face. Two 3 mm steel pins
    through the gearbox's M3 holes locate it; four M3 x 20 bolts (heads and nuts recessed below the cheek faces)
    clamp the halves. Crash bars front and back, IMU posts on the front face, tie
    tunnels for the wires and the K1 plug, motor-lead exit at the back."""
    blk = W.xz_round(-CORE_X, CORE_X, CORE_Z0, CORE_Z1, CORE_RND, -CH_HW, CH_HW)
    arms = Manifold()
    for s in (1, -1):
        arms += W.hull(W.cbox(s * (CORE_X - 2), 0, 3.0, 4, 16, 16), W.ycyl(s * SKID_X, SKID_Z, 4.5, -8, 8))
        arms += W.hull(W.ycyl(s * SKID_X, SKID_Z, 4.0, -SKID_HW, SKID_HW), W.ycyl(s * (SKID_X - 26), SKID_Z - 1, 3.5, -8, 8))
        arms += W.ycyl(s * SKID_X, SKID_Z, SKID_R, -SKID_HW, SKID_HW)
    # IMU: two posts under its holes and a rest pad under its top edge (below the header pins)
    # IMU: four posts (landscape uses the lower two, portrait the two on one side) and a rest pad clear of
    # the header joints in either orientation
    feat = W.union([W.xcyl(y, z, 3.0, CORE_X - 0.5, CORE_X + IMU_POST) for y, z in IMU_PILOTS])
    feat += W.box(CORE_X - 0.5, CORE_X + IMU_POST, -2.0, 2.0, IMU_Z0 + 9.5, IMU_Z0 + 12.5)
    # milestone B (printed for milestone A too, harmless): the TCRT5000 clip under the core, and rails for the module
    zc = -TCRT_R
    feat += W.box(-TCRT[1] / 2 - CLR - 1.8, TCRT[1] / 2 + CLR + 1.8, 0.0, TCRT_FACE - 0.5, zc - TCRT[0] / 2 - CLR - 1.8, CORE_Z0 + 1.0)
    feat += W.union([fb(-1, CORE_X, -0.5, 1.6, y0, y1, MOD_Z0 + 1, MOD_Z0 + MOD[1] - 1) for y0, y1 in ((-MOD[0] / 2, -MOD[0] / 2 + 2), (MOD[0] / 2 - 2, MOD[0] / 2))])
    body = blk + arms + feat
    # ---- cavities
    cav = W.box(-W.TT_W / 2 - CLR, W.TT_W / 2 + CLR, -POCKET_HW, POCKET_HW, GB_Z0 - CLR, GB_Z1 + CLR)
    flat = W.TT_CAN_FLAT / 2 + CLR
    cav += W.zcyl(0, 0, W.TT_CAN_D / 2 + CLR, GB_Z1, CAN_Z1 + CLR) ^ W.box(-20, 20, -flat, flat, -100, 200)
    cav += W.zcyl(0, 0, W.TT_CAN_D / 2 + CLR, CAN_Z1, CAN_Z1 + TAB_ROOM) ^ W.box(-20, 20, -6, 6, -100, 200)   # solder tabs
    cav += W.box(-CORE_X - 1, -6, -3.5, 3.5, CAN_Z1, CAN_Z1 + TAB_ROOM)            # motor leads out of the back
    cav += W.ycyl(0, 0, SHAFT_HOLE_R, -CH_HW - 1, CH_HW + 1)
    cav += W.union([W.ycyl(x, z, PIN_R, -CH_HW - 1, CH_HW + 1) for x, z in W.TT_HOLES])
    for x, z in BOLTS:
        cav += W.ycyl(x, z, M3_CLEAR_R, -CH_HW - 1, CH_HW + 1)
        cav += W.ycyl(x, z, HEAD_REC_R, CH_HW - REC_DEPTH, CH_HW + 1)               # heads on the left (+y) half
        cav += hexy(x, z, NUT_REC_AF, -CH_HW - 1, -CH_HW + REC_DEPTH)              # nuts on the right half
    cav += W.union([W.zcyl(x, y, M3_PILOT_R, CORE_Z1 - 10, CORE_Z1 + 1) for x, y in MAST_SCREWS])
    cav += W.union([W.xcyl(y, z, M25_PILOT_R, CORE_X + IMU_POST - 8, CORE_X + IMU_POST + 1) for y, z in IMU_PILOTS])
    # milestone B: TCRT5000 pocket (open towards the ring), lead slot out of the back of the clip to y < 0,
    # cheek recess under the ring (left half only), wire hole through the back crash-arm root
    cav += W.box(-TCRT[1] / 2 - CLR, TCRT[1] / 2 + CLR, TCRT_BACK, 30, zc - TCRT[0] / 2 - CLR, zc + TCRT[0] / 2 + CLR)
    cav += W.box(-1.8, 1.8, -1, TCRT_BACK + 0.01, zc - 4.0, zc + 4.0)
    rec = W.ycyl(0, 0, RECESS_R1, CH_HW - RECESS, CH_HW + 1, 96) + W.box(-50, 50, CH_HW - RECESS, CH_HW + 1, -50, -13.0)
    rec += W.union([W.box(*sorted((s * 19.5, s * 50)), CH_HW - RECESS, CH_HW + 1, -14.0, 14.0) for s in (1, -1)])
    cav += rec - W.ycyl(0, 0, RECESS_R0, CH_HW - RECESS - 1, CH_HW + 2, 96)     # clean exits through the faces, no slivers
    cav += W.box(-CORE_X - 4.4, -CORE_X - 1.4, -3.5, 0.5, -12, 14)
    for s, zs in ((1, (48.0, 63.0)), (-1, (17.0, 33.0, 49.0, 67.5))):             # tie tunnels along y
        for z in zs:
            cav += fb(s, CORE_X, -TUN_BAR - TUN_W, -TUN_BAR, -CH_HW - 1, CH_HW + 1, z - TUN_H / 2, z + TUN_H / 2)
    # 0.4 mm chamfer round the pocket mouth on the cut face (the bed face): stops elephant's foot pinching the gearbox
    e = 0.4
    cav += W.hull(W.box(-W.TT_W / 2 - CLR - e, W.TT_W / 2 + CLR + e, -0.01, 0.01, GB_Z0 - CLR - e, GB_Z1 + CLR),
                  W.box(-W.TT_W / 2 - CLR, W.TT_W / 2 + CLR, -e, e, GB_Z0 - CLR, GB_Z1 + CLR))
    cav += W.hull(W.zcyl(0, 0, W.TT_CAN_D / 2 + CLR + e, GB_Z1, CAN_Z1 + CLR) ^ W.box(-30, 30, -0.01, 0.01, -100, 200),
                  W.zcyl(0, 0, W.TT_CAN_D / 2 + CLR, GB_Z1, CAN_Z1 + CLR) ^ W.box(-30, 30, -e, e, -100, 200))
    return body - cav

def core_halves():
    c = core()
    return c ^ W.box(-200, 200, 0, 200, -200, 200), c ^ W.box(-200, 200, -200, 0, -200, 200)

def mast():
    """Foot, plate, edge flanges, a 45 deg root gusset behind, board mounts on both faces, and two bosses on top
    for the screws that hold the battery holder and tray. Prints upright on the foot, no supports."""
    foot = W.box(-CORE_X, CORE_X, -CH_HW, CH_HW, FOOT_Z0, FOOT_Z1)
    plate = W.box(-PF, PF, -MAST_HW, MAST_HW, FOOT_Z0, MAST_TOP)
    gus = W.hull(W.box(-CORE_X, -PF + 0.01, -GUSSET_T / 2, GUSSET_T / 2, FOOT_Z0, FOOT_Z1),
                 W.box(-PF - 0.01, -PF + 0.01, -GUSSET_T / 2, GUSSET_T / 2, FOOT_Z0, GUSSET_TOP))
    fy = lambda s: sorted((s * MAST_HW, s * (MAST_HW - FL_T)))
    fl = W.union([fb(1, PF, -0.5, FL_D, *fy(s), FOOT_Z0, MAST_TOP) for s in (1, -1)])          # front, full height
    fl += fb(-1, PF, -0.5, FL_D, *fy(-1), FOOT_Z0, MAST_TOP)                                     # back, robot right
    fl += fb(-1, PF, -0.5, FL_D, *fy(1), FOOT_Z0, ESP_ZC - 5.8)                                 # back, robot left, below the USB plug
    boss = W.union([W.hull(W.zcyl(x, y, 3.5, MAST_TOP - 14, MAST_TOP), W.zcyl(x, y, PF, MAST_TOP - 15.5, MAST_TOP - 14)) for x, y in HOLDER_SCREWS])
    # ESP32: four corner pads under the PCB corners (module side), before the first header pin, 45 deg chins
    pads = Manifold()
    for y0, y1 in ((ESP_Y0 + 0.3, ESP_Y0 + 2.5), (-ESP_Y0 - 2.5, -ESP_Y0 - 0.3)):
        for z0, z1, ch in ((ESP_Z0 + 0.4, ESP_Z0 + 3.4, 2.0), (ESP_Z0 + ESP[1] - 3.4, ESP_Z0 + ESP[1] - 0.4, ESP_GAP)):
            pads += W.hull(fb(-1, PF, -0.5, ESP_GAP, y0, y1, z0, z1), fb(-1, PF, -0.5, ESP_GAP - ch, y0, y1, z0 - ch, z1))
    pads -= fb(-1, PF, ESP_GAP - 3.4, ESP_GAP + 0.1, ESP_Y0 - 1, ESP_Y0 + 25.8, ESP_ZC - 9.3, ESP_ZC + 9.3)   # WROOM module
    # rails: 1.6 mm standoffs under the solder side of the boards without holes
    def rails_z(s, y0, y1, z0, z1):      # two rails along y under the top and bottom edges
        return W.union([fb(s, PF, -0.5, 1.6, y0 + 0.5, y1 - 0.5, z, z + 2.0) for z in (z0 + 0.2, z1 - 2.2)])
    def rails_y(s, y0, y1, z0, z1):      # two rails along z under the left and right edges
        return W.union([fb(s, PF, -0.5, 1.6, y, y + 2.0, z0 + 0.5, z1 - 0.5) for y in (y0 + 0.2, y1 - 2.2)])
    rails = rails_z(1, -BMS[0] / 2, BMS[0] / 2, BMS_Z0, BMS_Z0 + BMS[1]) + rails_y(1, SW_Y0, SW_Y0 + SW[0], SW_Z0, SW_Z0 + SW[1])
    rails += rails_y(1, JB_Y0, JB_Y0 + JB[0], JB_Z0, JB_Z0 + JB[1])
    rails += rails_y(-1, MX_Y0, MX_Y0 + MX[0], MX_Z0, MX_Z0 + MX[1]) + rails_y(-1, MINI_Y0, MINI_Y0 + MINI[0], MINI_Z0, MINI_Z0 + MINI[1])
    blocks, tuns = [], []
    for args in ((1, PF, -BMS[0] / 2 - 1.6, BMS_Z0 + BMS[1] / 2), (1, PF, BMS[0] / 2 + 1.6, BMS_Z0 + BMS[1] / 2),
                 (1, PF, SW_Y0 - 1.6, SW_Z0 + 3.5), (1, PF, SW_Y0 + SW[0] + 1.6, SW_Z0 + 3.5),
                 (1, PF, JB_Y0 - 1.6, JB_Z0 + JB[1] / 2), (1, PF, JB_Y0 + JB[0] + 1.6, JB_Z0 + JB[1] / 2),
                 (-1, PF, MINI_Y0 + MINI[0] + 1.6, BACK_TIE_Z), (-1, PF, -ESP_Y0 + 2.0, ESP_TIES[1])):
        b, t = anchor(*args); blocks.append(b); tuns.append(t)
    m = foot + plate + gus + fl + boss + pads + rails + W.union(blocks)
    m -= W.union(tuns)
    # MX1508 tie: robot-right end through the flange root, other end through the gusset root (shared with the Mini360)
    m -= fb(-1, PF, 0.0, TUN_W, -MAST_HW - 1, -MAST_HW + FL_T + 1, BACK_TIE_Z - 1.5, BACK_TIE_Z + 1.5)
    m -= fb(-1, PF, 0.0, TUN_W, -GUSSET_T / 2 - 1, GUSSET_T / 2 + 1, BACK_TIE_Z - 1.5, BACK_TIE_Z + 1.5)
    m -= W.union([W.zcyl(x, y, M3_CLEAR_R, FOOT_Z0 - 1, FOOT_Z1 + 1) for x, y in MAST_SCREWS])
    m -= W.union([W.zcyl(x, y, M3_PILOT_R, MAST_TOP - 12, MAST_TOP + 1) for x, y in HOLDER_SCREWS])
    # tie holes through the flanges near their free edge (the tie goes round the 2.2 mm bar outside the hole):
    # ESP32 ties on both back flanges, battery leads on the front-right flange, J1 on the front-left flange
    def hole(s, ys, z):
        y0, y1 = fy(ys)
        return fb(s, PF, FL_D - 4.0, FL_D - 2.2, y0 - 1, y1 + 1, z - 1.5, z + 1.5)
    m -= hole(-1, 1, ESP_TIES[0]) + hole(-1, -1, ESP_TIES[0]) + hole(-1, -1, ESP_TIES[1])
    m -= hole(1, -1, 124.0) + hole(1, 1, J1_POS[2] + 4.0) + hole(1, 1, J1_POS[2] + J1[1] - 4.0)
    m -= W.box(-PF - 1, PF + 1, -STRAP_W / 2 - 1, STRAP_W / 2 + 1, MAST_TOP - 2, MAST_TOP + 1)       # strap groove under the tray
    return m

def tray():
    """Battery tray: the holder sits in it, two screws go through holder and tray into the mast, a strap holds the
    cells in. Prints floor down, no supports."""
    t = W.box(-TX, TX, -TY, TY, TRAY_Z0, TRAY_Z1)
    t -= W.box(-TX + TRAY_WALL, TX - TRAY_WALL, -TY + TRAY_WALL, TY - TRAY_WALL, TRAY_Z0 + FLOOR_T, TRAY_Z1 + 1)
    t -= W.union([W.box(-8, 8, *sorted((s * TY - s * 3, s * TY + s * 3)), TRAY_Z0 + FLOOR_T, TRAY_Z1 + 1) for s in (1, -1)])  # lead exits
    t -= W.union([W.zcyl(x, y, M3_CLEAR_R, TRAY_Z0 - 1, TRAY_Z0 + FLOOR_T + 1) for x, y in HOLDER_SCREWS])
    return t

def encoder_ring_flat():
    """The ring as printed: flat in x-y, z 0..RING_T, 16 sector windows (11.25 deg window, 11.25 deg bar) between
    a 1.2 mm inner and outer rim that hold the bars together."""
    from manifold3d import CrossSection
    cs = CrossSection.circle(RING_R1, 128) - CrossSection.circle(RING_R0, 128)
    for k in range(RING_N):
        a0 = 2 * np.pi * k / RING_N - 1.5 * np.pi / RING_N; a1 = a0 + np.pi / RING_N   # a bar at the bottom as drawn
        pts = [(0.0, 0.0)] + [((RING_R1 + 2) * np.cos(a), (RING_R1 + 2) * np.sin(a)) for a in np.linspace(a0, a1, 6)]
        cs -= CrossSection([pts]) ^ (CrossSection.circle(RING_WIN[1], 128) - CrossSection.circle(RING_WIN[0], 128))
    return cs.extrude(RING_T)

def encoder_ring():
    """The ring on the left wheel's inside face (body frame)."""
    return encoder_ring_flat().rotate([-90, 0, 0]).translate([0, RING_Y0, 0])

PRINTED = {   # name: (builder, rotation into print orientation, note)
    'core_left':  (lambda: core_halves()[0], [90, 0, 0], 'flat on the cut face (y = 0 down), no supports'),
    'core_right': (lambda: core_halves()[1], [-90, 0, 0], 'flat on the cut face (y = 0 down), no supports'),
    'mast':       (mast, [0, 0, 0], 'upright on its foot, no supports'),
    'tray':       (tray, [0, 0, 0], 'floor down, no supports'),
    'encoder_ring': (encoder_ring_flat, [0, 0, 0], 'flat, 0.6 mm, black PLA (milestone B)'),
}

def print_pose(m, rot):
    p = m.rotate(rot)
    v = W.mesh_np(p)[0]
    lo, hi = v.min(0), v.max(0)
    return p.translate([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])

# =============================================================================================== the assembly
def board_solids():
    """Boards, plugs and the jack in the body frame: name -> (solid, mass g, source). Envelopes are [E]."""
    b = {}
    # GY-521 on the front of the core, chip forward, header along its top edge with the pins soldered forward
    f = CORE_X
    b['imu'] = (fb(1, f, IMU_POST, IMU_POST + IMU[2], -IMU[0] / 2, IMU[0] / 2, IMU_Z0, IMU_Z0 + IMU[1])
                + fb(1, f, IMU_POST + IMU[2], IMU_POST + IMU[2] + 1.0, -2, 2, IMU_Z0 + 6, IMU_Z0 + 10)
                + fb(1, f, IMU_POST + IMU[2], IMU_POST + IMU[2] + 2.5, -10.2, 10.2, IMU_Z0 + IMU[1] - 2.6, IMU_Z0 + IMU[1])
                + fb(1, f, IMU_POST + IMU[2] + 2.5, IMU_POST + IMU[2] + 8.5, -9.5, 9.5, IMU_Z0 + IMU[1] - 1.7, IMU_Z0 + IMU[1] - 0.9),
                2.5, '[E] GY-521 MPU-6050, 21 x 16 mm, front of the core, chip forward')
    f = PF
    # ESP32 across the back of the mast: module towards the plate, header pins outwards, USB to the left (+y)
    y0, y1, z0, z1 = ESP_Y0, -ESP_Y0, ESP_Z0, ESP_Z0 + ESP[1]
    pins_y = (y0 + 3.35, y1 - 3.35)
    esp = fb(-1, f, ESP_GAP, ESP_GAP + ESP[2], y0, y1, z0, z1)
    esp += fb(-1, f, ESP_GAP - 3.1, ESP_GAP, y0 - 0.5, y0 + 25.5, ESP_ZC - 9, ESP_ZC + 9)          # WROOM-32 module
    esp += fb(-1, f, ESP_GAP - 3.0, ESP_GAP, y1 - 5.6, y1 + 0.6, ESP_ZC - 3.9, ESP_ZC + 3.9)        # micro-USB
    for zr in (z0 + 1.27, z1 - 1.27):
        esp += fb(-1, f, ESP_GAP + ESP[2], ESP_GAP + ESP[2] + 2.5, *pins_y, zr - 1.27, zr + 1.27)     # header plastic
        esp += fb(-1, f, ESP_GAP + ESP[2] + 2.5, ESP_GAP + ESP[2] + 8.5, *pins_y, zr - 0.4, zr + 0.4)  # pins
    b['esp32'] = (esp, 10.0, '[E] ESP32 DevKit 55 x 28 mm, back of the mast, four corner pads, two zip ties')
    b['mx1508'] = (fb(-1, f, 1.6, 3.2, MX_Y0, MX_Y0 + MX[0], MX_Z0, MX_Z0 + MX[1])
                   + fb(-1, f, 3.2, 5.0, MX_Y0 + 8, MX_Y0 + 16, MX_Z0 + 6, MX_Z0 + 14)
                   + xcyl_c(-1, f, 3.2, 3.2 + 11, MX_Y0 + 4, MX_Z0 + 13, 4.0), 3.0 + 1.5,
                   '[E] MX1508 24.7 x 21 mm with C1 470 uF on its + / - pads, back of the mast, robot right')
    b['mini360'] = (fb(-1, f, 1.6, 2.6, MINI_Y0, MINI_Y0 + MINI[0], MINI_Z0, MINI_Z0 + MINI[1])
                    + fb(-1, f, 2.6, 5.6, MINI_Y0 + 3, MINI_Y0 + 10, MINI_Z0 + 2, MINI_Z0 + 9), 2.0, '[E] Mini360 buck 17.2 x 11.2 mm, back, robot left')
    b['bms_2s'] = (fb(1, f, 1.6, 2.6, -BMS[0] / 2, BMS[0] / 2, BMS_Z0, BMS_Z0 + BMS[1])
                   + fb(1, f, 2.6, 5.1, -18, 18, BMS_Z0 + 4, BMS_Z0 + BMS[1] - 4), 4.0, '[E] 2S BMS ~50 x 22 x 3.5 mm, front of the mast, top')
    b['switch'] = (fb(1, f, 1.6, 3.2, SW_Y0, SW_Y0 + SW[0], SW_Z0, SW_Z0 + SW[1])
                   + fb(1, f, 3.2, 6.2, SW_Y0 + 3, SW_Y0 + SW[0] - 3, SW_Z0 + 8, SW_Z0 + 13.5)
                   + fb(1, f, 6.2, 9.0, SW_Y0 + 6, SW_Y0 + 9.5, SW_Z0 + 9.5, SW_Z0 + 12), 1.5,
                   '[E] Pololu 2810 switch board 15.2 x 15.2 mm, front, robot left, slider forward')
    b['jb'] = (fb(1, f, 1.6, 3.2, JB_Y0, JB_Y0 + JB[0], JB_Z0, JB_Z0 + JB[1])
               + fb(1, f, 3.2, 5.6, JB_Y0 + 2, JB_Y0 + 12, JB_Z0 + 2, JB_Z0 + 6)
               + fb(1, f, 3.2, 3.2 + XH[2], JB_Y0 + 13, JB_Y0 + 13 + XH[0], JB_Z0 + 3.5, JB_Z0 + 3.5 + XH[1]), 1.5 + 1.0,
               '[E] junction perfboard ~21 x 13 mm with R1, R2, C2 and the K2 header (K2 plug mated, pointing forward)')
    # K1 motor plug: mated pair lying on the back of the core, tied through the core's upper back tunnel
    b['k1_plug'] = (fb(-1, CORE_X, 0.3, 0.3 + XH[1], -XH[0] / 2, XH[0] / 2, CAN_Z1 + 1.0, CAN_Z1 + 1.0 + XH[2]), 1.0,
                    '[E] JST-XH 2-pin, mated, in the motor leads')
    # J1 charge jack on its pigtail, tied round the front edge of the front-left flange, socket pointing down
    jx, jy, jz = J1_POS
    b['j1_jack'] = (W.zcyl(jx, jy, J1[0], jz, jz + J1[1]), 3.0, '[E] DC 5.5 x 2.1 mm pigtail jack, socket down')
    # milestone B: the TCRT5000 in its clip under the core, lenses towards the ring; the module on the back of the core
    zc = -TCRT_R
    tc = W.box(-TCRT[1] / 2, TCRT[1] / 2, TCRT_BACK, TCRT_FACE, zc - TCRT[0] / 2, zc + TCRT[0] / 2)
    tc += W.union([W.box(-0.3, 0.3, TCRT_BACK - 2.5, TCRT_BACK, z - 0.3, z + 0.3) for z in (zc - 3.8, zc - 1.3, zc + 1.3, zc + 3.8)])
    b['tcrt5000'] = (tc, 0.5, '[D] TCRT5000 10.2 x 5.8 x 7 mm, in the clip under the core, 2.5 mm from the ring (milestone B)')
    f = CORE_X; z0 = MOD_Z0
    mod = fb(-1, f, 1.6, 1.6 + MOD[2], -MOD[0] / 2, MOD[0] / 2, z0, z0 + MOD[1])
    mod += fb(-1, f, 3.2, 8.0, -3.5, 3.5, z0 + 16, z0 + 22)                       # trimmer
    mod += fb(-1, f, 3.2, 5.0, -4.5, 1.5, z0 + 8, z0 + 13)                         # LM393
    mod += fb(-1, f, 3.2, 4.5, 3.5, 5.5, z0 + 24, z0 + 28)                         # LEDs
    mod += fb(-1, f, 3.2, 5.7, -3.8, 3.8, z0 + MOD[1] - 2.5, z0 + MOD[1])          # 3-pin header, pins up
    mod += W.union([fb(-1, f, 4.2, 4.8, y - 0.3, y + 0.3, z0 + MOD[1], z0 + MOD[1] + 6) for y in (-2.54, 0, 2.54)])
    b['tcrt_module'] = (mod, 2.5, '[E] TCRT5000 module 32 x 14 mm (its sensor moved to the clip), back of the core (milestone B)')
    # C3 across the motor tabs, in the tab pocket above the can
    b['c3'] = (W.box(-3.5, 3.5, -1.3, 1.3, CAN_Z1 + 0.5, CAN_Z1 + 4.5), 0.2, '[E] 100 nF disc across the motor tabs')
    return b

def xcyl_c(s, xf, d0, d1, y, z, r):
    a, b_ = sorted((s * (xf + d0), s * (xf + d1)))
    return W.xcyl(y, z, r, a, b_)

def fasteners():
    """(name, solid, mass g, note). Steel masses from volume, [E]."""
    out = []
    seat = CH_HW - REC_DEPTH
    for x, z in BOLTS:
        out.append(('m3x20_bolt', W.ycyl(x, z, 1.5, seat - BOLT_L, seat) + W.ycyl(x, z, HEAD_D / 2, seat, seat + HEAD_H), 1.4))
        out.append(('m3_nut', hexy(x, z, NUT_AF, -seat - NUT_H, -seat) - W.ycyl(x, z, 1.5, -20, 20), 0.25))
    for x, z in W.TT_HOLES:
        out.append(('pin_3x22', W.ycyl(x, z, 1.45, -PIN_L / 2, PIN_L / 2), np.pi * 1.5 ** 2 * PIN_L * STEEL))
    for x, y in MAST_SCREWS:
        out.append(('m3x12_selftap', W.zcyl(x, y, 1.2, FOOT_Z1 - 12, FOOT_Z1) + W.zcyl(x, y, 2.8, FOOT_Z1, FOOT_Z1 + 2.0), 0.9))
    hz0 = TRAY_Z0 + FLOOR_T + HOLDER_FLOOR
    for x, y in HOLDER_SCREWS:
        out.append(('m3x16_selftap', W.zcyl(x, y, 1.2, hz0 - 16, hz0) + W.zcyl(x, y, 2.8, hz0, hz0 + 2.0), 1.1))
    for y, z in IMU_HOLES:
        h0 = CORE_X + IMU_POST + IMU[2]   # landscape: the lower two pilots
        out.append(('m25x8_selftap', W.xcyl(y, z, 1.0, h0 - 8, h0) + W.xcyl(y, z, 2.2, h0, h0 + 1.6), 0.4))
    return out

def wire_paths():
    """Main wire runs (name, points, radius, note). Clear of the wheels, tied at the anchors and tunnels."""
    zb = TRAY_Z0 + FLOOR_T
    return [
        ('motor_leads', [(0, 0, CAN_Z1 + 2.5), (-10, 0, CAN_Z1 + 2.5), (-CORE_X - 1.5, 0, CAN_Z1 + 1.8), (-CORE_X - 3.2, 0, CAN_Z1 + 1.2)], 1.2,
         'from the tabs (with C3) out of the back of the core into K1'),
        ('motor_leads_2', [(-CORE_X - 3.2, 0, CAN_Z1 + 1.0 + XH[2] + 0.5), (-CORE_X - 3.2, -3, FOOT_Z1 + 6), (-PF - 11.0, -6, FOOT_Z1 + 3),
                           (-PF - 11.0, -8, MX_Z0 + 2)], 1.2, 'K1 up to the MX1508 MOTOR-A pads'),
        ('imu_cable', [(CORE_X + 13.6, 0, IMU_Z0 + IMU[1] - 1.3), (CORE_X + 8, -3, IMU_Z0 + IMU[1] + 5), (CORE_X + 2.0, -4, 46), (CORE_X + 2.0, -4, 66),
                       (CORE_X + 2.0, -14, 69.0), (PF + FL_D + 3.0, -MAST_HW - 2.5, 74), (PF + FL_D + 3.0, -MAST_HW - 2.5, ESP_Z0 - 2),
                       (-PF - FL_D - 3.0, -MAST_HW - 2.5, ESP_Z0 - 2), (-PF - 11.0, -MAST_HW + 5, ESP_Z0 - 1.8)], 1.4,
         'four 20 cm jumpers: up the front of the core (tied at its two tunnels), round the robot-right edge of the mast, to the ESP32'),
        ('battery_leads', [(0, -TY - 1, zb + 1.5), (2, -TY - 2.5, TRAY_Z0 - 2), (PF + 4.0, -MAST_HW - 2.2, 126), (PF + 4.0, -MAST_HW - 2.2, 121),
                           (PF + FL_D + 2.5, -MAST_HW - 2.2, 120), (PF + FL_D + 2.5, -BMS[0] / 2 - 1.5, 119.5), (PF + 4.5, -BMS[0] / 2 - 1.5, 119.5)], 1.4,
         'from the holder down the robot-right edge (tied to the flange) to the BMS'),
        ('power', [(PF + 9, SW_Y0 + SW[0] / 2, SW_Z0 + SW[1] + 1.5), (PF + 9, SW_Y0 + SW[0] / 2, BMS_Z0 - 1.2)], 1.2,
         'BMS output and fuse to the switch'),
        ('rail', [(PF + 3.2 + XH[2] + 1.4, JB_Y0 + 16.75, JB_Z0 + 6.4), (PF + 3.2 + XH[2] + 1.4, SW_Y0 + 3, SW_Z0 + 4.0), (PF + 8.0, SW_Y0 + 3, SW_Z0 + 4.0)], 1.2,
         'K2 lead from the switch to JB'),
        ('tcrt_pigtail', [(0, -1.2, -TCRT_R - 1.3), (-17, -1.5, -17.2), (-CORE_X - 1.5, -1.5, -16.0), (-CORE_X - 2.9, -1.5, -9),
                          (-CORE_X - 2.9, -1.5, 12.5), (-CORE_X - 4.5, 0, MOD_Z0 + 2)], 1.0,
         'milestone B: 4 wires from the TCRT5000 leads (behind the clip) under the core, up through the back crash-arm root to the module'),
        ('tcrt_cable', [(-CORE_X - 4.5, 0, MOD_Z0 + MOD[1] + 6.5), (-CORE_X - 4.5, -8, MOD_Z0 + MOD[1] + 9), (-CORE_X - 4.5, -8, 72),
                        (-21, -8, 79), (-21, -8, 96), (-19, -12.7, ESP_Z0 - 1.5)], 1.2,
         'milestone B: VCC, GND, D0 from the module header up the back of the core (tied with K1) and behind the MX1508 to the ESP32 lower row'),
        ('rail_back', [(PF + 6.0, JB_Y0 + 4, JB_Z0 - 1.5), (PF + 6.0, -15, 69.5), (-PF - 6.0, -15, 69.5), (-PF - 6.0, -15, MX_Z0 - 1.5)], 1.2,
         'battery rail and ground from JB under the plate edge to the MX1508 + / - pads'),
    ]

def items():
    it = []
    L, Rr = core_halves()
    it += [It('core_left', L, C['body'], 'body', material='PLA', kind='printed'),
           It('core_right', Rr, C['body'], 'body', material='PLA', kind='printed'),
           It('mast', mast(), C['body2'], 'body', material='PLA', kind='printed'),
           It('tray', tray(), C['body2'], 'body', material='PLA', kind='printed')]
    gb, can, shaft = motor()
    it += [It('tt_gearbox', gb, C['tt'], 'body', 22.0, source='[E] dual-shaft TT gearmotor 1:48, ~29 g', kind='bought'),
           It('tt_can', can, C['can'], 'body', 7.0, source='[E]', kind='bought'),
           It('tt_shaft', shaft, C['horn'], 'wheels', 1.0, source='[E]', kind='bought')]
    (tl, rl), (tr, rr) = wheels()
    it += [It('tyre_l', tl, C['tyre'], 'wheels', 10.0, source='[E] 65 mm TT wheel ~22 g', kind='bought'),
           It('rim_l', rl, C['rim'], 'wheels', 12.0, source='[E]', kind='bought'),
           It('tyre_r', tr, C['tyre'], 'wheels', 10.0, source='[E]', kind='bought'),
           It('rim_r', rr, C['rim'], 'wheels', 12.0, source='[E]', kind='bought')]
    hx, hy, hz = HOLDER
    zb = TRAY_Z0 + FLOOR_T
    hold = W.box(-hx / 2, hx / 2, -hy / 2, hy / 2, zb, zb + hz) - W.box(-hx / 2 + 1.2, hx / 2 - 1.2, -hy / 2 + 4, hy / 2 - 4, zb + HOLDER_FLOOR, zb + hz + 1)
    hold -= W.union([W.zcyl(x, y, 1.6, zb - 1, zb + 3) for x, y in HOLDER_SCREWS])
    it += [It('holder', hold, C['holder'], 'body', 14.0, source='[E] 2 x 18650 holder 41 x 77.5 x 20.5', kind='bought'),
           It('cells', W.union([W.ycyl(x, zb + 10.2, 9.2, -32.5, 32.5) for x in (-10.0, 10.0)]), C['cell'], 'body', 92.0,
              source='[E] 18650 ~46 g each', kind='bought')]
    cols = dict(esp32=C['chip'], mx1508=C['pcb'], bms_2s=C['pcb'], jb=(0.75, 0.62, 0.35), k1_plug=(0.95, 0.95, 0.9),
                j1_jack=(0.12, 0.12, 0.13), c3=(0.85, 0.55, 0.15))
    for n, (s_, m, src) in board_solids().items():
        it.append(It(n, s_, cols.get(n, C['pcb_blue']), 'body', m, source=src, kind='board'))
    for k, (n, s, m) in enumerate(fasteners()):
        it.append(It(f'{n}_{k}', s, C['steel'], 'body', m, source='[E] steel', kind='fastener'))
    paths = wire_paths(); L_a = sum(plen(p) for n, p, _, _ in paths if not n.startswith('tcrt'))
    L_b = sum(plen(p) for n, p, _, _ in paths if n.startswith('tcrt'))
    for n, p, r, note in paths:
        b_ = n.startswith('tcrt')
        it.append(It('wire_' + n, tube(p, r), (0.55, 0.25, 0.65) if b_ else (0.75, 0.15, 0.12) if 'batt' in n or 'power' in n else (0.15, 0.35, 0.75), 'body',
                     (2.0 * plen(p) / L_b) if b_ else 14.0 * plen(p) / L_a,
                     source='[E] milestone B: share of 2 g of wire and Dupont ends' if b_ else '[E] share of 14 g of wire, jumpers and connectors', kind='wire'))
    it.append(It('encoder_ring', encoder_ring(), (0.10, 0.10, 0.11), 'wheels', encoder_ring_flat().volume() * 1e-3 * W.PLA,
                 source='[C] printed 0.6 mm black PLA, solid, glued to the left wheel (milestone B)', kind='printed', material='PLA'))
    # hook-and-loop strap round holder and tray, through the groove in the mast top
    st = W.box(-TX - 1.25, TX + 1.25, -STRAP_W / 2, STRAP_W / 2, MAST_TOP - 1.8, zb + hz + 1.2) - \
        W.box(-TX - 0.05, TX + 0.05, -STRAP_W / 2 - 1, STRAP_W / 2 + 1, MAST_TOP - 0.6, zb + hz + 0.05)
    it.append(It('strap', st, (0.25, 0.25, 0.28), 'body', 3.0, source='[E] 10 mm hook-and-loop strap', kind='bought'))
    it.append(It('zip_ties', None, None, 'body', 3.6, (-6, 0, 65), '[E] 12 x 2.5 mm zip ties (2 more for milestone B)', kind='mass'))
    it.append(It('fuse', None, None, 'body', 1.0, (PF + 8, 14, BMS_Z0 - 3), '[E] PTC fuse, in the lead from the BMS to the switch', kind='mass'))
    return it

# =============================================================================================== mass properties
def mass(i):
    return i.mass if i.mass else i.solid.volume() / 1000.0 * (W.PLA if i.material == 'PLA' else W.ABS) * W.FILL

def com(i):
    return np.array(i.point, float) if i.solid is None else W.centroid(i.solid)

def bbox(i):
    v = W.mesh_np(i.solid)[0]
    return v.min(0), v.max(0)

def mass_props(its):
    """Mass (kg), CoM (m) and inertia tensor about the CoM (kg m2). Each part is a uniform box the size of its
    bounding box at its own centroid: close enough at this stage."""
    M = sum(mass(i) for i in its) * 1e-3
    c = sum(mass(i) * com(i) for i in its) * 1e-3 / M * 1e-3
    I = np.zeros((3, 3))
    for i in its:
        m = mass(i) * 1e-3; p = com(i) * 1e-3 - c
        if i.solid is not None:
            lo, hi = bbox(i); s = (hi - lo) * 1e-3
            I += np.diag([m / 12 * (s[1] ** 2 + s[2] ** 2), m / 12 * (s[0] ** 2 + s[2] ** 2), m / 12 * (s[0] ** 2 + s[1] ** 2)])
        I += m * (p @ p * np.eye(3) - np.outer(p, p))
    return M, c, I

# =============================================================================================== checks
def section(cs):
    """Area, centroid (u, v) and second moment about the centroidal v axis (int (u - uc)^2 dA) and the
    extreme fibre distance in u, from a manifold CrossSection."""
    A = Su = Iuu = 0.0
    us = []
    for P in cs.to_polygons():
        P = np.asarray(P, float); u, v = P[:, 0], P[:, 1]; u2, v2 = np.roll(u, -1), np.roll(v, -1)
        cr = u * v2 - u2 * v
        A += cr.sum() / 2; Su += ((u + u2) * cr).sum() / 6; Iuu += ((u * u + u * u2 + u2 * u2) * cr).sum() / 12
        us.append(u)
    uc = Su / A; I = Iuu - A * uc ** 2
    us = np.concatenate(us)
    return A, uc, I, max(us.max() - uc, uc - us.min())

def wall_scan(m, n=5000, seed=0, skip_z=0.6):
    """Local wall thickness by ray casting inwards from random surface points (part in print pose).
    Returns thickness and point arrays. Points within skip_z of the bed are skipped (bed chamfers)."""
    v, t = W.mesh_np(m)
    a, b, c = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
    nrm = np.cross(b - a, c - a); area = np.linalg.norm(nrm, axis=1) / 2; nrm /= (2 * area[:, None] + 1e-12)
    rng = np.random.default_rng(seed)
    k = rng.choice(len(t), size=n, p=area / area.sum())
    r1, r2 = rng.random(n), rng.random(n); s = np.sqrt(r1)
    p = (1 - s)[:, None] * a[k] + (s * (1 - r2))[:, None] * b[k] + (s * r2)[:, None] * c[k]
    th, pts = [], []
    for q, nn in zip(p, nrm[k]):
        if q[2] < skip_z:
            continue
        o = q - nn * 1e-3
        hits = m.ray_cast(o.tolist(), (q - nn * 30).tolist())
        d = [h.distance * 30 for h in hits if h.distance * 30 > 2e-3]
        if d:
            th.append(d[0]); pts.append(q)
    return np.array(th), np.array(pts)

def overhangs(m, min_area=4.0):
    """Down-facing regions steeper than 45 deg in the print pose, not on the bed. Returns a list of
    (area mm2, z, size x, size y, flat) per connected region above min_area."""
    v, t = W.mesh_np(m)
    a, b, c = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
    nrm = np.cross(b - a, c - a); ar = np.linalg.norm(nrm, axis=1) / 2; nrm /= (2 * ar[:, None] + 1e-12)
    cz = (a[:, 2] + b[:, 2] + c[:, 2]) / 3
    sel = np.where((nrm[:, 2] < -0.7072) & (cz > 0.05))[0]
    parent = {}
    def find(x):
        while parent.setdefault(x, x) != x:
            parent[x] = parent[parent[x]]; x = parent[x]
        return x
    for f in sel:
        i0, i1, i2 = t[f]
        r0 = find(i0); parent[find(i1)] = r0; parent[find(i2)] = r0
    groups = {}
    for f in sel:
        groups.setdefault(find(t[f][0]), []).append(f)
    out = []
    for fs in groups.values():
        fs = np.array(fs); A = ar[fs].sum()
        if A < min_area:
            continue
        pv = np.vstack([a[fs], b[fs], c[fs]]); lo, hi = pv.min(0), pv.max(0)
        flat = bool((nrm[fs, 2] < -0.98).mean() > 0.8)
        out.append((A, (lo[2] + hi[2]) / 2, hi[0] - lo[0], hi[1] - lo[1], flat))
    return sorted(out, key=lambda r: -r[0])

def hole_walls(part, centre, axis, r, s0, s1, nsec=5, nang=24):
    """Wall round a hole: rays from the hole wall outwards (perpendicular to its axis) until the next surface.
    centre: a point on the axis; axis: unit vector; the hole runs from s0 to s1 along it. Returns min wall (mm)."""
    ax = np.array(axis, float); ax /= np.linalg.norm(ax)
    e1 = np.cross(ax, [0, 0, 1] if abs(ax[2]) < 0.9 else [1, 0, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(ax, e1)
    best = 1e9
    for s in np.linspace(s0 + 0.3, s1 - 0.3, nsec):
        for a in np.linspace(0, 2 * np.pi, nang, endpoint=False):
            d = np.cos(a) * e1 + np.sin(a) * e2
            o = np.array(centre, float) + s * ax + d * (r + 0.02)
            hits = part.ray_cast(o.tolist(), (o + d * 25).tolist())
            ds = [h.distance * 25 for h in hits if h.distance * 25 > 0.02]
            if len(ds) >= 1:
                best = min(best, ds[0])
    return best

def rest_angle(body_solids, sign=1):
    """Pitch (deg) at which the body first touches the floor when it falls forward (sign 1) or back (-1),
    pivoting on the axle with the wheels on the floor. Returns angle and the touching point (body frame)."""
    v = np.vstack([W.mesh_np(s)[0] for s in body_solids])
    lo, hi = 0.0, 90.0
    for _ in range(40):
        th = np.radians((lo + hi) / 2) * sign
        zf = -v[:, 0] * np.sin(th) + v[:, 2] * np.cos(th) + R
        lo, hi = (lo, (lo + hi) / 2) if zf.min() <= 0 else ((lo + hi) / 2, hi)
    th = np.radians(hi) * sign
    zf = -v[:, 0] * np.sin(th) + v[:, 2] * np.cos(th) + R
    return hi, v[np.argmin(zf)]

# =============================================================================================== analysis
def analyse():
    its = items()
    body = [i for i in its if i.group == 'body']; wh = [i for i in its if i.group == 'wheels']
    Mb, cb, Ib = mass_props(body); Mw, cw, Iw = mass_props(wh)
    M = Mb + Mw
    RES = []
    def chk(name, ok, text):
        RES.append((name, 'PASS' if ok else ('WARN' if ok is None else 'FAIL'), text))
    L = ['# Stage 1 balancer: design check', '',
         'Generated by `stage1.py`. Nothing is built or measured: bought-part sizes and masses are [E] estimates from '
         'typical listings, the same the published Tailwhip uses. Mechanical write-up: `docs/mechanical.md`.', '']
    # ---- mass
    L += ['## 1. Mass', '', '| Part | Kind | Mass (g) | Source | CoM x, z (mm) |', '|---|---|---|---|---|']
    groups = {}
    for i in its:
        key = i.name.rsplit('_', 1)[0] if i.kind == 'fastener' else i.name
        g = groups.setdefault(key, [0.0, np.zeros(3), 0, i])
        g[0] += mass(i); g[1] += mass(i) * com(i); g[2] += 1
    for key, (m, mc, n, i) in groups.items():
        c3 = mc / m
        nm = f'{key} x {n}' if n > 1 else key
        L += [f'| {nm} | {i.kind} | {m:.1f} | {i.source or "printed " + i.material + ", 55 % of solid [E]"} | {c3[0]:.0f}, {c3[2]:.0f} |']
    pm = sum(mass(i) for i in its if i.kind == 'printed')
    L += [f'| **Total** | | **{M * 1e3:.0f}** | printed {pm:.0f} g, everything else {M * 1e3 - pm:.0f} g | |', '']
    # ---- balance
    h = cb[2]
    Iax = Ib[1, 1] + Mb * (cb[0] ** 2 + cb[2] ** 2)
    tau = np.sqrt(Iax / (Mb * 9.81 * h))
    L += ['## 2. Balance', '',
          f'- Body (everything but the wheels and shaft) {Mb * 1e3:.0f} g, centre of mass **{cb[0] * 1e3:+.1f} mm** fore-aft, '
          f'{cb[1] * 1e3:+.1f} mm sideways and **{h * 1e3:.1f} mm above the axle**; wheels and shaft {Mw * 1e3:.0f} g.',
          f'- Body pitch inertia about the axle {Iax:.2e} kg m2. Fall time constant sqrt(I / m g h) = **{tau * 1e3:.0f} ms**: '
          f'a lean roughly doubles every {tau * 1e3 * 0.69:.0f} ms.',
          f'- Torque to hold a lean: m g h sin(th) = {Mb * 9.81 * h:.3f} sin(th) N m. One TT motor stalls at about 0.08 N m at 6 V [E]; '
          f'the tyres slip at mu m g r = {0.8 * M * 9.81 * R * 1e-3:.3f} N m (mu 0.8). Steady lean one motor can hold: '
          f'**{np.degrees(np.arcsin(min(1, min(0.08, 0.8 * M * 9.81 * R * 1e-3) / (Mb * 9.81 * h)))):.0f} deg**.',
          f'- Track {TRACK:.1f} mm (wheel centres). Centre of mass {(R * 1e-3 + (Mb * cb[2]) / M) * 1e3:.0f} mm above the floor, so it tips '
          f'sideways past **{np.degrees(np.arctan((TRACK / 2 + W.W_WHEEL / 2) / (R + Mb * cb[2] * 1e3 / M))):.0f} deg** of roll. Flat floors only.', '']
    chk('CoM on the axle fore-aft', abs(cb[0] * 1e3) <= 2.0, f'{cb[0] * 1e3:+.1f} mm (limit 2 mm)')
    chk('CoM height', h * 1e3 <= 89.5, f'{h * 1e3:.1f} mm above the axle (was 89.2, must not go higher)')
    # ---- falls
    solids_body = [i.solid for i in body if i.solid is not None]
    rf, pf = rest_angle(solids_body, 1); rb, pb = rest_angle(solids_body, -1)
    rest = np.degrees(np.arcsin(R / np.hypot(SKID_X, SKID_Z) - SKID_R / np.hypot(SKID_X, SKID_Z)) + np.arctan2(SKID_Z, SKID_X))
    L += ['## 3. Falls', '',
          f'- From the meshes: falling forward it first touches at **{rf:.1f} deg** at ({pf[0]:.0f}, {pf[1]:.0f}, {pf[2]:.0f}) mm; '
          f'falling back at **{rb:.1f} deg** at ({pb[0]:.0f}, {pb[1]:.0f}, {pb[2]:.0f}) mm. The crash bar formula gives {rest:.1f} deg.',
          f'- At rest the centre of mass is {h * 1e3 * np.sin(np.radians(rest)):.0f} mm out and the bar {SKID_X * np.cos(np.radians(rest)):.0f} mm out, so it stays there.',
          f'- Getting back up needs the axle to accelerate at g tan(th) = {9.81 * np.tan(np.radians(rest)):.1f} m/s2 '
          f'(tyre mu above {np.tan(np.radians(rest)):.2f}).', '']
    chk('Fall stops on the crash bars', abs(rf - rest) < 0.3 and abs(rb - rest) < 0.3 and abs(abs(pf[0]) - SKID_X) < SKID_R + 0.5,
        f'forward {rf:.1f} deg, back {rb:.1f} deg, both on a bar (target near 25 deg)')
    # ---- clashes
    solids = [i for i in its if i.solid is not None]
    allowed = [{'tyre_l', 'rim_l'}, {'tyre_r', 'rim_r'}, {'holder', 'cells'}, {'encoder_ring', 'rim_l'}, {'encoder_ring', 'tyre_l'}]
    cl = []
    for a in range(len(solids)):
        for b in range(a + 1, len(solids)):
            A, B = solids[a], solids[b]
            pair = {A.name, B.name}
            if 'tt_shaft' in pair or pair in allowed:
                continue
            if any(n.startswith('m25x8') for n in pair) and 'imu' in pair:
                continue      # the screws pass through the board's holes (not modelled)
            if any(n.startswith('m3x16') for n in pair) and 'holder' in pair:
                continue
            if pair in ({'wire_motor_leads', 'k1_plug'}, {'wire_motor_leads_2', 'k1_plug'}, {'wire_motor_leads', 'c3'},
                        {'wire_tcrt_pigtail', 'tcrt5000'}, {'wire_tcrt_pigtail', 'tcrt_module'}, {'wire_tcrt_cable', 'tcrt_module'}):
                continue      # the leads end in the plug / pass C3's legs
            v = (A.solid ^ B.solid).volume()
            if v > 0.5:
                cl.append(f'{A.name} x {B.name} {v:.1f} mm3')
    L += ['## 4. Clearances', '', 'Every pair of solids (printed parts, motor, wheels, boards, fasteners, wires, strap) checked for '
          'overlap above 0.5 mm3: ' + ('**none**.' if not cl else '; '.join(cl) + '.'), '']
    chk('No clashes (incl. boards, fasteners, wires)', not cl, f'{len(cl)} overlapping pairs')
    env = wheel_envelope()
    gaps = {i.name: i.solid.min_gap(env, 10.0) for i in solids if i.group == 'body'}
    worst = sorted(gaps.items(), key=lambda kv: kv[1])[:4]
    L += ['- **Wheels through a full turn** (each wheel swept to a solid 65 mm cylinder): nearest body parts '
          + ', '.join(f'{n} {g:.2f} mm' for n, g in worst) + '.']
    chk('Wheel sweep clearance >= 1.0 mm', worst[0][1] >= 1.0, f'{worst[0][0]} {worst[0][1]:.2f} mm')
    allbody = W.union([i.solid for i in solids if i.group == 'body'])
    jx, jy, jz = J1_POS
    keep = {
        'USB plug (micro-USB, 11 x 8 mm overmould, 30 mm long)': (W.box(-PF - ESP_GAP + 1.5 - 4.0, -PF - ESP_GAP + 1.5 + 4.0, -ESP_Y0 + 0.7, -ESP_Y0 + 31, ESP_ZC - 5.5, ESP_ZC + 5.5), 'esp32'),
        'finger on the switch slider': (W.box(PF + 9.2, PF + 40, SW_Y0 + 4, SW_Y0 + 11.5, SW_Z0 + 7.5, SW_Z0 + 14), 'switch'),
        'charger plug into J1 (10 mm body, 30 mm long)': (W.zcyl(jx, jy, 5.0, jz - 30, jz - 0.3), 'j1_jack'),
        'K2 unplugged forward (finger grip 12 mm round it)': (fb(1, PF, 3.2 + XH[2] - 6, 3.2 + XH[2] + 0.0, JB_Y0 + 13 - 2.2, JB_Y0 + 13 + XH[0] + 2.2, JB_Z0 + 3.5 - 2.5, JB_Z0 + 3.5 + XH[1] + 2.5), 'jb'),
        'cells lifted out (strap off)': (W.union([W.ycyl(x, TRAY_Z0 + FLOOR_T + 10.2, 9.2, -32.5, 32.5).translate([0, 0, 30]) for x in (-10, 10)])
                                         + W.box(-19.2, 19.2, -32.5, 32.5, TRAY_Z0 + FLOOR_T + 10.2 + 3, TRAY_Z0 + FLOOR_T + 50), 'cells'),
    }
    for k, (box, own) in keep.items():
        others = W.union([i.solid for i in solids if i.group == 'body' and i.name not in (own, 'strap', 'holder') and not (own == 'jb' and i.name == 'wire_rail')])
        g = box.min_gap(others, 10.0)
        L += [f'- **{k}**: free space, nearest part {g:.1f} mm away.']
        chk(f'Access: {k.split(" (")[0]}', g > 0.2, f'{g:.1f} mm')
    L += ['- **Cells**: the strap must come off first; the holder stays screwed down.', '']
    # ---- milestone B: the wheel sensor
    byname = {i.name: i for i in its}
    ring_i, tc_i, mod_i = byname['encoder_ring'], byname['tcrt5000'], byname['tcrt_module']
    tcv = W.mesh_np(tc_i.solid)[0]
    face = tcv[:, 1].max()
    gap = RING_Y0 - face
    arc = 2 * np.pi * TCRT_R / (2 * RING_N)
    cheek_gap = ring_i.solid.min_gap(W.union([byname['core_left'].solid, byname['core_right'].solid]), 5.0)
    trim = fb(-1, CORE_X, 8.2, 45, -3.0, 3.0, MOD_Z0 + 17, MOD_Z0 + 21)
    others = W.union([i.solid for i in solids if i.group == 'body' and i.name != 'tcrt_module'])
    g_trim = trim.min_gap(others, 10.0)
    clip_low = W.mesh_np(byname['core_left'].solid)[0][:, 2].min()
    bset = [i for i in its if i.name in ('tcrt5000', 'tcrt_module', 'encoder_ring') or i.name.startswith('wire_tcrt')]
    mB = sum(mass(i) for i in bset) + 0.6
    L += ['## 4b. Milestone B: the wheel sensor', '',
          f'- **TCRT5000** [D 10.2 x 5.8 x 7 mm] in a clip under the core, lenses facing the left wheel, centred at '
          f'(x 0, z {-TCRT_R:.1f}) mm, lens face at y {face:.1f}. Its long axis (emitter beside detector) is radial, so a window '
          f'edge crosses both at once.',
          f'- **Gap, lens to ring: {gap:.2f} mm**, the TCRT5000\'s peak operating distance [D Vishay: 2.5 mm, range 0.2-15 mm].',
          f'- **Ring**: {RING_N} windows of {360 / (2 * RING_N):.2f} deg (sectors, so 50 % mark/space at every radius), windows r '
          f'{RING_WIN[0]:.1f}-{RING_WIN[1]:.1f} mm, read at r {TCRT_R} mm where a window is {arc:.1f} mm long. 32 edges per turn = '
          f'{2 * np.pi * R / 32:.1f} mm of travel per edge. {RING_T} mm thick, glued to the inside face of the left wheel.',
          f'- **Ring to the core**: {cheek_gap:.2f} mm (the left cheek face is recessed {RECESS} mm under the ring; the cheek over '
          f'the gearbox is {CHEEK - RECESS:.1f} mm there). Ring to the bolt heads (ISO 7380 button heads): '
          f'{RING_Y0 - (CH_HW - REC_DEPTH + HEAD_H):.2f} mm; with socket heads {RING_Y0 - (CH_HW - REC_DEPTH + 3.0):.2f} mm.',
          f'- **Ground clearance** under the clip, upright: {clip_low + R:.1f} mm (the axle is {R} mm up). It never touches in a fall '
          '(section 3: the crash bars land first).',
          f'- **Module trimmer**: screwdriver path from behind is clear, nearest part {g_trim:.1f} mm away. The module LEDs face back.',
          f'- **Mass of milestone B**: {mB:.1f} g (sensor, module, ring, wires, two ties). Milestone A is that much lighter; everything '
          'printed is the same for both.', '']
    chk('Sensor gap = TCRT5000 peak distance', abs(gap - TCRT_GAP) < 0.05, f'{gap:.2f} mm (peak 2.5 mm [D])')
    chk('Ring clear of the core', cheek_gap >= 1.0, f'{cheek_gap:.2f} mm')
    chk('Sensor reads inside the window band', RING_WIN[0] + 1.5 <= TCRT_R <= RING_WIN[1] - 1.5, f'r {TCRT_R} in {RING_WIN[0]:.1f}-{RING_WIN[1]:.1f}')
    chk('Trimmer reachable', g_trim > 0.5, f'{g_trim:.1f} mm')
    # ---- printed parts
    L += ['## 5. Printed parts (Ender 3: 220 x 220 x 250 bed, 0.4 mm nozzle)', '',
          '| Part | Size in print pose x, y, z (mm) | Mass (g) | Orientation, supports | Valid manifold | Pieces | z min | Fits bed |', '|---|---|---|---|---|---|---|---|']
    prints = {}
    for n, (fn, rot, note) in PRINTED.items():
        m = fn(); p = print_pose(m, rot); prints[n] = (m, p)
        v = W.mesh_np(p)[0]; s = v.max(0) - v.min(0)
        ok = str(p.status()).endswith('NoError'); pieces = len(p.decompose())
        fits = s[0] <= W.BED[0] - 10 and s[1] <= W.BED[1] - 10 and s[2] <= W.BED[2]
        L += [f'| {n} | {s[0]:.0f} x {s[1]:.0f} x {s[2]:.1f} | {m.volume() / 1000 * W.PLA * (1.0 if n == "encoder_ring" else W.FILL):.1f} | {note} | {"yes" if ok else "NO"} | {pieces} | {v[:, 2].min():.2f} | {"yes" if fits else "NO"} |']
        chk(f'{n}: one valid manifold on the bed', ok and pieces == 1 and abs(v[:, 2].min()) < 1e-6, f'{pieces} piece(s), z min {v[:, 2].min():.3f}')
        chk(f'{n}: fits the bed', fits, f'{s[0]:.0f} x {s[1]:.0f} x {s[2]:.0f} mm')
        W.to_stl(p, os.path.join(OUT, 'print', n + '.stl'))
    L += ['']
    # walls
    L += ['### Wall thickness', '', 'Ray-cast from 5000 random surface points straight into the material (print pose, first 0.6 mm above the bed '
          'skipped). Thin readings are listed with where they are.', '', '| Part | Thinnest (mm) | 1st percentile (mm) | Points under 1.6 mm |', '|---|---|---|---|']
    thin_notes = []
    for n, (m, p) in prints.items():
        if n == 'encoder_ring':
            L += [f'| {n} | {RING_T} (by design) | | thin by design: a flat ring glued to the wheel |']
            continue
        th, pts = wall_scan(p)
        thin = pts[th < 1.6]
        L += [f'| {n} | {th.min():.2f} | {np.percentile(th, 1):.2f} | {len(thin)} of {len(th)} |']
        if len(thin):
            # back to the body frame for the report
            rot = PRINTED[n][1]
            v0 = W.mesh_np(m)[0]; v1 = W.mesh_np(m.rotate(rot))[0]
            off = np.array([-(v1[:, 0].min() + v1[:, 0].max()) / 2, -(v1[:, 1].min() + v1[:, 1].max()) / 2, -v1[:, 2].min()])
            ang = np.radians(rot)
            def rx(a): c, s_ = np.cos(a), np.sin(a); return np.array([[1, 0, 0], [0, c, -s_], [0, s_, c]])
            Rot = rx(ang[0])
            bp = (thin - off) @ Rot      # inverse rotation (orthonormal)
            cells = {}
            for q, tq in zip(bp, th[th < 1.6]):
                k = tuple(np.round(q / 6).astype(int))
                cells.setdefault(k, []).append((tq, q))
            for k, lst in sorted(cells.items(), key=lambda kv: min(x[0] for x in kv[1])):
                tmin, q = min(lst, key=lambda x: x[0])
                thin_notes.append((n, tmin, q, len(lst)))
    L += ['']
    if thin_notes:
        L += ['Thin readings (body frame, mm):', '']
        for n, tmin, q, cnt in thin_notes[:30]:
            L += [f'- {n}: {tmin:.2f} mm at ({q[0]:.1f}, {q[1]:.1f}, {q[2]:.1f}), {cnt} point(s): {explain_thin(n, q)}']
        L += ['']
    real_thin = [t for t in thin_notes if explain_thin(t[0], t[2]).startswith('REAL')]
    chk('No real walls under 1.6 mm', not real_thin, f'{len(real_thin)} unexplained thin spots' if real_thin else 'all thin readings are edges or chamfers (listed)')
    # overhangs
    L += ['### Overhangs and bridges (print pose)', '',
          'Down-facing regions steeper than 45 deg that are not on the bed, larger than 4 mm2. "Flat" means a horizontal roof: '
          'a bridge if both ends are held, an overhang if not.', '', '| Part | Area (mm2) | Height (mm) | Size x, y (mm) | Flat | What it is |', '|---|---|---|---|---|---|']
    for n, (m, p) in prints.items():
        for A, z, sx, sy, flat in overhangs(p):
            L += [f'| {n} | {A:.0f} | {z:.1f} | {sx:.1f} x {sy:.1f} | {"yes" if flat else "no"} | {explain_overhang(n, z, sx, sy)} |']
    L += ['']
    # fastener walls
    L += ['### Fastener holes: wall to the next surface', '', '| Hole | Part | Size | Wall (mm) |', '|---|---|---|---|']
    L_, R_ = core_halves(); mst = mast(); try_ = tray()
    fw = []
    for x, z in BOLTS:
        fw.append((f'clamp bolt ({x:+.0f}, {z:+.0f})', L_, 'M3 clearance 3.4', hole_walls(L_, (x, 0, z), (0, 1, 0), M3_CLEAR_R, 0.0, CH_HW - REC_DEPTH)))
        top = CH_HW - (RECESS if RECESS_R0 < np.hypot(x, z) < RECESS_R1 else 0)
        fw.append((f'  head recess ({x:+.0f}, {z:+.0f})', L_, f'6.2 x {top - CH_HW + REC_DEPTH:.1f} deep', hole_walls(L_, (x, 0, z), (0, 1, 0), HEAD_REC_R, CH_HW - REC_DEPTH, top)))
        fw.append((f'  nut recess ({x:+.0f}, {z:+.0f})', R_, 'hex 5.8 AF x 3.2 deep', hole_walls(R_, (x, 0, z), (0, 1, 0), NUT_REC_AF / np.sqrt(3), -CH_HW, -CH_HW + REC_DEPTH)))
    for x, z in W.TT_HOLES:
        fw.append((f'pin ({x:+.0f}, {z:+.0f}) in the cheek', L_, '3.0 press fit', hole_walls(L_, (x, 0, z), (0, 1, 0), PIN_R, POCKET_HW, CH_HW)))
    fw.append(('shaft hole', L_, '8.0', hole_walls(L_, (0, 0, 0), (0, 1, 0), SHAFT_HOLE_R, POCKET_HW, CH_HW)))
    for x, y in MAST_SCREWS:
        prt = L_ if y > 0 else R_
        fw.append((f'mast screw pilot ({x:+.0f}, {y:+.0f})', prt, 'M3 self-tap 2.5 x 10 deep', hole_walls(prt, (x, y, 0), (0, 0, 1), M3_PILOT_R, CORE_Z1 - 10, CORE_Z1)))
        fw.append((f'mast foot hole ({x:+.0f}, {y:+.0f})', mst, 'M3 clearance 3.4', hole_walls(mst, (x, y, 0), (0, 0, 1), M3_CLEAR_R, FOOT_Z0, FOOT_Z1)))
    for y, z in IMU_PILOTS:
        prt = L_ if y > 0 else R_
        fw.append((f'IMU screw pilot ({y:+.1f}, {z:+.1f})', prt, 'M2.5 self-tap 2.1 x 8 deep', hole_walls(prt, (0, y, z), (1, 0, 0), M25_PILOT_R, CORE_X + IMU_POST - 7.5, CORE_X + IMU_POST)))
    for x, y in HOLDER_SCREWS:
        fw.append((f'holder screw boss ({x:+.0f}, {y:+.0f})', mst, 'M3 self-tap 2.5 x 12 deep', hole_walls(mst, (x, y, 0), (0, 0, 1), M3_PILOT_R, MAST_TOP - 12, MAST_TOP)))
        fw.append((f'tray floor hole ({x:+.0f}, {y:+.0f})', try_, 'M3 clearance 3.4', hole_walls(try_, (x, y, 0), (0, 0, 1), M3_CLEAR_R, TRAY_Z0, TRAY_Z0 + FLOOR_T)))
    for n, prt, size, wv in fw:
        L += [f'| {n} | {"core" if prt in (L_, R_) else "mast" if prt is mst else "tray"} | {size} | {wv:.2f} |']
    L += ['']
    wmin = min(f[3] for f in fw)
    chk('Fastener walls >= 1.6 mm', wmin >= 1.6, f'thinnest {wmin:.2f} mm ({min(fw, key=lambda f: f[3])[0].strip()})')
    # screw lengths
    seat = CH_HW - REC_DEPTH
    L += ['### Screw lengths', '',
          f'- **Clamp bolts M3 x 20**: head seat {seat:.1f} mm from the centre plane, head top {seat + HEAD_H:.1f} (cheek face {CH_HW}); nut from '
          f'{-seat:.1f} to {-seat - NUT_H:.1f}; bolt tip at {seat - BOLT_L:.1f}. Fully through the nut, nothing proud of either cheek.',
          f'- **Pins 3 x {PIN_L:.0f}**: ends at +-{PIN_L / 2:.1f}, {CH_HW - PIN_L / 2:.1f} mm below the cheek faces; {PIN_L / 2 - POCKET_HW:.1f} mm in each cheek.',
          f'- **Mast screws M3 x 12 self-tap**: 4 mm foot, {12 - 4:.0f} mm into a 10 mm pilot.',
          f'- **Holder screws M3 x 16 self-tap**: {HOLDER_FLOOR} mm holder floor [E] + {FLOOR_T} mm tray floor, {16 - HOLDER_FLOOR - FLOOR_T:.1f} mm into a 12 mm pilot.',
          f'- **IMU screws M2.5 x 8 self-tap**: 1.6 mm board, {8 - 1.6:.1f} mm into an 8 mm pilot.', '']
    chk('Nothing proud of the cheeks', seat + HEAD_H <= CH_HW - 0.1 and seat - BOLT_L >= -CH_HW + 0.1 and PIN_L / 2 <= CH_HW - 0.1,
        f'head top {seat + HEAD_H:.1f}, bolt tip {seat - BOLT_L:.1f}, pin end {PIN_L / 2:.1f} vs cheek {CH_HW}')
    # ---- robustness
    L += robustness(its, Mb, cb, Iax, chk)
    # ---- size and summary
    v = np.vstack([W.mesh_np(i.solid)[0] for i in solids])
    L += ['## 7. Size', '', f'{v[:, 0].max() - v[:, 0].min():.0f} mm front to back, {v[:, 1].max() - v[:, 1].min():.0f} mm wide, '
          f'{v[:, 2].max() + R:.0f} mm tall.', '']
    L += ['## 8. Summary', '', '| Check | Result | Detail |', '|---|---|---|'] + [f'| {a} | {b} | {c} |' for a, b, c in RES] + ['']
    head = ['', f'**{sum(r[1] == "PASS" for r in RES)} PASS, {sum(r[1] == "WARN" for r in RES)} WARN, {sum(r[1] == "FAIL" for r in RES)} FAIL** (table in section 8).', '']
    L = L[:4] + head + L[4:]
    open(os.path.join(OUT, 'design.md'), 'w').write('\n'.join(L))
    # ---- simulator interface (same keys as before)
    imu = com(next(i for i in its if i.name == 'imu')) * 1e-3
    hx, hy, hz = HOLDER
    json.dump(dict(body_mass=Mb, body_com=list(cb), body_inertia=Ib.tolist(), wheel_mass=Mw, wheel_inertia=Iw.tolist(),
                   r_wheel=R * 1e-3, wheel_in=WHEEL_IN * 1e-3, wheel_w=W.W_WHEEL * 1e-3, imu=list(imu),
                   skid=[SKID_X * 1e-3, SKID_Z * 1e-3, SKID_R * 1e-3, SKID_HW * 1e-3],
                   battery=[0, 0, (TRAY_Z0 + FLOOR_T + hz / 2) * 1e-3, hx / 2e3, hy / 2e3, hz / 2e3], rest_deg=float(rest)),
              open(os.path.join(OUT, 'model.json'), 'w'), indent=1)
    ring_svg(os.path.join(OUT, 'encoder_ring_template.svg'))
    W.to_stl(allbody, os.path.join(OUT, 'body.stl'))
    W.to_stl(W.union([i.solid for i in solids if i.group == 'wheels']), os.path.join(OUT, 'wheels.stl'))
    print('\n'.join(L))
    return its

def ring_svg(path):
    """1:1 paper version of the encoder ring (print at 100 %, no scaling): black bars, white windows."""
    cs = encoder_ring_flat().slice(RING_T / 2)
    d = ' '.join('M ' + ' L '.join(f'{x + 30:.3f},{30 - y:.3f}' for x, y in P) + ' Z' for P in cs.to_polygons())
    open(path, 'w').write(f'<svg xmlns="http://www.w3.org/2000/svg" width="60mm" height="66mm" viewBox="0 0 60 66">'
                          f'<path d="{d}" fill="black" fill-rule="evenodd"/>'
                          f'<text x="30" y="63" font-size="2.6" text-anchor="middle" font-family="sans-serif">Stage 1 encoder ring, '
                          f'{RING_R1 * 2:.0f} mm across: print at 100 %</text></svg>')

def explain_thin(n, q):
    """Classify a thin reading by where it is (body frame). Anything not recognised is reported as REAL."""
    x, y, z = q
    if n.startswith('core'):
        if z < CORE_Z0 - 0.5 and abs(x) < 6:
            return 'TCRT5000 clip wall (1.8 mm) or its edge'
        if abs(abs(x) - SKID_X) < 6 or abs(x) > CORE_X + 1:
            return 'crash bar / arm edge (rounded edge, ray leaves through the adjacent face)'
        if abs(y) < 0.7:
            return 'pocket-mouth chamfer at the bed face'
        if abs(abs(y) - CH_HW) < 0.5:
            return 'cheek face next to a recess or hole rim (edge)'
        if abs(abs(x) - CORE_X) < 1.0 and abs(abs(y) - CH_HW) > 0.5:
            return 'tie-tunnel bar or post edge'
        return 'REAL: check'
    if n == 'mast':
        return 'rail, pad or anchor edge' if abs(x) > PF - 0.1 else 'REAL: check'
    if n == 'tray':
        return 'rail or anchor edge' if x > TX - 0.1 else 'REAL: check'
    return 'REAL: check'

def explain_overhang(n, z, sx, sy):
    if n.startswith('core'):
        if abs(z - (POCKET_HW)) < 0.6:
            return 'gearbox pocket roof: bridge across the pocket (the cheek)'
        if abs(z - (W.TT_CAN_FLAT / 2 + CLR)) < 0.6:
            return 'can pocket roof: bridge'
        if abs(z - 6.0) < 0.6:
            return 'solder-tab pocket roof: bridge'
        if abs(z - 3.5) < 0.6:
            return 'motor-lead slot roof: bridge'
        if abs(z - (CH_HW - REC_DEPTH)) < 0.6:
            return 'bolt head / nut recess floor round the 3.4 hole (short bridge)'
        if z < 9:
            return 'crash-arm / web underside'
        return 'hole roof or rounded edge'
    if n == 'mast':
        if z < 3.0:
            return 'top of the foot ring round a hole (bridge) / foot edge'
        return 'rail, pad, anchor or tie-slot roof (3.6 mm or less)'
    if n == 'tray':
        return 'hole or notch roof'
    return ''

def impact(Ib, mt, zt, km, kc, xc, w0, T=0.03, dt=1e-6):
    """Two-mass fall impact, SI units. The body (inertia Ib about the axle, axle held still: conservative) lands on a
    contact spring kc at xc from the axle; the mass mt riding on the mast at height zt is tied to the body by the
    mast's bending spring km. Starts at first touch with angular speed w0. Returns peak contact and mast forces (N)."""
    ph, phd, u, ud = 0.0, w0, 0.0, 0.0
    fc = fm = 0.0
    for _ in range(int(T / dt)):
        Mc = kc * xc * ph * xc if ph > 0 else 0.0
        phdd = (-Mc + km * u * zt) / Ib
        udd = -km * u / mt - zt * phdd
        phd += phdd * dt; ud += udd * dt; ph += phd * dt; u += ud * dt
        fc = max(fc, kc * xc * ph); fm = max(fm, km * abs(u))
        if ph < 0 and phd < 0 and abs(u) < 1e-7:
            break
    return fc, fm

def robustness(its, Mb, cb, Iax, chk):
    """Falls onto the crash bars: a small two-mass impact model with the stiffnesses of the printed sections."""
    L = ['## 6. Robustness: falls', '',
         f'Printed PLA [E]: E = {E_PLA:.0f} MPa, strength {S_ALONG:.0f} MPa along the layers and {S_ACROSS:.0f} MPa across them. '
         'Model: the body pivots on a fixed axle (the wheels cannot roll away, which makes it worse than real), lands on '
         'the crash bar through the bending stiffness of the arm (rigid floor), and the battery, tray and ESP32 ride on the mast\'s '
         'own bending spring. Peak forces come from integrating that through the impact; stress = F L c / I on sections '
         'sliced from the meshes. Two falls: a topple from upright (released at rest), and a hard fall (the same after a '
         '3 rad/s shove, about twice the largest shove in the simulator).', '']
    th = np.radians(24.7)
    w_soft = np.sqrt(2 * Mb * 9.81 * cb[2] * (1 - np.cos(th)) / Iax)
    w_hard = np.sqrt(3.0 ** 2 + w_soft ** 2)
    L += [f'- Body angular speed when the bar lands: {w_soft:.2f} rad/s (topple), {w_hard:.2f} rad/s (hard). Body energy '
          f'{0.5 * Iax * w_soft ** 2 * 1e3:.0f} / {0.5 * Iax * w_hard ** 2 * 1e3:.0f} mJ.', '']
    L_, R_ = core_halves(); core_m = L_ + R_
    # crash arm root: slice just outside the core, bending in x-z
    A, uc, I_arm, c_arm = section(core_m.rotate([0, -90, 0]).slice(CORE_X + 3.0))
    L_arm = SKID_X - CORE_X - 3.0
    k_arm = 3 * E_PLA * I_arm / L_arm ** 3
    # bar end beyond the arm (one end lands first when it falls with some roll): slice at y = 12
    A2, vc2, I_end, c_end = section_v((core_m ^ W.box(SKID_X - 35, SKID_X + 10, -50, 50, -50, 50)).rotate([90, 0, 0]).slice(12.0))
    L_end = SKID_HW - 12.0
    k_end = 3 * E_PLA * I_end / L_end ** 3
    # mast: the mass riding on it, and its weakest section
    above = [i for i in its if i.group == 'body' and com(i)[2] > FOOT_Z1 + 1 and i.name != 'mast']
    m_top = sum(mass(i) for i in above) * 1e-3
    z_top = sum(mass(i) * com(i)[2] for i in above) / (m_top * 1e3)
    mst = mast()
    secs = []
    for zc in (FOOT_Z1 + 0.5, GUSSET_TOP + 1.0, 110.0):
        A3, uc3, I3, c3 = section(mst.slice(zc)); secs.append((zc, I3, c3))
    zc, I_m, c_m = min(secs, key=lambda s_: s_[1] / (z_top - s_[0]) / s_[2])
    L_m = z_top - zc
    k_m = 3 * E_PLA * I_m / L_m ** 3
    Ib = Iax - m_top * (z_top * 1e-3) ** 2
    rows = ['| Part | Section I (mm4) | c (mm) | Lever (mm) | k (N/mm) | Peak force topple / hard (N) | Stress topple / hard (MPa) | Layers | Safety factor (hard) |',
            '|---|---|---|---|---|---|---|---|---|']
    sfs = []
    def row(name, I, c, Lv, k, f1, f2, allow, layer):
        s1, s2 = f1 * Lv * c / I, f2 * Lv * c / I
        rows.append(f'| {name} | {I:.0f} | {c:.1f} | {Lv:.0f} | {k:.0f} | {f1:.0f} / {f2:.0f} | {s1:.1f} / {s2:.1f} | {layer} | {allow / s2:.1f} |')
        sfs.append((name, allow / s2))
    # whole bar lands: contact spring = arm
    r1 = [impact(Ib, m_top, z_top * 1e-3, k_m * 1e3, k_arm * 1e3, SKID_X * 1e-3, w) for w in (w_soft, w_hard)]
    # one bar end lands: arm and bar end in series
    kse = 1 / (1 / k_arm + 1 / k_end)
    r2 = [impact(Ib, m_top, z_top * 1e-3, k_m * 1e3, kse * 1e3, SKID_X * 1e-3, w) for w in (w_soft, w_hard)]
    row('Crash arm root (bar lands square)', I_arm, c_arm, L_arm, k_arm, r1[0][0], r1[1][0], S_ALONG, 'along')
    row('Crash bar end (one end lands first)', I_end, c_end, L_end, k_end, r2[0][0], r2[1][0], S_ACROSS, 'across')
    row(f'Mast at z {zc:.0f} (its weakest section)', I_m, c_m, L_m, k_m, max(r1[0][1], r2[0][1]), max(r1[1][1], r2[1][1]), S_ACROSS, 'across')
    L += rows + ['']
    f_m = np.sqrt(k_m * 1e3 / m_top) / (2 * np.pi)
    m_old = sum(mass(i) for i in above if i.name not in ('mast',)) * 1e-3
    L += [f'- The mast carries {m_top * 1e3:.0f} g (tray, holder, cells, strap, switch, ESP32, BMS, buck, wires) with its centre at '
          f'z {z_top:.0f} mm. Bending stiffness, sliced: ' + ', '.join(f'z {s_[0]:.0f}: I {s_[1]:.0f} mm4' for s_ in secs) +
          f'. The old 3 mm plate was {62 * 27 / 12:.0f} mm4.',
          f'- First bending mode of the loaded mast, rough: {f_m:.0f} Hz (the old plate, same load: '
          f'{np.sqrt(3 * E_PLA * 62 * 27 / 12 / (z_top - 76) ** 3 * 1e3 / m_top) / (2 * np.pi):.0f} Hz). Far from the 4 Hz wobble.',
          f'- Mast to core: the hard-fall mast force {max(r1[1][1], r2[1][1]):.0f} N acts {z_top - FOOT_Z1:.0f} mm above the foot, '
          f'{max(r1[1][1], r2[1][1]) * (z_top - FOOT_Z1):.0f} N mm. Over the 32 mm between the screw rows that is '
          f'{max(r1[1][1], r2[1][1]) * (z_top - FOOT_Z1) / 32 / 2:.0f} N per M3 self-tapper, against a few hundred N to pull one out of 8 mm of PLA [E].',
          f'- The same force on the tray and holder: {max(r1[1][1], r2[1][1]):.0f} N, carried by the tray floor bearing on the mast top '
          'and the two M3 screws in shear. A side fall onto the tray is the larger unknown: drop-test it on carpet first.']
    per = 2 * (W.TT_W + 2 * CLR + (GB_Z1 - GB_Z0) + 2 * CLR)
    Fs = 0.34 * 9.81 * 30
    L += [f'- Cheeks ({CHEEK} mm): a sideways landing on one wheel (0.34 kg at 30 g, {Fs:.0f} N [E]) pushes the gearbox into the far '
          f'cheek, which is backed by the whole gearbox face: under 0.2 MPa in bearing, and {Fs / (per * CHEEK):.2f} MPa sheared round the '
          f'{per:.0f} mm pocket edge. Pressing a wheel on (50 N [E]) is smaller still. The cheek is not a strength problem; '
          'its risk is print quality, because it prints as a bridge (section 5).', '']
    worst = min(sfs, key=lambda s_: s_[1])
    chk('Falls: safety factor >= 1.5 (hard fall)', worst[1] >= 1.5, f'lowest {worst[1]:.1f} ({worst[0]})')
    return L

def section_v(cs):
    """Like section() but about the u axis (int (v - vc)^2 dA): used for the bar slice, where the load is along v."""
    A = Sv = Ivv = 0.0
    vs = []
    for P in cs.to_polygons():
        P = np.asarray(P, float); u, v = P[:, 0], P[:, 1]; u2, v2 = np.roll(u, -1), np.roll(v, -1)
        cr = u * v2 - u2 * v
        A += cr.sum() / 2; Sv += ((v + v2) * cr).sum() / 6; Ivv += ((v * v + v * v2 + v2 * v2) * cr).sum() / 12
        vs.append(v)
    vc = Sv / A; I = Ivv - A * vc ** 2
    vs = np.concatenate(vs)
    return A, vc, I, max(vs.max() - vc, vc - vs.min())

# =============================================================================================== renders
def renders(its):
    import render as RP
    from PIL import Image
    def posed(pitch=0, shift=(0, 0, 0), sel=None):
        Tb = W.T(shift[0], shift[1], R) @ W.Ry(pitch)
        return [(RP.It(i.name, i.solid, i.colour, i.group), W.apply(i.solid, Tb)) for i in its if i.solid is not None and (sel is None or sel(i))]
    hero = (330, -420, 240)
    img = RP.render(posed(), {'h': (hero, RP.look(hero, (0, 0, 95)), 30)})['h']
    RP.label(img, 'Stage 1, front: IMU on the core; BMS, switch and junction board on the mast').save(os.path.join(OUT, 'render_hero.png'))
    back = (-360, 330, 230)
    img = RP.render(posed(), {'h': (back, RP.look(back, (0, 0, 95)), 30)})['h']
    RP.label(img, 'Back: ESP32, MX1508 and buck on the mast; K1 and the wheel-sensor module on the core').save(os.path.join(OUT, 'render_back.png'))
    rest = json.load(open(os.path.join(OUT, 'model.json')))['rest_deg']
    tiles = []
    for nm, p in (('Side, balancing', 0), (f'Fallen forward onto the crash bar ({rest:.0f} deg)', rest)):
        cam = (0, -800, 100)
        img = RP.render(posed(p), {'s': (cam, RP.look(cam, (0, 0, 100)), 260.0)}, size=(900, 800), ortho=True)['s']
        tiles.append(RP.label(img, nm, sz=24))
    for nm, cam in (('From the front', (800, 0, 100)), ('From behind', (-800, 0, 100))):
        img = RP.render(posed(), {'s': (cam, RP.look(cam, (0, 0, 100)), 260.0)}, size=(900, 800), ortho=True)['s']
        tiles.append(RP.label(img, nm, sz=24))
    sheet = Image.new('RGB', (1800, 1600), 'white')
    for n, im in enumerate(tiles):
        sheet.paste(im, ((n % 2) * 900, (n // 2) * 800))
    sheet.save(os.path.join(OUT, 'render_views.png'))
    # exploded
    off = {'core_left': (0, 55, 0), 'core_right': (0, -55, 0), 'tt_gearbox': (0, 0, 0), 'tt_can': (0, 0, 0), 'tt_shaft': (0, 0, 0),
           'tyre_l': (0, 105, 0), 'rim_l': (0, 105, 0), 'tyre_r': (0, -105, 0), 'rim_r': (0, -105, 0),
           'mast': (0, 0, 70), 'esp32': (-30, 0, 70), 'bms_2s': (30, 0, 70), 'mini360': (-30, 0, 70), 'mx1508': (-30, 0, 70),
           'switch': (30, 0, 70), 'jb': (30, 0, 70), 'j1_jack': (30, 12, 70), 'imu': (35, 0, 0), 'k1_plug': (-30, 0, 0), 'c3': (0, 0, 0),
           'tray': (0, 0, 115), 'holder': (0, 0, 150), 'cells': (0, 0, 185)}
    ex = []
    for i in its:
        if i.solid is None or i.kind == 'wire' or i.name == 'strap':
            continue
        if i.kind == 'fastener':
            n = i.name
            d = (0, 85, 0) if n.startswith('m3x20') else (0, -85, 0) if n.startswith('m3_nut') else (0, 0, 0) if n.startswith('pin') else \
                (0, 0, 95) if n.startswith('m3x12') else (0, 0, 175) if n.startswith('m3x16') else (55, 0, 0)
        elif i.name in off:
            d = off[i.name]
        else:
            continue
        ex.append((RP.It(i.name, None, i.colour), i.solid.translate([d[0], d[1], d[2] + R])))
    cam = (440, -640, 470)
    img = RP.render(ex, {'h': (cam, RP.look(cam, (0, 0, 165)), 37)}, size=(1400, 1200))['h']
    RP.label(img, 'Exploded: core halves, pins and bolts, motor, mast with boards, tray, holder, cells').save(os.path.join(OUT, 'render_parts.png'))
    # each printed part in its print pose on a 220 x 220 bed
    tiles = []
    bed = W.box(-110, 110, -110, 110, 0.5, 2.0)
    for n, (fn, rot, note) in PRINTED.items():
        p = print_pose(fn(), rot)
        col = C['body'] if n.startswith('core') else (0.15, 0.15, 0.16) if n == 'encoder_ring' else C['body2']
        pr = [(RP.It('bed', None, (0.80, 0.82, 0.85)), bed), (RP.It(n, None, col), p.translate([0, 0, 2.0]))]
        cam = (150, -200, 170) if n.startswith('core') else (45, -60, 75) if n == 'encoder_ring' else (110, -150, 130)
        img = RP.render(pr, {'h': (cam, RP.look(cam, (0, 0, 12 if n != 'encoder_ring' else 2)), 40)}, size=(900, 700))['h']
        tiles.append(RP.label(img, f'{n}: {note}', sz=22))
    sheet = Image.new('RGB', (2700, 1400), 'white')
    for n, im in enumerate(tiles):
        sheet.paste(im, ((n % 3) * 900, (n // 3) * 700))
    sheet.save(os.path.join(OUT, 'render_print.png'))
    # close-ups of the mounts: core front and back without the wheels and mast
    sel = lambda i: i.group == 'body' and i.name not in ('tray', 'holder', 'cells', 'strap', 'switch') and not i.name.startswith('m3x16')
    tiles = []
    for nm, cam, tgt in (('Core front: IMU on two screws, its cable tied up the face', (190, -130, 95), (15, 0, 35)),
                         ('Core back: motor leads out of the back into K1, tied', (-190, 130, 95), (-15, 0, 45)),
                         ('Mast front: BMS, switch, junction board on rails; J1 on the flange', (230, -130, 185), (0, 0, 102)),
                         ('Mast back: ESP32 on corner pads, MX1508 and buck on rails; gusset', (-230, 130, 185), (0, 0, 102))):
        img = RP.render(posed(sel=sel), {'h': ((cam[0], cam[1], cam[2] + R), RP.look(cam, tgt), 34)}, size=(900, 700))['h']
        tiles.append(RP.label(img, nm, sz=22))
    sheet = Image.new('RGB', (1800, 1400), 'white')
    for n, im in enumerate(tiles):
        sheet.paste(im, ((n % 2) * 900, (n // 2) * 700))
    sheet.save(os.path.join(OUT, 'render_mounts.png'))
    # milestone B: the sensor and ring. Left: from below and in front, left wheel pulled out 45 mm along the axle so its
    # ring shows. Right: section through x = 0 seen from the front (cheek, gap, ring, wheel, lens).
    out_l = lambda i: i.name in ('tyre_l', 'rim_l', 'encoder_ring')
    pr = [(RP.It(i.name, None, i.colour), W.apply(i.solid.translate([0, 45 if out_l(i) else 0, 0]), W.T(0, 0, R)))
          for i in its if i.solid is not None and i.name not in ('tray', 'holder', 'cells', 'strap')]
    cam = (110, -20, 7)
    img = RP.render(pr, {'h': (cam, RP.look(cam, (0, 22, R - 16)), 42)}, size=(900, 800))['h']
    t1 = RP.label(Image.fromarray(img), '', sz=20)
    t1.save(os.path.join(OUT, 'step_e.png'))
    from PIL import ImageDraw
    d = ImageDraw.Draw(t1); d.rectangle([12, 10, 880, 46], fill=(255, 255, 255))
    t1 = RP.label(t1, 'TCRT5000 in its clip under the core; ring on the left wheel (pulled out 45 mm)', sz=20)
    cut = W.box(-200, 0.0, -200, 200, -200, 400)
    pr = [(RP.It(i.name, None, i.colour), W.apply(i.solid ^ cut, W.T(0, 0, R))) for i in its
          if i.solid is not None and i.name not in ('tray', 'holder', 'cells', 'strap') and not (i.solid ^ cut).is_empty()]
    cam = (600, 8, -17 + R)
    img = RP.render(pr, {'s': (cam, RP.look(cam, (0, 8, -17 + R)), 34.0)}, size=(900, 800), ortho=True)['s']
    t2 = RP.label(img, f'Section at x = 0 from the front: lens {TCRT_GAP} mm from the ring, ring 1.2 mm from the cheek', sz=20)
    sheet = Image.new('RGB', (1800, 800), 'white'); sheet.paste(t1, (0, 0)); sheet.paste(t2, (900, 0))
    sheet.save(os.path.join(OUT, 'render_sensor.png'))
    # build steps for the website
    hero = (300, -380, 230)
    def boxed(img, text, sz):
        from PIL import ImageDraw
        im = Image.fromarray(img) if not isinstance(img, Image.Image) else img
        d = ImageDraw.Draw(im); w = d.textlength(text, font=RP.font(sz))
        d.rectangle([12, 10, 36 + w, 26 + sz], fill=(255, 255, 255))
        return RP.label(im, text, sz=sz)
    def step(fname, names, caption, target=(0, 0, 80), extra=None, fov=30, cam=None):
        sel_ = [i for i in its if i.solid is not None and names(i)]
        pr = [(RP.It(i.name, None, i.colour), W.apply(i.solid, W.T(0, 0, R))) for i in sel_]
        if extra:
            pr += extra
        c = cam or hero
        img = RP.render(pr, {'h': (c, RP.look(c, target), fov)}, size=(1200, 900))['h']
        boxed(img, caption, 26).save(os.path.join(OUT, fname))
    A = lambda i: i.name in ('core_right', 'tt_gearbox', 'tt_can', 'tt_shaft', 'c3') or i.name.startswith(('m3_nut', 'pin_3x22')) \
        or i.name == 'wire_motor_leads'
    step('step_a.png', A, 'Motor, C3, pins and nuts in the right core half', target=(0, 0, 25 + R), fov=28, cam=(230, 300, 190))
    Bc = lambda i: A(i) or i.name in ('core_left', 'tyre_l', 'rim_l', 'tyre_r', 'rim_r', 'imu', 'k1_plug') \
        or i.name.startswith(('m3x20_bolt', 'm25x8'))
    step('step_b.png', Bc, 'Core closed, IMU on, wheels on', target=(0, 0, 30), fov=28)
    Cc = lambda i: Bc(i) or i.name in ('mast', 'esp32', 'mx1508', 'mini360', 'bms_2s', 'switch', 'jb', 'j1_jack') \
        or i.name.startswith('m3x12')
    step('step_c.png', Cc, 'Boards on the mast, mast on the core (wires not shown)', target=(0, 0, 75))
    step('step_d.png', lambda i: i.name not in ('tcrt5000', 'tcrt_module', 'encoder_ring') and not i.name.startswith('wire_tcrt'),
         'Wired, tray, battery and strap on: milestone A', target=(0, 0, 90))
    boxed(Image.open(os.path.join(OUT, 'step_e.png')), 'Milestone B: wheel sensor under the core, ring on the left wheel', 20).save(os.path.join(OUT, 'step_e.png'))

if __name__ == '__main__':
    its = analyse()
    if '--no-render' not in sys.argv:
        renders(its)
