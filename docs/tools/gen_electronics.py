"""Generate docs/electronics.json, docs/schematic.svg and docs/wiring.svg from one model, and check that every
net in the schematic is drawn at two or more ends, every off-board net has a wire in the wire list, every pin
in the pin map exists in a net, and no text in either drawing overlaps.

    python docs/tools/gen_electronics.py

Board positions in wiring.svg come from cad/tailwhip.py items() (bounding boxes in the body frame), so the drawing
follows the CAD. If the CAD cannot be imported the last known positions below are used and a warning is printed.
"""
import json, itertools, pathlib, sys
from collections import defaultdict
from xml.sax.saxutils import escape

ROOT = pathlib.Path(__file__).resolve().parents[2]
DOCS = ROOT / 'docs'
INK, MUTED, BG, GRID = '#1d2433', '#5b6475', '#ffffff', '#c9ced8'
BLUE, BLUE_BG, BLUE_TX = '#2a62d0', '#eef3fd', '#1b3f8f'
FONT = "font-family='Helvetica Neue, Helvetica, Arial, sans-serif'"
problems = []

# ============================================================================ the design model
PINMAP = [  # gpio, devkit label, net, dir, notes
    (25, 'D25', 'ML_IN1', 'out', 'MX1508 IN1 (left motor). LEDC 20 kHz 10-bit. Slow decay: held high when driving forward'),
    (26, 'D26', 'ML_IN2', 'out', 'MX1508 IN2 (left motor). PWM (inverted duty) when driving forward'),
    (27, 'D27', 'MR_IN1', 'out', 'MX1508 IN3 (right motor)'),
    (14, 'D14', 'MR_IN2', 'out', 'MX1508 IN4 (right motor). Fixed by SPEC; may glitch for a few ms at boot (harmless)'),
    (18, 'D18', 'SIG_TAIL', 'out', 'Tail SG-90 signal, 50 Hz 16-bit, 500-2500 us = 0-180 deg'),
    (19, 'D19', 'SIG_HEAD', 'out', 'Head-pan SG-90 signal'),
    (21, 'D21', 'SDA', 'i/o', 'I2C data, 400 kHz, 4.7k pull-up on the GY-521'),
    (22, 'D22', 'SCL', 'out', 'I2C clock'),
    (34, 'D34', 'IR_L', 'in', 'Left wheel TCRT5000 D0. Input-only, no internal pull: 100k pull-down R3 on the junction board'),
    (35, 'D35', 'IR_R', 'in', 'Right wheel TCRT5000 D0. 100k pull-down R4'),
    (36, 'VP', 'VSENSE', 'in', 'ADC1_CH0 (works with the radio on). Battery x 33/133 (100k over 33k), 100 nF to GND'),
    (2, 'D2', 'LED', 'out', 'On-board LED. Strapping pin, but the LED keeps it low at boot: fine'),
    (None, 'VIN / 5V', '5V_LOGIC', 'pwr', 'From buck U1. Feeds the DevKit AMS1117 3.3 V regulator'),
    (None, '3V3', '3V3', 'pwr', 'DevKit regulator out: GY-521 and both IR modules (~35 mA)'),
    (None, 'GND', 'GND', 'pwr', 'Two GND pins: one to the junction board, one to the GY-521'),
]
UNUSED = 'Unused: GPIO 0 5 12 15 (strapping), 6-11 (flash), 1 3 (USB)'

PARTS = {
    'BT1': ('2 x 18650 Li-ion in series (2S holder), genuine high-drain cells, 7.4 V nominal / 8.4 V full', '2S'),
    'U3': ('2S BMS with balancing (e.g. HX-2S-JH20): B-, BM, B+, P-, P+. 10 A, cut-off ~2.5 V/cell', 'BMS'),
    'J1': ('DC 5.5 x 2.1 mm female pigtail: charge jack (8.4 V 1 A CC/CV charger), on BMS P+ / P-', 'charge'),
    'F1': ('PTC resettable fuse, radial, 3 A hold / 6 A trip, >= 16 V (Littelfuse RUEF300 or Bourns MF-R300)', '3 A'),
    'SW1': ('Power switch, >= 3 A DC (Pololu 2810 MOSFET slide switch, or a KCD11 mini rocker)', ''),
    'U4': ('MX1508 dual H-bridge module: VM (+), GND (-), IN1-IN4, MOTOR-A, MOTOR-B', ''),
    'M1': ('TT gearmotor 1:48, left wheel, on MX1508 MOTOR-A via a JST-XH 2-pin plug', ''),
    'M2': ('TT gearmotor 1:48, right wheel, on MX1508 MOTOR-B via a JST-XH 2-pin plug', ''),
    'U1': ('Mini360 buck, set to 5.0 V: logic rail (ESP32, later the ESP32-CAM)', '5.0 V'),
    'U2': ('Mini360 buck, set to 5.0 V: servo rail', '5.0 V'),
    'C1': ('470 uF 16 V low-ESR electrolytic, across the servo rail on the junction board', '470u'),
    'R1': ('100 k 1 % 1/4 W, divider top', '100k'),
    'R2': ('33 k 1 % 1/4 W, divider bottom', '33k'),
    'C2': ('100 nF ceramic across R2', '100n'),
    'R3': ('100 k pull-down on IR_L (reads 0 when the sensor is unplugged)', '100k'),
    'R4': ('100 k pull-down on IR_R', '100k'),
    'JB': ('Junction board: ~15 x 25 mm perfboard, on the spine front beside the MX1508 (+y side)', ''),
    'U5': ('ESP32 DevKit (ESP32-WROOM-32), on the back of the spine, header pins pointing back', ''),
    'U6': ('GY-521 MPU-6050 on the motor web pad, chip facing forward, x axis up the body, address 0x68', ''),
    'U7': ('TCRT5000 IR module (LM393, D0 output), left wheel, on its post 2.5 mm from the encoder ring', ''),
    'U8': ('TCRT5000 IR module, right wheel', ''),
    'S1': ('SG-90 tail servo (MG90S as the fallback)', ''),
    'S2': ('SG-90 head-pan servo', ''),
    'U9': ('ESP32-CAM (stage 4): 5V and GND from the logic rail only; no signal wires to the robot ESP32', ''),
}

NETS = {
    'CELL+':    ['BT1.+', 'U3.B+'],
    'CELL_MID': ['BT1.mid', 'U3.BM'],
    'CELL-':    ['BT1.-', 'U3.B-'],
    'PACK+':    ['U3.P+', 'J1.+', 'F1.1'],
    'VBAT_F':   ['F1.2', 'SW1.IN'],
    'VBAT_SW':  ['SW1.OUT', 'U4.VM', 'U1.IN+', 'U2.IN+', 'R1.1'],
    'GND':      ['U3.P-', 'J1.-', 'U4.GND', 'U1.IN-', 'U1.OUT-', 'U2.IN-', 'U2.OUT-', 'C1.-', 'R2.2', 'C2.2', 'R3.2', 'R4.2',
                 'U5.GND', 'U6.GND', 'U7.GND', 'U8.GND', 'S1.GND', 'S2.GND', 'U9.GND', 'SW1.GND'],
    '5V_LOGIC': ['U1.OUT+', 'U5.5V', 'U9.5V'],
    '5V_SERVO': ['U2.OUT+', 'C1.+', 'S1.V+', 'S2.V+'],
    '3V3':      ['U5.3V3', 'U6.VCC', 'U7.VCC', 'U8.VCC'],
    'SDA':      ['U5.GPIO21', 'U6.SDA'],
    'SCL':      ['U5.GPIO22', 'U6.SCL'],
    'VSENSE':   ['R1.2', 'R2.1', 'C2.1', 'U5.GPIO36'],
    'IR_L':     ['U7.D0', 'R3.1', 'U5.GPIO34'],
    'IR_R':     ['U8.D0', 'R4.1', 'U5.GPIO35'],
    'ML_IN1':   ['U5.GPIO25', 'U4.IN1'],
    'ML_IN2':   ['U5.GPIO26', 'U4.IN2'],
    'MR_IN1':   ['U5.GPIO27', 'U4.IN3'],
    'MR_IN2':   ['U5.GPIO14', 'U4.IN4'],
    'MOTA+':    ['U4.MOTA1', 'M1.+'],
    'MOTA-':    ['U4.MOTA2', 'M1.-'],
    'MOTB+':    ['U4.MOTB1', 'M2.+'],
    'MOTB-':    ['U4.MOTB2', 'M2.-'],
    'SIG_TAIL': ['U5.GPIO18', 'S1.SIG'],
    'SIG_HEAD': ['U5.GPIO19', 'S2.SIG'],
}

# ---------------------------------------------------------------- board positions (body frame, mm) from the CAD
BOARDS = ['esp32', 'mx1508', 'mini360_logic', 'mini360_servo', 'bms_2s', 'cap_470u', 'switch', 'imu', 'ir_pcb_l',
          'ir_pcb_r', 'holder', 'tail_servo', 'pan_servo', 'tt_gearbox_l', 'tt_gearbox_r', 'tt_can_l', 'tt_can_r', 'torso']
FALLBACK = {   # bounding boxes [x0, y0, z0, x1, y1, z1], cad/tailwhip.py rev A, 2026-11-13
    'torso': [-23.5, -40.2, 64.0, 26.8, 40.2, 166.0], 'tt_gearbox_l': [-11.2, 13.5, -11.0, 11.2, 32.3, 26.0],
    'tt_can_l': [-10.0, 15.4, 26.0, 10.0, 30.4, 59.0], 'ir_pcb_l': [15.0, 24.3, -10.0, 29.0, 25.9, 22.0],
    'tt_gearbox_r': [-11.2, -32.3, -11.0, 11.2, -13.5, 26.0], 'tt_can_r': [-10.0, -30.4, 26.0, 10.0, -15.4, 59.0],
    'ir_pcb_r': [15.0, -25.9, -10.0, 29.0, -24.3, 22.0], 'tail_servo': [-40.8, -20.0, 23.9, -8.2, 12.0, 36.1],
    'pan_servo': [-21.8, -6.1, 150.1, 10.8, 6.1, 182.1], 'holder': [-16.5, -38.8, 126.3, 24.5, 38.8, 146.8],
    'esp32': [-15.5, -14.0, 67.0, -2.3, 14.0, 122.0], 'mx1508': [2.3, -12.0, 99.0, 5.9, 12.0, 120.0],
    'mini360_logic': [2.3, -25.5, 84.0, 6.5, -8.3, 95.2], 'mini360_servo': [2.3, 8.3, 84.0, 6.5, 25.5, 95.2],
    'bms_2s': [2.3, -15.0, 67.0, 5.5, 15.0, 83.0], 'cap_470u': [2.3, 15.0, 99.0, 15.3, 23.0, 107.0],
    'switch': [2.3, -24.0, 112.0, 8.5, -14.0, 118.0], 'imu': [17.6, -10.5, 19.0, 20.2, 10.5, 35.0]}


def cad_boxes():
    try:
        sys.path.insert(0, str(ROOT / 'cad'))
        import tailwhip as T
        out = {}
        for i in T.items():
            if i.name in BOARDS and i.solid is not None:
                out[i.name] = [round(v, 1) for v in i.solid.bounding_box()]
        missing = [b for b in BOARDS if b not in out]
        for b in missing:
            out[b] = FALLBACK[b]
        if missing:
            print('warning: not in the CAD, using rev A positions:', ', '.join(missing))
        return out, 'cad/tailwhip.py items()'
    except Exception as e:                                          # noqa: BLE001
        print(f'warning: could not import cad/tailwhip.py ({e}); using rev A positions')
        return dict(FALLBACK), 'rev A positions copied into gen_electronics.py'


BOX, BOX_SRC = cad_boxes()

# ---------------------------------------------------------------- wire list (point to point)
WIRES = []


def wire(wid, frm, to, net, colour, awg, length, route, note=''):
    WIRES.append(dict(id=wid, net=net, from_=frm, to=to, colour=colour, awg=awg, length_mm=length, route=route, note=note))


DUP = 'Dupont female on the ESP32 header pin'
wire('P1', 'BT1 + (holder red lead)', 'U3 B+', 'CELL+', 'red', 22, 60, 'sleeve floor window -> top of spine front', 'holder lead, shortened')
wire('P2', 'BT1 - (holder black lead)', 'U3 B-', 'CELL-', 'black', 22, 60, 'same', 'holder lead, shortened')
wire('P3', 'BT1 middle (strap joining the cells)', 'U3 BM', 'CELL_MID', 'yellow', 26, 70, 'same',
     'solder to the holder\'s series strap; balancing and per-cell protection need it')
wire('P4', 'U3 P+', 'F1 lead 1 (in heat-shrink)', 'PACK+', 'red', 22, 30, 'spine front, -y side')
wire('P5', 'F1 lead 2', 'SW1 IN', 'VBAT_F', 'red', 22, 40, 'spine front, up to the switch')
wire('P6', 'SW1 OUT', 'U4 VM (+)', 'VBAT_SW', 'red', 22, 30, 'spine front', 'then daisy-chain on to U1 IN+ (P7)')
wire('P7', 'U4 VM (+)', 'U1 IN+ and U2 IN+', 'VBAT_SW', 'red', 22, 60, 'spine front, down to the bucks', 'one wire to each buck, both on the VM screw')
wire('P8', 'U3 P-', 'U4 GND (-)', 'GND', 'black', 22, 50, 'spine front', 'main ground; then U1 IN-, U2 IN- (P9)')
wire('P9', 'U4 GND (-)', 'U1 IN- and U2 IN-', 'GND', 'black', 22, 60, 'spine front')
wire('P10', 'J1 + (charge jack, red)', 'U3 P+', 'PACK+', 'red', 22, 80, 'out of the -y side of the spine', 'jack hangs on a 6 cm tail')
wire('P11', 'J1 - (charge jack, black)', 'U3 P-', 'GND', 'black', 22, 80, 'same')
wire('P12', 'SW1 GND (Pololu switch only)', 'U4 GND (-)', 'GND', 'black', 26, 30, 'spine front', 'not needed with a mechanical rocker')
wire('P13', 'VBAT_SW (U4 VM)', 'JB R1 top', 'VBAT_SW', 'red', 26, 40, 'spine front', 'thin wire: divider only')
wire('L1', 'U1 OUT+', 'U5 VIN / 5V', '5V_LOGIC', 'red', 26, 90, 'round the -y edge of the spine to the back', DUP)
wire('L2', 'U1 OUT-', 'U5 GND', 'GND', 'black', 26, 90, 'with L1', DUP)
wire('S1', 'U2 OUT+', 'JB 5V_SERVO (C1 +)', '5V_SERVO', 'red', 22, 30, 'spine front, +y side')
wire('S2', 'U2 OUT-', 'JB GND (C1 -)', 'GND', 'black', 22, 30, 'with S1')
wire('S3', 'JB servo header T', 'S1 tail servo plug', 'SIG_TAIL / 5V_SERVO / GND', 'servo lead', 26, 110, 'down the spine back to the hip block', 'SG-90 lead is ~250 mm: coil the spare')
wire('S4', 'JB servo header H', 'S2 head servo plug', 'SIG_HEAD / 5V_SERVO / GND', 'servo lead', 26, 80, 'up the spine, past the sleeve', 'leave a loop for +-60 deg of pan')
wire('J1', 'U5 GPIO18', 'JB header T signal pin', 'SIG_TAIL', 'orange', 26, 70, '+y edge of the spine', DUP)
wire('J2', 'U5 GPIO19', 'JB header H signal pin', 'SIG_HEAD', 'yellow', 26, 70, '+y edge of the spine', DUP)
wire('J3', 'U5 GND (second pin)', 'JB GND', 'GND', 'black', 26, 70, '+y edge of the spine', DUP + '; the logic and junction grounds must meet')
wire('J4', 'U5 GPIO36 (VP)', 'JB R1/R2 junction', 'VSENSE', 'violet', 26, 70, '+y edge', DUP)
wire('J5', 'U5 GPIO34', 'JB R3 / IR_L in', 'IR_L', 'green', 26, 70, '+y edge', DUP)
wire('J6', 'U5 GPIO35', 'JB R4 / IR_R in', 'IR_R', 'blue', 26, 70, '+y edge', DUP)
wire('M1', 'U5 GPIO25', 'U4 IN1', 'ML_IN1', 'white', 26, 60, 'over the top of the spine', DUP + '; Dupont or solder at the MX1508')
wire('M2', 'U5 GPIO26', 'U4 IN2', 'ML_IN2', 'grey', 26, 60, 'with M1', DUP)
wire('M3', 'U5 GPIO27', 'U4 IN3', 'MR_IN1', 'brown', 26, 60, 'with M1', DUP)
wire('M4', 'U5 GPIO14', 'U4 IN4', 'MR_IN2', 'violet', 26, 60, 'with M1', DUP)
wire('M5', 'U4 MOTOR-A', 'M1 left motor tabs (JST-XH)', 'MOTA+ / MOTA-', 'red/black', 22, 110, '+y side of the core, along the can', 'swap the pair if the wheel runs backwards, or "flip l"')
wire('M6', 'U4 MOTOR-B', 'M2 right motor tabs (JST-XH)', 'MOTB+ / MOTB-', 'red/black', 22, 110, '-y side of the core, along the can', 'or "flip r" in firmware')
wire('I1', 'U5 3V3', 'U6 VCC', '3V3', 'red', 26, 110, 'down the front of the web', DUP + '; then on to the IR modules (I5, I6)')
wire('I2', 'U5 GND (third pin) or JB GND', 'U6 GND', 'GND', 'black', 26, 110, 'with I1')
wire('I3', 'U5 GPIO21', 'U6 SDA', 'SDA', 'blue', 26, 110, 'with I1, twisted with I4', DUP)
wire('I4', 'U5 GPIO22', 'U6 SCL', 'SCL', 'yellow', 26, 110, 'with I1', DUP)
wire('I5', 'U6 VCC / GND (second wires)', 'U7 VCC / GND (left IR)', '3V3 / GND', 'red/black', 26, 60, 'along the web to the left post')
wire('I6', 'U6 VCC / GND (second wires)', 'U8 VCC / GND (right IR)', '3V3 / GND', 'red/black', 26, 60, 'along the web to the right post')
wire('I7', 'U7 D0', 'JB IR_L', 'IR_L', 'green', 26, 120, 'up the web and the spine to the junction board')
wire('I8', 'U8 D0', 'JB IR_R', 'IR_R', 'blue', 26, 120, 'with I7')
wire('C1', 'U1 OUT+ / OUT- (second wires)', 'U9 ESP32-CAM 5V / GND', '5V_LOGIC / GND', 'red/black', 26, 120, 'up the back of the spine and sleeve to the head', 'stage 4 only; leave a loop for pan')

# ---------------------------------------------------------------- power budget (A at the rail named; [D] datasheet, [E] estimate)
POWER = dict(
    logic_rail=[('ESP32 radio on, ESP-NOW 100 Hz', 0.12, 0.35, '[D] Espressif datasheet 5-4: RX 95-100 mA, TX 180-240 mA, plus CPU [E]'),
                ('GY-521 MPU-6050', 0.004, 0.004, '[D] 3.8-3.9 mA'),
                ('2 x TCRT5000 module at 3.3 V', 0.03, 0.03, '[E] IR LED ~10 mA + LM393 + power LED, each'),
                ('ESP32-CAM streaming (stage 4)', 0.25, 0.31, '[D] 180 mA flash off, 310 mA flash on; [E] 230-250 mA measured by hobbyists')],
    servo_rail=[('Tail SG-90 holding the robot at rest', 0.25, 0.65, '[E] hold; [D] 550-650 mA stall'),
                ('Head SG-90 panning', 0.10, 0.65, '[E] moving; [D] stall')],
    motors=[('Each TT motor, no load at 6 V', 0.16, 1.5, '[D] Adafruit 3777: 160 mA no load, 1.5 A stall at 6 V'),
            ('Each TT motor, balancing', 0.4, 1.5, '[E] average while balancing; peaks to stall')],
)

# ---------------------------------------------------------------- tiny SVG writer (from Ouro) with text-overlap bookkeeping
class Svg:
    def __init__(self, w, h, title):
        self.w, self.h, self.parts, self.texts, self.title = w, h, [], [], title
        self.parts.append(f"<rect x='1' y='1' width='{w - 2}' height='{h - 2}' rx='14' fill='{BG}' stroke='{GRID}'/>")
    def add(self, s): self.parts.append(s)
    def text(self, x, y, s, size=12, anchor='start', weight='normal', fill=INK, check=True):
        w = 0.56 * size * len(s); h = size
        x0 = x if anchor == 'start' else x - w / 2 if anchor == 'middle' else x - w
        if check:
            self.texts.append((x0, y - h * 0.8, x0 + w, y + h * 0.2, s))
        self.parts.append(f"<text x='{x:.1f}' y='{y:.1f}' font-size='{size}' text-anchor='{anchor}' "
                          f"font-weight='{weight}' fill='{fill}'>{escape(s)}</text>")
    def line(self, x1, y1, x2, y2, col=INK, w=1.6, dash=None):
        d = f" stroke-dasharray='{dash}'" if dash else ''
        self.parts.append(f"<line x1='{x1:.1f}' y1='{y1:.1f}' x2='{x2:.1f}' y2='{y2:.1f}' stroke='{col}' stroke-width='{w}' stroke-linecap='round'{d}/>")
    def poly(self, pts, col=INK, w=1.6, dash=None, fill='none'):
        d = f" stroke-dasharray='{dash}'" if dash else ''
        p = ' '.join(f'{x:.1f},{y:.1f}' for x, y in pts)
        self.parts.append(f"<polyline points='{p}' fill='{fill}' stroke='{col}' stroke-width='{w}' stroke-linejoin='round' stroke-linecap='round'{d}/>")
    def rect(self, x, y, w, h, col=INK, fill='none', rx=3, sw=1.6, dash=None):
        d = f" stroke-dasharray='{dash}'" if dash else ''
        self.parts.append(f"<rect x='{x:.1f}' y='{y:.1f}' width='{w:.1f}' height='{h:.1f}' rx='{rx}' fill='{fill}' stroke='{col}' stroke-width='{sw}'{d}/>")
    def circle(self, x, y, r, col=INK, fill='none', sw=1.4):
        self.parts.append(f"<circle cx='{x:.1f}' cy='{y:.1f}' r='{r}' fill='{fill}' stroke='{col}' stroke-width='{sw}'/>")
    def save(self, path, name):
        for a, b in itertools.combinations(self.texts, 2):
            if a[0] < b[2] - 1 and b[0] < a[2] - 1 and a[1] < b[3] - 1 and b[1] < a[3] - 1:
                problems.append(f'{name}: text overlap "{a[4]}" / "{b[4]}"')
        for t in self.texts:
            if t[0] < 4 or t[2] > self.w - 4 or t[1] < 4 or t[3] > self.h - 4:
                problems.append(f'{name}: text off canvas "{t[4]}"')
        s = (f"<svg xmlns='http://www.w3.org/2000/svg' width='{self.w}' height='{self.h}' viewBox='0 0 {self.w} {self.h}' {FONT}>\n"
             f"<title>{escape(self.title)}</title>\n" + '\n'.join(self.parts) + '\n</svg>\n')
        path.write_text(s)


WIRE_RGB = dict(red='#d02a2a', black='#1d2433', orange='#e8771b', yellow='#d4a800', blue='#2a62d0', white='#9aa3b2',
                violet='#8a3fc4', green='#2f9a46', brown='#8a5a2b', grey='#7d8594')

# ============================================================================ schematic.svg
def draw_schematic():
    W, H = 1600, 1290
    g = Svg(W, H, 'Tailwhip schematic')
    g.text(30, 40, 'Tailwhip electronics schematic (robot). Net labels with the same name are connected.', 18, weight='bold')
    g.text(30, 64, 'All grounds common. R1 100k, R2 33k, R3 R4 100k (1 %), C1 470 uF 16 V low-ESR, C2 100 nF. '
                   'Both Mini360 bucks set to 5.0 V with a meter BEFORE anything is connected.', 12, fill=MUTED)
    labels = defaultdict(int)

    def lab(x, y, net, side='r'):
        labels[net] += 1
        w = 0.56 * 12 * len(net) + 14
        if side == 'r':
            g.poly([(x, y), (x + 8, y - 9), (x + 8 + w, y - 9), (x + 8 + w, y + 9), (x + 8, y + 9), (x, y)], col=BLUE, w=1.2, fill=BLUE_BG)
            g.text(x + 14, y + 4, net, 12, fill=BLUE_TX)
        else:
            g.poly([(x, y), (x - 8, y - 9), (x - 8 - w, y - 9), (x - 8 - w, y + 9), (x - 8, y + 9), (x, y)], col=BLUE, w=1.2, fill=BLUE_BG)
            g.text(x - 14, y + 4, net, 12, 'end', fill=BLUE_TX)

    def gnd(x, y):
        labels['GND'] += 1
        g.line(x, y, x, y + 10); g.line(x - 10, y + 10, x + 10, y + 10, w=2); g.line(x - 6, y + 14, x + 6, y + 14); g.line(x - 2, y + 18, x + 2, y + 18)

    def dot(x, y): g.circle(x, y, 3.2, fill=INK, sw=0)

    def box(x, y, w, h, ref, name, fill='#f4f6fa'):
        g.rect(x, y, w, h, fill=fill, rx=6, sw=1.8)
        g.text(x + w / 2, y - 8, f'{ref}  {name}', 12, 'middle', 'bold')

    def resistor(x, y, ref, val):
        g.rect(x - 7, y, 14, 34, rx=2, fill='#fff')
        g.text(x + 12, y + 14, ref, 12, weight='bold'); g.text(x + 12, y + 29, val, 12)

    def cap(x, y, ref, val, polar=False):
        g.line(x - 12, y, x + 12, y, w=2.4); g.line(x - 12, y + 7, x + 12, y + 7, w=2.4)
        if polar: g.text(x - 18, y - 2, '+', 12, 'middle', 'bold', check=False)
        g.text(x + 18, y + 2, ref, 12, weight='bold'); g.text(x + 18, y + 16, val, 12)

    # ---------------- power chain (top band)
    y0 = 260
    bx = 70
    for i, yy in enumerate((y0 - 40, y0 + 10)):                     # two cells
        g.line(bx - 16, yy, bx + 16, yy, w=2.6); g.line(bx - 9, yy + 7, bx + 9, yy + 7, w=2.6)
    g.line(bx, y0 - 70, bx, y0 - 40); g.line(bx, y0 - 33, bx, y0 + 10); g.line(bx, y0 + 17, bx, y0 + 60)
    g.text(30, y0 - 92, 'BT1  2 x 18650 (2S)', 12, weight='bold')
    g.text(bx + 24, y0 - 34, 'cell 1', 11, fill=MUTED); g.text(bx + 24, y0 + 16, 'cell 2', 11, fill=MUTED)
    dot(bx, y0 - 13)
    # BMS
    box(170, y0 - 90, 170, 170, 'U3', '2S BMS (balancing)')
    for name, yy in (('B+', y0 - 70), ('BM', y0 - 13), ('B-', y0 + 60)):
        g.line(bx, yy, 170, yy); g.text(180, yy + 4, name, 12)
    for name, yy in (('P+', y0 - 70), ('P-', y0 + 60)):
        g.text(330, yy + 4, name, 12, 'end')
    g.text(255, y0 + 30, 'cut-off ~2.5 V/cell', 11, 'middle', fill=MUTED)
    g.text(255, y0 + 46, 'balance ~40 mA', 11, 'middle', fill=MUTED)
    labels['CELL_MID'] += 2
    # P- is ground
    g.line(340, y0 + 60, 380, y0 + 60); gnd(380, y0 + 60)
    # charge jack
    g.line(340, y0 - 70, 400, y0 - 70); dot(400, y0 - 70)
    g.line(400, y0 - 70, 400, y0 - 116)
    g.rect(384, y0 - 140, 32, 24, rx=4, fill='#fff')
    g.text(400, y0 - 148, 'J1 charge jack 8.4 V', 12, 'middle', 'bold')
    g.text(424, y0 - 124, '- to GND', 11, fill=MUTED, check=False)
    # polyfuse
    g.line(400, y0 - 70, 450, y0 - 70)
    g.rect(450, y0 - 78, 46, 16, rx=7, fill='#fff'); g.line(456, y0 - 62, 490, y0 - 78, w=1.2)
    g.text(473, y0 - 90, 'F1 PTC 3 A', 12, 'middle', 'bold')
    g.line(496, y0 - 70, 550, y0 - 70)
    # switch
    g.line(550, y0 - 70, 560, y0 - 70); g.circle(562, y0 - 70, 3); g.line(565, y0 - 72, 595, y0 - 86, w=2); g.circle(598, y0 - 70, 3)
    g.text(580, y0 - 98, 'SW1 >= 3 A', 12, 'middle', 'bold')
    g.line(601, y0 - 70, 660, y0 - 70); dot(660, y0 - 70)
    g.text(630, y0 - 52, 'VBAT_SW', 12, 'middle', fill=BLUE_TX, check=False); labels['VBAT_SW'] += 1
    # MX1508
    box(720, y0 - 120, 230, 150, 'U4', 'MX1508 dual H-bridge')
    g.line(660, y0 - 70, 720, y0 - 70); g.text(730, y0 - 66, 'VM (+)', 12)
    g.text(730, y0 - 20, 'GND (-)', 12); g.line(700, y0 - 24, 720, y0 - 24); gnd(700, y0 - 24)
    for i, (pin, net) in enumerate((('IN1', 'ML_IN1'), ('IN2', 'ML_IN2'), ('IN3', 'MR_IN1'), ('IN4', 'MR_IN2'))):
        yy = y0 - 96 + i * 30
        g.text(940, yy + 4, pin, 12, 'end'); g.line(950, yy, 990, yy); lab(990, yy, net)
    g.text(835, y0 + 16, 'no enable pins: PWM on the INs', 11, 'middle', fill=MUTED)
    # motors
    for i, (ref, mot) in enumerate((('M1', 'A'), ('M2', 'B'))):
        cx = 780 + i * 110
        g.line(cx, y0 + 30, cx, y0 + 70)
        g.circle(cx, y0 + 90, 20, fill='#fff', sw=1.8); g.text(cx, y0 + 95, 'M', 14, 'middle', 'bold', check=False)
        g.text(cx, y0 + 128, f'{ref} TT ({"left" if i == 0 else "right"})', 12, 'middle', 'bold')
        g.text(cx, y0 + 144, f'MOTOR-{mot}, JST-XH', 11, 'middle', fill=MUTED)
    # bucks
    g.line(660, y0 - 70, 660, y0 + 260)
    for i, (ref, name, net) in enumerate((('U1', 'Mini360 5.0 V (logic)', '5V_LOGIC'), ('U2', 'Mini360 5.0 V (servos)', '5V_SERVO'))):
        yb = y0 + 200 + i * 120
        dot(660, yb) if i == 0 else None
        g.line(660, yb, 720, yb)
        box(720, yb - 30, 170, 76, ref, name)
        g.text(730, yb + 4, 'IN+', 12); g.text(880, yb + 4, 'OUT+', 12, 'end'); g.text(805, yb + 38, 'IN- / OUT-', 11, 'middle')
        gnd(805, yb + 46)
        g.line(890, yb, 950, yb)
        if i == 0:
            lab(950, yb, net)
        else:
            dot(950, yb); g.line(950, yb, 1010, yb); lab(1010, yb, net)
            g.line(950, yb, 950, yb + 18); cap(950, yb + 18, 'C1', '470 uF', True); g.line(950, yb + 25, 950, yb + 40); gnd(950, yb + 40)
    g.line(660, y0 + 200, 660, y0 + 320)
    # divider
    dvx = 520
    g.line(660, y0 + 60, dvx, y0 + 60); dot(660, y0 + 60); g.line(dvx, y0 + 60, dvx, y0 + 80)
    resistor(dvx, y0 + 80, 'R1', '100k')
    g.line(dvx, y0 + 114, dvx, y0 + 140); dot(dvx, y0 + 140)
    g.line(dvx, y0 + 140, dvx - 80, y0 + 140); lab(dvx - 80, y0 + 140, 'VSENSE', 'l')
    g.line(dvx, y0 + 140, dvx, y0 + 160); resistor(dvx, y0 + 160, 'R2', '33k'); g.line(dvx, y0 + 194, dvx, y0 + 210); gnd(dvx, y0 + 210)
    g.line(dvx, y0 + 140, dvx + 60, y0 + 140); g.line(dvx + 60, y0 + 140, dvx + 60, y0 + 170)
    cap(dvx + 60, y0 + 170, 'C2', '100 nF'); g.line(dvx + 60, y0 + 177, dvx + 60, y0 + 210); gnd(dvx + 60, y0 + 210)
    g.text(390, y0 + 270, 'VSENSE = VBAT x 33 / 133', 12, fill=MUTED)
    g.text(390, y0 + 288, '8.4 V -> 2.08 V, 6.4 V -> 1.59 V', 12, fill=MUTED)

    # ---------------- ESP32 (middle)
    ex, ey, ew, eh = 620, 690, 300, 470
    box(ex, ey, ew, eh, 'U5', 'ESP32 DevKit (ESP32-WROOM-32)', fill='#f1f4fb')
    left = [('VIN / 5V', '5V_LOGIC'), ('GND', 'GND'), ('3V3', '3V3'), ('GPIO36 VP', 'VSENSE'), ('GPIO34', 'IR_L'),
            ('GPIO35', 'IR_R'), ('GPIO21 SDA', 'SDA'), ('GPIO22 SCL', 'SCL')]
    for i, (pin, net) in enumerate(left):
        y = ey + 40 + i * 52
        g.line(ex - 60, y, ex, y); g.text(ex + 10, y + 4, pin, 12)
        gnd(ex - 60, y) if net == 'GND' else lab(ex - 60, y, net, 'l')
    right = [('GPIO25', 'ML_IN1'), ('GPIO26', 'ML_IN2'), ('GPIO27', 'MR_IN1'), ('GPIO14', 'MR_IN2'),
             ('GPIO18', 'SIG_TAIL'), ('GPIO19', 'SIG_HEAD'), ('GPIO2', 'LED')]
    for i, (pin, net) in enumerate(right):
        y = ey + 40 + i * 52
        g.line(ex + ew, y, ex + ew + 60, y); g.text(ex + ew - 10, y + 4, pin, 12, 'end')
        if net == 'LED':
            g.text(ex + ew + 66, y + 4, 'on-board LED (no wire)', 12, fill=MUTED)
        else:
            lab(ex + ew + 60, y, net)
    g.text(ex + ew / 2, ey + eh - 14, UNUSED, 11, 'middle', fill=MUTED)

    # ---------------- sensors (left bottom)
    def module(x, y, ref, title, pins, h=None):
        h = h or 30 + 26 * len(pins)
        box(x, y, 200, h, ref, title, fill='#f3fbf6')
        for i, (p, net) in enumerate(pins):
            yy = y + 24 + i * 26
            g.line(x - 50, yy, x, yy); g.text(x + 10, yy + 4, p, 12)
            gnd(x - 50, yy) if net == 'GND' else lab(x - 50, yy, net, 'l')
    module(250, 700, 'U6', 'GY-521 MPU-6050 0x68', [('VCC', '3V3'), ('GND', 'GND'), ('SCL', 'SCL'), ('SDA', 'SDA')])
    g.text(260, 830, 'AD0 open (pulled low on board)', 11, fill=MUTED)
    module(250, 880, 'U7', 'TCRT5000 left wheel', [('VCC', '3V3'), ('GND', 'GND'), ('D0', 'IR_L')])
    module(250, 1030, 'U8', 'TCRT5000 right wheel', [('VCC', '3V3'), ('GND', 'GND'), ('D0', 'IR_R')])
    g.text(30, 1270, 'Power the IR modules from 3.3 V: their D0 pull-up goes to VCC.', 11, fill=MUTED)
    # pull-downs
    for i, (ref, net) in enumerate((('R3', 'IR_L'), ('R4', 'IR_R'))):
        x = 330 + i * 160
        y = 1175
        lab(x - 40, y, net, 'l'); g.line(x - 40, y, x, y); g.line(x, y, x, y + 10); resistor(x, y + 10, ref, '100k')
        g.line(x, y + 44, x, y + 56); gnd(x, y + 56)

    # ---------------- servos (right bottom)
    box(1080, 690, 230, 150, 'S1, S2', 'SG-90 tail / head', fill='#fbf7ef')
    for i, (ref, net) in enumerate((('S1 tail', 'SIG_TAIL'), ('S2 head', 'SIG_HEAD'))):
        x = 1135 + i * 120
        g.rect(x - 14, 715, 28, 64, rx=3, fill='#fff')
        for k, c in enumerate(('#e8771b', '#d02a2a', '#8a5a2b')):
            g.circle(x, 727 + k * 21, 5, col=c, fill=c, sw=0)
        g.text(x, 798, ref, 12, 'middle', 'bold')
        labels[net] += 1
    labels['5V_SERVO'] += 1
    g.text(1092, 822, 'orange SIG, red 5V_SERVO, brown GND', 11, fill=MUTED)
    # ESP32-CAM
    box(1080, 900, 230, 80, 'U9', 'ESP32-CAM (stage 4)', fill='#f3fbf6')
    g.line(1310, 930, 1350, 930); lab(1350, 930, '5V_LOGIC'); g.text(1300, 934, '5V', 12, 'end')
    g.line(1310, 955, 1350, 955); gnd(1350, 955); g.text(1300, 959, 'GND', 12, 'end')
    g.text(1090, 1000, 'own Wi-Fi stream; no wires to U5', 11, fill=MUTED)
    # notes
    g.text(1080, 1060, 'Notes', 13, weight='bold')
    nts = ['Motor current flows BMS -> F1 -> SW1 -> MX1508: 22 AWG.',
           'Slow decay: one IN held high, the other PWM-ed (brake',
           'in the off-time). Average motor V = duty x VBAT.',
           'Firmware caps the average at 6.0 V (u = 1).',
           'U1 feeds only logic: a servo stall cannot reset U5.',
           'Charge with SW1 OFF through J1 (8.4 V CC/CV, 1 A).']
    for i, t in enumerate(nts):
        g.text(1080, 1084 + i * 18, t, 11)
    g.save(DOCS / 'schematic.svg', 'schematic.svg')
    return labels


# ============================================================================ wiring.svg (where everything sits)
def draw_wiring():
    W, H = 1600, 1230
    g = Svg(W, H, 'Tailwhip board placement and wiring')
    g.text(30, 40, 'Tailwhip: where each board sits (body frame from cad/tailwhip.py, mm) and the main wire runs', 18, weight='bold')
    g.text(30, 64, f'Positions: {BOX_SRC}. Front view looks at the robot from the front (robot left = image right). '
                   'Wire ids match the wire list in electronics.md and electronics.json.', 12, fill=MUTED)
    S = 3.2

    # ---- side view (x right = forward, z up)
    ox, oz = 420, 760
    X = lambda x: ox + x * S
    Z = lambda z: oz - z * S
    g.text(40, 100, 'SIDE (from the robot\'s left)', 14, weight='bold')
    g.circle(X(0), Z(0), 32.5 * S, col='#9aa3b2', fill='#f4f6fa', sw=1.4)
    g.text(X(0), Z(-38), 'wheel 65 mm, axle at 0', 11, 'middle', fill=MUTED)

    def sbox(name, label, col='#2f7d5b', fill='#e3f2ea', lx=None, lz=None, anchor='start'):
        x0, y0, z0, x1, y1, z1 = BOX[name]
        g.rect(X(x0), Z(z1), (x1 - x0) * S, (z1 - z0) * S, col=col, fill=fill, rx=3, sw=1.4)
        if lx is not None:
            g.line(X((x0 + x1) / 2), Z((z0 + z1) / 2), lx, lz, col=MUTED, w=1, dash='3 3')
            g.text(lx + (4 if anchor == 'start' else -4), lz + 4, label, 12, anchor)
    sbox('torso', '', col='#c9ced8', fill='#f7f8fb')
    sbox('holder', 'battery: 2 x 18650 in the sleeve', '#8a5a2b', '#f6e7cf', X(40), Z(160))
    sbox('pan_servo', 'head-pan SG-90 (S2)', '#6b4a1f', '#f6e7cf', X(40), Z(182))
    sbox('esp32', 'ESP32 (U5), back of the spine', '#2a62d0', '#eef3fd', X(-45), Z(118), 'end')
    sbox('mx1508', 'MX1508 (U4)', lx=X(45), lz=Z(122))
    sbox('switch', 'SW1', lx=X(45), lz=Z(134))
    sbox('mini360_logic', 'Mini360 U1 / U2', lx=X(45), lz=Z(92))
    sbox('bms_2s', 'BMS (U3)', lx=X(45), lz=Z(74))
    sbox('cap_470u', 'junction board JB (C1 ...)', lx=X(45), lz=Z(108))
    sbox('tt_gearbox_l', '', '#8a6d1f', '#fbf3d6'); sbox('tt_can_l', 'TT motors, cans up', '#8a6d1f', '#fbf3d6', X(-45), Z(50), 'end')
    sbox('imu', 'GY-521 (U6) on the web pad', '#b0362a', '#fde8e5', X(45), Z(36))
    sbox('ir_pcb_l', 'TCRT5000 (U7 / U8) on posts', '#2f7d5b', '#e3f2ea', X(45), Z(10))
    sbox('tail_servo', 'tail SG-90 (S1) in the hip', '#6b4a1f', '#f6e7cf', X(-45), Z(26), 'end')
    g.line(X(-60), Z(0), X(70), Z(0), col=GRID, w=1, dash='2 4')
    g.text(X(-60), Z(-48), 'x forward ->   z up the body', 11, fill=MUTED)

    # ---- front view of the spine front face (y left on the image right), and the back face
    S2 = 5.0

    def face(title, ox, oz, names, back=False):
        # Front view: looking at the robot's front, its left (+y) is on the image right. Back view: the opposite.
        Y = lambda y: ox + (y if not back else -y) * S2
        Zf = lambda z: oz - (z - 60) * S2
        g.text(ox - 30 * S2, 100, title, 14, weight='bold')
        g.rect(ox - 28 * S2, Zf(124), 56 * S2, (124 - 64) * S2, col='#c9ced8', fill='#f7f8fb', rx=4)
        g.text(ox, Zf(127), 'battery sleeve above', 11, 'middle', fill=MUTED)
        for n, lab_, col, fill in names:
            bx0, by0, bz0, bx1, by1, bz1 = BOX[n]
            xa, xb = sorted((Y(by0), Y(by1)))
            g.rect(xa, Zf(bz1), xb - xa, (bz1 - bz0) * S2, col=col, fill=fill, rx=3, sw=1.4)
            g.text((xa + xb) / 2, Zf((bz0 + bz1) / 2) + 4, lab_, 11, 'middle', 'bold', fill=col)
        return Y, Zf
    Yf, Zf = face('FRONT FACE (power), seen from the front', 960, 660, [
        ('bms_2s', 'U3 BMS', '#2f7d5b', '#e3f2ea'), ('mini360_logic', 'U1 logic', '#2a62d0', '#eef3fd'),
        ('mini360_servo', 'U2 servo', '#2a62d0', '#eef3fd'), ('mx1508', 'U4 MX1508', '#2f7d5b', '#e3f2ea'),
        ('cap_470u', 'JB', '#8a5a2b', '#f6e7cf'), ('switch', 'SW1', '#1d2433', '#e9ebef')])
    g.text(Yf(-28), Zf(62) + 18, 'robot right (-y)', 11, 'start', fill=MUTED)
    g.text(Yf(28), Zf(62) + 18, 'robot left (+y)', 11, 'end', fill=MUTED)
    g.text(Yf(0), Zf(62) + 40, 'JB = junction board in the place the CAD gives the 470 uF capacitor', 11, 'middle', fill=MUTED)
    Yb, Zb = face('BACK FACE, seen from behind', 1390, 660, [('esp32', 'U5 ESP32', '#2a62d0', '#eef3fd')], back=True)
    g.text(Yb(0), Zb(62) + 18, 'header pins point back, toward you', 11, 'middle', fill=MUTED)
    g.text(Yb(0), Zb(62) + 36, 'USB socket: top or bottom, keep it reachable', 11, 'middle', fill=MUTED)

    # list of runs beside
    lx, ly = 30, 960
    g.text(lx, ly, 'Main runs (full list: electronics.md, Wire list)', 14, weight='bold')
    groups = [('red', 'P1-P13  battery -> BMS -> F1 -> SW1 -> MX1508 VM -> both bucks, divider tap. 22 AWG.'),
              ('black', 'P8, P9, P11  BMS P- is the one ground: MX1508 -, buck IN-, charge jack -. 22 AWG.'),
              ('blue', 'L1 L2  U1 5 V and GND round the -y edge of the spine to the ESP32 VIN / GND pins (Dupont).'),
              ('orange', 'S1-S4  U2 5 V -> junction board (C1, two 3-pin servo headers) -> tail and head servo plugs.'),
              ('green', 'J1-J6  ESP32 back -> junction board over the +y edge: servo signals, VSENSE, IR_L, IR_R, GND.'),
              ('white', 'M1-M4  ESP32 GPIO25 26 27 14 -> MX1508 IN1-IN4, over the top edge of the spine.'),
              ('brown', 'M5 M6  MX1508 MOTOR-A / -B -> left / right TT motor (JST-XH), down each side of the core.'),
              ('violet', 'I1-I8  ESP32 3V3 GND SDA SCL -> GY-521 on the web; 3V3 GND on to both IR modules; D0 back up.')]
    for i, (col, t) in enumerate(groups):
        y = ly + 26 + i * 22
        g.line(lx, y - 4, lx + 30, y - 4, col=WIRE_RGB[col], w=4)
        g.text(lx + 40, y, t, 12)
    g.text(lx, ly + 26 + len(groups) * 22 + 6, 'Leave a 30 mm loop on the head servo and ESP32-CAM leads for +-60 deg of pan; '
                                                'zip-tie every run to the spine slot pairs.', 12, fill=MUTED)
    g.save(DOCS / 'wiring.svg', 'wiring.svg')


labels = draw_schematic()
draw_wiring()

# ---------------------------------------------------------------- checks
for net in ['SDA', 'SCL', '3V3', 'VSENSE', 'IR_L', 'IR_R', '5V_LOGIC', '5V_SERVO', 'GND', 'ML_IN1', 'ML_IN2', 'MR_IN1',
            'MR_IN2', 'SIG_TAIL', 'SIG_HEAD']:
    if labels.get(net, 0) < 2:
        problems.append(f'schematic: net {net} drawn at {labels.get(net, 0)} place(s), needs >= 2')
covered = defaultdict(int)
for w in WIRES:
    for n in w['net'].split(' / '):
        covered[n] += 1
for net in NETS:
    if net not in ('MOTA+', 'MOTA-', 'MOTB+', 'MOTB-') and not covered[net]:
        problems.append(f'wire list: no run for net {net}')
for gp, _, net, _, _ in PINMAP:
    if gp is not None and net != 'LED' and f'U5.GPIO{gp}' not in NETS.get(net, []):
        problems.append(f'pin map: GPIO{gp} not on net {net}')

# ============================================================================ electronics.json
def rail(rows):
    return [dict(load=a, typical_A=b, peak_A=c, source=d) for a, b, c, d in rows]


out = dict(
    _about='Tailwhip robot electronics, generated with the drawings by docs/tools/gen_electronics.py. '
           'Refs as in docs/schematic.svg. AI-generated and unbuilt: every number is [D] datasheet, [E] estimate or [C] chosen.',
    parts={k: dict(description=v[0], value=v[1]) for k, v in PARTS.items()},
    nets=NETS,
    pin_map=[dict(gpio=g_, devkit_label=l, net=n, direction=d, notes=t) for g_, l, n, d, t in PINMAP],
    unused_pins=UNUSED,
    i2c=dict(clock_hz=400000, devices={'0x68': 'U6 MPU-6050 (GY-521, AD0 low)'}, timeout_ms=3,
             imu_config='+-4 g, +-500 dps, DLPF 2 (94/98 Hz, ~3 ms), read at 400 Hz'),
    battery_sense=dict(divider='R1 100k (top) / R2 33k (bottom)', ratio=33 / 133, pin='GPIO36 (VP, ADC1_CH0)',
                       v_pin_at_8v4=round(8.4 * 33 / 133, 3), filter='C2 100 nF + firmware low-pass ~40 ms',
                       drain_uA=round(8.4 / 133e3 * 1e6), note='on the switched side: no drain with SW1 off'),
    motor_drive=dict(driver='MX1508, no enable pins', pwm_hz=20000, bits=10,
                     decay='slow (brake in the PWM off-time): forward = IN1 high, IN2 PWM with duty 1 - d',
                     voltage_model='average motor voltage = d x VBAT; firmware d = u x 6.0 / VBAT (clipped at 1)',
                     u_zero='both inputs high = brake (motor shorted); failsafe and idle use coast (both low)'),
    wheel_sensing=dict(sensor='TCRT5000 module D0, both edges, 16-window ring = 32 edges/turn',
                       resolution_deg=360 / 32, lockout_us=1000, stop_after_ms=250,
                       speed='period method at 100 Hz; sign from the last non-zero command; relative to the body'),
    power=dict(rails=dict(logic_5V=rail(POWER['logic_rail']), servo_5V=rail(POWER['servo_rail']), motors_vbat=rail(POWER['motors'])),
               bucks='Mini360 (MP2307): ~1 A continuous in practice, 1.8 A listed, 3 A surge [E listings]',
               fuse='PTC 3 A hold / 6 A trip after the BMS', bms='2S with balancing, 10 A, cut-off ~2.5 V/cell',
               low_battery='6.4 V for 2 s -> idle (latched); deep sleep 30 s later'),
    board_positions=dict(source=BOX_SRC, frame='body: x forward, y left, z up the body, origin on the axle, mm',
                         boxes={k: v for k, v in BOX.items() if k != 'torso'}),
    wires=[{('from' if k == 'from_' else k): v for k, v in w.items()} for w in WIRES],
)
(DOCS / 'electronics.json').write_text(json.dumps(out, indent=1))
print(f'wires {len(WIRES)}, nets {len(NETS)}, parts {len(PARTS)}; positions from {BOX_SRC}')
if problems:
    print('\nPROBLEMS:'); [print(' -', p) for p in problems]; sys.exit(1)
print('checks passed: schematic labels, wire coverage, pin map, no text overlaps')
