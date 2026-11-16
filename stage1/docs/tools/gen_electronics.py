"""Stage 1 electronics: one model -> stage1/docs/electronics.json, schematic.svg and wiring.svg, plus checks.

    python stage1/docs/tools/gen_electronics.py

Adapted from docs/tools/gen_electronics.py (the published Tailwhip). Differences: every pin of every board is in
the model (used pins with their net, unused pins marked not connected), every physical wire has two pin ends,
and the checks are stricter:

- every net is connected end to end by wires and on-board joins (no implied connections);
- every wire joins two pins on the same net, and every pin a wire lands on exists on its board;
- every net appears in the schematic, and every drawn pin carries the net the model gives it;
- the firmware's pin numbers and constants (stage1/firmware/stage1_balance/stage1_balance.ino) match the model;
- no two texts overlap in either drawing, no text leaves the canvas, and no wire in wiring.svg runs through a
  board it does not land on.

Board positions in wiring.svg follow stage1/stage1.py (body frame: x forward, y left, z up, origin on the axle,
mm). They are copied below, not imported, because stage1.py needs the CAD stack; the drawing is not to scale.
"""
import itertools, json, math, pathlib, re, sys
from collections import defaultdict
from xml.sax.saxutils import escape

HERE = pathlib.Path(__file__).resolve().parent
DOCS = HERE.parent                                  # stage1/docs
STAGE1 = DOCS.parent
INO = STAGE1 / 'firmware' / 'stage1_balance' / 'stage1_balance.ino'
INK, MUTED, BG, GRID = '#1d2433', '#5b6475', '#ffffff', '#c9ced8'
BLUE, BLUE_BG, BLUE_TX = '#2a62d0', '#eef3fd', '#1b3f8f'
NC_COL = '#9aa3b2'
FONT = "font-family='Helvetica Neue, Helvetica, Arial, sans-serif'"
problems = []

# ============================================================================ parts
# ref: (description for the parts table, short value). Refs follow the published Tailwhip where the part is the same.
PARTS = {
    'BT1': ('2 x 18650 Li-ion, unprotected, genuine high-drain, matched pair, in a 2S series holder with leads', '2S'),
    'U3':  ('2S Li-ion BMS with balancing, 10 A (e.g. HX-2S-JH20): B-, BM, B+, P-, P+', 'BMS'),
    'J1':  ('DC 5.5 x 2.1 mm female jack, pigtail: charge input (8.4 V CC/CV charger only), tied in front of the robot-left flange', 'charge'),
    'F1':  ('PTC resettable fuse, radial, 3 A hold / 6 A trip, 16 V or more (Littelfuse RUEF300, Bourns MF-R300)', '3 A'),
    'SW1': ('Power switch: Pololu 2810 mini MOSFET slide switch (3 A continuous), or a KCD11 rocker rated 3 A', ''),
    'K2':  ('JST-XH 2-pin: header on JB, housing on the pack leads. The power plug.', 'XH'),
    'JB':  ('Junction board: perfboard 8 x 5 holes, cut to about 21 x 13 mm, on rails on the mast front, robot-right (-y) side', ''),
    'R1':  ('100 k 1 % 1/4 W, battery divider top (on JB)', '100k'),
    'R2':  ('33 k 1 % 1/4 W, battery divider bottom (on JB)', '33k'),
    'C2':  ('100 nF ceramic, 50 V, across R2 (on JB)', '100n'),
    'U4':  ('MX1508 dual H-bridge module, channel A used, channel B not connected', ''),
    'C1':  ('470 uF 16 V low-ESR electrolytic, legs soldered to the MX1508 + and - pads', '470u'),
    'R3':  ('100 k pull-down, MX1508 IN1 to - (soldered on the module)', '100k'),
    'R4':  ('100 k pull-down, MX1508 IN2 to - (soldered on the module)', '100k'),
    'U1':  ('Mini360 buck (MP2307), set to 5.0 V before it is connected: ESP32 supply', '5.0 V'),
    'U5':  ('ESP32 DevKitC, 38-pin, ESP32-WROOM-32 (classic), on the back of the mast', ''),
    'U6':  ('GY-521 MPU-6050 breakout, 2 mm off the front of the core on two M2.5 screws, address 0x68', ''),
    'K1':  ('JST-XH 2-pin wire-to-wire pair: the motor plug, tied to the back of the core', 'XH'),
    'C3':  ('100 nF ceramic, 50 V, across the motor tabs, in the pocket above the can (brush noise)', '100n'),
    'M1':  ('Dual-shaft TT gearmotor 1:48, 3-6 V', ''),
    # ---- milestone B: wheel sensor (leave all of these out for milestone A)
    'U7':  ('Milestone B. TCRT5000 module (LM393 comparator, trimmer, LEDs; D0 digital out) on rails on the back of the core, its sensor removed', ''),
    'U8':  ('Milestone B. The module\'s TCRT5000 sensor, desoldered and moved to the clip under the core, on a 60 mm 4-wire pigtail', ''),
    'C5':  ('Milestone B. 10 nF ceramic across the module\'s detector pads (C to E): filters noise picked up on the pigtail', '10n'),
    'K3':  ('Milestone B. JST-XH 3-pin wire-to-wire pair: the wheel-sensor plug, just above the module', 'XH'),
    'R5':  ('Milestone B. 100 k pull-down on IR_D0, soldered across K3 pins 3 and 2 on the robot side, in heat-shrink', '100k'),
    'C4':  ('Milestone B. 10 nF ceramic on IR_D0, beside R5 (with the module pull-up: ~0.1 ms low-pass)', '10n'),
}
MILESTONE_B = {'U7', 'U8', 'C5', 'K3', 'R5', 'C4'}

# ============================================================================ pins, as printed on each board
# PINS[ref] = [(key, printed label, net or None, note)]. key is unique on the board; the label is what is printed.
ROW_A = ['3V3', 'EN', 'SVP', 'SVN', 'IO34', 'IO35', 'IO32', 'IO33', 'IO25', 'IO26', 'IO27', 'IO14', 'IO12', 'GND',
         'IO13', 'SD2', 'SD3', 'CMD', '5V']                      # Espressif DevKitC header J1, antenna end first
ROW_B = ['GND', 'IO23', 'IO22', 'TXD0', 'RXD0', 'IO21', 'GND', 'IO19', 'IO18', 'IO5', 'IO17', 'IO16', 'IO4', 'IO0',
         'IO2', 'IO15', 'SD1', 'SD0', 'CLK']                     # Espressif DevKitC header J3, antenna end first
ESP_USE = {('A', 1): ('3V3', 'U6 VCC (GY-521); (B) also the wheel sensor through the Y-lead W28'), ('A', 3): ('VSENSE', 'battery sense, ADC1_CH0, input only'),
           ('A', 9): ('M_IN1', 'MX1508 IN1'), ('A', 10): ('M_IN2', 'MX1508 IN2'),
           ('A', 5): ('IR_D0', 'milestone B: TCRT5000 D0, input only, no internal pulls: R5 pull-down'),
           ('A', 14): ('GND', 'Mini360 OUT-; (B) also the wheel sensor through the Y-lead W29'), ('A', 19): ('5V', 'Mini360 OUT+ (5.0 V in)'),
           ('B', 1): ('GND', 'U6 GND (GY-521)'), ('B', 3): ('SCL', 'U6 SCL'), ('B', 6): ('SDA', 'U6 SDA')}
ESP_NOTE = {'EN': 'reset button', 'IO0': 'BOOT button, strapping', 'IO2': 'on-board LED (firmware), strapping',
            'IO12': 'strapping', 'IO5': 'strapping', 'IO15': 'strapping', 'TXD0': 'USB serial', 'RXD0': 'USB serial',
            'SD2': 'flash', 'SD3': 'flash', 'CMD': 'flash', 'CLK': 'flash', 'SD0': 'flash', 'SD1': 'flash',
            'SVN': 'input only', 'IO34': 'input only', 'IO35': 'input only'}
ALT = {'IO34': 'D34', 'IO25': 'D25', 'IO26': 'D26', 'IO21': 'D21', 'IO22': 'D22', 'SVP': 'VP', '5V': 'VIN', 'IO2': 'D2'}


def esp_key(row, k):
    lab = (ROW_A if row == 'A' else ROW_B)[k - 1]
    return f'{lab}.{row}{k}' if lab == 'GND' else lab


def gpio_of(label):
    return 36 if label == 'SVP' else 39 if label == 'SVN' else int(label[2:]) if label.startswith('IO') else None


PINS = {
    'BT1': [('+', '+ (red lead)', 'CELL+', ''), ('MID', 'cell middle (series strap)', 'CELL_MID', ''),
            ('-', '- (black lead)', 'CELL-', '')],
    'U3': [('B+', 'B+', 'CELL+', ''), ('BM', 'BM', 'CELL_MID', ''), ('B-', 'B-', 'CELL-', 'only the cell connects here'),
           ('P+', 'P+', 'PACK+', 'same copper as B+ on most boards'), ('P-', 'P-', 'GND', 'the robot ground')],
    'J1': [('+', 'centre (+, red)', 'PACK+', ''), ('-', 'sleeve (-, black)', 'GND', '')],
    'F1': [('1', 'lead 1', 'PACK+', ''), ('2', 'lead 2', 'VBAT_F', '')],
    'SW1': [('VIN', 'VIN', 'VBAT_F', ''), ('GND', 'GND', 'GND', 'needed by the MOSFET gate'),
            ('VOUT', 'VOUT', 'VBAT_SW', '')],
    'K2': [('1', '1 (+)', 'VBAT_SW', ''), ('2', '2 (-)', 'GND', '')],
    'JB': [('K2.1', 'K2 pin 1', 'VBAT_SW', 'header'), ('K2.2', 'K2 pin 2', 'GND', 'header'), ('VB', 'VB', 'VBAT_SW', 'pad'),
           ('GND', 'GND', 'GND', 'pad'), ('VS', 'VS', 'VSENSE', '1-pin male header')],
    'R1': [('1', '1', 'VBAT_SW', ''), ('2', '2', 'VSENSE', '')],
    'R2': [('1', '1', 'VSENSE', ''), ('2', '2', 'GND', '')],
    'C2': [('1', '1', 'VSENSE', ''), ('2', '2', 'GND', '')],
    'U4': [('+', '+', 'VBAT_SW', 'motor supply, also the chip supply'), ('-', '-', 'GND', ''),
           ('IN1', 'IN1', 'M_IN1', ''), ('IN2', 'IN2', 'M_IN2', ''), ('IN3', 'IN3', None, 'channel B unused'),
           ('IN4', 'IN4', None, 'channel B unused'), ('MA1', 'MOTOR-A', 'MOT_1', ''), ('MA2', 'MOTOR-A', 'MOT_2', ''),
           ('MB1', 'MOTOR-B', None, 'unused'), ('MB2', 'MOTOR-B', None, 'unused')],
    'C1': [('+', '+', 'VBAT_SW', ''), ('-', '- (stripe)', 'GND', '')],
    'R3': [('1', '1', 'M_IN1', ''), ('2', '2', 'GND', '')],
    'R4': [('1', '1', 'M_IN2', ''), ('2', '2', 'GND', '')],
    'U1': [('IN+', 'IN+', 'VBAT_SW', ''), ('IN-', 'IN-', 'GND', ''), ('OUT+', 'OUT+', '5V', ''), ('OUT-', 'OUT-', 'GND', '')],
    'U5': [(esp_key(r, k), (ROW_A if r == 'A' else ROW_B)[k - 1], ESP_USE.get((r, k), (None, ''))[0],
            ESP_USE.get((r, k), (None, ESP_NOTE.get((ROW_A if r == 'A' else ROW_B)[k - 1], '')))[1])
           for r in 'AB' for k in range(1, 20)],
    'U6': [('VCC', 'VCC', '3V3', '3.3 V from the ESP32'), ('GND', 'GND', 'GND', ''), ('SCL', 'SCL', 'SCL', ''),
           ('SDA', 'SDA', 'SDA', ''), ('XDA', 'XDA', None, 'auxiliary I2C master: not connected'),
           ('XCL', 'XCL', None, 'auxiliary I2C master: not connected'),
           ('AD0', 'AD0', None, 'not connected: pulled low on the board, address 0x68 (0x69 if tied to 3V3)'),
           ('INT', 'INT', None, 'interrupt out: not connected, the firmware polls')],
    'K1': [('1', '1', 'MOT_1', ''), ('2', '2', 'MOT_2', '')],
    'U7': [('VCC', 'VCC', '3V3', '3.3 V from the ESP32 (Y-lead W28)'), ('GND', 'GND', 'GND', ''),
           ('D0', 'D0', 'IR_D0', 'LM393 output with the module pull-up; printed DO on some boards'),
           ('A0', 'A0', None, 'analog phototransistor output: not connected (printed AO on some boards)'),
           ('SA', 'sensor pad A', 'IR_A', 'IR LED anode pad: to VCC through the module\'s LED resistor'),
           ('SK', 'sensor pad K', 'GND', 'IR LED cathode pad: module GND'),
           ('SC', 'sensor pad C', 'IR_C', 'detector collector pad: to VCC through ~10 k and to the LM393 input'),
           ('SE', 'sensor pad E', 'GND', 'detector emitter pad: module GND')],
    'U8': [('A', 'A (anode)', 'IR_A', 'IR LED'), ('K', 'K (cathode)', 'GND', 'IR LED'),
           ('C', 'C (collector)', 'IR_C', 'phototransistor'), ('E', 'E (emitter)', 'GND', 'phototransistor')],
    'C5': [('1', '1', 'IR_C', ''), ('2', '2', 'GND', '')],
    'K3': [('1', '1', '3V3', ''), ('2', '2', 'GND', ''), ('3', '3', 'IR_D0', '')],
    'R5': [('1', '1', 'IR_D0', ''), ('2', '2', 'GND', '')],
    'C4': [('1', '1', 'IR_D0', ''), ('2', '2', 'GND', '')],
    'C3': [('1', '1', 'MOT_1', ''), ('2', '2', 'MOT_2', '')],
    'M1': [('1', 'tab 1', 'MOT_1', ''), ('2', 'tab 2', 'MOT_2', '')],
}
PIN = {(r, k): (lab, net, note) for r, pins in PINS.items() for k, lab, net, note in pins}
NETS = defaultdict(list)
for (r, k), (_, net, _) in PIN.items():
    if net:
        NETS[net].append(f'{r}.{k}')
NETS = dict(NETS)

# ============================================================================ firmware facts (checked against the .ino)
FW = dict(PIN_M1=25, PIN_M2=26, PIN_SDA=21, PIN_SCL=22, PIN_BATT=36, PIN_LED=2, MPU=0x68, MOTOR_PWM_HZ=20000,
          MOTOR_BITS=10, V_MOTOR_MAX=6.0, LOW_BATT_MV=6400, R_TOP=100.0, R_BOT=33.0)
FW_NET = dict(PIN_M1='M_IN1', PIN_M2='M_IN2', PIN_SDA='SDA', PIN_SCL='SCL', PIN_BATT='VSENSE', PIN_IR='IR_D0')
FW_OPTIONAL = dict(PIN_IR=34)          # milestone B: the firmware may not define it yet

# ============================================================================ physical wires and on-board joins
WIRES = []
DUP_F = 'Dupont female on the pin'


def wire(wid, a, b, colour, awg, length, end_a, end_b, route, kind='wire'):
    WIRES.append(dict(id=wid, a=a, b=b, colour=colour, awg=awg, length_mm=length, end_a=end_a, end_b=end_b, route=route,
                      kind=kind))


S, XH = 'solder', 'JST-XH crimp'
JMP10, JMP20 = 'F-F jumper 10 cm', 'F-F jumper 20 cm'
# battery and protection, mast front
wire('W1', 'BT1.+', 'U3.B+', 'red', 22, 70, S, S, 'holder red lead, shortened: out of the tray\'s robot-right notch, down outside the front-right flange (tie at z 124), into the BMS at z 119')
wire('W2', 'BT1.-', 'U3.B-', 'black', 22, 70, S, S, 'holder black lead, shortened, with W1')
wire('W3', 'BT1.MID', 'U3.BM', 'orange', 26, 150, S, S, 'soldered to the strap joining the cells (robot-left end of the holder), along the holder to the notch, then with W1')
wire('W4', 'U3.P+', 'F1.1', 'red', 22, 15, S, S, 'F1 inline, in heat-shrink')
wire('W5', 'F1.2', 'SW1.VIN', 'red', 22, 20, S, S, 'straight down from the BMS to the switch below it')
wire('W6', 'U3.P-', 'SW1.GND', 'black', 26, 30, S, S, 'switch gate ground (not needed with a mechanical rocker)')
wire('W7', 'SW1.VOUT', 'K2.1', 'red', 22, 40, S, XH, 'across the mast front to the K2 plug on JB')
wire('W8', 'U3.P-', 'K2.2', 'black', 22, 50, S, XH, 'under the BMS to the K2 plug on JB')
wire('W9', 'J1.+', 'U3.P+', 'red', 22, 60, S, S, 'jack pigtail, shortened; the jack hangs in front of the robot-left front flange, socket down')
wire('W10', 'J1.-', 'U3.P-', 'black', 22, 60, S, S, 'with W9')
# junction board
wire('B1', 'JB.K2.1', 'JB.VB', '-', None, 0, 'board', 'board', 'perfboard track (solder bridge or tinned wire)', 'board')
wire('B2', 'JB.VB', 'R1.1', '-', None, 0, 'board', 'board', 'perfboard track', 'board')
wire('B3', 'R1.2', 'JB.VS', '-', None, 0, 'board', 'board', 'perfboard track', 'board')
wire('B4', 'R2.1', 'JB.VS', '-', None, 0, 'board', 'board', 'perfboard track', 'board')
wire('B5', 'C2.1', 'JB.VS', '-', None, 0, 'board', 'board', 'perfboard track', 'board')
wire('B6', 'JB.K2.2', 'JB.GND', '-', None, 0, 'board', 'board', 'perfboard track', 'board')
wire('B7', 'R2.2', 'JB.GND', '-', None, 0, 'board', 'board', 'perfboard track', 'board')
wire('B8', 'C2.2', 'JB.GND', '-', None, 0, 'board', 'board', 'perfboard track', 'board')
wire('W11', 'JB.VB', 'U4.+', 'red', 22, 60, S, S, 'down from JB, under the bottom edge of the mast plate at y -15, up to the MX1508 + pad on the back')
wire('W12', 'JB.GND', 'U4.-', 'black', 22, 60, S, S, 'with W11. The main ground: all motor current returns on it')
wire('W13', 'U4.+', 'U1.IN+', 'red', 26, 40, S, S, 'across the MX1508 to the Mini360 beside it')
wire('W14', 'U4.-', 'U1.IN-', 'black', 26, 40, S, S, 'with W13')
wire('M1x', 'K2.1', 'JB.K2.1', '-', None, 0, 'mate', 'mate', 'K2 housing plugged onto the K2 header on JB', 'mate')
wire('M2x', 'K2.2', 'JB.K2.2', '-', None, 0, 'mate', 'mate', '', 'mate')
wire('B9', 'U1.IN-', 'U1.OUT-', '-', None, 0, 'board', 'board', 'Mini360 IN- and OUT- are the same copper', 'board')
wire('B10', 'U5.GND.A14', 'U5.GND.B1', '-', None, 0, 'board', 'board', 'the DevKit GND pins are joined on the board', 'board')
wire('L1', 'C1.+', 'U4.+', '-', None, 0, 'leg', S, 'C1 + leg on the MX1508 + pad (stripe side to -)', 'leg')
wire('L2', 'C1.-', 'U4.-', '-', None, 0, 'leg', S, 'C1 - leg on the MX1508 - pad', 'leg')
wire('L3', 'R3.1', 'U4.IN1', '-', None, 0, 'leg', S, 'R3 on the underside: IN1 header pin to the - pad', 'leg')
wire('L4', 'R3.2', 'U4.-', '-', None, 0, 'leg', S, '', 'leg')
wire('L5', 'R4.1', 'U4.IN2', '-', None, 0, 'leg', S, 'R4 on the underside: IN2 header pin to the - pad', 'leg')
wire('L6', 'R4.2', 'U4.-', '-', None, 0, 'leg', S, '', 'leg')
# ESP32 supply and signals, mast back
wire('W15', 'U1.OUT+', 'U5.5V', 'red', 26, 60, S, DUP_F, 'half of a F-F jumper: bare end soldered to the Mini360 pad')
wire('W16', 'U1.OUT-', 'U5.GND.A14', 'black', 26, 60, S, DUP_F, 'half of a F-F jumper, with W15')
wire('W17', 'JB.VS', 'U5.SVP', 'violet', 28, 100, 'Dupont female', DUP_F, JMP10 + ', round the robot-right edge of the mast at z 98')
wire('W18', 'U5.IO25', 'U4.IN1', 'white', 28, 100, DUP_F, 'Dupont female', JMP10)
wire('W19', 'U5.IO26', 'U4.IN2', 'grey', 28, 100, DUP_F, 'Dupont female', JMP10)
# motor
wire('W20', 'U4.MA1', 'K1.1', 'red', 22, 40, S, XH, 'MX1508 MOTOR-A pad (beside the gusset) down to the K1 plug on the back of the core')
wire('W21', 'U4.MA2', 'K1.2', 'black', 22, 40, S, XH, 'with W20')
wire('W22', 'K1.1', 'M1.1', 'red', 22, 60, XH, S, 'motor tab, out of the slot in the back of the core (z 59-64), into K1')
wire('W23', 'K1.2', 'M1.2', 'black', 22, 60, XH, S, 'with W22')
wire('L7', 'C3.1', 'M1.1', '-', None, 0, 'leg', S, 'C3 across the motor tabs, legs short', 'leg')
wire('L8', 'C3.2', 'M1.2', '-', None, 0, 'leg', S, '', 'leg')
# ---- milestone B: wheel sensor. A 3-wire lead from the ESP32 to K3 by the left wheel, and a short pigtail K3 to the module.
JY = 'F-F jumper 20 cm, made into a Y-lead'
wire('W28', 'U5.3V3', 'K3.1', 'red', 28, 130, 'Dupont female (Y)', XH,
     'Y-lead: one Dupont female on the 3V3 pin, splitting 3 cm along into a male pin for W24 and this run. Behind the MX1508, down the back of the core (tied with K1) to K3', kind='wire')
wire('W29', 'U5.GND.A14', 'K3.2', 'black', 28, 130, 'Dupont female (Y)', XH, 'Y-lead on the row-A GND pin, shared with W16 the same way; with W28', kind='wire')
wire('W30', 'U5.IO34', 'K3.3', 'green', 28, 130, DUP_F, XH, 'with W28; a 20 cm F-F jumper with its far end cut and crimped into K3', kind='wire')
wire('W31', 'K3.1', 'U7.VCC', 'red', 26, 40, XH, 'Dupont female', 'module lead: K3 plug to the module header', kind='wire')
wire('W32', 'K3.2', 'U7.GND', 'black', 26, 40, XH, 'Dupont female', 'module lead', kind='wire')
wire('W33', 'K3.3', 'U7.D0', 'green', 26, 40, XH, 'Dupont female', 'module lead', kind='wire')
wire('W34', 'U7.SA', 'U8.A', 'red', 28, 60, S, S, 'sensor pigtail, LED pair (twisted with W35): from the back of the clip under the core, up the slot in the back crash-arm root', kind='wire')
wire('W35', 'U7.SK', 'U8.K', 'black', 28, 60, S, S, 'sensor pigtail, LED pair', kind='wire')
wire('W36', 'U7.SC', 'U8.C', 'white', 28, 60, S, S, 'sensor pigtail, detector pair (twisted with W37)', kind='wire')
wire('W37', 'U7.SE', 'U8.E', 'blue', 28, 60, S, S, 'sensor pigtail, detector pair', kind='wire')
wire('B11', 'U7.SK', 'U7.GND', '-', None, 0, 'board', 'board', 'module ground track', 'board')
wire('B12', 'U7.SE', 'U7.GND', '-', None, 0, 'board', 'board', 'module ground track', 'board')
wire('L13', 'C5.1', 'U7.SC', '-', None, 0, 'leg', S, 'C5 on the module, across the C and E sensor pads', 'leg')
wire('L14', 'C5.2', 'U7.SE', '-', None, 0, 'leg', S, '', 'leg')
wire('L9', 'R5.1', 'K3.3', '-', None, 0, 'leg', S, 'R5 across K3 pins 3 and 2, robot side, in heat-shrink', 'leg')
wire('L10', 'R5.2', 'K3.2', '-', None, 0, 'leg', S, '', 'leg')
wire('L11', 'C4.1', 'K3.3', '-', None, 0, 'leg', S, 'C4 beside R5', 'leg')
wire('L12', 'C4.2', 'K3.2', '-', None, 0, 'leg', S, '', 'leg')
MILESTONE_B_WIRES = {'W28', 'W29', 'W30', 'W31', 'W32', 'W33', 'W34', 'W35', 'W36', 'W37', 'L9', 'L10', 'L11', 'L12', 'B11', 'B12', 'L13', 'L14'}
# IMU
wire('W24', 'U5.3V3', 'U6.VCC', 'red', 28, 200, DUP_F, 'Dupont female', JMP20 + ': up the front of the core (ties at z 48 and 63), under the plate to the robot-right side, up outside the flange, round its edge at z 98 to the lower header row')
wire('W25', 'U5.GND.B1', 'U6.GND', 'black', 28, 200, DUP_F, 'Dupont female', JMP20 + ', with W24, then up beside the antenna end of the ESP32 to the upper header row')
wire('W26', 'U5.IO22', 'U6.SCL', 'yellow', 28, 200, DUP_F, 'Dupont female', JMP20 + ', as W25; twist loosely with W27')
wire('W27', 'U5.IO21', 'U6.SDA', 'blue', 28, 200, DUP_F, 'Dupont female', JMP20 + ', with W24, then up beside the antenna end of the ESP32 to the upper header row')


def pin_of(end):
    ref, key = end.split('.', 1)
    return ref, key


def net_of(end):
    p = PIN.get(pin_of(end))
    return p[1] if p else None


# ---------------------------------------------------------------- power budget ([D] datasheet, [E] estimate, [S] stage 1 simulation)
POWER = [
    # load, rail, typical A, peak A, source
    ('ESP32 DevKit, CPU at 240 MHz, radio off', '5 V', 0.06, 0.12, '[D] ESP32 datasheet: 30-68 mA modem-sleep at 240 MHz; [E] plus the DevKit regulator, USB bridge and LED'),
    ('GY-521 MPU-6050', '3V3', 0.005, 0.005, '[D] 3.8-3.9 mA; [E] plus the board LED and regulator'),
    ('Battery divider R1 + R2', 'VBAT_SW', 0.00006, 0.00006, '[C] 8.4 V / 133 k'),
    ('Mini360 own losses (85 % efficient)', 'VBAT_SW', 0.01, 0.02, '[E]'),
    ('TT motor, balancing, average |I|', 'motor', 0.44, 1.65, '[S] sim.py nominal runs: mean 0.41-0.44 A, peak 1.55-1.65 A'),
    ('TT motor, stall at 6.0 V average', 'motor', 1.5, 1.5, '[D] Adafruit 3777: 1.5 A stall, 0.16 A no load at 6 V'),
    ('TT motor, full-speed reversal (worst case, not seen in sim)', 'motor', 0, 3.0, '[E] (6 V + 6 V back-EMF) / 4 ohm, tens of ms'),
    ('Milestone B: TCRT5000 module (IR LED, LM393, two LEDs)', '3V3', 0.02, 0.025, '[E] IR LED 10-20 mA + LM393 ~0.4 mA [D] + power and D0 LEDs ~1-2 mA each'),
]

# ============================================================================ SVG writer with text-overlap and wire bookkeeping
class Svg:
    def __init__(self, w, h, title):
        self.w, self.h, self.parts, self.texts, self.title = w, h, [], [], title
        self.segs = []            # (wire id, x1, y1, x2, y2) of drawn wires, for hops and the board check
        self.rects = []           # (name, x0, y0, x1, y1) of boards, for the through-board check
        self.hops = 0
        self.parts.append(f"<rect x='1' y='1' width='{w - 2}' height='{h - 2}' rx='14' fill='{BG}' stroke='{GRID}'/>")

    def add(self, s): self.parts.append(s)

    def text(self, x, y, s, size=12, anchor='start', weight='normal', fill=INK, check=True, rot=0):
        w = 0.56 * size * len(s); h = size
        if rot == 0:
            x0 = x if anchor == 'start' else x - w / 2 if anchor == 'middle' else x - w
            box = (x0, y - h * 0.8, x0 + w, y + h * 0.2)
            tf = ''
        else:                                    # rot -90: text reads bottom to top; anchor start = text goes up from y
            y1 = y if anchor == 'start' else y + w / 2 if anchor == 'middle' else y + w
            box = (x - h * 0.8, y1 - w, x + h * 0.2, y1)
            tf = f" transform='rotate(-90 {x:.1f} {y:.1f})'"
        if check:
            self.texts.append(box + (s,))
        self.parts.append(f"<text x='{x:.1f}' y='{y:.1f}' font-size='{size}' text-anchor='{anchor}' "
                          f"font-weight='{weight}' fill='{fill}'{tf}>{escape(s)}</text>")

    def line(self, x1, y1, x2, y2, col=INK, w=1.6, dash=None):
        d = f" stroke-dasharray='{dash}'" if dash else ''
        self.parts.append(f"<line x1='{x1:.1f}' y1='{y1:.1f}' x2='{x2:.1f}' y2='{y2:.1f}' stroke='{col}' stroke-width='{w}' stroke-linecap='round'{d}/>")

    def poly(self, pts, col=INK, w=1.6, dash=None, fill='none'):
        d = f" stroke-dasharray='{dash}'" if dash else ''
        p = ' '.join(f'{x:.1f},{y:.1f}' for x, y in pts)
        self.parts.append(f"<polyline points='{p}' fill='{fill}' stroke='{col}' stroke-width='{w}' stroke-linejoin='round' stroke-linecap='round'{d}/>")

    def rect(self, x, y, w, h, col=INK, fill='none', rx=3, sw=1.6, dash=None, board=None):
        d = f" stroke-dasharray='{dash}'" if dash else ''
        self.parts.append(f"<rect x='{x:.1f}' y='{y:.1f}' width='{w:.1f}' height='{h:.1f}' rx='{rx}' fill='{fill}' stroke='{col}' stroke-width='{sw}'{d}/>")
        if board:
            self.rects.append((board, x, y, x + w, y + h))

    def circle(self, x, y, r, col=INK, fill='none', sw=1.4):
        self.parts.append(f"<circle cx='{x:.1f}' cy='{y:.1f}' r='{r}' fill='{fill}' stroke='{col}' stroke-width='{sw}'/>")

    def wire(self, wid, pts, col, w=3.0, net=None, dash=None):
        """Orthogonal wire. Where it crosses an earlier wire of another net it hops over it (r = 5 px)."""
        r = 5.0
        d = f'M{pts[0][0]:.1f},{pts[0][1]:.1f}'
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            cuts = []
            for (owid, onet, a, b, c, e) in self.segs:
                if onet == net:
                    continue
                if y1 == y2 and a == c:                  # this horizontal, the other vertical
                    if min(x1, x2) + r < a < max(x1, x2) - r and min(b, e) + 1 < y1 < max(b, e) - 1:
                        cuts.append(a)
                elif x1 == x2 and b == e:                # this vertical, the other horizontal
                    if min(y1, y2) + r < b < max(y1, y2) - r and min(a, c) + 1 < x1 < max(a, c) - 1:
                        cuts.append(b)
            if y1 == y2:
                sgn = 1 if x2 > x1 else -1
                for cx in sorted(set(cuts), key=lambda v: sgn * v):
                    d += f' L{cx - sgn * r:.1f},{y1:.1f} A{r},{r} 0 0 {1 if sgn > 0 else 0} {cx + sgn * r:.1f},{y1:.1f}'
                    self.hops += 1
            elif x1 == x2:
                sgn = 1 if y2 > y1 else -1
                for cy in sorted(set(cuts), key=lambda v: sgn * v):
                    d += f' L{x1:.1f},{cy - sgn * r:.1f} A{r},{r} 0 0 {0 if sgn > 0 else 1} {x1:.1f},{cy + sgn * r:.1f}'
                    self.hops += 1
            else:
                problems.append(f'{self.title}: wire {wid} has a diagonal segment')
            d += f' L{x2:.1f},{y2:.1f}'
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            self.segs.append((wid, net, x1, y1, x2, y2))
        self.parts.append(f"<path d='{d}' fill='none' stroke='#ffffff' stroke-width='{w + 3.5}' stroke-linejoin='round' stroke-linecap='round'/>")
        da = f" stroke-dasharray='{dash}'" if dash else ''
        self.parts.append(f"<path d='{d}' fill='none' stroke='{col}' stroke-width='{w}' stroke-linejoin='round' stroke-linecap='round'{da}/>")

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


WIRE_RGB = dict(red='#d02a2a', black='#1d2433', orange='#e8771b', yellow='#d4a800', blue='#2a62d0', white='#b4bcc8',
                violet='#8a3fc4', green='#2f9a46', brown='#8a5a2b', grey='#6f7787')
drawn = defaultdict(set)          # schematic: (ref, key) -> nets drawn at that pin
sch_nets = defaultdict(int)       # schematic: net -> places drawn

# ============================================================================ schematic.svg
def draw_schematic():
    W, H = 1700, 1330
    g = Svg(W, H, 'Tailwhip stage 1 schematic')
    g.text(30, 40, 'Tailwhip stage 1: full schematic. Every pin of every board, by the name printed on the board.', 18, weight='bold')
    g.text(30, 64, 'Lines are connections. Every ground symbol is the same net, GND = BMS P-. Grey pins and "nc" are not connected. '
                   'Wire ids (W..) match the wire list and wiring.svg.', 12, fill=MUTED)

    def mark(ref, key, net=None):
        p = PIN[(ref, key)]
        drawn[(ref, key)].add(net if net is not None else p[1])
        if net is not None:
            sch_nets[net] += 1

    def gnd(x, y, ref=None, key=None):
        sch_nets['GND'] += 1
        if ref:
            drawn[(ref, key)].add('GND')
        g.line(x, y, x, y + 10); g.line(x - 10, y + 10, x + 10, y + 10, w=2); g.line(x - 6, y + 14, x + 6, y + 14); g.line(x - 2, y + 18, x + 2, y + 18)

    def gnd_up(x, y):
        sch_nets['GND'] += 1
        g.line(x, y, x, y - 10); g.line(x - 10, y - 10, x + 10, y - 10, w=2); g.line(x - 6, y - 14, x + 6, y - 14); g.line(x - 2, y - 18, x + 2, y - 18)

    def dot(x, y): g.circle(x, y, 3.4, fill=INK, sw=0)

    def nc(x, y, side):
        dx = 8 if side == 'r' else -8
        g.line(x, y, x + dx, y, col=NC_COL, w=1.2)
        g.line(x + dx - 3, y - 3, x + dx + 3, y + 3, col=NC_COL, w=1.2); g.line(x + dx - 3, y + 3, x + dx + 3, y - 3, col=NC_COL, w=1.2)

    def box(x, y, w, h, title, fill='#f4f6fa', title_y=None, dash=None):
        g.rect(x, y, w, h, fill=fill, rx=6, sw=1.8, dash=dash)
        if title:
            g.text(x + w / 2, title_y if title_y else y - 8, title, 12, 'middle', 'bold')

    def resistor_v(x, y, ref, val, side='r'):
        g.rect(x - 7, y, 14, 34, rx=2, fill='#fff')
        if side == 'r':
            g.text(x + 12, y + 14, ref, 12, weight='bold'); g.text(x + 12, y + 29, val, 12)
        else:
            g.text(x - 12, y + 14, ref, 12, 'end', 'bold'); g.text(x - 12, y + 29, val, 12, 'end')

    def cap_v(x, y, ref, val, polar=False, side='r'):           # plates horizontal, current flows vertically, top plate at y
        g.line(x - 12, y, x + 12, y, w=2.4); g.line(x - 12, y + 7, x + 12, y + 7, w=2.4)
        if polar: g.text(x - 18, y - 2, '+', 12, 'middle', 'bold', check=False)
        if side == 'r':
            g.text(x + 18, y + 2, ref, 12, weight='bold'); g.text(x + 18, y + 16, val, 12)
        else:
            g.text(x - 18, y + 2, ref, 12, 'end', 'bold'); g.text(x - 18, y + 16, val, 12, 'end')

    def lab(x, y, net, side='r'):
        sch_nets[net] += 1
        w = 0.56 * 12 * len(net) + 14
        if side == 'r':
            g.poly([(x, y), (x + 8, y - 9), (x + 8 + w, y - 9), (x + 8 + w, y + 9), (x + 8, y + 9), (x, y)], col=BLUE, w=1.2, fill=BLUE_BG)
            g.text(x + 14, y + 4, net, 12, fill=BLUE_TX)
        else:
            g.poly([(x, y), (x - 8, y - 9), (x - 8 - w, y - 9), (x - 8 - w, y + 9), (x - 8, y + 9), (x, y)], col=BLUE, w=1.2, fill=BLUE_BG)
            g.text(x - 14, y + 4, net, 12, 'end', fill=BLUE_TX)

    def wl(x, y, s, anchor='middle'):           # wire id tag
        g.text(x, y, s, 10, anchor, fill=MUTED)

    YP, YG = 170, 330                           # the + and - lines of the power chain
    # ---------------- BT1
    bx = 80
    g.line(bx, YP, bx, 205); g.line(bx - 16, 205, bx + 16, 205, w=2.6); g.line(bx - 9, 212, bx + 9, 212, w=2.6)
    g.line(bx, 212, bx, 280); g.line(bx - 16, 280, bx + 16, 280, w=2.6); g.line(bx - 9, 287, bx + 9, 287, w=2.6); g.line(bx, 287, bx, YG)
    dot(bx, 250)
    g.text(30, 118, 'BT1  2 x 18650 (2S)', 12, weight='bold')
    g.text(30, 134, 'unprotected cells', 11, fill=MUTED)
    g.text(bx - 22, 200, '+', 13, 'middle', 'bold', check=False)
    g.text(bx - 34, 243, 'mid', 11, fill=MUTED)
    for k in ('+', 'MID', '-'):
        mark('BT1', k)
    # ---------------- U3 BMS
    box(160, 140, 140, 220, 'U3  2S BMS', title_y=132)
    for name, yy in (('B+', YP), ('BM', 250), ('B-', YG)):
        g.line(bx if name != 'BM' else bx, yy, 160, yy); g.text(168, yy + 4, name, 12); mark('U3', name)
    wl(120, YP - 6, 'W1'); wl(120, 244, 'W3'); wl(120, YG - 6, 'W2')
    for name, yy in (('P+', YP), ('P-', YG)):
        g.text(292, yy + 4, name, 12, 'end'); mark('U3', name)
    g.text(230, 232, 'balancing', 11, 'middle', fill=MUTED); g.text(230, 248, 'cut-off', 11, 'middle', fill=MUTED)
    g.text(230, 264, '~2.5 V/cell', 11, 'middle', fill=MUTED)
    g.line(300, YP, 450, YP); g.line(300, YG, 1100, YG)
    # ---------------- J1 charge jack
    dot(370, YP); dot(370, YG)
    g.line(370, YP, 370, 220); g.line(370, 280, 370, YG)
    box(345, 220, 50, 60, None, fill='#fff')
    g.text(370, 247, 'J1', 12, 'middle', 'bold'); g.text(370, 262, 'jack', 11, 'middle', fill=MUTED)
    g.text(377, 216, '+', 11, check=False); g.text(377, 294, '-', 11, check=False)
    g.text(370, 384, 'J1 charge 8.4 V', 11, 'middle', fill=MUTED)
    mark('J1', '+'); mark('J1', '-')
    wl(340, YP - 6, 'W4'); wl(330, YG + 14, 'W10'); wl(398, 200, 'W9', 'start')
    # ---------------- F1
    g.rect(450, YP - 8, 46, 16, rx=7, fill='#fff'); g.line(456, YP + 8, 490, YP - 8, w=1.2)
    g.text(473, YP - 18, 'F1 PTC 3 A', 12, 'middle', 'bold')
    mark('F1', '1'); mark('F1', '2')
    g.line(496, YP, 540, YP); wl(518, YP + 16, 'W5')
    # ---------------- SW1
    box(540, 140, 120, 110, 'SW1  Pololu 2810', title_y=132)
    g.text(548, YP + 4, 'VIN', 12); g.text(652, YP + 4, 'VOUT', 12, 'end'); g.text(600, 240, 'GND', 12, 'middle')
    g.line(565, 205, 580, 190, w=2); g.circle(563, 207, 2.5); g.circle(583, 207, 2.5)
    g.text(620, 210, 'slide', 11, fill=MUTED)
    g.line(600, 250, 600, YG); dot(600, YG); wl(606, 300, 'W6', 'start')
    for k in ('VIN', 'GND', 'VOUT'):
        mark('SW1', k)
    g.line(660, YP, 710, YP); wl(685, YP - 6, 'W7'); wl(685, YG - 6, 'W8')
    # ---------------- K2
    box(710, 150, 40, 200, None, fill='#fff')
    g.text(730, 142, 'K2', 12, 'middle', 'bold')
    g.text(730, YP + 4, '1', 11, 'middle'); g.text(730, YG + 4, '2', 11, 'middle')
    g.text(730, 252, 'XH', 10, 'middle', fill=MUTED)
    mark('K2', '1'); mark('K2', '2')
    g.line(750, YP, 790, YP)
    # ---------------- JB
    box(790, 130, 310, 250, 'JB  junction board (perfboard)', fill='#fbf7ef', title_y=122, dash='5 4')
    g.text(798, YP - 6, 'K2.1', 10, fill=MUTED); g.text(798, YG - 6, 'K2.2', 10, fill=MUTED)
    g.text(1092, YP - 6, 'VB', 11, 'end', 'bold'); g.text(1092, YG - 6, 'GND', 11, 'end', 'bold')
    for k in ('K2.1', 'K2.2', 'VB', 'GND', 'VS'):
        mark('JB', k)
    g.line(790, YP, 1100, YP)
    dot(860, YP); g.line(860, YP, 860, 220); lab(860, 220, 'VBAT_SW')
    g.text(806, 262, 'R1, R2, C2 sit on JB: drawn below,', 11, fill=MUTED)
    g.text(806, 278, 'next to the ESP32 pin they feed.', 11, fill=MUTED)
    g.text(806, 300, 'VS: 1-pin header -> W17 -> SVP', 11, fill=MUTED)
    sch_nets['VBAT_SW'] += 1; sch_nets['PACK+'] += 2; sch_nets['VBAT_F'] += 2; sch_nets['CELL+'] += 2; sch_nets['CELL_MID'] += 2
    sch_nets['CELL-'] += 2; sch_nets['GND'] += 6
    # ---------------- feeds to the MX1508 and the buck
    wl(1110, YP - 6, 'W11', 'start'); wl(1108, YG - 6, 'W12', 'start')
    g.line(1100, YP, 1640, YP); dot(1250, YP)
    g.line(1250, YP, 1250, 560)
    g.line(1100, YG, 1150, YG); g.line(1150, YG, 1150, 560)
    dot(1150, 470); dot(1250, 470)
    g.line(1150, 470, 1196, 470); g.line(1204, 470, 1250, 470)
    g.line(1196, 458, 1196, 482, w=2.4); g.line(1204, 458, 1204, 482, w=2.4)
    g.text(1212, 456, '+', 12, 'middle', 'bold', check=False)
    g.text(1200, 446, 'C1 470 uF 16 V', 12, 'middle', 'bold')
    g.text(1200, 500, 'on the U4 pads', 11, 'middle', fill=MUTED)
    mark('C1', '+'); mark('C1', '-')
    g.line(1640, YP, 1640, 1100); g.line(1640, 1100, 1190, 1100); wl(1600, 1094, 'W13 (from U4 +)')
    sch_nets['VBAT_SW'] += 3; sch_nets['GND'] += 2
    # ---------------- U4 MX1508
    box(1110, 560, 220, 350, None, fill='#f3fbf6')
    g.text(1220, 640, 'U4  MX1508', 13, 'middle', 'bold')
    g.text(1220, 658, 'dual H-bridge', 11, 'middle', fill=MUTED)
    g.text(1220, 674, 'no enable pins', 11, 'middle', fill=MUTED)
    g.text(1150, 580, '-', 13, 'middle', 'bold'); g.text(1250, 580, '+', 13, 'middle', 'bold')
    mark('U4', '+'); mark('U4', '-')
    for k, yy in (('IN1', 800), ('IN2', 830), ('IN3', 860), ('IN4', 890)):
        g.text(1118, yy + 4, k + ('  nc' if k in ('IN3', 'IN4') else ''), 12, fill=INK if k in ('IN1', 'IN2') else NC_COL)
        if k in ('IN3', 'IN4'):
            nc(1110, yy, 'l'); mark('U4', k)
    for k, lab_, yy, used in (('MA1', 'MOTOR-A', 640, True), ('MA2', 'MOTOR-A', 700, True), ('MB1', 'MOTOR-B  nc', 780, False), ('MB2', 'MOTOR-B  nc', 840, False)):
        g.text(1322, yy + 4, lab_, 12, 'end', fill=INK if used else NC_COL)
        if not used:
            nc(1330, yy, 'r')
        mark('U4', k)
    g.text(1220, 724, 'channel A only', 11, 'middle', fill=MUTED)
    # motor
    g.line(1330, 640, 1380, 640); g.line(1330, 700, 1380, 700); wl(1355, 634, 'W20'); wl(1355, 716, 'W21')
    box(1380, 615, 30, 110, None, fill='#fff'); g.text(1395, 607, 'K1', 12, 'middle', 'bold')
    g.text(1395, 644, '1', 11, 'middle'); g.text(1395, 704, '2', 11, 'middle')
    mark('K1', '1'); mark('K1', '2')
    g.line(1410, 640, 1560, 640); g.line(1410, 700, 1560, 700); wl(1430, 634, 'W22'); wl(1430, 716, 'W23')
    dot(1470, 640); dot(1470, 700); g.line(1470, 640, 1470, 666); cap_v(1470, 666, 'C3', '100 nF'); g.line(1470, 673, 1470, 700)
    mark('C3', '1'); mark('C3', '2')
    g.line(1560, 640, 1560, 646); g.line(1560, 694, 1560, 700)
    g.circle(1560, 670, 24, fill='#fff', sw=1.8); g.text(1560, 675, 'M', 14, 'middle', 'bold', check=False)
    g.text(1560, 744, 'M1 TT 1:48', 12, 'middle', 'bold'); g.text(1560, 760, 'dual shaft', 11, 'middle', fill=MUTED)
    mark('M1', '1'); mark('M1', '2')
    sch_nets['MOT_1'] += 4; sch_nets['MOT_2'] += 4

    # ---------------- U5 ESP32 (row B on the left, row A on the right)
    ex0, ex1, ey0 = 620, 920, 530
    py = lambda k: 560 + 30 * (k - 1)
    box(ex0, ey0, ex1 - ex0, 600, None, fill='#f1f4fb')
    g.text((ex0 + ex1) / 2, 1172, 'U5  ESP32 DevKitC 38-pin (ESP32-WROOM-32)', 13, 'middle', 'bold')
    g.text((ex0 + ex1) / 2, 1190, 'top = antenna end, bottom = USB end. Left: row B, right: row A', 11, 'middle', fill=MUTED)
    for row, x, side in (('B', ex0, 'l'), ('A', ex1, 'r')):
        for k in range(1, 20):
            key = esp_key(row, k); lab_, net, _ = PIN[('U5', key)]
            used = net is not None
            alt = ALT.get(lab_)
            s = lab_ + (f' ({alt})' if alt and used else '')
            if side == 'l':
                g.text(x + 8, py(k) + 4, s, 11, fill=INK if used else NC_COL, weight='bold' if used else 'normal')
                g.text(x - 4, py(k) - 4, f'{row}{k}', 8, 'end', fill=MUTED, check=False)
            else:
                g.text(x - 8, py(k) + 4, s, 11, 'end', fill=INK if used else NC_COL, weight='bold' if used else 'normal')
                g.text(x + 4, py(k) - 4, f'{row}{k}', 8, fill=MUTED, check=False)
            if not used:
                nc(x, py(k), side); mark('U5', key)
    g.text(770, 840 + 4, 'GPIO2 drives the on-board', 10, 'middle', fill=MUTED)
    g.text(770, 856 + 4, 'LED: no wire', 10, 'middle', fill=MUTED)

    # ---------------- U6 GY-521
    box(250, 530, 200, 270, None, fill='#f3fbf6')
    g.text(350, 520, 'U6  GY-521 (MPU-6050)', 12, 'middle', 'bold')
    g.text(330, 720, 'MPU-6050', 12, 'middle', fill=MUTED); g.text(330, 736, 'I2C 0x68', 12, 'middle', fill=MUTED)
    g.text(330, 752, 'header pins', 10, 'middle', fill=MUTED); g.text(330, 766, 'top to bottom', 10, 'middle', fill=MUTED)
    gy = dict(VCC=560, GND=590, SCL=620, SDA=650, XDA=680, XCL=710, AD0=740, INT=770)
    for k, yy in gy.items():
        used = PIN[('U6', k)][1] is not None
        g.text(442, yy + 4, k, 12, 'end', weight='bold' if used else 'normal', fill=INK if used else NC_COL)
        if not used:
            nc(450, yy, 'r'); g.text(466, yy + 4, 'not connected', 11, fill=NC_COL); mark('U6', k)
    # wires IMU <-> ESP32
    g.poly([(ex1, py(1)), (950, py(1)), (950, 490), (500, 490), (500, 560), (450, 560)], col=BLUE_TX, w=1.8)
    mark('U5', '3V3'); mark('U6', 'VCC'); sch_nets['3V3'] += 2
    g.text(725, 484, 'W24  3V3 (A1) -> GY-521 VCC', 11, 'middle', fill=BLUE_TX)
    g.poly([(450, 590), (560, 590), (560, 560), (ex0, 560)]); mark('U6', 'GND'); mark('U5', 'GND.B1'); sch_nets['GND'] += 2
    g.poly([(450, 620), (ex0, 620)]); mark('U6', 'SCL'); mark('U5', 'IO22'); sch_nets['SCL'] += 2
    g.poly([(450, 650), (590, 650), (590, 710), (ex0, 710)]); mark('U6', 'SDA'); mark('U5', 'IO21'); sch_nets['SDA'] += 2
    wl(505, 584, 'W25'); wl(505, 614, 'W26  SCL'); wl(520, 644, 'W27  SDA')
    g.text(250, 830, 'AD0 is pulled low on the GY-521: address 0x68.', 11, fill=MUTED)
    g.text(250, 846, 'Tied to 3V3 it would be 0x69 and the firmware', 11, fill=MUTED)
    g.text(250, 862, 'would not find it. SCL/SDA pull-ups are on the', 11, fill=MUTED)
    g.text(250, 878, 'GY-521 (to its 3.3 V): add none.', 11, fill=MUTED)

    # ---------------- divider (on JB) at SVP
    dv = 1002
    box(990, 505, 115, 210, None, fill='none', dash='5 4')
    g.text(1041, 498, 'on JB', 11, 'middle', fill=MUTED)
    g.line(dv, 530, dv, 545); g.line(dv, 530, 1012, 530); lab(1012, 530, 'VBAT_SW')
    resistor_v(dv, 545, 'R1', '100k'); g.line(dv, 579, dv, py(3)); dot(dv, py(3))
    g.line(ex1, py(3), dv, py(3)); wl(955, py(3) - 6, 'W17')
    g.line(dv, py(3), dv, 640); resistor_v(dv, 640, 'R2', '33k'); g.line(dv, 674, dv, 690); gnd(dv, 690)
    g.line(dv, py(3), 1058, py(3)); g.line(1058, py(3), 1058, 650); cap_v(1058, 650, 'C2', '100n')
    g.line(1058, 657, 1058, 680); gnd(1058, 680)
    for r_, k in (('R1', '1'), ('R1', '2'), ('R2', '1'), ('R2', '2'), ('C2', '1'), ('C2', '2')):
        mark(r_, k)
    mark('U5', 'SVP'); sch_nets['VSENSE'] += 2

    # ---------------- motor inputs with pull-downs
    g.line(ex1, py(9), 1110, py(9)); g.line(ex1, py(10), 1110, py(10)); wl(1000, py(9) - 6, 'W18'); wl(1000, py(10) + 14, 'W19')
    dot(1060, py(9)); g.line(1060, py(9), 1060, 784); resistor_v(1060, 750, 'R3', '100k', 'l'); g.line(1060, 750, 1060, 742); gnd_up(1060, 742)
    dot(1085, py(10)); g.line(1085, py(10), 1085, 845); resistor_v(1085, 845, 'R4', '100k', 'l'); g.line(1085, 879, 1085, 884); gnd(1085, 884)
    for r_, k in (('R3', '1'), ('R3', '2'), ('R4', '1'), ('R4', '2')):
        mark(r_, k)
    mark('U5', 'IO25'); mark('U5', 'IO26'); mark('U4', 'IN1'); mark('U4', 'IN2'); sch_nets['M_IN1'] += 2; sch_nets['M_IN2'] += 2

    # ---------------- milestone B: wheel sensor
    g.line(ex1, py(5), 936, py(5)); mark('U5', 'IO34'); sch_nets['IR_D0'] += 1
    g.poly([(936, py(5)), (942, py(5) - 8), (984, py(5) - 8), (984, py(5) + 8), (942, py(5) + 8), (936, py(5))], col=BLUE, w=1.2, fill=BLUE_BG)
    g.text(946, py(5) + 4, 'IR_D0', 10, fill=BLUE_TX)
    bx0, by0 = 30, 905
    box(bx0, by0, 556, 300, None, fill='none', dash='6 4')
    g.text(bx0 + 12, by0 + 20, 'Milestone B: wheel sensor. Leave all of this out for milestone A.', 12, weight='bold')
    g.text(bx0 + 12, by0 + 37, 'U8 (bare TCRT5000, clip under the core) reads the left-wheel ring; U7 (its module) on the back of the core.', 10, fill=MUTED)
    yk = {'1': 985, '2': 1015, '3': 1045}
    box(245, 965, 30, 100, None, fill='#fff'); g.text(260, 958, 'K3', 12, 'middle', 'bold')
    for k, yy in yk.items():
        g.text(260, yy + 4, k, 11, 'middle'); mark('K3', k)
    box(325, 965, 140, 135, None, fill='#f3fbf6')
    for k, yy in (('VCC', 985), ('GND', 1015), ('D0', 1045), ('A0', 1075)):
        used = k != 'A0'
        g.text(333, yy + 4, k + ('  nc' if not used else ''), 11, weight='bold' if used else 'normal', fill=INK if used else NC_COL)
        if not used:
            nc(325, yy, 'l')
        mark('U7', k)
    for k, yy in (('SA', 985), ('SK', 1015), ('SC', 1045), ('SE', 1075)):
        g.text(457, yy + 4, k[1], 11, 'end', 'bold'); mark('U7', k)
    for i, t in enumerate(('U7', 'module', 'LM393')):
        g.text(410, 1004 + i * 14, t, 11 if i else 12, 'middle', 'bold' if i == 0 else 'normal', fill=INK if i == 0 else MUTED)
    g.text(395, 1092, 'sensor pads', 9, 'middle', fill=MUTED)
    for yy, t in ((985, 'W31'), (1015, 'W32'), (1045, 'W33')):
        g.line(275, yy, 325, yy); wl(300, yy - 6, t)
    box(530, 965, 50, 135, None, fill='#fbf7ef')
    for k, yy in (('A', 985), ('K', 1015), ('C', 1045), ('E', 1075)):
        g.text(537, yy + 4, k, 11, weight='bold'); mark('U8', k)
    for i, t in enumerate(('U8', 'TCRT', '5000')):
        g.text(563, 1000 + i * 14, t, 10, 'middle', 'bold' if i == 0 else 'normal')
    g.text(555, 1093, 'LED / det.', 8, 'middle', fill=MUTED, check=False)
    for yy, t in ((985, 'W34'), (1015, 'W35'), (1045, 'W36'), (1075, 'W37')):
        g.line(465, yy, 530, yy); wl(512, yy - 6, t)
    dot(482, 1045); dot(482, 1075); g.line(482, 1045, 482, 1056); g.line(472, 1056, 492, 1056, w=2.4); g.line(472, 1063, 492, 1063, w=2.4)
    g.line(482, 1063, 482, 1075); g.text(486, 1095, 'C5 10n', 10, 'middle', 'bold')
    mark('C5', '1'); mark('C5', '2')
    sch_nets['IR_A'] += 2; sch_nets['IR_C'] += 2
    g.line(90, yk['1'], 245, yk['1']); lab(90, yk['1'], '3V3', 'l'); g.text(98, yk['1'] - 6, 'W28: Y-lead on ESP32 3V3 (A1)', 10, fill=MUTED)
    g.line(220, yk['2'], 245, yk['2']); gnd(220, yk['2']); g.text(42, yk['2'] + 4, 'W29: Y-lead on GND (A14)', 10, fill=MUTED)
    g.line(90, yk['3'], 245, yk['3']); lab(90, yk['3'], 'IR_D0', 'l'); g.text(98, yk['3'] - 6, 'W30: to IO34 (A5)', 10, fill=MUTED)
    dot(150, yk['3']); g.line(150, yk['3'], 150, 1055); resistor_v(150, 1055, 'R5', '100k', 'l'); g.line(150, 1089, 150, 1095); gnd(150, 1095)
    dot(190, yk['3']); g.line(190, yk['3'], 190, 1068); cap_v(190, 1068, 'C4', '10n'); g.line(190, 1075, 190, 1095); gnd(190, 1095)
    for r_ in ('R5', 'C4'):
        mark(r_, '1'); mark(r_, '2')
    sch_nets['3V3'] += 1; sch_nets['IR_D0'] += 1
    g.text(bx0 + 12, 1140, 'D0: about 0 V and the D0 LED on over a window (yellow rim reflects), about 3.0 V over a black bar.', 10, fill=MUTED)
    g.text(bx0 + 12, 1154, 'R5 holds IO34 low with the module unplugged. C4 (with the module pull-up) and C5 (on the detector) are', 10, fill=MUTED)
    g.text(bx0 + 12, 1168, '0.1 ms low-passes against motor noise. Power from 3V3, never 5 V: GPIO34 is a 3.3 V pin.', 10, fill=MUTED)
    g.text(bx0 + 12, 1182, 'Pigtail: A, K = IR LED (blue lens); C, E = phototransistor (black lens). K and E are module GND.', 10, fill=MUTED)

    # ---------------- U1 Mini360 and the ESP32 supply
    box(1030, 1010, 160, 120, None, fill='#eef3fd')
    g.text(1110, 1066, 'U1  Mini360', 12, 'middle', 'bold'); g.text(1110, 1082, 'set to 5.00 V', 11, 'middle', fill=MUTED)
    g.text(1038, 1044, 'OUT-', 11); g.text(1038, 1104, 'OUT+', 11)
    g.text(1182, 1044, 'IN-', 11, 'end'); g.text(1182, 1104, 'IN+', 11, 'end')
    for k in ('IN+', 'IN-', 'OUT+', 'OUT-'):
        mark('U1', k)
    g.line(ex1, py(19), 1030, py(19)); wl(975, py(19) - 6, 'W15'); mark('U5', '5V'); sch_nets['5V'] += 2
    g.line(ex1, py(14), 950, py(14)); gnd(950, py(14)); mark('U5', 'GND.A14')
    g.line(1030, 1040, 1005, 1040); gnd(1005, 1040); wl(1003, 1034, 'W16', 'end')
    g.line(1190, 1040, 1215, 1040); gnd(1215, 1040); wl(1222, 1034, 'W14', 'start')

    # ---------------- notes
    nx, ny = 1250, 800
    g.text(nx + 120, ny, 'Notes', 13, weight='bold')
    nts = ['All motor current: BMS -> F1 -> SW1 -> K2 -> JB',
           '-> U4, and back on W12/W8. 22 AWG on that loop.',
           'Slow decay: one IN held high, the other PWM-ed',
           'at 20 kHz. Average motor V = duty x VBAT;',
           'the firmware caps it at 6.0 V (u = 1).',
           'R3/R4 hold IN1/IN2 low while the ESP32 boots,',
           'so the motor stays in coast until setup() runs.',
           'Charge through J1 with SW1 OFF (8.4 V CC/CV).',
           'VSENSE = VBAT x 33/133: 8.4 V -> 2.08 V.']
    for i, t in enumerate(nts):
        g.text(nx + 120, ny + 22 + i * 17, t, 11)
    g.text(30, 1230, 'Ground symbols: ESP32 GND (A14) to U1 OUT- is wire W16; U1 IN- to U4 - is W14; R2, C2 to JB GND are perfboard tracks; '
                     'R3, R4 to U4 - are their own legs.', 11, fill=MUTED)
    g.text(30, 1248, 'Pin tags A1..A19 and B1..B19 count from the antenna end. Wiring.svg shows the same pins as you see them on the robot, '
                     'counted from the USB end.', 11, fill=MUTED)
    g.save(DOCS / 'schematic.svg', 'schematic.svg')
    return g


# ============================================================================ wiring.svg (the robot, unfolded)
# Two faces of the mast, unfolded about its -y (robot-right) edge: the BACK on the left, seen from behind, and the
# FRONT on the right, seen from the front. Wires that go round the -y edge are straight lines across the gutter.
SH, SV = 11.0, 7.0                     # px per mm, horizontal (y) and vertical (z)
XB0, XF0 = 830.0, 990.0                # x of the -y edge on the back and the front face
BX = lambda y: XB0 - (y + 31.0) * SH   # back face: robot left (+y) on the image left
FX = lambda y: XF0 + (y + 31.0) * SH   # front face: robot left (+y) on the image right
ZTOP, Z0 = 158.0, 142.0                # z at the top of the drawing area, and its y px
# (z, px/mm above it): piecewise vertical scale, not to scale. Crowded bands (under the ESP32, round K1) get more room.
ZBREAK = [(158.0, 0.0), (135.0, 4.0), (103.0, 7.0), (96.0, 24.0), (90.0, 16.0), (79.0, 7.0), (60.0, 9.0), (44.0, 7.0), (14.0, 4.5), (-28.0, 6.0)]


def ZY(z):
    y, zp = Z0, ZBREAK[0][0]
    for zb, k in ZBREAK[1:]:
        if z >= zb:
            return y + (zp - z) * k
        y += (zp - zb) * k; zp = zb
    return y + (zp - z) * ZBREAK[-1][1]


def P(face, y, z):
    return ((BX if face == 'B' else FX)(y), ZY(z))


# where each pin lands in the drawing: (face, y, z) in the body frame, mm. Board outlines from stage1.py (final,
# 2026-11-14); pad positions on each module are drawn where they are on common versions, but they vary: go by the
# printed names.
ESP_ROW_Z = dict(A=102.0, B=126.0)                         # lower row = row A (3V3 ... 5V), upper = row B; board z 100..128


def esp_y(k_from_antenna):
    return -22.2 + (k_from_antenna - 1) * 2.54             # antenna end at -y, USB end at +y (USB to the robot's left)


PHYS = {('U5', esp_key(r, k)): ('B', esp_y(k), ESP_ROW_Z[r]) for r in 'AB' for k in range(1, 20)}
PHYS.update({
    ('U4', '+'): ('B', -11.3, 77.6), ('U4', '-'): ('B', -13.8, 77.6),
    ('U4', 'IN1'): ('B', -14.12, 96.5), ('U4', 'IN2'): ('B', -11.58, 96.5), ('U4', 'IN3'): ('B', -9.04, 96.5), ('U4', 'IN4'): ('B', -6.5, 96.5),
    ('U4', 'MA1'): ('B', -6.0, 77.6), ('U4', 'MA2'): ('B', -8.5, 77.6), ('U4', 'MB1'): ('B', -22.0, 77.6), ('U4', 'MB2'): ('B', -24.5, 77.6),
    ('U1', 'IN-'): ('B', 5.2, 89.0), ('U1', 'IN+'): ('B', 5.2, 85.5), ('U1', 'OUT-'): ('B', 19.8, 89.0), ('U1', 'OUT+'): ('B', 19.8, 82.5),
    ('K1', '1'): ('B', -1.2, 78.0), ('K1', '2'): ('B', 1.2, 78.0),
    ('M1', '1'): ('B', -1.2, 59.0), ('M1', '2'): ('B', 1.2, 59.0),
    ('U3', 'BM'): ('F', -22.5, 119.0), ('U3', 'B+'): ('F', -22.5, 112.0), ('U3', 'B-'): ('F', -22.5, 105.0),
    ('U3', 'P+'): ('F', 16.0, 102.2), ('U3', 'P-'): ('F', 10.0, 102.2),
    ('F1', '1'): ('F', 15.6, 98.4), ('F1', '2'): ('F', 15.6, 95.8),
    ('SW1', 'VIN'): ('F', 18.0, 92.2), ('SW1', 'GND'): ('F', 13.5, 92.2), ('SW1', 'VOUT'): ('F', 9.0, 92.2),
    ('K2', '1'): ('F', -7.5, 94.0), ('K2', '2'): ('F', -10.0, 94.0),
    ('JB', 'K2.1'): ('F', -7.5, 91.0), ('JB', 'K2.2'): ('F', -10.0, 91.0), ('JB', 'VS'): ('F', -24.5, 90.4),
    ('JB', 'GND'): ('F', -24.5, 86.4), ('JB', 'VB'): ('F', -24.5, 82.4),
    ('U6', 'VCC'): ('F', -8.9, 30.5), ('U6', 'GND'): ('F', -6.35, 30.5), ('U6', 'SCL'): ('F', -3.8, 30.5),
    ('U6', 'SDA'): ('F', -1.27, 30.5), ('U6', 'XDA'): ('F', 1.27, 30.5), ('U6', 'XCL'): ('F', 3.8, 30.5),
    ('U6', 'AD0'): ('F', 6.35, 30.5), ('U6', 'INT'): ('F', 8.9, 30.5),
    ('J1', '+'): ('F', 29.5, 114.0), ('J1', '-'): ('F', 27.5, 114.0),
    ('BT1', '+'): ('F', -34.0, 135.0), ('BT1', '-'): ('F', -36.0, 135.0), ('BT1', 'MID'): ('F', 36.0, 135.0),
    # milestone B. The mount is the mechanical design's (stage1.py); until it lands, drawn at the core's left side by the
    # left wheel: module below the axle line, K3 between it and the GY-521.
    ('U7', 'VCC'): ('B', -3.81, 44.6), ('U7', 'GND'): ('B', -1.27, 44.6), ('U7', 'D0'): ('B', 1.27, 44.6), ('U7', 'A0'): ('B', 3.81, 44.6),
    ('U7', 'SA'): ('B', 1.2, 16.5), ('U7', 'SK'): ('B', 0.0, 16.5), ('U7', 'SC'): ('B', -1.2, 16.5), ('U7', 'SE'): ('B', -2.4, 16.5),
    ('U8', 'A'): ('B', 2.9, -16.6), ('U8', 'K'): ('B', 2.9, -19.1), ('U8', 'C'): ('B', 2.9, -21.9), ('U8', 'E'): ('B', 2.9, -24.4),
    ('K3', '1'): ('B', -4.4, 57.0), ('K3', '2'): ('B', -3.5, 57.0), ('K3', '3'): ('B', -2.6, 57.0),
})
BOARDS_W = {   # drawn outlines: face, y0, y1, z0, z1, fill, stroke (boxes from stage1.py; K1 drawn narrower than its 7.5 mm)
    'U5': ('B', -27.5, 27.5, 100.0, 128.0, '#eef3fd', '#2a62d0'), 'U4': ('B', -27.0, -2.3, 76.5, 97.5, '#e3f2ea', '#2f7d5b'),
    'U1': ('B', 4.0, 21.2, 80.0, 91.2, '#eef3fd', '#2a62d0'), 'K1': ('B', -2.0, 2.0, 60.0, 78.0, '#ffffff', '#1d2433'),
    'U3': ('F', -25.0, 25.0, 101.0, 123.0, '#e3f2ea', '#2f7d5b'), 'SW1': ('F', 6.0, 21.2, 77.0, 92.2, '#e9ebef', '#1d2433'),
    'JB': ('F', -25.5, -4.5, 79.0, 92.0, '#f6e7cf', '#8a5a2b'), 'J1': ('F', 25.5, 31.0, 100.0, 114.0, '#d9dde5', '#1d2433'),
    'U6': ('F', -10.5, 10.5, 16.0, 32.0, '#e3f2ea', '#2f7d5b'),
    'K3': ('B', -5.2, -1.8, 50.0, 57.0, '#ffffff', '#1d2433'), 'U7': ('B', -7.0, 7.0, 14.0, 46.0, '#e3f2ea', '#2f7d5b'),
    'U8': ('B', 2.9, 9.9, -25.6, -15.4, '#3a3f4b', '#1d2433'),
}
PADS = {   # multi-wire pads and plug ends drawn as small rectangles: (face, y0, y1, z0, z1)
    ('U3', 'P+'): ('F', 14.6, 17.8, 101.2, 103.2), ('U3', 'P-'): ('F', 8.0, 12.0, 101.2, 103.2),
    ('U4', '+'): ('B', -12.2, -10.4, 77.0, 78.2), ('U4', '-'): ('B', -14.7, -12.9, 77.0, 78.2),
    ('K1', '1'): ('B', -1.9, -0.5, 60.0, 78.0), ('K1', '2'): ('B', 0.5, 1.9, 60.0, 78.0),
    ('K3', '1'): ('B', -4.7, -4.1, 50.0, 57.0), ('K3', '2'): ('B', -3.8, -3.2, 50.0, 57.0), ('K3', '3'): ('B', -2.9, -2.3, 50.0, 57.0),
}

# Every wire as a full path in (face, y, z), first point on the 'a' pin, last on the 'b' pin. 'B' = back, 'F' = front.
# A path that changes face crosses at y = -31: round the robot-right edge, or (W11, W12) under the plate's bottom edge.
ROUTES = {
    'W1': [('F', -34, 135), ('F', -34, 134.2), ('F', -25.6, 134.2), ('F', -25.6, 112), ('F', -22.5, 112)],
    'W2': [('F', -36, 135), ('F', -36, 133.4), ('F', -26.6, 133.4), ('F', -26.6, 105), ('F', -22.5, 105)],
    'W3': [('F', 36, 135), ('F', 36, 134.8), ('F', -22.5, 134.8), ('F', -22.5, 119)],
    'W4': [('F', 15.6, 101.2), ('F', 15.6, 98.4)],
    'W5': [('F', 15.6, 95.8), ('F', 15.6, 94.2), ('F', 18, 94.2), ('F', 18, 92.2)],
    'W6': [('F', 10.0, 101.2), ('F', 10.0, 95.2), ('F', 13.5, 95.2), ('F', 13.5, 92.2)],
    'W7': [('F', 9, 92.2), ('F', 9, 96.4), ('F', -7.5, 96.4), ('F', -7.5, 94.0)],
    'W8': [('F', 8.5, 101.2), ('F', 8.5, 100.0), ('F', -10, 100.0), ('F', -10, 94.0)],
    'W9': [('F', 29.5, 114), ('F', 29.5, 115), ('F', 32, 115), ('F', 32, 99.6), ('F', 17.2, 99.6), ('F', 17.2, 101.2)],
    'W10': [('F', 27.5, 114), ('F', 27.5, 116), ('F', 32.8, 116), ('F', 32.8, 99.0), ('F', 11.6, 99.0), ('F', 11.6, 101.2)],
    'W11': [('F', -24.5, 82.4), ('F', -26.4, 82.4), ('F', -26.4, 73.0), ('F', -31, 73.0), ('B', -31, 73.0), ('B', -11.3, 73.0), ('B', -11.3, 77.0)],
    'W12': [('F', -24.5, 86.4), ('F', -27.4, 86.4), ('F', -27.4, 74.0), ('F', -31, 74.0), ('B', -31, 74.0), ('B', -13.8, 74.0), ('B', -13.8, 77.0)],
    'W13': [('B', -11.3, 78.2), ('B', -11.3, 85.5), ('B', 5.2, 85.5)],
    'W14': [('B', -13.8, 78.2), ('B', -13.8, 89.0), ('B', 5.2, 89.0)],
    'W15': [('B', 19.8, 82.5), ('B', 22.8, 82.5), ('B', 22.8, 99.0), ('B', esp_y(19), 99.0), ('B', esp_y(19), 102)],
    'W16': [('B', 19.8, 89.0), ('B', 19.8, 98.4), ('B', esp_y(14), 98.4), ('B', esp_y(14), 102)],
    'W17': [('F', -24.5, 90.4), ('F', -27.7, 90.4), ('F', -27.7, 100.0), ('F', -31, 100.0), ('B', -31, 100.0), ('B', esp_y(3), 100.0), ('B', esp_y(3), 102)],
    'W18': [('B', esp_y(9), 102), ('B', esp_y(9), 99.4), ('B', -14.12, 99.4), ('B', -14.12, 96.5)],
    'W19': [('B', esp_y(10), 102), ('B', esp_y(10), 98.9), ('B', -11.58, 98.9), ('B', -11.58, 96.5)],
    'W20': [('B', -6.0, 77.6), ('B', -6.0, 79.0), ('B', -1.2, 79.0), ('B', -1.2, 78.0)],
    'W21': [('B', -8.5, 77.6), ('B', -8.5, 79.8), ('B', 1.2, 79.8), ('B', 1.2, 78.0)],
    'W22': [('B', -1.2, 60.0), ('B', -1.2, 59.0)],
    'W23': [('B', 1.2, 60.0), ('B', 1.2, 59.0)],
    'W24': [('B', esp_y(1), 102), ('B', esp_y(1), 98.15), ('B', -31, 98.15), ('F', -31, 98.15), ('F', -29.45, 98.15), ('F', -29.45, 66.5),
            ('F', -8.9, 66.5), ('F', -8.9, 30.5)],
    'W25': [('B', esp_y(1), 126), ('B', esp_y(1), 129.0), ('B', -29.1, 129.0), ('B', -29.1, 98.6), ('B', -31, 98.6), ('F', -31, 98.6),
            ('F', -29.0, 98.6), ('F', -29.0, 67.0), ('F', -6.35, 67.0), ('F', -6.35, 30.5)],
    'W26': [('B', esp_y(3), 126), ('B', esp_y(3), 130.0), ('B', -29.8, 130.0), ('B', -29.8, 99.1), ('B', -31, 99.1), ('F', -31, 99.1),
            ('F', -28.55, 99.1), ('F', -28.55, 67.5), ('F', -3.8, 67.5), ('F', -3.8, 30.5)],
    'W27': [('B', esp_y(6), 126), ('B', esp_y(6), 131.0), ('B', -30.5, 131.0), ('B', -30.5, 99.6), ('B', -31, 99.6), ('F', -31, 99.6),
            ('F', -28.1, 99.6), ('F', -28.1, 68.0), ('F', -1.27, 68.0), ('F', -1.27, 30.5)],
    # milestone B (mechanical.md): module cable up the back of the core, tied with K1, behind the MX1508 to row A
    'W28': [('B', -4.4, 57.0), ('B', -4.4, 97.75), ('B', esp_y(1), 97.75), ('B', esp_y(1), 102)],
    'W29': [('B', -3.5, 57.0), ('B', -3.5, 98.55), ('B', esp_y(14), 98.55), ('B', esp_y(14), 102)],
    'W30': [('B', -2.6, 57.0), ('B', -2.6, 98.15), ('B', esp_y(5), 98.15), ('B', esp_y(5), 102)],
    'W31': [('B', -4.4, 50.0), ('B', -4.4, 47.8), ('B', -3.81, 47.8), ('B', -3.81, 44.6)],
    'W32': [('B', -3.5, 50.0), ('B', -3.5, 48.6), ('B', -1.27, 48.6), ('B', -1.27, 44.6)],
    'W33': [('B', -2.6, 50.0), ('B', -2.6, 49.4), ('B', 1.27, 49.4), ('B', 1.27, 44.6)],
    # the sensor pigtail: from the leads at the back of the clip, under the core and up the back crash-arm root
    'W34': [('B', 2.9, -16.6), ('B', 1.2, -16.6), ('B', 1.2, 16.5)],
    'W35': [('B', 2.9, -19.1), ('B', 0.0, -19.1), ('B', 0.0, 16.5)],
    'W36': [('B', 2.9, -21.9), ('B', -1.2, -21.9), ('B', -1.2, 16.5)],
    'W37': [('B', 2.9, -24.4), ('B', -2.4, -24.4), ('B', -2.4, 16.5)],
}
for _k in ('W28', 'W29', 'W30', 'W34', 'W35', 'W36', 'W37'):
    ROUTES[_k] = ROUTES[_k][::-1]          # drawn from the module side; the model runs ESP32 -> K3 and pads -> sensor
DRAW_ORDER = ['W1', 'W2', 'W3', 'W4', 'W5', 'W6', 'W8', 'W7', 'W9', 'W10', 'W24', 'W25', 'W26', 'W27', 'W11', 'W12', 'W17',
              'W13', 'W14', 'W15', 'W16', 'W18', 'W19', 'W20', 'W21', 'W22', 'W23', 'W28', 'W29', 'W30', 'W31', 'W32', 'W33', 'W34', 'W35', 'W36', 'W37']


def draw_wiring():
    W, H = 1780, 1900
    g = Svg(W, H, 'Tailwhip stage 1 wiring')
    g.text(30, 40, 'Tailwhip stage 1: every wire, end to end, on the robot (not to scale)', 18, weight='bold')
    g.text(30, 64, 'The mast unfolded about its robot-right (-y) edge: BACK on the left, seen from behind; FRONT on the right, seen from the front. '
                   'A wire crossing the gap passes from one face to the other.', 12, fill=MUTED)
    g.text(30, 82, 'Positions from stage1/stage1.py. Line colour = wire colour to use. Pins named as printed on each board; '
                   'bold = used, grey = not connected. A hop means two wires cross without touching.', 12, fill=MUTED)

    def P2(face, y, z):
        if y <= -31.0 + 1e-6:                    # on the edge: the gap side of the face
            return (XB0 if face == 'B' else XF0, ZY(z))
        return P(face, y, z)

    # faces: mast plate z 72..132 (foot 72..76), core below
    for face, title in (('B', 'BACK of the mast and the core, seen from behind'), ('F', 'FRONT of the mast and the core, seen from the front')):
        xa, xb = sorted((P(face, 31, 0)[0], P(face, -31, 0)[0]))
        g.rect(xa, ZY(132), xb - xa, ZY(72) - ZY(132), col=GRID, fill='#f7f8fb', rx=4)
        cx = (xa + xb) / 2
        g.text(cx, 116, title, 14, 'middle', 'bold')
        ca, cb_ = sorted((P(face, -11.8, 0)[0], P(face, 11.8, 0)[0]))
        g.rect(ca, ZY(72), cb_ - ca, ZY(-15) - ZY(72), col=GRID, fill='#f7f8fb', rx=4)
        g.text(cx if face == 'F' else BX(-10.5) + 8, ZY(-15) + 18, 'core, around the gearbox' if face == 'F' else 'core (motor inside)', 11, 'middle' if face == 'F' else 'start', fill=MUTED)
        if face == 'B':
            g.text(xa + 4, ZY(132) + 14, 'robot left (+y)', 10, fill=MUTED)
        else:
            g.text(xb - 4, ZY(132) + 14, 'robot left (+y)', 10, 'end', fill=MUTED)
    for x in (XB0, XF0):
        g.line(x, ZY(134), x, ZY(60), col=GRID, w=1, dash='3 4')
    gx = (XB0 + XF0) / 2
    for i, t in enumerate(('the gap is the', 'robot-right edge.', 'W17 and the IMU', 'jumpers go round', 'it at z 98;', 'W11, W12 pass',
                           'under the plate', 'at y -15.')):
        g.text(gx, ZY(120) + i * 13, t, 10, 'middle', fill=MUTED)

    # motor (dashed: inside the core, behind the back face)
    ma, mb = sorted((BX(-10), BX(10)))
    g.rect(ma, ZY(59), mb - ma, ZY(26) - ZY(59), col='#8a6d1f', fill='none', rx=6, dash='5 4')
    ga, gb_ = sorted((BX(-11.2), BX(11.2)))
    g.rect(ga, ZY(26), gb_ - ga, ZY(-11) - ZY(26), col='#8a6d1f', fill='none', rx=3, dash='5 4')
    for i, t in enumerate(('M1 TT motor (dashed:', 'inside the core). Its', 'leads leave by the slot', 'in the back of the core', 'into K1.')):
        g.text(BX(12.6), ZY(52) + i * 13, t, 10 if i else 11, 'end', 'bold' if i == 0 else 'normal', fill='#6b5418')
    g.text(BX(12.6), ZY(8), 'gearbox (dashed)', 10, 'end', fill='#6b5418')
    g.line(BX(-14), ZY(0), BX(14), ZY(0), col='#8a6d1f', w=1, dash='2 3')
    g.text(BX(-14) + 6, ZY(0) + 4, 'axle', 10, fill='#6b5418')

    # boards
    for ref, (face, y0, y1, z0, z1, fill, col) in BOARDS_W.items():
        xa, xb = sorted((P(face, y0, 0)[0], P(face, y1, 0)[0]))
        g.rect(xa, ZY(z1), xb - xa, ZY(z0) - ZY(z1), col=col, fill=fill, rx=4, sw=1.6 if ref != 'K1' else 1.3, board=ref)
    hx0, hx1 = FX(-38.75), FX(38.75)
    g.rect(hx0, ZY(155.5), hx1 - hx0, ZY(135) - ZY(155.5), col='#8a5a2b', fill='#f6e7cf', rx=6, board='BT1')
    g.text((hx0 + hx1) / 2, ZY(151.5), 'BT1  2 x 18650 holder, on the battery tray on top, under the strap', 12, 'middle', 'bold')
    g.text((hx0 + hx1) / 2, ZY(147.0), 'red + and black - leads leave through the tray\'s robot-right notch; the strap joining the cells is at the other end',
           11, 'middle', fill=MUTED)
    g.text(hx0 + 30, ZY(138.5), '- lead   + lead', 10, fill=MUTED); g.text(hx1 - 10, ZY(138.5), 'strap = cell middle', 10, 'end', fill=MUTED)
    # multi-wire pads
    for (ref, key), (face, y0, y1, z0, z1) in PADS.items():
        if ref in ('K1', 'K3'):
            continue
        xa, xb = sorted((P(face, y0, 0)[0], P(face, y1, 0)[0]))
        g.rect(xa, ZY(z1), xb - xa, ZY(z0) - ZY(z1), col=INK, fill='#e2c46a', rx=2, sw=1.2)

    # wires
    wd = {w['id']: w for w in WIRES}
    for wid in DRAW_ORDER:
        w = wd[wid]
        route = ROUTES[wid]
        for end, q in ((w['a'], route[0]), (w['b'], route[-1])):
            f, y, z = PHYS[pin_of(end)]
            pad = PADS.get(pin_of(end))
            ok = (q[0] == f and (abs(q[1] - y) < 0.3 and abs(q[2] - z) < 0.3 or
                                 pad and pad[1] - 0.1 <= q[1] <= pad[2] + 0.1 and pad[3] - 0.1 <= q[2] <= pad[4] + 0.1))
            if not ok:
                problems.append(f'wiring.svg: {wid} does not start/end on {end}')
        pts = []
        for (f, y, z) in route:
            p_ = P2(f, y, z)
            if pts and abs(pts[-1][0] - p_[0]) < 0.01 and abs(pts[-1][1] - p_[1]) < 0.01:
                continue
            pts.append((round(p_[0], 1), round(p_[1], 1)))
        g.wire(wid, pts, WIRE_RGB[w['colour']], 3.2 if w['awg'] == 22 else 2.4, net_of(w['a']), dash='8 4' if wid in MILESTONE_B_WIRES else None)
        own = {pin_of(w['a'])[0], pin_of(w['b'])[0]}          # a wire may lie on the boards it is soldered to
        if 'K2' in own:
            own.add('JB')                                     # K2 stands on JB
        if wid in ('W28', 'W29', 'W30'):
            own.add('U4')                                     # the sensor cable runs behind the MX1508 (mechanical.md)
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            for (name, rx0, ry0, rx1, ry1) in g.rects:
                if name in own:
                    continue
                if max(x1, x2) > rx0 + 2 and min(x1, x2) < rx1 - 2 and max(y1, y2) > ry0 + 2 and min(y1, y2) < ry1 - 2:
                    problems.append(f'wiring.svg: wire {wid} runs through {name}')

    def pt(ref, key):
        f, y, z = PHYS[(ref, key)]
        return P(f, y, z)

    def pin_dot(ref, key):
        x, y = pt(ref, key)
        if PIN[(ref, key)][1] is not None:
            g.circle(x, y, 3.8, col=INK, fill='#ffffff', sw=1.6)
        else:
            g.circle(x, y, 2.6, col=NC_COL, fill=NC_COL, sw=0)

    # ---- ESP32
    for r in 'AB':
        for k in range(1, 20):
            key = esp_key(r, k); lab_, net, _ = PIN[('U5', key)]
            used = net is not None
            pin_dot('U5', key)
            x, y = pt('U5', key)
            col = INK if used else NC_COL
            if r == 'A':
                g.text(x + 4, y - 8, lab_, 10, 'start', 'bold' if used else 'normal', col, rot=-90)
            else:
                g.text(x + 4, y + 8, lab_, 10, 'end', 'bold' if used else 'normal', col, rot=-90)
    for r, z in (('A', ESP_ROW_Z['A']), ('B', ESP_ROW_Z['B'])):
        g.text(BX(27.5) - 4, ZY(z) + 4, f'row {r}', 10, 'end', 'bold', fill=BLUE_TX)
    g.text(BX(27.5) - 4, ZY(ESP_ROW_Z['A']) + 18, 'pin 1 = USB end', 9, 'end', fill=BLUE_TX)
    ux0, ux1 = sorted((BX(29.5), BX(25.5)))
    g.rect(ux0, ZY(119), ux1 - ux0, ZY(109) - ZY(119), col=INK, fill='#c9ced8', rx=2)
    g.text((ux0 + ux1) / 2 + 3, ZY(114), 'USB', 9, 'middle', fill=INK, check=False, rot=-90)
    g.text(BX(2), ZY(117.0), 'U5  ESP32 DevKitC, 38 pins', 13, 'middle', 'bold', fill=BLUE_TX)
    g.text(BX(2), ZY(114.2), 'seen from the pin side; the module faces the mast', 11, 'middle', fill=BLUE_TX)
    g.text(BX(2), ZY(111.6), 'labels are printed on the far side: count pins from the USB end', 11, 'middle', fill=BLUE_TX)
    # ---- MX1508
    for key in ('IN1', 'IN2', 'IN3', 'IN4', 'MA1', 'MA2', 'MB1', 'MB2'):
        pin_dot('U4', key)
    x, y = pt('U4', '+'); g.text(x, y - 12, '+', 14, 'middle', 'bold')
    x, y = pt('U4', '-'); g.text(x, y - 12, '-', 14, 'middle', 'bold')
    for key in ('IN1', 'IN2', 'IN3', 'IN4'):
        x, y = pt('U4', key); used = key in ('IN1', 'IN2')
        g.text(x + 4, y + 8, key, 10, 'end', 'bold' if used else 'normal', INK if used else NC_COL, rot=-90)
    x1_, _ = pt('U4', 'MA1'); x2_, _ = pt('U4', 'MA2'); g.text((x1_ + x2_) / 2, ZY(81.6), 'MOTOR-A', 10, 'middle', 'bold')
    x1_, y1_ = pt('U4', 'MB1'); x2_, _ = pt('U4', 'MB2'); g.text((x1_ + x2_) / 2, y1_ - 12, 'MOTOR-B: nc', 10, 'middle', fill=NC_COL)
    g.text(BX(-20.5), ZY(93.0), 'U4  MX1508', 12, 'middle', 'bold', fill='#2f7d5b')
    g.text(BX(-20.5), ZY(91.2), 'R3 R4 underneath', 10, 'middle', fill='#2f7d5b')
    cx_, cy_ = BX(-21.5), ZY(84.0)
    g.circle(cx_, cy_, 22, col=INK, fill='#d9dde5', sw=1.4)
    g.text(cx_, cy_ - 2, 'C1', 10, 'middle', 'bold', check=False); g.text(cx_, cy_ + 10, '470u', 9, 'middle', check=False)
    xp, yp = pt('U4', '+'); xm, ym = pt('U4', '-')
    g.line(cx_ + 18, cy_ + 8, xm - 6, ym - 4, w=1.2); g.line(cx_ + 20, cy_, xp - 6, yp - 6, w=1.2)
    # ---- Mini360
    for key in ('IN+', 'IN-', 'OUT+', 'OUT-'):
        pin_dot('U1', key)
        x, y = pt('U1', key)
        g.text(x + (-7 if key.startswith('IN') else 7), y + 4, key, 10, 'end' if key.startswith('IN') else 'start', 'bold')
    g.text(BX(12.5), ZY(87.6), 'U1 Mini360', 10, 'middle', 'bold', fill=BLUE_TX)
    g.text(BX(12.5), ZY(85.6), '5.00 V', 9, 'middle', fill=BLUE_TX)
    # ---- K1, motor tabs, C3
    g.text(BX(3.2), ZY(70.0), 'K1 motor plug,', 10, 'end', 'bold')
    g.text(BX(3.2), ZY(70.0) + 13, 'tied on the back', 10, 'end')
    g.text(BX(3.2), ZY(70.0) + 26, 'of the core', 10, 'end')
    pin_dot('M1', '1'); pin_dot('M1', '2')
    g.line(BX(-1.2), ZY(59) + 4, BX(-1.2), ZY(56.5)); g.line(BX(1.2), ZY(59) + 4, BX(1.2), ZY(56.5))
    g.line(BX(-1.2), ZY(56.5), BX(1.2), ZY(56.5), w=2.4)
    g.text(BX(2.6), ZY(57.5) + 4, 'C3 100n across the tabs', 9, 'end', fill=INK)
    # ---- BMS
    for key in ('B+', 'BM', 'B-'):
        pin_dot('U3', key)
        x, y = pt('U3', key); g.text(x + 8, y + 4, key, 11, 'start', 'bold')
    for key in ('P+', 'P-'):
        x, y = pt('U3', key); g.text(x, y - 14, key, 11, 'middle', 'bold')
    g.text(FX(0), ZY(117.5) + 4, 'U3  2S BMS, balancing', 12, 'middle', 'bold', fill='#2f7d5b')
    g.text(FX(0), ZY(114.0) + 4, 'pad positions vary: go by the printed names', 10, 'middle', fill='#2f7d5b')
    g.text(FX(0), ZY(111.0) + 4, 'never connect anything else to B-', 10, 'middle', fill='#b0362a')
    # ---- F1
    fx = FX(15.6); fz0, fz1 = ZY(98.4), ZY(95.8)
    g.rect(fx - 7, fz0, 14, fz1 - fz0, col=INK, fill='#fff3b0', rx=6, board='F1')
    g.text(fx + 10, ZY(97.1) + 4, 'F1 PTC 3 A', 10, 'start', 'bold')
    # ---- switch
    for key in ('VIN', 'GND', 'VOUT'):
        pin_dot('SW1', key)
        x, y = pt('SW1', key); g.text(x + 4, y + 8, key, 10, 'end', 'bold', rot=-90)
    g.text(FX(13.6), ZY(81.5), 'SW1', 11, 'middle', 'bold')
    g.text(FX(13.6), ZY(79.6), 'Pololu 2810', 9, 'middle')
    # ---- JB and K2
    ka, kb = sorted((FX(-11.2), FX(-6.3)))
    g.rect(ka, ZY(95.0), kb - ka, ZY(91.0) - ZY(95.0), col=INK, fill='#ffffff', rx=2, sw=1.3)
    for key in ('1', '2'):
        pin_dot('K2', key)
    g.text(FX(-5.6), ZY(93.0) + 4, 'K2 plug', 10, 'start', 'bold')
    for key in ('VB', 'GND', 'VS'):
        pin_dot('JB', key)
        x, y = pt('JB', key); g.text(x + 8, y + 4, key, 10, 'start', 'bold')
    g.text(FX(-13.5), ZY(87.0) + 4, 'JB', 13, 'middle', 'bold', fill='#8a5a2b')
    g.text(FX(-13.5), ZY(84.6) + 4, 'R1 R2 C2', 10, 'middle', fill='#8a5a2b')
    g.text(FX(-13.5), ZY(82.4) + 4, 'perfboard', 10, 'middle', fill='#8a5a2b')
    # ---- J1
    pin_dot('J1', '+'); pin_dot('J1', '-')
    g.text(FX(29.5), ZY(112.0), '+', 10, 'middle', 'bold', check=False); g.text(FX(27.5), ZY(112.0), '-', 10, 'middle', 'bold', check=False)
    g.text(FX(28.2), ZY(108.0), 'J1', 10, 'middle', 'bold')
    for i, t in enumerate(('J1 charge jack', '(8.4 V) hangs in', 'front of the left', 'flange, socket down')):
        g.text(FX(21.9), ZY(91.0) + i * 13, t, 10, 'start', 'bold' if i == 0 else 'normal')
    # ---- GY-521
    for key in ('VCC', 'GND', 'SCL', 'SDA', 'XDA', 'XCL', 'AD0', 'INT'):
        used = PIN[('U6', key)][1] is not None
        pin_dot('U6', key)
        x, y = pt('U6', key)
        g.text(x + 4, y + 8, key, 10, 'end', 'bold' if used else 'normal', INK if used else NC_COL, rot=-90)
    for i, t in enumerate(('U6  GY-521', '2 mm off the core front,', 'two M2.5 screws,', 'chip forward, X arrow up', '(or 16 wide x 21 tall)')):
        g.text(FX(-12.0), ZY(31.0) + i * 13, t, 11 if i == 0 else 10, 'end', 'bold' if i == 0 else 'normal', fill='#2f7d5b')
    g.text(FX(12.5), ZY(30.5) + 4, 'XDA XCL AD0 INT:', 10, 'start', fill=NC_COL)
    g.text(FX(12.5), ZY(30.5) + 17, 'not connected', 10, 'start', fill=NC_COL)
    yi = ZY(58)
    for i, t in enumerate(('IMU cable, four 20 cm jumpers:', 'W24 red: ESP32 3V3 -> VCC', 'W25 black: ESP32 GND -> GND',
                           'W26 yellow: ESP32 IO22 -> SCL', 'W27 blue: ESP32 IO21 -> SDA',
                           'tied at the core-front tunnels (z 48, 63)')):
        g.text(FX(4), yi + i * 15, t, 10, 'start', 'bold' if 0 < i < 5 else 'normal')

    # ---- milestone B: module, K3, R5 C4, the bare sensor and its pigtail (back of the core)
    for key in ('VCC', 'GND', 'D0', 'A0'):
        pin_dot('U7', key)
        x, y = pt('U7', key); used = key != 'A0'
        g.text(x + 4, y + 8, key, 9, 'end', 'bold' if used else 'normal', INK if used else NC_COL, rot=-90)
    for key in ('SA', 'SK', 'SC', 'SE'):
        pin_dot('U7', key)
        x, y = pt('U7', key)
        g.text(x + 4, y - 8, key[1], 9, 'start', 'bold', rot=-90)
    for i, t in enumerate(('U7', 'module', 'LM393,', 'trimmer', 'and LEDs', 'face you')):
        g.text(BX(-4.6), ZY(36.0) + i * 12, t, 10 if i else 11, 'middle', 'bold' if i == 0 else 'normal', fill='#2f7d5b')
    g.text(BX(-4.6), ZY(19.0) + 4, 'C5 10n', 9, 'middle', 'bold', fill='#2f7d5b')
    g.text(BX(-4.6), ZY(19.0) + 15, 'on C-E', 9, 'middle', fill='#2f7d5b')
    for k in ('A', 'K', 'C', 'E'):
        x, y = pt('U8', k); g.text(x + 6, y + 3, k, 8, 'start', 'bold', fill='#ffffff', check=False)
    g.text(BX(10.6), ZY(-17.0) + 4, 'U8 TCRT5000 (bare), in the clip', 10, 'end', 'bold')
    g.text(BX(10.6), ZY(-17.0) + 17, 'under the core; lenses face the', 10, 'end')
    g.text(BX(10.6), ZY(-17.0) + 30, 'ring on the left wheel (to the left)', 10, 'end')
    g.text(BX(10.6), ZY(-17.0) + 43, 'A, K: LED (blue lens). C, E: detector', 10, 'end')
    g.text(BX(-2.0), ZY(54.0) + 4, 'K3', 10, 'start', 'bold')
    g.text(BX(-1.4), ZY(52.5), 'R5 100k + C4 10n across', 9, 'end')
    g.text(BX(-1.4), ZY(52.5) + 11, 'K3 pins 3-2, robot side', 9, 'end')
    g.text(BX(-3.0), ZY(2.0), 'pigtail: twist A-K and C-E', 9, 'start', fill=INK)
    bxa, bxb = BX(-10.5), BX(12.6) - 150
    g.rect(bxb, ZY(58.5), bxa - bxb, ZY(-27.5) - ZY(58.5), col='#6f7787', fill='none', rx=6, sw=1.4, dash='6 4')
    g.text(bxb + 6, ZY(-6.0), 'Milestone B: wheel sensor', 11, 'start', 'bold', fill='#4a5160')
    g.text(bxb + 6, ZY(-6.0) + 13, '(dashed wires; leave out for A)', 10, 'start', fill='#4a5160')

    # ---- legend: every wire
    ly = ZY(-28) + 40
    g.text(30, ly, 'Every wire. Swatch = wire colour. Thick = 22 AWG power and motor wire; thin = 26 AWG or a Dupont jumper. '
                   'Full detail (gauge, length, connector): electronics.md, Wire list.', 13, weight='bold')
    rows = [w for w in WIRES if w['kind'] == 'wire' and w['id'] not in MILESTONE_B_WIRES]
    per = math.ceil(len(rows) / 3)
    for i, w in enumerate(rows):
        cx = 30 + (i // per) * 565; cy = ly + 26 + (i % per) * 19
        g.line(cx, cy - 4, cx + 26, cy - 4, col=WIRE_RGB[w['colour']], w=4.5 if w['awg'] == 22 else 3)
        g.text(cx + 34, cy, f"{w['id']}  {desc(w['a'])} -> {desc(w['b'])}", 11)
    lb = ly + 26 + per * 19 + 34
    g.text(30, lb, 'Milestone B only (dashed): sensor cable, module lead and sensor pigtail. W28 and W29 are Y-leads sharing the 3V3 pin with W24 and the GND pin with W16.',
           12, weight='bold')
    rows = [w for w in WIRES if w['kind'] == 'wire' and w['id'] in MILESTONE_B_WIRES]
    for i, w in enumerate(rows):
        cx = 30 + (i // 4) * 565; cy = lb + 24 + (i % 4) * 19
        g.line(cx, cy - 4, cx + 26, cy - 4, col=WIRE_RGB[w['colour']], w=3)
        g.add(f"<line x1='{cx + 10}' y1='{cy - 4}' x2='{cx + 14}' y2='{cy - 4}' stroke='#ffffff' stroke-width='5'/>")
        g.text(cx + 34, cy, f"{w['id']}  {desc(w['a'])} -> {desc(w['b'])}", 11)
    g.text(30, lb + 24 + 4 * 19 + 6, 'Joins with no wire: C1 legs on the MX1508 + and - pads (stripe to -); R3 from IN1 to - and R4 from IN2 to - under the MX1508; '
                                     'C3 legs on the motor tabs; R1, R2, C2 and the K2 header on JB; (B) R5, C4 across K3 pins 3-2, C5 on the module C-E pads.', 11, fill=MUTED)
    g.save(DOCS / 'wiring.svg', 'wiring.svg')
    return g


def desc(end):
    ref, key = pin_of(end)
    lab_ = PIN[(ref, key)][0]
    if ref == 'U5':
        if key.startswith('GND.'):
            r, k = key[4], int(key[5:])
        else:
            r, k = ('A', ROW_A.index(key) + 1) if key in ROW_A else ('B', ROW_B.index(key) + 1)
        return f'ESP32 {lab_} (row {r}, pin {20 - k} from USB)'
    names = dict(BT1='holder', U3='BMS', J1='jack', F1='F1', SW1='SW1', K2='K2', JB='JB', U4='MX1508', U1='Mini360',
                 U6='GY-521', K1='K1', M1='motor', K3='K3', U7='module', U8='sensor')
    return f'{names.get(ref, ref)} {lab_}'


# ============================================================================ run and check
sch = draw_schematic()
wir = draw_wiring()

# 1. every wire joins two pins of the same net
for w in WIRES:
    for e in (w['a'], w['b']):
        if pin_of(e) not in PIN:
            problems.append(f'wire {w["id"]}: no pin {e}')
    na, nb = net_of(w['a']), net_of(w['b'])
    if na is None or na != nb:
        problems.append(f'wire {w["id"]}: {w["a"]} ({na}) and {w["b"]} ({nb}) are not on one net')
# 2. every net connected end to end by wires and joins
for net, pins in NETS.items():
    parent = {p: p for p in pins}
    def find(p):
        while parent[p] != p:
            parent[p] = parent[parent[p]]; p = parent[p]
        return p
    for w in WIRES:
        if w['a'] in parent and w['b'] in parent:
            parent[find(w['a'])] = find(w['b'])
    groups = {find(p) for p in pins}
    if len(groups) > 1:
        problems.append(f'net {net}: not connected end to end ({len(groups)} pieces: ' +
                        ' | '.join(','.join(p for p in pins if find(p) == g_) for g_ in groups) + ')')
    if len(pins) < 2:
        problems.append(f'net {net}: only one pin')
# 3. schematic: every pin of every board drawn, with its net
for (ref, key), (lab_, net, _) in PIN.items():
    if (ref, key) not in drawn:
        problems.append(f'schematic: pin {ref}.{key} ({lab_}) not drawn')
for net in NETS:
    if sch_nets.get(net, 0) < 2:
        problems.append(f'schematic: net {net} drawn at {sch_nets.get(net, 0)} place(s)')
# 4. firmware
src = INO.read_text()
def const(name):
    m = re.search(rf'\b{name}\s*=\s*([0-9.x]+)f?\b', src)
    return None if not m else (int(m.group(1), 16) if m.group(1).startswith('0x') else float(m.group(1)))
for k, v in FW.items():
    if k in ('R_TOP', 'R_BOT'):
        continue
    got = const(k)
    if got is None or abs(got - v) > 1e-6:
        problems.append(f'firmware: {k} is {got}, the electronics model says {v}')
m = re.search(r'BATT_DIV\s*=\s*\(([\d.]+)f?\s*\+\s*([\d.]+)f?\)\s*/\s*([\d.]+)f?', src)
if not m or (float(m.group(1)), float(m.group(2)), float(m.group(3))) != (FW['R_TOP'], FW['R_BOT'], FW['R_BOT']):
    problems.append('firmware: BATT_DIV does not match R1 100k / R2 33k')
fw_optional_present = {}
for k, v in FW_OPTIONAL.items():
    got = const(k)
    fw_optional_present[k] = got is not None
    if got is not None and abs(got - v) > 1e-6:
        problems.append(f'firmware: {k} is {got}, the electronics model says {v}')
for k, net in FW_NET.items():
    if k in FW_OPTIONAL and not fw_optional_present[k]:
        continue
    gp = FW.get(k, FW_OPTIONAL.get(k))
    keys = [kk for (r, kk), (lab_, n, _) in PIN.items() if r == 'U5' and gpio_of(lab_) == gp]
    if not keys or PIN[('U5', keys[0])][1] != net:
        problems.append(f'firmware: {k} = GPIO{gp}, but that ESP32 pin is not on net {net}')
if PIN[('U5', 'IO2')][1] is not None:
    problems.append('GPIO2 (LED) should have no wire')

# ============================================================================ electronics.json
def pinmap():
    rows = []
    for r in 'AB':
        for k in range(1, 20):
            key = esp_key(r, k); lab_, net, note = PIN[('U5', key)]
            rows.append(dict(row=r, pin_from_antenna=k, pin_from_usb=20 - k, label=lab_, gpio=gpio_of(lab_), net=net, note=note))
    return rows


out = dict(
    _about='Tailwhip stage 1 electronics, generated with schematic.svg and wiring.svg by stage1/docs/tools/gen_electronics.py. '
           'AI-generated, designed and simulated, not built: every number is [D] datasheet, [E] estimate, [C] chosen, or [S] from sim.py.',
    parts={k: dict(description=v[0], value=v[1]) for k, v in PARTS.items()},
    pins={ref: [dict(key=k, label=lab_, net=net, note=note) for k, lab_, net, note in pins] for ref, pins in PINS.items()},
    nets=NETS,
    esp32_pins=pinmap(),
    wires=[{**{k: v for k, v in w.items() if k not in ('a', 'b')}, 'from': w['a'], 'to': w['b'], 'net': net_of(w['a'])}
           for w in WIRES],
    firmware=dict(file=str(INO.relative_to(STAGE1.parent)), **{k: v for k, v in FW.items()}, **FW_OPTIONAL,
                  optional_present=fw_optional_present),
    milestone_b=dict(about='Wheel sensor, added after the robot balances on the IMU alone (milestone A). Leave these out for milestone A.',
                     parts=sorted(MILESTONE_B), wires=sorted(MILESTONE_B_WIRES),
                     ring='16 windows on the inside of the left wheel, both edges counted: 32 edges per turn, 11.25 deg, 6.4 mm of travel',
                     top_edge_rate_hz=round(21.0 / (2 * math.pi) * 32), edge_period_ms_at_top=round(1000 / (21.0 / (2 * math.pi) * 32), 1),
                     firmware_lockout_ms_published=1.0, rc_filter='module pull-up ~10 k [E] with C4 10 nF: ~0.1 ms rise',
                     d0_high_v_with_r5=round(3.3 * 100 / 110, 2)),
    i2c=dict(clock_hz=400000, address='0x68', pullups='on the GY-521, to its own 3.3 V regulator output', timeout_ms=3),
    battery_sense=dict(divider='R1 100k (top) / R2 33k (bottom), C2 100 nF across R2', ratio=round(33 / 133, 4),
                       pin='GPIO36 (SVP, ADC1_CH0), 11 dB attenuation', v_pin_at_8v4=round(8.4 * 33 / 133, 3),
                       v_pin_at_6v4=round(6.4 * 33 / 133, 3), adc_usable_max_v=2.45, pack_max_readable_v=round(2.45 * 133 / 33, 2),
                       source_impedance_kohm=round(100 * 33 / 133, 1), drain_uA_at_8v4=round(8.4 / 133e3 * 1e6)),
    power=[dict(load=a, rail=b, typical_A=c, peak_A=d, source=e) for a, b, c, d, e in POWER],
)
(DOCS / 'electronics.json').write_text(json.dumps(out, indent=1))
print(f'parts {len(PARTS)}, nets {len(NETS)}, wires {sum(w["kind"] == "wire" for w in WIRES)} + {sum(w["kind"] != "wire" for w in WIRES)} joins, '
      f'wiring hops {wir.hops}')
if problems:
    print('\nPROBLEMS:'); [print(' -', p) for p in problems]; sys.exit(1)
print('firmware PIN_IR: ' + ('present' if fw_optional_present['PIN_IR'] else 'not defined yet (allowed; checked when it appears)'))
print('checks passed: nets connected end to end, every pin drawn, firmware pins and constants, no text overlaps, no wire through a board')
