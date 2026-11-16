"""Stage 1 balancer in MuJoCo: does one TT motor keep it upright with a PID loop, and does one wheel sensor hold it still?

The model is generated from stage1.py's numbers (out/model.json). The motor, gearbox slop, dead zone, lag and
tyre contact are the published Tailwhip simulator's models with the same [E] estimates. The controller is the
firmware's, line for line: MPU-6050 accelerometer + gyro -> complementary filter -> PID at 400 Hz -> one motor.
Two milestones, one controller:
  A  no wheel sensor (or it is unplugged): speed comes from a model of the motor, as in the firmware. Stays up, wanders.
  B  one TCRT5000 reading a 16-window ring on the left wheel (32 edges per turn, no direction): speed and position
     from the edges, the sign from the motor command. The firmware detects the sensor and falls back to A by itself.

  python sim.py tune [ir|imu]   random search for one gain set over randomised robots       -> out/gains.json
  python sim.py test            all the experiments, both milestones side by side            -> out/results.json, out/sim.md, out/plot_*.png
  python sim.py video P [--sensor]  a clip of the nominal test run (IMU only; --sensor: with the wheel sensor) -> P (mp4)
"""
import os, sys, json
from multiprocessing import Pool
import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'out')
MODEL = json.load(open(os.path.join(OUT, 'model.json')))
TICK, SUB = 0.0025, 2                 # 400 Hz control, 800 Hz physics
V_MAX = 6.0                           # firmware caps the motor at 6 V average
ARMATURE = 2.8e-4                     # rotor inertia reflected through the 1:48 gearbox (published model)
FALL = np.radians(22.0)               # past this it is on its way to a crash bar
W_NOLOAD = 21.0                       # rad/s at 6 V, the controller's motor model: 200 rpm, the simulated motor. Measured at
                                      # bring-up step 3 and set with `wnl` in the firmware; this is the default.
NOM = dict(mass=1.0, com_dz=0.0, com_dx=0.0, mu=0.8, stall=0.08, w0=21.0, deadzone=0.10, backlash_deg=5.0,
           motor_tau=0.015, vbat=7.6, pitch_bias_deg=0.0, gyro_bias=0.0, floor_tilt_deg=0.0, seed=0,
           ir='absent', ir_fail_at=None, ring_windows=16, ring_duty_deg=0.5, ring_sd_deg=0.3, ir_hyst_deg=0.5,
           ir_chatter=0.1, ir_drop=0.2, ir_noise_hz=100.0, ir_force_off=False, ir_true_sign=False, sign_rule='pred')
GAINS0 = dict(w_noload=W_NOLOAD, dz=0.10, imu=dict(kp=6.0, kd=0.6, ki=0.0, kv=3.0, kx=0.5), ir=dict(kp=6.0, kd=0.6, ki=0.0, kv=3.0, kx=0.5, tc=0.0))

def xml():
    m = MODEL; c = m['body_com']; I = np.array(m['body_inertia']); Iw = np.array(m['wheel_inertia'])
    sx, sz, sr, shw = m['skid']; b = m['battery']; r = m['r_wheel']; y0, y1 = m['wheel_in'], m['wheel_in'] + m['wheel_w']
    fi = lambda A: f'{A[0,0]:.6e} {A[1,1]:.6e} {A[2,2]:.6e} {A[0,1]:.6e} {A[0,2]:.6e} {A[1,2]:.6e}'
    return f'''<mujoco model="stage1">
  <compiler angle="radian" meshdir="{OUT}" autolimits="true"/>
  <option timestep="{TICK / SUB}" integrator="implicitfast" cone="elliptic"/>
  <visual><global offwidth="1280" offheight="720"/><quality shadowsize="4096" offsamples="4"/>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.55 0.55 0.55" specular="0.1 0.1 0.1"/><map znear="0.005" zfar="8"/></visual>
  <default>
    <geom contype="1" conaffinity="1" condim="3" friction="0.8 0.02 0.001" solref="0.004 1" solimp="0.9 0.95 0.001"/>
    <default class="vis"><geom contype="0" conaffinity="0" group="1"/></default>
    <default class="col"><geom rgba="0 0 0 0" group="3"/></default>
  </default>
  <asset>
    <mesh name="body" file="body.stl" scale="0.001 0.001 0.001"/><mesh name="wheels" file="wheels.stl" scale="0.001 0.001 0.001"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.94 0.93 0.90" rgb2="0.86 0.85 0.81" width="512" height="512"/>
    <material name="desk" texture="grid" texrepeat="40 40" reflectance="0.03"/>
    <texture type="skybox" builtin="flat" rgb1="0.86 0.89 0.93" rgb2="0.86 0.89 0.93" width="16" height="16"/>
  </asset>
  <worldbody>
    <light pos="0.3 -0.5 1.2" dir="-0.3 0.5 -1.2" diffuse="0.5 0.5 0.5" castshadow="true"/>
    <geom name="floor" type="plane" size="4 4 0.01" material="desk"/>
    <body name="chassis" pos="0 0 {r}">
      <freejoint name="root"/>
      <inertial pos="{c[0]} {c[1]} {c[2]}" mass="{m['body_mass']}" fullinertia="{fi(I)}"/>
      <geom class="vis" type="mesh" mesh="body" rgba="0.96 0.62 0.2 1"/>
      <geom class="col" name="skid_f" type="capsule" fromto="{sx} {-shw} {sz} {sx} {shw} {sz}" size="{sr}" friction="0.4 0.02 0.001"/>
      <geom class="col" name="skid_b" type="capsule" fromto="{-sx} {-shw} {sz} {-sx} {shw} {sz}" size="{sr}" friction="0.4 0.02 0.001"/>
      <geom class="col" name="batt" type="box" pos="{b[0]} {b[1]} {b[2]}" size="{b[3]} {b[4]} {b[5]}" friction="0.4 0.02 0.001"/>
      <site name="imu" pos="{m['imu'][0]} {m['imu'][1]} {m['imu'][2]}"/>
      <site name="push" pos="0 0 {b[2]}"/>
      <body name="gear">
        <joint name="motor" type="hinge" axis="0 1 0" armature="{ARMATURE}" damping="0.0004" frictionloss="0.002"/>
        <inertial pos="0 0 0" mass="0.001" diaginertia="1e-8 1e-8 1e-8"/>
        <body name="wheels">
          <joint name="slop" type="hinge" axis="0 1 0" range="-0.04 0.04" damping="0.0002"/>
          <inertial pos="0 0 0" mass="{m['wheel_mass']}" fullinertia="{fi(Iw)}"/>
          <geom class="vis" type="mesh" mesh="wheels" rgba="0.15 0.15 0.16 1"/>
          <geom class="col" name="tyre_l" type="cylinder" fromto="0 {y0} 0 0 {y1} 0" size="{r}"/>
          <geom class="col" name="tyre_r" type="cylinder" fromto="0 {-y1} 0 0 {-y0} 0" size="{r}"/>
        </body>
      </body>
    </body>
    <camera name="track" mode="trackcom" pos="0.25 -0.75 0.28" xyaxes="1 0.33 0 -0.08 0.25 1" fovy="34"/>
  </worldbody>
  <sensor><accelerometer name="acc" site="imu"/><gyro name="gyr" site="imu"/></sensor>
</mujoco>'''

class IrWheel:
    """The firmware's IR wheel sensor (stage1_balance.ino: isrIr, updateIr, irPresent), same constants.
    One TCRT5000 looks at a 16-window ring on the left wheel: 32 edges per turn, magnitude only. The ISR counts edges
    with a 1 ms lockout and keeps the last three edge times. Every control tick: speed = two edges / time across the
    last two edges (one window plus one bar, so a ring printed with uneven windows still reads right); with no new
    edge the estimate can only fall (one edge / time since the last edge) and is 0 after 250 ms. The sensor cannot
    tell direction. The published robot takes the sign of the last motor command; here it is the sign of the
    direction the wheel should be turning on the body, motor model speed minus pitch rate (Controller.SIGN; the
    test's 'sign rule' section shows why). Detection, so milestone A needs no setting:
      UNKNOWN until 8 edges at a believable rate have arrived   -> the controller uses the motor model, as without one
      OK                                                        -> the controller uses the sensor
      FAILED  if more than 4 edges arrived while the gyro was calibrating (a floating input), if 10 edges in a row
              came faster than 1.6 x the no-load speed (noise). Latched: the input is not a wheel.
      back to UNKNOWN if the motor model says the wheel has turned 6 edges' worth on the body since the last edge
              (unplugged, misaligned, dead, or the model was wrong). It needs 8 fresh edges to be trusted again.
    A failed calibration check clears with the next good gyro calibration (firmware "cal")."""
    LOCKOUT, STOP, CONFIRM, MAX_RATIO, FAULT_EDGES, CAL_NOISE, STRIKES = 0.001, 0.25, 8, 1.6, 6, 4, 10
    UNKNOWN, OK, FAILED = 0, 1, 2
    def __init__(self, edges=32, w_noload=None, off=False, cal_edges=0, cheat=False):
        self.rpe = 2 * np.pi / edges; self.wmax = self.MAX_RATIO * (w_noload or W_NOLOAD); self.off = off
        self.n = 0; self.t0 = self.t1 = self.t2 = -1e9; self.prev_n = 0; self.dn = 0; self.rel = 0.0; self.sign = 1.0
        self.status = self.FAILED if cal_edges > self.CAL_NOISE else self.UNKNOWN
        self.good = 0; self.strikes = 0; self.quiet = 0.0; self.faults = 0
        self.cheat = cheat; self.dsum = 0; self.lastdir = 1     # sim-only: the true direction, to price the sign rule
    def isr(self, t, d=0):
        if t - self.t0 > self.LOCKOUT:
            self.n += 1; self.t2 = self.t1; self.t1 = self.t0; self.t0 = t
            self.dsum += d; self.lastdir = d or self.lastdir
            return True
        return False
    def update(self, now, w_model):
        dn = self.n - self.prev_n; self.prev_n = self.n; self.dn = dn
        if dn > 0:
            self.quiet = 0.0
            span = self.t0 - self.t2
            w = 2 * self.rpe / span if span > 0 else 1e9
            if w <= self.wmax:
                self.rel = w; self.good += dn; self.strikes = max(0, self.strikes - 1)
            else:
                self.strikes += 1
                if self.strikes >= self.STRIKES:
                    self.status = self.FAILED
            if self.status == self.UNKNOWN and self.good >= self.CONFIRM:
                self.status = self.OK
        else:
            since = max(now - self.t0, 1e-6)
            self.rel = 0.0 if since > self.STOP else min(self.rel, self.rpe / since)
            if self.status == self.OK:
                self.quiet += abs(w_model) * TICK
                if self.quiet > self.FAULT_EDGES * self.rpe:     # silent: distrust it until 8 fresh edges
                    self.status = self.UNKNOWN; self.good = 0; self.faults += 1
    def present(self):
        return self.status == self.OK and not self.off
    def signed(self):
        return (self.lastdir if self.cheat else self.sign) * self.rel
    def take_dx(self):                                        # signed edges since the last tick, in rad of wheel
        d = self.dsum if self.cheat else self.sign * self.dn
        self.dsum = 0
        return d * self.rpe
    def set_sign(self, u, eps=0.005):
        if u > eps: self.sign = 1.0
        elif u < -eps: self.sign = -1.0

class Controller:
    """The firmware's filter, wheel sensor and PID (firmware/stage1_balance/stage1_balance.ino), same constants.
    Two gain sets: 'imu' (milestone A, no sensor or sensor not trusted) and 'ir' (milestone B, sensor OK)."""
    FILTER_TAU, ACC_GATE, V_EST_TAU = 0.5, 0.25, 0.08
    UX = 0.10            # the position term never asks for more than this (IMU only: 0.137 x 0.5 m = 0.069 at most)
    SIGN = 'pred'        # 'cmd': the published robot's rule (sign of the last command); see test, 'sign rule'
    def __init__(self, g, edges=32, ir_off=False, cal_edges=0, cheat=False):
        g = full_gains(g); self.G = g; self.W_NOLOAD = g['w_noload']; self.dz = g['dz']
        self.pitch = None; self.x = 0.0; self.integ = 0.0; self.v = 0.0; self.u_prev = 0.0; self.pitch_x = None
        self.ir = IrWheel(edges, self.W_NOLOAD, ir_off, cal_edges, cheat); self.vg = 0.0; self.vc = 0.0
    def step(self, acc, gyr, now):
        an = np.sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]) / 9.81
        pa = np.arctan2(-acc[0], acc[2])
        if self.pitch is None:
            self.pitch = pa
        self.pitch += gyr[1] * TICK
        if abs(an - 1.0) < self.ACC_GATE:
            self.pitch += (pa - self.pitch) * TICK / self.FILTER_TAU
        th = self.pitch
        if self.pitch_x is None:
            self.pitch_x = th
        r = MODEL['r_wheel']
        self.ir.update(now, self.v / r - gyr[1])                                            # last tick's model, on the body
        self.v += (self.u_prev * self.W_NOLOAD * r - self.v) * TICK / self.V_EST_TAU        # motor model, always running
        dx = self.ir.take_dx()
        if self.ir.present():                                                               # ground = wheel rel. body + pitch
            g = self.G['ir']; vm = r * (self.ir.signed() + gyr[1])
            if g['tc'] > 0:                                                                 # model speed, corrected slowly
                self.vc += (vm - self.v - self.vc) * TICK / g['tc']
            self.vg = self.v + self.vc
            self.x = np.clip(self.x + r * (dx + th - self.pitch_x), -0.5, 0.5)
        else:
            g = self.G['imu']; self.vg = self.v
            self.x = np.clip(self.x + self.v * TICK, -0.5, 0.5)
        self.pitch_x = th
        self.integ = np.clip(self.integ + th * TICK, -0.2, 0.2)
        u = g['kp'] * th + g['kd'] * gyr[1] + g['ki'] * self.integ + g['kv'] * self.vg + np.clip(g['kx'] * self.x, -self.UX, self.UX)
        u = float(np.clip(u, -1, 1)); self.u_prev = u
        self.ir.set_sign(u if self.SIGN == 'cmd' else self.v / r - gyr[1])         # direction the wheel turns on the body
        if abs(u) > 0.005:
            u += np.sign(u) * self.dz
        return float(np.clip(u, -1, 1))

class Ring:
    """The plant side of the sensor: a printed ring of `windows` windows on the left wheel, seen from the body.
    Both edges of every window count. Edges sit where the print put them (duty error on alternate edges, per-edge
    scatter, random phase), the comparator has hysteresis, slow crossings chatter. `ir`: 'ok', 'absent' (unplugged,
    pull-down fitted: no edges), 'flaky' (drops a fraction of edges), 'floating' (unplugged, no pull-down: mains-hum
    edges). `ir_fail_at`: the sensor goes dead at this time."""
    def __init__(self, p, rng, th):
        self.p = p; self.rng = rng; N = 2 * p['ring_windows']; self.N = N; self.d = 2 * np.pi / N
        self.err = np.radians(p['ring_sd_deg']) * rng.normal(0, 1, N) + np.radians(p['ring_duty_deg']) * (np.arange(N) % 2)
        self.phase = rng.uniform(0, self.d); self.h = np.radians(p['ir_hyst_deg']) / 2
        self.k = int(np.floor((th - self.phase) / self.d)); self.pend = []
        while th > self.B(self.k + 1):
            self.k += 1
        while th < self.B(self.k):
            self.k -= 1
    def B(self, j):
        return self.phase + j * self.d + self.err[j % self.N]
    def _emit(self, te, d, speed):
        p = self.p
        if p['ir'] in ('absent', 'floating') or (p['ir_fail_at'] is not None and te >= p['ir_fail_at']):
            return
        if p['ir'] == 'flaky' and self.rng.random() < p['ir_drop']:
            return
        self.pend.append((te, d))
        if self.rng.random() < p['ir_chatter']:
            gap = 0.0004 if speed > 1.0 else 0.0015                    # fast crossings chatter inside the lockout
            self.pend += [(te + gap, d), (te + 2 * gap, d)]
    def step(self, th0, th1, t0, dt, isr):
        """Advance one physics step; deliver edges up to t0 + dt to isr(t, true_dir). Returns delivered (counted) dirs."""
        sp = abs(th1 - th0) / dt
        if th1 > th0:
            while th1 > self.B(self.k + 1) + self.h:
                b = self.B(self.k + 1) + self.h; self.k += 1; self._emit(t0 + dt * (b - th0) / (th1 - th0), 1, sp)
        elif th1 < th0:
            while th1 < self.B(self.k) - self.h:
                b = self.B(self.k) - self.h; self.k -= 1; self._emit(t0 + dt * (b - th0) / (th1 - th0), -1, sp)
        if self.p['ir'] == 'floating':
            for _ in range(self.rng.poisson(self.p['ir_noise_hz'] * dt)):
                self.pend.append((t0 + self.rng.uniform(0, dt), 0))
        out = []; keep = []
        for (te, d) in sorted(self.pend):
            if te <= t0 + dt:
                if isr(te, d):
                    out.append(d)
            else:
                keep.append((te, d))
        self.pend = keep
        return out

def full_gains(g):
    """gains.json: {'w_noload', 'dz', 'imu': {kp, kd, ki, kv, kx}, 'ir': {...}}. A flat dict means one set for both."""
    if g is None:
        g = load_gains()
    if 'imu' not in g:
        s = {k: g[k] for k in ('kp', 'kd', 'ki', 'kv', 'kx') if k in g}
        g = dict(w_noload=W_NOLOAD, dz=g.get('dz', 0.10), imu=dict(GAINS0['imu'], **s), ir=dict(GAINS0['ir'], **s))
    return g

def with_set(G, which, s):
    G = full_gains(G); G = dict(G); G[which] = dict(G[which], **s); return G

_M = None; _NOM = None
def model():
    global _M, _NOM
    if _M is None:
        _M = mujoco.MjModel.from_xml_string(xml())
        _NOM = dict(mass=_M.body_mass.copy(), inertia=_M.body_inertia.copy(), ipos=_M.body_ipos.copy())
    return _M

def randomised(seed, ir=False):
    """A random robot. ir=False: no sensor fitted (milestone A). ir=True: sensor fitted, and honest about it, as the
    published env: 10 % of robots have it unplugged or misaligned (no edges), 10 % flaky (drop 10-30 % of edges), the
    rest work; every ring has its own print errors, hysteresis and chatter. The sensor draws come from their own random
    stream, so robot k is the same robot in both modes."""
    r = np.random.default_rng(seed); u = r.uniform
    p = dict(mass=u(0.85, 1.15), com_dz=u(-0.015, 0.015), com_dx=u(-0.004, 0.004), mu=u(0.4, 1.0), stall=0.08 * u(0.75, 1.25),
             w0=21.0 * u(0.85, 1.15), deadzone=u(0.04, 0.16), backlash_deg=u(2.0, 8.0), motor_tau=u(0.008, 0.025),
             vbat=u(6.6, 8.4), pitch_bias_deg=u(-1.5, 1.5), gyro_bias=u(-0.01, 0.01), floor_tilt_deg=u(-1.5, 1.5), seed=int(seed))
    s = np.random.default_rng(seed + 7_000_000); q = s.random()
    p.update(ring_duty_deg=s.uniform(-1.5, 1.5), ring_sd_deg=s.uniform(0.1, 0.6), ir_hyst_deg=s.uniform(0.2, 1.0),
             ir_chatter=s.uniform(0.0, 0.3), ir_drop=s.uniform(0.1, 0.3))
    p['ir'] = 'absent' if not ir else ('absent' if q < 0.1 else 'flaky' if q < 0.2 else 'ok')
    return p

SENSOR = dict(ir='ok')                    # the nominal robot with its sensor plugged in (milestone B)

def rollout(p=None, gains=None, T=10.0, init_deg=0.0, shoves=(), trace=False, kick=None, frames=None):
    """shoves: (time s, impulse N s at battery height, + = forward). kick: dict to start lying on a crash bar.
    Returns a dict of metrics, plus time series if trace."""
    p = dict(NOM, **(p or {})); m = model(); n = _NOM
    b = m.body('chassis').id
    m.body_mass[b] = n['mass'][b] * p['mass']; m.body_inertia[b] = n['inertia'][b] * p['mass']
    m.body_ipos[b] = n['ipos'][b] + np.array([p['com_dx'], 0, p['com_dz']])
    for gname in ('tyre_l', 'tyre_r'):
        m.geom_friction[m.geom(gname).id, 0] = p['mu']
    bl = np.radians(p['backlash_deg']) / 2
    m.jnt_range[m.joint('slop').id] = (-bl, bl)
    ft = np.radians(p['floor_tilt_deg'])
    m.opt.gravity[:] = 9.81 * np.array([np.sin(ft), 0, -np.cos(ft)])
    d = mujoco.MjData(m); rng = np.random.default_rng(p['seed'] + 1000)
    th0 = np.radians(init_deg if kick is None else kick['side'] * (MODEL['rest_deg'] - 0.3))
    d.qpos[3:7] = [np.cos(th0 / 2), 0, np.sin(th0 / 2), 0]
    mujoco.mj_forward(m, d)
    va = m.joint('motor').dofadr[0]; vs = m.joint('slop').dofadr[0]
    qa = m.joint('motor').qposadr[0]; qs = m.joint('slop').qposadr[0]
    srng = np.random.default_rng(p['seed'] + 2000)                                           # sensor noise: own stream
    cal = int(srng.poisson(p['ir_noise_hz'] * 1.0)) if p['ir'] == 'floating' else 0            # edges during gyro cal
    ctl = Controller(gains, edges=2 * p['ring_windows'], ir_off=p['ir_force_off'], cal_edges=cal, cheat=p['ir_true_sign'])
    ctl.SIGN = p['sign_rule']
    ring = Ring(p, srng, d.qpos[qa] + d.qpos[qs]); fresh = []; wrong = 0; counted = 0; rev_t = -1.0; wrong_rev = 0
    ok_at = None; fail_at = None
    v_motor = 0.0; u = 0.0
    bias = np.radians(p['pitch_bias_deg']); cb, sb = np.cos(bias), np.sin(bias)
    ctl.pitch = th0 + bias          # the filter starts from the accelerometer while the robot is held still
    prev = None; N = int(round(T / TICK)); fell_at = None
    log = dict(t=[], pitch=[], est=[], u=[], w=[], wm=[], x=[], xe=[], vm=[], vt=[], ir=[]) if trace else None
    sat = 0; wmax = 0.0; p2 = 0.0; p1 = 0.0; pmax = 0.0; du = 0.0; phase = 'kick' if kick else 'pid'; t_kick = 0.0
    push = m.site('push').id; s_prev = 1.0
    for k in range(N):
        t = k * TICK
        acc = d.sensordata[0:3] + rng.normal(0, 0.15, 3); gyr = d.sensordata[3:6] + rng.normal(0, 0.01, 3) + p['gyro_bias']
        acc = np.array([cb * acc[0] - sb * acc[2], acc[1], sb * acc[0] + cb * acc[2]])        # IMU mounted a little off
        meas = (acc, gyr)
        if prev is None:
            prev = meas
        s_used = ctl.ir.sign                                                                 # sign the fresh edges get
        u_pid = ctl.step(*prev, t)                                                           # one tick of sensor delay
        prev = meas
        counted += len(fresh); bad = sum(1 for dd in fresh if dd != s_used); wrong += bad
        if bad and t - rev_t < 0.1:
            wrong_rev += bad
        fresh = []
        if ctl.ir.sign != s_prev:
            rev_t = t; s_prev = ctl.ir.sign
        if ok_at is None and ctl.ir.status == IrWheel.OK:
            ok_at = t
        if fail_at is None and ctl.ir.faults and ok_at is not None:
            fail_at = t
        R = d.xmat[b].reshape(3, 3); pitch = np.arctan2(R[0, 2], R[2, 2])
        if phase == 'kick':                                                                  # get up off a crash bar
            s = np.sign(ctl.pitch)
            if t < kick['settle']:
                u = 0.0
            elif t < kick['settle'] + kick['back']:
                u = -s * 1.0                                                                 # roll away first, to load the bar
            elif abs(ctl.pitch) > np.radians(kick['handover']) and t < kick['settle'] + kick['back'] + 1.0:
                u = s * 1.0                                                                  # then drive under it, flat out
            else:
                phase = 'pid'; ctl.x = 0.0; ctl.v = ctl.u_prev * 0.5; t_kick = t
        if phase == 'pid':
            u_new = u_pid; du += (u_new - u) ** 2; u = u_new
        else:
            ctl.u_prev = u
            if ctl.SIGN == 'cmd':
                ctl.ir.set_sign(u)
        for (ts, J) in shoves:
            if ts <= t < ts + 0.05:
                F = np.array([J / 0.05, 0, 0]); d.xfrc_applied[b, :3] = F
                d.xfrc_applied[b, 3:] = np.cross(d.site_xpos[push] - d.xipos[b], F)
            elif abs(t - ts - 0.05) < TICK / 2:
                d.xfrc_applied[b] = 0
        for j in range(SUB):
            v_cmd = (0.0 if abs(u) < p['deadzone'] else u) * min(V_MAX, p['vbat'])
            v_motor += (v_cmd - v_motor) * min(1.0, (TICK / SUB) / p['motor_tau'])
            d.qfrc_applied[va] = p['stall'] * (v_motor / 6.0) - p['stall'] / p['w0'] * d.qvel[va]
            a0 = d.qpos[qa] + d.qpos[qs]
            mujoco.mj_step(m, d)
            fresh += ring.step(a0, d.qpos[qa] + d.qpos[qs], t + j * TICK / SUB, TICK / SUB, ctl.ir.isr)
        w = d.qvel[va] + d.qvel[vs]
        if phase == 'pid' and (kick is None or t > t_kick + 0.3):
            sat += abs(u) >= 0.999; wmax = max(wmax, abs(w)); p2 += pitch ** 2; p1 += pitch; pmax = max(pmax, abs(pitch))
        if trace:
            for key, val in zip(('t', 'pitch', 'est', 'u', 'w', 'wm', 'x', 'xe', 'vm', 'vt', 'ir'),
                                (t, np.degrees(pitch), np.degrees(ctl.pitch), u, w, ctl.ir.signed(), d.qpos[0], ctl.x, ctl.vg,
                                 d.qvel[0], ctl.ir.status)):
                log[key].append(float(val))
        if frames is not None and k % frames['every'] == 0:
            frames['r'].update_scene(d, camera='track'); frames['out'].append(frames['r'].render().copy())
        if fell_at is None and phase == 'pid' and abs(pitch) > FALL and (kick is None or t > t_kick + 0.05):
            fell_at = t
            if not trace and frames is None:
                break
    Rm = d.xmat[b].reshape(3, 3)
    out = dict(ok=fell_at is None and phase == 'pid', fell_at=fell_at, T=T, rms_deg=float(np.degrees(np.sqrt(p2 / N))),
               wobble_deg=float(np.degrees(np.sqrt(max(0.0, p2 / N - (p1 / N) ** 2)))),
               max_deg=float(np.degrees(pmax)), drift=float(d.qpos[0]), side=float(d.qpos[1]), yaw_deg=float(np.degrees(np.arctan2(Rm[1, 0], Rm[0, 0]))),
               sat=float(sat / N), wmax=float(wmax), du=float(np.sqrt(du / N)),
               ir_status=int(ctl.ir.status), ir_ok_at=ok_at, ir_fail_at=fail_at, edges=int(counted), wrong_sign=int(wrong),
               wrong_after_reversal=int(wrong_rev), x_est=float(ctl.x), x_err=float(ctl.x - (d.qpos[0] - 0.0)))
    if trace:
        out['log'] = log
    return out

def load_gains():
    p = os.path.join(OUT, 'gains.json')
    return full_gains(json.load(open(p))) if os.path.exists(p) else full_gains(GAINS0)

# ------------------------------------------------------------------------------------------------ tuning
def _score(G):
    """IMU-only set: (robots that survive a 5 deg release and two 30 mN s shoves, mean wobble standing still, mean saturation)."""
    ok = 0; wob = []; sat = []
    for s in range(100, 112):
        sg = 1 if s % 2 else -1
        ok += rollout(randomised(s), G, T=7.0, init_deg=5.0 * sg, shoves=((3.0, 0.03 * sg), (5.0, -0.03 * sg)))['ok']
        q = rollout(randomised(s), G, T=5.0)
        wob.append(q['wobble_deg'] if q['ok'] else 9.0); sat.append(q['sat'])
    return ok, float(np.mean(wob)), float(np.mean(sat))

def _score_ir(G):
    """Sensor set, on robots whose sensor works (each with its own ring errors): (robots that survive a random lean
    and two random 20-50 mN s shoves in 20 s, mean distance from the start after 30 s standing still (m, 1 if it fell),
    mean wobble standing still)."""
    ok = 0; dr = []; wob = []; rng = np.random.default_rng(11)
    for s in range(100, 116):
        p = dict(randomised(s, ir=True), ir='ok'); a, b = rng.uniform(0.02, 0.05, 2) * rng.choice([-1, 1], 2)
        ok += rollout(p, G, T=20.0, init_deg=float(rng.uniform(-5, 5)), shoves=((6.0, float(a)), (12.0, float(b))))['ok']
        q = rollout(p, G, T=30.0)
        dr.append(abs(q['drift']) if q['ok'] else 1.0); wob.append(q['wobble_deg'] if q['ok'] else 9.0)
    return ok, float(np.mean(dr)), float(np.mean(wob))

def sample(rng, centre=None, spread=1.0):
    if centre is None:
        return dict(kp=10 ** rng.uniform(-0.3, 1.3), kd=10 ** rng.uniform(-1.5, 0.4), ki=0.0, kv=10 ** rng.uniform(-0.5, 1.0),
                    kx=10 ** rng.uniform(-1.0, 0.5))
    return dict({k: centre[k] * 10 ** rng.normal(0, 0.12 * spread) for k in ('kp', 'kd', 'kv', 'kx')}, ki=0.0)

def sample_ir(rng, centre, spread=1.0):
    """Around the IMU-only set (the inner loop is the same motor model), position gain and speed correction wide."""
    if spread is None:
        return dict({k: centre[k] * 10 ** rng.normal(0, 0.15) for k in ('kp', 'kd', 'kv')}, ki=0.0,
                    kx=10 ** rng.uniform(-1.0, 0.7), tc=float(rng.choice([0.0, 10 ** rng.uniform(-0.3, 1.0)])))
    tc = centre['tc'] * 10 ** rng.normal(0, 0.15 * spread) if centre['tc'] > 0 else 0.0
    return dict({k: centre[k] * 10 ** rng.normal(0, 0.12 * spread) for k in ('kp', 'kd', 'kv', 'kx')}, ki=0.0, tc=tc)

def tune(which='ir'):
    """Random search for one gain set; the other set and w_noload stay as they are in out/gains.json.
    imu: keep the gains that save the most randomised robots; among those, the quietest when standing still. A
    bang-bang controller survives shoves too, but it hammers the gearbox, so quiet wins ties.
    ir: the most robots saved, then the least drift in 30 s plus wobble (1 cm counts as much as 1/3 deg rms)."""
    G = load_gains(); rng = np.random.default_rng(5)
    if which == 'imu':
        score, key = _score, (lambda t: (-t[1][0], t[1][1]))
        first = [dict(G['imu'])] + [sample(rng) for _ in range(900)]
        more = lambda best: [best] + [sample(rng, best, 0.6) for _ in range(200)]
    else:
        score, key = _score_ir, (lambda t: (-t[1][0], 100 * t[1][1] + 3 * t[1][2]))
        first = [dict(G['ir'])] + [dict(G['imu'], tc=0.0, kx=kx) for kx in (0.14, 0.3, 0.5, 0.9)] + [sample_ir(rng, G['imu'], None) for _ in range(200)]
        more = lambda best: [best] + [sample_ir(rng, best, 0.6) for _ in range(80)]
    with Pool() as pool:
        sc = pool.map(score, [with_set(G, which, c) for c in first])
        best, bs = sorted(zip(first, sc), key=key)[0]; print('global', bs, best)
        cands = more(best)
        sc = pool.map(score, [with_set(G, which, c) for c in cands])
        best, bs = sorted(zip(cands, sc), key=key)[0]; print('refine', bs, best)
    G[which] = {k: float(round(v, 3)) for k, v in best.items()}
    json.dump(G, open(os.path.join(OUT, 'gains.json'), 'w'), indent=1)
    return G

# ------------------------------------------------------------------------------------------------ experiments
def _run(a):
    return rollout(**a)

def frac(pool, jobs):
    return float(np.mean([r['ok'] for r in pool.map(_run, jobs)]))

def _best_lean(a):
    """Largest release angle (deg, 2 deg steps, both directions must recover) for one robot variant, with a small
    gain search of its own (IMU-only set) so every variant gets a fair controller. Milestone A."""
    p, G, n = a; rng = np.random.default_rng(7); best = 0; base = G['imu']
    for s in [base] + [sample(rng, base, 2.5) for _ in range(n)]:
        g = with_set(G, 'imu', s); ang = 0
        for th in range(2, 25, 2):
            if all(rollout(p, g, T=4.0, init_deg=sg * th)['ok'] for sg in (1, -1)):
                ang = th
            else:
                break
        best = max(best, ang)
    return best

MODES = (('A', 'IMU only'), ('B', 'wheel sensor'))
def rand_mode(s, mode, **kw):
    """Robot s for a milestone: A has no sensor; B has one fitted, with the honest mix of absent and flaky sensors."""
    return dict(randomised(s, ir=(mode == 'B')), **kw)
def nom_mode(mode, **kw):
    return dict(SENSOR if mode == 'B' else {}, **kw)

def _summ(rr, key='drift'):
    ok = [x for x in rr if x['ok']]; dr = [abs(x[key]) for x in ok] or [np.nan]
    return dict(n=len(rr), survived=len(ok), drift_m=float(np.median(dr)), drift_p90=float(np.percentile(dr, 90)),
                wobble_deg=float(np.median([x['wobble_deg'] for x in ok] or [np.nan])))

def test():
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    g = load_gains(); res = dict(gains=g, modes={}); L = []
    P = lambda s='': L.append(s)
    r_w = MODEL['r_wheel']; SH = ((6.0, 0.05), (12.0, -0.05)); C = dict(A='#1f77b4', B='#d62728')
    with Pool() as pool:
        logs = {}; d60 = {}; kick_params = None
        for M, name in MODES:
            R = res['modes'][M] = {}
            # 1. nominal run
            r = rollout(nom_mode(M), g, T=20.0, init_deg=8.0, shoves=SH, trace=True)
            logs[M] = r.pop('log'); R['nominal'] = r
            r60 = pool.map(_run, [dict(p=nom_mode(M), gains=g, T=60.0)])[0]
            R['nominal_60'] = r60
            # 2. release angle
            seeds = list(range(500, 560)); angs = list(range(1, 23))
            rel = [frac(pool, [dict(p=rand_mode(s, M), gains=g, T=5.0, init_deg=th * (1 if s % 2 else -1)) for s in seeds]) for th in angs]
            rel_nom = max([th for th in angs if all(x['ok'] for x in pool.map(_run, [dict(p=nom_mode(M), gains=g, T=5.0, init_deg=sg * th) for sg in (1, -1)]))] or [0])
            R['release'] = dict(angles=angs, frac=rel, nominal_max=rel_nom)
            # 3. shove
            imps = [round(x, 3) for x in np.arange(0.01, 0.161, 0.01)]
            sh = [frac(pool, [dict(p=rand_mode(s, M), gains=g, T=6.0, shoves=((1.5, J * (1 if s % 2 else -1)),)) for s in seeds]) for J in imps]
            sh_nom = max([J for J in imps if all(x['ok'] for x in pool.map(_run, [dict(p=nom_mode(M), gains=g, T=6.0, shoves=((1.5, sg * J),)) for sg in (1, -1)]))] or [0])
            R['shove'] = dict(impulses=imps, frac=sh, nominal_max=sh_nom)
            # 4. robustness: random lean, two random shoves, 20 s
            rng = np.random.default_rng(3); jobs = []
            for s in range(1000, 1300):
                a, b_ = rng.uniform(0.02, 0.05, 2) * rng.choice([-1, 1], 2)
                jobs.append(dict(p=rand_mode(s, M), gains=g, T=20.0, init_deg=float(rng.uniform(-5, 5)), shoves=((6.0, float(a)), (12.0, float(b_)))))
            rr = pool.map(_run, jobs)
            ok = [x for x in rr if x['ok']]
            R['robust'] = dict(n=len(rr), survived=len(ok), rms_deg=float(np.median([x['rms_deg'] for x in ok])),
                               drift_m=float(np.median([abs(x['drift']) for x in ok])), drift_p90=float(np.percentile([abs(x['drift']) for x in ok], 90)),
                               sat=float(np.median([x['sat'] for x in ok])), wmax_p90=float(np.percentile([x['wmax'] for x in ok], 90)),
                               yaw_p90=float(np.percentile([abs(x['yaw_deg']) for x in ok], 90)))
            fails = [j['p'] for j, x in zip(jobs, rr) if not x['ok']]
            if fails:
                R['robust']['failed_mean'] = {k: float(np.mean([f[k] for f in fails])) for k in ('mu', 'stall', 'w0', 'backlash_deg', 'motor_tau', 'deadzone', 'vbat', 'mass', 'com_dz')}
            if M == 'B':
                R['robust']['by_sensor'] = {kind: _summ([x for j, x in zip(jobs, rr) if j['p']['ir'] == kind]) for kind in ('ok', 'flaky', 'absent')}
            # 5. left alone: 20 s and 60 s
            st = pool.map(_run, [dict(p=rand_mode(s, M), gains=g, T=20.0) for s in range(2000, 2060)])
            R['still'] = dict(survived=int(sum(x['ok'] for x in st)), n=len(st), rms_deg=float(np.median([x['wobble_deg'] for x in st if x['ok']])),
                              drift_m=float(np.median([abs(x['drift']) for x in st if x['ok']])))
            jobs60 = [dict(p=rand_mode(s, M), gains=g, T=60.0) for s in range(2000, 2060)]
            st60 = pool.map(_run, jobs60)
            R['still_60'] = _summ(st60); d60[M] = [max(0.1, abs(x['drift']) * 100) for x in st60 if x['ok']]
            if M == 'B':
                R['still_60']['by_sensor'] = {kind: _summ([x for j, x in zip(jobs60, st60) if j['p']['ir'] == kind]) for kind in ('ok', 'flaky', 'absent')}
                R['still_60']['ok_at_median'] = float(np.median([x['ir_ok_at'] for x in st60 if x['ir_ok_at'] is not None]))
            if M == 'A':
                # 6. what limits it (milestone A, as before): motor speed, battery height, torque
                rpms = [30, 60, 90, 120, 160, 200, 250, 300]
                sp = pool.map(_best_lean, [(dict(w0=rpm * 2 * np.pi / 60), g, 40) for rpm in rpms])
                res['speed'] = dict(rpm=rpms, lean=sp)
                dzs = [-0.06, -0.04, -0.02, 0.0, 0.02, 0.04, 0.06]
                hs = pool.map(_best_lean, [(dict(com_dz=dz), g, 40) for dz in dzs])
                h0 = MODEL['body_com'][2]
                res['height'] = dict(com_mm=[round((h0 + dz) * 1000) for dz in dzs], lean=hs)
                sts = [0.03, 0.04, 0.06, 0.08, 0.10, 0.12, 0.16]
                tq = pool.map(_best_lean, [(dict(stall=s_), g, 40) for s_ in sts])
                res['torque'] = dict(stall=sts, lean=tq)
                # 8. getting up off a crash bar by itself
                best = (0, None)
                for back in (0.0, 0.08, 0.12, 0.16, 0.2, 0.25):
                    for hand in (4, 6, 8, 10, 12, 15):
                        kk = dict(settle=0.6, back=back, handover=hand)
                        f = frac(pool, [dict(p=rand_mode(s, M), gains=g, T=5.0, kick=dict(kk, side=1 if s % 2 else -1)) for s in range(3000, 3040)])
                        if f > best[0]:
                            best = (f, kk)
                kick_params = best[1]
                R['kickup'] = dict(frac=best[0], params=best[1])
            elif kick_params:
                R['kickup'] = dict(frac=frac(pool, [dict(p=rand_mode(s, M), gains=g, T=5.0, kick=dict(kick_params, side=1 if s % 2 else -1)) for s in range(3000, 3040)]),
                                   params=kick_params)
            if R.get('kickup', {}).get('params'):
                R['kickup']['nominal'] = [x['ok'] for x in pool.map(_run, [dict(p=nom_mode(M), gains=g, T=5.0, kick=dict(R['kickup']['params'], side=sd)) for sd in (1, -1)])]
        # ---------------------------------------------------------------- sensor checks (milestone B)
        S = res['sensor'] = {}
        okp = lambda s, **kw: dict(randomised(s, ir=True), ir='ok', **kw)               # every sensor working
        # ring resolution: same gains, the firmware's edge constant follows the ring
        S['ring'] = {}
        for wn in (8, 16, 32):
            a60 = pool.map(_run, [dict(p=okp(s, ring_windows=wn), gains=g, T=60.0) for s in range(4000, 4040)])
            rng = np.random.default_rng(4); jobs = []
            for s in range(4000, 4040):
                a, b_ = rng.uniform(0.02, 0.05, 2) * rng.choice([-1, 1], 2)
                jobs.append(dict(p=okp(s, ring_windows=wn), gains=g, T=20.0, init_deg=float(rng.uniform(-5, 5)), shoves=((6.0, float(a)), (12.0, float(b_)))))
            sh_ = pool.map(_run, jobs)
            S['ring'][wn] = dict(still_60=_summ(a60), shoved=_summ(sh_), edges_per_turn=2 * wn,
                                 wrong_frac=float(np.sum([x['wrong_sign'] for x in a60]) / max(1, np.sum([x['edges'] for x in a60]))))
        # sign rule: the published command rule vs this sketch's rule vs a (sim-only) true direction
        S['sign'] = {}
        for rule, kw in (('command (published robot)', dict(sign_rule='cmd')), ('model speed - pitch rate (this sketch)', {}),
                         ('true direction (sim only, unreachable)', dict(ir_true_sign=True))):
            a60 = pool.map(_run, [dict(p=okp(s, **kw), gains=g, T=60.0) for s in range(4000, 4040)])
            nm = pool.map(_run, [dict(p=nom_mode('B', **kw), gains=g, T=20.0, init_deg=8.0, shoves=SH)])[0]
            E = max(1, np.sum([x['edges'] for x in a60]))
            S['sign'][rule] = dict(still_60=_summ(a60), wrong_frac=float(np.sum([x['wrong_sign'] for x in a60]) / E),
                                   wrong_after_rev_frac=float(np.sum([x['wrong_after_reversal'] for x in a60]) / E),
                                   nominal_shoved=dict(ok=nm['ok'], drift=nm['drift'], x_err=nm['x_err'], edges=nm['edges'], wrong=nm['wrong_sign'],
                                                       wrong_after_reversal=nm['wrong_after_reversal']))
        # the sensor dies mid-run: it must notice, fall back to the motor model and stay up
        fail = rollout(nom_mode('B', ir_fail_at=10.0), g, T=30.0, shoves=((20.0, 0.03),), trace=True)
        flog = fail.pop('log'); S['fail_nominal'] = fail
        rng = np.random.default_rng(6); jobs = []; jobs0 = []
        for s in range(5000, 5060):
            tf = float(rng.uniform(5, 15)); sg = 1 if s % 2 else -1
            jobs.append(dict(p=okp(s, ir_fail_at=tf), gains=g, T=30.0, shoves=((20.0, 0.03 * sg),)))
            jobs0.append(dict(p=okp(s), gains=g, T=30.0, shoves=((20.0, 0.03 * sg),)))
        jobsA = [dict(j, p=dict(j['p'], ir='absent', ir_fail_at=None)) for j in jobs0]
        fr = pool.map(_run, jobs); f0 = pool.map(_run, jobs0); fA = pool.map(_run, jobsA)
        lag = [x['ir_fail_at'] - j['p']['ir_fail_at'] for j, x in zip(jobs, fr) if x['ir_fail_at'] is not None]
        S['fail'] = dict(n=len(fr), survived=int(sum(x['ok'] for x in fr)), survived_no_fault=int(sum(x['ok'] for x in f0)), survived_imu_only=int(sum(x['ok'] for x in fA)),
                         noticed=len(lag), lag_median=float(np.median(lag)) if lag else None, lag_max=float(np.max(lag)) if lag else None)
        # dies just after a shove, when it matters most
        S['fail_after_shove'] = pool.map(_run, [dict(p=nom_mode('B', ir_fail_at=6.05), gains=g, T=20.0, init_deg=8.0, shoves=SH)])[0]
        # unplugged without the pull-down (floating input, mains-hum edges), and "wheel off"
        S['floating'] = pool.map(_run, [dict(p=nom_mode('B', ir='floating'), gains=g, T=20.0, init_deg=8.0, shoves=SH)])[0]
        off = pool.map(_run, [dict(p=nom_mode('B', ir_force_off=True), gains=g, T=20.0, init_deg=8.0, shoves=SH)])[0]
        S['forced_off'] = off; A0 = res['modes']['A']['nominal']
        S['forced_off_same_as_A'] = bool(off['drift'] == A0['drift'] and off['rms_deg'] == A0['rms_deg'])
        # what the robot sees: true wheel speed and measured, around the first shove
        # sample traces for the 60 s plot
        tr = {M: pool.map(_run, [dict(p=rand_mode(s, M) if s else nom_mode(M), gains=g, T=60.0, trace=True) for s in (0, 2000, 2001, 2002, 2003, 2004, 2005)]) for M, _ in MODES}

    for M in ('A', 'B'):
        for x in tr[M]:
            x['log'] = {k: x['log'][k][::20] for k in ('t', 'x')}
    json.dump(res, open(os.path.join(OUT, 'results.json'), 'w'), indent=1, default=float)

    # ---------------------------------------------------------------- plots
    fig, ax = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
    for M, name in MODES:
        lg = logs[M]
        ax[0].plot(lg['t'], lg['pitch'], c=C[M], lw=0.9, label=f'{M}: {name}')
        ax[1].plot(lg['t'], lg['u'], c=C[M], lw=0.8)
        ax[2].plot(lg['t'], np.array(lg['w']) * r_w, c=C[M], lw=0.8)
        ax[3].plot(lg['t'], lg['x'], c=C[M], label=f'{M}: true')
    ax[3].plot(logs['B']['t'], logs['B']['xe'], c=C['B'], ls='--', lw=0.8, label='B: what it thinks')
    ax[0].set_ylabel('pitch (deg)'); ax[0].legend(loc='upper right'); ax[1].set_ylabel('motor command (-1..1)'); ax[1].set_ylim(-1.1, 1.1)
    ax[2].axhline(0.68, ls=':', c='grey'); ax[2].axhline(-0.68, ls=':', c='grey'); ax[2].set_ylabel('wheel rim speed\non the body (m/s)')
    ax[3].set_ylabel('position (m)'); ax[3].set_xlabel('time (s)'); ax[3].legend(loc='lower left', fontsize=8)
    for a in ax:
        a.axvline(6, c='k', lw=0.5, ls='--'); a.axvline(12, c='k', lw=0.5, ls='--'); a.grid(alpha=0.3)
    ax[0].set_title('Nominal robot: released at 8 deg, shoved at 6 s and 12 s (0.05 N s). Dotted: no-load speed.', fontsize=10)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, 'plot_nominal.png'), dpi=110); plt.close(fig)

    fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
    for M, name in MODES:
        R = res['modes'][M]
        ax[0].plot(R['release']['angles'], np.array(R['release']['frac']) * 100, 'o-', c=C[M], ms=3, label=f'{M}: {name}')
        ax[1].plot(np.array(R['shove']['impulses']) * 1000, np.array(R['shove']['frac']) * 100, 'o-', c=C[M], ms=3)
    ax[0].set_xlabel('released at this lean (deg)'); ax[0].set_ylabel('recovered (% of 60 randomised robots)'); ax[0].legend()
    ax[1].set_xlabel('shove at battery height (mN s)')
    for a in ax:
        a.grid(alpha=0.3); a.set_ylim(-3, 103)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, 'plot_limits.png'), dpi=110); plt.close(fig)

    rpms, sp, hs, tq, sts = res['speed']['rpm'], res['speed']['lean'], res['height']['lean'], res['torque']['lean'], res['torque']['stall']
    h0 = MODEL['body_com'][2]
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.6))
    ax[0].plot(rpms, sp, 'o-'); ax[0].axvline(200, c='r', ls='--', lw=0.8); ax[0].set_xlabel('motor no-load speed (rpm at 6 V)'); ax[0].set_ylabel('largest lean recovered (deg)')
    ax[1].plot(res['height']['com_mm'], hs, 'o-'); ax[1].axvline(round(h0 * 1000), c='r', ls='--', lw=0.8); ax[1].set_xlabel('centre of mass above the axle (mm)')
    ax[2].plot(sts, tq, 'o-'); ax[2].axvline(0.08, c='r', ls='--', lw=0.8); ax[2].set_xlabel('motor stall torque at 6 V (N m)')
    for a in ax:
        a.grid(alpha=0.3); a.set_ylim(0, 25)
    fig.suptitle('What limits it (red: this design; IMU only). Each point has its own gain search.', y=1.02)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, 'plot_sweeps.png'), dpi=110, bbox_inches='tight'); plt.close(fig)

    fig, ax = plt.subplots(2, 3, figsize=(15, 7.5))
    a = ax[0, 0]
    for M, name in MODES:
        for i, x in enumerate(tr[M]):
            a.plot(x['log']['t'], x['log']['x'], c=C[M], lw=1.6 if i == 0 else 0.7, alpha=1 if i == 0 else 0.6,
                   label=f'{M}: {name}' if i == 0 else None)
    a.set_xlabel('time (s)'); a.set_ylabel('position (m)'); a.set_title('Left alone for 60 s: nominal (thick) and 6 randomised', fontsize=10); a.legend(fontsize=8)
    a = ax[0, 1]; lg = logs['B']; i0, i1 = int(5.5 / TICK), int(8.0 / TICK)
    a.plot(lg['t'][i0:i1], lg['w'][i0:i1], c='k', lw=0.9, label='true wheel speed on the body')
    a.plot(lg['t'][i0:i1], lg['wm'][i0:i1], c=C['B'], lw=0.9, label='what the sensor reports (signed)')
    a.set_xlabel('time (s)'); a.set_ylabel('rad/s'); a.set_title('Sensor B around the 6 s shove (nominal)', fontsize=10); a.legend(fontsize=8)
    a = ax[0, 2]; t_ = np.array(flog['t'])
    a.plot(t_, flog['x'], c='k', lw=0.9, label='position (m)')
    a2 = a.twinx(); a2.plot(t_, flog['pitch'], c=C['B'], lw=0.5, alpha=0.7); a2.set_ylabel('pitch (deg)', color=C['B'])
    a.axvline(10, c='r', ls='--', lw=0.8, label='sensor dies')
    if fail['ir_fail_at']:
        a.axvline(fail['ir_fail_at'], c='g', ls='--', lw=0.8, label='firmware notices, IMU only')
    a.axvline(20, c='grey', ls=':', lw=0.8, label='30 mN s shove')
    a.set_xlabel('time (s)'); a.set_ylabel('position (m)'); a.set_title('Sensor dies at 10 s (nominal)', fontsize=10); a.legend(fontsize=8, loc='lower left')
    a = ax[1, 0]; wns = [8, 16, 32]
    a.bar([str(w) for w in wns], [S['ring'][w]['still_60']['drift_m'] * 100 for w in wns], color=C['B'], alpha=0.8, label='median')
    a.plot([str(w) for w in wns], [S['ring'][w]['still_60']['drift_p90'] * 100 for w in wns], 'k^', label='90th percentile')
    a.set_xlabel('windows on the ring'); a.set_ylabel('drift in 60 s (cm)'); a.set_title('Ring resolution, 40 robots, every sensor working', fontsize=10); a.legend(fontsize=8)
    a = ax[1, 1]; rules = list(S['sign'])
    a.bar(range(3), [S['sign'][k]['still_60']['drift_m'] * 100 for k in rules], color=['grey', C['B'], '#2ca02c'], alpha=0.8)
    a.plot(range(3), [S['sign'][k]['still_60']['drift_p90'] * 100 for k in rules], 'k^')
    a.set_xticks(range(3)); a.set_xticklabels(['command\n(published)', 'model - pitch rate\n(this sketch)', 'true direction\n(sim only)'], fontsize=8)
    a.set_yscale('log'); a.set_ylabel('drift in 60 s (cm), log'); a.set_title('Where the direction comes from (median, 90th pct)', fontsize=10)
    a = ax[1, 2]
    for M, name in MODES:
        v = np.sort(d60[M]); a.step(v, np.arange(1, len(v) + 1) / len(v) * 100, where='post', c=C[M], label=f'{M}: {name}')
    a.set_xscale('log'); a.set_xlabel('drift in 60 s (cm), log'); a.set_ylabel('% of robots that stood, within this')
    a.set_title('60 randomised robots left alone 60 s (B: honest sensor mix)', fontsize=10); a.legend(fontsize=8); a.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, 'plot_sensor.png'), dpi=110); plt.close(fig)

    # ---------------------------------------------------------------- sim.md
    A, B = res['modes']['A'], res['modes']['B']
    pct = lambda d: f"{d['survived']} of {d['n']} ({100 * d['survived'] / d['n']:.0f} %)"
    cm = lambda v: f'{v * 100:.0f} cm' if v == v else '-'
    half = lambda xs, fs: max([x for x, f in zip(xs, fs) if f >= 0.9] or [0])
    stays = lambda r: 'stays up' if r['ok'] else 'FALLS at %.1f s' % r['fell_at']
    gi, gr = g['imu'], g['ir']
    P('# Stage 1 balancer: simulation results'); P()
    P('Generated by `sim.py test`. MuJoCo, 800 Hz physics, the firmware\'s filter, wheel sensor code and PID at 400 Hz. Motor and '
      'gearbox figures are estimates until the bench measurements exist. Nothing here has run on hardware.'); P()
    P('Two milestones, one sketch. **A: IMU only** (no wheel sensor, or it is unplugged): speed comes from a model of the motor. '
      '**B: wheel sensor**: one TCRT5000 reading a 16-window ring on the left wheel, 32 edges per turn, no direction; the firmware '
      'trusts it after 8 believable edges and falls back to A by itself if it stops. Randomised robots in B have the sensor fitted '
      'the honest way, as the published env does: 10 % unplugged or misaligned, 10 % flaky (missing 10-30 % of edges), and every ring '
      'with its own print errors, comparator hysteresis and chatter.'); P()
    P(f"Motor model no-load speed (`wnl`, measured at bring-up step 3): {g['w_noload']} rad/s at 6 V. Dead-zone compensation {g['dz']}.")
    P(); P('| Gains (u per rad, per rad/s, per m/s, per m) | kp | kd | ki | kv | kx | tc (s) |'); P('|---|---|---|---|---|---|---|')
    P(f"| A, `imu` set | {gi['kp']} | {gi['kd']} | {gi['ki']} | {gi['kv']} | {gi['kx']} | - |")
    P(f"| B, `ir` set (used while the sensor is trusted) | {gr['kp']} | {gr['kd']} | {gr['ki']} | {gr['kv']} | {gr['kx']} | {gr['tc']} |"); P()
    P('In B, v is still the motor model (tc 0: no correction from the measured speed, which is too coarse and too late to balance on) '
      'and x is counted from the wheel edges plus the change in pitch, so it holds position instead of integrating the model.'); P()
    P('## Side by side'); P()
    P('| | A: IMU only | B: wheel sensor |'); P('|---|---|---|')
    na, nb = A['nominal'], B['nominal']
    P(f"| Nominal robot, 20 s, released at 8 deg, two 50 mN s shoves | {stays(na)}, pitch rms {na['rms_deg']:.1f} deg, ends {cm(abs(na['drift']))} away, "
      f"saturated {na['sat'] * 100:.0f} %, peak wheel {na['wmax'] * r_w:.2f} m/s | {stays(nb)}, pitch rms {nb['rms_deg']:.1f} deg, ends {cm(abs(nb['drift']))} away, "
      f"saturated {nb['sat'] * 100:.0f} %, peak wheel {nb['wmax'] * r_w:.2f} m/s |")
    P(f"| Nominal robot left alone, 60 s | {stays(A['nominal_60'])}, ends {cm(abs(A['nominal_60']['drift']))} away | {stays(B['nominal_60'])}, ends {cm(abs(B['nominal_60']['drift']))} away |")
    P(f"| Release angle, nominal (either way) | **{A['release']['nominal_max']} deg** | **{B['release']['nominal_max']} deg** |")
    P(f"| Release angle 90 % of 60 randomised robots recover from | {half(A['release']['angles'], A['release']['frac'])} deg | {half(B['release']['angles'], B['release']['frac'])} deg |")
    P(f"| Shove at battery height, nominal | **{A['shove']['nominal_max'] * 1000:.0f} mN s** | **{B['shove']['nominal_max'] * 1000:.0f} mN s** |")
    P(f"| Shove 90 % of randomised robots take | {half(A['shove']['impulses'], A['shove']['frac']) * 1000:.0f} mN s | {half(B['shove']['impulses'], B['shove']['frac']) * 1000:.0f} mN s |")
    ra, rb = A['robust'], B['robust']
    P(f"| 300 randomised robots, 20 s, start lean up to 5 deg, two shoves of 20-50 mN s: stay up | **{pct(ra)}** | **{pct(rb)}** |")
    P(f"| ... median pitch rms; drift median (90th pct) | {ra['rms_deg']:.1f} deg; {cm(ra['drift_m'])} ({cm(ra['drift_p90'])}) | {rb['rms_deg']:.1f} deg; {cm(rb['drift_m'])} ({cm(rb['drift_p90'])}) |")
    P(f"| ... 90th-percentile peak wheel speed | {ra['wmax_p90'] * r_w:.2f} m/s | {rb['wmax_p90'] * r_w:.2f} m/s |")
    P(f"| 60 randomised robots left alone, 20 s: stay up; wobble about their own balance point | {A['still']['survived']} of 60; {A['still']['rms_deg']:.2f} deg rms | {B['still']['survived']} of 60; {B['still']['rms_deg']:.2f} deg rms |")
    P(f"| ... drift in **20 s**, median | **{cm(A['still']['drift_m'])}** | **{cm(B['still']['drift_m'])}** |")
    P(f"| Same robots, **60 s**: stay up; drift median (90th pct) | {A['still_60']['survived']} of 60; **{cm(A['still_60']['drift_m'])}** ({cm(A['still_60']['drift_p90'])}) | "
      f"{B['still_60']['survived']} of 60; **{cm(B['still_60']['drift_m'])}** ({cm(B['still_60']['drift_p90'])}) |")
    ka, kb = A.get('kickup', {}), B.get('kickup', {})
    if ka.get('params'):
        P(f"| Gets up off a crash bar by itself (same fixed kick) | {ka['frac'] * 100:.0f} % of 40; nominal {'yes' if all(ka['nominal']) else 'no'} | "
          f"{kb.get('frac', 0) * 100:.0f} % of 40; nominal {'yes' if all(kb.get('nominal', [False])) else 'no'} |")
    P()
    bs, b6 = rb['by_sensor'], B['still_60']['by_sensor']
    P('B broken down by the state of the sensor:'); P()
    P('| Sensor | 300 shoved robots: stay up | drift in 20 s, median | 60 robots left alone 60 s: stay up | drift in 60 s, median (90th pct) |'); P('|---|---|---|---|---|')
    for kind, label in (('ok', 'working'), ('flaky', 'flaky'), ('absent', 'unplugged (runs as A)')):
        P(f"| {label} | {pct(bs[kind])} | {cm(bs[kind]['drift_m'])} | {pct(b6[kind])} | {cm(b6[kind]['drift_m'])} ({cm(b6[kind]['drift_p90'])}) |")
    P(); P(f"With the sensor plugged in, the firmware trusts it a median {B['still_60']['ok_at_median']:.1f} s after it arms (nominal: {nb['ir_ok_at']:.2f} s); "
           'until then it runs exactly as A.'); P()
    P('![nominal](plot_nominal.png)'); P(); P('![limits](plot_limits.png)'); P()
    P('## The wheel sensor'); P()
    P('![sensor](plot_sensor.png)'); P()
    P('**Ring resolution** (40 robots, every sensor working, same gains, the firmware\'s edges-per-turn constant changed to match):'); P()
    P('| Windows (edges per turn) | Left alone 60 s: stay up | drift median (90th pct) | 20 s with two shoves: stay up | edges counted the wrong way |'); P('|---|---|---|---|---|')
    for wn in (8, 16, 32):
        q = S['ring'][wn]
        P(f"| {wn} ({q['edges_per_turn']}) | {pct(q['still_60'])} | {cm(q['still_60']['drift_m'])} ({cm(q['still_60']['drift_p90'])}) | {pct(q['shoved'])} | {q['wrong_frac'] * 100:.0f} % |")
    P(); P('**Where the direction comes from.** The sensor gives magnitude only. Every counted edge gets a sign from a rule (40 robots, every sensor working, 60 s left alone; '
           'and the nominal 20 s shoved run):'); P()
    P('| Sign rule | Edges counted the wrong way | ... of which within 100 ms of a sign reversal | Drift in 60 s, median (90th pct) | Nominal shoved run: ends; position estimate off by |'); P('|---|---|---|---|---|')
    for k, q in S['sign'].items():
        ns = q['nominal_shoved']
        cheat = k.startswith('true')
        P(f"| {k} | {'0 %' if cheat else '%.0f %%' % (q['wrong_frac'] * 100)} | {'-' if cheat else '%.0f %%' % (q['wrong_after_rev_frac'] * 100)} | {cm(q['still_60']['drift_m'])} ({cm(q['still_60']['drift_p90'])}) | "
          f"{'up' if ns['ok'] else 'FELL'}, {cm(abs(ns['drift']))} away; {cm(abs(ns['x_err']))} |")
    P(); P('A balancer standing still rocks over its wheel. The sensor sits on the body, so most edges at a standstill come from the body '
           'turning over a wheel that hardly moves on the floor, and they run against the motor command. The published rule (sign of the last '
           'command) therefore counts about half of them backwards and the position estimate walks off. This sketch signs each edge with the direction the '
           'wheel should be turning on the body: the motor model\'s speed minus the pitch rate. The residual error is edges near a reversal, '
           'where the model and the gearbox slop disagree.'); P()
    fs, fsn = S['fail'], S['fail_nominal']
    P('**The sensor dies mid-run.** ' + (f"Nominal robot, sensor dead at 10 s: the firmware notices at {fsn['ir_fail_at']:.2f} s, runs IMU only from then, "
      f"{stays(fsn)} through a 30 mN s shove at 20 s. " if fsn['ir_fail_at'] else f"Nominal robot, sensor dead at 10 s: the firmware never notices; {stays(fsn)}. ")
      + f"60 randomised robots with a working sensor that dies at a random time between 5 and 15 s, shoved (30 mN s) at 20 s: **{fs['survived']} of {fs['n']} stay up**; "
      f"the same robots: {fs['survived_no_fault']} when it does not die, {fs['survived_imu_only']} with no sensor at all. The firmware noticed in {fs['noticed']} of {fs['n']}"
      + (f", a median {fs['lag_median']:.2f} s (worst {fs['lag_max']:.2f} s) after it died." if fs['lag_median'] is not None else '.')
      + ' Until it notices, the sensor reads a stopped wheel and the position stands still, so it holds a stale position; the inner loop is the motor model either way.')
    fa = S['fail_after_shove']
    P(); P(f"Dead 50 ms after a 50 mN s shove (nominal): {stays(fa)}" + (f", noticed at {fa['ir_fail_at']:.2f} s." if fa['ir_fail_at'] else '.'))
    fl = S['floating']
    P(); P(f"**Unplugged with no pull-down** (floating input, about {NOM['ir_noise_hz']:.0f} edges a second of hum): the edges counted during gyro calibration "
           f"mark it as noise, so it runs as A: {stays(fl)}, sensor {['unknown', 'OK', 'FAILED'][fl['ir_status']]}, ends {cm(abs(fl['drift']))} away.")
    P(); P(f"**`wheel off`** with a working sensor plugged in: {stays(S['forced_off'])}, "
           f"{'the same run as A to the last digit' if S['forced_off_same_as_A'] else 'not identical to A (check!)'}."); P()
    P('## What limits it (milestone A)'); P()
    P('| Motor no-load speed (rpm) | ' + ' | '.join(str(x) for x in rpms) + ' |'); P('|---|' + '---|' * len(rpms))
    P('| Largest lean recovered (deg) | ' + ' | '.join(str(x) for x in sp) + ' |'); P()
    P('| Centre of mass above axle (mm) | ' + ' | '.join(str(x) for x in res['height']['com_mm']) + ' |'); P('|---|' + '---|' * len(hs))
    P('| Largest lean recovered (deg) | ' + ' | '.join(str(x) for x in hs) + ' |'); P()
    P('| Stall torque at 6 V (N m) | ' + ' | '.join(str(x) for x in sts) + ' |'); P('|---|' + '---|' * len(sts))
    P('| Largest lean recovered (deg) | ' + ' | '.join(str(x) for x in tq) + ' |'); P()
    P('![sweeps](plot_sweeps.png)'); P()
    P('## Getting up by itself'); P()
    if ka.get('params'):
        P(f"Lying on a crash bar at {MODEL['rest_deg']:.0f} deg, a fixed kick (roll away for {ka['params']['back'] * 1000:.0f} ms, then full power under the body, hand "
          f"over to the PID inside {ka['params']['handover']} deg) gets **{ka['frac'] * 100:.0f} %** of 40 randomised robots back up in A, "
          f"{kb.get('frac', 0) * 100:.0f} % in B; nominal robot: {'yes' if all(ka['nominal']) else 'no'} (A), {'yes' if all(kb.get('nominal', [False])) else 'no'} (B).")
    else:
        P('No fixed kick got it off the crash bar.')
    P()
    open(os.path.join(OUT, 'sim.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))

def video(path, sensor=False):
    import subprocess
    m = model(); g = load_gains()
    with mujoco.Renderer(m, 720, 1280) as r:
        fr = dict(r=r, out=[], every=int(round(1 / 30 / TICK)))
        rollout(SENSOR if sensor else None, g, T=20.0, init_deg=8.0, shoves=((6.0, 0.05), (12.0, -0.05)), frames=fr, trace=True)   # the nominal test run
    fps = 1 / (fr['every'] * TICK)
    enc = os.environ.get('STAGE1_VCODEC', 'h264_videotoolbox')
    cmd = ['/opt/homebrew/bin/ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', '1280x720', '-r', f'{fps:.4f}',
           '-i', '-', '-r', '30', '-c:v', enc, '-b:v', '6M', '-pix_fmt', 'yuv420p', path]
    pr = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    for f in fr['out']:
        pr.stdin.write(f.tobytes())
    pr.stdin.close(); pr.wait()
    print('wrote', path, len(fr['out']), 'frames')

if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'test'
    if cmd == 'tune':
        print(tune(sys.argv[2] if len(sys.argv) > 2 else 'ir'))
    elif cmd == 'test':
        test()
    elif cmd == 'video':
        video(sys.argv[2], '--sensor' in sys.argv[3:])
    elif cmd == 'one':
        for p in (None, SENSOR):
            print(rollout(p, None, T=10.0, init_deg=5.0, shoves=((4.0, 0.04),)))
