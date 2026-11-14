"""Tailwhip gymnasium environment (MuJoCo).

One policy, commanded like the real robot: STAND (rear up, balance, follow a speed and turn command, survive
shoves, get up off the feet or the tail) or REST (lie back down onto the tail).

The policy sees only what the robot sends over ESP-NOW (SPEC section 7, docs/software.md):
  pitch from the on-board filter, pitch rate, yaw rate, two wheel speeds from 16-window IR encoders (coarse, and
  signed from the COMMAND because the sensor can't tell direction), its last tail command, its previous action,
  battery voltage, and the command. Four steps stacked, newest first: 4 x 13 = 52 inputs.
Actions (each -1..1): left wheel, right wheel (motor voltage = u * 6 V), tail target = 85 + 65 * a deg.

Physics at 500 Hz, policy at 50 Hz. Everything SPEC section 8 asks to randomise is randomised per episode.
Nominal values are [E] estimates until host/bench.py writes host/measurements.json, which overrides them.
"""
import os, sys, json
import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'cad'))
import tailwhip as W

CTRL_DT, PHYS_DT = 0.02, 0.002
SUBSTEPS = int(round(CTRL_DT / PHYS_DT))
FRAME, STACK = 13, 4
V_MAX = 6.0                                    # [C] firmware caps the motors at 6 V average
TAIL_C, TAIL_S = 85.0, 65.0                    # tail target deg = 85 + 65 a
REST_PITCH = np.radians(-25.0)
TAIL_REST = W.tail_angle_for_foot_on_floor(-25.0)
EDGES = 32                                     # encoder edges per wheel turn (16 windows)
ENC_WIN = 0.04                                 # s, firmware speed window [C]

NOMINAL = dict(                                # [E] unless host/measurements.json says otherwise
    motor_stall_torque_nm_at_6v=0.08,          # per motor, at the wheel
    motor_no_load_rad_s_at_6v=21.0,            # ~200 rpm
    motor_deadzone_u=0.10,
    backlash_deg=5.0,
    servo_deg_per_s=500.0,                     # SG-90: 0.12 s / 60 deg loaded
    servo_stall_nm=0.13,
    loop_delay_ms=25.0,
    tyre_mu=0.8,
)

def load_measurements():
    p = os.path.join(HERE, '..', 'host', 'measurements.json')
    n = dict(NOMINAL)
    if os.path.exists(p):
        try:
            got = json.load(open(p))
            n.update({k: float(v) for k, v in got.items() if k in n and v is not None})
        except Exception:
            pass
    return n


def model_path():
    p = os.path.join(HERE, 'tailwhip.xml')
    if not os.path.exists(p):
        import tailwhip_model
        tailwhip_model.build()
    return p


class TailwhipEnv(gym.Env):
    metadata = {'render_modes': ['rgb_array'], 'render_fps': 50}

    def __init__(self, randomize=True, seed=None, render_mode=None, horizon=500, start=None, command=None,
                 shoves=True):
        self.randomize, self.render_mode, self.horizon = randomize, render_mode, horizon
        self.force_start, self.force_command, self.shoves = start, command, shoves
        self.model = mujoco.MjModel.from_xml_path(model_path())
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.bid = m.body('chassis').id
        self.j = {n: m.joint(n) for n in ('motor_l', 'motor_r', 'slop_l', 'slop_r', 'tail', 'leaf')}
        self.qa = {n: j.qposadr[0] for n, j in self.j.items()}
        self.va = {n: j.dofadr[0] for n, j in self.j.items()}
        self.gid = {n: m.geom(n).id for n in ('floor', 'foot', 'tyre_l', 'tyre_r', 'feet', 'heel', 'chin', 'torso', 'headc')}
        self.push_site = m.site('push').id
        self.nominal = dict(mass=m.body_mass.copy(), inertia=m.body_inertia.copy(), ipos=m.body_ipos.copy(),
                            friction=m.geom_friction.copy(), jrange=m.jnt_range.copy())
        self.meas = load_measurements()
        self.action_space = spaces.Box(-1, 1, (3,), np.float32)
        self.observation_space = spaces.Box(-10, 10, (FRAME * STACK,), np.float32)
        self.np_random = np.random.default_rng(seed)
        self._renderer = None
        self._set_params(nominal=not randomize)

    # ------------------------------------------------------------------------------------------ randomisation
    def _u(self, lo, hi, nominal=False):
        return (lo + hi) / 2 if nominal else float(self.np_random.uniform(lo, hi))

    def _set_params(self, nominal=False):
        m, n, M = self.model, self.nominal, self.meas
        u = lambda lo, hi: self._u(lo, hi, nominal)
        b = self.bid
        k = u(0.85, 1.15)                                           # mass +-15 %
        m.body_mass[b] = n['mass'][b] * k
        m.body_inertia[b] = n['inertia'][b] * k
        m.body_ipos[b] = n['ipos'][b] + np.array([u(-0.004, 0.004), 0, u(-0.015, 0.015)])   # CoM height +-15 mm
        mu = u(0.4, 1.0) if not nominal else M['tyre_mu']
        for g in ('tyre_l', 'tyre_r'):
            m.geom_friction[self.gid[g], 0] = mu
        m.geom_friction[self.gid['foot'], 0] = u(0.3, 0.9)
        for g in ('feet', 'heel', 'chin'):
            m.geom_friction[self.gid[g], 0] = u(0.25, 0.6)
        self.stall = M['motor_stall_torque_nm_at_6v'] * np.array([u(0.75, 1.25), u(0.75, 1.25)])
        self.w0 = M['motor_no_load_rad_s_at_6v'] * u(0.85, 1.15)
        self.deadzone = u(0.04, 0.16) if not nominal else M['motor_deadzone_u']
        bl = np.radians(u(2.0, 8.0) if not nominal else M['backlash_deg']) / 2
        for s in ('slop_l', 'slop_r'):
            m.jnt_range[self.j[s].id] = (-bl, bl)
        self.motor_tau = u(0.008, 0.025)                            # electrical + driver lag, s
        self.servo_rate = np.radians(M['servo_deg_per_s'] * u(0.8, 1.2))
        st = M['servo_stall_nm'] * u(0.8, 1.25)
        a = m.actuator('tail').id
        m.actuator_forcerange[a] = (-st, st)
        self.tail_offset = np.radians(u(-4, 4))                     # horn spline error
        delay_ms = u(10.0, 40.0) if not nominal else M['loop_delay_ms']
        self.delay = int(round(delay_ms / 1000 / PHYS_DT))          # in physics steps
        self.vbat0 = u(6.6, 8.4)
        self.pitch_bias = np.radians(u(-1.5, 1.5))
        self.gyro_bias = np.array([u(-0.03, 0.03), u(-0.03, 0.03)])
        self.enc_present = (self.np_random.random() > 0.1) if not nominal else True
        self.floor_tilt = np.radians(u(-1.5, 1.5))
        m.opt.gravity[:] = 9.81 * np.array([np.sin(self.floor_tilt), 0, -np.cos(self.floor_tilt)])

    # ------------------------------------------------------------------------------------------ state helpers
    def _tilt(self):
        """Body z axis expressed against the heading frame (body x and y axes flattened onto the floor)."""
        R = self.data.xmat[self.bid].reshape(3, 3)
        z = R[:, 2]
        fx = np.array([R[0, 0], R[1, 0], 0.0]); fx /= np.linalg.norm(fx) + 1e-9
        fy = np.array([-fx[1], fx[0], 0.0])
        return z @ fx, z @ fy, z[2]

    def pitch(self):
        a, b, c = self._tilt()
        return float(np.arctan2(a, c))

    def roll(self):
        a, b, c = self._tilt()
        return float(np.arctan2(-b, c))

    def body_rates(self):
        """Angular velocity in the body frame: (roll, pitch, yaw) rates."""
        w = self.data.cvel[self.bid][:3]          # world frame (com-based spatial velocity, rotational part)
        R = self.data.xmat[self.bid].reshape(3, 3)
        return R.T @ w

    def wheel_speed(self):
        d, va = self.data, self.va
        return np.array([d.qvel[va['motor_l']] + d.qvel[va['slop_l']], d.qvel[va['motor_r']] + d.qvel[va['slop_r']]])

    def forward_speed(self):
        R = self.data.xmat[self.bid].reshape(3, 3)
        fwd = np.array([R[0, 0], R[1, 0], 0.0]); fwd /= np.linalg.norm(fwd) + 1e-9
        return float(self.data.qvel[0:3] @ fwd)

    def contact(self, g):
        gid = self.gid[g]
        for c in self.data.contact[:self.data.ncon]:
            if gid in (c.geom1, c.geom2) and self.gid['floor'] in (c.geom1, c.geom2):
                return True
        return False

    def head_height(self):
        return float(self.data.site_xpos[self.model.site('headtop').id][2])

    # ------------------------------------------------------------------------------------------ actuators
    def _apply(self, act):
        """One physics step's worth of actuation from the (already delayed) action."""
        d, m = self.data, self.model
        u = np.clip(act[:2], -1, 1)
        v_cmd = np.where(np.abs(u) < self.deadzone, 0.0, u) * min(V_MAX, self.vbat)
        self.v_motor += (v_cmd - self.v_motor) * min(1.0, PHYS_DT / self.motor_tau)
        w = np.array([d.qvel[self.va['motor_l']], d.qvel[self.va['motor_r']]])
        tau = self.stall * (self.v_motor / 6.0) - self.stall / self.w0 * w     # DC motor, slow-decay H-bridge
        d.qfrc_applied[self.va['motor_l']] = tau[0]
        d.qfrc_applied[self.va['motor_r']] = tau[1]
        self.motor_current = np.abs(tau) / self.stall * 1.5                     # ~1.5 A stall [E]
        tgt = np.radians(TAIL_C + TAIL_S * np.clip(act[2], -1, 1)) + self.tail_offset
        step = self.servo_rate * PHYS_DT
        self.servo_pos += np.clip(tgt - self.servo_pos, -step, step)
        d.ctrl[m.actuator('tail').id] = np.clip(self.servo_pos, 0.31, 2.69)

    # ------------------------------------------------------------------------------------------ observation
    def _frame(self, cmd):
        nz = (lambda s: self.np_random.normal(0, s)) if self.randomize else (lambda s: 0.0)
        rates = self.body_rates()
        pitch = self.pitch() + self.pitch_bias * self.randomize + nz(np.radians(0.3))
        prate = rates[1] + self.gyro_bias[0] * self.randomize + nz(0.02)
        yrate = rates[2] + self.gyro_bias[1] * self.randomize + nz(0.02)
        ws = self.wheel_speed()
        q = 2 * np.pi / EDGES / ENC_WIN
        meas = np.floor(np.abs(ws) / q + (self.np_random.random(2) if self.randomize else 0.5)) * q
        sign = np.where(self.last_action[:2] >= 0, 1.0, -1.0)
        wl, wr = (meas * sign) if self.enc_present else (0.0, 0.0)
        vb = self.vbat + nz(0.02)
        return np.array([pitch, prate / 5, yrate / 5, wl / 20, wr / 20, self.last_action[2],
                         *self.prev_action, (vb - 7.6) / 0.8, cmd[0], cmd[1] / 0.3, cmd[2] / 2], np.float32)

    def _obs(self):
        return np.concatenate(self.frames).astype(np.float32)

    # ------------------------------------------------------------------------------------------ reset
    def _place(self, pitch, tail_deg, pitch_rate=0.0, yaw=0.0):
        d = self.data
        mujoco.mj_resetData(self.model, d)
        qp = np.array([np.cos(pitch / 2), 0, np.sin(pitch / 2), 0])
        qy = np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
        w1, x1, y1, z1 = qy; w2, x2, y2, z2 = qp
        d.qpos[3:7] = [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                       w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]
        d.qpos[2] = W.R_WHEEL / 1000 + 0.0005
        t = np.radians(tail_deg)
        d.qpos[self.qa['tail']] = t
        d.qvel[4] = pitch_rate
        self.servo_pos = t
        d.ctrl[self.model.actuator('tail').id] = t
        mujoco.mj_forward(self.model, d)

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.np_random = np.random.default_rng(seed)
        self._set_params(nominal=not self.randomize)
        r = self.np_random
        kind = self.force_start or r.choice(['rest', 'upright', 'shove', 'feet', 'lie'], p=[0.35, 0.2, 0.15, 0.15, 0.15])
        self.kind = kind
        yaw = r.uniform(-np.pi, np.pi)
        if kind == 'rest':
            self._place(REST_PITCH + r.uniform(-0.05, 0.05), TAIL_REST, yaw=yaw)
        elif kind in ('upright', 'lie'):
            self._place(r.uniform(-0.05, 0.05), TAIL_REST + r.uniform(-10, 10), yaw=yaw)
        elif kind == 'shove':
            self._place(r.uniform(-0.05, 0.05), TAIL_REST, pitch_rate=r.choice([-1, 1]) * r.uniform(1.0, 3.0), yaw=yaw)
        elif kind == 'feet':
            self._place(np.radians(18.0), TAIL_REST, yaw=yaw)
        self.v_motor = np.zeros(2); self.motor_current = np.zeros(2)
        self.vbat = self.vbat0
        tail_a = (TAIL_REST - TAIL_C) / TAIL_S
        self.last_action = np.array([0.0, 0.0, tail_a]); self.prev_action = self.last_action.copy()
        self.queue = [self.last_action.copy()] * (self.delay + 1)
        # settle 0.2 s with the motors off
        for _ in range(100):
            self._apply(self.last_action); mujoco.mj_step(self.model, self.data)
        self.cmd = self._new_command(kind)
        if self.force_command is not None:
            self.cmd = np.array(self.force_command, float)
        self.t, self.k = 0.0, 0
        self.next_shove = r.uniform(1.5, 4.0) if (self.shoves and self.randomize) else 1e9
        self.shove_end, self.shove_f = -1.0, np.zeros(3)
        self.stats = dict(up_time=0.0, max_leaf_rate=0.0, falls=0, foot_time=0.0)
        f = self._frame(self.cmd)
        self.frames = [f.copy() for _ in range(STACK)]
        return self._obs(), {}

    def _new_command(self, kind):
        r = self.np_random
        if kind == 'lie':
            return np.array([0.0, 0.0, 0.0])
        v = r.uniform(-0.25, 0.25) if r.random() < 0.5 else 0.0
        w = r.uniform(-1.5, 1.5) if r.random() < 0.3 else 0.0
        return np.array([1.0, v, w])

    # ------------------------------------------------------------------------------------------ step
    def step(self, action):
        a = np.clip(np.asarray(action, float), -1, 1)
        d, m = self.data, self.model
        self.prev_action = self.last_action.copy()
        self.last_action = a.copy()
        leaf_rate = 0.0
        for i in range(SUBSTEPS):
            self.queue.append(a)
            act = self.queue.pop(0)
            if self.t >= self.next_shove:                                  # a shove: 0.1 s horizontal push
                ang = self.np_random.uniform(0, 2 * np.pi)
                mag = self.np_random.uniform(0.5, 2.5)
                self.shove_f = mag * np.array([np.cos(ang), np.sin(ang) * 0.3, 0.0])
                self.shove_end = self.t + 0.1
                self.next_shove = self.t + self.np_random.uniform(2.0, 4.5)
            d.xfrc_applied[self.bid, :3] = self.shove_f if self.t < self.shove_end else 0.0
            self._apply(act)
            mujoco.mj_step(m, d)
            self.t += PHYS_DT
            leaf_rate = max(leaf_rate, abs(d.qvel[self.va['leaf']]))
        self.k += 1
        # battery sags with current [E]
        self.vbat = self.vbat0 - 0.25 * float(self.motor_current.sum()) - 0.0001 * self.k
        if self.randomize and self.np_random.random() < 0.004:             # occasionally change the command
            self.cmd = self._new_command('lie' if self.np_random.random() < 0.1 else 'stand')
        f = self._frame(self.cmd)
        self.frames = [f] + self.frames[:-1]
        pitch, roll = self.pitch(), self.roll()
        rates = self.body_rates()
        foot = self.contact('foot')
        up = np.exp(-(pitch / 0.25) ** 2)
        effort = float(np.sum(a[:2] ** 2))
        da = float(np.sum((a - self.prev_action) ** 2))
        if self.cmd[0] > 0.5:
            v = self.forward_speed()
            r = (1.0 * up
                 + 0.5 * up * np.exp(-((v - self.cmd[1]) / 0.12) ** 2)
                 + 0.3 * up * np.exp(-((rates[2] - self.cmd[2]) / 0.6) ** 2)
                 - 0.25 * up * float(foot)                                  # lift the tail once it is up
                 - 0.04 * rates[1] ** 2 * up)
        else:
            near = np.exp(-((pitch - REST_PITCH) / 0.12) ** 2)
            r = 1.0 * near + 0.5 * near * float(foot) - 0.05 * abs(rates[1]) - 0.3 * effort * near
        r -= 0.03 * effort + 0.15 * da
        r -= 0.002 * max(0.0, leaf_rate - 8.0)                              # hard landings on the tail
        tail_deg = TAIL_C + TAIL_S * a[2]
        r -= 0.3 * max(0.0, tail_deg - 100.0) / 50.0                        # don't stow the tail: it is the back bumper
        self.stats['up_time'] += CTRL_DT * float(abs(pitch) < np.radians(10))
        self.stats['foot_time'] += CTRL_DT * float(foot)
        self.stats['max_leaf_rate'] = max(self.stats['max_leaf_rate'], leaf_rate)
        fell = abs(pitch) > np.radians(65) or abs(roll) > np.radians(40)
        if fell:
            r -= 30.0
        trunc = self.k >= self.horizon
        info = dict(pitch=pitch, up=float(abs(pitch) < np.radians(10)), kind=self.kind, cmd=self.cmd.copy(),
                    v=self.forward_speed(), yaw_rate=float(rates[2]), fell=fell, **self.stats)
        return self._obs(), float(r), bool(fell), bool(trunc), info

    # ------------------------------------------------------------------------------------------ render
    def render(self, cam_pos=None):
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, 480, 854)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = self.data.qpos[:3] + np.array([0, 0, 0.08])
        cam.distance, cam.azimuth, cam.elevation = 0.62, -62, -10
        self._renderer.update_scene(self.data, camera=cam)
        return self._renderer.render()
