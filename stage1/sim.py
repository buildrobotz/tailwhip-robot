"""Stage 1 balancer in MuJoCo: does one TT motor keep it upright with a PID loop?

The model is generated from stage1.py's numbers (out/model.json). The motor, gearbox slop, dead zone, lag and
tyre contact are the published Tailwhip simulator's models with the same [E] estimates. The controller is the
firmware's, line for line: MPU-6050 accelerometer + gyro -> complementary filter -> PID at 400 Hz -> one motor.
There are no wheel sensors; speed comes from a model of the motor, as in the firmware.

  python sim.py tune      random search for PID gains over randomised robots  -> out/gains.json
  python sim.py test      all the experiments                                -> out/results.json, out/sim.md, out/plot_*.png
  python sim.py video P   a clip of the nominal robot                        -> P (mp4)
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
NOM = dict(mass=1.0, com_dz=0.0, com_dx=0.0, mu=0.8, stall=0.08, w0=21.0, deadzone=0.10, backlash_deg=5.0,
           motor_tau=0.015, vbat=7.6, pitch_bias_deg=0.0, gyro_bias=0.0, floor_tilt_deg=0.0, seed=0)
GAINS0 = dict(kp=6.0, kd=0.6, ki=0.0, kv=3.0, kx=0.5, dz=0.10)

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

class Controller:
    """The firmware's filter and PID (firmware/stage1_balance/stage1_balance.ino), same constants."""
    FILTER_TAU, ACC_GATE, V_EST_TAU, W_NOLOAD = 0.5, 0.25, 0.08, 26.2
    def __init__(self, g):
        self.g = dict(GAINS0, **g); self.pitch = None; self.x = 0.0; self.integ = 0.0; self.v = 0.0; self.u_prev = 0.0
    def step(self, acc, gyr):
        g = self.g
        an = np.linalg.norm(acc) / 9.81
        pa = np.arctan2(-acc[0], acc[2])
        if self.pitch is None:
            self.pitch = pa
        self.pitch += gyr[1] * TICK
        if abs(an - 1.0) < self.ACC_GATE:
            self.pitch += (pa - self.pitch) * TICK / self.FILTER_TAU
        th = self.pitch
        self.v += (self.u_prev * self.W_NOLOAD * MODEL['r_wheel'] - self.v) * TICK / self.V_EST_TAU
        self.integ = np.clip(self.integ + th * TICK, -0.2, 0.2)
        self.x = np.clip(self.x + self.v * TICK, -0.5, 0.5)
        u = g['kp'] * th + g['kd'] * gyr[1] + g['ki'] * self.integ + g['kv'] * self.v + g['kx'] * self.x
        u = float(np.clip(u, -1, 1)); self.u_prev = u
        if abs(u) > 0.005:
            u += np.sign(u) * g['dz']
        return float(np.clip(u, -1, 1))

_M = None; _NOM = None
def model():
    global _M, _NOM
    if _M is None:
        _M = mujoco.MjModel.from_xml_string(xml())
        _NOM = dict(mass=_M.body_mass.copy(), inertia=_M.body_inertia.copy(), ipos=_M.body_ipos.copy())
    return _M

def randomised(seed):
    r = np.random.default_rng(seed); u = r.uniform
    return dict(mass=u(0.85, 1.15), com_dz=u(-0.015, 0.015), com_dx=u(-0.004, 0.004), mu=u(0.4, 1.0), stall=0.08 * u(0.75, 1.25),
                w0=21.0 * u(0.85, 1.15), deadzone=u(0.04, 0.16), backlash_deg=u(2.0, 8.0), motor_tau=u(0.008, 0.025),
                vbat=u(6.6, 8.4), pitch_bias_deg=u(-1.5, 1.5), gyro_bias=u(-0.01, 0.01), floor_tilt_deg=u(-1.5, 1.5), seed=int(seed))

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
    ctl = Controller(gains or load_gains()); v_motor = 0.0; u = 0.0
    bias = np.radians(p['pitch_bias_deg']); cb, sb = np.cos(bias), np.sin(bias)
    ctl.pitch = th0 + bias          # the filter starts from the accelerometer while the robot is held still
    prev = None; N = int(round(T / TICK)); fell_at = None
    log = dict(t=[], pitch=[], est=[], u=[], w=[], x=[]) if trace else None
    sat = 0; wmax = 0.0; p2 = 0.0; p1 = 0.0; pmax = 0.0; du = 0.0; phase = 'kick' if kick else 'pid'; t_kick = 0.0
    push = m.site('push').id
    for k in range(N):
        t = k * TICK
        acc = d.sensordata[0:3] + rng.normal(0, 0.15, 3); gyr = d.sensordata[3:6] + rng.normal(0, 0.01, 3) + p['gyro_bias']
        acc = np.array([cb * acc[0] - sb * acc[2], acc[1], sb * acc[0] + cb * acc[2]])        # IMU mounted a little off
        meas = (acc, gyr)
        if prev is None:
            prev = meas
        u_pid = ctl.step(*prev)                                                              # one tick of sensor delay
        prev = meas
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
        for (ts, J) in shoves:
            if ts <= t < ts + 0.05:
                F = np.array([J / 0.05, 0, 0]); d.xfrc_applied[b, :3] = F
                d.xfrc_applied[b, 3:] = np.cross(d.site_xpos[push] - d.xipos[b], F)
            elif abs(t - ts - 0.05) < TICK / 2:
                d.xfrc_applied[b] = 0
        for _ in range(SUB):
            v_cmd = (0.0 if abs(u) < p['deadzone'] else u) * min(V_MAX, p['vbat'])
            v_motor += (v_cmd - v_motor) * min(1.0, (TICK / SUB) / p['motor_tau'])
            d.qfrc_applied[va] = p['stall'] * (v_motor / 6.0) - p['stall'] / p['w0'] * d.qvel[va]
            mujoco.mj_step(m, d)
        w = d.qvel[va] + d.qvel[vs]
        if phase == 'pid' and (kick is None or t > t_kick + 0.3):
            sat += abs(u) >= 0.999; wmax = max(wmax, abs(w)); p2 += pitch ** 2; p1 += pitch; pmax = max(pmax, abs(pitch))
        if trace:
            for key, val in zip(('t', 'pitch', 'est', 'u', 'w', 'x'), (t, np.degrees(pitch), np.degrees(ctl.pitch), u, w, d.qpos[0])):
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
               sat=float(sat / N), wmax=float(wmax), du=float(np.sqrt(du / N)))
    if trace:
        out['log'] = log
    return out

def load_gains():
    p = os.path.join(OUT, 'gains.json')
    return json.load(open(p)) if os.path.exists(p) else dict(GAINS0)

# ------------------------------------------------------------------------------------------------ tuning
def _score(g):
    """(robots that survive a 5 deg release and two 30 mN s shoves, mean wobble standing still, mean saturation)."""
    ok = 0; wob = []; sat = []
    for s in range(100, 112):
        sg = 1 if s % 2 else -1
        ok += rollout(randomised(s), g, T=7.0, init_deg=5.0 * sg, shoves=((3.0, 0.03 * sg), (5.0, -0.03 * sg)))['ok']
        q = rollout(randomised(s), g, T=5.0)
        wob.append(q['wobble_deg'] if q['ok'] else 9.0); sat.append(q['sat'])
    return ok, float(np.mean(wob)), float(np.mean(sat))

def sample(rng, centre=None, spread=1.0):
    if centre is None:
        return dict(kp=10 ** rng.uniform(-0.3, 1.3), kd=10 ** rng.uniform(-1.5, 0.4), ki=0.0, kv=10 ** rng.uniform(-0.5, 1.0),
                    kx=10 ** rng.uniform(-1.0, 0.5), dz=0.10)
    return dict({k: centre[k] * 10 ** rng.normal(0, 0.12 * spread) for k in ('kp', 'kd', 'kv', 'kx')}, ki=0.0, dz=centre.get('dz', 0.10))

def tune():
    """Random search. Keep the gains that save the most randomised robots; among those, the quietest when standing
    still. A bang-bang controller survives shoves too, but it hammers the gearbox, so quiet wins ties."""
    rng = np.random.default_rng(5)
    with Pool() as pool:
        cands = [dict(GAINS0)] + [sample(rng) for _ in range(900)]
        sc = pool.map(_score, cands)
        key = lambda t: (-t[1][0], t[1][1])
        best, bs = sorted(zip(cands, sc), key=key)[0]; print('global', bs, best)
        cands = [best] + [sample(rng, best, 0.6) for _ in range(200)]
        sc = pool.map(_score, cands)
        best, bs = sorted(zip(cands, sc), key=key)[0]; print('refine', bs, best)
    best = {k: float(round(v, 2)) for k, v in best.items()}
    json.dump(best, open(os.path.join(OUT, 'gains.json'), 'w'), indent=1)
    return best

# ------------------------------------------------------------------------------------------------ experiments
def _run(a):
    return rollout(**a)

def frac(pool, jobs):
    return float(np.mean([r['ok'] for r in pool.map(_run, jobs)]))

def _best_lean(a):
    """Largest release angle (deg, 2 deg steps, both directions must recover) for one robot variant, with a small
    gain search of its own so every variant gets a fair controller."""
    p, base, n = a; rng = np.random.default_rng(7); best = 0
    for g in [base] + [sample(rng, base, 2.5) for _ in range(n)]:
        ang = 0
        for th in range(2, 25, 2):
            if all(rollout(p, g, T=4.0, init_deg=s * th)['ok'] for s in (1, -1)):
                ang = th
            else:
                break
        best = max(best, ang)
    return best

def test():
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    g = load_gains(); res = dict(gains=g); L = []
    P = lambda s='': L.append(s)
    with Pool() as pool:
        # 1. nominal run
        r = rollout(None, g, T=20.0, init_deg=8.0, shoves=((6.0, 0.05), (12.0, -0.05)), trace=True)
        lg = r.pop('log'); res['nominal'] = r
        fig, ax = plt.subplots(4, 1, figsize=(9, 8.5), sharex=True)
        ax[0].plot(lg['t'], lg['pitch'], label='true'); ax[0].plot(lg['t'], lg['est'], lw=0.8, label='filter'); ax[0].set_ylabel('pitch (deg)'); ax[0].legend(loc='upper right')
        ax[1].plot(lg['t'], lg['u']); ax[1].set_ylabel('motor command (-1..1)'); ax[1].set_ylim(-1.1, 1.1)
        ax[2].plot(lg['t'], np.array(lg['w']) * MODEL['r_wheel']); ax[2].axhline(0.68, ls=':', c='grey'); ax[2].axhline(-0.68, ls=':', c='grey'); ax[2].set_ylabel('wheel rim speed (m/s)')
        ax[3].plot(lg['t'], lg['x']); ax[3].set_ylabel('position (m)'); ax[3].set_xlabel('time (s)')
        for a in ax:
            a.axvline(6, c='r', lw=0.6, ls='--'); a.axvline(12, c='r', lw=0.6, ls='--'); a.grid(alpha=0.3)
        ax[0].set_title('Nominal robot: released at 8 deg, shoved at 6 s and 12 s (0.05 N s). Dotted: the motor\'s no-load speed.')
        fig.tight_layout(); fig.savefig(os.path.join(OUT, 'plot_nominal.png'), dpi=110); plt.close(fig)
        # 2. release angle
        seeds = list(range(500, 560)); angs = list(range(1, 23))
        rel = [frac(pool, [dict(p=randomised(s), gains=g, T=5.0, init_deg=th * (1 if s % 2 else -1)) for s in seeds]) for th in angs]
        rel_nom = max([th for th in angs if all(rollout(None, g, T=5.0, init_deg=sg * th)['ok'] for sg in (1, -1))] or [0])
        res['release'] = dict(angles=angs, frac=rel, nominal_max=rel_nom)
        # 3. shove
        imps = [round(x, 3) for x in np.arange(0.01, 0.161, 0.01)]
        sh = [frac(pool, [dict(p=randomised(s), gains=g, T=6.0, shoves=((1.5, J * (1 if s % 2 else -1)),)) for s in seeds]) for J in imps]
        sh_nom = max([J for J in imps if all(rollout(None, g, T=6.0, shoves=((1.5, sg * J),))['ok'] for sg in (1, -1))] or [0])
        res['shove'] = dict(impulses=imps, frac=sh, nominal_max=sh_nom)
        fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
        ax[0].plot(angs, np.array(rel) * 100, 'o-'); ax[0].set_xlabel('released at this lean (deg)'); ax[0].set_ylabel('recovered (% of 60 randomised robots)')
        ax[1].plot(np.array(imps) * 1000, np.array(sh) * 100, 'o-'); ax[1].set_xlabel('shove at battery height (mN s)')
        for a in ax:
            a.grid(alpha=0.3); a.set_ylim(-3, 103)
        fig.tight_layout(); fig.savefig(os.path.join(OUT, 'plot_limits.png'), dpi=110); plt.close(fig)
        # 4. robustness
        rng = np.random.default_rng(3); jobs = []
        for s in range(1000, 1300):
            a, b_ = rng.uniform(0.02, 0.05, 2) * rng.choice([-1, 1], 2)
            jobs.append(dict(p=randomised(s), gains=g, T=20.0, init_deg=float(rng.uniform(-5, 5)), shoves=((6.0, float(a)), (12.0, float(b_)))))
        rr = pool.map(_run, jobs)
        ok = [x for x in rr if x['ok']]
        res['robust'] = dict(n=len(rr), survived=len(ok), rms_deg=float(np.median([x['rms_deg'] for x in ok])),
                             drift_m=float(np.median([abs(x['drift']) for x in ok])), drift_p90=float(np.percentile([abs(x['drift']) for x in ok], 90)),
                             sat=float(np.median([x['sat'] for x in ok])), wmax_p90=float(np.percentile([x['wmax'] for x in ok], 90)),
                             yaw_p90=float(np.percentile([abs(x['yaw_deg']) for x in ok], 90)))
        fails = [j['p'] for j, x in zip(jobs, rr) if not x['ok']]
        if fails:
            res['robust']['failed_mean'] = {k: float(np.mean([f[k] for f in fails])) for k in ('mu', 'stall', 'w0', 'backlash_deg', 'motor_tau', 'deadzone', 'vbat', 'mass', 'com_dz')}
        # still, no shoves: how quiet is it
        st = pool.map(_run, [dict(p=randomised(s), gains=g, T=20.0) for s in range(2000, 2060)])
        res['still'] = dict(survived=int(sum(x['ok'] for x in st)), n=len(st), rms_deg=float(np.median([x['wobble_deg'] for x in st])),
                            drift_m=float(np.median([abs(x['drift']) for x in st])))
        # 5. how fast does the motor need to be (same stall torque)
        rpms = [30, 60, 90, 120, 160, 200, 250, 300]
        sp = pool.map(_best_lean, [(dict(w0=rpm * 2 * np.pi / 60), g, 40) for rpm in rpms])
        res['speed'] = dict(rpm=rpms, lean=sp)
        # 6. battery height
        dzs = [-0.06, -0.04, -0.02, 0.0, 0.02, 0.04, 0.06]
        hs = pool.map(_best_lean, [(dict(com_dz=dz), g, 40) for dz in dzs])
        h0 = MODEL['body_com'][2]
        res['height'] = dict(com_mm=[round((h0 + dz) * 1000) for dz in dzs], lean=hs)
        # 7. motor torque
        sts = [0.03, 0.04, 0.06, 0.08, 0.10, 0.12, 0.16]
        tq = pool.map(_best_lean, [(dict(stall=s_), g, 40) for s_ in sts])
        res['torque'] = dict(stall=sts, lean=tq)
        fig, ax = plt.subplots(1, 3, figsize=(13, 3.6))
        ax[0].plot(rpms, sp, 'o-'); ax[0].axvline(200, c='r', ls='--', lw=0.8); ax[0].set_xlabel('motor no-load speed (rpm at 6 V)'); ax[0].set_ylabel('largest lean recovered (deg)')
        ax[1].plot(res['height']['com_mm'], hs, 'o-'); ax[1].axvline(round(h0 * 1000), c='r', ls='--', lw=0.8); ax[1].set_xlabel('centre of mass above the axle (mm)')
        ax[2].plot(sts, tq, 'o-'); ax[2].axvline(0.08, c='r', ls='--', lw=0.8); ax[2].set_xlabel('motor stall torque at 6 V (N m)')
        for a in ax:
            a.grid(alpha=0.3); a.set_ylim(0, 25)
        fig.suptitle('What limits it (red: this design). Each point has its own gain search.', y=1.02)
        fig.tight_layout(); fig.savefig(os.path.join(OUT, 'plot_sweeps.png'), dpi=110, bbox_inches='tight'); plt.close(fig)
        # 8. getting up off a crash bar by itself
        best = (0, None)
        for back in (0.0, 0.08, 0.12, 0.16, 0.2, 0.25):
            for hand in (4, 6, 8, 10, 12, 15):
                kk = dict(settle=0.6, back=back, handover=hand)
                f = frac(pool, [dict(p=randomised(s), gains=g, T=5.0, kick=dict(kk, side=1 if s % 2 else -1)) for s in range(3000, 3040)])
                if f > best[0]:
                    best = (f, kk)
        res['kickup'] = dict(frac=best[0], params=best[1])
        if best[1]:
            nomk = [rollout(None, g, T=5.0, kick=dict(best[1], side=sd))['ok'] for sd in (1, -1)]
            res['kickup']['nominal'] = nomk
    json.dump(res, open(os.path.join(OUT, 'results.json'), 'w'), indent=1)
    rb, n0 = res['robust'], res['nominal']
    P('# Stage 1 balancer: simulation results'); P()
    P('Generated by `sim.py test`. MuJoCo, 800 Hz physics, the firmware\'s filter and PID at 400 Hz, IMU only. Motor and '
      'gearbox figures are estimates until the bench measurements exist.'); P()
    P(f"Gains (u per rad, per rad/s, per m/s, per m): kp {g['kp']}, kd {g['kd']}, kv {g['kv']}, kx {g['kx']}, ki 0, dead-zone compensation {g['dz']}."); P()
    P('## Does it stay up'); P()
    P(f"- **Nominal robot, 20 s, released at 8 deg and shoved twice (0.05 N s):** {'stays up' if n0['ok'] else 'FALLS at %.1f s' % n0['fell_at']}. "
      f"Pitch rms {n0['rms_deg']:.1f} deg, ends {abs(n0['drift']) * 100:.0f} cm from where it started, motor saturated {n0['sat'] * 100:.0f} % of the time, "
      f"peak wheel speed {n0['wmax'] * MODEL['r_wheel']:.2f} m/s of 0.68 available.")
    P(f"- **{rb['n']} randomised robots, 20 s, random start lean up to 5 deg, two shoves of 0.02-0.05 N s:** **{rb['survived']} stay up "
      f"({100 * rb['survived'] / rb['n']:.0f} %)**. Median pitch rms {rb['rms_deg']:.1f} deg, median drift {rb['drift_m'] * 100:.0f} cm "
      f"(90th percentile {rb['drift_p90'] * 100:.0f} cm), 90th-percentile peak wheel speed {rb['wmax_p90'] * MODEL['r_wheel']:.2f} m/s.")
    if 'failed_mean' in rb:
        P('  - The ones that fell, on average: ' + ', '.join(f'{k} {v:.3g}' for k, v in rb['failed_mean'].items()) + '.')
    P(f"- **Left alone (no shoves), {res['still']['n']} randomised robots, 20 s:** {res['still']['survived']} stay up; median wobble about its own balance point "
      f"{res['still']['rms_deg']:.2f} deg rms, median drift {res['still']['drift_m'] * 100:.0f} cm."); P()
    P('![nominal](plot_nominal.png)'); P()
    P('## How much it can take'); P()
    half = lambda xs, fs: max([x for x, f in zip(xs, fs) if f >= 0.9] or [0])
    P(f"- **Release angle:** the nominal robot recovers from **{res['release']['nominal_max']} deg** either way; 90 % of randomised robots recover from "
      f"{half(angs, rel)} deg.")
    P(f"- **Shove at battery height:** nominal **{res['shove']['nominal_max'] * 1000:.0f} mN s**; 90 % of randomised robots take "
      f"{half(imps, sh) * 1000:.0f} mN s. For scale, 50 mN s is a 50 g ball arriving at 1 m/s."); P()
    P('![limits](plot_limits.png)'); P()
    P('## What limits it'); P()
    P('| Motor no-load speed (rpm) | ' + ' | '.join(str(x) for x in rpms) + ' |'); P('|---|' + '---|' * len(rpms))
    P('| Largest lean recovered (deg) | ' + ' | '.join(str(x) for x in sp) + ' |'); P()
    P('| Centre of mass above axle (mm) | ' + ' | '.join(str(x) for x in res['height']['com_mm']) + ' |'); P('|---|' + '---|' * len(dzs))
    P('| Largest lean recovered (deg) | ' + ' | '.join(str(x) for x in hs) + ' |'); P()
    P('| Stall torque at 6 V (N m) | ' + ' | '.join(str(x) for x in sts) + ' |'); P('|---|' + '---|' * len(sts))
    P('| Largest lean recovered (deg) | ' + ' | '.join(str(x) for x in tq) + ' |'); P()
    P('![sweeps](plot_sweeps.png)'); P()
    P('## Getting up by itself'); P()
    k = res['kickup']
    P(f"Lying on a crash bar at {MODEL['rest_deg']:.0f} deg, a fixed kick (roll away for {k['params']['back'] * 1000:.0f} ms, then full power under the body, hand "
      f"over to the PID inside {k['params']['handover']} deg) gets **{k['frac'] * 100:.0f} %** of 40 randomised robots back up; nominal robot: "
      f"{'yes' if all(k.get('nominal', [False])) else 'no' if not any(k.get('nominal', [False])) else 'one side only'}." if k['params'] else
      'No fixed kick got it off the crash bar.'); P()
    open(os.path.join(OUT, 'sim.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))

def video(path):
    import subprocess
    m = model(); g = load_gains()
    with mujoco.Renderer(m, 720, 1280) as r:
        fr = dict(r=r, out=[], every=int(round(1 / 30 / TICK)))
        rollout(None, g, T=20.0, init_deg=8.0, shoves=((6.0, 0.05), (12.0, -0.05)), frames=fr, trace=True)   # the nominal test run
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
        print(tune())
    elif cmd == 'test':
        test()
    elif cmd == 'video':
        video(sys.argv[2])
    elif cmd == 'one':
        r = rollout(None, None, T=10.0, init_deg=5.0, shoves=((4.0, 0.04),))
        print(r)
