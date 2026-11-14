"""Score the trained policy over many randomised robots (SPEC section 11, milestone 3).

    python sim/evaluate.py                 # 50 robots per test, writes sim/eval.json and prints a table
    python sim/evaluate.py --n 200

Each test uses a fresh randomised robot (mass, CoM height, tyre friction, motor strength, dead zone, backlash,
delay, IMU bias and noise, battery, servo speed and torque, floor tilt) and the full observation flaws.
  rear_up      start resting on the tail, command STAND: success = upright (|pitch| < 10 deg) within 4 s and
               still upright at 10 s
  balance      start upright, random shoves every 2-4.5 s: success = never fell, upright >= 90 % of the time
  from_feet    start leaning on the front feet: success = upright within 4 s and at 10 s
  shove        start upright with a 1-3 rad/s pitch kick: success = not fallen and upright at 10 s
  lie_down     start upright, command REST: success = resting on the tail (pitch -20..-32 deg, foot down) at 10 s
  drive        start upright, command 0.2 m/s: mean forward speed over the last 5 s
  pid_baseline the shove and balance tests with a hand-written PID like the firmware's pid mode
"""
import argparse, json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from tailwhip_env import TailwhipEnv, CTRL_DT


def run(policy, start, seed, command=None, shoves=False, horizon=500):
    env = TailwhipEnv(randomize=True, seed=seed, start=start, command=command, shoves=shoves, horizon=horizon)
    obs, _ = env.reset()
    pitches, foot, speeds, first_up = [], [], [], None
    done = False
    while not done:
        a = policy(obs, env)
        obs, r, term, trunc, info = env.step(a)
        p = np.degrees(info['pitch']); pitches.append(p); foot.append(env.contact('foot')); speeds.append(info['v'])
        if first_up is None and abs(p) < 10:
            first_up = env.k * CTRL_DT
        done = term or trunc
    return dict(fell=info['fell'], pitches=np.array(pitches), foot=np.array(foot), speeds=np.array(speeds), first_up=first_up)


def score(policy, n, seed0=5000):
    out = {}
    def rate(xs): return float(np.mean(xs))
    r = [run(policy, 'rest', seed0 + i) for i in range(n)]
    out['rear_up'] = rate([(x['first_up'] is not None and x['first_up'] <= 4 and abs(x['pitches'][-1]) < 10 and not x['fell']) for x in r])
    out['rear_up_time_s'] = float(np.median([x['first_up'] for x in r if x['first_up'] is not None] or [np.nan]))
    r = [run(policy, 'upright', seed0 + 100 + i, shoves=True) for i in range(n)]
    out['balance'] = rate([(not x['fell'] and np.mean(np.abs(x['pitches']) < 10) >= 0.9) for x in r])
    r = [run(policy, 'feet', seed0 + 200 + i) for i in range(n)]
    out['from_feet'] = rate([(x['first_up'] is not None and x['first_up'] <= 4 and abs(x['pitches'][-1]) < 10 and not x['fell']) for x in r])
    r = [run(policy, 'shove', seed0 + 300 + i) for i in range(n)]
    out['shove'] = rate([(not x['fell'] and abs(x['pitches'][-1]) < 10) for x in r])
    r = [run(policy, 'lie', seed0 + 400 + i, command=[0, 0, 0]) for i in range(n)]
    out['lie_down'] = rate([(-32 <= x['pitches'][-1] <= -20 and x['foot'][-25:].mean() > 0.8) for x in r])
    r = [run(policy, 'upright', seed0 + 500 + i, command=[1, 0.2, 0]) for i in range(n)]
    out['drive_0.2_mps'] = float(np.mean([x['speeds'][-250:].mean() for x in r if not x['fell']] or [np.nan]))
    out['drive_fell'] = rate([x['fell'] for x in r])
    return out


def ppo_policy(path):
    from stable_baselines3 import PPO
    m = PPO.load(path, device='cpu')
    return lambda obs, env: m.predict(obs, deterministic=True)[0]


def pid_policy(kp=3.0, kd=0.4, kv=2.0, kx=0.3, dz=0.10):
    """The firmware's 'pid' mode (tailwhip_robot.ino) with its default gains, for the comparison in SPEC milestone 4.
    u = kp*theta + kd*theta_dot + kv*v + kx*x, plus the dead-zone step. It balances; it cannot rear up."""
    state = dict(x=0.0)
    def f(obs, env):
        p, pr = obs[0], obs[1] * 5
        v = 0.0325 * ((obs[3] + obs[4]) / 2 * 20 + pr)            # ground speed: wheel rel. body + pitch rate
        state['x'] += v * CTRL_DT
        u = kp * p + kd * pr + kv * v + kx * state['x']
        if abs(u) > 0.005:
            u += np.sign(u) * dz
        u = float(np.clip(u, -1, 1))
        return np.array([u, u, (73.6 - 85) / 65])
    return f


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=50)
    ap.add_argument('--policy', default=os.path.join(HERE, 'policies', 'tailwhip.zip'))
    args = ap.parse_args()
    res = dict(n=args.n, policy=score(ppo_policy(args.policy), args.n))
    r = [run(pid_policy(), 'shove', 5300 + i) for i in range(args.n)]
    res['pid_baseline'] = dict(shove=float(np.mean([(not x['fell'] and abs(x['pitches'][-1]) < 10) for x in r])))
    r = [run(pid_policy(), 'upright', 5100 + i, shoves=True) for i in range(args.n)]
    res['pid_baseline']['balance'] = float(np.mean([(not x['fell'] and np.mean(np.abs(x['pitches']) < 10) >= 0.9) for x in r]))
    json.dump(res, open(os.path.join(HERE, 'eval.json'), 'w'), indent=1)
    print(f"over {args.n} randomised robots each:")
    for k, v in res['policy'].items():
        print(f'  policy  {k:16s} {v:.2f}')
    for k, v in res['pid_baseline'].items():
        print(f'  PID     {k:16s} {v:.2f}')
