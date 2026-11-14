"""Run the trained policy through the whole act in MuJoCo and film it.

    python sim/demo.py                                  # -> sim/out/demo.mp4 + sim/out/demo.json (per-step log)
    python sim/demo.py --randomize --seed 3             # same act on a randomised robot

Act (commands are what the laptop or the on-board mode would send):
   0- 2 s  REST          lying back on its tail
   2- 7 s  STAND         rears up and balances
   7- 9 s  STAND 0.2 m/s drives forward
   9-11 s  STAND, turn   turns on the spot
  11-15 s  STAND         gets shoved backwards at 12 s
  15-19 s  REST          lies back down on its tail
"""
import argparse, json, os, subprocess, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from tailwhip_env import TailwhipEnv, CTRL_DT

ACT = [(0, 2, [0, 0, 0], 'Resting on its tail'), (2, 7, [1, 0, 0], 'Stand: rears up and balances'),
       (7, 9, [1, 0.2, 0], 'Drive forward at 0.2 m/s'), (9, 11, [1, 0, 1.5], 'Turn on the spot'),
       (11, 15, [1, 0, 0], 'Shoved backwards'), (15, 19, [0, 0, 0], 'Rest: lies back down on its tail')]
SHOVES = [(12.0, -1.4)]           # (time s, force N along the heading, for 0.1 s)


def ffmpeg():
    for p in ('/opt/homebrew/bin/ffmpeg', '/usr/local/bin/ffmpeg', 'ffmpeg'):
        if p == 'ffmpeg' or os.path.exists(p):
            return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--policy', default=os.path.join(HERE, 'policies', 'tailwhip.zip'))
    ap.add_argument('--randomize', action='store_true')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--out', default=os.path.join(HERE, 'out'))
    ap.add_argument('--no-video', action='store_true')
    args = ap.parse_args()
    from stable_baselines3 import PPO
    model = PPO.load(args.policy, device='cpu')
    T = ACT[-1][1]
    env = TailwhipEnv(randomize=args.randomize, seed=args.seed, start='rest', command=[0, 0, 0], shoves=False,
                      horizon=int(T / CTRL_DT) + 5)
    obs, _ = env.reset()
    os.makedirs(args.out, exist_ok=True)
    proc = None
    if not args.no_video:
        proc = subprocess.Popen([ffmpeg(), '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                                 '-s', '854x480', '-r', '50', '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                                 '-crf', '20', '-movflags', '+faststart', os.path.join(args.out, 'demo.mp4')],
                                stdin=subprocess.PIPE)
    log = []
    for k in range(int(T / CTRL_DT)):
        t = k * CTRL_DT
        seg = next(s for s in ACT if s[0] <= t < s[1])
        env.cmd = np.array(seg[2], float)
        obs[10::13] = env.cmd[0]; obs[11::13] = env.cmd[1] / 0.3; obs[12::13] = env.cmd[2] / 2
        for ts, f in SHOVES:
            if ts <= t < ts + 0.1:
                R = env.data.xmat[env.bid].reshape(3, 3)
                fwd = np.array([R[0, 0], R[1, 0], 0]); fwd /= np.linalg.norm(fwd)
                env.shove_f, env.shove_end = f * fwd, env.t + 0.1
        a, _ = model.predict(obs, deterministic=True)
        obs, r, term, trunc, info = env.step(a)
        log.append(dict(t=round(t, 2), caption=seg[3], cmd=seg[2], pitch_deg=round(float(np.degrees(info['pitch'])), 2),
                        tail_deg=round(85 + 65 * float(a[2]), 1), u=[round(float(a[0]), 3), round(float(a[1]), 3)],
                        x=[round(float(v), 4) for v in env.data.qpos[:2]], v=round(info['v'], 3), fell=bool(info['fell'])))
        if proc:
            proc.stdin.write(env.render().tobytes())
        if term:
            break
    if proc:
        proc.stdin.close(); proc.wait()
    json.dump(log, open(os.path.join(args.out, 'demo.json'), 'w'))
    up = np.mean([abs(l['pitch_deg']) < 10 for l in log if l['cmd'][0] == 1 and l['t'] > 4])
    print(f'{len(log)} steps, fell: {log[-1]["fell"]}, upright {100 * up:.0f} % of the stand time after 4 s -> {args.out}')


if __name__ == '__main__':
    main()
