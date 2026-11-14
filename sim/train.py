"""Train the Tailwhip policy with PPO on a laptop CPU.

    python sim/train.py                      # 20 M steps, about 25 min on a 10-core laptop
    python sim/train.py --steps 5000000      # a quick one
    python sim/train.py --init sim/policies/tailwhip.zip   # continue (e.g. after updating host/measurements.json)
    tensorboard --logdir sim/runs            # watch rollout/ep_rew_mean and tw/* climb

The network is two hidden layers of 64 (tanh), small enough to run on the ESP32 (sim/export_policy.py).
"""
import argparse, os, time
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback

HERE = os.path.dirname(os.path.abspath(__file__))


def make(seed):
    def f():
        import sys; sys.path.insert(0, HERE)
        from tailwhip_env import TailwhipEnv
        return TailwhipEnv(randomize=True, seed=seed)
    return f


class Progress(BaseCallback):
    """Prints the numbers worth watching: time upright, falls, time on the tail foot."""
    def __init__(self):
        super().__init__(); self.buf = []; self.t0 = time.time()
    def _on_step(self):
        for info, done in zip(self.locals['infos'], self.locals['dones']):
            if done:
                self.buf.append((info['up_time'], float(info['fell']), info['foot_time'], info['episode']['r'], info['kind']))
        return True
    def _on_rollout_end(self):
        if len(self.buf) > 20:
            b = self.buf[-300:]
            up = np.mean([x[0] for x in b]); fell = np.mean([x[1] for x in b]); ret = np.mean([x[3] for x in b])
            rest_up = np.mean([x[0] for x in b if x[4] == 'rest'] or [0])
            for k, v in (('up_s', up), ('fell', fell), ('return', ret), ('rest_start_up_s', rest_up)):
                self.logger.record(f'tw/{k}', float(v))
            print(f'{self.num_timesteps:>11,d} steps {time.time() - self.t0:6.0f}s  return {ret:7.1f}  upright {up:4.1f} s/10  '
                  f'from rest {rest_up:4.1f} s  fell {fell:.2f}', flush=True)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--steps', type=int, default=20_000_000)
    ap.add_argument('--envs', type=int, default=max(2, (os.cpu_count() or 4) - 2))
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--init')
    ap.add_argument('--out', default=os.path.join(HERE, 'policies', 'tailwhip'))
    args = ap.parse_args()
    env = VecMonitor(SubprocVecEnv([make(args.seed * 100 + i) for i in range(args.envs)]))
    model = PPO('MlpPolicy', env, n_steps=2048, batch_size=4096, n_epochs=8, gamma=0.99, gae_lambda=0.95,
                learning_rate=3e-4, ent_coef=0.0, clip_range=0.2, max_grad_norm=0.5,
                policy_kwargs=dict(net_arch=dict(pi=[64, 64], vf=[128, 128]), log_std_init=-0.5),
                tensorboard_log=os.path.join(HERE, 'runs'), device='cpu', seed=args.seed, verbose=0)
    if args.init:
        model.set_parameters(args.init, exact_match=False, device='cpu')
    ckpt = CheckpointCallback(max(1, 2_000_000 // args.envs), os.path.join(HERE, 'runs', 'ckpt'), name_prefix='tailwhip')
    model.learn(args.steps, callback=[Progress(), ckpt], tb_log_name='tailwhip')
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    model.save(args.out)
    print('saved', args.out + '.zip')
