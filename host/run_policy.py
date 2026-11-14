"""Run the trained policy on the laptop and drive the real robot with it (milestone 4, robot in 'remote' mode).

    python host/run_policy.py PORT                          # PORT = the bridge ESP32, e.g. /dev/cu.usbserial-0001
    python host/run_policy.py PORT --log run1.npz           # also record every packet (host/log_plot.py run1.npz)

Type a command and Enter while it runs:
    stand            rear up and balance            rest        lie back down onto the tail
    drive 0.15       forward speed, m/s (-0.25..0.25)
    turn 1.0         turn rate, rad/s (+ = left, -1.5..1.5)
    stop             speed and turn back to 0       quit        put the tail down, motors off, exit

It builds the observation exactly as the simulator does (host/link.py obs_step: 13 floats per step, 4 steps newest
first) at 50 Hz and sends [u_left, u_right, tail] back. If this program stops or the radio drops for 100 ms, the
robot's own failsafe stops the wheels and puts the tail to rest.
"""
import argparse, os, sys, threading, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import link as L

ROOT = os.path.dirname(HERE)
DT = 0.02


def load_policy(path):
    """numpy forward pass of the PPO actor (same maths as the ESP32), so torch is only needed to read the file."""
    sys.path.insert(0, os.path.join(ROOT, 'sim'))
    from export_policy import weights, forward
    Ws, _ = weights(path)
    return lambda obs: forward(Ws, obs.astype(np.float32))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('port')
    ap.add_argument('--policy', default=os.path.join(ROOT, 'sim', 'policies', 'tailwhip.zip'))
    ap.add_argument('--log')
    ap.add_argument('--head', type=float, default=90.0, help='head pan angle, deg (90 = forward)')
    args = ap.parse_args()
    pi = load_policy(args.policy)
    link = L.Link(args.port)
    if args.log:
        link.start_log(args.log)
    s = link.wait_state(3.0)
    if s is None:
        print('no state packets: is the robot on and the bridge plugged in?'); link.close(); return
    print(f'robot answering: pitch {s.pitch_deg:+.1f} deg, battery {s.vbat:.2f} V. Commands: stand, rest, drive V, turn W, stop, quit')
    cmd = dict(stand=0, v_mps=0.0, yaw_rps=0.0)
    run = [True]

    def keyboard():
        for line in sys.stdin:
            w = line.split()
            if not w:
                continue
            k = w[0].lower()
            if k == 'stand': cmd['stand'] = 1
            elif k == 'rest': cmd.update(stand=0, v_mps=0.0, yaw_rps=0.0)
            elif k == 'drive' and len(w) > 1: cmd['v_mps'] = float(np.clip(float(w[1]), -0.25, 0.25))
            elif k == 'turn' and len(w) > 1: cmd['yaw_rps'] = float(np.clip(float(w[1]), -1.5, 1.5))
            elif k == 'stop': cmd.update(v_mps=0.0, yaw_rps=0.0)
            elif k in ('quit', 'q', 'exit'): run[0] = False; return
            else: print('?  stand | rest | drive V | turn W | stop | quit')
    threading.Thread(target=keyboard, daemon=True).start()

    stack = L.ObsStack()
    a_rest = (L.TAIL_REST - 85.0) / 65.0
    prev, prev2 = np.array([0.0, 0.0, a_rest]), np.array([0.0, 0.0, a_rest])
    t_next, k = time.perf_counter(), 0
    try:
        while run[0]:
            s = link.state()
            obs = stack.push(L.obs_step(s, prev, (cmd['stand'], cmd['v_mps'], cmd['yaw_rps']), prev2))
            a = np.clip(pi(obs), -1, 1)
            link.send(float(a[0]), float(a[1]), 85.0 + 65.0 * float(a[2]), args.head, 'remote',
                      stand=cmd['stand'], v_mps=cmd['v_mps'], yaw_rps=cmd['yaw_rps'])
            prev2, prev = prev, a
            k += 1
            if k % 25 == 0:
                age = (time.perf_counter() - s.t_rx) * 1000
                print(f"\r{'STAND' if cmd['stand'] else 'REST '} v {cmd['v_mps']:+.2f} w {cmd['yaw_rps']:+.2f} | pitch {s.pitch_deg:+6.1f} "
                      f"tail {85 + 65 * a[2]:5.1f} u {a[0]:+.2f} {a[1]:+.2f} | {s.vbat:.2f} V state age {age:3.0f} ms   ", end='', flush=True)
            t_next += DT
            time.sleep(max(0.0, t_next - time.perf_counter()))
    except KeyboardInterrupt:
        pass
    finally:
        print('\nstopping: tail down, motors off')
        link.idle()
        if args.log:
            link.stop_log()
        link.close()


if __name__ == '__main__':
    main()
