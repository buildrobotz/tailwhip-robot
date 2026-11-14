"""Keyboard teleop over the radio (milestone 1: drive on the tail).

    python drive.py PORT [--max 0.5] [--log drive1.npz]

The robot rests on its tail (tail 73.6 deg) and you drive it like a tank. Keys (in the terminal window):
  w / s     faster forward / backward (steps of 0.1)        space  stop
  a / d     turn left / right (steps of 0.1)                 x      straighten (turn 0)
  [ / ]     tail -5 / +5 deg        r  tail to rest 73.6     f  tail to 24.3 (foot under the axle: rears up!)
  , / .     head pan left / right 10 deg                     c      head forward
  p         switch to on-board PID balance (hold the robot upright first), o  back to remote
  q         quit (robot goes idle)
Motor voltage = u x 6.0 V. --max caps |u| (default 0.5 = 3 V) until you trust it. Commands go out at 50 Hz;
if this program stops, the robot's 100 ms failsafe stops the wheels and puts the tail to rest.
"""
import argparse, curses, time
import numpy as np
from link import Link, TAIL_REST


def main(scr, a):
    link = Link(a.port)
    if a.log:
        link.start_log(a.log)
    scr.nodelay(True); curses.curs_set(0)
    fwd = turn = 0.0; tail, head, mode = TAIL_REST, 90.0, 'remote'
    t_next = time.perf_counter()
    try:
        while True:
            k = scr.getch()
            if k != -1:
                ch = chr(k) if k < 256 else ''
                if ch == 'q': break
                fwd += {'w': 0.1, 's': -0.1}.get(ch, 0); turn += {'a': 0.1, 'd': -0.1}.get(ch, 0)
                if ch == ' ': fwd = turn = 0.0
                if ch == 'x': turn = 0.0
                tail += {'[': -5, ']': 5}.get(ch, 0); head += {',': 10, '.': -10}.get(ch, 0)
                if ch == 'r': tail = TAIL_REST
                if ch == 'f': tail = 24.3
                if ch == 'c': head = 90.0
                if ch == 'p': mode, fwd, turn = 'pid', 0.0, 0.0
                if ch == 'o': mode = 'remote'
                fwd, turn = np.clip(fwd, -a.max, a.max), np.clip(turn, -a.max, a.max)
                tail, head = float(np.clip(tail, 20, 150)), float(np.clip(head, 30, 150))
            uL, uR = np.clip(fwd - turn, -a.max, a.max), np.clip(fwd + turn, -a.max, a.max)
            if mode == 'pid':      # w/s and a/d become speed and turn setpoints for the PID
                link.send(0, 0, 150, head, 'pid', v_mps=fwd * 0.6, yaw_rps=turn * 4)
            else:
                link.send(uL, uR, tail, head, 'remote')
            s = link.state()
            scr.erase()
            scr.addstr(0, 0, f'mode {mode:7s} fwd {fwd:+.1f} turn {turn:+.1f} -> u L {uL:+.2f} R {uR:+.2f}   tail {tail:5.1f}  head {head:5.1f}')
            if s:
                fl = ' '.join(n for n in ('failsafe', 'tilt_cut', 'wheels', 'imu_fault', 'low_batt', 'volt_clip', 'pid_fallen') if s.flag(n))
                scr.addstr(2, 0, f'robot {s.mode_name:7s} pitch {s.pitch_deg:+6.1f} deg  rate {s.pitch_rate_dps:+7.1f}  yaw {s.yaw_rate_dps:+7.1f} dps')
                scr.addstr(3, 0, f'wheels L {s.wheel_l_rads:+6.2f} R {s.wheel_r_rads:+6.2f} rad/s  applied u {s.u_left:+.2f} {s.u_right:+.2f}  {s.vbat:.2f} V')
                scr.addstr(4, 0, f'flags: {fl or "-"}   age {1e3 * (time.perf_counter() - s.t_rx):.0f} ms')
            else:
                scr.addstr(2, 0, 'no state yet: robot on? bridge on this port?')
            scr.addstr(6, 0, 'w/s a/d space x | [ ] r f tail | , . c head | p pid  o remote | q quit')
            scr.refresh()
            t_next += 0.02
            time.sleep(max(0.0, t_next - time.perf_counter()))
    finally:
        link.idle()
        link.close()


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('port'); ap.add_argument('--max', type=float, default=0.5); ap.add_argument('--log')
    curses.wrapper(main, ap.parse_args())
