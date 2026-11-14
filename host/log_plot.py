"""Plot a run: live from the robot, or from a log written by Link.start_log (drive.py --log, run_policy.py, ...).

    python log_plot.py run1.npz                   plot a saved log (.npz or .csv)
    python log_plot.py --live PORT                live 10 s window; sends nothing, robot keeps its mode
    python log_plot.py --live PORT --mode pid --log pid1.npz
                                                  keep the robot in pid mode (50 Hz keep-alive) and log it

Only one program can own the bridge's serial port, so close drive.py before plotting live.
Panels: pitch (filter and accelerometer-only), pitch and yaw rate, wheel speeds, wheel commands, tail and head
commands, battery voltage.
"""
import argparse, threading, time
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from link import Link, load_log, MODES

PANELS = [('pitch (deg)', ['pitch_deg', 'pitch_acc_deg']), ('rate (deg/s)', ['pitch_rate_dps', 'yaw_rate_dps']),
          ('wheel (rad/s)', ['wheel_l_rads', 'wheel_r_rads']), ('u applied', ['u_left', 'u_right']),
          ('servo cmd (deg)', ['tail_cmd_deg', 'head_cmd_deg']), ('battery (V)', ['vbat'])]


def plot_file(path):
    st, cm = load_log(path)
    t = (st['t_robot_ms'] - st['t_robot_ms'][0]) / 1000
    fig, ax = plt.subplots(len(PANELS), 1, sharex=True, figsize=(11, 10))
    for a, (title, keys) in zip(ax, PANELS):
        for k in keys:
            a.plot(t, st[k], lw=1, label=k)
        a.set_ylabel(title); a.legend(loc='upper right', fontsize=8); a.grid(alpha=.3)
    ax[-1].set_xlabel('robot time (s)')
    seq = st['seq'].astype(int)
    gaps = int(np.sum((np.diff(seq) & 0xFFFF) - 1)) if len(seq) > 1 else 0
    fig.suptitle(f'{path}: {len(t)} states over {t[-1]:.1f} s, {gaps} state packets missing')
    plt.tight_layout(); plt.show()


def plot_live(port, mode, log):
    link = Link(port)
    if log:
        link.start_log(log)
    stop = threading.Event()
    if mode:
        def keep():
            while not stop.is_set():
                link.send(0, 0, 150 if mode == 'pid' else 73.6, 90, mode); time.sleep(0.02)
        threading.Thread(target=keep, daemon=True).start()
    buf = {k: deque(maxlen=1000) for _, ks in PANELS for k in ks}; tq = deque(maxlen=1000)
    fig, ax = plt.subplots(len(PANELS), 1, sharex=True, figsize=(11, 10))
    lines = {}
    for a, (title, keys) in zip(ax, PANELS):
        for k in keys:
            lines[k], = a.plot([], [], lw=1, label=k)
        a.set_ylabel(title); a.legend(loc='upper left', fontsize=8); a.grid(alpha=.3)
    plt.ion(); plt.show()
    last = None
    try:
        while plt.fignum_exists(fig.number):
            s = link.state()
            if s is not None and s is not last:
                last = s; tq.append(s.t_robot_ms / 1000)
                for k in buf:
                    buf[k].append(getattr(s, k))
            if tq:
                t = np.array(tq) - tq[-1]
                for k, ln in lines.items():
                    ln.set_data(t, np.array(buf[k]))
                for a in ax:
                    a.relim(); a.autoscale_view(); a.set_xlim(-10, 0)
                fig.suptitle(f'live: mode {last.mode_name}  flags 0x{last.flags:04X}  {last.vbat:.2f} V')
            plt.pause(0.05)
    finally:
        stop.set()
        if mode:
            link.idle()
        link.close()


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log_file', nargs='?'); ap.add_argument('--live', metavar='PORT')
    ap.add_argument('--mode', choices=[m for m in MODES if m not in ('keep', 'remote', 'bench')])
    ap.add_argument('--log')
    a = ap.parse_args()
    if a.live:
        plot_live(a.live, a.mode, a.log)
    elif a.log_file:
        plot_file(a.log_file)
    else:
        ap.error('give a log file or --live PORT')
