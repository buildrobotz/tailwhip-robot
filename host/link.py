"""Laptop <-> bridge ESP32 <-> robot. The only file that knows about bytes.

    from link import Link, obs_step
    link = Link('/dev/cu.usbserial-0001')         # Windows 'COM5', Linux '/dev/ttyUSB0'
    link.send(u_left, u_right, tail_deg, head_deg, mode='remote')
    s = link.state()                              # latest State, or None before the first packet
    link.start_log('run1.npz')  ...  link.stop_log()
    link.close()

Units and signs (shared with the simulator and the firmware):
  pitch + = leaning forward, 0 = upright; u in [-1, 1], + = forward, motor volts = u x 6.0 V;
  tail deg: 0 = straight down the body, + = backwards (20..150, rest 73.6); head deg: 90 = forward, + = left.
"""
import csv, struct, threading, time, pathlib
from dataclasses import dataclass, asdict, fields
import numpy as np
import serial

ROBOT_ID = 1                                  # must match tailwhip_common.h
BAUD = 921600
CMD = struct.Struct('<4BH6h')                 # CmdPacket, 18 bytes (firmware/*/tailwhip_common.h)
STATE = struct.Struct('<4B3HI7hH2h2HhH')      # StatePacket, 42 bytes
CMD_MAGIC, STATE_MAGIC = 0xC7, 0x7D
MODES = dict(idle=0, pid=1, remote=2, onboard=3, bench=4, keep=255)
MODE_NAMES = {v: k for k, v in MODES.items()}
CF_STAND, CF_SERVOS_OFF = 0x01, 0x02
FLAGS = dict(failsafe=0x0001, tilt_cut=0x0002, wheels=0x0004, imu_fault=0x0008, low_batt=0x0010,
             motors_on=0x0020, servos_on=0x0040, pid_fallen=0x0080, policy=0x0100, no_batt=0x0200,
             volt_clip=0x0400, link=0x0800)
TAIL_MIN, TAIL_MAX, TAIL_REST = 20.0, 150.0, 73.6
V_MOTOR = 6.0                                 # u = 1 means 6.0 V (remote, onboard, pid)
V_BENCH = 8.4                                 # u = 1 means 8.4 V (bench mode only)


def crc8(data):
    """CRC-8, polynomial 0x07, init 0 (same as tw_crc8 in tailwhip_common.h)."""
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def frame(payload):
    return bytes([0xAA, 0x55, len(payload)]) + payload + bytes([crc8(payload)])


@dataclass
class State:
    seq: int
    t_robot_ms: int
    pitch_deg: float
    pitch_rate_dps: float
    yaw_rate_dps: float
    wheel_l_rads: float          # signed by the last non-zero command to that wheel; relative to the body
    wheel_r_rads: float
    tail_cmd_deg: float
    head_cmd_deg: float
    vbat: float
    mode: int
    flags: int
    t_rx: float                  # time.perf_counter() on the laptop when the frame arrived
    # extras (not needed by the policy)
    last_cmd_seq: int = 0
    cmd_age_ms: int = 0
    u_left: float = 0.0          # wheel command the robot actually applied
    u_right: float = 0.0
    edges_l: int = 0
    edges_r: int = 0
    pitch_acc_deg: float = 0.0
    cmd_lost: int = 0

    def flag(self, name):
        return bool(self.flags & FLAGS[name])

    @property
    def mode_name(self):
        return MODE_NAMES.get(self.mode, '?')


def pack_cmd(seq, u_left, u_right, tail_deg, head_deg, mode, stand=1, v_mps=0.0, yaw_rps=0.0, servos_off=False):
    m = MODES[mode] if isinstance(mode, str) else int(mode)
    flags = (CF_STAND if stand else 0) | (CF_SERVOS_OFF if servos_off else 0)
    q = lambda x, s, lo, hi: int(round(np.clip(x * s, lo, hi)))
    return CMD.pack(CMD_MAGIC, ROBOT_ID, m, flags, seq & 0xFFFF,
                    q(u_left, 1e4, -10000, 10000), q(u_right, 1e4, -10000, 10000),
                    q(np.clip(tail_deg, TAIL_MIN, TAIL_MAX), 100, -32000, 32000), q(np.clip(head_deg, 30, 150), 100, -32000, 32000),
                    q(v_mps, 1000, -32000, 32000), q(yaw_rps, 1000, -32000, 32000))


def unpack_state(payload, t_rx=0.0):
    v = STATE.unpack(payload)
    (magic, rid, mode, age, flags, seq, last, t_ms, p, pr, yr, wl, wr, tail, head, vb, ul, ur, el, er, pacc, lost) = v
    return State(seq=seq, t_robot_ms=t_ms, pitch_deg=p / 100, pitch_rate_dps=pr / 10, yaw_rate_dps=yr / 10,
                 wheel_l_rads=wl / 100, wheel_r_rads=wr / 100, tail_cmd_deg=tail / 100, head_cmd_deg=head / 100,
                 vbat=vb / 1000, mode=mode, flags=flags, t_rx=t_rx, last_cmd_seq=last, cmd_age_ms=age,
                 u_left=ul / 1e4, u_right=ur / 1e4, edges_l=el, edges_r=er, pitch_acc_deg=pacc / 100, cmd_lost=lost)


def obs_step(state, prev_action, cmd, prev_action2=None):
    """The 13-float observation for one 50 Hz step, identical to the firmware's obsStep() and the simulator.

    state: State; prev_action: [u_left, u_right, a_tail] sent last step; prev_action2: the one sent the step before
    (defaults to prev_action, as at start-up); cmd: (stand, v_mps, yaw_rps) or a dict with those keys.
    Layout (as sim/tailwhip_env.py): [5] = last tail action, [6..8] = the action before that. The policy input is the last 4 of these stacked NEWEST FIRST (see ObsStack)."""
    if isinstance(cmd, dict):
        stand, v, w = cmd.get('stand', 1), cmd.get('v_mps', cmd.get('v', 0.0)), cmd.get('yaw_rps', cmd.get('w', 0.0))
    else:
        stand, v, w = cmd
    a = np.asarray(prev_action, np.float32)
    b = a if prev_action2 is None else np.asarray(prev_action2, np.float32)
    vb = state.vbat if state.vbat > 3.0 else 7.6          # on USB only: pretend nominal (motors cannot move anyway)
    return np.array([np.deg2rad(state.pitch_deg), np.deg2rad(state.pitch_rate_dps) / 5, np.deg2rad(state.yaw_rate_dps) / 5,
                     state.wheel_l_rads / 20, state.wheel_r_rads / 20, a[2], b[0], b[1], b[2],
                     (vb - 7.6) / 0.8, 1.0 if stand else 0.0, v / 0.3, w / 2.0], np.float32)


class ObsStack:
    """Last n observation steps, newest first. On the first push every slot is filled with that step (as on the robot)."""
    def __init__(self, n=4, dim=13):
        self.n, self.dim, self.buf = n, dim, None

    def push(self, o):
        o = np.asarray(o, np.float32)
        self.buf = np.tile(o, self.n) if self.buf is None else np.concatenate([o, self.buf[:-self.dim]])
        return self.buf.copy()

    def reset(self):
        self.buf = None


class Link:
    def __init__(self, port, baud=BAUD, ser=None):
        self.ser = ser if ser is not None else serial.Serial(port, baud, timeout=0.01)
        self.seq, self._state, self._lock = 0, None, threading.Lock()
        self.sent_at = {}                     # cmd seq -> perf_counter when sent
        self.rtt_ms = []                      # (state rx - cmd send - cmd_age), one per command echoed
        self.n_rx = self.n_bad = 0
        self._log, self._log_path = None, None
        self._run = True
        self._t = threading.Thread(target=self._reader, daemon=True)
        self._t.start()

    # ------------------------------------------------------------------ sending
    def send(self, u_left, u_right, tail_deg, head_deg=90.0, mode='remote', stand=1, v_mps=0.0, yaw_rps=0.0, servos_off=False):
        self.seq = (self.seq + 1) & 0xFFFF
        p = pack_cmd(self.seq, u_left, u_right, tail_deg, head_deg, mode, stand, v_mps, yaw_rps, servos_off)
        t = time.perf_counter()
        self.sent_at[self.seq] = t
        if len(self.sent_at) > 2000:
            for k in list(self.sent_at)[:1000]:
                del self.sent_at[k]
        self.ser.write(frame(p))
        if self._log is not None:
            self._log['cmd'].append([t, self.seq, u_left, u_right, tail_deg, head_deg,
                                     MODES[mode] if isinstance(mode, str) else mode, stand, v_mps, yaw_rps])

    def idle(self, n=5):
        for _ in range(n):
            self.send(0, 0, TAIL_REST, 90, 'idle'); time.sleep(0.02)

    # ------------------------------------------------------------------ receiving
    def state(self):
        with self._lock:
            return self._state

    def wait_state(self, timeout=2.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            s = self.state()
            if s is not None:
                return s
            time.sleep(0.01)
        raise TimeoutError('no state packets: is the bridge plugged in and the robot switched on?')

    def _reader(self):
        buf = bytearray()
        while self._run:
            try:
                buf += self.ser.read(256)
            except Exception:
                if not self._run:
                    return
                raise
            while len(buf) >= 4:
                if buf[0] != 0xAA or buf[1] != 0x55:
                    del buf[0]; continue
                n = buf[2]
                if len(buf) < n + 4:
                    break
                p, chk = bytes(buf[3:3 + n]), buf[3 + n]
                if crc8(p) != chk or n != STATE.size or p[0] != STATE_MAGIC or p[1] != ROBOT_ID:
                    self.n_bad += 1
                    del buf[0]; continue          # resync one byte on
                del buf[:n + 4]
                t = time.perf_counter()
                s = unpack_state(p, t)
                t0 = self.sent_at.pop(s.last_cmd_seq, None)    # first echo only
                if t0 is not None and s.cmd_age_ms < 255:
                    self.rtt_ms.append((t - t0) * 1e3 - s.cmd_age_ms)
                self.n_rx += 1
                with self._lock:
                    self._state = s
                if self._log is not None:
                    self._log['state'].append(s)

    # ------------------------------------------------------------------ logging
    def start_log(self, path):
        """Log every state packet and every command sent until stop_log(). .npz or .csv by extension."""
        self._log_path, self._log = pathlib.Path(path), dict(state=[], cmd=[])

    def stop_log(self):
        if self._log is None:
            return None
        log, path, self._log = self._log, self._log_path, None
        save_log(path, log['state'], log['cmd'])
        return path

    def close(self):
        self.stop_log()
        self._run = False
        time.sleep(0.03)
        try:
            self.ser.close()
        except Exception:
            pass


STATE_COLS = [f.name for f in fields(State)]
CMD_COLS = ['t', 'seq', 'u_left', 'u_right', 'tail_deg', 'head_deg', 'mode', 'stand', 'v_mps', 'yaw_rps']


def save_log(path, states, cmds):
    path = pathlib.Path(path)
    if path.suffix == '.csv':
        with open(path, 'w', newline='') as f:
            w = csv.writer(f); w.writerow(STATE_COLS)
            for s in states:
                w.writerow([getattr(s, c) for c in STATE_COLS])
        with open(path.with_name(path.stem + '_cmd.csv'), 'w', newline='') as f:
            w = csv.writer(f); w.writerow(CMD_COLS); w.writerows(cmds)
    else:
        arr = {f's_{c}': np.array([getattr(s, c) for s in states], float) for c in STATE_COLS}
        cm = np.array(cmds, float).reshape(-1, len(CMD_COLS))
        arr.update({f'c_{c}': cm[:, i] for i, c in enumerate(CMD_COLS)})
        np.savez_compressed(path, **arr)


def load_log(path):
    """Returns (state dict of arrays, cmd dict of arrays) from a .npz or .csv log."""
    path = pathlib.Path(path)
    if path.suffix == '.csv':
        def rd(p):
            if not p.exists():
                return {}
            rows = list(csv.reader(open(p)))
            return {k: np.array([float(r[i]) for r in rows[1:]]) for i, k in enumerate(rows[0])}
        return rd(path), rd(path.with_name(path.stem + '_cmd.csv'))
    z = np.load(path)
    return ({k[2:]: z[k] for k in z.files if k.startswith('s_')}, {k[2:]: z[k] for k in z.files if k.startswith('c_')})
