"""Check host/link.py against the firmware's packet layout, with no hardware.

    python test_protocol.py

1. The two copies of tailwhip_common.h (robot and bridge) are identical.
2. Compiles a small C++ probe against tailwhip_common.h (needs c++/clang, skipped if absent): sizes, every field offset,
   a sample StatePacket byte for byte, and tw_crc8 against link.crc8.
3. Runs the real Link against a fake serial port: the command frame it writes is what the bridge accepts, and a state
   frame laid out as the firmware sends it (with junk and a corrupted frame in front) unpacks to the right values.
4. obs_step and ObsStack produce the 13 / 52-float layout the policy expects.
5. A random network written in the policy.h format gives the same actions in C++ (the firmware's loops) and numpy.
"""
import pathlib, shutil, struct, subprocess, sys, tempfile, time
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
FW = HERE.parent / 'firmware'
HDR = FW / 'tailwhip_robot' / 'tailwhip_common.h'
sys.path.insert(0, str(HERE))
import link as L                                                       # noqa: E402

CMD_FIELDS = dict(magic=0, robot_id=1, mode=2, flags=3, seq=4, u_left=6, u_right=8, tail_cdeg=10, head_cdeg=12,
                  v_mm_s=14, yaw_mrad_s=16)
STATE_FIELDS = dict(magic=0, robot_id=1, mode=2, cmd_age_ms=3, flags=4, seq=6, last_cmd_seq=8, t_ms=10, pitch_cdeg=14,
                    pitch_rate_ddps=16, yaw_rate_ddps=18, wheel_l_crad_s=20, wheel_r_crad_s=22, tail_cmd_cdeg=24,
                    head_cmd_cdeg=26, vbat_mV=28, u_left=30, u_right=32, edges_l=34, edges_r=36, pitch_acc_cdeg=38,
                    cmd_lost=40)
SAMPLE = dict(mode=3, cmd_age_ms=7, flags=0x0965, seq=65000, last_cmd_seq=1234, t_ms=987654, pitch_cdeg=-2500,
              pitch_rate_ddps=-1234, yaw_rate_ddps=567, wheel_l_crad_s=-1520, wheel_r_crad_s=2047, tail_cmd_cdeg=7360,
              head_cmd_cdeg=9000, vbat_mV=7812, u_left=-3333, u_right=10000, edges_l=65535, edges_r=12, pitch_acc_cdeg=-2471,
              cmd_lost=3)
FMT = dict(magic='B', robot_id='B', mode='B', cmd_age_ms='B', flags='H', seq='H', last_cmd_seq='H', t_ms='I',
           vbat_mV='H', edges_l='H', edges_r='H', cmd_lost='H')


def firmware_bytes(p):
    """A StatePacket exactly as the ESP32 lays it out: field by field at the header's offsets, little-endian."""
    b = bytearray(42)
    vals = dict(p, magic=0x7D, robot_id=L.ROBOT_ID)
    for k, off in STATE_FIELDS.items():
        struct.pack_into('<' + FMT.get(k, 'h'), b, off, vals[k])
    return bytes(b)


def check_copies():
    a, b = HDR.read_bytes(), (FW / 'tailwhip_bridge' / 'tailwhip_common.h').read_bytes()
    assert a == b, 'firmware/tailwhip_robot/tailwhip_common.h and firmware/tailwhip_bridge/tailwhip_common.h differ'
    print('PASS  robot and bridge copies of tailwhip_common.h are identical')


def compiler():
    return shutil.which('c++') or shutil.which('clang++') or shutil.which('g++')


def run_cpp(src, d):
    c, exe = pathlib.Path(d, 'probe.cpp'), pathlib.Path(d, 'probe')
    c.write_text(src)
    subprocess.run([compiler(), '-std=c++17', '-O1', '-I', str(d), '-o', str(exe), str(c)], check=True)
    return subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout.split('\n')


def check_with_compiler():
    if not compiler():
        print('SKIP  C++ probe (no compiler found)'); return
    nl = '\\n'
    offs = ' '.join(f'printf("C {k} %zu{nl}", offsetof(CmdPacket, {k}));' for k in CMD_FIELDS)
    offs += ' '.join(f'printf("S {k} %zu{nl}", offsetof(StatePacket, {k}));' for k in STATE_FIELDS)
    assign = ' '.join(f's.{k} = {v};' for k, v in SAMPLE.items())
    crc_in = list(range(0, 250, 7))
    src = f'''#include <cstdio>
#include <cstddef>
#include <cstring>
#include "{HDR}"
int main() {{
  printf("%zu %zu\\n", sizeof(CmdPacket), sizeof(StatePacket));
  {offs}
  StatePacket s; memset(&s, 0, sizeof s); s.magic = STATE_MAGIC; s.robot_id = ROBOT_ID; {assign}
  const unsigned char *p = (const unsigned char *)&s; printf("B");
  for (size_t i = 0; i < sizeof s; i++) printf(" %u", p[i]);
  printf("\\n");
  const uint8_t crcin[] = {{{','.join(map(str, crc_in))}}};
  printf("R %u\\n", tw_crc8(crcin, sizeof crcin));
}}'''
    with tempfile.TemporaryDirectory() as d:
        out = run_cpp(src, d)
    sizes = tuple(map(int, out[0].split()))
    assert sizes == (L.CMD.size, L.STATE.size) == (18, 42), f'sizes C++ {sizes} vs Python {L.CMD.size, L.STATE.size}'
    for line in out[1:]:
        if line.startswith(('C ', 'S ')):
            kind, name, off = line.split()
            want = (CMD_FIELDS if kind == 'C' else STATE_FIELDS)[name]
            assert int(off) == want, f'{kind} {name}: header offset {off}, test expects {want}'
        elif line.startswith('B'):
            assert bytes(map(int, line.split()[1:])) == firmware_bytes(SAMPLE), 'C++ bytes differ from firmware_bytes()'
        elif line.startswith('R'):
            assert int(line.split()[1]) == L.crc8(bytes(crc_in)), 'tw_crc8 and link.crc8 disagree'
    assert L.crc8(b'123456789') == 0xF4, 'CRC-8/SMBUS check value'
    print(f'PASS  C++ probe ({compiler()}): sizes 18/42, every field offset, sample state bytes, CRC-8 identical')


class FakeSerial:
    def __init__(self):
        self.rx, self.tx = bytearray(), bytearray()
    def read(self, n):
        if not self.rx:
            time.sleep(0.002); return b''
        out = bytes(self.rx[:n]); del self.rx[:n]; return out
    def write(self, b):
        self.tx += b
    def close(self):
        pass


def bridge_accepts(fr):
    """What tailwhip_bridge.ino checks before forwarding a frame to the radio."""
    return (fr[:2] == b'\xAA\x55' and fr[2] == 18 and len(fr) == 22 and L.crc8(fr[3:21]) == fr[21]
            and fr[3] == 0xC7 and fr[4] == L.ROBOT_ID)


def check_link():
    fs = FakeSerial()
    k = L.Link('fake', ser=fs)
    k.send(0.5, -1.0, 73.6, 90.0, 'remote', stand=1, v_mps=0.25, yaw_rps=-1.5)
    f = bytes(fs.tx)
    assert bridge_accepts(f), f
    v = L.CMD.unpack(f[3:21])
    assert v == (0xC7, L.ROBOT_ID, 2, 1, k.seq, 5000, -10000, 7360, 9000, 250, -1500), v
    k.send(3.0, -3.0, 10.0, 200.0, 'bench', stand=0, servos_off=True)          # clipping
    v = L.CMD.unpack(bytes(fs.tx)[22 + 3:22 + 21])
    assert v[2:] == (4, 2, k.seq, 10000, -10000, 2000, 15000, 0, 0), v
    print('PASS  command frame: AA 55 12 + 18-byte CmdPacket + CRC-8, units, mode, flags, clipping')

    t = firmware_bytes(SAMPLE)
    bad = bytearray(L.frame(t)); bad[10] ^= 0xFF                                  # corrupted frame must be dropped
    fs.rx += b'\x00\xAA\x13' + bytes(bad) + L.frame(t)
    k.sent_at[SAMPLE['last_cmd_seq']] = time.perf_counter() - 0.020                # pretend we sent it 20 ms ago
    deadline = time.time() + 2
    while k.state() is None and time.time() < deadline:
        time.sleep(0.01)
    s = k.state()
    assert s is not None, 'state frame not parsed'
    assert k.n_bad >= 1, 'corrupted frame was not rejected'
    assert (s.seq, s.t_robot_ms, s.mode, s.flags) == (65000, 987654, 3, 0x0965)
    assert abs(s.pitch_deg + 25.0) < 1e-9 and abs(s.pitch_rate_dps + 123.4) < 1e-9 and abs(s.yaw_rate_dps - 56.7) < 1e-9
    assert abs(s.wheel_l_rads + 15.2) < 1e-9 and abs(s.wheel_r_rads - 20.47) < 1e-9
    assert abs(s.tail_cmd_deg - 73.6) < 1e-9 and s.head_cmd_deg == 90 and abs(s.vbat - 7.812) < 1e-9
    assert abs(s.u_left + 0.3333) < 1e-9 and s.u_right == 1.0 and s.edges_l == 65535 and s.cmd_lost == 3
    assert s.flag('failsafe') and s.flag('policy') and s.flag('link') and not s.flag('tilt_cut') and s.mode_name == 'onboard'
    assert len(k.rtt_ms) == 1 and 10 < k.rtt_ms[0] < 20, k.rtt_ms                 # 20 ms minus cmd_age 7 ms
    print('PASS  state frame: resyncs past junk, rejects a bad CRC, 42-byte StatePacket unpacks to the expected values')

    with tempfile.TemporaryDirectory() as d:
        for ext in ('.npz', '.csv'):
            p = pathlib.Path(d, 'log' + ext)
            k.start_log(p); k.send(0.1, 0.2, 100, 90); k._log['state'].append(s); k.stop_log()
            st, cm = L.load_log(p)
            assert abs(st['pitch_deg'][0] + 25) < 1e-9 and abs(cm['u_right'][0] - 0.2) < 1e-9 and len(cm['t']) == 1
    k.close()
    print('PASS  logging: .npz and .csv round trip')


def check_obs():
    s = L.unpack_state(firmware_bytes(SAMPLE))
    o = L.obs_step(s, [0.1, -0.2, 0.3], (1, 0.15, -1.0))
    want = [np.deg2rad(-25), np.deg2rad(-123.4) / 5, np.deg2rad(56.7) / 5, -15.2 / 20, 20.47 / 20, 0.3, 0.1, -0.2, 0.3,
            (7.812 - 7.6) / 0.8, 1.0, 0.5, -0.5]
    assert o.shape == (13,) and np.allclose(o, want, atol=1e-6), o
    o2 = L.obs_step(s, [0.1, -0.2, 0.3], (1, 0.15, -1.0), prev_action2=[0.4, 0.5, -0.6])
    assert np.allclose(o2[5:9], [0.3, 0.4, 0.5, -0.6]), 'sim layout: [5] last tail action, [6..8] the action before'
    st = L.ObsStack()
    a = st.push(o); assert a.shape == (52,) and np.allclose(a, np.tile(o, 4))
    o2 = o + 1; b = st.push(o2)
    assert np.allclose(b[:13], o2) and np.allclose(b[13:], np.tile(o, 3)), 'stack must be newest first'
    print('PASS  obs_step: 13 floats in the documented order and scaling; ObsStack is 52 floats, newest first')


def check_policy_format():
    if not compiler():
        print('SKIP  policy.h format probe (no compiler found)'); return
    rng = np.random.default_rng(0)
    W0, B0 = rng.normal(0, .3, (64, 52)), rng.normal(0, .1, 64)
    W1, B1 = rng.normal(0, .2, (64, 64)), rng.normal(0, .1, 64)
    W2, B2 = rng.normal(0, .5, (3, 64)), rng.normal(0, .1, 3)
    mean, std = rng.normal(0, 1, 52), rng.uniform(.5, 2, 52)
    x = rng.normal(0, 1, 52)
    arr = lambda a: '{' + ','.join((arr(r) if np.ndim(r) else f'{r:.9g}f') for r in a) + '}'
    hdr = ('#pragma once\n#define TW_OBS_DIM 52\n#define TW_HID 64\n#define TW_ACT_DIM 3\n'
           + ''.join(f'static const float {n}{dims} = {arr(v)};\n' for n, dims, v in
                     [('TW_W0', '[TW_HID][TW_OBS_DIM]', W0), ('TW_B0', '[TW_HID]', B0), ('TW_W1', '[TW_HID][TW_HID]', W1),
                      ('TW_B1', '[TW_HID]', B1), ('TW_W2', '[TW_ACT_DIM][TW_HID]', W2), ('TW_B2', '[TW_ACT_DIM]', B2),
                      ('TW_OBS_MEAN', '[TW_OBS_DIM]', mean), ('TW_OBS_STD', '[TW_OBS_DIM]', std)]))
    # The same loops as policyStep() in tailwhip_robot.ino.
    src = f'''#include <cstdio>
#include <cmath>
#include "policy.h"
static float clampf(float v) {{ return v < -1 ? -1 : v > 1 ? 1 : v; }}
int main() {{
  float in[52] = {arr(x)}, xn[52], h0[64], h1[64];
  for (int i = 0; i < TW_OBS_DIM; i++) xn[i] = (in[i] - TW_OBS_MEAN[i]) / TW_OBS_STD[i];
  for (int j = 0; j < TW_HID; j++) {{ float s = TW_B0[j]; for (int i = 0; i < TW_OBS_DIM; i++) s += TW_W0[j][i] * xn[i]; h0[j] = tanhf(s); }}
  for (int j = 0; j < TW_HID; j++) {{ float s = TW_B1[j]; for (int i = 0; i < TW_HID; i++) s += TW_W1[j][i] * h0[i]; h1[j] = tanhf(s); }}
  for (int j = 0; j < TW_ACT_DIM; j++) {{ float s = TW_B2[j]; for (int i = 0; i < TW_HID; i++) s += TW_W2[j][i] * h1[i]; printf("%.6f\\n", clampf(s)); }}
}}'''
    with tempfile.TemporaryDirectory() as d:
        pathlib.Path(d, 'policy.h').write_text(hdr)
        out = [float(v) for v in run_cpp(src, d) if v.strip()]
    xn = (x - mean) / std
    a = np.clip(W2 @ np.tanh(W1 @ np.tanh(W0 @ xn + B0) + B1) + B2, -1, 1)
    assert np.allclose(out, a, atol=1e-4), (out, a)
    # The shipped policy.h is either the placeholder (marks itself) or a sim/export_policy.py export (right sizes).
    ph = (FW / 'tailwhip_robot' / 'policy.h').read_text()
    assert 'TW_POLICY_PLACEHOLDER' in ph or ('#define TW_OBS_DIM 52' in ph and '#define TW_HID 64' in ph
                                             and '#define TW_ACT_DIM 3' in ph and 'TW_W2' in ph)
    print('PASS  policy.h format: W[out][in] row-major, (x - mean) / std, tanh, tanh, linear, clip: C++ == numpy')


def main():
    check_copies()
    check_with_compiler()
    check_link()
    check_obs()
    check_policy_format()
    print('all protocol checks passed')


if __name__ == '__main__':
    main()
