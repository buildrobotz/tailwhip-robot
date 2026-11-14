"""Drop tests in MuJoCo with the motors off: does every fall end on the feet or on the tail, never flat?

    python sim/test_falls.py

Each case starts upright (or nearly) with a push, the tail held at a given angle by its servo, motors OFF, 4 s.
PASS if the final pitch is within 50 deg of vertical. Two cases are expected to roll over: they mark the passive
limit, beyond which the wheels have to catch the fall (the policy's job). This is the check that found the rev A heel/chin problem.
"""
import os, sys
import numpy as np
import mujoco
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'cad'))
import tailwhip as W

def run(m, d, pitch0, rate0, tail_deg):
    mujoco.mj_resetData(m, d)
    p = np.radians(pitch0)
    d.qpos[3:7] = [np.cos(p / 2), 0, np.sin(p / 2), 0]; d.qpos[2] = W.R_WHEEL / 1000 + 0.0002
    t = np.radians(tail_deg)
    d.qpos[m.joint('tail').qposadr[0]] = t; d.ctrl[2] = t; d.qvel[4] = rate0
    peak = 0.0
    for _ in range(2000):
        mujoco.mj_step(m, d)
        R = d.xmat[1].reshape(3, 3); peak = max(peak, abs(np.degrees(np.arctan2(R[0, 2], R[2, 2]))))
    R = d.xmat[1].reshape(3, 3)
    return np.degrees(np.arctan2(R[0, 2], R[2, 2])), peak

def main():
    m = mujoco.MjModel.from_xml_path(os.path.join(HERE, 'tailwhip.xml')); d = mujoco.MjData(m)
    ready = W.tail_ready()          # == the rest angle: the tail waits there while it balances
    cases = [('topple back, tail at ready/rest', -3, -0.5, ready, True), ('shoved back, 4 rad/s', 0, -4.0, ready, True),
             ('topple forward', 3, 0.5, ready, True), ('shoved forward, 4 rad/s', 0, 4.0, ready, True),
             ('shoved forward, 6 rad/s', 0, 6.0, ready, True),
             ('shoved back, 5 rad/s (beyond the passive limit)', 0, -5.0, ready, False),
             ('topple back, tail STOWED up the back (do not do this)', -3, -0.5, 150.0, False)]
    ok_all = True
    print(f'{"case":52s} {"final":>7s} {"peak":>6s}')
    for name, p0, r0, tail, should in cases:
        fin, pk = run(m, d, p0, r0, tail)
        ok = abs(fin) < 50
        expect_fail = not should
        ok_all &= ok or expect_fail
        print(f'{name:52s} {fin:7.1f} {pk:6.1f}  {"PASS" if ok else ("expected: rolls over (motors off)" if expect_fail else "FAIL")}')
    print('ALL PASS' if ok_all else 'SOME FAILED')
    return ok_all

if __name__ == '__main__':
    sys.exit(0 if main() else 1)
