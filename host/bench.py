"""Bench measurements from SPEC section 9, one subcommand each. Results go to host/measurements.json, which the
simulator reads. Each run updates only its own keys, and records where each number came from.

    python bench.py motor PORT      TT motor speed at 3, 5 and 6.5 V, currents (meter), stall torque, dead zone
    python bench.py backlash        gearbox backlash, by hand with a ruler
    python bench.py servo PORT      SG-90 60 deg move time: unloaded (phone slow-motion) and pushing the robot (IMU)
    python bench.py radio PORT      ESP-NOW round trip and packet loss at 50 and 100 Hz, and the loop delay
    python bench.py mass            total mass and centre-of-mass height (scale, balance on an edge)
    python bench.py friction        tyre friction on your demo surface (tilt a board)
    python bench.py show            print measurements.json

Keys in measurements.json (all SI-ish units in the key name; every key also has a "<key>_source" entry):
  motor_no_load_rad_s_at_6v    wheel speed with no load at 6.0 V, rad/s (fitted through the 3 / 5 / 6.5 V points)
  motor_stall_torque_nm_at_6v  wheel torque with the wheel held, N m (lever and scale, or from the meter currents)
  motor_deadzone_u             smallest |u| (u = 1 is 6 V) that moves a loaded wheel (robot resting on its tail)
  backlash_deg                 free play at the wheel, deg
  servo_deg_per_s              SG-90 speed with no load, deg/s (servo_deg_per_s_loaded: pushing the robot)
  loop_delay_ms                sensor sample -> wheel command applied, remote mode at 50 Hz, median, ms
  packet_loss                  fraction of commands lost at 50 Hz (packet_loss_detail: both rates, both directions)
  mass_g, com_height_mm        whole robot; centre of mass above the wheel axle along the body
  tyre_mu                      static friction coefficient, tyre on your surface
"""
import argparse, datetime, json, math, pathlib, statistics, sys, time
import numpy as np

OUT = pathlib.Path(__file__).with_name('measurements.json')
V_BENCH = 8.4                    # bench mode: u = 1 means 8.4 V at the motor
V_U = 6.0                        # everywhere else: u = 1 means 6.0 V
WHEEL_D_MM = 65.0


def load():
    return json.loads(OUT.read_text()) if OUT.exists() else {}


def save(upd, source):
    d = load()
    stamp = datetime.date.today().isoformat()
    for k, v in upd.items():
        d[k] = v
        if not k.endswith('_detail'):
            d[k + '_source'] = f'measured {stamp}: {source}'
    OUT.write_text(json.dumps(dict(sorted(d.items())), indent=1))
    print(f'\nwrote {", ".join(upd)} to {OUT}')


def ask_float(prompt, allow_blank=False):
    while True:
        s = input(prompt + ' ').strip()
        if not s and allow_blank:
            return None
        try:
            return float(s)
        except ValueError:
            print('  a number, please')


def connect(port):
    from link import Link
    link = Link(port)
    s = link.wait_state()
    print(f'robot: mode {s.mode_name}, {s.vbat:.2f} V, pitch {s.pitch_deg:+.1f} deg, flags 0x{s.flags:04X}')
    return link


def hold(link, seconds, u_l, u_r, tail=73.6, head=90.0, mode='bench', rate=50):
    """Send the same command at `rate` Hz for `seconds`; return the states received meanwhile."""
    out, t0, n = [], time.perf_counter(), 0
    last = None
    while time.perf_counter() - t0 < seconds:
        link.send(u_l, u_r, tail, head, mode)
        s = link.state()
        if s is not None and s is not last:
            out.append(s); last = s
        n += 1
        time.sleep(max(0.0, t0 + n / rate - time.perf_counter()))
    return out


# ------------------------------------------------------------------------------------------------- motor
def cmd_motor(a):
    link = connect(a.port)
    print('\nTT motor test. Put the robot on a stand (a mug under the core) so BOTH WHEELS SPIN FREELY.')
    print('Meter: put it in series with the battery red lead, 10 A range, if you want currents (Enter to skip each).')
    input('Ready? Enter to start, Ctrl-C to stop at any time.')
    pts = []
    try:
        for V in (3.0, 5.0, 6.5):
            u = V / V_BENCH
            st = hold(link, 3.0, u, u)
            tail = [s for s in st if s.t_rx > st[-1].t_rx - 1.5]
            wl = float(np.mean([s.wheel_l_rads for s in tail])); wr = float(np.mean([s.wheel_r_rads for s in tail]))
            vb = float(np.mean([s.vbat for s in tail]))
            print(f'{V:.1f} V (pack {vb:.2f} V): left {wl:.2f} rad/s, right {wr:.2f} rad/s')
            hold(link, 0.5, u, u)
            i_nl = ask_float(f'  meter current now, wheels free at {V} V (A)?', True) if a.currents else None
            pts.append(dict(volts=V, vbat=vb, speed_l=wl, speed_r=wr, current_noload_A=i_nl))
            hold(link, 0.8, 0, 0)
        hold(link, 0.5, 0, 0)
        # Stall: short, at 6 V only. The motor takes 1.5 A [D Adafruit 3777]; keep it under 2 s.
        stall_I = stall_T = None
        if input('\nStall test at 6.0 V for 2 s (hold the LEFT wheel still with your fingers)? [y/N] ').lower().startswith('y'):
            hold(link, 2.0, V_U / V_BENCH, 0)
            hold(link, 0.3, 0, 0)
            stall_I = ask_float('  stall current from the meter (A, Enter to skip)?', True)
            print('  Torque, if you have a kitchen scale: tape a stick to the wheel, radius L from the axle to where it')
            print('  presses on the scale. The test runs 2 s at 6 V again; read the peak.')
            if input('  do the lever test? [y/N] ').lower().startswith('y'):
                L_mm = ask_float('  lever length L (mm)?')
                input('  stick resting on the scale, tare it, Enter to run')
                hold(link, 2.0, V_U / V_BENCH, 0)
                hold(link, 0.3, 0, 0)
                g = ask_float('  peak scale reading (g)?')
                stall_T = g / 1000 * 9.81 * L_mm / 1000
        # Dead zone with load: robot resting on its tail on the floor, ramp the voltage until edges appear.
        print('\nDead zone. Put the robot on the floor resting on its tail (normal rest pose), 1 m clear in front.')
        input('Enter to ramp the voltage slowly (both wheels) until each wheel moves.')
        dz = [None, None]
        e0 = link.state(); base = [e0.edges_l, e0.edges_r]
        for V in np.arange(0.2, 4.0, 0.05):
            hold(link, 0.15, V / V_BENCH, V / V_BENCH, tail=73.6)
            s = link.state()
            for k, e in enumerate((s.edges_l, s.edges_r)):
                if dz[k] is None and ((e - base[k]) & 0xFFFF) >= 2:
                    dz[k] = V / V_U; print(f'  {"left" if k == 0 else "right"} wheel moved at {V:.2f} V (u = {dz[k]:.3f})')
            if None not in dz:
                break
        hold(link, 0.5, 0, 0)
    finally:
        link.idle(); link.close()

    V = np.array([p['volts'] for p in pts]); w = np.array([(abs(p['speed_l']) + abs(p['speed_r'])) / 2 for p in pts])
    k, c = np.polyfit(V, w, 1)
    w6 = float(k * 6.0 + c)
    upd = dict(motor_no_load_rad_s_at_6v=round(w6, 2),
               motor_detail=dict(points=pts, fit=f'speed = {k:.3f} x V + {c:.3f} rad/s', stall_current_A_at_6v=stall_I))
    src = 'bench.py motor'
    if stall_T is None and stall_I and pts[-1]['current_noload_A']:
        # DC motor estimate: R = 6 / I_stall, k_e = (V - I_nl R) / w_nl at the wheel, T_stall = k (I_stall - I_nl)
        i_nl = statistics.mean(p['current_noload_A'] for p in pts if p['current_noload_A'])
        R = 6.0 / stall_I; ke = (6.0 - i_nl * R) / w6
        stall_T = ke * (stall_I - i_nl)
        src += ' (stall torque estimated from the meter currents, not a lever)'
    if stall_T is not None:
        upd['motor_stall_torque_nm_at_6v'] = round(stall_T, 4)
    valid = [d for d in dz if d is not None]
    if valid:
        upd['motor_deadzone_u'] = round(max(valid), 3)
    print(f'\nno-load speed at 6 V: {w6:.1f} rad/s ({w6 * 60 / 2 / math.pi:.0f} rpm)   stall torque: {stall_T}   dead zone u: {dz}')
    save(upd, src)


# ------------------------------------------------------------------------------------------------- backlash
def cmd_backlash(a):
    print('Gearbox backlash, measured at the tyre. Robot switched ON and in idle (motors coast is fine too).')
    print('1. Put a strip of tape on the tyre tread and a ruler or paper scale beside it, touching the tread.')
    print('2. With one finger, push the tyre gently forward until you feel the gears take up, note the mark.')
    print('3. Push gently backward until the gears take up again. The distance between the two is the free play.')
    print('   Light force only: you want the free play, not the motor turning. Do each wheel 3 times.')
    vals = []
    for wheel in ('left', 'right'):
        for i in range(3):
            vals.append(ask_float(f'{wheel} wheel, try {i + 1}: free play along the tread (mm)?'))
    mm = statistics.median(vals)
    deg = mm / (math.pi * WHEEL_D_MM) * 360
    print(f'median {mm:.1f} mm = {deg:.1f} deg of wheel rotation')
    save(dict(backlash_deg=round(deg, 2), backlash_detail=dict(tread_mm=vals)), 'bench.py backlash (ruler at the tread)')


# ------------------------------------------------------------------------------------------------- servo
def cmd_servo(a):
    link = connect(a.port)
    upd = {}
    try:
        print('\nUnloaded: robot on a stand, tail free in the air. Film the tail with a phone in slow motion (240 fps).')
        print('The tail steps 60 -> 120 deg, then back, three times, 1.5 s apart.')
        input('Start filming, then Enter.')
        for _ in range(3):
            hold(link, 1.5, 0, 0, tail=60); hold(link, 1.5, 0, 0, tail=120)
        hold(link, 1.0, 0, 0, tail=73.6)
        print('In the video, count frames from the first frame the tail moves to the first frame it is still.')
        fps = ask_float('video frame rate (fps, e.g. 240)?', True)
        if fps:
            fr = [ask_float(f'move {i + 1}: frames?') for i in range(3)]
            t = statistics.median(fr) / fps
            upd['servo_deg_per_s'] = round(60 / t, 1)
            print(f'unloaded: {t * 1e3:.0f} ms per 60 deg = {60 / t:.0f} deg/s')

        print('\nLoaded: robot on the floor. The tail goes to 103.6 deg (body sinks back to about 42 deg, still on its')
        print('foot), then steps 30 deg to 73.6 (rest), pushing the body up. The IMU times how long the body takes to settle.')
        if input('Do the loaded test? [y/N] ').lower().startswith('y'):
            res = []
            for _ in range(3):
                hold(link, 2.0, 0, 0, tail=103.6)
                t0 = time.perf_counter()
                st = hold(link, 2.0, 0, 0, tail=73.6)
                p = np.array([s.pitch_deg for s in st]); t = np.array([s.t_rx - t0 for s in st])
                if len(p) < 20 or abs(p[-1] - p[0]) < 3:
                    print('  body hardly moved: skipped'); continue
                frac = (p - p[0]) / (p[-1] - p[0])
                t10, t90 = t[np.argmax(frac > 0.1)], t[np.argmax(frac > 0.9)]
                res.append(t90 / 0.9)
                print(f'  pitch {p[0]:+.1f} -> {p[-1]:+.1f} deg, 90 % reached at {t90 * 1e3:.0f} ms (10 % at {t10 * 1e3:.0f} ms)')
            if res:
                tl = statistics.median(res)
                upd['servo_deg_per_s_loaded'] = round(30 / tl, 1)
                upd['servo_detail'] = dict(loaded_move_s=res, note='loaded time includes radio delay (~10-20 ms)')
    finally:
        link.idle(); link.close()
    if upd:
        save(upd, 'bench.py servo')


# ------------------------------------------------------------------------------------------------- radio
def cmd_radio(a):
    link = connect(a.port)
    detail, loop50 = {}, None
    try:
        for rate in (50, 100):
            s0 = link.state(); lost0 = s0.cmd_lost; rx0 = link.n_rx
            link.rtt_ms.clear()
            n0 = link.seq
            t0, n = time.perf_counter(), 0
            print(f'{rate} Hz for {a.seconds:.0f} s ...')
            while time.perf_counter() - t0 < a.seconds:
                link.send(0, 0, 73.6, 90, 'idle')
                n += 1
                time.sleep(max(0.0, t0 + n / rate - time.perf_counter()))
            time.sleep(0.2)
            s1 = link.state()
            sent = (link.seq - n0) & 0xFFFF
            down = ((s1.cmd_lost - lost0) & 0xFFFF) / max(sent, 1)
            span = (s1.seq - s0.seq) & 0xFFFF                 # state packets the robot sent meanwhile
            up = 1 - (link.n_rx - rx0) / max(span, 1)
            r = np.array(link.rtt_ms) if link.rtt_ms else np.array([np.nan])
            detail[f'{rate}hz'] = dict(commands_sent=sent, down_loss=round(down, 4), up_loss=round(max(up, 0.0), 4),
                                       rtt_ms_median=round(float(np.nanmedian(r)), 2), rtt_ms_p95=round(float(np.nanpercentile(r, 95)), 2))
            print('  ', detail[f'{rate}hz'])
            if rate == 50:
                # Loop delay for a remote policy: state age (average half a 10 ms state period) + round trip + half a
                # 2.5 ms robot tick. Laptop compute time is not included: add yours.
                loop50 = float(np.nanmedian(r)) + 5.0 + 1.25
    finally:
        link.idle(); link.close()
    save(dict(loop_delay_ms=round(loop50, 1), packet_loss=detail['50hz']['down_loss'],
              packet_loss_detail=detail), 'bench.py radio (idle robot, robot and bridge 1-2 m apart)')


# ------------------------------------------------------------------------------------------------- mass, friction
def cmd_mass(a):
    print('Mass: the whole robot, battery in, on a kitchen scale.')
    m = ask_float('mass (g)?')
    print('\nCentre of mass: lay the robot on its side with the body horizontal and slide it along a ruler edge (or a')
    print('pencil) under the body until it balances. Measure from the centre of the wheel axle to the balance line,')
    print('along the body (towards the head is positive). Do it 3 times.')
    h = [ask_float(f'try {i + 1}: axle to balance line (mm)?') for i in range(3)]
    save(dict(mass_g=m, com_height_mm=round(statistics.median(h), 1)), 'bench.py mass (kitchen scale, balance on an edge)')


def cmd_friction(a):
    print('Tyre friction on the surface you will demo on. Use a board covered with that surface (or the floor tile).')
    print('Stand the robot on the board SIDEWAYS (axle pointing down the slope) so the wheels cannot roll, tail stowed')
    print('or resting. Raise one end slowly until the robot slides. Measure the angle (phone level app), or the height')
    print('of the raised end and the board length. Do it 3 times.')
    mus = []
    for i in range(3):
        ang = ask_float(f'try {i + 1}: angle in degrees (Enter to give height and length instead)?', True)
        if ang is None:
            hgt = ask_float('  raised height (mm)?'); L = ask_float('  board length (mm)?')
            ang = math.degrees(math.asin(hgt / L))
        mus.append(math.tan(math.radians(ang)))
        print(f'  mu = {mus[-1]:.2f}')
    save(dict(tyre_mu=round(statistics.median(mus), 3), tyre_mu_detail=dict(tries=mus)),
         'bench.py friction (tilted board, sideways)')


def cmd_show(a):
    print(json.dumps(load(), indent=1) if OUT.exists() else f'{OUT} does not exist yet')


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('motor'); p.add_argument('port'); p.add_argument('--no-currents', dest='currents', action='store_false')
    sub.add_parser('backlash')
    p = sub.add_parser('servo'); p.add_argument('port')
    p = sub.add_parser('radio'); p.add_argument('port'); p.add_argument('--seconds', type=float, default=20)
    sub.add_parser('mass'); sub.add_parser('friction'); sub.add_parser('show')
    a = ap.parse_args()
    try:
        globals()['cmd_' + a.cmd](a)
    except KeyboardInterrupt:
        print('\nstopped'); sys.exit(1)
