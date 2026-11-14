"""Does the Tailwhip design hold together? Numerical checks on cad/tailwhip.py. Run: python analysis.py
Writes cad/out/analysis.md (tables + PASS/FAIL); also tail_torque.png if matplotlib is installed.

Every result inherits the tags of its inputs: anything computed from an [E] number is an estimate until the
bench measurements in SPEC.md section 9 are in.
"""
import os
import numpy as np
from manifold3d import Manifold
import tailwhip as W

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
G = 9.81
EPS = 1.0                    # mm3 of overlap treated as mesh noise
LINES, RESULTS = [], []

# ------------------------------------------------------------------------------------- bench-independent inputs
SG90_STALL = 0.18            # [D] N m, 1.8 kg cm at 4.8 V (TowerPro). Clones often manage 1.2-1.5 kg cm.
SG90_STALL_REAL = 0.13       # [E] what a clone SG-90 really gives at 5 V
TT_STALL = 0.08              # [E] N m per motor at ~6 V (0.8 kg cm, typical listings 0.8-1.0)
V_WHEEL = 0.68               # [E] m/s, 200 rpm at 6 V on a 65 mm wheel
MU = 0.8                     # [E] tyre on a wooden desk (SPEC section 6); randomised 0.4-1.0 in the simulator
E_ABS, S_ABS = 2000.0, 40.0  # [D] MPa, typical ABS modulus and tensile yield (printed parts ~70 % of that)


def say(*a):
    s = ' '.join(str(x) for x in a); print(s); LINES.append(s)

def result(name, ok, text):
    RESULTS.append((name, 'PASS' if ok else ('WARN' if ok is None else 'FAIL'), text))


def body_frame_items(its, alpha=0.0, pan=0.0):
    """All solids in the body frame for one tail angle and pan angle."""
    out = []
    for i in its:
        if i.solid is None:
            continue
        M = {'tail': W.tail_tf(alpha), 'head': W.head_tf(pan)}.get(i.group, np.eye(4))
        out.append((i, W.apply(i.solid, M)))
    return out

def vol(m):
    return 0.0 if m.is_empty() else m.volume()


# ============================================================================================ 1 mass and CoM
def mass_table(its):
    say('\n## 1. Mass and centre of mass (upright, tail straight down, head forward)\n')
    say('| Part | Group | Mass (g) | Source | CoM x, y, z (mm, body frame) |')
    say('|---|---|---|---|---|')
    tot, mom = 0.0, np.zeros(3)
    for i in its:
        m = W.item_mass(i)
        c = W.item_com(i)
        M = {'tail': W.tail_tf(0), 'head': W.head_tf(0)}.get(i.group, np.eye(4))
        c = (M @ np.append(c, 1))[:3]
        src = i.source or (f'[E] printed {i.material}, {W.FILL:.0%} of solid' if i.material else '')
        say(f'| {i.name} | {i.group} | {m:.1f} | {src} | {c[0]:.0f}, {c[1]:.0f}, {c[2]:.0f} |')
        tot += m; mom += m * c
    com = mom / tot
    printed = sum(W.item_mass(i) for i in its if i.material)
    say(f'| **Total** | | **{tot:.0f}** | printed {printed:.0f} g, bought {tot - printed:.0f} g | **{com[0]:.1f}, {com[1]:.1f}, {com[2]:.1f}** |')
    result('Total mass vs SPEC 250-270 g', None if tot > 270 else True,
           f'{tot:.0f} g [E]. Bought parts alone are {tot - printed:.0f} g, so the SPEC target was not reachable; '
           f'every torque below uses {tot:.0f} g')
    result('CoM height vs SPEC 70-90 mm', 70 <= com[2] <= 90, f'{com[2]:.0f} mm above the axle [E]')
    result('CoM fore-aft', abs(com[0]) < 3, f'x = {com[0]:.1f} mm: balances within {np.degrees(np.arctan2(abs(com[0]), com[2])):.1f} deg of vertical')
    return tot / 1000.0, com


# ============================================================================================ 2 interference
def interference(its):
    say('\n## 2. Interference (overlap volume between parts, mm3)\n')
    allowed = {frozenset(p) for p in [('holder', 'cells'), ('tyre_l', 'rim_l'), ('tyre_r', 'rim_r')]}
    shaft_ok = ('tt_shaft', 'rim', 'tt_gearbox', 'encoder_ring')
    bad = []
    for alpha, pan in ((25, 0), (74, 0), (150, 0), (74, 90), (74, -90)):
        S = body_frame_items(its, alpha, pan)
        for a in range(len(S)):
            for b in range(a + 1, len(S)):
                ia, ma = S[a]; ib, mb = S[b]
                if frozenset((ia.name, ib.name)) in allowed:
                    continue
                if any(ia.name.startswith(p) for p in shaft_ok) and any(ib.name.startswith(p) for p in shaft_ok) \
                        and ('tt_shaft' in ia.name or 'tt_shaft' in ib.name):
                    continue
                if {ia.name, ib.name} & {'tail_horn', 'tail_servo'} and {ia.name, ib.name} <= {'tail_horn', 'tail_servo'}:
                    continue
                if {ia.name, ib.name} <= {'pan_horn', 'pan_servo'} or {ia.name, ib.name} <= {'tail_horn', 'tail'} \
                        or {ia.name, ib.name} <= {'pan_horn', 'head'}:
                    continue
                v = vol(ma ^ mb)
                if v > EPS:
                    bad.append((alpha, ia.name, ib.name, v))
    if bad:
        say('| tail angle | part A | part B | overlap mm3 |'); say('|---|---|---|---|')
        for r in bad:
            say(f'| {r[0]} | {r[1]} | {r[2]} | {r[3]:.1f} |')
    else:
        say('No overlaps above 1 mm3 between any two parts at tail angles 25 (foot down, upright), 74 (rest) and 150 (stowed) deg, head 0 and +-90 deg.')
    result('Static interference', not bad, f'{len(bad)} overlapping pairs')

    # tail sweep against everything on the body
    body = W.union([m for i, m in body_frame_items(its) if i.group not in ('tail',) and i.name != 'tail_horn'
                    and not i.name.startswith(('tyre', 'rim', 'encoder', 'tt_shaft'))])
    wheels = W.union([m for i, m in body_frame_items(its) if i.name.startswith(('tyre', 'rim', 'encoder'))])
    tl = [i for i in its if i.group == 'tail' and i.name != 'tail_horn']
    free = []
    for a in range(-40, 211, 5):
        tm = W.union([W.apply(i.solid, W.tail_tf(a)) for i in tl])
        v = vol(tm ^ body) + vol(tm ^ wheels)
        free.append((a, v))
    ok = [a for a, v in free if v <= EPS]
    lo, hi = (min(ok), max(ok)) if ok else (999, -999)
    contiguous = all(v <= EPS for a, v in free if lo <= a <= hi)
    say(f'\nTail sweep, -40 to 210 deg in 5 deg steps: free from {lo} to {hi} deg' + ('' if contiguous else ' (with gaps!)'))
    hits = [f'{a}: {v:.0f}' for a, v in free if v > EPS]
    say('Blocked angles (deg: mm3): ' + (', '.join(hits) if hits else 'none'))
    result('Tail range of motion', lo <= 20 and hi >= 150 and contiguous, f'free {lo} to {hi} deg (needs 20 to 150: foot on the floor when upright is ~24 deg, stowed up the back ~150)')

    # head pan sweep
    hd = [i for i in its if i.group == 'head']
    body2 = W.union([m for i, m in body_frame_items(its) if i.group == 'body' and i.name != 'pan_servo'])
    hits = []
    for p in range(-90, 91, 10):
        hm = W.union([W.apply(i.solid, W.head_tf(p)) for i in hd])
        v = vol(hm ^ body2)
        if v > EPS:
            hits.append((p, v))
    say(f'Head pan sweep -90 to 90 deg: {"free" if not hits else hits}')
    result('Head pan range', not hits, 'free over +-90 deg' if not hits else str(hits))
    return lo, hi


# ============================================================================================ 3 rest angles
def rest_angles(its, com):
    """Where it lands when it falls, and whether it STAYS there. First contact alone is not enough: the robot only
    rests on a bumper if the bumper touches down beyond the centre of mass (seen from the wheel's contact point);
    otherwise it rolls over the bumper. That is what the MuJoCo model showed for the rev A heel and chin."""
    say('\n## 3. Rest angles (where the robot lies when it falls, and whether it stays there)\n')
    ready = W.tail_ready()
    pts, tpts = [], []
    for i, m in body_frame_items(its, alpha=ready):
        if i.group in ('body', 'head') and not i.name.startswith(('tyre', 'rim')):
            v, _ = W.mesh_np(m); pts.append(v)
        if i.group == 'tail':
            v, _ = W.mesh_np(m); tpts.append(v)
    P = np.vstack(pts)
    PT = np.vstack(pts + tpts)

    def hull_rest(sign, P=P):
        """Roll the body over in 0.1 deg steps; each time a new point touches the floor, check whether the
        centre of mass is between the wheel contact and that point. Return the first stable angle."""
        for th in np.arange(0, 120.01, 0.1):
            p = np.radians(sign * th)
            z = -P[:, 0] * np.sin(p) + P[:, 2] * np.cos(p)
            if z.min() <= -W.R_WHEEL:
                k = np.argmin(z)
                xc = P[k, 0] * np.cos(p) + P[k, 2] * np.sin(p)            # horizontal, from the axle
                xg = com[0] * np.cos(p) + com[2] * np.sin(p)
                if abs(xc) > abs(xg) and np.sign(xc) == np.sign(xg):
                    return th, P[k], xc, xg
        return None
    rows = []
    for name, sign, pp in ((f'back, tail at its ready angle {ready:.0f} deg', -1, PT), ('forward', +1, P)):
        r = hull_rest(sign, pp)
        th, q, xc, xg = r
        rows.append(th)
        say(f'- Falling {name}: stops at **{th:.1f} deg**, on the point x {q[0]:.0f}, z {q[2]:.0f} mm. That point is '
            f'{abs(xc):.0f} mm from the axle horizontally, the centre of mass {abs(xg):.0f} mm: '
            f'{abs(xc) - abs(xg):.0f} mm of margin.')
    say('\nRev A relied on the heel and chin on the skid. They touch first (about 37 deg), but the MuJoCo model showed a '
        'real fall carries the body straight over them, and over any bumper close to the centre of mass. Rev B stops a '
        'forward fall on long low feet at the front and a backward fall on the tail, held at its ready angle while '
        'balancing (the failsafe moves it to the rest angle, which catches even earlier). The fall tests in '
        '`sim/test_falls.py` check this dynamically.')
    back, front = rows
    result('Back fall lands on the tail, stable', back <= 50, f'{back:.1f} deg')
    result('Forward fall lands on the feet, stable', front <= 50, f'{front:.1f} deg')
    return back, front, P


def tail_rest(P, back):
    say('\n### Tail-down rest\n')
    rows = []
    for th in (25.0, 27.5, 30.0):
        a = W.tail_angle_for_foot_on_floor(-th)
        # heel clearance at that pitch
        z = -P[:, 0] * np.sin(np.radians(-th)) + P[:, 2] * np.cos(np.radians(-th)) + W.R_WHEEL
        rows.append((th, a, z.min()))
    say('| rest pitch (deg back) | tail command (deg) | lowest body point above floor (mm) |')
    say('|---|---|---|')
    for th, a, c in rows:
        say(f'| {th} | {a:.1f} | {c:.1f} |')
    REST = 25.0
    a = W.tail_angle_for_foot_on_floor(-REST)
    clr = rows[0][2]
    result('Tail rest 25 deg clears the heel', clr > 3, f'tail command {a:.1f} deg, heel {clr:.1f} mm off the floor')
    return REST, a


# ============================================================================================ 4 rear-up torque
def rear_up(mass, com, rest):
    """Quasi-static rear-up from the tail rest to upright with the wheels free to roll and the foot not slipping.
    Free wheels carry only vertical force through the axle, so moments about the axle give the foot force, and the
    servo torque is that vertical force times its lever about the tail pivot."""
    say('\n## 4. Tail servo torque to rear up (quasi-static, wheels free, foot pinned)\n')
    rows = []
    mg = mass * G
    for th in np.arange(rest, -0.01, -2.5):
        p = -th
        a = W.tail_angle_for_foot_on_floor(p)
        if a is None:
            break
        R = W.Ry(p)
        c = (R @ np.append(com, 1))[:3]
        foot = (R @ np.append(W.foot_centre_body(a), 1))[:3]
        piv = (R @ np.array([W.TAIL_PIVOT[0], 0, W.TAIL_PIVOT[1], 1]))[:3]
        if foot[0] >= -1e-6:
            break
        N = mg * (c[0]) / (foot[0])           # both negative behind the axle
        tau = N * (piv[0] - foot[0]) / 1000.0
        rows.append((th, a, N, tau, -c[0]))
    say('| pitch back (deg) | tail cmd (deg) | CoM behind axle (mm) | foot load (N) | servo torque (N m) |')
    say('|---|---|---|---|---|')
    for th, a, N, tau, cx in rows:
        say(f'| {th:.1f} | {a:.1f} | {cx:.1f} | {N:.2f} | {tau:.3f} |')
    tmax = max(r[3] for r in rows)
    naive = mg * com[2] / 1000 * np.sin(np.radians(rest))
    say(f'\nPeak {tmax:.3f} N m at the start ({rest:.0f} deg, caught on the tail). For comparison, holding the body at {rest:.0f} deg about the axle '
        f'(m g h sin th, the SPEC section 6 formula) needs {naive:.3f} N m: putting the tail pivot {abs(W.TAIL_PIVOT[0]):.0f} mm '
        f'behind and {W.TAIL_PIVOT[1]:.0f} mm above the axle cuts the servo torque to {tmax / naive:.0%} of that, '
        'because the foot force acts on a short lever about the pivot while it acts on a long one about the axle.')
    say(f'SG-90 stall: {SG90_STALL} N m on the datasheet [D], about {SG90_STALL_REAL} N m for a clone [E]. '
        f'Margin over the real figure: {SG90_STALL_REAL / tmax:.1f}x. Driving the wheels backwards while rearing '
        'up adds to this; the policy will learn that.')
    t25 = max(r[3] for r in rows if r[0] <= 25.01)
    say(f'From the tail rest (25 deg) the peak is **{t25:.3f} N m** ({SG90_STALL_REAL / t25:.1f}x margin on a clone SG-90). '
        f'From a ready-angle catch ({rest:.0f} deg) it is {tmax:.3f} N m ({SG90_STALL_REAL / tmax:.1f}x): there the wheels must drive '
        'backwards under the body while the tail pushes, or the MG90S (about 0.18-0.22 N m) is needed.')
    result('Tail servo can rear up from the tail rest', SG90_STALL_REAL / t25 >= 1.5,
           f'peak {t25:.3f} N m vs {SG90_STALL_REAL} N m clone stall ({SG90_STALL_REAL / t25:.1f}x margin)')
    result('Tail alone, up from the ready-angle catch', None if SG90_STALL_REAL / tmax < 1.5 else True,
           f'{tmax:.3f} N m ({SG90_STALL_REAL / tmax:.1f}x)' + (': needs the wheels to help, or an MG90S' if SG90_STALL_REAL / tmax < 1.5 else ''))
    return rows


# ============================================================================================ 5 other physics
def physics(mass, com, back, front):
    say('\n## 5. Wheels, catching and getting up off the chin\n')
    h = com[2] / 1000.0
    r = W.R_WHEEL / 1000.0
    mg = mass * G
    hold = mg * h
    trac = MU * mg * r
    say(f'- Torque to hold the body at lean th about the axle: m g h sin th = **{hold:.3f} sin th N m** '
        f'(SPEC estimated 0.21; heavier and higher now).')
    say(f'- Wheel torque before the tyres slip (both wheels): mu m g r = {trac:.3f} N m at mu {MU}; two TT motors stall at '
        f'about {2 * TT_STALL:.2f} N m [E], so traction, not the motors, is the limit.')
    lim = np.degrees(np.arcsin(min(1, trac / hold)))
    say(f'- Steady lean the wheels alone can hold: about {lim:.0f} deg (traction-limited). Beyond that the tail or a bumper takes over.')
    catch = np.degrees(V_WHEEL / np.sqrt(G * h))
    say(f'- Rough catch bound v / sqrt(g h) with v = {V_WHEEL} m/s: {catch:.0f} deg on paper, about {catch / 2:.0f} deg after '
        'backlash and motor lag (the halving is a rule of thumb).')
    # chin recovery: lift off the toe needs the wheels to accelerate the axle under the CoM
    th = np.radians(front)
    tanphi = h * np.sin(th) / (r + h * np.cos(th))
    say(f'- Getting up off the front feet at {front:.0f} deg: the axle must accelerate at about g tan(phi) = '
        f'{G * tanphi:.1f} m/s2 to lift the feet, which needs tyre friction mu >= {tanphi:.2f}. '
        + ('Fine on a desk (mu ~0.8), marginal on slick floors.' if tanphi < 0.6 else 'Marginal on a desk; the policy has to find a rocking manoeuvre.'))
    result('Front-feet recovery friction', tanphi <= 0.6, f'needs mu >= {tanphi:.2f} (assumed desk mu {MU})')
    th = np.radians(back)
    tanphi_b = h * np.sin(th) / (r + h * np.cos(th))
    say(f'- Off the tail at {back:.0f} deg with the wheels alone: mu >= {tanphi_b:.2f}. With the tail it first '
        'pushes up to the tail rest (section 4).')


def leaf(mass):
    say('\n## 6. Tail leaf spring\n')
    b, t, Lf = W.LEAF_W, W.LEAF_T, np.pi * 0.5 * abs(W.LEAF_Z0 - W.LEAF_Z1) * 1.1
    I = b * t ** 3 / 12
    E = E_ABS * 0.85
    k = E * I / Lf / 1000.0              # N m / rad
    M = SG90_STALL                       # worst steady case: servo at stall
    stress = M * 1000 * (t / 2) / I
    defl = np.degrees(M / k)
    say(f'C-shaped ABS leaf, {b:.0f} mm wide x {t} mm thick, about {Lf:.0f} mm of arc. Bending stiffness about '
        f'**{k:.2f} N m/rad** [E]. At SG-90 stall torque ({M} N m) it bends {defl:.0f} deg with a peak stress of '
        f'{stress:.0f} MPa against roughly {S_ABS * 0.7:.0f} MPa for printed ABS (safety factor {S_ABS * 0.7 / stress:.1f}).')
    # landing: robot falls from upright onto the tail; energy m g dh absorbed by the leaf
    dh = (W.R_WHEEL + 75) - (W.R_WHEEL + 75 * np.cos(np.radians(25)))
    Eland = mass * G * dh / 1000.0
    th_land = np.sqrt(2 * Eland / k)
    M_land = k * th_land
    say(f'Falling from upright onto the tail drops the CoM about {dh:.0f} mm ({Eland * 1000:.0f} mJ). If the leaf took all '
        f'of it (worst case, nothing lost in the tyres or the gears) it would bend {np.degrees(th_land):.0f} deg and carry '
        f'{M_land:.2f} N m: that moment passes through the servo gears, about {M_land / SG90_STALL:.1f}x the stall torque, '
        'over roughly 30 ms instead of a few. This is why the MG90S is the planned fallback.')
    result('Leaf spring stress at servo stall', S_ABS * 0.7 / stress >= 2, f'{stress:.0f} MPa, SF {S_ABS * 0.7 / stress:.1f}')
    result('Landing load on the tail servo', None, f'{M_land:.2f} N m peak through the gears (worst case); watch for stripping, MG90S on hand')


def bed():
    say('\n## 7. Printed parts and the Ender 3 bed\n')
    say('| Part | Material | Count | Size x, y, z (mm) | Mass each (g) [E] | Print orientation |')
    say('|---|---|---|---|---|---|')
    allok = True
    for name, (fn, mat, n, note) in W.PRINTED.items():
        m = fn()
        bb = m.bounding_box()
        size = sorted([bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2]], reverse=True)
        ok = size[0] <= W.BED[0] and size[1] <= W.BED[1] and size[2] <= W.BED[2]
        allok &= ok
        g = m.volume() / 1000 * (W.PLA if mat == 'PLA' else W.ABS) * W.FILL
        say(f'| {name} | {mat} | {n} | {bb[3] - bb[0]:.0f} x {bb[4] - bb[1]:.0f} x {bb[5] - bb[2]:.0f} | {g:.1f} | {note} |')
    result('Every part fits the 220 x 220 x 250 bed', allok, '')


def dims(its):
    say('\n## 8. Overall size\n')
    S = body_frame_items(its, 0, 0)
    bbs = np.array([m.bounding_box() for _, m in S])
    lo, hi = bbs[:, :3].min(0), bbs[:, 3:].max(0)
    say(f'Upright: {hi[0] - lo[0]:.0f} mm front to back, {hi[1] - lo[1]:.0f} mm wide (track {W.TRACK:.0f} mm), '
        f'{hi[2] + W.R_WHEEL:.0f} mm tall from the floor to the top of the head.')
    result('Height vs SPEC 22-28 cm', 220 <= hi[2] + W.R_WHEEL <= 280, f'{hi[2] + W.R_WHEEL:.0f} mm')


def torque_plot(rows):
    try:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    except ImportError:
        return
    th = [r[0] for r in rows]; tau = [r[3] for r in rows]
    fig, ax = plt.subplots(figsize=(6.4, 3.6), dpi=150)
    ax.plot(th, tau, color='#1f6f8b', lw=2, label='tail servo, wheels free')
    ax.axhline(SG90_STALL_REAL, color='#c0392b', ls='--', lw=1.2, label=f'SG-90 clone stall ~{SG90_STALL_REAL} N m [E]')
    ax.axhline(SG90_STALL, color='#c0392b', ls=':', lw=1.2, label=f'SG-90 datasheet {SG90_STALL} N m [D]')
    ax.set_xlabel('body pitch back from vertical (deg)'); ax.set_ylabel('servo torque (N m)')
    ax.invert_xaxis(); ax.set_ylim(0, 0.2); ax.grid(alpha=0.3); ax.legend(fontsize=8, frameon=False)
    ax.set_title('Rearing up from the tail rest (estimate)', fontsize=10)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, 'tail_torque.png')); plt.close(fig)


def main():
    os.makedirs(OUT, exist_ok=True)
    its = W.items()
    say('# Tailwhip design check\n')
    say('Generated by `cad/analysis.py` from `cad/tailwhip.py`. Nothing has been built or measured: every number '
        'is an estimate built from the tagged inputs in `tailwhip.py` ([D] datasheet, [E] estimate, [C] chosen).')
    mass, com = mass_table(its)
    lo, hi = interference(its)
    back, front, P = rest_angles(its, com)
    rest, a_rest = tail_rest(P, back)
    rows = rear_up(mass, com, back)
    physics(mass, com, back, front)
    leaf(mass)
    bed()
    dims(its)
    torque_plot(rows)
    say('\n## Summary\n')
    say('| Check | Result | Detail |'); say('|---|---|---|')
    for n, v, t in RESULTS:
        say(f'| {n} | {v} | {t} |')
    with open(os.path.join(OUT, 'analysis.md'), 'w') as f:
        f.write('\n'.join(LINES) + '\n')


if __name__ == '__main__':
    main()
