"""The demo act as poses, one per video frame, consistent with the robot's geometry:
the wheels roll without slipping, the tail foot stays planted while the robot rears up or lies down, and the foot
never goes through the floor. Writes visual/out/motion.json for blender_animate.py.

This is a KINEMATIC storyboard (what the behaviour should look like), not a physics simulation: the timing
is hand-chosen and nothing here says a controller can do it. The MuJoCo model will answer that.

Run: python visual/motion.py
"""
import os, sys, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'cad'))
import tailwhip as W

FPS = 30
REST = -25.0
A_REST = W.tail_angle_for_foot_on_floor(REST)
A_STOW = A_REST            # rev B: the tail waits at its rest angle while balancing (it is the back bumper)
A_UPFOOT = W.tail_angle_for_foot_on_floor(0.0)


def ease(u):
    u = np.clip(u, 0, 1); return u * u * (3 - 2 * u)

def foot_rel_x(pitch, alpha):
    return W.foot_world(0.0, pitch, alpha)[0]

def foot_z(X, pitch, alpha):
    return W.foot_world(X, pitch, alpha)[2]


class Act:
    """Piecewise timeline. Each segment is a function (t_local, dur, state) -> state."""
    def __init__(self):
        self.segs = []      # (t0, t1, fn, caption)

    def add(self, dur, fn, caption=None):
        t0 = self.segs[-1][1] if self.segs else 0.0
        self.segs.append((t0, t0 + dur, fn, caption))

    def run(self):
        s = dict(X=0.0, pitch=REST, tail=A_REST, pan=0.0, spin_extra=0.0, ball=None)
        frames, cap = [], ''
        T = self.segs[-1][1]
        n = int(round(T * FPS))
        seg_i, start_state = 0, dict(s)
        for k in range(n + 1):
            t = k / FPS
            while seg_i < len(self.segs) - 1 and t >= self.segs[seg_i][1]:
                start_state = dict(frames[-1]['state']) if frames else dict(s)
                seg_i += 1
            t0, t1, fn, c = self.segs[seg_i]
            if k == 0:
                start_state = dict(s)
            st = fn(t - t0, t1 - t0, dict(start_state))
            if c is not None:
                cap = c
            # never let the foot through the floor: if it would, bend the leaf (raise the tail angle)
            if foot_z(st['X'], st['pitch'], st['tail']) < W.FOOT_R - 0.01:
                a = W.tail_angle_for_foot_on_floor(st['pitch'])
                if a is not None:
                    st['tail'] = max(st['tail'], a)
            frames.append(dict(t=t, state=st, caption=cap))
        return frames


# ------------------------------------------------------------------------------------------------- segments
def hold(dt, d, s):
    return s

def look(angles):
    def f(dt, d, s):
        u = dt / d * (len(angles) - 1)
        i = min(int(u), len(angles) - 2)
        s['pan'] = angles[i] + (angles[i + 1] - angles[i]) * ease(u - i)
        return s
    return f

def drive(dist, lean=0.0):
    def f(dt, d, s):
        u = dt / d
        s['X'] = s['X'] + dist * ease(u)
        if lean:
            s['pitch'] = s['pitch'] + lean * np.sin(np.pi * u) ** 2 * np.sign(0.5 - u) * 1.0 + lean * np.sin(np.pi * u) * 0.6
        return s
    return f

def drive_tripod(dist):
    def f(dt, d, s):
        s['X'] = s['X'] + dist * ease(dt / d)
        return s
    return f

def pivot_on_foot(p0, p1, shape=ease):
    """Rear up / lie down with the foot planted: the axle rolls so the foot stays put."""
    def f(dt, d, s):
        a0 = W.tail_angle_for_foot_on_floor(p0)
        F = s['X'] + foot_rel_x(p0, a0)
        p = p0 + (p1 - p0) * shape(dt / d)
        a = W.tail_angle_for_foot_on_floor(p)
        s['pitch'], s['tail'] = p, a
        s['X'] = F - foot_rel_x(p, a)
        return s
    return f

def move_tail(a1, wobble=0.0):
    def f(dt, d, s):
        u = ease(dt / d)
        s['tail'] = s['tail'] + (a1 - s['tail']) * u
        s['pitch'] = s['pitch'] * (1 - u) + wobble * np.sin(np.pi * dt / d)
        return s
    return f

def balance(amp=1.2, hz=1.1, sway=7.0, pans=None):
    lk = look(pans) if pans else None
    def f(dt, d, s):
        ph = 2 * np.pi * hz * dt
        env = np.sin(np.pi * dt / d) ** 0.5
        s['pitch'] = amp * env * np.sin(ph) + 0.4 * env * np.sin(2.7 * ph + 1)
        s['X'] = s['X'] + sway * env * np.sin(ph - 0.6)
        if lk:
            s = lk(dt, d, s)
        return s
    return f

def lean_drive(dist, lean):
    """Lean forward to accelerate, ride, lean back to stop."""
    def f(dt, d, s):
        u = dt / d
        s['X'] = s['X'] + dist * ease(u)
        s['pitch'] = lean * np.sin(2 * np.pi * u) * (1 if u < 0.5 else 1.0)
        return s
    return f

def shoved(hit_t, t_tail, fall_end):
    """Hit at hit_t: pitches back, wheels race backwards, can't catch it, tail swings down and takes the landing."""
    def f(dt, d, s):
        if dt < hit_t:
            return s
        u = min(1.0, (dt - hit_t) / (fall_end - hit_t))
        s['pitch'] = REST * u ** 1.6
        s['X'] = s['X'] - 75.0 * ease(min(1.0, (dt - hit_t) / (fall_end - hit_t) * 1.2))
        v = min(1.0, max(0.0, (dt - hit_t) / t_tail))
        s['tail'] = A_STOW + (A_REST - A_STOW) * ease(v)
        return s
    return f

def bounce(dt, d, s):
    u = dt / d
    s['pitch'] = REST - 2.6 * np.exp(-5 * u) * np.sin(3 * np.pi * u)
    s['tail'] = A_REST
    return s

def shake(dt, d, s):
    s['pan'] = 28 * np.sin(2 * np.pi * 2.2 * dt / d * 1.6) * np.sin(np.pi * dt / d)
    return s


def build():
    a = Act()
    a.add(1.0, hold, 'Tailwhip rests on its tail: two wheels and a foot')
    a.add(1.4, drive_tripod(140.0), 'It can drive around like this')
    a.add(1.8, look([0, 45, 45, -45, -45, 0]), None)
    a.add(1.7, pivot_on_foot(REST, 0.0), 'The tail pushes down and it rears up')
    a.add(0.5, move_tail(A_STOW, wobble=1.0), None)
    a.add(2.2, balance(pans=[0, -55, -55, 50, 50, 0]), 'Balancing on two wheels, looking around')
    a.add(1.3, lean_drive(260.0, 6.0), None)
    a.add(1.4, shoved(0.55, 0.32, 1.25), 'A shove it cannot catch with the wheels...')
    a.add(0.6, bounce, '...so it lands on its tail, which was waiting for it')
    a.add(0.9, shake, None)
    a.add(1.7, pivot_on_foot(REST, 0.0), 'And it gets back up')
    a.add(0.5, move_tail(A_STOW, wobble=0.8), None)
    a.add(1.6, balance(amp=0.9, pans=[0, -30, -30, 0]), None)
    a.add(0.5, move_tail(A_UPFOOT), 'Lying down: tail first')
    a.add(1.6, pivot_on_foot(0.0, REST), None)
    a.add(1.4, look([0, -35, -35]), None)
    return a


def ball_track(frames, hit_t_abs):
    """A soft ball thrown at the chest; bounces off and rolls away."""
    k_hit = int(round(hit_t_abs * FPS))
    Xh = frames[k_hit]['state']['X']
    r = 35.0
    hit = np.array([Xh + 29 + r, -6.0, W.R_WHEEL + 150.0])
    start = hit + np.array([620.0, -260.0, 160.0])
    t_in = 0.5
    land = hit + np.array([240.0, -60.0, 0.0]); land[2] = r
    stop = land + np.array([300.0, -80.0, 0.0])
    for f in frames:
        t = f['t'] - hit_t_abs
        if t < -t_in:
            p = None
        elif t < 0:
            u = (t + t_in) / t_in
            p = start + (hit - start) * u + np.array([0, 0, 90.0]) * np.sin(np.pi * u)
        elif t < 0.55:
            u = t / 0.55
            p = hit + (land - hit) * u + np.array([0, 0, 70.0]) * np.sin(np.pi * u)
            p[2] = max(p[2], r)
        elif t < 2.2:
            u = (t - 0.55) / 1.65
            p = land + (stop - land) * (1 - (1 - u) ** 2)
            p[2] = r + 18 * abs(np.sin(np.pi * u * 2)) * (1 - u)
        else:
            p = stop
        f['ball'] = None if p is None else (p / 1000.0).tolist()


def camera(frames):
    xs = np.array([f['state']['X'] for f in frames])
    k = 30
    sm = np.convolve(np.pad(xs, (k, k), mode='edge'), np.ones(2 * k + 1) / (2 * k + 1), mode='valid')
    n = len(frames)
    for i, f in enumerate(frames):
        u = i / (n - 1)
        az = np.radians(-108 + 40 * u)                   # slow orbit from front-right towards the side
        dist = 980.0
        target = np.array([sm[i] - 10, 0.0, 120.0])
        pos = target + np.array([dist * np.cos(az), dist * np.sin(az), 170.0])
        f['cam'] = dict(pos=(pos / 1000).tolist(), target=(target / 1000).tolist())


def main():
    act = build()
    frames = act.run()
    hit_abs = [s[0] for s in act.segs if s[3] == 'A shove it cannot catch with the wheels...'][0] + 0.55
    ball_track(frames, hit_abs)
    camera(frames)
    out = []
    for f in frames:
        s = f['state']
        tf = W.group_tfs(s['X'], s['pitch'], s['tail'], s['pan'])
        out.append(dict(t=f['t'], caption=f['caption'], ball=f['ball'], cam=f['cam'],
                        pose=dict(X=s['X'], pitch=s['pitch'], tail=s['tail'], pan=s['pan']),
                        groups={g: (np.diag([1e-3, 1e-3, 1e-3, 1]) @ M @ np.diag([1e3, 1e3, 1e3, 1])).tolist()
                                for g, M in tf.items()}))
    # sanity: foot height and lowest body point never below the floor
    worst = min(foot_z(f['pose']['X'], f['pose']['pitch'], f['pose']['tail']) - W.FOOT_R for f in out)
    print(f'{len(out)} frames, {out[-1]["t"]:.1f} s; lowest foot clearance {worst:.2f} mm')
    os.makedirs(os.path.join(HERE, 'out'), exist_ok=True)
    with open(os.path.join(HERE, 'out', 'motion.json'), 'w') as fh:
        json.dump(dict(fps=FPS, frames=out), fh)


if __name__ == '__main__':
    main()
