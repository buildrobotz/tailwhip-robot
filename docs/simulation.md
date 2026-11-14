# Simulate and train

## The model

`sim/tailwhip_model.py` builds the MuJoCo model straight from the CAD: every part's mass, centre of mass and
inertia come from the meshes in `cad/tailwhip.py`, so changing the CAD changes the simulator. Run it after any
CAD change.

| Body | Joint | What it models |
|---|---|---|
| chassis | free | Everything that rides on the body: motors, boards, battery, head, tail servo. Collision: the feet, the heel and chin scuff guards, a torso box and a head sphere |
| gear_l, gear_r | hinge, driven by torque | The TT gearbox output. The motor's rotor inertia, reflected through the 48:1 gearbox, is the joint's armature |
| wheel_l, wheel_r | hinge, limited to +-backlash/2 | **Gearbox backlash**: the wheel rattles freely inside a few degrees before the gear catches it |
| tail | hinge, position servo | The SG-90: torque-limited and speed-limited |
| foot | hinge with a spring | **The printed leaf spring**: 0.66 N m/rad, from the CAD's leaf dimensions |

Physics runs at 500 Hz, the policy at 50 Hz.

## Everything sloppy, on purpose

A policy trained on a perfect robot fails on a real one. So the simulated Tailwhip is as bad as the real one is
likely to be, and differently bad every episode:

| Flaw | How it is modelled | Randomised per episode |
|---|---|---|
| Weak, slow DC motors | torque = stall x (V / 6) - (stall / no-load speed) x speed, with 8-25 ms of electrical lag. u = 0 brakes, as the firmware's slow-decay drive does | stall torque +-25 %, no-load speed +-15 % |
| Motor dead zone | small commands do nothing | 0.04 to 0.16 of full command |
| Gearbox backlash | free wheel joint inside +-b/2 | 2 to 8 deg |
| Cheap servo | speed-limited target, torque limit, horn spline error | speed +-20 %, torque 0.10 to 0.16 N m, offset +-4 deg |
| Radio and loop delay | actions applied late | 10 to 40 ms |
| IMU | filtered pitch with bias and noise, gyro bias and noise | pitch bias +-1.5 deg |
| IR wheel encoders | 32 edges per turn counted in a 40 ms window, **sign taken from the command** (the sensor can't tell direction) | 10 % of episodes with the sensors unplugged |
| Mass and balance | body mass, centre of mass height and fore-aft | +-15 %, +-15 mm, +-4 mm |
| Floor | tyre friction, foot friction, floor tilt | tyre 0.4 to 1.0, tilt +-1.5 deg |
| Battery | voltage, sag under load | 6.6 to 8.4 V |
| Shoves | 0.1 s pushes on the chest every 2-4.5 s | 0.5 to 2.5 N, any direction |

When you have measured your own robot (`host/bench.py` writes `host/measurements.json`), the simulator uses your
numbers as the centre of each range.

## What the policy sees and does

The policy only gets what the robot actually sends over the radio, 13 numbers per step, and the last 4 steps
stacked newest first (52 inputs):

| # | Input |
|---|---|
| 0 | pitch, rad (+ = leaning forward) |
| 1, 2 | pitch rate and yaw rate, rad/s / 5 |
| 3, 4 | left and right wheel speed, rad/s / 20, signed from the command |
| 5 | the tail action it sent last step |
| 6, 7, 8 | the action before that (left, right, tail) |
| 9 | (battery voltage - 7.6) / 0.8 |
| 10, 11, 12 | the command: stand (1) or rest (0), speed / 0.3 m/s, turn rate / 2 rad/s |

It never sees the tail angle (an SG-90 has no position feedback), the wheel angles, or where it is.

Three outputs, each -1 to 1: left wheel, right wheel (motor voltage = u x 6 V), and the tail target
(85 + 65 x a degrees, so 20 to 150).

The network is two hidden layers of 64 with tanh: 7,747 numbers, small enough to run on the ESP32.

## What it is rewarded for

| Command | Reward each step |
|---|---|
| STAND | being upright (a bell curve on pitch, width 0.25 rad), plus following the speed and turn commands while upright, minus resting on the tail foot once it is up, minus wobble |
| REST | being at the tail rest angle (25 deg back), plus the foot on the floor, minus fidgeting |
| always | minus motor effort, minus jerky actions, minus hard landings on the tail, minus holding the tail above 100 deg (it is the back bumper: stowed, a fall ends flat on its back), and -30 for falling over |

Episodes last 10 s and start in five ways: resting on the tail, upright, upright with a kick, leaning on the feet,
and upright with the REST command. The command changes now and then during an episode.

## Results

Scored by `sim/evaluate.py` over 50 randomised robots per test (each a different mass, centre of mass, tyre
friction, motor strength, dead zone, backlash, delay, sensor bias, battery and servo). Policy v1, 20 M steps,
about 45 minutes of training on a 10-core laptop.

| Test | Trained policy | PID baseline |
|---|---|---|
| Rear up from the tail rest within 4 s, still up at 10 s | **84 %** (median 0.24 s to pass 10 deg) | cannot rear up |
| Get up off the feet within 4 s | **90 %** | cannot |
| Survive a 1-3 rad/s kick from upright | **84 %** | 72 % |
| Balance 10 s with random shoves, upright 90 % of the time | **58 %** | 30 % |
| Lie down onto the tail when told to | 14 % | n/a |
| Follow a 0.2 m/s drive command | 0.09 m/s, fell 6 % | n/a |

What this says, plainly:

- **It does the hard part.** It rears up from the tail and off the feet in most randomised robots, and it takes
  shoves better than the hand-tuned PID, which only balances.
- **It is weak at three things:** lying down on command (it tends to stay up), following a speed command (it
  creeps at about half the commanded speed), and hard **forward** shoves: it drives forward to catch them and
  sometimes goes over its feet. A better reward and longer training should improve all three; a fine-tuned
  version is in progress and this page will be updated if it scores better.
- These are simulation numbers on a deliberately pessimistic model. The real robot will differ; milestone 4 of the
  build plan is about closing that gap.

## Train your own

```bash
python sim/train.py                   # 20 M steps, about 45 min on a 10-core laptop
tensorboard --logdir sim/runs         # tw/up_s, tw/fell and tw/rest_start_up_s are the ones to watch
python sim/evaluate.py                # score it
python sim/demo.py                    # film it
python sim/export_policy.py           # C header for the ESP32
```

After you measure your robot, retrain from the current policy so it adapts to your numbers:

```bash
python sim/train.py --init sim/policies/tailwhip.zip --steps 5000000
```

Things to try: make the tail reward stricter and watch it learn to keep the foot off the floor; turn off the
randomisation (`TailwhipEnv(randomize=False)`) and see how much worse the policy does on randomised robots;
remove the wheel sensors from the observation entirely and see whether it can still hold its position.

## The drop tests

`sim/test_falls.py` drops the robot from upright with the motors off and checks where it ends up. It is the test
that found the rev A bumper problem (see the Mechanical page): the heel and chin touched first, but every fall
rolled straight over them. With the feet and the tail held at its rest angle, every fall up to the passive limit
ends on the feet or the tail.

| Case (motors off) | Final pitch | Peak | Result |
|---|---|---|---|
| topple back, tail at ready/rest | -27.6 deg | 38.2 deg | PASS |
| shoved back, 4 rad/s | -27.6 deg | 54.8 deg | PASS |
| topple forward | 17.9 deg | 20.1 deg | PASS |
| shoved forward, 4 rad/s | 17.9 deg | 27.0 deg | PASS |
| shoved forward, 6 rad/s | -27.5 deg | 46.3 deg | PASS |
| shoved back, 5 rad/s (beyond the passive limit) | -108.8 deg | 109.6 deg | expected: rolls over (motors off) |
| topple back, tail STOWED up the back (do not do this) | -79.6 deg | 96.4 deg | expected: rolls over (motors off) |
