# Build plan

Seven milestones, in order. Each ends with a test; don't move on until it passes. Times are estimates for
someone who has soldered and 3D printed before.

| # | Milestone | Time | Passes when |
|---|---|---|---|
| 0 | Order, measure, print, bench test | 1-2 weeks of waiting, 2 evenings of work | Every sensor and actuator works from the laptop; the bench measurements are in `host/measurements.json` |
| 1 | Drives on its tail | 1 evening | Driven by keyboard while resting on its tail, 2 m forward and back, turns both ways |
| 2 | PID balance | 1-2 evenings | Placed upright by hand, it balances for 60 s and drifts less than 1 m |
| 3 | Policy in simulation | 1 hour (mostly waiting) | The policy you trained rears up, balances and survives shoves across the randomised robots |
| 4 | Policy on the robot | 1-3 evenings | It rears up by itself 8 times in 10, balances 60 s, and survives a shove that knocks the PID over |
| 5 | Policy on board | 1 evening | The same, with the laptop unplugged |
| 6 | Head | 1 evening | The ESP32-CAM streams video while it balances, and turning the head doesn't upset it |

## Milestone 0: order, measure, print, bench test

1. **Order the parts** on the [Parts](parts.html) page. If you don't have them already: TT motors with wheels,
   SG-90s (or MG90S), an MPU6050, two ESP32 dev boards, an MX1508, two Mini360 bucks, a 2S protection board,
   a 2 x 18650 holder, and two **good** 18650 cells.
2. **Install the software.** Arduino IDE with the ESP32 core, and Python 3.10+:
   ```bash
   python -m venv ~/.venvs/tailwhip
   ~/.venvs/tailwhip/bin/pip install mujoco gymnasium stable-baselines3 numpy manifold3d trimesh pyserial
   ```
3. **Train the policy now**, while you wait for parts (milestone 3 below). It takes about an hour on a laptop and
   you'll want it later.
4. **Measure before you print.** Calipers on one TT motor and one wheel: the five numbers listed under "Before you
   print" on the [Mechanical](mechanical.html) page. Edit them in `cad/tailwhip.py` and run
   `python cad/analysis.py && python cad/export.py`. Every check in `cad/out/analysis.md` should still say PASS.
5. **Print `core_left` first** and check that a TT motor sits on its bosses with the bolt holes lined up. Then
   print the rest (print settings on the [Mechanical](mechanical.html) page). The tail is ABS.
6. **Build the power system on the bench, without the robot**: cells, holder, protection board, fuse, switch, the
   two Mini360s set to 5.0 V with a meter **before** anything is connected to them. The
   [Electronics](electronics.html) page walks through it with a meter check at every step.
7. **Flash both ESP32s** (robot and bridge) and run `firmware/i2c_scan` first to see the MPU6050 at 0x68.
8. **Assemble** the robot (Mechanical page, Assembly), wire it (Electronics page, wiring table).
9. **Bench test from the laptop**: `python host/bench.py PORT ...` has one subcommand per actuator and sensor and
   one per measurement. Run them all; they write `host/measurements.json`, which the simulator reads.

**Check:** both wheels spin both ways, the tail and head sweep, the IMU pitch follows your hand, the IR sensors
count edges when you turn the wheels, the battery voltage reads within 0.1 V of your meter, and
`measurements.json` has no empty fields.

## Milestone 1: drives on its tail

Set it down on its tail and run `python host/drive.py PORT`. Arrow keys drive; the tail stays at the rest angle.

**Check:** 2 m forward and back and turns both ways without tipping. If it pulls to one side, the motors differ:
note it, the simulator randomises motor strength by 25 % for exactly this reason.

## Milestone 2: PID balance

Switch to `pid` mode and hold the robot upright, then let go. Start from the gains in `docs/software.md` and tune
them in the order given there.

**Check:** 60 s upright, less than 1 m of drift with the wheel sensors plugged in. Then unplug the IR sensors and
check it still balances (it will drift more). This is the baseline the policy has to beat.

## Milestone 3: policy in simulation

```bash
python sim/train.py               # about 45 minutes on a 10-core laptop; tensorboard --logdir sim/runs to watch
python sim/evaluate.py            # scores it over 50 randomised robots per test
python sim/demo.py                # films the whole act to sim/out/demo.mp4
```

If you ran the bench measurements first, the simulator's nominal robot is your robot.

**Check:** `evaluate.py` rears up and balances in nearly every randomised robot, and the demo video looks like the
one on the [Overview](index.html). The [Simulation and RL](learning.html) page has the numbers to compare against.

## Milestone 4: policy on the robot

```bash
python host/run_policy.py PORT                       # policy runs on the laptop, robot in 'remote' mode
```

Start with the robot on its tail and give the `stand` command. Keep a hand near it the first few times.

**Check:** rears up by itself 8 times in 10, balances 60 s, and survives a shove (a push on the battery sleeve) that
makes the PID version fall. If it twitches or rocks where the simulator was calm, measure the radio delay and the
backlash again, update `measurements.json`, retrain from the current policy with
`python sim/train.py --init sim/policies/tailwhip.zip --steps 5000000` and try again. That loop is the core of
sim-to-real.

## Milestone 5: policy on board

```bash
python sim/export_policy.py       # writes firmware/tailwhip_robot/policy.h (about 30 KB of flash)
```

Reflash the robot, unplug the laptop, and switch to `onboard` mode with the switch sequence in `docs/software.md`.

**Check:** the same as milestone 4, with no laptop.

## Milestone 6: head

Fit the ESP32-CAM in the head (its own 5 V, from the logic buck, or a third Mini360 if it browns out) and flash
the stock camera web server example. Pan the head with the `head` command while it balances.

**Check:** video streams while it balances, and a full pan doesn't knock it over.
