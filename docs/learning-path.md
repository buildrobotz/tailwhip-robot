# Learning path

Twelve sessions of about an hour each. The first five need no hardware, so you can do them while the parts are
in the post. If you are building the robot, build [stage 1](stage1.html), the one-motor PID balancer, before
session 6; then carry on here. Each has one thing to do, one idea to learn, and one check so you know it worked.

| # | Do | Learn | Check |
|---|---|---|---|
| 1 | Run `python sim/test_falls.py` and `python sim/demo.py`, watch the video | What the robot is supposed to do; how a MuJoCo model is laid out (bodies, joints, actuators, contacts) | You can point at the line in `sim/tailwhip.xml` for the tail joint, the leaf spring and the backlash joint |
| 2 | Open `cad/tailwhip.py`, change `TAIL_PIVOT`, rerun `cad/analysis.py` | Why the tail pivot sits high and back: torque = force x lever, and the lever that matters is about the pivot | You can explain why moving the pivot to the axle roughly doubles the servo torque in section 4 of `analysis.md` |
| 3 | In `sim/test_falls.py`, run the stowed-tail case, then add your own cases (tail angle, shove strength) | Static versus dynamic checks: why a bumper under the centre of mass doesn't stop a fall | You can say in one sentence why the feet are 86 mm long |
| 4 | Read `sim/tailwhip_env.py` from the top; print one observation | What the policy sees: 13 numbers per step, 4 steps stacked, wheel speeds signed from the command | You can name every one of the 52 inputs |
| 5 | Train for 2 M steps, then 20 M, with tensorboard open | How PPO improves a policy; what the reward terms do | Your policy's `evaluate.py` numbers are within a few per cent of the ones on the Simulation page |
| 6 | Build the power system on the bench with a meter | 2S Li-ion, protection boards, buck converters, why the servos get their own 5 V rail | Both bucks read 5.0 V with no load, and the servo rail stays above 4.8 V while you hold a servo against its stop |
| 7 | Assemble, flash, run every `host/bench.py` test | How each sensor works and how bad it is | `host/measurements.json` is filled in |
| 8 | Drive it on its tail with `host/drive.py` | Open-loop driving, and the motor dead zone you just measured | 2 m forward and back, turns both ways |
| 9 | Tune the PID balancer | The classic inverted-pendulum controller: angle, rate, and the slow speed loop that stops it running away | 60 s upright, less than 1 m drift |
| 10 | Retrain with your measurements, run the policy from the laptop | Domain randomisation, and closing the sim-to-real gap by measuring | It rears up by itself |
| 11 | Shove it, log it, compare with the simulator | System identification: where does the real robot differ, and which parameter explains it | You can name the one parameter you changed and why |
| 12 | Export the policy to the ESP32 and run it with no laptop | Running a neural network on a microcontroller: 7,747 numbers, three small matrix multiplies per step | Same behaviour as session 10 with the laptop unplugged |

## Where to go next

- **Position estimation:** the spare LSM303D board has a magnetometer. Add it, fuse heading with the wheel
  odometry, and teach the policy to go back to where it started.
- **Vision:** the ESP32-CAM streams to your laptop. Have the head track a coloured ball, then have the robot follow it.
- **A better tail:** the tail is a single servo used as a leg. Try a two-joint tail in the simulator first; the
  CAD and the model are both parametric.
