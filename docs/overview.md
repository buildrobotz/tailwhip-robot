# Overview

## The robot

**Tailwhip** is a small two-wheeled self-balancing robot with a tail and a camera head. It is 264 mm tall,
weighs about 394 g, and is made from the parts most robotics hobbyists already have in a drawer: two yellow TT
gearmotors, two SG-90 servos, an ESP32, an MPU6050 and two 18650 cells.

It reads as a creature, not a machine. Switched off, it sits leaning back on its tail like a kangaroo. Told to
stand, the tail pushes down and it rears up onto its two wheels. It balances, looks around with its camera head,
and drives. Shove it hard and it can't catch itself with the wheels, so it lands on its tail, which was waiting
for it, and gets back up. Shove it forwards and it lands on its feet.

**Your first 30 seconds with a finished robot.** Power on and run `python host/drive.py PORT`: it drives around
on its tail with the arrow keys. Press `p` and the hand-written PID controller holds it upright if you lift it
there yourself. Then run `python host/run_policy.py PORT` and type `stand`: the trained policy pushes down with the
tail, rolls the wheels back under the body and rises. Getting from "PID balances when I hold it upright" to "it
gets up by itself and survives a shove" is what the rest of the project teaches.

## Where to start: stage 1, then the full robot

Build **[stage 1](stage1.html)** first, following its one-page step-by-step guide. It is a bare-minimum balancer: one TT motor with a wheel on each end of its
shaft, two 18650s, an ESP32, an IMU and a motor driver, kept upright by a PID loop. Its one job is to show that a TT
motor is fast enough to balance a body, before you build a robot that depends on it. Like the full robot, it is
designed and simulated but not built yet. A second milestone adds one wheel sensor so it holds its position. Stage 2 (two motors, the body held at a lean by a fixed weight) is
planned but not designed yet. When stage 1 passes, build the full robot, starting with the
[Build plan](build.html). The [pages below](#build-it-page-by-page) are in that order.

## Why this is a good robot to learn on

Plain balancing on two wheels is a solved, hand-written control problem: a PID loop on the tilt angle does it.
Tailwhip is built so that the interesting part is everything around that loop, which is hard to write by hand:

- **Rearing up** needs the tail and both wheels to move together, with a servo that has only twice the
  torque it needs.
- **Deciding when a shove is too big** for the wheels, and letting the tail take it instead.
- **Getting up off the feet** needs a quick push of the wheels at the right moment.
- **Sloppy parts**: TT gearboxes with several degrees of backlash, encoders that can't tell forwards from
  backwards, a radio link with 10-40 ms of delay.

Those are exactly the things reinforcement learning is good at, and exactly the things that make simulated
robots fail on real hardware. The simulator here models every one of them on purpose.

**What you will learn by building it.**

- Mechanical design: a parametric CAD in Python, a printed leaf spring, and checking a design numerically before
  printing it (including the fall problem the simulator caught, on the Mechanical page).
- Electronics: a 2S 18650 pack with a protection board, two buck converters, an H-bridge, I2C sensors and an
  ESP-NOW radio link.
- Control: a PID balancer you tune yourself, as the baseline.
- Simulation: a MuJoCo model built straight from the CAD, with the motors, servo and sensors made as bad as the
  real ones.
- Reinforcement learning: observations, actions, rewards, PPO, and evaluating over hundreds of randomised robots.
- Sim to real: bench measurements that feed the simulator, then running the policy from the laptop and finally on
  the ESP32 itself.

## How it works, in one picture

The robot's ESP32 reads the IMU 500 times a second and estimates its tilt. 100 times a second it sends a state
packet over ESP-NOW to a second ESP32 plugged into your laptop. The laptop runs the policy, a small neural network
(52 inputs, two layers of 64, three outputs), 50 times a second, and sends back two wheel commands and a tail angle.
If the commands stop for 100 ms the robot cuts the motors and puts the tail down. Once the policy works from the
laptop, you export it to a C header and it runs on the robot with no laptop at all.

## Sister project

Tailwhip follows [Ouro](https://ouro.buildrobotz.com/), the worm that bites its own tail and rolls away: same
toolchain (Python CAD, MuJoCo, PPO, ESP32 + ESP-NOW), same way of documenting. If you have built Ouro, you will
recognise everything here.
