# Tailwhip: the original design brief

Written 2026-11-13 as the brief for the AI agent that produced the detailed
design (CAD, wiring, firmware, simulator, training code, test plan). It is kept
as written so you can see what was asked for; where the finished design differs,
the docs say so and why (chiefly: mass 396 g [E] against the 250-270 g target,
the battery on top of the spine, the feet and the tail as the bumpers, and the
tail never stowed: see `docs/mechanical.md`).

Read this whole file before starting. Sections 2 and 3 are decided and should
not be reopened without asking Srini. Section 9 lists what is still open.

## 1. What Tailwhip is

A small two-wheeled self-balancing robot with a servo tail and a camera head,
built from parts Srini already owns. It should read as a creature, in the
spirit of MicroDuck: it rests on its tail, rears up onto two wheels, balances,
looks around, catches itself with its tail when shoved, and lies back down.

It is a demo robot first (one unit, built by Srini). Later it becomes a
BuildRobotz teaching platform: reinforcement learning first, then position
estimation, mapping and vision. Design choices should keep that later path
open but must not add cost or complexity for it now.

Ouro (https://github.com/buildrobotz/ouro-robot, MuJoCo + RL + ESP32) is the precedent for repo layout, simulator
setup and documentation style. Reuse its structure and tooling where it fits.

## 2. Decisions already made

| Decision | Choice | Reason |
|---|---|---|
| Drive | 2 standard yellow TT gearmotors (1:48), 65 mm TT wheels | ~200 rpm at 6 V gives ~0.68 m/s, enough to catch a fall. The 32-64 rpm encoder gearmotors Srini also owns are too slow for balancing. |
| Tail | 1 SG-90 servo | Enough torque at this robot's weight to push up from a tail-down rest. Used as a leg and prop, not as a swinging counterweight. |
| Head | ESP32-CAM on 1 SG-90 (pan) | Stage 4 only. Camera streams over Wi-Fi separately from the control link. |
| Motor driver | MX1508 | 2-10 V, about 1.5 A per channel, weighs a few grams. The L298N board is heavy and drops about 2 V; do not use it. |
| Controller | ESP32 dev board on the robot, second ESP32 on the laptop's USB as a radio bridge | ESP-NOW between them |
| Where the policy runs | Laptop during development, ESP32 later | Srini's stated approach |
| Simulator and algorithm | MuJoCo, PPO | Same as Ouro |
| Battery | 2 x 18650 in series (7.4 V nominal, 8.4 V full) | Within MX1508's 10 V limit |
| Structure | 3D printed | Srini has a printer |

## 3. Constraints

- Use only the parts in section 4 plus the purchases in section 5. Extra
  budget for the whole project is $50-70; this design should need under $15.
- One unit. Do not optimise for kit manufacture yet.
- Text and code go in git; video and other heavy media stay out of it.

## 4. Parts on hand

| Part | Detail | Used for |
|---|---|---|
| TT motors | Standard yellow, 1:48 | Wheels |
| SG-90 servos | Several | Tail, head pan |
| ESP32 dev boards | Several | Robot controller, USB radio bridge |
| ESP32-CAM | | Head (stage 4) |
| MX1508 | Dual H-bridge | Wheel motors |
| MPU6050 | 6-axis accelerometer + gyro | **The balance IMU** |
| LSM303D board (with gyro) | Accelerometer + magnetometer, gyro on the same board | Spare. Its magnetometer is a heading source for the later position-estimation lessons |
| IR reflective modules | Line-follower type, digital output | Wheel speed sensing |
| 18650 cells | Plenty; **unprotected** | Power, with a 2S protection board |
| Mini360 buck converters | Small adjustable buck, about 1 A continuous in practice (1.8 A is the peak figure) | 5 V rails |
| Boost converters | | Not needed |
| Ender 3 (220 x 220 x 250 mm bed), PLA and ABS, no TPU | | Frame, tail, bumpers, encoder discs. Every part must fit the bed. Stock TT tyres; no printed flexible parts |

## 5. To buy

| Item | Why | About |
|---|---|---|
| MG90S servo x2 | Same size as SG-90, metal gears. Fallback if the SG-90 tail strips when the robot lands on it. | $6 |
| **2S protection board (BMS)** | **Required**: the cells are unprotected. Over-discharge, over-current and short protection. Add an inline fuse or polyfuse as well | $3 |

## 6. Design targets (estimates, to be replaced by measurements)

| Quantity | Target or estimate |
|---|---|
| Total mass | 250-270 g |
| Centre of mass above the wheel axle | 7-9 cm (cells mid-body, not at the axle: a higher centre of mass falls more slowly) |
| Overall height | 22-28 cm including head |
| Wheel speed, unloaded at 6 V | ~0.68 m/s |
| Torque to hold the body at lean θ | m·g·h·sin(θ) ≈ 0.21·sin(θ) N·m |
| SG-90 stall torque | ~0.15 N·m |
| Wheel torque before tyres slip | μ·m·g·r ≈ 0.07 N·m (μ ≈ 0.8) |
| Tail-down rest angle | 25-30° back from vertical (needs ~0.09-0.10 N·m to leave) |
| Shallowest rest angle anywhere | ~50° from vertical, front and back, set by printed bumpers |

Consequences the mechanical design must respect:

- Flat on its back needs ~0.21 N·m, about what tail plus wheels can give
  together with no margin. **The body shape must make lying flat impossible**:
  a rounded back or bumper so it never rests flatter than about 50°.
- A printed chin bumper does the same for forward falls.
- The tail must survive the robot landing on it. Design a spring section
  into the tail geometry (thin curved PLA or ABS leaf; there is no TPU) so
  shock does not go straight into the servo gears.
- TT motors run above their 6 V rating on a full 2S pack. Firmware caps PWM
  duty to keep the average at or below about 6.5 V.

### Electrical

- Battery → 2S protection board → MX1508 motor supply directly.
- **Two Mini360 bucks, two separate 5 V rails:** one for the ESP32 (and later
  the ESP32-CAM), one for the servos. A single Mini360 cannot do both: each
  SG-90 can pull ~0.7 A at stall and the Mini360 is good for about 1 A
  continuous. Set both to 5.0 V with a meter before connecting anything.
  470-1000 µF across the servo rail. If the ESP32-CAM browns out at stage 6,
  give it a third Mini360.
- MX1508 has no enable pins: PWM goes on the IN pins. Choose and document the
  decay mode; it changes the motor's low-speed behaviour and belongs in the
  simulator's motor model.
- Battery voltage to an ADC pin through a divider. The policy and the PWM cap
  both use it.
- Suggested ESP32 pins (change if needed, avoid strapping pins): motors
  25, 26, 27, 14; tail servo 18; head servo 19; I2C SDA 21, SCL 22; wheel IR
  sensors 34, 35; battery sense 36.

### Sensing

- MPU6050 read at 200 Hz or faster; pitch from a complementary or Mahony filter
  on the robot. Mount the IMU rigidly near the axle.
- Wheel speed: one IR module per wheel reading a printed disc or the wheel
  spokes. This gives speed but not direction; take the sign from the
  commanded direction and model that flaw in the simulator. Resolution will
  be coarse (tens of counts per turn).
- The robot must still balance with the wheel sensors unplugged (IMU only),
  accepting drift across the floor.

## 7. Software architecture

```
Robot ESP32  --ESP-NOW-->  bridge ESP32  --USB serial-->  laptop (Python)
Robot ESP32  <--ESP-NOW--  bridge ESP32  <--USB serial--  commands
```

| Layer | Runs on | Hand-written or learned |
|---|---|---|
| IMU and wheel sensing, tilt estimate, packets, failsafe, PWM and servo output | Robot ESP32 | Hand-written |
| Baseline balance controller (PID on pitch, with a slow position loop) | Robot ESP32 | Hand-written; also the fallback and the comparison for RL |
| Policy: senses in, motor and tail commands out | Laptop, later ESP32 | Learned |
| Simulator, reward, training | Laptop | Hand-written |

- State packet, 50-100 Hz: sequence number, timestamp, pitch, pitch rate,
  yaw rate, two wheel speeds, tail angle command, battery voltage, mode.
- Command packet: sequence number, left wheel (-1..1), right wheel (-1..1),
  tail target angle, head target angle, mode.
- **Failsafe:** no valid command for 100 ms → motors off, tail to rest
  position. Motors also cut if pitch is beyond the bumper angle.
- Log every packet on the laptop. Measure real round-trip delay and loss and
  feed the measured delay into the simulator.
- Modes: `idle`, `pid` (on-board baseline), `remote` (laptop policy),
  `onboard` (policy flashed to the ESP32).

## 8. Reinforcement learning

Plain balancing near upright is a PID job. The policy is for what is hard to
hand-write: coordinating tail and wheels to rear up, deciding when a shove is
too big and the tail must go down, and getting off a bumper, all with sloppy
gearboxes and tyres that slip.

- **Simulator:** MuJoCo. Chassis body, two wheel hinges, tail hinge, ground
  contact with friction, bumper geometry. Motor model from measurements:
  speed and torque versus PWM and battery voltage, dead zone, backlash,
  first-order lag. Servo model: rate limit and torque limit. No tail position
  feedback exists on an SG-90, so the simulated policy only sees the commanded
  angle.
- **Observation:** pitch, pitch rate, yaw rate, two wheel speeds, last tail
  command, previous action, battery voltage, command (stand / rest / drive
  speed / turn rate). Stack the last 3-5 observations so the policy can cope
  with delay and backlash.
- **Action:** left wheel, right wheel, tail target. 50 Hz.
- **Reward (starting point):** head height when "stand" is commanded; track
  speed and turn commands; minus motor effort; minus action change between
  steps; minus time resting on tail or bumpers while "stand" is commanded;
  small penalty for hard tail impacts.
- **Randomise per episode:** mass ±15%, centre-of-mass height ±1.5 cm, tyre
  friction 0.4-1.0, motor strength ±25%, dead zone, backlash 2-8°, loop delay
  10-40 ms, IMU bias and noise, battery voltage 6.6-8.4 V, servo speed ±20%.
- **Episode starts:** tail-down rest, upright, mid-shove, on the back bumper,
  on the chin bumper.
- **Algorithm:** PPO, small network (two hidden layers of 64) so it can later
  run on the ESP32.
- **Sim-to-real loop:** when real behaviour differs from simulated, fix the
  simulator model (usually delay, backlash, friction) and retrain. Fine-tuning
  on the real robot is allowed since every fall is caught by a bumper.

## 9. Open items

Answered by Srini on 2026-11-13: IMU is an MPU6050; 18650s are plentiful and
unprotected; bucks are Mini360; printer is an Ender 3 with PLA and ABS; the
project lives in this repo (`buildrobotz/tailwhip-robot`).

Still to ask: 18650 capacity and whether they are high-drain cells; any
servos besides SG-90s.

To measure on the bench before the simulator is trusted:

1. TT motor: no-load speed and stall current at 3, 5, 6.5 V through the MX1508;
   the lowest PWM at which the loaded wheel starts to move; gearbox backlash
   in degrees.
2. SG-90: time for a 60° move, unloaded and pushing the robot's weight.
3. Mass of every part, and the assembled centre of mass (balance it on an edge).
4. ESP-NOW round-trip delay and packet loss at 50 and 100 Hz.
5. Tyre friction on the demo surface (tilt a board until the locked-wheel
   robot slides).

## 10. What the next agent should produce

1. **Mechanical design:** parametric CAD (same tool as Ouro) for chassis,
   motor mounts, battery tray, tail with flexure, back and chin bumpers, head
   mount, encoder discs. Dimensioned drawings, print settings, an assembly
   guide, and a computed mass and centre-of-mass table checked against
   section 6.
2. **Wiring:** full schematic and a point-to-point wiring table, power budget,
   and the final pin map.
3. **Firmware:** robot and bridge ESP32 projects implementing section 7,
   including the PID baseline and a bench test mode for each actuator and
   sensor.
4. **Laptop tools:** serial bridge library, logger, live plot, a script for
   each bench measurement in section 9.
5. **Simulator:** MuJoCo model built from the CAD and the measured values,
   with every randomised parameter in one config file.
6. **Training:** PPO training and evaluation scripts, the reward as specified,
   a script that exports the trained network to a C header for the ESP32.
7. **Test plan** covering the milestones in section 11.
8. **Bill of materials** with what is owned, what is bought, and total spend.
9. **README** in the Ouro style, stating plainly that the design is
   AI-generated and unbuilt until Srini has built it.

Every number in the outputs must be labelled as measured, taken from a
datasheet, or estimated.

## 11. Milestones and how each is judged

| # | Milestone | Passes when |
|---|---|---|
| 0 | Bench | Every sensor and actuator works from the laptop over ESP-NOW; section 9 measurements logged |
| 1 | Drives on its tail | Remote-driven in tail-down rest, 2 m forward and back, turns both ways |
| 2 | PID balance | Placed upright by hand, balances 60 s; drifts less than 1 m with wheel sensors on |
| 3 | Policy in simulation | Rears up from tail rest, balances, survives randomised shoves, recovers from both bumpers, across the full randomisation range |
| 4 | Policy on the robot | Rears up unaided 8 times out of 10; balances 60 s; recovers from a shove that makes the PID baseline fall |
| 5 | Policy on board | Same as 4 with the laptop disconnected |
| 6 | Head | ESP32-CAM streaming while balancing; head pan does not upset balance |

## 12. Known risks

| Risk | Sign | Response |
|---|---|---|
| TT backlash and dead zone make balance too wobbly | Large limit-cycle rocking at milestone 2 | Model it and let the policy handle it; raise the centre of mass; as a last resort add a small 6 V N20 gearmotor pair (~$12) |
| SG-90 too weak or strips | Cannot leave tail rest; gear noise after falls | Shallower rest angle; flexure tail; MG90S |
| Wheel sensing too coarse | Position loop hunts | Finer printed disc; or run IMU-only and accept drift |
| Servo rail sags when both servos stall | Tail goes limp or twitches | Bigger capacitor; parallel a second Mini360 is not safe, so use a larger buck (~$3) |
| ESP-NOW loss or jitter | Policy works in sim, twitches on robot | Measure, add the measured delay and loss to the simulator, move policy on board sooner |
