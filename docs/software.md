# Software

> **AI-generated and unbuilt.** The firmware compiles and the protocol tests pass on a laptop, but none of it has
> run on a Tailwhip yet. Gains, filter constants and timings are [C] chosen or [E] estimated until the bench and
> milestone runs replace them.

Three computers run three programs.

| Where | Program | Language | Job | Rate |
|---|---|---|---|---|
| ESP32 on the robot | `firmware/tailwhip_robot` | Arduino C++ | IMU and pitch filter, wheel speed, PWM to the MX1508, two servos, failsafe. Runs the PID baseline (`pid` mode) and the trained policy (`onboard` mode). | 400 Hz control, 100 Hz state packets, 50 Hz policy |
| ESP32 on the laptop's USB | `firmware/tailwhip_bridge` | Arduino C++ | Copy packets between USB serial and ESP-NOW | as they arrive |
| Laptop | `host/` and `sim/` | Python 3.10+ | Teleop, bench measurements, logging, plotting, the simulator, training, and the policy during development | 50 Hz commands |

Unlike Ouro, the robot is not just a relay: balancing needs a fast loop, so the filter, the PID and (later) the policy all run on the robot's ESP32. The laptop can still drive every actuator directly in `remote` mode, which is how a policy runs during development.

## Install

```
python -m venv tailwhip-env && source tailwhip-env/bin/activate
pip install numpy pyserial matplotlib
```

(`pip install numpy pyserial`, plus matplotlib for `log_plot.py`.) Simulation and training need more; see the Simulation page (`sim/`).

Flash the firmware as described on the [Electronics](electronics.md#set-up-the-firmware-toolchain) page: `tailwhip_robot` to the robot, `tailwhip_bridge` to the board that stays on the laptop. `PORT` below is the bridge's serial port: `/dev/cu.usbserial-XXXX` or `/dev/cu.wchusbserial-XXXX` on macOS, `/dev/ttyUSB0` on Linux, `COM5` on Windows.

## What talks to what

```
 sim/ (MuJoCo, PPO) ──export_policy.py──> firmware/tailwhip_robot/policy.h ──> onboard mode
        ▲         └──trained policy──> host/run_policy.py ─┐
        │ measured numbers                                  │ Link.send(u_l, u_r, tail, head, mode)
 host/measurements.json <── host/bench.py ──┐               ▼
                                            host/link.py  <── host/drive.py, host/log_plot.py
                                                 │ USB serial 921600, framed + CRC-8
                                                 ▼
                                       tailwhip_bridge (ESP32)
                                                 │ ESP-NOW broadcast, channel 6
                                                 ▼
                                       tailwhip_robot (ESP32) ── USB serial 115200: bench and tuning commands
                                   PWM ×4 ▼     ▼ PWM ×2          ▲ I2C, GPIO interrupts, ADC
                                  MX1508, 2 TT    2 SG-90     MPU-6050, 2 IR modules, battery
```

## Conventions shared with the simulator

These must match the simulator exactly. They are the contract between `sim/`, `host/` and the firmware.

| Quantity | Convention |
|---|---|
| Body pitch | + = leaning **forward**, 0 = upright, tail-down rest about -25 deg. Radians in the policy, degrees in packets. |
| Tail angle | deg; 0 = straight down the body, + = swung backwards. Servo angle = tail angle (servo 90 = tail straight back). Foot on the floor while upright 24.3, rest 73.6 (also where it waits while balancing: it is the back bumper). Never stow it up the back: a fall from there ends flat on its back. Allowed 20..150. |
| Head pan | deg; servo 90 = facing forward, + = turn left. Allowed 30..150. |
| Wheel command u | -1..1, + = drive forward. Motor voltage = u x 6.0 V (clipped at the battery voltage). |
| Policy action | 3 floats in -1..1: u_left, u_right, a_tail. Tail target = 85 + 65 x a_tail deg (so -1..1 is 20..150). 50 Hz. |
| Wheel speed | rad/s of the wheel **relative to the body** (the sensor is on the body). Sign = sign of the last **non-zero** command to that wheel. 32 edges per turn. |
| Yaw rate | rad/s, + = turning left. |

## Modes

| Mode | Wheels | Tail | Head | Without radio |
|---|---|---|---|---|
| `idle` (0) | off (coast) | rest 73.6 | from the command, else 90 | stays idle |
| `pid` (1) | on-board PID balance; speed and turn setpoints from the command | from the command, else rest 73.6 | command, else 90 | keeps balancing with speed and turn 0 |
| `remote` (2) | u_left, u_right from the command | from the command | from the command | **failsafe**: wheels off, tail to rest |
| `onboard` (3) | policy.h, 50 Hz; stand / speed / turn from the command | policy | command, else 90 | keeps running with speed and turn 0, last stand command |
| `bench` (4) | u x **8.4 V** (no 6 V cap, no tilt cut) | command or serial | command or serial | failsafe if it was driven by radio |

Every command packet carries a mode, so the laptop sets the mode simply by sending it (use `keep`, 255, to leave the mode alone). On the robot's own USB you can type `mode idle|pid|onboard|bench`. The robot refuses `onboard` while `policy.h` is the placeholder, and refuses every mode but `idle` with a low battery.

### Safety rules (all modes)

- **Failsafe:** in `remote` and radio-driven `bench`, no valid command for **100 ms** turns the wheels off (coast) and puts the tail to rest 73.6 deg. Flag `failsafe`.
- **Tilt cut:** in `remote` and `onboard`, |pitch| over **45 deg** turns the wheels off until it is back under 40 deg. Falls stop on the feet at 18 deg and on the tail at 25 deg, well inside 45 deg, so the wheels still work there to get it back up.
- **PID fall:** in `pid` mode the controller disarms when |pitch - trim| passes 30 deg and re-arms only after it has been held within 5 deg of upright for 0.5 s. Flag `pid_fallen` while disarmed.
- **IMU fault:** if the MPU-6050 stops answering, the wheels are off (except in `bench`) and the firmware retries every 500 ms.
- **Low battery:** under 6.4 V for 2 s the robot latches into `idle` (flag `low_batt`, LED blinks slowly). 30 s later it puts itself to sleep. Switch it off and charge.

## Timing on the robot

| Every | What |
|---|---|
| 2.5 ms (400 Hz) | Read the MPU-6050 (14 bytes, ~0.4 ms), update the pitch filter, run the mode, write PWM and servo pulses. PID runs here. |
| 10 ms (100 Hz) | Battery reading, wheel speed update, send a StatePacket, LED. |
| 20 ms (50 Hz) | `onboard` mode: one policy step (about 7,600 multiply-adds, well under 1 ms). |

The pitch filter is complementary: `pitch += rate x dt`, then `pitch += (accel_pitch - pitch) x dt / 0.5 s`, skipping the accelerometer when its magnitude is more than 0.25 g from 1 g. Accelerometer pitch is `atan2(-a_x, a_z)` in the body frame. Details on the [Electronics](electronics.md#imu) page.

## Packets

Both packets are packed little-endian structs defined in `tailwhip_common.h` (one copy in each sketch folder, kept identical) and mirrored in `host/link.py`.

On the USB line between the laptop and the bridge every packet is framed as `0xAA 0x55 <length> <payload> <CRC-8>` (CRC-8 polynomial 0x07, initial 0, the "SMBus" CRC). The bridge drops frames with a bad CRC, a wrong length, magic or robot id. Over ESP-NOW the bare payload travels as a broadcast on channel 6 (no acknowledgement, no retry; the radio has its own CRC).

**CmdPacket**, laptop to robot, 18 bytes, `'<4BH6h'`:

| Offset | Field | Type | Units |
|---|---|---|---|
| 0 | magic | u8 | 0xC7 |
| 1 | robot_id | u8 | `ROBOT_ID` (1) |
| 2 | mode | u8 | 0 idle, 1 pid, 2 remote, 3 onboard, 4 bench, 255 keep |
| 3 | flags | u8 | bit 0 stand (1 = stand), bit 1 servos off (bench only) |
| 4 | seq | u16 | +1 every packet; echoed back as last_cmd_seq |
| 6 | u_left | i16 | u x 10000 |
| 8 | u_right | i16 | u x 10000 |
| 10 | tail | i16 | 0.01 deg |
| 12 | head | i16 | 0.01 deg |
| 14 | v | i16 | forward speed command, mm/s (pid, onboard) |
| 16 | yaw | i16 | turn rate command, mrad/s, + = left (pid, onboard) |

**StatePacket**, robot to laptop, 100 Hz, 42 bytes, `'<4B3HI7hH2h2HhH'`:

| Offset | Field | Type | Units |
|---|---|---|---|
| 0 | magic | u8 | 0x7D |
| 1 | robot_id | u8 | |
| 2 | mode | u8 | current mode |
| 3 | cmd_age_ms | u8 | ms since last_cmd_seq arrived (255 = none or older) |
| 4 | flags | u16 | see below |
| 6 | seq | u16 | +1 every state packet |
| 8 | last_cmd_seq | u16 | seq of the last command received |
| 10 | t_ms | u32 | robot millis() at sampling |
| 14 | pitch | i16 | 0.01 deg (filtered) |
| 16 | pitch_rate | i16 | 0.1 deg/s (gyro, bias removed) |
| 18 | yaw_rate | i16 | 0.1 deg/s |
| 20, 22 | wheel_l, wheel_r | i16 | 0.01 rad/s, signed by the last non-zero command |
| 24 | tail_cmd | i16 | 0.01 deg, what the servo is being told |
| 26 | head_cmd | i16 | 0.01 deg |
| 28 | vbat | u16 | mV |
| 30, 32 | u_left, u_right | i16 | wheel command actually applied, x 10000 (0 when cut) |
| 34, 36 | edges_l, edges_r | u16 | IR edge counters, free-running |
| 38 | pitch_acc | i16 | 0.01 deg, accelerometer-only pitch |
| 40 | cmd_lost | u16 | commands missed (gaps in seq), free-running |

Flags: 0x0001 failsafe, 0x0002 tilt cut, 0x0004 wheel sensors working, 0x0008 IMU fault, 0x0010 low battery, 0x0020 motors driven, 0x0040 servos on, 0x0080 PID disarmed (fallen), 0x0100 trained policy loaded, 0x0200 no battery (USB only), 0x0400 a wheel asked for more than the battery has, 0x0800 radio link alive.

`cmd_age_ms` lets the laptop measure the true round trip: (time the state arrived - time that command was sent) - cmd_age_ms.

## The PID baseline

`pid` mode is the hand-written controller: the comparison for RL, the fallback, and milestone 2. It runs at 400 Hz on the robot:

```
theta = pitch - trim                                  (rad)
v     = ground speed, m/s: r x (mean wheel speed + pitch rate)   with the wheel sensors
      = first-order model of the motor: lag 80 ms towards u x 26.2 rad/s x r   without them
x     = integral of (v - v_cmd), clamped to +-0.5 m
u     = kp theta + kd theta_dot + ki integral(theta) + kv (v - v_cmd) + kx x
turn  = kyff w_cmd + kyaw (w_cmd - yaw_rate)
u_left = u - turn,  u_right = u + turn  (plus the dead-zone term dz, then clipped to +-1)
```

All gains are positive. kv and kx have the same sign as kp because a balancing robot that drifts forward has to speed up first, so the body tips back, before it can slow down.

Starting gains [C]:

| Gain | Value | Units |
|---|---|---|
| kp | 3.0 | u per rad (full 6 V at 19 deg) |
| kd | 0.40 | u per rad/s |
| ki | 0 | u per rad s |
| kv | 2.0 | u per m/s |
| kx | 0.3 | u per m |
| kyff, kyaw | 0.06, 0.03 | u per rad/s |
| dz | 0.10 | u added in the direction of motion (set it to your measured dead zone) |

They come from a grid search in the MuJoCo model (`sim/evaluate.py` runs the same controller as its PID baseline), which has the backlash, the dead zone, the coarse encoders and the radio delay. On the nominal simulated robot they hold it within 5 deg 85 % of the time; across randomised robots, much less. A plain PID struggles with sloppy TT gearboxes, which is why the project has a policy. Expect to tune on your robot.

### Tuning on the robot

Do it over the robot's USB (Serial Monitor 115200, or `screen PORT 115200`) with `s` streaming, holding the robot upright by hand above a soft surface.

1. **Find the balance point.** Wheels off (`mode idle`). Hold the robot where it feels balanced, read pitch, and set `trim <deg>` to it.
2. **IMU-only first.** `wheels off`, `kv 0`, `kx 0`. `mode pid`, hold it upright until it arms. With kp alone it should push back against your hand. Raise `kp` until it oscillates fast (a few Hz buzz), then back off 30 %.
3. **kd** damps that oscillation. Raise it until the robot stops rocking; too much gives a high-pitched jitter (gyro noise through the gain).
4. **Speed loop.** `kv 1`, then up to 2-3. Without it the robot runs away across the floor; with too much it rocks slowly back and forth.
5. **Position loop.** `kx 0.5` to 1. It pulls the robot back to where it started. Too much: slow oscillation of a few cm.
6. **Wheel sensors back on.** `wheels auto`. The speed now comes from the IR rings and drift falls.
7. **Dead zone.** If it rocks in a small limit cycle, set `dz` to the measured `motor_deadzone_u` (from `bench.py motor`) and lower it until the rocking just stops.
8. `save` stores everything in flash. `gains` prints it; `defaults` restores the starting values.

Pass mark (milestone 2): placed upright by hand, balances 60 s, drifts less than 1 m with the wheel sensors on.

## The policy: remote and onboard

A policy maps the last 4 observation steps to 3 actions at 50 Hz. Training, the reward and the export are on the Simulation page (`sim/`); this section is only how the robot side consumes the result.

**Observation**, 13 floats per step, built by `link.obs_step(state, prev_action, cmd)` on the laptop and by `obsStep()` in the firmware, identically:

| i | Value |
|---|---|
| 0 | pitch, rad |
| 1 | pitch rate, rad/s, / 5 |
| 2 | yaw rate, rad/s, / 5 |
| 3, 4 | left, right wheel speed, rad/s, / 20 |
| 5 | tail action sent last step, a_tail |
| 6, 7, 8 | the action sent the step before that: u_left, u_right, a_tail (at start-up, the same as the last one) |
| 9 | (battery V - 7.6) / 0.8 |
| 10 | stand command: 1 stand, 0 rest / lie down |
| 11 | forward speed command, m/s, / 0.3 |
| 12 | turn rate command, rad/s, / 2 |

The policy input is the last 4 steps stacked **newest first** (52 floats). When the policy starts, all 4 slots are filled with the first step, and the previous action starts as [0, 0, (current tail command - 85) / 65]. `link.ObsStack` does the stacking on the laptop the same way.

**Remote** (`host/run_policy.py`, see the Simulation page): the laptop reads the latest state, builds the observation, runs the network and sends `Link.send(u_left, u_right, 85 + 65 a_tail, head, 'remote')` every 20 ms. The radio delay sits inside the loop; `bench.py radio` measures it.

**Onboard**: `sim/export_policy.py` writes `firmware/tailwhip_robot/policy.h`, then you reflash `tailwhip_robot`. The format:

```
#define TW_OBS_DIM 52
#define TW_HID 64
#define TW_ACT_DIM 3
static const float TW_W0[TW_HID][TW_OBS_DIM], TW_B0[TW_HID];
static const float TW_W1[TW_HID][TW_HID],     TW_B1[TW_HID];
static const float TW_W2[TW_ACT_DIM][TW_HID], TW_B2[TW_ACT_DIM];
static const float TW_OBS_MEAN[TW_OBS_DIM], TW_OBS_STD[TW_OBS_DIM];   // x_n = (x - mean) / std
```

Hidden layers are tanh, the output is linear and then clipped to -1..1. The repository ships the trained policy already exported (`sim/policies/tailwhip.zip`). A placeholder file that defines `TW_POLICY_PLACEHOLDER` (all zeros) makes the robot refuse `onboard` mode; the exporter's file does not define it. `host/test_protocol.py` checks that a network written in this format gives the same actions in C++ and numpy. With the laptop gone, the robot keeps running the policy with speed and turn 0 and the last stand command.

## The robot's own USB: serial commands

Serial Monitor at 115200, one command per line. `help` lists them.

| Command | Does |
|---|---|
| `p` / `s` | print one status line / toggle a 10 Hz stream |
| `mode idle\|pid\|onboard\|bench` | switch mode (remote is radio only) |
| `m <VL> <VR> [sec]` | bench: wheels at VL, VR volts for sec (default 2 s, up to 8.4 V) |
| `tail <deg>`, `head <deg>` | bench: servo angle |
| `sweep tail\|head` | bench: sweep the full range at 30 deg/s and back |
| `servos on\|off` | bench: servo pulses on or off |
| `stand 0\|1`, `go <v> <w>` | setpoints for pid and onboard when there is no radio |
| `kp kd ki kv kx kyff kyaw dz <value>`, `trim <deg>` | PID gains, dead zone, balance point |
| `ttrim <us>`, `htrim <us>` | servo trim |
| `flip l\|r\|tail\|head` | reverse a motor or a servo |
| `wheels auto\|off` | use the IR wheel sensors or run IMU-only |
| `cal` | redo the gyro bias (hold still) |
| `gains`, `save`, `defaults` | print, store in flash, restore defaults |

## Laptop tools

```
cd host
python test_protocol.py                    # no hardware needed: packet layout, framing, CRC, obs, policy.h format
python drive.py PORT --log drive1.npz      # milestone 1: drive on the tail
python bench.py radio PORT                 # and the other bench subcommands
python log_plot.py drive1.npz              # or --live PORT
```

| Script | Use it to |
|---|---|
| `link.py` | The only file that knows about bytes. `Link(port)`: `.state()` latest `State` or None, `.send(u_left, u_right, tail_deg, head_deg, mode, stand=1, v_mps=0, yaw_rps=0)`, `.start_log(path)` / `.stop_log()` (every state and command, `.npz` or `.csv`), `.close()`, a background reader thread, `.rtt_ms`. Also `obs_step()`, `ObsStack`, `load_log()`. |
| `test_protocol.py` | Check everything above against the firmware headers with a C++ compiler and a fake serial port. Run it after any change to a packet. |
| `drive.py` | Keyboard teleop (w/s a/d, tail and head keys, `p` for PID). Milestone 1. `--max` caps u (default 0.5 = 3 V). |
| `bench.py` | The SPEC section 9 measurements, one subcommand each (below). |
| `log_plot.py` | Plot a log, or a live 10 s window (`--live PORT`, optionally `--mode pid --log file` to keep the robot in PID and record it). |

### Bench tools

`bench.py` writes `host/measurements.json`, which the simulator reads. Each subcommand updates only its own keys and records the date and method in `<key>_source`.

| Subcommand | Measures | Keys |
|---|---|---|
| `motor PORT` | Wheels free on a stand: speed at 3, 5 and 6.5 V (bench mode, so 6.5 V is allowed), the currents from your meter, a 2 s stall at 6 V (meter, and optionally a stick on a kitchen scale for torque), then on the floor resting on the tail: ramps the voltage until each wheel moves. | `motor_no_load_rad_s_at_6v`, `motor_stall_torque_nm_at_6v`, `motor_deadzone_u`, `motor_detail` |
| `backlash` | Free play at the tread, by hand with a ruler, 3 times per wheel. | `backlash_deg` |
| `servo PORT` | Unloaded 60 deg moves filmed in slow motion (you count frames); loaded: tail from 133.6 to 73.6 deg lifting the body, timed by the IMU. | `servo_deg_per_s`, `servo_deg_per_s_loaded` |
| `radio PORT` | 20 s at 50 Hz and 20 s at 100 Hz: round trip and loss both ways. Loop delay = median round trip + 5 ms (average age of a 100 Hz state) + 1.25 ms (half a robot tick), laptop compute not included. | `loop_delay_ms`, `packet_loss` (commands lost at 50 Hz), `packet_loss_detail` |
| `mass` | Kitchen scale; centre of mass by balancing the robot on an edge. | `mass_g`, `com_height_mm` |
| `friction` | Robot sideways on a tilted board of your demo surface. | `tyre_mu` |
| `show` | Print the file. | |

Units: `motor_deadzone_u` is in u (u = 1 is 6 V); `backlash_deg` is degrees at the wheel; `com_height_mm` is along the body from the axle. Until a key exists, the simulator uses its own estimates.
