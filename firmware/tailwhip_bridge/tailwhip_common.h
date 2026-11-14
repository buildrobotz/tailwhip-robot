// Shared by tailwhip_robot and tailwhip_bridge. Keep the two copies IDENTICAL
// (firmware/tailwhip_robot/ and firmware/tailwhip_bridge/). host/test_protocol.py checks that they are.
// Byte layout is little-endian and packed; host/link.py mirrors it with struct formats
//   CmdPacket    '<4BH6h'              (18 bytes)  laptop -> bridge -> robot
//   StatePacket  '<4B3HI7hH2h2HhH'     (42 bytes)  robot -> bridge -> laptop, 100 Hz
// On the USB serial line between the laptop and the bridge every packet is framed as
//   0xAA 0x55 <len> <payload...> <crc8 of payload>      (CRC-8, polynomial 0x07, init 0x00)
// ESP-NOW carries the bare payload (the radio has its own CRC).
#pragma once
#include <stdint.h>

#define ROBOT_ID      1      // change if two Tailwhips share a room
#define WIFI_CHANNEL  6      // 1, 6 or 11; pick the quietest

#define CMD_MAGIC    0xC7
#define STATE_MAGIC  0x7D

// Modes (CmdPacket.mode, StatePacket.mode)
#define MODE_IDLE     0      // motors off (coast), tail at rest 73.6 deg, head forward
#define MODE_PID      1      // on-board PID balance; wheel u from the PID, tail from the command
#define MODE_REMOTE   2      // u_left, u_right, tail, head straight from the laptop (policy or teleop)
#define MODE_ONBOARD  3      // policy.h runs on the ESP32 at 50 Hz; the command gives stand / speed / turn
#define MODE_BENCH    4      // u_left, u_right are fractions of 8.4 V (no 6 V cap, no tilt cut); for bench.py
#define MODE_KEEP   255      // CmdPacket only: refresh the link, keep the current mode

// CmdPacket.flags
#define CF_STAND      0x01   // stand command (policy obs[10]): 1 = stand, 0 = rest / lie down
#define CF_SERVOS_OFF 0x02   // no servo pulses (bench only: lets you turn the horns by hand)

// StatePacket.flags
#define SF_FAILSAFE      0x0001  // no valid command for 100 ms: motors off, tail to rest (remote and bench)
#define SF_TILT_CUT      0x0002  // |pitch| > 45 deg: motors cut until |pitch| < 40 deg
#define SF_WHEELS        0x0004  // wheel IR sensors seen working (edges while driving)
#define SF_IMU_FAULT     0x0008  // MPU-6050 not answering: pitch holds its last value, motors cut
#define SF_LOW_BATT      0x0010  // pack under 6.4 V for 2 s (latched until reset): forced to idle
#define SF_MOTORS_ON     0x0020  // the H-bridge is being driven this tick
#define SF_SERVOS_ON     0x0040  // servo pulses are being sent
#define SF_PID_FALLEN    0x0080  // pid mode disarmed after a fall; re-arms when held upright 0.5 s
#define SF_POLICY        0x0100  // policy.h holds a trained network (not the zero placeholder)
#define SF_NO_BATT       0x0200  // pack under 3 V: running from USB, motors cannot move
#define SF_VOLT_CLIP     0x0400  // a wheel asked for more voltage than the pack has (duty clipped at 100 %)
#define SF_LINK          0x0800  // a valid radio command arrived in the last 100 ms

struct __attribute__((packed)) CmdPacket {     // 18 bytes
  uint8_t  magic;          // 0xC7
  uint8_t  robot_id;
  uint8_t  mode;           // MODE_*
  uint8_t  flags;          // CF_*
  uint16_t seq;            // increments every packet; the robot echoes it in StatePacket.last_cmd_seq
  int16_t  u_left;         // wheel command x 10000, -10000..10000 (+ = forward). Voltage = u x 6.0 V
  int16_t  u_right;
  int16_t  tail_cdeg;      // tail target, 0.01 deg (allowed 20..150 deg)
  int16_t  head_cdeg;      // head pan target, 0.01 deg (90 = forward, + = left)
  int16_t  v_mm_s;         // forward speed command, mm/s (pid and onboard modes)
  int16_t  yaw_mrad_s;     // turn rate command, mrad/s, + = left (pid and onboard modes)
};

struct __attribute__((packed)) StatePacket {   // 42 bytes, 100 Hz
  uint8_t  magic;          // 0x7D
  uint8_t  robot_id;
  uint8_t  mode;
  uint8_t  cmd_age_ms;     // ms since last_cmd_seq arrived, saturates at 255 (round-trip timing)
  uint16_t flags;          // SF_*
  uint16_t seq;
  uint16_t last_cmd_seq;
  uint32_t t_ms;           // robot millis() when this state was sampled
  int16_t  pitch_cdeg;     // filtered body pitch, 0.01 deg, + = leaning forward, 0 = upright
  int16_t  pitch_rate_ddps;// gyro pitch rate, 0.1 deg/s (bias removed, unfiltered)
  int16_t  yaw_rate_ddps;  // gyro yaw rate, 0.1 deg/s, + = turning left
  int16_t  wheel_l_crad_s; // wheel speed, 0.01 rad/s, sign = sign of that wheel's last non-zero command
  int16_t  wheel_r_crad_s;
  int16_t  tail_cmd_cdeg;  // tail angle being commanded to the servo, 0.01 deg
  int16_t  head_cmd_cdeg;
  uint16_t vbat_mV;        // pack voltage (filtered)
  int16_t  u_left;         // wheel command actually applied this tick, x 10000 (after cuts)
  int16_t  u_right;
  uint16_t edges_l;        // IR edge counters, free-running, wrap at 65536 (32 edges per wheel turn)
  uint16_t edges_r;
  int16_t  pitch_acc_cdeg; // accelerometer-only pitch, 0.01 deg (for checking the filter)
  uint16_t cmd_lost;       // commands missed (gaps in CmdPacket.seq), free-running
};

static_assert(sizeof(CmdPacket) == 18, "CmdPacket must be 18 bytes");
static_assert(sizeof(StatePacket) == 42, "StatePacket must be 42 bytes");

// CRC-8, polynomial 0x07, init 0, no reflection (CRC-8/SMBUS). host/link.py has the same function.
static inline uint8_t tw_crc8(const uint8_t *p, int n) {
  uint8_t c = 0;
  for (int i = 0; i < n; i++) {
    c ^= p[i];
    for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}
