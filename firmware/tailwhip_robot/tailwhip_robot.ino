// Tailwhip, on-robot firmware. ESP32 DevKit (ESP32-WROOM-32, classic ESP32). Board: "ESP32 Dev Module"
// (FQBN esp32:esp32:esp32). Builds on Arduino-ESP32 core 2.0.x and 3.x, no extra libraries.
//
// Every 2.5 ms (400 Hz): read the MPU-6050, update the pitch filter, run the active mode, drive the
// MX1508 and the two servos. Every 10 ms (100 Hz): estimate wheel speeds, broadcast a StatePacket.
// Every 20 ms (50 Hz) in onboard mode: run the policy in policy.h.
//
// Modes: idle, pid (on-board balance), remote (laptop commands), onboard (policy on the ESP32), bench.
// Safety: no valid radio command for 100 ms in remote/bench -> motors off, tail to rest. |pitch| > 45 deg
// -> motors off. Pack under 6.4 V for 2 s -> idle, then deep sleep 30 s later.
// Wiring, pin map and bring-up: docs/electronics.md. Modes, packets, tuning: docs/software.md.
// Type "help" in the Serial Monitor (115200 baud) for the bench and tuning commands.
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>
#include "tailwhip_common.h"
#include "policy.h"

// ---------------------------------------------------------------- pins (docs/electronics.md, pin map)
const uint8_t PIN_ML1 = 25, PIN_ML2 = 26;     // MX1508 left motor IN1, IN2
const uint8_t PIN_MR1 = 27, PIN_MR2 = 14;     // MX1508 right motor IN1, IN2
const uint8_t PIN_TAIL = 18, PIN_HEAD = 19;   // SG-90 signal
const uint8_t PIN_SDA = 21, PIN_SCL = 22;     // MPU-6050 (GY-521)
const uint8_t PIN_IR_L = 34, PIN_IR_R = 35;   // TCRT5000 module D0, input-only pins, 100k pull-downs on the board
const uint8_t PIN_BATT = 36;                  // VP, ADC1_CH0, 100k / 33k divider
const uint8_t PIN_LED = 2;
// LEDC channels: 0-3 motors (timers 0, 1 at 20 kHz), 4-5 servos (timer 2 at 50 Hz). Channel pairs share a timer.
const uint8_t CH_ML1 = 0, CH_ML2 = 1, CH_MR1 = 2, CH_MR2 = 3, CH_TAIL = 4, CH_HEAD = 5;
const uint8_t MPU = 0x68;

// ---------------------------------------------------------------- fixed numbers ([D] datasheet, [E] estimate, [C] chosen)
const uint32_t TICK_US = 2500;            // [C] 400 Hz control tick
const uint32_t MOTOR_PWM_HZ = 20000;      // [C] above hearing; MX1508 datasheet gives no limit. Drop to 5000 if it runs hot.
const uint8_t  MOTOR_BITS = 10;           // [C] 1024 steps
const uint32_t MOTOR_MAX = (1u << MOTOR_BITS) - 1;
const float    V_MOTOR_MAX = 6.0f;        // [C] TT motors are rated 6 V: u = 1 means 6.0 V average at the motor
const float    V_BENCH_FULL = 8.4f;       // [C] bench mode: u = 1 means 8.4 V (so bench.py can test at 6.5 V)
const float    BATT_DIV = (100.0f + 33.0f) / 33.0f;   // [C] 100k over 33k: 8.4 V -> 2.08 V at the pin
const float    BATT_CAL = 1.000f;         // trim: meter reading / reported value (bring-up step 7)
const uint16_t LOW_BATT_MV = 6400;        // [C] 3.2 V per cell under load
const uint32_t LOW_BATT_HOLD_MS = 2000, SLEEP_AFTER_LOW_MS = 30000;
const uint32_t LINK_TIMEOUT_MS = 100;     // SPEC 7: no valid command for 100 ms -> failsafe
const float    TILT_CUT_DEG = 45.0f, TILT_RESUME_DEG = 40.0f;
const float    PID_FALL_DEG = 30.0f, PID_ARM_DEG = 5.0f;  const uint32_t PID_ARM_MS = 500;
const float    TAIL_REST = 73.6f, TAIL_MIN = 20.0f, TAIL_MAX = 150.0f;   // mechanical.md
// Rev B: the tail waits at TAIL_REST while balancing. It is the back bumper; never stow it up the back.
const float    HEAD_MIN = 30.0f, HEAD_MAX = 150.0f;
const float    US_PER_DEG = 2000.0f / 180.0f;   // [C] 500..2500 us over 0..180 deg; SG-90 clones vary, check with servo_center
// IMU: +-4 g, +-500 dps, DLPF 2 (accel 94 Hz / gyro 98 Hz, about 3 ms delay [D] MPU-6000 register map)
const float    ACC_LSB = 8192.0f, GYR_LSB = 65.5f;
const float    FILTER_TAU = 0.5f;         // [C] complementary filter time constant, s: gyro below, accelerometer above
const float    ACC_GATE_G = 0.25f;        // [C] ignore the accelerometer when |a| is more than 0.25 g away from 1 g
// Wheels: 16-window ring, both edges counted = 32 edges per turn (mechanical.md)
const float    RAD_PER_EDGE = 2.0f * PI / 32.0f;
const uint32_t IR_LOCKOUT_US = 1000;      // [C] ignore edges closer than 1 ms (comparator chatter); top speed is ~9 ms/edge
const uint32_t WHEEL_STOP_US = 250000;    // [C] no edge for 250 ms = stopped
const float    WHEEL_R = 0.0325f;         // [D] 65 mm wheel
const float    WHEEL_NOLOAD_6V = 26.2f;   // [D] 250 rpm at 6 V, Adafruit 3777 TT 1:48 page (measure: bench.py motor)
const float    V_EST_TAU = 0.08f;         // [E] IMU-only speed model: wheel speed lags the command by ~80 ms

// ---------------------------------------------------------------- settings kept in flash (serial "save")
struct Config {
  uint32_t version;
  float kp, kd, ki, kv, kx, kyff, kyaw, deadzone, pitchTrimDeg;
  int8_t  dirL, dirR, dirTail, dirHead;    // +1 / -1
  int16_t trimTailUs, trimHeadUs;
  uint8_t wheelsMode;                      // 0 auto, 1 force off (IMU-only)
};
const uint32_t CFG_VERSION = 4;
// PID starting gains [C], from a planar model of rev A (383 g, CoM 76 mm, TT 0.078 N m stall at 6 V) that stays
// stable for 10-40 ms loop delay and a 0.1 dead zone. Units: u per rad, per rad/s, per m/s, per m.
// PID defaults tuned in the MuJoCo model (sim/evaluate.py pid_policy, the same structure), rev B, 396 g.
const Config DEFAULTS = {CFG_VERSION, 3.0f, 0.40f, 0.0f, 2.0f, 0.3f, 0.06f, 0.03f, 0.10f, 0.0f,
                         1, 1, 1, 1, 0, 0, 0};
Config cfg = DEFAULTS;
Preferences prefs;

// ---------------------------------------------------------------- state
uint8_t broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;
CmdPacket rxCmd = {};  uint32_t rxMs = 0;  bool rxAny = false;  uint16_t cmdLost = 0;

volatile uint32_t edgeCount[2] = {0, 0}, lastEdgeUs[2] = {0, 0};
uint32_t prevEdgeN[2] = {0, 0}, prevEdgeUs[2] = {0, 0};
float wheelRel[2] = {0, 0};               // |wheel speed| relative to the body, rad/s
int8_t wheelSign[2] = {1, 1};
bool irOk[2] = {false, false};  uint32_t irNoEdgeSince[2] = {0, 0};

struct Dev { bool ok; uint8_t fails; } imu = {false, 0};
float gyroBias[3] = {0, 0, 0};
float pitch = 0, pitchAcc = 0, pitchRate = 0, yawRate = 0;   // rad, rad, rad/s, rad/s
float vbat = 0;  bool lowBatt = false;  uint32_t lowBattSince = 0, lowBattAt = 0;

uint8_t mode = MODE_IDLE;
float uApplied[2] = {0, 0}, tailCmd = TAIL_REST, headCmd = 90.0f;
bool motorsOn = false, servosOn = false, tiltCut = false, voltClip = false, failsafe = false;
// pid
bool pidArmed = false;  uint32_t pidUprightSince = 0;  float pidInteg = 0, pidX = 0, vEst = 0, uBalPrev = 0;
// onboard policy
float obsHist[TW_OBS_DIM], prevAct[3] = {0, 0, 0}, prevAct2[3] = {0, 0, 0}, polAct[3] = {0, 0, 0};
// prevAct = the action sent last step, prevAct2 = the one before it (the simulator's layout, sim/tailwhip_env.py)
// setpoints when there is no radio (serial "stand", "go")
uint8_t serialStand = 1;  float serialV = 0, serialW = 0;
// serial bench
uint32_t benchUntil = 0;  float benchV[2] = {0, 0};  bool benchServosOff = false;
int sweepWhich = -1;  float sweepDeg = 0, sweepDir = 1;  uint8_t sweepLeg = 0;
float benchTail = TAIL_REST, benchHead = 90.0f;
bool benchFromRadio = false;
bool stream = false;
uint16_t seq = 0;

// ---------------------------------------------------------------- LEDC, core 2.x and 3.x
void pwmAttach(uint8_t pin, uint8_t ch, uint32_t hz, uint8_t bits) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(pin, hz, bits, ch);
#else
  ledcSetup(ch, hz, bits);  ledcAttachPin(pin, ch);
#endif
}
void pwmWrite(uint8_t pin, uint8_t ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;  ledcWrite(pin, duty);
#else
  (void)pin;  ledcWrite(ch, duty);
#endif
}

// ---------------------------------------------------------------- ESP-NOW
void handleCmd(const uint8_t *data, int len) {
  if (len != (int)sizeof(CmdPacket) || data[0] != CMD_MAGIC || data[1] != ROBOT_ID) return;
  portENTER_CRITICAL(&cmdMux);
  uint16_t s = (uint16_t)(data[4] | (data[5] << 8));
  if (rxAny) { uint16_t gap = (uint16_t)(s - rxCmd.seq - 1); if (gap > 0 && gap < 1000) cmdLost += gap; }
  memcpy(&rxCmd, data, sizeof(CmdPacket));
  rxMs = millis();  rxAny = true;
  portEXIT_CRITICAL(&cmdMux);
}
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) { (void)info; handleCmd(data, len); }
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) { (void)mac; handleCmd(data, len); }
#endif

// ---------------------------------------------------------------- IR wheel sensors
void IRAM_ATTR isrL() { uint32_t t = micros(); if (t - lastEdgeUs[0] > IR_LOCKOUT_US) { edgeCount[0] = edgeCount[0] + 1; lastEdgeUs[0] = t; } }
void IRAM_ATTR isrR() { uint32_t t = micros(); if (t - lastEdgeUs[1] > IR_LOCKOUT_US) { edgeCount[1] = edgeCount[1] + 1; lastEdgeUs[1] = t; } }

// Period method: speed = edges since the last update / time between the last edges of the two updates.
// With no new edge the estimate can only fall (1 edge / time since the last edge), and is 0 after 250 ms.
void updateWheels(uint32_t nowUs, uint32_t nowMs) {
  for (int s = 0; s < 2; s++) {
    uint32_t n, t;
    noInterrupts();  n = edgeCount[s];  t = lastEdgeUs[s];  interrupts();
    uint32_t dn = n - prevEdgeN[s];
    if (dn > 0) {
      uint32_t dt = t - prevEdgeUs[s];
      wheelRel[s] = (dt > 0 && dt < 1000000) ? dn * RAD_PER_EDGE / (dt * 1e-6f) : 0.0f;
      prevEdgeN[s] = n;  prevEdgeUs[s] = t;
      irOk[s] = true;  irNoEdgeSince[s] = 0;
    } else {
      uint32_t since = nowUs - prevEdgeUs[s];
      if (since > WHEEL_STOP_US) wheelRel[s] = 0;
      else wheelRel[s] = fminf(wheelRel[s], RAD_PER_EDGE / (since * 1e-6f));
      // Sensor missing or misaligned: driven hard for 1 s with no edge at all.
      if (fabsf(uApplied[s]) > 0.35f && motorsOn && vbat > 6.0f) {
        if (!irNoEdgeSince[s]) irNoEdgeSince[s] = nowMs ? nowMs : 1;
        else if (nowMs - irNoEdgeSince[s] > 1000) irOk[s] = false;
      } else irNoEdgeSince[s] = 0;
    }
  }
}
bool wheelsPresent() { return cfg.wheelsMode == 0 && irOk[0] && irOk[1]; }
float wheelSigned(int s) { return wheelSign[s] * wheelRel[s]; }

// ---------------------------------------------------------------- I2C and the MPU-6050
bool wr8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool rd(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(MPU); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, (size_t)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}
void i2cRecover() {        // free a bus held low by a half-finished byte: up to 9 clocks, then a STOP
  Wire.end();
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, OUTPUT_OPEN_DRAIN);  digitalWrite(PIN_SCL, HIGH);
  for (int i = 0; i < 9 && digitalRead(PIN_SDA) == LOW; i++) {
    digitalWrite(PIN_SCL, LOW);  delayMicroseconds(5);  digitalWrite(PIN_SCL, HIGH);  delayMicroseconds(5);
  }
  pinMode(PIN_SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_SDA, LOW);  delayMicroseconds(5);  digitalWrite(PIN_SDA, HIGH);  delayMicroseconds(5);
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  Wire.setTimeOut(3);
}
bool initImu() {
  imu.ok = wr8(0x6B, 0x01)      // wake, PLL on gyro X
        && wr8(0x19, 0x00)      // sample rate divider 0: 1 kHz with the DLPF on
        && wr8(0x1A, 0x02)      // DLPF 2: accel 94 Hz, gyro 98 Hz, ~3 ms delay
        && wr8(0x1B, 0x08)      // +-500 dps
        && wr8(0x1C, 0x08);     // +-4 g
  imu.fails = 0;
  return imu.ok;
}
inline int16_t be16(const uint8_t *b) { return (int16_t)(((uint16_t)b[0] << 8) | b[1]); }

// Body frame (cad/tailwhip.py): x forward, y left, z up the body. The GY-521 sits on the motor web with the chip
// facing forward and its x axis up the body, so body x = chip z, body y = -chip y, body z = chip x.
// If your board is mounted differently, change only these six lines (bench: "imu", then tilt the robot).
bool readImu(float acc[3], float gyr[3]) {
  if (!imu.ok) return false;
  uint8_t b[14];
  if (!rd(0x3B, b, 14)) { if (++imu.fails >= 3) imu.ok = false; return false; }
  imu.fails = 0;
  float ax = be16(b) / ACC_LSB, ay = be16(b + 2) / ACC_LSB, az = be16(b + 4) / ACC_LSB;          // g
  float gx = be16(b + 8) / GYR_LSB, gy = be16(b + 10) / GYR_LSB, gz = be16(b + 12) / GYR_LSB;   // deg/s
  acc[0] = az;  acc[1] = -ay;  acc[2] = ax;
  gyr[0] = gz * DEG_TO_RAD;  gyr[1] = -gy * DEG_TO_RAD;  gyr[2] = gx * DEG_TO_RAD;
  return true;
}
// Pitch (+ = leaning forward): gravity seen by the accelerometer in the body frame is (-sin p, 0, cos p) g.
float accelPitch(const float a[3]) { return atan2f(-a[0], a[2]); }

bool calibrateGyro() {     // robot must be still: 1 s of samples, rejected if it moved
  for (int attempt = 0; attempt < 3; attempt++) {
    double s[3] = {0, 0, 0}, s2[3] = {0, 0, 0};  int n = 0;
    float a[3], g[3];
    for (int i = 0; i < 400; i++) {
      if (readImu(a, g)) { for (int k = 0; k < 3; k++) { s[k] += g[k]; s2[k] += g[k] * g[k]; }  n++; }
      delayMicroseconds(2500);
    }
    if (n < 300) continue;
    bool still = true;
    for (int k = 0; k < 3; k++) { double m = s[k] / n, var = s2[k] / n - m * m; if (var > 2.5e-4) still = false; }  // 0.9 deg/s rms
    if (still) { for (int k = 0; k < 3; k++) gyroBias[k] = s[k] / n;  return true; }
    Serial.println("gyro cal: robot moved, retrying");
  }
  return false;
}

// ---------------------------------------------------------------- actuators
// MX1508 truth table: IN1 H / IN2 L forward, L / H reverse, H / H brake, L / L coast.
// SLOW DECAY: forward holds IN1 high and PWMs IN2, so the off-time is BRAKE (both inputs high), not
// coast. The average motor voltage is then duty x Vbat whatever the current does, so speed is close to linear
// in duty down to low speeds, and u = 0 shorts the motor (back-EMF braking), the same as 0 V in the simulator.
void driveMotor(int side, float d) {       // d = signed duty, -1..1, already direction-corrected
  uint8_t p1 = side ? PIN_MR1 : PIN_ML1, p2 = side ? PIN_MR2 : PIN_ML2;
  uint8_t c1 = side ? CH_MR1 : CH_ML1, c2 = side ? CH_MR2 : CH_ML2;
  uint32_t on = (uint32_t)lroundf(fminf(fabsf(d), 1.0f) * MOTOR_MAX);
  if (d >= 0) { pwmWrite(p1, c1, MOTOR_MAX);  pwmWrite(p2, c2, MOTOR_MAX - on); }
  else        { pwmWrite(p2, c2, MOTOR_MAX);  pwmWrite(p1, c1, MOTOR_MAX - on); }
}
void coastMotor(int side) {
  if (side) { pwmWrite(PIN_MR1, CH_MR1, 0);  pwmWrite(PIN_MR2, CH_MR2, 0); }
  else      { pwmWrite(PIN_ML1, CH_ML1, 0);  pwmWrite(PIN_ML2, CH_ML2, 0); }
}
// u (wheel command, -1..1) -> duty. vFull = volts that u = 1 stands for (6.0 V normally, 8.4 V in bench mode).
void setWheels(float uL, float uR, float vFull) {
  float vb = vbat > 5.0f ? vbat : 7.4f;    // on USB only there is no motor supply anyway
  float u[2] = {uL, uR};  int8_t dir[2] = {cfg.dirL, cfg.dirR};
  voltClip = false;
  for (int s = 0; s < 2; s++) {
    float d = u[s] * vFull / vb;
    if (fabsf(d) > 1.0f) { voltClip = true;  d = d > 0 ? 1.0f : -1.0f; }
    driveMotor(s, dir[s] * d);
    uApplied[s] = u[s];
    if (u[s] > 0.005f) wheelSign[s] = 1; else if (u[s] < -0.005f) wheelSign[s] = -1;
  }
  motorsOn = true;
}
void motorsOff() { coastMotor(0);  coastMotor(1);  uApplied[0] = uApplied[1] = 0;  motorsOn = false; }

uint32_t servoDuty(float deg, int8_t dir, int16_t trimUs) {
  float us = 1500.0f + dir * (deg - 90.0f) * US_PER_DEG + trimUs;
  us = constrain(us, 500.0f, 2500.0f);
  return (uint32_t)lroundf(us * 65536.0f / 20000.0f);     // 16-bit duty at 50 Hz
}
void setServos(bool on, float tail, float head) {
  tailCmd = constrain(tail, TAIL_MIN, TAIL_MAX);  headCmd = constrain(head, HEAD_MIN, HEAD_MAX);
  servosOn = on;
  pwmWrite(PIN_TAIL, CH_TAIL, on ? servoDuty(tailCmd, cfg.dirTail, cfg.trimTailUs) : 0);
  pwmWrite(PIN_HEAD, CH_HEAD, on ? servoDuty(headCmd, cfg.dirHead, cfg.trimHeadUs) : 0);
}

// ---------------------------------------------------------------- PID baseline
// u = kp*theta + kd*theta_dot + ki*int(theta) + kv*(v - v_cmd) + kx*int(v - v_cmd)    (theta = pitch - trim, rad)
// All gains positive. kv and kx have the same sign as kp: to stop a forward drift the wheels first speed up, the body
// tips back, and then they slow down. v is ground speed from the wheel sensors; without them it comes from a
// first-order model of the motor (u x 26.2 rad/s x r, lag 80 ms), so the robot still balances but drifts.
void pidReset() { pidInteg = 0;  pidX = 0;  vEst = 0;  uBalPrev = 0; }
void pidStep(float vCmd, float wCmd, float out[2]) {
  const float dt = TICK_US * 1e-6f;
  float th = pitch - cfg.pitchTrimDeg * DEG_TO_RAD;
  vEst += (uBalPrev * WHEEL_NOLOAD_6V * WHEEL_R - vEst) * dt / V_EST_TAU;
  // The IR ring measures the wheel relative to the body; ground speed adds the body's own pitch rate.
  float v = wheelsPresent() ? WHEEL_R * (0.5f * (wheelSigned(0) + wheelSigned(1)) + pitchRate) : vEst;
  pidInteg = constrain(pidInteg + th * dt, -0.2f, 0.2f);
  pidX = constrain(pidX + (v - vCmd) * dt, -0.5f, 0.5f);
  float u = cfg.kp * th + cfg.kd * pitchRate + cfg.ki * pidInteg + cfg.kv * (v - vCmd) + cfg.kx * pidX;
  u = constrain(u, -1.0f, 1.0f);
  uBalPrev = u;
  float turn = cfg.kyff * wCmd + cfg.kyaw * (wCmd - yawRate);          // + = turn left: right wheel faster
  float s[2] = {u - turn, u + turn};
  for (int k = 0; k < 2; k++) {
    if (fabsf(s[k]) > 0.005f) s[k] += (s[k] > 0 ? cfg.deadzone : -cfg.deadzone);
    out[k] = constrain(s[k], -1.0f, 1.0f);
  }
}

// ---------------------------------------------------------------- onboard policy (policy.h)
// One observation step, 13 floats, exactly as host/link.py obs_step() builds it.
void obsStep(float o[13], uint8_t stand, float vCmd, float wCmd) {
  o[0] = pitch;  o[1] = pitchRate / 5.0f;  o[2] = yawRate / 5.0f;
  o[3] = wheelSigned(0) / 20.0f;  o[4] = wheelSigned(1) / 20.0f;
  o[5] = prevAct[2];  o[6] = prevAct2[0];  o[7] = prevAct2[1];  o[8] = prevAct2[2];
  o[9] = ((vbat > 3.0f ? vbat : 7.6f) - 7.6f) / 0.8f;
  o[10] = stand ? 1.0f : 0.0f;  o[11] = vCmd / 0.3f;  o[12] = wCmd / 2.0f;
}
void policyReset(uint8_t stand, float vCmd, float wCmd) {
  prevAct[0] = prevAct[1] = 0;  prevAct[2] = (tailCmd - 85.0f) / 65.0f;
  memcpy(prevAct2, prevAct, sizeof(prevAct2));
  float o[13];  obsStep(o, stand, vCmd, wCmd);
  for (int k = 0; k < 4; k++) memcpy(obsHist + 13 * k, o, sizeof(o));   // all four slots = the first step
  memcpy(polAct, prevAct, sizeof(polAct));
}
void policyStep(uint8_t stand, float vCmd, float wCmd) {
  float o[13];  obsStep(o, stand, vCmd, wCmd);
  memmove(obsHist + 13, obsHist, sizeof(float) * 13 * 3);                // newest first
  memcpy(obsHist, o, sizeof(o));
  static float x[TW_OBS_DIM], h0[TW_HID], h1[TW_HID];
  for (int i = 0; i < TW_OBS_DIM; i++) x[i] = (obsHist[i] - TW_OBS_MEAN[i]) / TW_OBS_STD[i];
  for (int j = 0; j < TW_HID; j++) { float s = TW_B0[j]; for (int i = 0; i < TW_OBS_DIM; i++) s += TW_W0[j][i] * x[i];  h0[j] = tanhf(s); }
  for (int j = 0; j < TW_HID; j++) { float s = TW_B1[j]; for (int i = 0; i < TW_HID; i++) s += TW_W1[j][i] * h0[i];  h1[j] = tanhf(s); }
  for (int j = 0; j < TW_ACT_DIM; j++) { float s = TW_B2[j]; for (int i = 0; i < TW_HID; i++) s += TW_W2[j][i] * h1[i];  polAct[j] = constrain(s, -1.0f, 1.0f); }
  memcpy(prevAct2, prevAct, sizeof(prevAct2));
  memcpy(prevAct, polAct, sizeof(prevAct));
}
#ifdef TW_POLICY_PLACEHOLDER
const bool POLICY_LOADED = false;
#else
const bool POLICY_LOADED = true;
#endif

// ---------------------------------------------------------------- modes
const char *modeName(uint8_t m) {
  switch (m) { case MODE_IDLE: return "idle"; case MODE_PID: return "pid"; case MODE_REMOTE: return "remote";
               case MODE_ONBOARD: return "onboard"; case MODE_BENCH: return "bench"; default: return "?"; }
}
void setMode(uint8_t m, uint8_t stand, float vCmd, float wCmd) {
  if (m > MODE_BENCH || m == mode) return;
  if (m == MODE_ONBOARD && !POLICY_LOADED) { Serial.println("onboard refused: policy.h is the placeholder"); return; }
  if (m != MODE_IDLE && lowBatt) { Serial.println("refused: low battery"); return; }
  mode = m;
  pidReset();  pidArmed = false;  pidUprightSince = 0;  tiltCut = false;
  sweepWhich = -1;  benchUntil = 0;
  if (m == MODE_ONBOARD) policyReset(stand, vCmd, wCmd);
  Serial.printf("mode %s\n", modeName(m));
}

// ---------------------------------------------------------------- settings
void loadConfig() {
  prefs.begin("tailwhip", true);
  Config c;
  if (prefs.getBytes("cfg", &c, sizeof(c)) == sizeof(c) && c.version == CFG_VERSION) cfg = c;
  prefs.end();
}
void saveConfig() { prefs.begin("tailwhip", false);  prefs.putBytes("cfg", &cfg, sizeof(cfg));  prefs.end();  Serial.println("saved"); }
void printGains() {
  Serial.printf("kp %.3f kd %.3f ki %.3f kv %.3f kx %.3f kyff %.3f kyaw %.3f dz %.3f trim %.2f deg\n",
                cfg.kp, cfg.kd, cfg.ki, cfg.kv, cfg.kx, cfg.kyff, cfg.kyaw, cfg.deadzone, cfg.pitchTrimDeg);
  Serial.printf("dir L %d R %d tail %d head %d   trim tail %d us head %d us   wheels %s\n",
                cfg.dirL, cfg.dirR, cfg.dirTail, cfg.dirHead, cfg.trimTailUs, cfg.trimHeadUs, cfg.wheelsMode ? "off" : "auto");
}

// ---------------------------------------------------------------- serial commands (the robot's own USB)
void printHelp() {
  Serial.println(F(
    "Tailwhip serial commands (115200 baud, one per line):\n"
    "  p                 print one status line          s   toggle a 10 Hz status stream\n"
    "  mode idle|pid|onboard|bench                       (remote mode is radio only)\n"
    "  m <VL> <VR> [sec] bench: run the wheels at VL, VR volts (default 2 s, max 8.4 V)\n"
    "  tail <deg>        bench: tail angle (20..150)    head <deg>   head pan (30..150)\n"
    "  sweep tail|head   bench: slow sweep across the range and back\n"
    "  servos on|off     bench: servo pulses on or off (off = turn horns by hand)\n"
    "  stand 0|1   go <v m/s> <w rad/s>                  setpoints for pid/onboard when no radio\n"
    "  kp|kd|ki|kv|kx|kyff|kyaw|dz <value>   trim <deg>  PID gains, dead-zone, balance point\n"
    "  ttrim|htrim <us>  servo trim    flip l|r|tail|head   reverse a motor or servo\n"
    "  wheels auto|off   use the IR wheel sensors or run IMU-only\n"
    "  cal   gyro bias (hold still)    gains   print settings    save   store in flash    defaults\n"
    "  help"));
}
uint16_t stateFlags();
void printStatus() {
  uint32_t nL, nR;  noInterrupts();  nL = edgeCount[0];  nR = edgeCount[1];  interrupts();
  Serial.printf("%-7s pitch %6.2f acc %6.2f rate %7.1f yaw %7.1f dps | wheel L %6.2f R %6.2f rad/s edges %lu %lu ir %d%d | "
                "u %5.2f %5.2f | tail %5.1f head %5.1f | %.2f V | flags 0x%04X\n",
                modeName(mode), pitch * RAD_TO_DEG, pitchAcc * RAD_TO_DEG, pitchRate * RAD_TO_DEG, yawRate * RAD_TO_DEG,
                wheelSigned(0), wheelSigned(1), (unsigned long)nL, (unsigned long)nR, digitalRead(PIN_IR_L), digitalRead(PIN_IR_R),
                uApplied[0], uApplied[1], tailCmd, headCmd, vbat, (unsigned)stateFlags());
}
void handleLine(char *line) {
  char *cmd = strtok(line, " \t");
  if (!cmd) return;
  char *a = strtok(nullptr, " \t"), *b = strtok(nullptr, " \t"), *c = strtok(nullptr, " \t");
  float fa = a ? atof(a) : 0, fb = b ? atof(b) : 0;
  String k(cmd);
  if (k == "help" || k == "?") printHelp();
  else if (k == "p") printStatus();
  else if (k == "s") stream = !stream;
  else if (k == "mode" && a) {
    String m(a);
    uint8_t want = m == "idle" ? MODE_IDLE : m == "pid" ? MODE_PID : m == "onboard" ? MODE_ONBOARD : m == "bench" ? MODE_BENCH : 255;
    if (want == 255) Serial.println("modes: idle pid onboard bench");
    else setMode(want, serialStand, serialV, serialW);
  }
  else if (k == "m" && a && b) {
    setMode(MODE_BENCH, 1, 0, 0);
    float sec = c ? constrain(atof(c), 0.1f, 30.0f) : 2.0f;
    benchV[0] = constrain(fa, -V_BENCH_FULL, V_BENCH_FULL);  benchV[1] = constrain(fb, -V_BENCH_FULL, V_BENCH_FULL);
    benchUntil = millis() + (uint32_t)(sec * 1000);
    Serial.printf("wheels %.2f V / %.2f V for %.1f s\n", benchV[0], benchV[1], sec);
  }
  else if (k == "tail" && a) { setMode(MODE_BENCH, 1, 0, 0);  benchTail = fa;  sweepWhich = -1;  benchFromRadio = false; }
  else if (k == "head" && a) { setMode(MODE_BENCH, 1, 0, 0);  benchHead = fa;  sweepWhich = -1;  benchFromRadio = false; }
  else if (k == "sweep" && a) {
    setMode(MODE_BENCH, 1, 0, 0);
    sweepWhich = String(a) == "head" ? 1 : 0;  sweepDeg = sweepWhich ? HEAD_MIN : TAIL_MIN;  sweepDir = 1;  sweepLeg = 0;
  }
  else if (k == "servos" && a) benchServosOff = String(a) == "off";
  else if (k == "stand" && a) serialStand = atoi(a) ? 1 : 0;
  else if (k == "go" && a && b) { serialV = constrain(fa, -0.3f, 0.3f);  serialW = constrain(fb, -2.0f, 2.0f); }
  else if (k == "kp" && a) cfg.kp = fa;   else if (k == "kd" && a) cfg.kd = fa;   else if (k == "ki" && a) cfg.ki = fa;
  else if (k == "kv" && a) cfg.kv = fa;   else if (k == "kx" && a) cfg.kx = fa;
  else if (k == "kyff" && a) cfg.kyff = fa;  else if (k == "kyaw" && a) cfg.kyaw = fa;
  else if (k == "dz" && a) cfg.deadzone = constrain(fa, 0.0f, 0.4f);
  else if (k == "trim" && a) cfg.pitchTrimDeg = constrain(fa, -15.0f, 15.0f);
  else if (k == "ttrim" && a) cfg.trimTailUs = constrain(atoi(a), -300, 300);
  else if (k == "htrim" && a) cfg.trimHeadUs = constrain(atoi(a), -300, 300);
  else if (k == "flip" && a) {
    String w(a);
    if (w == "l") cfg.dirL = -cfg.dirL; else if (w == "r") cfg.dirR = -cfg.dirR;
    else if (w == "tail") cfg.dirTail = -cfg.dirTail; else if (w == "head") cfg.dirHead = -cfg.dirHead;
  }
  else if (k == "wheels" && a) cfg.wheelsMode = String(a) == "off" ? 1 : 0;
  else if (k == "cal") { motorsOff();  Serial.println(calibrateGyro() ? "gyro bias ok" : "gyro cal FAILED: keep the robot still"); }
  else if (k == "gains") printGains();
  else if (k == "save") saveConfig();
  else if (k == "defaults") { cfg = DEFAULTS;  printGains(); }
  else Serial.println("? (type help)");
  if (k.startsWith("k") || k == "dz" || k == "trim" || k == "ttrim" || k == "htrim" || k == "flip" || k == "wheels") printGains();
}
void pollSerial() {
  static char buf[96];  static uint8_t n = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') { buf[n] = 0;  handleLine(buf);  n = 0; }
    else if (n < sizeof(buf) - 1) buf[n++] = ch;
  }
}

// ---------------------------------------------------------------- setup
void goToSleep() {
  motorsOff();  setServos(false, TAIL_REST, 90);
  wr8(0x6B, 0x40);                                        // MPU-6050 sleep
  digitalWrite(PIN_LED, LOW);
  esp_now_deinit();  WiFi.mode(WIFI_OFF);
  Serial.println("low battery: deep sleep. SWITCH THE ROBOT OFF NOW.");  Serial.flush();
  esp_deep_sleep_start();
}

void setup() {
  // Motor inputs low (coast) before anything else.
  for (uint8_t p : {PIN_ML1, PIN_ML2, PIN_MR1, PIN_MR2, PIN_TAIL, PIN_HEAD}) { pinMode(p, OUTPUT);  digitalWrite(p, LOW); }
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_IR_L, INPUT);  pinMode(PIN_IR_R, INPUT);    // 34/35 have no internal pulls: 100k pull-downs on the board
  analogSetPinAttenuation(PIN_BATT, ADC_11db);
  loadConfig();

  pwmAttach(PIN_ML1, CH_ML1, MOTOR_PWM_HZ, MOTOR_BITS);  pwmAttach(PIN_ML2, CH_ML2, MOTOR_PWM_HZ, MOTOR_BITS);
  pwmAttach(PIN_MR1, CH_MR1, MOTOR_PWM_HZ, MOTOR_BITS);  pwmAttach(PIN_MR2, CH_MR2, MOTOR_PWM_HZ, MOTOR_BITS);
  pwmAttach(PIN_TAIL, CH_TAIL, 50, 16);  pwmAttach(PIN_HEAD, CH_HEAD, 50, 16);
  motorsOff();
  setServos(false, TAIL_REST, 90);

  attachInterrupt(digitalPinToInterrupt(PIN_IR_L), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IR_R), isrR, CHANGE);

  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  Wire.setTimeOut(3);
  if (digitalRead(PIN_SDA) == LOW) i2cRecover();
  initImu();
  delay(100);
  bool calOk = imu.ok && calibrateGyro();
  float a[3], g[3];
  if (readImu(a, g)) pitch = pitchAcc = accelPitch(a);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                                   // radio always on: no modem-sleep gaps
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  bool nowOk = esp_now_init() == ESP_OK;
  nowOk = nowOk && esp_now_register_recv_cb(onRecv) == ESP_OK;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastMac, 6);  peer.channel = WIFI_CHANNEL;  peer.ifidx = WIFI_IF_STA;
  nowOk = nowOk && esp_now_add_peer(&peer) == ESP_OK;

  vbat = analogReadMilliVolts(PIN_BATT) * BATT_DIV * BATT_CAL / 1000.0f;
  setServos(true, TAIL_REST, 90);
  Serial.printf("\nTailwhip robot  id %d  ch %d  MAC %s  esp-now %s\n", ROBOT_ID, WIFI_CHANNEL,
                WiFi.macAddress().c_str(), nowOk ? "ok" : "FAILED");
  Serial.printf("MPU-6050 %s  gyro cal %s  pitch %.1f deg  battery %.2f V  policy %s\n", imu.ok ? "ok" : "MISSING",
                calOk ? "ok" : "FAILED", pitch * RAD_TO_DEG, vbat, POLICY_LOADED ? "loaded" : "placeholder");
  printGains();
  Serial.println("type help for commands");
  if (!nowOk) while (true) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(100); }   // fast blink: radio failed
}

// ---------------------------------------------------------------- loop
uint16_t stateFlags() {
  return (failsafe ? SF_FAILSAFE : 0) | (tiltCut ? SF_TILT_CUT : 0) | (wheelsPresent() ? SF_WHEELS : 0)
       | (imu.ok ? 0 : SF_IMU_FAULT) | (lowBatt ? SF_LOW_BATT : 0) | (motorsOn ? SF_MOTORS_ON : 0)
       | (servosOn ? SF_SERVOS_ON : 0) | (mode == MODE_PID && !pidArmed ? SF_PID_FALLEN : 0)
       | (POLICY_LOADED ? SF_POLICY : 0) | (vbat < 3.0f ? SF_NO_BATT : 0) | (voltClip ? SF_VOLT_CLIP : 0)
       | (rxAny && millis() - rxMs < LINK_TIMEOUT_MS ? SF_LINK : 0);
}

void loop() {
  static uint32_t nextUs = micros(), tick = 0, lastRetry = 0, lastStream = 0;
  static uint8_t stand = 1;
  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextUs) < 0) { pollSerial(); return; }
  nextUs += TICK_US;
  if ((int32_t)(nowUs - nextUs) > (int32_t)(4 * TICK_US)) nextUs = nowUs + TICK_US;   // fell far behind: resync
  tick++;
  uint32_t now = millis();
  const float dt = TICK_US * 1e-6f;

  // ---- IMU and pitch filter
  float acc[3], gyr[3];
  if (readImu(acc, gyr)) {
    pitchRate = gyr[1] - gyroBias[1];  yawRate = gyr[2] - gyroBias[2];
    pitchAcc = accelPitch(acc);
    pitch += pitchRate * dt;
    float an = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if (fabsf(an - 1.0f) < ACC_GATE_G) pitch += (pitchAcc - pitch) * dt / FILTER_TAU;
  } else if (now - lastRetry > 500) {
    lastRetry = now;
    if (digitalRead(PIN_SDA) == LOW || digitalRead(PIN_SCL) == LOW) i2cRecover();
    initImu();
  }

  // ---- battery (ADC1 works with the radio on), ~40 ms low-pass
  if ((tick & 3) == 0) {
    float vb = analogReadMilliVolts(PIN_BATT) * BATT_DIV * BATT_CAL / 1000.0f;
    vbat += (vb - vbat) / 4.0f;
    bool onBatt = vbat > 3.0f;
    if (onBatt && vbat * 1000 < LOW_BATT_MV) { if (!lowBattSince) lowBattSince = now ? now : 1; } else lowBattSince = 0;
    if (!lowBatt && lowBattSince && now - lowBattSince > LOW_BATT_HOLD_MS) {
      lowBatt = true;  lowBattAt = now;  mode = MODE_IDLE;  Serial.println("LOW BATTERY: idle");
    }
    if (lowBatt && now - lowBattAt > SLEEP_AFTER_LOW_MS) goToSleep();
  }

  // ---- latest radio command
  CmdPacket c;  uint32_t cmdMs;  bool any;  uint16_t lost;
  portENTER_CRITICAL(&cmdMux);
  c = rxCmd;  cmdMs = rxMs;  any = rxAny;  lost = cmdLost;
  portEXIT_CRITICAL(&cmdMux);
  bool link = any && (int32_t)(now - cmdMs) <= (int32_t)LINK_TIMEOUT_MS;
  if (link) {
    stand = c.flags & CF_STAND ? 1 : 0;
    if (c.mode != MODE_KEEP) setMode(c.mode, stand, c.v_mm_s / 1000.0f, c.yaw_mrad_s / 1000.0f);
  } else stand = serialStand;
  float vCmd = link ? c.v_mm_s / 1000.0f : serialV, wCmd = link ? c.yaw_mrad_s / 1000.0f : serialW;
  float cmdTail = c.tail_cdeg / 100.0f, cmdHead = link ? c.head_cdeg / 100.0f : 90.0f;
  pollSerial();

  // ---- tilt cut (remote, onboard), with hysteresis
  float apitch = fabsf(pitch) * RAD_TO_DEG;
  if (apitch > TILT_CUT_DEG) tiltCut = true; else if (apitch < TILT_RESUME_DEG) tiltCut = false;

  // ---- the active mode decides wheels, tail, head
  bool wantMotors = false, wantServos = true;  float uL = 0, uR = 0, vFull = V_MOTOR_MAX;
  float tail = TAIL_REST, head = cmdHead;
  failsafe = false;
  switch (mode) {
    case MODE_IDLE: break;
    case MODE_REMOTE:
      if (link) { uL = c.u_left / 10000.0f;  uR = c.u_right / 10000.0f;  tail = cmdTail;  wantMotors = !tiltCut; }
      else { failsafe = true;  head = 90; }
      break;
    case MODE_BENCH: {
      vFull = V_BENCH_FULL;  wantServos = !benchServosOff;
      bool serialRun = (int32_t)(benchUntil - now) > 0;
      if (benchUntil && !serialRun) { benchUntil = 0;  Serial.println("wheels stopped"); }
      if (serialRun || sweepWhich >= 0) benchFromRadio = false;
      if (sweepWhich >= 0) {                                  // 30 deg/s up, back down, then stop
        float lo = sweepWhich ? HEAD_MIN : TAIL_MIN, hi = sweepWhich ? HEAD_MAX : TAIL_MAX;
        sweepDeg += sweepDir * 30.0f * dt;
        if (sweepDeg >= hi) { sweepDeg = hi;  sweepDir = -1;  sweepLeg++; }
        if (sweepWhich == 0) benchTail = sweepDeg; else benchHead = sweepDeg;
        if (sweepDeg <= lo && sweepLeg) { sweepWhich = -1;  Serial.println("sweep done"); }
      }
      if (link && !serialRun && sweepWhich < 0) {             // bench.py drives the bench over the radio
        uL = c.u_left / 10000.0f;  uR = c.u_right / 10000.0f;  tail = cmdTail;  head = cmdHead;
        wantMotors = true;  wantServos = !(c.flags & CF_SERVOS_OFF);  benchFromRadio = true;
      } else if (serialRun) {
        uL = benchV[0] / V_BENCH_FULL;  uR = benchV[1] / V_BENCH_FULL;  wantMotors = true;  tail = benchTail;  head = benchHead;
      } else if (benchFromRadio) { failsafe = true;  tail = TAIL_REST;  head = 90; }   // radio went quiet
      else { tail = benchTail;  head = benchHead; }
      break; }
    case MODE_PID: {
      tail = link ? cmdTail : TAIL_REST;
      float th = fabsf(pitch * RAD_TO_DEG - cfg.pitchTrimDeg);
      if (pidArmed && th > PID_FALL_DEG) { pidArmed = false;  Serial.println("pid: fallen, disarmed"); }
      if (!pidArmed) {                                        // re-arm once held upright for 0.5 s
        if (th < PID_ARM_DEG) { if (!pidUprightSince) pidUprightSince = now ? now : 1;
                                else if (now - pidUprightSince > PID_ARM_MS) { pidArmed = true;  pidReset();  Serial.println("pid: armed"); } }
        else pidUprightSince = 0;
      }
      if (pidArmed) { float o[2];  pidStep(vCmd, wCmd, o);  uL = o[0];  uR = o[1];  wantMotors = true; }
      break; }
    case MODE_ONBOARD:
      if (tick % 8 == 0) policyStep(stand, vCmd, wCmd);      // 50 Hz
      uL = polAct[0];  uR = polAct[1];  tail = 85.0f + 65.0f * polAct[2];  wantMotors = !tiltCut;
      break;
  }
  if (!imu.ok && mode != MODE_BENCH) wantMotors = false;
  if (lowBatt) wantMotors = false;
  if (wantMotors) setWheels(constrain(uL, -1.0f, 1.0f), constrain(uR, -1.0f, 1.0f), vFull); else motorsOff();
  setServos(wantServos, tail, head);

  // ---- 100 Hz: wheel speeds, state packet, LED
  if ((tick & 3) == 0) {
    updateWheels(nowUs, now);
    StatePacket s = {};
    s.magic = STATE_MAGIC;  s.robot_id = ROBOT_ID;  s.mode = mode;
    s.cmd_age_ms = any ? (uint8_t)min<uint32_t>(now - cmdMs, 255) : 255;
    s.flags = stateFlags();  s.seq = seq++;  s.last_cmd_seq = c.seq;  s.t_ms = now;
    s.pitch_cdeg = (int16_t)constrain(lroundf(pitch * RAD_TO_DEG * 100), -32000, 32000);
    s.pitch_rate_ddps = (int16_t)constrain(lroundf(pitchRate * RAD_TO_DEG * 10), -32000, 32000);
    s.yaw_rate_ddps = (int16_t)constrain(lroundf(yawRate * RAD_TO_DEG * 10), -32000, 32000);
    s.wheel_l_crad_s = (int16_t)constrain(lroundf(wheelSigned(0) * 100), -32000, 32000);
    s.wheel_r_crad_s = (int16_t)constrain(lroundf(wheelSigned(1) * 100), -32000, 32000);
    s.tail_cmd_cdeg = (int16_t)lroundf(tailCmd * 100);  s.head_cmd_cdeg = (int16_t)lroundf(headCmd * 100);
    s.vbat_mV = (uint16_t)constrain(lroundf(vbat * 1000), 0, 65535);
    s.u_left = (int16_t)lroundf(uApplied[0] * 10000);  s.u_right = (int16_t)lroundf(uApplied[1] * 10000);
    noInterrupts();  s.edges_l = (uint16_t)edgeCount[0];  s.edges_r = (uint16_t)edgeCount[1];  interrupts();
    s.pitch_acc_cdeg = (int16_t)constrain(lroundf(pitchAcc * RAD_TO_DEG * 100), -32000, 32000);
    s.cmd_lost = lost;
    esp_now_send(broadcastMac, (const uint8_t *)&s, sizeof(s));   // broadcast: no ACK, no retry
    // LED: solid = motors driven, slow blink = low battery, 50 ms flash each second = alive
    digitalWrite(PIN_LED, lowBatt ? (now / 500) & 1 : motorsOn ? 1 : (now % 1000) < 50);
    if (stream && now - lastStream >= 100) { lastStream = now;  printStatus(); }
  }
}
