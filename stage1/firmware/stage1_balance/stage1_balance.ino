// Stage 1 balancer: one TT motor, MPU-6050, MX1508, ESP32. PID only, no radio, no wheel sensors.
// Board: "ESP32 Dev Module" (esp32 core 2.x or 3.x). Compiles; NOT yet run on hardware.
//
// The filter and the PID are the ones sim.py simulates (class Controller), with the same constants and the same
// default gains. If you change one, change the other.
//
// Use: hold the robot upright and still, switch on, wait for the LED to go solid (gyro calibrated), keep holding
// it within 5 degrees of upright for half a second and it takes over. Past 22 degrees it lets go and waits.
// Serial, 115200: type "help".
#include <Wire.h>
#include <Preferences.h>

// ---------------------------------------------------------------- pins
const uint8_t PIN_M1 = 25, PIN_M2 = 26;       // MX1508 IN1, IN2 (motor A)
const uint8_t PIN_SDA = 21, PIN_SCL = 22;     // MPU-6050 (GY-521), powered from 3V3
const uint8_t PIN_BATT = 36;                  // VP: battery through 100k over 33k
const uint8_t PIN_LED = 2;
const uint8_t CH_M1 = 0, CH_M2 = 1;
const uint8_t MPU = 0x68;

// ---------------------------------------------------------------- fixed numbers
const uint32_t TICK_US = 2500;                // 400 Hz
const uint32_t MOTOR_PWM_HZ = 20000;
const uint8_t  MOTOR_BITS = 10;
const uint32_t MOTOR_MAX = (1u << MOTOR_BITS) - 1;
const float    V_MOTOR_MAX = 6.0f;            // u = 1 means 6.0 V average at the motor, whatever the battery is at
const float    BATT_DIV = (100.0f + 33.0f) / 33.0f;
const uint16_t LOW_BATT_MV = 6400;            // 3.2 V per cell
const float    ARM_DEG = 5.0f, FALL_DEG = 22.0f;
const uint32_t ARM_MS = 500;
const float    ACC_LSB = 8192.0f, GYR_LSB = 65.5f;   // +-4 g, +-500 dps
const float    FILTER_TAU = 0.5f;             // complementary filter: gyro below 0.5 s, accelerometer above
const float    ACC_GATE_G = 0.25f;            // ignore the accelerometer when it reads more than 0.25 g off 1 g
const float    WHEEL_R = 0.0325f;
const float    WHEEL_NOLOAD_6V = 26.2f;       // rad/s, for the speed model (measure it: README, bring-up step 3)
const float    V_EST_TAU = 0.08f;

struct Config { uint32_t version; float kp, kd, ki, kv, kx, deadzone, trimDeg; int8_t dir; };
const uint32_t CFG_VERSION = 1;
// Defaults = out/gains.json from sim.py tune. Units: u per rad, per rad/s, per rad s, per m/s, per m.
const Config DEFAULTS = {CFG_VERSION, 1.62f, 0.37f, 0.0f, 1.94f, 0.11f, 0.10f, 0.0f, 1};
Config cfg = DEFAULTS;
Preferences prefs;

bool imuOk = false, armed = false, stream = false, lowBatt = false;
float gyroBias[3] = {0, 0, 0};
float pitch = 0, pitchRate = 0, vbat = 0;
float integ = 0, xEst = 0, vEst = 0, uPrev = 0, uOut = 0;
uint32_t uprightSince = 0, lowSince = 0;

// ---------------------------------------------------------------- PWM, core 2.x and 3.x
void pwmAttach(uint8_t pin, uint8_t ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(pin, MOTOR_PWM_HZ, MOTOR_BITS, ch);
#else
  ledcSetup(ch, MOTOR_PWM_HZ, MOTOR_BITS);  ledcAttachPin(pin, ch);
#endif
}
void pwmWrite(uint8_t pin, uint8_t ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;  ledcWrite(pin, duty);
#else
  (void)pin;  ledcWrite(ch, duty);
#endif
}
// Slow decay: one input held high, the other PWMed low, so the off-time brakes and the average motor voltage is
// duty x battery. u = 0 shorts the motor, the same as 0 V in the simulator.
void drive(float u) {
  float vb = vbat > 5.0f ? vbat : 7.4f;
  float d = constrain(cfg.dir * u * V_MOTOR_MAX / vb, -1.0f, 1.0f);
  uint32_t on = (uint32_t)lroundf(fabsf(d) * MOTOR_MAX);
  if (d >= 0) { pwmWrite(PIN_M1, CH_M1, MOTOR_MAX);  pwmWrite(PIN_M2, CH_M2, MOTOR_MAX - on); }
  else        { pwmWrite(PIN_M2, CH_M2, MOTOR_MAX);  pwmWrite(PIN_M1, CH_M1, MOTOR_MAX - on); }
}
void coast() { pwmWrite(PIN_M1, CH_M1, 0);  pwmWrite(PIN_M2, CH_M2, 0); }

// ---------------------------------------------------------------- IMU
bool wr8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU);  Wire.write(reg);  Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool rd(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(MPU);  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, (size_t)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}
bool initImu() {
  return wr8(0x6B, 0x01)      // wake, PLL on gyro X
      && wr8(0x19, 0x00)      // 1 kHz
      && wr8(0x1A, 0x02)      // DLPF 2: about 94 Hz, 3 ms delay
      && wr8(0x1B, 0x08)      // +-500 dps
      && wr8(0x1C, 0x08);     // +-4 g
}
inline int16_t be16(const uint8_t *b) { return (int16_t)(((uint16_t)b[0] << 8) | b[1]); }
// Body frame: x forward, y left, z up. The GY-521 sits on the front of the core with the chip facing forward and
// its x axis up, so body x = chip z, body y = -chip y, body z = chip x. Mounted differently? Change these lines.
bool readImu(float acc[3], float gyr[3]) {
  uint8_t b[14];
  if (!rd(0x3B, b, 14)) return false;
  float ax = be16(b) / ACC_LSB, ay = be16(b + 2) / ACC_LSB, az = be16(b + 4) / ACC_LSB;
  float gx = be16(b + 8) / GYR_LSB, gy = be16(b + 10) / GYR_LSB, gz = be16(b + 12) / GYR_LSB;
  acc[0] = az;  acc[1] = -ay;  acc[2] = ax;
  gyr[0] = gz * DEG_TO_RAD;  gyr[1] = -gy * DEG_TO_RAD;  gyr[2] = gx * DEG_TO_RAD;
  return true;
}
// Pitch, + = leaning forward: at rest the accelerometer reads (-sin p, 0, cos p) g in the body frame.
float accelPitch(const float a[3]) { return atan2f(-a[0], a[2]); }

bool calibrateGyro() {      // hold it still: 1 s of samples, thrown away if it moved
  for (int attempt = 0; attempt < 5; attempt++) {
    double s[3] = {0, 0, 0}, s2[3] = {0, 0, 0}, sp = 0;  int n = 0;
    float a[3], g[3];
    for (int i = 0; i < 400; i++) {
      if (readImu(a, g)) { for (int k = 0; k < 3; k++) { s[k] += g[k];  s2[k] += g[k] * g[k]; }  sp += accelPitch(a);  n++; }
      delayMicroseconds(2500);
    }
    if (n < 300) continue;
    bool still = true;
    for (int k = 0; k < 3; k++) { double m = s[k] / n;  if (s2[k] / n - m * m > 2.5e-4) still = false; }
    if (still) { for (int k = 0; k < 3; k++) gyroBias[k] = s[k] / n;  pitch = sp / n;  return true; }
    Serial.println("gyro cal: it moved, trying again");
  }
  return false;
}

// ---------------------------------------------------------------- PID
// u = kp th + kd th_dot + ki int(th) + kv v + kx int(v), all gains positive, th = pitch - trim.
// kv and kx have the same sign as kp: to stop a forward drift the wheels first speed up, the body tips back, and
// then they slow down. There are no wheel sensors, so v comes from a model of the motor: the last command times
// its no-load speed, lagged 80 ms. It is good enough to stop a runaway; it will not hold position.
void pidReset() { integ = 0;  xEst = 0;  vEst = 0;  uPrev = 0; }
float pidStep() {
  const float dt = TICK_US * 1e-6f;
  float th = pitch - cfg.trimDeg * DEG_TO_RAD;
  vEst += (uPrev * WHEEL_NOLOAD_6V * WHEEL_R - vEst) * dt / V_EST_TAU;
  integ = constrain(integ + th * dt, -0.2f, 0.2f);
  xEst = constrain(xEst + vEst * dt, -0.5f, 0.5f);
  float u = cfg.kp * th + cfg.kd * pitchRate + cfg.ki * integ + cfg.kv * vEst + cfg.kx * xEst;
  u = constrain(u, -1.0f, 1.0f);
  uPrev = u;
  if (fabsf(u) > 0.005f) u += (u > 0 ? cfg.deadzone : -cfg.deadzone);    // step over the motor's dead zone
  return constrain(u, -1.0f, 1.0f);
}

// ---------------------------------------------------------------- serial
void help() {
  Serial.println(
    "kp|kd|ki|kv|kx|dz <value>   gains and dead-zone compensation\n"
    "trim <deg>                  balance point (+ if it creeps forward)\n"
    "dir <1|-1>                  flip if the wheels drive the wrong way\n"
    "show | save | defaults      print, store in flash, restore\n"
    "stream <0|1>                CSV at 50 Hz: ms,pitch_deg,rate_dps,u,v_est,vbat\n"
    "motor <u> <ms>              bench: run the motor at u (-1..1) for ms, robot off the ground\n"
    "cal                         recalibrate the gyro (hold it still)");
}
void show() {
  Serial.printf("kp %.3f kd %.3f ki %.3f kv %.3f kx %.3f dz %.3f trim %.2f dir %d | pitch %.1f deg vbat %.2f V %s%s\n",
                cfg.kp, cfg.kd, cfg.ki, cfg.kv, cfg.kx, cfg.deadzone, cfg.trimDeg, cfg.dir, pitch * RAD_TO_DEG, vbat,
                armed ? "armed" : "waiting", lowBatt ? " LOW BATTERY" : "");
}
uint32_t benchUntil = 0;  float benchU = 0;
void handleLine(String s) {
  s.trim();  if (!s.length()) return;
  int sp = s.indexOf(' ');
  String c = sp < 0 ? s : s.substring(0, sp), a = sp < 0 ? "" : s.substring(sp + 1);
  float v = a.toFloat();
  if (c == "kp") cfg.kp = v; else if (c == "kd") cfg.kd = v; else if (c == "ki") cfg.ki = v;
  else if (c == "kv") cfg.kv = v; else if (c == "kx") cfg.kx = v; else if (c == "dz") cfg.deadzone = v;
  else if (c == "trim") cfg.trimDeg = v; else if (c == "dir") cfg.dir = v < 0 ? -1 : 1;
  else if (c == "save") { prefs.putBytes("cfg", &cfg, sizeof(cfg));  Serial.println("saved"); }
  else if (c == "defaults") cfg = DEFAULTS;
  else if (c == "stream") stream = v != 0;
  else if (c == "cal") { armed = false;  coast();  Serial.println(calibrateGyro() ? "gyro ok" : "gyro cal failed"); }
  else if (c == "motor") {
    int sp2 = a.indexOf(' ');
    benchU = constrain(a.toFloat(), -1.0f, 1.0f);
    benchUntil = millis() + (sp2 < 0 ? 1000 : constrain(a.substring(sp2 + 1).toInt(), 0, 5000));
    armed = false;
  }
  else if (c == "help") { help();  return; }
  else if (c != "show") { Serial.println("? type help");  return; }
  show();
}

// ---------------------------------------------------------------- setup and loop
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  pwmAttach(PIN_M1, CH_M1);  pwmAttach(PIN_M2, CH_M2);  coast();
  analogSetPinAttenuation(PIN_BATT, ADC_11db);
  prefs.begin("stage1", false);
  Config c;
  if (prefs.getBytes("cfg", &c, sizeof(c)) == sizeof(c) && c.version == CFG_VERSION) cfg = c;
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  Wire.setTimeOut(3);
  delay(100);
  imuOk = initImu();
  if (!imuOk) Serial.println("MPU-6050 not found: check SDA 21, SCL 22, 3V3, GND");
  vbat = analogReadMilliVolts(PIN_BATT) * BATT_DIV / 1000.0f;
  if (imuOk) {
    Serial.println("hold it upright and still...");
    if (!calibrateGyro()) Serial.println("gyro cal failed: type cal");
  }
  digitalWrite(PIN_LED, imuOk);
  help();  show();
}

void loop() {
  static uint32_t next = micros();
  static String line;
  while (Serial.available()) { char ch = Serial.read();  if (ch == '\n' || ch == '\r') { handleLine(line);  line = ""; } else line += ch; }
  if ((int32_t)(micros() - next) < 0) return;
  next += TICK_US;
  const float dt = TICK_US * 1e-6f;
  static uint16_t tick = 0;  tick++;
  uint32_t now = millis();

  if (tick % 40 == 0) {                                   // battery, 10 Hz, smoothed
    float vb = analogReadMilliVolts(PIN_BATT) * BATT_DIV / 1000.0f;
    vbat += (vb - vbat) * 0.2f;
    if (vbat > 3.0f && vbat * 1000 < LOW_BATT_MV) { if (!lowSince) lowSince = now;  if (now - lowSince > 2000) lowBatt = true; }
    else lowSince = 0;
  }

  float acc[3], gyr[3];
  if (!imuOk || !readImu(acc, gyr)) { armed = false;  coast();  digitalWrite(PIN_LED, (now / 100) & 1);  return; }
  pitchRate = gyr[1] - gyroBias[1];
  pitch += pitchRate * dt;
  float an = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
  if (fabsf(an - 1.0f) < ACC_GATE_G) pitch += (accelPitch(acc) - pitch) * dt / FILTER_TAU;

  float th = fabsf(pitch * RAD_TO_DEG - cfg.trimDeg);
  if (benchUntil) {                                        // bench: open loop, wheels off the ground
    if (now < benchUntil) { drive(benchU);  uOut = benchU; } else { benchUntil = 0;  coast();  uOut = 0; }
  } else if (lowBatt) {
    armed = false;  coast();  uOut = 0;  digitalWrite(PIN_LED, (now / 500) & 1);
  } else if (armed) {
    if (th > FALL_DEG) { armed = false;  coast();  uOut = 0;  uprightSince = 0;  Serial.println("fallen: let go"); }
    else { uOut = pidStep();  drive(uOut); }
  } else {
    coast();  uOut = 0;
    if (th < ARM_DEG) {
      if (!uprightSince) uprightSince = now ? now : 1;
      else if (now - uprightSince > ARM_MS) { armed = true;  pidReset();  Serial.println("armed"); }
    } else uprightSince = 0;
  }
  if (!lowBatt) digitalWrite(PIN_LED, armed ? HIGH : (now / 250) & 1);
  if (stream && tick % 8 == 0)
    Serial.printf("%lu,%.2f,%.1f,%.3f,%.3f,%.2f\n", (unsigned long)now, pitch * RAD_TO_DEG, pitchRate * RAD_TO_DEG, uOut, vEst, vbat);
}
