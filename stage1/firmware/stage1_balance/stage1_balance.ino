// Stage 1 balancer: one TT motor, MPU-6050, MX1508, ESP32, and (milestone B) one IR wheel sensor. PID, no radio.
// Board: "ESP32 Dev Module" (esp32 core 2.x or 3.x). Compiles; NOT yet run on hardware.
//
// The filter, the wheel sensor code and the PID are the ones sim.py simulates (classes Controller and IrWheel), with
// the same constants and the same default gains. If you change one, change the other.
//
// Milestone A: no wheel sensor (or unplugged). Speed comes from a model of the motor. It stands, and wanders.
// Milestone B: plug in the TCRT5000 that reads the 16-window ring on the left wheel. Nothing to set: the sketch
// sees the edges, trusts the sensor after 8 believable ones and switches to its position-holding gains. If the
// sensor stops (unplugged, misaligned, dead) it falls back to milestone A by itself. "wheel off" forces A.
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
const uint8_t PIN_IR = 34;                    // TCRT5000 module D0, module on 3V3. Input only, no internal pull:
                                              // the pull-down on the board holds it low when the sensor is unplugged
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
const float    WHEEL_NOLOAD_6V = 21.0f;       // rad/s (200 rpm) at 6 V, default for the speed model. Measure it at
                                              // bring-up step 3 and set it with "wnl"
const float    V_EST_TAU = 0.08f;
const float    U_X_MAX = 0.10f;               // the position term never asks for more than this much command
// Wheel sensor: 16 windows, both edges counted = 32 edges per wheel turn. Magnitude only.
const float    RAD_PER_EDGE = 2.0f * PI / 32.0f;
const uint32_t IR_LOCKOUT_US = 1000;          // ignore edges closer than 1 ms (comparator chatter); top speed is ~9 ms/edge
const uint32_t IR_STOP_US = 250000;           // no edge for 250 ms = stopped
const uint16_t IR_CONFIRM = 8;                // believable edges before the sensor is trusted
const float    IR_MAX_RATIO = 1.6f;           // faster than 1.6 x no-load speed is not a wheel
const uint8_t  IR_STRIKES = 10;               // ... that many times in a row: noise, sensor off
const uint8_t  IR_FAULT_EDGES = 6;            // the model says the wheel turned this many edges on the body with no
                                              // edge seen: distrust the sensor until IR_CONFIRM fresh edges
const uint16_t IR_CAL_NOISE = 4;              // more edges than this while the gyro calibrates: floating input

struct Gains { float kp, kd, ki, kv, kx, tc; };
struct Config { uint32_t version; Gains g[2]; float deadzone, trimDeg, wNoload; int8_t dir; uint8_t wheelMode; };
const uint32_t CFG_VERSION = 2;
// Defaults = out/gains.json from sim.py tune. g[0] "imu" (milestone A), g[1] "ir" (milestone B, sensor trusted).
// Units: u per rad, per rad/s, per rad s, per m/s, per m; tc in s (0 = speed from the model only).
const Config DEFAULTS = {CFG_VERSION,
                         {{1.62f, 0.37f, 0.0f, 2.4202f, 0.1372f, 0.0f},      // imu
                          {1.791f, 0.304f, 0.0f, 2.086f, 0.539f, 0.0f}},     // ir
                         0.10f, 0.0f, WHEEL_NOLOAD_6V, 1, 0};
Config cfg = DEFAULTS;
Preferences prefs;
const char *SET_NAME[2] = {"imu", "ir"};
uint8_t editSet = 0;                          // which gain set kp..tc change

bool imuOk = false, armed = false, stream = false, lowBatt = false, irWatch = false;
float gyroBias[3] = {0, 0, 0};
float pitch = 0, pitchRate = 0, vbat = 0;
float integ = 0, xEst = 0, vEst = 0, vCorr = 0, vUsed = 0, uPrev = 0, uOut = 0, pitchPrevX = 0;
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

// ---------------------------------------------------------------- IR wheel sensor
// The ISR counts edges with a lockout and keeps the last three edge times. Every tick, updateIr(): speed = two
// edges / the time across the last two (one window plus one bar, so uneven printed windows still read right); with
// no new edge the estimate can only fall (one edge / time since the last edge) and is 0 after 250 ms.
// The sensor cannot tell direction. The sign is the direction the wheel should be turning ON THE BODY: the motor
// model's speed minus the pitch rate. (The published Tailwhip takes the sign of the last command. At a standstill
// most edges come from the body rocking over the wheel, against the command, and sim.py finds that rule counts
// about half of them the wrong way and drifts 2 m in 60 s; see out/sim.md, "Where the direction comes from".)
// Detection: UNKNOWN (use the motor model) -> OK after 8 believable edges. OK -> UNKNOWN again when the model says
// the wheel has turned 6 edges' worth on the body with no edge (unplugged, misaligned, dead): it falls back to the
// IMU-only gains at once and needs 8 fresh edges to be trusted again. FAILED (latched until a clean "cal") when the
// input is not a wheel: edges while the gyro calibrates (floating input) or 10 implausibly fast edges in a row.
enum { IR_UNKNOWN = 0, IR_OK = 1, IR_FAILED = 2 };
const char *IR_NAME[3] = {"unknown", "OK", "FAILED"};
volatile uint32_t irCount = 0, irT0 = 0, irT1 = 0, irT2 = 0;
uint32_t irPrevN = 0, irDn = 0;
uint8_t irStatus = IR_UNKNOWN, irStrikes = 0;
uint32_t irFaults = 0;                       // times it went quiet while the wheel should have turned
uint16_t irGood = 0;
bool irNoisy = false;
float irRel = 0, irSign = 1, irQuiet = 0;

void IRAM_ATTR isrIr() {
  uint32_t t = micros();
  if (t - irT0 > IR_LOCKOUT_US) { irCount = irCount + 1;  irT2 = irT1;  irT1 = irT0;  irT0 = t; }
}
void updateIr(uint32_t nowUs, float wModel) {
  const float dt = TICK_US * 1e-6f;
  uint32_t n, t0, t2;
  noInterrupts();  n = irCount;  t0 = irT0;  t2 = irT2;  interrupts();
  irDn = n - irPrevN;  irPrevN = n;
  if (irDn > 0) {
    irQuiet = 0;
    uint32_t span = t0 - t2;
    float w = span > 0 ? 2.0f * RAD_PER_EDGE / (span * 1e-6f) : 1e9f;
    if (w <= IR_MAX_RATIO * cfg.wNoload) { irRel = w;  irGood += irDn > 1000 ? 1000 : irDn;  if (irStrikes) irStrikes--; }
    else if (++irStrikes >= IR_STRIKES) irStatus = IR_FAILED;
    if (irStatus == IR_UNKNOWN && irGood >= IR_CONFIRM) irStatus = IR_OK;
  } else {
    uint32_t since = nowUs - t0;
    if (since < 1) since = 1;
    irRel = since > IR_STOP_US ? 0.0f : fminf(irRel, RAD_PER_EDGE / (since * 1e-6f));
    if (irStatus == IR_OK) {
      irQuiet += fabsf(wModel) * dt;
      if (irQuiet > IR_FAULT_EDGES * RAD_PER_EDGE) { irStatus = IR_UNKNOWN;  irGood = 0;  irFaults++; }
    }
  }
}
bool irPresent() { return irStatus == IR_OK && cfg.wheelMode == 0; }
float irSigned() { return irSign * irRel; }
float irTakeDx() { return irSign * (float)irDn * RAD_PER_EDGE; }     // rad of wheel since the last tick
void irSetSign(float w) { if (w > 0.005f) irSign = 1; else if (w < -0.005f) irSign = -1; }

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
    uint32_t e0 = irCount;
    for (int i = 0; i < 400; i++) {
      if (readImu(a, g)) { for (int k = 0; k < 3; k++) { s[k] += g[k];  s2[k] += g[k] * g[k]; }  sp += accelPitch(a);  n++; }
      delayMicroseconds(2500);
    }
    if (n < 300) continue;
    bool still = true;
    for (int k = 0; k < 3; k++) { double m = s[k] / n;  if (s2[k] / n - m * m > 2.5e-4) still = false; }
    if (still) {
      for (int k = 0; k < 3; k++) gyroBias[k] = s[k] / n;
      pitch = sp / n;
      // Held still, motor off: the wheel should not move. Edges now mean the input is floating or noisy.
      uint32_t e = irCount - e0;
      irNoisy = e > IR_CAL_NOISE;
      if (irNoisy) { irStatus = IR_FAILED;  Serial.printf("wheel sensor: %lu edges while still: floating input? IMU only\n", (unsigned long)e); }
      else if (irStatus == IR_FAILED) { irStatus = IR_UNKNOWN;  irGood = 0;  irStrikes = 0; }
      return true;
    }
    Serial.println("gyro cal: it moved, trying again");
  }
  return false;
}

// ---------------------------------------------------------------- PID
// u = kp th + kd th_dot + ki int(th) + kv v + kx x, all gains positive, th = pitch - trim.
// kv and kx have the same sign as kp: to stop a forward drift the wheels first speed up, the body tips back, and
// then they slow down. v is the motor model: the last command times the no-load speed, lagged 80 ms. Without the
// sensor x is its integral, which stops a runaway but does not hold position. With the sensor x is counted: wheel
// edges (relative to the body) plus the change in pitch, so it holds position. The sensor's speed is too coarse
// and too late to balance on, so v stays the model, corrected towards the measured speed over tc s if tc > 0.
void pidReset() { integ = 0;  xEst = 0;  vEst = 0;  vCorr = 0;  uPrev = 0;  pitchPrevX = pitch; }
void speedModel(float u) {      // every tick, armed or not, so detection and the sign always have a model speed
  const float dt = TICK_US * 1e-6f;
  vEst += (u * cfg.wNoload * WHEEL_R - vEst) * dt / V_EST_TAU;
}
float pidStep() {
  const float dt = TICK_US * 1e-6f;
  float th = pitch - cfg.trimDeg * DEG_TO_RAD;
  speedModel(uPrev);
  float dx = irTakeDx();
  const Gains &g = cfg.g[irPresent() ? 1 : 0];
  if (irPresent()) {                                     // ground speed = wheel relative to the body + pitch rate
    float vm = WHEEL_R * (irSigned() + pitchRate);
    if (g.tc > 0) vCorr += (vm - vEst - vCorr) * dt / g.tc;
    vUsed = vEst + vCorr;
    xEst = constrain(xEst + WHEEL_R * (dx + pitch - pitchPrevX), -0.5f, 0.5f);
  } else {
    vUsed = vEst;
    xEst = constrain(xEst + vEst * dt, -0.5f, 0.5f);
  }
  pitchPrevX = pitch;
  integ = constrain(integ + th * dt, -0.2f, 0.2f);
  float u = g.kp * th + g.kd * pitchRate + g.ki * integ + g.kv * vUsed + constrain(g.kx * xEst, -U_X_MAX, U_X_MAX);
  u = constrain(u, -1.0f, 1.0f);
  uPrev = u;
  irSetSign(vEst / WHEEL_R - pitchRate);
  if (fabsf(u) > 0.005f) u += (u > 0 ? cfg.deadzone : -cfg.deadzone);    // step over the motor's dead zone
  return constrain(u, -1.0f, 1.0f);
}

// ---------------------------------------------------------------- serial
void help() {
  Serial.println(
    "edit <imu|ir>               which gain set the next five commands change (imu: no sensor, ir: sensor)\n"
    "kp|kd|ki|kv|kx|tc <value>   gains of that set\n"
    "dz <value>                  dead-zone compensation\n"
    "wnl <rad/s>                 motor no-load speed at 6 V, for the speed model (bring-up step 3)\n"
    "trim <deg>                  balance point (+ if it creeps forward)\n"
    "dir <1|-1>                  flip if the wheels drive the wrong way\n"
    "wheel <auto|off>            use the wheel sensor when it works, or never (milestone A)\n"
    "ir | irw <0|1>              wheel sensor: one line now, or 5 lines a second\n"
    "show | save | defaults      print, store in flash, restore\n"
    "stream <0|1>                CSV at 50 Hz: ms,pitch_deg,rate_dps,u,v,x,ir,edges,vbat\n"
    "motor <u> <ms>              bench: run the motor at u (-1..1) for ms, robot off the ground\n"
    "cal                         recalibrate the gyro (hold it still)");
}
void showIr() {
  uint32_t n;  noInterrupts();  n = irCount;  interrupts();
  Serial.printf("wheel sensor %s%s%s | quiet spells %lu | edges %lu  D0 %d | wheel on body %.2f rad/s (%+.2f signed) | ground %.3f m/s  x %.3f m\n",
                IR_NAME[irStatus], irNoisy ? " (floating input)" : "", cfg.wheelMode ? ", forced off" : irPresent() ? ", in use" : "",
                (unsigned long)irFaults, (unsigned long)n, digitalRead(PIN_IR), irRel, irSigned(), vUsed, xEst);
}
void show() {
  for (int k = 0; k < 2; k++)
    Serial.printf("%s%-3s kp %.3f kd %.3f ki %.3f kv %.3f kx %.3f tc %.2f%s\n", k == editSet ? "*" : " ", SET_NAME[k],
                  cfg.g[k].kp, cfg.g[k].kd, cfg.g[k].ki, cfg.g[k].kv, cfg.g[k].kx, cfg.g[k].tc,
                  (irPresent() ? 1 : 0) == k ? "  <- in use" : "");
  Serial.printf("dz %.3f wnl %.1f rad/s trim %.2f dir %d wheel %s | pitch %.1f deg vbat %.2f V %s%s\n",
                cfg.deadzone, cfg.wNoload, cfg.trimDeg, cfg.dir, cfg.wheelMode ? "off" : "auto", pitch * RAD_TO_DEG, vbat,
                armed ? "armed" : "waiting", lowBatt ? " LOW BATTERY" : "");
  showIr();
}
uint32_t benchUntil = 0, benchHalf = 0, benchN = 0, benchT = 0;  float benchU = 0;
void handleLine(String s) {
  s.trim();  if (!s.length()) return;
  int sp = s.indexOf(' ');
  String c = sp < 0 ? s : s.substring(0, sp), a = sp < 0 ? "" : s.substring(sp + 1);
  float v = a.toFloat();
  Gains &g = cfg.g[editSet];
  if (c == "kp") g.kp = v; else if (c == "kd") g.kd = v; else if (c == "ki") g.ki = v;
  else if (c == "kv") g.kv = v; else if (c == "kx") g.kx = v; else if (c == "tc") g.tc = v;
  else if (c == "edit") editSet = a == "ir" ? 1 : 0;
  else if (c == "dz") cfg.deadzone = v; else if (c == "wnl") cfg.wNoload = constrain(v, 5.0f, 60.0f);
  else if (c == "trim") cfg.trimDeg = v; else if (c == "dir") cfg.dir = v < 0 ? -1 : 1;
  else if (c == "wheel") cfg.wheelMode = a == "off" ? 1 : 0;
  else if (c == "ir") { showIr();  return; }
  else if (c == "irw") { irWatch = v != 0;  return; }
  else if (c == "save") { prefs.putBytes("cfg", &cfg, sizeof(cfg));  Serial.println("saved"); }
  else if (c == "defaults") cfg = DEFAULTS;
  else if (c == "stream") stream = v != 0;
  else if (c == "cal") { armed = false;  coast();  Serial.println(calibrateGyro() ? "gyro ok" : "gyro cal failed"); }
  else if (c == "motor") {
    int sp2 = a.indexOf(' ');
    benchU = constrain(a.toFloat(), -1.0f, 1.0f);
    uint32_t ms = sp2 < 0 ? 1000 : constrain(a.substring(sp2 + 1).toInt(), 0, 5000);
    benchUntil = millis() + ms;  benchHalf = millis() + ms / 2;  benchT = 0;
    armed = false;
    return;
  }
  else if (c == "help") { help();  return; }
  else if (c != "show") { Serial.println("? type help");  return; }
  show();
}
// End of a bench run: the wheel speed over its second half, from the sensor. At u = 1 this is the no-load speed.
void benchReport() {
  uint32_t n, t;  noInterrupts();  n = irCount;  t = irT0;  interrupts();
  if (!benchT || n - benchN < 4 || t == benchT) { Serial.println("bench: no wheel edges. Count turns by eye to get the no-load speed"); return; }
  float w = (n - benchN) * RAD_PER_EDGE / ((t - benchT) * 1e-6f);
  Serial.printf("bench: wheel %.1f rad/s (%.0f rpm) at u %.2f, from %lu edges", w, w * 60.0f / (2.0f * PI), benchU, (unsigned long)(n - benchN));
  if (fabsf(benchU) > 0.99f) Serial.printf(". No-load speed: type  wnl %.1f  then save\n", w); else Serial.println();
}

// ---------------------------------------------------------------- setup and loop
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_IR, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_IR), isrIr, CHANGE);
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
  uint32_t now = millis(), nowUs = micros();

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
  updateIr(nowUs, vEst / WHEEL_R - pitchRate);             // last tick's model speed, on the body, as sim.py

  float th = fabsf(pitch * RAD_TO_DEG - cfg.trimDeg);
  if (benchUntil) {                                        // bench: open loop, wheels off the ground
    if (now < benchUntil) {
      drive(benchU);  uOut = benchU;  speedModel(benchU);  irSetSign(vEst / WHEEL_R - pitchRate);
      if (!benchT && now >= benchHalf) { noInterrupts();  benchN = irCount;  benchT = irT0;  interrupts();  if (!benchT) benchT = 1; }
    } else { benchUntil = 0;  coast();  uOut = 0;  benchReport(); }
  } else if (lowBatt) {
    armed = false;  coast();  uOut = 0;  speedModel(0);  digitalWrite(PIN_LED, (now / 500) & 1);
  } else if (armed) {
    if (th > FALL_DEG) { armed = false;  coast();  uOut = 0;  uprightSince = 0;  Serial.println("fallen: let go"); }
    else { uOut = pidStep();  drive(uOut); }
  } else {
    coast();  uOut = 0;  speedModel(0);  irSetSign(vEst / WHEEL_R - pitchRate);
    if (th < ARM_DEG) {
      if (!uprightSince) uprightSince = now ? now : 1;
      else if (now - uprightSince > ARM_MS) { armed = true;  pidReset();  Serial.println(irPresent() ? "armed, wheel sensor in use" : "armed"); }
    } else uprightSince = 0;
  }
  if (!lowBatt) digitalWrite(PIN_LED, armed ? HIGH : (now / 250) & 1);
  if (stream && tick % 8 == 0) {
    uint32_t n;  noInterrupts();  n = irCount;  interrupts();
    Serial.printf("%lu,%.2f,%.1f,%.3f,%.3f,%.3f,%d,%lu,%.2f\n", (unsigned long)now, pitch * RAD_TO_DEG, pitchRate * RAD_TO_DEG, uOut,
                  vUsed, xEst, irStatus, (unsigned long)n, vbat);
  }
  if (irWatch && tick % 80 == 0) showIr();
}
