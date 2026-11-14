// Tailwhip assembly helper: hold both servos at 90 deg (1500 us) so the horns can be fitted.
// Tail: horn fitted with the tail pointing straight back along the body (-x). Head: horn pointing forward.
// Flash to the ROBOT's ESP32, Serial Monitor 115200, battery switched on (servo power comes from the servo buck).
//   t / h  pick the tail (GPIO18) or head (GPIO19) servo for the commands below
//   c      both servos to 90 deg            s   sweep the picked servo 20 -> 150 -> 90 deg, slowly
//   + / -  nudge the picked servo 10 us     l   limp (no pulses on either)
// Board: "ESP32 Dev Module" (FQBN esp32:esp32:esp32). Builds on core 2.0.x and 3.x, no libraries.
const uint8_t PIN[2] = {18, 19};          // tail, head: same as tailwhip_robot.ino
const char *NAME[2] = {"tail", "head"};
int us[2] = {1500, 1500}, pick = 0;

void attachServo(uint8_t i) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(PIN[i], 50, 16, 4 + i);
#else
  ledcSetup(4 + i, 50, 16);  ledcAttachPin(PIN[i], 4 + i);
#endif
}
void writeDuty(uint8_t i, uint32_t d) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN[i], d);
#else
  ledcWrite(4 + i, d);
#endif
}
void drive(uint8_t i, int pulse) {
  us[i] = constrain(pulse, 500, 2500);
  writeDuty(i, (uint32_t)(us[i] * 65536L / 20000L));
  Serial.printf("%s (GPIO %d): %d us = %.1f deg\n", NAME[i], PIN[i], us[i], 90.0 + (us[i] - 1500) / (2000.0 / 180.0));
}
int degUs(float deg) { return (int)(1500 + (deg - 90.0f) * 2000.0f / 180.0f); }

void setup() {
  for (uint8_t p : PIN) { pinMode(p, OUTPUT);  digitalWrite(p, LOW); }
  // Motor inputs low (coast), in case the MX1508 is already wired.
  for (uint8_t p : {25, 26, 27, 14}) { pinMode(p, OUTPUT);  digitalWrite(p, LOW); }
  Serial.begin(115200);
  delay(300);
  attachServo(0);  attachServo(1);
  Serial.println("\nservo_center: t/h pick, c centre both (90 deg), s sweep, +/- 10 us, l limp");
  drive(0, 1500);  drive(1, 1500);
}

void loop() {
  if (!Serial.available()) { delay(5); return; }
  char k = (char)Serial.read();
  if (k == 't' || k == 'h') { pick = (k == 'h');  Serial.printf("picked %s\n", NAME[pick]); }
  else if (k == 'c') { drive(0, 1500);  drive(1, 1500); }
  else if (k == '+') drive(pick, us[pick] + 10);
  else if (k == '-') drive(pick, us[pick] - 10);
  else if (k == 'l') { writeDuty(0, 0);  writeDuty(1, 0);  Serial.println("limp"); }
  else if (k == 's') {
    for (int p = us[pick]; p >= degUs(20); p -= 5) { writeDuty(pick, p * 65536L / 20000L); delay(15); }
    for (int p = degUs(20); p <= degUs(150); p += 5) { writeDuty(pick, p * 65536L / 20000L); delay(15); }
    for (int p = degUs(150); p >= 1500; p -= 5) { writeDuty(pick, p * 65536L / 20000L); delay(15); }
    drive(pick, 1500);
  }
}
