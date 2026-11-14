// Tailwhip bring-up: scan the I2C bus on GPIO21 (SDA) / GPIO22 (SCL) every 2 s and say what answered.
// Expected: 0x68 MPU-6050 (GY-521, AD0 low), WHO_AM_I 0x68. Clone MPU-6500s answer 0x70 and work the same.
// Serial Monitor 115200. Board: "ESP32 Dev Module" (FQBN esp32:esp32:esp32).
#include <Wire.h>

const uint8_t PIN_SDA = 21, PIN_SCL = 22;

const char *name(uint8_t a) {
  switch (a) {
    case 0x68: return "MPU-6050 (AD0 low)";
    case 0x69: return "MPU-6050 with AD0 high: tie AD0 to GND, the firmware expects 0x68";
    default:   return "unexpected";
  }
}

int scan(uint32_t hz) {
  Wire.setClock(hz);
  int found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      found++;
      Serial.printf("  0x%02X  %s", a, name(a));
      if (a == 0x68 || a == 0x69) {
        Wire.beginTransmission(a); Wire.write(0x75); Wire.endTransmission(false);
        if (Wire.requestFrom(a, (size_t)1) == 1) Serial.printf("   WHO_AM_I 0x%02X", Wire.read());
      }
      Serial.println();
    }
  }
  return found;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(PIN_SDA, INPUT);  pinMode(PIN_SCL, INPUT);
  Serial.printf("\ni2c_scan: idle levels SDA=%d SCL=%d (both must be 1: GY-521 pull-ups present, nothing holding the bus)\n",
                digitalRead(PIN_SDA), digitalRead(PIN_SCL));
  Wire.begin(PIN_SDA, PIN_SCL, 100000);
  Wire.setTimeOut(5);
}

void loop() {
  int a = scan(100000);
  Serial.printf("100 kHz: %d device(s)\n", a);
  int b = scan(400000);
  Serial.printf("400 kHz: %d device(s)%s\n\n", b, a == b ? "" : "   <-- differs from 100 kHz: check wiring");
  delay(2000);
}
