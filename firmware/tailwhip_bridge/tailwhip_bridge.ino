// Tailwhip, USB radio bridge. A second ESP32 DevKit plugged into the laptop.
// USB serial (921600 baud) <-> ESP-NOW. On the serial line every packet is framed as
//   0xAA 0x55 <len> <payload> <crc8(payload)>          (tailwhip_common.h, tw_crc8)
// Laptop -> robot: CmdPacket (18 bytes). Robot -> laptop: StatePacket (42 bytes).
// Board: "ESP32 Dev Module" (FQBN esp32:esp32:esp32). Builds on Arduino-ESP32 core 2.0.x and 3.x.
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include "tailwhip_common.h"

const uint32_t BAUD = 921600;      // CP2102 / CH340C both manage this; if frames drop use 460800 here AND in link.py
const uint8_t PIN_LED = 2;          // toggles every 50 state packets: steady flicker = robot heard
uint8_t broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
QueueHandle_t stateQ;               // receive callback (Wi-Fi task) -> loop(): no Serial calls inside the callback

void sendFrame(const uint8_t *p, uint8_t len) {
  uint8_t head[3] = {0xAA, 0x55, len}, crc = tw_crc8(p, len);
  Serial.write(head, 3);  Serial.write(p, len);  Serial.write(crc);
}

void handleState(const uint8_t *data, int len) {
  if (len != (int)sizeof(StatePacket) || data[0] != STATE_MAGIC || data[1] != ROBOT_ID) return;
  StatePacket s;  memcpy(&s, data, sizeof(s));
  xQueueSend(stateQ, &s, 0);        // never wait inside the Wi-Fi task; a full queue drops the packet
}
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) { (void)info; handleState(data, len); }
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) { (void)mac; handleState(data, len); }
#endif

void setup() {
  pinMode(PIN_LED, OUTPUT);
  stateQ = xQueueCreate(16, sizeof(StatePacket));
  Serial.setRxBufferSize(1024);
  Serial.setTxBufferSize(1024);
  Serial.begin(BAUD);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  bool ok = esp_now_init() == ESP_OK && esp_now_register_recv_cb(onRecv) == ESP_OK;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastMac, 6);  peer.channel = WIFI_CHANNEL;  peer.ifidx = WIFI_IF_STA;
  ok = ok && esp_now_add_peer(&peer) == ESP_OK;
  if (!ok) while (true) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(100); }   // fast blink = radio failed
}

void loop() {
  static uint32_t rxCount = 0;
  StatePacket s;
  while (xQueueReceive(stateQ, &s, 0) == pdTRUE) {
    sendFrame((const uint8_t *)&s, sizeof(s));
    if (++rxCount % 50 == 0) digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  }
  // Parse serial frames from the laptop; forward valid commands at once as ESP-NOW broadcasts.
  static uint8_t buf[64], n = 0, need = 0, state = 0;
  while (Serial.available()) {
    uint8_t c = (uint8_t)Serial.read();
    if (state == 0) { if (c == 0xAA) state = 1; }
    else if (state == 1) { state = (c == 0x55) ? 2 : (c == 0xAA ? 1 : 0); }
    else if (state == 2) { need = c;  n = 0;  state = (need > 0 && need <= sizeof(buf)) ? 3 : 0; }
    else if (state == 3) { buf[n++] = c;  if (n == need) state = 4; }
    else {
      if (c == tw_crc8(buf, need) && need == sizeof(CmdPacket) && buf[0] == CMD_MAGIC && buf[1] == ROBOT_ID)
        esp_now_send(broadcastMac, buf, need);
      state = 0;
    }
  }
  delayMicroseconds(200);           // poll every 0.2 ms: the bridge adds well under 1 ms each way
}
