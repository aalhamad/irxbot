#include <Wire.h>

#define SPS30_ADDR 0x69

uint8_t crc8(const uint8_t *data) {
  uint8_t crc = 0xFF;
  for (int i = 0; i < 2; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
  }
  return crc;
}

void sendCmd(uint16_t cmd) {
  Wire.beginTransmission(SPS30_ADDR);
  Wire.write(cmd >> 8);
  Wire.write(cmd & 0xFF);
  Wire.endTransmission();
}

bool startMeasurement() {
  uint8_t arg[2] = {0x03, 0x00};          // float output format
  Wire.beginTransmission(SPS30_ADDR);
  Wire.write(0x00); Wire.write(0x10);     // start measurement
  Wire.write(arg[0]); Wire.write(arg[1]);
  Wire.write(crc8(arg));
  return Wire.endTransmission() == 0;
}

bool dataReady() {
  sendCmd(0x0202);
  delay(5);
  if (Wire.requestFrom(SPS30_ADDR, 3) != 3) return false;
  uint8_t d[2] = {(uint8_t)Wire.read(), (uint8_t)Wire.read()};
  uint8_t c = Wire.read();
  return (crc8(d) == c) && d[1] == 1;
}

bool readMeasurement(float *v) {   // 10 floats
  sendCmd(0x0300);
  delay(5);
  if (Wire.requestFrom(SPS30_ADDR, 60) != 60) return false;
  for (int i = 0; i < 10; i++) {
    uint8_t b[4];
    for (int w = 0; w < 2; w++) {
      uint8_t d[2] = {(uint8_t)Wire.read(), (uint8_t)Wire.read()};
      if (crc8(d) != Wire.read()) return false;
      b[w * 2] = d[0]; b[w * 2 + 1] = d[1];
    }
    uint32_t raw = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
                   ((uint32_t)b[2] << 8)  |  b[3];
    memcpy(&v[i], &raw, 4);
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(8, 9);
  Wire.setClock(100000);   // SPS30 max 100kHz

  Serial.println("\n=== SPS30 Test (no library) ===");

  Wire.beginTransmission(SPS30_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("ERROR: SPS30 not found at 0x69 — check 5V, SEL→GND, SDA/SCL");
    while (1) delay(100);
  }
  Serial.println("SPS30 found ✓");

  if (!startMeasurement()) Serial.println("ERROR: start failed");
  else Serial.println("Measuring... stable after ~30s");
}

void loop() {
  if (!dataReady()) { delay(200); return; }

  float v[10];
  if (!readMeasurement(v)) {
    Serial.println("Read/CRC error");
    delay(1000);
    return;
  }

  Serial.println("─────────────────────────────");
  Serial.printf("PM1.0  : %6.2f µg/m³\n", v[0]);
  Serial.printf("PM2.5  : %6.2f µg/m³\n", v[1]);
  Serial.printf("PM4.0  : %6.2f µg/m³\n", v[2]);
  Serial.printf("PM10   : %6.2f µg/m³\n", v[3]);
  Serial.printf("Size   : %6.2f µm\n",    v[9]);
  delay(1000);
}