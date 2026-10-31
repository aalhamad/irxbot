#include <Wire.h>

// Raw I2C (no library), same register access as the ESP-IDF firmware.

#define VEML_ADDR 0x10
#define REG_CONF  0x00
#define REG_ALS   0x04
#define REG_WHITE 0x05

uint8_t lastErr = 0;

uint16_t readReg(uint8_t reg) {
  Wire.beginTransmission(VEML_ADDR);
  Wire.write(reg);
  lastErr = Wire.endTransmission(false);
  if (lastErr != 0) return 0;

  if (Wire.requestFrom(VEML_ADDR, 2) != 2) {
    lastErr = 100;   // 100 = short read
    return 0;
  }
  uint8_t lo = Wire.read();
  uint8_t hi = Wire.read();
  return (uint16_t)(lo | (hi << 8));
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(8, 9);
  Wire.setClock(10000);
  Wire.setTimeOut(200);

  Serial.println("VEML7700 raw I2C test");

  Serial.println("Scanning bus...");
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  found: 0x%02X\n", addr);
    }
  }

  // ALS_CONF = 0x0000: gain 1, IT 100 ms, powered on
  Wire.beginTransmission(VEML_ADDR);
  Wire.write(REG_CONF);
  Wire.write(0x00);
  Wire.write(0x00);
  uint8_t err = Wire.endTransmission();
  Serial.printf("Config write err=%u\n", err);

  delay(200);   // allow the first 100 ms integration to finish
}

void loop() {
  uint16_t als = readReg(REG_ALS);
  uint8_t alsErr = lastErr;
  uint16_t white = readReg(REG_WHITE);
  uint8_t whiteErr = lastErr;
  uint16_t conf = readReg(REG_CONF);
  uint8_t confErr = lastErr;

  Serial.printf("t=%lus als=%u(err=%u) white=%u(err=%u) conf=0x%04X(err=%u) lux=%.1f\n",
                millis() / 1000, als, alsErr, white, whiteErr, conf, confErr, als * 0.0576f);

  delay(1000);
}
