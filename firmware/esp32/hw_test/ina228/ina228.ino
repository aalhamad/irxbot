W#include <Wire.h>

#define INA228_ADDR     0x40
#define REG_CONFIG      0x00
#define REG_ADC_CONFIG  0x01
#define REG_SHUNT_CAL   0x02
#define REG_VSHUNT      0x04
#define REG_VBUS        0x05
#define REG_DIETEMP     0x06
#define REG_CURRENT     0x07
#define REG_POWER       0x08
#define REG_MANUF_ID    0x3E
#define REG_DEVICE_ID   0x3F

#define SHUNT_OHMS      0.002f
#define MAX_CURRENT_A   5.0f

#define VBUS_LSB        0.0001953125f    // 195.3125 uV/LSB
#define VSHUNT_LSB      0.0000003125f    // 312.5 nV/LSB (ADCRANGE=0)
#define TEMP_LSB        0.0078125f       // 7.8125 m°C/LSB

float CURRENT_LSB = 0;

// ───── I2C Helpers ─────

void writeReg16(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(INA228_ADDR);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}

uint16_t readReg16(uint8_t reg) {
  Wire.beginTransmission(INA228_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(INA228_ADDR, (uint8_t)2);
  return ((uint16_t)Wire.read() << 8) | Wire.read();
}

// Correct 24-bit signed read with proper sign extension
int32_t readReg24_signed(uint8_t reg) {
  Wire.beginTransmission(INA228_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(INA228_ADDR, (uint8_t)3);
  uint32_t raw = ((uint32_t)Wire.read() << 16) |
                 ((uint32_t)Wire.read() << 8)  |
                  (uint32_t)Wire.read();
  // Sign extend from 24-bit to 32-bit, then shift right 4 for 20-bit result
  if (raw & 0x800000) raw |= 0xFF000000;  // sign extend
  return ((int32_t)raw) >> 4;
}

uint32_t readReg24_unsigned(uint8_t reg) {
  Wire.beginTransmission(INA228_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(INA228_ADDR, (uint8_t)3);
  return ((uint32_t)Wire.read() << 16) |
         ((uint32_t)Wire.read() << 8)  |
          (uint32_t)Wire.read();
}

// ───── Sensor Reads ─────

float getBusVoltage_V() {
  return (float)readReg24_signed(REG_VBUS) * VBUS_LSB;
}

float getShuntVoltage_mV() {
  return (float)readReg24_signed(REG_VSHUNT) * VSHUNT_LSB * 1000.0f;
}

float getCurrent_mA() {
  return (float)readReg24_signed(REG_CURRENT) * CURRENT_LSB * 1000.0f;
}

float getPower_W() {
  return (float)readReg24_unsigned(REG_POWER) * 3.2f * CURRENT_LSB;
}

float getTemperature_C() {
  // DIETEMP is 16-bit register, upper 12 bits are temp (signed), lower 4 are unused
  int16_t raw = (int16_t)readReg16(REG_DIETEMP);
  return (float)(raw >> 4) * TEMP_LSB;
}

// ───── Zero-offset calibration (run with NO load) ─────
float current_offset_mA = 0.0f;

void calibrateZero() {
  Serial.println("Calibrating zero offset (no load)...");
  float sum = 0;
  int samples = 20;
  for (int i = 0; i < samples; i++) {
    sum += getCurrent_mA();
    delay(100);
  }
  current_offset_mA = sum / samples;
  Serial.printf("Zero offset: %.4f mA (will be subtracted from readings)\n\n", current_offset_mA);
}

// ───── Setup ─────

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(21, 22);

  Serial.println("\n=== INA228 Fixed Version ===");

  uint16_t manuf = readReg16(REG_MANUF_ID);
  uint16_t devid = readReg16(REG_DEVICE_ID);
  Serial.printf("Manufacturer ID : 0x%04X %s\n", manuf, manuf == 0x5449 ? "✓" : "✗ WRONG!");
  Serial.printf("Device ID       : 0x%04X %s\n", devid, (devid & 0xFFF0) == 0x2280 ? "✓" : "✗ WRONG!");

  if (manuf != 0x5449) {
    Serial.println("ERROR: INA228 not found! Check wiring.");
    while (1) delay(100);
  }

  // Reset
  writeReg16(REG_CONFIG, 0x8000);
  delay(20);

  // CONFIG: ADCRANGE=0 (±163.84mV shunt range), no alerts
  writeReg16(REG_CONFIG, 0x0000);

  // ADC_CONFIG: Continuous all, 1052us conv time, 16x averaging
  writeReg16(REG_ADC_CONFIG, 0xFB6B);

  // Shunt calibration
  CURRENT_LSB = MAX_CURRENT_A / 524288.0f;
  uint16_t shunt_cal = (uint16_t)(13107.2e6f * CURRENT_LSB * SHUNT_OHMS);
  writeReg16(REG_SHUNT_CAL, shunt_cal);

  Serial.printf("CURRENT_LSB : %.9f A/bit\n", CURRENT_LSB);
  Serial.printf("SHUNT_CAL   : %d\n", shunt_cal);

  delay(200);  // Let ADC settle before calibration
  calibrateZero();  // ← comment this out if you have load connected at startup
}

// ───── Loop ─────

void loop() {
  float voltage  = getBusVoltage_V();
  float shuntmV  = getShuntVoltage_mV();
  float current  = getCurrent_mA() - current_offset_mA;  // offset corrected
  float power    = getPower_W();
  float temp     = getTemperature_C();

  Serial.println("─────────────────────────────");
  Serial.printf("Bus Voltage  : %8.4f V\n",  voltage);
  Serial.printf("Shunt Voltage: %8.4f mV\n", shuntmV);
  Serial.printf("Current      : %8.3f mA\n", current);
  Serial.printf("Power        : %8.4f W\n",  power);
  Serial.printf("Temperature  : %6.2f °C\n", temp);   // Should now show ~25°C

  delay(1000);
}