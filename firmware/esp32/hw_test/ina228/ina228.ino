#include <Wire.h>

#define INA_ADDR 0x40

#define REG_VSHUNT  0x04
#define REG_VBUS    0x05
#define REG_MANUF   0x3E
#define REG_DEVICE  0x3F

#define SHUNT_OHMS  0.002f   // R002 = 0.002 ohm

int32_t read20_signed(uint8_t reg) {
  Wire.beginTransmission(INA_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    Serial.println("Reg select failed");
    return 0;
  }

  int n = Wire.requestFrom(INA_ADDR, (uint8_t)3);
  if (n != 3) {
    Serial.println("Read failed");
    return 0;
  }

  uint32_t raw24 = ((uint32_t)Wire.read() << 16) |
                   ((uint32_t)Wire.read() << 8) |
                    Wire.read();

  int32_t raw20 = raw24 >> 4;

  if (raw20 & 0x80000) {
    raw20 |= 0xFFF00000;
  }

  return raw20;
}

uint32_t read20_unsigned(uint8_t reg) {
  Wire.beginTransmission(INA_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    Serial.println("Reg select failed");
    return 0;
  }

  int n = Wire.requestFrom(INA_ADDR, (uint8_t)3);
  if (n != 3) {
    Serial.println("Read failed");
    return 0;
  }

  uint32_t raw24 = ((uint32_t)Wire.read() << 16) |
                   ((uint32_t)Wire.read() << 8) |
                    Wire.read();

  return raw24 >> 4;
}

uint16_t read16(uint8_t reg) {
  Wire.beginTransmission(INA_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) return 0;

  int n = Wire.requestFrom(INA_ADDR, (uint8_t)2);
  if (n != 2) return 0;

  return ((uint16_t)Wire.read() << 8) | Wire.read();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  Wire.setClock(10000);
  Wire.setTimeOut(200);

  Serial.println("INA228 Simple Voltage / Current Test");

  uint16_t manuf = read16(REG_MANUF);
  uint16_t device = read16(REG_DEVICE);

  Serial.print("Manufacturer ID: 0x");
  Serial.println(manuf, HEX);

  Serial.print("Device ID: 0x");
  Serial.println(device, HEX);
}

void loop() {
  uint32_t raw_vbus = read20_unsigned(REG_VBUS);
  int32_t raw_shunt = read20_signed(REG_VSHUNT);

  float busVoltage = raw_vbus * 0.0001953125f;
  float shuntVoltage_V = raw_shunt * 0.0000003125f;
  float shuntVoltage_mV = -shuntVoltage_V * 1000.0f;

  float current_A = -shuntVoltage_V / SHUNT_OHMS;
  float power_W = busVoltage * current_A;

  Serial.println("----------------------------");

  Serial.print("Bus Voltage   : ");
  Serial.print(busVoltage, 4);
  Serial.println(" V");

  Serial.print("Shunt Voltage : ");
  Serial.print(shuntVoltage_mV, 4);
  Serial.println(" mV");

  Serial.print("Current       : ");
  Serial.print(current_A, 4);
  Serial.println(" A");

  Serial.print("Power         : ");
  Serial.print(power_W, 4);
  Serial.println(" W");

  delay(1000);
}