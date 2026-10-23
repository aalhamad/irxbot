#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  Wire.setClock(10000);
  Wire.setTimeOut(200);

  Serial.println("I2C Scanner");

  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte err = Wire.endTransmission();

    if (err == 0) {
      Serial.print("Found: 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
    }
  }

  Serial.println("Done");
}

void loop() {}