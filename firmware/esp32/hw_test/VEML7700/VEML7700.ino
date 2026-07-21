#include <Wire.h>
#include <Adafruit_VEML7700.h>

Adafruit_VEML7700 veml = Adafruit_VEML7700();

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  Wire.setClock(10000);     // slow/stable I2C for your setup
  Wire.setTimeOut(200);

  Serial.println("VEML7700 Light Sensor Test");

  if (!veml.begin()) {
    Serial.println("VEML7700 not found. Check wiring.");
    while (1) delay(1000);
  }

  Serial.println("VEML7700 found.");

  veml.setGain(VEML7700_GAIN_1);
  veml.setIntegrationTime(VEML7700_IT_100MS);
}

void loop() {
  float lux = veml.readLux();
  uint16_t white = veml.readWhite();
  uint16_t als = veml.readALS();

  Serial.println("--------------------");
  Serial.print("Lux   : ");
  Serial.println(lux);

  Serial.print("White : ");
  Serial.println(white);

  Serial.print("ALS   : ");
  Serial.println(als);

  delay(1000);
}