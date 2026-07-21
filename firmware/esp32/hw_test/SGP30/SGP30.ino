#include <Wire.h>
#include <Adafruit_SGP30.h>

Adafruit_SGP30 sgp;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  Wire.setClock(100000);

  Serial.println("SGP30 Test");

  if (!sgp.begin()) {
    Serial.println("SGP30 not found!");
    while (1) delay(1000);
  }

  Serial.println("SGP30 found");

  Serial.print("Serial Number: ");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(":");
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.print(":");
  Serial.println(sgp.serialnumber[2], HEX);
}

void loop() {

  if (!sgp.IAQmeasure()) {
    Serial.println("Measurement failed");
    delay(1000);
    return;
  }

  Serial.println("---------------------");

  Serial.print("TVOC : ");
  Serial.print(sgp.TVOC);
  Serial.println(" ppb");

  Serial.print("eCO2 : ");
  Serial.print(sgp.eCO2);
  Serial.println(" ppm");

  delay(1000);
}