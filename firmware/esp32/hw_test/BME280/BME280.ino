#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SDA_PIN 8
#define SCL_PIN 9

Adafruit_BME280 bme;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("BME280 Test");

  // Most BME280 modules use 0x76.
  // Some use 0x77.
  if (!bme.begin(0x76, &Wire)) {
    Serial.println("BME280 not found at 0x76.");

    if (!bme.begin(0x77, &Wire)) {
      Serial.println("BME280 not found at 0x77 either.");
      while (1) {
        delay(1000);
      }
    }
  }

  Serial.println("BME280 found!");
}

void loop() {
  Serial.print("Temperature: ");
  Serial.print(bme.readTemperature());
  Serial.println(" °C");

  Serial.print("Humidity: ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.print("Pressure: ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.println("--------------------");

  delay(2000);
}