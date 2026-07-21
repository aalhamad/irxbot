#include <Wire.h>
#include <Adafruit_BME280.h>

#define BME280_ADDR 0x76

Adafruit_BME280 bme;

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(21, 22);

  Serial.println("\n=== BME280 Test ===");

  if (!bme.begin(BME280_ADDR, &Wire)) {
    Serial.println("ERROR: BME280 not found! Check wiring.");
    while (1) delay(100);
  }

  Serial.println("BME280 found ✓");
}

void loop() {
  Serial.println("─────────────────────────────");
  Serial.printf("Temperature : %.2f °C\n",   bme.readTemperature());
  Serial.printf("Humidity    : %.2f %%\n",    bme.readHumidity());
  Serial.printf("Pressure    : %.2f hPa\n",  bme.readPressure() / 100.0f);
  delay(2000);
}