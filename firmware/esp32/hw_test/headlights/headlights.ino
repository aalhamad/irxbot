#include <Arduino.h>

#define HEADLIGHT_PIN 25

void setup() {
  Serial.begin(115200);
  pinMode(HEADLIGHT_PIN, OUTPUT);
  digitalWrite(HEADLIGHT_PIN, LOW);
  delay(500);
  Serial.println("=== Headlight Test ===");

  // Test ON/OFF 3 times
  for (int i = 0; i < 3; i++) {
    Serial.println("ON");
    digitalWrite(HEADLIGHT_PIN, HIGH);
    delay(1000);
    Serial.println("OFF");
    digitalWrite(HEADLIGHT_PIN, LOW);
    delay(1000);
  }
}

void loop() {
  // Auto brightness based on light level (VEML7700 later)
  Serial.println("Headlights ON");
  digitalWrite(HEADLIGHT_PIN, HIGH);
  delay(5000);
  Serial.println("Headlights OFF");
  digitalWrite(HEADLIGHT_PIN, LOW);
  delay(5000);
}