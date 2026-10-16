#define SPOT_PIN 14

void setup() {
  pinMode(SPOT_PIN, OUTPUT);
  digitalWrite(SPOT_PIN, LOW);
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== Spotlight Test (GPIO 14) ===");

  for (int i = 0; i < 3; i++) {
    Serial.println("ON");  digitalWrite(SPOT_PIN, HIGH); delay(1000);
    Serial.println("OFF"); digitalWrite(SPOT_PIN, LOW);  delay(1000);
  }
}

void loop() {
  Serial.println("Spotlights ON");  digitalWrite(SPOT_PIN, HIGH); delay(5000);
  Serial.println("Spotlights OFF"); digitalWrite(SPOT_PIN, LOW);  delay(5000);
}