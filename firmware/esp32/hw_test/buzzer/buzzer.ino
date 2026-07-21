#include <Arduino.h>

#define BUZZER_PIN 32

void beep(int duration_ms, int pause_ms) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration_ms);
  digitalWrite(BUZZER_PIN, LOW);
  delay(pause_ms);
}

void bootSound() {
  beep(100, 50);
  beep(100, 50);
  beep(300, 0);
}

void alertSound() {
  for (int i = 0; i < 3; i++) {
    beep(50, 50);
  }
}

void okSound() {
  beep(200, 100);
  beep(400, 0);
}

void warningSound() {
  for (int i = 0; i < 5; i++) {
    beep(100, 100);
  }
}

void faultSound() {
  for (int i = 0; i < 10; i++) {
    beep(50, 50);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  delay(500);

  Serial.println("=== Buzzer Sound Test ===");

  Serial.println("Boot sound");
  bootSound();
  delay(1000);

  Serial.println("OK sound");
  okSound();
  delay(1000);

  Serial.println("Alert sound");
  alertSound();
  delay(1000);

  Serial.println("Warning sound");
  warningSound();
  delay(1000);

  Serial.println("Fault sound");
  faultSound();
  delay(1000);
}

void loop() {
  Serial.println("OK");
  okSound();
  delay(3000);
}