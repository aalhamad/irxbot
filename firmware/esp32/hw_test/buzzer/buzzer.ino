#define BUZZER_PIN 21

void beep(int on, int off) {
  digitalWrite(BUZZER_PIN, HIGH); delay(on);
  digitalWrite(BUZZER_PIN, LOW);  delay(off);
}

void bootSound()    { beep(100, 50); beep(100, 50); beep(300, 0); }
void okSound()      { beep(200, 100); beep(400, 0); }
void warningSound() { for (int i = 0; i < 5; i++)  beep(100, 100); }
void faultSound()   { for (int i = 0; i < 10; i++) beep(50, 50); }

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== Buzzer Test (GPIO 21) ===");
  Serial.println("Boot");  bootSound();
  delay(1500);
}

void loop() {
  Serial.println("OK");      okSound();      delay(2000);
  Serial.println("Warning"); warningSound(); delay(2000);
  Serial.println("Fault");   faultSound();   delay(3000);
}