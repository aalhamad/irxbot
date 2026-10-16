#define AMBER_L   15
#define AMBER_R   16
#define BLINK_MS  330     // standard turn-signal rate (~90/min)
#define BLINKS    6       // blinks per mode

void setLamps(bool l, bool r) {
  digitalWrite(AMBER_L, l ? HIGH : LOW);
  digitalWrite(AMBER_R, r ? HIGH : LOW);
}

void blink(bool useL, bool useR, const char* name) {
  Serial.println(name);
  for (int i = 0; i < BLINKS; i++) {
    setLamps(useL, useR);  delay(BLINK_MS);
    setLamps(false, false); delay(BLINK_MS);
  }
  delay(800);   // pause between modes
}

void setup() {
  pinMode(AMBER_L, OUTPUT);
  pinMode(AMBER_R, OUTPUT);
  setLamps(false, false);
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== Front Amber — Continuous Demo ===");
}

void loop() {
  blink(true,  false, "TURN LEFT");
  blink(false, true,  "TURN RIGHT");
  blink(true,  true,  "HAZARD");
}