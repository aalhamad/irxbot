#define RED_L     17
#define RED_R     18
#define TAIL      64      // ~25% brightness
#define BRAKE     255     // 100%
#define BLINK_MS  330
#define BLINKS    6

void setRed(int l, int r) {
  analogWrite(RED_L, l);
  analogWrite(RED_R, r);
}

void turn(bool left, bool right, const char* name) {
  Serial.println(name);
  for (int i = 0; i < BLINKS; i++) {
    setRed(left ? BRAKE : TAIL, right ? BRAKE : TAIL);  delay(BLINK_MS);
    setRed(left ? 0 : TAIL,     right ? 0 : TAIL);      delay(BLINK_MS);
  }
  setRed(TAIL, TAIL);
  delay(800);
}

void setup() {
  pinMode(RED_L, OUTPUT);
  pinMode(RED_R, OUTPUT);
  setRed(0, 0);
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== Rear Red — Continuous Demo ===");
}

void loop() {
  Serial.println("TAIL (dim)");   setRed(TAIL, TAIL);   delay(2000);
  Serial.println("BRAKE (full)"); setRed(BRAKE, BRAKE); delay(2000);
  setRed(TAIL, TAIL);             delay(800);

  turn(true,  false, "TURN LEFT");
  turn(false, true,  "TURN RIGHT");
  turn(true,  true,  "HAZARD");

  Serial.println("OFF");          setRed(0, 0);         delay(1500);
}