#define MQ2_PIN    1
#define DIV_RATIO  1.5f        // (10k + 20k) / 20k
#define WARMUP_MS  180000UL    // 3 minutes

unsigned long startTime;
float baseline = 0;
bool baselineSet = false;

float readAout() {
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) {           // average 16 samples
    sum += analogReadMilliVolts(MQ2_PIN);
    delay(5);
  }
  return (sum / 16.0f) * DIV_RATIO / 1000.0f;  // real AOUT volts
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  analogReadResolution(12);
  analogSetPinAttenuation(MQ2_PIN, ADC_11db);   // 0–3.3V range
  startTime = millis();
  Serial.println("\n=== MQ-2 Test (GPIO 1) ===");
  Serial.println("Warming up 3 min — keep in clean air");
}

void loop() {
  unsigned long elapsed = millis() - startTime;
  float aout = readAout();

  Serial.println("─────────────────────────────");

  if (elapsed < WARMUP_MS) {
    Serial.printf("WARMING UP — %lu s left\n", (WARMUP_MS - elapsed) / 1000);
    Serial.printf("AOUT : %.3f V (not valid yet)\n", aout);
    delay(5000);
    return;
  }

  if (!baselineSet) {                        // clean-air reference
    float sum = 0;
    for (int i = 0; i < 10; i++) { sum += readAout(); delay(200); }
    baseline = sum / 10.0f;
    baselineSet = true;
    Serial.printf("Baseline set: %.3f V\n", baseline);
  }

  float rise = (aout - baseline) / baseline * 100.0f;

  Serial.printf("AOUT     : %.3f V\n", aout);
  Serial.printf("Baseline : %.3f V\n", baseline);
  Serial.printf("Rise     : %+.1f %%\n", rise);
  Serial.printf("Status   : %s\n",
                rise < 20 ? "CLEAN" : rise < 60 ? "ELEVATED" : "ALARM");

  delay(2000);
}