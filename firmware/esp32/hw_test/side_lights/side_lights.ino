#include <Adafruit_NeoPixel.h>

#define N 23
Adafruit_NeoPixel L(N, 38, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel R(N, 39, NEO_GRB + NEO_KHZ800);

void fill(Adafruit_NeoPixel &s, uint32_t c) { s.fill(c); s.show(); }
void both(uint8_t r, uint8_t g, uint8_t b) {
  fill(L, L.Color(r, g, b)); fill(R, R.Color(r, g, b));
}

// Sequential amber sweep, front → rear
void sweep(Adafruit_NeoPixel &s) {
  for (int k = 0; k < 3; k++) {
    s.clear(); s.show();
    for (int i = 0; i < N; i++) {
      s.setPixelColor(i, s.Color(255, 90, 0));
      s.show(); delay(12);
    }
    delay(250);
    s.clear(); s.show(); delay(250);
  }
}

void setup() {
  Serial.begin(115200);
  L.begin(); R.begin();
  L.setBrightness(60); R.setBrightness(60);
  both(0, 0, 0);
  Serial.println("\n=== WS2812B Side Strips (23 LEDs) ===");
}

void loop() {
  Serial.println("LEFT only");  both(0,0,0); fill(L, L.Color(255,255,255)); delay(1500);
  Serial.println("RIGHT only"); both(0,0,0); fill(R, R.Color(255,255,255)); delay(1500);

  Serial.println("OK green");      both(0, 255, 0);   delay(2000);
  Serial.println("Charging blue"); both(0, 0, 255);   delay(2000);
  Serial.println("Warning");       both(255, 150, 0); delay(2000);
  Serial.println("Fault red");     both(255, 0, 0);   delay(2000);

  both(0, 0, 0);
  Serial.println("Turn LEFT sweep");  sweep(L);
  Serial.println("Turn RIGHT sweep"); sweep(R);
}