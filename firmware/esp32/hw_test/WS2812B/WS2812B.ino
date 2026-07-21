#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN    5 //5, 12, 13, 14
#define LED_COUNT  12 // 12, 18

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setAll(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < LED_COUNT; i++)
    strip.setPixelColor(i, strip.Color(r, g, b));
  strip.show();
}

void pulse(uint8_t r, uint8_t g, uint8_t b, int times) {
  for (int t = 0; t < times; t++) {
    for (int i = 0; i < 255; i += 5) {
      setAll(r * i / 255, g * i / 255, b * i / 255);
      delay(10);
    }
    for (int i = 255; i > 0; i -= 5) {
      setAll(r * i / 255, g * i / 255, b * i / 255);
      delay(10);
    }
  }
}

void setup() {
  Serial.begin(115200);
  strip.begin();
  strip.setBrightness(80);
  strip.show();
  Serial.println("\n=== WS2812B Test ===");
}

void loop() {
  Serial.println("GREEN  — All OK");
  setAll(0, 255, 0);
  delay(2000);

  Serial.println("BLUE   — Charging");
  setAll(0, 0, 255);
  delay(2000);

  Serial.println("YELLOW — Warning");
  setAll(255, 150, 0);
  delay(2000);

  Serial.println("RED    — Fault");
  setAll(255, 0, 0);
  delay(2000);

  Serial.println("ORANGE — Battery low");
  setAll(255, 50, 0);
  delay(2000);

  Serial.println("WHITE PULSE — Jetson comms lost");
  pulse(255, 255, 255, 3);

  Serial.println("OFF");
  setAll(0, 0, 0);
  delay(1000);
}