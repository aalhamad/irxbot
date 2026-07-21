#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define TFT_MOSI  23
#define TFT_SCLK  18
#define TFT_CS    15
#define TFT_DC     2
#define TFT_RST   27

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n=== ST7789 Test ===");

  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  Serial.println("Screen initialized ✓");

  // Test colors
  Serial.println("RED");
  tft.fillScreen(ST77XX_RED);
  delay(1000);

  Serial.println("GREEN");
  tft.fillScreen(ST77XX_GREEN);
  delay(1000);

  Serial.println("BLUE");
  tft.fillScreen(ST77XX_BLUE);
  delay(1000);

  tft.fillScreen(ST77XX_BLACK);

  // Test text
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("IRXBot v1.0");

  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(1);
  tft.setCursor(10, 40);
  tft.println("Display OK");

  // Test status layout
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(10, 70);
  tft.println("Voltage : 24.8V");

  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(10, 90);
  tft.println("Current : 2.3A");

  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 110);
  tft.println("Temp    : 28.4C");

  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(10, 130);
  tft.println("Status  : OK");

  Serial.println("Done ✓");
}

void loop() {
}