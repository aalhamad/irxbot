#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define TFT_CS   10
#define TFT_DC    41
#define TFT_RST   5

#define TFT_SCLK 12
#define TFT_MOSI 11


Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);

  tft.init(240, 320);
  tft.setRotation(1);

  // Test screen
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(4);
  tft.setCursor(40, 60);
  tft.println("IRX");

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(35, 120);
  tft.println("SYSTEM TEST");

  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(35, 160);
  tft.println("ESP32-S3 ONLINE");
}

void loop() {
}