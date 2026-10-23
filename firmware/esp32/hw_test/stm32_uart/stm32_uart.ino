/*
 * stm32_uart.ino — USART3 link test for IRXBot.
 *
 * Wiring:
 *   STM32 PC10 (USART3 TX) → ESP32 GPIO 35 (RX)
 *   STM32 PC5  (USART3 RX) ← ESP32 GPIO 36 (TX)
 *   GND ──────────────────── GND
 *
 * Open Serial Monitor at 115200. Press a bumper on the robot
 * to see BMP messages, wait ~10 s for SPEED messages.
 */

#define STM32_RX  35
#define STM32_TX  36

void setup()
{
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, STM32_RX, STM32_TX);
    Serial.println("STM32 UART test — waiting for messages...");
}

void loop()
{
    while (Serial1.available())
        Serial.write(Serial1.read());
}
