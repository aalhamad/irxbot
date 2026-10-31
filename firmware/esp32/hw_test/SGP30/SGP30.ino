#include <Wire.h>

#define SGP30_ADDR 0x58

uint8_t sgp30_crc(uint8_t a, uint8_t b)
{
    uint8_t crc = 0xFF;
    crc ^= a; for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    crc ^= b; for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    return crc;
}

bool sendCmd(uint8_t a, uint8_t b)
{
    Wire.beginTransmission(SGP30_ADDR);
    Wire.write(a);
    Wire.write(b);
    return Wire.endTransmission() == 0;
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Wire.begin(8, 9);
    Wire.setClock(10000);
    Wire.setTimeOut(200);

    Serial.println("SGP30 raw I2C test");

    /* Address probe */
    Wire.beginTransmission(SGP30_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("ERROR: not found at 0x58");
        while (1) delay(1000);
    }
    Serial.println("Found at 0x58");

    /* Init_air_quality */
    if (!sendCmd(0x20, 0x03)) {
        Serial.println("ERROR: Init_air_quality failed");
        while (1) delay(1000);
    }
    Serial.println("Init OK — waiting 1s before first measurement");
    delay(1000);
}

void loop()
{
    /* Measure_air_quality */
    if (!sendCmd(0x20, 0x08)) {
        Serial.println("ERROR: Measure cmd NACK");
        delay(1000);
        return;
    }

    delay(12);   /* SGP30 needs ≤12ms to compute */

    if (Wire.requestFrom(SGP30_ADDR, 6) != 6) {
        Serial.println("ERROR: read returned <6 bytes");
        delay(1000);
        return;
    }

    uint8_t buf[6];
    for (int i = 0; i < 6; i++) buf[i] = Wire.read();

    Serial.printf("raw: %02X %02X crc=%02X(calc=%02X)  %02X %02X crc=%02X(calc=%02X)\n",
                  buf[0], buf[1], buf[2], sgp30_crc(buf[0], buf[1]),
                  buf[3], buf[4], buf[5], sgp30_crc(buf[3], buf[4]));

    if (sgp30_crc(buf[0], buf[1]) != buf[2]) {
        Serial.println("ERROR: eCO2 CRC fail");
        delay(1000);
        return;
    }
    if (sgp30_crc(buf[3], buf[4]) != buf[5]) {
        Serial.println("ERROR: TVOC CRC fail");
        delay(1000);
        return;
    }

    uint16_t eco2 = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t tvoc = ((uint16_t)buf[3] << 8) | buf[4];

    Serial.printf("eCO2: %u ppm   TVOC: %u ppb\n", eco2, tvoc);

    delay(1000);
}
