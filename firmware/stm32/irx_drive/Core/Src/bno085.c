/*
 * bno085.c
 *
 * BNO085 IMU driver — STM32 HAL + SH2 library integration
 */

#include "bno085.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include <string.h>

/* ── Private state ─────────────────────────────────────────────────────────── */

static I2C_HandleTypeDef *_hi2c        = NULL;
static sh2_Hal_t          _sh2hal;
static BNO085_Data_t      _imu_data;
static uint8_t            _initialized = 0;

/* ── SH2 HAL callbacks (I2C transport) ─────────────────────────────────────── */

static int hal_open(sh2_Hal_t *self)
{
    (void)self;
    return SH2_OK;
}

static void hal_close(sh2_Hal_t *self)
{
    (void)self;
}

static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    (void)self;

    uint8_t header[4];
    if (HAL_I2C_Master_Receive(_hi2c, BNO085_I2C_ADDR, header, 4, 10) != HAL_OK)
        return 0;

    uint16_t packet_len = ((uint16_t)(header[1] & 0x7F) << 8) | header[0];

    if (packet_len == 0 || packet_len > (uint16_t)len)
        return 0;

    if (HAL_I2C_Master_Receive(_hi2c, BNO085_I2C_ADDR, pBuffer, packet_len, 50) != HAL_OK)
        return 0;

    *t_us = HAL_GetTick() * 1000;
    return (int)packet_len;
}

static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    (void)self;
    if (HAL_I2C_Master_Transmit(_hi2c, BNO085_I2C_ADDR, pBuffer, len, 50) != HAL_OK)
        return 0;
    return (int)len;
}

static uint32_t hal_getTimeUs(sh2_Hal_t *self)
{
    (void)self;
    return HAL_GetTick() * 1000;
}

/* ── Async event callback (reset/errors — required by sh2_open) ─────────────── */

static void async_event_handler(void *cookie, sh2_AsyncEvent_t *event)
{
    (void)cookie;
    if (event->eventId == SH2_RESET)
    {
        _initialized = 0;
    }
}

/* ── Sensor event callback ──────────────────────────────────────────────────── */

static void sensor_event_handler(void *cookie, sh2_SensorEvent_t *event)
{
    (void)cookie;

    sh2_SensorValue_t value;
    if (sh2_decodeSensorEvent(&value, event) != SH2_OK)
        return;

    _imu_data.timestamp_ms = HAL_GetTick();
    _imu_data.valid        = 1;

    switch (value.sensorId)
    {
        case SH2_ROTATION_VECTOR:
            _imu_data.qw               = value.un.rotationVector.real;
            _imu_data.qx               = value.un.rotationVector.i;
            _imu_data.qy               = value.un.rotationVector.j;
            _imu_data.qz               = value.un.rotationVector.k;
            _imu_data.heading_accuracy = value.un.rotationVector.accuracy;
            _imu_data.accuracy         = value.status & 0x03;
            break;

        case SH2_ACCELEROMETER:
            _imu_data.ax = value.un.accelerometer.x;
            _imu_data.ay = value.un.accelerometer.y;
            _imu_data.az = value.un.accelerometer.z;
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            _imu_data.gx = value.un.gyroscope.x;
            _imu_data.gy = value.un.gyroscope.y;
            _imu_data.gz = value.un.gyroscope.z;
            break;

        default:
            break;
    }
}

/* ── Public API ─────────────────────────────────────────────────────────────── */

HAL_StatusTypeDef BNO085_Init(I2C_HandleTypeDef *hi2c)
{
    _hi2c = hi2c;
    memset(&_imu_data, 0, sizeof(_imu_data));
    _imu_data.qw = 1.0f;

    _sh2hal.open      = hal_open;
    _sh2hal.close     = hal_close;
    _sh2hal.read      = hal_read;
    _sh2hal.write     = hal_write;
    _sh2hal.getTimeUs = hal_getTimeUs;

    HAL_Delay(100);

    /* sh2_open takes async event callback (reset/errors) */
    if (sh2_open(&_sh2hal, async_event_handler, NULL) != SH2_OK)
        return HAL_ERROR;

    /* Sensor data callback registered separately */
    sh2_setSensorCallback(sensor_event_handler, NULL);

    sh2_SensorConfig_t config;
    memset(&config, 0, sizeof(config));

    config.reportInterval_us = BNO085_ROT_VECTOR_US;
    if (sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config) != SH2_OK)
        return HAL_ERROR;

    config.reportInterval_us = BNO085_ACCEL_US;
    if (sh2_setSensorConfig(SH2_ACCELEROMETER, &config) != SH2_OK)
        return HAL_ERROR;

    config.reportInterval_us = BNO085_GYRO_US;
    if (sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &config) != SH2_OK)
        return HAL_ERROR;

    _initialized = 1;
    return HAL_OK;
}

void BNO085_Process(void)
{
    if (!_initialized) return;
    sh2_service();
}

BNO085_Data_t* BNO085_GetData(void)
{
    return &_imu_data;
}

uint8_t BNO085_IsReady(void)
{
    return _initialized && _imu_data.valid;
}
