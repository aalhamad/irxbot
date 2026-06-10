/*
 * bno085.h
 *
 * BNO085 IMU driver for STM32 HAL via SH2 library (I2C)
 * Provides: rotation vector (quaternion), accelerometer, gyroscope
 *
 * Wiring:
 *   SDA → PB8, SCL → PB9
 *   PS0 → GND, PS1 → GND, ADDR → GND  (I2C mode, addr 0x4A)
 */

#ifndef BNO085_H
#define BNO085_H

#include "main.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

/* I2C address (ADDR pin = GND) */
#define BNO085_I2C_ADDR     (0x4A << 1)   /* HAL uses 8-bit address */

/* Report intervals in microseconds */
#define BNO085_ROT_VECTOR_US    10000      /* 100 Hz */
#define BNO085_ACCEL_US         10000      /* 100 Hz */
#define BNO085_GYRO_US          10000      /* 100 Hz */

/* IMU data structure */
typedef struct {
    /* Rotation vector (quaternion) */
    float qw, qx, qy, qz;
    float heading_accuracy;   /* radians */

    /* Accelerometer m/s^2 */
    float ax, ay, az;

    /* Gyroscope rad/s */
    float gx, gy, gz;

    /* Status flags */
    uint8_t valid;            /* 1 = data has been received at least once */
    uint8_t accuracy;         /* 0-3, sensor fusion accuracy */
    uint32_t timestamp_ms;
} BNO085_Data_t;

/* Public API */
HAL_StatusTypeDef BNO085_Init(I2C_HandleTypeDef *hi2c);
void              BNO085_Process(void);
BNO085_Data_t*    BNO085_GetData(void);
uint8_t           BNO085_IsReady(void);

#endif /* BNO085_H */
