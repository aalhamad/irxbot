/*
 * vl53l1x.h
 *
 * Minimal VL53L1X driver for IRXBot — 4 sensors, shared I2C1, individual XSHUT.
 * Register-level implementation (no ST ULD dependency).
 */

#ifndef VL53L1X_H
#define VL53L1X_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define TOF_COUNT            5

/* Indices — ordered by XSHUT wiring */
#define TOF_RIGHT            0   /* XSHUT PB0 → address 0x30 */
#define TOF_LEFT             1   /* XSHUT PA8 → address 0x31 */
#define TOF_FRONT_LEFT       2   /* XSHUT PC7 → address 0x32 */
#define TOF_FRONT_RIGHT      3   /* XSHUT PB6 → address 0x33 */
#define TOF_REAR             4   /* XSHUT PC9 → address 0x34 */

/* Re-assigned 7-bit I2C addresses (shifted <<1 for HAL internally) */
#define TOF_ADDR_BASE        0x30

typedef struct
{
    uint16_t distance_mm;   /* last valid distance */
    uint8_t  status;        /* 0 = valid range, else error/wraparound code */
    uint8_t  online;        /* 1 = sensor initialized and responding */
} ToF_Data_t;

/* Initialize all 4 sensors: XSHUT sequencing, address assignment, start ranging.
 * Returns number of sensors successfully brought online (0..4).
 * Never blocks indefinitely; a dead sensor is marked offline and skipped. */
uint8_t ToF_InitAll(I2C_HandleTypeDef *hi2c);

/* Non-blocking-ish service call: checks each online sensor for a ready sample,
 * reads it if available. Call from main loop. */
void ToF_Process(void);

/* Access latest data for sensor idx (TOF_FRONT..TOF_RIGHT). */
ToF_Data_t* ToF_Get(uint8_t idx);

#endif /* VL53L1X_H */
