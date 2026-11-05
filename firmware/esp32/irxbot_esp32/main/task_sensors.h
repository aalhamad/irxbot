#pragma once

#include <stdint.h>
#include <stdbool.h>

#define SENSOR_ERR_COUNT 6

typedef struct {
    /* INA228 — power */
    float    bus_v;
    float    current_a;
    float    power_w;

    /* BME280 — environment */
    float    temp_c;
    float    humidity_pct;
    float    pressure_hpa;

    /* SGP30 — air quality */
    uint16_t tvoc_ppb;
    uint16_t eco2_ppm;

    /* SPS30 — particulates */
    float    pm2_5;
    float    pm10;

    /* VEML7700 — light */
    float    lux;

    /* DS18B20 — temperatures (index by discovery order) */
    float    temp_batt_c;
    float    temp_elec_rear_c;
    float    temp_elec_front_c;

    /* Rail voltages (voltage dividers) */
    float    v_12v;
    float    v_5v;
    float    v_3v3;

    /* MQ-2 — smoke/gas */
    float    mq2_v;
    uint8_t  mq2_status;   /* 0=clean 1=elevated 2=alarm */

    /* Status flags */
    uint8_t  ok;           /* bit0=ina bit1=bme bit2=sgp bit3=sps bit4=veml bit5=ds bit6=mq2_warmed
                              bits 0-5 clear after 3 failed reads in a row */
    uint16_t err[SENSOR_ERR_COUNT];   /* failed reads since boot: ina,bme,sgp,sps,veml,ds */
} sensor_data_t;

extern volatile sensor_data_t g_sensors;

void task_sensors_start(void);
