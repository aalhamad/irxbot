#pragma once

#include <stddef.h>

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

    /* Status flags */
    uint8_t  ok;           /* bit0=ina bit1=bme bit2=sgp bit3=sps bit4=veml bit5=ds
                              bits 0-5 clear after 3 failed reads in a row */
    uint16_t err[SENSOR_ERR_COUNT];   /* failed reads since boot: ina,bme,sgp,sps,veml,ds */
} sensor_data_t;

extern volatile sensor_data_t g_sensors;

void task_sensors_start(void);

/* Addresses that answered the last I2C scan, e.g. "0x10 0x40 0x69", or
   "none". Refreshed every 10 s. */
void sensors_get_i2c_scan(char *out, size_t len);

/* DS18B20s found on the 1-Wire data line, by LCD slot, e.g.
   "bat=8600000079394828 rear=B4000000149D4628:parasite", or "none" */
void sensors_get_ow_scan(char *out, size_t len);
