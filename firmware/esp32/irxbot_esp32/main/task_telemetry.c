#include "task_telemetry.h"
#include "task_sensors.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#define TELEMETRY_PERIOD_MS 5000

/* Sends one "ENV ..." line to the console, which also reaches USB. Logs share
   that port, so the Jetson must keep only lines starting with "ENV ". */
static void telemetry_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));

        printf("ENV vbat=%.2f i=%.3f v12=%.2f v5=%.2f v33=%.2f "
               "t=%.1f h=%.0f p=%.1f tvoc=%u eco2=%u pm25=%.1f pm10=%.1f lux=%.0f "
               "tb=%.1f tr=%.1f tf=%.1f mq2=%.3f st=%u ok=0x%02X "
               "err=%u,%u,%u,%u,%u,%u\n",
               g_sensors.bus_v, g_sensors.current_a,
               g_sensors.v_12v, g_sensors.v_5v, g_sensors.v_3v3,
               g_sensors.temp_c, g_sensors.humidity_pct, g_sensors.pressure_hpa,
               g_sensors.tvoc_ppb, g_sensors.eco2_ppm,
               g_sensors.pm2_5, g_sensors.pm10, g_sensors.lux,
               g_sensors.temp_batt_c, g_sensors.temp_elec_rear_c, g_sensors.temp_elec_front_c,
               g_sensors.mq2_v, g_sensors.mq2_status, g_sensors.ok,
               g_sensors.err[0], g_sensors.err[1], g_sensors.err[2],
               g_sensors.err[3], g_sensors.err[4], g_sensors.err[5]);
        fflush(stdout);
    }
}

void task_telemetry_start(void)
{
    xTaskCreate(telemetry_task, "telemetry", 4096, NULL, 2, NULL);
}
