#include "task_alarm.h"
#include "task_sensors.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BUZZER_GPIO    GPIO_NUM_21   /* MOSFET, active HIGH */
#define BEEP_MS        500
#define MQ2_ALARM      2             /* sensor_data_t.mq2_status value */

/* Beeps on and off while the MQ-2 reports alarm, silent otherwise. */
static void alarm_task(void *arg)
{
    bool on = false;

    while (1) {
        on = (g_sensors.mq2_status == MQ2_ALARM) && !on;
        gpio_set_level(BUZZER_GPIO, on);
        vTaskDelay(pdMS_TO_TICKS(BEEP_MS));
    }
}

void task_alarm_start(void)
{
    gpio_set_level(BUZZER_GPIO, 0);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER_GPIO, 0);

    xTaskCreate(alarm_task, "alarm", 2048, NULL, 2, NULL);
}
