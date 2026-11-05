#include "task_alarm.h"

#include "driver/gpio.h"

#define BUZZER_GPIO    GPIO_NUM_21   /* MOSFET, active HIGH */

/* Holds the buzzer off. Its only alarm source, the MQ-2, was removed from the
   robot; give it a new one here when needed. */
void task_alarm_start(void)
{
    gpio_set_level(BUZZER_GPIO, 0);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER_GPIO, 0);
}
