#include "task_uart.h"
#include "task_lighting.h"
#include "task_sensors.h"
#include "task_telemetry.h"

void app_main(void)
{
    task_uart_start();
    task_lighting_start();
    task_sensors_start();
    task_telemetry_start();
}
