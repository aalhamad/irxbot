#include "task_uart.h"

#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define TAG      "uart"
#define BUF_SIZE 256

QueueHandle_t g_light_queue;
QueueHandle_t g_stm32_queue;

/* Parse: LIGHT <target> <R> <G> <B> <mode>\n */
static void parse_light(const char *args)
{
    char   target_s[8], mode_s[8];
    int    r, g, b;

    if (sscanf(args, "%7s %d %d %d %7s", target_s, &r, &g, &b, mode_s) != 5)
        return;

    light_cmd_t cmd;
    cmd.r = (uint8_t)r;
    cmd.g = (uint8_t)g;
    cmd.b = (uint8_t)b;

    if      (strcmp(target_s, "ALL")   == 0) cmd.target = LIGHT_TARGET_ALL;
    else if (strcmp(target_s, "LEFT")  == 0) cmd.target = LIGHT_TARGET_LEFT;
    else if (strcmp(target_s, "RIGHT") == 0) cmd.target = LIGHT_TARGET_RIGHT;
    else if (strcmp(target_s, "FRONT") == 0) cmd.target = LIGHT_TARGET_FRONT;
    else if (strcmp(target_s, "REAR")  == 0) cmd.target = LIGHT_TARGET_REAR;
    else if (strcmp(target_s, "SIDE")  == 0) cmd.target = LIGHT_TARGET_SIDE;
    else return;

    if      (strcmp(mode_s, "SOLID") == 0) cmd.mode = LIGHT_MODE_SOLID;
    else if (strcmp(mode_s, "BLINK") == 0) cmd.mode = LIGHT_MODE_BLINK;
    else if (strcmp(mode_s, "PULSE") == 0) cmd.mode = LIGHT_MODE_PULSE;
    else return;

    xQueueSend(g_light_queue, &cmd, 0);
}

static void parse_line(const char *line)
{
    if (strncmp(line, "LIGHT ", 6) == 0) {
        parse_light(line + 6);
    }
    else if (strncmp(line, "SPEED ", 6) == 0) {
        stm32_event_t ev = { .type = SPEED_UPDATE, .value = atoi(line + 6) };
        xQueueSend(g_stm32_queue, &ev, 0);
    }
    else if (strncmp(line, "BMP ", 4) == 0) {
        stm32_event_t ev = { .type = BMP_UPDATE, .value = atoi(line + 4) };
        xQueueSend(g_stm32_queue, &ev, 0);
    }
}

static void uart_task(void *arg)
{
    uint8_t buf[BUF_SIZE];
    char    line[BUF_SIZE];
    int     line_len = 0;

    while (1) {
        int n = uart_read_bytes(STM32_UART_NUM, buf, sizeof(buf) - 1,
                                pdMS_TO_TICKS(20));
        for (int i = 0; i < n; i++) {
            char c = (char)buf[i];
            if (c == '\n' || c == '\r') {
                if (line_len > 0) {
                    line[line_len] = '\0';
                    ESP_LOGD(TAG, "rx: %s", line);
                    parse_line(line);
                    line_len = 0;
                }
            } else if (line_len < (int)sizeof(line) - 1) {
                line[line_len++] = c;
            }
        }
    }
}

void task_uart_start(void)
{
    g_light_queue = xQueueCreate(8,  sizeof(light_cmd_t));
    g_stm32_queue = xQueueCreate(16, sizeof(stm32_event_t));

    uart_config_t cfg = {
        .baud_rate  = STM32_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(STM32_UART_NUM, &cfg);
    uart_set_pin(STM32_UART_NUM, STM32_TX_PIN, STM32_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(STM32_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    xTaskCreate(uart_task, "uart", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "started on UART%d rx=%d tx=%d",
             STM32_UART_NUM, STM32_RX_PIN, STM32_TX_PIN);
}
