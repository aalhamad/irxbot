#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/* GPIO pins for STM32 link */
#define STM32_UART_NUM   UART_NUM_1
#define STM32_RX_PIN     35
#define STM32_TX_PIN     36
#define STM32_BAUD       115200

/* ── LIGHT command ──────────────────────────────────────────────────────────── */

typedef enum {
    LIGHT_TARGET_ALL,
    LIGHT_TARGET_LEFT,
    LIGHT_TARGET_RIGHT,
    LIGHT_TARGET_FRONT,
    LIGHT_TARGET_REAR,
    LIGHT_TARGET_SIDE,
} light_target_t;

typedef enum {
    LIGHT_MODE_SOLID,
    LIGHT_MODE_BLINK,
    LIGHT_MODE_PULSE,
} light_mode_t;

typedef struct {
    light_target_t target;
    uint8_t        r, g, b;
    light_mode_t   mode;
} light_cmd_t;

/* ── General STM32 events (SPEED, BMP) ─────────────────────────────────────── */

typedef enum {
    SPEED_UPDATE,
    BMP_UPDATE,
} stm32_event_type_t;

typedef struct {
    stm32_event_type_t type;
    int32_t value;   /* SPEED: rpm; BMP: bitmask FL=1 FR=2 RL=4 RR=8 */
} stm32_event_t;

/* g_light_queue  — task_lighting reads (light_cmd_t)   */
/* g_stm32_queue  — other tasks read  (stm32_event_t)   */
extern QueueHandle_t g_light_queue;
extern QueueHandle_t g_stm32_queue;

void task_uart_start(void);
