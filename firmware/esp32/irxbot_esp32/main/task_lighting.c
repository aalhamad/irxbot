#include "task_lighting.h"
#include "task_uart.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "led_strip.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#define TAG "lighting"

static led_strip_handle_t _strip_l;
static led_strip_handle_t _strip_r;

/* Current command */
static light_cmd_t _cmd = {
    .target = LIGHT_TARGET_ALL,
    .r = 0, .g = 0, .b = 0,
    .mode = LIGHT_MODE_SOLID,
};

/* ── Output helpers ─────────────────────────────────────────────────────────── */

static void set_amber(bool left, bool right)
{
    gpio_set_level(PIN_AMBER_L, left  ? 1 : 0);
    gpio_set_level(PIN_AMBER_R, right ? 1 : 0);
}

static void set_rear(uint8_t left, uint8_t right)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, left);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, right);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

static void set_strip(led_strip_handle_t strip, uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < STRIP_COUNT; i++)
        led_strip_set_pixel(strip, i, r, g, b);
    led_strip_refresh(strip);
}

/* ── Apply scaled color to each group based on target ───────────────────────── */

static void apply_color(uint8_t r, uint8_t g, uint8_t b)
{
    light_target_t t = _cmd.target;
    bool do_left  = (t == LIGHT_TARGET_ALL || t == LIGHT_TARGET_LEFT);
    bool do_right = (t == LIGHT_TARGET_ALL || t == LIGHT_TARGET_RIGHT);
    bool do_front = (t == LIGHT_TARGET_ALL || t == LIGHT_TARGET_FRONT ||
                     t == LIGHT_TARGET_LEFT || t == LIGHT_TARGET_RIGHT);
    bool do_rear  = (t == LIGHT_TARGET_ALL || t == LIGHT_TARGET_REAR  ||
                     t == LIGHT_TARGET_LEFT || t == LIGHT_TARGET_RIGHT);
    bool do_side  = (t == LIGHT_TARGET_ALL || t == LIGHT_TARGET_SIDE  ||
                     t == LIGHT_TARGET_LEFT || t == LIGHT_TARGET_RIGHT);

    /* Front ambers — use any non-zero color as on, scale brightness to max channel */
    uint8_t brightness = r > g ? (r > b ? r : b) : (g > b ? g : b);
    if (do_front) {
        set_amber(do_left  ? (brightness > 0) : false,
                  do_right ? (brightness > 0) : false);
    } else {
        set_amber(false, false);
    }

    /* Rear red — map brightness to PWM_BRAKE scale */
    uint8_t rear_val = (uint8_t)((brightness * PWM_BRAKE) / 255);
    if (do_rear) {
        set_rear(do_left  ? rear_val : 0,
                 do_right ? rear_val : 0);
    } else {
        set_rear(0, 0);
    }

    /* Side WS2812B strips */
    if (do_side) {
        if (do_left)  set_strip(_strip_l, r, g, b);
        else          set_strip(_strip_l, 0, 0, 0);
        if (do_right) set_strip(_strip_r, r, g, b);
        else          set_strip(_strip_r, 0, 0, 0);
    } else {
        set_strip(_strip_l, 0, 0, 0);
        set_strip(_strip_r, 0, 0, 0);
    }
}

/* ── Task ───────────────────────────────────────────────────────────────────── */

static void lighting_task(void *arg)
{
    uint32_t tick       = xTaskGetTickCount();
    uint32_t blink_tick = tick;
    bool     blink_on   = true;
    float    pulse_phase = 0.0f;

    while (1) {
        light_cmd_t incoming;
        if (xQueueReceive(g_light_queue, &incoming, pdMS_TO_TICKS(20)) == pdTRUE) {
            _cmd      = incoming;
            blink_on  = true;
            blink_tick = xTaskGetTickCount();
            pulse_phase = 0.0f;
            ESP_LOGI(TAG, "cmd: target=%d r=%d g=%d b=%d mode=%d",
                     _cmd.target, _cmd.r, _cmd.g, _cmd.b, _cmd.mode);
        }

        tick = xTaskGetTickCount();

        switch (_cmd.mode) {
            case LIGHT_MODE_SOLID:
                apply_color(_cmd.r, _cmd.g, _cmd.b);
                vTaskDelay(pdMS_TO_TICKS(50));
                break;

            case LIGHT_MODE_BLINK:
                if ((tick - blink_tick) >= pdMS_TO_TICKS(BLINK_MS)) {
                    blink_tick = tick;
                    blink_on   = !blink_on;
                }
                apply_color(blink_on ? _cmd.r : 0,
                            blink_on ? _cmd.g : 0,
                            blink_on ? _cmd.b : 0);
                vTaskDelay(pdMS_TO_TICKS(20));
                break;

            case LIGHT_MODE_PULSE: {
                pulse_phase += 0.05f;
                if (pulse_phase > 2.0f * 3.14159f) pulse_phase -= 2.0f * 3.14159f;
                float scale = (sinf(pulse_phase) + 1.0f) / 2.0f;
                apply_color((uint8_t)(_cmd.r * scale),
                            (uint8_t)(_cmd.g * scale),
                            (uint8_t)(_cmd.b * scale));
                vTaskDelay(pdMS_TO_TICKS(20));
                break;
            }
        }
    }
}

/* ── Init ───────────────────────────────────────────────────────────────────── */

static void init_gpio(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_AMBER_L) | (1ULL << PIN_AMBER_R) |
                        (1ULL << PIN_SPOTLIGHT),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
}

static void init_ledc(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz         = 5000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .duty       = 0,
        .hpoint     = 0,
    };
    ch.channel = LEDC_CHANNEL_0; ch.gpio_num = PIN_RED_L; ledc_channel_config(&ch);
    ch.channel = LEDC_CHANNEL_1; ch.gpio_num = PIN_RED_R; ledc_channel_config(&ch);
}

static void init_strips(void)
{
    led_strip_config_t cfg = {
        .strip_gpio_num         = PIN_STRIP_L,
        .max_leds               = STRIP_COUNT,
        .led_model              = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rmt = {
        .resolution_hz = 10 * 1000 * 1000,
    };
    led_strip_new_rmt_device(&cfg, &rmt, &_strip_l);

    cfg.strip_gpio_num = PIN_STRIP_R;
    led_strip_new_rmt_device(&cfg, &rmt, &_strip_r);
}

void task_lighting_start(void)
{
    init_gpio();
    init_ledc();
    init_strips();

    xTaskCreate(lighting_task, "lighting", 4096, NULL, 4, NULL);
    ESP_LOGI(TAG, "started");
}
