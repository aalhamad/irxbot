#pragma once

/* Front amber turn signals (digital) */
#define PIN_AMBER_L   15
#define PIN_AMBER_R   16

/* Rear red turn/brake (PWM via LEDC) */
#define PIN_RED_L     17
#define PIN_RED_R     18
#define PWM_TAIL      64    /* dim tail brightness  */
#define PWM_BRAKE     255   /* full brake brightness */

/* WS2812B side strips */
#define PIN_STRIP_L   38
#define PIN_STRIP_R   39
#define STRIP_COUNT   23

/* Spotlight */
#define PIN_SPOTLIGHT 14

/* Blink period in ms (standard ~90 blinks/min) */
#define BLINK_MS      330

void task_lighting_start(void);
