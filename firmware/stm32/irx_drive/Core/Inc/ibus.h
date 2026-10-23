/*
 * ibus.h — FlySky iBUS receiver driver for IRXBot.
 * 115200 8N1, 32-byte packets, 14 channels (1000–2000).
 */

#ifndef IBUS_H
#define IBUS_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define IBUS_NUM_CHANNELS  14
#define IBUS_PACKET_LEN    32

/* CH5 > this threshold = RC override active */
#define IBUS_OVERRIDE_CH   4        /* zero-based index: channel 5 */
#define IBUS_OVERRIDE_THR  1700

/* Signal loss declared after this many ms without a valid packet */
#define IBUS_TIMEOUT_MS    500

typedef struct {
    uint16_t channel[IBUS_NUM_CHANNELS]; /* 1000 = min, 1500 = centre, 2000 = max */
    uint8_t  valid;                       /* 1 = signal present */
} IBus_Data_t;

void        IBus_Init(UART_HandleTypeDef *huart);
void        IBus_RxByte(uint8_t b);   /* call from USART1_IRQHandler */
void        IBus_Process(void);
IBus_Data_t* IBus_Get(void);

#endif /* IBUS_H */
