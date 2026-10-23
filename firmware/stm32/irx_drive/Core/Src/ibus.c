/*
 * ibus.c — FlySky iBUS receiver driver for IRXBot.
 *
 * Reception is interrupt-driven: IBus_RxByte() is called from
 * USART1_IRQHandler and stores bytes in a ring buffer. IBus_Process()
 * drains the ring buffer and parses complete packets.
 *
 * Packet format (32 bytes):
 *   [0]     0x20  — length
 *   [1]     0x40  — command
 *   [2..29] 14 × 2 bytes little-endian channel values (1000–2000)
 *   [30..31] checksum = 0xFFFF − sum(bytes 0..29), little-endian
 */

#include "ibus.h"
#include <string.h>

#define RING_SIZE  64   /* must be power of 2 */

static volatile uint8_t  _ring[RING_SIZE];
static volatile uint8_t  _ring_head = 0;   /* written by ISR */
static volatile uint8_t  _ring_tail = 0;   /* read by main loop */

static uint8_t     _buf[IBUS_PACKET_LEN];
static uint8_t     _idx      = 0;
static IBus_Data_t _data;
static uint32_t    _last_pkt = 0;

/* Called from USART1_IRQHandler — stores one byte to ring buffer */
void IBus_RxByte(uint8_t b)
{
    uint8_t next = (_ring_head + 1) & (RING_SIZE - 1);
    if (next != _ring_tail)   /* drop if full */
    {
        _ring[_ring_head] = b;
        _ring_head = next;
    }
}

void IBus_Init(UART_HandleTypeDef *huart)
{
    memset(&_data, 0, sizeof(_data));
    for (int i = 0; i < IBUS_NUM_CHANNELS; i++)
        _data.channel[i] = 1500;

    /* Enable USART1 RXNE interrupt */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
}

void IBus_Process(void)
{
    /* Drain ring buffer */
    while (_ring_tail != _ring_head)
    {
        uint8_t b = _ring[_ring_tail];
        _ring_tail = (_ring_tail + 1) & (RING_SIZE - 1);

        if (_idx == 0)
        {
            if (b == 0x20) _buf[_idx++] = b;
            continue;
        }
        if (_idx == 1 && b != 0x40) { _idx = 0; continue; }

        _buf[_idx++] = b;

        if (_idx == IBUS_PACKET_LEN)
        {
            _idx = 0;
            uint16_t sum = 0xFFFF;
            for (int i = 0; i < 30; i++) sum -= _buf[i];
            uint16_t chk = (uint16_t)_buf[30] | ((uint16_t)_buf[31] << 8);
            if (sum == chk)
            {
                for (int i = 0; i < IBUS_NUM_CHANNELS; i++)
                    _data.channel[i] = (uint16_t)_buf[2 + i * 2]
                                     | ((uint16_t)_buf[3 + i * 2] << 8);
                _data.valid  = 1;
                _last_pkt    = HAL_GetTick();
            }
        }
    }

    /* Signal loss: no valid packet for IBUS_TIMEOUT_MS */
    if (_data.valid && (HAL_GetTick() - _last_pkt) > IBUS_TIMEOUT_MS)
        _data.valid = 0;
}

IBus_Data_t* IBus_Get(void)
{
    return &_data;
}
