#include "vesc_can.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart2;

#define CAN_PACKET_SET_RPM    3
#define CAN_PACKET_STATUS     9

/* ── Cached VESC feedback ──────────────────────────────────────────────────── */

/* Written by CAN1_RX0_IRQHandler, read by the main loop */
static volatile int32_t  _vesc_rpm[4]       = {0};
static volatile uint32_t _vesc_last_tick[4] = {0};

/* ── Start: filter, CAN start, RX interrupt ────────────────────────────────── */

HAL_StatusTypeDef vesc_can_start(void)
{
    /* Accept only extended frames whose packet ID (bits 8-15) is STATUS, from
     * any VESC ID (bits 0-7). Status 2-4 frames would only fill the FIFO. */
    const uint32_t id   = ((uint32_t)CAN_PACKET_STATUS << 8);
    const uint32_t mask = 0x1FFFFF00U;

    CAN_FilterTypeDef f = {0};
    f.FilterBank           = 0;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = (uint16_t)((id << 3) >> 16);
    f.FilterIdLow          = (uint16_t)((id << 3) | CAN_ID_EXT);
    f.FilterMaskIdHigh     = (uint16_t)((mask << 3) >> 16);
    f.FilterMaskIdLow      = (uint16_t)((mask << 3) | CAN_ID_EXT);
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;   /* banks 0-13 belong to CAN1 */

    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK) return HAL_ERROR;
    if (HAL_CAN_Start(&hcan1) != HAL_OK)            return HAL_ERROR;

    /* Drain from the ISR: the main loop blocks on I2C and UART for longer than
     * the 3-frame FIFO lasts. */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    return HAL_OK;
}

/* ── TX ────────────────────────────────────────────────────────────────────── */

HAL_StatusTypeDef vesc_set_rpm(uint8_t id, int32_t rpm)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t mailbox;
    uint8_t data[4];

    data[0] = (rpm >> 24) & 0xFF;
    data[1] = (rpm >> 16) & 0xFF;
    data[2] = (rpm >> 8)  & 0xFF;
    data[3] =  rpm        & 0xFF;

    txHeader.IDE                = CAN_ID_EXT;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.DLC                = 4;
    txHeader.TransmitGlobalTime = DISABLE;
    txHeader.ExtId              = ((uint32_t)CAN_PACKET_SET_RPM << 8) | id;

    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &mailbox);
}

/* ── RX processing (called from CAN1_RX0_IRQHandler) ───────────────────────── */

void vesc_process_rx(void)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t data[8];

    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, data) != HAL_OK)
            break;

        if (rxHeader.IDE != CAN_ID_EXT) continue;

        uint8_t vesc_id = rxHeader.ExtId & 0xFF;
        uint8_t pkt_id  = (rxHeader.ExtId >> 8) & 0xFF;

        if (pkt_id == CAN_PACKET_STATUS && vesc_id >= 1 && vesc_id <= 4)
        {
            _vesc_rpm[vesc_id - 1] = (int32_t)(
                ((uint32_t)data[0] << 24) |
                ((uint32_t)data[1] << 16) |
                ((uint32_t)data[2] << 8)  |
                data[3]
            );
            _vesc_last_tick[vesc_id - 1] = HAL_GetTick();
        }
    }
}

/* ── Diagnostics ───────────────────────────────────────────────────────────── */

void vesc_can_dump_status(void)
{
    uint32_t esr = hcan1.Instance->ESR;
    uint32_t tsr = hcan1.Instance->TSR;
    uint32_t msr = hcan1.Instance->MSR;

    /* LEC (Last Error Code), bits 6:4 of ESR:
     *   0 = no error        1 = stuff error     2 = form error
     *   3 = ACK error       4 = bit recessive   5 = bit dominant
     *   6 = CRC error       7 = set by software                     */
    char buf[96];
    int n = snprintf(buf, sizeof(buf),
        "CAN LEC=%lu BOFF=%lu EPVF=%lu EWGF=%lu TEC=%lu REC=%lu TSR=%08lX MSR=%08lX\r\n",
        (esr >> 4) & 0x7,          /* last error code          */
        (esr >> 2) & 0x1,          /* bus-off flag             */
        (esr >> 1) & 0x1,          /* error passive flag       */
        (esr >> 0) & 0x1,          /* error warning flag       */
        (esr >> 16) & 0xFF,        /* transmit error counter   */
        (esr >> 24) & 0xFF,        /* receive error counter    */
        tsr, msr);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, n, 50);
}

/* ── Accessors ─────────────────────────────────────────────────────────────── */

HAL_StatusTypeDef vesc_get_rpm(uint8_t id, int32_t *rpm)
{
    if (id < 1 || id > 4) return HAL_ERROR;
    *rpm = _vesc_rpm[id - 1];
    return (_vesc_last_tick[id - 1] > 0) ? HAL_OK : HAL_ERROR;
}

uint32_t vesc_last_seen(uint8_t id)
{
    if (id < 1 || id > 4) return 0;
    return _vesc_last_tick[id - 1];
}
