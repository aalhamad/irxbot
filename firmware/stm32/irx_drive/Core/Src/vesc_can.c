#include "vesc_can.h"
#include "main.h"
#include <string.h>

extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart2;

#define CAN_PACKET_SET_DUTY          0
#define CAN_PACKET_SET_CURRENT       1
#define CAN_PACKET_SET_CURRENT_BRAKE 2
#define CAN_PACKET_SET_RPM           3
#define CAN_PACKET_SET_POS           4

HAL_StatusTypeDef vesc_set_rpm(uint8_t id, int32_t rpm)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t mailbox;
    uint8_t data[4];

    data[0] = (rpm >> 24) & 0xFF;
    data[1] = (rpm >> 16) & 0xFF;
    data[2] = (rpm >> 8)  & 0xFF;
    data[3] =  rpm        & 0xFF;

    txHeader.IDE = CAN_ID_EXT;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 4;
    txHeader.TransmitGlobalTime = DISABLE;
    txHeader.ExtId = ((uint32_t)CAN_PACKET_SET_RPM << 8) | id;

    HAL_StatusTypeDef ret =
        HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &mailbox);

    if (ret != HAL_OK)
    {
        HAL_UART_Transmit(&huart2,
                          (uint8_t*)"TX_FAIL\r\n",
                          strlen("TX_FAIL\r\n"),
                          100);
    }

    return ret;
}

HAL_StatusTypeDef vesc_get_rpm(uint8_t id, int32_t *rpm)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t data[8];

    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(&hcan1,
                                 CAN_RX_FIFO0,
                                 &rxHeader,
                                 data) == HAL_OK)
        {
            uint8_t vesc_id = rxHeader.ExtId & 0xFF;
            uint8_t pkt_id  = (rxHeader.ExtId >> 8) & 0xFF;

            if (pkt_id == 9 && vesc_id == id)
            {
                *rpm = (int32_t)(
                    ((uint32_t)data[0] << 24) |
                    ((uint32_t)data[1] << 16) |
                    ((uint32_t)data[2] << 8)  |
                    ((uint32_t)data[3])
                );

                return HAL_OK;
            }
        }
    }

    return HAL_ERROR;
}
