/*
 * vesc_can.c
 *
 *  Created on: Mar 24, 2026
 *      Author: aalhamad
 */


#include "vesc_can.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;

#include "vesc_can.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;

#define CAN_PACKET_SET_DUTY          0
#define CAN_PACKET_SET_CURRENT       1
#define CAN_PACKET_SET_CURRENT_BRAKE 2
#define CAN_PACKET_SET_RPM           3
#define CAN_PACKET_SET_POS           4

HAL_StatusTypeDef vesc_set_rpm(uint8_t id, int32_t rpm)
{
    // Wait for free mailbox — inside the function
    uint32_t timeout = HAL_GetTick() + 10; // 10ms timeout
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
    {
        if (HAL_GetTick() > timeout) return HAL_TIMEOUT;
    }

    CAN_TxHeaderTypeDef txHeader;
    uint8_t data[4] = {0};
    uint32_t mailbox;

    data[0] = (rpm >> 24) & 0xFF;
    data[1] = (rpm >> 16) & 0xFF;
    data[2] = (rpm >> 8)  & 0xFF;
    data[3] =  rpm        & 0xFF;

    txHeader.IDE = CAN_ID_EXT;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 4;
    txHeader.TransmitGlobalTime = DISABLE;
    txHeader.ExtId = ((uint32_t)CAN_PACKET_SET_RPM << 8) | id;

    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &mailbox);
}
