/*
 * robot_drive.c
 *
 *  Created on: Mar 26, 2026
 *      Author: aalhamad
 */


#include "robot_drive.h"

HAL_StatusTypeDef robot_set_rpm(int32_t left_rpm, int32_t right_rpm)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (vesc_set_rpm(VESC_FL,  left_rpm)  != HAL_OK) status = HAL_ERROR;
    if (vesc_set_rpm(VESC_RL,  left_rpm)  != HAL_OK) status = HAL_ERROR;
    if (vesc_set_rpm(VESC_FR, -right_rpm) != HAL_OK) status = HAL_ERROR;
    if (vesc_set_rpm(VESC_RR, -right_rpm) != HAL_OK) status = HAL_ERROR;

    return status;
}
