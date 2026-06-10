#include "robot_drive.h"

HAL_StatusTypeDef robot_set_rpm(int32_t left_rpm, int32_t right_rpm)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (vesc_set_rpm(1,  left_rpm)  != HAL_OK) status = HAL_ERROR;
    HAL_Delay(2);

    if (vesc_set_rpm(2, -right_rpm) != HAL_OK) status = HAL_ERROR;
    HAL_Delay(2);

    if (vesc_set_rpm(3,  left_rpm)  != HAL_OK) status = HAL_ERROR;
    HAL_Delay(2);

    if (vesc_set_rpm(4, -right_rpm) != HAL_OK) status = HAL_ERROR;

    return status;
}
