#include "robot_drive.h"
#include "main.h"

extern UART_HandleTypeDef huart2;

HAL_StatusTypeDef robot_set_rpm(int32_t left_rpm, int32_t right_rpm)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Each wheel is independent — a single TX failure does not stop the others. */
    if (vesc_set_rpm(VESC_FL,  left_rpm)  != HAL_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"ERR:VESC1\r\n", 11, 50);
        status = HAL_ERROR;
    }
    HAL_Delay(2);

    if (vesc_set_rpm(VESC_FR,  right_rpm) != HAL_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"ERR:VESC2\r\n", 11, 50);
        status = HAL_ERROR;
    }
    HAL_Delay(2);

    if (vesc_set_rpm(VESC_RL,  left_rpm)  != HAL_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"ERR:VESC3\r\n", 11, 50);
        status = HAL_ERROR;
    }
    HAL_Delay(2);

    if (vesc_set_rpm(VESC_RR,  right_rpm) != HAL_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"ERR:VESC4\r\n", 11, 50);
        status = HAL_ERROR;
    }

    return status;
}
