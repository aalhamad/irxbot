/*
 * vesc_can.h
 *
 *  Created on: Mar 24, 2026
 *      Author: aalhamad
 */

#ifndef INC_VESC_CAN_H_
#define INC_VESC_CAN_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

HAL_StatusTypeDef vesc_set_rpm(uint8_t id, int32_t rpm);

#endif /* INC_VESC_CAN_H_ */
