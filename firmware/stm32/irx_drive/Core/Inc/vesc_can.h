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
HAL_StatusTypeDef vesc_get_rpm(uint8_t id, int32_t *rpm);
void              vesc_process_rx(void);
uint32_t          vesc_last_seen(uint8_t id);
void              vesc_can_dump_status(void);

#endif /* INC_VESC_CAN_H_ */
