/*
 * robot_drive.h
 *
 *  Created on: Mar 26, 2026
 *      Author: aalhamad
 */

#ifndef ROBOT_DRIVE_H
#define ROBOT_DRIVE_H

#include "vesc_can.h"

// Wheel IDs
#define VESC_FL  1  // Front Left
#define VESC_FR  2  // Front Right
#define VESC_RL  3  // Rear Left
#define VESC_RR  4  // Rear Right

HAL_StatusTypeDef robot_set_rpm(int32_t left_rpm, int32_t right_rpm);

#endif
