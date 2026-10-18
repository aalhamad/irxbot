	/*
	 * robot_drive.h
	 *
	 *  Created on: Mar 26, 2026
	 *      Author: aalhamad
	 */

	#ifndef ROBOT_DRIVE_H
	#define ROBOT_DRIVE_H

	#include "vesc_can.h"

	// Wheel IDs — matched to physical VESC ID assignments
	#define VESC_FR  1  // Front Right
	#define VESC_FL  2  // Front Left
	#define VESC_RR  3  // Rear Right
	#define VESC_RL  4  // Rear Left

	HAL_StatusTypeDef robot_set_rpm(int32_t left_rpm, int32_t right_rpm);

	#endif
