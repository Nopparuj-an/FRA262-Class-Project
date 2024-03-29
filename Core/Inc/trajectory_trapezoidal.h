/*
 * trajectory_trapezodal.h
 *
 *  Created on: Jun 3, 2023
 *      Author: user
 */

#ifndef INC_TRAJECTORY_TRAPEZOIDAL_H_
#define INC_TRAJECTORY_TRAPEZOIDAL_H_

// PRIVATE INCLUDE ================================================================================

//#include "stm32f4xx_hal.h"

// PRIVATE FUNCTION PROTOTYPE =====================================================================

void Trajectory(float setpoint_now,float velocity_max,float acceleration_max, int *position_out, float *velocity_out, float *acceleration_out, int homemode);

// USER CODE END ==================================================================================

#endif /* INC_TRAJECTORY_TRAPEZOIDAL_H_ */
