#ifndef INC_MAINLOGIC_H_
#define INC_MAINLOGIC_H_

// PRIVATE INCLUDE ================================================================================

#include <I2C_EndEffector.h>
#include "i2c.h"

// PRIVATE TYPEDEF ================================================================================

typedef enum {
	MSwait, MSidle, MSpick, MSplace, MShome, MSrun, MSpoint
} MachineState;

// PRIVATE VARIABLE ===============================================================================

MachineState state = MSidle;

uint8_t PID_enable = 1;
uint8_t home_status = 0;
uint8_t jog_enable = 0;

extern int receivedByte[4];

float voltage;
extern int32_t setpoint_x;
extern int32_t setpoint_y;
extern int32_t setpointtraj_y;
int32_t traj_velocity;
int32_t traj_acceleration;
extern float KP;
extern float KI;
extern float KD;

// PRIVATE FUNCTION PROTOTYPE =====================================================================

void main_logic(MB *variables);
void interrupt_logic();
void end_effector_gripper(MB *variables, uint8_t mode);	// 0 pick, 1 place
void end_effector_laser(MB *variables, uint8_t mode);	// 0 off, 1 on
void home_handler();
void data_report(MB *variables);
void x_spam_position(MB *variables);
void joystick_callback();

// USER CODE ======================================================================================

void main_logic(MB *variables) {
	I2C_TO_BASESYSTEM(&variables->end_effector_status, &hi2c1);
	RGB_logic();
	data_report(variables);

	static uint32_t wait_timer;
	switch (state) {
	case MSwait:
		if (HAL_GetTick() - wait_timer > 1500) {
			state = MSidle;
		}
		break;
	case MSidle:
		wait_timer = HAL_GetTick();
		variables->y_moving_status = 0;
		jog_enable = 0;

		if (variables->base_system_status & 0b100) {
			// home mode
			variables->base_system_status = 0;
			state = MShome;
			variables->y_moving_status = 4;
//			variables->x_moving_status = 1;
		}

		if (variables->base_system_status & 0b10000) {
			// point mode
			variables->base_system_status = 0;
			state = MSpoint;
			variables->y_moving_status = 32;
		}

		if (variables->base_system_status & 0b1) {
			// pick mode
			variables->base_system_status = 0;
			state = MSpick;
			variables->y_moving_status = 8;
			jog_enable = 1;
		}

		if (variables->base_system_status & 0b10) {
			// place mode
			variables->base_system_status = 0;
			state = MSplace;
			variables->y_moving_status = 16;
			jog_enable = 1;
		}
		break;
	case MSpick:
		variables->x_target_position = setpoint_x;
		x_spam_position(variables);
		break;
	case MSplace:
		variables->x_target_position = setpoint_x;
		x_spam_position(variables);
		break;
	case MShome:
		if (!home_status) {
			home_status = 1;
			PID_enable = 0;
			voltage = -13000;
		}
		break;
	case MSrun:
		break;
	case MSpoint:
		setpoint_y = variables->goal_point_y / 0.3;
		variables->x_target_position = variables->goal_point_x;
		variables->x_moving_status = 2;

		if (setpoint_y == getLocalPosition()) {
			state = MSwait;
		}
		break;
	}
}

void interrupt_logic() {
	// Call trajectory function
	Trajectory(setpoint_y, 34000, 80000, (int*) &setpointtraj_y, (float*) &traj_velocity, (float*) &traj_acceleration, 0);

	// Call PID function
	if (PID_enable) {
		static int count = 0;
		count++;
		if (count >= 5) {
			PositionControlPID(setpointtraj_y, setpoint_y, getLocalPosition(), KP, KI, KD, &voltage);
			count = 0;
		}
	}

	// Call motor function
	motor(voltage);
}

void end_effector_gripper(MB *variables, uint8_t mode) {
	if ((variables->end_effector_status & 0b0010) == 0) {
		return;
	}

	if (!mode) {
		// pick
		variables->end_effector_status |= 0b0100;
	} else {
		// place
		variables->end_effector_status |= 0b1000;
	}
}

void end_effector_laser(MB *variables, uint8_t mode) {
	if (!mode) {
		// off
		variables->end_effector_status &= 0b1110;
	} else {
		// on
		variables->end_effector_status |= 0b0001;
	}
}

void home_handler() {
	if (!home_status) {
		return;
	}
	motor(0);
	homeoffset = getRawPosition() + 11500;
	setpointtraj_y = -11500;
	setpoint_y = -11500;
	Trajectory(setpoint_y, 34000, 80000, (int*) &setpointtraj_y, (float*) &traj_velocity, (float*) &traj_acceleration, 1);
	home_status = 0;
	PID_enable = 1;
	state = MSwait;
	setpoint_y = 0;
}

void data_report(MB *variables) {
	variables->y_actual_position = getLocalPosition() * 0.3;
	variables->y_actual_speed = traj_velocity * 0.3;
	variables->y_actual_acceleration = traj_acceleration * 0.3;
}

void x_spam_position(MB *variables) {
	if ((variables->x_actual_position - variables->x_target_position) != 0 && variables->x_moving_status == 0) {
		variables->x_moving_status = 2;
	}
}

void joystick_callback() {
	if (!jog_enable) {
		return;
	}

	setpoint_x += receivedByte[0];
	setpoint_y += receivedByte[1];

	if (setpoint_x > 1400) {
		setpoint_x = 1400;
	} else if (setpoint_x < -1400) {
		setpoint_x = -1400;
	}

	if (setpoint_y > 11667) {
		setpoint_y = 11667;
	} else if (setpoint_y < -11667) {
		setpoint_y = -11667;
	}
}

// USER CODE END ==================================================================================

#endif /* INC_MAINLOGIC_H_ */
