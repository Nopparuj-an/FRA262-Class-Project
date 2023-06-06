#ifndef INC_MAINLOGIC_H_
#define INC_MAINLOGIC_H_

// PRIVATE INCLUDE ================================================================================

#include <I2C_EndEffector.h>
#include "i2c.h"

// PRIVATE TYPEDEF ================================================================================

typedef enum {
	MSidle, MSpick, MSplace, MShome, MSrun, MSpoint
} MachineState;

// PRIVATE VARIABLE ===============================================================================

MachineState state = MSidle;

uint8_t PID_enable = 1;
uint8_t home_status = 0;

float voltage;
extern int32_t setpoint;
extern int32_t setpointtraj;
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

// USER CODE ======================================================================================

void main_logic(MB *variables) {
	I2C_TO_BASESYSTEM(&variables->end_effector_status, &hi2c1);
	RGB_logic();
	data_report(variables);

	switch (state) {
	case MSidle:
		variables->y_moving_status = 0;

		if (variables->base_system_status & 0b100) {
			// home mode
			variables->base_system_status = 0;
			state = MShome;
			variables->y_moving_status = 4;
		}
		break;
	case MSpick:
		break;
	case MSplace:
		break;
	case MShome:
		if (!home_status) {
			home_status = 1;
			PID_enable = 0;
			voltage = -8000;
		}
		break;
	case MSrun:
		break;
	case MSpoint:
		break;
	}
}

void interrupt_logic() {
	// Call trajectory function
	Trajectory(setpoint, 34000, 80000, (int*) &setpointtraj, (float*) &traj_velocity, (float*) &traj_acceleration, 0);

	// Call PID function
	if (PID_enable) {
		static int count = 0;
		count++;
		if (count >= 5) {
			PositionControlPID(setpointtraj, setpoint, getLocalPosition(), KP, KI, KD, &voltage);
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
	setpointtraj = -11500;
	setpoint = -11500;
	Trajectory(setpoint, 34000, 80000, (int*) &setpointtraj, (float*) &traj_velocity, (float*) &traj_acceleration, 1);
	home_status = 0;
	PID_enable = 1;
	state = MSidle;
	setpoint = 0;
}

void data_report(MB *variables){
	variables->y_actual_position = getLocalPosition() * 0.3;
	variables->y_actual_speed = traj_velocity * 0.3;
	variables->y_actual_acceleration = traj_acceleration * 0.3;
}

// USER CODE END ==================================================================================

#endif /* INC_MAINLOGIC_H_ */
