#ifndef INC_MAINLOGIC_H_
#define INC_MAINLOGIC_H_

// PRIVATE INCLUDE ================================================================================

#include <I2C_EndEffector.h>
#include "i2c.h"

// PRIVATE TYPEDEF ================================================================================

// PRIVATE VARIABLE ===============================================================================

uint8_t PID_enable = 1;
float voltage;
extern int32_t setpoint;
extern int32_t setpointtraj;
int32_t traj_velocity;
int32_t traj_acceleration;
extern float KP;
extern float KI;
extern float KD;

uint8_t runonce = 0;

// PRIVATE FUNCTION PROTOTYPE =====================================================================

void main_logic(MB *variables);
void interrupt_logic();
void end_effector_gripper(MB *variables, uint8_t mode);	// 0 pick, 1 place
void end_effector_laser(MB *variables, uint8_t mode);	// 0 off, 1 on
void home();

// USER CODE ======================================================================================

void main_logic(MB *variables) {
	I2C_TO_BASESYSTEM(&variables->end_effector_status, &hi2c1);

	if (runonce) {
		home();
		runonce = 0;
	}
}

void interrupt_logic() {
	// Call trajectory function
	Trajectory(setpoint, 34000, 80000, &setpointtraj, &traj_velocity, &traj_acceleration, 0);

	// Call PID function
	if (PID_enable) {
		static int count = 0;
		count++;
		if (count >= 5) {
			PositionControlPID(setpointtraj, getLocalPosition(), KP, KI, KD, &voltage);
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

void home() {
	homeoffset = getRawPosition();
	setpointtraj = 0;
	setpoint = 0;
	Trajectory(0, 34000, 80000, &setpointtraj, &traj_velocity, &traj_acceleration, 1);
}

// USER CODE END ==================================================================================

#endif /* INC_MAINLOGIC_H_ */
