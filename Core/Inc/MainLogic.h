#ifndef INC_MAINLOGIC_H_
#define INC_MAINLOGIC_H_

// PRIVATE INCLUDE ================================================================================

#include <I2C_EndEffector.h>
#include "i2c.h"

// PRIVATE TYPEDEF ================================================================================

// PRIVATE VARIABLE ===============================================================================

// PRIVATE FUNCTION PROTOTYPE =====================================================================

void main_logic(MB *variables);
void end_effector_gripper(MB *variables, uint8_t mode);	// 0 pick, 1 place
void end_effector_laser(MB *variables, uint8_t mode);	// 0 off, 1 on

// USER CODE ======================================================================================

void main_logic(MB *variables) {
	I2C_TO_BASESYSTEM(&variables->end_effector_status, &hi2c1);
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

// USER CODE END ==================================================================================

#endif /* INC_MAINLOGIC_H_ */
