#ifndef INC_MAINLOGIC_H_
#define INC_MAINLOGIC_H_

// PRIVATE INCLUDE ================================================================================

// PRIVATE TYPEDEF ================================================================================

// PRIVATE VARIABLE ===============================================================================

// PRIVATE FUNCTION PROTOTYPE =====================================================================

void main_logic(MB variables);
void end_effector_handler();
void end_effector_gripper(MB variables, uint8_t mode);	// 0 pick, 1 place

// USER CODE ======================================================================================

void main_logic(MB variables) {
	end_effector_handler(variables);
}

void end_effector_handler(MB variables) {
	// handle base system turning on/off laser
	if (variables.end_effector_status & 0b0001) {
		// laser on
		// TODO insert laser on command here
	} else {
		// laser off
		// TODO insert laser off command here
	}

	// handle base system pick/place
	if (variables.end_effector_status & 0b0100) {
		// gripper pick
		end_effector_gripper(variables, 0);
	} else if (variables.end_effector_status & 0b1000) {
		// gripper place
		end_effector_gripper(variables, 1);
	}
}

void end_effector_gripper(MB variables, uint8_t mode) {
	if ((variables.end_effector_status & 0b0010) == 0) {
		return;
	}

	if (!mode) {
		// pick
		variables.end_effector_status |= 0b0100;
		// TODO insert I2C pick command here
		variables.end_effector_status &= 0b1011;
	} else {
		// place
		variables.end_effector_status |= 0b1000;
		// TODO insert I2C place command here
		variables.end_effector_status &= 0b0111;
	}
}

// USER CODE END ==================================================================================

#endif /* INC_MAINLOGIC_H_ */
