#ifndef INC_MAINLOGIC_H_
#define INC_MAINLOGIC_H_

// PRIVATE INCLUDE ================================================================================

// PRIVATE TYPEDEF ================================================================================

// PRIVATE FUNCTION PROTOTYPE =====================================================================

void main_logic(MB variables);
void end_effector_handler();
//void end_effector(MB variables);

// USER CODE ======================================================================================

void main_logic(MB variables){
//	end_effector(variables);
}

void end_effector_handler(MB variables){
	if(variables.end_effector_status & 0b0001){
	}
}

// USER CODE END ==================================================================================

#endif /* INC_MAINLOGIC_H_ */
