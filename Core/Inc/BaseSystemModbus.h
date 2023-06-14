#ifndef INC_BASESYSTEMMODBUS_H_
#define INC_BASESYSTEMMODBUS_H_

// PRIVATE INCLUDE ================================================================================

#include <ModBusRTU.h>
#include "usart.h"

// PRIVATE VARIABLE ===============================================================================

ModbusHandleTypedef hmodbus;
u16u8_t MBregisterFrame[70];

// PRIVATE TYPEDEF ================================================================================

typedef struct {
	// read only variables ============================================================================
	int16_t base_system_status;		// [0 set pick, 1 set place, 2 home, 3 tray mode, 4 point mode]
	int16_t goal_point_x;			// telling our system where to go
	int16_t goal_point_y;
	int16_t x_actual_position;
	int16_t x_actual_speed;

	// write only variable ============================================================================
	int16_t y_moving_status;		// [0 jog pick, 1 jog place, 2 home, 3 go pick, 4 go place, 5 go point]
	int16_t y_actual_position;
	int16_t y_actual_speed;
	int16_t y_actual_acceleration;
	int16_t pick_tray_origin_x;
	int16_t pick_tray_origin_y;
	int16_t pick_tray_orientation;
	int16_t place_tray_origin_x;
	int16_t place_tray_origin_y;
	int16_t place_tray_orientation;
	int16_t x_target_position;
	int16_t x_target_speed;
	int16_t x_target_acceleration_time;	// [1 100ms, 2 500ms, 3 1000ms]

	// read/write variables  ==========================================================================
	int16_t heartbeat;					// [0 base system disconnected, 1 base system is connected]
	int16_t end_effector_status;		// [0 laser on/off, 1 power, 2 picking, 3 placing]
	int16_t x_moving_status;			// [0 home, 1 run, 2 jog left -, 3 jog right +]
} MB;

// PRIVATE FUNCTION PROTOTYPE =====================================================================

void modbus_heartbeat_handler(MB *variables);
void modbus_data_sync(MB *variables);
void modbus_init();

// USER CODE ======================================================================================

void modbus_init() {
	hmodbus.huart = &huart2;
	hmodbus.htim = &htim11;
	hmodbus.slaveAddress = 0x15;
	hmodbus.RegisterSize = 70;
	Modbus_init(&hmodbus, MBregisterFrame);
}

void modbus_heartbeat_handler(MB *variables) {
	static int8_t fail = 0;
	static uint32_t timestamp = 0;
	if (HAL_GetTick() >= timestamp) {
		timestamp = HAL_GetTick() + 200;

		// check if the base system send heartbeat
		if (MBregisterFrame[0].U16 == 18537) {
			// success
			variables->heartbeat = 1;
			fail = 0;
		} else {
			// fail, count failure
			if (fail < 126) {
				fail++;
			}
			// if fail is too high then system is disconnected
			if (fail > 17) {
				variables->heartbeat = 0;
			}
		}

		// set heartbeat for base system to see
		MBregisterFrame[0].U16 = 22881;
	}
}

void modbus_data_sync(MB *variables) {
	// report data back to base system
	MBregisterFrame[0x10].U16 = variables->y_moving_status;
	MBregisterFrame[0x11].U16 = variables->y_actual_position;
	MBregisterFrame[0x12].U16 = variables->y_actual_speed;
	MBregisterFrame[0x13].U16 = variables->y_actual_acceleration;
	MBregisterFrame[0x20].U16 = variables->pick_tray_origin_x;
	MBregisterFrame[0x21].U16 = variables->pick_tray_origin_y;
	MBregisterFrame[0x22].U16 = variables->pick_tray_orientation;
	MBregisterFrame[0x23].U16 = variables->place_tray_origin_x;
	MBregisterFrame[0x24].U16 = variables->place_tray_origin_y;
	MBregisterFrame[0x25].U16 = variables->place_tray_orientation;
	MBregisterFrame[0x41].U16 = variables->x_target_position;
	MBregisterFrame[0x42].U16 = variables->x_target_speed;
	MBregisterFrame[0x43].U16 = variables->x_target_acceleration_time;

	// get data from base system
	variables->goal_point_x = MBregisterFrame[0x30].U16;
	variables->goal_point_y = MBregisterFrame[0x31].U16;
	variables->x_actual_position = MBregisterFrame[0x44].U16;
	variables->x_actual_speed = MBregisterFrame[0x45].U16;

	static int16_t base_system_status_slave_temp;
	static int16_t base_system_status_master_temp;
	if (base_system_status_master_temp != MBregisterFrame[0x01].U16) {
		variables->base_system_status = MBregisterFrame[0x01].U16;
		base_system_status_master_temp = variables->base_system_status;
		base_system_status_slave_temp = variables->base_system_status;
	} else if (base_system_status_slave_temp != variables->base_system_status) {
		MBregisterFrame[0x01].U16 = variables->base_system_status;
		base_system_status_master_temp = variables->base_system_status;
		base_system_status_slave_temp = variables->base_system_status;
	}

	// update read/write variable
	static int16_t end_effector_status_slave_temp;
	static int16_t end_effector_status_master_temp;
	if (end_effector_status_master_temp != MBregisterFrame[0x02].U16) {
		// there is an update from master
		variables->end_effector_status = MBregisterFrame[0x02].U16;
		end_effector_status_master_temp = variables->end_effector_status;
		end_effector_status_slave_temp = variables->end_effector_status;
	} else if (end_effector_status_slave_temp != variables->end_effector_status) {
		// there is an update locally
		MBregisterFrame[0x02].U16 = variables->end_effector_status;
		end_effector_status_slave_temp = variables->end_effector_status;
		end_effector_status_master_temp = variables->end_effector_status;
	}
	static int16_t x_moving_status_slave_temp;
	static int16_t x_moving_status_master_temp;
	if (x_moving_status_master_temp != MBregisterFrame[0x40].U16) {
		// there is an update from master
		variables->x_moving_status = MBregisterFrame[0x40].U16;
		x_moving_status_master_temp = variables->x_moving_status;
		x_moving_status_slave_temp = variables->x_moving_status;
	} else if (x_moving_status_slave_temp != variables->x_moving_status) {
		// there is an update locally
		MBregisterFrame[0x40].U16 = variables->x_moving_status;
		x_moving_status_slave_temp = variables->x_moving_status;
		x_moving_status_master_temp = variables->x_moving_status;
	}
}

// USER CODE END ==================================================================================

#endif /* INC_BASESYSTEMMODBUS_H_ */
