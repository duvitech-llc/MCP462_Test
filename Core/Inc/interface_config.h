/*
 * interface_config.h
 *
 *  Created on: Oct 5, 2022
 *      Author: Alexandre Gauthier
 */

#ifndef INC_INTERFACE_CONFIG_H_
#define INC_INTERFACE_CONFIG_H_

#include "stdbool.h"
#include "main.h"


/*
 * Board parameters
 * TODO: Use a config file
 */
#define FIRMWARE_VERSION_SIZE	3
#define FIRMWARE_VERSION_MAJOR	1
#define FIRMWARE_VERSION_MINOR	3
#ifdef ENABLE_MICROSTEP
#define FIRMWARE_VERSION_PATCH	7
#else
#define FIRMWARE_VERSION_PATCH	6
#endif

/*
 * OS config
 */
typedef enum {
	NO_ERROR = 0,
	LS_HW_ERROR,
} SystemWideErrors;

#define OS_TIMEOUT				0xF42400	// Close to 100msec

/*
 * Uart and Command configuration
 */
#define COMMAND_READY_FLAG		1
#define COMMAND_DONE_FLAG		(1<<COMMAND_READY_FLAG)

/*
 *  Optics configuration
 */

#define OPTICS_SAMPLE_PERIOD_MS 5

/*
 * Stepper motor configuration
 */

// Stepper speed curve is re-calculated every STEPPER_MIN_UPDATE_PERIOD ms
#define STEPPER_MIN_UPDATE_PERIOD 2

#define STEPPER_X_STEPS_PER_TURN 400
#define STEPPER_X_HIGH_LIMIT 5
#define STEPPER_X_LOW_LIMIT 4
#define STEPPER_X_STARTING_FREQ 500
#define STEPPER_X_MAX_FREQ 550
#define STEPPER_X_ACCELERATION 200

/*
 * Akshit - 06/14/23
 * TODO: Update the remaining 3 stepper code according to stepper X above
 */

#define STEPPER_Y_STEPS_PER_TURN 400
#define STEPPER_Y_HIGH_LIMIT 7
#define STEPPER_Y_LOW_LIMIT 6
#ifdef ENABLE_MICROSTEP
#define STEPPER_Y_STARTING_FREQ 1500                       // Akshit 06/07/23 - TODO: Change this
#else
#define STEPPER_Y_STARTING_FREQ 1000
#endif
#define STEPPER_Y_MAX_FREQ 8000                            // Akshit 06/07/23 - TODO: Change this
#define STEPPER_Y_ACCELERATION 8000

#define STEPPER_DRAWER_STEPS_PER_TURN 200
#define STEPPER_DRAWER_HIGH_LIMIT 0 //4
#define STEPPER_DRAWER_LOW_LIMIT 1 //3
#define STEPPER_DRAWER_STARTING_FREQ 600
#define STEPPER_DRAWER_MAX_FREQ 900
#define STEPPER_DRAWER_ACCELERATION 600

#define STEPPER_HEATPUMP_STEPS_PER_TURN 200
#define STEPPER_HEATPUMP_HIGH_LIMIT 3 //1
#define STEPPER_HEATPUMP_LOW_LIMIT 2 //0
#define STEPPER_HEATPUMP_STARTING_FREQ 400
#define STEPPER_HEATPUMP_MAX_FREQ 400
#define STEPPER_HEATPUMP_ACCELERATION 1

/*
 * Limit switch configuration
 */
#define LIMIT_SW_NO_CONFLICT 		-1
#define MAX_NUM_SWITCHES			8
#define LIMIT_SW_0_CONFLICT LIMIT_SW_NO_CONFLICT
#define LIMIT_SW_1_CONFLICT LIMIT_SW_NO_CONFLICT
#define LIMIT_SW_2_CONFLICT LIMIT_SW_NO_CONFLICT
#define LIMIT_SW_3_CONFLICT LIMIT_SW_NO_CONFLICT
#define LIMIT_SW_4_CONFLICT LIMIT_SW_NO_CONFLICT
#define LIMIT_SW_5_CONFLICT LIMIT_SW_NO_CONFLICT
#define LIMIT_SW_6_CONFLICT LIMIT_SW_NO_CONFLICT
#define LIMIT_SW_7_CONFLICT LIMIT_SW_NO_CONFLICT

/*
 * #defines are turned into arrays in interface_config.c
 *
 * */

extern const uint8_t LIMIT_SW_CONFLICT[];
extern const GPIO_TypeDef* LIMIT_SW_PORT[];
extern const uint16_t LIMIT_SW_PIN[];
extern bool admin_mode;

#endif /* INC_INTERFACE_CONFIG_H_ */
