#ifndef ROBOTPORTS2016_H
#define ROBOTPORTS2016_H
#include "Debugging.h"

//TODO These motor/solenoid ports are arbitrary fix, them to fit the real robot

// ***************** PWM PORTS *****************
static const int LEFT_DRIVE_MOTOR_A_PWM_PORT 			= 7;
static const int LEFT_DRIVE_MOTOR_B_PWM_PORT			= 8;
static const int RIGHT_DRIVE_MOTOR_A_PWM_PORT			= 2;
static const int RIGHT_DRIVE_MOTOR_B_PWM_PORT			= 1;
static const int INTAKE_MOTOR_PWM_PORT					= 0;
static const int OUTTAKE_MOTOR_A_PWM_PORT				= 4;
static const int OUTTAKE_MOTOR_B_PWM_PORT				= 5;
static const int SERVO_MOTOR_PORT						= 9; //arbitrary value

// ***************** PDP CHANNELS *****************

static const int LEFT_DRIVE_MOTOR_A_PDP_CHAN			= 13;
static const int LEFT_DRIVE_MOTOR_B_PDP_CHAN			= 12;
static const int RIGHT_DRIVE_MOTOR_A_PDP_CHAN			= 2;
static const int RIGHT_DRIVE_MOTOR_B_PDP_CHAN			= 3;
static const int INTAKE_MOTOR_PDP_CHAN					= 11;

// ***************** DIGITAL I/O PORTS *****************
/*
 * practice bot??

static const int LEFT_ENCODER_A_PWM_PORT 				= 0;
static const int LEFT_ENCODER_B_PWM_PORT				= 1;

static const int RIGHT_ENCODER_A_PWM_PORT				= 2;
static const int RIGHT_ENCODER_B_PWM_PORT				= 3;
*/

static const int LEFT_ENCODER_A_PWM_PORT 				= 2;
static const int LEFT_ENCODER_B_PWM_PORT				= 3;

static const int RIGHT_ENCODER_A_PWM_PORT				= 0;
static const int RIGHT_ENCODER_B_PWM_PORT				= 1;

static const int OUTTAKE_ENCODER_1_A_PWM_PORT			= 7; //arbitrary value
static const int OUTTAKE_ENCODER_1_B_PWM_PORT			= 8; //arbitrary value

static const int OUTTAKE_ENCODER_2_A_PWM_PORT			= 7; //arbitrary value
static const int OUTTAKE_ENCODER_2_B_PWM_PORT			= 8; //arbitrary value

static const int INTAKE_SWITCH_PWM_PORT					= 4;

//******************* ANALOG IN PORTS*******************

static const int PRESSURE_SENSOR_PORT					= 3;
static const int ULTRASONIC_SENSOR_PORT				    = 2; //arbitrary value

// ***************** MISC *****************
static const int COMPRESSOR_PORT						= 1;
static const int PNEUMATICS_CONTROL_MODULE_ID			= 1;

// ***************** SOLENOID PORTS *****************
static const int GEAR_SHIFT_SOLENOID_PORT				= 7;
static const int BRAKE_SOLENOID_A_PORT					= 6;
static const int BRAKE_SOLENOID_B_PORT					= 5;
static const int INTAKE_SOLENOID_A_PORT					= 0;
static const int INTAKE_SOLENOID_B_PORT					= 1;
static const int DEFENSE_MANIP_SOLENOID_A_PORT			= 3;
static const int DEFENSE_MANIP_SOLENOID_B_PORT			= 2;

// ***************** JOYSTICK USB PORTS *****************
static const int LEFT_JOY_USB_PORT						= 0;
static const int RIGHT_JOY_USB_PORT						= 1;
static const int OPERATOR_JOY_USB_PORT					= 2;
static const int OPERATOR_JOY_B_USB_PORT				= 3;

// ***************** BUTTON PORTS *****************
//Drive controller button ports
static const int DRIVE_DIRECTION_BUTTON_PORT			= 3; //new on left joystick, was 12 on op b
static const int HIGH_LOW_GEAR_BUTTON_PORT				= 8;
static const int ARCADE_DRIVE_BUTTON_PORT				= 3;
static const int QUICK_TURN_BUTTON_PORT					= 1;
static const int DIAL_PIVOT_BUTTON_PORT					= 7;
static const int DIAL_PIVOT_SWITCH_PORT					= 2;
static const int BRAKE_BUTTON_PORT						= 2;

//Superstructure controller button ports
static const int DEFENSE_MANIP_BUTTON_PORT				= 5;
static const int INTAKE_PISTON_BUTTON_PORT				= 1;
static const int INTAKE_MOTOR_FORWARD_BUTTON_PORT		= 11;
static const int INTAKE_MOTOR_REVERSE_BUTTON_PORT		= 12;
static const int BALL_IN_INTAKE_BUTTON_PORT				= 6;
static const int OUTTAKE_BUTTON_PORT					= 4;
static const int MANUAL_OUTTAKE_FORWARD_BUTTON_PORT		= 8;
static const int MANUAL_OUTTAKE_REVERSE_BUTTON_PORT		= 9;


//Other controller button ports
static const int DEFENSE_ID_1_BUTTON_PORT				= 3;
static const int DEFENSE_ID_2_BUTTON_PORT				= 4;
static const int DEFENSE_ID_3_BUTTON_PORT				= 5;
static const int STOP_AUTO_BUTTON_PORT					= 6;
static const int DEFENSE_POSITION_ID_1_BUTTON_PORT		= 1;
static const int DEFENSE_POSITION_ID_2_BUTTON_PORT		= 2;

// Power controller button port
static const int POWER_BUDGET_SWITCH					= 7;

#endif
