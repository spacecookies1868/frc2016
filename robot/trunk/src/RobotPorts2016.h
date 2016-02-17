#ifndef ROBOTPORTS2016_H
#define ROBOTPORTS2016_H
#include "Debugging.h"

// ***************** PWM PORTS *****************

static const int LEFT_DRIVE_MOTOR_A_PWM_PORT 			= 1;
static const int LEFT_DRIVE_MOTOR_B_PWM_PORT			= 2;
static const int RIGHT_DRIVE_MOTOR_A_PWM_PORT			= 7;
static const int RIGHT_DRIVE_MOTOR_B_PWM_PORT			= 8;

// ***************** DIGITAL I/O PORTS *****************

static const int LEFT_ENCODER_A_PWM_PORT 				= 0;
static const int LEFT_ENCODER_B_PWM_PORT				= 1;

static const int RIGHT_ENCODER_A_PWM_PORT				= 2;
static const int RIGHT_ENCODER_B_PWM_PORT				= 3;


// ***************** MISC *****************
static const int COMPRESSOR_PORT						= 1;
static const int PNEUMATICS_CONTROL_MODULE_ID			= 0;

// ***************** SOLENOID PORTS *****************
static const int GEAR_SHIFT_SOLENOID_PORT				= 7;

// ***************** JOYSTICK USB PORTS *****************
static const int LEFT_JOY_USB_PORT						= 0;
static const int RIGHT_JOY_USB_PORT						= 1;
static const int OPERATOR_JOY_USB_PORT					= 2;

// ***************** BUTTON PORTS *****************
//Drive controller button ports
static const int DRIVE_DIRECTION_BUTTON_PORT			= 3;
static const int HIGH_LOW_GEAR_BUTTON_PORT				= 3;

//Superstructure controller button ports
static const int DEFENSE_MANIP_BUTTON_PORT				= 5;
static const int INTAKE_PISTON_BUTTON_PORT				= 1;
static const int INTAKE_MOTOR_FORWARD_BUTTON_PORT		= 4;
static const int INTAKE_MOTOR_REVERSE_BUTTON_PORT		= 3;
static const int OUTTAKE_BUTTON_PORT					= 9;

#endif
