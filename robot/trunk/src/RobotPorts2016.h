#ifndef ROBOTPORTS2016_H
#define ROBOTPORTS2016_H
#include "Debugging.h"

// ***************** PWM PORTS *****************
static const int LEFT_DRIVE_MOTOR_A_PWM_PORT 			= 0;
static const int LEFT_DRIVE_MOTOR_B_PWM_PORT			= 1;
static const int RIGHT_DRIVE_MOTOR_A_PWM_PORT			= 2;
static const int RIGHT_DRIVE_MOTOR_B_PWM_PORT			= 3;

// ***************** DIGITAL I/O PORTS *****************

//static const int LEFT_ENCODER_A_PWM_PORT 			= -1;
//static const int LEFT_ENCODER_B_PWM_PORT			= -1;
//
//static const int RIGHT_ENCODER_A_PWM_PORT			= -1;
//static const int RIGHT_ENCODER_B_PWM_PORT			= -1;
//
//static const int REAR_LEFT_ENCODER_A_PORT			= -1;
//static const int REAR_LEFT_ENCODER_B_PORT			= -1;
//
//static const int REAR_RIGHT_ENCODER_A_PORT			= -1;
//static const int REAR_RIGHT_ENCODER_B_PORT			= -1;


// ***************** MISC *****************
static const int COMPRESSOR_PORT					= 1;
static const int PNEUMATICS_CONTROL_MODULE_ID		= 0;

// ***************** SOLENOID PORTS *****************
static const int GEAR_SHIFT_SOLENOID_PORT				= 1;

// ***************** JOYSTICK USB PORTS *****************
static const int LEFT_JOY_USB_PORT						= 1;
static const int RIGHT_JOY_USB_PORT						= 0;
static const int OPERATOR_JOY_USB_PORT					= 2;

// ***************** BUTTON PORTS *****************
static const int DRIVE_DIRECTION_BUTTON_PORT			= 3;
static const int HIGH_LOW_GEAR_BUTTON_PORT				= 3;

#endif
