#ifndef ROBOTPORTS2016_H
#define ROBOTPORTS2016_H
#include "Debugging.h"

// ***************** PWM PORTS *****************
#if COMP_BOT

#else
static const int FRONT_LEFT_MOTOR_PWM_PORT 			= -1;
static const int REAR_LEFT_MOTOR_PWM_PORT			= -1;
static const int FRONT_RIGHT_MOTOR_PWM_PORT			= -1;
static const int REAR_RIGHT_MOTOR_PWM_PORT			= -1;
#endif

// ***************** DIGITAL I/O PORTS *****************
#if COMP_BOT

#else
static const int LEFT_ENCODER_A_PWM_PORT 			= -1;
static const int LEFT_ENCODER_B_PWM_PORT			= -1;

static const int RIGHT_ENCODER_A_PWM_PORT			= -1;
static const int RIGHT_ENCODER_B_PWM_PORT			= -1;

static const int REAR_LEFT_ENCODER_A_PORT			= -1;
static const int REAR_LEFT_ENCODER_B_PORT			= -1;

static const int REAR_RIGHT_ENCODER_A_PORT			= -1;
static const int REAR_RIGHT_ENCODER_B_PORT			= -1;
#endif

// ***************** MISC *****************
static const int COMPRESSOR_PORT					= -1;
static const int PNEUMATICS_CONTROL_MODULE_ID		= -1;
static const int GYRO_PORT 							= -1;

// ***************** SOLENOID PORTS *****************

// ***************** JOYSTICK USB PORTS *****************
static const int LEFT_JOY_USB_PORT						= -1;
static const int RIGHT_JOY_USB_PORT						= -1;
static const int OPERATOR_JOY_USB_PORT					= -1;

// ***************** BUTTON PORTS *****************
static const int REVERSE_DRIVE_BUTTON_PORT				= -1;
static const int FIELD_ROBOT_BUTTON_PORT				= -1;

#endif
