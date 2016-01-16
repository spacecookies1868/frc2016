#include "WPILib.h"
#include "ControlBoard.h"
#include "RobotPorts2016.h"
#include <iostream>
#include <cstring>
#include <stdlib.h>

ControlBoard::ControlBoard() {
	/*
	mLeftJoy  = new Joystick(LEFT_JOY_USB_PORT);
	mRightJoy = new Joystick(RIGHT_JOY_USB_PORT);
	mOperatorJoy = new Joystick(OPERATOR_JOY_USB_PORT);
*/
/*
	mLeftJoyX = 0.0;
	mLeftJoyY = 0.0;
	mRightJoyX = 0.0;
	mRightJoyY = 0.0;
	*/
};

void ControlBoard::ReadControls() {
	ReadAllButtons();
/*
	mLeftJoyX = mLeftJoy->GetX();
	mLeftJoyY = -mLeftJoy->GetY();
	mRightJoyX = mRightJoy->GetX();
	mRightJoyY = -mRightJoy->GetY();
	*/
}
/*
double ControlBoard::GetJoystickValue(Joysticks j, Axes a) {
	switch (j) {
		case (kLeftJoy):
			if (a == kX) {
				return mLeftJoyX;
			}
			if (a == kY) {
				return mLeftJoyY;
			}
			break;
		case (kRightJoy):
			if (a == kX) {
				return mRightJoyX;
			}
			if (a == kY) {
				return mRightJoyY;
			}
			break;
		break;
		default:
			return 0.0;
			break;
	}
	return 0.0;
}
*/


void ControlBoard::ReadAllButtons() {
	//fieldRobotButton->ReadValue();
}
