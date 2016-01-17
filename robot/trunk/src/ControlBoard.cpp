#include "WPILib.h"
#include "ControlBoard.h"
#include "RobotPorts2016.h"
#include <iostream>
#include <cstring>
#include <stdlib.h>

ControlBoard::ControlBoard() {
	leftJoy  = new Joystick(LEFT_JOY_USB_PORT);
	rightJoy = new Joystick(RIGHT_JOY_USB_PORT);
	operatorJoy = new Joystick(OPERATOR_JOY_USB_PORT);

	driveDirectionButton = new ButtonReader(operatorJoy, DRIVE_DIRECTION_BUTTON_PORT);

	leftJoyX = 0.0;
	leftJoyY = 0.0;
	rightJoyX = 0.0;
	rightJoyY = 0.0;
	reverseDriveDesired = false;
};

void ControlBoard::ReadControls() {
	ReadAllButtons();
	leftJoyX = leftJoy->GetX();
	leftJoyY = leftJoy->GetY();
	rightJoyX = rightJoy->GetX();
	rightJoyY = rightJoy->GetY();
	SetReverseDriveDesired(driveDirectionButton->GetState());
}

double ControlBoard::GetJoystickValue(Joysticks j, Axes a) {
	switch (j) {
		case (kLeftJoy):
			if (a == kX) {
				return leftJoyX;
			}
			if (a == kY) {
				return leftJoyY;
			}
			break;
		case (kRightJoy):
			if (a == kX) {
				return rightJoyX;
			}
			if (a == kY) {
				return rightJoyY;
			}
			break;
		break;
		default:
			return 0.0;
			break;
	}
	return 0.0;
}

bool ControlBoard::ReverseDriveDesired() {
	return reverseDriveDesired;
}

void ControlBoard::SetReverseDriveDesired(bool desired) {
	reverseDriveDesired = desired;
}

void ControlBoard::ReadAllButtons() {
	driveDirectionButton->ReadValue();
}
