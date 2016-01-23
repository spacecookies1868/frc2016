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
	gearShiftButton = new ButtonReader(operatorJoy, HIGH_LOW_GEAR_BUTTON_PORT);

	leftJoyX = 0.0;
	leftJoyY = 0.0;
	rightJoyX = 0.0;
	rightJoyY = 0.0;
	reverseDriveDesired = false;
	lowGearDesired = false;
};

void ControlBoard::ReadControls() {
	ReadAllButtons();
	leftJoyX = leftJoy->GetX();
	leftJoyY = leftJoy->GetY();
	rightJoyX = rightJoy->GetX();
	rightJoyY = rightJoy->GetY();

	if (driveDirectionButton->IsDown()) {
		reverseDriveDesired = true;
	}

	// IMPORTANT: This assumes WasJustPressed() applies to high gear and WasJustReleased applies to low gear
	if (gearShiftButton->IsDown()) {
		lowGearDesired = false;
	} else {
		lowGearDesired = true;
	}
}

double ControlBoard::GetJoystickValue(Joysticks j, Axes a) {
	switch (j) {
		case (kLeftJoy):
			if (a == kX) {
				return leftJoyX;
			} else if (a == kY) {
				return leftJoyY;
			}
			break;
		case (kRightJoy):
			if (a == kX) {
				return rightJoyX;
			} else if (a == kY) {
				return rightJoyY;
			}
			break;
		default:
			return 0.0;
			break;
	}
	return 0.0;
}

bool ControlBoard::GetReverseDriveDesired() {
	return reverseDriveDesired;
}

bool ControlBoard::GetLowGearDesired() {
	return lowGearDesired;
}

void ControlBoard::ReadAllButtons() {
	driveDirectionButton->ReadValue();
	gearShiftButton->ReadValue();
}
