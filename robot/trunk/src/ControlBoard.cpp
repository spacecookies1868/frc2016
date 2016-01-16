#include "WPILib.h"
#include "ControlBoard.h"
#include "RobotPorts2016.h"
#include <iostream>
#include <cstring>
#include <stdlib.h>

ControlBoard::ControlBoard() {
	mLeftJoy  = new Joystick(LEFT_JOY_USB_PORT);
	mRightJoy = new Joystick(RIGHT_JOY_USB_PORT);
	mOperatorJoy = new Joystick(OPERATOR_JOY_USB_PORT);

	mReverseDriveDesired = false;
	fieldCentricDesired = true;

	fieldRobotButton = new ButtonReader(mRightJoy, FIELD_ROBOT_BUTTON_PORT);
	mDriveDirectionButton = new ButtonReader(mOperatorJoy, REVERSE_DRIVE_BUTTON_PORT);

	mLeftJoyX = 0.0;
	mLeftJoyY = 0.0;
	mRightJoyX = 0.0;
	mRightJoyY = 0.0;
};

void ControlBoard::ReadControls() {
	ReadAllButtons();

	if (fieldRobotButton->IsDown()) {
		SetFieldCentricDesired(true);
	} else {
		SetFieldCentricDesired(false);
	}

	if (mDriveDirectionButton->GetState()) {
		SetReverseDriveDesired(true);
	} else {
		SetReverseDriveDesired(false);
	}

	mLeftJoyX = mLeftJoy->GetX();
	mLeftJoyY = -mLeftJoy->GetY();
	mRightJoyX = mRightJoy->GetX();
	mRightJoyY = -mRightJoy->GetY();
}

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

void ControlBoard::SetJoystickValue(Joysticks j, Axes a, double value) {
	switch (j) {
		case (kLeftJoy):
			if (a == kX) {
				mLeftJoyX = value;
			}
			if (a == kY) {
				mLeftJoyY = value;
			}
		break;
		case (kRightJoy):
			if (a == kX) {
				mRightJoyX = value;
			}
			if (a == kY) {
				mRightJoyY = value;
			}
		break;
	}
}

bool ControlBoard::ReverseDriveDesired() {
	return mReverseDriveDesired;
}

bool ControlBoard::FieldCentricDesired() {
	return fieldCentricDesired;
}

bool ControlBoard::RobotCentricDesired() {
	return !fieldCentricDesired;
}

void ControlBoard::SetReverseDriveDesired(bool desired) {
	mReverseDriveDesired = desired;
}

void ControlBoard::SetFieldCentricDesired(bool des) {
	fieldCentricDesired = des;
}

void ControlBoard::ReadAllButtons() {
	fieldRobotButton->ReadValue();
	mDriveDirectionButton->ReadValue();
}
