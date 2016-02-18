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
	gearShiftButton = new ButtonReader(leftJoy, HIGH_LOW_GEAR_BUTTON_PORT);
	arcadeDriveButton = new ButtonReader(rightJoy, ARCADE_DRIVE_BUTTON_PORT);
	quickTurnButton = new ButtonReader(rightJoy, QUICK_TURN_BUTTON_PORT);
	defenseManipButton = new ButtonReader(operatorJoy, DEFENSE_MANIP_BUTTON_PORT);
	intakePistonButton = new ButtonReader(operatorJoy, INTAKE_PISTON_BUTTON_PORT);
	intakeMotorForwardButton = new ButtonReader(operatorJoy, INTAKE_MOTOR_FORWARD_BUTTON_PORT);
	intakeMotorReverseButton = new ButtonReader(operatorJoy, INTAKE_MOTOR_REVERSE_BUTTON_PORT);
	outtakeButton = new ButtonReader(operatorJoy, OUTTAKE_BUTTON_PORT);

	leftJoyX = 0.0;
	leftJoyY = 0.0;
	rightJoyX = 0.0;
	rightJoyY = 0.0;

	reverseDriveDesired = false;
	lowGearDesired = false;
	arcadeDriveDesired = false;
	quickTurnDesired = false;
	defenseManipDesired = false;
	intakePistonDesired = false;
	intakeMotorForwardDesired = false;
	intakeMotorReverseDesired = false;
	outtakeDesired = false;
};

void ControlBoard::ReadControls() {
	ReadAllButtons();
	leftJoyX = leftJoy->GetX();
	leftJoyY = leftJoy->GetY();
	rightJoyX = rightJoy->GetX();
	rightJoyY = rightJoy->GetY();

	reverseDriveDesired = driveDirectionButton->IsDown();
	lowGearDesired = gearShiftButton->IsDown();
	arcadeDriveDesired = arcadeDriveButton->IsDown();
	quickTurnDesired = quickTurnButton->IsDown();

	defenseManipDesired = defenseManipButton->WasJustPressed();
	intakePistonDesired = intakePistonButton->WasJustPressed();
	intakeMotorForwardDesired = intakeMotorForwardButton->IsDown();
	intakeMotorReverseDesired = intakeMotorReverseButton->IsDown();
	outtakeDesired = outtakeButton->WasJustPressed();
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

bool ControlBoard::GetArcadeDriveDesired() {
	return arcadeDriveDesired;
}

bool ControlBoard::GetQuickTurnDesired() {
	return quickTurnDesired;
}

bool ControlBoard::GetDefenseManipDesired() {
	return defenseManipDesired;
}

bool ControlBoard::GetIntakePistonDesired() {
	return intakePistonDesired;
}

bool ControlBoard::GetIntakeMotorForwardDesired() {
	return intakeMotorForwardDesired;
}

bool ControlBoard::GetIntakeMotorReverseDesired() {
	return intakeMotorReverseDesired;
}

bool ControlBoard::GetOuttakeDesired() {
	return outtakeDesired;
}

void ControlBoard::ReadAllButtons() {
	driveDirectionButton->ReadValue();
	gearShiftButton->ReadValue();
	arcadeDriveButton->ReadValue();
	quickTurnButton->ReadValue();
	defenseManipButton->ReadValue();
	intakePistonButton ->ReadValue();
	intakeMotorForwardButton->ReadValue();
	intakeMotorReverseButton->ReadValue();
	outtakeButton->ReadValue();
}
