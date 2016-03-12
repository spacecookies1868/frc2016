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
	operatorJoyB = new Joystick(OPERATOR_JOY_B_USB_PORT);

	driveDirectionButton = new ButtonReader(operatorJoy, DRIVE_DIRECTION_BUTTON_PORT);
	gearShiftButton = new ButtonReader(leftJoy, HIGH_LOW_GEAR_BUTTON_PORT);
	arcadeDriveButton = new ButtonReader(rightJoy, ARCADE_DRIVE_BUTTON_PORT);
	quickTurnButton = new ButtonReader(rightJoy, QUICK_TURN_BUTTON_PORT);
	defenseManipButton = new ButtonReader(operatorJoy, DEFENSE_MANIP_BUTTON_PORT);
	intakePistonButton = new ButtonReader(operatorJoy, INTAKE_PISTON_BUTTON_PORT);
	intakeMotorForwardButton = new ButtonReader(operatorJoy, INTAKE_MOTOR_FORWARD_BUTTON_PORT);
	intakeMotorReverseButton = new ButtonReader(operatorJoy, INTAKE_MOTOR_REVERSE_BUTTON_PORT);
	outtakeButton = new ButtonReader(operatorJoy, OUTTAKE_BUTTON_PORT);
	manualOuttakeForwardButton = new ButtonReader(operatorJoyB, MANUAL_OUTTAKE_FORWARD_BUTTON_PORT);
	manualOuttakeReverseButton = new ButtonReader(operatorJoyB, MANUAL_OUTTAKE_REVERSE_BUTTON_PORT);

	defense_ID_1_Button = new ButtonReader(operatorJoyB, DEFENSE_ID_1_BUTTON_PORT);
	defense_ID_2_Button = new ButtonReader(operatorJoyB, DEFENSE_ID_2_BUTTON_PORT);
	defense_ID_3_Button = new ButtonReader(operatorJoyB, DEFENSE_ID_3_BUTTON_PORT);

	powerBudgetButton = new ButtonReader(operatorJoy, POWER_BUDGET_SWITCH);

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
	defense = LowBar;
	manualOuttakeForwardDesired = false;
	manualOuttakeReverseDesired = false;
	powerBudgetDesired = false;
};

void ControlBoard::ReadControls() {
	ReadAllButtons();
	leftJoyX = leftJoy->GetX();
	leftJoyY = leftJoy->GetY();
	rightJoyX = rightJoy->GetX();
	rightJoyY = rightJoy->GetY();

	reverseDriveDesired = driveDirectionButton->IsDown();
	lowGearDesired = gearShiftButton->IsDown();
	arcadeDriveDesired = !arcadeDriveButton->IsDown();
	quickTurnDesired = quickTurnButton->IsDown();
/*
 * Add buttons for defense combinations and if statement on buttons here, then assign defense to
 * the corresponding defense enum
 */
	bool firstDown = defense_ID_1_Button->IsDown();
	bool secondDown = defense_ID_2_Button->IsDown();
	bool thirdDown = defense_ID_3_Button->IsDown();

	if (firstDown) {
		if (secondDown) {
			if (thirdDown) {
				defense = RoughTerrain;
			} else {
				defense = RockWall;
			}
		} else {
			if (thirdDown) {
				defense = LowBar;
				//filler one bc no sally port or drawbridge
			} else {
				defense = Moat;
			}
		}
	} else {
		if (secondDown) {
			if (thirdDown) {
				defense = Ramparts;
			} else {
				defense = ChevalDeFrise;
			}
		} else {
			if (thirdDown) {
				defense = Portcullis;
			} else {
				defense = LowBar;
			}
		}
	}

	defenseManipDesired = defenseManipButton->WasJustPressed();
	intakePistonDesired = intakePistonButton->WasJustPressed();
	intakeMotorForwardDesired = intakeMotorForwardButton->IsDown();
	intakeMotorReverseDesired = intakeMotorReverseButton->IsDown();
	outtakeDesired = outtakeButton->WasJustPressed();
	manualOuttakeForwardDesired = manualOuttakeForwardButton->IsDown();
	manualOuttakeReverseDesired = manualOuttakeReverseButton->IsDown();
	powerBudgetDesired = powerBudgetButton->IsDown();
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

uint32_t ControlBoard::GetDefense() {
	return defense;
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

bool ControlBoard::GetManualOuttakeForwardDesired() {
	return manualOuttakeForwardDesired;
}

bool ControlBoard::GetManualOuttakeReverseDesired() {
	return manualOuttakeReverseDesired;
}

bool ControlBoard::GetPowerBudgetDesired() {
	return powerBudgetDesired;
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
	manualOuttakeForwardButton->ReadValue();
	manualOuttakeReverseButton->ReadValue();
	defense_ID_1_Button->ReadValue();
	defense_ID_2_Button->ReadValue();
	defense_ID_3_Button->ReadValue();
	powerBudgetButton->ReadValue();
}
