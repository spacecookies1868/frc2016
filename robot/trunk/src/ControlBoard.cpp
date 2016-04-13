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

	driveDirectionButton = new ButtonReader(leftJoy, DRIVE_DIRECTION_BUTTON_PORT);
	gearShiftButton = new ButtonReader(operatorJoyB, HIGH_LOW_GEAR_BUTTON_PORT);
	arcadeDriveButton = new ButtonReader(rightJoy, ARCADE_DRIVE_BUTTON_PORT);
	quickTurnButton = new ButtonReader(rightJoy, QUICK_TURN_BUTTON_PORT);
	defenseManipButton = new ButtonReader(operatorJoy, DEFENSE_MANIP_BUTTON_PORT);
	intakePistonButton = new ButtonReader(operatorJoy, INTAKE_PISTON_BUTTON_PORT);
	intakeMotorForwardButton = new ButtonReader(operatorJoy, INTAKE_MOTOR_FORWARD_BUTTON_PORT);
	intakeMotorReverseButton = new ButtonReader(operatorJoy, INTAKE_MOTOR_REVERSE_BUTTON_PORT);
	outtakeButton = new ButtonReader(operatorJoy, OUTTAKE_BUTTON_PORT);
	manualOuttakeForwardButton = new ButtonReader(operatorJoy, MANUAL_OUTTAKE_FORWARD_BUTTON_PORT);
	manualOuttakeReverseButton = new ButtonReader(operatorJoy, MANUAL_OUTTAKE_REVERSE_BUTTON_PORT);
	dialPivotButton = new ButtonReader(operatorJoy, DIAL_PIVOT_BUTTON_PORT);
	dialPivotSwitch = new ButtonReader(operatorJoy, DIAL_PIVOT_SWITCH_PORT);

	defense_ID_1_Button = new ButtonReader(operatorJoyB, DEFENSE_ID_1_BUTTON_PORT);
	defense_ID_2_Button = new ButtonReader(operatorJoyB, DEFENSE_ID_2_BUTTON_PORT);
	defense_ID_3_Button = new ButtonReader(operatorJoyB, DEFENSE_ID_3_BUTTON_PORT);
	stopAutoButton = new ButtonReader(operatorJoyB, STOP_AUTO_BUTTON_PORT);
	defensePos_ID_1_Button = new ButtonReader(operatorJoyB, DEFENSE_POSITION_ID_1_BUTTON_PORT);
	defensePos_ID_2_Button = new ButtonReader(operatorJoyB, DEFENSE_POSITION_ID_2_BUTTON_PORT);

	powerBudgetButton = new ButtonReader(operatorJoy, POWER_BUDGET_SWITCH);

	leftJoyX = 0.0;
	leftJoyY = 0.0;
	rightJoyX = 0.0;
	rightJoyY = 0.0;

	reverseDriveDesired = false;
	gearShiftDesired = false;
	arcadeDriveDesired = false;
	quickTurnDesired = false;
	defenseManipToggleDesired = false;
	defenseManipDownDesired = false;
	intakePistonToggleDesired = false;
	intakePistonDownDesired = false;
	intakeMotorForwardDesired = false;
	intakeMotorReverseDesired = false;
	outtakeDesired = false;
	pivotButtonDesired = false;
	pivotSwitchDesired = false;
	desiredAngle = 0.0;
	defense = LowBar;
	defensePos = kNone;
	manualOuttakeForwardDesired = false;
	manualOuttakeReverseDesired = false;
	powerBudgetDesired = false;
	stopAutoDesired = false;
};

void ControlBoard::ReadControls() {
	ReadAllButtons();
	leftJoyX = leftJoy->GetX();
	leftJoyY = leftJoy->GetY();
	rightJoyX = rightJoy->GetX();
	rightJoyY = rightJoy->GetY();

	reverseDriveDesired = driveDirectionButton->IsDown();
	gearShiftDesired = gearShiftButton->StateJustChanged();
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
	//defense = ChevalDeFrise;
	printf("MY DEFENSE %i\n", defense);

	bool firstPosDown = !defensePos_ID_1_Button->IsDown();
	bool secondPosDown = !defensePos_ID_2_Button->IsDown();

	if (defense == LowBar) {
		defensePos = kLowBar;
	} else {
		if (firstPosDown) {
			if (secondPosDown) {
				defensePos = kFifth;
			} else {
				defensePos = kFourth;
			}
		} else {
			if (secondPosDown) {
				defensePos = kThird;
			} else {
				defensePos = kSecond;
			}
		}
	}

	stopAutoDesired = !stopAutoButton->IsDown();

	defenseManipToggleDesired = defenseManipButton->WasJustPressed();
	defenseManipDownDesired = defenseManipButton->IsDown();
	intakePistonToggleDesired = intakePistonButton->WasJustPressed();
	intakePistonDownDesired = intakePistonButton->IsDown();
	intakeMotorForwardDesired = intakeMotorForwardButton->IsDown();
	intakeMotorReverseDesired = intakeMotorReverseButton->IsDown();
	outtakeDesired = outtakeButton->WasJustPressed();
	manualOuttakeForwardDesired = manualOuttakeForwardButton->IsDown();
	manualOuttakeReverseDesired = manualOuttakeReverseButton->IsDown();
	pivotButtonDesired = dialPivotButton->WasJustPressed();
	pivotSwitchDesired = dialPivotSwitch->IsDown();
	desiredAngle = operatorJoy->GetY() * -180;
	powerBudgetDesired = powerBudgetButton->IsDown();
}

bool ControlBoard::GetPivotButtonDesired() {
	return pivotButtonDesired;
}

bool ControlBoard::GetPivotSwitchDesired() {
	return pivotSwitchDesired;
}

double ControlBoard::GetDesiredAngle() {
	return desiredAngle;
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

bool ControlBoard::GetGearShiftDesired() {
	return gearShiftDesired;
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

uint32_t ControlBoard::GetDefensePosition() {
	return defensePos;
}

bool ControlBoard::GetStopAutoDesired() {
	return stopAutoDesired;
}

bool ControlBoard::GetDefenseManipToggleDesired() {
	return defenseManipToggleDesired;
}

bool ControlBoard::GetDefenseManipDownDesired() {
	return defenseManipDownDesired;
}

bool ControlBoard::GetIntakePistonToggleDesired() {
	return intakePistonToggleDesired;
}

bool ControlBoard::GetIntakePistonDownDesired() {
	return intakePistonDownDesired;
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
	defensePos_ID_1_Button->ReadValue();
	defensePos_ID_2_Button->ReadValue();
	stopAutoButton->ReadValue();
	dialPivotButton->ReadValue();
	dialPivotSwitch->ReadValue();
	powerBudgetButton->ReadValue();
}
