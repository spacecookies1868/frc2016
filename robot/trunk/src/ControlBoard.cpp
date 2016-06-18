#include "WPILib.h"
#include "ControlBoard.h"
#include "RobotPorts2016.h"
#include <iostream>
#include <cstring>
#include <stdlib.h>

//This file defines all the joysticks, buttons, other variables, and functions necessary
//to control the robot during teleop by getting human input from the driver station.

//Constructor defines all variables
ControlBoard::ControlBoard() {
	//Four joysticks
	leftJoy  = new Joystick(LEFT_JOY_USB_PORT);
	rightJoy = new Joystick(RIGHT_JOY_USB_PORT);
	operatorJoy = new Joystick(OPERATOR_JOY_USB_PORT);
	operatorJoyB = new Joystick(OPERATOR_JOY_B_USB_PORT);

	//Drivetrain buttons
	driveDirectionButton = new ButtonReader(leftJoy, DRIVE_DIRECTION_BUTTON_PORT);
	gearShiftButton = new ButtonReader(operatorJoyB, HIGH_LOW_GEAR_BUTTON_PORT);
	arcadeDriveButton = new ButtonReader(rightJoy, ARCADE_DRIVE_BUTTON_PORT);
	quickTurnButton = new ButtonReader(rightJoy, QUICK_TURN_BUTTON_PORT);
	dialPivotButton = new ButtonReader(operatorJoy, DIAL_PIVOT_BUTTON_PORT);
	dialPivotSwitch = new ButtonReader(operatorJoy, DIAL_PIVOT_SWITCH_PORT);
	brakeButton = new ButtonReader(leftJoy, BRAKE_BUTTON_PORT);

	//Superstructure buttons
	defenseManipButton = new ButtonReader(operatorJoy, DEFENSE_MANIP_BUTTON_PORT);
	intakePistonButton = new ButtonReader(operatorJoy, INTAKE_PISTON_BUTTON_PORT);
	intakeMotorForwardButton = new ButtonReader(operatorJoy, INTAKE_MOTOR_FORWARD_BUTTON_PORT);
	intakeMotorReverseButton = new ButtonReader(operatorJoy, INTAKE_MOTOR_REVERSE_BUTTON_PORT);
	ballInIntakeButton = new ButtonReader(operatorJoy, BALL_IN_INTAKE_BUTTON_PORT);
	outtakeButton = new ButtonReader(operatorJoy, OUTTAKE_BUTTON_PORT);
	manualOuttakeForwardButton = new ButtonReader(operatorJoy, MANUAL_OUTTAKE_FORWARD_BUTTON_PORT);
	manualOuttakeReverseButton = new ButtonReader(operatorJoy, MANUAL_OUTTAKE_REVERSE_BUTTON_PORT);

	//Buttons for autonomous—-choosing defenses and their positions
	defense_ID_1_Button = new ButtonReader(operatorJoyB, DEFENSE_ID_1_BUTTON_PORT);
	defense_ID_2_Button = new ButtonReader(operatorJoyB, DEFENSE_ID_2_BUTTON_PORT);
	defense_ID_3_Button = new ButtonReader(operatorJoyB, DEFENSE_ID_3_BUTTON_PORT);
	stopAutoButton = new ButtonReader(operatorJoyB, STOP_AUTO_BUTTON_PORT);
	defensePos_ID_1_Button = new ButtonReader(operatorJoyB, DEFENSE_POSITION_ID_1_BUTTON_PORT);
	defensePos_ID_2_Button = new ButtonReader(operatorJoyB, DEFENSE_POSITION_ID_2_BUTTON_PORT);

	//PowerController button
	powerBudgetButton = new ButtonReader(operatorJoy, POWER_BUDGET_SWITCH);

	//Joystick positions that will set speed of robot movement
	leftJoyX = 0.0;
	leftJoyY = 0.0;
	rightJoyX = 0.0;
	rightJoyY = 0.0;

	//Drivetrain variables
	reverseDriveDesired = false;
	gearShiftDesired = false;
	arcadeDriveDesired = false;
	quickTurnDesired = false;
	pivotButtonDesired = false;
	pivotSwitchDesired = false;
	desiredAngle = 0.0;
	brakeDesired = false;

	//Superstructure variables
	defenseManipToggleDesired = false;
	defenseManipDownDesired = false;
	intakePistonToggleDesired = false;
	intakePistonDownDesired = false;
	intakeMotorForwardDesired = false;
	intakeMotorReverseDesired = false;
	ballInIntakeDesired = false;
	outtakeDesired = false;
	manualOuttakeForwardDesired = false;
	manualOuttakeReverseDesired = false;

	//Autonomous variables
	defense = LowBar;
	defensePos = kNone;
	stopAutoDesired = false;

	//PowerController variable
	powerBudgetDesired = false;

};

//ReadControls reads the states of all the buttons and joysticks, and sets variables
//according to the values of the controls.
void ControlBoard::ReadControls() {
	ReadAllButtons();

	//Joystick Positions
	leftJoyX = leftJoy->GetX();
	leftJoyY = leftJoy->GetY();
	rightJoyX = rightJoy->GetX();
	rightJoyY = rightJoy->GetY();

	//DriveTrain Variables
	reverseDriveDesired = driveDirectionButton->IsDown();
	gearShiftDesired = gearShiftButton->StateJustChanged();
	arcadeDriveDesired = !arcadeDriveButton->IsDown();
	quickTurnDesired = quickTurnButton->IsDown();
	brakeDesired = brakeButton->IsDown();

	//Autonomous defense choosing
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

	//Autonomous variables
	stopAutoDesired = !stopAutoButton->IsDown();
	justBeforeBrakeDesired = defense_ID_3_Button->IsDown();

	defenseManipToggleDesired = defenseManipButton->WasJustPressed();
	defenseManipDownDesired = defenseManipButton->IsDown();
	intakePistonToggleDesired = intakePistonButton->WasJustPressed();
	intakePistonDownDesired = intakePistonButton->IsDown();
	intakeMotorForwardDesired = intakeMotorForwardButton->IsDown();
	intakeMotorReverseDesired = intakeMotorReverseButton->IsDown();
	ballInIntakeDesired = ballInIntakeButton->WasJustPressed();
	outtakeDesired = outtakeButton->WasJustPressed();
	manualOuttakeForwardDesired = manualOuttakeForwardButton->IsDown();
	manualOuttakeReverseDesired = manualOuttakeReverseButton->IsDown();
	pivotButtonDesired = dialPivotButton->WasJustPressed();
	pivotSwitchDesired = dialPivotSwitch->IsDown();
	desiredAngle = operatorJoy->GetY() * -180;
	powerBudgetDesired = powerBudgetButton->IsDown();
}

//Returns true if pivoting is desired by the button
bool ControlBoard::GetPivotButtonDesired() {
	return pivotButtonDesired;
}

//Returns true if pivoting is desired by the button
bool ControlBoard::GetPivotSwitchDesired() {
	return pivotSwitchDesired;
}

//Returns the angle to pivot to from the dial
double ControlBoard::GetDesiredAngle() {
	return desiredAngle;
}

//Returns the joystick and axis being used
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

//Returns true if reverse drive is desired
bool ControlBoard::GetReverseDriveDesired() {
	return reverseDriveDesired;
}

//Returns true if gear shifting is desired
bool ControlBoard::GetGearShiftDesired() {
	return gearShiftDesired;
}

//Returns true if arcade drive is desired
bool ControlBoard::GetArcadeDriveDesired() {
	return arcadeDriveDesired;
}

//Returns true if quick turn is desired
bool ControlBoard::GetQuickTurnDesired() {
	return quickTurnDesired;
}

//Returns the defense for autonomous
uint32_t ControlBoard::GetDefense() {
	return defense;
}

//Returns the position of the defense for autonomous
uint32_t ControlBoard::GetDefensePosition() {
	return defensePos;
}

//Returns true if autonomous needs to be stopped
bool ControlBoard::GetStopAutoDesired() {
	return stopAutoDesired;
}

//Returns true if defense manip arm is desired by the toggle
bool ControlBoard::GetDefenseManipToggleDesired() {
	return defenseManipToggleDesired;
}

//Returns true if defense manip arm is desired to go down
bool ControlBoard::GetDefenseManipDownDesired() {
	return defenseManipDownDesired;
}

//Returns true if intake should go in/up
bool ControlBoard::GetIntakePistonToggleDesired() {
	return intakePistonToggleDesired;
}

//Returns true if intake should go down
bool ControlBoard::GetIntakePistonDownDesired() {
	return intakePistonDownDesired;
}

//Returns true if intake motor should go forward (intake balls)
bool ControlBoard::GetIntakeMotorForwardDesired() {
	return intakeMotorForwardDesired;
}

//Returns true if intake motor should go backward (outtake balls)
bool ControlBoard::GetIntakeMotorReverseDesired() {
	return intakeMotorReverseDesired;
}

//Returns true if intaking a ball is desired
bool ControlBoard::GetBallInIntakeDesired() {
	return ballInIntakeDesired;
}

//Returns true if outtaking is desired
bool ControlBoard::GetOuttakeDesired() {
	return outtakeDesired;
}

//Returns true if manual outtake should go forward
bool ControlBoard::GetManualOuttakeForwardDesired() {
	return manualOuttakeForwardDesired;
}

//Returns true if manual outtake should go in reverse
bool ControlBoard::GetManualOuttakeReverseDesired() {
	return manualOuttakeReverseDesired;
}

//Returns true if power budgeting is desired
bool ControlBoard::GetPowerBudgetDesired() {
	return powerBudgetDesired;
}

//Returns true if brake is desired
bool ControlBoard::GetBrakeDesired() {
	return brakeDesired; // purposely using stop auto switch
}

//Returns whether justBeforeBrake is desired
bool ControlBoard::GetJustBeforeDisableBrakeDesired() {
	return justBeforeBrakeDesired;
}

//Reads the values of all buttons defined by this class
void ControlBoard::ReadAllButtons() {
	driveDirectionButton->ReadValue();
	gearShiftButton->ReadValue();
	arcadeDriveButton->ReadValue();
	quickTurnButton->ReadValue();
	defenseManipButton->ReadValue();
	intakePistonButton ->ReadValue();
	intakeMotorForwardButton->ReadValue();
	intakeMotorReverseButton->ReadValue();
	ballInIntakeButton->ReadValue();
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
	brakeButton->ReadValue();
}
