#ifndef CONTROLBOARD_H
#define CONTROLBOARD_H

#include "WPILib.h"
#include "RemoteControl.h"
#include "ButtonReader.h"
#include <iostream>
#include <fstream> // file streams defined in this library
#include <cstring>
#include <stdlib.h>

//Header file for control board��gets input from the human controlling the robot through buttons and joysticks on the driver station
//ControlBoard inherits methods of RemoteControl
class ControlBoard : public RemoteControl {
public:
	ControlBoard();
	~ControlBoard() {};

	virtual void ReadControls();

	//Drive joystick accessors
	double GetJoystickValue(Joysticks j, Axes a);

	//Drive controller button accessors
	bool GetReverseDriveDesired();
	bool GetGearShiftDesired();
	bool GetArcadeDriveDesired();
	bool GetQuickTurnDesired();
	bool GetPivotButtonDesired();
	bool GetPivotSwitchDesired();
	double GetDesiredAngle();

	//Auto buttons
	uint32_t GetDefense();
	uint32_t GetDefensePosition();
	bool GetStopAutoDesired();

	//Superstructure controller button accessors
	bool GetDefenseManipToggleDesired();
	bool GetDefenseManipDownDesired();
	bool GetIntakePistonToggleDesired();
	bool GetIntakePistonDownDesired();
	bool GetIntakeMotorForwardDesired();
	bool GetIntakeMotorReverseDesired();
	bool GetBallInIntakeDesired();
	bool GetOuttakeDesired();
	bool GetManualOuttakeForwardDesired();
	bool GetManualOuttakeReverseDesired();
	bool GetBrakeDesired();
	bool GetJustBeforeDisableBrakeDesired();

	// power controller button accessor
	bool GetPowerBudgetDesired();

	enum Defenses {
		LowBar = 0,
		Portcullis = 1,
		ChevalDeFrise = 2,
		Ramparts = 3,
		Moat = 4,
		SallyPort = 5,
		Drawbridge = 6,
		RockWall = 7,
		RoughTerrain = 8
	};

	enum DefensePositions {
		kNone = 0,
		kLowBar = 1,
		kSecond = 2,
		kThird = 3,
		kFourth = 4,
		kFifth = 5
	};

private:
	//Boolean variables for desired states of drivetrain, superstructure, and autonomous
	bool reverseDriveDesired, gearShiftDesired, arcadeDriveDesired, quickTurnDesired, defenseManipToggleDesired, defenseManipDownDesired,
		 intakePistonToggleDesired,intakePistonDownDesired,intakeMotorForwardDesired, intakeMotorReverseDesired, ballInIntakeDesired,
		 outtakeDesired, manualOuttakeForwardDesired, manualOuttakeReverseDesired, pivotButtonDesired, pivotSwitchDesired,
		 powerBudgetDesired, stopAutoDesired, brakeDesired, justBeforeBrakeDesired;

	//Desired values for driving and pivoting
	double leftJoyX, leftJoyY, rightJoyX, rightJoyY, desiredAngle;

	//Autonomous defense and defense positions
	uint32_t defense;
	uint32_t defensePos;

	//Joysticks and buttons
	Joystick *leftJoy, *rightJoy, *operatorJoy, *operatorJoyB;
	ButtonReader *driveDirectionButton, *gearShiftButton, *arcadeDriveButton, *quickTurnButton, *defenseManipButton, *intakePistonButton,
				 *intakeMotorForwardButton,*intakeMotorReverseButton, *ballInIntakeButton, *outtakeButton, *manualOuttakeForwardButton, *dialPivotButton, *dialPivotSwitch,
				 *manualOuttakeReverseButton, *defense_ID_1_Button, *defense_ID_2_Button, *defense_ID_3_Button,
				 *powerBudgetButton, *stopAutoButton, *defensePos_ID_1_Button, *defensePos_ID_2_Button, *brakeButton;
	void ReadAllButtons();
};

#endif
