#ifndef CONTROLBOARD_H
#define CONTROLBOARD_H

#include "WPILib.h"
#include "RemoteControl.h"
#include "ButtonReader.h"
#include <iostream>
#include <fstream> // file streams defined in this library
#include <cstring>
#include <stdlib.h>

class ControlBoard : public RemoteControl {
public:
	ControlBoard();
	~ControlBoard() {};

	virtual void ReadControls();

	//Drive joystick accessors
	double GetJoystickValue(Joysticks j, Axes a);

	//Drive controller button accessors
	bool GetReverseDriveDesired();
	bool GetLowGearDesired();
	bool GetArcadeDriveDesired();
	bool GetQuickTurnDesired();
	uint32_t GetDefense();

	//Superstructure controller button accessors
	bool GetDefenseManipDesired();
	bool GetIntakePistonDesired();
	bool GetIntakeMotorForwardDesired();
	bool GetIntakeMotorReverseDesired();
	bool GetOuttakeDesired();
	bool GetManualOuttakeForwardDesired();
	bool GetManualOuttakeReverseDesired();

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

private:
	bool reverseDriveDesired, lowGearDesired, arcadeDriveDesired, quickTurnDesired, defenseManipDesired, intakePistonDesired,
		 intakeMotorForwardDesired, intakeMotorReverseDesired, outtakeDesired, manualOuttakeForwardDesired, manualOuttakeReverseDesired,
		 powerBudgetDesired;
	double leftJoyX, leftJoyY, rightJoyX, rightJoyY;
	uint32_t defense;
	Joystick *leftJoy, *rightJoy, *operatorJoy, *operatorJoyB;
	ButtonReader *driveDirectionButton, *gearShiftButton, *arcadeDriveButton, *quickTurnButton, *defenseManipButton, *intakePistonButton,
				 *intakeMotorForwardButton,*intakeMotorReverseButton, *outtakeButton, *manualOuttakeForwardButton,
				 *manualOuttakeReverseButton, *powerBudgetButton;
	void ReadAllButtons();
};

#endif
