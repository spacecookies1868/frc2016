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

	//Superstructure controller button accessors
	bool GetDefenseManipDesired();
	bool GetIntakePistonDesired();
	bool GetIntakeMotorForwardDesired();
	bool GetIntakeMotorReverseDesired();
	bool GetOuttakeDesired();
	bool GetManualOuttakeForwardDesired();
	bool GetManualOuttakeReverseDesired();

private:
	bool reverseDriveDesired, lowGearDesired, arcadeDriveDesired, quickTurnDesired, defenseManipDesired, intakePistonDesired,
		 intakeMotorForwardDesired, intakeMotorReverseDesired, outtakeDesired, manualOuttakeForwardDesired, manualOuttakeReverseDesired;
	double leftJoyX, leftJoyY, rightJoyX, rightJoyY;
	Joystick *leftJoy, *rightJoy, *operatorJoy, *operatorJoyB;
	ButtonReader *driveDirectionButton, *gearShiftButton, *arcadeDriveButton, *quickTurnButton, *defenseManipButton, *intakePistonButton,
				 *intakeMotorForwardButton,*intakeMotorReverseButton, *outtakeButton, *manualOuttakeForwardButton,
				 *manualOuttakeReverseButton;
	void ReadAllButtons();
};

#endif
