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

	//Superstructure controller button accessors
	bool GetDefenseManipDesired();
	bool GetIntakePistonDesired();
	bool GetIntakeMotorForwardDesired();
	bool GetIntakeMotorReverseDesired();
	bool GetOuttakeDesired();

private:
	bool reverseDriveDesired, lowGearDesired, defenseManipDesired, intakePistonDesired, intakeMotorForwardDesired,
		 intakeMotorReverseDesired, outtakeDesired;
	double leftJoyX, leftJoyY, rightJoyX, rightJoyY;
	Joystick *leftJoy, *rightJoy, *operatorJoy;
	ButtonReader *driveDirectionButton, *gearShiftButton, *defenseManipButton, *intakePistonButton, *intakeMotorForwardButton,
				 *intakeMotorReverseButton, *outtakeButton;
	void ReadAllButtons();
};

#endif
