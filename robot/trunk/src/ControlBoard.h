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

	double GetJoystickValue(Joysticks j, Axes a);

	bool GetReverseDriveDesired();
	bool GetLowGearDesired();
protected:
	bool reverseDriveDesired, lowGearDesired;
	double leftJoyX, leftJoyY, rightJoyX, rightJoyY;

private:
	Joystick *leftJoy, *rightJoy, *operatorJoy;
	ButtonReader *driveDirectionButton, *gearShiftButton;
	void ReadAllButtons();
};

#endif
