#ifndef CONTROLBOARD_H
#define CONTROLBOARD_H

#include "WPILib.h"
#include "RemoteControl.h"
#include "ButtonReader.h"
#include <iostream>
#include <fstream> // file streams defined in this library
#include <cstring>
#include <stdlib.h>

class ControlBoard : public RemoteController {
public:
	ControlBoard();
	~ControlBoard() {};

	virtual void ReadControls();

	double GetJoystickValue(Joysticks j, Axes a);
	bool ReverseDriveDesired();
	void SetReverseDriveDesired(bool desired);

protected:
	bool reverseDriveDesired;
	double leftJoyX, leftJoyY, rightJoyX, rightJoyY;

private:
	Joystick *leftJoy, *rightJoy, *operatorJoy;
	ButtonReader *driveDirectionButton;
	void ReadAllButtons();
};

#endif
