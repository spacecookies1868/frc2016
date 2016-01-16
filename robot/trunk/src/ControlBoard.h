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
/*
	double GetJoystickValue(Joysticks j, Axes a) ;
*/



protected:
	//bool fieldCentricDesired, mReverseDriveDesired;
	//double mLeftJoyX, mLeftJoyY, mRightJoyX, mRightJoyY;

private:
	//Joystick *mLeftJoy, *mRightJoy, *mOperatorJoy;
	void ReadAllButtons();
};

#endif
