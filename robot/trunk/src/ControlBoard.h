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
	void OpenFile();
	void CloseFile();

	double GetJoystickValue(Joysticks j, Axes a) /*const*/;
	void SetJoystickValue(Joysticks j, Axes a, double value); // used to be in acb
	bool ReverseDriveDesired();

	/**
	 * Puts robot on an absolute orientation so that it moves relative to the
	 * field, not the direction it's facing.
	 */
	bool FieldCentricDesired();

	/**
	 * Puts robot on an orientation relative to itself, ie turning left means
	 * the robot will turn in the direction that is left from its point of view.
	 */
	bool RobotCentricDesired();

	void SetReverseDriveDesired(bool desired);
	void SetFieldCentricDesired(bool des);

protected:
	bool fieldCentricDesired, mReverseDriveDesired;
	double mLeftJoyX, mLeftJoyY, mRightJoyX, mRightJoyY;

private:
	Joystick *mLeftJoy, *mRightJoy, *mOperatorJoy;
	ButtonReader *mDriveDirectionButton, *fieldRobotButton;
	void ReadAllButtons();
};

#endif
