#include "WPILib.h"
#include <ButtonReader.h>

#ifndef SRC_CONTROLBOARD_H_
#define SRC_CONTROLBOARD_H_

class ControlBoard {
public:
	ControlBoard();
	virtual ~ControlBoard();
	virtual void ReadControls();

	bool GetPivotDesired();
	double GetDesiredAngle();

	void ReadAllButtons();

private:
	Joystick* mOperatorJoy;
	ButtonReader* dialPivotButton;
	ButtonReader* dialPivotSwitch;

	double desiredAngle;
	bool pivotDesired;
	bool pivotButtonWasJustPressed;
	double dialPivotAngleValue;
};

#endif /* SRC_CONTROLBOARD_H_ */
