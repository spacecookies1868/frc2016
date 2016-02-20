#include "WPILib.h"
#include <ButtonReader.h>

#ifndef SRC_CONTROLBOARD_H_
#define SRC_CONTROLBOARD_H_

class ControlBoard {
public:
	ControlBoard();
	virtual ~ControlBoard();
	virtual void ReadControls();

	bool DialPivotDesired();
//	void SetDialPivotAngleValue(double pivotAngle);
	double GetDialPivotValue();

	void ReadAllButtons();

private:
	Joystick* mOperatorJoy;
	ButtonReader* dialPivotButton;

	double dialPivotAngleValue;
	bool dialPivotDesired;
};

#endif /* SRC_CONTROLBOARD_H_ */
