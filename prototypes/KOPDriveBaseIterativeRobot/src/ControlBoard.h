#ifndef CONTROLBOARD_H_
#define CONTROLBOARD_H_

#include "WPILib.h"
#include "RemoteControl.h"

class ControlBoard : public RemoteControl {
public:
	ControlBoard();

	void ReadControls();

	double GetJoystickValues(Joysticks j, Axes a);
	bool GetArmControlButtonDown();
	bool GetArmControlButtonUp();
	bool GetIntakeButtonIn();
	bool GetIntakeButtonOut();
	bool GetArmControlButtonPressed();

	~ControlBoard() {};

private:
	Joystick *rightJoy;
	Joystick *leftJoy;
	Joystick *operatorJoy;
	Joystick *intakeJoy;

	double leftJoyX;
	double leftJoyY;
	double rightJoyX;
	double rightJoyY;

	bool armControlButtonDownDesired;
	bool armControlButtonUpDesired;
	bool intakeButtonInDesired;
	bool intakeButtonOutDesired;
	bool armControlButtonWasPressed;
	bool armControlButtonPressed;

};



#endif /* CONTROLBOARD_H_ */
