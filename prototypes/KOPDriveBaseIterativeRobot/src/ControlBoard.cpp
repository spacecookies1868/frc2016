#include "WPILib.h"
#include "ControlBoard.h"

ControlBoard::ControlBoard() {
	rightJoy = new Joystick(1);
	leftJoy = new Joystick(0);
	operatorJoy = new Joystick(2);
	intakeJoy = new Joystick(3);

	leftJoyX = 0.0;
	leftJoyY = 0.0;
	rightJoyX = 0.0;
	rightJoyY = 0.0;

	armControlButtonDownDesired = false;
	armControlButtonUpDesired = false;
	intakeButtonInDesired = false;
	intakeButtonOutDesired = false;
	armControlButtonWasPressed = false;
	armControlButtonPressed = false;

}

void ControlBoard::ReadControls() {
	//Gets Joystick Values
	leftJoyX = leftJoy->GetX();
	leftJoyY = leftJoy->GetY();
	rightJoyX = rightJoy->GetX();
	rightJoyY = rightJoy->GetY();

	//Checks if buttons are down
	armControlButtonDownDesired = operatorJoy->GetRawButton(3);
	armControlButtonUpDesired = operatorJoy->GetRawButton(4);
	intakeButtonInDesired = intakeJoy->GetRawButton(3);
	intakeButtonOutDesired = intakeJoy->GetRawButton(4);

	//Checks if button was just pressed
	if (operatorJoy->GetRawButton(8) && !armControlButtonWasPressed) {
		armControlButtonPressed = true;
		armControlButtonWasPressed = true;
	} else if (operatorJoy->GetRawButton(8)) {
		armControlButtonPressed = false;
		armControlButtonWasPressed = true;
	} else {
		armControlButtonPressed = false;
		armControlButtonWasPressed = false;
	}
}

double ControlBoard::GetJoystickValues(Joysticks j, Axes a) {
	switch (j) {
	case (kLeftJoy):
		if(a == kX) {
			return leftJoyX;
		} else {
			return leftJoyY;
		}
		break;
	case (kRightJoy):
		if(a == kX) {
			return rightJoyX;
		} else {
			return rightJoyY;
		}
		break;
	default:
		return 0.0;
	}
}
bool ControlBoard::GetArmControlButtonDown() {
	return armControlButtonDownDesired;
}
bool ControlBoard::GetArmControlButtonUp() {
	return armControlButtonUpDesired;
}
bool ControlBoard::GetIntakeButtonIn() {
	return intakeButtonInDesired;
}
bool ControlBoard::GetIntakeButtonOut() {
	return intakeButtonOutDesired;
}
bool ControlBoard::GetArmControlButtonPressed() {
	return armControlButtonPressed;
}
