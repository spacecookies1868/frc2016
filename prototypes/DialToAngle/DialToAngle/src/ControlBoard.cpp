#include <ControlBoard.h>

ControlBoard::ControlBoard() {
	mOperatorJoy = new Joystick(2);
	dialPivotButton = new ButtonReader(mOperatorJoy, 1); //Don't know what port it is

	dialPivotDesired = false;
	dialPivotAngleValue = 0.0;
}

void ControlBoard::ReadControls(){
	ReadAllButtons();
	if (dialPivotButton->WasJustPressed()) {
		dialPivotDesired = true;
	}

	dialPivotAngleValue = mOperatorJoy->GetY(); // Not sure of port
}

bool ControlBoard::DialPivotDesired() {
	return dialPivotDesired;
}

double ControlBoard::GetDialPivotValue() {
	return dialPivotAngleValue;
}

void ControlBoard::ReadAllButtons(){
	dialPivotButton->ReadValue();
}

ControlBoard::~ControlBoard() {
	// TODO Auto-generated destructor stub
}

