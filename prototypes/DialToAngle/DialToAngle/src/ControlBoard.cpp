#include <ControlBoard.h>

ControlBoard::ControlBoard() {
	mOperatorJoy = new Joystick(1);//port 2 on real driver station
	dialPivotButton = new ButtonReader(mOperatorJoy, 2); //port 7 on real driver station

	pivotDesired = false;
	desiredAngle = 0.0;
}

void ControlBoard::ReadControls(){
	ReadAllButtons();
	if (dialPivotButton->WasJustPressed()) {
		pivotDesired = true;
	} else {
		pivotDesired = false;
	}

	desiredAngle = mOperatorJoy->GetZ() * 180.0; // Not sure of port
}

bool ControlBoard::GetPivotDesired() {
	return pivotDesired;
}

double ControlBoard::GetDesiredAngle() {
	return desiredAngle;
}

void ControlBoard::ReadAllButtons(){
	dialPivotButton->ReadValue();
}

ControlBoard::~ControlBoard() {
	// TODO Auto-generated destructor stub
}

