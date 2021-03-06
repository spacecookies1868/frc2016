#include <ControlBoard.h>

ControlBoard::ControlBoard() {
	mOperatorJoy = new Joystick(2);
	dialPivotButton = new ButtonReader(mOperatorJoy, 7);
	dialPivotSwitch = new ButtonReader(mOperatorJoy, 2);

	pivotDesired = false;
	desiredAngle = 0.0;
}

void ControlBoard::ReadControls(){
	ReadAllButtons();
	if (dialPivotButton->WasJustPressed() || dialPivotSwitch->IsDown()) {
		pivotDesired = true;
	} else {
		pivotDesired = false;
	}

	desiredAngle = mOperatorJoy->GetY() * -150.0; // Not sure of port
}

bool ControlBoard::GetPivotDesired() {
	return pivotDesired;
}

double ControlBoard::GetDesiredAngle() {
	return desiredAngle;
}

void ControlBoard::ReadAllButtons(){
	dialPivotButton->ReadValue();
	dialPivotSwitch->ReadValue();
}

ControlBoard::~ControlBoard() {
	// TODO Auto-generated destructor stub
}

