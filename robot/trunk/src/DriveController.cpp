#include "DriveController.h"
#include "ControlBoard.h"
#include "RobotPorts2016.h"
#include "WPILib.h"
#include <math.h>

DriveController::DriveController(RobotModel *myRobot, RemoteController *myHumanControl){
	robot = myRobot;
	humanControl = myHumanControl;
	m_stateVal = kInitialize;
	nextState = kInitialize;
	joyX = 0.0;
	joyY = 0.0;
}

void DriveController::Update(double currTimeSec, double deltaTimeSec) {
	switch (m_stateVal) {
	case (kInitialize):
		nextState = kTeleopDrive;
		break;

	case (kTeleopDrive):
		joyX = humanControl->GetJoystickValue(RemoteController::kRightJoy, RemoteController::kX);
		joyY = humanControl->GetJoystickValue(RemoteController::kLeftJoy, RemoteController::kY);

		ArcadeDrive(joyY, joyX);

		nextState = kTeleopDrive;
		break;
	}

	m_stateVal = nextState;
}

void DriveController::Reset() {
	m_stateVal = kInitialize;
}

void DriveController::ArcadeDrive(double leftJoy, double rightJoy) {
	robot->SetWheelSpeed(RobotModel::kLeftWheels, DriveDirection() * leftJoy + rightJoy);
	robot->SetWheelSpeed(RobotModel::kRightWheels, DriveDirection() * leftJoy - rightJoy);
}

int DriveController::DriveDirection(){
	if (humanControl->ReverseDriveDesired()) {
		return -1;
	} else {
		return 1;
	}
}

void DriveController::RefreshIni() {

}

DriveController::~DriveController() {}
