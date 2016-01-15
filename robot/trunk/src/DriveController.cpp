#include "DriveController.h"
#include "ControlBoard.h"
#include "RobotPorts2016.h"
#include "WPILib.h"
#include <math.h>

DriveController::DriveController(RobotModel *myRobot, RemoteController *myHumanControl){
	robot = myRobot;
	humanControl = myHumanControl;
	m_stateVal = kInitialize;
}

void DriveController::Update(double currTimeSec, double deltaTimeSec) {
	switch (m_stateVal) {
		case (kInitialize):
			nextState = kTeleopDrive;
			break;

		case (kReset):
			nextState = kTeleopDrive;
			break;

		case (kTeleopDrive):
			nextState = kTeleopDrive;
			break;
	}

	m_stateVal = nextState;
}

void DriveController::Reset() {
	m_stateVal = kInitialize;
}

void DriveController::RefreshIni() {

}

DriveController::~DriveController() {
}
