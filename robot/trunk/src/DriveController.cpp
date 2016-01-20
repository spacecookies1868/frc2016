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
		if(humanControl->GetLowGearDesired()){
			if (!(robot->IsLowGear())){
				robot->ShiftToLowGear();
			}
		} else {
			if (robot->IsLowGear()) {
				robot->ShiftToHighGear();
			}
		}

		joyX = DriveDirection() * humanControl->GetJoystickValue(RemoteController::kRightJoy, RemoteController::kX);
		joyY = DriveDirection() * humanControl->GetJoystickValue(RemoteController::kLeftJoy, RemoteController::kY);

		ArcadeDrive(joyX, joyY);

		nextState = kTeleopDrive;
		break;
	}

	m_stateVal = nextState;
}

void DriveController::Reset() {
	m_stateVal = kInitialize;
}

void DriveController::ArcadeDrive(double myX, double myY) {
	float moveValue = myY;
	float rotateValue = -myX;

	float leftMotorOutput = moveValue;
	float rightMotorOutput = moveValue;

	leftMotorOutput += rotateValue;
	rightMotorOutput -= rotateValue;

	if (leftMotorOutput > 1.0) {
		rightMotorOutput = rightMotorOutput / leftMotorOutput;
		leftMotorOutput = 1.0;
	} else if (leftMotorOutput < -1.0) {
		rightMotorOutput = -rightMotorOutput / leftMotorOutput;
		leftMotorOutput = -1.0;
	} else if (rightMotorOutput > 1.0){
		leftMotorOutput = leftMotorOutput / rightMotorOutput;
		rightMotorOutput = 1.0;
	} else if (rightMotorOutput < -1.0) {
		leftMotorOutput = -leftMotorOutput/rightMotorOutput;
		rightMotorOutput = -1.0;
	}

	robot->SetWheelSpeed(RobotModel::kLeftWheels, -leftMotorOutput);
	robot->SetWheelSpeed(RobotModel::kRightWheels, rightMotorOutput);
}

int DriveController::DriveDirection(){
	if (humanControl->GetReverseDriveDesired()) {
		return -1;
	} else {
		return 1;
	}
}

void DriveController::RefreshIni() {

}

DriveController::~DriveController() {}
