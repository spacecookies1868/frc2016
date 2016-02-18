#include "DriveController.h"
#include "ControlBoard.h"
#include "RobotPorts2016.h"
#include "WPILib.h"
#include <math.h>

DriveController::DriveController(RobotModel *myRobot, RemoteControl *myHumanControl){
	robot = myRobot;
	humanControl = myHumanControl;
	m_stateVal = kInitialize;
	nextState = kInitialize;
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

		double rightJoyX = humanControl->GetJoystickValue(RemoteControl::kRightJoy, RemoteControl::kX);
		double rightJoyY = humanControl->GetJoystickValue(RemoteControl::kRightJoy, RemoteControl::kY);
		double leftJoyY = humanControl->GetJoystickValue(RemoteControl::kLeftJoy, RemoteControl::kY);

		if(humanControl->GetQuickTurnDesired()) {
			QuickTurn(rightJoyX);
		} else if(humanControl->GetArcadeDriveDesired()) {
			ArcadeDrive(rightJoyX, DriveDirection() * leftJoyY);
		} else {
			TankDrive(leftJoyY, rightJoyY);
		}

		nextState = kTeleopDrive;
		break;
	}

	m_stateVal = nextState;
}

void DriveController::Reset() {
	m_stateVal = kInitialize;
}

void DriveController::QuickTurn(double myRight) {
	robot->SetWheelSpeed(RobotModel::kLeftWheels, -myRight);
	robot->SetWheelSpeed(RobotModel::kRightWheels, myRight);
}

void DriveController::ArcadeDrive(double myX, double myY) {

	float moveValue = myY;
	float rotateValue = myX * myY;

	float leftMotorOutput = moveValue;
	float rightMotorOutput = moveValue;

	leftMotorOutput -= rotateValue;
	rightMotorOutput += rotateValue;

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

	robot->SetWheelSpeed(RobotModel::kLeftWheels, leftMotorOutput);
	robot->SetWheelSpeed(RobotModel::kRightWheels, rightMotorOutput);
}

void DriveController::TankDrive(double myLeft, double myRight) {
	robot->SetWheelSpeed(RobotModel::kLeftWheels, myLeft);
	robot->SetWheelSpeed(RobotModel::kRightWheels, myRight);
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
