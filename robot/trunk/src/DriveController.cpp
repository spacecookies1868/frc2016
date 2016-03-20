#include "DriveController.h"
#include "ControlBoard.h"
#include "RobotPorts2016.h"
#include "WPILib.h"
#include <math.h>
#include "Debugging.h"
#include "Logger.h"

#define PI 3.14159

DriveController::DriveController(RobotModel *myRobot, RemoteControl *myHumanControl){
	robot = myRobot;
	humanControl = myHumanControl;
	m_stateVal = kInitialize;
	nextState = kInitialize;
	rPFac = 0.0;
	rIFac = 0.0;
	rDFac = 0.0;
	rDesiredAccuracy = 0.0;
	rMaxAbsOutput = 0.0;
	rMaxAbsDiffError = 0.0;
	rMaxAbsError = 0.0;
	rMaxAbsITerm = 0.0;
	rTimeLimit = 0.0;
	lastR = robot->GetNavXYaw();
	rPIDConfig = CreateRPIDConfig();
	rPID = new PIDControlLoop(rPIDConfig);
	initializedRPID = false;
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
				DO_PERIODIC(1, printf("Shifting to Low Gear\n"));
			}
		} else {
			if (robot->IsLowGear()) {
				robot->ShiftToHighGear();
				DO_PERIODIC(1, printf("Shifting to High Gear\n"));
			}
		}

		double rightJoyX = humanControl->GetJoystickValue(RemoteControl::kRightJoy, RemoteControl::kX);
		double rightJoyY = -humanControl->GetJoystickValue(RemoteControl::kRightJoy, RemoteControl::kY);
		double leftJoyY = -humanControl->GetJoystickValue(RemoteControl::kLeftJoy, RemoteControl::kY);

		if(humanControl->GetQuickTurnDesired()) {
			QuickTurn(rightJoyX);
		} else if(humanControl->GetArcadeDriveDesired()) {
			ArcadeDrive(rightJoyX, leftJoyY);
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
	robot->SetWheelSpeed(RobotModel::kLeftWheels, myRight);
	robot->SetWheelSpeed(RobotModel::kRightWheels, -myRight);
}

void DriveController::ArcadeDrive(double myX, double myY) {
	double moveValue = myY * DriveDirection();
	double rotateValue = myX;

	// does not rotate at all if not moving forward/backward
	if (fabs(moveValue) < 0.1) {
		rotateValue = 0.0;
	}

	double leftMotorOutput = moveValue;
	double rightMotorOutput = moveValue;

	double currR = robot->GetNavXYaw();

	if (fabs(rotateValue) < 0.1) {
		// makes robot go straight without drifting
		if (!initializedRPID) {
			lastR = currR;
			rPID->Init(currR, lastR);
			initializedRPID = true;
		} else {
			double output = rPID->Update(currR, lastR);
			leftMotorOutput += output;
			rightMotorOutput -= output;
		}
	} else {
		leftMotorOutput += rotateValue;
		rightMotorOutput -= rotateValue;
		initializedRPID = false;
	}

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

	leftMotorOutput = sin(leftMotorOutput * PI / 2);
	rightMotorOutput = sin(rightMotorOutput * PI / 2);

	if (fabs(moveValue) < 0.05) {
		leftMotorOutput = 0.0;
		rightMotorOutput = 0.0;
	}

	LOG(robot, "MoveValue", moveValue);
	LOG(robot, "RotateValue", rotateValue);
	LOG(robot, "Left Motor Output", leftMotorOutput);
	LOG(robot, "Right Motor Output", rightMotorOutput);

	robot->SetWheelSpeed(RobotModel::kLeftWheels, leftMotorOutput);
	robot->SetWheelSpeed(RobotModel::kRightWheels, rightMotorOutput);
}

void DriveController::TankDrive(double myLeft, double myRight) {
	double leftMotorOutput = 0.0;
	double rightMotorOutput = 0.0;

	if (humanControl->GetReverseDriveDesired()) {
		leftMotorOutput = myRight;
		rightMotorOutput = myLeft;
	} else {
		leftMotorOutput = myLeft;
		rightMotorOutput = myRight;
	}

	leftMotorOutput = sin(leftMotorOutput * PI / 2) * DriveDirection();
	rightMotorOutput = sin(rightMotorOutput * PI / 2) * DriveDirection();

	robot->SetWheelSpeed(RobotModel::kLeftWheels, leftMotorOutput);
	robot->SetWheelSpeed(RobotModel::kRightWheels, rightMotorOutput);
}

int DriveController::DriveDirection(){
	if (humanControl->GetReverseDriveDesired()) {
		return -1;
	} else {
		return 1;
	}
}

PIDConfig* DriveController::CreateRPIDConfig(){
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = rPFac;
	pidConfig->iFac = rIFac;
	pidConfig->dFac = rDFac;
	pidConfig->desiredAccuracy = rDesiredAccuracy;
	pidConfig->maxAbsOutput = rMaxAbsOutput;
	pidConfig->maxAbsDiffError = rMaxAbsDiffError;
	pidConfig->maxAbsError = rMaxAbsError;
	pidConfig->maxAbsITerm = rMaxAbsITerm;
	pidConfig->timeLimit = rTimeLimit;
	return pidConfig;
}

void DriveController::RefreshIni() {
	rPFac = robot->pini->getf("DRIVESTRAIGHTTELEOP", "rPFac", 0.0);
	rIFac = robot->pini->getf("DRIVESTRAIGHTTELEOP", "rIFac", 0.0);
	rDFac = robot->pini->getf("DRIVESTRAIGHTTELEOP", "rDFac", 0.0);
	rDesiredAccuracy = robot->pini->getf("DRIVESTRAIGHTTELEOP", "rDesiredAccuracy", 0.0);
	rMaxAbsOutput = robot->pini->getf("DRIVESTRAIGHTTELEOP", "rMaxAbsOutput", 0.0);
	rMaxAbsDiffError = robot->pini->getf("DRIVESTRAIGHTTELEOP", "rMaxAbsError", 0.0);
	rMaxAbsError = robot->pini->getf("DRIVESTRAIGHTTELEOP", "rMaxAbsDiffError", 0.0);
	rMaxAbsITerm = robot->pini->getf("DRIVESTRAIGHTTELEOP", "rMaxAbsITerm", 0.0);
	rTimeLimit = robot->pini->getf("DRIVESTRAIGHTTELEOP", "rTimeLimit", 0.0);
}

DriveController::~DriveController() {}
