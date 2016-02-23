#include "AutoPivot.h"

AutoPivot::AutoPivot(RobotModel* myRobot) {
	robot = myRobot;
	desiredDeltaYaw = 0.0;
	isDone = false;
	currYaw = 0.0;
	lastYaw = 0.0;
	accumulatedYaw = 0.0;
	rPIDConfig = NULL;
	rPID = NULL;
}

void AutoPivot::SetDesiredDeltaYaw(double myDesiredDeltaYaw) {
	desiredDeltaYaw = myDesiredDeltaYaw;
}

void AutoPivot::SetDesiredYaw(double myDesiredYaw) {
	double yaw = robot->GetYaw();
	while (myDesiredYaw > 180.0) {
		myDesiredYaw -= 360;
	}
	while (myDesiredYaw < -180.0) {
		myDesiredYaw += 360;
	}
	desiredDeltaYaw = myDesiredYaw - yaw;
	if (desiredDeltaYaw > 180.0) {
		desiredDeltaYaw -= 360;
	} else if (desiredDeltaYaw < -180.0) {
		desiredDeltaYaw += 360;
	}
}

void AutoPivot::Init() {
	isDone = false;
	currYaw = robot->GetYaw();
	accumulatedYaw = 0.0;

	rPIDConfig = CreatePIDConfig();
	rPID = new PIDControlLoop(rPIDConfig);
	rPID->Init(accumulatedYaw, desiredDeltaYaw);
}

PIDConfig* AutoPivot::CreatePIDConfig() {
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = 0.09;
	pidConfig->iFac = 0.0;
	pidConfig->dFac = 0.0;
	pidConfig->maxAbsOutput = 0.3;
	pidConfig->maxAbsError = 10.0;
	pidConfig->maxAbsDiffError = 10.0;
	pidConfig->desiredAccuracy = 0.3;
	pidConfig->maxAbsITerm = 0.1;
	pidConfig->minAbsError = 10.0;
	return pidConfig;
}

void AutoPivot::Update(double currTimeSec, double deltaTimeSec) {
	if (isDone) {
		return;
	}
	UpdateAccumulatedYaw();
	SmartDashboard::PutNumber("Accumulated Yaw: ", accumulatedYaw);
	bool pidDone = rPID->ControlLoopDone(accumulatedYaw, deltaTimeSec);
	if (pidDone) {
		isDone = true;
		robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);

	} else {
		double output = rPID->Update(accumulatedYaw);
		SmartDashboard::PutNumber("Output: ", output);
		robot->SetWheelSpeed(RobotModel::kLeftWheels, output);
		robot->SetWheelSpeed(RobotModel::kRightWheels, -output);
	}
}

void AutoPivot::UpdateAccumulatedYaw() {
	lastYaw = currYaw;
	currYaw = robot->GetYaw();
	double deltaYaw = currYaw - lastYaw;

	if (deltaYaw < -180) {			// going clockwise (from 180 to -180)
		accumulatedYaw += (180 - lastYaw) + (180 + currYaw);
	} else if (deltaYaw > 180) {	// going counterclockwise (from -180 to 180)
		accumulatedYaw -= (180 + lastYaw) + (180 - currYaw);
	} else {
		accumulatedYaw += deltaYaw;
	}
}

bool AutoPivot::IsDone() {
	return isDone;
}

AutoPivot::~AutoPivot() {

}
