#include "AutoPivot.h"

AutoPivot::AutoPivot(RobotModel* myRobot, double desiredAngle) {
	robot = myRobot;
	desiredR = desiredAngle;
}

void AutoPivot::Init() {
	rPIDConfig = CreatePIDConfig();
	initialR = robot->GetYaw();
	rPID = new PIDControlLoop(rPIDConfig);
	rPID->Init(initialR, initialR + desiredR);

	accumulatedYaw = GetAccumulatedYaw();
	lastYaw = GetAccumulatedYaw();
	currYaw = GetAccumulatedYaw();
	deltaYaw = 0.0;
}

PIDConfig* AutoPivot::CreatePIDConfig() {
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = 0.09;
	pidConfig->iFac = 0.0;
	pidConfig->dFac = 0.0;
	pidConfig->maxAbsOutput = 0.6;
	pidConfig->maxAbsError = 10.0;
	pidConfig->maxAbsDiffError = 10.0;
	pidConfig->desiredAccuracy = 0.3;
	pidConfig->maxAbsITerm = 0.1;
	pidConfig->minAbsError = 10.0;
	return pidConfig;
}

void AutoPivot::Update(double currTimeSec, double deltaTimeSec) {
	accumulatedYaw = GetAccumulatedYaw();
	double outputR = rPID->Update(accumulatedYaw);
	double leftWheels = outputR;
	double rightWheels = -outputR;

	robot->SetWheelSpeed(RobotModel::kLeftWheels, leftWheels);
	robot->SetWheelSpeed(RobotModel::kRightWheels, rightWheels);
	printf("Yaw: %f\n", accumulatedYaw);
}

bool AutoPivot::IsDone(double deltaTimeSec) {
	if (rPID->ControlLoopDone(accumulatedYaw, deltaTimeSec)) {
		printf("DONE, yaw: %f\n", accumulatedYaw);
	}
	return rPID->ControlLoopDone(accumulatedYaw, deltaTimeSec);
}

double AutoPivot::GetAccumulatedYaw() {
	lastYaw = currYaw;
	currYaw = robot->GetYaw();
	deltaYaw = currYaw - lastYaw;

	if (deltaYaw < -180) {			// going clockwise (from 180 to -180)
		accumulatedYaw += (180 - lastYaw) + (180 + currYaw);
	} else if (deltaYaw > 180) {	// going counterclockwise (from -180 to 180)
		accumulatedYaw -= (180 + lastYaw) + (180 - currYaw);
	} else {
		accumulatedYaw += deltaYaw;
	}
	return accumulatedYaw;
}

AutoPivot::~AutoPivot() {

}
