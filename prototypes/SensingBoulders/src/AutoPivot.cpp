#include "AutoPivot.h"

AutoPivot::AutoPivot(RobotModel* myRobot, double myDesiredR) {
	robot = myRobot;
	desiredR = myDesiredR;
	isDone = false;
	lastYaw = 0.0;
	currYaw = 0.0;
}

void AutoPivot::Init() {
	rPIDConfig = CreatePIDConfig();
	rPID = new PIDControlLoop(rPIDConfig);
	initialR = GetAccumulatedYaw();
	desiredR = CalculateDesiredYaw(desiredR);
	rPID->Init(initialR, desiredR);

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
	bool pidDone = rPID->ControlLoopDone(GetAccumulatedYaw(), deltaTimeSec);
	if (pidDone) {
		isDone = true;
		robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
	} else {
		double output = rPID->Update(GetAccumulatedYaw());
		robot->SetWheelSpeed(RobotModel::kLeftWheels, -output);
		robot->SetWheelSpeed(RobotModel::kRightWheels, output);
	}
}

double AutoPivot::GetAccumulatedYaw() {
	lastYaw = currYaw;
	currYaw = robot->GetYaw();;
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

bool AutoPivot::IsDone() {
	return isDone;
}

double AutoPivot::CalculateDesiredYaw(double myDesired) {
	double normalizedInitial = fmod(initialR, 360.0);
	if (normalizedInitial < 0) {
		normalizedInitial += 360;
	}
	double change1 = myDesired - normalizedInitial;
	double change2 = change1 + 360.0;
	double change3  = change1 - 360.0;
	double change;
	if (fabs(change1) < fmin(fabs(change2), fabs(change3))){
		change = change1;
	} else if (fabs(change2) < fabs(change3)) {
		change = change2;
	} else {
		change = change3;
	}
	double desired = initialR + change;
	return desired;
}


AutoPivot::~AutoPivot() {

}
