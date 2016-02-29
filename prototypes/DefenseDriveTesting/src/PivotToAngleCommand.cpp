#include "PivotToAngleCommand.h"
#include <math.h>

PivotToAngleCommand::PivotToAngleCommand(RobotModel* myRobot, double myDesiredR) {
	/*
	 * Desired R is coming in as a 0 to 360 degree measure.
	 */
	robot = myRobot;
	desiredR = myDesiredR;
	isDone = false;
}

void PivotToAngleCommand::Init() {
	rPIDConfig = CreateRPIDConfig();
	rPID = new PIDControlLoop(rPIDConfig);
	initialR = GetAccumulatedYaw();
	desiredR = CalculateDesiredYaw(desiredR);
	rPID->Init(initialR, desiredR);
}

PIDConfig* PivotToAngleCommand::CreateRPIDConfig(){
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = 0.13;
	pidConfig->iFac = 0.0;
	pidConfig->dFac = 0.6;
	pidConfig->desiredAccuracy = 1.0;
	pidConfig->maxAbsOutput = 0.5;
	pidConfig->maxAbsDiffError = 5.0;
	pidConfig->maxAbsError = 60.0;
	pidConfig->maxAbsITerm = 5.0;
	pidConfig->timeLimit = 0.1;
	return pidConfig;
}

double PivotToAngleCommand::GetAccumulatedYaw() {
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

void PivotToAngleCommand::Update(double currTimeSec, double deltaTimeSec) {
	bool pidDone = rPID->ControlLoopDone(GetAccumulatedYaw(), deltaTimeSec);
	if (pidDone) {
		isDone = true;
		robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
	} else {
		double output = rPID->Update(GetAccumulatedYaw());
		robot->SetWheelSpeed(RobotModel::kLeftWheels, output);
		robot->SetWheelSpeed(RobotModel::kRightWheels, -output);
	}
}

bool PivotToAngleCommand::IsDone() {
	return isDone;
}

double PivotToAngleCommand::CalculateDesiredYaw(double myDesired) {
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
