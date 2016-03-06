#include <DriveStraightCommand.h>
#include <math.h>

DriveStraightCommand::DriveStraightCommand(RobotModel* myRobot, double myDriveStraightSpeed) {
	robot = myRobot;
	currYaw = 0.0;
	lastYaw = 0.0;
	deltaYaw = 0.0;
	driveStraightSpeed = myDriveStraightSpeed;
	isDone = false;
}

void DriveStraightCommand::Init() {
	initialR = GetAccumulatedYaw();
	desiredR = initialR;
	rPIDConfig = CreateRPIDConfig();
	rPID = new PIDControlLoop(rPIDConfig);
	rPID->Init(initialR, desiredR);
	isDone = false;
}

PIDConfig* DriveStraightCommand::CreateRPIDConfig() {
	PIDConfig* rPIDConfig = new PIDConfig();
	rPIDConfig->pFac = 0.02;
	rPIDConfig->iFac = 0.0;
	rPIDConfig->dFac = 0.1;
	rPIDConfig->maxAbsOutput = 0.65;
	rPIDConfig->maxAbsError = 0.0;
	rPIDConfig->maxAbsDiffError = 0.0;
	rPIDConfig->desiredAccuracy = 0.0;
	rPIDConfig->maxAbsITerm = 0.0;
	rPIDConfig->minAbsError = 0.0;
	rPIDConfig->timeLimit = 0.0;
	return rPIDConfig;
}

void DriveStraightCommand::Update(double currTimeSec, double deltaTimeSec) {
	double rPIDOutput = rPID->Update(GetAccumulatedYaw(), desiredR);
	double leftOutput = driveStraightSpeed + rPIDOutput;
	double rightOutput = driveStraightSpeed - rPIDOutput;

	// makes sure motor outputs are not over 1
	double maxOutput = fmax(fabs(leftOutput), fabs(rightOutput));
	if (maxOutput > 1.0) {
		leftOutput /= maxOutput;
		rightOutput /= maxOutput;
	}

	robot->SetWheelSpeed(RobotModel::kLeftWheels, leftOutput);
	robot->SetWheelSpeed(RobotModel::kRightWheels, rightOutput);
}

bool DriveStraightCommand::IsDone() {
	return isDone;
}

double DriveStraightCommand::GetAccumulatedYaw() {
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

DriveStraightCommand::~DriveStraightCommand() {

}
