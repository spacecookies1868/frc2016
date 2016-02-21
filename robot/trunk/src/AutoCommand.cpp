#include "AutoCommand.h"
#include <math.h>
#include "ini.h"
#include <iostream>
#include <string>
#include "Logger.h"
#include "Debugging.h"

#define PI 3.14159265358979

/*
 * Waiting Command
 *
 * checked by Katy, works
 */

WaitingCommand::WaitingCommand(double myWaitTimeSec) {
	waitTimeSec = myWaitTimeSec;
	timer = new Timer();
	isDone = false;
}

void WaitingCommand::Init() {
	timer->Start();
}

void WaitingCommand::Update(double currTimeSec, double deltaTimeSec) {
	isDone = (timer->Get() >= waitTimeSec);
}

bool WaitingCommand::IsDone() {
	return isDone;
}

/*
 * Pivot Command
 *
 * NOT TESTED because navx issues, mainly taken from prototype code
 *
 * hopefully positive desiredR makes turn counterclockwise
 */
#if USE_NAVX
PivotCommand::PivotCommand(RobotModel* myRobot, double myDesiredR) {
	robot = myRobot;
	desiredR = myDesiredR;
	isDone = false;
	accumulatedYaw = 0;
	lastYaw = 0;
	currYaw = 0;
}

void PivotCommand::Init() {
	rPIDConfig = CreateRPIDConfig();
	initialR = GetAccumulatedYaw();
	rPID = new PIDControlLoop(rPIDConfig);
	rPID->Init(initialR, initialR + desiredR);
	LOG(robot, "START YAW", robot->GetNavXYaw());
}

double PivotCommand::rPFac = 0.0;
double PivotCommand::rIFac = 0.0;
double PivotCommand::rDFac = 0.0;
double PivotCommand::rDesiredAccuracy = 0.0;
double PivotCommand::rMaxAbsOutput = 0.0;
double PivotCommand::rMaxAbsDiffError = 0.0;
double PivotCommand::rMaxAbsError = 0.0;
double PivotCommand::rMaxAbsITerm = 0.0;
double PivotCommand::rTimeLimit = 0.0;

PIDConfig* PivotCommand::CreateRPIDConfig(){
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

double PivotCommand::GetAccumulatedYaw() {
	lastYaw = currYaw;
	currYaw = robot->GetNavXYaw();
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

void PivotCommand::Update(double currTimeSec, double deltaTimeSec) {
	bool pidDone = rPID->ControlLoopDone(GetAccumulatedYaw(), deltaTimeSec);
	if (pidDone) {
		isDone = true;
		robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
		LOG(robot, "END YAW", robot->GetNavXYaw());
	} else {
		double output = rPID->Update(GetAccumulatedYaw());
		DO_PERIODIC(5, printf("My Yaw: %f\n", GetAccumulatedYaw()));
		DO_PERIODIC(5, printf("Desired Yaw: %f\n", desiredR));
		DO_PERIODIC(5, printf("Output: %f\n", output));
		DO_PERIODIC(1, LOG(robot, "Accumulated Yaw", GetAccumulatedYaw()));
		DO_PERIODIC(1, LOG(robot, "Yaw", robot->GetNavXYaw()));
		DO_PERIODIC(1, LOG(robot, "Desired Yaw", desiredR));
		DO_PERIODIC(1, LOG(robot, "Output", output));
		robot->SetWheelSpeed(RobotModel::kLeftWheels, output);
		robot->SetWheelSpeed(RobotModel::kRightWheels, -output);
	}
}

bool PivotCommand::IsDone() {
	return isDone;
}

#endif

#if USE_NAVX
/*
 * Pivot to Angle Command:
 * NOT TESTED because navx issues bleh
 */

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
	LOG(robot, "START YAW", robot->GetNavXYaw());
}

double PivotToAngleCommand::rPFac = 0.0;
double PivotToAngleCommand::rIFac = 0.0;
double PivotToAngleCommand::rDFac = 0.0;
double PivotToAngleCommand::rDesiredAccuracy = 0.0;
double PivotToAngleCommand::rMaxAbsOutput = 0.0;
double PivotToAngleCommand::rMaxAbsDiffError = 0.0;
double PivotToAngleCommand::rMaxAbsError = 0.0;
double PivotToAngleCommand::rMaxAbsITerm = 0.0;
double PivotToAngleCommand::rTimeLimit = 0.0;

PIDConfig* PivotToAngleCommand::CreateRPIDConfig(){
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

double PivotToAngleCommand::GetAccumulatedYaw() {
	lastYaw = currYaw;
	currYaw = robot->GetNavXYaw();
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
		LOG(robot, "END YAW", robot->GetNavXYaw());
	} else {
		double output = rPID->Update(GetAccumulatedYaw());
		DO_PERIODIC(5, printf("My Yaw: %f\n", GetAccumulatedYaw()));
		DO_PERIODIC(5, printf("Desired Yaw: %f\n", desiredR));
		DO_PERIODIC(5, printf("Output: %f\n", output));
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

#endif

/*
 * DriveStraight Command
 * basic driving forward command with rPID.
 */

DriveStraightCommand::DriveStraightCommand(RobotModel* myRobot, double myDesiredDis) {
	robot = myRobot;
	desiredDis = myDesiredDis;
	isDone = false;
	accumulatedYaw = 0.0;
	lastYaw = 0.0;
	currYaw = 0.0;
	deltaYaw = 0.0;
	initialR = 0.0;
	initialDis = 0.0;
}

void DriveStraightCommand::Init() {
	disPIDConfig = CreateDisPIDConfig();
	rPIDConfig = CreateRPIDConfig();
	initialDis = (robot->GetLeftEncoderVal() + robot->GetRightEncoderVal()) / 2.0;
	initialR = GetAccumulatedYaw();
	desiredR = 0.0;
	disPID = new PIDControlLoop(disPIDConfig);
	rPID = new PIDControlLoop(rPIDConfig);
	disPID->Init(initialDis, initialDis + desiredDis);
	rPID->Init(initialR, initialR + desiredR);
	LOG(robot, "START DIS", initialDis);
	LOG(robot, "START YAW", initialR);
}

double DriveStraightCommand::disPFac = 0.0;
double DriveStraightCommand::disIFac = 0.0;
double DriveStraightCommand::disDFac = 0.0;
double DriveStraightCommand::disDesiredAccuracy = 0.0;
double DriveStraightCommand::disMaxAbsOutput = 0.0;
double DriveStraightCommand::disMaxAbsError = 0.0;
double DriveStraightCommand::disMaxAbsDiffError = 0.0;
double DriveStraightCommand::disMaxAbsITerm = 0.0;
double DriveStraightCommand::disTimeLimit = 0.0;

double DriveStraightCommand::rPFac = 0.0;
double DriveStraightCommand::rIFac = 0.0;
double DriveStraightCommand::rDFac = 0.0;
double DriveStraightCommand::rDesiredAccuracy = 0.0;
double DriveStraightCommand::rMaxAbsOutput = 0.0;
double DriveStraightCommand::rMaxAbsError = 0.0;
double DriveStraightCommand::rMaxAbsDiffError = 0.0;
double DriveStraightCommand::rMaxAbsITerm = 0.0;
double DriveStraightCommand::rTimeLimit = 0.0;

PIDConfig* DriveStraightCommand::CreateDisPIDConfig() {
	PIDConfig* dPIDConfig = new PIDConfig();
	dPIDConfig->pFac = disPFac;
	dPIDConfig->iFac = disIFac;
	dPIDConfig->dFac = disDFac;
	dPIDConfig->desiredAccuracy = disDesiredAccuracy;
	dPIDConfig->maxAbsOutput = disMaxAbsOutput;
	dPIDConfig->maxAbsError = disMaxAbsError;
	dPIDConfig->maxAbsDiffError = disMaxAbsDiffError;
	dPIDConfig->maxAbsITerm = disMaxAbsITerm;
	dPIDConfig->timeLimit = disTimeLimit;
	return dPIDConfig;
}

PIDConfig* DriveStraightCommand::CreateRPIDConfig() {
	PIDConfig* rPIDConfig = new PIDConfig();
	rPIDConfig->pFac = rPFac;
	rPIDConfig->iFac = rIFac;
	rPIDConfig->dFac = rDFac;
	rPIDConfig->desiredAccuracy = rDesiredAccuracy;
	rPIDConfig->maxAbsOutput = rMaxAbsOutput;
	rPIDConfig->maxAbsError = rMaxAbsError;
	rPIDConfig->maxAbsDiffError = rMaxAbsDiffError;
	rPIDConfig->maxAbsITerm = rMaxAbsITerm;
	rPIDConfig->timeLimit = rTimeLimit;
	return rPIDConfig;
}

double DriveStraightCommand::GetAccumulatedYaw() {
	lastYaw = currYaw;
	currYaw = robot->GetNavXYaw();
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

void DriveStraightCommand::Update(double currTimeSec, double deltaTimeSec) {
	double currDis = (robot->GetLeftEncoderVal() + robot->GetRightEncoderVal()) / 2.0;
	bool disPIDDone = disPID->ControlLoopDone(currDis,deltaTimeSec);
	if (disPIDDone) {
		isDone = true;
		robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
		LOG(robot, "END DIS", currDis);
		LOG(robot, "END YAW", GetAccumulatedYaw());
	} else {
		double disOutput = disPID->Update(currDis);
		double rOutput = rPID->Update(GetAccumulatedYaw());
		LOG(robot, "Current Distance", currDis);
		LOG(robot, "Current Yaw", GetAccumulatedYaw());
		LOG(robot, "Distance Output", disOutput);
		LOG(robot, "R Output", rOutput);

		double leftOutput = disOutput + rOutput;
		double rightOutput = disOutput - rOutput;

		if (fmax(fabs(leftOutput), fabs(rightOutput)) > 1.0) {
			leftOutput = leftOutput / (fmax(fabs(leftOutput), fabs(rOutput)));
			rightOutput = rightOutput / (fmax(fabs(leftOutput), fabs(rOutput)));
		}

		LOG(robot, "Left Output", leftOutput);
		LOG(robot, "Right Output", rightOutput);

		robot->SetWheelSpeed(RobotModel::kLeftWheels, leftOutput);
		robot->SetWheelSpeed(RobotModel::kRightWheels, rightOutput);
	}
}

bool DriveStraightCommand::IsDone() {
	return isDone;
}

/*
 * DriveFromCamera Command
 *
 * Notes: in Camera Command, setting x and y in Camera Controller. Get x and y in Camera Controller and do a Drive Command
 * accordingly. Needs flushing, this is the basic framework.
 */

DriveFromCameraCommand::DriveFromCameraCommand(RobotModel* myRobot, CameraController* myCamera) {
	robot = myRobot;
	camera = myCamera;
	isDone = false;
}

void DriveFromCameraCommand::Init() {

}

void DriveFromCameraCommand::Update(double currTimeSec, double deltaTimeSec) {

}

bool DriveFromCameraCommand::IsDone() {
	return isDone;
}

/*
 * Camera Command
 *
 * checked by Katy, command works -- need to print less once the camera is done because the
 * netconsole print queue is filling and stalling, camera might need help still
 */

CameraCommand::CameraCommand(CameraController* myCamera) {
	camera = myCamera;
	isDone = false;
	x = 0;
	y = 0;
	sumx = 0;
	sumy = 0;
	numIterations = 5;
	iterationCounter = 0;
	waitTime = 0.1;
	lastReadTime = 0;
}

void CameraCommand::Init() {
	camera->Reset();
}

void CameraCommand::Update(double currTimeSec, double deltaTimeSec) {
	if (iterationCounter == 0) {
		camera->CalculateDistanceWithAngles();
		sumx += camera->GetX();
		sumy += camera->GetY();
		lastReadTime = currTimeSec;
		iterationCounter++;
		camera->Reset();
	} else if (iterationCounter < numIterations){
		if ((currTimeSec - lastReadTime) >= waitTime) {
			camera->CalculateDistanceWithAngles();
			sumx += camera->GetX();
			sumy += camera->GetY();
			lastReadTime = currTimeSec;
			iterationCounter++;
			camera->Reset();
		}
	} else {
		isDone = true;
		x = sumx / numIterations;
		y = sumy / numIterations;
		camera->SetX(x);
		camera->SetY(y);
	}
}

bool CameraCommand::IsDone() {
	return isDone;
}
