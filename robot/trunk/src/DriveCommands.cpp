/*
 * DriveCommands.cpp
 */
#include "DriveCommands.h"
#include <math.h>
#include "ini.h"
#include <iostream>
#include <string>
#include "Logger.h"

#define PI 3.14159265358979


/*
 * Pivot Command
 *
 * hopefully positive desiredR makes turn counterclockwise
 * Pivots according to the robot's perspective
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
#if USE_NAVX
	LOG(robot, "START YAW", robot->GetNavXYaw());
#else
	LOG(robot, "START YAW", "USE_NAVX is false");
#endif
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

/*
 * Becuase the navx returns values from 180 to -180 the accumulated yaw
 * must be calculated to avoid the issue of the sudden change from 180 to -180
 */
double PivotCommand::GetAccumulatedYaw() {
	lastYaw = currYaw;
#if USE_NAVX
	currYaw = robot->GetNavXYaw();
#else
	currYaw = 0.0;
#endif
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

void PivotCommand::SetDesiredR(double desiredRotation) {
	desiredR = desiredRotation;
}

void PivotCommand::Update(double currTimeSec, double deltaTimeSec) {
#if USE_NAVX
	bool pidDone = rPID->ControlLoopDone(GetAccumulatedYaw(), deltaTimeSec);
#else
	bool pidDone = true;
#endif
	if (pidDone) {
		isDone = true;
		robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
#if USE_NAVX
		LOG(robot, "END YAW", robot->GetNavXYaw());
#endif
	} else {
		double output = rPID->Update(GetAccumulatedYaw());
		DO_PERIODIC(5, printf("My Yaw: %f\n", GetAccumulatedYaw()));
		DO_PERIODIC(5, printf("Desired Yaw: %f\n", desiredR));
		DO_PERIODIC(5, printf("Output: %f\n", output));
		DO_PERIODIC(1, LOG(robot, "Accumulated Yaw", GetAccumulatedYaw()));
#if USE_NAVX
		DO_PERIODIC(1, LOG(robot, "Yaw", robot->GetNavXYaw()));
#endif
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
 * Pivots according to the perspective of the field.
 */

PivotToAngleCommand::PivotToAngleCommand(RobotModel* myRobot, double myDesiredR) {
	/*
	 * Desired R is coming in as a 0 to 360 degree measure.
	 */
	robot = myRobot;
	desiredR = myDesiredR;
	isDone = false;
	lastYaw = 0.0;
	deltaYaw = 0.0;
	currYaw = 0.0;
	accumulatedYaw = 0.0;
}

void PivotToAngleCommand::Init() {
	rPIDConfig = CreateRPIDConfig();
	rPID = new PIDControlLoop(rPIDConfig);
#if USE_NAVX
	currYaw = robot->GetNavXYaw();
#endif
	initialR = GetAccumulatedYaw();
	double desiredChange = CalculateDesiredChange(desiredR);
	LOG(robot, "initialR", initialR);
	LOG(robot, "desiredR", desiredR);
	LOG(robot, "desiredChange", desiredChange);
	rPID->Init(initialR, initialR + desiredChange);
#if USE_NAVX
	LOG(robot, "START YAW", robot->GetNavXYaw());
#else
	LOG(robot, "START YAW", "USE_NAVX is false");
#endif
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
#if USE_NAVX
	currYaw = robot->GetNavXYaw();
#else
	currYaw = 0.0;
#endif
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
	LOG(robot, "my current angle", GetAccumulatedYaw());
#if USE_NAVX
	bool pidDone = rPID->ControlLoopDone(GetAccumulatedYaw(), deltaTimeSec);
#else
	bool pidDone = false;
#endif
	if (pidDone) {
		isDone = true;
		robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
#if USE_NAVX
		LOG(robot, "END YAW", robot->GetNavXYaw());
#endif
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

double PivotToAngleCommand::CalculateDesiredChange(double myDesired) {
	/*
	 * Using the current angle and the desired angle, the desired change must take
	 * into account of the direction the robot needs to turn to be efficiently
	 * at the right position
	 */
	double normalizedInitial = fmod(currYaw, 360.0);
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
	return change;
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
#if USE_NAVX
	currYaw = robot->GetNavXYaw();
#else
	currYaw = 0.0;
#endif
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

		printf("Current Distance %f\n", currDis);
		LOG(robot, "Current Distance", currDis);
		LOG(robot, "Current Yaw", GetAccumulatedYaw());
		printf("Current Yaw %f\n", GetAccumulatedYaw());
		LOG(robot, "Distance Output", disOutput);
		printf("Distance Output %f\n", disOutput);
		LOG(robot, "R Output", rOutput);
		printf("Angle Output %f\n", rOutput);

		double leftOutput = disOutput + rOutput;
		double rightOutput = disOutput - rOutput;

		//to make sure the output does not exceed 1.0
		double maxOutput = fmax(fabs(leftOutput), fabs(rightOutput));
		if (maxOutput > 1.0) {
			leftOutput = leftOutput / maxOutput;
			rightOutput = rightOutput / maxOutput;
		}

		LOG(robot, "Left Output", leftOutput);
		LOG(robot, "Right Output", rightOutput);
		printf("Left Output %f\n", leftOutput);
		printf("Right Output %f\n", rightOutput);

		robot->SetWheelSpeed(RobotModel::kLeftWheels, leftOutput);
		robot->SetWheelSpeed(RobotModel::kRightWheels, rightOutput);
	}
}

bool DriveStraightCommand::IsDone() {
	return isDone;

}

/*
 * Curve Command
 * Driving to a given (x,y) coordinate
 */

CurveCommand::CurveCommand(RobotModel* myRobot, double myDesiredX, double myDesiredY) {
	robot = myRobot;
	desiredX = myDesiredX;
	desiredY = myDesiredY;
	initialX = 0.0;
	initialY = 0.0;
	lastX = 0.0;
	lastY = 0.0;
	lastLeft = 0.0;
	lastRight = 0.0;
	initialLeft = 0.0;
	initialRight = 0.0;
	lastAccumulatedYaw = 0.0;

	isDone = false;
	lastYaw = 0.0;
	currYaw = 0.0;
	deltaYaw = 0.0;
	accumulatedYaw = 0.0;


}

void CurveCommand::Init() {
	radiusPIDConfig = CreateRadiusPIDConfig();
	anglePIDConfig = CreateAnglePIDConfig();
	initialYaw = GetAccumulatedYaw();
	initialRadius = 0.0;
	initialLeft = robot->GetLeftEncoderVal();
	initialRight = robot->GetRightEncoderVal();
	desiredRadius = GetSign(desiredY) * sqrt(desiredX*desiredX + desiredY*desiredY);
	radiusPID = new PIDControlLoop(radiusPIDConfig);
//	radiusPID->Init(initialRadius, desiredRadius);
	radiusPID->Init(initialRadius-desiredRadius, initialRadius);
	desiredAngle = atan(desiredX/desiredY) * 360.0 / (2 * PI);
	anglePID = new PIDControlLoop(anglePIDConfig);
	anglePID->Init(initialYaw, initialYaw + desiredAngle);
}

void CurveCommand::Update(double currTimeSec, double deltaTimeSec) {
	currLeft = robot->GetLeftEncoderVal() - initialLeft;
	currRight = robot->GetRightEncoderVal() - initialRight;

	angle = GetAccumulatedYaw();
	double x = CalculateX();
	double y = CalculateY();

//	double radius = GetSign(y) * sqrt((x - initialX)*(x - initialX) + (y - initialY) * (y - initialY));
	double radius = -GetSign(desiredY - y) * sqrt((desiredX - x)*(desiredX - x) + (desiredY - y)*(desiredY - y));
	bool radiusPIDDone = radiusPID->ControlLoopDone(radius, deltaTimeSec);
	//bool anglePIDDone = anglePID->ControlLoopDone(accumulatedYaw, deltaTimeSec);
	bool anglePIDDone = true;
	printf("Radius PID Done? %i\n", radiusPIDDone);
	printf("Angle PID Done? %i\n", anglePIDDone);

	if (radiusPIDDone && anglePIDDone) {
		isDone = true;
		robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
	} else {
		double radiusOutput = radiusPID->Update(radius);
		double newDesiredAngle = atan((desiredX - x)/(desiredY - y)) * 360.0 / (2*PI);
		double angleOutput = anglePID->Update(angle - initialYaw, newDesiredAngle);

		double leftOutput = radiusOutput + angleOutput;
		double rightOutput = radiusOutput - angleOutput;

		double maxOutput = fmax(fabs(leftOutput), fabs(rightOutput));
		if (maxOutput > 1.0) {
			leftOutput = leftOutput / maxOutput;
			rightOutput = rightOutput / maxOutput;
		}

		LOG(robot, "Curr X", x);
		LOG(robot, "Curr Y", y);
		LOG(robot, "Radius", radius);
		LOG(robot, "Desired Radius", desiredRadius)
		LOG(robot, "New Desired Angle", newDesiredAngle);
		LOG(robot, "Radius Output", radiusOutput);
		LOG(robot, "Angle Output", angleOutput);
		LOG(robot, "Left Output", leftOutput);
		LOG(robot, "Right Output", rightOutput);
		printf("Current X %f\n", x);
		printf("Current Y %f\n", y);
		printf("radius %f\n", radius);
		printf("desired radius %f\n", desiredRadius);
		printf("new Desired Angle %f\n", newDesiredAngle);
		printf("Radius Output %f\n", radiusOutput);
		printf("Angle Output %f\n", angleOutput);
		printf("Left Output %f\n", leftOutput);
		printf("Right Output %f\n",rightOutput);

		robot->SetWheelSpeed(RobotModel::kLeftWheels, leftOutput);
		robot->SetWheelSpeed(RobotModel::kRightWheels, rightOutput);
	}
	lastAccumulatedYaw = angle;
	lastLeft = currLeft;
	lastRight = currRight;

}

bool CurveCommand::IsDone() {
	return isDone;
}

double CurveCommand::radiusPFac = 0.0;
double CurveCommand::radiusIFac = 0.0;
double CurveCommand::radiusDFac = 0.0;
double CurveCommand::radiusDesiredAccuracy = 0.0;
double CurveCommand::radiusMaxAbsOutput = 0.0;
double CurveCommand::radiusMaxAbsError = 0.0;
double CurveCommand::radiusMaxAbsDiffError = 0.0;
double CurveCommand::radiusMaxAbsITerm = 0.0;
double CurveCommand::radiusTimeLimit = 0.0;

double CurveCommand::anglePFac = 0.0;
double CurveCommand::angleIFac = 0.0;
double CurveCommand::angleDFac = 0.0;
double CurveCommand::angleDesiredAccuracy = 0.0;
double CurveCommand::angleMaxAbsOutput = 0.0;
double CurveCommand::angleMaxAbsError = 0.0;
double CurveCommand::angleMaxAbsDiffError = 0.0;
double CurveCommand::angleMaxAbsITerm = 0.0;
double CurveCommand::angleTimeLimit = 0.0;

PIDConfig* CurveCommand::CreateRadiusPIDConfig() {
	PIDConfig* temp = new PIDConfig();
	temp->pFac = radiusPFac;
	temp->iFac = radiusIFac;
	temp->dFac = radiusDFac;
	temp->desiredAccuracy = radiusDesiredAccuracy;
	temp->maxAbsOutput = radiusMaxAbsOutput;
	temp->maxAbsError = radiusMaxAbsError;
	temp->maxAbsDiffError = radiusMaxAbsDiffError;
	temp->maxAbsITerm = radiusMaxAbsITerm;
	temp->timeLimit = radiusTimeLimit;
	return temp;
}

PIDConfig* CurveCommand::CreateAnglePIDConfig() {
	PIDConfig* temp = new PIDConfig();
	temp->pFac = anglePFac;
	temp->iFac = angleIFac;
	temp->dFac = angleDFac;
	temp->desiredAccuracy = angleDesiredAccuracy;
	temp->maxAbsOutput = angleMaxAbsOutput;
	temp->maxAbsError = angleMaxAbsError;
	temp->maxAbsDiffError = angleMaxAbsDiffError;
	temp->maxAbsITerm = angleMaxAbsITerm;
	temp->timeLimit = angleTimeLimit;
	return temp;
}

double CurveCommand::GetAccumulatedYaw() {
	lastYaw = currYaw;
#if USE_NAVX
	currYaw = robot->GetNavXYaw();
#else
	currYaw = 0.0;
#endif
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

double CurveCommand::CalculateX() {
	LOG(robot, "CALCULATING X -------", 0.0);
	printf("CALCULATING X ---------------------- \n");
	LOG(robot, "CurrLeft", currLeft);
	LOG(robot, "CurrRight", currRight);
	printf("CurrLeft %f\n", currLeft);
	printf("CurrRight %f\n", currRight);
	LOG(robot, "LastLeft", lastLeft);
	LOG(robot, "LastRight", lastRight);
	double deltaLeft = currLeft - lastLeft;
	double deltaRight = currRight - lastRight;
	double deltaS = (deltaLeft + deltaRight)/2.0;
	LOG(robot, "DeltaS", deltaS);
	printf("Delta S %f\n", deltaS);


	LOG(robot, "accumulateYaw", angle);
	printf("accumulateYaw %f\n", angle);
	LOG(robot, "Angle - initial", angle-initialYaw);
	double currX = lastX + deltaS * sin(((angle - initialYaw))*2*PI/360.0);
	lastX = currX;
	LOG(robot, "CurrentX", currX);
	printf("Current X %f\n", currX);
	return currX;
}

double CurveCommand::CalculateY() {
	LOG(robot, "CALCULATING Y-------", 0.0);
	printf("CALCULATING Y----------------------\n");
	LOG(robot, "currLeft", currLeft);
	LOG(robot, "currRight", currRight);
	printf("CurrLeft %f\n", currLeft);
	printf("CurrRight %f\n", currRight);
	LOG(robot, "LastLeft", lastLeft);
	LOG(robot, "LastRight", lastRight);

	double deltaLeft = currLeft - lastLeft;
	double deltaRight = currRight - lastRight;
	double deltaS = (deltaLeft + deltaRight)/2.0;
	LOG(robot, "deltaS", deltaS);
	printf("delta S %f\n", deltaS);


	LOG(robot, "currentYaw", angle);
	LOG(robot, "Angle - initial", angle - initialYaw);
	printf("Current Yaw %f\n", angle);
	printf("Delta Yaw %f\n", deltaYaw);
	double currY = lastY + deltaS * cos(((angle - initialYaw))*2*PI/360.0);
	lastY = currY;
	lastAccumulatedYaw = angle;
	LOG(robot, "CurrentY", currY);
	printf("Current Y %f\n", currY);
	return currY;
}

double CurveCommand::GetSign(double n) {
	if (n >= 0) {
		return 1.0;
	} else {
		return -1.0;
	}
}

/*
 * Defense Command
 * Lines up to the defense and drives over it
 */

DefenseCommand::DefenseCommand(RobotModel* myRobot, SuperstructureController* mySuperstructure, uint32_t myDefense,
		bool forward, bool lineUp, bool defenseManipFirst) {
	robot = myRobot;
	superstructure = mySuperstructure;
	defense = myDefense;
	isDone = false;
	this->forward = forward;
	this->lineUp = lineUp;
	this->defenseManipFirst = defenseManipFirst;

	//defense: low bar
	if (forward) {
		if (lineUp) {
			lowBarDriveUp = new DriveStraightCommand(robot, -2.0);
		} else {
			lowBarDriveUp = new DriveStraightCommand(robot, 0.0);
		}
		lowBarDriving = new DriveStraightCommand(robot, -11.0);
	} else {
		if (lineUp) {
			lowBarDriveUp = new DriveStraightCommand(robot, 2.0);
		} else {
			lowBarDriveUp = new DriveStraightCommand(robot, 0.0);
		}
		lowBarDriving = new DriveStraightCommand(robot, 7.0);
	}
	lowBarDefenseDown = new DefenseManipPosCommand(superstructure, true);
	lowBarIntakeDown = new IntakePositionCommand(superstructure, true);

	//defense: portcullis
	if (lineUp) {
		portcullisDriveUp = new DriveStraightCommand(robot, 5.75);
	} else {
		portcullisDriveUp = new DriveStraightCommand(robot, 0.0);
	}
	portcullisDriving = new DriveStraightCommand(robot, 6.3);
//	portcullisDriving = new DriveStraightCommand(robot, 0.0);
	portcullisDefenseUp = new DefenseManipPosCommand(superstructure, false);
	portcullisDefenseDown = new DefenseManipPosCommand(superstructure, true);
	portcullisIntakeDown = new IntakePositionCommand(superstructure, true);
	portcullisWaitTimeDone = false;
	portcullisWaiting = 0.0;
	portcullisDriveTimeOut = 3.5;

	//defense: cheval de frise
	if (lineUp) {
		chevalDeFriseDriveUp = new DriveStraightCommand(robot, 4.4);
	} else {
		chevalDeFriseDriveUp = new DriveStraightCommand(robot, 0.0);
	}
	chevalDeFriseDriving = new DriveStraightCommand(robot, 6.0);
	chevalDeFriseStay = new DriveStraightCommand(robot, 0.0);
//	chevalDeFriseDriving = new DriveStraightCommand(robot, 0.0);
	chevalDeFriseDefenseUp = new DefenseManipPosCommand(superstructure, false);
	chevalDeFriseDefenseDown = new DefenseManipPosCommand(superstructure, true);
	chevalDeFriseDefenseUpInitted = false;
	chevalDeFriseWaitTimeDone = false;
	chevalDeFriseWaiting = 0.0;
	chevalDeFriseTimeOut = 2.0;
	chevalDeFriseTime = 0.0;
	chevalDeFriseFirstTime = true;
	chevalDeFriseInitTime = 0.0;

	//defense: ramparts
	if (forward) {
		if (lineUp) {
			rampartsDriveStraight = new DriveStraightCommand(robot, 3.0);
		} else {
			rampartsDriveStraight = new DriveStraightCommand(robot, 0.0);
		}
		rampartsDriveOver = new DriveStraightCommand(robot, 13.0);
		rampartsPivotToAngle = new PivotToAngleCommand(robot, 330);
	} else {
		if (lineUp) {
			rampartsDriveStraight = new DriveStraightCommand(robot, -2.0);
		} else {
			rampartsDriveStraight = new DriveStraightCommand(robot, 0.0);
		}
		rampartsPivotToAngle = new PivotToAngleCommand(robot, 330.0);
		rampartsDriveOver = new DriveStraightCommand(robot, -7.0);
	}
	currRampartsState = kRampartsBeforeRamp;
	nextRampartsState = kRampartsBeforeRamp;

	//defense: moat
	if (forward) {
		if (lineUp) {
			if (defenseManipFirst) {
				hardCodeMoat = new DriveStraightCommand(robot, 16.0);
			} else {
				hardCodeMoat = new DriveStraightCommand(robot, -16.0);
			}
		} else {
			if (defenseManipFirst) {
				hardCodeMoat = new DriveStraightCommand(robot, 16.0);
			} else {
				hardCodeMoat = new DriveStraightCommand(robot, -16.0);
			}
		}
	} else {
		if (defenseManipFirst) {
			hardCodeMoat = new DriveStraightCommand(robot, 10.0);
		} else {
			hardCodeMoat = new DriveStraightCommand(robot, -10.0);
		}
	}

	//defense: rock wall
	if (forward) {
		if (lineUp) {
			if (defenseManipFirst) {
				hardCodeRockWall = new DriveStraightCommand(robot, 17.0);
			} else {
				hardCodeRockWall = new DriveStraightCommand(robot, -17.0);
			}
		} else {
			if (defenseManipFirst) {
				hardCodeRockWall = new DriveStraightCommand(robot, 11.0);
			} else {
				hardCodeRockWall = new DriveStraightCommand(robot, -11.0);
			}
		}
	} else {
		if (defenseManipFirst) {
			hardCodeRockWall = new DriveStraightCommand(robot, 11.0);
		} else {
			hardCodeRockWall = new DriveStraightCommand(robot, -11.0);
		}
	}

	//defense: rough terrain
	if (forward) {
		if (lineUp) {
			hardCodeRoughTerrain = new DriveStraightCommand(robot, 14.0);
			// when the rough terrain gets reversed
			// hardCodeRoughTerrain = new DriveStraightCommand(robot, -14.0);
		} else {
			hardCodeRoughTerrain = new DriveStraightCommand(robot, 8.0);
			// when the rough terrain get reversed
			// hardCodeRoughTerrain = new DriveStraightCommand(robot, -8.0);
		}
	} else {
		hardCodeRoughTerrain = new DriveStraightCommand(robot, -8.0);
		// when the rough terrain gets reversed
		// hardCodeRoughtTerrain = new DriveStraightCommand(robot, 8.0);
	}
}

void DefenseCommand::Init() {
	switch (defense) {
	case (LowBar): {
		printf("Low Bar Init \n");
		lowBarDriveUp->Init();
		break;
	}
	case (Portcullis): {
		printf("Portcullis Init \n");
		portcullisDriveUp->Init();
		portcullisDriveUp->disPIDConfig->pFac = 0.3;
		portcullisDriveUp->disPIDConfig->iFac = 0.001;
		portcullisDriveUp->disPIDConfig->dFac = 7.7;
		portcullisDriveUp->disPIDConfig->desiredAccuracy = 0.1;
		portcullisDriveUp->disPIDConfig->maxAbsOutput = 0.6;
		portcullisDriveUp->disPIDConfig->maxAbsITerm = 0.5;
		portcullisDriveUp->disPIDConfig->timeLimit = 0.125;
		portcullisDefenseDown->Init();
		break;
	}
	case (ChevalDeFrise): {
		printf("Cheval de Frise Init \n");
		chevalDeFriseDriveUp->Init();
		chevalDeFriseDriveUp->disPIDConfig->pFac = 0.3;
		//chevalDeFriseDriveUp->disPIDConfig->pFac = 0.0;
		chevalDeFriseDriveUp->disPIDConfig->iFac = 0.001;
		//chevalDeFriseDriveUp->disPIDConfig->iFac = 0.000;
		chevalDeFriseDriveUp->disPIDConfig->dFac = 6.7;
		chevalDeFriseDriveUp->disPIDConfig->desiredAccuracy = 0.15;
		chevalDeFriseDriveUp->disPIDConfig->maxAbsOutput = 0.5;
		chevalDeFriseDriveUp->disPIDConfig->maxAbsITerm = 0.5;
		chevalDeFriseDriveUp->disPIDConfig->timeLimit = 0.125;
		chevalDeFriseDefenseDown->Init();
		break;
	}
	case (Ramparts): {
		printf("Ramparts Init \n");
		rampartsDriveStraight->disPFac = 0.75;
		rampartsDriveStraight->disIFac = 0.005;
		rampartsDriveStraight->disDFac = 4.0;
		rampartsDriveStraight->Init(); //drive straight for 4 feet until bottom of ramp
		nextRampartsState = kRampartsBeforeRamp;
		break;
	}
	case (Moat): {
		printf("Moat Init \n");
		hardCodeMoat->Init();
		break;
	}
	case (SallyPort): {
		printf("Sally Port Init \n");
		break;
	}
	case (Drawbridge): {
		printf("Drawbridge Init \n");
		break;
	}
	case (RockWall): {
		printf("Rock Wall Init \n");
		hardCodeRockWall->Init();
		break;
	}
	case (RoughTerrain): {
		printf("Rough Terrain Init \n");
		hardCodeRoughTerrain->Init();
		break;
	}
	}
}

void DefenseCommand::Update(double currTimeSec, double deltaTimeSec) {
	switch (defense) {
	case (LowBar): {
		printf("Low Bar Update \n");
		if (!lowBarDriveUp->IsDone()) {
			lowBarDriveUp->Update(currTimeSec, deltaTimeSec);
			lowBarDefenseDown->Init();
			lowBarIntakeDown->Init();
		} else if (!lowBarDefenseDown->IsDone() || !lowBarDefenseDown->IsDone()) {
			lowBarDefenseDown->Update(currTimeSec, deltaTimeSec);
			lowBarIntakeDown->Update(currTimeSec, deltaTimeSec);
			lowBarDriving->Init();
			lowBarDriving->disPIDConfig->maxAbsOutput = 0.5;
		} else if (!lowBarDriving->IsDone()) {
			lowBarDriving->Update(currTimeSec, deltaTimeSec);
		} else {
			isDone = true;
			robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
		}
		break;
	}
	case (Portcullis): {
		printf("Portcullis Update \n");
		isDone = false;
		if (!portcullisDefenseDown->IsDone()) {
			portcullisDefenseDown->Update(currTimeSec, deltaTimeSec);
			portcullisInitTime = currTimeSec;
		} else if (!portcullisDriveUp->IsDone() && (currTimeSec - portcullisInitTime) < portcullisDriveTimeOut) {
			portcullisDriveUp->Update(currTimeSec, deltaTimeSec);
			portcullisDefenseUp->Init();
			portcullisIntakeDown->Init();
		} else if (!portcullisDefenseUp->IsDone() && !portcullisIntakeDown->IsDone()) {
			portcullisDefenseUp->Update(currTimeSec, deltaTimeSec);
			portcullisIntakeDown->Update(currTimeSec, deltaTimeSec);
			portcullisDriving->Init();
			portcullisDriving->disPIDConfig->maxAbsOutput = 0.5;
		} else if (!portcullisDriving->IsDone()) {
			if (portcullisWaitTimeDone) {
				portcullisDriving->Update(currTimeSec, deltaTimeSec);
			} else {
				portcullisWaiting += deltaTimeSec;
				portcullisWaitTimeDone = portcullisWaiting > 1.0;
			}
		}
		isDone = (portcullisDriving->IsDone()) && (portcullisDefenseUp->IsDone());
		break;
	}
	case (ChevalDeFrise): {
		printf("Cheval de Frise Update \n");
		isDone = false;
		if (chevalDeFriseFirstTime) {
			chevalDeFriseTime = currTimeSec;
			chevalDeFriseInitTime = currTimeSec;
			chevalDeFriseFirstTime = false;
		} else if (!chevalDeFriseDriveUp->IsDone() && (chevalDeFriseTime - chevalDeFriseInitTime) < chevalDeFriseTimeOut) {
			chevalDeFriseDriveUp->Update(currTimeSec, deltaTimeSec);
			chevalDeFriseTime += deltaTimeSec;
			if (chevalDeFriseTime >= chevalDeFriseTimeOut) {
				printf("TIMEOUT TIMEOUT \n");
			}
			chevalDeFriseStay->Init();
			chevalDeFriseStay->disPIDConfig->pFac = 0.5;
			chevalDeFriseStay->disPIDConfig->iFac = 0.01;
		} else if (!chevalDeFriseDefenseDown->IsDone()) {
			DUMP("Puttin my defense down now",0.0);
			chevalDeFriseDriving->Init();
			chevalDeFriseDefenseDown->Update(currTimeSec, deltaTimeSec);
			chevalDeFriseStay->Update(currTimeSec, deltaTimeSec);
		} else if (!chevalDeFriseDriving->IsDone()) {
			if (chevalDeFriseWaitTimeDone) {
				chevalDeFriseDriving->Update(currTimeSec, deltaTimeSec);
			} else {
				chevalDeFriseWaiting += deltaTimeSec;
				chevalDeFriseWaitTimeDone = chevalDeFriseWaiting > 0.5;
				chevalDeFriseStay->Update(currTimeSec, deltaTimeSec);
			}
		}
		if ((robot->GetLeftEncoderVal() + robot->GetRightEncoderVal())/2.0 > 7.0) {
			if (!chevalDeFriseDefenseUpInitted) {
				chevalDeFriseDefenseUp->Init();
				chevalDeFriseDefenseUpInitted = true;
			} else {
				chevalDeFriseDefenseUp->Update(currTimeSec, deltaTimeSec);
			}
		}
		isDone = (chevalDeFriseDriving->IsDone())
				&& (chevalDeFriseDefenseUp->IsDone());

		break;
	}
	case (Ramparts): {
		printf("Ramparts Update \n");
		isDone = false;
		switch (currRampartsState) {
		case (kRampartsBeforeRamp) : {
			if (!rampartsDriveStraight->IsDone()) {
				rampartsDriveStraight->Update(currTimeSec, deltaTimeSec);
				nextRampartsState = kRampartsBeforeRamp;
			} else {
				nextRampartsState = kRampartsTurn;
				rampartsPivotToAngle->Init();
				printf("nextRampartstate = kRampartsTurn/n");
			}
			break;
		}
		case (kRampartsTurn) : {
			if (!rampartsPivotToAngle->IsDone()) {
				rampartsPivotToAngle->Update(currTimeSec, deltaTimeSec);
				nextRampartsState = kRampartsTurn;
			} else {
				nextRampartsState = kRampartsDrive;
				rampartsDriveOver->Init();
				printf("nextRampartstate = kRampartsDone/n");
			}
			break;
		}

		case (kRampartsDrive) : {
			if (!rampartsDriveOver->IsDone()) {
				rampartsDriveOver->Update(currTimeSec, deltaTimeSec);
				nextRampartsState = kRampartsDrive;
			} else {
				nextRampartsState = kRampartsDone;
				printf("nextRampartstate = kRampartsDone/n");
			}

			break;
		}
		case (kRampartsDone) : {
			isDone = true;
			nextRampartsState = kRampartsDone;
			printf("kRampartsDone!!/n");
			break;
		}
		}
		currRampartsState = nextRampartsState;

		break;
	}
	case (Moat): {
		printf("Moat Update \n");
		isDone = hardCodeMoat->IsDone();
		if (!isDone) {
			hardCodeMoat->Update(currTimeSec, deltaTimeSec);
		}
		break;
	}
	case (SallyPort): {
		printf("Sally Port Update \n");
		isDone = true;
		break;
	}
	case (Drawbridge): {
		printf("Drawbridge Update \n");
		isDone = true;
		break;
	}
	case (RockWall): {
		printf("Rock Wall Update \n");
		isDone = hardCodeRockWall->IsDone();
		if (!isDone) {
			hardCodeRockWall->Update(currTimeSec, deltaTimeSec);
		}
		break;
	}
	case (RoughTerrain): {
		printf("Rough Terrain Update \n");
		isDone = hardCodeRoughTerrain->IsDone();
		if (!isDone) {
			hardCodeRoughTerrain->Update(currTimeSec, deltaTimeSec);
		}
		break;
	}
	}

}

bool DefenseCommand::IsDone() {
	return isDone;
}
