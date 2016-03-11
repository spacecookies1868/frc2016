/*
 * DetectionCommands.cpp
 *
 *  Created on: Feb 25, 2016
 *      Author: origa_000
 */

#include "DetectionCommands.h"
#include <math.h>
#include "ini.h"
#include <iostream>
#include <string>
#include "Logger.h"


#define PI 3.14159265358979

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

CameraDetectionCommand::CameraDetectionCommand(CameraController* myCamera) {
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

void CameraDetectionCommand::Init() {
	camera->Reset();
}

void CameraDetectionCommand::Update(double currTimeSec, double deltaTimeSec) {
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
	}
}

bool CameraDetectionCommand::IsDone() {
	return isDone;
}

double CameraDetectionCommand::GetX() {
	return x;
}

double CameraDetectionCommand::GetY() {
	return y;
}

UltrasonicDetectionCommand::UltrasonicDetectionCommand() {
	x = 0.0;
	y = 0.0;
	isDone = false;
}

void UltrasonicDetectionCommand::Init() {

}

void UltrasonicDetectionCommand::Update(double currTimeSec, double deltaTimeSec) {

}

bool UltrasonicDetectionCommand::IsDone() {
	return isDone;
}

double UltrasonicDetectionCommand::GetX() {
	return x;
}

double UltrasonicDetectionCommand::GetY() {
	return y;
}

UltrasonicCommand::UltrasonicCommand(RobotModel* myRobot, SuperstructureController* mySuperstructure) {
	robot = myRobot;
	superstructure = mySuperstructure;
	isDone = false;
	currState = kInit;
	nextState = kInit;
	uDetect = new UltrasonicDetectionCommand(); //change parameters as necessary
	forwardDrive = new DriveStraightCommand(robot, 2.8);
	backingDrive = new DriveStraightCommand(robot, -3.0);
}

void UltrasonicCommand::Init() {
	uDetect->Init();
}

void UltrasonicCommand::Update(double currTimeSec, double deltaTimeSec) {
	switch (currState) {
	case (kInit): {
		nextState = kDetecting;
		break;
	}
	case (kDetecting) : {
		if (uDetect->IsDone()) {
			nextState = kNearingInit;
		} else {
			uDetect->Update(currTimeSec, deltaTimeSec);
		}
		break;
	}
	case (kNearingInit) : {
		nearingDrive = new CurveCommand(robot, uDetect->GetX(), uDetect->GetY() - 3.0);
		nearingDrive->Init();
		nearingDrive->radiusPIDConfig->dFac = nearingDrive->radiusPIDConfig->dFac * 2.0;
		nearingDrive->radiusPIDConfig->desiredAccuracy = 0.1;
		nearingDrive->anglePIDConfig->desiredAccuracy = 2.0;
		break;
	}
	case (kNearing) : {
		if (nearingDrive->IsDone()) {
			nextState = kIntakingInit;
		} else {
			nearingDrive->Update(currTimeSec, deltaTimeSec);
		}
		break;
	}
	case (kIntakingInit) : {
		superstructure->SetAutoIntakeMotorForward(true);
		superstructure->Update(currTimeSec, deltaTimeSec);
		superstructure->Update(currTimeSec, deltaTimeSec);
		forwardDrive->Init();
		forwardDrive->disPIDConfig->dFac = forwardDrive->disPIDConfig->dFac * 2.0;
		forwardDrive->disPIDConfig->desiredAccuracy = 0.1;
		nextState = kForward;
		break;
	}
	case (kForward) : {
		if (forwardDrive->IsDone()) { //add an or with the superstructure boolean for intaking Done
			nextState = kBacking;
			backingDrive->Init();
		} else {
			forwardDrive->Update(currTimeSec, deltaTimeSec);
		}
		break;
	}
	case (kBacking) : {
		if (backingDrive->IsDone()) {
			nextState = kDone;
		} else {
			backingDrive->Update(currTimeSec, deltaTimeSec);
		}
		break;
	}
	case (kDone) : {
		isDone = true;
		robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
		superstructure->SetAutoIntakeMotorForward(false);
		superstructure->Update(currTimeSec, deltaTimeSec);
		break;
	}
	}
	currState = nextState;
}

bool UltrasonicCommand::IsDone() {
	return isDone;
}


