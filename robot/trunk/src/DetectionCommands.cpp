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
 * Camera Command
 *
 * checked by Katy, command works -- need to print less once the camera is done because the
 * netconsole print queue is filling and stalling, camera might need help still
 */

CameraDetectionCommand::CameraDetectionCommand(CameraController* myCamera, bool myOnLeft) {
	camera = myCamera;
	isDone = false;
	onLeft = myOnLeft;
	x = 0;
	y = 0;
	sumx = 0;
	sumy = 0;
	numIterations = 1;
	iterationCounter = 0;
	waitTime = 0.1;
	lastReadTime = 0;
printf("constructed! \n");
}

void CameraDetectionCommand::Init() {
	camera->Reset();
}

void CameraDetectionCommand::Update(double currTimeSec, double deltaTimeSec) {
#if USE_CAMERA //If using camera perform calculations for distance
DUMP("Camera State", "IN UPDATE");
printf("camera state, in update");
	if (iterationCounter == 0) {
		camera->CalculateDistanceWithAngles(onLeft);
		sumx += camera->GetX();
		sumy += camera->GetY();
		lastReadTime = currTimeSec;
		iterationCounter++;
		camera->Reset();
	}
	if (iterationCounter < numIterations){
		if ((currTimeSec - lastReadTime) >= waitTime) {
			camera->CalculateDistanceWithAngles(onLeft);
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
#else
	x = 0.0;
	y = 0.0;
	isDone = true;
	DUMP("Camera State", "USE_CAMERA is false");
#endif
}

bool CameraDetectionCommand::IsDone() {
	return isDone;
}

double CameraDetectionCommand::GetX() {
	//returns in feet
	return x/12.0;
}

double CameraDetectionCommand::GetY() {
	//returns in feet
	return y/12.0;
}


/*
 * DriveFromCamera Command
 *
 * Notes: in Camera Command, setting x and y in Camera Controller. Get x and y in Camera Controller and do a Drive Command
 * accordingly. Needs flushing, this is the basic framework.
 */

CameraCommand::CameraCommand(RobotModel* myRobot, CameraController* myCamera, double myDesiredX, double myDesiredY, bool myOnLeft) {
	/*
	 * desired X and Y are compared to the target
	 */
	robot = myRobot;
	camera = myCamera;
	isDone = false;
	desiredX = myDesiredX;
	desiredY = myDesiredY;
	currState = kInit;
	nextState = kInit;
	onLeft = myOnLeft;
	detect = new CameraDetectionCommand(camera, onLeft);
	if (onLeft) {
		pivoting = new PivotToAngleCommand(robot, 60.0); //ARBITRARY VALUE
	} else {
		pivoting = new PivotToAngleCommand(robot, 300.0); //ARBITRARY VALUE
	}
}

void CameraCommand::Init() {
	pivoting->Init();
	detect->Init();
}

void CameraCommand::Update(double currTimeSec, double deltaTimeSec) {
	switch (currState) {
	case (kInit): {
		nextState = kPivoting;
		break;
	 }
	case (kPivoting): {
		if (pivoting->IsDone()) {
			nextState = kDetecting;
			robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
		} else {
			pivoting->Update(currTimeSec, deltaTimeSec);
		}
		break;
	}
	case (kDetecting): {
		if (detect->IsDone()) {
			nextState = kDriveInit;
		} else {
			detect->Update(currTimeSec, deltaTimeSec);
			nextState = kDetecting;
		}
		break;
	}
	case (kDriveInit): {
		drive = new CurveCommand(robot, detect->GetX() - desiredX, detect->GetY() - desiredY);
		drive->Init();
		nextState = kDriving;
		break;
	}
	case (kDriving): {
		if (drive->IsDone()) {
			nextState = kDone;
		} else {
			drive->Update(currTimeSec, deltaTimeSec);
		}
		break;
	}
	case (kDone): {
		nextState = kDone;
		isDone = true;
		robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
		break;
	}
	}
	currState = nextState;
}

bool CameraCommand::IsDone() {
	return isDone;
}

/*
 * Ultrasonic Detection Command:
 * Detects the boulder using ultrasonic and returns (x,y) coordinates
 * The actual code has not been tested on the competition robot and therefore
 * has not been committed into the code
 */
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

/*
 * Ultrasonic Command
 * Using the (x,y) coordinates given from the ultrasonic detection command,
 * the robot  drives towards the boulder and intakes it from the center line
 * and backs out to avoid penalty
 */
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
			nextState = kDetecting;
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
			nextState = kNearing;
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
			nextState = kForward;
		}
		break;
	}
	case (kBacking) : {
		if (backingDrive->IsDone()) {
			nextState = kDone;
		} else {
			backingDrive->Update(currTimeSec, deltaTimeSec);
			nextState = kBacking;
		}
		break;
	}
	case (kDone) : {
		isDone = true;
		robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
		superstructure->SetAutoIntakeMotorForward(false);
		superstructure->Update(currTimeSec, deltaTimeSec);
		nextState = kDone;
		break;
	}
	}
	currState = nextState;
}

bool UltrasonicCommand::IsDone() {
	return isDone;
}


