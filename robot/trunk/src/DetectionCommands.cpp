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





