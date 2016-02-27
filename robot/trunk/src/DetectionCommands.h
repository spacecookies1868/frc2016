/*
 * DetectionCommands.h
 *
 *  Created on: Feb 25, 2016
 *      Author: origa_000
 */

#ifndef SRC_DETECTIONCOMMANDS_H_
#define SRC_DETECTIONCOMMANDS_H_

#include "WPILib.h"
#include "AutoCommand.h"
#include "Debugging.h"
#include "RobotModel.h"
#include "RemoteControl.h"
#include "DriveController.h"
#include "SuperstructureController.h"
#include "ControlBoard.h"
#include <vector>
#include <string>
#include <iostream>
#include "PIDControlLoop.h"
#include "DriveController.h"
#include "CameraController.h"

using namespace std;

/*
 * NOT DONE NOT DONE NOT DONE NOT DONE NOT DONE
 */
class DriveFromCameraCommand : public SimpleAutoCommand {
public:
	DriveFromCameraCommand(RobotModel* myRobot, CameraController* myCamera);
	~DriveFromCameraCommand() {}
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();
private:
	RobotModel* robot;
	CameraController* camera;
	bool isDone;
};

/*
 * CAMERA COMMANDS YAY!
 */

class CameraCommand : public SimpleAutoCommand {
public:
	CameraCommand(CameraController* myCamera);
	~CameraCommand() {}
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();

private:
	CameraController* camera;
	double x;
	double y;
	bool isDone;

	double sumx;
	double sumy;
	int iterationCounter;
	int numIterations;
	double waitTime;
	double lastReadTime;
};

/*
 * ULTRASONIC COMMANDS YAY!
 */




#endif /* SRC_DETECTIONCOMMANDS_H_ */
