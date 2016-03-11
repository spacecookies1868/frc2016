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
#include <vector>
#include <string>
#include <iostream>
#include "PIDControlLoop.h"
#include "CameraController.h"
#include "DriveCommands.h"
#include "SuperstructureCommands.h"

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

class CameraDetectionCommand : public SimpleAutoCommand {
public:
	CameraDetectionCommand(CameraController* myCamera);
	~CameraDetectionCommand() {}
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();
	double GetX();
	double GetY();

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

class UltrasonicDetectionCommand : public SimpleAutoCommand {
public:
	UltrasonicDetectionCommand(); //add parameters
	~UltrasonicDetectionCommand() {}
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();
	double GetX();
	double GetY();
private:
	double x;
	double y;
	bool isDone;
};

class UltrasonicCommand : public SimpleAutoCommand {
public:
	enum ultrasonicState {
		kInit, kDetecting, kNearingInit, kNearing, kIntakingInit, kForward, kBacking, kDone
	};
	UltrasonicCommand(RobotModel* myRobot, SuperstructureController* mySuperstructure);
	~UltrasonicCommand() {}
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();
private:
	RobotModel* robot;
	SuperstructureController* superstructure;
	bool isDone;
	uint32_t currState;
	uint32_t nextState;

	UltrasonicDetectionCommand* uDetect;
	CurveCommand* nearingDrive;
	DriveStraightCommand* forwardDrive;
	DriveStraightCommand* backingDrive;
};


#endif /* SRC_DETECTIONCOMMANDS_H_ */
