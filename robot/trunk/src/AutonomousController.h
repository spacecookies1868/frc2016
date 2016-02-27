#ifndef AUTONOMOUSCONTROLLER_H
#define AUTONOMOUSCONTROLLER_H

#include "AutoCommand.h"
#include "DriveCommands.h"
#include "SuperstructureCommands.h"
#include "DetectionCommands.h"
#include "DriveController.h"
#include "SuperstructureController.h"
#include "CameraController.h"
#include <vector>
#include <string>
#include <iostream>

// use #include to add controllers

class AutonomousController {
public:
	enum AutoMode { kTestAuto = 0,
					kBlankAuto = 1,
					kReachAuto = 2,
					kCrossAuto = 3,
					kShootAuto = 4,
					kHoardingAuto = 5};

	AutonomousController(RobotModel* myRobot, DriveController* myDrive, SuperstructureController* mySuperstructure, CameraController* myCamera); //add controllers as we create them as parameters of the constructor
	~AutonomousController() {}

	void StartAutonomous();
	void Update(double currTimeSec, double deltaTimeSec);
	void Reset();
	void RefreshIni();

	unsigned int autoMode;

private:
	void CreateQueue();
	void AddtoQueue(AutoCommand* myNewAutoCommand, SimpleAutoCommand* myLastAutoCommand);
	AutoCommand* firstCommand;
	AutoCommand* nextCommand;
	AutoCommand* currentCommand;
	RobotModel* robot;
	DriveController* drive;
	SuperstructureController* superstructure;
	CameraController* camera;
	unsigned int autoStart;
	double timeFinished;
};

#endif
