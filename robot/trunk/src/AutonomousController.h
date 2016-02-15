#ifndef AUTONOMOUSCONTROLLER_H
#define AUTONOMOUSCONTROLLER_H

#include "AutoCommand.h"
#include <vector>
#include <string>
#include <iostream>

// use #include to add controllers

class AutonomousController {
public:
	enum AutoMode {};

	AutonomousController(RobotModel* myRobot, DriveController* myDrive, SuperstructureController* mySuperstructure); //add controllers as we create them as parameters of the constructor
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
	unsigned int autoStart;
	double timeFinished;
};

#endif
