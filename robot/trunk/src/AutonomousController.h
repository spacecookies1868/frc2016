#ifndef AUTONOMOUSCONTROLLER_H
#define AUTONOMOUSCONTROLLER_H

#include "WPILib.h"
#include "RobotModel.h"
#include "AutoCommand.h"
#include <vector>
#include <string>
#include <iostream>

//include the controllers as we create their auto implementations as #include

class AutonomousController {
public:
	enum AutoMode {};
	enum StartingPosition {};

	AutonomousController(RobotModel* myRobot, DriveController* myDriver); //add controllers as we create them as parameters of the constructor
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

	unsigned int autoStart;
};

#endif
