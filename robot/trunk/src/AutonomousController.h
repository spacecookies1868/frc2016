#ifndef AUTONOMOUSCONTROLLER_H
#define AUTONOMOUSCONTROLLER_H

#include "AutoCommand.h"
#include "DriveCommands.h"
#include "SuperstructureCommands.h"
#include "DetectionCommands.h"
#include "DriveController.h"
#include "SuperstructureController.h"
#include "RemoteControl.h"
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
					kHoardingAuto = 5,
					kSpyBotShootAuto = 6,
					kSpyBotCatCAuto = 7};
	enum FirstDefensePos {
		kNone = 0,
		kLowBar = 1,
		kSecond = 2,
		kThird = 3,
		kFourth = 4,
		kFifth = 5
	};

	AutonomousController(RobotModel* myRobot, DriveController* myDrive,
			SuperstructureController* mySuperstructure,
			CameraController* myCamera,
			RemoteControl* myHumanControl); //add controllers as we create them as parameters of the constructor
	~AutonomousController() {}

	void StartAutonomous();
	void Update(double currTimeSec, double deltaTimeSec);
	void Reset();
	void RefreshIni();

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
	RemoteControl* humanControl;
	unsigned int autoMode;
	unsigned int autoStart;
	bool hardCodeShoot;
	unsigned int firstDefense;
	unsigned int secondDefensePos;
	double timeFinished;
	bool useSallyPort;
};

#endif
