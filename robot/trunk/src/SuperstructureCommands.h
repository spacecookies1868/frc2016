/*
 * SuperstructureCommands.h
 *
 *  Created on: Feb 25, 2016
 *      Author: origa_000
 */

#ifndef SRC_SUPERSTRUCTURECOMMANDS_H_
#define SRC_SUPERSTRUCTURECOMMANDS_H_

#include "WPILib.h"
#include "Debugging.h"
#include "RobotModel.h"
#include "SuperstructureController.h"
#include "ControlBoard.h"
#include <vector>
#include <string>
#include <iostream>
#include "PIDControlLoop.h"
#include "AutoCommand.h"

using namespace std;

/*
 * SUPERSTRUCTURE COMMANDS YAY!
 */
class IntakePositionCommand : public SimpleAutoCommand {
public:
	IntakePositionCommand(SuperstructureController* mySuperstructure, bool myIntakeDown);
	~IntakePositionCommand() {}
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
private:
	SuperstructureController* superstructure;
	bool isDone;
	bool intakeDown;
};

class IntakeRollersCommand : public SimpleAutoCommand {
public:
	IntakeRollersCommand(SuperstructureController* mySuperstructure, bool rollForward, double myWaitTime);
	~IntakeRollersCommand() {}
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
private:
	SuperstructureController* superstructure;
	bool isDone;
	double waitTime;
	double initTime;
	bool firstTime;
	bool forward;
};

class IntakeCommand : public SimpleAutoCommand {
public:
	IntakeCommand(SuperstructureController* mySuperstructure, double myWaitTime);
	~IntakeCommand() {}
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();

private:
	SuperstructureController* superstructure;
	double waitTime;
	bool isDone;
	double initTime;
	bool firstTime;
};

class DefenseManipPosCommand : public SimpleAutoCommand {
public:
	DefenseManipPosCommand(SuperstructureController* mySuperstructure, bool downDesired);
	~DefenseManipPosCommand() {}
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
private:
	SuperstructureController* superstructure;
	bool isDone;
	bool desiredDown;
};

class OuttakeCommand : public SimpleAutoCommand {
public:
	OuttakeCommand(SuperstructureController* mySuperstructure);
	~OuttakeCommand() {}
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
private:
	SuperstructureController* superstructure;
	//DefenseManipPosCommand* defenseDown;
	bool isDone;
};

class OuttakeByTimeCommand : public SimpleAutoCommand {
public:
	OuttakeByTimeCommand(SuperstructureController* mySuperstructure, double myTime);
	~OuttakeByTimeCommand() {}
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
private:
	SuperstructureController* superstructure;
	WaitingCommand* wait;
	DefenseManipPosCommand* defenseDown;
	bool isDone;
	double time;
	double initTime;
};

#endif /* SRC_SUPERSTRUCTURECOMMANDS_H_ */
