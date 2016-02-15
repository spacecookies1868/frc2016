#ifndef AUTOCOMMAND_H
#define AUTOCOMMAND_H

#include "WPILib.h"
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

using namespace std;

class AutoCommand {
public:
	AutoCommand() {}
	virtual ~AutoCommand() {}
	virtual void Init() = 0;
	virtual void Update(double currTimeSec, double deltaTimeSec) = 0;
	virtual bool IsDone() = 0;
	virtual AutoCommand* GetNextCommand() = 0;
};

class SimpleAutoCommand : public AutoCommand {
public:
	SimpleAutoCommand() {
		nextCommand = NULL;
	}
	virtual ~SimpleAutoCommand() {}
	virtual void SetNextCommand(AutoCommand* myNextCommand) {
		nextCommand = myNextCommand;
	}
	virtual AutoCommand* GetNextCommand() {
		return nextCommand;
	}

private:
	AutoCommand *nextCommand;
};

class ConditionalAutoCommand : public AutoCommand {
public:
	ConditionalAutoCommand() {
		trueNextCommand = NULL;
		falseNextCommand = NULL;
		condition = false;
	}
	virtual ~ConditionalAutoCommand() {}
	virtual void SetTrueNextCommand(AutoCommand* myTrueNextCmd) {
		trueNextCommand = myTrueNextCmd;
	}
	virtual void SetFalseNextCommand(AutoCommand* myFalseNextCmd) {
		falseNextCommand = myFalseNextCmd;
	}
	virtual AutoCommand* GetNextCommand() {
		if (condition) {
			return trueNextCommand;
		} else {
			return falseNextCommand;
		}
	}

private:
	AutoCommand *trueNextCommand;
	AutoCommand *falseNextCommand;
	bool condition;
};

class ParallelAutoCommand : public AutoCommand {
public:
	ParallelAutoCommand(SimpleAutoCommand* myFirst, SimpleAutoCommand* mySecond) {
		first = myFirst;
		second = mySecond;
		nextCommand = NULL;
		firstDone = false;
		secondDone = false;
		done = false;
	}
	virtual void Init() {
		first->Init();
		second->Init();
	}
	virtual void Update(double currTimeSec, double deltaTimeSec) {
		firstDone = first->IsDone();
		secondDone = second->IsDone();
		if (firstDone && secondDone) {
			done = true;
		} else if (firstDone && !secondDone) {
			second->Update(currTimeSec, deltaTimeSec);
		} else if (!firstDone && secondDone) {
			first->Update(currTimeSec, deltaTimeSec);
		} else {
			first->Update(currTimeSec, deltaTimeSec);
			second->Update(currTimeSec, deltaTimeSec);
		}
	}
	virtual bool IsDone() {
		return done;
	}
	virtual ~ParallelAutoCommand() {}
	virtual void SetNextCommand(AutoCommand* myNextCommand) {
		nextCommand = myNextCommand;
	}
	virtual AutoCommand* GetNextCommand() {
		return nextCommand;
	}

private:
	SimpleAutoCommand* first;
	SimpleAutoCommand* second;
	AutoCommand* nextCommand;
	bool firstDone;
	bool secondDone;
	bool done;
};

class WaitingCommand: public SimpleAutoCommand {
public:
	WaitingCommand(double myWaitTimeSec);
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();

private:
	double waitTimeSec;
	Timer *timer;
};

#endif
