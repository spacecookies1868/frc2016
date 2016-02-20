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
#include "CameraController.h"

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
	~WaitingCommand() {}
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();

private:
	double waitTimeSec;
	Timer *timer;
	bool isDone;
};

/*
 * DRIVING COMMANDS YAY!
 */

class DriveCommand : public SimpleAutoCommand {
public:
	DriveCommand() {}
	virtual ~DriveCommand() {}
	virtual double GetError() = 0;
	virtual double GetLeftOutput() = 0;
	virtual double GetRightOutput() = 0;
};

class TransitionedDriveCommand : public DriveCommand{
public:
	TransitionedDriveCommand(RobotModel* myRobot, DriveCommand* myFirstDrive, double myFirstErrorThreshold,
			DriveCommand* mySecondDrive) {
		first = myFirstDrive;
		second = mySecondDrive;
		firstThreshold = myFirstErrorThreshold;
		robot = myRobot;
		isDone = false;
		m_stateVal = kFirstCommand;
		nextState = m_stateVal;
		left = 0.0;
		right = 0.0;
	}
	virtual void Init() {
		first->Init();
		second->Init();
	}
	/*
	 * FIGURE OUT WHERE TO INIT THE SECOND COMMAND
	 */
	virtual void Update(double currTimeSec, double deltaTimeSec) {
		switch(m_stateVal) {
		case(kFirstCommand): {
			first->Update(currTimeSec, deltaTimeSec);
			if (first->GetError() < firstThreshold) {
				nextState = kInitializeTransition;
			}
			break;
		}
		case (kInitializeTransition): {
			second->Init();
			first->Update(currTimeSec, deltaTimeSec);
			second->Update(currTimeSec, deltaTimeSec);
			left = first->GetLeftOutput() + second->GetLeftOutput();
			right = first->GetRightOutput() + second->GetRightOutput();
			left = left / fmax(fabs(left), fabs(right));
			right = right / fmax(fabs(left), fabs(right));
			robot->SetWheelSpeed(RobotModel::kLeftWheels, left);
			robot->SetWheelSpeed(RobotModel::kRightWheels, right);
			nextState = kTransitioning;
			break;
		}
		case (kTransitioning): {
			first->Update(currTimeSec, deltaTimeSec);
			second->Update(currTimeSec, deltaTimeSec);
			left = first->GetLeftOutput() + second->GetLeftOutput();
			right = first->GetRightOutput() + second->GetRightOutput();
			left = left / fmax(fabs(left), fabs(right));
			right = right / fmax(fabs(left), fabs(right));
			robot->SetWheelSpeed(RobotModel::kLeftWheels, left);
			robot->SetWheelSpeed(RobotModel::kRightWheels, right);
			if (first->IsDone()) {
				nextState = kSecondCommand;
			}
			break;
		}
		case (kSecondCommand): {
			second->Update(currTimeSec, deltaTimeSec);
			if (second->IsDone()){
				nextState = kDone;
			}
			break;
		}
		case(kDone): {
			isDone = true;
			robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
		}
		}
		m_stateVal = nextState;
	}
	virtual bool IsDone() {
		return isDone;
	}

	virtual double GetLeftOutput() {
		return left;
	}

	virtual double GetRightOutput() {
		return right;
	}

	virtual double GetError() {
		return second->GetError();
	}
	enum UpdateStates{
		kFirstCommand, kInitializeTransition, kTransitioning, kSecondCommand, kDone
	};
private:
	RobotModel* robot;
	DriveCommand* first;
	DriveCommand* second;
	double firstThreshold;
	bool isDone;
	double left;
	double right;

	uint32_t m_stateVal;
	uint32_t nextState;
};


/*
 * Pivot Command WARNING: Do not use unless USE_NAVX is true
 */
class PivotCommand : public SimpleAutoCommand {
public:
	PivotCommand(RobotModel* myRobot, double myDesiredR);
	~PivotCommand() {}
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();
	static double rPFac, rIFac, rDFac, rDesiredAccuracy, rMaxAbsOutput, rMaxAbsDiffError,
			rMaxAbsError, rMaxAbsITerm, rTimeLimit;
private:
	PIDConfig* CreateRPIDConfig();
	double GetAccumulatedYaw();
	RobotModel* robot;
	PIDConfig* rPIDConfig;
	PIDControlLoop* rPID;
	bool isDone;

	double desiredR;
	double initialR;

	double lastYaw;
	double currYaw;
	double deltaYaw;
	double accumulatedYaw;
};

/*
 * WARNING: Do not use unless USE_NAVX is true
 */

class PivotToAngleCommand : public SimpleAutoCommand {
public:
	PivotToAngleCommand(RobotModel* myRobot, double myDesiredR);
	~PivotToAngleCommand() {}
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();
	static double rPFac, rIFac, rDFac, rDesiredAccuracy, rMaxAbsOutput, rMaxAbsDiffError,
			rMaxAbsError, rMaxAbsITerm, rTimeLimit;
private:
	PIDConfig* CreateRPIDConfig();
	double GetAccumulatedYaw();
	double CalculateDesiredYaw(double myDesired);
	RobotModel* robot;
	PIDConfig* rPIDConfig;
	PIDControlLoop* rPID;
	bool isDone;

	double desiredR;
	double initialR;

	double lastYaw;
	double currYaw;
	double deltaYaw;
	double accumulatedYaw;
};

class DriveStraightCommand : public SimpleAutoCommand {
public:
	DriveStraightCommand(RobotModel* myRobot, double myDesiredDis);
	~DriveStraightCommand() {}
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();

	static double disPFac, disIFac, disDFac, disDesiredAccuracy,
			disMaxAbsOutput, disMaxAbsDiffError, disMaxAbsError, disMaxAbsITerm,
			disTimeLimit;
	static double rPFac, rIFac, rDFac, rDesiredAccuracy, rMaxAbsOutput,
			rMaxAbsDiffError, rMaxAbsError, rMaxAbsITerm, rTimeLimit;
private:
	PIDConfig* CreateDisPIDConfig();
	PIDConfig* CreateRPIDConfig();
	double GetAccumulatedYaw();

	RobotModel* robot;
	PIDConfig* disPIDConfig;
	PIDConfig* rPIDConfig;
	PIDControlLoop* disPID;
	PIDControlLoop* rPID;

	double lastYaw;
	double currYaw;
	double deltaYaw;
	double accumulatedYaw;

	bool isDone;
	double desiredDis;
	double initialDis;
	double desiredR;
	double initialR;
};

/*
 *
 */

//class CurveCommand : public SimpleAutoCommand {
//public:
//	CurveCommand(RobotModel* myRobot, double myDesiredX, double myDesiredY, double myDesiredR);
//	~CurveCommand() {}
//	virtual void Init();
//	virtual void Update(double currTimeSec, double deltaTimeSec);
//	virtual bool IsDone();
//private:
//	RobotModel* robot;
//	double desiredX;
//	double desiredY;
//	double desiredR;
//};


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
#endif
