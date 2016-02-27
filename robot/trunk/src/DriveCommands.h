/*
 * DriveCommands.h
 *
 *  Created on: Feb 25, 2016
 *      Author: origa_000
 */

#ifndef SRC_DRIVECOMMANDS_H_
#define SRC_DRIVECOMMANDS_H_

#include "WPILib.h"
#include "AutoCommand.h"
#include "Debugging.h"
#include "Logger.h"
#include "RobotModel.h"
#include "RemoteControl.h"
#include "DriveController.h"
#include <vector>
#include <string>
#include <iostream>
#include "PIDControlLoop.h"

using namespace std;

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


class CurveCommand : public SimpleAutoCommand {
public:
	CurveCommand(RobotModel* myRobot, double myDesiredX, double myDesiredY);
	~CurveCommand() {}
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();

	static double radiusPFac, radiusIFac, radiusDFac, radiusDesiredAccuracy, radiusMaxAbsOutput,
		radiusMaxAbsError, radiusMaxAbsDiffError, radiusMaxAbsITerm, radiusTimeLimit;
	static double anglePFac, angleIFac, angleDFac, angleDesiredAccuracy, angleMaxAbsOutput,
		angleMaxAbsError, angleMaxAbsDiffError, angleMaxAbsITerm, angleTimeLimit;

private:
	PIDConfig* CreateRadiusPIDConfig();
	PIDConfig* CreateAnglePIDConfig();
	double CalculateX();
	double CalculateY();
	double GetAccumulatedYaw();
	RobotModel* robot;
	PIDConfig* radiusPIDConfig;
	PIDConfig* anglePIDConfig;
	PIDControlLoop* radiusPID;
	PIDControlLoop* anglePID;
	bool isDone;
	double desiredX;
	double desiredY;
	double initialX;
	double initialY;

	double lastX;
	double lastY;
	double lastLeft;
	double lastRight;
	double lastAccumulatedYaw;
	double initialYaw;

	double desiredRadius;
	double initialRadius;
	double desiredAngle;
	double initialAngle;

	double lastYaw;
	double currYaw;
	double deltaYaw;
	double accumulatedYaw;

};




#endif /* SRC_DRIVECOMMANDS_H_ */
