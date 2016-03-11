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
#include "DriveController.h"
#include <vector>
#include <string>
#include <iostream>
#include "PIDControlLoop.h"
#include "SuperstructureCommands.h"

using namespace std;

/*
 * DRIVING COMMANDS YAY!
 */

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
	double CalculateDesiredChange(double myDesired);
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
	PIDConfig* disPIDConfig;
	PIDConfig* rPIDConfig;

private:
	PIDConfig* CreateDisPIDConfig();
	PIDConfig* CreateRPIDConfig();
	double GetAccumulatedYaw();

	RobotModel* robot;
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
	PIDConfig* radiusPIDConfig;
	PIDConfig* anglePIDConfig;

private:
	PIDConfig* CreateRadiusPIDConfig();
	PIDConfig* CreateAnglePIDConfig();
	double CalculateX();
	double CalculateY();
	double GetAccumulatedYaw();
	double GetSign(double n);
	RobotModel* robot;
	PIDControlLoop* radiusPID;
	PIDControlLoop* anglePID;
	bool isDone;
	double desiredX;
	double desiredY;
	double initialX;
	double initialY;

	double lastX;
	double lastY;
	double initialLeft;
	double initialRight;
	double lastLeft;
	double lastRight;
	double currLeft;
	double currRight;
	double lastAccumulatedYaw;
	double initialYaw;

	double desiredRadius;
	double initialRadius;
	double desiredAngle;
	double initialAngle;
	double angle;

	double lastYaw;
	double currYaw;
	double deltaYaw;
	double accumulatedYaw;

};

class DefenseCommand : public SimpleAutoCommand {
public:
	enum Defenses {
		LowBar = 0,
		Portcullis = 1,
		ChevalDeFrise = 2,
		Ramparts = 3,
		Moat = 4,
		SallyPort = 5,
		Drawbridge = 6,
		RockWall = 7,
		RoughTerrain = 8
	};

	DefenseCommand(RobotModel* myRobot, SuperstructureController* mySuperstructure, uint32_t myDefense);
	~DefenseCommand() {}
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();
	/*
	 * Add statics for different PIDs and stuff here
	 */
private:
	RobotModel* robot;
	SuperstructureController* superstructure;
	uint32_t defense;
	bool isDone;

	DriveStraightCommand* hardCodeLowBar;

	DriveStraightCommand* portcullisDriveUp;
	DriveStraightCommand* portcullisDriving;
	DefenseManipPosCommand* portcullisDefenseUp;
	DefenseManipPosCommand* portcullisDefenseDown;
	bool portcullisWaitTimeDone;
	double portcullisWaiting;

	DriveStraightCommand* chevalDeFriseDriveUp;
	DriveStraightCommand* chevalDeFriseDriving;
	DefenseManipPosCommand* chevalDeFriseDefenseDown;
	DefenseManipPosCommand* chevalDeFriseDefenseUp;
	bool chevalDeFriseDefenseUpInitted;
	bool chevalDeFriseWaitTimeDone;
	double chevalDeFriseWaiting;

	DriveStraightCommand* hardCodeMoat;

	DriveStraightCommand* hardCodeRockWall;

	DriveStraightCommand* hardCodeRoughTerrain;
};


#endif /* SRC_DRIVECOMMANDS_H_ */
