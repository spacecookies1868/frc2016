#ifndef SRC_MOATDRIVE_H_
#define SRC_MOATDRIVE_H_

#include "RobotModel.h"
#include "PivotToAngleCommand.h"
#include "DriveStraightCommand.h"

class MoatDrive {
public:
	MoatDrive(RobotModel* myRobot);
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
	int GetState();
	virtual ~MoatDrive();

private:
	RobotModel* robot;
	enum defenseDriveStates {
		kInit, kBeforeUpRamp, kUpRamp, kMiddleRamp, kDownRamp, kWait, kStraighten, kDone
	};
	uint32_t currState;
	uint32_t nextState;
	double currRoll, lastRoll, startRoll, diffRoll;
	double isFlatThreshold, onRampThreshold, rollDerivativeIsFlatThreshold, endThreshold, speed;
	PivotToAngleCommand* pivotToAngleCommand;
	double startTime, startWaitTime;

	double derivativeOfRoll;
	DriveStraightCommand* driveStraightCommand;
};

#endif
