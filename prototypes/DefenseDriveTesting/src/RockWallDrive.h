#ifndef SRC_ROCKWALLDRIVE_H_
#define SRC_ROCKWALLDRIVE_H_

#include "RobotModel.h"
#include "PivotToAngleCommand.h"
#include "DriveStraightCommand.h"

class RockWallDrive {
public:
	RockWallDrive(RobotModel* myRobot);
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
	int GetState();
	virtual ~RockWallDrive();

private:
	RobotModel* robot;
	enum defenseDriveStates {
		kInit, kBeforeUpRamp, kUpRamp, kWait, kStraighten, kDone
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
