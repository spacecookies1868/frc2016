#ifndef SRC_DEFENSEDRIVE_H_
#define SRC_DEFENSEDRIVE_H_

#include "RobotModel.h"
#include "PivotToAngleCommand.h"
#include "DriveStraightCommand.h"

class DefenseDrive {
public:
	DefenseDrive(RobotModel* myRobot);
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
	int GetState();
	virtual ~DefenseDrive();

private:
	RobotModel* robot;
	enum defenseDriveStates {
		kInit, kBeforeUpRamp, kUpRamp, kMiddleRamp, kDownRamp, kWait, kStraighten, kDone
	};
	uint32_t currState;
	uint32_t nextState;
	double currRoll, lastRoll, startRoll, diffRoll;
	double isFlatThreshold, onRampThreshold, endThreshold, speed;
	PivotToAngleCommand* pivotToAngleCommand;
	double startTime, startWaitTime;
	DriveStraightCommand* driveStraightCommand;
};

#endif
