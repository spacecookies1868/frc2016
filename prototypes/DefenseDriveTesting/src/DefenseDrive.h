#ifndef SRC_DEFENSEDRIVE_H_
#define SRC_DEFENSEDRIVE_H_

#include <RobotModel.h>

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
		kInit, kBeforeUpRamp, kUpRamp, kMiddleRamp, kDownRamp, kDone
	};
	uint32_t currState;
	uint32_t nextState;
	double currRoll, lastRoll, deltaRoll, startRoll, diffRoll;
	double isFlatThreshold, onRampThreshold, slopeOfRamp, speed;
	double startTime;
};

#endif
