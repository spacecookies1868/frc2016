#ifndef SRC_DRIVESTRAIGHTCOMMAND_H_
#define SRC_DRIVESTRAIGHTCOMMAND_H_

#include "RobotModel.h"
#include "PIDControlLoop.h"

class DriveStraightCommand {
public:
	DriveStraightCommand(RobotModel* myRobot, double driveStraightSpeed);
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
	virtual ~DriveStraightCommand();
private:
	PIDConfig* CreateRPIDConfig();
	double GetAccumulatedYaw();

	RobotModel* robot;
	PIDConfig* rPIDConfig;
	PIDControlLoop* rPID;

	double driveStraightSpeed;
	double initialR, desiredR;
	double currYaw, lastYaw, deltaYaw, accumulatedYaw;
	bool isDone;
};

#endif
