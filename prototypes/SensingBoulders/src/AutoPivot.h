#ifndef SRC_AUTOPIVOT_H_
#define SRC_AUTOPIVOT_H_

#include <WPILib.h>
#include <AHRS.h>
#include "RobotModel.h"
#include "PIDControlLoop.h"

class AutoPivot {
public:
	AutoPivot(RobotModel* myRobot, double myDesiredR);
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
	double GetAccumulatedYaw();
	double CalculateDesiredYaw(double myDesired);
	virtual ~AutoPivot();
private:
	RobotModel* robot;
	PIDConfig* CreatePIDConfig();
	PIDConfig* rPIDConfig;
	PIDControlLoop* rPID;
	double initialR, desiredR, accumulatedYaw, lastYaw, currYaw, deltaYaw;
	bool isDone;
};

#endif
