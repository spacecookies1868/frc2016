#ifndef SRC_AUTOPIVOT_H_
#define SRC_AUTOPIVOT_H_

#include <WPILib.h>
#include <AHRS.h>
#include "RobotModel.h"
#include "PIDControlLoop.h"

class AutoPivot {
public:
	AutoPivot(RobotModel* myRobot);
	// One of the following two must be called before Init()
	void SetDesiredDeltaYaw(double myDesiredDeltaYaw);
	void SetDesiredYaw(double myDesiredYaw);
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
	void UpdateAccumulatedYaw();
	virtual ~AutoPivot();
private:
	RobotModel* robot;
	PIDConfig* CreatePIDConfig();
	PIDConfig* rPIDConfig;
	PIDControlLoop* rPID;
	double desiredDeltaYaw, accumulatedYaw, lastYaw, currYaw;
	bool isDone;
};

#endif
