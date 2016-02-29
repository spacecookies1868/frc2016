#ifndef SRC_PIVOTTOANGLE_H_
#define SRC_PIVOTTOANGLE_H_

#include "RobotModel.h"
#include "PivotToAngleCommand.h"
#include "PIDControlLoop.h"

class PivotToAngleCommand {
public:
	PivotToAngleCommand(RobotModel* myRobot, double myDesiredR);
	~PivotToAngleCommand() {}
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
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

#endif
