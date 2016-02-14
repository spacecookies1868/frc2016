#ifndef SRC_SENSINGBOULDERS_H_
#define SRC_SENSINGBOULDERS_H_

#include "RobotModel.h"
#include "UltrasonicSensor.h"
#include "WPILib.h"

class SensingBoulders {
public:
	SensingBoulders(RobotModel* myRobot);
	void Init();
	void Update();
	bool IsDone();
	double GetCenterBoulderAngle();
	double GetCenterBoulderDistance();
	virtual ~SensingBoulders();
private:
	UltrasonicSensor* ultrasonicSensor;
	double startServoAngle, endServoAngle, deltaServoAngle;
	double thresholdBoulderDistance;
	double startBoulderAngle, endBoulderAngle, centerBoulderAngle;
	double startBoulderDistance, endBoulderDistance, centerBoulderDistance;
	double currAngle, currDistance;
	bool isDone;
	RobotModel* robot;
};

#endif
