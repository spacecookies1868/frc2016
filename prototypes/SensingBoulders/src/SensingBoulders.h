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
	double GetDistanceInches();
	virtual ~SensingBoulders();
private:
	UltrasonicSensor* ultrasonicSensor;
	double startServoAngle, endServoAngle, deltaServoAngle;
	double thresholdDistance;
	double startBoulderAngle, endBoulderAngle, centerBoulderAngle;
	double startBoulderDistance, endBoulderDistance, centerBoulderDistance;
	double currAngle, currDistance;
	double servoAccuracy;
	bool isDone;
	RobotModel* robot;

	void PrintState();
};

#endif
