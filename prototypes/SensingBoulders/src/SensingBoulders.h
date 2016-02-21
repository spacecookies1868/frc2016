#ifndef SRC_SENSINGBOULDERS_H_
#define SRC_SENSINGBOULDERS_H_

#include "AutoPivot.h"
#include "RobotModel.h"
#include "UltrasonicSensor.h"
#include "WPILib.h"

class SensingBoulders {
public:
	SensingBoulders(RobotModel* myRobot);
	void Init();
	void Update(double myCurrTimeSec, double myLastTimeSec);
//	void Update();
	void IsSensingDone();
	double GetCenterBoulderAngle();
	double GetCenterBoulderDistance();
	double GetDistanceInches();

	virtual ~SensingBoulders();
private:
	enum SensingBouldersStates {
		kSensingBoulder, kInitPivot, kPivot, kDone
	};


	UltrasonicSensor* ultrasonicSensor;
	AutoPivot* autoPivotCommand;

	uint32_t currState;
	uint32_t nextState;

	double startServoAngle, endServoAngle, deltaServoAngle;
	double thresholdDistance;
	double startBoulderAngle, endBoulderAngle, centerBoulderAngle;
	double startBoulderDistance, endBoulderDistance, centerBoulderDistance;
	double currAngle, currDistance;
	double servoAccuracy;
	double currTimeSec, lastTimeSec, deltaTimeSec;
	double desiredDeltaAngle;
	bool endBoulderFound;
	bool isSensingDone;
	bool isDone;

	RobotModel* robot;

	void PrintState();
};

#endif
