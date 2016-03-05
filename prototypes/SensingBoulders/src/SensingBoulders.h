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
	void CalculateDesiredDeltaAngle();
	void CalculateBoulderRadius();
	void CalculateAverageStartBoulderAngle();
	void CalculateMedianStartBoulderAngle();
	double *BubbleSort(double array[5]);
	bool IsDone();

	virtual ~SensingBoulders();
private:
	enum SensingBouldersStates {
		kSensingBoulder, kInitPivot, kPivot, kDone
	};

	AutoPivot* autoPivotCommand;
	UltrasonicSensor* ultrasonicSensor;

	uint32_t currState;
	uint32_t nextState;

	double xDistanceToCenterRobot; //the distance from the ultrasonic sensor to the front of the center of the robot
	double yDistanceToCenterRobot; // distance from the ultrasonic sensor to the side of the center of the robot
	double angleThreshold; 		   // in case the boulder happens to be in front of the robot already
	double boulderRadius;		   // the radius of the boulder
	double boulderRadiusThreshold;
	double thresholdDistance;

	int numberOfScans;
	double calculatedRadius;
	double startServoAngle, endServoAngle, deltaServoAngle;
	double startBoulderAngle, endBoulderAngle, centerBoulderAngle;
	double startBoulderAngles[5];
	double startBoulderDistances[5];
	double centerToBoulder;
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
