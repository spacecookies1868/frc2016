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
	void IsSensingDone();
	double GetCenterBoulderAngle();
	double GetCenterBoulderDistance();
	double GetDistanceInches();
	double GetBoulderXDistance();
	double GetBoulderYDistance();
//	void CalculateDesiredDeltaAngle();
//	void CalculateBoulderRadius();
	void CalculateBoulderAxisDistance();
//	void IsBoulder();
	bool IsDone();

	virtual ~SensingBoulders();
private:
/*	enum SensingBouldersStates {
		kSensingBoulder, kInitPivot, kPivot, kDone
	};
*/
//	AutoPivot* autoPivotCommand;
	UltrasonicSensor* ultrasonicSensor;

//	uint32_t currState;
//	uint32_t nextState;

	double xDistanceToCenterRobot; //the distance from the ultrasonic sensor to the front of the center of the robot
	double yDistanceToCenterRobot; // distance from the ultrasonic sensor to the side of the center of the robot
//	double angleThreshold; 		   // in case the boulder happens to be in front of the robot already
//	double boulderRadius;		   // the radius of the boulder
//	double boulderRadiusThreshold;
	double thresholdDistance;

//	int numberOfScans;
	int readingCount;
//	double calculatedRadius;
	double startServoAngle, endServoAngle, deltaServoAngle;
	double deltaServoRange;
	double startBoulderAngle, endBoulderAngle, centerBoulderAngle;
	double lastBoulderAngle;
	double deltaMeasuredDistance;
	double sumAngle;
	double averageAngle;
//	double sumStartBoulderAngle;
//	double sumEndBoulderAngle;
//	double sumBoulderDistance;
	double averageDistance;
	double sumDistance;
//	double sumStartBoulderDistance;
	double startBoulderDistance, endBoulderDistance, centerBoulderDistance;
	double lastBoulderDistance;
	double minBoulderDistance;
	double currAngle;
	double currDistance;
	double currTimeSec, lastTimeSec, deltaTimeSec;
	double desiredDeltaAngle;
	double xBoulderDistance; // horizontal distance from the center of the robot to the center of the boulder
	double yBoulderDistance; // vertical distane from the center of the robot to the center of the boulder
	bool endBoulderFound;
	bool isSensingDone;
	bool isBoulder;
	bool isDone;

	RobotModel* robot;

	void PrintState();
};

#endif
