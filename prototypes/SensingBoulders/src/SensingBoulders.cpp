#include <SensingBoulders.h>
#include "Math.h"

#define PI 3.14159265359

SensingBoulders::SensingBoulders(RobotModel* myRobot) {
	robot = myRobot;
	printf("Creating ultrasonic sensor \n");
	ultrasonicSensor = new UltrasonicSensor(0);

	centerToBoulder = 0.0;
	xDistanceToCenterRobot = 12.2; // the number should be changed for comp bot, should ask mechanical for this
	yDistanceToCenterRobot = 12.0; // the number should be changed for comp bot, should ask mechanical for this
	angleThreshold = 2.5;
	boulderRadius = 5.0;		   // the number should be changed if the position of the ultrasonic is changed
	boulderRadiusThreshold = 1.0;
}

void SensingBoulders::Init() {
	printf("Initializing SensingBoulders \n");

	numberOfScans = 0;
	startServoAngle = 5;
	endServoAngle = 160;
	deltaServoAngle = 2.0;
	servoAccuracy = 1;

	robot->InitServo(startServoAngle);

	Wait(1);

	currAngle = robot->GetServoAngle();
	startBoulderAngle = currAngle;
	endBoulderAngle = currAngle;
	centerBoulderAngle = currAngle;

	currDistance = ultrasonicSensor->GetRangeInInches();
	startBoulderDistance = ultrasonicSensor->GetRangeInInches();
	endBoulderDistance = startBoulderDistance;
	centerBoulderDistance = startBoulderDistance;

	startBoulderAngles[numberOfScans] = 0;
	startBoulderDistances[numberOfScans] = 0;

	thresholdDistance = 1.2;	// 20 percent from minBoulderDistance
	isSensingDone = false;
	isDone = false;
	endBoulderFound = false;

	currState = kSensingBoulder;
	nextState = currState;

	desiredDeltaAngle = 0.0;

	PrintState();
}

void SensingBoulders::Update(double myCurrTimeSec, double myLastTimeSec) {
	currAngle = robot->GetServoAngle();
	currDistance = ultrasonicSensor->GetRangeInInches();

	if (isSensingDone) {
		return;
	}

	if (currDistance * thresholdDistance < startBoulderDistance) {
		startBoulderDistance = currDistance;
		startBoulderAngle = currAngle;
		endBoulderFound = false;
		endBoulderAngle = startBoulderAngle;
		endBoulderDistance = startBoulderDistance;
		printf("changed start boulder\n");
//		printf("currDistance: %f\n", currDistance);
//		printf("currAngle: %f\n", currAngle);
//		PrintState();
	}
//	printf("startBoulderDistance: %f\n", startBoulderDistance);

	if (currDistance > thresholdDistance * startBoulderDistance && !endBoulderFound) {
		endBoulderFound = true;
		endBoulderDistance = currDistance;
		endBoulderAngle = currAngle;
//		centerBoulderAngle = (startBoulderAngle + endBoulderAngle) / 2;
//		centerBoulderDistance = (startBoulderDistance + endBoulderDistance) / 2;
//		printf("End Boulder Distance: %f\n", endBoulderDistance);
//		printf("Changed end boulder \n");
//		PrintState();
	}

	SmartDashboard::PutNumber("currDistance: \n", currDistance);
	SmartDashboard::PutNumber("servoAngle: \n", currAngle);
//	printf("currDistance: %f     ", currDistance);
//	printf("servoAngle: %f\n", currAngle);

	robot->SetServo(startServoAngle, endServoAngle, deltaServoAngle);

	IsSensingDone();
}

/*
void SensingBoulders::Update(double myCurrTimeSec, double myDeltaTimeSec) {
	currAngle = robot->GetServoAngle();
	currDistance = ultrasonicSensor->GetRangeInInches();
	currTimeSec = myCurrTimeSec;
	deltaTimeSec = myDeltaTimeSec;

	switch(currState){
	case(kSensingBoulder):
		if (currDistance * thresholdDistance < startBoulderDistance) {
			startBoulderDistance = currDistance;
			startBoulderAngle = currAngle;
			endBoulderFound = false;
			endBoulderAngle = startBoulderAngle;
			endBoulderDistance = startBoulderDistance;
			printf("changed start boulder\n");
			printf("currDistance: %f\n", currDistance);
			printf("currAngle: %f\n", currAngle);
			PrintState();
		}
	//	printf("startBoulderDistance: %f\n", startBoulderDistance);

		if (currDistance > thresholdDistance * startBoulderDistance && !endBoulderFound) {
			endBoulderFound = true;
			endBoulderDistance = currDistance;
			endBoulderAngle = currAngle;
	//		centerBoulderAngle = (startBoulderAngle + endBoulderAngle) / 2;
	//		centerBoulderDistance = (startBoulderDistance + endBoulderDistance) / 2;
			printf("Changed end boulder \n");
			PrintState();
		}

		SmartDashboard::PutNumber("currDistance: \n", currDistance);
		SmartDashboard::PutNumber("servoAngle: \n", currAngle);
//		printf("currDistance: %f     ", currDistance);
//		printf("servoAngle: %f\n", currAngle);

		robot->SetServo(startServoAngle, endServoAngle, deltaServoAngle);

		IsSensingDone();
		nextState = kSensingBoulder;

		if (isSensingDone){
			CalculateDesiredDeltaAngle();

			printf("Center Boulder Angle: %f \n", centerBoulderAngle);
			printf("Center Boulder Distance: %f\n", centerBoulderDistance);
			printf("Desired Angle: %f\n", desiredDeltaAngle);

			if (fabs(desiredDeltaAngle) > angleThreshold) {
				nextState = kInitPivot;
			} else {
				nextState = kDone;
			}
		}
		break;
	case(kInitPivot):
	//setting the desiredAngle so that the robot would turn according to ultrasonic values
//		desiredDeltaAngle = centerBoulderAngle - 90;

		printf("desired delta angle %f \n", desiredDeltaAngle);
		autoPivotCommand = new AutoPivot(robot);
		autoPivotCommand->SetDesiredDeltaYaw(desiredDeltaAngle);
		autoPivotCommand->Init();
		nextState = kPivot;
		break;
	case (kPivot):
		if (autoPivotCommand->IsDone()) {
			nextState = kDone;
		} else {
			autoPivotCommand->Update(currTimeSec, deltaTimeSec);
			nextState = kPivot;
		}
		break;
	case (kDone):
		robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
		printf("Pivot done \n");
		isDone = true;
		robot->InitServo(90);
		break;
	}
	currState = nextState;
}*/


void SensingBoulders::IsSensingDone() {		// may have to change if endServoAngle is not actually end
	if (currAngle > endServoAngle || currAngle < startServoAngle){
		centerBoulderAngle = fabs(startBoulderAngle + endBoulderAngle) / 2;
		centerBoulderDistance = fabs(startBoulderDistance + endBoulderDistance) / 2;

		if (robot->GetServoDirection()) {
			startBoulderAngles[numberOfScans] = startBoulderAngle;
			startBoulderDistances[numberOfScans] = startBoulderDistance;
		} else {
			startBoulderAngles[numberOfScans] = endBoulderAngle;
			startBoulderDistances[numberOfScans] = endBoulderDistance;
		}
		printf("Start Boulder Angle arr: %f\n", startBoulderAngles[numberOfScans]);
		printf("Start Boulder Distance arr: %f\n", startBoulderDistances[numberOfScans]);

		numberOfScans++;
		startBoulderDistance = 225;
		startBoulderAngles[numberOfScans] = 0;
		startBoulderDistances[numberOfScans] = 0;

		printf("Sensing Done Scan: %d \n", numberOfScans);
	}
	if (numberOfScans > 5) {
		BubbleSort(startBoulderAngles);
		BubbleSort(startBoulderDistances);

		startBoulderAngle = startBoulderAngles[3];
		startBoulderDistance = startBoulderDistances[3];

		printf("Median Start Boulder Angle: %f\n", startBoulderAngle);
		printf("Median Start Boulder Distance: %f\n", startBoulderDistance);

		CalculateBoulderRadius();
		printf("Boulder Radius: %f\n", calculatedRadius);

		isSensingDone = true;
	} else {
		isSensingDone = false;
	}
}

double SensingBoulders::GetCenterBoulderAngle() {
	return centerBoulderAngle;
}

double SensingBoulders::GetCenterBoulderDistance() {
	return centerBoulderDistance;
}

double SensingBoulders::GetDistanceInches(){
	return ultrasonicSensor->GetRangeInInches();
}

void SensingBoulders::CalculateDesiredDeltaAngle() {
	double angle = centerBoulderAngle * PI / 180;
	double x = - centerBoulderDistance * cos(angle);	//the x distance from the ultrasonic sensor to the boulder
	double y = centerBoulderDistance * sin(angle);	//the y distance from the ultrasonic sensor to the boulder
	double x1 = x + xDistanceToCenterRobot;	//the x distance from the boulder to the center of the robot
	double y1 = y + yDistanceToCenterRobot; //the y distance from the boulder to the center of the robot

	desiredDeltaAngle = 90 - atan2(y1, x1) * 180 / PI;
	printf("Desired Delta Angle: %f\n", desiredDeltaAngle);
}

void SensingBoulders::CalculateBoulderRadius() {
	double deltaAngle = endBoulderAngle - startBoulderAngle;
	printf("delta angle: %f\n", deltaAngle);
	calculatedRadius = sin(deltaAngle * PI / 2 / 180) * startBoulderDistance;
}

double *SensingBoulders::BubbleSort(double array[5]) {
	float change;
	for (int a = 0; a <= 5; a++) {
		for (int b = a + 1; b <= 5; b++) {
			if (array[a] < array[b]) {
				change = array[a];
				array[a] = array[b];
				array[b] = change;
			}
		}
	}
	return array;
}

bool SensingBoulders::IsDone() {
	return isDone;
}

SensingBoulders::~SensingBoulders() {

}

void SensingBoulders::PrintState() {
	printf("Start Boulder Angle    : %f\n", startBoulderAngle);
	printf("End Boulder Angle      : %f\n", endBoulderAngle);
	printf("Start Boulder Distance : %f\n", startBoulderDistance);
	printf("End Boulder Distance   : %f\n", endBoulderDistance);
}
