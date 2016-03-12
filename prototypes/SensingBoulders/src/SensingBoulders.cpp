#include <SensingBoulders.h>
#include "Math.h"

#define PI 3.14159265359

SensingBoulders::SensingBoulders(RobotModel* myRobot) {
	robot = myRobot;
	printf("Creating ultrasonic sensor \n");
	ultrasonicSensor = new UltrasonicSensor(0);

	centerToBoulder = 0.0;
	xBoulderDistance = 0.0;
	yBoulderDistance = 0.0;
	xDistanceToCenterRobot = 12.2; // the number should be changed for comp bot, should ask mechanical for this
	yDistanceToCenterRobot = 12.0; // the number should be changed for comp bot, should ask mechanical for this
//	angleThreshold = 0.5;
//	boulderRadius = 0.0;		   // the number should be changed if the position of the ultrasonic is changed
//	boulderRadiusThreshold = 1.5;
	deltaServoAngle = 0.3;
}

void SensingBoulders::Init() {
	printf("Initializing SensingBoulders \n");

	readingCount = 0;
	startServoAngle = 10;
	endServoAngle = 160;

	robot->InitServo(startServoAngle);

	Wait(1);

	currAngle = robot->GetServoAngle();
	startBoulderAngle = currAngle;
	endBoulderAngle = currAngle;
	centerBoulderAngle = currAngle;

	currDistance = ultrasonicSensor->GetRangeInInches();
	startBoulderDistance = 225;
	endBoulderDistance = startBoulderDistance;
	centerBoulderDistance = startBoulderDistance;

	thresholdDistance = 1.2;	// 20 percent from minBoulderDistance
	isSensingDone = false;
	isDone = false;
	endBoulderFound = false;

	xBoulderDistance = 0.0;
	yBoulderDistance = 0.0;

//	currState = kSensingBoulder;
//	nextState = currState;

	desiredDeltaAngle = 0.0;

	PrintState();
}

// Finding the boulder and then calculated the (x,y) coordinates of it with the origin as the center of the robot
void SensingBoulders::Update(double myCurrTimeSec, double myLastTimeSec) {
	currAngle = robot->GetServoAngle();	//actually takes the value the servo was set to last
	currDistance = ultrasonicSensor->GetRangeInInches();

	if (isSensingDone) {
		return;
	}
	// Gets the average of 3 readings in order to get accurate results
	if ((readingCount <= 3) && (currAngle < endServoAngle)) {
		sumAngle += currAngle;
		sumDistance += currDistance;
		readingCount++;
		robot->SetServo(startServoAngle, endServoAngle, deltaServoAngle);
		return;
	} else {
		averageAngle = sumAngle / readingCount;
		averageDistance = sumDistance / readingCount;
		sumAngle = 0;
		sumDistance = 0;
		readingCount = 0;
	}

	if (averageAngle * thresholdDistance < startBoulderDistance) {
		startBoulderDistance = averageDistance;
		startBoulderAngle = averageAngle;
		minBoulderDistance = startBoulderDistance;
		centerBoulderAngle = startBoulderAngle;
		endBoulderAngle = startBoulderAngle;
		endBoulderDistance = startBoulderDistance;
		printf("changed start boulder\n");
		printf("currDistance: %f\n", currDistance);
//		printf("currAngle: %f\n", currAngle);
//		PrintState();
		endBoulderFound = false;
	}

	// sets the centerBoulderDistance as the minimum distance from the boulder
	if (!endBoulderFound && (currDistance  < minBoulderDistance)) {
//		centerBoulderAngle = currAngle;
		minBoulderDistance = currDistance;
		// adds 5 inches to the centerBoulderDistance to get the actual center of the boulder since I'm not sure if we want the tip of the boulder
		centerBoulderDistance = minBoulderDistance + 5;

		printf("CHANGED MIN DISTANCE: %f \n", minBoulderDistance);
	}

	if ((averageDistance > thresholdDistance * startBoulderDistance) && !endBoulderFound) {
		endBoulderFound = true;
		endBoulderDistance = lastBoulderDistance;
		endBoulderAngle = lastBoulderAngle;
		printf("Curr Distance: %f\n", averageDistance);
		printf("CHANGED END BOULDER \n");
/*		if (fabs(endBoulderAngle - startBoulderAngle) < 20) {
			startBoulderDistance = 225;
			printf("REJECTED!!!\n");
		}*/
//		PrintState();
	}

//	SmartDashboard::PutNumber("currDistance: \n", currDistance);
//	SmartDashboard::PutNumber("servoAngle: \n", currAngle);
	printf("currDistance: %f     ", averageDistance);
	printf("servoAngle: %f\n", averageAngle);

	lastBoulderAngle = averageAngle;
	lastBoulderDistance = averageDistance;

	IsSensingDone();
}

/* 	FOR DETECTING THE BOULDER AND TURNING TOWARDS IT
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
			printf("Found boulder: %d\n", isBoulder);

			if (fabs(desiredDeltaAngle) > angleThreshold && isBoulder) {
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
}


void SensingBoulders::CalculateDesiredDeltaAngle() {
	desiredDeltaAngle = atan2(yBoulderDistance, xBoulderDistance) * 180 / PI - 90;
	printf("Desired Delta Angle: %f\n", desiredDeltaAngle);
}

void SensingBoulders::CalculateBoulderRadius() {
	double deltaAngle = fabs(endBoulderAngle - startBoulderAngle);
	printf("delta angle: %f\n", deltaAngle);
	calculatedRadius = sin(deltaAngle * PI / 2 / 180) * startBoulderDistance;
}
*/
void SensingBoulders::CalculateBoulderAxisDistance() {
	double angle = centerBoulderAngle * PI / 180;
	double x = centerBoulderDistance * cos(angle);	//the x distance from the ultrasonic sensor to the boulder
	double y = centerBoulderDistance * sin(angle);	//the y distance from the ultrasonic sensor to the boulder
	xBoulderDistance = xDistanceToCenterRobot - x;
	yBoulderDistance = y + yDistanceToCenterRobot;

	printf ("Calculate x: %f \n", x);
	printf ("Calculate y: %f \n", y);
}

void SensingBoulders::IsSensingDone() {		// may have to change if endServoAngle is not actually end
	if (currAngle >= endServoAngle) {
		printf("SENSING DONE!! \n");
		PrintState();

		centerBoulderAngle = fabs(startBoulderAngle + endBoulderAngle) / 2;
		CalculateBoulderAxisDistance();

		printf("Center Boulder Angle %f \n", centerBoulderAngle);
		printf("Center Boulder Distance %f \n", centerBoulderDistance);
		printf ("X distance from boulder: %f\n", xBoulderDistance);
		printf ("Y distance from boulder: %f\n", yBoulderDistance);

		endBoulderFound = false;
		robot->InitServo(startServoAngle);
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

double SensingBoulders::GetDistanceInches() {
	return ultrasonicSensor->GetRangeInInches();
}

double SensingBoulders::GetBoulderXDistance() {
	return xBoulderDistance;
}

double SensingBoulders::GetBoulderYDistance() {
	return yBoulderDistance;
}
/*
void SensingBoulders::IsBoulder() {
	printf("Expected Boulder Radius: %f \n", boulderRadius);
	if (calculatedRadius < boulderRadius * boulderRadiusThreshold &&
			calculatedRadius * boulderRadiusThreshold > boulderRadius) {
		printf("IS BOULDER :D!!!! \n");
		isBoulder = true;
		printf("bool: %d", isBoulder);
	} else {
		isBoulder = false;
	}
}*/

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
