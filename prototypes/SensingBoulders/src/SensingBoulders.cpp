#include <SensingBoulders.h>
#include "Math.h"

#define PI 3.14159265359

SensingBoulders::SensingBoulders(RobotModel* myRobot) {
	robot = myRobot;
	printf("Creating ultrasonic sensor \n");
	ultrasonicSensor = new UltrasonicSensor(0);

	centerToBoulder = 0.0;
	xDistanceToCenterRobot = 12.2; //the number should be changed for comp bot, should ask mechanical for this
	yDistanceToCenterRobot = 12.0; // the number should be changed for comp bot, should ask mechanical for this
}

void SensingBoulders::Init() {
	printf("Initializing SensingBoulders \n");

	startServoAngle = 40;
	endServoAngle = 135;
	deltaServoAngle = 2.0;
	servoAccuracy = 1;

	robot->InitServo(startServoAngle);

	Wait(0.3);

	currAngle = robot->GetServoAngle();
	startBoulderAngle = currAngle;
	endBoulderAngle = currAngle;
	centerBoulderAngle = currAngle;

	currDistance = ultrasonicSensor->GetRangeInInches();
	startBoulderDistance = ultrasonicSensor->GetRangeInInches();
	endBoulderDistance = startBoulderDistance;
	centerBoulderDistance = startBoulderDistance;

	thresholdDistance = 1.2;	// 20 percent from minBoulderDistance
	isSensingDone = false;
	isDone = false;
	endBoulderFound = false;

	currState = kSensingBoulder;
	nextState = currState;

	desiredDeltaAngle = 0.0;

	PrintState();
}

/*void SensingBoulders::Update(double myCurrTimeSec, double myLastTimeSec) {
	currAngle = robot->GetServoAngle();
	currDistance = ultrasonicSensor->GetRangeInInches();

	if (isSensingDone){
		CalculateDesiredDeltaAngle();

		printf("Center Boulder Angle: %f \n", centerBoulderAngle);
		printf("Center Boulder Distance: %f\n", centerBoulderDistance);
		printf("Desired Angle: %f\n", desiredDeltaAngle);

		robot->InitServo(centerBoulderAngle);
		return;
	}

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

	robot->SetServo(startServoAngle, endServoAngle, deltaServoAngle);

	IsSensingDone();
}*/


void SensingBoulders::Update(double myCurrTimeSec, double myLastTimeSec) {
	currAngle = robot->GetServoAngle();
	currDistance = ultrasonicSensor->GetRangeInInches();
	currTimeSec = myCurrTimeSec;
	lastTimeSec = myLastTimeSec;
	deltaTimeSec = currTimeSec - lastTimeSec;

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

		robot->SetServo(startServoAngle, endServoAngle, deltaServoAngle);

		IsSensingDone();
		nextState = kSensingBoulder;

		if (isSensingDone){
			CalculateDesiredDeltaAngle();

			printf("Center Boulder Angle: %f \n", centerBoulderAngle);
			printf("Center Boulder Distance: %f\n", centerBoulderDistance);
			printf("Desired Angle: %f\n", desiredDeltaAngle);

			nextState = kInitPivot;
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


void SensingBoulders::IsSensingDone() {		// may have to change if endServoAngle is not actually end
	if (currAngle > endServoAngle) {
		centerBoulderAngle = (startBoulderAngle + endBoulderAngle) / 2;
		centerBoulderDistance = (startBoulderDistance + endBoulderDistance) / 2;

		SmartDashboard::PutNumber("Center Boulder Angle \n", centerBoulderAngle);
		SmartDashboard::PutNumber("Start Boulder Angle \n", startBoulderAngle);
		SmartDashboard::PutNumber("End Boulder Angle \n", endBoulderAngle);

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
/*
double SensingBoulders::CalculateDesiredDeltaAngle() {
	double a = centerBoulderDistance * cos(centerBoulderAngle - 90);
	double b = centerBoulderDistance * sin(centerBoulderAngle - 90);

	printf("a %f\n", a);
	printf("b %f\n", b);


	if (centerBoulderAngle > 90) {
		a = centerBoulderDistance * cos(centerBoulderAngle);
		b = centerBoulderDistance * sin(centerBoulderAngle);

		double c = a + distanceToCenterRobot;
		desiredDeltaAngle = atan(b/c);
	} else if (a < 12.2){
		double c = sqrt(pow(centerBoulderDistance, 2) + pow(distanceToCenterRobot, 2) - 2 * centerBoulderDistance * distanceToCenterRobot * cos(centerBoulderAngle));
		desiredDeltaAngle = -asin(b/c);
	} else {
		double c = a - distanceToCenterRobot;
		desiredDeltaAngle = -atan(b/c);
	}

	return desiredDeltaAngle;
}*/

void SensingBoulders::CalculateDesiredDeltaAngle() {
	double angle = centerBoulderAngle * PI / 180;
	double x = centerBoulderDistance * cos(angle);	//the x distance from the ultrasonic sensor to the boulder
	double y = centerBoulderDistance * sin(angle);	//the y distance from the ultrasonic sensor to the boulder
	double x1 = x + xDistanceToCenterRobot;	//the x distance from the boulder to the center of the robot
	double y1 = y + yDistanceToCenterRobot; //the y distance from the boulder to the center of the robot

	desiredDeltaAngle = 90 - (atan2(y1, x1) * 180 / PI);
	printf("Desired Delta Angle: %f\n", desiredDeltaAngle);
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
//	printf("End Boulder Distance   : %f\n", endBoulderDistance);
}
