#include <SensingBoulders.h>

SensingBoulders::SensingBoulders(RobotModel* myRobot) {
	robot = myRobot;
	ultrasonicSensor = new UltrasonicSensor(0);
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
	isDone = false;
	endBoulderFound = false;

	PrintState();
}

void SensingBoulders::Update() {
	currAngle = robot->GetServoAngle();
	currDistance = ultrasonicSensor->GetRangeInInches();

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

}

bool SensingBoulders::IsDone() {		// may have to change if endServoAngle is not actually end
	if (currAngle > endServoAngle) {
//		robot->SetServo(startServoAngle, endServoAngle, 0.0);
		centerBoulderAngle = (startBoulderAngle + endBoulderAngle) / 2;
		centerBoulderDistance = (startBoulderDistance + endBoulderDistance) / 2;
		robot->InitServo(centerBoulderAngle);

		SmartDashboard::PutNumber("Center Boulder Angle \n", centerBoulderAngle);
		SmartDashboard::PutNumber("Start Boulder Angle \n", startBoulderAngle);
		SmartDashboard::PutNumber("End Boulder Angle \n", endBoulderAngle);

		return true;
	} else {
		return false;
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

SensingBoulders::~SensingBoulders() {

}

void SensingBoulders::PrintState() {
	printf("Start Boulder Angle    : %f\n", startBoulderAngle);
	printf("End Boulder Angle      : %f\n", endBoulderAngle);
	printf("Start Boulder Distance : %f\n", startBoulderDistance);
//	printf("End Boulder Distance   : %f\n", endBoulderDistance);
}
