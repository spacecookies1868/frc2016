#include <SensingBoulders.h>

SensingBoulders::SensingBoulders(RobotModel* myRobot) {
	robot = myRobot;
	ultrasonicSensor = new UltrasonicSensor(0);

	startServoAngle = 0.0;
	endServoAngle = 0.0;
	deltaServoAngle = 0.0;

	thresholdBoulderDistance = 0.0;

	startBoulderAngle = 0.0;
	endBoulderAngle = 0.0;
	centerBoulderAngle = 0.0;

	startBoulderDistance = 0.0;
	endBoulderDistance = 0.0;
	centerBoulderDistance = 0.0;

	currAngle = 0.0;
	currDistance = 0.0;
	isDone = false;
}

void SensingBoulders::Init() {
	currAngle = robot->GetServoAngle();
	startBoulderAngle = robot->GetServoAngle();
	currDistance = ultrasonicSensor->GetRangeInInches();

	startBoulderDistance = ultrasonicSensor->GetRangeInInches();
	endBoulderDistance = ultrasonicSensor->GetRangeInInches();

	startServoAngle = 0;
	endServoAngle = 180;
	deltaServoAngle = 0.25;
	thresholdBoulderDistance = 1.2;	// 20 percent from minBoulderDistance
	isDone = false;
}

void SensingBoulders::Update() {
	currAngle = robot->GetServoAngle();
	currDistance = ultrasonicSensor->GetRangeInInches();
	if (currDistance * thresholdBoulderDistance < startBoulderDistance) {
		startBoulderDistance = currDistance;
		startBoulderAngle = currAngle;
		printf("changed start boulder\n");
	}
	printf("startBoulderDistance: %f\n", startBoulderDistance);

	if (currDistance > thresholdBoulderDistance * startBoulderDistance) {
		endBoulderDistance = currDistance;
		endBoulderAngle = currAngle;
		centerBoulderAngle = (startBoulderAngle + endBoulderAngle) / 2;
		centerBoulderDistance = (startBoulderDistance + endBoulderDistance) / 2;
	}
	SmartDashboard::PutNumber("currDistance: \n", currDistance);
	SmartDashboard::PutNumber("servoAngle: \n", currAngle);

	printf("currDistance: %f\n", currDistance);
	printf("currAngle: %f\n", currAngle);
	robot->SetServo(startServoAngle, endServoAngle, deltaServoAngle);
}

bool SensingBoulders::IsDone() {		// may have to change if endServoAngle is not actually end
	if (currAngle > endServoAngle) {
		robot->SetServo(startServoAngle, endServoAngle, 0.0);
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

SensingBoulders::~SensingBoulders() {

}
