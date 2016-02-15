#include <SensingBoulders.h>

SensingBoulders::SensingBoulders(RobotModel* myRobot) {
	robot = myRobot;
	ultrasonicSensor = new UltrasonicSensor(0);
}

void SensingBoulders::Init() {
	robot->InitServo(20);

	startBoulderAngle = robot->GetServoAngle();
	endBoulderAngle = 0.0;
	centerBoulderAngle = 0.0;

	currAngle = robot->GetServoAngle();
	currDistance = ultrasonicSensor->GetRangeInInches();

	startBoulderDistance = ultrasonicSensor->GetRangeInInches();
	endBoulderDistance = ultrasonicSensor->GetRangeInInches();
	centerBoulderDistance = 0.0;

	startServoAngle = 20;
	endServoAngle = 160;
	deltaServoAngle = 0.25;
	thresholdBoulderDistance = 1.2;	// 20 percent from minBoulderDistance
	isDone = false;

	PrintState();
}

void SensingBoulders::Update() {
	currAngle = robot->GetServoAngle();
	currDistance = ultrasonicSensor->GetRangeInInches();
	if (currDistance * thresholdBoulderDistance < startBoulderDistance) {
		startBoulderDistance = currDistance;
		startBoulderAngle = currAngle;
		printf("changed start boulder\n");
		printf("currDistance: %f\n", currDistance);
		printf("currAngle: %f\n", currAngle);
		PrintState();

	}
//	printf("startBoulderDistance: %f\n", startBoulderDistance);

	if (currDistance > thresholdBoulderDistance * startBoulderDistance) {
		endBoulderDistance = currDistance;
		endBoulderAngle = currAngle;
//		centerBoulderAngle = (startBoulderAngle + endBoulderAngle) / 2;
//		centerBoulderDistance = (startBoulderDistance + endBoulderDistance) / 2;
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

SensingBoulders::~SensingBoulders() {

}

void SensingBoulders::PrintState() {
	printf("Start Boulder Angle    : %f\n", startBoulderAngle);
	printf("End Boulder Angle      : %f\n", endBoulderAngle);
	printf("Start Boulder Distance : %f\n", startBoulderDistance);
//	printf("End Boulder Distance   : %f\n", endBoulderDistance);
}
