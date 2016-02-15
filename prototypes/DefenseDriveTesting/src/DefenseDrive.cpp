#include <DefenseDrive.h>

DefenseDrive::DefenseDrive(RobotModel* myRobot) {
	robot = myRobot;
	currState = kInit;
	nextState = kBeforeUpRamp;
	currRoll = 0.0;
	lastRoll = 0.0;
	deltaRoll = 0.0;
	speed = 0.0;
	startRoll = 0.0;
	isFlatThreshold = 0.0;
	onRampThreshold = 0.0;
	slopeOfRamp = 0.0;
	diffRoll = 0.0;
}

void DefenseDrive::Init() {
	speed = 0.4;
	isFlatThreshold = 2.0;
	onRampThreshold = 5.0;
	slopeOfRamp = 3.0;
	currRoll = robot->GetRoll();
	startRoll = robot->GetRoll();
	deltaRoll = 0.0;
	currState = kInit;
	nextState = kBeforeUpRamp;
	startRoll = 0.0;
}

void DefenseDrive::Update(double currTimeSec, double deltaTimeSec) {
	lastRoll = currRoll;
	currRoll = robot->GetRoll();
	printf("curr Roll: %f\n", currRoll);
	deltaRoll = currRoll - lastRoll;
	printf("deltaRoll: %f\n", deltaRoll);
	diffRoll = currRoll - startRoll;
	printf("diff Roll: %f\n", diffRoll);

	switch(currState) {
	case (kInit):
		nextState = kBeforeUpRamp;
		printf("kInit\n");
		break;
	case (kBeforeUpRamp):
		robot->SetWheelSpeed(RobotModel::kAllWheels, speed);
		printf("Wheel Speed: %f\n", speed);
		if (diffRoll > onRampThreshold) {
			nextState = kUpRamp;
		} else {
			nextState = kBeforeUpRamp;
		}
		printf("kBeforeUpRamp\n");
		break;
	case (kUpRamp):
		robot->SetWheelSpeed(RobotModel::kAllWheels, speed);
		if (diffRoll < isFlatThreshold) {
			nextState = kMiddleRamp;
		} else {
			nextState = kUpRamp;
		}
		printf("kUpRamp\n");
		break;
	case (kMiddleRamp):
		robot->SetWheelSpeed(RobotModel::kAllWheels, speed);
		if (diffRoll < -onRampThreshold) {
			nextState = kDownRamp;
		} else {
			nextState = kMiddleRamp;
		}
		printf("kMiddleRamp\n");
		break;
	case (kDownRamp):
		robot->SetWheelSpeed(RobotModel::kAllWheels, speed);
		if (diffRoll > -isFlatThreshold){
			nextState = kDone;
		} else {
			nextState = kDownRamp;
		}
		printf("kDownRamp\n");
		break;
	case (kDone):
		robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
		nextState = kDone;
		printf("kDone\n");	// never actually enters kDone in this program
		break;
	}
	currState = nextState;
}

bool DefenseDrive::IsDone() {
	return (currState == kDone) ;
}

DefenseDrive::~DefenseDrive() {
}
