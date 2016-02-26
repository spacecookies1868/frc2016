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
	startTime = 0.0;
}

void DefenseDrive::Init() {
	speed = 0.5;
	isFlatThreshold = 2.0;
	onRampThreshold = 5.0;
	slopeOfRamp = 3.0;
	currRoll = robot->GetRoll();
	startRoll = robot->GetRoll();
	deltaRoll = 0.0;
	currState = kInit;
	nextState = kBeforeUpRamp;
	startRoll = 0.0;
	startTime = 0.0;
}

void DefenseDrive::Update(double currTimeSec, double deltaTimeSec) {
	lastRoll = currRoll;
	currRoll = robot->GetRoll();
	deltaRoll = currRoll - lastRoll;
	diffRoll = currRoll - startRoll;

	printf("curr Roll: %f\n", currRoll);
	printf("deltaRoll: %f\n", deltaRoll);
	printf("diff Roll: %f\n", diffRoll);

	SmartDashboard::PutNumber("curr Roll: ", currRoll);
	SmartDashboard::PutNumber("delta Roll: ", deltaRoll);
	SmartDashboard::PutNumber("diff Roll: ", diffRoll);

	switch(currState) {
	case (kInit):
		nextState = kBeforeUpRamp;
		startTime = currTimeSec;
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
		if (abs(diffRoll) < isFlatThreshold) {		// maybe try abs(diffRoll)
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

	SmartDashboard::PutNumber("state: ", currState);
	currState = nextState;

	if (currTimeSec - startTime > 3.5) {		// safeguard to 2.5 sec
		printf("done from timer: %f", currTimeSec - startTime);
		SmartDashboard::PutString("Done from ", "timer");
		currState = kDone;
	}
}

int DefenseDrive::GetState() {
	return currState;
}

bool DefenseDrive::IsDone() {
	return (currState == kDone) ;
}

DefenseDrive::~DefenseDrive() {
}
