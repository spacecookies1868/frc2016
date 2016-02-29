#include <DefenseDrive.h>

DefenseDrive::DefenseDrive(RobotModel* myRobot) {
	robot = myRobot;
	currState = kInit;
	nextState = kBeforeUpRamp;
	currRoll = 0.0;
	lastRoll = 0.0;
	speed = 0.0;
	startRoll = 0.0;
	isFlatThreshold = 0.0;
	onRampThreshold = 0.0;
	diffRoll = 0.0;
	startTime = 0.0;
	startWaitTime = 0.0;
	pivotToAngleCommand = nullptr;
}

void DefenseDrive::Init() {
	speed = 0.5;

	isFlatThreshold = 2.0;
	onRampThreshold = 5.0;

	currRoll = robot->GetRoll();
	startRoll = currRoll;

	currState = kInit;
	nextState = kBeforeUpRamp;

	startTime = 0.0;
}

void DefenseDrive::Update(double currTimeSec, double deltaTimeSec) {
	lastRoll = currRoll;
	currRoll = robot->GetRoll();
	diffRoll = currRoll - startRoll;

	//printf("diff Roll: %f\n", diffRoll);

	//SmartDashboard::PutNumber("diff Roll: ", diffRoll);

	double startYaw = 0.0;

	switch(currState) {
	case (kInit):
		printf("next state: kInit\n");
		startYaw = robot->GetYaw();
		printf("pivotToAngle yaw: %f\n", startYaw);
		pivotToAngleCommand = new PivotToAngleCommand(robot, startYaw);
		pivotToAngleCommand->Init();
		nextState = kBeforeUpRamp;
		printf("next state: kBeforeUpRamp\n");
		startTime = currTimeSec;
		break;
	case (kBeforeUpRamp):
		robot->SetWheelSpeed(RobotModel::kAllWheels, speed);
		if (diffRoll > onRampThreshold) {
			nextState = kUpRamp;
			printf("next state: kUpRamp\n");
		} else {
			nextState = kBeforeUpRamp;
		}
		break;
	case (kUpRamp):
		robot->SetWheelSpeed(RobotModel::kAllWheels, speed);
		if (abs(diffRoll) < isFlatThreshold) {
			nextState = kMiddleRamp;
			printf("next state: kMiddleRamp\n");
		} else {
			nextState = kUpRamp;
		}
		break;
	case (kMiddleRamp):
		robot->SetWheelSpeed(RobotModel::kAllWheels, speed);
		//if (diffRoll < -onRampThreshold) {
		if (diffRoll <= -isFlatThreshold) {
			nextState = kDownRamp;
			printf("next state: kDownRamp\n");
		} else {
			nextState = kMiddleRamp;
		}
		break;
	case (kDownRamp):
		robot->SetWheelSpeed(RobotModel::kAllWheels, speed);
		if (diffRoll > -isFlatThreshold){
			startWaitTime = currTimeSec;
			nextState = kWait;
			printf("next state: kWait\n");
		} else {
			nextState = kDownRamp;
		}
		break;
	case (kWait) :
		if ((currTimeSec - startWaitTime) > 0.1) {
			nextState = kStraighten;
			printf("next state: kStraighten\n");
		} else {
			nextState = kWait;
		}
		break;
	case (kStraighten) :
		if (pivotToAngleCommand->IsDone()) {
			nextState = kDone;
			printf("next state: kDone\n");
		} else {
			pivotToAngleCommand->Update(currTimeSec, deltaTimeSec);
			printf("yaw: %f\n", robot->GetYaw());
			nextState = kStraighten;
		}
		break;
	case (kDone) :
		robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
		nextState = kDone;
		break;
	}

	currState = nextState;

	SmartDashboard::PutNumber("state: ", currState);

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
