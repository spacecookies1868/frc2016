#include <MoatDrive.h>

MoatDrive::MoatDrive(RobotModel* myRobot) {
	robot = myRobot;
	currState = kInit;
	nextState = kBeforeUpRamp;
	currRoll = 0.0;
	lastRoll = 0.0;
	speed = 0.0;
	startRoll = 0.0;
	diffRoll = 0.0;

	isFlatThreshold = 0.0;
	onRampThreshold = 0.0;
	endThreshold = 0.0;
	rollDerivativeIsFlatThreshold = 0.0;

	derivativeOfRoll = 0.0;
	startTime = 0.0;
	startWaitTime = 0.0;

	pivotToAngleCommand = nullptr;
	driveStraightCommand = nullptr;
}

void MoatDrive::Init() {
	speed = 0.65;

	isFlatThreshold = 12.0;
	rollDerivativeIsFlatThreshold = 35.0;
	onRampThreshold = 16.0;

	currRoll = robot->GetRoll();
	startRoll = currRoll;

	currState = kInit;
	nextState = kBeforeUpRamp;

	startTime = 0.0;
}

void MoatDrive::Update(double currTimeSec, double deltaTimeSec) {
	lastRoll = currRoll;
	currRoll = robot->GetRoll();
	diffRoll = currRoll - startRoll;

	double deltaRoll = lastRoll - currRoll;
	derivativeOfRoll = deltaRoll / deltaTimeSec;
	double startYaw = 0.0;

	switch(currState) {
	case (kInit):
		printf("next state: kInit\n");
		startYaw = robot->GetYaw();
		printf("pivotToAngle yaw: %f\n", startYaw);
		pivotToAngleCommand = new PivotToAngleCommand(robot, startYaw);
		pivotToAngleCommand->Init();

		driveStraightCommand = new DriveStraightCommand(robot, speed);
		driveStraightCommand->Init();

		nextState = kBeforeUpRamp;
		printf("next state: kBeforeUpRamp\n");
		startTime = currTimeSec;
		break;
	case (kBeforeUpRamp):
		//robot->SetWheelSpeed(RobotModel::kAllWheels, speed);
		driveStraightCommand->Update(currTimeSec, deltaTimeSec);
		if (diffRoll > onRampThreshold) {
			nextState = kUpRamp;
			printf("next state: kUpRamp\n");
		} else {
			nextState = kBeforeUpRamp;
		}
		break;
	case (kUpRamp):
		//robot->SetWheelSpeed(RobotModel::kAllWheels, speed);
		driveStraightCommand->Update(currTimeSec, deltaTimeSec);
		if ((fabs(derivativeOfRoll) < rollDerivativeIsFlatThreshold) &&
			(fabs(diffRoll) < isFlatThreshold)) {
		//if (fabs(diffRoll) < isFlatThreshold) {
			nextState = kMiddleRamp;
			printf("next state: kMiddle\n");
		} else {
			nextState = kUpRamp;
		}
		break;
	case (kMiddleRamp):
		//robot->SetWheelSpeed(RobotModel::kAllWheels, speed);
		driveStraightCommand->Update(currTimeSec, deltaTimeSec);
		if (diffRoll <= -isFlatThreshold) {
			nextState = kDownRamp;
			printf("next state: kDownRamp\n");
		} else {
			nextState = kMiddleRamp;
		}
		break;
	case (kDownRamp):
		driveStraightCommand->Update(currTimeSec, deltaTimeSec);
		//robot->SetWheelSpeed(RobotModel::kAllWheels, speed);
		//if (diffRoll > -endThreshold){
		if ((fabs(derivativeOfRoll) < rollDerivativeIsFlatThreshold) &&
				(fabs(diffRoll) < isFlatThreshold)) {
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
			printf("done time: %f", currTimeSec - startTime);
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

	if (currTimeSec - startTime > 3.0) {		// safeguard to 2.5 sec
		printf("done from timer: %f", currTimeSec - startTime);
		SmartDashboard::PutString("Done from ", "timer");
		currState = kDone;
	}
}

int MoatDrive::GetState() {
	return currState;
}

bool MoatDrive::IsDone() {
	return (currState == kDone) ;
}

MoatDrive::~MoatDrive() {
}
