#include "AutonomousController.h"
#include "AutoCommand.h"
#include "Debugging.h"
#include "Logger.h"

AutonomousController::AutonomousController(RobotModel* myRobot, DriveController* myDrive, SuperstructureController* mySuperstructure, CameraController* myCamera) {
	robot = myRobot;
	firstCommand = NULL;
	nextCommand = NULL;
	currentCommand = NULL;
	autoMode = 0;
	autoStart = 0;
	drive = myDrive;
	superstructure = mySuperstructure;
	camera = myCamera;
	timeFinished = 0.0;
}

/**
 * Creates the queue of AutoCommand instances
 */
void AutonomousController::StartAutonomous() {
	CreateQueue();
	currentCommand = firstCommand;
	if (currentCommand != NULL) {
		currentCommand->Init();
	}
}

/**
 * Calls the Init(), then Update(double currTimeSec, double deltaTimeSec) of the current
 * AutoCommand until IsDone() returns true.
 */
void AutonomousController::Update(double currTimeSec, double deltaTimeSec) {
	if (currentCommand != NULL) {
		if (currentCommand->IsDone()) {
			DO_PERIODIC(1, printf("Command complete at: %f \n", currTimeSec));
			currentCommand = currentCommand->GetNextCommand();
			if (currentCommand != NULL) {
				currentCommand->Init();
			} else {
				timeFinished = currTimeSec;
			}
		} else {
			currentCommand->Update(currTimeSec, deltaTimeSec);
		}
	} else {
		DO_PERIODIC(100, printf("Queue finished at: %f \n", timeFinished));
	}
}

/**
 * Stops the robot's movement and the whole queue.
 */
void AutonomousController::Reset() {
	firstCommand = NULL;
	currentCommand = NULL;
	/*
	robot->SetWheelSpeed(RobotModel::kFrontLeftWheel, 0.0);
	robot->SetWheelSpeed(RobotModel::kRearLeftWheel, 0.0);
	robot->SetWheelSpeed(RobotModel::kFrontRightWheel, 0.0);
	robot->SetWheelSpeed(RobotModel::kRearRightWheel, 0.0);
	*/
}

void AutonomousController::RefreshIni() {
	autoMode = robot->pini->geti("AUTONOMOUS","AUTOMODE",0);

#if USE_NAVX
	/*
	 * Setting all defaults to 0, should change when actually tune
	 */
	PivotCommand::rPFac = robot->pini->getf("PIVOTCOMMAND", "PFac", 0.0);
	PivotCommand::rIFac = robot->pini->getf("PIVOTCOMMAND", "IFac", 0.0);
	PivotCommand::rDFac = robot->pini->getf("PIVOTCOMMAND", "DFac", 0.0);
	PivotCommand::rDesiredAccuracy = robot->pini->getf("PIVOTCOMMAND", "DesiredAccuracy", 0.0);
	PivotCommand::rMaxAbsOutput = robot->pini->getf("PIVOTCOMMAND", "MaxAbsOutput", 0.0);
	PivotCommand::rMaxAbsError = robot->pini->getf("PIVOTCOMMAND", "MaxAbsError", 0.0);
	PivotCommand::rMaxAbsDiffError = robot->pini->getf("PIVOTCOMMAND", "MaxAbsDiffError", 0.0);
	PivotCommand::rMaxAbsITerm = robot->pini->getf("PIVOTCOMMAND", "MaxAbsITerm", 0.0);
	PivotCommand::rTimeLimit = robot->pini->getf("PIVOTCOMMAND", "TimeLimit", 0.0);

	PivotToAngleCommand::rPFac = robot->pini->getf("PIVOTTOANGLE", "PFac", 0.0);
	PivotToAngleCommand::rIFac = robot->pini->getf("PIVOTTOANGLE", "IFac", 0.0);
	PivotToAngleCommand::rDFac = robot->pini->getf("PIVOTTOANGLE", "DFac", 0.0);
	PivotToAngleCommand::rDesiredAccuracy = robot->pini->getf("PIVOTTOANGLE",
			"DesiredAccuracy", 0.0);
	PivotToAngleCommand::rMaxAbsOutput = robot->pini->getf("PIVOTTOANGLE",
			"MaxAbsOutput", 0.0);
	PivotToAngleCommand::rMaxAbsError = robot->pini->getf("PIVOTTOANGLE",
			"MaxAbsError", 0.0);
	PivotToAngleCommand::rMaxAbsDiffError = robot->pini->getf("PIVOTTOANGLE",
			"MaxAbsDiffError", 0.0);
	PivotToAngleCommand::rMaxAbsITerm = robot->pini->getf("PIVOTTOANGLE",
			"MaxAbsITerm", 0.0);
	PivotToAngleCommand::rTimeLimit = robot->pini->getf("PIVOTTOANGLE", "TimeLimit",
			0.0);
#endif


}

/**
 * Pushes new AutoCommand objects into the queue.
 * @param AutoCommand* myNewAutoCommand is the command to be pushed in, and
 * SimpleAutoCommand* myLastAutoCommand is the command that comes before it
 */
void AutonomousController::AddtoQueue(AutoCommand* myNewAutoCommand, SimpleAutoCommand* myLastAutoCommand) {
	myLastAutoCommand->SetNextCommand(myNewAutoCommand);
}

/**
 * Contains many different states for different versions of autonomous, all
 * of which have different AutoCommand objects in their queues.
 */
void AutonomousController::CreateQueue() {
	firstCommand = NULL;
	printf("AutoMode: %i \n", autoMode);

	switch (autoMode) {
	case (kTestAuto): {
		printf("kTestAuto ------------------\n");
		PivotToAngleCommand* p = new PivotToAngleCommand(robot, 0.0);
		firstCommand = p;
		break;
	}
	case (kBlankAuto): {
		printf("kBlankAuto ----------------------\n");
		break;
	}
	case (kReachAuto): {
		printf("kReachAuto ------------------------\n");
		break;
	}
	case (kCrossAuto): {
		printf("kCrossAuto -----------------------------\n");
		break;
	}
	case (kShootAuto): {
		printf("kShootAuto --------------------------------\n");
		break;
	}
	case (kHoardingAuto): {
		printf("kHoardingAuto ------------------------------\n");
		break;
	}
	}
}
