#include "AutonomousController.h"
#include "AutoCommand.h"
#include "Debugging.h"

AutonomousController::AutonomousController(RobotModel* myRobot) {
	robot = myRobot;
	firstCommand = NULL;
	nextCommand = NULL;
	currentCommand = NULL;
	autoMode = 0;
	autoStart = 0;
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
			}
		} else {
			currentCommand->Update(currTimeSec, deltaTimeSec);
		}
	} else {
		DO_PERIODIC(100, printf("Queue finished at: %f \n", currTimeSec));
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

	}
}
