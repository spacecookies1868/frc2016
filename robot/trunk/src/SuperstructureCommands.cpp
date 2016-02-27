/*
 * SuperstructureCommands.cpp
 *
 *  Created on: Feb 25, 2016
 *      Author: origa_000
 */

#include "SuperstructureCommands.h"
#include <math.h>
#include "ini.h"
#include <iostream>
#include <string>
#include "Logger.h"
#include "Debugging.h"

#define PI 3.14159265358979

/*
 * Intake Position Command -- Just changes intake position
 */

IntakePositionCommand::IntakePositionCommand(SuperstructureController* mySuperstructure, bool myIntakeDown) {
	superstructure = mySuperstructure;
	isDone = false;
	intakeDown = myIntakeDown;
}

void IntakePositionCommand::Init() {
	if (intakeDown) {
		superstructure->SetAutoIntakeDown(true);
	} else {
		superstructure->SetAutoIntakeUp(true);
	}
}

void IntakePositionCommand::Update(double currTimeSec, double deltaTimeSec) {
	superstructure->Update(currTimeSec, deltaTimeSec);
	/*if (isDone) {
		superstructure->SetAutoIntakeDown(false);
		superstructure->SetAutoIntakeUp(false);
	}*/
	superstructure->Update(currTimeSec, deltaTimeSec);
	isDone = true;
}

bool IntakePositionCommand::IsDone() {
	return isDone;
}

IntakeRollersCommand::IntakeRollersCommand(SuperstructureController* mySuperstructure, bool rollForward, double myWaitTime) {
	superstructure = mySuperstructure;
	isDone = false;
	waitTime = myWaitTime;
	initTime = 0.0;
	firstTime = false;
	forward = rollForward;
}

void IntakeRollersCommand::Init() {
	firstTime = true;
}

void IntakeRollersCommand::Update(double currTimeSec, double deltaTimeSec) {
	if (firstTime) {
		initTime = currTimeSec;
		if (forward) {
			superstructure->SetAutoIntakeMotorForward(true);
		} else {
			superstructure->SetAutoIntakeMotorBackward(true);
		}
		firstTime = false;
	} else if ((currTimeSec - initTime) < waitTime) {
		if (forward) {
			superstructure->SetAutoIntakeMotorForward(true);
		} else {
			superstructure->SetAutoIntakeMotorBackward(true);
		}
	} else {
		superstructure->SetAutoIntakeMotorForward(false);
		superstructure->SetAutoIntakeMotorBackward(false);
		isDone = true;
	}
	superstructure->Update(currTimeSec, deltaTimeSec);
}

bool IntakeRollersCommand::IsDone() {
	return isDone;
}

IntakeCommand::IntakeCommand(SuperstructureController* mySuperstructure, double myWaitTime) {
	superstructure = mySuperstructure;
	waitTime = myWaitTime;
	isDone = false;
	initTime = 0.0;
	firstTime = true;
}

void IntakeCommand::Init() {
	firstTime = true;
}

void IntakeCommand::Update(double currTimeSec, double deltaTimeSec) {
	if(firstTime) {
		superstructure->SetAutoIntakeDown(true);
		initTime = currTimeSec;
		superstructure->SetAutoIntakeMotorForward(true);
		firstTime = false;
	} else if((currTimeSec - initTime) < waitTime) {
		superstructure->SetAutoIntakeMotorForward(true);
	}
	else {
		superstructure->SetAutoIntakeDown(false);
		superstructure->SetAutoIntakeMotorForward(false);
		isDone = true;
	}
	superstructure->Update(currTimeSec, deltaTimeSec);
}

bool IntakeCommand::IsDone() {
	return isDone;
}

DefenseManipPosCommand::DefenseManipPosCommand(SuperstructureController* mySuperstructure, bool downDesired) {
	superstructure = mySuperstructure;
	desiredDown = downDesired;
	isDone = false;
}

void DefenseManipPosCommand::Init() {
	isDone = false;
}

void DefenseManipPosCommand::Update(double currTimeSec, double deltaTimeSec) {
	if(desiredDown) {
		superstructure->SetAutoDefenseManipDown(true);
	}
	else {
		superstructure->SetAutoDefenseManipUp(true);
	}
	superstructure->Update(currTimeSec, deltaTimeSec);
	isDone = true;
}

bool DefenseManipPosCommand::IsDone() {
	return isDone;
}




