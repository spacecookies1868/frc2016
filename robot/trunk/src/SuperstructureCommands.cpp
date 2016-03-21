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
		superstructure->Update(currTimeSec, deltaTimeSec);
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
	if (desiredDown) {
		superstructure->SetAutoDefenseManipDown(true);
	} else {
		superstructure->SetAutoDefenseManipUp(true);
	}
}

void DefenseManipPosCommand::Update(double currTimeSec, double deltaTimeSec) {
	superstructure->Update(currTimeSec, deltaTimeSec);
	superstructure->Update(currTimeSec, deltaTimeSec);
	isDone = true;
}

bool DefenseManipPosCommand::IsDone() {
	return isDone;
}

OuttakeCommand::OuttakeCommand(SuperstructureController* mySuperstructure) {
	superstructure = mySuperstructure;
	//defenseDown = new DefenseManipPosCommand(superstructure, true);
	isDone = false;
}

void OuttakeCommand::Init() {
	//defenseDown->Init();
	superstructure->SetAutoOuttake(true);
}

void OuttakeCommand::Update(double currTimeSec, double deltaTimeSec) {
	//if (!defenseDown->IsDone()) {
		//defenseDown->Update(currTimeSec, deltaTimeSec);
	//} else
	if (superstructure->GetOuttakeFinished()) {
		isDone = true;
		superstructure->SetAutoOuttake(false);
	} else {
		superstructure->Update(currTimeSec, deltaTimeSec);
	}
}

bool OuttakeCommand::IsDone() {
	return isDone;
}

OuttakeByTimeCommand::OuttakeByTimeCommand(SuperstructureController* mySuperstructure, double myTime) {
	superstructure = mySuperstructure;
	defenseDown = new DefenseManipPosCommand(superstructure, true);
	wait = new WaitingCommand(0.5);
	isDone = false;
	time = myTime;
	initTime = 0.0;
}

void OuttakeByTimeCommand::Init() {
	defenseDown->Init();
}

void OuttakeByTimeCommand::Update(double currTimeSec, double deltaTimeSec) {
	if (!defenseDown->IsDone()) {
		defenseDown->Update(currTimeSec, deltaTimeSec);
		wait->Init();
	} else if (!wait->IsDone()) {
		wait->Update(currTimeSec, deltaTimeSec);
		initTime = currTimeSec;
	} else if ((currTimeSec - initTime) <= time) {
		printf("Outtaking \n");
		superstructure->SetAutoManualOuttakeForward(true);
		superstructure->Update(currTimeSec, deltaTimeSec);
	} else {
		printf("Done \n");
		superstructure->SetAutoManualOuttakeForward(false);
		superstructure->Update(currTimeSec, deltaTimeSec);
		isDone = true;
	}
}

bool OuttakeByTimeCommand::IsDone() {
	return isDone;
}
