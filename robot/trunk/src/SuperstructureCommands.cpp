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

//Intake Position Command: changes the position of the intake

//Constructor: initializes variables and objects
IntakePositionCommand::IntakePositionCommand(SuperstructureController* mySuperstructure, bool myIntakeDown) {
	superstructure = mySuperstructure;
	isDone = false;
	intakeDown = myIntakeDown;
}

//sets the intake
void IntakePositionCommand::Init() {
	//if the intake is desired to be down, set it to down position;
	//otherwise, set it to up position
	if (intakeDown) {
		superstructure->SetAutoIntakeDown(true);
	} else {
		superstructure->SetAutoIntakeUp(true);
	}
}

//updates variables
void IntakePositionCommand::Update(double currTimeSec, double deltaTimeSec) {
	superstructure->Update(currTimeSec, deltaTimeSec);
	superstructure->Update(currTimeSec, deltaTimeSec);
	isDone = true;
}

//returns if the command is done
bool IntakePositionCommand::IsDone() {
	return isDone;
}

//Constructor: initializes variables and objects
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

//Constructor: initializes variables and objects
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

//returns if the command is done
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

//moves the intake and sets the rollers to grab the boulder
void IntakeCommand::Update(double currTimeSec, double deltaTimeSec) {
	//Puts the intake down and sets the rollers to intake the boulder
	//Once done (After waiting time), will out intake back up
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

//returns if command is done
bool IntakeCommand::IsDone() {
	return isDone;
}

DefenseManipPosCommand::DefenseManipPosCommand(SuperstructureController* mySuperstructure, bool downDesired) {
	superstructure = mySuperstructure;
	desiredDown = downDesired;
	isDone = false;
}

//sets the desired position of the defense manipulator
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
	isDone = false;
}

//sets the rollers to outake
void OuttakeCommand::Init() {
	superstructure->SetAutoOuttake(true);
}

//stops the rollers if done
void OuttakeCommand::Update(double currTimeSec, double deltaTimeSec) {
	if (superstructure->GetOuttakeFinished()) {
		isDone = true;
		superstructure->SetAutoOuttake(false);
	} else {
		superstructure->Update(currTimeSec, deltaTimeSec);
	}
}

//returns if command is done
bool OuttakeCommand::IsDone() {
	return isDone;
}

OuttakeByTimeCommand::OuttakeByTimeCommand(SuperstructureController* mySuperstructure, double myTime, bool myDefenseOut) {
	superstructure = mySuperstructure;
	defenseDown = new DefenseManipPosCommand(superstructure, true);
	wait = new WaitingCommand(0.5);
	isDone = false;
	time = myTime;
	initTime = 0.0;
	out = myDefenseOut;
}

void OuttakeByTimeCommand::Init() {
	if (out) {
		defenseDown->Init();
	} else {
		superstructure->SetAutoDefenseManipUp(true);
		superstructure->SetAutoIntakeUp(true);
	}
}

void OuttakeByTimeCommand::Update(double currTimeSec, double deltaTimeSec) {
	//If desired to be outaken through the defense manipulator...
	if (out) {
		//First, set defense down,wait, and then outake forward.
		//If done, stop rollers.
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
	} else {
		//outake until done
		initTime += deltaTimeSec;
		if (initTime <= time) {
			superstructure->SetAutoManualOuttakeReverse(true);
			superstructure->Update(currTimeSec, deltaTimeSec);
		} else {
			superstructure->SetAutoManualOuttakeReverse(false);
			superstructure->SetAutoIntakeUp(false);
			superstructure->SetAutoDefenseManipUp(false);
			superstructure->Update(currTimeSec, deltaTimeSec);
			isDone = true;
			initTime = 0.0;
		}
	}
}

bool OuttakeByTimeCommand::IsDone() {
	return isDone;
}

OuttakeByEncoderCommand::OuttakeByEncoderCommand(RobotModel* myRobot, SuperstructureController* mySuperstructure, double myRevs) {
	superstructure = mySuperstructure;
	robot = myRobot;
	desiredRevs = myRevs;
	isDone = false;
	initRevs = 0.0;
}

void OuttakeByEncoderCommand::Init() {
	initRevs = robot->GetOuttakeEncoderVal();
	if (desiredRevs < 0) {
		superstructure->SetAutoManualOuttakeReverse(true);
	} else {
		superstructure->SetAutoManualOuttakeForward(true);
	}
}

void OuttakeByEncoderCommand::Update(double currTimeSec, double deltaTimeSec) {
	double currRev = robot->GetOuttakeEncoderVal();
	if (fabs(currRev - initRevs) >= fabs(desiredRevs)) {
		superstructure->SetAutoManualOuttakeForward(false);
		superstructure->SetAutoManualOuttakeReverse(false);
		isDone = true;
	}
	superstructure->Update(currTimeSec, deltaTimeSec);
}

bool OuttakeByEncoderCommand::IsDone() {
	return isDone;
}

IntakeHoldCommand::IntakeHoldCommand(SuperstructureController* mySuperstructure) {
	superstructure = mySuperstructure;
	isDone = false;
}

void IntakeHoldCommand::Init() {
	superstructure->SetAutoBallInIntake(true);
}

void IntakeHoldCommand::Update(double currTimeSec, double deltaTimeSec) {
	if (superstructure->GetAutoBallInIntakeFinished()) {
		superstructure->SetAutoBallInIntake(false);
		isDone = true;
	}
	superstructure->Update(currTimeSec, deltaTimeSec);
}

bool IntakeHoldCommand::IsDone() {
	return isDone;
}
