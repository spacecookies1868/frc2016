/*
 * SuperstructureCommands.h
 *
 *  Created on: Feb 25, 2016
 *      Author: origa_000
 */

#ifndef SRC_SUPERSTRUCTURECOMMANDS_H_
#define SRC_SUPERSTRUCTURECOMMANDS_H_

#include "WPILib.h"
#include "Debugging.h"
#include "RobotModel.h"
#include "SuperstructureController.h"
#include "ControlBoard.h"
#include <vector>
#include <string>
#include <iostream>
#include "PIDControlLoop.h"
#include "AutoCommand.h"

using namespace std;

//Intake Position Command: changes the position of the intake
class IntakePositionCommand : public SimpleAutoCommand {
public:
	IntakePositionCommand(SuperstructureController* mySuperstructure, bool myIntakeDown);
	~IntakePositionCommand() {}
	void Init(); //sets the intake
	void Update(double currTimeSec, double deltaTimeSec); //updates variables
	bool IsDone(); //returns if the command is done
private:
	SuperstructureController* superstructure;
	bool isDone;
	bool intakeDown;
};

//Intake Rollers Command: turns the intake rollers forward or backwards for a given time
class IntakeRollersCommand : public SimpleAutoCommand {
public:
	IntakeRollersCommand(SuperstructureController* mySuperstructure, bool rollForward, double myWaitTime);
	~IntakeRollersCommand() {}
	void Init();
	void Update(double currTimeSec, double deltaTimeSec); //sets the direction of the intake rollers for a given time
	bool IsDone(); //returns if the command is done
private:
	SuperstructureController* superstructure;
	bool isDone;
	double waitTime;
	double initTime;
	bool firstTime;
	bool forward;
};

//Intake Command: intakes the boulder assuming it is in front of robot
class IntakeCommand : public SimpleAutoCommand {
public:
	IntakeCommand(SuperstructureController* mySuperstructure, double myWaitTime);
	~IntakeCommand() {}
	void Init();
	void Update(double currTimeSec, double deltaTimeSec); //moves the intake and sets the rollers to grab the boulder
	bool IsDone(); //returns if command is done

private:
	SuperstructureController* superstructure;
	double waitTime;
	bool isDone;
	double initTime;
	bool firstTime;
};

//Defense Manipulator Position Command: sets the position of the defense manipulator
class DefenseManipPosCommand : public SimpleAutoCommand {
public:
	DefenseManipPosCommand(SuperstructureController* mySuperstructure, bool downDesired);
	~DefenseManipPosCommand() {}
	void Init(); //sets the desired position of the defense manipulator
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone(); //returns if command is done
private:
	SuperstructureController* superstructure;
	bool isDone;
	bool desiredDown;
};

//Outake Command: outakes an intaken boulder
class OuttakeCommand : public SimpleAutoCommand {
public:
	OuttakeCommand(SuperstructureController* mySuperstructure);
	~OuttakeCommand() {}
	void Init(); //sets the rollers to outake
	void Update(double currTimeSec, double deltaTimeSec); //stops the rollers if done
	bool IsDone(); //returns if command is done
private:
	SuperstructureController* superstructure;
	//DefenseManipPosCommand* defenseDown;
	bool isDone;
};

//Outake By Time Command: outakes the ball for a given time through either the front (intake-side) or through back (defense manipulator)
class OuttakeByTimeCommand : public SimpleAutoCommand {
public:
	OuttakeByTimeCommand(SuperstructureController* mySuperstructure, double myTime, bool myDefenseOut);
	~OuttakeByTimeCommand() {}
	void Init();
	void Update(double currTimeSec, double deltaTimeSec); //outakes the ball for a given time through either the front (intake-side) or through back (defense manipulator)
	bool IsDone(); //returns if command is done
private:
	SuperstructureController* superstructure;
	WaitingCommand* wait;
	DefenseManipPosCommand* defenseDown;
	bool isDone;
	bool out;
	double time;
	double initTime;
};

//Outake By Encoder Command: outakes the bouldern for a given numer of revolutions
class OuttakeByEncoderCommand : public SimpleAutoCommand {
public:
	OuttakeByEncoderCommand(RobotModel* myRobot, SuperstructureController* mySuperstructure, double myRevs);
	~OuttakeByEncoderCommand() {}
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
private:
	SuperstructureController* superstructure;
	RobotModel* robot;
	double desiredRevs;
	bool isDone;
	double initRevs;
};

//Intake Hold Command: intakes a boulder and holds it in the intake
class IntakeHoldCommand : public SimpleAutoCommand {
public:
	IntakeHoldCommand(SuperstructureController* mySuperstructure);
	~IntakeHoldCommand() {}
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
private:
	SuperstructureController* superstructure;
	bool isDone;
};

#endif /* SRC_SUPERSTRUCTURECOMMANDS_H_ */
