#include "AutonomousController.h"
#include "AutoCommand.h"
#include "Debugging.h"
#include "Logger.h"

AutonomousController::AutonomousController(RobotModel* myRobot, DriveController* myDrive, SuperstructureController* mySuperstructure,
		CameraController* myCamera,	RemoteControl* myHumanControl) {
	robot = myRobot;
	firstCommand = NULL;
	nextCommand = NULL;
	currentCommand = NULL;
	autoMode = 0;
	autoStart = 0;
	drive = myDrive;
	superstructure = mySuperstructure;
	camera = myCamera;
	humanControl = myHumanControl;
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
	humanControl->ReadControls();
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

	DriveStraightCommand::disPFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "disPFac", 0.0);
	DriveStraightCommand::disIFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "disIFac", 0.0);
	DriveStraightCommand::disDFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "disDFac", 0.0);
	DriveStraightCommand::disDesiredAccuracy = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "disDesiredAccuracy", 0.0);
	DriveStraightCommand::disMaxAbsOutput = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "disMaxAbsOutput", 0.0);
	DriveStraightCommand::disMaxAbsError = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "disMaxAbsError", 0.0);
	DriveStraightCommand::disMaxAbsDiffError = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "disMaxAbsDiffError", 0.0);
	DriveStraightCommand::disMaxAbsITerm = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "disMaxAbsITerm", 0.0);
	DriveStraightCommand::disTimeLimit = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "disTimeLimit", 0.0);

	DriveStraightCommand::rPFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rPFac", 0.0);
	DriveStraightCommand::rIFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rIFac", 0.0);
	DriveStraightCommand::rDFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rDFac", 0.0);
	DriveStraightCommand::rDesiredAccuracy = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rDesiredAccuracy", 0.0);
	DriveStraightCommand::rMaxAbsOutput = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rMaxAbsOutput", 0.0);
	DriveStraightCommand::rMaxAbsError = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rMaxAbsError", 0.0);
	DriveStraightCommand::rMaxAbsDiffError = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rMaxAbsDiffError", 0.0);
	DriveStraightCommand::rMaxAbsITerm = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rMaxAbsITerm", 0.0);
	DriveStraightCommand::rTimeLimit = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rTimeLimit", 0.0);

	CurveCommand::radiusPFac = robot->pini->getf("CURVECOMMAND", "radiusPFac", 0.0);
	CurveCommand::radiusIFac = robot->pini->getf("CURVECOMMAND", "radiusIFac", 0.0);
	CurveCommand::radiusDFac = robot->pini->getf("CURVECOMMAND", "radiusDFac", 0.0);
	CurveCommand::radiusDesiredAccuracy = robot->pini->getf("CURVECOMMAND", "radiusDesiredAccuracy", 0.0);
	CurveCommand::radiusMaxAbsOutput = robot->pini->getf("CURVECOMMAND", "radiusMaxAbsOutput", 0.0);
	CurveCommand::radiusMaxAbsError = robot->pini->getf("CURVECOMMAND", "radiusMaxAbsError", 0.0);
	CurveCommand::radiusMaxAbsDiffError = robot->pini->getf("CURVECOMMAND", "radiusMaxAbsDiffError", 0.0);
	CurveCommand::radiusMaxAbsITerm = robot->pini->getf("CURVECOMMAND", "radiusMaxAbsITerm", 0.0);
	CurveCommand::radiusTimeLimit = robot->pini->getf("CURVECOMMAND", "radiusTimeLimit", 0.0);

	CurveCommand::anglePFac = robot->pini->getf("CURVECOMMAND", "anglePFac", 0.0);
	CurveCommand::angleIFac = robot->pini->getf("CURVECOMMAND", "angleIFac", 0.0);
	CurveCommand::angleDFac = robot->pini->getf("CURVECOMMAND", "angleDFac", 0.0);
	CurveCommand::angleDesiredAccuracy = robot->pini->getf("CURVECOMMAND", "angleDesiredAccuracy", 0.0);
	CurveCommand::angleMaxAbsOutput = robot->pini->getf("CURVECOMMAND", "angleMaxAbsOutput", 0.0);
	CurveCommand::angleMaxAbsError = robot->pini->getf("CURVECOMMAND", "angleMaxAbsError", 0.0);
	CurveCommand::angleMaxAbsDiffError = robot->pini->getf("CURVECOMMAND", "angleMaxAbsDiffError", 0.0);
	CurveCommand::angleMaxAbsITerm = robot->pini->getf("CURVECOMMAND", "angleMaxAbsITerm", 0.0);
	CurveCommand::angleTimeLimit = robot->pini->getf("CURVECOMMAND","angleTimeLimit", 0.0);

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

		IntakePositionCommand* down = new IntakePositionCommand(superstructure, true);
		firstCommand = down;
		IntakeRollersCommand* forward = new IntakeRollersCommand(superstructure, true, 2.0);
		down->SetNextCommand(forward);
		IntakePositionCommand* up = new IntakePositionCommand(superstructure, false);
		forward->SetNextCommand(up);
		IntakeRollersCommand* backward = new IntakeRollersCommand(superstructure, false, 2.0);
		up->SetNextCommand(backward);

//		CurveCommand* c3 = new CurveCommand(robot, 0.0, 6.0);
//		firstCommand = c3;
		break;
	}
	case (kBlankAuto): {
		printf("kBlankAuto ----------------------\n");
		break;
	}
	case (kReachAuto): {
		/*
		 * Assumption: starting position is back of robot on auto line
		 * Length of robot is 2.823ft
		 * Distance from auto line to outerworks is 6.167;
		 */
		DriveStraightCommand* reachDrive = new DriveStraightCommand(robot, 3.35);
		firstCommand = reachDrive;
		printf("kReachAuto ------------------------\n");
		break;
	}
	case (kCrossAuto): {
		printf("kCrossAuto -----------------------------\n");
		/*
		 * Assumption: starting position is back of robot on auto line
		 * Length of robot is 2.823 ft
		 * Distance from auto line to end of autoworks is
		 * Added clearance of 3 ft
		 */
		switch (humanControl->GetDefense()) {
		case (LowBar): {
			printf("Autonomous Controller, Low Bar \n");
			DefenseCommand* lowBarCross = new DefenseCommand(robot, superstructure, DefenseCommand::LowBar);
			firstCommand = lowBarCross;
			break;
		}
		case (Portcullis): {
			printf("Autonomous Controller, Portcullis \n");
			DefenseCommand* portcullisCross = new DefenseCommand(robot, superstructure, DefenseCommand::Portcullis);
			firstCommand = portcullisCross;
			break;
		}
		case (ChevalDeFrise): {
			printf("Autonomous Controller, Cheval de Frise \n");
			DefenseCommand* chevalDeFriseCross = new DefenseCommand(robot, superstructure, DefenseCommand::ChevalDeFrise);
			firstCommand = chevalDeFriseCross;
			break;
		}
		case (Ramparts): {
			printf("Autonomous Controller, Ramparts \n");
			DefenseCommand* rampartsCross = new DefenseCommand(robot, superstructure, DefenseCommand::Ramparts);
			firstCommand = rampartsCross;
			break;
		}
		case (Moat): {
			printf("Autonomous Controller, Moat \n");
			DefenseCommand* moatCross = new DefenseCommand(robot, superstructure, DefenseCommand::Moat);
			firstCommand = moatCross;
			break;
		}
		case (SallyPort): {
			printf("Autonomous Controller, Sally Port \n");
			DefenseCommand* sallyPortCross = new DefenseCommand(robot, superstructure, DefenseCommand::SallyPort);
			firstCommand = sallyPortCross;
			break;
		}
		case (Drawbridge): {
			printf("Autonomous Controller, Drawbridge \n");
			DefenseCommand* drawbridgeCross = new DefenseCommand(robot, superstructure, DefenseCommand::Drawbridge);
			firstCommand = drawbridgeCross;
			break;
		}
		case (RockWall): {
			printf("Autonomous Controller, Rock Wall \n");
			DefenseCommand* rockWallCross = new DefenseCommand(robot, superstructure, DefenseCommand::RockWall);
			firstCommand = rockWallCross;
			break;
		}
		case (RoughTerrain): {
			printf("Autonomous Controller, Rough Terrain \n");
			DefenseCommand* roughTerrainCross = new DefenseCommand(robot, superstructure, DefenseCommand::RoughTerrain);
			firstCommand = roughTerrainCross;
			break;
		}
		}
		break;
	}
	case (kShootAuto): {
		printf("kShootAuto --------------------------------\n");
		/*
		 * Assumption: starting position is back of robot on auto line
		 * GOING THROUGH LOW BAR IN THESE CALCULATIONS
		 * Length of robot is 2.823 ft
		 * Distance from auto line to end of autoworks is
		 *
		 */

		break;
	}
	case (kHoardingAuto): {
		printf("kHoardingAuto ------------------------------\n");
		break;
	}
	}
}
