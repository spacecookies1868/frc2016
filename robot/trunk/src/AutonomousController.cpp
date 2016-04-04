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
	hardCodeShoot = true;
	secondDefensePos = 0;
	useSallyPort = true;
}

/**
 * Creates the queue of AutoCommand instances
 */
void AutonomousController::StartAutonomous() {
	humanControl->ReadControls();
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

	robot->ShiftToLowGear();
	robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
}

void AutonomousController::RefreshIni() {
	autoMode = robot->pini->geti("AUTONOMOUS","AutoMode",0);
	hardCodeShoot = robot->pini->getbool("AUTONOMOUS", "HardCodeShoot", true);
	secondDefensePos = robot->pini->geti("AUTONOMOUS", "SecondDefense", 0);
	useSallyPort = robot->pini->getbool("AUTONOMOUS", "UseSallyPort", true);

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
//	if (humanControl->GetStopAutoDesired()) {
//		autoMode = kBlankAuto;
//	}
	switch (autoMode) {
	case (kTestAuto): {
		printf("kTestAuto ------------------\n");
		DUMP("TEST AUTO", 0.0);
//		DriveStraightCommand* driveStraight = new DriveStraightCommand(robot, 10.0);
//		firstCommand =  driveStraight;
//		IntakeCommand* intakeTest = new IntakeCommand(superstructure, 2.0);
//		firstCommand = intakeTest;
//		OuttakeByTimeCommand* outtakeTest = new OuttakeByTimeCommand(superstructure, 3.0);
//		intakeTest->SetNextCommand(outtakeTest);
//		firstCommand = outtakeTest;
//		CurveCommand* testmurple = new CurveCommand(robot, 3.0, 12.3);
//		firstCommand = testmurple;
//		PivotToAngleCommand* pivotmurple = new PivotToAngleCommand(robot, 42.0);
//		testmurple->SetNextCommand(pivotmurple);
//		OuttakeByTimeCommand* outtakemurple = new OuttakeByTimeCommand(superstructure, 1.0);
//		pivotmurple->SetNextCommand(outtakemurple);
		IntakePositionCommand* intakeUp = new IntakePositionCommand(
				superstructure, false);
		DefenseManipPosCommand* defenseUp = new DefenseManipPosCommand(
				superstructure, false);
		ParallelAutoCommand* mechanismsUp = new ParallelAutoCommand(
				intakeUp, defenseUp);
		firstCommand = mechanismsUp;
		//SHOOTING IN THE GOAL

		CurveCommand* driveToGoal = new CurveCommand(robot, 1.8, 6.8); //was 0.8 for x
		mechanismsUp->SetNextCommand(driveToGoal);
		PivotToAngleCommand* pivotForGoal = new PivotToAngleCommand(robot, 330.0);
		driveToGoal->SetNextCommand(pivotForGoal);
		OuttakeByTimeCommand* shooting = new OuttakeByTimeCommand(
				superstructure, 1.0);
		pivotForGoal->SetNextCommand(shooting);

		//LINING UP
		DriveStraightCommand* driveOffBatter = new DriveStraightCommand(
				robot, -0.0);
		shooting->SetNextCommand(driveOffBatter);
		PivotToAngleCommand* LiningUp = new PivotToAngleCommand(robot,
				270.0);
				driveOffBatter->SetNextCommand(LiningUp);

		//DRIVING TO CAT C
		CurveCommand* drivingToCatCT = new CurveCommand(robot,
				-8.2 + 4.2 * 2, -10.0);
		LiningUp->SetNextCommand(drivingToCatCT);
		PivotToAngleCommand* straightenMe = new PivotToAngleCommand(robot,
				270.0); //should be -90
		drivingToCatCT->SetNextCommand(straightenMe);
		if (!useSallyPort) {
			//DRAWBRIDGE DRIVING ROUTINE
			DriveStraightCommand* drivingThroughDT = new DriveStraightCommand(
					robot, -9.0); //ARBITRARY VALUES
			DefenseManipPosCommand* defenseGoDownNowT =
					new DefenseManipPosCommand(superstructure, true);
			ParallelAutoCommand* throughDrawbridgeT = new ParallelAutoCommand(
					drivingThroughDT, defenseGoDownNowT);
			straightenMe->SetNextCommand(throughDrawbridgeT);
			DriveStraightCommand* drawbridgeGoBackT = new DriveStraightCommand(
					robot, -10.0); //ARBITRARY VALUES
			throughDrawbridgeT->SetNextCommand(drawbridgeGoBackT);
		} else {
			//SALLYPORT DRIVING ROUTINE
			CurveCommand* drivingThroughSPT = new CurveCommand(
					robot, -0.5, -10.0);
			straightenMe->SetNextCommand(drivingThroughSPT);
			PivotCommand* turningAroundSPT = new PivotCommand(robot, -180.0);
			drivingThroughSPT->SetNextCommand(turningAroundSPT);
			DriveStraightCommand* driveBackThroughSPT = new DriveStraightCommand(
					robot, -10.0);
			//turningAroundSPT->SetNextCommand(driveBackThroughSPT);

		}

		break;
	}
	case (kBlankAuto): {
		printf("kBlankAuto ----------------------\n");
		DUMP("BLANK AUTO", 0.0);
		IntakePositionCommand* andYouThoughtItWasBlankIntakeUp = new IntakePositionCommand(
				superstructure, false);
		DefenseManipPosCommand* andYouThoughtItWasBlankDefenseUp = new DefenseManipPosCommand(
				superstructure, false);
		ParallelAutoCommand* andYouThoughtItWasBlankMechanismsUp = new ParallelAutoCommand(
				andYouThoughtItWasBlankIntakeUp, andYouThoughtItWasBlankDefenseUp);
		firstCommand = andYouThoughtItWasBlankMechanismsUp;

		break;
	}
	case (kReachAuto): {
		/*
		 * Assumption: starting position is back of robot on auto line
		 * Length of robot is 2.823ft
		 * Distance from auto line to outerworks is 6.167;
		 */
		IntakePositionCommand* intakeUpWhee = new IntakePositionCommand(
				superstructure, false);
		DefenseManipPosCommand* defenseUpWhee = new DefenseManipPosCommand(
				superstructure, false);
		ParallelAutoCommand* mechanismsUpWhee = new ParallelAutoCommand(intakeUpWhee,
				defenseUpWhee);
		firstCommand = mechanismsUpWhee;

		printf("kReachAuto ------------------------\n");
		DUMP("REACH AUTO", 0.0);
		DriveStraightCommand* reachDrive = new DriveStraightCommand(robot, 6.0);
		mechanismsUpWhee->SetNextCommand(reachDrive);

		break;
	}
	case (kCrossAuto): {
		printf("kCrossAuto -----------------------------\n");
		DUMP("CROSS AUTO", 0.0);
		IntakePositionCommand* myIntakeGoesUp = new IntakePositionCommand(
				superstructure, false);
		DefenseManipPosCommand* myDefenseGoesUp = new DefenseManipPosCommand(
				superstructure, false);
		ParallelAutoCommand* andThemMechanismsGoUp = new ParallelAutoCommand(myIntakeGoesUp,
				myDefenseGoesUp);
		firstCommand = andThemMechanismsGoUp;

		DefenseCommand* cross = new DefenseCommand(robot, superstructure, humanControl->GetDefense());
		andThemMechanismsGoUp->SetNextCommand(cross);
		OuttakeByTimeCommand* crossOuttake = new OuttakeByTimeCommand(superstructure, 3.0);
		cross->SetNextCommand(crossOuttake);
		break;
	}
	case (kShootAuto): {
		printf("kShootAuto --------------------------------\n");
		DUMP("SHOOT AUTO", 0.0);
		DUMP("DEFENSE POSITION", (double) humanControl->GetDefensePosition());
		/*
		 * Assumption: starting position is back of robot on auto line
		 * GOING THROUGH LOW BAR IN THESE CALCULATIONS
		 * Length of robot is 2.823 ft
		 * Distance from auto line to end of autoworks is
		 *
		 */
		IntakePositionCommand* intakeUp = new IntakePositionCommand(superstructure, false);
		DefenseManipPosCommand* defenseUp = new DefenseManipPosCommand(superstructure, false);
		ParallelAutoCommand* mechanismsUp = new ParallelAutoCommand(intakeUp, defenseUp);
		firstCommand = mechanismsUp;
		if (hardCodeShoot) {
			DefenseCommand* hardCodeCross = new DefenseCommand(robot, superstructure, humanControl->GetDefense());
			mechanismsUp->SetNextCommand(hardCodeCross);
			PivotToAngleCommand* hardCodeStraighten = new PivotToAngleCommand(robot, 0.0);
			hardCodeCross->SetNextCommand(hardCodeStraighten);
			CurveCommand* hardCodeDrive;
			PivotToAngleCommand* hardCodeLineUpShoot;
			switch(humanControl->GetDefensePosition()) {
			case (kNone): {
				printf("UH OH ERROR ERROR EEEEEEEKKKKKKK! \n");
				break;
			}
			case (kLowBar): {
				hardCodeDrive = new CurveCommand(robot, 6.2, -10.0); //was 12.3
				hardCodeLineUpShoot = new PivotToAngleCommand(robot, 40.0);
				break;
			}
			case (kSecond): {
				hardCodeDrive = new CurveCommand(robot, 2.2, 10.0); //was 12.3
				hardCodeLineUpShoot = new PivotToAngleCommand(robot, 40.0);
				break;
			}
			case (kThird): {
				hardCodeDrive = new CurveCommand(robot, -2.0, 10.0); //was 12.3
				hardCodeLineUpShoot = new PivotToAngleCommand(robot, 40.0);
				break;
			}
			case (kFourth): {
				hardCodeDrive = new CurveCommand(robot, 3.2, 10.0); //was 12.3
				hardCodeLineUpShoot = new PivotToAngleCommand(robot, 290.0);
				break;
			}
			case (kFifth): {
				// hardCodeDrive = new CurveCommand(robot, -0.5, 12.3);
				hardCodeDrive = new CurveCommand(robot, -1.0, 10.0); //was 12.3
				hardCodeLineUpShoot = new PivotToAngleCommand(robot, 290.0);
				break;
			}
			}
			hardCodeStraighten->SetNextCommand(hardCodeDrive);
			hardCodeDrive->SetNextCommand(hardCodeLineUpShoot);
			OuttakeByTimeCommand* outtaking = new OuttakeByTimeCommand(superstructure, 1.0);
			hardCodeLineUpShoot->SetNextCommand(outtaking);

		} else {
			DefenseCommand* cameraCross = new DefenseCommand(robot, superstructure, humanControl->GetDefense());
			mechanismsUp->SetNextCommand(cameraCross);
			PivotToAngleCommand* cameraStraighten = new PivotToAngleCommand(robot, 0.0);
			cameraCross->SetNextCommand(cameraStraighten);
			CurveCommand* weShallGetNear;
			CameraCommand* theCameraCommand;
			PivotToAngleCommand* gottaLineUp;
			switch (humanControl->GetDefensePosition()) {
			case (kNone): {
				printf("UH OH ERROR ERROR EEEEEEEKKKKKKK! \n");
				break;
			}
			case (kLowBar): {
				weShallGetNear = new CurveCommand(robot, 1.0, 7.0); //ARBITRARY VALUES
				theCameraCommand = new CameraCommand(robot, camera, 0.0, 5.0, true); //ARBITRARY VALUES
				gottaLineUp = new PivotToAngleCommand(robot, 60.0); //ARBITRARY VALUES
				break;
			}
			case (kSecond): {
				weShallGetNear = new CurveCommand(robot, 0.0, 7.0); //ARBITRARY VALUES
				theCameraCommand = new CameraCommand(robot, camera, 0.0, 5.0, true); //ARBITRARY VALUES
				gottaLineUp = new PivotToAngleCommand(robot, 60.0); //ARBITRARY VALUES
				break;
			}
			case (kThird): {
				weShallGetNear = new CurveCommand(robot, -4.0, 7.0); //ARBITRARY VALUES
				theCameraCommand = new CameraCommand(robot, camera, 0.0, 5.0, true); //ARBITRARY VALUES
				gottaLineUp = new PivotToAngleCommand(robot, 60.0); //ARBITRARY VALUES
				break;
			}
			case (kFourth): {
				weShallGetNear = new CurveCommand(robot, 6.0, 7.0); //ARBITRARY VALUES
				theCameraCommand = new CameraCommand(robot, camera, 0.0, 5.0, false); //ARBITRARY VALUES
				gottaLineUp = new PivotToAngleCommand(robot, 300.0); //ARBITRARY VALUES
				break;
			}
			case (kFifth): {
				weShallGetNear = new CurveCommand(robot, 2.0, 7.0); //ARBITRARY VALUES
				theCameraCommand = new CameraCommand(robot, camera, 0.0, 5.0, false); //ARBITRARY VALUES
				gottaLineUp = new PivotToAngleCommand(robot, 300.0); //ARBITRARY VALUES
				break;
			}
			cameraStraighten->SetNextCommand(weShallGetNear);
			weShallGetNear->SetNextCommand(theCameraCommand);
			theCameraCommand->SetNextCommand(gottaLineUp);
			OuttakeCommand* shootThatBoulder = new OuttakeCommand(superstructure);
			gottaLineUp->SetNextCommand(shootThatBoulder);
			}
		}

		break;
	}
	case (kHoardingAuto): {
		printf("kHoardingAuto ------------------------------\n");
		DUMP("HOARDING AUTO", 0.0);
		IntakePositionCommand* hoardIntakeUp = new IntakePositionCommand(superstructure, false);
		DefenseManipPosCommand* hoardDefenseUp = new DefenseManipPosCommand(superstructure, false);
		ParallelAutoCommand* hoardMechanismsUp = new ParallelAutoCommand(hoardIntakeUp, hoardDefenseUp);
		firstCommand = hoardMechanismsUp;
		DefenseCommand* hoardFirstCrossA = new DefenseCommand(robot, superstructure, humanControl->GetDefense());
		hoardMechanismsUp->SetNextCommand(hoardFirstCrossA);
		CurveCommand* hoardGoToCatC = new CurveCommand(robot, 4.0*(secondDefensePos - humanControl->GetDefensePosition()), 1.0);
		hoardFirstCrossA->SetNextCommand(hoardGoToCatC);
		PivotToAngleCommand* hoardStraightenBeforeCatC = new PivotToAngleCommand(robot, 0.0);
		hoardGoToCatC->SetNextCommand(hoardStraightenBeforeCatC);
		DriveStraightCommand* hoardCrossCatC;
		if (useSallyPort) {
			hoardCrossCatC = new DriveStraightCommand(robot, -8.0);
		} else {
			hoardCrossCatC = new DriveStraightCommand(robot, -7.5);
		}
		hoardStraightenBeforeCatC->SetNextCommand(hoardCrossCatC);
		PivotToAngleCommand* hoardStraightenBeforeBall = new PivotToAngleCommand(robot, 0.0);
		hoardCrossCatC->SetNextCommand(hoardStraightenBeforeBall);
		CurveCommand* hoardGetBall;
		PivotToAngleCommand* hoardLineUpBall;
		IntakePositionCommand* hoardIntakeDown;
		IntakeRollersCommand* hoardStartIntaking;
		DriveStraightCommand* hoardDriveTo;
		DriveStraightCommand* hoardDriveBack;
		ChainedCommand* hoardDriveToAndFromBall;
		ParallelAutoCommand* hoardCollectBall;
		switch (secondDefensePos) {
		case (kSecond) : {
			hoardGetBall = new CurveCommand(robot, -1.5, 3.0); //ARBITRARY VALUES
			hoardStraightenBeforeBall->SetNextCommand(hoardGetBall);
			hoardLineUpBall = new PivotToAngleCommand(robot, 0.0);
			hoardGetBall->SetNextCommand(hoardLineUpBall);
			hoardIntakeDown = new IntakePositionCommand(superstructure, true);
			hoardLineUpBall->SetNextCommand(hoardIntakeDown);
			hoardStartIntaking = new IntakeRollersCommand(superstructure, true,  5.0);
			hoardDriveTo = new DriveStraightCommand(robot, -3.0); //ARBITRARY VALUES
			hoardDriveTo->disDFac = hoardDriveTo->disDFac * 3.0;
			hoardDriveBack = new DriveStraightCommand(robot, 3.0); // ARBITRARY VALUES
			hoardDriveToAndFromBall = new ChainedCommand(hoardDriveTo, hoardDriveBack);
			hoardCollectBall = new ParallelAutoCommand(hoardDriveToAndFromBall, hoardStartIntaking);
		}
		case (kThird) : {
			hoardGetBall = new CurveCommand(robot, -1.0, 3.0); //ARBITRARY VALUES
			hoardStraightenBeforeBall->SetNextCommand(hoardGetBall);
			hoardLineUpBall = new PivotToAngleCommand(robot, 0.0);
			hoardGetBall->SetNextCommand(hoardLineUpBall);
			hoardIntakeDown = new IntakePositionCommand(superstructure, true);
			hoardLineUpBall->SetNextCommand(hoardIntakeDown);
			hoardStartIntaking = new IntakeRollersCommand(superstructure, true,
					5.0);
			hoardDriveTo = new DriveStraightCommand(robot, -3.0); //ARBITRARY VALUES
			hoardDriveTo->disDFac = hoardDriveTo->disDFac * 3.0;
			hoardDriveBack = new DriveStraightCommand(robot, 3.0); // ARBITRARY VALUES
			hoardDriveToAndFromBall = new ChainedCommand(hoardDriveTo,
					hoardDriveBack);
			hoardCollectBall = new ParallelAutoCommand(hoardDriveToAndFromBall,
					hoardStartIntaking);
		}
		case (kFourth) : {
			hoardGetBall = new CurveCommand(robot, 0.0, 3.0); //ARBITRARY VALUES
			hoardStraightenBeforeBall->SetNextCommand(hoardGetBall);
			hoardLineUpBall = new PivotToAngleCommand(robot, 0.0);
			hoardGetBall->SetNextCommand(hoardLineUpBall);
			hoardIntakeDown = new IntakePositionCommand(superstructure, true);
			hoardLineUpBall->SetNextCommand(hoardIntakeDown);
			hoardStartIntaking = new IntakeRollersCommand(superstructure, true,
					5.0);
			hoardDriveTo = new DriveStraightCommand(robot, -3.0); //ARBITRARY VALUES
			hoardDriveTo->disDFac = hoardDriveTo->disDFac * 3.0;
			hoardDriveBack = new DriveStraightCommand(robot, 3.0); // ARBITRARY VALUES
			hoardDriveToAndFromBall = new ChainedCommand(hoardDriveTo,
					hoardDriveBack);
			hoardCollectBall = new ParallelAutoCommand(hoardDriveToAndFromBall,
					hoardStartIntaking);
		}
		case (kFifth) : {
			hoardGetBall = new CurveCommand(robot, 0.75, 3.0); //ARBITRARY VALUES
			hoardStraightenBeforeBall->SetNextCommand(hoardGetBall);
			hoardLineUpBall = new PivotToAngleCommand(robot, 0.0);
			hoardGetBall->SetNextCommand(hoardLineUpBall);
			hoardIntakeDown = new IntakePositionCommand(superstructure, true);
			hoardLineUpBall->SetNextCommand(hoardIntakeDown);
			hoardStartIntaking = new IntakeRollersCommand(superstructure, true,
					5.0);
			hoardDriveTo = new DriveStraightCommand(robot, -3.0); //ARBITRARY VALUES
			hoardDriveTo->disDFac = hoardDriveTo->disDFac * 3.0;
			hoardDriveBack = new DriveStraightCommand(robot, 3.0); // ARBITRARY VALUES
			hoardDriveToAndFromBall = new ChainedCommand(hoardDriveTo,
					hoardDriveBack);
			hoardCollectBall = new ParallelAutoCommand(hoardDriveToAndFromBall,
					hoardStartIntaking);
		}
		}
		break;
	}
	case (kSpyBotShootAuto): {
		printf("kSpyBotShootAuto -------------------------------------\n");
		DUMP("SPYBOTSHOOTAUTO", 0.0);
		IntakePositionCommand* intakeUpSBSA = new IntakePositionCommand(
				superstructure, false);
		DefenseManipPosCommand* defenseUpSBSA = new DefenseManipPosCommand(
				superstructure, false);
		ParallelAutoCommand* mechanismsUpSBSA = new ParallelAutoCommand(intakeUpSBSA,
				defenseUpSBSA);
		firstCommand = mechanismsUpSBSA;

		CurveCommand* drivingToTheLowGoal = new CurveCommand(robot, 1.8, 6.8); //was 0.8 on x
		mechanismsUpSBSA->SetNextCommand(drivingToTheLowGoal);
		PivotToAngleCommand* pivotMe = new PivotToAngleCommand(robot, 330.0);
		drivingToTheLowGoal->SetNextCommand(pivotMe);
		OuttakeByTimeCommand* yeahShooting = new OuttakeByTimeCommand(superstructure, 1.0);
		pivotMe->SetNextCommand(yeahShooting);

		break;
	}
	case (kSpyBotCatCAuto): {
		printf("kSpyBotCatCAuto --------------------------------------\n");
		DUMP("SPYBOTCATCAUTO", 0.0);
		// BEGINNING OF AUTO
		IntakePositionCommand* intakeUpSBC = new IntakePositionCommand(
				superstructure, false);
		DefenseManipPosCommand* defenseUpSBC = new DefenseManipPosCommand(
				superstructure, false);
		ParallelAutoCommand* mechanismsUpSBC = new ParallelAutoCommand(
				intakeUpSBC, defenseUpSBC);
		firstCommand = mechanismsUpSBC;
		//SHOOTING IN THE GOAL

		CurveCommand* driveToGoalSBC = new CurveCommand(robot, 1.8, 6.8); //was 0.8 for x
		mechanismsUpSBC->SetNextCommand(driveToGoalSBC);
		PivotToAngleCommand* pivotForGoalSBC = new PivotToAngleCommand(robot, 330.0);
		driveToGoalSBC->SetNextCommand(pivotForGoalSBC);
		OuttakeByTimeCommand* shootingSBC = new OuttakeByTimeCommand(superstructure, 1.0);
		pivotForGoalSBC->SetNextCommand(shootingSBC);

		//LINING UP
		DriveStraightCommand* driveOffBatterSBC = new DriveStraightCommand(robot, -0.0);
		shootingSBC->SetNextCommand(driveOffBatterSBC);
		PivotToAngleCommand* SBCLiningUp = new PivotToAngleCommand(robot, -90.0);
		driveOffBatterSBC->SetNextCommand(SBCLiningUp);

		//DRIVING TO CAT C
		CurveCommand* drivingToCatC = new CurveCommand(robot, -10.6 + 4.2 * humanControl->GetDefensePosition(), -12.0);
		SBCLiningUp->SetNextCommand(drivingToCatC);
		PivotToAngleCommand* straightenMeSBC = new PivotToAngleCommand(robot, 0.0); //should be -90
		drivingToCatC->SetNextCommand(straightenMeSBC);
		if (!useSallyPort) {
			//DRAWBRIDGE DRIVING ROUTINE
			DriveStraightCommand* drivingThroughD = new DriveStraightCommand(robot, -9.0); //ARBITRARY VALUES
			DefenseManipPosCommand* defenseGoDownNow = new DefenseManipPosCommand(superstructure, true);
			ParallelAutoCommand* throughDrawbridge = new ParallelAutoCommand(drivingThroughD, defenseGoDownNow);
			straightenMeSBC->SetNextCommand(throughDrawbridge);
			DriveStraightCommand* drawbridgeGoBack = new DriveStraightCommand(robot, -10.0); //ARBITRARY VALUES
			throughDrawbridge->SetNextCommand(drawbridgeGoBack);
		} else {
			//SALLYPORT DRIVING ROUTINE
			DriveStraightCommand* drivingThroughSP = new DriveStraightCommand(robot, -10.0);
			straightenMeSBC->SetNextCommand(drivingThroughSP);
			PivotCommand* turningAroundSP = new PivotCommand(robot, -180.0);
			drivingThroughSP->SetNextCommand(turningAroundSP);
			DriveStraightCommand* driveBackThroughSP = new DriveStraightCommand(robot, -10.0);
			turningAroundSP->SetNextCommand(driveBackThroughSP);

		}
		break;
	}
	}
}
