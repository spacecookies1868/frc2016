#include "SuperstructureController.h"


SuperstructureController::SuperstructureController(RobotModel* myRobot, RemoteControl* myHumanControl) {
	robot = myRobot;
	humanControl = myHumanControl;

	m_stateVal = kInit;
	nextState = kInit;

	useDoorbellButtons = false;

	startedPTO = false;
	startTimePTO = 0.0;
	deltaTimePTO = 0.0;

	startedOT = false;
	startEncoderValEOT = 0.0;
	deltaEncoderValEOT = 0.0;
	startTimeTOT = 0.0;
	deltaTimeTOT = 0.0;

	autoDefenseManipUp = false;
	autoDefenseManipDown = false;
	autoIntakeUp = false;
	autoIntakeDown = false;
	autoIntakeMotorForward = false;
	autoIntakeMotorBackward = false;
	autoBallInIntake = false;
	autoBallInIntakeFinished = false;
	autoOuttake = false;
	autoOuttakeFinished = false;
	autoManualOuttakeForward = false;
	autoManualOuttakeReverse = false;

	intakeSpeed = 0.0;
	outtakeSpeed = 0.0;
}

void SuperstructureController::Reset() {
	m_stateVal = kInit;
	nextState = kInit;

	startedPTO = false;
	startTimePTO = 0.0;

	startedOT = false;
	startEncoderValEOT = 0.0;
	startTimeTOT = 0.0;

	autoDefenseManipUp = false;
	autoDefenseManipDown = false;
	autoIntakeUp = false;
	autoIntakeDown = false;
	autoIntakeMotorForward = false;
	autoIntakeMotorBackward = false;
	autoBallInIntake = false;
	autoBallInIntakeFinished = false;
	autoOuttake = false;
	autoOuttakeFinished = true; //The outtake is not going, therefore it is finished
	autoManualOuttakeForward = false;
	autoManualOuttakeReverse = false;
}

void SuperstructureController::Update(double currTimeSec, double deltaTimeSec) {
	switch(m_stateVal) {
	case (kInit):
		//maybe set all single solenoids to their base positions so that they don't move when you enable
		robot->SetIntakeMotorSpeed(0.0);
		robot->SetOuttakeMotorSpeed(0.0);
		nextState = kIdle;
		break;
	case (kIdle):
		nextState = kIdle;
		//Change the position of the defense manipulator in teleop
		if (useDoorbellButtons) {
			if (humanControl->GetDefenseManipDownDesired()) {
				robot->MoveDefenseManipDown();
			} else {
				robot->MoveDefenseManipUp();
			}
		} else {
			if (humanControl->GetDefenseManipToggleDesired()) {
				robot->ChangeDefenseManipState();
			}
		}

		//Change the position of the defense manipulator in auto
		if (autoDefenseManipUp) {
			robot->MoveDefenseManipUp();
			autoDefenseManipUp = false;
		} else if (autoDefenseManipDown) {
			robot->MoveDefenseManipDown();
			autoDefenseManipDown = false;
		}

		//Change the position of the intake in teleop
		if (useDoorbellButtons) {
			if (humanControl->GetIntakePistonDownDesired()) {
				robot->MoveIntakeArmDown();
			} else {
				robot->MoveIntakeArmUp();
			}
		} else {
			if (humanControl->GetIntakePistonToggleDesired()) {
				robot->ChangeIntakeArmState();
			}

		}

		//Change the position of the intake in auto
		if (autoIntakeUp) {
			robot->MoveIntakeArmUp();
			autoIntakeUp = false;
		} else if (autoIntakeDown) {
			robot->MoveIntakeArmDown();
			autoIntakeDown = false;
		}

		//Move the intake motor forward or backward
		if (humanControl->GetIntakeMotorForwardDesired() || autoIntakeMotorForward) {
			robot->SetIntakeMotorSpeed(intakeSpeed);
		} else if (humanControl->GetIntakeMotorReverseDesired() || autoIntakeMotorBackward) {
			robot->SetIntakeMotorSpeed(-intakeSpeed);
		} else {
			robot->SetIntakeMotorSpeed(0.0);
		}

		//Makes sure that the defense manipulator is down first (must be in that position to outtake) and then outtakes
		//If already down, goes straight to outtake
		if (autoOuttake || humanControl->GetOuttakeDesired()) {
			autoOuttakeFinished = false;
			if (!robot->IsIntakeArmDown()) {
				nextState = kTimeOuttake;
			} else {
				nextState = kPrepToOuttake;
			}
			autoOuttake = false;
		}

		if (humanControl->GetBallInIntakeDesired() || autoBallInIntake) {
			autoBallInIntake = false;
			nextState = kBallInIntake;
		}

		//Allows for manual control of the outtake motors
		if (humanControl->GetManualOuttakeForwardDesired() || autoManualOuttakeForward) {
			robot->SetOuttakeMotorSpeed(outtakeSpeed);
		} else if (humanControl->GetManualOuttakeReverseDesired() || autoManualOuttakeReverse) {
			robot->SetOuttakeMotorSpeed(-outtakeSpeed);
		} else {
			robot->SetOuttakeMotorSpeed(0.0);
		}

		break;

	case (kPrepToOuttake):
		//Moves the intake arm, so that it can outtake and waits
		if(!startedPTO) {
			DO_PERIODIC(1, "Starting kPrepToOuttake\n");
			startTimePTO = currTimeSec;
			robot->MoveIntakeArmUp();
			startedPTO = true;
			nextState = kPrepToOuttake;
		} else if (currTimeSec - startTimePTO < deltaTimePTO){
			nextState = kPrepToOuttake;
		} else {
			startedPTO = false;
			nextState = kTimeOuttake;
		}
		break;
	case (kTimeOuttake):
		//Runs the outtake motors for a certain amount of time to ensure that they have outtaken
		if (!startedOT) {
			startTimeTOT = robot->GetTime();
			robot->SetOuttakeMotorSpeed(-outtakeSpeed);
			startedOT = true;
			nextState = kTimeOuttake;
		} else if (robot->GetTime() - startTimeTOT < deltaTimeTOT) {
			robot->SetOuttakeMotorSpeed(-outtakeSpeed);
			nextState = kTimeOuttake;
		} else {
			robot->SetOuttakeMotorSpeed(0.0);
			startedOT = false;
			nextState = kIdle;
		}
		break;
	case (kEncOuttake):
		//Runs the outtake motors for a certain amount of encoder ticks to ensure that they have outtaken
		if (!startedOT) {
			startEncoderValEOT = robot->GetOuttakeEncoderVal();
			robot->SetOuttakeMotorSpeed(outtakeSpeed);
			startedOT = true;
			nextState = kEncOuttake;
		} else if (robot->GetOuttakeEncoderVal() - startEncoderValEOT < deltaEncoderValEOT) {
			robot->SetOuttakeMotorSpeed(outtakeSpeed);
			nextState = kEncOuttake;
		} else {
			robot->SetOuttakeMotorSpeed(0.0);
			startedOT = false;
			autoOuttakeFinished = true;
			nextState = kIdle;
		}
		break;
	case (kBallInIntake):
		if (humanControl->GetBallInIntakeDesired()) {
			robot->SetIntakeMotorSpeed(0.0);
			nextState = kIdle;
		} else if (!robot->GetIntakeSwitchState()) {
			robot->SetIntakeMotorSpeed(intakeSpeed);
			nextState = kBallInIntake;
		} else {
			robot->SetIntakeMotorSpeed(0.0);
			autoBallInIntakeFinished = true;
			nextState = kIdle;
		}
		break;
	default:
		break;
	}
	m_stateVal = nextState;
}

void SuperstructureController::RefreshIni() {
	useDoorbellButtons = robot->pini->getbool("SUPERSTRUCTURE", "useDoorbellButton", false);
	intakeSpeed = robot->pini->getf("SUPERSTRUCTURE", "intakeSpeed", 0.4);
	outtakeSpeed = robot->pini->getf("SUPERSTRUCTURE", "outtakeSpeed", 0.4);
	deltaTimePTO = robot->pini->getf("SUPERSTRUCTURE", "deltaTimePTO", 1.0);
	deltaEncoderValEOT = robot->pini->getf("SUPERSTRUCTURE", "deltaEncoderValEOT", 0.0);
	deltaTimeTOT = robot->pini->getf("SUPERSTRUCTURE", "deltaTimeTOT", 0.0);

}

void SuperstructureController::SetAutoDefenseManipUp(bool desired) {
	autoDefenseManipUp = desired;
}

void SuperstructureController::SetAutoDefenseManipDown(bool desired) {
	autoDefenseManipDown = desired;
}

void SuperstructureController::SetAutoIntakeUp(bool desired) {
	autoIntakeUp = desired;
}

void SuperstructureController::SetAutoIntakeDown(bool desired) {
	autoIntakeDown = desired;
}

void SuperstructureController::SetAutoIntakeMotorForward(bool desired) {
	autoIntakeMotorForward = desired;
}

void SuperstructureController::SetAutoIntakeMotorBackward(bool desired) {
	autoIntakeMotorBackward = desired;
}

void SuperstructureController::SetAutoBallInIntake(bool desired) {
	autoBallInIntake = desired;
	autoBallInIntakeFinished = false;
}

bool SuperstructureController::GetAutoBallInIntakeFinished() {
	return autoBallInIntakeFinished;
}

void SuperstructureController::SetAutoOuttake(bool desired) {
	autoOuttake = desired;
	autoOuttakeFinished = false;
}

bool SuperstructureController::GetOuttakeFinished() {
	return autoOuttakeFinished;
}

void SuperstructureController::SetAutoManualOuttakeForward(bool desired) {
	autoManualOuttakeForward = desired;
}

void SuperstructureController::SetAutoManualOuttakeReverse(bool desired) {
	autoManualOuttakeReverse = desired;
}

