#include "SuperstructureController.h"


SuperstructureController::SuperstructureController(RobotModel* myRobot, RemoteControl* myHumanControl) {
	robot = myRobot;
	humanControl = myHumanControl;

	m_stateVal = kInit;
	nextState = kInit;

	startedPAO = false;
	startTimePAO = 0.0;
	deltaTimePAO = 0.0;

	autoDefenseManipUp = false;
	autoDefenseManipDown = false;
	autoIntakeUp = false;
	autoIntakeDown = false;
	autoIntakeMotorForward = false;
	autoIntakeMotorBackward = false;
	autoOuttakeIn = false;
	autoOuttakeOut = false;
}

void SuperstructureController::Reset() {
	//Zero all necessary variables
	m_stateVal = kInit;
	nextState = kInit;

	startedPAO = false;
	startTimePAO = 0.0;

	autoDefenseManipUp = false;
	autoDefenseManipDown = false;
	autoIntakeUp = false;
	autoIntakeDown = false;
	autoIntakeMotorForward = false;
	autoIntakeMotorBackward = false;
	autoOuttakeIn = false;
	autoOuttakeOut = false;
}

//correct state for defense manipulator piston is the state that allows the robot to outtake
void SuperstructureController::Update(double currTimeSec, double deltaTimeSec) {
	switch(m_stateVal) {
	case (kInit):
		//check position of all pistons
		//maybe set outtake piston to in?
		//set intake motor speed to zero
		nextState = kIdle;
		break;
	case (kIdle):
		nextState = kIdle;
		if (false /*defense manipulator button -- button 5 -- was just pressed*/) {
			//defense manipulator piston change state
		}

		if (autoDefenseManipUp) {
			//defense manipulator piston up
			autoDefenseManipUp = false;
		} else if (autoDefenseManipDown) {
			//defense manipulator piston down
			autoDefenseManipDown = false;
		}

		if (false /*intake piston button -- button 1 -- was just pressed*/) {
			//intake piston change state
		}

		if (autoIntakeUp) {
			//intake piston up
			autoIntakeUp = false;
		} else if (autoIntakeDown) {
			//intake piston down
			autoIntakeDown = false;
		}

		if (false /*intake motor button -- button 4 -- is forward*/ || autoIntakeMotorForward) {
			//set intake motor speed to intake
			//set outtake piston to in state
		} else if (false /*intake motor button -- button 3 -- is backward*/ || autoIntakeMotorBackward) {
			//set intake motor speed to outtake
			//set outtake piston to in state
		} else {
			//set intake motor speed to zero
		}

		if (false /*outtake piston button -- button 9 -- was just pressed*/ || autoOuttakeIn || autoOuttakeOut) {
			if (false /*outtake piston at in state*/ || autoOuttakeOut) {
				if (false /*defense manipulator piston in correct state*/) {
					//set the outtake piston to out state
				} else {
					nextState = kPrepAndOuttake;
				}
				autoOuttakeOut = false;
			} else {
				//set outtake piston to in state
				autoOuttakeIn = false;
			}
		}

		break;
	case (kPrepAndOuttake):
		if(!startedPAO) {
			//set defense manipulator piston to correct state
			startTimePAO = currTimeSec;
			startedPAO = true;
			nextState = kPrepAndOuttake;
		} else if (currTimeSec - startTimePAO < deltaTimePAO){
			nextState = kPrepAndOuttake;
		} else {
			//set outtake piston to out state
			startedPAO = false;
			nextState = kIdle;
		}
		break;
	default:
		DO_PERIODIC(1, printf("SuperstructureController default. You should not be here!\n"));
		break;
	}
	m_stateVal = nextState;
}

void SuperstructureController::RefreshIni() {
	//TODO Figure out actual time it takes to deploy the defense manipulator
	deltaTimePAO = 1.0;
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

void SuperstructureController::SetAutoOuttakeIn(bool desired) {
	autoOuttakeIn = desired;
}

void SuperstructureController::SetAutoOuttakeOut(bool desired) {
	autoOuttakeOut = desired;
}
